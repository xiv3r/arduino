/*
============================================================================
*  ESP32 16-Channel Relay Scheduler & Web Manager
============================================================================
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <heap_caps.h>
#include <sys/time.h>

#if !defined(ARDUINOJSON_VERSION_MAJOR) || ARDUINOJSON_VERSION_MAJOR < 7
#error "ArduinoJson v7.x is required. Install/upgrade ArduinoJson (v7) from Library Manager."
#endif

#define FW_NAME     "RM16"
#define FW_VERSION  "1.0.0"
#define N_RELAYS    16
#define N_PAIRS     8
#define WEB_PORT    80
#define DNS_PORT    53
#define NVS_NS      "rm16"
#define TZ_DEFAULT  25
#define TZ_COUNT    34

static const uint8_t RELAY_PINS[N_RELAYS] = {23,32,33,25,26,27,14,13,1,3,19,18,5,4,2,15};
#define PIN_RESET_BTN  0
#define PIN_RTC_SDA    21
#define PIN_RTC_SCL    22

#define TSRC_BOOT    0
#define TSRC_NTP     1
#define TSRC_RTC     2
#define TSRC_MANUAL  3

#define LOOKBACK_DAYS 1500

#define BODY_MAX       16384
#define BODY_MAX_RELAY 16384

struct DstRule {
  uint8_t mon, week, dow, hour, minute;
};

struct TzPreset {
  const char *name;
  int32_t stdOff;
  int16_t dstMin;
  DstRule start;
  DstRule end;
};

struct TimePattern {
  uint64_t sec;
  uint64_t min;
  uint32_t hr;
  uint32_t dom;
  uint16_t mon;
  uint8_t  dow;
  uint8_t  flags;
};
static_assert(sizeof(TimePattern) == 32, "TimePattern layout");

struct SchedulePair {
  uint8_t enabled;
  uint8_t rsv;
  TimePattern start;
  TimePattern stop;
};
static_assert(sizeof(SchedulePair) == 72, "SchedulePair layout");

struct RelayCfg {
  char name[24];
  uint8_t polarity;
  uint8_t overrideMode;
  uint8_t schedulesEnabled;
  uint8_t rsv;
  SchedulePair pairs[N_PAIRS];
};
static_assert(sizeof(RelayCfg) == 608, "RelayCfg layout");

struct SysCfg {
  char staSsid[33], staPass[65], staHost[33];
  char apSsid[33], apPass[65];
  char ntp1[49], ntp2[49], ntp3[49];
  uint32_t staIp, staGw, staMask, staDns1, staDns2;
  uint32_t apIp, apGw, apMask;
  uint32_t ntpTimeoutS, ntpIntervalS;
  int32_t  gmtOffS;
  uint32_t bootCount;
  uint16_t dstShiftMin, rtcResyncMin;
  uint8_t  staEnabled, staStatic, apPolicy, apChannel, apHidden, apMaxConn, apOpen;
  uint8_t  tzMode;
  uint8_t  dstMode;
  uint8_t  dsMon, dsWeek, dsDow, dsHour, dsMin;
  uint8_t  deMon, deWeek, deDow, deHour, deMin;
  uint8_t  tzPreset, rtcEnabled, globalActiveHigh;
  int8_t   driftPerDay;
};
static_assert(sizeof(SysCfg) == 452, "SysCfg layout");

#define SYSCFG_MAGIC 0x524D3136u
#define SYSCFG_VER   1
#define RLYCFG_MAGIC 0x524C5931u
#define RLYCFG_VER   1

struct SysCfgBlob { uint32_t magic; uint16_t version; uint16_t gen; uint32_t crc; SysCfg c; };
struct RlyCfgBlob { uint32_t magic; uint16_t version; uint16_t rsv;  uint32_t crc; RelayCfg c; };

struct NtpState {
  uint8_t  state;
  int8_t   srv;
  uint8_t  attempt;
  uint32_t fails;
  int64_t  nextTryUtc;
  int64_t  waitUs;
  int64_t  lastOkUtc;
  char     lastOkSrv[49];
  uint64_t origTs;
};

struct DnsQ {
  uint8_t  state;
  char     host[49];
  uint32_t ip;
  int8_t   srvIdx;
  int64_t  txUs;
  uint16_t id;
};

struct ScanNet { char ssid[33]; int32_t rssi; uint8_t sec; uint8_t ch; };

static const TzPreset TZ_PRESETS[TZ_COUNT] = {
  {"UTC",          0,     0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Honolulu",  -36000,   0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Anchorage", -32400,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"Los Angeles",-28800, 60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"Denver",    -25200,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"Phoenix",   -25200,   0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Chicago",   -21600,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"New York",  -18000,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"Halifax",   -14400,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"St. John's",-12600,  60, {3,2,0,2,0},  {11,1,0,2,0}},
  {"Sao Paulo", -10800,   0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"London",        0,   60, {3,5,0,1,0},  {10,5,0,2,0}},
  {"Paris",      3600,   60, {3,5,0,2,0},  {10,5,0,3,0}},
  {"Berlin",     3600,   60, {3,5,0,2,0},  {10,5,0,3,0}},
  {"Athens",     7200,   60, {3,5,0,3,0},  {10,5,0,4,0}},
  {"Moscow",    10800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Istanbul",  10800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Dubai",     14400,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Karachi",   18000,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Kolkata",   19800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Kathmandu", 20700,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Dhaka",     21600,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Bangkok",   25200,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Shanghai",  28800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Hong Kong", 28800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Manila",    28800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Singapore", 28800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Tokyo",     32400,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Seoul",     32400,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Perth",     28800,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Adelaide",  34200,  30, {10,1,0,2,0}, {4,1,0,3,0}},
  {"Sydney",    36000,  60, {10,1,0,2,0}, {4,1,0,3,0}},
  {"Brisbane",  36000,    0, {0,0,0,0,0},  {0,0,0,0,0}},
  {"Auckland",  43200,  60, {9,5,0,2,0},  {4,1,0,3,0}},
};

SysCfg    cfg;
RelayCfg  rcfg[N_RELAYS];
bool      desiredOn[N_RELAYS];

int64_t   g_baseUtc   = 0;
int64_t   g_baseUs    = 0;
int64_t   curUtcSec   = 0;
int64_t   lastEvalSec = 0;
bool      timeValid   = false;
uint8_t   timeSrc     = TSRC_BOOT;
char      timeSrcName[24] = "Build";
int64_t   lastSyncUtc = 0;

bool      dstActive    = false;
int32_t   curOffSec    = 28800;
int32_t   g_stdOff     = 28800;
int16_t   g_dstMin     = 0;
int64_t   nextChangeUtc = 0;
int8_t    nextChangeDir = 0;

Preferences prefs;
bool      prefsOpen   = false;
uint16_t  sysGen      = 0;
char      sysLastSlot = 'B';
uint8_t   sysDirty    = 0;
uint16_t  rlyDirtyBits = 0;
int64_t   sysDirtyUs  = 0, rlyDirtyUs = 0, lastNvsWriteUs = 0;
uint32_t  nvsWrites   = 0;

WebServer server(WEB_PORT);
DNSServer dnsServer;
bool      dnsRunning = false;
bool      apActive   = false;
int8_t    staState   = 0;
int64_t   staLastTryUs = 0, staLastBeginUs = 0, apLastChangeUs = 0;
int64_t   apGraceUntilUs = 0;
bool      staHold   = false;
bool      mdnsUp    = false;
bool      scanBusy  = false;
int64_t   scanStartUs = 0;
ScanNet   scanNets[32];
uint8_t   scanNetCount = 0;

NtpState ntp;
DnsQ     dnsq;
WiFiUDP  ntpUdp, dnsUdp;
bool     udpBound = false;
uint8_t  ntpPkt[48];

RTC_DS3231 rtc;
bool      rtcPresent   = false;
bool      rtcLostPower = false;
int64_t   rtcLastWriteUs  = 0;
int64_t   rtcLastResyncUs = 0;

int64_t   rebootAtUs = 0;
char      rebootReason[24] = "";

int64_t  floorDiv(int64_t a, int64_t b);
int      dowOf(int64_t dayIdx);
int64_t  daysFromCivil(int64_t y, int m, int d);
void     civilFromDays(int64_t z, int64_t *y, int *m, int *d);
int      daysInMonth(int y, int m);
int64_t  wallSecOf(int y, int mon, int day, int h, int mi, int s);
int      nthWeekdayDay(int y, int mon, uint8_t week, uint8_t dow);
int64_t  dstStartUtc(int yy, const DstRule *r, int32_t stdOff);
int64_t  dstEndUtc(int yy, const DstRule *r, int32_t stdOff, int16_t dstMin);
void     tzCompute(int64_t utc);
int64_t  nowUtc();
void     setSystemTime(int64_t epoch, uint8_t src, const char *name, bool valid);
void     fieldsFromWallSec(int64_t ws, int *y, int *mo, int *d, int *h, int *mi, int *s, int *dow);
int64_t  compileEpoch();
void     offStr(int32_t offSec, char *buf, size_t n);
void     sstrlcpy(char *dst, const char *src, size_t n);
void     sstrlcat_(char *dst, const char *src, size_t n);

void     ntpInit();
void     ntpTask();
void     ntpKick();
uint8_t  ntpServerCount();
void     dnsBegin(const char *host);
int8_t   dnsPoll(uint32_t *ip);
uint32_t dnsServerIp(int8_t idx);

void     rtcInit();
void     rtcTask();
bool     rtcReadEpoch(int64_t *epoch);
void     rtcWriteGuarded(int64_t utc);
void     rtcWriteNow(int64_t utc);

uint32_t crc32buf(const void *data, size_t len);
void     defaultsSys();
void     defaultsRelay(uint8_t i);
bool     loadAllCfg();
bool     saveSysCfgNow();
bool     saveRelayCfgNow(uint8_t i);
void     markSysDirty();
void     markRlyDirty(uint8_t i);
void     nvsTask();
void     factoryReset();

bool     patMatch(const TimePattern *p, int sec, int mi, int hr, int dom, int mon, int dow);
int64_t  lastMatchBefore(const TimePattern *p, int64_t wallSec);
void     schedProcessSecond(int64_t utc);
void     schedTick();
void     recomputeOne(uint8_t i);
void     recomputeAll();
bool     relayOutputLevel(uint8_t i);
bool     relayLogicalOn(uint8_t i);
void     applyOutputs();
void     btnTask();
bool     parseSpec(const char *s, int lo, int hi, uint64_t *mask);
void     buildSpec(uint64_t mask, int lo, int hi, String *out);

void     wifiInit();
void     wifiTask();
void     staTryConnect();
void     staApplyLive();
void     apEvaluate();
void     startAp();
void     stopAp();
void     scanStart();
void     scanPoll();
void     onStaConnected();
void     onStaLost();
uint32_t ipToU32(IPAddress ip);
IPAddress u32ToIp(uint32_t v);

void     webSetup();
void     sendPage(const char *pg);
void     handlePortalProbe();
void     handleNotFound();
void     sendDoc(int code, JsonDocument *doc);
void     jErr(int code, const char *msg);
bool     getBodyJson(JsonDocument *doc, int maxLen);
bool     parsePatternObj(JsonVariant src, TimePattern *tp);
void     patternToJson(const TimePattern *tp, JsonObject dst);
bool     parsePairObj(JsonObject src, SchedulePair *sp);
void     pairToJson(const SchedulePair *sp, JsonObject dst);
bool     parseTimeCfg(JsonObject o);
bool     parseStaCfg(JsonObject o);
bool     parseApCfg(JsonObject o);
void     sysCfgToJson(JsonObject o);
void     relayToJson(uint8_t i, JsonObject dst);
bool     parseRelayCfg(JsonObject src, uint8_t i);
bool     parseMiscCfg(JsonObject o);
bool     ipFromStr(const char *s, uint32_t *out);
void     ipToStr(uint32_t v, char *buf, size_t n);
void     handleRoot();
void     handleRelaysPage();
void     handleGpioPage();
void     handleTimePage();
void     handleStaPage();
void     handleApPage();
void     handleStatusPage();
void     handlePortalPage();
void     handleCss();
void     handleJs();
void     handleApiSummary();
void     handleApiRelays();
void     handleApiRelayGet();
void     handleApiRelayName();
void     handleApiRelayPair();
void     handleApiRelayPairsClear();
void     handleApiRelaySchedEnable();
void     handleApiGpioGet();
void     handleApiGpioGlobal();
void     handleApiGpioPolarity();
void     handleApiGpioForce();
void     handleApiGpioRemove();
void     handleApiTimeGet();
void     handleApiTimeConfig();
void     handleApiTimeNtpSync();
void     handleApiTimeBrowser();
void     handleApiTimeRtcRead();
void     handleApiTimeRtcWrite();
void     handleApiWifiScan();
void     handleApiWifiScanStart();
void     handleApiWifiStaGet();
void     handleApiWifiStaPost();
void     handleApiWifiStaConnect();
void     handleApiWifiStaDisconnect();
void     handleApiWifiApGet();
void     handleApiWifiApPost();
void     handleApiStatus();
void     handleApiReboot();
void     handleApiFactory();
void     handleApiSchedRecalc();
void     handleApiConfigExport();
void     handleApiConfigSys();
void     handleApiConfigRelay();

void     pendingReboot(const char *reason, int32_t delayMs);
void     maybeReboot();
void     setup();
void     loop();

int64_t floorDiv(int64_t a, int64_t b) {
  int64_t q = a / b, r = a % b;
  return (r != 0 && ((r < 0) != (b < 0))) ? q - 1 : q;
}

int dowOf(int64_t dayIdx) {
  int64_t x = dayIdx + 4;
  return (int)(x - floorDiv(x, 7) * 7);
}

int64_t daysFromCivil(int64_t y, int m, int d) {
  y -= (m <= 2);
  int64_t era = (y >= 0 ? y : y - 399) / 400;
  uint64_t yoe = (uint64_t)(y - era * 400);
  uint64_t doy = (153u * (uint64_t)(m + (m > 2 ? -3 : 9)) + 2u) / 5u + (uint64_t)d - 1u;
  uint64_t doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

void civilFromDays(int64_t z, int64_t *y, int *m, int *d) {
  z += 719468;
  int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  uint64_t doe = (uint64_t)(z - era * 146097);
  uint64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int64_t yy = (int64_t)yoe + era * 400;
  uint64_t doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
  uint64_t mp = (5u * doy + 2u) / 153u;
  *d = (int)(doy - (153u * mp + 2u) / 5u + 1u);
  *m = (int)(mp + (mp < 10u ? 3u : (uint64_t)-9));
  if (*m <= 2) yy += 1;
  *y = yy;
}

int daysInMonth(int y, int m) {
  static const int dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2) {
    int64_t yy = y;
    if (((yy % 4 == 0) && (yy % 100 != 0)) || (yy % 400 == 0)) return 29;
  }
  return dm[m - 1];
}

int64_t wallSecOf(int y, int mon, int day, int h, int mi, int s) {
  return daysFromCivil((int64_t)y, mon, day) * 86400LL + (int64_t)h * 3600 + mi * 60 + s;
}

int nthWeekdayDay(int y, int mon, uint8_t week, uint8_t dow) {
  int first = dowOf(daysFromCivil((int64_t)y, mon, 1));
  int day = 1 + (((int)dow - first + 7) % 7) + ((int)week - 1) * 7;
  int dim = daysInMonth(y, mon);
  while (day > dim) day -= 7;
  return day;
}

int64_t dstStartUtc(int yy, const DstRule *r, int32_t stdOff) {
  int day = nthWeekdayDay(yy, r->mon, r->week, r->dow);
  return wallSecOf(yy, r->mon, day, r->hour, r->minute, 0) - (int64_t)stdOff;
}

int64_t dstEndUtc(int yy, const DstRule *r, int32_t stdOff, int16_t dstMin) {
  int day = nthWeekdayDay(yy, r->mon, r->week, r->dow);
  return wallSecOf(yy, r->mon, day, r->hour, r->minute, 0) - (int64_t)stdOff - (int64_t)dstMin * 60;
}

void tzCompute(int64_t utc) {
  int32_t stdOff;
  int16_t dstMin = 0;
  DstRule rs, re;
  bool useDst = false;

  if (cfg.tzMode == 1 && cfg.tzPreset < TZ_COUNT) {
    const TzPreset *pz = &TZ_PRESETS[cfg.tzPreset];
    stdOff = pz->stdOff;
    if (pz->dstMin > 0) {
      useDst = true; dstMin = pz->dstMin;
      rs = pz->start; re = pz->end;
    }
  } else {
    stdOff = cfg.gmtOffS;
    if (cfg.dstMode == 1 && cfg.dstShiftMin > 0) {
      useDst = true; dstMin = (int16_t)cfg.dstShiftMin;
      rs.mon = cfg.dsMon; rs.week = cfg.dsWeek; rs.dow = cfg.dsDow; rs.hour = cfg.dsHour; rs.minute = cfg.dsMin;
      re.mon = cfg.deMon; re.week = cfg.deWeek; re.dow = cfg.deDow; re.hour = cfg.deHour; re.minute = cfg.deMin;
    }
  }
  g_stdOff = stdOff; g_dstMin = dstMin;
  dstActive = false; curOffSec = stdOff;
  nextChangeUtc = 0; nextChangeDir = 0;
  if (!useDst || dstMin == 0) return;

  int64_t ly; int lm, ld;
  civilFromDays(floorDiv(utc + stdOff, 86400), &ly, &lm, &ld);
  int y = (int)ly;

  for (int yy = y - 1; yy <= y + 1; ++yy) {
    int64_t s = dstStartUtc(yy, &rs, stdOff);
    int64_t e = dstEndUtc(yy, &re, stdOff, dstMin);
    int64_t pe = (e > s) ? e : dstEndUtc(yy + 1, &re, stdOff, dstMin);
    if (pe <= s) continue;
    if (utc >= s && utc < pe) {
      dstActive = true;
      curOffSec = stdOff + (int32_t)dstMin * 60;
      nextChangeUtc = pe; nextChangeDir = -1;
      return;
    }
  }
  int64_t best = 0; int8_t bd = 0;
  for (int yy = y - 1; yy <= y + 1; ++yy) {
    int64_t s = dstStartUtc(yy, &rs, stdOff);
    int64_t e = dstEndUtc(yy, &re, stdOff, dstMin);
    if (s > utc && (best == 0 || s < best)) { best = s; bd = 1; }
    if (e > utc && (best == 0 || e < best)) { best = e; bd = -1; }
  }
  nextChangeUtc = best; nextChangeDir = bd;
}

int64_t nowUtc() {
  int64_t el = (esp_timer_get_time() - g_baseUs) / 1000000LL;
  if (el < 0) el = 0;
  if (el > 315360000LL) el = 315360000LL;
  int64_t corr = 0;
  if (cfg.driftPerDay != 0)
    corr = ((int64_t)cfg.driftPerDay * el) / 86400LL;
  return g_baseUtc + el + corr;
}

void setSystemTime(int64_t epoch, uint8_t src, const char *name, bool valid) {
  if (epoch < 946684800LL || epoch > 4354819200LL) return;
  bool wasValid = timeValid;
  g_baseUtc = epoch;
  g_baseUs = esp_timer_get_time();
  timeSrc = src;
  sstrlcpy(timeSrcName, name, sizeof(timeSrcName));
  if (valid) timeValid = true;
  if (src != TSRC_BOOT) lastSyncUtc = epoch;
  if (epoch > 0 && epoch < 2147483647LL) {
    struct timeval tv;
    tv.tv_sec = (time_t)epoch;
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
  }
  if (timeValid && !wasValid) recomputeAll();
}

void fieldsFromWallSec(int64_t ws, int *y, int *mo, int *d, int *h, int *mi, int *s, int *dow) {
  int64_t day = floorDiv(ws, 86400);
  int32_t sod = (int32_t)(ws - day * 86400);
  int64_t yy; int mm, dd;
  civilFromDays(day, &yy, &mm, &dd);
  *y = (int)yy; *mo = mm; *d = dd;
  *h = sod / 3600; *mi = (sod % 3600) / 60; *s = sod % 60;
  *dow = dowOf(day);
}

int64_t compileEpoch() {
  static const char monN[12][4] = {"Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"};
  const char *d = __DATE__;
  const char *t = __TIME__;
  int mo = 1;
  for (int i = 0; i < 12; ++i)
    if (d[0]==monN[i][0] && d[1]==monN[i][1] && d[2]==monN[i][2]) { mo = i + 1; break; }
  int dd = (d[4] == ' ') ? (d[5] - '0') : ((d[4] - '0') * 10 + (d[5] - '0'));
  int yy = (d[9]-'0')*1000 + (d[10]-'0')*100 + (d[11]-'0')*10 + (d[12]-'0');
  int hh = (t[0]-'0')*10 + (t[1]-'0');
  int mi = (t[3]-'0')*10 + (t[4]-'0');
  int ss = (t[6]-'0')*10 + (t[7]-'0');
  return wallSecOf(yy, mo, dd, hh, mi, ss);
}

void offStr(int32_t offSec, char *buf, size_t n) {
  int32_t a = offSec; if (a < 0) a = -a;
  int h = a / 3600, m = (a % 3600) / 60;
  snprintf(buf, n, "%c%02d:%02d", (offSec < 0) ? '-' : '+', h, m);
}

void sstrlcpy(char *dst, const char *src, size_t n) {
  if (!n) return;
  size_t i = 0;
  if (src) { for (; i < n - 1 && src[i]; ++i) dst[i] = src[i]; }
  dst[i] = 0;
}

void ntpInit() {
  memset(&ntp, 0, sizeof(ntp));
  memset(&dnsq, 0, sizeof(dnsq));
  ntp.srv = -1;
}

uint8_t ntpServerCount() {
  uint8_t n = 0;
  if (cfg.ntp1[0]) n++;
  if (cfg.ntp2[0]) n++;
  if (cfg.ntp3[0]) n++;
  return n;
}

void ntpKick() {
  ntp.nextTryUtc = 0;
  ntp.state = 0;
}

uint32_t dnsServerIp(int8_t idx) {
  if (idx == 0) {
    if (cfg.staStatic && cfg.staDns1) return cfg.staDns1;
    uint32_t d = ipToU32(WiFi.dnsIP(0));
    if (d) return d;
    if (cfg.staDns1) return cfg.staDns1;
    return 0x08080808u;
  }
  if (idx == 1) {
    if (cfg.staStatic && cfg.staDns2) return cfg.staDns2;
    uint32_t d = ipToU32(WiFi.dnsIP(1));
    if (d) return d;
    if (cfg.staDns2) return cfg.staDns2;
    return 0x01010101u;
  }
  return 0;
}

static void dnsSend() {
  uint8_t q[96];
  int n = 0;
  uint32_t srv = dnsServerIp(dnsq.srvIdx);
  if (srv == 0) return;
  q[n++] = (uint8_t)(dnsq.id >> 8); q[n++] = (uint8_t)dnsq.id;
  q[n++] = 0x01; q[n++] = 0x00;
  q[n++] = 0; q[n++] = 1;
  q[n++] = 0; q[n++] = 0; q[n++] = 0; q[n++] = 0;
  const char *h = dnsq.host;
  while (*h && n < 90) {
    int lbl = 0;
    const char *p = h;
    while (*p && *p != '.') { ++p; ++lbl; }
    if (lbl == 0 || lbl > 63) return;
    q[n++] = (uint8_t)lbl;
    for (int i = 0; i < lbl; ++i) q[n++] = (uint8_t)*h++;
    if (*h == '.') ++h;
    if (!*h) break;
  }
  q[n++] = 0;
  q[n++] = 0; q[n++] = 1;
  q[n++] = 0; q[n++] = 1;
  dnsUdp.beginPacket(u32ToIp(srv), 53);
  dnsUdp.write(q, n);
  dnsUdp.endPacket();
  dnsq.txUs = esp_timer_get_time();
}

void dnsBegin(const char *host) {
  memset(&dnsq, 0, sizeof(dnsq));
  sstrlcpy(dnsq.host, host, sizeof(dnsq.host));
  dnsq.id = (uint16_t)(esp_timer_get_time() & 0xFFFF);
  dnsq.srvIdx = 0;
  dnsq.state = 1;
  dnsSend();
}

int8_t dnsPoll(uint32_t *ip) {
  if (dnsq.state == 0) return 0;
  int64_t now = esp_timer_get_time();
  int pkt = dnsUdp.parsePacket();
  if (pkt > 0) {
    uint8_t b[256];
    int len = dnsUdp.read(b, sizeof(b));
    while (dnsUdp.available() > 0) dnsUdp.read();
    if (len >= 12) {
      uint16_t id = ((uint16_t)b[0] << 8) | b[1];
      uint8_t rcode = b[3] & 0x0F;
      uint16_t an = ((uint16_t)b[6] << 8) | b[7];
      if (id == dnsq.id && rcode == 0 && an > 0) {
        int i = 12;
        while (i < len && b[i]) { if (b[i] >= 0xC0) { ++i; break; } i += b[i] + 1; }
        i += 5;
        for (int a = 0; a < (int)an && i + 12 <= len; ++a) {
          if ((b[i] & 0xC0) == 0xC0) i += 2;
          else { while (i < len && b[i]) i += b[i] + 1; ++i; }
          if (i + 10 > len) break;
          uint16_t tp = ((uint16_t)b[i] << 8) | b[i+1];
          uint16_t cl = ((uint16_t)b[i+2] << 8) | b[i+3];
          uint16_t rd = ((uint16_t)b[i+8] << 8) | b[i+9];
          i += 10;
          if (tp == 1 && cl == 1 && rd == 4 && i + 4 <= len) {
            *ip = ((uint32_t)b[i] << 24) | ((uint32_t)b[i+1] << 16) |
                  ((uint32_t)b[i+2] << 8) | (uint32_t)b[i+3];
            dnsq.state = 0;
            return 1;
          }
          i += rd;
        }
      }
    }
  }
  if (now - dnsq.txUs >= 1200000LL) {
    dnsq.srvIdx++;
    if (dnsServerIp(dnsq.srvIdx) != 0) { dnsSend(); return -1; }
    dnsq.state = 0;
    return 0;
  }
  return -1;
}

static bool ipLiteral(const char *s, uint32_t *ip) { return ipFromStr(s, ip); }

static void ntpBuildAndSend(uint32_t ip) {
  memset(ntpPkt, 0, sizeof(ntpPkt));
  int64_t now = esp_timer_get_time();
  uint64_t secs1900 = (uint64_t)(nowUtc() + 2208988800LL);
  if ((secs1900 & 0x8000000000000000ULL) == 0) {
  }
  uint32_t s32 = (uint32_t)(secs1900 & 0xFFFFFFFFULL);
  uint32_t f32 = (uint32_t)((now % 1000000ULL) * 4294967295ULL / 1000000ULL);
  ntpPkt[0] = 0x1B;
  ntpPkt[40] = (uint8_t)(s32 >> 24); ntpPkt[41] = (uint8_t)(s32 >> 16);
  ntpPkt[42] = (uint8_t)(s32 >> 8);  ntpPkt[43] = (uint8_t)s32;
  ntpPkt[44] = (uint8_t)(f32 >> 24); ntpPkt[45] = (uint8_t)(f32 >> 16);
  ntpPkt[46] = (uint8_t)(f32 >> 8);  ntpPkt[47] = (uint8_t)f32;
  ntp.origTs = ((uint64_t)s32 << 32) | f32;
  ntpUdp.beginPacket(u32ToIp(ip), 123);
  ntpUdp.write(ntpPkt, 48);
  ntpUdp.endPacket();
  ntp.waitUs = now;
}

static void ntpNextServer() {
  int8_t cnt = (int8_t)ntpServerCount();
  if (cnt == 0) { ntp.state = 0; return; }
  for (int k = 0; k < 3; ++k) {
    ntp.srv++;
    if (ntp.srv > 2) ntp.srv = 0;
    const char *h = (ntp.srv == 0) ? cfg.ntp1 : (ntp.srv == 1) ? cfg.ntp2 : cfg.ntp3;
    if (h[0]) {
      uint32_t ip;
      if (ipLiteral(h, &ip)) { ntpBuildAndSend(ip); ntp.state = 2; }
      else { dnsBegin(h); ntp.state = 1; }
      return;
    }
  }
  ntp.state = 0;
}

static void ntpBurstFail() {
  ntp.fails++;
  ntp.state = 0;
  ntp.nextTryUtc = nowUtc() + 30;
}

void ntpTask() {
  if (staState != 2) {
    if (ntp.state != 0) { ntp.state = 0; dnsq.state = 0; }
    return;
  }
  if (!udpBound) {
    ntpUdp.begin(0);
    dnsUdp.begin(0);
    udpBound = true;
  }
  if (ntpServerCount() == 0) return;

  int64_t nowU = nowUtc();

  if (ntp.state == 0) {
    if (nowU < ntp.nextTryUtc) return;
    if (ntp.srv < 0) ntp.srv = 2;
    ntp.attempt = 0;
    ntpNextServer();
    return;
  }

  if (ntp.state == 1) {
    uint32_t ip = 0;
    int8_t r = dnsPoll(&ip);
    if (r == 1) { ntpBuildAndSend(ip); ntp.state = 2; }
    else if (r == 0) {
      ntp.attempt++;
      if (ntp.attempt >= ntpServerCount()) ntpBurstFail();
      else ntpNextServer();
    }
    return;
  }

  int pkt = ntpUdp.parsePacket();
  if (pkt > 0) {
    uint8_t b[48];
    int n = (pkt >= 48) ? ntpUdp.read(b, 48) : 0;
    while (ntpUdp.available() > 0) ntpUdp.read();
    if (n == 48 && ((b[0] & 0x07) == 4)) {
      uint64_t orig = ((uint64_t)b[16] << 56) | ((uint64_t)b[17] << 48) |
                      ((uint64_t)b[18] << 40) | ((uint64_t)b[19] << 32) |
                      ((uint64_t)b[20] << 24) | ((uint64_t)b[21] << 16) |
                      ((uint64_t)b[22] << 8) | (uint64_t)b[23];
      uint32_t ts = ((uint32_t)b[40] << 24) | ((uint32_t)b[41] << 16) |
                    ((uint32_t)b[42] << 8) | (uint32_t)b[43];
      uint32_t fr = ((uint32_t)b[44] << 24) | ((uint32_t)b[45] << 16) |
                    ((uint32_t)b[46] << 8) | (uint32_t)b[47];
      if (orig == ntp.origTs && ts != 0) {
        uint64_t s1900 = ts;
        if ((s1900 & 0x80000000u) == 0) s1900 += 0x100000000ULL;
        int64_t epoch = (int64_t)s1900 - 2208988800LL;
        epoch += (int64_t)((fr >> 31) & 1);
        bool sane = (epoch >= 946684800LL && epoch <= 4354819200LL);
        int64_t delta = epoch - nowU;
        if (delta < 0) delta = -delta;
        if (!sane || (timeValid && delta > 31622400LL)) {
          ntp.attempt++;
          if (ntp.attempt >= ntpServerCount()) ntpBurstFail();
          else ntpNextServer();
          return;
        }
        const char *srvName = (ntp.srv == 0) ? cfg.ntp1 : (ntp.srv == 1) ? cfg.ntp2 : cfg.ntp3;
        setSystemTime(epoch, TSRC_NTP, srvName, true);
        rtcWriteGuarded(epoch);
        ntp.lastOkUtc = epoch;
        sstrlcpy(ntp.lastOkSrv, srvName, sizeof(ntp.lastOkSrv));
        ntp.fails = 0;
        ntp.state = 0;
        ntp.nextTryUtc = epoch + (int64_t)cfg.ntpIntervalS;
        return;
      }
    }
  }

  if (esp_timer_get_time() - ntp.waitUs >= (int64_t)cfg.ntpTimeoutS * 1000000LL) {
    ntp.attempt++;
    if (ntp.attempt >= ntpServerCount()) ntpBurstFail();
    else ntpNextServer();
  }
}

void rtcInit() {
  rtcPresent = false;
  rtcLostPower = false;
  Wire.begin(PIN_RTC_SDA, PIN_RTC_SCL);
  if (rtc.begin(&Wire)) {
    rtcPresent = true;
    if (rtc.lostPower()) {
      rtcLostPower = true;
    } else {
      int64_t e;
      if (rtcReadEpoch(&e)) setSystemTime(e, TSRC_RTC, "DS3231", true);
    }
  }
}

bool rtcReadEpoch(int64_t *epoch) {
  if (!rtcPresent) return false;
  DateTime d = rtc.now();
  uint32_t u = d.unixtime();
  if (u < 946684800UL || u > 4102444800UL) return false;
  *epoch = (int64_t)u;
  return true;
}

void rtcWriteNow(int64_t utc) {
  if (!rtcPresent) return;
  if (utc < 946684800LL || utc > 4102444800LL) return;
  rtc.adjust(DateTime((uint32_t)utc));
  rtcLastWriteUs = esp_timer_get_time();
  rtcLostPower = false;
}

void rtcWriteGuarded(int64_t utc) {
  if (!rtcPresent || !cfg.rtcEnabled) return;
  if (utc < 946684800LL || utc > 4102444800LL) return;
  int64_t now = esp_timer_get_time();
  if (rtcLastWriteUs != 0 && (now - rtcLastWriteUs) < 60000000LL) return;
  int64_t cur;
  if (rtcReadEpoch(&cur)) {
    int64_t d = cur - utc; if (d < 0) d = -d;
    if (d <= 2) { rtcLastWriteUs = now; return; }
  }
  rtc.adjust(DateTime((uint32_t)utc));
  rtcLastWriteUs = now;
  rtcLostPower = false;
}

void rtcTask() {
  if (!rtcPresent) return;
  int64_t now = esp_timer_get_time();
  if (rtcLastResyncUs == 0) { rtcLastResyncUs = now; return; }
  if ((now - rtcLastResyncUs) >= (int64_t)cfg.rtcResyncMin * 60000000LL) {
    rtcLastResyncUs = now;
    if (cfg.rtcEnabled && timeValid) rtcWriteGuarded(nowUtc());
  }
}

uint32_t crc32buf(const void *data, size_t len) {
  static uint32_t tbl[256];
  static bool init = false;
  if (!init) {
    for (uint32_t i = 0; i < 256; ++i) {
      uint32_t c = i;
      for (int k = 0; k < 8; ++k) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
      tbl[i] = c;
    }
    init = true;
  }
  const uint8_t *p = (const uint8_t *)data;
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; ++i) c = tbl[(c ^ p[i]) & 0xFFu] ^ (c >> 8);
  return c ^ 0xFFFFFFFFu;
}

void defaultsSys() {
  memset(&cfg, 0, sizeof(cfg));
  sstrlcpy(cfg.staHost, "relay16", sizeof(cfg.staHost));
  sstrlcpy(cfg.apPass, "relay12345", sizeof(cfg.apPass));
  cfg.apSsid[0] = 0;
  sstrlcpy(cfg.ntp1, "pool.ntp.org", sizeof(cfg.ntp1));
  sstrlcpy(cfg.ntp2, "time.google.com", sizeof(cfg.ntp2));
  sstrlcpy(cfg.ntp3, "time.cloudflare.com", sizeof(cfg.ntp3));
  cfg.apIp   = 0xC0A80401u;
  cfg.apGw   = 0xC0A80401u;
  cfg.apMask = 0xFFFFFF00u;
  cfg.ntpTimeoutS  = 5;
  cfg.ntpIntervalS = 3600;
  cfg.gmtOffS = 28800;
  cfg.dstShiftMin = 60;
  cfg.rtcResyncMin = 60;
  cfg.staEnabled = 1;
  cfg.apPolicy = 0;
  cfg.apChannel = 1;
  cfg.apMaxConn = 4;
  cfg.tzMode = 1;
  cfg.tzPreset = TZ_DEFAULT;
  cfg.dstMode = 0;
  cfg.dsMon = 3;  cfg.dsWeek = 2; cfg.dsDow = 0; cfg.dsHour = 2; cfg.dsMin = 0;
  cfg.deMon = 11; cfg.deWeek = 1; cfg.deDow = 0; cfg.deHour = 2; cfg.deMin = 0;
  cfg.rtcEnabled = 1;
  cfg.globalActiveHigh = 1;
  cfg.driftPerDay = 0;
}

void defaultsRelay(uint8_t i) {
  memset(&rcfg[i], 0, sizeof(rcfg[i]));
  snprintf(rcfg[i].name, sizeof(rcfg[i].name), "Relay %u", (unsigned)(i + 1));
  rcfg[i].polarity = 0;
  rcfg[i].overrideMode = 0;
  rcfg[i].schedulesEnabled = 1;
  for (int p = 0; p < N_PAIRS; ++p) rcfg[i].pairs[p].enabled = 0;
}

static void sanitizeSys() {
  cfg.staSsid[32]=0; cfg.staPass[64]=0; cfg.staHost[32]=0;
  cfg.apSsid[32]=0; cfg.apPass[64]=0;
  cfg.ntp1[48]=0; cfg.ntp2[48]=0; cfg.ntp3[48]=0;
  if (cfg.ntpTimeoutS == 0 || cfg.ntpTimeoutS > 60) cfg.ntpTimeoutS = 5;
  if (cfg.ntpIntervalS < 60 || cfg.ntpIntervalS > 86400) cfg.ntpIntervalS = 3600;
  if (cfg.gmtOffS < -50400 || cfg.gmtOffS > 50400) cfg.gmtOffS = 0;
  if (cfg.dstShiftMin == 0 || cfg.dstShiftMin > 720) cfg.dstShiftMin = 60;
  if (cfg.rtcResyncMin == 0 || cfg.rtcResyncMin > 1440) cfg.rtcResyncMin = 60;
  if (cfg.apPolicy > 2) cfg.apPolicy = 0;
  if (cfg.apChannel < 1 || cfg.apChannel > 13) cfg.apChannel = 1;
  if (cfg.apMaxConn < 1 || cfg.apMaxConn > 8) cfg.apMaxConn = 4;
  if (cfg.tzMode > 1) cfg.tzMode = 1;
  if (cfg.tzPreset >= TZ_COUNT) cfg.tzPreset = TZ_DEFAULT;
  if (cfg.dstMode > 1) cfg.dstMode = 0;
  if (cfg.dsMon < 1 || cfg.dsMon > 12) cfg.dsMon = 3;
  if (cfg.deMon < 1 || cfg.deMon > 12) cfg.deMon = 11;
  if (cfg.dsWeek < 1 || cfg.dsWeek > 5) cfg.dsWeek = 2;
  if (cfg.deWeek < 1 || cfg.deWeek > 5) cfg.deWeek = 1;
  if (cfg.dsDow > 6) cfg.dsDow = 0;
  if (cfg.deDow > 6) cfg.deDow = 0;
  if (cfg.dsHour > 23) cfg.dsHour = 2;
  if (cfg.deHour > 23) cfg.deHour = 2;
  if (cfg.dsMin > 59) cfg.dsMin = 0;
  if (cfg.deMin > 59) cfg.deMin = 0;
  if (cfg.driftPerDay < -30 || cfg.driftPerDay > 30) cfg.driftPerDay = 0;
  if (cfg.globalActiveHigh > 1) cfg.globalActiveHigh = 1;
}

static void sanitizeRelay(uint8_t i) {
  rcfg[i].name[23] = 0;
  if (rcfg[i].polarity > 2) rcfg[i].polarity = 0;
  if (rcfg[i].overrideMode > 2) rcfg[i].overrideMode = 0;
  if (rcfg[i].schedulesEnabled > 1) rcfg[i].schedulesEnabled = 1;
}

static bool readSysBlob(const char *key, SysCfgBlob *b) {
  size_t need = prefs.getBytesLength(key);
  if (need != sizeof(SysCfgBlob)) return false;
  if (prefs.getBytes(key, b, sizeof(SysCfgBlob)) != sizeof(SysCfgBlob)) return false;
  if (b->magic != SYSCFG_MAGIC || b->version != SYSCFG_VER) return false;
  if (b->crc != crc32buf(&b->c, sizeof(SysCfg))) return false;
  return true;
}

bool loadAllCfg() {
  defaultsSys();
  for (uint8_t i = 0; i < N_RELAYS; ++i) defaultsRelay(i);

  if (!prefs.begin(NVS_NS, false)) return false;
  prefsOpen = true;

  SysCfgBlob a, b;
  bool va = readSysBlob("sysA", &a);
  bool vb = readSysBlob("sysB", &b);
  if (va && vb) {
    if (a.gen >= b.gen) { cfg = a.c; sysGen = a.gen; sysLastSlot = 'A'; }
    else               { cfg = b.c; sysGen = b.gen; sysLastSlot = 'B'; }
  } else if (va) { cfg = a.c; sysGen = a.gen; sysLastSlot = 'A'; }
  else if (vb)   { cfg = b.c; sysGen = b.gen; sysLastSlot = 'B'; }
  else           { sysGen = 0; sysLastSlot = 'B'; }
  sanitizeSys();

  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    char key[8];
    snprintf(key, sizeof(key), "rly%02u", (unsigned)i);
    RlyCfgBlob rb;
    size_t need = prefs.getBytesLength(key);
    if (need == sizeof(RlyCfgBlob) &&
        prefs.getBytes(key, &rb, sizeof(RlyCfgBlob)) == sizeof(RlyCfgBlob) &&
        rb.magic == RLYCFG_MAGIC && rb.version == RLYCFG_VER &&
        rb.crc == crc32buf(&rb.c, sizeof(RelayCfg))) {
      rcfg[i] = rb.c;
      sanitizeRelay(i);
    }
  }
  return true;
}

bool saveSysCfgNow() {
  if (!prefsOpen) return false;
  char slot = (sysLastSlot == 'A') ? 'B' : 'A';
  SysCfgBlob blob;
  blob.magic = SYSCFG_MAGIC;
  blob.version = SYSCFG_VER;
  blob.gen = (uint16_t)(sysGen + 1);
  blob.crc = crc32buf(&cfg, sizeof(SysCfg));
  blob.c = cfg;
  const char *key = (slot == 'A') ? "sysA" : "sysB";
  if (prefs.putBytes(key, &blob, sizeof(blob)) != sizeof(blob)) return false;
  SysCfgBlob chk;
  if (!readSysBlob(key, &chk) || chk.gen != blob.gen) return false;
  sysGen = blob.gen;
  sysLastSlot = slot;
  sysDirty = 0;
  lastNvsWriteUs = esp_timer_get_time();
  nvsWrites++;
  return true;
}

bool saveRelayCfgNow(uint8_t i) {
  if (!prefsOpen || i >= N_RELAYS) return false;
  char key[8];
  snprintf(key, sizeof(key), "rly%02u", (unsigned)i);
  RlyCfgBlob blob;
  blob.magic = RLYCFG_MAGIC;
  blob.version = RLYCFG_VER;
  blob.rsv = 0;
  blob.crc = crc32buf(&rcfg[i], sizeof(RelayCfg));
  blob.c = rcfg[i];
  if (prefs.putBytes(key, &blob, sizeof(blob)) != sizeof(blob)) return false;
  RlyCfgBlob chk;
  if (prefs.getBytes(key, &chk, sizeof(chk)) != sizeof(chk)) return false;
  if (chk.crc != blob.crc) return false;
  lastNvsWriteUs = esp_timer_get_time();
  nvsWrites++;
  return true;
}

void markSysDirty() {
  sysDirty = 1;
  sysDirtyUs = esp_timer_get_time();
}

void markRlyDirty(uint8_t i) {
  if (i >= N_RELAYS) return;
  rlyDirtyBits |= (uint16_t)(1u << i);
  rlyDirtyUs = esp_timer_get_time();
}

void nvsTask() {
  if (!prefsOpen) return;
  int64_t now = esp_timer_get_time();
  if (sysDirty) {
    if ((now - sysDirtyUs >= 5000000LL) ||
        (lastNvsWriteUs && (now - lastNvsWriteUs >= 30000000LL)) ||
        (!lastNvsWriteUs && (now - sysDirtyUs >= 5000000LL))) {
      saveSysCfgNow();
    }
  }
  if (rlyDirtyBits) {
    if ((now - rlyDirtyUs >= 5000000LL) ||
        (lastNvsWriteUs && (now - lastNvsWriteUs >= 30000000LL)) ||
        (!lastNvsWriteUs && (now - rlyDirtyUs >= 5000000LL))) {
      for (uint8_t i = 0; i < N_RELAYS; ++i)
        if (rlyDirtyBits & (1u << i)) saveRelayCfgNow(i);
      rlyDirtyBits = 0;
    }
  }
}

void factoryReset() {
  sysDirty = 0;
  rlyDirtyBits = 0;
  if (prefsOpen) prefs.clear();
  pendingReboot("factory", 250);
}

bool patMatch(const TimePattern *p, int sec, int mi, int hr, int dom, int mon, int dow) {
  if (!((p->sec >> sec) & 1ULL)) return false;
  if (!((p->min >> mi) & 1ULL)) return false;
  if (!((p->hr  >> hr)  & 1UL)) return false;
  if (!((p->dom >> dom) & 1UL)) return false;
  if (!((p->mon >> mon) & 1UL)) return false;
  if (!((p->dow >> dow) & 1UL)) return false;
  return true;
}

int64_t lastMatchBefore(const TimePattern *p, int64_t wallSec) {
  int64_t dayIdx = floorDiv(wallSec, 86400);
  int32_t sod = (int32_t)(wallSec - dayIdx * 86400);
  int curH = sod / 3600, curM = (sod % 3600) / 60, curS = sod % 60;
  int64_t endDay = dayIdx - (int64_t)LOOKBACK_DAYS;
  for (int64_t d = dayIdx; d > endDay; --d) {
    int64_t y; int m, dd;
    civilFromDays(d, &y, &m, &dd);
    int w = dowOf(d);
    if (!((p->mon >> m) & 1UL)) continue;
    if (!((p->dom >> dd) & 1UL)) continue;
    if (!((p->dow >> w) & 1UL)) continue;
    int hMax = (d == dayIdx) ? curH : 23;
    for (int h = hMax; h >= 0; --h) {
      if (!((p->hr >> h) & 1UL)) continue;
      int mMax = (d == dayIdx && h == curH) ? curM : 59;
      for (int mi = mMax; mi >= 0; --mi) {
        if (!((p->min >> mi) & 1ULL)) continue;
        int sMax = (d == dayIdx && h == curH && mi == curM) ? curS : 59;
        for (int s = sMax; s >= 0; --s)
          if ((p->sec >> s) & 1ULL)
            return d * 86400 + (int64_t)h * 3600 + mi * 60 + s;
      }
    }
  }
  return -1;
}

void schedProcessSecond(int64_t utc) {
  tzCompute(utc);
  int64_t wall = utc + (int64_t)curOffSec;
  int y, mo, d, h, mi, s, dow;
  fieldsFromWallSec(wall, &y, &mo, &d, &h, &mi, &s, &dow);
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    if (!rcfg[i].schedulesEnabled) continue;
    int winPair = -1;
    uint8_t winType = 0;
    for (int p = N_PAIRS - 1; p >= 0; --p) {
      const SchedulePair *pr = &rcfg[i].pairs[p];
      if (!pr->enabled) continue;
      bool stop = patMatch(&pr->stop, s, mi, h, d, mo, dow);
      if (stop) { winPair = p; winType = 2; break; }
      bool start = patMatch(&pr->start, s, mi, h, d, mo, dow);
      if (start) { winPair = p; winType = 1; break; }
    }
    if (winPair >= 0) desiredOn[i] = (winType == 1);
  }
}

void recomputeOne(uint8_t i) {
  if (!timeValid) { desiredOn[i] = false; return; }
  int64_t now = nowUtc();
  tzCompute(now);
  int64_t wall = now + (int64_t)curOffSec;
  int64_t bestT = -1;
  int bestPair = -1;
  uint8_t bestType = 0;
  for (int p = 0; p < N_PAIRS; ++p) {
    if (!rcfg[i].pairs[p].enabled) continue;
    int64_t ts = lastMatchBefore(&rcfg[i].pairs[p].start, wall);
    int64_t te = lastMatchBefore(&rcfg[i].pairs[p].stop,  wall);
    int64_t tp; uint8_t ty;
    if (ts >= te) { tp = ts; ty = 1; } else { tp = te; ty = 2; }
    if (tp < 0) continue;
    if (tp > bestT || (tp == bestT && p >= bestPair)) { bestT = tp; bestPair = p; bestType = ty; }
  }
  desiredOn[i] = (bestT >= 0) && (bestType == 1);
  if (!rcfg[i].schedulesEnabled) desiredOn[i] = false;
}

void recomputeAll() {
  if (!timeValid) {
    for (uint8_t i = 0; i < N_RELAYS; ++i) desiredOn[i] = false;
    applyOutputs();
    return;
  }
  for (uint8_t i = 0; i < N_RELAYS; ++i) recomputeOne(i);
  applyOutputs();
  lastEvalSec = nowUtc();
}

void schedTick() {
  int64_t now = nowUtc();
  curUtcSec = now;
  if (!timeValid) {
    lastEvalSec = now;
    return;
  }
  if (lastEvalSec == 0) { lastEvalSec = now - 1; }
  int64_t delta = now - lastEvalSec;
  if (delta <= 0) {
    if (delta < -1) recomputeAll();
    lastEvalSec = now;
    return;
  }
  if (delta > 5) { recomputeAll(); lastEvalSec = now; return; }
  for (int64_t t = lastEvalSec + 1; t <= now; ++t) schedProcessSecond(t);
  applyOutputs();
  lastEvalSec = now;
}

bool relayOutputLevel(uint8_t i) {
  bool on = desiredOn[i];
  if (rcfg[i].overrideMode == 1) on = true;
  else if (rcfg[i].overrideMode == 2) on = false;
  bool activeHigh;
  if (rcfg[i].polarity == 1) activeHigh = true;
  else if (rcfg[i].polarity == 2) activeHigh = false;
  else activeHigh = (cfg.globalActiveHigh == 1);
  return on ? activeHigh : !activeHigh;
}

void applyOutputs() {
  for (uint8_t i = 0; i < N_RELAYS; ++i)
    digitalWrite(RELAY_PINS[i], relayOutputLevel(i) ? HIGH : LOW);
}

void btnTask() {
  static uint8_t lastRaw = 1, stab = 0, state = 0;
  static int64_t downUs = 0;
  static bool fired = false;
  uint8_t raw = (digitalRead(PIN_RESET_BTN) == LOW) ? 1 : 0;
  if (raw != lastRaw) { lastRaw = raw; stab = 0; }
  if (stab < 50) { ++stab; return; }
  if (raw != state) {
    state = raw;
    if (raw) { downUs = esp_timer_get_time(); fired = false; }
  }
  if (raw && !fired && (esp_timer_get_time() - downUs) >= 5000000LL) {
    fired = true;
    factoryReset();
  }
}

bool parseSpec(const char *s, int lo, int hi, uint64_t *mask) {
  uint64_t m = 0;
  const char *p = s;
  if (!p) return false;
  while (*p == ' ') ++p;
  if (*p == 0) return false;
  if (p[0] == '*' && p[1] == 0) {
    for (int v = lo; v <= hi; ++v) m |= (1ULL << v);
    *mask = m;
    return true;
  }
  while (*p) {
    while (*p == ' ') ++p;
    if (!*p) break;
    const char *st = p;
    long a = strtol(p, (char **)&p, 10);
    if (p == st) return false;
    while (*p == ' ') ++p;
    if (*p == '-') {
      ++p;
      while (*p == ' ') ++p;
      const char *st2 = p;
      long b = strtol(p, (char **)&p, 10);
      if (p == st2) return false;
      if (a < lo || b > hi || a > b) return false;
      for (long v = a; v <= b; ++v) m |= (1ULL << v);
    } else {
      if (a < lo || a > hi) return false;
      m |= (1ULL << a);
    }
    while (*p == ' ') ++p;
    if (*p == ',') { ++p; continue; }
    if (*p == 0) break;
    return false;
  }
  if (m == 0) return false;
  *mask = m;
  return true;
}

void buildSpec(uint64_t mask, int lo, int hi, String *out) {
  uint64_t full = 0;
  for (int v = lo; v <= hi; ++v) full |= (1ULL << v);
  *out = "";
  if (mask == full) { *out += "*"; return; }
  bool first = true;
  int run = -2;
  for (int v = lo; v <= hi; ++v) {
    bool bit = ((mask >> v) & 1ULL) != 0;
    if (bit) {
      if (run < 0) run = v;
    } else {
      if (run >= 0) {
        int end = v - 1;
        if (!first) *out += ",";
        *out += String(run);
        if (end != run) { *out += "-"; *out += String(end); }
        first = false;
        run = -2;
      }
    }
  }
  if (run >= 0) {
    int end = hi;
    if (!first) *out += ",";
    *out += String(run);
    if (end != run) { *out += "-"; *out += String(end); }
  }
  if (out->length() == 0) *out += "*";
}

uint32_t ipToU32(IPAddress ip) {
  return ((uint32_t)ip[0] << 24) | ((uint32_t)ip[1] << 16) |
         ((uint32_t)ip[2] << 8) | (uint32_t)ip[3];
}

IPAddress u32ToIp(uint32_t v) {
  return IPAddress((uint8_t)(v >> 24), (uint8_t)(v >> 16), (uint8_t)(v >> 8), (uint8_t)v);
}

void staTryConnect() {
  if (!cfg.staEnabled || !cfg.staSsid[0]) return;
  if (WiFi.status() == WL_CONNECTED) return;
  int64_t now = esp_timer_get_time();
  if (staLastBeginUs && (now - staLastBeginUs) < 15000000LL) {
    WiFi.reconnect();
    return;
  }
  WiFi.disconnect(false, false);
  WiFi.setHostname(cfg.staHost);
  if (cfg.staStatic && cfg.staIp) {
    WiFi.config(u32ToIp(cfg.staIp), u32ToIp(cfg.staGw), u32ToIp(cfg.staMask),
                u32ToIp(cfg.staDns1), u32ToIp(cfg.staDns2));
  } else {
    WiFi.config(u32ToIp(0), u32ToIp(0), u32ToIp(0), u32ToIp(0), u32ToIp(0));
  }
  if (cfg.staPass[0]) WiFi.begin(cfg.staSsid, cfg.staPass);
  else WiFi.begin(cfg.staSsid);
  staLastBeginUs = now;
}

void staApplyLive() {
  staLastBeginUs = 0;
  staHold = false;
  staTryConnect();
}

void onStaConnected() {
  ntpKick();
  ntpUdp.stop(); dnsUdp.stop();
  ntpUdp.begin(0); dnsUdp.begin(0);
  udpBound = true;
  if (!mdnsUp) {
    if (MDNS.begin(cfg.staHost)) {
      mdnsUp = true;
      MDNS.addService("http", "tcp", WEB_PORT);
    }
  }
}

void onStaLost() { }

void startAp() {
  WiFi.mode(WIFI_AP_STA);
  char ssid[33];
  if (cfg.apSsid[0]) {
    sstrlcpy(ssid, cfg.apSsid, sizeof(ssid));
  } else {
    String mac = WiFi.macAddress();
    const char *m = mac.c_str();
    size_t ml = mac.length();
    char hex[13]; int hk = 0;
    for (size_t k = 0; k < ml && hk < 12; ++k)
      if (m[k] != ':') hex[hk++] = m[k];
    sstrlcpy(ssid, "RM16-", sizeof(ssid));
    if (hk >= 6) {
      char tail[7];
      for (int k = 0; k < 6; ++k) tail[k] = hex[hk - 6 + k];
      tail[6] = 0;
      sstrlcat_(ssid, tail, sizeof(ssid));
    }
    sstrlcpy(cfg.apSsid, ssid, sizeof(cfg.apSsid));
    markSysDirty();
  }
  WiFi.softAPConfig(u32ToIp(cfg.apIp), u32ToIp(cfg.apGw), u32ToIp(cfg.apMask));
  bool ok;
  if (cfg.apOpen || !cfg.apPass[0])
    ok = WiFi.softAP(ssid, NULL, cfg.apChannel, cfg.apHidden, cfg.apMaxConn);
  else
    ok = WiFi.softAP(ssid, cfg.apPass, cfg.apChannel, cfg.apHidden, cfg.apMaxConn);
  if (ok) {
    apActive = true;
    dnsServer.start(DNS_PORT, "*", u32ToIp(cfg.apIp));
    dnsRunning = true;
  }
  apLastChangeUs = esp_timer_get_time();
}

void stopAp() {
  if (dnsRunning) { dnsServer.stop(); dnsRunning = false; }
  WiFi.softAPdisconnect(true);
  apActive = false;
  apLastChangeUs = esp_timer_get_time();
}

void apEvaluate() {
  bool want;
  if (cfg.apPolicy == 2) want = false;
  else if (cfg.apPolicy == 1) want = true;
  else want = (staState != 2);
  if (cfg.apPolicy != 2 && (!cfg.staEnabled || !cfg.staSsid[0])) want = true;

  int64_t now = esp_timer_get_time();
  if (want && !apActive) {
    if (apLastChangeUs == 0 || (now - apLastChangeUs) >= 2000000LL) startAp();
  } else if (!want && apActive) {
    if (apGraceUntilUs == 0 || now >= apGraceUntilUs) stopAp();
  }
}

void wifiTask() {
  int64_t nowUs = esp_timer_get_time();
  if (cfg.staEnabled && cfg.staSsid[0] && !staHold) {
    if (WiFi.status() == WL_CONNECTED) {
      if (staState != 2) { staState = 2; onStaConnected(); }
    } else {
      if (staState != 1) { staState = 1; onStaLost(); }
      if (nowUs - staLastTryUs >= 1000000LL) {
        staLastTryUs = nowUs;
        staTryConnect();
      }
    }
  } else {
    if (staState != 0) {
      staState = 0;
      if (!staHold) WiFi.disconnect(false, false);
    }
  }
  apEvaluate();
}

void scanStart() {
  if (scanBusy) return;
  if (WiFi.getMode() == WIFI_MODE_NULL) WiFi.mode(WIFI_AP_STA);
  WiFi.scanNetworks(true);
  scanBusy = true;
  scanStartUs = esp_timer_get_time();
}

void scanPoll() {
  if (!scanBusy) return;
  int8_t n = WiFi.scanComplete();
  if (n >= 0) {
    scanNetCount = 0;
    for (int i = 0; i < n && scanNetCount < 32; ++i) {
      String ss = WiFi.SSID((uint8_t)i);
      sstrlcpy(scanNets[scanNetCount].ssid, ss.c_str(), 33);
      scanNets[scanNetCount].rssi = WiFi.RSSI((uint8_t)i);
      scanNets[scanNetCount].sec  = (WiFi.encryptionType((uint8_t)i) != WIFI_AUTH_OPEN);
      scanNets[scanNetCount].ch   = (uint8_t)WiFi.channel((uint8_t)i);
      scanNetCount++;
    }
    WiFi.scanDelete();
    scanBusy = false;
  } else if (n == -2) {
    scanNetCount = 0;
    scanBusy = false;
  } else if (esp_timer_get_time() - scanStartUs > 20000000LL) {
    WiFi.scanDelete();
    scanNetCount = 0;
    scanBusy = false;
  }
}

void wifiInit() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  WiFi.setHostname(cfg.staHost);
  WiFi.mode(WIFI_AP_STA);
  staLastBeginUs = 0;
  staLastTryUs = 0;
  if (cfg.staEnabled && cfg.staSsid[0] && !staHold) staTryConnect();
  apEvaluate();
  if (!mdnsUp) {
    if (MDNS.begin(cfg.staHost)) {
      mdnsUp = true;
      MDNS.addService("http", "tcp", WEB_PORT);
    }
  }
}

void sstrlcat_(char *dst, const char *src, size_t n) {
  if (!n) return;
  size_t l = 0;
  while (l < n - 1 && dst[l]) ++l;
  size_t i = 0;
  while (l < n - 1 && src[i]) dst[l++] = src[i++];
  dst[l] = 0;
}

static const char PG_INDEX[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 Dashboard</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="index"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_RELAYS[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 Relay Schedules</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="relays"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_GPIO[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 GPIO Control</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="gpio"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_TIME[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 Time Settings</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="time"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_STA[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 WiFi Station</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="wifi-sta"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_AP[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 WiFi Access Point</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="wifi-ap"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_STATUS[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0f1420"><title>RM16 System Status</title><link rel="stylesheet" href="/style.css"></head>
<body data-page="status"><div id="app"></div><script src="/app.js"></script></body></html>)HTML";

static const char PG_CSS[] PROGMEM = R"CSS(
:root{--bg:#0f1420;--card:#171f2e;--card2:#1c2638;--line:#26324a;--tx:#e8ecf4;--tx2:#93a1b8;--acc:#4da3ff;--ok:#2ecc71;--warn:#f5c451;--bad:#ff5d6c;--r:14px}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
html{font-size:16px}
body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;min-height:100vh;padding-bottom:48px}
a{color:var(--acc);text-decoration:none}
.wrap{max-width:1080px;margin:0 auto;padding:0 14px}
header{position:sticky;top:0;z-index:50;background:rgba(15,20,32,.96);backdrop-filter:blur(8px);border-bottom:1px solid var(--line)}
.hrow{display:flex;align-items:center;justify-content:space-between;gap:10px;padding:10px 14px;max-width:1080px;margin:0 auto}
.brand{display:flex;align-items:center;gap:9px;font-weight:700;font-size:1.05rem;letter-spacing:.4px;color:var(--tx)}
.brand .dot{width:12px;height:12px;border-radius:4px;background:linear-gradient(135deg,var(--acc),#8f6bff);box-shadow:0 0 10px #4da3ff88}
.hclock{display:flex;flex-direction:column;align-items:flex-end;line-height:1.25}
.hclock .t{font-variant-numeric:tabular-nums;font-weight:600;font-size:1.02rem;letter-spacing:.5px}
.hclock .d{font-size:.74rem;color:var(--tx2);letter-spacing:1.2px;font-weight:600}
.burger{background:none;border:1px solid var(--line);color:var(--tx);border-radius:10px;width:42px;height:42px;font-size:1.15rem;cursor:pointer;display:none}
nav.menu{display:flex;gap:6px;flex-wrap:wrap;padding:0 14px 10px;max-width:1080px;margin:0 auto}
nav.menu a{padding:7px 13px;border-radius:999px;font-size:.86rem;font-weight:600;color:var(--tx2);border:1px solid transparent}
nav.menu a.on{color:#fff;background:var(--acc);border-color:var(--acc)}
nav.menu a:not(.on):hover{color:var(--tx);border-color:var(--line)}
main{padding:16px 0}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:16px;margin-bottom:14px}
.card h2{font-size:1rem;margin-bottom:4px;letter-spacing:.2px}
.card .sub{color:var(--tx2);font-size:.82rem;margin-bottom:12px}
.grid{display:grid;gap:14px}
@media(min-width:720px){.g2{grid-template-columns:1fr 1fr}.g3{grid-template-columns:1fr 1fr 1fr}}
.row{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.spread{display:flex;align-items:center;justify-content:space-between;gap:10px;flex-wrap:wrap}
.muted{color:var(--tx2);font-size:.85rem}
.mono{font-variant-numeric:tabular-nums}
label.f{display:block;margin:10px 0 4px;font-size:.82rem;color:var(--tx2);font-weight:600}
input[type=text],input[type=password],input[type=number],select{width:100%;padding:11px 12px;border-radius:10px;border:1px solid var(--line);background:var(--card2);color:var(--tx);font-size:16px;outline:none}
input:focus,select:focus{border-color:var(--acc)}
input[type=checkbox]{width:22px;height:22px;accent-color:var(--acc)}
.inline{display:flex;gap:8px;flex-wrap:wrap}
.inline>*{flex:1;min-width:110px}
button.btn{padding:11px 18px;border-radius:10px;border:1px solid var(--line);background:var(--card2);color:var(--tx);font-size:.92rem;font-weight:600;cursor:pointer;transition:filter .12s}
button.btn:hover{filter:brightness(1.18)}
button.btn:disabled{opacity:.45;cursor:default}
button.btn.pri{background:var(--acc);border-color:var(--acc);color:#08111f}
button.btn.ok{background:var(--ok);border-color:var(--ok);color:#07220f}
button.btn.bad{background:var(--bad);border-color:var(--bad);color:#2a0509}
button.btn.sm{padding:7px 12px;font-size:.82rem}
.seg{display:inline-flex;border:1px solid var(--line);border-radius:10px;overflow:hidden}
.seg button{border:0;background:var(--card2);color:var(--tx2);padding:8px 13px;font-size:.84rem;font-weight:700;cursor:pointer}
.seg button.on-a{background:var(--acc);color:#08111f}
.seg button.on-g{background:var(--ok);color:#07220f}
.seg button.on-r{background:var(--bad);color:#2a0509}
.rlist{display:grid;gap:12px}
@media(min-width:900px){.rlist{grid-template-columns:1fr 1fr}}
.rcard{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:14px}
.rhead{display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.rname{font-weight:700;font-size:1rem;cursor:pointer;padding:2px 4px;border-radius:6px;border:1px dashed transparent}
.rname:hover{border-color:var(--line)}
.led{width:13px;height:13px;border-radius:50%;background:#3a4560;flex:none;box-shadow:0 0 0 3px #00000030}
.led.on{background:var(--ok);box-shadow:0 0 10px var(--ok)}
.pinb{font-size:.7rem;color:var(--tx2);border:1px solid var(--line);padding:2px 7px;border-radius:6px;font-weight:600}
.exp{margin-left:auto;background:none;border:0;color:var(--tx2);font-size:1.1rem;cursor:pointer;padding:8px 10px}
.editic{background:none;border:0;color:var(--tx2);cursor:pointer;font-size:.9rem;padding:6px 8px}
.rbody{display:none;margin-top:12px}
.rcard.open .rbody{display:block}
.pairs{display:flex;flex-direction:column;gap:10px;margin-top:10px}
.pair{background:var(--card2);border:1px solid var(--line);border-radius:12px;padding:12px}
.pair .phead{display:flex;align-items:center;gap:10px}
.pair.dim{opacity:.55}
.pcol{display:grid;gap:8px;margin-top:10px}
@media(min-width:640px){.pcol{grid-template-columns:1fr 1fr}}
.flab{font-size:.7rem;font-weight:800;letter-spacing:1.1px;color:var(--tx2);margin-bottom:2px}
.grp{border:1px solid var(--line);border-radius:10px;background:var(--card)}
.grp>.ghead{display:flex;align-items:center;gap:8px;width:100%;background:none;border:0;color:var(--tx);padding:9px 11px;cursor:pointer;font-size:.82rem;font-weight:600;text-align:left}
.grp .val{color:var(--acc);font-family:ui-monospace,Menlo,Consolas,monospace;font-size:.8rem;word-break:break-all;flex:1}
.grp .arr{color:var(--tx2);transition:transform .15s}
.grp.open .arr{transform:rotate(90deg)}
.gbody{display:none;padding:4px 10px 10px;max-height:190px;overflow:auto;border-top:1px solid var(--line)}
.grp.open .gbody{display:block}
.chips{display:flex;flex-wrap:wrap;gap:5px;padding-top:6px}
.chip{min-width:30px;min-height:30px;padding:3px 6px;border-radius:8px;border:1px solid var(--line);background:var(--card);color:var(--tx2);font-size:.74rem;font-weight:700;cursor:pointer;touch-action:manipulation;font-variant-numeric:tabular-nums}
.chip.sel{background:var(--acc);border-color:var(--acc);color:#08111f}
.chip.wide{min-width:44px}
.kv{display:flex;justify-content:space-between;gap:12px;padding:7px 0;border-bottom:1px dashed var(--line);font-size:.88rem}
.kv:last-child{border-bottom:0}
.kv b{font-weight:600;text-align:right;word-break:break-all}
.net{display:flex;align-items:center;gap:10px;width:100%;background:var(--card2);border:1px solid var(--line);border-radius:10px;padding:11px 12px;margin-bottom:8px;color:var(--tx);cursor:pointer;text-align:left}
.net:hover{border-color:var(--acc)}
.net .sig{font-size:.72rem;color:var(--tx2);min-width:52px}
.net .lk{color:var(--warn);font-size:.75rem;margin-left:auto}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(150px,1fr));gap:10px}
.tile{background:var(--card2);border:1px solid var(--line);border-radius:12px;padding:10px 12px}
.tile .tn{font-weight:700;font-size:.9rem;margin-bottom:6px;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.st{display:inline-block;font-size:.7rem;font-weight:800;letter-spacing:.8px;padding:3px 8px;border-radius:6px}
.st.auto{color:var(--acc);background:#4da3ff22}
.st.on{color:var(--ok);background:#2ecc7122}
.st.off{color:var(--tx2);background:#93a1b822}
.st.forced{color:var(--warn);background:#f5c45122}
#toasts{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);z-index:200;display:flex;flex-direction:column;gap:8px;width:min(92vw,420px)}
.toast{background:var(--card2);border:1px solid var(--line);border-left:4px solid var(--acc);border-radius:10px;padding:12px 14px;font-size:.88rem;box-shadow:0 8px 24px #0009;animation:tin .18s ease-out}
.toast.ok{border-left-color:var(--ok)}
.toast.bad{border-left-color:var(--bad)}
@keyframes tin{from{opacity:0;transform:translateY(8px)}to{opacity:1;transform:none}}
#mback{position:fixed;inset:0;background:#000a;z-index:150;display:none;align-items:center;justify-content:center;padding:20px}
#mback.show{display:flex}
.mbox{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:20px;width:min(92vw,380px)}
.mbox h3{margin-bottom:8px;font-size:1rem}
.mbox p{color:var(--tx2);font-size:.9rem;margin-bottom:16px;white-space:pre-line}
.mbox .row{justify-content:flex-end}
.bigclock{font-variant-numeric:tabular-nums;font-size:2.2rem;font-weight:700;letter-spacing:1px}
.badge{display:inline-block;font-size:.68rem;font-weight:800;letter-spacing:.8px;padding:3px 8px;border-radius:6px;background:var(--card2);border:1px solid var(--line);color:var(--tx2)}
.badge.acc{color:var(--acc);border-color:#4da3ff55}
.badge.ok{color:var(--ok);border-color:#2ecc7155}
.badge.bad{color:var(--bad);border-color:#ff5d6c55}
.warnbox{border:1px solid #f5c45155;background:#f5c45110;border-radius:10px;padding:10px 12px;font-size:.82rem;color:#f5c451;margin-bottom:12px}
@media(max-width:720px){
 .burger{display:block}
 nav.menu{display:none;flex-direction:column;padding-bottom:14px}
 nav.menu.open{display:flex}
 nav.menu a{padding:12px 14px;border-radius:10px;font-size:.95rem}
 .hclock .t{font-size:.95rem}
}
)CSS";

static const char PG_PORTAL[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>RM16 Setup</title>
<style>
:root{--bg:#0f1420;--card:#171f2e;--line:#26324a;--tx:#e8ecf4;--tx2:#93a1b8;--acc:#4da3ff;--ok:#2ecc71;--bad:#ff5d6c}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:var(--bg);color:var(--tx);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Arial,sans-serif;min-height:100vh;display:flex;justify-content:center;padding:24px 16px}
.box{width:100%;max-width:420px}
h1{font-size:1.25rem;display:flex;align-items:center;gap:10px;margin-bottom:6px}
h1 .dot{width:14px;height:14px;border-radius:5px;background:linear-gradient(135deg,var(--acc),#8f6bff)}
.sub{color:var(--tx2);font-size:.85rem;margin-bottom:18px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-bottom:14px}
label{display:block;margin:12px 0 5px;font-size:.82rem;color:var(--tx2);font-weight:600}
input[type=text],input[type=password]{width:100%;padding:12px;border-radius:10px;border:1px solid var(--line);background:#1c2638;color:var(--tx);font-size:16px;outline:none}
input:focus{border-color:var(--acc)}
button{width:100%;margin-top:14px;padding:13px;border-radius:10px;border:0;background:var(--acc);color:#08111f;font-size:1rem;font-weight:700;cursor:pointer}
button:disabled{opacity:.5}
button.ghost{background:#1c2638;color:var(--tx);border:1px solid var(--line)}
.net{display:flex;align-items:center;gap:10px;width:100%;background:#1c2638;border:1px solid var(--line);border-radius:10px;padding:12px;margin-bottom:8px;color:var(--tx);cursor:pointer;text-align:left;font-size:.9rem}
.net.selnet{border-color:var(--acc)}
.net .sig{font-size:.72rem;color:var(--tx2);min-width:56px}
.net .lk{color:var(--bad);font-size:.75rem;margin-left:auto}
#status{font-size:.85rem;color:var(--tx2);line-height:1.5;word-break:break-all}
#status.ok{color:var(--ok)}#status.bad{color:var(--bad)}
.foot{color:var(--tx2);font-size:.75rem;text-align:center;line-height:1.6}
</style></head><body><div class="box">
<h1><span class="dot"></span>RM16 Relay Manager</h1>
<div class="sub">Connect this device to your WiFi network</div>
<div class="card">
<label>Available networks</label>
<div id="nets"><div class="sub" style="margin:8px 0 0">Scanning...</div></div>
<label>Network name (SSID)</label>
<input type="text" id="ssid" maxlength="32" autocomplete="off">
<label>Password (leave empty for open networks)</label>
<input type="password" id="pass" maxlength="64" autocomplete="off">
<button id="go">Connect</button>
<button class="ghost" id="rescan">Rescan</button>
</div>
<div class="card"><div id="status">Pick a network, enter its password, then press Connect.</div></div>
<div class="foot">Full control panel: <b>http://192.168.4.1/</b> while connected to this hotspot.<br>RM16 v1.0.0</div>
</div>
<script>
(function(){
"use strict";
var $=function(i){return document.getElementById(i)};
function setStatus(t,cls){var s=$("status");s.textContent=t;s.className=cls||""}
function api(p,m,b){return fetch(p,{method:m||"GET",headers:b?{"Content-Type":"application/json"}:undefined,body:b?JSON.stringify(b):undefined}).then(function(r){return r.json().catch(function(){return{}})}).catch(function(){return{}})}
function scan(){
  $("nets").innerHTML='<div class="sub" style="margin:8px 0 0">Scanning...</div>';
  api("/api/wifi/scanStart","POST").then(poll);
  function poll(){
    api("/api/wifi/scan").then(function(j){
      if(j.scanning){setTimeout(poll,1500);return}
      var n=j.nets||[];
      if(!n.length){$("nets").innerHTML='<div class="sub" style="margin:8px 0 0">No networks found.</div>';return}
      var h="";
      for(var i=0;i<n.length;i++){h+='<button type="button" class="net" data-s="'+esc(n[i].s)+'"><span class="sig">'+n[i].r+' dBm</span><span>'+esc(n[i].s||"(hidden)")+'</span>'+(n[i].sec?'<span class="lk">secured</span>':'<span class="lk" style="color:#2ecc71">open</span>')+'</button>'}
      $("nets").innerHTML=h;
      var bs=$("nets").querySelectorAll(".net");
      for(var k=0;k<bs.length;k++)bs[k].onclick=function(){var s=this.getAttribute("data-s");$("ssid").value=s;var all=$("nets").querySelectorAll(".net");for(var q=0;q<all.length;q++)all[q].classList.remove("selnet");this.classList.add("selnet")}
    })
  }
}
function esc(s){return String(s).replace(/[&<>"']/g,function(c){return{"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]})}
$("rescan").onclick=scan;
$("go").onclick=function(){
  var ssid=$("ssid").value.trim();
  if(!ssid){setStatus("Please enter or pick a network name.","bad");return}
  $("go").disabled=true;
  setStatus("Saving and connecting to "+ssid+" ...");
  api("/api/wifi/staConnect","POST",{ssid:ssid,pass:$("pass").value}).then(function(r){
    if(!r||!r.ok){setStatus("Could not save settings ("+((r&&r.error)||"error")+"). Try again.","bad");$("go").disabled=false;return}
    var tries=0;
    (function wait(){
      api("/api/wifi/sta").then(function(j){
        if(j.state===2){setStatus("Connected! IP: "+(j.ip||"?")+". This hotspot will close shortly - reconnect your phone/computer to '"+ssid+"', then open http://"+(j.ip||"relay16.local")+"/","ok")}
        else if(++tries<25){setStatus("Connecting"+new Array(tries%4+1).join(".") );setTimeout(wait,1000)}
        else{setStatus("Could not connect yet. Check the password and try again.","bad");$("go").disabled=false}
      })
    })();
  })
};
scan();
})();
</script></body></html>)HTML";

static const char PG_JS[] PROGMEM = R"JS(
(function(){
"use strict";
function $(s,r){return (r||document).querySelector(s)}
function $$(s,r){return Array.prototype.slice.call((r||document).querySelectorAll(s))}
function el(tag,attrs){
  var n=document.createElement(tag);
  if(attrs)for(var k in attrs){
    if(k==="cls")n.className=attrs[k];
    else if(k==="text")n.textContent=attrs[k];
    else if(k.slice(0,2)==="on")n.addEventListener(k.slice(2),attrs[k]);
    else n.setAttribute(k,attrs[k]);
  }
  for(var i=2;i<arguments.length;i++){var c=arguments[i];if(c==null)continue;n.appendChild(typeof c==="string"?document.createTextNode(c):c)}
  return n;
}
function esc(s){return String(s).replace(/[&<>"']/g,function(c){return{"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]})}
async function api(path,method,body){
  var o={method:method||"GET",headers:{}};
  if(body!==undefined){o.body=JSON.stringify(body);o.headers["Content-Type"]="application/json"}
  var r;
  try{r=await fetch(path,o)}
  catch(e){throw new Error("Network error - device not reachable")}
  var j=null;
  try{j=await r.json()}catch(e){}
  if(!r.ok||(j&&j.error))throw new Error((j&&j.error)||("HTTP "+r.status));
  return j||{};
}
function toast(msg,cls,ms){
  var t=el("div",{cls:"toast "+(cls||""),text:String(msg)});
  document.getElementById("toasts").appendChild(t);
  setTimeout(function(){t.style.opacity="0";t.style.transition="opacity .2s";setTimeout(function(){t.remove()},220)},ms||3400);
}
var confirmRes=null;
function closeConfirm(v){
  var mb=$("#mback");mb.classList.remove("show");
  if(confirmRes){var r=confirmRes;confirmRes=null;r(v)}
}
function confirmBox(title,text){
  return new Promise(function(res){
    $("#mtitle").textContent=title;$("#mtext").textContent=text;
    confirmRes=res;$("#mback").classList.add("show");
  });
}
var DOW=["SUN","MON","TUE","WED","THU","FRI","SAT"];
var MON=["Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec"];
function p2(n){return String(n).padStart(2,"0")}
var Clock={
  offSec:28800,baseUtc:0,baseLoc:0,ok:false,onTime:null,
  sync:function(){var self=this;return api("/api/time").then(function(t){
    self.offSec=t.offSec||0;self.baseUtc=t.epochUtc*1000;self.baseLoc=Date.now();self.ok=true;
    if(self.onTime)self.onTime(t);return t;
  }).catch(function(){Clock.ok=false;return null})},
  utcMs:function(){return this.baseUtc+(Date.now()-this.baseLoc)},
  localMs:function(){return this.utcMs()+this.offSec*1000},
  localDate:function(){return new Date(this.localMs())},
  tick:function(){
    var d=this.localDate(),t=$("#hclock-t");
    if(!t)return;
    t.textContent=p2(d.getUTCHours())+":"+p2(d.getUTCMinutes())+":"+p2(d.getUTCSeconds());
    $("#hclock-d").textContent=DOW[d.getUTCDay()]+" "+p2(d.getUTCMonth()+1)+"/"+p2(d.getUTCDate())+"/"+String(d.getUTCFullYear()).slice(2);
    var t2=$("#dclk-t");if(t2){t2.textContent=p2(d.getUTCHours())+":"+p2(d.getUTCMinutes())+":"+p2(d.getUTCSeconds());$("#dclk-d").textContent=DOW[d.getUTCDay()]+" "+MON[d.getUTCMonth()]+" "+d.getUTCDate()+", "+d.getUTCFullYear()}
  },
  start:function(){
    var self=this;
    this.sync();
    setInterval(function(){self.tick()},250);
    setInterval(function(){self.sync()},20000);
    document.addEventListener("visibilitychange",function(){if(!document.hidden)self.sync()});
    this.tick();
  }
};
var NAV=[["/","Home"],["/relays","Relays"],["/gpio","GPIO"],["/time","Time"],["/wifi-sta","WiFi STA"],["/wifi-ap","WiFi AP"],["/status","Status"]];
function buildShell(){
  var app=$("#app");
  var nav=el("nav",{cls:"menu",id:"menu"});
  var path=location.pathname;
  NAV.forEach(function(n){nav.appendChild(el("a",{href:n[0],cls:path===n[0]?"on":""},n[1]))});
  var hd=el("header",null,
    el("div",{cls:"hrow"},
      el("div",{cls:"brand"},el("span",{cls:"dot"}),el("span",{text:"RM16"})),
      el("div",{cls:"hclock"},
        el("span",{cls:"t",id:"hclock-t"},"--:--:--"),
        el("span",{cls:"d",id:"hclock-d"},"--- --/--/--")),
      el("button",{cls:"burger",id:"burger",onclick:function(){$("#menu").classList.toggle("open")}},"\u2630")),
    nav);
  var main=el("main",{cls:"wrap",id:"page"});
  app.appendChild(hd);app.appendChild(main);
  document.body.appendChild(el("div",{id:"toasts"}));
  document.body.appendChild(el("div",{id:"mback"},
    el("div",{cls:"mbox"},
      el("h3",{id:"mtitle"}),
      el("p",{id:"mtext"}),
      el("div",{cls:"row"},
        el("button",{cls:"btn",onclick:function(){closeConfirm(false)}},"Cancel"),
        el("button",{cls:"btn bad",onclick:function(){closeConfirm(true)}},"Confirm")))));
  return main;
}
function card(title,sub){
  var c=el("div",{cls:"card"});
  if(title)c.appendChild(el("h2",{text:title}));
  if(sub)c.appendChild(el("div",{cls:"sub",text:sub}));
  return c;
}
function kv(k,v){
  return el("div",{cls:"kv"},el("span",{cls:"muted",text:k}),el("b",{text:String(v)}));
}
function segBtns(items,current,clsFor,onPick){
  var seg=el("div",{cls:"seg"});
  items.forEach(function(it){
    var b=el("button",{cls:(it.v===current?(clsFor?clsFor(it.v):"on-a"):""),text:it.t,
      onclick:function(){$$( "button",seg).forEach(function(x){x.className=""});b.className=clsFor?clsFor(it.v):"on-a";onPick(it.v)}});
    seg.appendChild(b);
  });
  return seg;
}
async function pgIndex(m){
  var cClock=card("Device Time","Synchronised from the header clock source");
  cClock.appendChild(el("div",{cls:"bigclock",id:"dclk-t"},"--:--:--"));
  cClock.appendChild(el("div",{cls:"muted",id:"dclk-d"},""));
  var cInfo=el("div",{cls:"row",style:"margin-top:10px"},el("span",{cls:"badge acc",id:"dsrc"},"time source"));
  cClock.appendChild(cInfo);
  var bSync=el("button",{cls:"btn sm",text:"Sync from this browser",onclick:doBrowserSync});
  cClock.appendChild(bSync);
  cClock.appendChild(el("div",{cls:"sub",style:"margin-top:6px",text:"Uses this device's clock (computer/phone) when NTP is unreachable."}));

  var cWifi=card("WiFi");
  var wRows=el("div");cWifi.appendChild(wRows);

  var cRelay=card("Relays","Quick override per relay - full schedules on the Relays page");
  var tiles=el("div",{cls:"tiles"});cRelay.appendChild(tiles);

  m.appendChild(el("div",{cls:"grid g2"},cClock,cWifi));
  m.appendChild(cRelay);

  async function doBrowserSync(){
    try{
      await api("/api/time/browser","POST",{epochMs:Date.now()});
      toast("Device time set from this browser","ok");
      Clock.sync();
    }catch(e){toast(e.message,"bad")}
  }
  async function refresh(){
    try{
      var s=await api("/api/summary");
      wRows.innerHTML="";
      wRows.appendChild(kv("Station",s.wifi.staState===2?("connected: "+s.wifi.ssid):(s.wifi.staState===1?"connecting...":"disconnected")));
      wRows.appendChild(kv("IP",s.wifi.ip||"-"));
      wRows.appendChild(kv("Signal",s.wifi.rssi?String(s.wifi.rssi)+" dBm":"-"));
      wRows.appendChild(kv("Access point",s.wifi.apOn?(s.wifi.apSsid+" ("+s.wifi.clients+" client"+(s.wifi.clients===1?"":"s")+")"):"off"));
      wRows.appendChild(kv("Uptime",fmtDur(s.uptimeS)));
      wRows.appendChild(kv("Free heap",Math.round(s.heap.free/1024)+" kB"));
      var sd=$("#dsrc");if(sd)sd.textContent="Source: "+s.srcName+(s.valid?"":" (no valid time yet)");
      tiles.innerHTML="";
      s.relays.forEach(function(r){
        var tile=el("div",{cls:"tile"});
        tile.appendChild(el("div",{cls:"tn",text:r.n}));
        var st;
        if(r.ovr===1)st=el("span",{cls:"st forced",text:"ON (forced)"});
        else if(r.ovr===2)st=el("span",{cls:"st off",text:"OFF (forced)"});
        else st=el("span",{cls:"st "+(r.on?"on":"off"),text:r.on?"ON":"OFF"});
        var head=el("div",{cls:"spread"});head.appendChild(st);
        head.appendChild(el("span",{cls:"pinb",text:"GPIO"+r.pin}));
        tile.appendChild(head);
        tile.appendChild(el("div",{style:"margin-top:8px"},
          segBtns([{t:"Auto",v:0},{t:"On",v:1},{t:"Off",v:2}],r.ovr,function(v){return v===0?"on-a":v===1?"on-g":"on-r"},async function(v){
            try{await api("/api/gpio/force","POST",{i:r.i,mode:v});refresh()}
            catch(e){toast(e.message,"bad")}
          })));
        tiles.appendChild(tile);
      });
    }catch(e){}
  }
  refresh();
  setInterval(refresh,5000);
}
function fmtDur(s){
  if(!s&&s!==0)return "-";
  s=Math.floor(s);
  var d=Math.floor(s/86400),h=Math.floor(s%86400/3600),mi=Math.floor(s%3600/60);
  return (d?d+"d ":"")+h+"h "+mi+"m";
}

var FIELDS=[
 {k:"sec",lab:"Second",lo:0,hi:59},
 {k:"min",lab:"Minute",lo:0,hi:59},
 {k:"hr",lab:"Hour",lo:0,hi:23},
 {k:"dom",lab:"Day of Month",lo:1,hi:31},
 {k:"mon",lab:"Month",lo:1,hi:12,labels:MON},
 {k:"dow",lab:"Day of Week",lo:0,hi:6,labels:["Sun","Mon","Tue","Wed","Thu","Fri","Sat"]}
];
function parseSpec(s,lo,hi){
  s=String(s==null?"":s).trim();
  if(!s)return null;
  if(s==="*")return "*";
  var parts=s.split(","),st={},n=0;
  for(var i=0;i<parts.length;i++){
    var t=parts[i].trim();
    if(!t)return null;
    var m=/^(\d+)(-(\d+))?$/.exec(t);
    if(!m)return null;
    var a=parseInt(m[1],10),b=m[3]?parseInt(m[3],10):a;
    if(isNaN(a)||isNaN(b)||a<lo||b>hi||a>b)return null;
    for(var v=a;v<=b;v++){if(!st[v]){st[v]=1;n++}}
  }
  return n?st:null;
}
function buildSpec(st,lo,hi){
  if(st==="*")return "*";
  var out=[],run=null;
  for(var v=lo;v<=hi;v++){
    if(st[v]){if(run===null)run=v}
    else if(run!==null){out.push((v-1===run)?String(run):(run+"-"+(v-1)));run=null}
  }
  if(run!==null)out.push(hi===run?String(run):(run+"-"+hi));
  return out.length?out.join(","):"";
}
function chipGroup(field,spec,onChange){
  var st=parseSpec(spec,field.lo,field.hi)||{};
  if(spec==="*")st="*";
  var grp=el("div",{cls:"grp"});
  var val=el("span",{cls:"val"});
  var head=el("button",{cls:"ghead",onclick:toggle},
    el("span",{text:field.lab}),val,el("span",{cls:"arr",text:"\u25B6"}));
  var body=el("div",{cls:"gbody"});
  grp.appendChild(head);grp.appendChild(body);
  function isSel(v){return st==="*"||(st&&st[v])}
  function curSpec(){return st==="*"?"*":buildSpec(st,field.lo,field.hi)}
  grp.getSpec=curSpec;
  function renderVal(){
    var s=curSpec();
    val.textContent=(s==="*"||s==="")?(s===""?"(none)":"every"):((s.length>22)?(s.slice(0,20)+"..."):s);
  }
  function toggle(){
    var open=grp.classList.toggle("open");
    if(open&&!body.hasChildNodes())buildChips();
  }
  function buildChips(){
    var quick=el("div",{cls:"row",style:"padding-top:6px"},
      el("button",{cls:"btn sm",text:"All",onclick:function(){st="*";refresh();onChange(curSpec())}}),
      el("button",{cls:"btn sm",text:"None",onclick:function(){st={};refresh();onChange(curSpec())}}));
    var chips=el("div",{cls:"chips"});
    for(var v=field.lo;v<=field.hi;v++)(function(v){
      var lab=field.labels?field.labels[v]:String(v);
      chips.appendChild(el("button",{cls:"chip"+(field.labels?" wide":"")+(isSel(v)?" sel":""),
        text:lab,onclick:function(){
          if(st==="*"){st={};for(var w=field.lo;w<=field.hi;w++)st[w]=1}
          if(st[v])delete st[v];else st[v]=1;
          refresh();
        }}));
    })(v);
    body.appendChild(quick);body.appendChild(chips);
    function refresh(){var sel=false;for(var v2=field.lo;v2<=field.hi;v2++){var c=chips.children[v2-field.lo];var on=isSel(v2);if(c)c.className="chip"+(field.labels?" wide":"")+(on?" sel":"");if(on)sel=true}renderVal()}
    var _refresh=refresh;
    grp._refreshSel=function(){_refresh()};
    renderVal();
  }
  renderVal();
  return grp;
}
function patternEditor(patternObj){
  var box=el("div");
  var groups={};
  FIELDS.forEach(function(f){
    var spec=(patternObj&&patternObj[f.k]!=null)?String(patternObj[f.k]):"*";
    groups[f.k]=chipGroup(f,spec,function(){});
  });
  FIELDS.forEach(function(f){box.appendChild(groups[f.k])});
  box.getSpecs=function(){
    var o={};
    FIELDS.forEach(function(f){o[f.k]=groups[f.k].getSpec()});
    return o;
  };
  return box;
}
function pairSpecValid(d){
  if(!d||!d.st||!d.sp)return false;
  for(var i=0;i<FIELDS.length;i++){if(!d.st[FIELDS[i].k]||!d.sp[FIELDS[i].k])return false}
  return true;
}
function pairEditor(idx,pair,onSave){
  var wrap=el("div",{cls:"pair"});
  var en=!!(pair&&pair.en);
  var stObj=(pair&&pair.st)||null, spObj=(pair&&pair.sp)||null;
  var edStart=patternEditor(stObj), edStop=patternEditor(spObj);
  var cb=el("input",{type:"checkbox"});cb.checked=en;
  var head=el("div",{cls:"phead"},
    cb,
    el("span",{text:"Pair "+(idx+1)}),
    el("span",{cls:"muted",text:"schedule"}));
  cb.addEventListener("change",function(){wrap.classList.toggle("dim",!cb.checked)});
  var cols=el("div",{cls:"pcol"},
    el("div",null,el("div",{cls:"flab",text:"START (relay ON at this time)"}),edStart),
    el("div",null,el("div",{cls:"flab",text:"STOP (relay OFF at this time)"}),edStop));
  var save=el("button",{cls:"btn sm pri",text:"Save pair"});
  var clear=el("button",{cls:"btn sm",text:"Disable pair"});
  var foot=el("div",{cls:"row",style:"margin-top:10px"},save,clear);
  wrap.appendChild(head);wrap.appendChild(cols);wrap.appendChild(foot);
  wrap.classList.toggle("dim",!en);
  function push(enFlag){
    var d=save._get();
    d.en=enFlag;
    if(!pairSpecValid(d)){toast("Each field needs at least one value (or All)","bad");return}
    onSave(d);
  }
  save.addEventListener("click",function(){push(cb.checked?1:0)});
  clear.addEventListener("click",function(){push(0)});
  save._get=function(){
    return {en:cb.checked?1:0,st:edStart.getSpecs(),sp:edStop.getSpecs()};
  };
  wrap._cb=cb;
  return wrap;
}

function relayCard(r){
  var cardEl=el("div",{cls:"rcard"});
  var led=el("span",{cls:"led"+(r.on?" on":"")});
  var name=el("span",{cls:"rname",text:r.n,title:"Double-click to rename"});
  var editB=el("button",{cls:"editic",title:"Rename relay",text:"\u270E",onclick:function(){beginRename()}});
  var pin=el("span",{cls:"pinb",text:"GPIO"+r.pin});
  function beginRename(){
    var inp=el("input",{type:"text",value:r.n,maxlength:"23",style:"max-width:180px;padding:6px 8px"});
    name.replaceWith(inp);
    inp.focus();inp.select();
    var done=false;
    async function commit(){
      if(done)return;done=true;
      var v=inp.value.trim();
      if(v&&v!==r.n){
        try{await api("/api/relay/name","POST",{i:r.i,name:v});r.n=v;toast("Renamed to "+v,"ok")}
        catch(e){toast(e.message,"bad")}
      }
      inp.replaceWith(name);
      name.textContent=r.n;
    }
    inp.addEventListener("keydown",function(e){if(e.key==="Enter")commit();if(e.key==="Escape"){done=true;inp.replaceWith(name)}});
    inp.addEventListener("blur",commit);
  }
  name.addEventListener("dblclick",beginRename);
  var ovr=r.ovr||0;
  var seg=segBtns([{t:"Auto",v:0},{t:"On",v:1},{t:"Off",v:2}],ovr,function(v){return v===0?"on-a":v===1?"on-g":"on-r"},async function(v){
    try{await api("/api/gpio/force","POST",{i:r.i,mode:v});refreshOne()}
    catch(e){toast(e.message,"bad")}
  });
  var schedCb=el("input",{type:"checkbox"});
  schedCb.checked=!!r.se;
  schedCb.addEventListener("change",async function(){
    try{await api("/api/relay/schedEnable","POST",{i:r.i,on:schedCb.checked?1:0});refreshOne()}
    catch(e){toast(e.message,"bad")}
  });
  var head=el("div",{cls:"rhead"},
    led,name,editB,pin,seg,
    el("label",{cls:"muted",style:"display:flex;align-items:center;gap:6px"},schedCb,"schedules"),
    el("button",{cls:"exp",text:"\u25BE",onclick:function(){cardEl.classList.toggle("open");loadBody()}}));
  var body=el("div",{cls:"rbody"});
  cardEl.appendChild(head);cardEl.appendChild(body);
  var bodyLoaded=false;
  function loadBody(){
    if(bodyLoaded||!cardEl.classList.contains("open"))return;
    bodyLoaded=true;
    api("/api/relay?i="+r.i).then(function(d){
      body.innerHTML="";
      var top=el("div",{cls:"row"},
        el("button",{cls:"btn sm bad",text:"Clear all pairs",onclick:async function(){
          if(!await confirmBox("Clear all schedules","Delete all 8 schedule pairs of "+r.n+"? The relay will have no schedule until you create one."))return;
          try{await api("/api/relay/pairsClear","POST",{i:r.i});toast("All pairs cleared","ok");if(window.__refreshRelays)window.__refreshRelays()}
          catch(e){toast(e.message,"bad")}
        }}),
        el("span",{cls:"muted",text:d.se?"Tip: double-click the name to rename. Patterns are LOCAL time.":"Schedules are disabled for this relay."}));
      body.appendChild(top);
      var pairs=el("div",{cls:"pairs"});
      (d.pairs||[]).forEach(function(p,ix){
        var pe=pairEditor(ix,p,function(pair){
          api("/api/relay/pair","POST",{i:r.i,p:ix,pair:pair}).then(function(){
            toast(pair.en?("Pair "+(ix+1)+" saved"):("Pair "+(ix+1)+" disabled"),"ok");
            pe._cb.checked=!!pair.en;
            pe.classList.toggle("dim",!pair.en);
          }).catch(function(e){toast(e.message,"bad")});
        });
        pairs.appendChild(pe);
      });
      body.appendChild(pairs);
      body.appendChild(el("div",{cls:"muted",style:"margin-top:8px",text:"Priority when several pairs match the same second: higher pair number wins; STOP beats START. Overnight windows: set START later than STOP."}));
    }).catch(function(e){toast(e.message,"bad");bodyLoaded=false});
  }
  function refreshOne(){if(window.__refreshRelays)window.__refreshRelays()}
  cardEl._refresh=function(d){
    led.className="led"+(d.on?" on":"");
    name.textContent=d.n;r.n=d.n;pin.textContent="GPIO"+d.pin;
    schedCb.checked=!!d.se;
    var btns=$$("button",seg);
    btns.forEach(function(b){b.className=""});
    if(btns[d.ovr||0])btns[d.ovr||0].className=(d.ovr===1?"on-g":d.ovr===2?"on-r":"on-a");
  };
  return cardEl;
}
async function pgRelays(m){
  var wrap=el("div",{cls:"rlist"});
  m.appendChild(el("div",{cls:"card"},
    el("h2",{text:"Relay Schedules"}),
    el("div",{cls:"sub",text:"16 relays x 8 start/stop pairs. Calendar masks: seconds, minutes, hours, day-of-month, month, day-of-week. Times are local wall-clock."})));
  m.appendChild(wrap);
  var cards={};
  window.__refreshRelays=async function loadAll(){
    try{
      var s=await api("/api/relays");
      (s.relays||[]).forEach(function(r){
        if(cards[r.i])cards[r.i]._refresh(r);
        else{cards[r.i]=relayCard(r);wrap.appendChild(cards[r.i])}
      });
    }catch(e){}
  };
  await window.__refreshRelays();
  setInterval(window.__refreshRelays,5000);
}

async function pgGpio(m){
  var cG=card("Global Relay Polarity","Applies to every relay that is set to 'inherit'");
  var gSeg=el("div",{cls:"row"});
  cG.appendChild(gSeg);
  m.appendChild(cG);
  var cR=card("Per-Relay Control","Force ON / OFF (overrides schedules), set polarity, or Remove override to return to schedule");
  var list=el("div");
  cR.appendChild(list);
  m.appendChild(cR);
  var gState={};
  async function refresh(){
    try{
      var j=await api("/api/gpio");
      if(formFocused())return;
      gState.global=j.globalActiveHigh;
      gSeg.innerHTML="";
      gSeg.appendChild(el("span",{cls:"muted",style:"margin-right:6px",text:"Common relays are"}));
      gSeg.appendChild(segBtns([{t:"Active HIGH",v:1},{t:"Active LOW",v:0}],j.globalActiveHigh,null,async function(v){
        try{await api("/api/gpio/global","POST",{activeHigh:v});toast("Global polarity saved","ok");refresh()}
        catch(e){toast(e.message,"bad")}
      }));
      list.innerHTML="";
      (j.relays||[]).forEach(function(r){
        var row=el("div",{cls:"pair"});
        var sel=el("select");
        [["Inherit","0"],["Active HIGH","1"],["Active LOW","2"]].forEach(function(o){
          var op=el("option",{value:o[1],text:o[0]});if((r.pol|0)===parseInt(o[1],10))op.selected=true;sel.appendChild(op);
        });
        sel.addEventListener("change",async function(){
          try{await api("/api/gpio/polarity","POST",{i:r.i,pol:parseInt(sel.value,10)});toast(r.n+" polarity saved","ok");refresh()}
          catch(e){toast(e.message,"bad")}
        });
        var stTxt=r.ovr===1?"FORCED ON":r.ovr===2?"FORCED OFF":(r.on?"ON (schedule)":"OFF (schedule)");
        row.appendChild(el("div",{cls:"rhead"},
          el("span",{cls:"led"+(r.on?" on":"")}),
          el("span",{text:r.n,style:"font-weight:700"}),
          el("span",{cls:"pinb",text:"GPIO"+r.pin}),
          el("span",{cls:"st "+(r.ovr?"forced":(r.on?"on":"off")),text:stTxt}),
          el("span",{cls:"muted",text:"effective "+(r.eff?"active-high":"active-low")}),
          el("div",{style:"margin-left:auto;display:flex;gap:6px;align-items:center"},sel,
            el("button",{cls:"btn sm ok",text:"Set High",onclick:async function(){
              try{await api("/api/gpio/force","POST",{i:r.i,mode:1});refresh()}catch(e){toast(e.message,"bad")}}}),
            el("button",{cls:"btn sm bad",text:"Set Low",onclick:async function(){
              try{await api("/api/gpio/force","POST",{i:r.i,mode:2});refresh()}catch(e){toast(e.message,"bad")}}}),
            el("button",{cls:"btn sm",text:"Remove",onclick:async function(){
              try{await api("/api/gpio/remove","POST",{i:r.i});refresh()}catch(e){toast(e.message,"bad")}}})
          )));
        list.appendChild(row);
      });
    }catch(e){}
  }
  refresh();
  setInterval(refresh,5000);
}
async function pgTime(m){
  var cDisp=card("Current Time");
  cDisp.appendChild(el("div",{cls:"bigclock",id:"dclk-t"},"--:--:--"));
  cDisp.appendChild(el("div",{cls:"muted",id:"dclk-d"},""));
  var info=el("div",{cls:"row",style:"margin-top:10px"});
  cDisp.appendChild(info);
  var syncSlot=el("div",{style:"margin-top:10px"});
  cDisp.appendChild(syncSlot);
  m.appendChild(cDisp);

  var cTz=card("Timezone & DST");
  var tzMode=el("select");
  tzMode.appendChild(el("option",{value:"1",text:"Timezone preset (with automatic DST)"}));
  tzMode.appendChild(el("option",{value:"0",text:"Manual GMT offset + custom DST"}));
  var tzSel=el("select");
  var offH=el("input",{type:"number",value:"8",min:"-14",max:"14",step:"1"});
  var offM=el("input",{type:"number",value:"0",min:"-59",max:"59",step:"1"});
  var offS=el("input",{type:"number",value:"0",min:"-59",max:"59",step:"1"});
  var dstMode=el("select");
  dstMode.appendChild(el("option",{value:"0",text:"No DST"}));
  dstMode.appendChild(el("option",{value:"1",text:"Custom DST rules"}));
  var shiftMin=el("input",{type:"number",value:"60",min:"1",max:"300",step:"1"});
  var manualBox=el("div");
  var presetBox=el("div");
  function tzFields(t){
    tzSel.innerHTML="";
    (t.tz||[]).forEach(function(z){
      var op=el("option",{value:String(z.i),text:"(UTC"+z.o+") "+z.n});
      if(z.i===t.tzPreset)op.selected=true;
      tzSel.appendChild(op);
    });
    tzMode.value=String(t.tzMode||0);
    var g=t.gmtOffS||0,sign=g<0?-1:1,a=Math.abs(g);
    offH.value=String(sign*Math.floor(a/3600));
    offM.value=String(sign*Math.floor(a%3600/60));
    offS.value=String(sign*(a%60));
    dstMode.value=String(t.dstMode||0);
    shiftMin.value=String(t.dstShiftMin||60);
    applyMode();
  }
  function applyMode(){
    var preset=tzMode.value==="1";
    presetBox.style.display=preset?"block":"none";
    manualBox.style.display=preset?"none":"block";
  }
  tzMode.addEventListener("change",applyMode);
  cTz.appendChild(el("label",{cls:"f",text:"Mode"}));
  cTz.appendChild(tzMode);
  presetBox.appendChild(el("label",{cls:"f",text:"Timezone"}));
  presetBox.appendChild(tzSel);
  manualBox.appendChild(el("label",{cls:"f",text:"GMT offset (hours : minutes : seconds, negative allowed)"}));
  manualBox.appendChild(el("div",{cls:"inline"},offH,offM,offS));
  manualBox.appendChild(el("label",{cls:"f",text:"Daylight saving (manual mode)"}));
  manualBox.appendChild(dstMode);
  manualBox.appendChild(el("div",{id:"dstRules"}));
  manualBox.appendChild(el("label",{cls:"f",text:"DST shift minutes (e.g. 60, Lord Howe 30)"}));
  manualBox.appendChild(shiftMin);
  cTz.appendChild(presetBox);cTz.appendChild(manualBox);
  var bTz=el("button",{cls:"btn pri",text:"Save timezone",onclick:saveTz});
  cTz.appendChild(el("div",{style:"margin-top:12px"},bTz));
  m.appendChild(cTz);

  function ruleEditor(rule,label){
    var box=el("div");
    var mon=el("select"),week=el("select"),dow=el("select"),hh=el("input",{type:"number",min:"0",max:"23",step:"1"}),mm=el("input",{type:"number",min:"0",max:"59",step:"1"});
    for(var i=1;i<=12;i++)mon.appendChild(el("option",{value:String(i),text:MON[i-1]}));
    [["1","1st"],["2","2nd"],["3","3rd"],["4","4th"],["5","last"]].forEach(function(w){week.appendChild(el("option",{value:w[0],text:"week "+w[1]}))});
    for(var d=0;d<7;d++)dow.appendChild(el("option",{value:String(d),text:["Sun","Mon","Tue","Wed","Thu","Fri","Sat"][d]}));
    if(rule){mon.value=String(rule.mon);week.value=String(rule.week);dow.value=String(rule.dow);hh.value=String(rule.hour);mm.value=String(rule.min)}
    else{mon.value="3";week.value="2";dow.value="0";hh.value="2";mm.value="0"}
    box.appendChild(el("div",{cls:"flab",text:label}));
    box.appendChild(el("div",{cls:"inline"},mon,week,dow,hh,mm));
    box._get=function(){return{mon:parseInt(mon.value,10),week:parseInt(week.value,10),dow:parseInt(dow.value,10),hour:parseInt(hh.value,10)||0,min:parseInt(mm.value,10)||0}};
    box._set=function(r){if(!r)return;mon.value=String(r.mon);week.value=String(r.week);dow.value=String(r.dow);hh.value=String(r.hour);mm.value=String(r.min)};
    return box;
  }
  var rStart=ruleEditor(null,"DST STARTS (month, week, weekday, hour, minute - local standard time)");
  var rEnd=ruleEditor(null,"DST ENDS (month, week, weekday, hour, minute - local DST time)");
  $("#dstRules").appendChild(rStart);
  $("#dstRules").appendChild(rEnd);

  async function saveTz(){
    var body={tzMode:parseInt(tzMode.value,10)};
    if(body.tzMode===1){body.tzPreset=parseInt(tzSel.value,10)}
    else{
      var oh=parseInt(offH.value,10)||0,om=parseInt(offM.value,10)||0,os=parseInt(offS.value,10)||0;
      var g=oh*3600+om*60+os;
      if(g<-50400||g>50400){toast("GMT offset out of range","bad");return}
      body.gmtOffS=g;
      body.dstMode=parseInt(dstMode.value,10);
      body.dstShiftMin=parseInt(shiftMin.value,10)||60;
      if(body.dstMode===1){body.ds=rStart._get();body.de=rEnd._get()}
    }
    try{await api("/api/time/config","POST",body);toast("Timezone saved - schedules re-evaluated","ok");pollTime()}
    catch(e){toast(e.message,"bad")}
  }

  var cDrift=card("Clock Drift Compensation","Correct a fast/slow board: -30..+30 seconds per day, applied in 1-second steps between syncs");
  var drift=el("input",{type:"number",min:"-30",max:"30",step:"1",value:"0"});
  cDrift.appendChild(el("label",{cls:"f",text:"Seconds per day (negative = clock runs slow)"}));
  cDrift.appendChild(drift);
  cDrift.appendChild(el("div",{style:"margin-top:10px"},el("button",{cls:"btn pri",text:"Save drift",onclick:async function(){
    var v=parseInt(drift.value,10)||0;
    if(v<-30||v>30){toast("Drift must be -30..30","bad");return}
    try{await api("/api/time/config","POST",{driftPerDay:v});toast("Drift compensation saved","ok");pollTime()}
    catch(e){toast(e.message,"bad")}
  }})));
  m.appendChild(cDrift);

  var cNtp=card("NTP Time Servers","Servers are tried in rotation with automatic fallback");
  var n1=el("input",{type:"text",maxlength:"48",placeholder:"pool.ntp.org"});
  var n2=el("input",{type:"text",maxlength:"48",placeholder:"time.google.com"});
  var n3=el("input",{type:"text",maxlength:"48",placeholder:"time.cloudflare.com"});
  var nTo=el("input",{type:"number",min:"1",max:"60",step:"1",value:"5"});
  var nIn=el("input",{type:"number",min:"60",max:"86400",step:"1",value:"3600"});
  cNtp.appendChild(el("label",{cls:"f",text:"Server 1"}));cNtp.appendChild(n1);
  cNtp.appendChild(el("label",{cls:"f",text:"Server 2"}));cNtp.appendChild(n2);
  cNtp.appendChild(el("label",{cls:"f",text:"Server 3"}));cNtp.appendChild(n3);
  cNtp.appendChild(el("label",{cls:"f",text:"Per-server timeout (seconds) / Sync interval (seconds, up to 86400 = 24h)"}));
  cNtp.appendChild(el("div",{cls:"inline"},nTo,nIn));
  var nStatus=el("div",{cls:"muted",style:"margin-top:10px"});
  cNtp.appendChild(nStatus);
  cNtp.appendChild(el("div",{style:"margin-top:10px",cls:"row"},
    el("button",{cls:"btn pri",text:"Save NTP settings",onclick:async function(){
      try{
        await api("/api/time/config","POST",{ntp1:n1.value.trim(),ntp2:n2.value.trim(),ntp3:n3.value.trim(),
          ntpTimeoutS:parseInt(nTo.value,10)||5,ntpIntervalS:parseInt(nIn.value,10)||3600});
        toast("NTP settings saved","ok");pollTime();
      }catch(e){toast(e.message,"bad")}
    }}),
    el("button",{cls:"btn",text:"Sync now",onclick:async function(){
      try{await api("/api/time/ntpSync","POST");toast("NTP sync started","ok");setTimeout(pollTime,1500)}
      catch(e){toast(e.message,"bad")}
    }})));
  m.appendChild(cNtp);

  var cRtc=card("DS3231 Real-Time Clock","Backup clock on GPIO21 (SDA) / GPIO22 (SCL)");
  var rtcInfo=el("div");
  var rtcEn=el("input",{type:"checkbox"});
  var rtcRs=el("input",{type:"number",min:"1",max:"1440",step:"1",value:"60"});
  cRtc.appendChild(rtcInfo);
  cRtc.appendChild(el("label",{cls:"f",text:"Resync device time to the RTC chip every N minutes"}));
  cRtc.appendChild(el("div",{cls:"inline"},rtcRs,el("label",{cls:"muted",style:"display:flex;align-items:center;gap:6px;margin-top:8px"},rtcEn,"RTC enabled")));
  cRtc.appendChild(el("div",{style:"margin-top:10px",cls:"row"},
    el("button",{cls:"btn",text:"Read RTC into device",onclick:async function(){
      try{var r=await api("/api/time/rtcRead","POST");toast(r.ok?("Device time set from RTC: "+r.epochUtc):"RTC read failed",r.ok?"ok":"bad");pollTime()}
      catch(e){toast(e.message,"bad")}
    }}),
    el("button",{cls:"btn",text:"Write device time to RTC",onclick:async function(){
      try{var r=await api("/api/time/rtcWrite","POST");toast(r.ok?"RTC chip updated":"RTC write failed",r.ok?"ok":"bad")}
      catch(e){toast(e.message,"bad")}
    }}),
    el("button",{cls:"btn pri",text:"Save RTC settings",onclick:async function(){
      try{
        await api("/api/time/config","POST",{rtcEnabled:rtcEn.checked?1:0,rtcResyncMin:parseInt(rtcRs.value,10)||60});
        toast("RTC settings saved","ok");pollTime();
      }
      catch(e){toast(e.message,"bad")}
    }})));
  m.appendChild(cRtc);

  async function pollTime(){
    try{
      var t=await api("/api/time");
      Clock.offSec=t.offSec||0;Clock.baseUtc=t.epochUtc*1000;Clock.baseLoc=Date.now();
      info.innerHTML="";
      info.appendChild(el("span",{cls:"badge "+(t.valid?"ok":"bad"),text:t.valid?"time valid":"no valid time source"}));
      info.appendChild(el("span",{cls:"badge acc",text:"source: "+t.srcName}));
      info.appendChild(el("span",{cls:"badge",text:"UTC"+(t.offSec<0?"-":"+")+Math.trunc(Math.abs(t.offSec)/3600)+":"+p2(Math.floor(Math.abs(t.offSec)%3600/60))}));
      if(t.dstActive)info.appendChild(el("span",{cls:"badge",text:"DST active (+"+t.dstMin+" min)"}));
      if(t.nextChangeUtc)info.appendChild(el("span",{cls:"badge",text:(t.nextChangeDir>0?"DST starts ":"DST ends ")+new Date((t.nextChangeUtc+t.offSec)*1000).toISOString().slice(0,10)}));
      if(!formFocused()){
        tzFields(t);
        drift.value=String(t.driftPerDay||0);
        n1.value=(t.ntp&&t.ntp.s1)||n1.value;n2.value=(t.ntp&&t.ntp.s2)||n2.value;n3.value=(t.ntp&&t.ntp.s3)||n3.value;
        if(t.ntp){nTo.value=String(t.ntp.timeout||5);nIn.value=String(t.ntp.interval||3600)}
        if(t.ds)rStart._set(t.ds);
        if(t.de)rEnd._set(t.de);
        rtcEn.checked=!!(t.rtc&&t.rtc.enabled);
        rtcRs.value=String((t.rtc&&t.rtc.resyncMin)||60);
      }
      var st=t.ntp?String(t.ntp.state):"";
      nStatus.innerHTML="";
      nStatus.appendChild(kv("Client state",st==="0"?"idle":st==="1"?"resolving server":"waiting for reply"));
      if(t.ntp){
        nStatus.appendChild(kv("Last good sync",t.ntp.lastOkUtc?(t.ntp.lastOkSrv+" ("+new Date(t.ntp.lastOkUtc*1000).toISOString().slice(0,19).replace("T"," ")+")"):"never"));
        nStatus.appendChild(kv("Failed attempts",t.ntp.fails));
        nStatus.appendChild(kv("Next attempt",t.ntp.nextIn?"in "+t.ntp.nextIn+" s":"pending"));
      }
      rtcInfo.innerHTML="";
      rtcInfo.appendChild(kv("Chip present",t.rtc&&t.rtc.present?"yes":"no"));
      if(t.rtc&&t.rtc.present){
        rtcInfo.appendChild(kv("Battery power lost",t.rtc.lostPower?"YES - replace battery":"no"));
        rtcInfo.appendChild(kv("Temperature",t.rtc.temp+" C"));
        rtcInfo.appendChild(kv("RTC chip time (UTC)",t.rtc.epochUtc||"-"));
      }
      syncSlot.innerHTML="";
      if(!t.valid){
        syncSlot.appendChild(el("button",{cls:"btn sm pri",text:"Sync from this browser",onclick:async function(){
          try{await api("/api/time/browser","POST",{epochMs:Date.now()});toast("Device time set from browser","ok");pollTime()}
          catch(e){toast(e.message,"bad")}
        }}));
      }
    }catch(e){}
  }
  pollTime();
  setInterval(pollTime,5000);
}
function formFocused(){
  var a=document.activeElement;
  return !!a&&(a.tagName==="INPUT"||a.tagName==="SELECT"||a.tagName==="TEXTAREA");
}

async function pgSta(m){
  var cSt=card("Station Status");
  var stBox=el("div");cSt.appendChild(stBox);
  m.appendChild(cSt);

  var cCfg=card("Station Settings","Static IP is applied at connect; switching back to DHCP applies after a reboot");
  var en=el("input",{type:"checkbox"});
  var ssid=el("input",{type:"text",maxlength:"32",autocomplete:"off"});
  var pass=el("input",{type:"password",maxlength:"64",autocomplete:"off"});
  var host=el("input",{type:"text",maxlength:"32",value:"relay16"});
  var stat=el("select");
  stat.appendChild(el("option",{value:"0",text:"DHCP (automatic)"}));
  stat.appendChild(el("option",{value:"1",text:"Static IP"}));
  var ip=el("input",{type:"text",placeholder:"192.168.1.50"});
  var gw=el("input",{type:"text",placeholder:"192.168.1.1"});
  var mask=el("input",{type:"text",placeholder:"255.255.255.0"});
  var dns1=el("input",{type:"text",placeholder:"8.8.8.8"});
  var dns2=el("input",{type:"text",placeholder:"8.8.4.4"});
  cCfg.appendChild(el("div",{cls:"row"},el("label",{cls:"muted",style:"display:flex;align-items:center;gap:8px"},en,"Enable station")));
  cCfg.appendChild(el("label",{cls:"f",text:"Network name (SSID)"}));cCfg.appendChild(ssid);
  cCfg.appendChild(el("label",{cls:"f",text:"Password (empty for open networks)"}));cCfg.appendChild(pass);
  cCfg.appendChild(el("label",{cls:"f",text:"Hostname (mDNS: hostname.local)"}));cCfg.appendChild(host);
  cCfg.appendChild(el("label",{cls:"f",text:"IP mode"}));cCfg.appendChild(stat);
  var ipBox=el("div");
  ipBox.appendChild(el("label",{cls:"f",text:"Static IP / Gateway / Netmask / DNS 1 / DNS 2"}));
  ipBox.appendChild(el("div",{cls:"inline"},ip,gw,mask,dns1,dns2));
  cCfg.appendChild(ipBox);
  stat.addEventListener("change",function(){ipBox.style.display=stat.value==="1"?"block":"none"});
  cCfg.appendChild(el("div",{style:"margin-top:14px",cls:"row"},
    el("button",{cls:"btn pri",text:"Save & connect",onclick:save}),
    el("button",{cls:"btn bad",text:"Disconnect",onclick:async function(){
      if(!await confirmBox("Disconnect WiFi","Stop trying to connect and pause auto-reconnect until the next save or reboot?"))return;
      try{await api("/api/wifi/staDisconnect","POST");toast("Disconnected","ok");poll()}
      catch(e){toast(e.message,"bad")}
    }})));
  m.appendChild(cCfg);

  var cScan=card("Scan","Tap a network to copy its name into the settings");
  var scanBox=el("div");
  cScan.appendChild(el("div",{style:"margin-bottom:10px"},el("button",{cls:"btn",text:"Scan networks",onclick:doScan})));
  cScan.appendChild(scanBox);
  m.appendChild(cScan);

  function doScan(){
    scanBox.innerHTML="";
    scanBox.appendChild(el("div",{cls:"muted",text:"Scanning..."}));
    api("/api/wifi/scanStart","POST").then(function(){
      (function pollScan(){
        api("/api/wifi/scan").then(function(j){
          if(j.scanning){setTimeout(pollScan,1200);return}
          scanBox.innerHTML="";
          if(!j.nets||!j.nets.length){scanBox.appendChild(el("div",{cls:"muted",text:"No networks found."}));return}
          j.nets.forEach(function(n){
            scanBox.appendChild(el("button",{cls:"net",onclick:function(){ssid.value=n.s||""}},
              el("span",{cls:"sig",text:n.r+" dBm"}),
              el("span",{text:n.s||"(hidden)"}),
              el("span",{cls:"lk",style:n.sec?"":"color:#2ecc71",text:n.sec?"secured":"open"})));
          });
        }).catch(function(){scanBox.innerHTML=""});
      })();
    }).catch(function(e){toast(e.message,"bad")});
  }

  async function save(){
    var body={staEnabled:en.checked?1:0,ssid:ssid.value.trim(),pass:pass.value,hostname:host.value.trim()||"relay16",static:parseInt(stat.value,10)};
    if(body.static===1){body.ip=ip.value.trim();body.gw=gw.value.trim();body.mask=mask.value.trim();body.dns1=dns1.value.trim();body.dns2=dns2.value.trim()}
    try{await api("/api/wifi/sta","POST",body);toast("Saved - connecting...","ok");poll()}
    catch(e){toast(e.message,"bad")}
  }

  async function poll(){
    try{
      var j=await api("/api/wifi/sta");
      stBox.innerHTML="";
      stBox.appendChild(kv("State",j.state===2?"connected":j.state===1?"connecting":"disconnected"));
      stBox.appendChild(kv("SSID",j.ssid||"-"));
      stBox.appendChild(kv("IP",j.ip||"-"));
      stBox.appendChild(kv("Signal",j.rssi?j.rssi+" dBm":"-"));
      stBox.appendChild(kv("MAC",j.mac||"-"));
      if(!formFocused()){
        en.checked=!!j.enabled;
        ssid.value=j.ssidCfg||"";
        host.value=j.hostname||"relay16";
        stat.value=String(j.static?1:0);
        ipBox.style.display=j.static?"block":"none";
        ip.value=j.ipCfg||"";gw.value=j.gwCfg||"";mask.value=j.maskCfg||"";
        dns1.value=j.dns1Cfg||"";dns2.value=j.dns2Cfg||"";
      }
    }catch(e){}
  }
  poll();
  setInterval(poll,3000);
}
async function pgAp(m){
  var cSt=card("Access Point Status");
  var stBox=el("div");cSt.appendChild(stBox);
  m.appendChild(cSt);
  var cCfg=card("Access Point Settings");
  var policy=el("select");
  policy.appendChild(el("option",{value:"0",text:"Automatic - run AP whenever station is offline (recommended)"}));
  policy.appendChild(el("option",{value:"1",text:"Always on"}));
  policy.appendChild(el("option",{value:"2",text:"Always off (careful: can lock you out)"}));
  var ssid=el("input",{type:"text",maxlength:"32"});
  var pass=el("input",{type:"password",maxlength:"64"});
  var open=el("input",{type:"checkbox"});
  var ch=el("input",{type:"number",min:"1",max:"13",step:"1",value:"1"});
  var hidden=el("input",{type:"checkbox"});
  var maxc=el("input",{type:"number",min:"1",max:"8",step:"1",value:"4"});
  var ip=el("input",{type:"text"});
  var gw=el("input",{type:"text"});
  var mask=el("input",{type:"text"});
  cCfg.appendChild(el("label",{cls:"f",text:"Policy"}));cCfg.appendChild(policy);
  cCfg.appendChild(el("label",{cls:"f",text:"Network name (empty = automatic RM16-XXXXXX)"}));cCfg.appendChild(ssid);
  cCfg.appendChild(el("label",{cls:"f",text:"Password (8..64 characters; ignored for open networks)"}));cCfg.appendChild(pass);
  cCfg.appendChild(el("div",{cls:"row",style:"margin-top:8px"},el("label",{cls:"muted",style:"display:flex;align-items:center;gap:8px"},open,"Open network (no password)")));
  cCfg.appendChild(el("label",{cls:"f",text:"Channel / Max clients"}));
  cCfg.appendChild(el("div",{cls:"inline"},ch,maxc));
  cCfg.appendChild(el("div",{cls:"row",style:"margin-top:8px"},el("label",{cls:"muted",style:"display:flex;align-items:center;gap:8px"},hidden,"Hidden SSID")));
  cCfg.appendChild(el("label",{cls:"f",text:"AP IP / Gateway / Netmask"}));
  cCfg.appendChild(el("div",{cls:"inline"},ip,gw,mask));
  cCfg.appendChild(el("div",{style:"margin-top:14px"},el("button",{cls:"btn pri",text:"Apply settings",onclick:save})));
  m.appendChild(cCfg);
  async function save(){
    var body={policy:parseInt(policy.value,10),ssid:ssid.value.trim(),pass:pass.value,
      open:open.checked?1:0,channel:parseInt(ch.value,10)||1,hidden:hidden.checked?1:0,
      maxConn:parseInt(maxc.value,10)||4,ip:ip.value.trim(),gw:gw.value.trim(),mask:mask.value.trim()};
    try{await api("/api/wifi/ap","POST",body);toast("AP settings applied","ok");poll()}
    catch(e){toast(e.message,"bad")}
  }
  async function poll(){
    try{
      var j=await api("/api/wifi/ap");
      stBox.innerHTML="";
      stBox.appendChild(kv("State",j.on?"running":"stopped"));
      if(j.on){stBox.appendChild(kv("SSID",j.ssid));stBox.appendChild(kv("IP",j.ip));stBox.appendChild(kv("Clients",j.clients))}
      if(!formFocused()){
        policy.value=String(j.policy);
        ssid.value=j.ssidCfg||"";
        pass.value=j.passCfg||"";
        open.checked=!!j.open;
        ch.value=String(j.channel||1);
        hidden.checked=!!j.hidden;
        maxc.value=String(j.maxConn||4);
        ip.value=j.ipCfg||"";gw.value=j.gwCfg||"";mask.value=j.maskCfg||"";
      }
    }catch(e){}
  }
  poll();
  setInterval(poll,3000);
}

async function pgStatus(m){
  var cSys=card("System");
  var sysBox=el("div");cSys.appendChild(sysBox);
  var cWifi=card("WiFi");
  var wifiBox=el("div");cWifi.appendChild(wifiBox);
  var cTime=card("Time");
  var timeBox=el("div");cTime.appendChild(timeBox);
  var cRelay=card("Relays");
  var tiles=el("div",{cls:"tiles"});cRelay.appendChild(tiles);
  m.appendChild(el("div",{cls:"grid g2"},cSys,cWifi));
  m.appendChild(cTime);
  m.appendChild(cRelay);

  var cMaint=card("Maintenance");
  var fileIn=el("input",{type:"file",accept:".json,application/json",style:"display:none"});
  cMaint.appendChild(el("div",{cls:"row"},
    el("button",{cls:"btn",text:"Recalculate schedules",onclick:async function(){
      try{await api("/api/sched/recalc","POST");toast("Schedules recalculated","ok")}
      catch(e){toast(e.message,"bad")}
    }}),
    el("button",{cls:"btn bad",text:"Reboot device",onclick:async function(){
      if(!await confirmBox("Reboot","The web panel will be unreachable for a few seconds. Continue?"))return;
      try{toast("Rebooting...","ok");await api("/api/reboot","POST")}catch(e){}
    }}),
    el("button",{cls:"btn bad",text:"Factory reset",onclick:async function(){
      if(!await confirmBox("Factory reset","ALL settings and schedules are erased (WiFi, time, relays) and the device reboots. This also happens when GPIO0 is held for 5 seconds. Continue?"))return;
      try{toast("Resetting...","ok");await api("/api/factory","POST")}catch(e){}
    }})));
  cMaint.appendChild(el("div",{style:"margin-top:12px",cls:"row"},
    el("a",{cls:"btn",href:"/api/config/export",download:"rm16-config.json",text:"Export config"})));
  cMaint.appendChild(el("div",{style:"margin-top:12px"},
    el("button",{cls:"btn",text:"Import config",onclick:function(){fileIn.click()}}),
    el("span",{cls:"muted",style:"margin-left:10px",text:"Restores all settings + all relay schedules"})));
  cMaint.appendChild(fileIn);
  m.appendChild(cMaint);

  fileIn.addEventListener("change",function(){
    var f=fileIn.files&&fileIn.files[0];
    if(!f)return;
    var rd=new FileReader();
    rd.onload=async function(){
      try{
        var obj=JSON.parse(String(rd.result));
        if(!obj||(!obj.sys&&!obj.relays))throw new Error("Not an RM16 config file");
        var relays=obj.relays||[];
        for(var i=0;i<relays.length;i++){
          await api("/api/config/relay","POST",{i:relays[i].i,relay:relays[i]});
          toast("Relay "+((relays[i].i|0)+1)+" imported ("+(i+1)+"/"+relays.length+")","ok",1200);
        }
        if(obj.sys){
          toast("Importing system settings...","ok");
          await api("/api/config/sys","POST",obj.sys);
        }
        toast("Import complete","ok",5000);
        setTimeout(function(){location.reload()},1500);
      }catch(e){toast("Import failed: "+e.message,"bad",6000)}
    };
    rd.readAsText(f);
  });

  async function poll(){
    try{
      var s=await api("/api/status");
      sysBox.innerHTML="";
      sysBox.appendChild(kv("Firmware",s.fw));
      sysBox.appendChild(kv("Uptime",fmtDur(s.uptimeS)));
      sysBox.appendChild(kv("Boot count",s.bootCount));
      sysBox.appendChild(kv("Free heap",Math.round(s.heap.free/1024)+" kB"));
      sysBox.appendChild(kv("Largest free block",Math.round(s.heap.max/1024)+" kB"));
      sysBox.appendChild(kv("Minimum ever free",Math.round(s.heap.min/1024)+" kB"));
      sysBox.appendChild(kv("NVS writes",s.nvs.writes));
      wifiBox.innerHTML="";
      wifiBox.appendChild(kv("Station",s.wifi.staState===2?("connected: "+s.wifi.ssid):s.wifi.staState===1?"connecting":"disconnected"));
      wifiBox.appendChild(kv("Station IP",s.wifi.ip||"-"));
      wifiBox.appendChild(kv("Signal",s.wifi.rssi?String(s.wifi.rssi)+" dBm":"-"));
      wifiBox.appendChild(kv("Station MAC",s.wifi.mac||"-"));
      wifiBox.appendChild(kv("Access point",s.ap.on?(s.ap.ssid+" @ "+s.ap.ip+" ("+s.ap.clients+" clients)"):"off"));
      timeBox.innerHTML="";
      timeBox.appendChild(kv("Time source",s.time.srcName+(s.time.valid?"":" (not valid yet)")));
      timeBox.appendChild(kv("Last sync (UTC)",s.time.lastSyncUtc?new Date(s.time.lastSyncUtc*1000).toISOString().slice(0,19).replace("T"," "):"never"));
      timeBox.appendChild(kv("Drift compensation",s.time.driftPerDay+" s/day"));
      timeBox.appendChild(kv("DST",s.time.dstActive?"active":"inactive"));
      if(s.time.nextChangeUtc)timeBox.appendChild(kv("Next DST change",(s.time.nextChangeDir>0?"enters ":"leaves ")+"DST at UTC "+new Date(s.time.nextChangeUtc*1000).toISOString().slice(0,16).replace("T"," ")));
      timeBox.appendChild(kv("NTP failures (consecutive)",s.ntp.fails));
      tiles.innerHTML="";
      (s.relays||[]).forEach(function(r){
        tiles.appendChild(el("div",{cls:"tile"},
          el("div",{cls:"tn",text:r.n}),
          el("div",{cls:"spread"},
            el("span",{cls:"st "+(r.ovr?"forced":(r.on?"on":"off")),text:r.ovr===1?"FORCED ON":r.ovr===2?"FORCED OFF":(r.on?"ON":"OFF")}),
            el("span",{cls:"pinb",text:"GPIO"+r.pin}))));
      });
    }catch(e){}
  }
  poll();
  setInterval(poll,5000);
}
async function boot(){
  var m=buildShell();
  Clock.start();
  var pg=document.body.getAttribute("data-page");
  var fn={index:pgIndex,relays:pgRelays,gpio:pgGpio,time:pgTime,"wifi-sta":pgSta,"wifi-ap":pgAp,status:pgStatus}[pg];
  if(fn)await fn(m);
  else m.appendChild(card("Page not found","Use the menu above."));
}
if(document.readyState==="loading")document.addEventListener("DOMContentLoaded",boot);
else boot();
})();
)JS";

void ipToStr(uint32_t v, char *buf, size_t n) {
  snprintf(buf, n, "%u.%u.%u.%u",
           (unsigned)((v >> 24) & 0xFF), (unsigned)((v >> 16) & 0xFF),
           (unsigned)((v >> 8) & 0xFF), (unsigned)(v & 0xFF));
}

bool ipFromStr(const char *s, uint32_t *out) {
  if (!s || !*s) return false;
  uint32_t v = 0;
  int parts = 0;
  uint32_t cur = 0;
  bool any = false;
  const char *p = s;
  for (;;) {
    if (*p >= '0' && *p <= '9') {
      cur = cur * 10u + (uint32_t)(*p - '0');
      if (cur > 255) return false;
      any = true;
    } else if (*p == '.' || *p == 0) {
      if (!any) return false;
      v = (v << 8) | cur;
      parts++;
      cur = 0;
      any = false;
      if (*p == 0) break;
      if (parts >= 4) return false;
    } else {
      return false;
    }
    ++p;
  }
  if (parts != 4) return false;
  *out = v;
  return true;
}

void sendPage(const char *pg) {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/html", pg);
}

void sendDoc(int code, JsonDocument *doc) {
  String out;
  serializeJson(*doc, out);
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", out);
}

void jErr(int code, const char *msg) {
  JsonDocument doc;
  doc["error"] = msg;
  sendDoc(code, &doc);
}

bool getBodyJson(JsonDocument *doc, int maxLen) {
  if (!server.hasArg("plain")) { jErr(400, "missing JSON body"); return false; }
  String body = server.arg("plain");
  if ((int)body.length() > maxLen) { jErr(413, "body too large"); return false; }
  if (body.length() == 0) { jErr(400, "empty body"); return false; }
  if (deserializeJson(*doc, body.c_str())) { jErr(400, "invalid JSON"); return false; }
  return true;
}

bool parsePatternObj(JsonVariant src, TimePattern *tp) {
  if (src.isNull() || !src.is<JsonObject>()) return false;
  JsonObject o = src.as<JsonObject>();
  const char *a = o["sec"].as<const char *>();
  const char *b = o["min"].as<const char *>();
  const char *c = o["hr"].as<const char *>();
  const char *d = o["dom"].as<const char *>();
  const char *e = o["mon"].as<const char *>();
  const char *f = o["dow"].as<const char *>();
  if (!a || !b || !c || !d || !e || !f) return false;
  uint64_t m;
  if (!parseSpec(a, 0, 59, &m)) return false;
  tp->sec = m;
  if (!parseSpec(b, 0, 59, &m)) return false;
  tp->min = m;
  if (!parseSpec(c, 0, 23, &m)) return false;
  tp->hr = (uint32_t)m;
  if (!parseSpec(d, 1, 31, &m)) return false;
  tp->dom = (uint32_t)m;
  if (!parseSpec(e, 1, 12, &m)) return false;
  tp->mon = (uint16_t)m;
  if (!parseSpec(f, 0, 6, &m)) return false;
  tp->dow = (uint8_t)m;
  tp->flags = 0;
  return true;
}

void patternToJson(const TimePattern *tp, JsonObject dst) {
  String s;
  buildSpec(tp->sec, 0, 59, &s);  dst["sec"] = s;
  buildSpec(tp->min, 0, 59, &s);  dst["min"] = s;
  buildSpec(tp->hr, 0, 23, &s);   dst["hr"] = s;
  buildSpec(tp->dom, 1, 31, &s);  dst["dom"] = s;
  buildSpec(tp->mon, 1, 12, &s);  dst["mon"] = s;
  buildSpec(tp->dow, 0, 6, &s);   dst["dow"] = s;
}

bool parsePairObj(JsonObject src, SchedulePair *sp) {
  if (src.isNull()) return false;
  sp->enabled = src["en"].as<int>() ? 1 : 0;
  if (!parsePatternObj(src["st"], &sp->start)) return false;
  if (!parsePatternObj(src["sp"], &sp->stop)) return false;
  sp->rsv = 0;
  return true;
}

void pairToJson(const SchedulePair *sp, JsonObject dst) {
  dst["en"] = sp->enabled ? 1 : 0;
  JsonObject st = dst["st"].to<JsonObject>();
  patternToJson(&sp->start, st);
  JsonObject sp2 = dst["sp"].to<JsonObject>();
  patternToJson(&sp->stop, sp2);
}

bool relayLogicalOn(uint8_t i) {
  if (rcfg[i].overrideMode == 1) return true;
  if (rcfg[i].overrideMode == 2) return false;
  return desiredOn[i];
}

void handleRoot()        { sendPage(PG_INDEX); }
void handleRelaysPage()  { sendPage(PG_RELAYS); }
void handleGpioPage()    { sendPage(PG_GPIO); }
void handleTimePage()    { sendPage(PG_TIME); }
void handleStaPage()     { sendPage(PG_STA); }
void handleApPage()      { sendPage(PG_AP); }
void handleStatusPage()  { sendPage(PG_STATUS); }
void handlePortalPage()  { sendPage(PG_PORTAL); }

void handleCss() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/css", PG_CSS);
}
void handleJs() {
  server.sendHeader("Cache-Control", "no-cache");
  server.send_P(200, "text/javascript", PG_JS);
}

void handlePortalProbe() {
  if (apActive) {
    server.sendHeader("Location", "/portal");
    server.send(302, "text/plain", "");
  } else {
    jErr(404, "not found");
  }
}

void handleNotFound() {
  if (apActive) {
    char ipb[16];
    ipToStr(cfg.apIp, ipb, sizeof(ipb));
    String loc = "http://";
    loc += ipb;
    loc += "/portal";
    server.sendHeader("Location", loc);
    server.send(302, "text/plain", "");
    return;
  }
  jErr(404, "not found");
}

void webSetup() {
  static const char *PROBES[] = {
    "/generate_204", "/gen_204.nc", "/hotspot-detect.html",
    "/library/test/success.html", "/connecttest.txt",
    "/redirect", "/fwlink", "/icon.png", "/ncsi.txt", 0
  };
  server.on("/", HTTP_GET, handleRoot);
  server.on("/relays", HTTP_GET, handleRelaysPage);
  server.on("/gpio", HTTP_GET, handleGpioPage);
  server.on("/time", HTTP_GET, handleTimePage);
  server.on("/wifi-sta", HTTP_GET, handleStaPage);
  server.on("/wifi-ap", HTTP_GET, handleApPage);
  server.on("/status", HTTP_GET, handleStatusPage);
  server.on("/portal", HTTP_GET, handlePortalPage);
  server.on("/style.css", HTTP_GET, handleCss);
  server.on("/app.js", HTTP_GET, handleJs);
  for (int i = 0; PROBES[i]; ++i) server.on(PROBES[i], HTTP_GET, handlePortalProbe);

  server.on("/api/summary", HTTP_GET, handleApiSummary);
  server.on("/api/relays", HTTP_GET, handleApiRelays);
  server.on("/api/relay", HTTP_GET, handleApiRelayGet);
  server.on("/api/relay/name", HTTP_POST, handleApiRelayName);
  server.on("/api/relay/pair", HTTP_POST, handleApiRelayPair);
  server.on("/api/relay/pairsClear", HTTP_POST, handleApiRelayPairsClear);
  server.on("/api/relay/schedEnable", HTTP_POST, handleApiRelaySchedEnable);
  server.on("/api/gpio", HTTP_GET, handleApiGpioGet);
  server.on("/api/gpio/global", HTTP_POST, handleApiGpioGlobal);
  server.on("/api/gpio/polarity", HTTP_POST, handleApiGpioPolarity);
  server.on("/api/gpio/force", HTTP_POST, handleApiGpioForce);
  server.on("/api/gpio/remove", HTTP_POST, handleApiGpioRemove);
  server.on("/api/time", HTTP_GET, handleApiTimeGet);
  server.on("/api/time/config", HTTP_POST, handleApiTimeConfig);
  server.on("/api/time/ntpSync", HTTP_POST, handleApiTimeNtpSync);
  server.on("/api/time/browser", HTTP_POST, handleApiTimeBrowser);
  server.on("/api/time/rtcRead", HTTP_POST, handleApiTimeRtcRead);
  server.on("/api/time/rtcWrite", HTTP_POST, handleApiTimeRtcWrite);
  server.on("/api/wifi/scan", HTTP_GET, handleApiWifiScan);
  server.on("/api/wifi/scanStart", HTTP_POST, handleApiWifiScanStart);
  server.on("/api/wifi/sta", HTTP_GET, handleApiWifiStaGet);
  server.on("/api/wifi/sta", HTTP_POST, handleApiWifiStaPost);
  server.on("/api/wifi/staConnect", HTTP_POST, handleApiWifiStaConnect);
  server.on("/api/wifi/staDisconnect", HTTP_POST, handleApiWifiStaDisconnect);
  server.on("/api/wifi/ap", HTTP_GET, handleApiWifiApGet);
  server.on("/api/wifi/ap", HTTP_POST, handleApiWifiApPost);
  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/reboot", HTTP_POST, handleApiReboot);
  server.on("/api/factory", HTTP_POST, handleApiFactory);
  server.on("/api/sched/recalc", HTTP_POST, handleApiSchedRecalc);
  server.on("/api/config/export", HTTP_GET, handleApiConfigExport);
  server.on("/api/config/sys", HTTP_POST, handleApiConfigSys);
  server.on("/api/config/relay", HTTP_POST, handleApiConfigRelay);

  server.onNotFound(handleNotFound);
  server.begin();
}

void handleApiSummary() {
  int64_t now = nowUtc();
  tzCompute(now);
  JsonDocument doc;
  doc["fw"] = FW_VERSION;
  doc["epochUtc"] = now;
  doc["offSec"] = curOffSec;
  doc["dstActive"] = dstActive ? 1 : 0;
  doc["src"] = timeSrc;
  doc["srcName"] = timeSrcName;
  doc["valid"] = timeValid;
  JsonArray arr = doc["relays"].to<JsonArray>();
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    JsonObject o = arr.add<JsonObject>();
    o["i"] = i;
    o["n"] = rcfg[i].name;
    o["pin"] = RELAY_PINS[i];
    o["on"] = relayLogicalOn(i) ? 1 : 0;
    o["ovr"] = rcfg[i].overrideMode;
    o["se"] = rcfg[i].schedulesEnabled;
  }
  JsonObject w = doc["wifi"].to<JsonObject>();
  char b[16];
  w["staState"] = staState;
  w["ssid"] = WiFi.SSID();
  ipToStr(ipToU32(WiFi.localIP()), b, sizeof(b));
  w["ip"] = (staState == 2) ? b : "";
  w["rssi"] = WiFi.RSSI();
  w["mac"] = WiFi.macAddress();
  w["apOn"] = apActive ? 1 : 0;
  w["apSsid"] = cfg.apSsid;
  ipToStr(cfg.apIp, b, sizeof(b));
  w["apIp"] = b;
  w["clients"] = WiFi.softAPgetStationNum();
  JsonObject h = doc["heap"].to<JsonObject>();
  h["free"] = (uint32_t)esp_get_free_heap_size();
  h["max"] = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  h["min"] = esp_get_minimum_free_heap_size();
  doc["uptimeS"] = (int64_t)(esp_timer_get_time() / 1000000LL);
  doc["bootCount"] = cfg.bootCount;
  sendDoc(200, &doc);
}

void handleApiRelays() {
  JsonDocument doc;
  JsonArray arr = doc["relays"].to<JsonArray>();
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    JsonObject o = arr.add<JsonObject>();
    int en = 0;
    for (int p = 0; p < N_PAIRS; ++p) en += rcfg[i].pairs[p].enabled ? 1 : 0;
    o["i"] = i;
    o["n"] = rcfg[i].name;
    o["pin"] = RELAY_PINS[i];
    o["on"] = relayLogicalOn(i) ? 1 : 0;
    o["ovr"] = rcfg[i].overrideMode;
    o["se"] = rcfg[i].schedulesEnabled;
    o["enc"] = en;
  }
  sendDoc(200, &doc);
}

void handleApiRelayGet() {
  if (!server.hasArg("i")) { jErr(400, "missing i"); return; }
  long i = server.arg("i").toInt();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  JsonDocument doc;
  doc["i"] = (int)i;
  doc["n"] = rcfg[i].name;
  doc["pin"] = RELAY_PINS[i];
  doc["pol"] = rcfg[i].polarity;
  doc["ovr"] = rcfg[i].overrideMode;
  doc["se"] = rcfg[i].schedulesEnabled;
  JsonArray arr = doc["pairs"].to<JsonArray>();
  for (int p = 0; p < N_PAIRS; ++p) {
    JsonObject po = arr.add<JsonObject>();
    pairToJson(&rcfg[i].pairs[p], po);
  }
  sendDoc(200, &doc);
}

void handleApiRelayName() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull()) { jErr(400, "missing i"); return; }
  int i = o["i"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  const char *name = o["name"].as<const char *>();
  if (!name) { jErr(400, "missing name"); return; }
  size_t l = strlen(name);
  if (l < 1 || l > 23) { jErr(400, "name must be 1..23 chars"); return; }
  for (size_t k = 0; k < l; ++k) {
    unsigned char c = (unsigned char)name[k];
    if (c < 0x20 || c == 0x7F) { jErr(400, "name has control characters"); return; }
  }
  sstrlcpy(rcfg[i].name, name, sizeof(rcfg[i].name));
  markRlyDirty((uint8_t)i);
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiRelayPair() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX_RELAY)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull() || o["p"].isNull()) { jErr(400, "missing i or p"); return; }
  int i = o["i"].as<int>();
  int p = o["p"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  if (p < 0 || p >= N_PAIRS) { jErr(400, "p out of range"); return; }
  if (o["pair"].isNull()) { jErr(400, "missing pair"); return; }
  SchedulePair np;
  memset(&np, 0, sizeof(np));
  if (!parsePairObj(o["pair"].as<JsonObject>(), &np)) { jErr(400, "invalid pair (check every field has a value)"); return; }
  rcfg[i].pairs[p] = np;
  markRlyDirty((uint8_t)i);
  recomputeOne((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiRelayPairsClear() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull()) { jErr(400, "missing i"); return; }
  int i = o["i"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  for (int p = 0; p < N_PAIRS; ++p) rcfg[i].pairs[p].enabled = 0;
  markRlyDirty((uint8_t)i);
  recomputeOne((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiRelaySchedEnable() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull()) { jErr(400, "missing i"); return; }
  int i = o["i"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  if (o["on"].isNull()) { jErr(400, "missing on"); return; }
  rcfg[i].schedulesEnabled = o["on"].as<int>() ? 1 : 0;
  markRlyDirty((uint8_t)i);
  recomputeOne((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiGpioGet() {
  JsonDocument doc;
  doc["globalActiveHigh"] = cfg.globalActiveHigh;
  JsonArray arr = doc["relays"].to<JsonArray>();
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    JsonObject o = arr.add<JsonObject>();
    bool activeHigh;
    if (rcfg[i].polarity == 1) activeHigh = true;
    else if (rcfg[i].polarity == 2) activeHigh = false;
    else activeHigh = (cfg.globalActiveHigh == 1);
    o["i"] = i;
    o["n"] = rcfg[i].name;
    o["pin"] = RELAY_PINS[i];
    o["pol"] = rcfg[i].polarity;
    o["eff"] = activeHigh ? 1 : 0;
    o["ovr"] = rcfg[i].overrideMode;
    o["on"] = relayLogicalOn(i) ? 1 : 0;
  }
  sendDoc(200, &doc);
}

void handleApiGpioGlobal() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["activeHigh"].isNull()) { jErr(400, "missing activeHigh"); return; }
  int v = o["activeHigh"].as<int>();
  if (v != 0 && v != 1) { jErr(400, "activeHigh must be 0 or 1"); return; }
  cfg.globalActiveHigh = (uint8_t)v;
  markSysDirty();
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiGpioPolarity() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull() || o["pol"].isNull()) { jErr(400, "missing i or pol"); return; }
  int i = o["i"].as<int>();
  int p = o["pol"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  if (p < 0 || p > 2) { jErr(400, "pol must be 0,1,2"); return; }
  rcfg[i].polarity = (uint8_t)p;
  markRlyDirty((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

static bool forceHelper() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return false;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull()) { jErr(400, "missing i"); return false; }
  int i = o["i"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return false; }
  int mode = 0;
  if (!o["mode"].isNull()) mode = o["mode"].as<int>();
  if (mode < 0 || mode > 2) { jErr(400, "mode must be 0,1,2"); return false; }
  rcfg[i].overrideMode = (uint8_t)mode;
  markRlyDirty((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
  return true;
}

void handleApiGpioForce()  { forceHelper(); }
void handleApiGpioRemove() { forceHelper(); }

static bool validHost(const char *s) {
  if (!s) return true;
  size_t l = strlen(s);
  if (l == 0) return true;
  if (l > 48) return false;
  for (size_t k = 0; k < l; ++k) {
    char c = s[k];
    if (c >= 'a' && c <= 'z') continue;
    if (c >= 'A' && c <= 'Z') continue;
    if (c >= '0' && c <= '9') continue;
    if (c == '.' || c == '-' || c == ':' ) continue;
    return false;
  }
  return true;
}

bool parseTimeCfg(JsonObject o) {
  if (o.isNull()) return false;
  if (!o["tzMode"].isNull()) {
    int v = o["tzMode"].as<int>();
    if (v < 0 || v > 1) return false;
    cfg.tzMode = (uint8_t)v;
  }
  if (!o["tzPreset"].isNull()) {
    int v = o["tzPreset"].as<int>();
    if (v < 0 || v >= TZ_COUNT) return false;
    cfg.tzPreset = (uint8_t)v;
  }
  if (!o["gmtOffS"].isNull()) {
    int32_t v = o["gmtOffS"].as<int32_t>();
    if (v < -50400 || v > 50400) return false;
    cfg.gmtOffS = v;
  }
  if (!o["dstMode"].isNull()) {
    int v = o["dstMode"].as<int>();
    if (v < 0 || v > 1) return false;
    cfg.dstMode = (uint8_t)v;
  }
  if (!o["dstShiftMin"].isNull()) {
    int v = o["dstShiftMin"].as<int>();
    if (v < 1 || v > 720) return false;
    cfg.dstShiftMin = (uint16_t)v;
  }
  if (!o["ds"].isNull() && o["ds"].is<JsonObject>()) {
    JsonObject d = o["ds"].as<JsonObject>();
    int mon = d["mon"].as<int>(), week = d["week"].as<int>(), dow = d["dow"].as<int>();
    int hour = d["hour"].as<int>(), mi = d["min"].as<int>();
    if (mon < 1 || mon > 12 || week < 1 || week > 5 || dow < 0 || dow > 6 ||
        hour < 0 || hour > 23 || mi < 0 || mi > 59) return false;
    cfg.dsMon = (uint8_t)mon; cfg.dsWeek = (uint8_t)week; cfg.dsDow = (uint8_t)dow;
    cfg.dsHour = (uint8_t)hour; cfg.dsMin = (uint8_t)mi;
  }
  if (!o["de"].isNull() && o["de"].is<JsonObject>()) {
    JsonObject d = o["de"].as<JsonObject>();
    int mon = d["mon"].as<int>(), week = d["week"].as<int>(), dow = d["dow"].as<int>();
    int hour = d["hour"].as<int>(), mi = d["min"].as<int>();
    if (mon < 1 || mon > 12 || week < 1 || week > 5 || dow < 0 || dow > 6 ||
        hour < 0 || hour > 23 || mi < 0 || mi > 59) return false;
    cfg.deMon = (uint8_t)mon; cfg.deWeek = (uint8_t)week; cfg.deDow = (uint8_t)dow;
    cfg.deHour = (uint8_t)hour; cfg.deMin = (uint8_t)mi;
  }
  if (!o["driftPerDay"].isNull()) {
    int v = o["driftPerDay"].as<int>();
    if (v < -30 || v > 30) return false;
    cfg.driftPerDay = (int8_t)v;
  }
  const char *n1 = o["ntp1"].isNull() ? NULL : o["ntp1"].as<const char *>();
  const char *n2 = o["ntp2"].isNull() ? NULL : o["ntp2"].as<const char *>();
  const char *n3 = o["ntp3"].isNull() ? NULL : o["ntp3"].as<const char *>();
  if (!validHost(n1) || !validHost(n2) || !validHost(n3)) return false;
  if (n1) sstrlcpy(cfg.ntp1, n1, sizeof(cfg.ntp1));
  if (n2) sstrlcpy(cfg.ntp2, n2, sizeof(cfg.ntp2));
  if (n3) sstrlcpy(cfg.ntp3, n3, sizeof(cfg.ntp3));
  if (!o["ntpTimeoutS"].isNull()) {
    long v = o["ntpTimeoutS"].as<long>();
    if (v < 1 || v > 60) return false;
    cfg.ntpTimeoutS = (uint32_t)v;
  }
  if (!o["ntpIntervalS"].isNull()) {
    long v = o["ntpIntervalS"].as<long>();
    if (v < 60 || v > 86400) return false;
    cfg.ntpIntervalS = (uint32_t)v;
  }
  if (!o["rtcEnabled"].isNull()) {
    int v = o["rtcEnabled"].as<int>();
    if (v != 0 && v != 1) return false;
    cfg.rtcEnabled = (uint8_t)v;
  }
  if (!o["rtcResyncMin"].isNull()) {
    long v = o["rtcResyncMin"].as<long>();
    if (v < 1 || v > 1440) return false;
    cfg.rtcResyncMin = (uint16_t)v;
  }
  return true;
}

void handleApiTimeGet() {
  int64_t now = nowUtc();
  tzCompute(now);
  JsonDocument doc;
  doc["epochUtc"] = now;
  doc["offSec"] = curOffSec;
  doc["dstActive"] = dstActive ? 1 : 0;
  doc["dstMin"] = g_dstMin;
  JsonObject l = doc["local"].to<JsonObject>();
  int y, mo, d, h, mi, s, dow;
  fieldsFromWallSec(now + (int64_t)curOffSec, &y, &mo, &d, &h, &mi, &s, &dow);
  l["y"] = y; l["mo"] = mo; l["d"] = d;
  l["h"] = h; l["mi"] = mi; l["s"] = s; l["dow"] = dow;
  doc["src"] = timeSrc;
  doc["srcName"] = timeSrcName;
  doc["valid"] = timeValid;
  doc["lastSyncUtc"] = lastSyncUtc;
  doc["tzMode"] = cfg.tzMode;
  doc["tzPreset"] = cfg.tzPreset;
  doc["gmtOffS"] = cfg.gmtOffS;
  doc["dstMode"] = cfg.dstMode;
  doc["dstShiftMin"] = cfg.dstShiftMin;
  doc["tzName"] = (cfg.tzMode == 1 && cfg.tzPreset < TZ_COUNT) ? TZ_PRESETS[cfg.tzPreset].name : "Manual";
  JsonObject ds = doc["ds"].to<JsonObject>();
  ds["mon"] = cfg.dsMon; ds["week"] = cfg.dsWeek; ds["dow"] = cfg.dsDow;
  ds["hour"] = cfg.dsHour; ds["min"] = cfg.dsMin;
  JsonObject de = doc["de"].to<JsonObject>();
  de["mon"] = cfg.deMon; de["week"] = cfg.deWeek; de["dow"] = cfg.deDow;
  de["hour"] = cfg.deHour; de["min"] = cfg.deMin;
  doc["driftPerDay"] = cfg.driftPerDay;
  doc["nextChangeUtc"] = nextChangeUtc;
  doc["nextChangeDir"] = nextChangeDir;
  JsonArray tz = doc["tz"].to<JsonArray>();
  for (int i = 0; i < TZ_COUNT; ++i) {
    JsonObject z = tz.add<JsonObject>();
    char ob[8];
    offStr(TZ_PRESETS[i].stdOff, ob, sizeof(ob));
    z["i"] = i;
    z["n"] = TZ_PRESETS[i].name;
    z["o"] = ob;
  }
  JsonObject n = doc["ntp"].to<JsonObject>();
  n["state"] = ntp.state;
  n["s1"] = cfg.ntp1;
  n["s2"] = cfg.ntp2;
  n["s3"] = cfg.ntp3;
  n["timeout"] = cfg.ntpTimeoutS;
  n["interval"] = cfg.ntpIntervalS;
  n["fails"] = ntp.fails;
  n["lastOkUtc"] = ntp.lastOkUtc;
  n["lastOkSrv"] = ntp.lastOkSrv;
  n["nextIn"] = (ntp.state == 0 && ntp.nextTryUtc > now) ? (int32_t)(ntp.nextTryUtc - now) : 0;
  JsonObject r = doc["rtc"].to<JsonObject>();
  r["present"] = rtcPresent ? 1 : 0;
  r["lostPower"] = rtcLostPower ? 1 : 0;
  r["enabled"] = cfg.rtcEnabled;
  r["resyncMin"] = cfg.rtcResyncMin;
  r["temp"] = rtcPresent ? (double)rtc.getTemperature() : (double)0.0;
  int64_t re = 0;
  r["epochUtc"] = rtcReadEpoch(&re) ? re : (int64_t)0;
  sendDoc(200, &doc);
}

void handleApiTimeConfig() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  SysCfg backup = cfg;
  if (!parseTimeCfg(doc.as<JsonObject>())) {
    cfg = backup;
    jErr(400, "invalid time configuration value");
    return;
  }
  markSysDirty();
  tzCompute(nowUtc());
  recomputeAll();
  ntpKick();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiTimeNtpSync() {
  ntpKick();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiTimeBrowser() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["epochMs"].isNull()) { jErr(400, "missing epochMs"); return; }
  int64_t ms = o["epochMs"].as<int64_t>();
  int64_t epoch = ms / 1000;
  if (epoch < 946684800LL || epoch > 4354819200LL) { jErr(400, "epochMs out of range"); return; }
  setSystemTime(epoch, TSRC_MANUAL, "Browser", true);
  rtcWriteGuarded(epoch);
  JsonDocument r;
  r["ok"] = 1;
  r["epochUtc"] = epoch;
  sendDoc(200, &r);
}

void handleApiTimeRtcRead() {
  int64_t e;
  JsonDocument r;
  if (rtcReadEpoch(&e)) {
    setSystemTime(e, TSRC_RTC, "DS3231", true);
    r["ok"] = 1;
    r["epochUtc"] = e;
  } else {
    r["ok"] = 0;
  }
  sendDoc(200, &r);
}

void handleApiTimeRtcWrite() {
  if (!timeValid) { jErr(409, "no valid time source yet - sync first"); return; }
  int64_t now = nowUtc();
  rtcWriteNow(now);
  JsonDocument r;
  r["ok"] = (rtcPresent && now >= 946684800LL && now <= 4102444800LL) ? 1 : 0;
  sendDoc(200, &r);
}

bool parseStaCfg(JsonObject o) {
  if (o.isNull()) return false;
  if (!o["staEnabled"].isNull()) {
    int v = o["staEnabled"].as<int>();
    if (v != 0 && v != 1) return false;
    cfg.staEnabled = (uint8_t)v;
  }
  if (!o["ssid"].isNull()) {
    const char *s = o["ssid"].as<const char *>();
    if (s && strlen(s) > 32) return false;
    sstrlcpy(cfg.staSsid, s ? s : "", sizeof(cfg.staSsid));
  }
  if (!o["pass"].isNull()) {
    const char *s = o["pass"].as<const char *>();
    if (s && strlen(s) > 64) return false;
    sstrlcpy(cfg.staPass, s ? s : "", sizeof(cfg.staPass));
  }
  if (!o["hostname"].isNull()) {
    const char *s = o["hostname"].as<const char *>();
    if (s && strlen(s) > 32) return false;
    if (s && s[0]) {
      for (const char *p = s; *p; ++p) {
        char c = *p;
        if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
              (c >= '0' && c <= '9') || c == '-')) return false;
      }
      sstrlcpy(cfg.staHost, s, sizeof(cfg.staHost));
    } else {
      sstrlcpy(cfg.staHost, "relay16", sizeof(cfg.staHost));
    }
  }
  if (!o["static"].isNull()) {
    int v = o["static"].as<int>();
    if (v != 0 && v != 1) return false;
    cfg.staStatic = (uint8_t)v;
  }
  uint32_t ip;
  if (!o["ip"].isNull()) {
    const char *s = o["ip"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.staIp = ip;
  }
  if (!o["gw"].isNull()) {
    const char *s = o["gw"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.staGw = ip;
  }
  if (!o["mask"].isNull()) {
    const char *s = o["mask"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.staMask = ip;
  }
  if (!o["dns1"].isNull()) {
    const char *s = o["dns1"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.staDns1 = ip;
  }
  if (!o["dns2"].isNull()) {
    const char *s = o["dns2"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.staDns2 = ip;
  }
  return true;
}

bool parseApCfg(JsonObject o) {
  if (o.isNull()) return false;
  if (!o["policy"].isNull()) {
    int v = o["policy"].as<int>();
    if (v < 0 || v > 2) return false;
    cfg.apPolicy = (uint8_t)v;
  }
  if (!o["ssid"].isNull()) {
    const char *s = o["ssid"].as<const char *>();
    if (s && strlen(s) > 32) return false;
    sstrlcpy(cfg.apSsid, s ? s : "", sizeof(cfg.apSsid));
  }
  uint8_t open = cfg.apOpen;
  if (!o["open"].isNull()) {
    int v = o["open"].as<int>();
    if (v != 0 && v != 1) return false;
    open = (uint8_t)v;
    cfg.apOpen = open;
  }
  if (!o["pass"].isNull()) {
    const char *s = o["pass"].as<const char *>();
    if (s && strlen(s) > 64) return false;
    if (!open && s && s[0] && strlen(s) < 8) return false;
    sstrlcpy(cfg.apPass, s ? s : "", sizeof(cfg.apPass));
  }
  if (!open && cfg.apPass[0] == 0) {
    sstrlcpy(cfg.apPass, "relay12345", sizeof(cfg.apPass));
  }
  if (!o["channel"].isNull()) {
    long v = o["channel"].as<long>();
    if (v < 1 || v > 13) return false;
    cfg.apChannel = (uint8_t)v;
  }
  if (!o["hidden"].isNull()) {
    int v = o["hidden"].as<int>();
    if (v != 0 && v != 1) return false;
    cfg.apHidden = (uint8_t)v;
  }
  if (!o["maxConn"].isNull()) {
    long v = o["maxConn"].as<long>();
    if (v < 1 || v > 8) return false;
    cfg.apMaxConn = (uint8_t)v;
  }
  uint32_t ip;
  if (!o["ip"].isNull()) {
    const char *s = o["ip"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.apIp = ip;
  }
  if (!o["gw"].isNull()) {
    const char *s = o["gw"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.apGw = ip;
  }
  if (!o["mask"].isNull()) {
    const char *s = o["mask"].as<const char *>();
    if (s && *s && !ipFromStr(s, &ip)) return false;
    if (s && *s) cfg.apMask = ip;
  }
  return true;
}

bool parseMiscCfg(JsonObject o) {
  if (o.isNull()) return false;
  if (!o["globalActiveHigh"].isNull()) {
    int v = o["globalActiveHigh"].as<int>();
    if (v != 0 && v != 1) return false;
    cfg.globalActiveHigh = (uint8_t)v;
  }
  return true;
}

void handleApiWifiScanStart() {
  scanStart();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiWifiScan() {
  scanPoll();
  JsonDocument doc;
  doc["scanning"] = scanBusy ? 1 : 0;
  JsonArray arr = doc["nets"].to<JsonArray>();
  for (uint8_t i = 0; i < scanNetCount; ++i) {
    JsonObject n = arr.add<JsonObject>();
    n["s"] = scanNets[i].ssid;
    n["r"] = scanNets[i].rssi;
    n["sec"] = scanNets[i].sec ? 1 : 0;
    n["ch"] = scanNets[i].ch;
  }
  sendDoc(200, &doc);
}

void handleApiWifiStaGet() {
  JsonDocument doc;
  char b[16];
  doc["state"] = staState;
  doc["ssid"] = WiFi.SSID();
  ipToStr(ipToU32(WiFi.localIP()), b, sizeof(b));
  doc["ip"] = (staState == 2) ? b : "";
  doc["rssi"] = WiFi.RSSI();
  doc["mac"] = WiFi.macAddress();
  doc["enabled"] = cfg.staEnabled;
  doc["ssidCfg"] = cfg.staSsid;
  doc["hostname"] = cfg.staHost;
  doc["static"] = cfg.staStatic;
  ipToStr(cfg.staIp, b, sizeof(b));   doc["ipCfg"] = b;
  ipToStr(cfg.staGw, b, sizeof(b));   doc["gwCfg"] = b;
  ipToStr(cfg.staMask, b, sizeof(b)); doc["maskCfg"] = b;
  ipToStr(cfg.staDns1, b, sizeof(b)); doc["dns1Cfg"] = b;
  ipToStr(cfg.staDns2, b, sizeof(b)); doc["dns2Cfg"] = b;
  sendDoc(200, &doc);
}

void handleApiWifiStaPost() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  SysCfg backup = cfg;
  if (!parseStaCfg(o)) { cfg = backup; jErr(400, "invalid station settings"); return; }
  markSysDirty();
  staApplyLive();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiWifiStaConnect() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull()) { jErr(400, "invalid body"); return; }
  const char *ssid = o["ssid"].isNull() ? NULL : o["ssid"].as<const char *>();
  if (!ssid || !ssid[0] || strlen(ssid) > 32) { jErr(400, "invalid ssid"); return; }
  const char *pass = o["pass"].isNull() ? NULL : o["pass"].as<const char *>();
  if (pass && strlen(pass) > 64) { jErr(400, "invalid pass"); return; }
  sstrlcpy(cfg.staSsid, ssid, sizeof(cfg.staSsid));
  sstrlcpy(cfg.staPass, pass ? pass : "", sizeof(cfg.staPass));
  cfg.staEnabled = 1;
  staHold = false;
  saveSysCfgNow();
  apGraceUntilUs = esp_timer_get_time() + 120000000LL;
  staApplyLive();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiWifiStaDisconnect() {
  staHold = true;
  WiFi.disconnect(false, false);
  staState = 0;
  apEvaluate();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiWifiApGet() {
  JsonDocument doc;
  char b[16];
  doc["on"] = apActive ? 1 : 0;
  doc["ssid"] = cfg.apSsid;
  ipToStr(ipToU32(WiFi.softAPIP()), b, sizeof(b));
  doc["ip"] = b;
  doc["clients"] = WiFi.softAPgetStationNum();
  doc["policy"] = cfg.apPolicy;
  doc["ssidCfg"] = cfg.apSsid;
  doc["passCfg"] = cfg.apPass;
  doc["open"] = cfg.apOpen;
  doc["channel"] = cfg.apChannel;
  doc["hidden"] = cfg.apHidden;
  doc["maxConn"] = cfg.apMaxConn;
  ipToStr(cfg.apIp, b, sizeof(b));   doc["ipCfg"] = b;
  ipToStr(cfg.apGw, b, sizeof(b));   doc["gwCfg"] = b;
  ipToStr(cfg.apMask, b, sizeof(b)); doc["maskCfg"] = b;
  sendDoc(200, &doc);
}

void handleApiWifiApPost() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject o = doc.as<JsonObject>();
  SysCfg backup = cfg;
  if (!parseApCfg(o)) { cfg = backup; jErr(400, "invalid AP settings (password must be 8-64 chars unless open)"); return; }
  markSysDirty();
  if (apActive) { stopAp(); apLastChangeUs = 0; }
  apEvaluate();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiStatus() {
  int64_t now = nowUtc();
  tzCompute(now);
  JsonDocument doc;
  doc["fw"] = FW_VERSION;
  doc["uptimeS"] = (int64_t)(esp_timer_get_time() / 1000000LL);
  doc["bootCount"] = cfg.bootCount;
  JsonObject h = doc["heap"].to<JsonObject>();
  h["free"] = (uint32_t)esp_get_free_heap_size();
  h["max"] = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  h["min"] = esp_get_minimum_free_heap_size();
  JsonObject w = doc["wifi"].to<JsonObject>();
  char b[16];
  w["staState"] = staState;
  w["ssid"] = WiFi.SSID();
  ipToStr(ipToU32(WiFi.localIP()), b, sizeof(b));
  w["ip"] = (staState == 2) ? b : "";
  w["rssi"] = WiFi.RSSI();
  w["mac"] = WiFi.macAddress();
  JsonObject ap = doc["ap"].to<JsonObject>();
  ap["on"] = apActive ? 1 : 0;
  ap["ssid"] = cfg.apSsid;
  ipToStr(cfg.apIp, b, sizeof(b));
  ap["ip"] = b;
  ap["clients"] = WiFi.softAPgetStationNum();
  JsonObject t = doc["time"].to<JsonObject>();
  t["srcName"] = timeSrcName;
  t["valid"] = timeValid;
  t["lastSyncUtc"] = lastSyncUtc;
  t["driftPerDay"] = cfg.driftPerDay;
  t["dstActive"] = dstActive ? 1 : 0;
  t["nextChangeUtc"] = nextChangeUtc;
  t["nextChangeDir"] = nextChangeDir;
  JsonObject np = doc["ntp"].to<JsonObject>();
  np["fails"] = ntp.fails;
  np["lastOkSrv"] = ntp.lastOkSrv;
  JsonObject nv = doc["nvs"].to<JsonObject>();
  nv["writes"] = nvsWrites;
  JsonArray arr = doc["relays"].to<JsonArray>();
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    JsonObject o = arr.add<JsonObject>();
    o["i"] = i;
    o["n"] = rcfg[i].name;
    o["pin"] = RELAY_PINS[i];
    o["on"] = relayLogicalOn(i) ? 1 : 0;
    o["ovr"] = rcfg[i].overrideMode;
  }
  sendDoc(200, &doc);
}

void handleApiReboot() {
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
  pendingReboot("web", 400);
}

void handleApiFactory() {
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
  factoryReset();
}

void handleApiSchedRecalc() {
  recomputeAll();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void sysCfgToJson(JsonObject o) {
  char b[16];
  JsonObject st = o["sta"].to<JsonObject>();
  st["staEnabled"] = cfg.staEnabled;
  st["ssid"] = cfg.staSsid;
  st["pass"] = cfg.staPass;
  st["hostname"] = cfg.staHost;
  st["static"] = cfg.staStatic;
  ipToStr(cfg.staIp, b, sizeof(b));   st["ip"] = b;
  ipToStr(cfg.staGw, b, sizeof(b));   st["gw"] = b;
  ipToStr(cfg.staMask, b, sizeof(b)); st["mask"] = b;
  ipToStr(cfg.staDns1, b, sizeof(b)); st["dns1"] = b;
  ipToStr(cfg.staDns2, b, sizeof(b)); st["dns2"] = b;
  JsonObject ap = o["ap"].to<JsonObject>();
  ap["policy"] = cfg.apPolicy;
  ap["ssid"] = cfg.apSsid;
  ap["pass"] = cfg.apPass;
  ap["open"] = cfg.apOpen;
  ap["channel"] = cfg.apChannel;
  ap["hidden"] = cfg.apHidden;
  ap["maxConn"] = cfg.apMaxConn;
  ipToStr(cfg.apIp, b, sizeof(b));    ap["ip"] = b;
  ipToStr(cfg.apGw, b, sizeof(b));    ap["gw"] = b;
  ipToStr(cfg.apMask, b, sizeof(b));  ap["mask"] = b;
  JsonObject t = o["time"].to<JsonObject>();
  t["ntp1"] = cfg.ntp1;
  t["ntp2"] = cfg.ntp2;
  t["ntp3"] = cfg.ntp3;
  t["ntpTimeoutS"] = cfg.ntpTimeoutS;
  t["ntpIntervalS"] = cfg.ntpIntervalS;
  t["tzMode"] = cfg.tzMode;
  t["tzPreset"] = cfg.tzPreset;
  t["gmtOffS"] = cfg.gmtOffS;
  t["dstMode"] = cfg.dstMode;
  t["dstShiftMin"] = cfg.dstShiftMin;
  JsonObject ds = t["ds"].to<JsonObject>();
  ds["mon"] = cfg.dsMon; ds["week"] = cfg.dsWeek; ds["dow"] = cfg.dsDow;
  ds["hour"] = cfg.dsHour; ds["min"] = cfg.dsMin;
  JsonObject de = t["de"].to<JsonObject>();
  de["mon"] = cfg.deMon; de["week"] = cfg.deWeek; de["dow"] = cfg.deDow;
  de["hour"] = cfg.deHour; de["min"] = cfg.deMin;
  t["driftPerDay"] = cfg.driftPerDay;
  t["rtcEnabled"] = cfg.rtcEnabled;
  t["rtcResyncMin"] = cfg.rtcResyncMin;
  JsonObject ms = o["misc"].to<JsonObject>();
  ms["globalActiveHigh"] = cfg.globalActiveHigh;
}

void relayToJson(uint8_t i, JsonObject dst) {
  dst["i"] = i;
  dst["n"] = rcfg[i].name;
  dst["pol"] = rcfg[i].polarity;
  dst["ovr"] = rcfg[i].overrideMode;
  dst["se"] = rcfg[i].schedulesEnabled;
  JsonArray arr = dst["pairs"].to<JsonArray>();
  for (int p = 0; p < N_PAIRS; ++p) {
    JsonObject po = arr.add<JsonObject>();
    pairToJson(&rcfg[i].pairs[p], po);
  }
}

bool parseRelayCfg(JsonObject src, uint8_t i) {
  if (src.isNull()) return false;
  RelayCfg nc;
  memset(&nc, 0, sizeof(nc));
  const char *name = src["n"].isNull() ? NULL : src["n"].as<const char *>();
  if (name && strlen(name) > 23) return false;
  sstrlcpy(nc.name, name ? name : "Relay", sizeof(nc.name));
  if (!src["pol"].isNull()) {
    int v = src["pol"].as<int>();
    if (v < 0 || v > 2) return false;
    nc.polarity = (uint8_t)v;
  }
  if (!src["ovr"].isNull()) {
    int v = src["ovr"].as<int>();
    if (v < 0 || v > 2) return false;
    nc.overrideMode = (uint8_t)v;
  }
  if (!src["se"].isNull()) {
    int v = src["se"].as<int>();
    if (v != 0 && v != 1) return false;
    nc.schedulesEnabled = (uint8_t)v;
  } else nc.schedulesEnabled = 1;
  nc.rsv = 0;
  for (int p = 0; p < N_PAIRS; ++p) {
    nc.pairs[p].enabled = 0;
    nc.pairs[p].rsv = 0;
    uint64_t full60 = 0, full24 = 0, full31 = 0, full12 = 0, full7 = 0;
    for (int v = 0; v < 60; ++v) full60 |= (1ULL << v);
    for (int v = 0; v < 24; ++v) full24 |= (1ULL << v);
    for (int v = 1; v <= 31; ++v) full31 |= (1ULL << v);
    for (int v = 1; v <= 12; ++v) full12 |= (1ULL << v);
    for (int v = 0; v < 7; ++v) full7 |= (1ULL << v);
    nc.pairs[p].start = TimePattern{full60, full60, (uint32_t)full24, (uint32_t)full31, (uint16_t)full12, (uint8_t)full7, 0};
    nc.pairs[p].stop  = nc.pairs[p].start;
  }
  if (!src["pairs"].isNull() && src["pairs"].is<JsonArray>()) {
    JsonArray arr = src["pairs"].as<JsonArray>();
    size_t n = arr.size();
    if (n > (size_t)N_PAIRS) return false;
    for (size_t p = 0; p < n; ++p) {
      if (!parsePairObj(arr[p].as<JsonObject>(), &nc.pairs[p])) return false;
    }
  }
  rcfg[i] = nc;
  return true;
}

void handleApiConfigExport() {
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=rm16-config.json");
  server.send(200, "application/json", "");
  String buf;
  buf += "{\"fw\":\"";
  buf += FW_VERSION;
  buf += "\",\"sys\":";
  {
    JsonDocument doc;
    JsonObject o = doc.to<JsonObject>();
    sysCfgToJson(o);
    String js;
    serializeJson(doc, js);
    buf += js;
  }
  buf += ",\"relays\":[";
  server.sendContent(buf);
  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    buf = "";
    {
      JsonDocument doc;
      JsonObject o = doc.to<JsonObject>();
      relayToJson(i, o);
      String js;
      serializeJson(doc, js);
      buf += js;
    }
    if (i < N_RELAYS - 1) buf += ",";
    server.sendContent(buf);
  }
  server.sendContent("]}");
  server.sendContent("");
}

void handleApiConfigSys() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX)) return;
  JsonObject so = doc.as<JsonObject>();
  if (so.isNull()) { jErr(400, "invalid body"); return; }
  bool ok = true;
  SysCfg backup = cfg;
  if (!so["sta"].isNull() && so["sta"].is<JsonObject>())
    ok = ok && parseStaCfg(so["sta"].as<JsonObject>());
  if (!so["ap"].isNull() && so["ap"].is<JsonObject>())
    ok = ok && parseApCfg(so["ap"].as<JsonObject>());
  if (!so["time"].isNull() && so["time"].is<JsonObject>())
    ok = ok && parseTimeCfg(so["time"].as<JsonObject>());
  if (!so["misc"].isNull() && so["misc"].is<JsonObject>())
    ok = ok && parseMiscCfg(so["misc"].as<JsonObject>());
  if (!ok) { cfg = backup; jErr(400, "invalid config value(s)"); return; }
  markSysDirty();
  tzCompute(nowUtc());
  recomputeAll();
  staApplyLive();
  if (apActive) { stopAp(); apLastChangeUs = 0; }
  apEvaluate();
  ntpKick();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void handleApiConfigRelay() {
  JsonDocument doc;
  if (!getBodyJson(&doc, BODY_MAX_RELAY)) return;
  JsonObject o = doc.as<JsonObject>();
  if (o.isNull() || o["i"].isNull()) { jErr(400, "missing i"); return; }
  int i = o["i"].as<int>();
  if (i < 0 || i >= N_RELAYS) { jErr(400, "i out of range"); return; }
  if (o["relay"].isNull()) { jErr(400, "missing relay"); return; }
  if (!parseRelayCfg(o["relay"].as<JsonObject>(), (uint8_t)i)) {
    jErr(400, "invalid relay config");
    return;
  }
  markRlyDirty((uint8_t)i);
  recomputeOne((uint8_t)i);
  applyOutputs();
  JsonDocument r;
  r["ok"] = 1;
  sendDoc(200, &r);
}

void pendingReboot(const char *reason, int32_t delayMs) {
  sstrlcpy(rebootReason, reason ? reason : "", sizeof(rebootReason));
  rebootAtUs = esp_timer_get_time() + (int64_t)delayMs * 1000LL;
}

void maybeReboot() {
  if (rebootAtUs && esp_timer_get_time() >= rebootAtUs) esp_restart();
}

void setup() {
  loadAllCfg();
  cfg.bootCount++;
  markSysDirty();

  for (uint8_t i = 0; i < N_RELAYS; ++i) {
    desiredOn[i] = false;
    digitalWrite(RELAY_PINS[i], relayOutputLevel(i) ? HIGH : LOW);
    pinMode(RELAY_PINS[i], OUTPUT);
  }
  pinMode(PIN_RESET_BTN, INPUT_PULLUP);

  rtcInit();
  if (timeSrc != TSRC_RTC) {
    int64_t ce = compileEpoch();
    if (ce >= 1262304000LL) setSystemTime(ce, TSRC_BOOT, "Build", false);
    else                    setSystemTime(1262304000LL, TSRC_BOOT, "Build", false);
  }

  tzCompute(nowUtc());
  recomputeAll();

  wifiInit();
  webSetup();
  ntpInit();
}

void loop() {
  curUtcSec = nowUtc();
  server.handleClient();
  if (dnsRunning) dnsServer.processNextRequest();
  wifiTask();
  ntpTask();
  rtcTask();
  nvsTask();
  btnTask();
  scanPoll();
  schedTick();
  maybeReboot();
  delay(1);
}
