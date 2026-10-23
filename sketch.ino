#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <esp_partition.h>
#include <nvs.h>
#include <lwip/dns.h>
#include <time.h>

#if defined(ARDUINOJSON_VERSION_MAJOR) && (ARDUINOJSON_VERSION_MAJOR >= 7)
class RhJsonDoc : public JsonDocument {
 public:
  explicit RhJsonDoc(size_t capacity = 0) { (void)capacity; }
};
#else
class RhJsonDoc : public DynamicJsonDocument {
 public:
  explicit RhJsonDoc(size_t capacity) : DynamicJsonDocument(capacity) {}
};
#endif
#include <sys/time.h>
#include <stdarg.h>
#include <math.h>

#define FW_NAME              "ESP32 RelayHub"
#define FW_VERSION           "1.0.0"

#define NRELAYS              16
#define NSCHED               8
#define NTP_MAX              4

#define RELAY_NAME_LEN       24
#define SSID_LEN             32
#define PASS_LEN             64
#define HOST_LEN             32
#define NTP_HOST_LEN         64
#define POSIX_LEN            48
#define DEVNAME_LEN          32
#define AUSER_LEN            24
#define APASS_LEN            32

#define CFG_MAGIC            0x524C5948UL
#define SYS_CFG_VERSION      5
#define RELAY_CFG_VERSION    4

#define LOG_LINES            32
#define LOG_LEN              112

#define LOG_ERR              0
#define LOG_WARN             1
#define LOG_INFO             2
#define LOG_DBG              3

#define SCH_F_OVERNIGHT      0x01
#define SCH_F_END_INCL       0x02
#define SCH_F_TAIL           0x04
#define SCH_F_VALID_FROM     0x08
#define SCH_F_VALID_TO       0x10

#define COMB_AND             0
#define COMB_OR              1

#define LEAP_ANY             0
#define LEAP_ONLY            1
#define LEAP_NONLEAP         2

#define OVR_AUTO             0
#define OVR_FORCE_HIGH       1
#define OVR_FORCE_LOW        2

#define TSRC_NONE            0
#define TSRC_RTC             1
#define TSRC_NTP             2
#define TSRC_BROWSER         3

#define DST_NONE             0
#define DST_US               1
#define DST_EU               2
#define DST_AU               3
#define DST_NZ               4
#define DST_CUSTOM           5

#define TZ_MODE_MANUAL       0
#define TZ_MODE_AUTO         1

struct Schedule {
  uint8_t  enabled;
  uint8_t  flags;
  uint8_t  startH, startM, startS;
  uint8_t  stopH, stopM, stopS;
  uint32_t tailSec;
  uint16_t monthMask;
  uint32_t domMask;
  uint8_t  dowMask;
  uint8_t  combMode;
  uint8_t  leapMode;
  uint8_t  vfY, vfM, vfD;
  uint8_t  vtY, vtM, vtD;
  uint8_t  reserved[2];
};

struct DstRule {
  uint8_t enabled;
  uint8_t startMonth;
  uint8_t startWeek;
  uint8_t startDow;
  uint8_t startHour;
  uint8_t endMonth;
  uint8_t endWeek;
  uint8_t endDow;
  uint8_t endHour;
  int32_t dstSec;
};

struct TimeCtx {
  int32_t (*offsetAt)(const void *ctx, int64_t utcSec);
  const void *ctx;
};

struct LocalDT {
  int32_t  y;
  uint8_t  mo, d, hh, mi, ss, dow;
  uint16_t doy;
};

int64_t  floorDiv64(int64_t a, int64_t b);
int64_t  floorMod64(int64_t a, int64_t b);
bool     isLeapYear(int y);
int      daysInMonthOf(int y, int m);
int64_t  daysFromCivil(int y, int m, int d);
void     civilFromDays(int64_t z, int &yy, int &mm, int &dd);
int      dayOfWeekFromDays(int64_t days);
LocalDT  localDtFromEpoch(int64_t localEpoch);
int64_t  civilToEpoch(int y, int m, int d, int hh, int mi, int ss);
int64_t  localToUtc(int64_t localEpoch, const TimeCtx &ctx);
int32_t  dstEffectiveOffset(const DstRule &r, int32_t stdOff, int64_t utc);
int      dstNthWeekday(int year, int month, int week, int dow);
int64_t  dstRuleStartUtc(const DstRule &r, int year, int32_t stdOff);
int64_t  dstRuleEndUtc(const DstRule &r, int year, int32_t stdOff);
bool     dstRuleActiveAt(const DstRule &r, int32_t stdOff, int64_t utc);
bool     schedInValidity(const Schedule &s, int y, int m, int d);
bool     schedDateMatch(const Schedule &s, int y, int m, int d, int dow);
bool     schedWindowUtc(const Schedule &s, int y, int m, int d,
                        int64_t &startUtc, int64_t &endUtc, const TimeCtx &ctx);
bool     scheduleActiveAt(const Schedule &s, int64_t utc, const TimeCtx &ctx);
bool     anyScheduleActive(const Schedule *list, int n, int64_t utc, const TimeCtx &ctx);

int64_t floorDiv64(int64_t a, int64_t b) {
  int64_t q = a / b;
  int64_t r = a % b;
  if (r != 0 && ((r < 0) != (b < 0))) q--;
  return q;
}

int64_t floorMod64(int64_t a, int64_t b) {
  int64_t r = a % b;
  if (r != 0 && ((r < 0) != (b < 0))) r += b;
  return r;
}

bool isLeapYear(int y) {
  return ((y % 4) == 0 && (y % 100) != 0) || ((y % 400) == 0);
}

int daysInMonthOf(int y, int m) {
  static const uint8_t dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m < 1) m = 1;
  if (m > 12) m = 12;
  int d = dm[m - 1];
  if (m == 2 && isLeapYear(y)) d = 29;
  return d;
}

int64_t daysFromCivil(int y, int m, int d) {
  y -= (m <= 2) ? 1 : 0;
  const int64_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - (int)(era * 400));
  const unsigned doy = (unsigned)((153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + (d - 1));
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

void civilFromDays(int64_t z, int &yy, int &mm, int &dd) {
  z += 719468;
  const int64_t era = (z >= 0 ? z : z - 146096) / 146097;
  const unsigned doe = (unsigned)(z - era * 146097);
  const unsigned yoe = (doe - doe / 1460u + doe / 36524u - doe / 146096u) / 365u;
  const int64_t y = (int64_t)yoe + era * 400;
  const unsigned doy = doe - (365u * yoe + yoe / 4u - yoe / 100u);
  const unsigned mp = (5u * doy + 2u) / 153u;
  const unsigned d = doy - (153u * mp + 2u) / 5u + 1u;
  const unsigned m = (mp < 10u) ? (mp + 3u) : (mp - 9u);
  yy = (int)(y + ((m <= 2u) ? 1 : 0));
  mm = (int)m;
  dd = (int)d;
}

int dayOfWeekFromDays(int64_t days) {
  return (int)floorMod64(days + 4, 7);
}

int64_t civilToEpoch(int y, int m, int d, int hh, int mi, int ss) {
  if (m < 1) m = 1;
  if (m > 12) m = 12;
  int dim = daysInMonthOf(y, m);
  if (d < 1) d = 1;
  if (d > dim) d = dim;
  if (hh < 0) hh = 0;
  if (hh > 23) hh = 23;
  if (mi < 0) mi = 0;
  if (mi > 59) mi = 59;
  if (ss < 0) ss = 0;
  if (ss > 59) ss = 59;
  return daysFromCivil(y, m, d) * 86400 + (int64_t)hh * 3600 + (int64_t)mi * 60 + ss;
}

LocalDT localDtFromEpoch(int64_t localEpoch) {
  LocalDT t;
  int64_t days = floorDiv64(localEpoch, 86400);
  int64_t sod  = localEpoch - days * 86400;
  int y = 1970, m = 1, d = 1;
  civilFromDays(days, y, m, d);
  t.y   = (int32_t)y;
  t.mo  = (uint8_t)m;
  t.d   = (uint8_t)d;
  t.hh  = (uint8_t)(sod / 3600);
  t.mi  = (uint8_t)((sod % 3600) / 60);
  t.ss  = (uint8_t)(sod % 60);
  t.dow = (uint8_t)dayOfWeekFromDays(days);
  t.doy = (uint16_t)(days - daysFromCivil(y, 1, 1) + 1);
  return t;
}

int64_t localToUtc(int64_t localEpoch, const TimeCtx &ctx) {
  if (ctx.offsetAt == nullptr) return localEpoch;
  int64_t g = localEpoch - ctx.offsetAt(ctx.ctx, localEpoch);
  g = localEpoch - ctx.offsetAt(ctx.ctx, g);
  g = localEpoch - ctx.offsetAt(ctx.ctx, g);
  return g;
}

int dstNthWeekday(int year, int month, int week, int dow) {
  if (month < 1 || month > 12) return 0;
  if (dow < 0) dow = 0;
  if (dow > 6) dow = 6;
  int dim = daysInMonthOf(year, month);
  if (week >= 4) {
    int64_t dLast = daysFromCivil(year, month, dim);
    int wd = dayOfWeekFromDays(dLast);
    int back = (wd - dow + 7) % 7;
    return dim - back;
  }
  int64_t dFirst = daysFromCivil(year, month, 1);
  int wd = dayOfWeekFromDays(dFirst);
  int fwd = (dow - wd + 7) % 7;
  int day = 1 + fwd + week * 7;
  if (day > dim) return 0;
  return day;
}

int64_t dstRuleStartUtc(const DstRule &r, int year, int32_t stdOff) {
  int d = dstNthWeekday(year, r.startMonth, r.startWeek, r.startDow);
  if (d <= 0) return INT64_MIN;
  return civilToEpoch(year, r.startMonth, d, r.startHour, 0, 0) - stdOff;
}

int64_t dstRuleEndUtc(const DstRule &r, int year, int32_t stdOff) {
  int d = dstNthWeekday(year, r.endMonth, r.endWeek, r.endDow);
  if (d <= 0) return INT64_MIN;
  return civilToEpoch(year, r.endMonth, d, r.endHour, 0, 0) - stdOff - r.dstSec;
}

bool dstRuleActiveAt(const DstRule &r, int32_t stdOff, int64_t utc) {
  if (!r.enabled) return false;
  LocalDT lt = localDtFromEpoch(utc + stdOff);
  int y = (int)lt.y;
  const bool southern = (r.startMonth > r.endMonth);
  for (int k = -1; k <= 1; k++) {
    int yy = y + k;
    int64_t s = dstRuleStartUtc(r, yy, stdOff);
    int64_t e = southern ? dstRuleEndUtc(r, yy + 1, stdOff)
                         : dstRuleEndUtc(r, yy, stdOff);
    if (s == INT64_MIN || e == INT64_MIN) continue;
    if (utc >= s && utc < e) return true;
  }
  return false;
}

int32_t dstEffectiveOffset(const DstRule &r, int32_t stdOff, int64_t utc) {
  if (!r.enabled || r.dstSec == 0) return stdOff;
  return stdOff + (dstRuleActiveAt(r, stdOff, utc) ? r.dstSec : 0);
}

bool schedInValidity(const Schedule &s, int y, int m, int d) {
  int64_t day = daysFromCivil(y, m, d);
  if (s.flags & SCH_F_VALID_FROM) {
    int y0 = 2000 + s.vfY;
    int m0 = (s.vfM >= 1 && s.vfM <= 12) ? s.vfM : 1;
    int d0 = (s.vfD >= 1 && s.vfD <= daysInMonthOf(y0, m0)) ? s.vfD : 1;
    if (day < daysFromCivil(y0, m0, d0)) return false;
  }
  if (s.flags & SCH_F_VALID_TO) {
    int y1 = 2000 + s.vtY;
    int m1 = (s.vtM >= 1 && s.vtM <= 12) ? s.vtM : 12;
    int d1 = (s.vtD >= 1 && s.vtD <= daysInMonthOf(y1, m1)) ? s.vtD : daysInMonthOf(y1, m1);
    if (day > daysFromCivil(y1, m1, d1)) return false;
  }
  return true;
}

bool schedDateMatch(const Schedule &s, int y, int m, int d, int dow) {
  if (!schedInValidity(s, y, m, d)) return false;

  const bool andMode = (s.combMode == COMB_AND);
  bool any = false;
  bool ok  = false;

  if (s.monthMask != 0) {
    any = true;
    bool hit = ((s.monthMask >> (m - 1)) & 1u) != 0;
    if (andMode) { if (!hit) return false; } else { ok = ok || hit; }
  }
  if (s.domMask != 0) {
    any = true;
    bool hit = ((s.domMask >> (d - 1)) & 1u) != 0;
    if (andMode) { if (!hit) return false; } else { ok = ok || hit; }
  }
  if (s.dowMask != 0) {
    any = true;
    bool hit = ((s.dowMask >> dow) & 1u) != 0;
    if (andMode) { if (!hit) return false; } else { ok = ok || hit; }
  }
  if (s.leapMode == LEAP_ONLY || s.leapMode == LEAP_NONLEAP) {
    any = true;
    bool leap = isLeapYear(y);
    bool hit  = (s.leapMode == LEAP_ONLY) ? leap : !leap;
    if (andMode) { if (!hit) return false; } else { ok = ok || hit; }
  }
  if (!any) return true;
  return andMode ? true : ok;
}

bool schedWindowUtc(const Schedule &s, int y, int m, int d,
                    int64_t &startUtc, int64_t &endUtc, const TimeCtx &ctx) {
  int startTot = (int)s.startH * 3600 + (int)s.startM * 60 + (int)s.startS;
  int stopTot  = (int)s.stopH  * 3600 + (int)s.stopM  * 60 + (int)s.stopS;

  int64_t startLocal = civilToEpoch(y, m, d, s.startH, s.startM, s.startS);
  int addDays = (stopTot <= startTot) ? 1 : 0;

  int ey = y, em = m, ed = d;
  if (addDays) civilFromDays(daysFromCivil(y, m, d) + addDays, ey, em, ed);

  int64_t endLocal = civilToEpoch(ey, em, ed, s.stopH, s.stopM, s.stopS);
  if (endLocal - startLocal > 86400) endLocal = startLocal + 86400;

  startUtc = localToUtc(startLocal, ctx);
  endUtc   = localToUtc(endLocal, ctx);

  if (endUtc == startUtc && endLocal > startLocal) return false;

  if (s.flags & SCH_F_END_INCL) endUtc += 1;
  if ((s.flags & SCH_F_TAIL) && s.tailSec > 0) endUtc += (int64_t)s.tailSec;

  if (endUtc <= startUtc) endUtc = startUtc + 1;
  return true;
}

bool scheduleActiveAt(const Schedule &s, int64_t utc, const TimeCtx &ctx) {
  if (s.enabled == 0) return false;

  int32_t offNow = (ctx.offsetAt != nullptr) ? ctx.offsetAt(ctx.ctx, utc) : 0;
  LocalDT now = localDtFromEpoch(utc + offNow);
  int64_t todayDays = daysFromCivil(now.y, now.mo, now.d);

  for (int k = 0; k < 3; k++) {
    int64_t dd = todayDays - k;
    int y, m, d;
    civilFromDays(dd, y, m, d);
    if (!schedDateMatch(s, y, m, d, dayOfWeekFromDays(dd))) continue;
    int64_t su = 0, eu = 0;
    if (!schedWindowUtc(s, y, m, d, su, eu, ctx)) continue;
    if (utc >= su && utc < eu) return true;
  }
  return false;
}

bool anyScheduleActive(const Schedule *list, int n, int64_t utc, const TimeCtx &ctx) {
  if (list == nullptr) return false;
  for (int i = 0; i < n; i++) {
    if (scheduleActiveAt(list[i], utc, ctx)) return true;
  }
  return false;
}

struct NetStaCfg {
  char     hostname[HOST_LEN];
  char     ssid[SSID_LEN + 1];
  char     pass[PASS_LEN + 1];
  char     ssid2[SSID_LEN + 1];
  char     pass2[PASS_LEN + 1];
  char     ip[16];
  char     gw[16];
  char     sn[16];
  char     dns1[16];
  char     dns2[16];
  uint8_t  enabled;
  uint8_t  dhcp;
  uint8_t  autoReconnect;
  uint8_t  powerSave;
  uint16_t reconnectSec;
  uint8_t  reserved[2];
};

struct NetApCfg {
  char     ssid[SSID_LEN + 1];
  char     pass[PASS_LEN + 1];
  uint8_t  enabled;
  uint8_t  hidden;
  uint8_t  captive;
  uint8_t  fallback;
  uint8_t  autoOff;
  uint8_t  channel;
  uint8_t  maxConn;
  uint8_t  reserved;
  uint16_t fallbackSec;
  uint16_t reserved2;
};

struct TimeCfg {
  uint8_t  tzMode;
  int32_t  tzOffsetSec;
  uint8_t  dstMode;
  int32_t  dstOffsetSec;
  uint8_t  dstStartMonth;
  uint8_t  dstStartWeek;
  uint8_t  dstStartDow;
  uint8_t  dstStartHour;
  uint8_t  dstEndMonth;
  uint8_t  dstEndWeek;
  uint8_t  dstEndDow;
  uint8_t  dstEndHour;
  uint8_t  autoZone;
  char     posix[POSIX_LEN];
  uint8_t  ntpEnabled;
  uint8_t  ntpCount;
  char     ntp[NTP_MAX][NTP_HOST_LEN];
  uint16_t syncIntervalMin;
  uint8_t  useRtc;
  uint8_t  allowBrowserSync;
  int16_t  driftPpm100;
  uint8_t  syncOnReconnect;
  uint8_t  reserved[3];
};

struct UiCfg {
  char     deviceName[DEVNAME_LEN];
  char     adminUser[AUSER_LEN];
  char     adminPass[APASS_LEN];
  uint8_t  authEnabled;
  uint8_t  logLevel;
  uint16_t pollMs;
  uint8_t  reserved[2];
};

struct SysCfg {
  NetStaCfg sta;
  NetApCfg  ap;
  TimeCfg   time;
  UiCfg     ui;
};

struct RelayCfg {
  char     name[RELAY_NAME_LEN + 1];
  uint8_t  enabled;
  uint8_t  activeLow;
  uint8_t  override;
  uint8_t  reserved;
  Schedule sched[NSCHED];
};

struct RelayRt {
  bool     logicalOn;
  uint8_t  pinLevel;
  uint8_t  forced;
  uint32_t changes;
  uint32_t lastChangeMs;
  int64_t  lastChangeEpoch;
  uint8_t  activeRules;
};

struct StoreStat {
  uint32_t writes;
  uint32_t errors;
  uint32_t lastWriteMs;
  uint32_t seqSys;
  uint32_t seqRelay[NRELAYS];
};

struct NetRt {
  uint8_t  staPhase;
  uint8_t  staProfile;
  uint32_t lastAttemptMs;
  uint32_t connectStartMs;
  uint32_t attempts;
  uint32_t gotIpMs;
  uint32_t lastDisconnectMs;
  bool     staConnected;
  bool     apActive;
  bool     fallbackActive;
  bool     mdnsActive;
  uint32_t lastRtcResyncMs;
  int8_t   lastRssi;
  char     ip[16];
};

struct TimeRt {
  bool     valid;
  bool     rtcOk;
  bool     rtcLostPower;
  uint8_t  source;
  int64_t  lastSyncUtc;
  uint32_t lastSyncMs;
  uint32_t lastSyncTryMs;
  uint32_t syncOk;
  uint32_t syncFail;
  uint32_t lastRtcReadMs;
  int8_t   lastNtpServer;
  int8_t   lastNtpResult;
  uint32_t driftAccUs;
};

struct NtpState {
  uint8_t  phase;
  uint8_t  serverIdx;
  uint8_t  tries;
  uint32_t phaseStartMs;
  uint32_t nextSyncMs;
  uint32_t reqId;
  ip_addr_t addr;
  bool     dnsDone;
  bool     dnsOk;
} ;

struct LogEntry {
  uint32_t ms;
  int64_t  epoch;
  uint8_t  level;
  uint8_t  repeat;
  char     msg[LOG_LEN];
};

void     appSetup();
void     appLoop();

void     rhLog(uint8_t level, const char *fmt, ...);
void     logSetup();
const char *levelName(uint8_t lvl);

void     cfgDefaults();
void     cfgLoadAll();
bool     cfgLoadSys();
bool     cfgLoadRelay(int idx);
void     cfgSanitizeSys();
void     cfgSanitizeRelay(RelayCfg &r, int idx);
bool     cfgValidateName(const char *name);
void     cfgMarkSysDirty();
void     cfgMarkRelayDirty(int idx);
void     cfgMarkAllDirty();
void     cfgService();
void     cfgFlushNow();
bool     cfgPending();
void     cfgFactoryReset();
uint32_t crc32Of(const uint8_t *data, size_t len);
bool     nvsWriteRecord(const char *key, uint16_t version, const void *body, size_t bodySize, uint32_t &seq);
bool     nvsReadRecord(const char *key, uint16_t version, void *body, size_t bodySize, uint32_t &seq);
void     sysKeyForSeq(uint32_t seq, char *out, size_t cap);
void     relayKeyForSeq(int idx, uint32_t seq, char *out, size_t cap);
void     cfgDefaultsSys();
void     cfgDefaultsRelay(int idx);
void     statsFill();

SysCfg   gCfg;
RelayCfg gRelay[NRELAYS];
RelayRt  gRt[NRELAYS];
StoreStat gStore;
NetRt    gNet;
TimeRt   gTimeRt;
NtpState gNtp;

Preferences gPrefs;
const char *kPrefsNs = "relayhub";

const uint8_t kRelayPins[NRELAYS] = {
  23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19, 18, 5, 4, 2, 15
};

const uint8_t kRtcSda = 21;
const uint8_t kRtcScl = 22;
const uint8_t kBootPin = 0;

struct TzZone {
  const char *name;
  const char *posix;
};
const TzZone kTzZones[] = {
  { "UTC",                 "UTC0" },
  { "Asia/Manila",         "PHT-8" },
  { "Asia/Singapore",      "<+08>-8" },
  { "Asia/Tokyo",          "JST-9" },
  { "Asia/Shanghai",       "CST-8" },
  { "Asia/Kolkata",        "IST-5:30" },
  { "Asia/Dubai",          "<+04>-4" },
  { "Europe/London",       "GMT0BST,M3.5.0/1,M10.5.0" },
  { "Europe/Berlin",       "CET-1CEST,M3.5.0,M10.5.0/3" },
  { "Europe/Athens",       "EET-2EEST,M3.5.0/3,M10.5.0/4" },
  { "America/New_York",    "EST5EDT,M3.2.0,M11.1.0" },
  { "America/Chicago",     "CST6CDT,M3.2.0,M11.1.0" },
  { "America/Denver",      "MST7MDT,M3.2.0,M11.1.0" },
  { "America/Los_Angeles", "PST8PDT,M3.2.0,M11.1.0" },
  { "America/Sao_Paulo",   "<-03>3" },
  { "Australia/Sydney",    "AEST-10AEDT,M10.1.0,M4.1.0/3" },
  { "Australia/Perth",     "AWST-8" },
  { "Pacific/Auckland",    "NZST-12NZDT,M9.5.0,M4.1.0/3" },
  { "Africa/Johannesburg", "SAST-2" },
  { "Africa/Cairo",        "<+02>-2" }
};
const uint8_t kTzZoneCount = (uint8_t)(sizeof(kTzZones) / sizeof(kTzZones[0]));

static LogEntry gLogs[LOG_LINES];
static uint8_t  gLogHead = 0;
static uint8_t  gLogCount = 0;
uint8_t gLogLevelNow = LOG_INFO;
uint32_t gLoopCount = 0;
uint32_t gBootMs = 0;
uint32_t gNowEpoch = 0;

struct DirtyMgr {
  bool     sysDirty;
  uint32_t relayMask;
  uint32_t firstMs;
  uint32_t lastMs;
  bool     pending;
};
static DirtyMgr gDirty;

#define CFG_WRITE_DEBOUNCE_MS   8000UL
#define CFG_WRITE_MAXDELAY_MS   60000UL

void strCopy(char *dst, size_t cap, const char *src) {
  if (dst == nullptr || cap == 0) return;
  if (src == nullptr) { dst[0] = 0; return; }
  size_t n = strlen(src);
  if (n >= cap) n = cap - 1;
  memcpy(dst, src, n);
  dst[n] = 0;
}

bool strEmpty(const char *s) { return (s == nullptr) || (s[0] == 0); }

void clampStr(char *s, size_t cap) {
  if (s == nullptr || cap == 0) return;
  s[cap - 1] = 0;
  size_t n = strlen(s);
  if (n == 0) return;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)s[i];
    if (c < 0x20 || c == 0x7F) s[i] = '?';
  }
}

int clampI(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

uint32_t millisSince(uint32_t t0) { return (uint32_t)(millis() - t0); }

void fmtUptime(uint32_t ms, char *out, size_t cap) {
  uint32_t s = ms / 1000UL;
  uint32_t d = s / 86400UL; s %= 86400UL;
  uint32_t h = s / 3600UL;  s %= 3600UL;
  uint32_t m = s / 60UL;    s %= 60UL;
  snprintf(out, cap, "%ud %02u:%02u:%02u", (unsigned)d, (unsigned)h, (unsigned)m, (unsigned)s);
}

const char *levelName(uint8_t lvl) {
  switch (lvl) {
    case LOG_ERR:  return "ERR";
    case LOG_WARN: return "WRN";
    case LOG_INFO: return "INF";
    case LOG_DBG:  return "DBG";
    default:       return "???";
  }
}

void logSetup() {
  memset(gLogs, 0, sizeof(gLogs));
  gLogHead = 0;
  gLogCount = 0;
}

void rhLog(uint8_t level, const char *fmt, ...) {
  char buf[LOG_LEN];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  buf[sizeof(buf) - 1] = 0;

  if (gLogCount > 0) {
    uint8_t last = (uint8_t)((gLogHead + LOG_LINES - 1) % LOG_LINES);
    if (gLogs[last].level == level && strcmp(gLogs[last].msg, buf) == 0 &&
        millisSince(gLogs[last].ms) < 3000UL) {
      if (gLogs[last].repeat < 250) gLogs[last].repeat++;
      return;
    }
  }

  LogEntry &e = gLogs[gLogHead];
  e.ms = millis();
  e.epoch = (int64_t)time(nullptr);
  e.level = level;
  e.repeat = 0;
  strCopy(e.msg, sizeof(e.msg), buf);
  gLogHead = (uint8_t)((gLogHead + 1) % LOG_LINES);
  if (gLogCount < LOG_LINES) gLogCount++;

  if (level <= gLogLevelNow) {
    char up[24];
    fmtUptime(millis(), up, sizeof(up));
    Serial.printf("[%s] %s %s\r\n", up, levelName(level), buf);
  }
}

struct RecHdr {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  uint32_t seq;
  uint32_t crc;
};

uint32_t crc32Of(const uint8_t *data, size_t len) {
  uint32_t c = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    c ^= data[i];
    for (int k = 0; k < 8; k++) {
      uint32_t mask = (uint32_t)(-(int32_t)(c & 1u));
      c = (c >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~c;
}

static uint8_t gRecBuf[sizeof(RecHdr) + ((sizeof(SysCfg) > sizeof(RelayCfg))
                                         ? sizeof(SysCfg) : sizeof(RelayCfg)) + 8];

bool nvsWriteRecord(const char *key, const void *body, size_t bodySize, uint32_t &seq) {
  if (bodySize + sizeof(RecHdr) > sizeof(gRecBuf)) return false;
  uint32_t nextSeq = seq + 1;
  if (nextSeq == 0) nextSeq = 1;

  RecHdr h;
  h.magic = CFG_MAGIC;
  h.version = 0;
  h.payloadSize = (uint16_t)bodySize;
  h.seq = nextSeq;
  h.crc = crc32Of((const uint8_t *)body, bodySize);

  memcpy(gRecBuf, &h, sizeof(h));
  memcpy(gRecBuf + sizeof(h), body, bodySize);
  size_t total = sizeof(h) + bodySize;

  if (!gPrefs.begin(kPrefsNs, false)) {
    gStore.errors++;
    rhLog(LOG_ERR, "NVS open failed for %s", key);
    return false;
  }
  size_t wr = gPrefs.putBytes(key, gRecBuf, total);
  bool ok = (wr == total);
  gPrefs.end();

  if (!ok) {
    gStore.errors++;
    rhLog(LOG_ERR, "NVS write failed %s (%u/%u)", key, (unsigned)wr, (unsigned)total);
    return false;
  }

  uint8_t chk[sizeof(RecHdr)];
  if (gPrefs.begin(kPrefsNs, true)) {
    size_t rd = gPrefs.getBytes(key, chk, sizeof(chk));
    gPrefs.end();
    RecHdr *h2 = (RecHdr *)chk;
    if (rd != sizeof(RecHdr) || h2->seq != nextSeq || h2->crc != h.crc) {
      gStore.errors++;
      rhLog(LOG_ERR, "NVS verify failed %s", key);
      return false;
    }
  }
  seq = nextSeq;
  gStore.writes++;
  gStore.lastWriteMs = millis();
  return true;
}

bool nvsReadRecord(const char *key, void *body, size_t bodySize, uint32_t &seq) {
  RecHdr h;
  if (!gPrefs.begin(kPrefsNs, true)) return false;
  size_t rd = gPrefs.getBytes(key, &h, sizeof(h));
  if (rd != sizeof(h)) { gPrefs.end(); return false; }
  if (h.magic != CFG_MAGIC || h.payloadSize != bodySize) { gPrefs.end(); return false; }
  if (h.payloadSize + sizeof(RecHdr) > sizeof(gRecBuf)) { gPrefs.end(); return false; }
  size_t rd2 = gPrefs.getBytes(key, gRecBuf, sizeof(RecHdr) + bodySize);
  gPrefs.end();
  if (rd2 != sizeof(RecHdr) + bodySize) return false;
  RecHdr *h2 = (RecHdr *)gRecBuf;
  if (h2->magic != CFG_MAGIC || h2->payloadSize != bodySize) return false;
  uint32_t crc = crc32Of(gRecBuf + sizeof(RecHdr), bodySize);
  if (crc != h2->crc) return false;
  memcpy(body, gRecBuf + sizeof(RecHdr), bodySize);
  seq = h2->seq;
  return true;
}

bool nvsWriteRecord(const char *key, uint16_t version, const void *body,
                    size_t bodySize, uint32_t &seq) {
  if (bodySize + sizeof(RecHdr) > sizeof(gRecBuf)) return false;
  uint32_t nextSeq = seq + 1;
  if (nextSeq == 0) nextSeq = 1;

  RecHdr h;
  h.magic = CFG_MAGIC;
  h.version = version;
  h.payloadSize = (uint16_t)bodySize;
  h.seq = nextSeq;
  h.crc = crc32Of((const uint8_t *)body, bodySize);

  memcpy(gRecBuf, &h, sizeof(h));
  memcpy(gRecBuf + sizeof(h), body, bodySize);
  size_t total = sizeof(h) + bodySize;

  if (!gPrefs.begin(kPrefsNs, false)) {
    gStore.errors++;
    rhLog(LOG_ERR, "NVS open failed for %s", key);
    return false;
  }
  size_t wr = gPrefs.putBytes(key, gRecBuf, total);
  gPrefs.end();

  if (wr != total) {
    gStore.errors++;
    rhLog(LOG_ERR, "NVS write failed %s (%u/%u)", key, (unsigned)wr, (unsigned)total);
    return false;
  }

  uint8_t chk[sizeof(RecHdr)];
  memset(chk, 0, sizeof(chk));
  if (gPrefs.begin(kPrefsNs, true)) {
    size_t rd = gPrefs.getBytes(key, chk, sizeof(chk));
    gPrefs.end();
    RecHdr *h2 = (RecHdr *)chk;
    if (rd != sizeof(RecHdr) || h2->seq != nextSeq || h2->crc != h.crc ||
        h2->magic != CFG_MAGIC) {
      gStore.errors++;
      rhLog(LOG_ERR, "NVS verify failed %s", key);
      return false;
    }
  }
  seq = nextSeq;
  gStore.writes++;
  gStore.lastWriteMs = millis();
  return true;
}

bool nvsReadRecord(const char *key, uint16_t version, void *body,
                   size_t bodySize, uint32_t &seq) {
  RecHdr h;
  memset(&h, 0, sizeof(h));
  if (!gPrefs.begin(kPrefsNs, true)) return false;
  size_t rd = gPrefs.getBytes(key, &h, sizeof(h));
  if (rd != sizeof(h) || h.magic != CFG_MAGIC ||
      h.version != version || h.payloadSize != bodySize) {
    gPrefs.end();
    return false;
  }
  size_t total = sizeof(RecHdr) + bodySize;
  if (total > sizeof(gRecBuf)) { gPrefs.end(); return false; }
  size_t rd2 = gPrefs.getBytes(key, gRecBuf, total);
  gPrefs.end();
  if (rd2 != total) return false;
  RecHdr *h2 = (RecHdr *)gRecBuf;
  if (h2->magic != CFG_MAGIC || h2->version != version ||
      h2->payloadSize != bodySize) return false;
  uint32_t crc = crc32Of(gRecBuf + sizeof(RecHdr), bodySize);
  if (crc != h2->crc) return false;
  memcpy(body, gRecBuf + sizeof(RecHdr), bodySize);
  seq = h2->seq;
  return true;
}

void sysKeyForSeq(uint32_t seq, char *out, size_t cap) {
  snprintf(out, cap, "s%c", ((seq & 1u) != 0) ? 'B' : 'A');
}

void relayKeyForSeq(int idx, uint32_t seq, char *out, size_t cap) {
  snprintf(out, cap, "r%d%c", idx, ((seq & 1u) != 0) ? 'B' : 'A');
}

void strDefault(char *dst, size_t cap, const char *src) {
  memset(dst, 0, cap);
  strCopy(dst, cap, src);
}

void cfgDefaultsSys() {
  memset(&gCfg, 0, sizeof(gCfg));

  uint64_t mac = ESP.getEfuseMac();
  char apSsid[SSID_LEN + 1];
  snprintf(apSsid, sizeof(apSsid), "RelayHub-%04X", (unsigned)(mac & 0xFFFF));

  strDefault(gCfg.sta.hostname, sizeof(gCfg.sta.hostname), "relayhub");
  gCfg.sta.enabled = 0;
  gCfg.sta.dhcp = 1;
  gCfg.sta.autoReconnect = 1;
  gCfg.sta.powerSave = 0;
  gCfg.sta.reconnectSec = 1;

  strCopy(gCfg.ap.ssid, sizeof(gCfg.ap.ssid), apSsid);
  strDefault(gCfg.ap.pass, sizeof(gCfg.ap.pass), "relayhub123");
  gCfg.ap.enabled = 1;
  gCfg.ap.captive = 1;
  gCfg.ap.fallback = 1;
  gCfg.ap.channel = 6;
  gCfg.ap.maxConn = 4;
  gCfg.ap.fallbackSec = 60;

  gCfg.time.tzMode = TZ_MODE_MANUAL;
  gCfg.time.tzOffsetSec = 8 * 3600;
  gCfg.time.dstMode = DST_NONE;
  gCfg.time.dstOffsetSec = 3600;
  gCfg.time.dstStartMonth = 3; gCfg.time.dstStartWeek = 1;
  gCfg.time.dstStartDow = 0;   gCfg.time.dstStartHour = 2;
  gCfg.time.dstEndMonth = 11;  gCfg.time.dstEndWeek = 0;
  gCfg.time.dstEndDow = 0;     gCfg.time.dstEndHour = 2;
  gCfg.time.autoZone = 1;
  strDefault(gCfg.time.posix, sizeof(gCfg.time.posix), "PHT-8");
  gCfg.time.ntpEnabled = 1;
  gCfg.time.ntpCount = 3;
  strDefault(gCfg.time.ntp[0], NTP_HOST_LEN, "pool.ntp.org");
  strDefault(gCfg.time.ntp[1], NTP_HOST_LEN, "time.google.com");
  strDefault(gCfg.time.ntp[2], NTP_HOST_LEN, "time.cloudflare.com");
  strDefault(gCfg.time.ntp[3], NTP_HOST_LEN, "time.nist.gov");
  gCfg.time.syncIntervalMin = 60;
  gCfg.time.useRtc = 1;
  gCfg.time.allowBrowserSync = 1;
  gCfg.time.driftPpm100 = 100;
  gCfg.time.syncOnReconnect = 1;

  strDefault(gCfg.ui.deviceName, sizeof(gCfg.ui.deviceName), "RelayHub");
  strDefault(gCfg.ui.adminUser, sizeof(gCfg.ui.adminUser), "admin");
  strDefault(gCfg.ui.adminPass, sizeof(gCfg.ui.adminPass), "relayhub");
  gCfg.ui.logLevel = LOG_INFO;
  gCfg.ui.pollMs = 2500;
}

void cfgDefaultsRelay(int idx) {
  RelayCfg &r = gRelay[idx];
  memset(&r, 0, sizeof(r));
  snprintf(r.name, sizeof(r.name), "Relay %d", idx + 1);
  r.enabled = 1;
  r.activeLow = 1;
  r.override = OVR_AUTO;
  for (int s = 0; s < NSCHED; s++) {
    memset(&r.sched[s], 0, sizeof(Schedule));
    r.sched[s].enabled = 0;
    r.sched[s].startH = 8;
    r.sched[s].stopH = 18;
    r.sched[s].combMode = COMB_AND;
    r.sched[s].leapMode = LEAP_ANY;
    r.sched[s].vfY = 24; r.sched[s].vfM = 1; r.sched[s].vfD = 1;
    r.sched[s].vtY = 29; r.sched[s].vtM = 12; r.sched[s].vtD = 31;
  }
}

void cfgDefaults() {
  cfgDefaultsSys();
  for (int i = 0; i < NRELAYS; i++) cfgDefaultsRelay(i);
}

bool cfgValidateName(const char *name) {
  if (name == nullptr) return false;
  size_t n = strlen(name);
  if (n == 0 || n > RELAY_NAME_LEN) return false;
  bool hasGraphic = false;
  for (size_t i = 0; i < n; i++) {
    unsigned char c = (unsigned char)name[i];
    if (c < 0x20 || c == 0x7F) return false;
    if (c != ' ' && c != '\t') hasGraphic = true;
  }
  return hasGraphic;
}

static bool validIpv4OrEmpty(const char *s) {
  if (s == nullptr || s[0] == 0) return true;
  IPAddress a;
  return a.fromString(s);
}

void cfgSanitizeSys() {
  clampStr(gCfg.sta.hostname, sizeof(gCfg.sta.hostname));
  clampStr(gCfg.sta.ssid, sizeof(gCfg.sta.ssid));
  clampStr(gCfg.sta.pass, sizeof(gCfg.sta.pass));
  clampStr(gCfg.sta.ssid2, sizeof(gCfg.sta.ssid2));
  clampStr(gCfg.sta.pass2, sizeof(gCfg.sta.pass2));
  clampStr(gCfg.sta.ip, sizeof(gCfg.sta.ip));
  clampStr(gCfg.sta.gw, sizeof(gCfg.sta.gw));
  clampStr(gCfg.sta.sn, sizeof(gCfg.sta.sn));
  clampStr(gCfg.sta.dns1, sizeof(gCfg.sta.dns1));
  clampStr(gCfg.sta.dns2, sizeof(gCfg.sta.dns2));
  if (gCfg.sta.hostname[0] == 0)
    strCopy(gCfg.sta.hostname, sizeof(gCfg.sta.hostname), "relayhub");
  if (strlen(gCfg.sta.pass) > 0 && strlen(gCfg.sta.pass) < 8) gCfg.sta.pass[0] = 0;
  gCfg.sta.enabled = gCfg.sta.enabled ? 1 : 0;
  if (gCfg.sta.ssid[0] == 0) gCfg.sta.enabled = 0;
  gCfg.sta.dhcp = gCfg.sta.dhcp ? 1 : 0;
  gCfg.sta.autoReconnect = gCfg.sta.autoReconnect ? 1 : 0;
  gCfg.sta.powerSave = gCfg.sta.powerSave ? 1 : 0;
  if (gCfg.sta.reconnectSec < 1) gCfg.sta.reconnectSec = 1;
  if (gCfg.sta.reconnectSec > 60) gCfg.sta.reconnectSec = 60;
  if (gCfg.sta.dhcp == 0) {
    if (!validIpv4OrEmpty(gCfg.sta.ip)) gCfg.sta.ip[0] = 0;
    if (!validIpv4OrEmpty(gCfg.sta.gw)) gCfg.sta.gw[0] = 0;
    if (!validIpv4OrEmpty(gCfg.sta.sn)) gCfg.sta.sn[0] = 0;
    if (!validIpv4OrEmpty(gCfg.sta.dns1)) gCfg.sta.dns1[0] = 0;
    if (!validIpv4OrEmpty(gCfg.sta.dns2)) gCfg.sta.dns2[0] = 0;
    if (gCfg.sta.ip[0] == 0 || gCfg.sta.gw[0] == 0 || gCfg.sta.sn[0] == 0)
      gCfg.sta.dhcp = 1;
  }

  clampStr(gCfg.ap.ssid, sizeof(gCfg.ap.ssid));
  clampStr(gCfg.ap.pass, sizeof(gCfg.ap.pass));
  if (gCfg.ap.ssid[0] == 0) {
    uint64_t mac = ESP.getEfuseMac();
    snprintf(gCfg.ap.ssid, sizeof(gCfg.ap.ssid), "RelayHub-%04X", (unsigned)(mac & 0xFFFF));
  }
  if (strlen(gCfg.ap.pass) > 0 && strlen(gCfg.ap.pass) < 8) gCfg.ap.pass[0] = 0;
  gCfg.ap.enabled = gCfg.ap.enabled ? 1 : 0;
  gCfg.ap.hidden = gCfg.ap.hidden ? 1 : 0;
  gCfg.ap.captive = gCfg.ap.captive ? 1 : 0;
  gCfg.ap.fallback = gCfg.ap.fallback ? 1 : 0;
  gCfg.ap.autoOff = gCfg.ap.autoOff ? 1 : 0;
  if (gCfg.ap.channel < 1 || gCfg.ap.channel > 13) gCfg.ap.channel = 6;
  if (gCfg.ap.maxConn < 1 || gCfg.ap.maxConn > 4) gCfg.ap.maxConn = 4;
  if (gCfg.ap.fallbackSec < 10) gCfg.ap.fallbackSec = 10;
  if (gCfg.ap.fallbackSec > 3600) gCfg.ap.fallbackSec = 3600;

  if (gCfg.time.tzMode != TZ_MODE_AUTO) gCfg.time.tzMode = TZ_MODE_MANUAL;
  gCfg.time.tzOffsetSec = (int32_t)clampI(gCfg.time.tzOffsetSec, -50400, 50400);
  if (gCfg.time.dstMode > DST_CUSTOM) gCfg.time.dstMode = DST_NONE;
  gCfg.time.dstOffsetSec = (int32_t)clampI(gCfg.time.dstOffsetSec, 0, 7200);
  gCfg.time.dstStartMonth = (uint8_t)clampI(gCfg.time.dstStartMonth, 1, 12);
  gCfg.time.dstEndMonth   = (uint8_t)clampI(gCfg.time.dstEndMonth, 1, 12);
  gCfg.time.dstStartWeek  = (uint8_t)clampI(gCfg.time.dstStartWeek, 0, 4);
  gCfg.time.dstEndWeek    = (uint8_t)clampI(gCfg.time.dstEndWeek, 0, 4);
  gCfg.time.dstStartDow   = (uint8_t)clampI(gCfg.time.dstStartDow, 0, 6);
  gCfg.time.dstEndDow     = (uint8_t)clampI(gCfg.time.dstEndDow, 0, 6);
  gCfg.time.dstStartHour  = (uint8_t)clampI(gCfg.time.dstStartHour, 0, 23);
  gCfg.time.dstEndHour    = (uint8_t)clampI(gCfg.time.dstEndHour, 0, 23);
  clampStr(gCfg.time.posix, sizeof(gCfg.time.posix));
  if (gCfg.time.autoZone >= kTzZoneCount && gCfg.time.autoZone != 255) gCfg.time.autoZone = 0;
  if (gCfg.time.ntpCount > NTP_MAX) gCfg.time.ntpCount = NTP_MAX;
  for (int i = 0; i < NTP_MAX; i++) {
    clampStr(gCfg.time.ntp[i], sizeof(gCfg.time.ntp[i]));
    if (i < gCfg.time.ntpCount && gCfg.time.ntp[i][0] == 0)
      strCopy(gCfg.time.ntp[i], sizeof(gCfg.time.ntp[i]), "pool.ntp.org");
  }
  if (gCfg.time.syncIntervalMin < 1) gCfg.time.syncIntervalMin = 1;
  if (gCfg.time.syncIntervalMin > 1440) gCfg.time.syncIntervalMin = 1440;
  gCfg.time.useRtc = gCfg.time.useRtc ? 1 : 0;
  gCfg.time.allowBrowserSync = gCfg.time.allowBrowserSync ? 1 : 0;
  gCfg.time.syncOnReconnect = gCfg.time.syncOnReconnect ? 1 : 0;
  if (gCfg.time.driftPpm100 < -10000) gCfg.time.driftPpm100 = -10000;
  if (gCfg.time.driftPpm100 > 10000) gCfg.time.driftPpm100 = 10000;

  clampStr(gCfg.ui.deviceName, sizeof(gCfg.ui.deviceName));
  clampStr(gCfg.ui.adminUser, sizeof(gCfg.ui.adminUser));
  clampStr(gCfg.ui.adminPass, sizeof(gCfg.ui.adminPass));
  if (gCfg.ui.deviceName[0] == 0)
    strCopy(gCfg.ui.deviceName, sizeof(gCfg.ui.deviceName), "RelayHub");
  if (gCfg.ui.adminUser[0] == 0)
    strCopy(gCfg.ui.adminUser, sizeof(gCfg.ui.adminUser), "admin");
  if (gCfg.ui.adminPass[0] == 0)
    strCopy(gCfg.ui.adminPass, sizeof(gCfg.ui.adminPass), "relayhub");
  gCfg.ui.authEnabled = gCfg.ui.authEnabled ? 1 : 0;
  gCfg.ui.logLevel = (uint8_t)clampI(gCfg.ui.logLevel, 0, 3);
  if (gCfg.ui.pollMs < 1000) gCfg.ui.pollMs = 1000;
  if (gCfg.ui.pollMs > 10000) gCfg.ui.pollMs = 10000;
}

void cfgSanitizeRelay(RelayCfg &r, int idx) {
  clampStr(r.name, sizeof(r.name));
  if (!cfgValidateName(r.name)) snprintf(r.name, sizeof(r.name), "Relay %d", idx + 1);
  r.enabled = r.enabled ? 1 : 0;
  r.activeLow = r.activeLow ? 1 : 0;
  if (r.override > OVR_FORCE_LOW) r.override = OVR_AUTO;

  for (int s = 0; s < NSCHED; s++) {
    Schedule &q = r.sched[s];
    q.enabled = q.enabled ? 1 : 0;
    q.startH = (uint8_t)clampI(q.startH, 0, 23);
    q.startM = (uint8_t)clampI(q.startM, 0, 59);
    q.startS = (uint8_t)clampI(q.startS, 0, 59);
    q.stopH  = (uint8_t)clampI(q.stopH, 0, 23);
    q.stopM  = (uint8_t)clampI(q.stopM, 0, 59);
    q.stopS  = (uint8_t)clampI(q.stopS, 0, 59);
    if (q.tailSec > 86400) q.tailSec = 86400;
    q.monthMask &= 0x0FFF;
    q.domMask &= 0x7FFFFFFFUL;
    q.dowMask &= 0x7F;
    q.combMode = (q.combMode == COMB_OR) ? COMB_OR : COMB_AND;
    if (q.leapMode > LEAP_NONLEAP) q.leapMode = LEAP_ANY;
    q.flags &= (SCH_F_OVERNIGHT | SCH_F_END_INCL | SCH_F_TAIL |
                SCH_F_VALID_FROM | SCH_F_VALID_TO);
    q.vfM = (uint8_t)clampI(q.vfM, 1, 12);
    q.vtM = (uint8_t)clampI(q.vtM, 1, 12);
    q.vfD = (uint8_t)clampI(q.vfD, 1, 31);
    q.vtD = (uint8_t)clampI(q.vtD, 1, 31);
  }
}

bool cfgLoadSys() {
  char k[8];
  SysCfg a, b;
  uint32_t s0 = 0, s1 = 0;
  sysKeyForSeq(0, k, sizeof(k));
  bool okA = nvsReadRecord(k, SYS_CFG_VERSION, &a, sizeof(a), s0);
  sysKeyForSeq(1, k, sizeof(k));
  bool okB = nvsReadRecord(k, SYS_CFG_VERSION, &b, sizeof(b), s1);

  if (!okA && !okB) return false;
  if (okA && okB) {
    if (s0 >= s1) memcpy(&gCfg, &a, sizeof(a)), gStore.seqSys = s0;
    else          memcpy(&gCfg, &b, sizeof(b)), gStore.seqSys = s1;
  } else if (okA) {
    memcpy(&gCfg, &a, sizeof(a));
    gStore.seqSys = s0;
  } else {
    memcpy(&gCfg, &b, sizeof(b));
    gStore.seqSys = s1;
  }
  return true;
}

bool cfgLoadRelay(int idx) {
  char k[8];
  RelayCfg a, b;
  uint32_t s0 = 0, s1 = 0;
  relayKeyForSeq(idx, 0, k, sizeof(k));
  bool okA = nvsReadRecord(k, RELAY_CFG_VERSION, &a, sizeof(a), s0);
  relayKeyForSeq(idx, 1, k, sizeof(k));
  bool okB = nvsReadRecord(k, RELAY_CFG_VERSION, &b, sizeof(b), s1);

  if (!okA && !okB) return false;
  if (okA && okB) {
    if (s0 >= s1) { memcpy(&gRelay[idx], &a, sizeof(a)); gStore.seqRelay[idx] = s0; }
    else          { memcpy(&gRelay[idx], &b, sizeof(b)); gStore.seqRelay[idx] = s1; }
  } else if (okA) {
    memcpy(&gRelay[idx], &a, sizeof(a));
    gStore.seqRelay[idx] = s0;
  } else {
    memcpy(&gRelay[idx], &b, sizeof(b));
    gStore.seqRelay[idx] = s1;
  }
  return true;
}

void cfgLoadAll() {
  memset(&gStore, 0, sizeof(gStore));
  memset(&gCfg, 0, sizeof(gCfg));
  memset(gRelay, 0, sizeof(gRelay));
  memset(&gDirty, 0, sizeof(gDirty));

  bool sysOk = cfgLoadSys();
  if (!sysOk) {
    rhLog(LOG_WARN, "No valid system config found - using defaults");
    cfgDefaultsSys();
    gDirty.sysDirty = true;
  }

  for (int i = 0; i < NRELAYS; i++) {
    if (!cfgLoadRelay(i)) {
      cfgDefaultsRelay(i);
      gDirty.relayMask |= (1u << i);
    } else if (gRelay[i].name[0] == 0) {
      cfgDefaultsRelay(i);
      gDirty.relayMask |= (1u << i);
    }
  }

  cfgSanitizeSys();
  for (int i = 0; i < NRELAYS; i++) cfgSanitizeRelay(gRelay[i], i);
  gLogLevelNow = gCfg.ui.logLevel;

  if (gDirty.sysDirty || gDirty.relayMask) {
    gDirty.pending = true;
    gDirty.firstMs = millis();
    gDirty.lastMs = millis();
  }
}

void cfgMarkSysDirty() {
  uint32_t now = millis();
  if (!gDirty.pending) {
    gDirty.pending = true;
    gDirty.firstMs = now;
  }
  gDirty.sysDirty = true;
  gDirty.lastMs = now;
}

void cfgMarkRelayDirty(int idx) {
  if (idx < 0 || idx >= NRELAYS) return;
  uint32_t now = millis();
  if (!gDirty.pending) {
    gDirty.pending = true;
    gDirty.firstMs = now;
  }
  gDirty.relayMask |= (1u << idx);
  gDirty.lastMs = now;
}

void cfgMarkAllDirty() {
  uint32_t now = millis();
  gDirty.pending = true;
  gDirty.sysDirty = true;
  gDirty.relayMask = 0xFFFF;
  gDirty.firstMs = now;
  gDirty.lastMs = now;
}

bool cfgPending() { return gDirty.pending; }

void cfgFlushNow() {
  if (!gDirty.pending) return;
  bool ok = true;
  char key[8];

  if (gDirty.sysDirty) {
    sysKeyForSeq(gStore.seqSys, key, sizeof(key));
    if (nvsWriteRecord(key, SYS_CFG_VERSION, &gCfg, sizeof(gCfg), gStore.seqSys)) {
      gDirty.sysDirty = false;
    } else {
      ok = false;
    }
  }
  for (int i = 0; i < NRELAYS; i++) {
    if (!(gDirty.relayMask & (1u << i))) continue;
    relayKeyForSeq(i, gStore.seqRelay[i], key, sizeof(key));
    if (nvsWriteRecord(key, RELAY_CFG_VERSION, &gRelay[i], sizeof(gRelay[i]),
                       gStore.seqRelay[i])) {
      gDirty.relayMask &= ~(1u << i);
    } else {
      ok = false;
    }
  }
  if (!gDirty.sysDirty && gDirty.relayMask == 0) {
    gDirty.pending = false;
    rhLog(LOG_DBG, "Config flushed to NVS (%u writes total)", (unsigned)gStore.writes);
  }
  if (!ok) rhLog(LOG_WARN, "Some config records could not be written");
}

void cfgService() {
  if (!gDirty.pending) return;
  uint32_t now = millis();
  uint32_t sinceLast = now - gDirty.lastMs;
  uint32_t sinceFirst = now - gDirty.firstMs;
  if (sinceLast >= CFG_WRITE_DEBOUNCE_MS || sinceFirst >= CFG_WRITE_MAXDELAY_MS) {
    cfgFlushNow();
  }
}

void cfgFactoryReset() {
  rhLog(LOG_WARN, "Factory reset requested");
  if (gPrefs.begin(kPrefsNs, false)) {
    gPrefs.clear();
    gPrefs.end();
  }
  ESP.restart();
}

RTC_DS3231 rtc;
WiFiUDP    gNtpUdp;
DstRule    gDstRule;
bool       gDstRuleReady = false;

#define NTP_LOCAL_PORT     2390
#define NTP_PHASE_IDLE     0
#define NTP_PHASE_RESOLVE  1
#define NTP_PHASE_WAIT     2
#define NTP_RESOLVE_MS     3500UL
#define NTP_REPLY_MS       2000UL
#define NTP_RETRY_MS       30000UL

static uint8_t gNtpRot = 0;

int32_t  posixOffsetAt(int64_t utc);
int32_t  appOffsetAt(const void *ctx, int64_t utc);
DstRule  timeDstRuleBuild();
void     timeApplyTz();
void     timeSetEpochUs(int64_t sec, uint32_t usec, uint8_t source, bool writeRtc);
void     timeSetEpoch(int64_t sec, uint8_t source, bool writeRtc);
bool     timeReadRtc(int64_t &epoch);
bool     timeWriteRtc(int64_t epoch);
bool     timeSyncFromRtc(bool force);
bool     timeIsValid();
void     timeService();
void     timeSetup();
int32_t  timeOffsetNow();
void     timeLocalNow(LocalDT &out);
TimeCtx  timeContext();
void     ntpService();
void     ntpKick();
void     ntpAbort();

bool timePlausible(int64_t e) {
  return (e >= 1577836800LL && e < 4102444800LL);
}

int32_t posixOffsetAt(int64_t utc) {
  time_t t = (time_t)utc;
  struct tm lt;
  if (localtime_r(&t, &lt) == nullptr) return 0;
  int64_t local = civilToEpoch(lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday,
                               lt.tm_hour, lt.tm_min, lt.tm_sec);
  int32_t off = (int32_t)(local - utc);
  if (off < -18 * 3600 || off > 18 * 3600) off = 0;
  return off;
}

int32_t appOffsetAt(const void *ctx, int64_t utc) {
  (void)ctx;
  if (gCfg.time.tzMode == TZ_MODE_AUTO) return posixOffsetAt(utc);
  if (!gDstRuleReady) return gCfg.time.tzOffsetSec;
  return dstEffectiveOffset(gDstRule, gCfg.time.tzOffsetSec, utc);
}

DstRule timeDstRuleBuild() {
  DstRule r;
  memset(&r, 0, sizeof(r));
  switch (gCfg.time.dstMode) {
    case DST_US:
      r.enabled = 1;
      r.startMonth = 3;  r.startWeek = 1; r.startDow = 0; r.startHour = 2;
      r.endMonth = 11;   r.endWeek = 0;   r.endDow = 0;   r.endHour = 2;
      break;
    case DST_EU:
      r.enabled = 1;
      r.startMonth = 3;  r.startWeek = 4; r.startDow = 0; r.startHour = 2;
      r.endMonth = 10;   r.endWeek = 4;   r.endDow = 0;   r.endHour = 3;
      break;
    case DST_AU:
      r.enabled = 1;
      r.startMonth = 10; r.startWeek = 0; r.startDow = 0; r.startHour = 2;
      r.endMonth = 4;    r.endWeek = 0;   r.endDow = 0;   r.endHour = 3;
      break;
    case DST_NZ:
      r.enabled = 1;
      r.startMonth = 9;  r.startWeek = 4; r.startDow = 0; r.startHour = 2;
      r.endMonth = 4;    r.endWeek = 0;   r.endDow = 0;   r.endHour = 3;
      break;
    case DST_CUSTOM:
      r.enabled = 1;
      r.startMonth = gCfg.time.dstStartMonth;
      r.startWeek  = gCfg.time.dstStartWeek;
      r.startDow   = gCfg.time.dstStartDow;
      r.startHour  = gCfg.time.dstStartHour;
      r.endMonth   = gCfg.time.dstEndMonth;
      r.endWeek    = gCfg.time.dstEndWeek;
      r.endDow     = gCfg.time.dstEndDow;
      r.endHour    = gCfg.time.dstEndHour;
      break;
    default:
      r.enabled = 0;
      break;
  }
  r.dstSec = (gCfg.time.dstOffsetSec > 0) ? gCfg.time.dstOffsetSec : 3600;
  return r;
}

void timeApplyTz() {
  gDstRule = timeDstRuleBuild();
  gDstRuleReady = true;

  if (gCfg.time.tzMode == TZ_MODE_AUTO) {
    const char *p = nullptr;
    if (gCfg.time.autoZone == 255) p = gCfg.time.posix;
    else if (gCfg.time.autoZone < kTzZoneCount) p = kTzZones[gCfg.time.autoZone].posix;
    if (p == nullptr || p[0] == 0) p = "UTC0";
    setenv("TZ", p, 1);
  } else {
    setenv("TZ", "UTC0", 1);
  }
  tzset();
}

int32_t timeOffsetNow() {
  return appOffsetAt(nullptr, (int64_t)time(nullptr));
}

void timeLocalNow(LocalDT &out) {
  int64_t now = (int64_t)time(nullptr);
  out = localDtFromEpoch(now + appOffsetAt(nullptr, now));
}

TimeCtx timeContext() {
  TimeCtx c;
  c.offsetAt = appOffsetAt;
  c.ctx = nullptr;
  return c;
}

bool timeIsValid() {
  int64_t now = (int64_t)time(nullptr);
  return gTimeRt.valid && timePlausible(now);
}

void timeSetEpochUs(int64_t sec, uint32_t usec, uint8_t source, bool writeRtc) {
  if (!timePlausible(sec)) {
    rhLog(LOG_WARN, "Refusing implausible time %lld", (long long)sec);
    return;
  }
  struct timeval tv;
  tv.tv_sec = (time_t)sec;
  tv.tv_usec = (suseconds_t)(usec % 1000000UL);
  settimeofday(&tv, nullptr);

  bool wasValid = gTimeRt.valid;
  gTimeRt.valid = true;
  gTimeRt.source = source;
  gTimeRt.lastSyncUtc = sec;
  gTimeRt.lastSyncMs = millis();
  gTimeRt.lastSyncTryMs = millis();
  gTimeRt.driftAccUs = 0;
  gNowEpoch = (uint32_t)sec;

  if (source == TSRC_NTP || source == TSRC_BROWSER) gTimeRt.syncOk++;

  if (writeRtc && gCfg.time.useRtc && gTimeRt.rtcOk) timeWriteRtc(sec);

  if (!wasValid) {
    rhLog(LOG_INFO, "Clock is now valid (%s)", source == TSRC_NTP ? "NTP" :
                          source == TSRC_RTC ? "RTC" :
                          source == TSRC_BROWSER ? "browser" : "?");
  }
}

void timeSetEpoch(int64_t sec, uint8_t source, bool writeRtc) {
  timeSetEpochUs(sec, 0, source, writeRtc);
}

bool timeReadRtc(int64_t &epoch) {
  if (!gTimeRt.rtcOk) return false;
  DateTime dt = rtc.now();
  uint32_t u = dt.unixtime();
  if (!timePlausible((int64_t)u)) return false;
  epoch = (int64_t)u;
  return true;
}

bool timeWriteRtc(int64_t epoch) {
  if (!gTimeRt.rtcOk) return false;
  if (!timePlausible(epoch)) return false;
  rtc.adjust(DateTime((uint32_t)epoch));
  return true;
}

bool timeSyncFromRtc(bool force) {
  if (!gCfg.time.useRtc || !gTimeRt.rtcOk) return false;
  int64_t e = 0;
  if (!timeReadRtc(e)) return false;
  int64_t now = (int64_t)time(nullptr);
  int64_t diff = e - now;
  if (diff < 0) diff = -diff;
  if (!gTimeRt.valid || force) {
    if (diff >= 1) {
      timeSetEpochUs(e, 0, TSRC_RTC, false);
      return true;
    }
  } else if (diff >= 2) {
    timeSetEpochUs(e, 0, TSRC_RTC, false);
    return true;
  }
  return false;
}

void timeSetup() {
  Wire.begin(kRtcSda, kRtcScl);
  Wire.setClock(100000);
  gTimeRt.rtcOk = rtc.begin(&Wire);
  if (gTimeRt.rtcOk) {
    gTimeRt.rtcLostPower = rtc.lostPower();
    if (gTimeRt.rtcLostPower) {
      rhLog(LOG_WARN, "DS3231 detected but lost power (battery?) - time ignored");
    } else {
      int64_t e = 0;
      if (timeReadRtc(e)) {
        timeSetEpochUs(e, 0, TSRC_RTC, false);
        rhLog(LOG_INFO, "DS3231 time restored");
      }
    }
  } else {
    rhLog(LOG_WARN, "DS3231 not found on SDA=%u SCL=%u", kRtcSda, kRtcScl);
  }

  timeApplyTz();

  if (!gNtpUdp.begin(NTP_LOCAL_PORT)) rhLog(LOG_WARN, "NTP UDP port %u unavailable", (unsigned)NTP_LOCAL_PORT);

  gNtp.phase = NTP_PHASE_IDLE;
  gNtp.dnsDone = false;
  gNtp.dnsOk = false;
  gNtp.tries = 0;
  gNtp.nextSyncMs = millis() + 2000UL;

  if (!gTimeRt.valid) rhLog(LOG_WARN, "Clock not set yet - use NTP or browser sync");
}

bool ipAddrToIp(const ip_addr_t *a, IPAddress &out) {
  if (a == nullptr) return false;
  if (!IP_IS_V4(a)) return false;
  const ip4_addr_t *v4 = ip_2_ip4(a);
  out = IPAddress(ip4_addr1(v4), ip4_addr2(v4), ip4_addr3(v4), ip4_addr4(v4));
  return true;
}

void ntpDnsCallback(const char *name, const ip_addr_t *ip, void *arg) {
  (void)name; (void)arg;
  if (ip != nullptr) {
    memcpy(&gNtp.addr, ip, sizeof(ip_addr_t));
    gNtp.dnsOk = true;
  } else {
    gNtp.dnsOk = false;
  }
  gNtp.dnsDone = true;
}

void ntpBeginServer(uint8_t idx) {
  if (idx >= gCfg.time.ntpCount) idx = 0;
  gNtp.serverIdx = idx;
  gNtp.phase = NTP_PHASE_RESOLVE;
  gNtp.phaseStartMs = millis();
  gNtp.dnsDone = false;
  gNtp.dnsOk = false;

  const char *host = gCfg.time.ntp[idx];
  IPAddress lit;
  if (lit.fromString(host)) {
    IP_ADDR4(&gNtp.addr, lit[0], lit[1], lit[2], lit[3]);
    gNtp.dnsDone = true;
    gNtp.dnsOk = true;
    return;
  }
  err_t e = dns_gethostbyname(host, &gNtp.addr, ntpDnsCallback, nullptr);
  if (e == ERR_OK) {
    gNtp.dnsDone = true;
    gNtp.dnsOk = true;
  } else if (e != ERR_INPROGRESS) {
    gNtp.dnsDone = true;
    gNtp.dnsOk = false;
  }
}

void ntpSendQuery() {
  uint8_t pkt[48];
  memset(pkt, 0, sizeof(pkt));
  pkt[0] = (uint8_t)((0 << 6) | (4 << 3) | 3);
  IPAddress dst;
  if (!ipAddrToIp(&gNtp.addr, dst)) {
    gNtp.phase = NTP_PHASE_RESOLVE;
    gNtp.dnsDone = true;
    gNtp.dnsOk = false;
    return;
  }
  if (gNtpUdp.beginPacket(dst, 123) != 1) {
    gNtp.phase = NTP_PHASE_RESOLVE;
    gNtp.dnsDone = true;
    gNtp.dnsOk = false;
    return;
  }
  gNtpUdp.write(pkt, sizeof(pkt));
  gNtpUdp.endPacket();
  gNtp.phase = NTP_PHASE_WAIT;
  gNtp.phaseStartMs = millis();
}

void ntpNextAttempt(bool failed) {
  if (failed) {
    rhLog(LOG_WARN, "NTP attempt failed on server %u", (unsigned)gNtp.serverIdx);
    gNtp.tries++;
  }
  if (gNtp.tries >= gCfg.time.ntpCount) {
    gNtp.phase = NTP_PHASE_IDLE;
    gTimeRt.syncFail++;
    gTimeRt.lastNtpResult = 0;
    gNtp.nextSyncMs = millis() + NTP_RETRY_MS;
    rhLog(LOG_WARN, "All NTP servers failed - will retry in %us", (unsigned)(NTP_RETRY_MS / 1000));
    if (!gTimeRt.valid) timeSyncFromRtc(true);
    return;
  }
  uint8_t count = gCfg.time.ntpCount;
  uint8_t idx = (uint8_t)((gNtpRot + gNtp.tries) % count);
  ntpBeginServer(idx);
}

void ntpSuccess(uint32_t secs, uint32_t frac) {
  int64_t e = (int64_t)secs - 2208988800LL;
  if (e < 0) e += 4294967296LL;
  uint32_t usec = (uint32_t)(((uint64_t)frac * 1000000ULL) >> 32);

  timeSetEpochUs(e, usec, TSRC_NTP, true);
  gTimeRt.lastNtpResult = 1;
  gTimeRt.lastNtpServer = (int8_t)gNtp.serverIdx;
  gNtp.phase = NTP_PHASE_IDLE;
  gNtp.tries = 0;
  uint32_t intervalMs = (uint32_t)gCfg.time.syncIntervalMin * 60000UL;
  gNtp.nextSyncMs = millis() + intervalMs;
  gNtpRot = (uint8_t)((gNtpRot + 1) % (gCfg.time.ntpCount ? gCfg.time.ntpCount : 1));
  rhLog(LOG_INFO, "NTP sync ok via %s", gCfg.time.ntp[gNtp.serverIdx]);
}

void ntpPoll() {
  if (gNtp.phase != NTP_PHASE_WAIT) {
    gNtpUdp.parsePacket();
    return;
  }
  int sz = gNtpUdp.parsePacket();
  if (sz >= 48) {
    uint8_t pkt[48];
    int rd = gNtpUdp.read(pkt, sizeof(pkt));
    if (rd >= 48) {
      uint8_t mode = (uint8_t)(pkt[0] & 0x07);
      uint8_t stratum = pkt[1];
      if ((mode == 4 || mode == 5) && stratum > 0 && stratum < 16) {
        uint32_t secs = ((uint32_t)pkt[40] << 24) | ((uint32_t)pkt[41] << 16) |
                        ((uint32_t)pkt[42] << 8) | (uint32_t)pkt[43];
        if (secs == 0) { ntpNextAttempt(true); return; }
        uint32_t frac = ((uint32_t)pkt[44] << 24) | ((uint32_t)pkt[45] << 16) |
                        ((uint32_t)pkt[46] << 8) | (uint32_t)pkt[47];
        ntpSuccess(secs, frac);
        return;
      }
    }
  }
  if (millisSince(gNtp.phaseStartMs) > NTP_REPLY_MS) ntpNextAttempt(true);
}

void ntpAbort() {
  gNtp.phase = NTP_PHASE_IDLE;
  gNtp.tries = 0;
  gNtp.dnsDone = false;
}

void ntpKick() {
  if (!gCfg.time.ntpEnabled || gCfg.time.ntpCount == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  gNtp.nextSyncMs = millis();
  ntpAbort();
}

void ntpService() {
  ntpPoll();
  if (gNtp.phase == NTP_PHASE_RESOLVE) {
    if (gNtp.dnsDone) {
      if (gNtp.dnsOk) ntpSendQuery();
      else ntpNextAttempt(true);
    } else if (millisSince(gNtp.phaseStartMs) > NTP_RESOLVE_MS) {
      ntpNextAttempt(true);
    }
    return;
  }
  if (gNtp.phase == NTP_PHASE_WAIT) return;

  if (!gCfg.time.ntpEnabled || gCfg.time.ntpCount == 0) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if ((int32_t)(millis() - gNtp.nextSyncMs) < 0) return;

  gTimeRt.lastSyncTryMs = millis();
  gNtp.tries = 0;
  ntpBeginServer((uint8_t)(gNtpRot % gCfg.time.ntpCount));
}

void timeService() {
  static uint32_t lastSlowMs = 0;
  static uint32_t lastTickMs = 0;
  static uint32_t lastDriftMs = 0;
  uint32_t now = millis();

  if (now - lastTickMs >= 1000UL) {
    lastTickMs = now;
    gNowEpoch = (uint32_t)time(nullptr);
    gTimeRt.valid = gTimeRt.valid && timePlausible((int64_t)time(nullptr));
  }

  if (now - lastDriftMs >= 1000UL) {
    lastDriftMs = now;
    int32_t ppm100 = gCfg.time.driftPpm100;
    if (ppm100 != 0 && gTimeRt.valid) {
      int32_t addNs = (int32_t)(ppm100 * 10);
      int32_t acc = (int32_t)gTimeRt.driftAccUs + addNs;
      if (acc >= 20000 || acc <= -20000) {
        struct timeval tv;
        gettimeofday(&tv, nullptr);
        int64_t adjUs = (int64_t)(acc / 1000);
        int64_t totalUs = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec - adjUs;
        tv.tv_sec = (time_t)(totalUs / 1000000LL);
        tv.tv_usec = (suseconds_t)(totalUs % 1000000LL);
        if (tv.tv_usec < 0) { tv.tv_usec += 1000000; tv.tv_sec -= 1; }
        settimeofday(&tv, nullptr);
        acc = 0;
      }
      gTimeRt.driftAccUs = (uint32_t)acc;
    } else {
      gTimeRt.driftAccUs = 0;
    }
  }

  if (now - lastSlowMs >= 5000UL) {
    lastSlowMs = now;
    if (gCfg.time.useRtc && gTimeRt.rtcOk && WiFi.status() != WL_CONNECTED) {
      timeSyncFromRtc(false);
    }
  }

  ntpService();
}

DNSServer gDns;
bool      gDnsRunning = false;
bool      gApDirty = true;
uint32_t  gApAppliedHash = 0;
uint8_t   gStaLastReason = 0;
bool      gStaDisabledManually = false;

volatile bool gEvGotIp = false;
volatile bool gEvStaDown = false;
volatile uint8_t gEvStaReason = 0;

uint32_t apConfigHash();
void     wifiEventCallback(WiFiEvent_t event, WiFiEventInfo_t info);
void     wifiSetup();
void     wifiService();
void     wifiApplyConfig(bool restartSta);
void     staBeginNow();
void     apApply();
void     dnsService();
void     mdnsStart();
void     mdnsStop();
void     onStaUp();
void     onStaDown();
void     wifiScanStart();
bool     staWanted();
int16_t  wifiScanStatus();

uint32_t apConfigHash() {
  uint32_t h = 2166136261UL;
  const char *a = gCfg.ap.ssid;
  const char *b = gCfg.ap.pass;
  while (*a) { h = (h ^ (uint8_t)*a++) * 16777619UL; }
  while (*b) { h = (h ^ (uint8_t)*b++) * 16777619UL; }
  h = (h ^ gCfg.ap.channel) * 16777619UL;
  h = (h ^ gCfg.ap.hidden) * 16777619UL;
  h = (h ^ gCfg.ap.maxConn) * 16777619UL;
  return h;
}

bool staWanted() {
  return (gCfg.sta.enabled != 0) && (gCfg.sta.ssid[0] != 0);
}

void wifiEventCallback(WiFiEvent_t event, WiFiEventInfo_t info) {
  (void)info;
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      gEvGotIp = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      gEvStaDown = true;
      gEvStaReason = info.wifi_sta_disconnected.reason;
      break;
    default:
      break;
  }
}

void wifiSetup() {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.onEvent(wifiEventCallback);
  WiFi.setHostname(gCfg.sta.hostname);
  gNet.lastDisconnectMs = millis();
  gNet.connectStartMs = millis();
  wifiApplyConfig(true);
}

void wifiApplyConfig(bool restartSta) {
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(gCfg.sta.powerSave != 0);
  WiFi.setHostname(gCfg.sta.hostname);

  bool wantSta = staWanted();
  bool wantAp = apShouldRun();

  wifi_mode_t mode = WIFI_OFF;
  if (wantSta && wantAp) mode = WIFI_AP_STA;
  else if (wantSta) mode = WIFI_STA;
  else if (wantAp) mode = WIFI_AP;

  if (WiFi.getMode() != mode) {
    WiFi.mode(mode);
    gApDirty = true;
    rhLog(LOG_INFO, "WiFi mode -> %s", mode == WIFI_AP_STA ? "AP+STA" :
                          mode == WIFI_STA ? "STA" : mode == WIFI_AP ? "AP" : "OFF");
  }

  apApply();
  if (restartSta && wantSta) staBeginNow();
}

void staBeginNow() {
  if (!staWanted()) return;

  if (gCfg.sta.dhcp == 0) {
    IPAddress ip, gw, sn, d1, d2;
    bool ok = ip.fromString(gCfg.sta.ip) && gw.fromString(gCfg.sta.gw) &&
              sn.fromString(gCfg.sta.sn);
    if (ok) {
      if (gCfg.sta.dns1[0]) d1.fromString(gCfg.sta.dns1);
      if (gCfg.sta.dns2[0]) d2.fromString(gCfg.sta.dns2);
      WiFi.config(ip, gw, sn, d1, d2);
    } else {
      WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
    }
  } else {
    WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  }

  const char *ssid = gCfg.sta.ssid;
  const char *pass = gCfg.sta.pass;
  if (gNet.staProfile == 1 && gCfg.sta.ssid2[0] != 0) {
    ssid = gCfg.sta.ssid2;
    pass = gCfg.sta.pass2;
  }
  WiFi.begin(ssid, pass);
  gNet.connectStartMs = millis();
  gNet.lastAttemptMs = millis();
  rhLog(LOG_INFO, "STA connecting to '%s'%s", ssid, gNet.staProfile ? " (backup)" : "");
}

void onStaUp() {
  gNet.staConnected = true;
  gNet.gotIpMs = millis();
  gNet.attempts = 0;
  gNet.staPhase = 2;
  WiFi.localIP().toString().toCharArray(gNet.ip, sizeof(gNet.ip));
  gNet.lastRssi = (int8_t)WiFi.RSSI();
  gNet.fallbackActive = false;
  gStaLastReason = 0;
  rhLog(LOG_INFO, "STA up: %s  RSSI %d dBm", gNet.ip, (int)gNet.lastRssi);

  timeSyncFromRtc(false);
  if (gCfg.time.syncOnReconnect) ntpKick();
  mdnsStart();
  gApDirty = true;
}

void onStaDown() {
  bool was = gNet.staConnected;
  gNet.staConnected = false;
  gNet.staPhase = 1;
  gNet.lastDisconnectMs = millis();
  gNet.connectStartMs = millis();
  gNet.ip[0] = 0;
  mdnsStop();
  if (was) rhLog(LOG_WARN, "STA disconnected (reason %u)", (unsigned)gStaLastReason);
  timeSyncFromRtc(true);
}

bool apShouldRun() {
  if (gCfg.ap.enabled != 0) {
    if (gCfg.ap.autoOff != 0 && gNet.staConnected && !gNet.fallbackActive) return false;
    return true;
  }
  return gNet.fallbackActive;
}

void apApply() {
  bool want = apShouldRun();
  if (!want) {
    if (gNet.apActive) {
      if (gDnsRunning) { gDns.stop(); gDnsRunning = false; }
      WiFi.softAPdisconnect(true);
      gNet.apActive = false;
      gApAppliedHash = 0;
      rhLog(LOG_INFO, "Soft AP stopped");
    }
    return;
  }
  uint32_t h = apConfigHash();
  if (gNet.apActive && h == gApAppliedHash) return;

  if (gNet.apActive) {
    WiFi.softAPdisconnect(true);
    gNet.apActive = false;
  }

  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1),
                    IPAddress(255, 255, 255, 0));
  const char *pass = (gCfg.ap.pass[0] != 0) ? gCfg.ap.pass : nullptr;
  bool ok = WiFi.softAP(gCfg.ap.ssid, pass, gCfg.ap.channel,
                        gCfg.ap.hidden != 0, gCfg.ap.maxConn);
  gNet.apActive = ok;
  gApAppliedHash = ok ? h : 0;
  if (ok) rhLog(LOG_INFO, "Soft AP '%s' up on %s", gCfg.ap.ssid,
                WiFi.softAPIP().toString().c_str());
  else rhLog(LOG_ERR, "Soft AP failed to start");
}

void dnsService() {
  bool want = gNet.apActive && (gCfg.ap.captive != 0);
  if (want && !gDnsRunning) {
    gDns.setErrorReplyCode(DNSReplyCode::NoError);
    if (gDns.start(53, "*", WiFi.softAPIP())) {
      gDnsRunning = true;
      rhLog(LOG_INFO, "Captive DNS started");
    }
  } else if (!want && gDnsRunning) {
    gDns.stop();
    gDnsRunning = false;
  }
  if (gDnsRunning) gDns.processNextRequest();
}

void mdnsStart() {
  if (gNet.mdnsActive) return;
  if (!gNet.staConnected && !gNet.apActive) return;
  if (MDNS.begin(gCfg.sta.hostname)) {
    MDNS.addService("http", "tcp", 80);
    gNet.mdnsActive = true;
    rhLog(LOG_INFO, "mDNS: http://%s.local", gCfg.sta.hostname);
  } else {
    rhLog(LOG_WARN, "mDNS start failed");
  }
}

void mdnsStop() {
  if (!gNet.mdnsActive) return;
  MDNS.end();
  gNet.mdnsActive = false;
}

void wifiService() {
  uint32_t now = millis();

  if (gEvGotIp) {
    gEvGotIp = false;
    onStaUp();
  }
  if (gEvStaDown) {
    gEvStaDown = false;
    gStaLastReason = gEvStaReason;
    if (gNet.staConnected) onStaDown();
  }

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected && !gNet.staConnected) {
    onStaUp();
  } else if (!connected && gNet.staConnected) {
    onStaDown();
  }

  if (connected) {
    gNet.staPhase = 2;
    if ((now - gNet.gotIpMs) > 3000UL) {
      static uint32_t lastRssiMs = 0;
      if (now - lastRssiMs > 5000UL) {
        lastRssiMs = now;
        gNet.lastRssi = (int8_t)WiFi.RSSI();
      }
    }
  } else {
    gNet.staPhase = 1;
    if (staWanted() && gCfg.sta.autoReconnect && !gStaDisabledManually) {
      uint32_t interval = (uint32_t)gCfg.sta.reconnectSec * 1000UL;
      if (interval < 250UL) interval = 250UL;
      if (now - gNet.lastAttemptMs >= interval) {
        gNet.lastAttemptMs = now;
        gNet.attempts++;
        timeSyncFromRtc(false);
        if (now - gNet.connectStartMs >= 15000UL) {
          if (gCfg.sta.ssid2[0] != 0 && (gNet.attempts > 30)) {
            gNet.staProfile = (uint8_t)((gNet.attempts / 30) & 1);
          }
          staBeginNow();
          gNet.connectStartMs = now;
        } else if (WiFi.status() == WL_DISCONNECTED) {
          WiFi.reconnect();
        }
      }
    }
  }

  if (gCfg.ap.fallback && staWanted() && !connected) {
    uint32_t since = now - gNet.lastDisconnectMs;
    if (!gNet.fallbackActive && since > (uint32_t)gCfg.ap.fallbackSec * 1000UL) {
      gNet.fallbackActive = true;
      gApDirty = true;
      rhLog(LOG_WARN, "STA down - starting fallback AP");
      wifiApplyConfig(false);
    }
  } else if (connected && gNet.fallbackActive) {
    gNet.fallbackActive = false;
    gApDirty = true;
  }

  if (gApDirty || apShouldRun() != gNet.apActive) {
    gApDirty = false;
    apApply();
  }

  dnsService();
  if (gNet.staConnected || gNet.apActive) mdnsStart();
}

void wifiScanStart() {
  int16_t st = WiFi.scanComplete();
  if (st == WIFI_SCAN_RUNNING) return;
  WiFi.scanDelete();
  WiFi.scanNetworks(true, false);
  rhLog(LOG_DBG, "WiFi scan started");
}

int16_t wifiScanStatus() {
  return WiFi.scanComplete();
}

bool gTimeWarned = false;
uint32_t gRelayEvalMs = 0;
uint32_t gRelayTransitions = 0;

void relaySetup();
void relayService();
void relayForce();
void relayWritePin(int idx);
void relaySetOverride(int idx, uint8_t ovr);
void relaySetAllOverride(uint8_t ovr);
void relaySetAllActiveLow(uint8_t activeLow);
uint8_t relayPinLevelFor(int idx, bool logicalOn);

void relaySetup() {
  for (int i = 0; i < NRELAYS; i++) {
    gRt[i].pinLevel = 0xFF;
    gRt[i].logicalOn = false;
    gRt[i].changes = 0;
    gRt[i].activeRules = 0;
    gRt[i].lastChangeMs = 0;
    gRt[i].lastChangeEpoch = 0;
    uint8_t offLevel = gRelay[i].activeLow ? HIGH : LOW;
    digitalWrite(kRelayPins[i], offLevel);
    pinMode(kRelayPins[i], OUTPUT);
    digitalWrite(kRelayPins[i], offLevel);
    gRt[i].pinLevel = offLevel;
  }
  rhLog(LOG_INFO, "Relay outputs initialised (16 pins)");
}

uint8_t relayPinLevelFor(int idx, bool logicalOn) {
  return gRelay[idx].activeLow ? (logicalOn ? LOW : HIGH) : (logicalOn ? HIGH : LOW);
}

void relayWritePin(int idx) {
  RelayCfg &r = gRelay[idx];
  RelayRt &rt = gRt[idx];

  bool timeOk = timeIsValid();
  int active = 0;
  bool logical = false;

  if (r.enabled && timeOk) {
    int64_t utc = (int64_t)time(nullptr);
    TimeCtx ctx = timeContext();
    for (int s = 0; s < NSCHED; s++) {
      if (r.sched[s].enabled && scheduleActiveAt(r.sched[s], utc, ctx)) {
        active++;
        logical = true;
      }
    }
  }

  uint8_t level;
  if (r.override == OVR_FORCE_HIGH) {
    level = HIGH;
  } else if (r.override == OVR_FORCE_LOW) {
    level = LOW;
  } else if (!r.enabled) {
    level = r.activeLow ? HIGH : LOW;
  } else {
    level = relayPinLevelFor(idx, logical);
  }

  bool energized = r.activeLow ? (level == LOW) : (level == HIGH);

  if (level != rt.pinLevel) {
    digitalWrite(kRelayPins[idx], level);
    rt.changes++;
    gRelayTransitions++;
    rt.lastChangeMs = millis();
    rt.lastChangeEpoch = (int64_t)time(nullptr);
    rhLog(LOG_DBG, "Relay %d -> %s (pin %u, %d rules)", idx + 1,
          energized ? "ON" : "OFF", (unsigned)level, active);
  }
  rt.pinLevel = level;
  rt.logicalOn = energized;
  rt.activeRules = (uint8_t)(active > 255 ? 255 : active);
  rt.forced = (r.override != OVR_AUTO);
}

void relayService() {
  bool valid = timeIsValid();
  if (!valid && !gTimeWarned) {
    gTimeWarned = true;
    rhLog(LOG_WARN, "Time invalid: scheduled relays stay OFF until synced");
  }
  if (valid) gTimeWarned = false;

  uint32_t now = millis();
  if (now - gRelayEvalMs < 200UL) return;
  gRelayEvalMs = now;

  for (int i = 0; i < NRELAYS; i++) relayWritePin(i);
}

void relayForce() {
  for (int i = 0; i < NRELAYS; i++) {
    gRt[i].pinLevel = 0xFF;
  }
  relayService();
}

void relaySetOverride(int idx, uint8_t ovr) {
  if (idx < 0 || idx >= NRELAYS) return;
  if (ovr > OVR_FORCE_LOW) ovr = OVR_AUTO;
  gRelay[idx].override = ovr;
  cfgMarkRelayDirty(idx);
  relayWritePin(idx);
  rhLog(LOG_INFO, "Relay %d override -> %s", idx + 1,
        ovr == OVR_FORCE_HIGH ? "HIGH" : ovr == OVR_FORCE_LOW ? "LOW" : "AUTO");
}

void relaySetAllOverride(uint8_t ovr) {
  if (ovr > OVR_FORCE_LOW) ovr = OVR_AUTO;
  for (int i = 0; i < NRELAYS; i++) {
    gRelay[i].override = ovr;
    gRt[i].pinLevel = 0xFF;
    cfgMarkRelayDirty(i);
    relayWritePin(i);
  }
  rhLog(LOG_INFO, "All relays override -> %s",
        ovr == OVR_FORCE_HIGH ? "HIGH" : ovr == OVR_FORCE_LOW ? "LOW" : "AUTO");
}

void relaySetAllActiveLow(uint8_t activeLow) {
  for (int i = 0; i < NRELAYS; i++) {
    gRelay[i].activeLow = activeLow ? 1 : 0;
    gRt[i].pinLevel = 0xFF;
    cfgMarkRelayDirty(i);
    relayWritePin(i);
  }
  rhLog(LOG_INFO, "All relays polarity -> active %s", activeLow ? "LOW" : "HIGH");
}

uint32_t gBtnPressMs = 0;
bool     gBtnWasDown = false;
bool     gBtnWarned = false;

void buttonSetup();
void buttonService();

void buttonSetup() {
  pinMode(kBootPin, INPUT_PULLUP);
  gBtnPressMs = 0;
  gBtnWasDown = false;
}

void buttonService() {
  bool down = (digitalRead(kBootPin) == LOW);
  uint32_t now = millis();
  if (down) {
    if (!gBtnWasDown) {
      gBtnWasDown = true;
      gBtnPressMs = now;
      gBtnWarned = false;
      rhLog(LOG_DBG, "BOOT button pressed");
    } else if (!gBtnWarned && (now - gBtnPressMs) >= 3000UL) {
      gBtnWarned = true;
      rhLog(LOG_WARN, "BOOT held 3 s - keep holding for factory reset");
    }
    if ((now - gBtnPressMs) >= 5000UL) {
      rhLog(LOG_WARN, "BOOT held 5 s - factory reset");
      cfgFactoryReset();
    }
  } else {
    if (gBtnWasDown) rhLog(LOG_DBG, "BOOT released");
    gBtnWasDown = false;
    gBtnPressMs = 0;
  }
}

uint32_t freeHeapNow() { return (uint32_t)ESP.getFreeHeap(); }
uint32_t minHeapNow() { return (uint32_t)esp_get_minimum_free_heap_size(); }

uint32_t largestBlockNow() {
  return (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
}

uint8_t heapFragPct() {
  uint32_t freeH = freeHeapNow();
  if (freeH == 0) return 100;
  uint32_t largest = largestBlockNow();
  if (largest >= freeH) return 0;
  return (uint8_t)(100UL - (largest * 100UL) / freeH);
}

const char *resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_EXT:      return "external";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "int-wdt";
    case ESP_RST_TASK_WDT: return "task-wdt";
    case ESP_RST_WDT:      return "wdt";
    case ESP_RST_DEEPSLEEP:return "deep-sleep";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_SDIO:     return "sdio";
    default:               return "unknown";
  }
}

WebServer server(80);
String    gPage;
bool      gPageReserved = false;

static const char kCss[] PROGMEM = R"CSS(
:root{--bg:#0a0e13;--card:#141b24;--line:#22303f;--txt:#e7eef7;--dim:#8ea3b8;--acc:#2fd4a7;--acc2:#4aa8ff;--warn:#ffb020;--err:#ff5d6c;--ok:#2fd4a7;--r:14px}
*{box-sizing:border-box}
html,body{margin:0;background:radial-gradient(1100px 520px at 85% -12%,#123049 0,transparent 62%),var(--bg);color:var(--txt);font:14px/1.45 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
a{color:var(--acc2);text-decoration:none}
.top{position:sticky;top:0;z-index:30;display:flex;align-items:center;gap:14px;padding:10px 16px;background:rgba(9,13,18,.88);backdrop-filter:blur(10px);border-bottom:1px solid var(--line);flex-wrap:wrap}
.brand{font-weight:750;letter-spacing:.3px;display:flex;gap:9px;align-items:center;font-size:15px}
.brand .dot{width:10px;height:10px;border-radius:50%;background:var(--acc);box-shadow:0 0 14px var(--acc)}
.nav{display:flex;gap:2px;flex-wrap:wrap;flex:1}
.nav a{padding:6px 10px;border-radius:9px;color:var(--dim);font-size:13px;font-weight:550}
.nav a:hover{background:#16212c;color:var(--txt)}
.nav a.on{background:#1a2a39;color:#fff;box-shadow:inset 0 0 0 1px var(--line)}
.clock{display:flex;align-items:center;gap:8px;margin-left:auto;font-variant-numeric:tabular-nums;flex-wrap:wrap;justify-content:flex-end}
.clock .t{font-size:19px;font-weight:750;letter-spacing:.6px;color:#fff}
.clock .d{color:var(--dim);font-size:12px;font-weight:650;letter-spacing:.4px}
.pill{padding:3px 9px;border-radius:999px;font-size:11px;border:1px solid var(--line);color:var(--dim);background:#0d141c;white-space:nowrap;font-weight:600}
.pill.ok{color:var(--ok);border-color:#1d4a3d;background:#0b1e19}
.pill.warn{color:var(--warn);border-color:#4a3a15;background:#1f1a0d}
.pill.err{color:var(--err);border-color:#4a1e24;background:#1f0d10}
.pill.acc{color:var(--acc2);border-color:#1d3a4a;background:#0b1620}
main{padding:18px;max-width:1300px;margin:0 auto}
h1,h2,h3{margin:0 0 10px}
h1{font-size:21px;letter-spacing:-.2px}
h2{font-size:16px}
h3{font-size:12px;color:var(--dim);font-weight:700;text-transform:uppercase;letter-spacing:.09em}
.sub{color:var(--dim);font-size:13px;margin:-6px 0 16px}
.grid{display:grid;gap:14px}
.g2{grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}
.g3{grid-template-columns:repeat(auto-fit,minmax(230px,1fr))}
.g4{grid-template-columns:repeat(auto-fit,minmax(170px,1fr))}
.card{background:linear-gradient(180deg,rgba(255,255,255,.022),rgba(255,255,255,0)),var(--card);border:1px solid var(--line);border-radius:var(--r);padding:14px}
.card.tight{padding:10px 12px}
.row{display:flex;gap:10px;align-items:center;flex-wrap:wrap}
.row.between{justify-content:space-between}
.big{font-size:30px;font-weight:750;letter-spacing:.5px;font-variant-numeric:tabular-nums}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12.5px}
.dim{color:var(--dim)}
.small{font-size:12px}
.btn{appearance:none;border:1px solid var(--line);background:#18222e;color:var(--txt);padding:8px 12px;border-radius:9px;font:inherit;font-weight:600;cursor:pointer;transition:.12s;display:inline-flex;gap:6px;align-items:center;justify-content:center}
.btn:hover{background:#1f2c3a;border-color:#2f4358}
.btn:active{transform:translateY(1px)}
.btn:disabled{opacity:.45;cursor:not-allowed}
.btn.primary{background:linear-gradient(180deg,#1f8f74,#17735e);border-color:#1f8f74;color:#eafff9}
.btn.primary:hover{background:linear-gradient(180deg,#24a586,#1a816a)}
.btn.danger{background:#33161b;border-color:#5a2630;color:#ff98a3}
.btn.danger:hover{background:#3f1a20}
.btn.ghost{background:transparent}
.btn.sm{padding:5px 9px;font-size:12px;border-radius:7px}
.btn.on{background:#12463a;border-color:#1f8f74;color:#c9fff0}
input,select,textarea{background:#0c131a;border:1px solid var(--line);color:var(--txt);padding:8px 10px;border-radius:8px;font:inherit;width:100%;min-width:0}
input:focus,select:focus,textarea:focus{outline:none;border-color:#2f6f8f;box-shadow:0 0 0 3px rgba(74,168,255,.12)}
input[type=checkbox]{width:auto;min-width:auto}
input[type=range]{padding:0}
label.f{display:flex;flex-direction:column;gap:5px;margin-bottom:10px;font-size:12px;color:var(--dim);font-weight:600}
label.f > span{letter-spacing:.02em}
.fieldset{display:grid;grid-template-columns:1fr 1fr;gap:0 12px}
.fieldset.one{grid-template-columns:1fr}
@media(max-width:640px){.fieldset{grid-template-columns:1fr}}
.switch{display:flex;align-items:center;gap:8px;margin:6px 0 12px;font-size:13px}
.switch input{width:18px;height:18px;accent-color:#1f8f74}
.tbl{width:100%;border-collapse:collapse;font-size:13px}
.tbl th{text-align:left;color:var(--dim);font-weight:600;font-size:11px;text-transform:uppercase;letter-spacing:.07em;padding:6px 8px;border-bottom:1px solid var(--line)}
.tbl td{padding:7px 8px;border-bottom:1px solid #1a2530;vertical-align:middle}
.tbl tr:last-child td{border-bottom:none}
.relay{display:flex;flex-direction:column;gap:8px;min-height:150px}
.relay .name{font-size:15px;font-weight:700;cursor:pointer;border-bottom:1px dashed transparent}
.relay .name:hover{border-color:#2f4358}
.relay .meta{display:flex;gap:6px;flex-wrap:wrap}
.led{width:10px;height:10px;border-radius:50%;background:#33414f;box-shadow:inset 0 0 0 1px #0006;flex:none}
.led.on{background:var(--acc);box-shadow:0 0 10px var(--acc)}
.acts{display:flex;gap:6px;flex-wrap:wrap;margin-top:auto}
.sum{display:flex;gap:4px;flex-wrap:wrap;min-height:20px}
.chip{font-size:10.5px;padding:2px 6px;border-radius:6px;background:#0f1a24;border:1px solid #1d2c3b;color:var(--dim);font-variant-numeric:tabular-nums}
.chip.on{color:#bfffe9;border-color:#1d4a3d;background:#0b1e19}
.chip.off{opacity:.6}
.modal{position:fixed;inset:0;background:#000a;backdrop-filter:blur(3px);display:none;align-items:flex-start;justify-content:center;z-index:100;padding:20px;overflow:auto}
.modal.open{display:flex}
.sheet{background:#101821;border:1px solid var(--line);border-radius:16px;max-width:980px;width:100%;padding:16px;box-shadow:0 30px 80px #000a}
.sheet-head{display:flex;justify-content:space-between;align-items:center;gap:12px;margin-bottom:12px}
.sched-layout{display:grid;grid-template-columns:130px 1fr;gap:14px}
@media(max-width:760px){.sched-layout{grid-template-columns:1fr}}
.slots{display:flex;flex-direction:column;gap:6px}
.slot{padding:8px 10px;border-radius:8px;border:1px solid var(--line);background:#0c131a;cursor:pointer;font-size:12px;display:flex;justify-content:space-between;gap:6px}
.slot.on{background:#123a31;border-color:#1f8f74}
.slot.sel{box-shadow:0 0 0 2px #2f6f8f inset}
.cal{display:grid;grid-template-columns:repeat(7,1fr);gap:4px;max-width:330px}
.cal .dow{font-size:10px;color:var(--dim);text-align:center;padding:3px 0;font-weight:700}
.cal button{padding:6px 0;font-size:12px;border-radius:7px;border:1px solid var(--line);background:#0c131a;color:var(--txt);cursor:pointer}
.cal button.on{background:#12463a;border-color:#1f8f74;color:#c9fff0}
.cal button.blank{visibility:hidden}
.toggles{display:flex;flex-wrap:wrap;gap:5px}
.tg{padding:5px 9px;border-radius:7px;border:1px solid var(--line);background:#0c131a;color:var(--dim);cursor:pointer;font-size:12px;font-weight:600}
.tg.on{background:#123a31;border-color:#1f8f74;color:#c9fff0}
.time3{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:230px}
.time3 input{text-align:center}
.hint{font-size:11.5px;color:#6f8499;margin-top:4px}
.toasts{position:fixed;right:16px;bottom:16px;display:flex;flex-direction:column;gap:8px;z-index:200;max-width:min(420px,90vw)}
.toast{background:#16212c;border:1px solid var(--line);border-left:3px solid var(--acc2);padding:10px 12px;border-radius:10px;font-size:13px;box-shadow:0 12px 30px #0008;animation:in .18s ease-out}
.toast.err{border-left-color:var(--err)}
.toast.ok{border-left-color:var(--acc)}
@keyframes in{from{transform:translateY(6px);opacity:0}to{transform:none;opacity:1}}
.foot{color:#5f7288;font-size:11.5px;text-align:center;padding:18px 10px 26px}
.kv{display:grid;grid-template-columns:auto 1fr;gap:4px 14px;font-size:13px}
.kv .k{color:var(--dim)}
.kv .v{text-align:right;font-variant-numeric:tabular-nums}
.logs{font-family:ui-monospace,Menlo,Consolas,monospace;font-size:11.5px;max-height:320px;overflow:auto;background:#0a1118;border:1px solid var(--line);border-radius:10px;padding:8px}
.logs div{padding:2px 0;border-bottom:1px solid #101a23;white-space:pre-wrap;word-break:break-word}
.logs .ERR{color:#ff8a95}.logs .WRN{color:#ffcf7a}.logs .INF{color:#9fe8d4}.logs .DBG{color:#7f93a8}
.scan{max-height:260px;overflow:auto;border:1px solid var(--line);border-radius:10px}
.scan button{width:100%;text-align:left;background:transparent;border:0;border-bottom:1px solid #16212c;color:var(--txt);padding:8px 10px;font:inherit;cursor:pointer}
.scan button:hover{background:#16212c}
.bar{height:6px;border-radius:4px;background:#1a2530;overflow:hidden}
.bar > i{display:block;height:100%;background:linear-gradient(90deg,#1f8f74,#4aa8ff)}
.warnbox{border:1px solid #4a3a15;background:#1f1a0d;color:#ffd591;padding:10px 12px;border-radius:10px;font-size:12.5px;margin:10px 0}
.okbox{border:1px solid #1d4a3d;background:#0b1e19;color:#bfffe9;padding:10px 12px;border-radius:10px;font-size:12.5px;margin:10px 0}
hr.sep{border:0;border-top:1px solid var(--line);margin:14px 0}
)CSS";

static const char kJs[] PROGMEM = R"JS(
"use strict";
const $=(s,r=document)=>r.querySelector(s);
const $$=(s,r=document)=>Array.from(r.querySelectorAll(s));
const RH={time:{epoch:0,at:0,offset:0,valid:false,src:"--"},status:null,poll:2500};
const esc=(s)=>{return String(s==null?"":s).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));}
const pad2=(n)=>{return String(n).padStart(2,"0");}
const fmtDur=(sec)=>{sec=Math.max(0,Math.floor(sec||0));const d=Math.floor(sec/86400),h=Math.floor(sec%86400/3600),m=Math.floor(sec%3600/60),s=sec%60;return (d?d+"d ":"")+pad2(h)+":"+pad2(m)+":"+pad2(s);}
const toast=(msg,kind)=>{const wrap=$("#toasts");if(!wrap)return;const el=document.createElement("div");el.className="toast "+(kind||"");el.textContent=msg;wrap.appendChild(el);setTimeout(()=>{el.style.opacity="0";el.style.transition="opacity .25s";setTimeout(()=>el.remove(),260);},kind==="err"?6000:3200);}
const api=async (path,opts)=>{const o=Object.assign({cache:"no-store",headers:{}},opts||{});if(o.body!==undefined&&typeof o.body!=="string"){o.headers["Content-Type"]="application/json";o.body=JSON.stringify(o.body);}const r=await fetch(path,o);let j=null;try{j=await r.json();}catch(e){j={ok:false,error:"bad response (HTTP "+r.status+")"};}if(!r.ok||j.ok===false)throw new Error((j&&j.error)||("HTTP "+r.status));return j;}
const nowLocalMs=()=>{return (RH.time.epoch+Math.floor((performance.now()-RH.time.at)/1000)+RH.time.offset)*1000;}
const renderClock=()=>{const d=new Date(nowLocalMs());const t=pad2(d.getUTCHours())+":"+pad2(d.getUTCMinutes())+":"+pad2(d.getUTCSeconds());const dow=["SUN","MON","TUE","WED","THU","FRI","SAT"][d.getUTCDay()];const dt=dow+" "+pad2(d.getUTCMonth()+1)+"/"+pad2(d.getUTCDate())+"/"+String(d.getUTCFullYear()).slice(-2);const a=$("#hdrTime"),b=$("#hdrDate");if(a)a.textContent=RH.time.valid?t:"--:--:--";if(b)b.textContent=RH.time.valid?dt:"--- --/--/--";const s=$("#hdrSrc");if(s){const src=RH.time.valid?(RH.time.src||"SET"):"NO TIME";s.textContent=src;s.className="pill "+(RH.time.valid?(src==="NTP"?"ok":"acc"):"err");}}
const syncClock=async ()=>{try{const t=await api("/api/time");RH.time={epoch:t.epoch,at:performance.now(),offset:t.offset,valid:t.valid,src:t.src};renderClock();}catch(e){}}
const renderStatus=(s)=>{RH.status=s;if(s.time){RH.time.epoch=s.time.epoch;RH.time.offset=s.time.offset;RH.time.valid=s.time.valid;RH.time.at=performance.now();RH.time.src=s.time.src;}renderClock();const n=$("#hdrNet");if(n){if(s.sta){n.textContent="STA "+(s.rssi||0)+"dBm";n.className="pill ok";}else if(s.ap){n.textContent="AP "+(s.apClients||0);n.className="pill acc";}else{n.textContent="OFFLINE";n.className="pill err";}}const v=$("#hdrSave");if(v){v.textContent=s.save&&s.save.pending?"PENDING":"SAVED";v.className="pill "+(s.save&&s.save.pending?"warn":"ok");}}
const pollStatus=async ()=>{try{const s=await api("/api/status");renderStatus(s);window.dispatchEvent(new CustomEvent("rh:status",{detail:s}));}catch(e){const n=$("#hdrNet");if(n){n.textContent="OFFLINE";n.className="pill err";}}}
const onDouble=(fn)=>{let last=0;return function(ev){const n=performance.now();if(n-last<420){last=0;fn(ev);}else{last=n;}};}
const dblTap=(el,fn)=>{let last=0,fired=0;const go=(ev)=>{const n=Date.now();if(n-fired<700)return;if(n-last<450){fired=n;last=0;if(ev&&ev.preventDefault)ev.preventDefault();fn(ev);}else{last=n;}};if(el){el.ondblclick=go;el.ontouchend=go;}}
const openModal=(id)=>{const m=document.getElementById(id);if(m)m.classList.add("open");}
const closeModal=(id)=>{const m=document.getElementById(id);if(m)m.classList.remove("open");}
const confirmBox=(msg)=>{return new Promise(res=>{const w=document.createElement("div");w.className="modal open";w.innerHTML='<div class="sheet" style="max-width:420px"><h3 style="margin-bottom:8px">Confirm</h3><p class="small" style="margin:0 0 14px">'+esc(msg)+'</p><div class="row" style="justify-content:flex-end"><button class="btn ghost" data-x>Cancel</button><button class="btn danger" data-y>Confirm</button></div></div>';document.body.appendChild(w);const done=v=>{w.remove();res(v);};w.querySelector("[data-x]").onclick=()=>done(false);w.querySelector("[data-y]").onclick=()=>done(true);});}
const busy=(btn,on)=>{if(!btn)return;if(on){btn.dataset.txt=btn.textContent;btn.disabled=true;btn.textContent="Working…";}else{btn.disabled=false;if(btn.dataset.txt)btn.textContent=btn.dataset.txt;}}
const num=(v,d)=>{const n=parseInt(v,10);return Number.isFinite(n)?n:d;}
document.addEventListener("DOMContentLoaded",()=>{syncClock();setInterval(syncClock,30000);setInterval(renderClock,250);pollStatus();setInterval(pollStatus,RH.poll||2500);document.addEventListener("visibilitychange",()=>{if(!document.hidden){syncClock();pollStatus();}});document.addEventListener("keydown",e=>{if(e.key==="Escape")$$(".modal.open").forEach(m=>m.classList.remove("open"));});});
)JS";

void webSetup();
void sendPage();
bool webAuth();
void sendJsonRaw(int code, const char *body);
void sendOkMsg(const char *msg);
void sendErrMsg(int code, const char *msg);
bool readJsonBody(RhJsonDoc &doc);
void hEsc(const char *in, char *out, size_t cap);
void pageBegin(const char *title, const char *active);
void pageEnd();
void handleNotFound();
void handleCss();
void handleJs();
void handleFavicon();
void handlePortal();

void pageScript(const char *js);
void catInt(int v);
void handleDashboard();
void handleRelaysPage();
void handleGpioPage();
void handleTimePage();
void handleApPage();
void handleStaPage();
void handleSystemPage();
void handleSettingsPage();
void apiStatus();
void apiTimeGet();
void apiTimeSet();
void apiTimeSyncNow();
void apiTimeConfig();
void apiRelaysGet();
void apiRelayGet();
void apiRelaySave();
void apiRelayName();
void apiRelayOverride();
void apiGpioRelay();
void apiGpioGlobal();
void apiStaGet();
void apiStaSave();
void apiStaConnect();
void apiStaDisconnect();
void apiScanGet();
void apiScanStart();
void apiApGet();
void apiApSave();
void apiApClients();
void apiSystemGet();
void apiReboot();
void apiFactoryReset();
void apiSaveNow();
void apiLogsGet();
void apiLogsClear();
void apiSettingsGet();
void apiSettingsSave();
void apiConfigExport();
void apiConfigImport();
bool gGntpNextValid();

typedef void (*PageHandler)();
struct NavItem { const char *href; const char *label; const char *key; };
static const NavItem kNav[] = {
  { "/",          "Dashboard", "dash"  },
  { "/relays",    "Relays",    "relay" },
  { "/gpio",      "GPIO",      "gpio"  },
  { "/time",      "Time",      "time"  },
  { "/wifi-ap",   "WiFi AP",   "ap"    },
  { "/wifi-sta",  "WiFi STA",  "sta"   },
  { "/system",    "System",    "sys"   },
  { "/settings",  "Settings",  "set"   }
};

void hEsc(const char *in, char *out, size_t cap) {
  if (out == nullptr || cap == 0) return;
  if (in == nullptr) { out[0] = 0; return; }
  size_t o = 0;
  for (size_t i = 0; in[i] && o + 7 < cap; i++) {
    switch (in[i]) {
      case '&': memcpy(out + o, "&amp;", 5); o += 5; break;
      case '<': memcpy(out + o, "&lt;", 4); o += 4; break;
      case '>': memcpy(out + o, "&gt;", 4); o += 4; break;
      case '"': memcpy(out + o, "&quot;", 6); o += 6; break;
      case '\'':memcpy(out + o, "&#39;", 5); o += 5; break;
      default: out[o++] = in[i]; break;
    }
  }
  out[o] = 0;
}

bool webAuth() {
  if (!gCfg.ui.authEnabled) return true;
  if (server.authenticate(gCfg.ui.adminUser, gCfg.ui.adminPass)) return true;
  server.requestAuthentication();
  return false;
}

void sendJsonRaw(int code, const char *body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

void sendOkMsg(const char *msg) {
  char b[200];
  char e[160];
  hEsc(msg ? msg : "ok", e, sizeof(e));
  snprintf(b, sizeof(b), "{\"ok\":true,\"msg\":\"%s\"}", e);
  sendJsonRaw(200, b);
}

void sendErrMsg(int code, const char *msg) {
  char b[240];
  char e[180];
  hEsc(msg ? msg : "error", e, sizeof(e));
  snprintf(b, sizeof(b), "{\"ok\":false,\"error\":\"%s\"}", e);
  sendJsonRaw(code, b);
}

bool readJsonBody(RhJsonDoc &doc) {
  if (server.method() != HTTP_POST) { sendErrMsg(405, "POST required"); return false; }
  if (!server.hasArg("plain")) { sendErrMsg(400, "Missing request body"); return false; }
  String body = server.arg("plain");
  if (body.length() == 0) { sendErrMsg(400, "Empty request body"); return false; }
  if (body.length() > 40000) { sendErrMsg(413, "Body too large"); return false; }
  DeserializationError err = deserializeJson(doc, body);
  if (err) { sendErrMsg(400, err.c_str()); return false; }
  return true;
}

void pageBegin(const char *title, const char *active) {
  if (!gPageReserved) {
    gPage.reserve(24576);
    gPageReserved = true;
  }
  gPage = "";
  gPage += F("<!doctype html><html lang=\"en\"><head><meta charset=\"utf-8\">"
             "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
             "<meta name=\"color-scheme\" content=\"dark\"><title>");
  gPage += title;
  gPage += F(" · ");
  gPage += gCfg.ui.deviceName;
  gPage += F("</title><link rel=\"stylesheet\" href=\"/a.css\"></head><body>");
  gPage += F("<header class=\"top\"><div class=\"brand\"><span class=\"dot\"></span>");
  gPage += gCfg.ui.deviceName;
  gPage += F("</div><nav class=\"nav\">");
  for (uint8_t i = 0; i < (uint8_t)(sizeof(kNav) / sizeof(kNav[0])); i++) {
    gPage += F("<a href=\"");
    gPage += kNav[i].href;
    gPage += F("\"");
    if (active != nullptr && strcmp(active, kNav[i].key) == 0) gPage += F(" class=\"on\"");
    gPage += F(">");
    gPage += kNav[i].label;
    gPage += F("</a>");
  }
  gPage += F("</nav><div class=\"clock\">"
             "<span class=\"t\" id=\"hdrTime\">--:--:--</span>"
             "<span class=\"d\" id=\"hdrDate\">--- --/--/--</span>"
             "<span class=\"pill\" id=\"hdrSrc\">TIME</span>"
             "<span class=\"pill\" id=\"hdrNet\">NET</span>"
             "<span class=\"pill\" id=\"hdrSave\">SAVED</span>"
             "</div></header><main>");
}

void pageEnd() {
  char up[32];
  fmtUptime(millis(), up, sizeof(up));
  gPage += F("</main><footer class=\"foot\">");
  gPage += FW_NAME;
  gPage += F(" ");
  gPage += FW_VERSION;
  gPage += F(" · uptime ");
  gPage += up;
  gPage += F(" · heap ");
  gPage += String(freeHeapNow() / 1024);
  gPage += F(" KB</footer><div id=\"toasts\" class=\"toasts\"></div>"
             "<script src=\"/a.js\"></script></body></html>");
}

void sendPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", gPage);
}

void handleCss() {
  if (!webAuth()) return;
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send_P(200, "text/css", kCss);
}

void handleJs() {
  if (!webAuth()) return;
  server.sendHeader("Cache-Control", "public, max-age=600");
  server.send_P(200, "application/javascript", kJs);
}

void handleFavicon() {
  server.sendHeader("Cache-Control", "public, max-age=86400");
  server.send(204, "image/x-icon", "");
}

void handleNotFound() {
  String uri = server.uri();
  if (gNet.apActive && gCfg.ap.captive) {
    if (uri == "/generate_204" || uri == "/gen_204" ||
        uri == "/hotspot-detect.html" || uri == "/library/test/success.html" ||
        uri == "/connecttest.txt" || uri == "/ncsi.txt" ||
        uri == "/redirect" || uri == "/canonical.html" ||
        uri == "/success.txt" || uri == "/fwlink") {
      server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/portal", true);
      server.send(302, "text/plain", "");
      return;
    }
    String host = server.hostHeader();
    if (host != WiFi.softAPIP().toString() && host != gCfg.sta.hostname) {
      server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/portal", true);
      server.send(302, "text/plain", "");
      return;
    }
  }
  if (!webAuth()) return;
  server.sendHeader("Cache-Control", "no-store");
  server.send(404, "text/plain", "Not found");
}

void webSetup() {
  gPage.reserve(24576);
  gPageReserved = true;

  server.on("/", HTTP_GET, handleDashboard);
  server.on("/relays", HTTP_GET, handleRelaysPage);
  server.on("/gpio", HTTP_GET, handleGpioPage);
  server.on("/time", HTTP_GET, handleTimePage);
  server.on("/wifi-ap", HTTP_GET, handleApPage);
  server.on("/wifi-sta", HTTP_GET, handleStaPage);
  server.on("/system", HTTP_GET, handleSystemPage);
  server.on("/settings", HTTP_GET, handleSettingsPage);
  server.on("/portal", HTTP_GET, handlePortal);

  server.on("/a.css", HTTP_GET, handleCss);
  server.on("/a.js", HTTP_GET, handleJs);
  server.on("/favicon.ico", HTTP_GET, handleFavicon);

  server.on("/api/status", HTTP_GET, apiStatus);
  server.on("/api/time", HTTP_GET, apiTimeGet);
  server.on("/api/time/set", HTTP_POST, apiTimeSet);
  server.on("/api/time/sync", HTTP_POST, apiTimeSyncNow);
  server.on("/api/time/config", HTTP_POST, apiTimeConfig);

  server.on("/api/relays", HTTP_GET, apiRelaysGet);
  server.on("/api/relay", HTTP_GET, apiRelayGet);
  server.on("/api/relay/save", HTTP_POST, apiRelaySave);
  server.on("/api/relay/name", HTTP_POST, apiRelayName);
  server.on("/api/relay/override", HTTP_POST, apiRelayOverride);

  server.on("/api/gpio/relay", HTTP_POST, apiGpioRelay);
  server.on("/api/gpio/global", HTTP_POST, apiGpioGlobal);

  server.on("/api/wifi/sta", HTTP_GET, apiStaGet);
  server.on("/api/wifi/sta/save", HTTP_POST, apiStaSave);
  server.on("/api/wifi/sta/connect", HTTP_POST, apiStaConnect);
  server.on("/api/wifi/sta/disconnect", HTTP_POST, apiStaDisconnect);
  server.on("/api/wifi/scan", HTTP_GET, apiScanGet);
  server.on("/api/wifi/scan", HTTP_POST, apiScanStart);
  server.on("/api/wifi/ap", HTTP_GET, apiApGet);
  server.on("/api/wifi/ap/save", HTTP_POST, apiApSave);
  server.on("/api/wifi/ap/clients", HTTP_GET, apiApClients);

  server.on("/api/system", HTTP_GET, apiSystemGet);
  server.on("/api/system/reboot", HTTP_POST, apiReboot);
  server.on("/api/system/factory", HTTP_POST, apiFactoryReset);
  server.on("/api/system/save", HTTP_POST, apiSaveNow);
  server.on("/api/logs", HTTP_GET, apiLogsGet);
  server.on("/api/logs/clear", HTTP_POST, apiLogsClear);

  server.on("/api/settings", HTTP_GET, apiSettingsGet);
  server.on("/api/settings/save", HTTP_POST, apiSettingsSave);
  server.on("/api/config/export", HTTP_GET, apiConfigExport);
  server.on("/api/config/import", HTTP_POST, apiConfigImport);

  server.onNotFound(handleNotFound);
  server.begin();
  rhLog(LOG_INFO, "HTTP server on port 80");
}

void pageScript(const char *js) {
  gPage += F("<script>");
  gPage += js;
  gPage += F("</script>");
}

void catInt(int v) {
  char b[16];
  snprintf(b, sizeof(b), "%d", v);
  gPage += b;
}

void handleDashboard() {
  if (!webAuth()) return;
  pageBegin("Dashboard", "dash");
  gPage += F("<h1>Dashboard</h1><p class=\"sub\">Live state of relays, clock, network and storage.</p>"
             "<div class=\"grid g4\">"
             "<div class=\"card\"><h3>Local time</h3><div class=\"big\" id=\"dashClock\">--:--:--</div>"
             "<div class=\"dim small\" id=\"dashDate\">&mdash;</div>"
             "<div class=\"row\" style=\"margin-top:8px\"><span class=\"pill\" id=\"dashSrc\">&mdash;</span>"
             "<span class=\"pill\" id=\"dashTz\">&mdash;</span></div></div>"
             "<div class=\"card\"><h3>Network</h3><div class=\"big\" id=\"dashNet\">OFF</div>"
             "<div class=\"dim small\" id=\"dashNetInfo\">&mdash;</div></div>"
             "<div class=\"card\"><h3>Uptime</h3><div class=\"big\" id=\"dashUp\">&mdash;</div>"
             "<div class=\"dim small\" id=\"dashHeap\">&mdash;</div></div>"
             "<div class=\"card\"><h3>Relays energised</h3><div class=\"big\" id=\"dashOn\">&mdash;</div>"
             "<div class=\"dim small\" id=\"dashRules\">&mdash;</div></div>"
             "</div>"
             "<div class=\"grid g2\" style=\"margin-top:14px\">"
             "<div class=\"card\"><div class=\"row between\"><h2>Relays</h2>"
             "<a class=\"btn sm\" href=\"/relays\">Manage schedules</a></div>"
             "<div class=\"grid g4\" id=\"dashGrid\" style=\"margin-top:10px\"></div></div>"
             "<div class=\"card\"><div class=\"row between\"><h2>Recent events</h2>"
             "<a class=\"btn sm ghost\" href=\"/system\">All logs</a></div>"
             "<div class=\"logs\" id=\"dashLogs\" style=\"margin-top:10px\">Loading&hellip;</div></div>"
             "</div>");
  pageEnd();
  pageScript(R"JS(
let D=null;
const draw=()=>{
  if(!D) return;
  const d=new Date(nowLocalMs()), p=pad2;
  $("#dashClock").textContent=p(d.getUTCHours())+":"+p(d.getUTCMinutes())+":"+p(d.getUTCSeconds());
  $("#dashDate").textContent=["SUN","MON","TUE","WED","THU","FRI","SAT"][d.getUTCDay()]+" "+p(d.getUTCMonth()+1)+"/"+p(d.getUTCDate())+"/"+d.getUTCFullYear();
  const t=D.time;
  $("#dashSrc").textContent=t.valid?t.src:"NOT SET";
  $("#dashSrc").className="pill "+(t.valid?"ok":"err");
  {const o=t.offset,sg=o<0?"-":"+",a=Math.abs(o);$("#dashTz").textContent="UTC"+sg+pad2(Math.floor(a/3600))+":"+pad2(Math.floor(a%3600/60));}
  $("#dashNet").textContent=D.sta?"ONLINE":(D.ap?"AP MODE":"OFFLINE");
  $("#dashNet").style.fontSize="22px";
  $("#dashNetInfo").textContent=D.sta?(D.ip+" · "+D.rssi+" dBm"):(D.ap?("AP clients: "+D.apClients):"No link");
  $("#dashUp").textContent=fmtDur(D.uptime);
  $("#dashHeap").textContent="heap "+(D.heap/1024)+" KB · frag "+D.frag+"%";
  const on=D.relays.filter(r=>r.on).length;
  $("#dashOn").textContent=on+" / "+D.relays.length;
  $("#dashRules").textContent=D.relays.reduce((a,r)=>a+(r.act||0),0)+" active rules";
  const g=$("#dashGrid");
  if(g.dataset.n!=String(D.relays.length)){
    g.dataset.n=String(D.relays.length);g.innerHTML="";
    D.relays.forEach(r=>{
      const el=document.createElement("div");el.className="card tight";
      el.innerHTML='<div class="row between"><span class="led'+(r.on?" on":"")+'" id="dg-led-'+r.id+'"></span><span class="chip">GPIO '+r.pin+'</span></div>'+
        '<div class="small" style="margin:6px 0;font-weight:650" id="dg-n-'+r.id+'"></div>'+
        '<div class="row" style="gap:4px"><button class="btn sm" data-on="'+r.id+'">ON</button><button class="btn sm" data-off="'+r.id+'">OFF</button><button class="btn sm ghost" data-auto="'+r.id+'">A</button></div>';
      g.appendChild(el);
    });
    g.querySelectorAll("[data-on]").forEach(b=>b.onclick=()=>ovr(+b.dataset.on,1));
    g.querySelectorAll("[data-off]").forEach(b=>b.onclick=()=>ovr(+b.dataset.off,2));
    g.querySelectorAll("[data-auto]").forEach(b=>b.onclick=()=>ovr(+b.dataset.auto,0));
  }
  D.relays.forEach(r=>{
    const l=$("#dg-led-"+r.id); if(l) l.className="led"+(r.on?" on":"");
    const n=$("#dg-n-"+r.id); if(n) n.textContent=r.name;
  });
}
const ovr=async (id,mode)=>{try{await api("/api/relay/override",{method:"POST",body:{id:id,mode:mode}});toast("Relay "+(id+1)+" command sent","ok");pollStatus();}catch(e){toast(e.message,"err");}}
const logs=async ()=>{try{const j=await api("/api/logs?n=10");const el=$("#dashLogs");el.innerHTML="";(j.logs||[]).slice().reverse().forEach(l=>{const d=document.createElement("div");d.className=l.level;d.textContent=l.time+" "+l.level+" "+l.msg;el.appendChild(d);});if(!j.logs||!j.logs.length)el.textContent="No events yet.";}catch(e){}}
window.addEventListener("rh:status",e=>{D=e.detail;draw();});
document.addEventListener("DOMContentLoaded",()=>{pollStatus().then(()=>{});logs();setInterval(logs,8000);setInterval(draw,1000);});
)JS");
  sendPage();
}

void handleRelaysPage() {
  if (!webAuth()) return;
  pageBegin("Relays", "relay");
  gPage += F("<h1>Relay management</h1><p class=\"sub\">16 relays · 8 start/stop schedules each · double-tap a relay name to rename it.</p>"
             "<div class=\"row\" style=\"margin-bottom:12px\">"
             "<button class=\"btn ghost\" id=\"allAuto\">All relays to AUTO</button>"
             "<span class=\"pill\" id=\"schedCount\">&mdash;</span>"
             "<span class=\"hint\">Schedules are evaluated every 200 ms on the device. Times are local (timezone + DST aware).</span></div>"
             "<div class=\"grid g2\" id=\"relayGrid\">");
  char nameEsc[160];
  char b[220];
  for (int i = 0; i < NRELAYS; i++) {
    hEsc(gRelay[i].name, nameEsc, sizeof(nameEsc));
    snprintf(b, sizeof(b),
             "<article class=\"card relay\" data-id=\"%d\">"
             "<div class=\"row between\"><div class=\"name\" id=\"rn-%d\">%s</div>"
             "<span class=\"led\" id=\"rl-%d\"></span></div>"
             "<div class=\"meta\"><span class=\"pill\" id=\"rs-%d\">OFF</span>"
             "<span class=\"pill\" id=\"rp-%d\">GPIO %d</span>"
             "<span class=\"pill\" id=\"ra-%d\">0 rules</span></div>"
             "<div class=\"sum\" id=\"sum-%d\"></div>"
             "<div class=\"acts\">"
             "<button class=\"btn sm\" data-act=\"on\" data-id=\"%d\">Set ON</button>"
             "<button class=\"btn sm\" data-act=\"off\" data-id=\"%d\">Set OFF</button>"
             "<button class=\"btn sm ghost\" data-act=\"auto\" data-id=\"%d\">AUTO</button>"
             "<button class=\"btn sm primary\" data-act=\"edit\" data-id=\"%d\">Schedules</button>"
             "</div></article>",
             i, i, nameEsc, i, i, i, (int)kRelayPins[i], i, i, i, i, i, i);
    gPage += b;
  }
  gPage += F("</div>");

  gPage += F(
    "<div class=\"modal\" id=\"editModal\"><div class=\"sheet\">"
    "<div class=\"sheet-head\"><div><h2 id=\"edTitle\">Relay</h2>"
    "<div class=\"dim small\" id=\"edPin\"></div></div>"
    "<div class=\"row\"><label class=\"switch\" style=\"margin:0\"><input type=\"checkbox\" id=\"edRelayEn\"> Relay enabled</label>"
    "<button class=\"btn ghost\" data-close=\"editModal\">Close</button></div></div>"
    "<div class=\"sched-layout\"><div class=\"slots\" id=\"slots\"></div><div>"
    "<div class=\"row between\"><label class=\"switch\"><input type=\"checkbox\" id=\"edEn\"> Rule enabled</label>"
    "<span class=\"pill\" id=\"edState\">&mdash;</span></div>"
    "<div class=\"grid g2\">"
    "<div><h3>Start (HH:MM:SS)</h3><div class=\"time3\">"
    "<input id=\"edSh\" type=\"number\" min=\"0\" max=\"23\" inputmode=\"numeric\" placeholder=\"HH\">"
    "<input id=\"edSm\" type=\"number\" min=\"0\" max=\"59\" inputmode=\"numeric\" placeholder=\"MM\">"
    "<input id=\"edSs\" type=\"number\" min=\"0\" max=\"59\" inputmode=\"numeric\" placeholder=\"SS\"></div></div>"
    "<div><h3>Stop (HH:MM:SS)</h3><div class=\"time3\">"
    "<input id=\"edEh\" type=\"number\" min=\"0\" max=\"23\" inputmode=\"numeric\" placeholder=\"HH\">"
    "<input id=\"edEm\" type=\"number\" min=\"0\" max=\"59\" inputmode=\"numeric\" placeholder=\"MM\">"
    "<input id=\"edEs\" type=\"number\" min=\"0\" max=\"59\" inputmode=\"numeric\" placeholder=\"SS\"></div></div></div>"
    "<div class=\"row\" style=\"margin-top:10px\">"
    "<label class=\"switch\"><input type=\"checkbox\" id=\"edOvn\"> Overnight</label>"
    "<label class=\"switch\"><input type=\"checkbox\" id=\"edIncl\"> Include stop second</label>"
    "<label class=\"switch\"><input type=\"checkbox\" id=\"edTailOn\"> Tail</label>"
    "<input id=\"edTail\" type=\"number\" min=\"0\" max=\"86400\" style=\"max-width:110px\" placeholder=\"sec\">"
    "<span class=\"dim small\">extra ON seconds after stop</span></div>"
    "<hr class=\"sep\"><h3>Months</h3><div class=\"toggles\" id=\"edMonths\"></div>"
    "<h3 style=\"margin-top:12px\">Days of week</h3><div class=\"toggles\" id=\"edDow\"></div>"
    "<div class=\"grid g2\" style=\"margin-top:12px\"><div><h3>Days of month</h3>"
    "<div class=\"cal\" id=\"edCal\"></div>"
    "<div class=\"row\" style=\"margin-top:6px\"><button class=\"btn sm ghost\" id=\"edCalAll\">All</button>"
    "<button class=\"btn sm ghost\" id=\"edCalNone\">None</button><span class=\"hint\">empty = every day</span></div></div>"
    "<div><h3>Filters</h3>"
    "<label class=\"f\"><span>Month / day / weekday combination</span><select id=\"edComb\">"
    "<option value=\"0\">Match ALL selected filters</option><option value=\"1\">Match ANY selected filter</option></select></label>"
    "<label class=\"f\"><span>Leap-year filter</span><select id=\"edLeap\">"
    "<option value=\"0\">Any year</option><option value=\"1\">Leap years only</option><option value=\"2\">Non-leap years only</option></select></label>"
    "<div class=\"row\"><label class=\"f\" style=\"flex:1\"><span>Valid from</span><input id=\"edVf\" type=\"date\"></label>"
    "<label class=\"f\" style=\"flex:1\"><span>Valid until</span><input id=\"edVt\" type=\"date\"></label></div>"
    "<div class=\"hint\">Empty dates mean no bound. Rules remain valid across years.</div></div></div>"
    "<div class=\"row between\" style=\"margin-top:14px\"><span class=\"dim small\" id=\"edSummary\"></span>"
    "<div class=\"row\"><button class=\"btn ghost\" id=\"edClear\">Clear rule</button>"
    "<button class=\"btn primary\" id=\"edSave\">Save rule</button></div></div>"
    "</div></div></div></div>");
  pageEnd();
  pageScript(R"JS(
const RELAYS=16, SCHED=8, MONS=["JAN","FEB","MAR","APR","MAY","JUN","JUL","AUG","SEP","OCT","NOV","DEC"], DOWS=["SUN","MON","TUE","WED","THU","FRI","SAT"];
let R=[],cur={id:0,slot:0},draft=null;
const pad=p=>String(p).padStart(2,"0");
const hm=(t)=>{return t.slice(0,5);}
const blankRule=()=>{return {en:0,start:"08:00:00",stop:"18:00:00",flags:0,tail:0,months:0,dom:0,dow:0,comb:0,leap:0,vfy:24,vfm:1,vfd:1,vty:29,vtm:12,vtd:31};}
const chip=(txt,on)=>{const s=document.createElement("span");s.className="chip"+(on?" on":"");s.textContent=txt;return s;}
const renderCards=()=>{
  R.forEach(r=>{
    const nm=$("#rn-"+r.id); if(nm){nm.textContent=r.name;nm.title="Double-tap to rename";dblTap(nm,()=>{renameRelay(r.id);});}
    const l=$("#rl-"+r.id); if(l) l.className="led"+(r.on?" on":"");
    const st=$("#rs-"+r.id); if(st){st.textContent=r.on?"ON":"OFF";st.className="pill "+(r.on?"ok":"");}
    const pin=$("#rp-"+r.id); if(pin){pin.textContent="GPIO "+r.pin+" · "+(r.activeLow?"LOW":"HIGH");pin.className="pill";}
    const ac=$("#ra-"+r.id); if(ac) ac.textContent=(r.act||0)+" active";
    const sum=$("#sum-"+r.id);
    if(sum){sum.innerHTML="";let n=0;r.sched.forEach((s,i)=>{if(s.en){n++;sum.appendChild(chip((i+1)+" "+hm(s.start)+"→"+hm(s.stop),s.now));}});if(!n)sum.appendChild(chip("no schedules",false));}
  });
  const total=R.reduce((a,r)=>a+r.sched.filter(s=>s.en).length,0);
  const el=$("#schedCount"); if(el) el.textContent=total+" enabled schedules";
}
const renameRelay=(id)=>{
  const r=R[id];if(!r)return;
  const n=window.prompt("Relay "+(id+1)+" name (1-24 characters)",r.name);
  if(n===null)return;
  const v=n.trim().slice(0,24);
  if(!v||v===r.name)return;
  api("/api/relay/name",{method:"POST",body:{id:id,name:v}}).then(()=>{r.name=v;renderCards();const t=$("#edTitle");if(t&&cur.id===id)t.textContent=v+" — schedules";toast("Relay renamed","ok");}).catch(e=>toast(e.message,"err"));
}
const load=async ()=>{try{const j=await api("/api/relays");R=j.relays;renderCards();}catch(e){toast(e.message,"err");}}
const fillMonths=()=>{const w=$("#edMonths");w.innerHTML="";MONS.forEach((m,i)=>{const b=document.createElement("button");b.type="button";b.className="tg";b.textContent=m;b.onclick=()=>{draft.months^=(1<<i);syncForm();};w.appendChild(b);});}
const fillDow=()=>{const w=$("#edDow");w.innerHTML="";DOWS.forEach((d,i)=>{const b=document.createElement("button");b.type="button";b.className="tg";b.textContent=d;b.onclick=()=>{draft.dow^=(1<<i);syncForm();};w.appendChild(b);});}
const fillCal=()=>{const w=$("#edCal");w.innerHTML="";DOWS.forEach(d=>{const s=document.createElement("div");s.className="dow";s.textContent=d[0];w.appendChild(s);});for(let d=1;d<=31;d++){const b=document.createElement("button");b.type="button";b.textContent=d;b.onclick=()=>{draft.dom^=(1<<(d-1));syncForm();};w.appendChild(b);}}
const syncForm=()=>{
  $("#edEn").checked=!!draft.en;
  $("#edSh").value=+draft.start.slice(0,2);$("#edSm").value=+draft.start.slice(3,5);$("#edSs").value=+draft.start.slice(6,8);
  $("#edEh").value=+draft.stop.slice(0,2);$("#edEm").value=+draft.stop.slice(3,5);$("#edEs").value=+draft.stop.slice(6,8);
  $("#edOvn").checked=!!(draft.flags&1);$("#edIncl").checked=!!(draft.flags&2);$("#edTailOn").checked=!!(draft.flags&4);
  $("#edTail").value=draft.tail||0;
  $("#edComb").value=String(draft.comb);$("#edLeap").value=String(draft.leap);
  $("#edVf").value=(draft.flags&8)?String(2000+draft.vfy).padStart(4,"0")+"-"+pad(draft.vfm)+"-"+pad(draft.vfd):"";
  $("#edVt").value=(draft.flags&16)?String(2000+draft.vty).padStart(4,"0")+"-"+pad(draft.vtm)+"-"+pad(draft.vtd):"";
  $$("#edMonths .tg").forEach((b,i)=>b.className="tg"+((draft.months>>i)&1?" on":""));
  $$("#edDow .tg").forEach((b,i)=>b.className="tg"+((draft.dow>>i)&1?" on":""));
  $$("#edCal button").forEach((b,i)=>b.className=((draft.dom>>i)&1)?"on":"");
  $("#edState").textContent=draft.en?"ENABLED":"disabled";
  $("#edState").className="pill "+(draft.en?"ok":"");
  $("#edSummary").textContent=summary(draft);
}
const summary=(s)=>{
  let out=s.start.slice(0,5)+" → "+s.stop.slice(0,5);
  if(s.flags&1) out+=" (+1 day)";
  if(s.flags&4) out+=" +"+s.tail+"s tail";
  const f=[];if(s.months)f.push("months");if(s.dom)f.push("days");if(s.dow)f.push("weekdays");if(s.leap)f.push(s.leap==1?"leap":"non-leap");
  if(f.length) out+=" · "+f.join((s.comb?" OR ":" & "));
  return out;
}
const slotList=()=>{
  const w=$("#slots");w.innerHTML="";
  draft.sched.forEach((s,i)=>{
    const b=document.createElement("div");b.className="slot"+(s.en?" on":"")+(i===cur.slot?" sel":"");
    b.innerHTML="<span>S"+(i+1)+"</span><span>"+(s.en?hm(s.start)+"→"+hm(s.stop):"off")+"</span>";
    b.onclick=()=>{pullForm();cur.slot=i;openSlot();};
    w.appendChild(b);
  });
}
const openSlot=()=>{draft=JSON.parse(JSON.stringify(R[cur.id].sched[cur.slot]));draft.sched=R[cur.id].sched;syncForm();slotList();}
const pullForm=()=>{
  if(!draft)return;
  draft.en=$("#edEn").checked?1:0;
  draft.start=pad(num($("#edSh").value,0)%24)+":"+pad(num($("#edSm").value,0)%60)+":"+pad(num($("#edSs").value,0)%60);
  draft.stop=pad(num($("#edEh").value,0)%24)+":"+pad(num($("#edEm").value,0)%60)+":"+pad(num($("#edEs").value,0)%60);
  draft.tail=Math.max(0,Math.min(86400,num($("#edTail").value,0)));
  draft.flags=(draft.flags&6)|(($("#edOvn").checked)?1:0)|(($("#edIncl").checked)?2:0)|(($("#edTailOn").checked)?4:0);
  const vf=$("#edVf").value, vt=$("#edVt").value, dv=$("#edVf"), dt=$("#edVt");
  if(vf){const a=vf.split("-");draft.flags|=8;draft.vfy=Math.max(0,Math.min(255,+a[0]-2000));draft.vfm=+a[1];draft.vfd=+a[2];}else draft.flags&=~8;
  if(vt){const a=vt.split("-");draft.flags|=16;draft.vty=Math.max(0,Math.min(255,+a[0]-2000));draft.vtm=+a[1];draft.vtd=+a[2];}else draft.flags&=~16;
  draft.comb=num($("#edComb").value,0);draft.leap=num($("#edLeap").value,0);
  R[cur.id].sched[cur.slot]=JSON.parse(JSON.stringify(draft));
  delete R[cur.id].sched[cur.slot].sched;
}
const saveRelay=async ()=>{
  try{
    const r=R[cur.id];
    const body={id:r.id,name:r.name,activeLow:r.activeLow,enabled:r.enabled?1:0,sched:r.sched.map(s=>{const c=Object.assign({},s);delete c.sched;delete c.now;return c;})};
    await api("/api/relay/save",{method:"POST",body:body});
    toast("Relay "+(r.id+1)+" saved","ok");
  }catch(e){toast(e.message,"err");}
}
const openEditor=(id)=>{
  cur.id=id;cur.slot=0;
  draft=JSON.parse(JSON.stringify(R[id].sched[0]));draft.sched=R[id].sched;
  $("#edTitle").textContent=R[id].name+" — schedules";
  $("#edPin").textContent="GPIO "+R[id].pin+" · "+(R[id].activeLow?"active LOW":"active HIGH");
  $("#edRelayEn").checked=!!R[id].enabled;
  fillMonths();fillDow();fillCal();
  openModal("editModal");openSlot();
}
document.addEventListener("DOMContentLoaded",()=>{
  load();
  $("#relayGrid").addEventListener("click",e=>{
    const b=e.target.closest("button[data-act]");if(!b)return;
    const id=+b.dataset.id,act=b.dataset.act;
    if(act==="edit"){openEditor(id);return;}
    const mode=act==="on"?1:act==="off"?2:0;
    api("/api/relay/override",{method:"POST",body:{id:id,mode:mode}}).then(()=>{toast("Relay "+(id+1)+" "+(mode?"forced "+act.toUpperCase():"in AUTO"),"ok");pollStatus();}).catch(err=>toast(err.message,"err"));
  });
  $$("[data-close]").forEach(b=>b.onclick=()=>closeModal(b.dataset.close));
  $("#allAuto").onclick=()=>confirmBox("Set all 16 relays back to automatic schedule control?").then(v=>{if(v)api("/api/gpio/global",{method:"POST",body:{action:"auto"}}).then(()=>{toast("All relays in AUTO","ok");pollStatus();}).catch(e=>toast(e.message,"err"));});
  $("#edRelayEn").onchange=()=>{R[cur.id].enabled=$("#edRelayEn").checked?1:0;};
  $("#edSave").onclick=async()=>{pullForm();slotList();await saveRelay();};
  $("#edClear").onclick=()=>{draft.en=0;draft.months=0;draft.dom=0;draft.dow=0;draft.flags=0;draft.tail=0;draft.leap=0;draft.comb=0;syncForm();};
  $("#edCalAll").onclick=()=>{draft.dom=0x7FFFFFFF;syncForm();};
  $("#edCalNone").onclick=()=>{draft.dom=0;syncForm();};
  ["edEn","edSh","edSm","edSs","edEh","edEm","edEs","edOvn","edIncl","edTailOn","edTail","edComb","edLeap","edVf","edVt"].forEach(id=>{const el=$("#"+id);if(el)el.addEventListener("change",()=>{pullForm();syncForm();});});
  window.addEventListener("rh:status",e=>{
    const s=e.detail;
    if(!R.length)return;
    s.relays.forEach(x=>{const r=R.find(y=>y.id===x.id);if(r){r.on=x.on;r.act=x.act;r.override=x.ovr;}});
    renderCards();
  });
});
)JS");
  sendPage();
}

void handleGpioPage() {
  if (!webAuth()) return;
  pageBegin("GPIO", "gpio");
  gPage += F("<h1>GPIO management</h1><p class=\"sub\">Bulk polarity, global pin levels and per-relay overrides. "
             "GPIO 1 and 3 are relay outputs here, so the debug UART is mapped to GPIO16/17.</p>"
             "<div class=\"card\" style=\"margin-bottom:14px\"><div class=\"row between\">"
             "<div><h3>Global actions</h3><div class=\"dim small\">Force every output, regardless of schedules.</div></div>"
             "<div class=\"row\">"
             "<button class=\"btn sm\" data-global=\"activelow\">All active LOW</button>"
             "<button class=\"btn sm\" data-global=\"activehigh\">All active HIGH</button>"
             "<button class=\"btn sm\" data-global=\"high\">Set all HIGH</button>"
             "<button class=\"btn sm\" data-global=\"low\">Set all LOW</button>"
             "<button class=\"btn sm ghost\" data-global=\"auto\">All AUTO</button>"
             "</div></div><div class=\"warnbox\" style=\"margin-bottom:0\">"
             "Global HIGH/LOW force the electrical pin level. With active-LOW wiring, LOW energises the relay.</div></div>"
             "<div class=\"card\"><table class=\"tbl\"><thead><tr>"
             "<th>#</th><th>Name</th><th>GPIO</th><th>Polarity</th><th>Pin</th><th>Relay</th><th>Override</th><th style=\"text-align:right\">Actions</th>"
             "</tr></thead><tbody id=\"gpioBody\"></tbody></table></div>");
  pageEnd();
  pageScript(R"JS(
let R=[];
const pol=(r)=>{return r.pinLevel==1?"HIGH":"LOW";}
const state=(r)=>{return r.on?"ON":"OFF";}
const draw=()=>{
  const tb=$("#gpioBody");tb.innerHTML="";
  R.forEach(r=>{
    const tr=document.createElement("tr");
    tr.innerHTML="<td class=\"mono\">"+(r.id+1)+"</td><td>"+esc(r.name)+"</td><td class=\"mono\">"+r.pin+"</td>"+
      "<td><select data-pol=\""+r.id+"\"><option value=\"1\""+(r.activeLow?" selected":"")+">Active LOW</option><option value=\"0\""+(!r.activeLow?" selected":"")+">Active HIGH</option></select></td>"+
      "<td><span class=\"pill "+(r.pinLevel?"":"warn")+"\">"+pol(r)+"</span></td>"+
      "<td><span class=\"pill "+(r.on?"ok":"")+"\">"+state(r)+"</span></td>"+
      "<td><span class=\"pill "+(r.override?"warn":"")+"\">"+(r.override==1?"FORCED HIGH":r.override==2?"FORCED LOW":"AUTO")+"</span></td>"+
      "<td style=\"text-align:right\"><div class=\"row\" style=\"justify-content:flex-end;gap:4px\">"+
      "<button class=\"btn sm\" data-set=\"1\" data-id=\""+r.id+"\">Set HIGH</button>"+
      "<button class=\"btn sm\" data-set=\"2\" data-id=\""+r.id+"\">Set LOW</button>"+
      "<button class=\"btn sm ghost\" data-set=\"0\" data-id=\""+r.id+"\">Remove</button></div></td>";
    tb.appendChild(tr);
  });
  tb.querySelectorAll("[data-set]").forEach(b=>b.onclick=()=>act(+b.dataset.id,+b.dataset.set));
  tb.querySelectorAll("[data-pol]").forEach(s=>s.onchange=()=>{
    api("/api/gpio/relay",{method:"POST",body:{id:+s.dataset.pol,activeLow:+s.value}}).then(()=>{toast("Polarity updated","ok");pollStatus();}).catch(e=>toast(e.message,"err"));
  });
}
const act=async (id,mode)=>{
  try{await api("/api/gpio/relay",{method:"POST",body:{id:id,mode:mode}});toast("Relay "+(id+1)+" "+(mode==1?"set HIGH":mode==2?"set LOW":"override removed"),"ok");pollStatus();}catch(e){toast(e.message,"err");}
}
document.addEventListener("DOMContentLoaded",()=>{
  const load=async()=>{try{R=(await api("/api/relays")).relays;draw();}catch(e){toast(e.message,"err");}};
  load();
  $$("[data-global]").forEach(b=>b.onclick=async()=>{
    const a=b.dataset.global;
    if((a==="high"||a==="low")&&!(await confirmBox("Force ALL 16 outputs "+(a==="high"?"HIGH":"LOW")+"?")))return;
    try{await api("/api/gpio/global",{method:"POST",body:{action:a}});toast("Global action applied","ok");pollStatus();}catch(e){toast(e.message,"err");}
  });
  window.addEventListener("rh:status",e=>{const s=e.detail;if(!R.length)return;s.relays.forEach(x=>{const r=R.find(y=>y.id===x.id);if(r){r.on=x.on;r.pinLevel=x.pin;r.override=x.ovr;}});draw();});
});
)JS");
  sendPage();
}

void handleTimePage() {
  if (!webAuth()) return;
  pageBegin("Time", "time");
  char b[512];

  gPage += F("<h1>Time management</h1><p class=\"sub\">Internal RTC, DS3231, NTP rotation/fallback, manual GMT offset or automatic timezone, daylight saving and drift compensation.</p>"
             "<div class=\"grid g2\">"
             "<div class=\"card\"><h3>Clock</h3><div class=\"big\" id=\"tpClock\">--:--:--</div>"
             "<div class=\"dim small\" id=\"tpDate\">&mdash;</div>"
             "<div class=\"row\" style=\"margin-top:10px\"><span class=\"pill\" id=\"tpSrc\">&mdash;</span>"
             "<span class=\"pill\" id=\"tpRtc\">&mdash;</span><span class=\"pill\" id=\"tpValid\">&mdash;</span></div>"
             "<div class=\"kv\" style=\"margin-top:12px\">"
             "<span class=\"k\">UTC offset</span><span class=\"v\" id=\"tpOff\">&mdash;</span>"
             "<span class=\"k\">Last sync</span><span class=\"v\" id=\"tpLast\">&mdash;</span>"
             "<span class=\"k\">NTP ok / fail</span><span class=\"v\" id=\"tpCount\">&mdash;</span>"
             "<span class=\"k\">Next NTP try</span><span class=\"v\" id=\"tpNext\">&mdash;</span></div>"
             "<hr class=\"sep\"><div class=\"row\">"
             "<button class=\"btn primary\" id=\"btnBrowser\">Sync from this browser</button>"
             "<label class=\"switch\" style=\"margin:0\"><input type=\"checkbox\" id=\"useBrowserTz\"> also apply browser UTC offset</label>"
             "</div><div class=\"hint\">Useful when NTP is unreachable. Sets both the internal clock and the DS3231.</div></div>"
             "<div class=\"card\"><h3>NTP</h3>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"ntpEn\"");
  if (gCfg.time.ntpEnabled) gPage += F(" checked");
  gPage += F("> Enable NTP synchronisation</label>"
             "<label class=\"f\"><span>Servers (tried in rotation, with fallback)</span>"
             "<input id=\"ntp0\" maxlength=\"63\" placeholder=\"pool.ntp.org\">"
             "<input id=\"ntp1\" maxlength=\"63\" placeholder=\"time.google.com\" style=\"margin-top:6px\">"
             "<input id=\"ntp2\" maxlength=\"63\" placeholder=\"time.cloudflare.com\" style=\"margin-top:6px\">"
             "<input id=\"ntp3\" maxlength=\"63\" placeholder=\"time.nist.gov (optional)\" style=\"margin-top:6px\"></label>"
             "<label class=\"f\"><span>Automatic sync interval (1 &ndash; 1440 minutes = 24 h)</span>"
             "<input id=\"ntpInt\" type=\"number\" min=\"1\" max=\"1440\" step=\"1\"></label>"
             "<div class=\"row\"><button class=\"btn sm ghost\" data-int=\"5\">5m</button><button class=\"btn sm ghost\" data-int=\"15\">15m</button>"
             "<button class=\"btn sm ghost\" data-int=\"60\">1h</button><button class=\"btn sm ghost\" data-int=\"360\">6h</button>"
             "<button class=\"btn sm ghost\" data-int=\"1440\">24h</button>"
             "<button class=\"btn sm primary\" id=\"btnNtpNow\" style=\"margin-left:auto\">Sync now</button></div>"
             "<div class=\"hint\" id=\"ntpState\">Idle.</div></div>"
             "</div>");

  gPage += F("<div class=\"grid g2\" style=\"margin-top:14px\">"
             "<div class=\"card\"><h3>Timezone</h3>"
             "<label class=\"f\"><span>Mode</span><select id=\"tzMode\">"
             "<option value=\"0\">Manual GMT offset (with optional DST rule)</option>"
             "<option value=\"1\">Automatic POSIX timezone (DST included)</option></select></label>"
             "<div id=\"manualBox\">"
             "<label class=\"f\"><span>GMT offset in seconds (e.g. 28800 = UTC+8)</span>"
             "<input id=\"tzOff\" type=\"number\" min=\"-50400\" max=\"50400\" step=\"1\"></label>"
             "<div class=\"hint\" id=\"offHuman\"></div>"
             "<label class=\"f\" style=\"margin-top:10px\"><span>Daylight saving rule</span><select id=\"dstMode\">"
             "<option value=\"0\">No DST</option><option value=\"1\">United States (2nd Sun Mar &rarr; 1st Sun Nov)</option>"
             "<option value=\"2\">Europe (last Sun Mar &rarr; last Sun Oct)</option>"
             "<option value=\"3\">Australia (1st Sun Oct &rarr; 1st Sun Apr)</option>"
             "<option value=\"4\">New Zealand (last Sun Sep &rarr; 1st Sun Apr)</option>"
             "<option value=\"5\">Custom rule</option></select></label>"
             "<div id=\"dstCustom\" class=\"fieldset\">"
             "<label class=\"f\"><span>DST offset (s)</span><input id=\"dstSec\" type=\"number\" min=\"0\" max=\"7200\"></label>"
             "<label class=\"f\"><span>Start month</span><input id=\"dsM\" type=\"number\" min=\"1\" max=\"12\"></label>"
             "<label class=\"f\"><span>Start week (0=1st..4=last)</span><input id=\"dsW\" type=\"number\" min=\"0\" max=\"4\"></label>"
             "<label class=\"f\"><span>Start weekday (0=Sun)</span><input id=\"dsD\" type=\"number\" min=\"0\" max=\"6\"></label>"
             "<label class=\"f\"><span>Start hour (standard)</span><input id=\"dsH\" type=\"number\" min=\"0\" max=\"23\"></label>"
             "<label class=\"f\"><span>End month</span><input id=\"deM\" type=\"number\" min=\"1\" max=\"12\"></label>"
             "<label class=\"f\"><span>End week</span><input id=\"deW\" type=\"number\" min=\"0\" max=\"4\"></label>"
             "<label class=\"f\"><span>End weekday</span><input id=\"deD\" type=\"number\" min=\"0\" max=\"6\"></label>"
             "<label class=\"f\"><span>End hour (daylight)</span><input id=\"deH\" type=\"number\" min=\"0\" max=\"23\"></label>"
             "</div></div>"
             "<div id=\"autoBox\">"
             "<label class=\"f\"><span>Zone</span><select id=\"autoZone\">");
  for (uint8_t i = 0; i < kTzZoneCount; i++) {
    snprintf(b, sizeof(b), "<option value=\"%u\">%s</option>", (unsigned)i, kTzZones[i].name);
    gPage += b;
  }
  gPage += F("<option value=\"255\">Custom POSIX string</option></select></label>"
             "<label class=\"f\"><span>Custom POSIX TZ (used when Zone = custom)</span>"
             "<input id=\"posix\" maxlength=\"47\" placeholder=\"PHT-8\"></label>"
             "<div class=\"hint\">POSIX strings include DST, e.g. <span class=\"mono\">EST5EDT,M3.2.0,M11.1.0</span>.</div></div>"
             "<hr class=\"sep\"><h3>Drift compensation</h3>"
             "<label class=\"f\"><span>ppm &times; 100 (100 = 1.00 ppm, positive = device clock runs fast)</span>"
             "<input id=\"drift\" type=\"number\" min=\"-10000\" max=\"10000\" step=\"1\"></label>"
             "<div class=\"hint\">Applied in sub-millisecond steps; default is a fixed 1.00 ppm = 100 "
             "(&asymp;0.09 s/day). One second per day is 11.574 ppm &rarr; enter <span class=\"mono\">1157</span>.</div>"
             "</div>"
             "<div class=\"card\"><h3>DS3231 RTC</h3>"
             "<div class=\"kv\"><span class=\"k\">Bus</span><span class=\"v\">I2C SDA GPIO21 / SCL GPIO22</span>"
             "<span class=\"k\">Detected</span><span class=\"v\" id=\"rtcDet\">&mdash;</span>"
             "<span class=\"k\">Lost power flag</span><span class=\"v\" id=\"rtcLost\">&mdash;</span>"
             "<span class=\"k\">Temperature</span><span class=\"v\" id=\"rtcTemp\">&mdash;</span>"
             "<span class=\"k\">RTC time</span><span class=\"v\" id=\"rtcNow\">&mdash;</span></div>"
             "<hr class=\"sep\"><label class=\"switch\"><input type=\"checkbox\" id=\"useRtc\"> Use DS3231 as time reference</label>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"syncOnRe\"> Sync NTP immediately when the station reconnects</label>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"allowBr\"> Allow browser time sync</label>"
             "<div class=\"hint\">While WiFi is down the internal clock is refreshed from the DS3231 every few seconds, and on every reconnect attempt.</div>"
             "</div></div>"
             "<div class=\"row\" style=\"margin-top:14px\"><button class=\"btn primary\" id=\"btnApply\">Apply all time settings</button>"
             "<span class=\"hint\">Saving is debounced in NVS to reduce flash wear.</span></div>");

  pageEnd();
  pageScript(R"JS(
let T=null;
const humanOff=(o)=>{const sg=o<0?"-":"+",a=Math.abs(o);return "UTC"+sg+pad2(Math.floor(a/3600))+":"+pad2(Math.floor(a%3600/60))+":"+pad2(a%60);}
const fillAuto=(sel)=>{$("#autoZone").value=String(sel);}
const syncForm=(t)=>{
  $("#ntpEn").checked=!!t.ntpOn;$("#ntpInt").value=t.interval;
  $("#tzMode").value=String(t.tzMode);$("#tzOff").value=t.tzOffset;$("#offHuman").textContent=humanOff(t.tzOffset);
  $("#dstMode").value=String(t.dstMode);$("#dstSec").value=t.dstSec;
  $("#dsM").value=t.dsM;$("#dsW").value=t.dsW;$("#dsD").value=t.dsD;$("#dsH").value=t.dsH;
  $("#deM").value=t.deM;$("#deW").value=t.deW;$("#deD").value=t.deD;$("#deH").value=t.deH;
  fillAuto(t.autoZone);$("#posix").value=t.posix||"";
  $("#drift").value=t.drift;
  $("#useRtc").checked=!!t.useRtc;$("#syncOnRe").checked=!!t.syncOnReconnect;$("#allowBr").checked=!!t.allowBrowser;
  toggleBoxes();
}
const toggleBoxes=()=>{
  const man=$("#tzMode").value==="0";
  $("#manualBox").style.display=man?"":"none";
  $("#autoBox").style.display=man?"none":"";
  $("#dstCustom").style.display=(man&&$("#dstMode").value==="5")?"grid":"none";
}
const load=async ()=>{
  try{
    const t=await api("/api/time");
    T=t;
    syncForm(t);
    $("#rtcDet").textContent=t.rtcOk?"yes":"no";
    $("#rtcLost").textContent=t.rtcOk?(t.rtcLost?"yes (battery?)":"no"):"—";
    $("#rtcTemp").textContent=t.rtcTemp===null?"—":(t.rtcTemp+" °C");
    $("#rtcNow").textContent=t.rtcNow||"—";
    $("#tpCount").textContent=t.syncOk+" / "+t.syncFail;
    $("#tpLast").textContent=t.lastSyncStr||"never";
    $("#ntpState").textContent=t.ntpOn?("Last server: "+(t.lastNtp||"—")+" · last result: "+(t.lastResult||"—")):"NTP disabled.";
    T.nextAt=(performance.now()/1000)+(t.nextNtpIn||0);
  }catch(e){toast(e.message,"err");}
}
const tick=()=>{
  if(!T)return;
  const d=new Date(nowLocalMs());
  $("#tpClock").textContent=pad2(d.getUTCHours())+":"+pad2(d.getUTCMinutes())+":"+pad2(d.getUTCSeconds());
  $("#tpDate").textContent=["SUN","MON","TUE","WED","THU","FRI","SAT"][d.getUTCDay()]+" "+pad2(d.getUTCMonth()+1)+"/"+pad2(d.getUTCDate())+"/"+d.getUTCFullYear();
  $("#tpOff").textContent=humanOff(RH.time.offset);
  $("#tpSrc").textContent=RH.time.valid?RH.time.src:"NOT SET";
  $("#tpSrc").className="pill "+(RH.time.valid?"ok":"err");
  $("#tpValid").textContent=RH.time.valid?"valid":"invalid";
  $("#tpValid").className="pill "+(RH.time.valid?"ok":"warn");
  $("#tpRtc").textContent=T&&T.rtcOk?"RTC online":"RTC absent";
  $("#tpRtc").className="pill "+(T&&T.rtcOk?"acc":"warn");
  if(T&&T.nextAt){$("#tpNext").textContent=fmtDur(T.nextAt-performance.now()/1000);}
}
const apply=async ()=>{
  const body={ntpEnabled:$("#ntpEn").checked?1:0,ntpInterval:num($("#ntpInt").value,60),
    tzMode:num($("#tzMode").value,0),tzOffset:num($("#tzOff").value,28800),dstMode:num($("#dstMode").value,0),
    dstSec:num($("#dstSec").value,3600),dsM:num($("#dsM").value,3),dsW:num($("#dsW").value,1),dsD:num($("#dsD").value,0),dsH:num($("#dsH").value,2),
    deM:num($("#deM").value,11),deW:num($("#deW").value,0),deD:num($("#deD").value,0),deH:num($("#deH").value,2),
    autoZone:num($("#autoZone").value,0),posix:$("#posix").value,drift:num($("#drift").value,100),
    useRtc:$("#useRtc").checked?1:0,syncOnReconnect:$("#syncOnRe").checked?1:0,
    allowBrowser:$("#allowBr").checked?1:0,
    ntp:[$("#ntp0").value,$("#ntp1").value,$("#ntp2").value,$("#ntp3").value]};
  await api("/api/time/config",{method:"POST",body:body});
  toast("Time settings applied","ok");await syncClock();load();
}
document.addEventListener("DOMContentLoaded",()=>{
  load();
  api("/api/time").then(t=>{$("#ntp0").value=t.ntp[0]||"";$("#ntp1").value=t.ntp[1]||"";$("#ntp2").value=t.ntp[2]||"";$("#ntp3").value=t.ntp[3]||"";});
  $("#tzMode").onchange=toggleBoxes;$("#dstMode").onchange=toggleBoxes;
  $("#tzOff").oninput=()=>{$("#offHuman").textContent=humanOff(num($("#tzOff").value,0));};
  $$("[data-int]").forEach(b=>b.onclick=()=>{$("#ntpInt").value=b.dataset.int;});
  $("#btnApply").onclick=async()=>{try{await apply();}catch(e){toast(e.message,"err");}};
  $("#btnNtpNow").onclick=async()=>{try{await api("/api/time/sync",{method:"POST",body:{}});toast("NTP sync requested","ok");}catch(e){toast(e.message,"err");}};
  $("#btnBrowser").onclick=async()=>{
    try{
      const body={epoch:Math.floor(Date.now()/1000)};
      if($("#useBrowserTz").checked) body.offset=-new Date().getTimezoneOffset()*60;
      await api("/api/time/set",{method:"POST",body:body});
      toast("Clock set from browser","ok");await syncClock();load();
    }catch(e){toast(e.message,"err");}
  };
  setInterval(tick,300);setInterval(load,15000);
});
)JS");
  sendPage();
}

void handleApPage() {
  if (!webAuth()) return;
  pageBegin("WiFi AP", "ap");
  char ss[160], pp[160];
  hEsc(gCfg.ap.ssid, ss, sizeof(ss));
  hEsc(gCfg.ap.pass, pp, sizeof(pp));
  char b[220];

  gPage += F("<h1>WiFi access point</h1><p class=\"sub\">Soft-AP, captive portal and automatic fallback when the station link is down.</p>"
             "<div class=\"grid g2\"><div class=\"card\"><h3>Status</h3>"
             "<div class=\"kv\"><span class=\"k\">AP state</span><span class=\"v\" id=\"apState\">&mdash;</span>"
             "<span class=\"k\">SSID</span><span class=\"v\" id=\"apSsid\">&mdash;</span>"
             "<span class=\"k\">Address</span><span class=\"v\" id=\"apIp\">&mdash;</span>"
             "<span class=\"k\">Clients</span><span class=\"v\" id=\"apClients\">&mdash;</span>"
             "<span class=\"k\">Captive DNS</span><span class=\"v\" id=\"apDns\">&mdash;</span>"
             "<span class=\"k\">Fallback mode</span><span class=\"v\" id=\"apFb\">&mdash;</span></div>"
             "<hr class=\"sep\"><h3>Connected clients</h3><div class=\"small mono\" id=\"apList\">none</div></div>"
             "<div class=\"card\"><h3>Settings</h3>"
             "<form id=\"apForm\" onsubmit=\"return false\">"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"apEn\"> Enable access point</label>"
             "<label class=\"f\"><span>SSID</span><input id=\"apSsidIn\" maxlength=\"32\"></label>"
             "<label class=\"f\"><span>Password (8+ chars, empty = open)</span><input id=\"apPass\" maxlength=\"64\" type=\"password\"></label>"
             "<div class=\"fieldset\">"
             "<label class=\"f\"><span>Channel (1-13)</span><input id=\"apCh\" type=\"number\" min=\"1\" max=\"13\"></label>"
             "<label class=\"f\"><span>Max clients (1-4)</span><input id=\"apMax\" type=\"number\" min=\"1\" max=\"4\"></label>"
             "<label class=\"f\"><span>Fallback delay (seconds)</span><input id=\"apFbSec\" type=\"number\" min=\"10\" max=\"3600\"></label>"
             "<label class=\"f\"><span>Hidden SSID</span><select id=\"apHidden\"><option value=\"0\">No</option><option value=\"1\">Yes</option></select></label></div>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"apCap\"> Captive portal (DNS redirect)</label>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"apFallback\"> Start AP when the station cannot connect</label>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"apAutoOff\"> Stop AP while the station is connected</label>"
             "<div class=\"row\" style=\"margin-top:8px\"><button class=\"btn primary\" id=\"apSave\">Save &amp; apply</button>"
             "<a class=\"btn ghost\" href=\"/portal\" target=\"_blank\">Open portal preview</a></div>"
             "<div class=\"warnbox\">Changing AP settings restarts the access point. If you are connected through it, reconnect afterwards.</div>"
             "</form></div></div>");
  pageEnd();
  pageScript(R"JS(
let A=null;
const load=async ()=>{
  try{
    const a=await api("/api/wifi/ap");A=a;
    $("#apEn").checked=!!a.enabled;$("#apSsidIn").value=a.ssid;$("#apPass").value=a.pass;
    $("#apCh").value=a.channel;$("#apMax").value=a.maxConn;$("#apFbSec").value=a.fallbackSec;
    $("#apHidden").value=String(a.hidden);$("#apCap").checked=!!a.captive;
    $("#apFallback").checked=!!a.fallback;$("#apAutoOff").checked=!!a.autoOff;
    $("#apState").textContent=a.active?"running":"stopped";
    $("#apSsid").textContent=a.ssid;$("#apIp").textContent=a.ip||"—";
    $("#apClients").textContent=a.clients;$("#apDns").textContent=a.dns?"running":"stopped";
    $("#apFb").textContent=a.fallbackActive?"ACTIVE":"idle";
    const l=await api("/api/wifi/ap/clients");
    $("#apList").innerHTML=(l.clients&&l.clients.length)?l.clients.map(c=>esc(c.mac)+"  "+c.rssi+" dBm").join("<br>"):"none";
  }catch(e){toast(e.message,"err");}
}
document.addEventListener("DOMContentLoaded",()=>{
  load();setInterval(load,5000);
  $("#apSave").onclick=async()=>{
    const btn=$("#apSave");busy(btn,true);
    try{
      await api("/api/wifi/ap/save",{method:"POST",body:{
        enabled:$("#apEn").checked?1:0,ssid:$("#apSsidIn").value,pass:$("#apPass").value,
        channel:num($("#apCh").value,6),maxConn:num($("#apMax").value,4),hidden:num($("#apHidden").value,0),
        captive:$("#apCap").checked?1:0,fallback:$("#apFallback").checked?1:0,autoOff:$("#apAutoOff").checked?1:0,
        fallbackSec:num($("#apFbSec").value,60)}});
      toast("Access point settings applied","ok");load();
    }catch(e){toast(e.message,"err");}finally{busy(btn,false);}
  };
});
)JS");
  sendPage();
}

void handleStaPage() {
  if (!webAuth()) return;
  pageBegin("WiFi STA", "sta");
  char v[6][160];
  hEsc(gCfg.sta.ssid, v[0], sizeof(v[0]));
  hEsc(gCfg.sta.pass, v[1], sizeof(v[1]));
  hEsc(gCfg.sta.ssid2, v[2], sizeof(v[2]));
  hEsc(gCfg.sta.pass2, v[3], sizeof(v[3]));
  hEsc(gCfg.sta.hostname, v[4], sizeof(v[4]));
  char b[256];

  gPage += F("<h1>WiFi station</h1><p class=\"sub\">Primary and backup network, DHCP or static addressing, async scanning and 1-second reconnect attempts.</p>"
             "<div class=\"grid g2\"><div class=\"card\"><h3>Status</h3>"
             "<div class=\"kv\"><span class=\"k\">State</span><span class=\"v\" id=\"stState\">&mdash;</span>"
             "<span class=\"k\">SSID</span><span class=\"v\" id=\"stSsid\">&mdash;</span>"
             "<span class=\"k\">IP address</span><span class=\"v\" id=\"stIp\">&mdash;</span>"
             "<span class=\"k\">Gateway</span><span class=\"v\" id=\"stGw\">&mdash;</span>"
             "<span class=\"k\">RSSI</span><span class=\"v\" id=\"stRssi\">&mdash;</span>"
             "<span class=\"k\">mDNS</span><span class=\"v\" id=\"stMdns\">&mdash;</span>"
             "<span class=\"k\">Reconnect attempts</span><span class=\"v\" id=\"stAtt\">&mdash;</span>"
             "<span class=\"k\">Last disconnect reason</span><span class=\"v\" id=\"stWhy\">&mdash;</span></div>"
             "<hr class=\"sep\"><h3>Scan (non-blocking)</h3>"
             "<div class=\"row\"><button class=\"btn sm\" id=\"btnScan\">Scan networks</button><span class=\"pill\" id=\"scanState\">idle</span></div>"
             "<div class=\"scan\" id=\"scanList\" style=\"margin-top:8px\"></div></div>"
             "<div class=\"card\"><h3>Settings</h3><form onsubmit=\"return false\">"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"staEn\"> Enable station mode</label>"
             "<label class=\"f\"><span>Primary SSID</span><input id=\"ssid\" maxlength=\"32\"></label>"
             "<label class=\"f\"><span>Primary password</span><input id=\"pass\" maxlength=\"64\" type=\"password\"></label>"
             "<label class=\"f\"><span>Backup SSID (optional, used after repeated failures)</span><input id=\"ssid2\" maxlength=\"32\"></label>"
             "<label class=\"f\"><span>Backup password</span><input id=\"pass2\" maxlength=\"64\" type=\"password\"></label>"
             "<div class=\"fieldset\"><label class=\"f\"><span>Addressing</span><select id=\"dhcp\"><option value=\"1\">DHCP</option><option value=\"0\">Static</option></select></label>"
             "<label class=\"f\"><span>Hostname / mDNS</span><input id=\"host\" maxlength=\"31\"></label>"
             "<label class=\"f\"><span>IP address</span><input id=\"ip\" maxlength=\"15\" placeholder=\"192.168.1.50\"></label>"
             "<label class=\"f\"><span>Gateway</span><input id=\"gw\" maxlength=\"15\" placeholder=\"192.168.1.1\"></label>"
             "<label class=\"f\"><span>Subnet mask</span><input id=\"sn\" maxlength=\"15\" placeholder=\"255.255.255.0\"></label>"
             "<label class=\"f\"><span>DNS 1</span><input id=\"dns1\" maxlength=\"15\" placeholder=\"1.1.1.1\"></label>"
             "<label class=\"f\"><span>DNS 2</span><input id=\"dns2\" maxlength=\"15\" placeholder=\"8.8.8.8\"></label>"
             "<label class=\"f\"><span>Reconnect interval (s)</span><input id=\"recSec\" type=\"number\" min=\"1\" max=\"60\"></label></div>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"autoRec\"> Auto reconnect every interval</label>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"psave\"> WiFi power save (reduces range/latency)</label>"
             "<div class=\"row\" style=\"margin-top:8px\"><button class=\"btn primary\" id=\"btnSave\">Save &amp; connect</button>"
             "<button class=\"btn\" id=\"btnDisc\">Disconnect</button></div>"
             "<div class=\"warnbox\">If the ESP32 is currently your only link, saving a different network will drop this page.</div>"
             "</form></div></div>");
  pageEnd();
  pageScript(R"JS(
let S=null,scanTimer=null;
const load=async ()=>{
  try{
    const s=await api("/api/wifi/sta");S=s;
    $("#staEn").checked=!!s.enabled;$("#ssid").value=s.ssid;$("#pass").value=s.pass;
    $("#ssid2").value=s.ssid2;$("#pass2").value=s.pass2;$("#host").value=s.hostname;
    $("#dhcp").value=String(s.dhcp);$("#ip").value=s.ip;$("#gw").value=s.gw;$("#sn").value=s.sn;
    $("#dns1").value=s.dns1;$("#dns2").value=s.dns2;$("#recSec").value=s.reconnectSec;
    $("#autoRec").checked=!!s.autoReconnect;$("#psave").checked=!!s.powerSave;
    $("#stState").textContent=s.connected?"CONNECTED":(s.enabled?"connecting / retrying":"disabled");
    $("#stSsid").textContent=s.ssidCurrent||"—";$("#stIp").textContent=s.addr||"—";
    $("#stGw").textContent=s.gateway||"—";$("#stRssi").textContent=s.connected?(s.rssi+" dBm"):"—";
    $("#stMdns").textContent=s.mdns?("http://"+s.hostname+".local"):"off";
    $("#stAtt").textContent=s.attempts;$("#stWhy").textContent=s.lastReason||"—";
  }catch(e){toast(e.message,"err");}
}
const pollScan=async ()=>{
  try{
    const s=await api("/api/wifi/scan");
    const el=$("#scanList");
    if(s.status==="running"){$("#scanState").textContent="scanning…";$("#scanState").className="pill warn";return;}
    if(scanTimer){clearInterval(scanTimer);scanTimer=null;}
    $("#scanState").textContent=s.status==="done"?(s.networks.length+" found"):"idle";
    $("#scanState").className="pill "+(s.status==="done"?"ok":"");
    el.innerHTML="";
    (s.networks||[]).forEach(n=>{
      const b=document.createElement("button");b.type="button";
      b.innerHTML="<span class=\"row between\"><span>"+esc(n.ssid||"(hidden)")+(n.secure?" 🔒":"")+"</span><span class=\"dim\">"+n.rssi+" dBm · ch "+n.channel+"</span></span>";
      b.onclick=()=>{$("#ssid").value=n.ssid;toast("SSID filled in","ok");};
      el.appendChild(b);
    });
    if(!s.networks||!s.networks.length)el.innerHTML="<div class=\"hint\" style=\"padding:8px\">No networks reported.</div>";
  }catch(e){}
}
document.addEventListener("DOMContentLoaded",()=>{
  load();setInterval(load,5000);
  $("#btnScan").onclick=async()=>{
    try{await api("/api/wifi/scan",{method:"POST",body:{}});$("#scanState").textContent="scanning…";if(scanTimer)clearInterval(scanTimer);scanTimer=setInterval(pollScan,900);}catch(e){toast(e.message,"err");}
  };
  $("#btnSave").onclick=async()=>{
    const btn=$("#btnSave");busy(btn,true);
    try{
      await api("/api/wifi/sta/save",{method:"POST",body:{
        enabled:$("#staEn").checked?1:0,ssid:$("#ssid").value,pass:$("#pass").value,
        ssid2:$("#ssid2").value,pass2:$("#pass2").value,hostname:$("#host").value,
        dhcp:num($("#dhcp").value,1),ip:$("#ip").value,gw:$("#gw").value,sn:$("#sn").value,
        dns1:$("#dns1").value,dns2:$("#dns2").value,reconnectSec:num($("#recSec").value,1),
        autoReconnect:$("#autoRec").checked?1:0,powerSave:$("#psave").checked?1:0}});
      toast("Station settings applied","ok");await load();
    }catch(e){toast(e.message,"err");}finally{busy(btn,false);}
  };
  $("#btnDisc").onclick=async()=>{if(!(await confirmBox("Disconnect the station interface?")))return;try{await api("/api/wifi/sta/disconnect",{method:"POST",body:{}});toast("Disconnected","ok");load();}catch(e){toast(e.message,"err");}};
});
)JS");
  sendPage();
}

void handleSystemPage() {
  if (!webAuth()) return;
  pageBegin("System", "sys");
  gPage += F("<h1>System status</h1><p class=\"sub\">Memory, flash, storage, time and network diagnostics plus the live event log.</p>"
             "<div class=\"grid g4\">"
             "<div class=\"card\"><h3>Uptime</h3><div class=\"big\" id=\"sysUp\">&mdash;</div>"
             "<div class=\"dim small\" id=\"sysBoot\">&mdash;</div></div>"
             "<div class=\"card\"><h3>Free heap</h3><div class=\"big\" id=\"sysHeap\">&mdash;</div>"
             "<div class=\"bar\" style=\"margin:8px 0\"><i id=\"sysFragBar\" style=\"width:0\"></i></div>"
             "<div class=\"dim small\" id=\"sysFrag\">&mdash;</div></div>"
             "<div class=\"card\"><h3>Flash</h3><div class=\"big\" id=\"sysFlash\">&mdash;</div>"
             "<div class=\"dim small\" id=\"sysSketch\">&mdash;</div></div>"
             "<div class=\"card\"><h3>NVS writes</h3><div class=\"big\" id=\"sysWrites\">&mdash;</div>"
             "<div class=\"dim small\" id=\"sysNvs\">&mdash;</div></div>"
             "</div>"
             "<div class=\"grid g2\" style=\"margin-top:14px\">"
             "<div class=\"card\"><h3>Device</h3><div class=\"kv\" id=\"sysDev\"></div></div>"
             "<div class=\"card\"><h3>Network</h3><div class=\"kv\" id=\"sysNet\"></div></div>"
             "<div class=\"card\"><h3>Time &amp; RTC</h3><div class=\"kv\" id=\"sysTime\"></div></div>"
             "<div class=\"card\"><h3>Relay engine</h3><div class=\"kv\" id=\"sysRelay\"></div></div>"
             "</div>"
             "<div class=\"card\" style=\"margin-top:14px\"><div class=\"row between\">"
             "<h2>Event log</h2><div class=\"row\"><button class=\"btn sm ghost\" id=\"logClear\">Clear</button>"
             "<button class=\"btn sm ghost\" id=\"logRefresh\">Refresh</button></div></div>"
             "<div class=\"logs\" id=\"sysLogs\" style=\"margin-top:10px\">Loading&hellip;</div></div>"
             "<div class=\"card\" style=\"margin-top:14px\"><h3>Maintenance</h3>"
             "<div class=\"row\"><button class=\"btn\" id=\"btnSaveNow\">Flush config to NVS</button>"
             "<button class=\"btn\" id=\"btnReboot\">Restart</button>"
             "<button class=\"btn danger\" id=\"btnFactory\">Factory reset</button></div>"
             "<div class=\"hint\">Factory reset erases the configuration, relay names and schedules, then reboots. It can also be triggered by holding BOOT (GPIO0) for 5 seconds.</div></div>");
  pageEnd();
  pageScript(R"JS(
const kv=(el,rows)=>{el.innerHTML=rows.map(r=>'<span class="k">'+esc(r[0])+'</span><span class="v">'+esc(r[1])+'</span>').join("");}
const load=async ()=>{
  try{
    const s=await api("/api/system");
    $("#sysUp").textContent=fmtDur(s.uptime);
    $("#sysBoot").textContent="reset: "+s.resetReason;
    $("#sysHeap").textContent=(s.heap/1024).toFixed(0)+" KB";
    $("#sysFragBar").style.width=s.frag+"%";
    $("#sysFrag").textContent="largest block "+(s.largest/1024).toFixed(1)+" KB · fragmentation "+s.frag+"% · min free "+(s.minHeap/1024).toFixed(0)+" KB";
    $("#sysFlash").textContent=(s.flashSize/1048576).toFixed(1)+" MB";
    $("#sysSketch").textContent="sketch "+(s.sketchSize/1024).toFixed(0)+" KB of "+(s.sketchSpace/1024).toFixed(0)+" KB";
    $("#sysWrites").textContent=s.store.writes;
    $("#sysNvs").textContent="errors "+s.store.errors+" · pending "+(s.store.pending?"yes":"no");
    kv($("#sysDev"),[["Model",s.model],["Chip ID",s.chipId],["Cores",s.cores],["CPU",s.cpuMhz+" MHz"],["SDK",s.sdk],["Firmware",s.fw+" "+s.version]]);
    kv($("#sysNet"),[["STA",s.sta?(s.ip+" ("+s.rssi+" dBm)"):"disconnected"],["AP",s.ap?("up · "+s.apClients+" clients"):"down"],["Hostname",s.hostname],["mDNS",s.mdns?"active":"off"],["MAC",s.mac]]);
    kv($("#sysTime"),[["Source",s.time.src],["Valid",s.time.valid?"yes":"no"],["Local time",s.time.local],["UTC offset",s.time.offsetStr],["RTC",s.time.rtcOk?("DS3231 · "+s.time.rtcTemp+" °C"):"absent"],["Last sync",s.time.lastSync],["NTP ok/fail",s.time.syncOk+" / "+s.time.syncFail]]);
    kv($("#sysRelay"),[["Relays energised",s.relaysOn+" / 16"],["Pin transitions",s.transitions],["Evaluation period","200 ms"],["Schedule slots",(16*8)+" (128)"],["Time gated",s.time.valid?"no":"yes (outputs forced off)"],["Config writes",s.store.writes]]);
  }catch(e){toast(e.message,"err");}
}
const logs=async ()=>{
  try{
    const j=await api("/api/logs?n=40");const el=$("#sysLogs");el.innerHTML="";
    (j.logs||[]).slice().reverse().forEach(l=>{const d=document.createElement("div");d.className=l.level;d.textContent=l.time+"  "+l.level+"  "+l.msg;el.appendChild(d);});
    if(!j.logs||!j.logs.length)el.textContent="No events.";
  }catch(e){}
}
document.addEventListener("DOMContentLoaded",()=>{
  load();logs();setInterval(load,3000);setInterval(logs,6000);
  $("#logRefresh").onclick=logs;
  $("#logClear").onclick=async()=>{try{await api("/api/logs/clear",{method:"POST",body:{}});logs();toast("Log cleared","ok");}catch(e){toast(e.message,"err");}};
  $("#btnSaveNow").onclick=async()=>{try{await api("/api/system/save",{method:"POST",body:{}});toast("Configuration flushed","ok");load();}catch(e){toast(e.message,"err");}};
  $("#btnReboot").onclick=async()=>{if(!(await confirmBox("Restart the device now?")))return;try{await api("/api/system/reboot",{method:"POST",body:{}});toast("Restarting…","ok");}catch(e){toast(e.message,"err");}};
  $("#btnFactory").onclick=async()=>{if(!(await confirmBox("Factory reset erases ALL settings, names and schedules. Continue?")))return;try{await api("/api/system/factory",{method:"POST",body:{}});toast("Factory reset - device restarting","ok");}catch(e){toast(e.message,"err");}};
});
)JS");
  sendPage();
}

void handleSettingsPage() {
  if (!webAuth()) return;
  pageBegin("Settings", "set");
  char dev[160], host[160], user[160];
  hEsc(gCfg.ui.deviceName, dev, sizeof(dev));
  hEsc(gCfg.sta.hostname, host, sizeof(host));
  hEsc(gCfg.ui.adminUser, user, sizeof(user));
  char b[300];

  gPage += F("<h1>Settings</h1><p class=\"sub\">Device identity, admin access, UI polling, backup and recovery.</p>"
             "<div class=\"grid g2\"><div class=\"card\"><h3>Device</h3>"
             "<label class=\"f\"><span>Device name (shown in the UI)</span><input id=\"devName\" maxlength=\"31\"></label>"
             "<label class=\"f\"><span>Hostname / mDNS</span><input id=\"hostName\" maxlength=\"31\"></label>"
             "<div class=\"hint\" id=\"mdnsHint\"></div>"
             "<label class=\"f\"><span>Status polling interval (ms)</span><input id=\"pollMs\" type=\"number\" min=\"1000\" max=\"10000\"></label>"
             "<label class=\"f\"><span>Log level</span><select id=\"logLvl\"><option value=\"0\">Error</option><option value=\"1\">Warning</option><option value=\"2\">Info</option><option value=\"3\">Debug</option></select></label>"
             "<button class=\"btn primary\" id=\"btnSave\">Save settings</button></div>"
             "<div class=\"card\"><h3>Admin access</h3>"
             "<label class=\"switch\"><input type=\"checkbox\" id=\"authEn\"> Require HTTP basic authentication</label>"
             "<label class=\"f\"><span>Username</span><input id=\"adminUser\" maxlength=\"23\"></label>"
             "<label class=\"f\"><span>Password</span><input id=\"adminPass\" maxlength=\"31\" type=\"password\"></label>"
             "<div class=\"warnbox\">When authentication is enabled, every page and API requires these credentials. Keep them safe; there is no recovery other than a factory reset.</div></div>"
             "</div>"
             "<div class=\"grid g2\" style=\"margin-top:14px\">"
             "<div class=\"card\"><h3>Backup</h3>"
             "<p class=\"small dim\">Download the full configuration (settings + 128 schedules) as JSON, or restore a file produced by this firmware.</p>"
             "<div class=\"row\"><a class=\"btn\" href=\"/api/config/export\">Download config</a>"
             "<label class=\"btn\" style=\"cursor:pointer\">Restore file<input id=\"impFile\" type=\"file\" accept=\"application/json\" style=\"display:none\"></label></div>"
             "<div class=\"hint\" id=\"impState\"></div></div>"
             "<div class=\"card\"><h3>Storage &amp; recovery</h3>"
             "<div class=\"kv\"><span class=\"k\">NVS writes</span><span class=\"v\" id=\"setWrites\">&mdash;</span>"
             "<span class=\"k\">Write errors</span><span class=\"v\" id=\"setErrors\">&mdash;</span>"
             "<span class=\"k\">Pending writes</span><span class=\"v\" id=\"setPending\">&mdash;</span></div>"
             "<hr class=\"sep\"><div class=\"row\"><button class=\"btn\" id=\"btnFlush\">Flush now</button>"
             "<button class=\"btn danger\" id=\"btnFactory2\">Factory reset</button></div>"
             "<div class=\"hint\">Config writes are coalesced (8 s debounce, 60 s maximum delay) and stored in two CRC-protected NVS slots.</div></div>"
             "</div>");
  pageEnd();
  pageScript(R"JS(
let G=null;
const load=async ()=>{
  try{
    const s=await api("/api/settings");G=s;
    $("#devName").value=s.deviceName;$("#hostName").value=s.hostname;
    $("#pollMs").value=s.pollMs;$("#logLvl").value=String(s.logLevel);
    $("#authEn").checked=!!s.authEnabled;$("#adminUser").value=s.adminUser;$("#adminPass").value=s.adminPass;
    $("#setWrites").textContent=s.store.writes;$("#setErrors").textContent=s.store.errors;
    $("#setPending").textContent=s.store.pending?"yes":"no";
    $("#mdnsHint").textContent="http://"+s.hostname+".local";
  }catch(e){toast(e.message,"err");}
}
document.addEventListener("DOMContentLoaded",()=>{
  load();
  $("#btnSave").onclick=async()=>{
    try{
      await api("/api/settings/save",{method:"POST",body:{deviceName:$("#devName").value,hostname:$("#hostName").value,
        pollMs:num($("#pollMs").value,2500),logLevel:num($("#logLvl").value,2),
        authEnabled:$("#authEn").checked?1:0,adminUser:$("#adminUser").value,adminPass:$("#adminPass").value}});
      toast("Settings saved","ok");load();setTimeout(()=>location.reload(),700);
    }catch(e){toast(e.message,"err");}
  };
  $("#impFile").onchange=async e=>{
    const f=e.target.files[0];if(!f)return;
    $("#impState").textContent="Uploading "+f.name+"…";
    try{
      const txt=await f.text();
      const j=JSON.parse(txt);
      const r=await fetch("/api/config/import",{method:"POST",headers:{"Content-Type":"application/json"},body:JSON.stringify(j)});
      const out=await r.json();
      if(!r.ok||out.ok===false)throw new Error(out.error||("HTTP "+r.status));
      $("#impState").textContent="Restored. Rebooting…";toast("Configuration restored","ok");
    }catch(err){$("#impState").textContent="";toast(err.message,"err");}
  };
  $("#btnFlush").onclick=async()=>{try{await api("/api/system/save",{method:"POST",body:{}});toast("Flushed","ok");load();}catch(e){toast(e.message,"err");}};
  $("#btnFactory2").onclick=async()=>{if(!(await confirmBox("Factory reset erases ALL settings. Continue?")))return;try{await api("/api/system/factory",{method:"POST",body:{}});toast("Factory reset","ok");}catch(e){toast(e.message,"err");}};
  setInterval(load,5000);
});
)JS");
  sendPage();
}

void handlePortal() {
  pageBegin("Setup portal", "portal");
  char ss[160];
  hEsc(gCfg.ap.ssid, ss, sizeof(ss));
  char b[300];
  gPage += F("<div style=\"max-width:720px;margin:24px auto\">"
             "<div class=\"card\"><h1>RelayHub setup portal</h1>"
             "<p class=\"sub\">You are connected to the device access point. Configure it or open the dashboard.</p>"
             "<div class=\"kv\" style=\"margin:10px 0 16px\">"
             "<span class=\"k\">Access point</span><span class=\"v\">");
  gPage += ss;
  gPage += F("</span><span class=\"k\">Dashboard</span><span class=\"v\">http://192.168.4.1/</span></div>"
             "<div class=\"row\"><a class=\"btn primary\" href=\"/\">Open dashboard</a>"
             "<a class=\"btn\" href=\"/wifi-sta\">Join a WiFi network</a>"
             "<a class=\"btn ghost\" href=\"/time\">Set the clock</a></div>"
             "<div class=\"hint\" style=\"margin-top:12px\">If the dashboard asks for credentials, use the admin user configured in Settings.</div>"
             "</div></div>");
  pageEnd();
  sendPage();
}

struct Jb {
  char  *p;
  size_t cap;
  size_t len;
  bool   ok;
};

void jbInit(Jb &b, char *buf, size_t cap) {
  b.p = buf; b.cap = cap; b.len = 0; b.ok = (cap > 0);
  if (cap > 0) buf[0] = 0;
}

void jbAdd(Jb &b, const char *fmt, ...) {
  if (!b.ok || b.len + 1 >= b.cap) { b.ok = false; return; }
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(b.p + b.len, b.cap - b.len, fmt, ap);
  va_end(ap);
  if (w < 0) { b.ok = false; return; }
  if ((size_t)w >= b.cap - b.len) { b.len = b.cap - 1; b.p[b.len] = 0; b.ok = false; return; }
  b.len += (size_t)w;
}

void jbStr(Jb &b, const char *s) {
  if (!b.ok) return;
  if (s == nullptr) s = "";
  for (size_t i = 0; s[i]; i++) {
    unsigned char c = (unsigned char)s[i];
    if (c == '"' || c == '\\') {
      if (b.len + 2 >= b.cap) { b.ok = false; return; }
      b.p[b.len++] = '\\';
      b.p[b.len++] = (char)c;
    } else if (c < 0x20) {
      char tmp[8];
      snprintf(tmp, sizeof(tmp), "\\u%04x", c);
      if (b.len + 6 >= b.cap) { b.ok = false; return; }
      memcpy(b.p + b.len, tmp, 6);
      b.len += 6;
    } else {
      if (b.len + 1 >= b.cap) { b.ok = false; return; }
      b.p[b.len++] = (char)c;
    }
  }
  b.p[b.len] = 0;
}

void jsonEsc(const char *in, char *out, size_t cap) {
  if (out == nullptr || cap == 0) return;
  if (in == nullptr) in = "";
  size_t o = 0;
  for (size_t i = 0; in[i] && o + 7 < cap; i++) {
    unsigned char c = (unsigned char)in[i];
    if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = (char)c; }
    else if (c < 0x20) { o += (size_t)snprintf(out + o, cap - o, "\\u%04x", c); }
    else out[o++] = (char)c;
  }
  out[o] = 0;
}

const char *srcName(uint8_t s) {
  switch (s) {
    case TSRC_RTC:     return "RTC";
    case TSRC_NTP:     return "NTP";
    case TSRC_BROWSER: return "BROWSER";
    default:           return "NONE";
  }
}

void fmtLocalString(int64_t utc, char *out, size_t cap) {
  if (!timePlausible(utc)) { snprintf(out, cap, "--"); return; }
  int32_t off = appOffsetAt(nullptr, utc);
  LocalDT t = localDtFromEpoch(utc + off);
  static const char *dw[7] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
  snprintf(out, cap, "%04d-%02u-%02u %02u:%02u:%02u %s",
           (int)t.y, (unsigned)t.mo, (unsigned)t.d, (unsigned)t.hh,
           (unsigned)t.mi, (unsigned)t.ss, dw[t.dow % 7]);
}

void fmtOffsetString(int32_t off, char *out, size_t cap) {
  const char *sg = (off < 0) ? "-" : "+";
  int32_t a = off < 0 ? -off : off;
  snprintf(out, cap, "UTC%s%02d:%02d:%02d", sg, (int)(a / 3600),
           (int)((a % 3600) / 60), (int)(a % 60));
}

bool validHostname(const char *s) {
  if (s == nullptr) return false;
  size_t n = strlen(s);
  if (n == 0 || n > HOST_LEN - 1) return false;
  if (s[0] == '-' || s[n - 1] == '-') return false;
  for (size_t i = 0; i < n; i++) {
    char c = s[i];
    if (!(isalnum((unsigned char)c) || c == '-')) return false;
  }
  return true;
}

bool validNtpHost(const char *s) {
  if (s == nullptr) return false;
  size_t n = strlen(s);
  if (n == 0 || n >= NTP_HOST_LEN) return false;
  for (size_t i = 0; i < n; i++) {
    char c = s[i];
    if (c <= 0x20 || c == 0x7F) return false;
  }
  return true;
}

bool validSsid(const char *s) {
  if (s == nullptr) return false;
  size_t n = strlen(s);
  return (n >= 1 && n <= SSID_LEN);
}

bool validWpaPass(const char *s) {
  if (s == nullptr) return true;
  size_t n = strlen(s);
  return (n == 0) || (n >= 8 && n <= PASS_LEN);
}

bool parseHms(const char *s, uint8_t &h, uint8_t &m, uint8_t &sec) {
  if (s == nullptr) return false;
  unsigned hh = 0, mm = 0, ss = 0;
  char extra = 0;
  if (sscanf(s, "%u:%u:%u%c", &hh, &mm, &ss, &extra) != 3) return false;
  if (hh > 23 || mm > 59 || ss > 59) return false;
  h = (uint8_t)hh; m = (uint8_t)mm; sec = (uint8_t)ss;
  return true;
}

void appendScheduleJson(Jb &b, const Schedule &s, int64_t nowUtc, bool timeOk, const TimeCtx &ctx) {
  bool now = false;
  if (s.enabled && timeOk) now = scheduleActiveAt(s, nowUtc, ctx);
  jbAdd(b, "{\"en\":%u,\"start\":\"%02u:%02u:%02u\",\"stop\":\"%02u:%02u:%02u\","
            "\"flags\":%u,\"tail\":%lu,\"months\":%u,\"dom\":%lu,\"dow\":%u,"
            "\"comb\":%u,\"leap\":%u,\"vfy\":%u,\"vfm\":%u,\"vfd\":%u,"
            "\"vty\":%u,\"vtm\":%u,\"vtd\":%u,\"now\":%u}",
        (unsigned)s.enabled, (unsigned)s.startH, (unsigned)s.startM, (unsigned)s.startS,
        (unsigned)s.stopH, (unsigned)s.stopM, (unsigned)s.stopS, (unsigned)s.flags,
        (unsigned long)s.tailSec, (unsigned)s.monthMask, (unsigned long)s.domMask,
        (unsigned)s.dowMask, (unsigned)s.combMode, (unsigned)s.leapMode,
        (unsigned)s.vfY, (unsigned)s.vfM, (unsigned)s.vfD,
        (unsigned)s.vtY, (unsigned)s.vtM, (unsigned)s.vtD, now ? 1u : 0u);
}

void appendRelayJson(Jb &b, int i, int64_t nowUtc, bool timeOk, const TimeCtx &ctx) {
  char name[200];
  jsonEsc(gRelay[i].name, name, sizeof(name));
  jbAdd(b, "{\"id\":%d,\"name\":\"%s\",\"pin\":%u,\"enabled\":%u,\"activeLow\":%u,"
           "\"override\":%u,\"on\":%u,\"pinLevel\":%u,\"act\":%u,\"sched\":[",
        i, name, (unsigned)kRelayPins[i], (unsigned)gRelay[i].enabled,
        (unsigned)gRelay[i].activeLow, (unsigned)gRelay[i].override,
        gRt[i].logicalOn ? 1u : 0u, (unsigned)gRt[i].pinLevel,
        (unsigned)gRt[i].activeRules);
  for (int s = 0; s < NSCHED; s++) {
    if (s) jbAdd(b, ",");
    appendScheduleJson(b, gRelay[i].sched[s], nowUtc, timeOk, ctx);
  }
  jbAdd(b, "]}");
}

static char gStatusBuf[4600];

void apiStatus() {
  if (!webAuth()) return;
  int64_t nowUtc = (int64_t)time(nullptr);
  bool tOk = timeIsValid();
  int onCount = 0, rules = 0;
  for (int i = 0; i < NRELAYS; i++) {
    if (gRt[i].logicalOn) onCount++;
    rules += gRt[i].activeRules;
  }

  Jb b;
  jbInit(b, gStatusBuf, sizeof(gStatusBuf));
  char tgt[24];
  fmtLocalString(nowUtc, tgt, sizeof(tgt));
  jbAdd(b, "{\"ok\":true,\"uptime\":%lu,\"heap\":%lu,\"minheap\":%lu,\"largest\":%lu,"
           "\"frag\":%u,\"sta\":%u,\"rssi\":%d,\"ip\":\"%s\",\"ap\":%u,\"apClients\":%u,"
           "\"relaysOn\":%d,\"rules\":%d,\"transitions\":%lu,\"logCount\":%u,"
           "\"time\":{\"epoch\":%lld,\"offset\":%ld,\"valid\":%u,\"src\":\"%s\",\"local\":\"%s\","
           "\"rtcOk\":%u,\"lastSync\":%lu},\"save\":{\"pending\":%u,\"writes\":%lu,\"errors\":%lu},"
           "\"relays\":[",
        (unsigned long)millis(), (unsigned long)freeHeapNow(), (unsigned long)minHeapNow(),
        (unsigned long)largestBlockNow(), (unsigned)heapFragPct(),
        gNet.staConnected ? 1u : 0u, (int)gNet.lastRssi, gNet.ip,
        gNet.apActive ? 1u : 0u, (unsigned)WiFi.softAPgetStationNum(),
        onCount, rules, (unsigned long)gRelayTransitions, (unsigned)gLogCount,
        (long long)nowUtc, (long)timeOffsetNow(), tOk ? 1u : 0u, srcName(gTimeRt.source), tgt,
        gTimeRt.rtcOk ? 1u : 0u, (unsigned long)gTimeRt.lastSyncUtc,
        cfgPending() ? 1u : 0u, (unsigned long)gStore.writes, (unsigned long)gStore.errors);

  for (int i = 0; i < NRELAYS; i++) {
    char name[80];
    jsonEsc(gRelay[i].name, name, sizeof(name));
    jbAdd(b, "%s{\"id\":%d,\"name\":\"%s\",\"pin\":%u,\"on\":%u,\"pin\":%u,"
             "\"ovr\":%u,\"act\":%u}",
          i ? "," : "", i, name, (unsigned)kRelayPins[i],
          gRt[i].logicalOn ? 1u : 0u, (unsigned)gRt[i].pinLevel,
          (unsigned)gRelay[i].override, (unsigned)gRt[i].activeRules);
  }
  jbAdd(b, "]}");

  if (!b.ok) { sendErrMsg(500, "Status buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

static char gTimeBuf[2200];

void apiTimeGet() {
  if (!webAuth()) return;
  int64_t nowUtc = (int64_t)time(nullptr);
  char local[40], last[40], offs[24];
  fmtLocalString(nowUtc, local, sizeof(local));
  fmtLocalString(gTimeRt.lastSyncUtc, last, sizeof(last));
  fmtOffsetString(timeOffsetNow(), offs, sizeof(offs));

  char rtcNow[40] = "--";
  if (gTimeRt.rtcOk) {
    int64_t e = 0;
    if (timeReadRtc(e)) fmtLocalString(e, rtcNow, sizeof(rtcNow));
  }
  float temp = NAN;
  int tempInt = 0;
  bool tempOk = false;
  if (gTimeRt.rtcOk) {
    temp = rtc.getTemperature();
    if (!isnan(temp) && temp > -50 && temp < 100) { tempOk = true; tempInt = (int)lroundf(temp); }
  }

  int32_t nextIn = 0;
  if (gGntpNextValid()) {
    int32_t d = (int32_t)(gNtp.nextSyncMs - millis());
    nextIn = d > 0 ? d / 1000 : 0;
  }

  Jb b;
  jbInit(b, gTimeBuf, sizeof(gTimeBuf));
  jbAdd(b, "{\"ok\":true,\"epoch\":%lld,\"offset\":%ld,\"offsetStr\":\"%s\",\"valid\":%u,"
           "\"src\":\"%s\",\"local\":\"%s\",\"rtcOk\":%u,\"rtcLost\":%u,\"rtcTemp\":",
        (long long)nowUtc, (long)timeOffsetNow(), offs, timeIsValid() ? 1u : 0u,
        srcName(gTimeRt.source), local, gTimeRt.rtcOk ? 1u : 0u,
        gTimeRt.rtcLostPower ? 1u : 0u);
  if (tempOk) jbAdd(b, "%d", tempInt); else jbAdd(b, "null");
  jbAdd(b, ",\"rtcNow\":\"%s\",\"syncOk\":%lu,\"syncFail\":%lu,\"lastSync\":%lld,"
           "\"lastSyncStr\":\"%s\",\"ntpOn\":%u,\"interval\":%u,\"nextNtpIn\":%ld,"
           "\"lastNtp\":\"", rtcNow, (unsigned long)gTimeRt.syncOk,
        (unsigned long)gTimeRt.syncFail, (long long)gTimeRt.lastSyncUtc, last,
        gCfg.time.ntpEnabled ? 1u : 0u, (unsigned)gCfg.time.syncIntervalMin,
        (long)nextIn);
  if (gTimeRt.lastNtpServer >= 0 && gTimeRt.lastNtpServer < gCfg.time.ntpCount) {
    jbStr(b, gCfg.time.ntp[gTimeRt.lastNtpServer]);
  }
  jbAdd(b, "\",\"lastResult\":\"%s\",\"tzMode\":%u,\"tzOffset\":%ld,\"dstMode\":%u,"
           "\"dstSec\":%ld,\"dsM\":%u,\"dsW\":%u,\"dsD\":%u,\"dsH\":%u,"
           "\"deM\":%u,\"deW\":%u,\"deD\":%u,\"deH\":%u,\"autoZone\":%u,\"posix\":\"",
        gTimeRt.lastNtpResult < 0 ? "-" : (gTimeRt.lastNtpResult ? "ok" : "fail"),
        (unsigned)gCfg.time.tzMode, (long)gCfg.time.tzOffsetSec,
        (unsigned)gCfg.time.dstMode, (long)gCfg.time.dstOffsetSec,
        (unsigned)gCfg.time.dstStartMonth, (unsigned)gCfg.time.dstStartWeek,
        (unsigned)gCfg.time.dstStartDow, (unsigned)gCfg.time.dstStartHour,
        (unsigned)gCfg.time.dstEndMonth, (unsigned)gCfg.time.dstEndWeek,
        (unsigned)gCfg.time.dstEndDow, (unsigned)gCfg.time.dstEndHour,
        (unsigned)gCfg.time.autoZone);
  jbStr(b, gCfg.time.posix);
  jbAdd(b, "\",\"drift\":%d,\"useRtc\":%u,\"syncOnReconnect\":%u,\"allowBrowser\":%u,\"ntp\":[",
        (int)gCfg.time.driftPpm100, (unsigned)gCfg.time.useRtc,
        (unsigned)gCfg.time.syncOnReconnect, (unsigned)gCfg.time.allowBrowserSync);
  for (int i = 0; i < NTP_MAX; i++) {
    jbAdd(b, "%s\"", i ? "," : "");
    jbStr(b, gCfg.time.ntp[i]);
    jbAdd(b, "\"");
  }
  jbAdd(b, "]}");

  if (!b.ok) { sendErrMsg(500, "Time buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

bool gGntpNextValid() {
  return gCfg.time.ntpEnabled && WiFi.status() == WL_CONNECTED;
}

void apiTimeSet() {
  if (!webAuth()) return;
  if (!gCfg.time.allowBrowserSync) { sendErrMsg(403, "Browser time sync is disabled"); return; }
  RhJsonDoc doc(512);
  if (!readJsonBody(doc)) return;
  if (doc["epoch"].isNull()) {
    sendErrMsg(400, "epoch is required");
    return;
  }
  int64_t e = doc["epoch"].as<long long>();
  if (!timePlausible(e)) { sendErrMsg(400, "epoch out of range (2020-2100)"); return; }

  if (!doc["offset"].isNull()) {
    int32_t off = doc["offset"].as<long>();
    if (off < -50400 || off > 50400) { sendErrMsg(400, "offset out of range"); return; }
    gCfg.time.tzMode = TZ_MODE_MANUAL;
    gCfg.time.tzOffsetSec = off;
    gCfg.time.dstMode = DST_NONE;
    timeApplyTz();
    cfgMarkSysDirty();
  }
  timeSetEpochUs(e, 0, TSRC_BROWSER, true);
  sendOkMsg("clock set from browser");
}

void apiTimeSyncNow() {
  if (!webAuth()) return;
  if (!gCfg.time.ntpEnabled) { sendErrMsg(400, "NTP is disabled"); return; }
  if (WiFi.status() != WL_CONNECTED) { sendErrMsg(409, "Wi-Fi station is not connected"); return; }
  ntpKick();
  sendOkMsg("NTP sync requested");
}

bool applyTimeJson(JsonObjectConst o, char *err, size_t ec) {
  TimeCfg t = gCfg.time;

  if (!o["ntpEnabled"].isNull()) {
    JsonVariantConst v = o["ntpEnabled"];
    if (v.is<bool>()) {
      t.ntpEnabled = v.as<bool>() ? 1 : 0;
    } else if (v.is<int>() && (v.as<int>() == 0 || v.as<int>() == 1)) {
      t.ntpEnabled = (uint8_t)v.as<int>();
    } else {
      snprintf(err, ec, "ntpEnabled must be 0 or 1");
      return false;
    }
  }
  if (!o["ntpInterval"].isNull()) {
    int iv = o["ntpInterval"].as<int>();
    if (iv < 1 || iv > 1440) { snprintf(err, ec, "sync interval must be 1-1440 minutes"); return false; }
    t.syncIntervalMin = (uint16_t)iv;
  }
  if (!o["tzMode"].isNull()) {
    int m = o["tzMode"].as<int>();
    if (m != TZ_MODE_MANUAL && m != TZ_MODE_AUTO) { snprintf(err, ec, "invalid timezone mode"); return false; }
    t.tzMode = (uint8_t)m;
  }
  if (!o["tzOffset"].isNull()) {
    long v = o["tzOffset"].as<long>();
    if (v < -50400 || v > 50400) { snprintf(err, ec, "GMT offset must be within +/-50400 seconds"); return false; }
    t.tzOffsetSec = (int32_t)v;
  }
  if (!o["dstMode"].isNull()) {
    int m = o["dstMode"].as<int>();
    if (m < 0 || m > DST_CUSTOM) { snprintf(err, ec, "invalid DST mode"); return false; }
    t.dstMode = (uint8_t)m;
  }
  if (!o["dstSec"].isNull()) {
    int v = o["dstSec"].as<int>();
    if (v < 0 || v > 7200) { snprintf(err, ec, "DST offset must be 0-7200 s"); return false; }
    t.dstOffsetSec = v;
  }
  if (!o["dsM"].isNull()) {
    int m = o["dsM"].as<int>();
    if (m < 1 || m > 12) { snprintf(err, ec, "DST start month 1-12"); return false; }
    t.dstStartMonth = (uint8_t)m;
  }
  if (!o["dsW"].isNull()) {
    int v = o["dsW"].as<int>();
    if (v < 0 || v > 4) { snprintf(err, ec, "DST start week 0-4"); return false; }
    t.dstStartWeek = (uint8_t)v;
  }
  if (!o["dsD"].isNull()) {
    int v = o["dsD"].as<int>();
    if (v < 0 || v > 6) { snprintf(err, ec, "DST start weekday 0-6"); return false; }
    t.dstStartDow = (uint8_t)v;
  }
  if (!o["dsH"].isNull()) {
    int v = o["dsH"].as<int>();
    if (v < 0 || v > 23) { snprintf(err, ec, "DST start hour 0-23"); return false; }
    t.dstStartHour = (uint8_t)v;
  }
  if (!o["deM"].isNull()) {
    int m = o["deM"].as<int>();
    if (m < 1 || m > 12) { snprintf(err, ec, "DST end month 1-12"); return false; }
    t.dstEndMonth = (uint8_t)m;
  }
  if (!o["deW"].isNull()) {
    int v = o["deW"].as<int>();
    if (v < 0 || v > 4) { snprintf(err, ec, "DST end week 0-4"); return false; }
    t.dstEndWeek = (uint8_t)v;
  }
  if (!o["deD"].isNull()) {
    int v = o["deD"].as<int>();
    if (v < 0 || v > 6) { snprintf(err, ec, "DST end weekday 0-6"); return false; }
    t.dstEndDow = (uint8_t)v;
  }
  if (!o["deH"].isNull()) {
    int v = o["deH"].as<int>();
    if (v < 0 || v > 23) { snprintf(err, ec, "DST end hour 0-23"); return false; }
    t.dstEndHour = (uint8_t)v;
  }
  if (!o["autoZone"].isNull()) {
    int z = o["autoZone"].as<int>();
    if (z < 0 || (z >= kTzZoneCount && z != 255)) { snprintf(err, ec, "invalid timezone index"); return false; }
    t.autoZone = (uint8_t)z;
  }
  if (!o["posix"].isNull()) {
    const char *p = o["posix"].as<const char*>();
    if (p == nullptr || strlen(p) >= POSIX_LEN) { snprintf(err, ec, "POSIX string too long"); return false; }
    strCopy(t.posix, sizeof(t.posix), p);
  }
  if (!o["drift"].isNull()) {
    int v = o["drift"].as<int>();
    if (v < -10000 || v > 10000) { snprintf(err, ec, "drift must be within +/-10000"); return false; }
    t.driftPpm100 = (int16_t)v;
  }
  if (!o["useRtc"].isNull()) t.useRtc = o["useRtc"].as<int>() ? 1 : 0;
  if (!o["syncOnReconnect"].isNull()) t.syncOnReconnect = o["syncOnReconnect"].as<int>() ? 1 : 0;
  if (!o["allowBrowser"].isNull()) t.allowBrowserSync = o["allowBrowser"].as<int>() ? 1 : 0;

  JsonArrayConst arr = o["ntp"].as<JsonArrayConst>();
  if (!arr.isNull()) {
    char tmp[NTP_MAX][NTP_HOST_LEN];
    int cnt = 0;
    memset(tmp, 0, sizeof(tmp));
    for (JsonVariantConst v : arr) {
      const char *s = v.as<const char*>();
      if (s == nullptr) { snprintf(err, ec, "NTP server must be a string"); return false; }
      if (s[0] == 0) continue;
      if (!validNtpHost(s)) { snprintf(err, ec, "invalid NTP server name"); return false; }
      if (cnt < NTP_MAX) { strCopy(tmp[cnt], NTP_HOST_LEN, s); cnt++; }
    }
    if (cnt == 0) { snprintf(err, ec, "at least one NTP server required"); return false; }
    memset(t.ntp, 0, sizeof(t.ntp));
    for (int i = 0; i < cnt; i++) strCopy(t.ntp[i], NTP_HOST_LEN, tmp[i]);
    t.ntpCount = (uint8_t)cnt;
  }

  gCfg.time = t;
  cfgSanitizeSys();
  return true;
}

void apiTimeConfig() {
  if (!webAuth()) return;
  RhJsonDoc doc(2048);
  if (!readJsonBody(doc)) return;
  char err[96] = {0};
  if (!applyTimeJson(doc.as<JsonObjectConst>(), err, sizeof(err))) {
    sendErrMsg(400, err);
    return;
  }
  timeApplyTz();
  ntpAbort();
  gNtp.nextSyncMs = millis() + 500UL;
  cfgMarkSysDirty();
  rhLog(LOG_INFO, "Time settings updated");
  sendOkMsg("time settings applied");
}

bool schedFromJson(JsonObjectConst o, Schedule &s, char *err, size_t ec) {
  memset(&s, 0, sizeof(s));
  s.combMode = COMB_AND;

  int en = o["en"].isNull() ? 0 : o["en"].as<int>();
  if (en < 0 || en > 1) { snprintf(err, ec, "en must be 0 or 1"); return false; }
  s.enabled = (uint8_t)en;

  const char *st = o["start"].isNull() ? "00:00:00" : o["start"].as<const char*>();
  const char *sp = o["stop"].isNull() ? "00:00:00" : o["stop"].as<const char*>();
  if (st == nullptr || sp == nullptr ||
      !parseHms(st, s.startH, s.startM, s.startS) ||
      !parseHms(sp, s.stopH, s.stopM, s.stopS)) {
    snprintf(err, ec, "start/stop must be HH:MM:SS");
    return false;
  }

  int flags = o["flags"].isNull() ? 0 : o["flags"].as<int>();
  if (flags < 0 || (flags & ~0x1F)) { snprintf(err, ec, "invalid schedule flags"); return false; }
  s.flags = (uint8_t)flags;

  long tail = o["tail"].isNull() ? 0 : o["tail"].as<long>();
  if (tail < 0 || tail > 86400) { snprintf(err, ec, "tail must be 0-86400 seconds"); return false; }
  s.tailSec = (uint32_t)tail;

  long mo = o["months"].isNull() ? 0 : o["months"].as<long>();
  if (mo < 0 || mo > 0x0FFF) { snprintf(err, ec, "months mask out of range"); return false; }
  s.monthMask = (uint16_t)mo;

  long dom = o["dom"].isNull() ? 0 : o["dom"].as<long>();
  if (dom < 0 || dom > 0x7FFFFFFFL) { snprintf(err, ec, "day-of-month mask out of range"); return false; }
  s.domMask = (uint32_t)dom;

  int dw = o["dow"].isNull() ? 0 : o["dow"].as<int>();
  if (dw < 0 || dw > 0x7F) { snprintf(err, ec, "weekday mask out of range"); return false; }
  s.dowMask = (uint8_t)dw;

  int comb = o["comb"].isNull() ? 0 : o["comb"].as<int>();
  if (comb != COMB_AND && comb != COMB_OR) { snprintf(err, ec, "comb must be 0 or 1"); return false; }
  s.combMode = (uint8_t)comb;

  int leap = o["leap"].isNull() ? 0 : o["leap"].as<int>();
  if (leap < 0 || leap > 2) { snprintf(err, ec, "leap must be 0-2"); return false; }
  s.leapMode = (uint8_t)leap;

  int vfy = o["vfy"].isNull() ? 24 : o["vfy"].as<int>();
  int vfm = o["vfm"].isNull() ? 1 : o["vfm"].as<int>();
  int vfd = o["vfd"].isNull() ? 1 : o["vfd"].as<int>();
  int vty = o["vty"].isNull() ? 29 : o["vty"].as<int>();
  int vtm = o["vtm"].isNull() ? 12 : o["vtm"].as<int>();
  int vtd = o["vtd"].isNull() ? 31 : o["vtd"].as<int>();
  if (vfy < 0 || vfy > 255 || vty < 0 || vty > 255 ||
      vfm < 1 || vfm > 12 || vtm < 1 || vtm > 12 ||
      vfd < 1 || vfd > 31 || vtd < 1 || vtd > 31) {
    snprintf(err, ec, "validity date out of range");
    return false;
  }
  s.vfY = (uint8_t)vfy; s.vfM = (uint8_t)vfm; s.vfD = (uint8_t)vfd;
  s.vtY = (uint8_t)vty; s.vtM = (uint8_t)vtm; s.vtD = (uint8_t)vtd;
  return true;
}

bool applyRelayJson(JsonObjectConst o, int &idOut, RelayCfg &out, char *err, size_t ec) {
  if (o["id"].isNull()) { snprintf(err, ec, "relay id is required"); return false; }
  int id = o["id"].as<int>();
  if (id < 0 || id >= NRELAYS) { snprintf(err, ec, "relay id 0-15 required"); return false; }

  RelayCfg r = gRelay[id];

  if (!o["name"].isNull()) {
    const char *n = o["name"].as<const char*>();
    if (n == nullptr || !cfgValidateName(n)) { snprintf(err, ec, "invalid relay name"); return false; }
    strCopy(r.name, sizeof(r.name), n);
  }
  if (!o["activeLow"].isNull()) r.activeLow = (o["activeLow"].as<int>() != 0) ? 1 : 0;
  if (!o["enabled"].isNull()) r.enabled = (o["enabled"].as<int>() != 0) ? 1 : 0;

  JsonArrayConst arr = o["sched"].as<JsonArrayConst>();
  if (!arr.isNull()) {
    if (arr.size() > NSCHED) { snprintf(err, ec, "at most 8 schedules per relay"); return false; }
    int i = 0;
    for (JsonVariantConst v : arr) {
      Schedule s;
      if (!schedFromJson(v.as<JsonObjectConst>(), s, err, ec)) {
        char tmp[96];
        snprintf(tmp, sizeof(tmp), "schedule %d: %s", i + 1, err);
        strCopy(err, ec, tmp);
        return false;
      }
      r.sched[i] = s;
      i++;
    }
    for (; i < NSCHED; i++) {
      memset(&r.sched[i], 0, sizeof(Schedule));
      r.sched[i].combMode = COMB_AND;
      r.sched[i].startH = 8;
      r.sched[i].stopH = 18;
      r.sched[i].vfY = 24; r.sched[i].vfM = 1; r.sched[i].vfD = 1;
      r.sched[i].vtY = 29; r.sched[i].vtM = 12; r.sched[i].vtD = 31;
    }
  }

  idOut = id;
  out = r;
  return true;
}

void apiRelaysGet() {
  if (!webAuth()) return;
  int64_t nowUtc = (int64_t)time(nullptr);
  bool tOk = timeIsValid();
  TimeCtx ctx = timeContext();

  gPage = "";
  gPage += F("{\"ok\":true,\"count\":");
  gPage += String((int)NRELAYS);
  gPage += F(",\"relays\":[");

  static char rb[2600];
  for (int i = 0; i < NRELAYS; i++) {
    Jb b;
    jbInit(b, rb, sizeof(rb));
    appendRelayJson(b, i, nowUtc, tOk, ctx);
    if (!b.ok) { sendErrMsg(500, "Relay JSON overflow"); return; }
    if (i) gPage += F(",");
    gPage += rb;
  }
  gPage += F("]}");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", gPage);
}

void apiRelayGet() {
  if (!webAuth()) return;
  if (!server.hasArg("id")) { sendErrMsg(400, "id required"); return; }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= NRELAYS) { sendErrMsg(400, "id 0-15 required"); return; }
  static char rb[2600];
  Jb b;
  jbInit(b, rb, sizeof(rb));
  jbAdd(b, "{\"ok\":true,\"relay\":");
  appendRelayJson(b, id, (int64_t)time(nullptr), timeIsValid(), timeContext());
  jbAdd(b, "}");
  if (!b.ok) { sendErrMsg(500, "Relay JSON overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiRelaySave() {
  if (!webAuth()) return;
  RhJsonDoc doc(12288);
  if (!readJsonBody(doc)) return;
  RelayCfg r;
  int id = -1;
  char err[110] = {0};
  if (!applyRelayJson(doc.as<JsonObjectConst>(), id, r, err, sizeof(err))) {
    sendErrMsg(400, err);
    return;
  }
  cfgSanitizeRelay(r, id);
  gRelay[id] = r;
  cfgMarkRelayDirty(id);
  relayWritePin(id);
  rhLog(LOG_INFO, "Relay %d config saved", id + 1);
  sendOkMsg("relay saved");
}

void apiRelayName() {
  if (!webAuth()) return;
  RhJsonDoc doc(512);
  if (!readJsonBody(doc)) return;
  if (doc["id"].isNull() || doc["name"].isNull()) { sendErrMsg(400, "id and name required"); return; }
  int id = doc["id"].as<int>();
  const char *name = doc["name"].as<const char*>();
  if (id < 0 || id >= NRELAYS) { sendErrMsg(400, "id 0-15 required"); return; }
  if (!cfgValidateName(name)) { sendErrMsg(400, "name must be 1-24 printable characters"); return; }
  strCopy(gRelay[id].name, sizeof(gRelay[id].name), name);
  cfgMarkRelayDirty(id);
  rhLog(LOG_INFO, "Relay %d renamed", id + 1);
  sendOkMsg("name saved");
}

void apiRelayOverride() {
  if (!webAuth()) return;
  RhJsonDoc doc(256);
  if (!readJsonBody(doc)) return;
  if (doc["id"].isNull() || doc["mode"].isNull()) { sendErrMsg(400, "id and mode required"); return; }
  int id = doc["id"].as<int>();
  int mode = doc["mode"].as<int>();
  if (id < 0 || id >= NRELAYS || mode < 0 || mode > 2) { sendErrMsg(400, "bad id or mode"); return; }
  relaySetOverride(id, (uint8_t)mode);
  sendOkMsg("override applied");
}

void apiGpioRelay() {
  if (!webAuth()) return;
  RhJsonDoc doc(384);
  if (!readJsonBody(doc)) return;
  if (doc["id"].isNull()) { sendErrMsg(400, "id required"); return; }
  int id = doc["id"].as<int>();
  if (id < 0 || id >= NRELAYS) { sendErrMsg(400, "id 0-15 required"); return; }

  if (!doc["activeLow"].isNull()) {
    gRelay[id].activeLow = (doc["activeLow"].as<int>() != 0) ? 1 : 0;
    gRt[id].pinLevel = 0xFF;
    cfgMarkRelayDirty(id);
    relayWritePin(id);
    sendOkMsg("polarity updated");
    return;
  }
  if (doc["mode"].isNull()) { sendErrMsg(400, "mode or activeLow required"); return; }
  int mode = doc["mode"].as<int>();
  if (mode < 0 || mode > 2) { sendErrMsg(400, "mode 0-2 required"); return; }
  relaySetOverride(id, (uint8_t)mode);
  sendOkMsg("relay updated");
}

void apiGpioGlobal() {
  if (!webAuth()) return;
  RhJsonDoc doc(256);
  if (!readJsonBody(doc)) return;
  const char *a = doc["action"].as<const char*>();
  if (a == nullptr) { sendErrMsg(400, "action required"); return; }
  if (strcmp(a, "high") == 0) relaySetAllOverride(OVR_FORCE_HIGH);
  else if (strcmp(a, "low") == 0) relaySetAllOverride(OVR_FORCE_LOW);
  else if (strcmp(a, "auto") == 0) relaySetAllOverride(OVR_AUTO);
  else if (strcmp(a, "activelow") == 0) relaySetAllActiveLow(1);
  else if (strcmp(a, "activehigh") == 0) relaySetAllActiveLow(0);
  else { sendErrMsg(400, "unknown action"); return; }
  sendOkMsg("global action applied");
}

uint32_t gRebootAtMs = 0;
uint8_t  gRebootMode = 0;

void scheduleReboot(uint32_t delayMs, uint8_t mode) {
  gRebootAtMs = millis() + delayMs;
  if (gRebootAtMs == 0) gRebootAtMs = 1;
  gRebootMode = mode;
}

bool applyStaJson(JsonObjectConst o, char *err, size_t ec) {
  NetStaCfg s = gCfg.sta;
  const char *v = nullptr;

  if (!o["ssid"].isNull()) {
    v = o["ssid"].as<const char*>();
    if (v != nullptr && v[0] != 0 && !validSsid(v)) {
      snprintf(err, ec, "SSID must be 1-32 characters");
      return false;
    }
    strCopy(s.ssid, sizeof(s.ssid), v ? v : "");
  }
  if (!o["pass"].isNull()) {
    v = o["pass"].as<const char*>();
    if (!validWpaPass(v)) { snprintf(err, ec, "password must be empty or 8-64 characters"); return false; }
    strCopy(s.pass, sizeof(s.pass), v ? v : "");
  }
  if (!o["ssid2"].isNull()) {
    v = o["ssid2"].as<const char*>();
    if (v != nullptr && v[0] != 0 && !validSsid(v)) { snprintf(err, ec, "backup SSID must be 1-32 characters"); return false; }
    strCopy(s.ssid2, sizeof(s.ssid2), v ? v : "");
  }
  if (!o["pass2"].isNull()) {
    v = o["pass2"].as<const char*>();
    if (!validWpaPass(v)) { snprintf(err, ec, "backup password must be empty or 8-64 characters"); return false; }
    strCopy(s.pass2, sizeof(s.pass2), v ? v : "");
  }
  if (!o["hostname"].isNull()) {
    v = o["hostname"].as<const char*>();
    if (!validHostname(v)) { snprintf(err, ec, "hostname must be 1-31 letters, digits or dashes"); return false; }
    strCopy(s.hostname, sizeof(s.hostname), v);
  }
  if (!o["dhcp"].isNull()) s.dhcp = (o["dhcp"].as<int>() != 0) ? 1 : 0;

  const char *fields[5] = { "ip", "gw", "sn", "dns1", "dns2" };
  char *dsts[5] = { s.ip, s.gw, s.sn, s.dns1, s.dns2 };
  size_t caps[5] = { sizeof(s.ip), sizeof(s.gw), sizeof(s.sn), sizeof(s.dns1), sizeof(s.dns2) };
  for (int i = 0; i < 5; i++) {
    if (o[fields[i]].isNull()) continue;
    v = o[fields[i]].as<const char*>();
    if (v == nullptr) v = "";
    if (v[0] != 0) {
      IPAddress test;
      if (!test.fromString(v)) { snprintf(err, ec, "%s is not a valid IPv4 address", fields[i]); return false; }
    }
    strCopy(dsts[i], caps[i], v);
  }

  if (!o["reconnectSec"].isNull()) {
    int r = o["reconnectSec"].as<int>();
    if (r < 1 || r > 60) { snprintf(err, ec, "reconnect interval must be 1-60 s"); return false; }
    s.reconnectSec = (uint16_t)r;
  }
  if (!o["autoReconnect"].isNull()) s.autoReconnect = o["autoReconnect"].as<int>() ? 1 : 0;
  if (!o["powerSave"].isNull()) s.powerSave = o["powerSave"].as<int>() ? 1 : 0;
  if (!o["enabled"].isNull()) s.enabled = o["enabled"].as<int>() ? 1 : 0;

  if (s.dhcp == 0) {
    if (s.ip[0] == 0 || s.gw[0] == 0 || s.sn[0] == 0) {
      snprintf(err, ec, "static addressing needs IP, gateway and subnet mask");
      return false;
    }
  }
  if (s.enabled && s.ssid[0] == 0) {
    snprintf(err, ec, "station mode needs an SSID");
    return false;
  }
  gCfg.sta = s;
  cfgSanitizeSys();
  return true;
}

void apiStaGet() {
  if (!webAuth()) return;
  char curSsid[40] = "", addr[24] = "", gw[24] = "", mac[24] = "";
  WiFi.SSID().toCharArray(curSsid, sizeof(curSsid));
  WiFi.localIP().toString().toCharArray(addr, sizeof(addr));
  WiFi.gatewayIP().toString().toCharArray(gw, sizeof(gw));
  WiFi.macAddress().toCharArray(mac, sizeof(mac));

  static char buf[2200];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  jbAdd(b, "{\"ok\":true,\"enabled\":%u,\"ssid\":\"", (unsigned)gCfg.sta.enabled);
  jbStr(b, gCfg.sta.ssid);
  jbAdd(b, "\",\"pass\":\"");
  jbStr(b, gCfg.sta.pass);
  jbAdd(b, "\",\"ssid2\":\"");
  jbStr(b, gCfg.sta.ssid2);
  jbAdd(b, "\",\"pass2\":\"");
  jbStr(b, gCfg.sta.pass2);
  jbAdd(b, "\",\"hostname\":\"");
  jbStr(b, gCfg.sta.hostname);
  jbAdd(b, "\",\"dhcp\":%u,\"ip\":\"%s\",\"gw\":\"%s\",\"sn\":\"%s\",\"dns1\":\"%s\",\"dns2\":\"%s\","
           "\"reconnectSec\":%u,\"autoReconnect\":%u,\"powerSave\":%u,\"connected\":%u,"
           "\"ssidCurrent\":\"%s\",\"addr\":\"%s\",\"gateway\":\"%s\",\"rssi\":%d,"
           "\"attempts\":%lu,\"lastReason\":\"%s\",\"mdns\":%u,\"mac\":\"%s\"}",
        (unsigned)gCfg.sta.dhcp, gCfg.sta.ip, gCfg.sta.gw, gCfg.sta.sn, gCfg.sta.dns1, gCfg.sta.dns2,
        (unsigned)gCfg.sta.reconnectSec, (unsigned)gCfg.sta.autoReconnect,
        (unsigned)gCfg.sta.powerSave, gNet.staConnected ? 1u : 0u, curSsid, addr, gw,
        (int)gNet.lastRssi, (unsigned long)gNet.attempts,
        gNet.staConnected ? "connected" : (gStaDisabledManually ? "stopped by user" : "disconnected"),
        gNet.mdnsActive ? 1u : 0u, mac);
  if (!b.ok) { sendErrMsg(500, "STA buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiStaSave() {
  if (!webAuth()) return;
  RhJsonDoc doc(2048);
  if (!readJsonBody(doc)) return;
  char err[100] = {0};
  bool hostChanged = false;
  if (!applyStaJson(doc.as<JsonObjectConst>(), err, sizeof(err))) { sendErrMsg(400, err); return; }
  hostChanged = strcmp(gCfg.sta.hostname, WiFi.getHostname() ? WiFi.getHostname() : "") != 0;
  gStaDisabledManually = false;
  cfgMarkSysDirty();
  if (hostChanged) mdnsStop();
  WiFi.setHostname(gCfg.sta.hostname);
  wifiApplyConfig(true);
  if (hostChanged) mdnsStart();
  rhLog(LOG_INFO, "STA settings applied");
  sendOkMsg("station settings applied");
}

void apiStaConnect() {
  if (!webAuth()) return;
  if (!staWanted()) { sendErrMsg(400, "no SSID configured"); return; }
  gStaDisabledManually = false;
  WiFi.disconnect(false, false);
  staBeginNow();
  sendOkMsg("connecting");
}

void apiStaDisconnect() {
  if (!webAuth()) return;
  gStaDisabledManually = true;
  WiFi.disconnect(false, false);
  gNet.staConnected = false;
  gNet.ip[0] = 0;
  gNet.lastDisconnectMs = millis();
  mdnsStop();
  timeSyncFromRtc(true);
  rhLog(LOG_INFO, "STA disconnected by user");
  sendOkMsg("disconnected");
}

void apiScanStart() {
  if (!webAuth()) return;
  wifiScanStart();
  sendOkMsg("scan started");
}

void apiScanGet() {
  if (!webAuth()) return;
  int16_t st = wifiScanStatus();
  static char buf[4096];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  if (st == WIFI_SCAN_RUNNING) {
    jbAdd(b, "{\"ok\":true,\"status\":\"running\",\"networks\":[]}");
  } else if (st < 0) {
    jbAdd(b, "{\"ok\":true,\"status\":\"idle\",\"networks\":[]}");
  } else {
    jbAdd(b, "{\"ok\":true,\"status\":\"done\",\"networks\":[");
    for (int i = 0; i < st; i++) {
      char ssid[40];
      WiFi.SSID(i).toCharArray(ssid, sizeof(ssid));
      jbAdd(b, "%s{\"ssid\":\"", i ? "," : "");
      jbStr(b, ssid);
      jbAdd(b, "\",\"rssi\":%d,\"channel\":%d,\"secure\":%u}",
            (int)WiFi.RSSI(i), (int)WiFi.channel(i),
            (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? 0u : 1u);
      if (!b.ok) break;
    }
    jbAdd(b, "]}");
  }
  if (!b.ok) { sendErrMsg(500, "scan buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

bool applyApJson(JsonObjectConst o, char *err, size_t ec) {
  NetApCfg a = gCfg.ap;
  const char *v = nullptr;
  if (!o["ssid"].isNull()) {
    v = o["ssid"].as<const char*>();
    if (!validSsid(v)) { snprintf(err, ec, "AP SSID must be 1-32 characters"); return false; }
    strCopy(a.ssid, sizeof(a.ssid), v);
  }
  if (!o["pass"].isNull()) {
    v = o["pass"].as<const char*>();
    if (!validWpaPass(v)) { snprintf(err, ec, "AP password must be empty or 8-64 characters"); return false; }
    strCopy(a.pass, sizeof(a.pass), v ? v : "");
  }
  if (!o["channel"].isNull()) {
    int c = o["channel"].as<int>();
    if (c < 1 || c > 13) { snprintf(err, ec, "channel must be 1-13"); return false; }
    a.channel = (uint8_t)c;
  }
  if (!o["maxConn"].isNull()) {
    int c = o["maxConn"].as<int>();
    if (c < 1 || c > 4) { snprintf(err, ec, "max clients must be 1-4"); return false; }
    a.maxConn = (uint8_t)c;
  }
  if (!o["fallbackSec"].isNull()) {
    int c = o["fallbackSec"].as<int>();
    if (c < 10 || c > 3600) { snprintf(err, ec, "fallback delay must be 10-3600 s"); return false; }
    a.fallbackSec = (uint16_t)c;
  }
  if (!o["hidden"].isNull()) a.hidden = o["hidden"].as<int>() ? 1 : 0;
  if (!o["captive"].isNull()) a.captive = o["captive"].as<int>() ? 1 : 0;
  if (!o["fallback"].isNull()) a.fallback = o["fallback"].as<int>() ? 1 : 0;
  if (!o["autoOff"].isNull()) a.autoOff = o["autoOff"].as<int>() ? 1 : 0;
  if (!o["enabled"].isNull()) a.enabled = o["enabled"].as<int>() ? 1 : 0;

  gCfg.ap = a;
  cfgSanitizeSys();
  return true;
}

void apiApGet() {
  if (!webAuth()) return;
  char ip[24] = "";
  WiFi.softAPIP().toString().toCharArray(ip, sizeof(ip));
  static char buf[1800];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  jbAdd(b, "{\"ok\":true,\"enabled\":%u,\"ssid\":\"", (unsigned)gCfg.ap.enabled);
  jbStr(b, gCfg.ap.ssid);
  jbAdd(b, "\",\"pass\":\"");
  jbStr(b, gCfg.ap.pass);
  jbAdd(b, "\",\"channel\":%u,\"maxConn\":%u,\"hidden\":%u,\"captive\":%u,\"fallback\":%u,"
           "\"autoOff\":%u,\"fallbackSec\":%u,\"active\":%u,\"ip\":\"%s\",\"clients\":%u,"
           "\"dns\":%u,\"fallbackActive\":%u}",
        (unsigned)gCfg.ap.channel, (unsigned)gCfg.ap.maxConn, (unsigned)gCfg.ap.hidden,
        (unsigned)gCfg.ap.captive, (unsigned)gCfg.ap.fallback, (unsigned)gCfg.ap.autoOff,
        (unsigned)gCfg.ap.fallbackSec, gNet.apActive ? 1u : 0u, ip,
        (unsigned)WiFi.softAPgetStationNum(), gDnsRunning ? 1u : 0u,
        gNet.fallbackActive ? 1u : 0u);
  if (!b.ok) { sendErrMsg(500, "AP buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiApSave() {
  if (!webAuth()) return;
  RhJsonDoc doc(1536);
  if (!readJsonBody(doc)) return;
  char err[100] = {0};
  if (!applyApJson(doc.as<JsonObjectConst>(), err, sizeof(err))) { sendErrMsg(400, err); return; }
  cfgMarkSysDirty();
  gApDirty = true;
  wifiApplyConfig(false);
  rhLog(LOG_INFO, "AP settings applied");
  sendOkMsg("access point settings applied");
}

void apiApClients() {
  if (!webAuth()) return;
  static char buf[1200];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  jbAdd(b, "{\"ok\":true,\"clients\":[");
  wifi_sta_list_t list;
  memset(&list, 0, sizeof(list));
  bool first = true;
  if (gNet.apActive && esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
    for (int i = 0; i < list.num && i < 8; i++) {
      char mac[20];
      snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
               list.sta[i].mac[0], list.sta[i].mac[1], list.sta[i].mac[2],
               list.sta[i].mac[3], list.sta[i].mac[4], list.sta[i].mac[5]);
      jbAdd(b, "%s{\"mac\":\"%s\",\"rssi\":%d}", first ? "" : ",", mac, (int)list.sta[i].rssi);
      first = false;
    }
  }
  jbAdd(b, "]}");
  if (!b.ok) { sendErrMsg(500, "client buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiSystemGet() {
  if (!webAuth()) return;
  char local[40], offs[24], mac[24] = "";
  fmtLocalString((int64_t)time(nullptr), local, sizeof(local));
  fmtOffsetString(timeOffsetNow(), offs, sizeof(offs));
  WiFi.macAddress().toCharArray(mac, sizeof(mac));

  int onCount = 0;
  for (int i = 0; i < NRELAYS; i++) if (gRt[i].logicalOn) onCount++;

  nvs_stats_t ns;
  memset(&ns, 0, sizeof(ns));
  nvs_get_stats(NULL, &ns);

  char chip[24];
  snprintf(chip, sizeof(chip), "%04X%08X", (unsigned)(ESP.getEfuseMac() >> 32),
           (unsigned)(ESP.getEfuseMac() & 0xFFFFFFFFUL));

  int tempInt = 0;
  bool tempOk = false;
  if (gTimeRt.rtcOk) {
    float t = rtc.getTemperature();
    if (!isnan(t) && t > -50 && t < 100) { tempOk = true; tempInt = (int)lroundf(t); }
  }

  static char buf[3000];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  jbAdd(b, "{\"ok\":true,\"ttl\":0,\"uptime\":%lu,\"heap\":%lu,\"minHeap\":%lu,\"largest\":%lu,"
           "\"frag\":%u,\"resetReason\":\"%s\",\"model\":\"%s\",\"cores\":%u,\"cpuMhz\":%u,"
           "\"sdk\":\"%s\",\"fw\":\"%s\",\"version\":\"%s\",\"flashSize\":%lu,\"sketchSize\":%lu,"
           "\"sketchSpace\":%lu,\"sta\":%u,\"rssi\":%d,\"ip\":\"%s\",\"ap\":%u,\"apClients\":%u,"
           "\"hostname\":\"%s\",\"mdns\":%u,\"mac\":\"%s\",\"transitions\":%lu,\"relaysOn\":%d,"
           "\"store\":{\"writes\":%lu,\"errors\":%lu,\"pending\":%u,\"seqSys\":%lu},"
           "\"nvs\":{\"used\":%lu,\"free\":%lu,\"total\":%lu,\"namespaces\":%lu},"
           "\"time\":{\"src\":\"%s\",\"valid\":%u,\"local\":\"%s\",\"offsetStr\":\"%s\","
           "\"rtcOk\":%u,\"rtcTemp\":",
        (unsigned long)millis(), (unsigned long)freeHeapNow(), (unsigned long)minHeapNow(),
        (unsigned long)largestBlockNow(), (unsigned)heapFragPct(), resetReasonStr(),
        ESP.getChipModel(), (unsigned)ESP.getChipCores(), (unsigned)ESP.getCpuFreqMHz(),
        ESP.getSdkVersion(), FW_NAME, FW_VERSION,
        (unsigned long)ESP.getFlashChipSize(), (unsigned long)ESP.getSketchSize(),
        (unsigned long)(ESP.getSketchSize() + ESP.getFreeSketchSpace()),
        gNet.staConnected ? 1u : 0u, (int)gNet.lastRssi, gNet.ip,
        gNet.apActive ? 1u : 0u, (unsigned)WiFi.softAPgetStationNum(),
        gCfg.sta.hostname, gNet.mdnsActive ? 1u : 0u, mac,
        (unsigned long)gRelayTransitions, onCount,
        (unsigned long)gStore.writes, (unsigned long)gStore.errors, cfgPending() ? 1u : 0u,
        (unsigned long)gStore.seqSys,
        (unsigned long)ns.used_entries, (unsigned long)ns.free_entries,
        (unsigned long)ns.total_entries, (unsigned long)ns.namespace_count,
        srcName(gTimeRt.source), timeIsValid() ? 1u : 0u, local, offs,
        gTimeRt.rtcOk ? 1u : 0u);
  if (tempOk) jbAdd(b, "%d", tempInt); else jbAdd(b, "null");
  jbAdd(b, ",\"lastSync\":%lu,\"syncOk\":%lu,\"syncFail\":%lu}}",
        (unsigned long)gTimeRt.lastSyncUtc, (unsigned long)gTimeRt.syncOk,
        (unsigned long)gTimeRt.syncFail);
  if (!b.ok) { sendErrMsg(500, "system buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiReboot() {
  if (!webAuth()) return;
  cfgFlushNow();
  sendOkMsg("restarting");
  scheduleReboot(700, 1);
}

void apiFactoryReset() {
  if (!webAuth()) return;
  sendOkMsg("factory reset, restarting");
  scheduleReboot(700, 2);
}

void apiSaveNow() {
  if (!webAuth()) return;
  cfgFlushNow();
  sendOkMsg(gDirty.pending ? "some records failed" : "configuration flushed");
}

static char gLogJson[9000];

void apiLogsGet() {
  if (!webAuth()) return;
  int want = LOG_LINES;
  if (server.hasArg("n")) {
    int n = server.arg("n").toInt();
    if (n > 0 && n <= LOG_LINES) want = n;
  }
  Jb b;
  jbInit(b, gLogJson, sizeof(gLogJson));
  jbAdd(b, "{\"ok\":true,\"logs\":[");
  int count = gLogCount < want ? gLogCount : want;
  bool first = true;
  for (int k = 0; k < count; k++) {
    int idx = (int)gLogHead - 1 - k;
    while (idx < 0) idx += LOG_LINES;
    LogEntry &e = gLogs[idx];
    char ts[40];
    if (timePlausible(e.epoch)) {
      fmtLocalString(e.epoch, ts, sizeof(ts));
    } else {
      snprintf(ts, sizeof(ts), "boot+%lums", (unsigned long)e.ms);
    }
    jbAdd(b, "%s{\"time\":\"%s\",\"level\":\"%s\",\"msg\":\"", first ? "" : ",", ts, levelName(e.level));
    jbStr(b, e.msg);
    jbAdd(b, "\",\"repeat\":%u}", (unsigned)e.repeat);
    first = false;
    if (!b.ok) break;
  }
  jbAdd(b, "]}");
  if (!b.ok) { sendErrMsg(500, "log buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiLogsClear() {
  if (!webAuth()) return;
  logSetup();
  rhLog(LOG_INFO, "Log cleared");
  sendOkMsg("log cleared");
}

bool applyUiJson(JsonObjectConst o, char *err, size_t ec) {
  UiCfg u = gCfg.ui;
  NetStaCfg sta = gCfg.sta;
  const char *v = nullptr;

  if (!o["deviceName"].isNull()) {
    v = o["deviceName"].as<const char*>();
    if (v == nullptr || strlen(v) < 1 || strlen(v) > DEVNAME_LEN - 1) {
      snprintf(err, ec, "device name must be 1-31 characters"); return false;
    }
    for (size_t i = 0; v[i]; i++) if ((unsigned char)v[i] < 0x20) { snprintf(err, ec, "invalid device name"); return false; }
    strCopy(u.deviceName, sizeof(u.deviceName), v);
  }
  if (!o["hostname"].isNull()) {
    v = o["hostname"].as<const char*>();
    if (!validHostname(v)) { snprintf(err, ec, "invalid hostname"); return false; }
    strCopy(sta.hostname, sizeof(sta.hostname), v);
  }
  if (!o["adminUser"].isNull()) {
    v = o["adminUser"].as<const char*>();
    if (v == nullptr || strlen(v) < 1 || strlen(v) > AUSER_LEN - 1) {
      snprintf(err, ec, "admin user must be 1-23 characters"); return false;
    }
    strCopy(u.adminUser, sizeof(u.adminUser), v);
  }
  if (!o["adminPass"].isNull()) {
    v = o["adminPass"].as<const char*>();
    if (v == nullptr || strlen(v) < 1 || strlen(v) > APASS_LEN - 1) {
      snprintf(err, ec, "admin password must be 1-31 characters"); return false;
    }
    strCopy(u.adminPass, sizeof(u.adminPass), v);
  }
  if (!o["authEnabled"].isNull()) u.authEnabled = o["authEnabled"].as<int>() ? 1 : 0;
  if (!o["logLevel"].isNull()) {
    int l = o["logLevel"].as<int>();
    if (l < 0 || l > 3) { snprintf(err, ec, "log level 0-3"); return false; }
    u.logLevel = (uint8_t)l;
  }
  if (!o["pollMs"].isNull()) {
    int p = o["pollMs"].as<int>();
    if (p < 1000 || p > 10000) { snprintf(err, ec, "poll interval 1000-10000 ms"); return false; }
    u.pollMs = (uint16_t)p;
  }
  if (u.authEnabled && strlen(u.adminPass) < 4) {
    snprintf(err, ec, "admin password must be at least 4 characters when auth is enabled");
    return false;
  }
  gCfg.ui = u;
  gCfg.sta = sta;
  cfgSanitizeSys();
  return true;
}

void apiSettingsGet() {
  if (!webAuth()) return;
  static char buf[1600];
  Jb b;
  jbInit(b, buf, sizeof(buf));
  jbAdd(b, "{\"ok\":true,\"deviceName\":\"");
  jbStr(b, gCfg.ui.deviceName);
  jbAdd(b, "\",\"hostname\":\"");
  jbStr(b, gCfg.sta.hostname);
  jbAdd(b, "\",\"adminUser\":\"");
  jbStr(b, gCfg.ui.adminUser);
  jbAdd(b, "\",\"adminPass\":\"");
  jbStr(b, gCfg.ui.adminPass);
  jbAdd(b, "\",\"authEnabled\":%u,\"logLevel\":%u,\"pollMs\":%u,"
           "\"store\":{\"writes\":%lu,\"errors\":%lu,\"pending\":%u}}",
        (unsigned)gCfg.ui.authEnabled, (unsigned)gCfg.ui.logLevel, (unsigned)gCfg.ui.pollMs,
        (unsigned long)gStore.writes, (unsigned long)gStore.errors, cfgPending() ? 1u : 0u);
  if (!b.ok) { sendErrMsg(500, "settings buffer overflow"); return; }
  sendJsonRaw(200, b.p);
}

void apiSettingsSave() {
  if (!webAuth()) return;
  RhJsonDoc doc(1024);
  if (!readJsonBody(doc)) return;
  char err[110] = {0};
  char oldHost[32];
  strCopy(oldHost, sizeof(oldHost), gCfg.sta.hostname);
  if (!applyUiJson(doc.as<JsonObjectConst>(), err, sizeof(err))) { sendErrMsg(400, err); return; }
  cfgMarkSysDirty();
  gLogLevelNow = gCfg.ui.logLevel;
  if (strcmp(oldHost, gCfg.sta.hostname) != 0) {
    mdnsStop();
    WiFi.setHostname(gCfg.sta.hostname);
    mdnsStart();
  }
  rhLog(LOG_INFO, "UI settings saved");
  sendOkMsg("settings saved");
}

void appendFmt(String &out, const char *fmt, ...) {
  static char tmp[1400];
  va_list ap;
  va_start(ap, fmt);
  int w = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  if (w < 0) return;
  if ((size_t)w >= sizeof(tmp)) tmp[sizeof(tmp) - 1] = 0;
  out += tmp;
}

void apiConfigExport() {
  if (!webAuth()) return;

  static char devName[200], host[200], user[200], pass[200];
  static char ssid[200], spass[400], ssid2[200], spass2[400];
  static char aSsid[200], aPass[400], posix[300];

  jsonEsc(gCfg.ui.deviceName, devName, sizeof(devName));
  jsonEsc(gCfg.sta.hostname, host, sizeof(host));
  jsonEsc(gCfg.ui.adminUser, user, sizeof(user));
  jsonEsc(gCfg.ui.adminPass, pass, sizeof(pass));
  jsonEsc(gCfg.sta.ssid, ssid, sizeof(ssid));
  jsonEsc(gCfg.sta.pass, spass, sizeof(spass));
  jsonEsc(gCfg.sta.ssid2, ssid2, sizeof(ssid2));
  jsonEsc(gCfg.sta.pass2, spass2, sizeof(spass2));
  jsonEsc(gCfg.ap.ssid, aSsid, sizeof(aSsid));
  jsonEsc(gCfg.ap.pass, aPass, sizeof(aPass));
  jsonEsc(gCfg.time.posix, posix, sizeof(posix));

  if (!gPageReserved) { gPage.reserve(24576); gPageReserved = true; }
  gPage = "";
  appendFmt(gPage, "{\"ok\":true,\"fw\":\"%s\",\"version\":\"%s\",\"saved\":%lld,",
            FW_NAME, FW_VERSION, (long long)time(nullptr));

  appendFmt(gPage,
    "\"ui\":{\"deviceName\":\"%s\",\"hostname\":\"%s\",\"adminUser\":\"%s\",\"adminPass\":\"%s\","
    "\"authEnabled\":%u,\"logLevel\":%u,\"pollMs\":%u},",
    devName, host, user, pass, (unsigned)gCfg.ui.authEnabled,
    (unsigned)gCfg.ui.logLevel, (unsigned)gCfg.ui.pollMs);

  appendFmt(gPage,
    "\"sta\":{\"ssid\":\"%s\",\"pass\":\"%s\",\"ssid2\":\"%s\",\"pass2\":\"%s\",\"hostname\":\"%s\","
    "\"dhcp\":%u,\"ip\":\"%s\",\"gw\":\"%s\",\"sn\":\"%s\",\"dns1\":\"%s\",\"dns2\":\"%s\","
    "\"reconnectSec\":%u,\"autoReconnect\":%u,\"powerSave\":%u,\"enabled\":%u},",
    ssid, spass, ssid2, spass2, host, (unsigned)gCfg.sta.dhcp,
    gCfg.sta.ip, gCfg.sta.gw, gCfg.sta.sn, gCfg.sta.dns1, gCfg.sta.dns2,
    (unsigned)gCfg.sta.reconnectSec, (unsigned)gCfg.sta.autoReconnect,
    (unsigned)gCfg.sta.powerSave, (unsigned)gCfg.sta.enabled);

  appendFmt(gPage,
    "\"ap\":{\"ssid\":\"%s\",\"pass\":\"%s\",\"channel\":%u,\"maxConn\":%u,\"hidden\":%u,"
    "\"captive\":%u,\"fallback\":%u,\"autoOff\":%u,\"fallbackSec\":%u,\"enabled\":%u},",
    aSsid, aPass, (unsigned)gCfg.ap.channel, (unsigned)gCfg.ap.maxConn,
    (unsigned)gCfg.ap.hidden, (unsigned)gCfg.ap.captive, (unsigned)gCfg.ap.fallback,
    (unsigned)gCfg.ap.autoOff, (unsigned)gCfg.ap.fallbackSec, (unsigned)gCfg.ap.enabled);

  appendFmt(gPage,
    "\"time\":{\"tzMode\":%u,\"tzOffset\":%ld,\"dstMode\":%u,\"dstSec\":%ld,"
    "\"dsM\":%u,\"dsW\":%u,\"dsD\":%u,\"dsH\":%u,\"deM\":%u,\"deW\":%u,\"deD\":%u,\"deH\":%u,"
    "\"autoZone\":%u,\"posix\":\"%s\",\"ntpEnabled\":%u,\"ntpInterval\":%u,"
    "\"drift\":%d,\"useRtc\":%u,\"syncOnReconnect\":%u,\"allowBrowser\":%u,\"ntp\":[",
    (unsigned)gCfg.time.tzMode, (long)gCfg.time.tzOffsetSec, (unsigned)gCfg.time.dstMode,
    (long)gCfg.time.dstOffsetSec, (unsigned)gCfg.time.dstStartMonth,
    (unsigned)gCfg.time.dstStartWeek, (unsigned)gCfg.time.dstStartDow,
    (unsigned)gCfg.time.dstStartHour, (unsigned)gCfg.time.dstEndMonth,
    (unsigned)gCfg.time.dstEndWeek, (unsigned)gCfg.time.dstEndDow,
    (unsigned)gCfg.time.dstEndHour, (unsigned)gCfg.time.autoZone, posix,
    (unsigned)gCfg.time.ntpEnabled, (unsigned)gCfg.time.syncIntervalMin,
    (int)gCfg.time.driftPpm100, (unsigned)gCfg.time.useRtc,
    (unsigned)gCfg.time.syncOnReconnect, (unsigned)gCfg.time.allowBrowserSync);
  for (int i = 0; i < gCfg.time.ntpCount && i < NTP_MAX; i++) {
    char tmp[200];
    jsonEsc(gCfg.time.ntp[i], tmp, sizeof(tmp));
    appendFmt(gPage, "%s\"%s\"", i ? "," : "", tmp);
  }
  appendFmt(gPage, "]},");

  appendFmt(gPage, "\"relays\":[");
  static char rb[2600];
  for (int i = 0; i < NRELAYS; i++) {
    Jb b;
    jbInit(b, rb, sizeof(rb));
    appendRelayJson(b, i, (int64_t)time(nullptr), timeIsValid(), timeContext());
    if (!b.ok) { sendErrMsg(500, "export overflow"); return; }
    if (i) gPage += F(",");
    gPage += rb;
  }
  appendFmt(gPage, "]}");

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", "attachment; filename=\"relayhub-config.json\"");
  server.send(200, "application/json", gPage);
}

void apiConfigImport() {
  if (!webAuth()) return;

  SysCfg *cfgBak = (SysCfg *)malloc(sizeof(SysCfg));
  RelayCfg *relBak = (RelayCfg *)malloc(sizeof(RelayCfg) * NRELAYS);
  if (cfgBak == nullptr || relBak == nullptr) {
    if (cfgBak) free(cfgBak);
    if (relBak) free(relBak);
    sendErrMsg(500, "out of memory");
    return;
  }

  RhJsonDoc doc(40000);
  if (!readJsonBody(doc)) { free(cfgBak); free(relBak); return; }

  memcpy(cfgBak, &gCfg, sizeof(SysCfg));
  memcpy(relBak, gRelay, sizeof(RelayCfg) * NRELAYS);

  JsonObjectConst root = doc.as<JsonObjectConst>();
  char err[130] = {0};
  bool ok = true;

  if (!root["ui"].isNull())  ok = applyUiJson(root["ui"].as<JsonObjectConst>(), err, sizeof(err));
  if (ok && !root["sta"].isNull()) ok = applyStaJson(root["sta"].as<JsonObjectConst>(), err, sizeof(err));
  if (ok && !root["ap"].isNull())  ok = applyApJson(root["ap"].as<JsonObjectConst>(), err, sizeof(err));
  if (ok && !root["time"].isNull()) ok = applyTimeJson(root["time"].as<JsonObjectConst>(), err, sizeof(err));

  if (ok && !root["relays"].isNull()) {
    JsonArrayConst arr = root["relays"].as<JsonArrayConst>();
    if (arr.isNull() || arr.size() > NRELAYS) {
      snprintf(err, sizeof(err), "relays must be an array of at most 16 objects");
      ok = false;
    } else {
      for (JsonVariantConst v : arr) {
        RelayCfg r;
        int id = -1;
        if (!applyRelayJson(v.as<JsonObjectConst>(), id, r, err, sizeof(err))) { ok = false; break; }
        gRelay[id] = r;
      }
    }
  }

  if (!ok) {
    memcpy(&gCfg, cfgBak, sizeof(SysCfg));
    memcpy(gRelay, relBak, sizeof(RelayCfg) * NRELAYS);
    free(cfgBak);
    free(relBak);
    sendErrMsg(400, err);
    return;
  }
  free(cfgBak);
  free(relBak);

  for (int i = 0; i < NRELAYS; i++) cfgSanitizeRelay(gRelay[i], i);
  cfgSanitizeSys();
  gLogLevelNow = gCfg.ui.logLevel;
  timeApplyTz();
  WiFi.setHostname(gCfg.sta.hostname);
  gApDirty = true;
  wifiApplyConfig(true);
  cfgMarkAllDirty();
  cfgFlushNow();
  rhLog(LOG_INFO, "Configuration imported");
  sendOkMsg("configuration restored - rebooting");
  scheduleReboot(900, 1);
}

void appSetup() {
  Serial.begin(115200, SERIAL_8N1, 16, 17);
  Serial.setDebugOutput(false);
  gBootMs = millis();
  logSetup();

  rhLog(LOG_INFO, "==============================================");
  rhLog(LOG_INFO, "%s %s starting (reset: %s)", FW_NAME, FW_VERSION, resetReasonStr());

  cfgLoadAll();
  rhLog(LOG_INFO, "Config loaded: %u relays, AP '%s', tz mode %u",
        (unsigned)NRELAYS, gCfg.ap.ssid, (unsigned)gCfg.time.tzMode);

  buttonSetup();
  relaySetup();
  timeSetup();
  wifiSetup();
  webSetup();

  rhLog(LOG_INFO, "free heap: %u bytes, largest block: %u",
        (unsigned)freeHeapNow(), (unsigned)largestBlockNow());
  rhLog(LOG_INFO, "setup complete");
}

void appLoop() {
  gLoopCount++;
  server.handleClient();
  wifiService();
  timeService();
  relayService();
  cfgService();
  buttonService();

  if (gRebootAtMs != 0 && (int32_t)(millis() - gRebootAtMs) >= 0) {
    uint8_t mode = gRebootMode;
    gRebootAtMs = 0;
    gRebootMode = 0;
    if (mode == 2) cfgFactoryReset();
    cfgFlushNow();
    ESP.restart();
  }

  static uint32_t lastHeapCheck = 0;
  if (millisSince(lastHeapCheck) > 30000UL) {
    lastHeapCheck = millis();
    if (largestBlockNow() < 8192) {
      rhLog(LOG_WARN, "Heap tight: free %u, largest %u",
            (unsigned)freeHeapNow(), (unsigned)largestBlockNow());
    }
  }

  yield();
}

void setup() { appSetup(); }
void loop()  { appLoop(); }
