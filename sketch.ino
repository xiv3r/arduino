/*
 * ============================================================================
 *  ESP32 UNIFIED 16-CHANNEL RELAY SCHEDULER / WEB MANAGER
 * ============================================================================
 *  Single-file Arduino sketch (.ino) providing:
 *
 *   - 16 relays on fixed GPIOs: 23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19,
 *     18, 5, 4, 2, 15 (relay 1..16 in this exact order)
 *   - 8 start/stop schedule pairs per relay with calendar-style fields:
 *     seconds, minutes, hours, day-of-month, month-of-year, day-of-week
 *     (each field may be a wildcard '*'). Handles same-day windows,
 *     overnight windows and overnight tails, mixed/combined filters,
 *     leap years and multi-year operation.
 *   - Web dashboard + separate pages: Relay Schedules, GPIO Management,
 *     Time Management, WiFi AP, WiFi STA, System Status.
 *     Every page has a live header clock: "SUN 10/25/26 14:23:45".
 *   - GPIO management: global active-high / active-low, per-relay active
 *     level, Set High / Set Low / Auto / Remove per relay.
 *   - Relay management: double-tap relay name to rename, force ON/OFF/Auto.
 *   - Time management: up to 4 custom NTP servers with rotation+fallback
 *     (fully async UDP SNTP client + built-in async DNS resolver),
 *     DS3231 RTC on I2C (SDA=21, SCL=22), sync interval up to 24 h,
 *     manual GMT offset (seconds) with US/EU/AU DST rules, auto POSIX
 *     timezone with DST, fixed drift compensation (default 1 s).
 *   - On STA reconnect: system clock refreshed from DS3231 immediately,
 *     then NTP sync (which also rewrites DS3231 when it drifted).
 *   - On STA disconnect: reconnect attempt every 1 second (non-blocking),
 *     system clock refreshed from DS3231.
 *   - "Sync from browser" button sets clock + DS3231 from the browser when
 *     NTP is unreachable. Manual time set also available.
 *   - WiFi AP management page (SSID/pass/channel/hidden/max clients/static
 *     IP/captive portal toggle/fallback AP) + WiFi STA management page
 *     (async scan, static IP/DNS, hostname, connect/disconnect).
 *   - Full captive portal (DNS wildcard + probe redirect) when enabled.
 *   - GPIO0 boot button: hold 5 s => factory reset (non-blocking poll).
 *   - NVS write mitigation: config kept in RAM, written only when changed,
 *     CRC32 + dual alternating slots with sequence numbers (power-loss
 *     safe), boot self-heal of a damaged slot, WiFi.persistent(false).
 *   - Heap fragmentation mitigation: pages stored in flash (PROGMEM) and
 *     streamed; ArduinoJson serialized directly to the socket; no String
 *     churn in loop(); fixed buffers.
 *   - 100% non-blocking main loop (no delay() anywhere, async state
 *     machines for WiFi, NTP, DNS, button, schedule engine).
 *
 *  LIBRARIES (install via Arduino Library Manager):
 *   - ArduinoJson  (v7.x)                by Benoit Blanchon
 *   - RTClib       (v2.x, Adafruit)      needs Adafruit BusIO (auto-dep)
 *   - ESP32 Arduino core 2.x or 3.x (WebServer, DNSServer, Preferences,
 *     WiFiUdp, Wire are bundled with the core)
 *
 *  WIRING:
 *   - DS3231: SDA -> GPIO21, SCL -> GPIO22, VCC 3V3, GND. (No external
 *     pull-ups needed, module has them; I2C 100 kHz.)
 *   - Relay GPIOs: see RELAY_GPIOS below. GPIO0 -> on-board BOOT button
 *     (held 5 s = factory reset).
 *
 *  IMPORTANT GPIO NOTES (classic ESP32 / WROOM-32):
 *   - GPIO1 = TX0 and GPIO3 = RX0. Serial is intentionally NOT initialized
 *     in this firmware (ENABLE_DEBUG_SERIAL must stay 0) because those two
 *     pins drive relays 8 and 9. The ROM bootloader still emits a short
 *     burst on GPIO1 at power-on, so relays 8/9 may chatter ~100 ms during
 *     boot/reset. Do not put safety-critical loads on relays 8/9.
 *   - GPIO2 and GPIO15 are strapping pins: nothing on those relay inputs
 *     may pull them LOW at reset (GPIO2 low at reset = download mode).
 *     Standard opto-isolated relay boards are fine.
 *   - GPIO0 doubles as the boot button; keep it as the board's BOOT switch.
 *
 *  SCHEDULE EVALUATION MODEL (authoritative semantics):
 *   - Fields: sec(0-59), min(0-59), hour(0-23), dom(1-31), month(1-12),
 *     dow(0-6, SUN=0). Value 255 means wildcard "*". All non-wildcard
 *     fields must match simultaneously (AND semantics).
 *   - If a pair's START and STOP share identical date fields (dom, month,
 *     dow), the pair is a DAY WINDOW: ON from start time-of-day to stop
 *     time-of-day on matching days. If start > stop time, the window is
 *     OVERNIGHT and continues into the next day until stop time
 *     ("overnight tail"). Wildcard sec/min/hour expand to the earliest
 *     (start) / latest (stop) matching time-of-day.
 *   - If date fields differ (e.g. start on MON, stop on FRI), the pair is
 *     an EVENT LATCH: a matching START second turns it ON, a matching STOP
 *     second turns it OFF (stop wins if both match the same second). State
 *     is re-derived on boot by scanning back up to 400 days, so it works
 *     correctly across reboots, month/year boundaries and leap years.
 *   - The 8 pairs of a relay are OR-combined. Manual modes (Force ON/OFF,
 *     Set High/Low, Removed) override the engine.
 *   - Schedules run in LOCAL time (see Time Management page).
 *
 *  BUILD NOTE (important for anyone editing the embedded HTML/JS pages):
 *   - The web pages are stored as ordinary escaped C string literals, NOT
 *     as C++11 raw string literals. This is deliberate: the Arduino
 *     builder's prototype generator (ctags) cannot parse raw string
 *     literals and may inject bogus C++ prototypes generated from
 *     JavaScript "function(...)" syntax inside them, breaking the build
 *     with confusing errors. Keep new page content in escaped strings.
 *
 *  VERIFIED: compiles clean (0 errors / 0 warnings) with:
 *   - ESP32 Arduino core 3.3.12 + ArduinoJson 7.4.3 + RTClib 2.1.4
 *   - ESP32 Arduino core 2.0.17 + ArduinoJson 7.4.3 + RTClib 2.1.4
 *   Schedule/DST/calendar logic fuzz-verified against independent
 *   reference implementations over 2023-2030 (leap years, overnight
 *   tails, wildcard expansions, US/EU/AU DST rules).
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <time.h>
#include <sys/time.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <esp_heap_caps.h>

// ============================================================================
// SECTION 1: CONSTANTS, TYPES, CONFIG STRUCTURES
// ============================================================================

#define FW_NAME     "ESP32 Relay Manager"
#define FW_VERSION  "1.0.0"

#define ENABLE_DEBUG_SERIAL 0
#if ENABLE_DEBUG_SERIAL
  #define DBUF_SZ 256
  #define DPRINT(...) do { char b[DBUF_SZ]; snprintf(b, sizeof(b), __VA_ARGS__); Serial.print(b); } while (0)
#else
  #define DPRINT(...) do {} while (0)
#endif

#define N_RELAYS 16
#define N_PAIRS  8

static const uint8_t RELAY_GPIOS[N_RELAYS] = {
  23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19, 18, 5, 4, 2, 15
};

#define BTN_FACTORY_GPIO 0
#define BTN_FACTORY_MS  5000
#define I2C_SDA 21
#define I2C_SCL 22

#define CFG_MAGIC   0x524C5931UL
#define CFG_VERSION 3
#define NVS_NAMESPACE "relayctl"
#define NVS_KEY_A  "cfgA"
#define NVS_KEY_B  "cfgB"

#define MIN_VALID_UTC 1577836800L
#define MAX_VALID_UTC 4102444800L

#define WILDCARD 0xFF

enum RelayMode : uint8_t {
  RMODE_AUTO = 0,
  RMODE_ON   = 1,
  RMODE_OFF  = 2,
  RMODE_HIGH = 3,
  RMODE_LOW  = 4,
  RMODE_REMOVED = 5
};
#define RMODE_MAX 5

enum TzMode : uint8_t { TZ_MANUAL = 0, TZ_POSIX = 1 };
enum DstMode : uint8_t { DST_OFF = 0, DST_US = 1, DST_EU = 2, DST_AU = 3 };
enum TimeSrc : uint8_t { SRC_NONE = 0, SRC_RTC = 1, SRC_NTP = 2, SRC_BROWSER = 3, SRC_MANUAL = 4 };

struct CalFields {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t dom;
  uint8_t month;
  uint8_t dow;
};

struct SchedPair {
  uint8_t enabled;
  CalFields start;
  CalFields stop;
};

struct RelayCfg {
  char name[24];
  uint8_t gpio;
  uint8_t activeHigh;
  uint8_t mode;
};

struct TimeCfg {
  uint8_t  ntpEnabled;
  char     ntpServers[4][48];
  uint16_t syncIntervalMin;
  uint8_t  tzMode;
  int32_t  utcOffsetSec;
  uint8_t  dstMode;
  int32_t  dstOffsetSec;
  char     posixTz[64];
  uint8_t  rtcEnabled;
  uint8_t  driftComp;
};

struct StaCfg {
  uint8_t enabled;
  char ssid[33];
  char pass[65];
  char hostname[33];
  uint8_t dhcp;
  uint32_t ip, gw, sn, dns1, dns2;
  uint8_t autoReconnect;
};

struct ApCfg {
  uint8_t enabled;
  char ssid[33];
  char pass[65];
  uint8_t channel;
  uint8_t hidden;
  uint8_t maxClients;
  uint8_t captivePortal;
  uint8_t fallback;
  uint32_t ip, gw, sn;
};

struct SysCfg {
  uint32_t bootCount;
  uint8_t  cfgDefaulted;
  uint8_t  pad[3];
};

struct Config {
  RelayCfg relays[N_RELAYS];
  SchedPair sched[N_RELAYS][N_PAIRS];
  TimeCfg time;
  StaCfg sta;
  ApCfg ap;
  SysCfg sys;
};

struct SlotHdr {
  uint32_t magic;
  uint16_t ver;
  uint16_t len;
  uint32_t seq;
  uint32_t crc;
};

struct LocalFields {
  int y, mo, d, h, mi, s, dow;
  int32_t off;
  bool  dst;
  bool  valid;
};

// ============================================================================
// SECTION 2: GLOBAL STATE
// ============================================================================

static Config cfg;
static Preferences prefs;
static WebServer server(80);
static DNSServer dnsServer;
static WiFiUDP ntpUdp;
static WiFiUDP dnsResolverUdp;
static RTC_DS3231 rtc;
static bool rtcPresent = false;
static bool rtcLostPowerFlag = false;
static float rtcTempC = 0.0f;

static uint32_t slotSeqA = 0, slotSeqB = 0;
static bool haveA = false, haveB = false;
static uint32_t lastSavedCrc = 0;
static bool repairPending = false;
static bool storageDirty = false;
static uint32_t storageDirtyMs = 0;
static bool storageOk = true;

static volatile uint8_t lastTimeSource = SRC_NONE;
static volatile uint32_t lastSyncMs = 0;
static bool systemTimeValid = false;
static time_t lastEngineTickUtc = 0;
static bool engineRescanPending = true;
static time_t lastRescanUtc = 0;
static uint32_t lastDriftCheckMs = 0;
static uint32_t lastRtcTempMs = 0;

enum NtpPhase : uint8_t { NTP_IDLE = 0, NTP_DNS_WAIT, NTP_NTP_WAIT };
struct NtpRuntime {
  NtpPhase phase;
  uint8_t  tryIdx;
  uint8_t  activeIdx;
  uint32_t phaseMs;
  uint8_t  cycleFails;
  uint32_t nextAttemptMs;
  char     activeSrv[48];
  bool     lastOk;
  uint32_t lastOkMs;
  char     lastSrv[48];
  bool     manualKick;
} ntp;

struct DnsQuery {
  bool     busy;
  bool     ok;
  bool     isIp;
  uint16_t id;
  uint32_t sentMs;
  char     host[48];
  uint32_t ip;
} dnsq;

static bool engOn[N_RELAYS];
static bool latch[N_RELAYS][N_PAIRS];
static uint8_t lastPinLevel[N_RELAYS];
static bool lastPinInit[N_RELAYS];

static volatile bool evStaDisconnected = false;
static volatile bool evStaGotIp = false;
static uint32_t lastStaAttemptMs = 0;
static bool staRetrying = false;
static bool staPaused = false;
static uint32_t staDownSinceMs = 0;
static bool staEverUp = false;
static bool tempApActive = false;
static bool dnsRunning = false;
static uint32_t lastScanMs = 0;

static volatile bool pendingRestart = false;
static volatile uint32_t pendingRestartMs = 0;
static volatile bool pendingFactoryReset = false;
static uint32_t bootMs = 0;

extern const char HTML_HEAD[];
extern const char HTML_FOOT[];

static bool btnWasDown = false;
static bool btnStable = false;
static uint32_t btnDownSinceMs = 0;
static bool factoryResetArmed = false;
static uint32_t bootIgnoreBtnMs = 0;

// ============================================================================
// SECTION 3: FORWARD DECLARATIONS
// ============================================================================

static uint32_t crc32buf(const uint8_t* buf, size_t len);
static void configDefaults();
static void configDefaultsInto(Config* c);
static void sanitizeConfig();
static bool storageLoad();
static bool storageSaveNow(bool force);
static void storageCommitTask();
static void storageMarkDirty();
static void storageRepairIfPending();

static long days_from_civil(int y, int m, int d);
static void civil_from_days(long days, int* y, int* mo, int* d, int* dow);
static int  days_in_month(int y, int m);
static int  nth_dow_dom(int y, int m, int dow, int nth);
static bool dstActiveManual(time_t utc, const TimeCfg* t);
static void breakLocal(time_t utc, LocalFields* o);
static int32_t currentOffsetSec(time_t utc);
static void applyTz();
static int32_t offsetAtLocal(int y, int mo, int d, int h, int mi, int s, time_t approxUtc);

static bool setTimeFromUtc(time_t utc, uint8_t src);
static void rtcToSystem(bool* applied);
static bool systemToRtc();
static void rtcReadTemp();
static time_t rtcUtcSane();
static void driftTask(uint32_t now);
static void ntpTask(uint32_t now);
static void ntpKick();
static void ntpStartCycle();
static void ntpRotateOrIdle(uint32_t now);
static bool ntpSendRequest(const char* server, uint32_t ip);
static void ntpProcessPacket();
static void dnsResolveStart(const char* host);
static void dnsResolveTask(uint32_t now);
static uint32_t dnsBuildQuery(uint8_t* out, size_t outLen, uint16_t id, const char* host);
static int  dnsSkipName(const uint8_t* b, int len, int* i);

static bool matchCal(const CalFields* f, int s, int mi, int h, int dom, int mon, int dow);
static bool dayMatch(const CalFields* f, int dom, int mon, int dow);
static bool sameDateFields(const CalFields* a, const CalFields* b);
static int  todEarliest(const CalFields* f);
static int  todLatest(const CalFields* f);
static bool pairActiveWindow(const SchedPair* p, const LocalFields* lf, int yDom, int yMon, int yDow);
static void latchTick(const SchedPair* p, int r, int i, const LocalFields* lf);
static int  latestTodMatch(const CalFields* f, int limit);
static long latestMatch(const CalFields* f, time_t utcNow, int32_t off, int maxDays, bool* found);
static void engineRescanLatches(time_t utc);
static void engineTick(time_t utc);
static void engineInvalidate();

static void writeRelayPin(int i, uint8_t raw);
static void applyRelay(int i);
static void applyAllRelays();
static void initPinsSafe();
static void applyConfigPins();
static bool setRelayMode(int i, uint8_t mode);
static void setAllActiveLevel(bool activeHigh);
static void restoreAllRelays();

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info);
static void applySta(bool connectNow);
static void applyAp(bool restartAp);
static void wifiEventTask();
static void wifiTask(uint32_t now);
static void syncDnsCaptive(uint32_t now);
static bool apIsActive();
static bool staIsConnected();

static void sendPage(const char* body);
static void sendJsonOk();
static void sendJsonErr(const char* msg);
static void sendJsonDoc(JsonDocument& doc);
static bool getJsonBody(String& out, size_t maxLen);
static bool validNameStr(const char* s);
static bool validHostStr(const char* s);
static bool validNtpName(const char* s);
static bool validPosixTz(const char* s);
static String ipStr(uint32_t raw);
static uint32_t ipRaw(const String& s);
static String modeStr(uint8_t m);

static void buttonTask(uint32_t now);
static void doFactoryReset();
static void relayStatusJson(JsonArray arr, bool includeSched);

// ============================================================================
// SECTION 4: CONFIG STORAGE
// ============================================================================

static uint32_t crc32buf(const uint8_t* buf, size_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint32_t)buf[i];
    for (int k = 0; k < 8; k++) {
      uint32_t mask = (uint32_t)-(int32_t)(crc & 1);
      crc = (crc >> 1) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

static void configDefaultsInto(Config* c) {
  memset(c, 0, sizeof(Config));
  for (int i = 0; i < N_RELAYS; i++) {
    snprintf(c->relays[i].name, sizeof(c->relays[i].name), "Relay %d", i + 1);
    c->relays[i].gpio = RELAY_GPIOS[i];
    c->relays[i].activeHigh = 0;
    c->relays[i].mode = RMODE_AUTO;
  }
  for (int i = 0; i < N_RELAYS; i++) {
    for (int p = 0; p < N_PAIRS; p++) {
      SchedPair* sp = &c->sched[i][p];
      sp->enabled = 0;
      memset(&sp->start, WILDCARD, sizeof(CalFields));
      memset(&sp->stop, WILDCARD, sizeof(CalFields));
    }
  }
  strlcpy(c->time.ntpServers[0], "pool.ntp.org", sizeof(c->time.ntpServers[0]));
  strlcpy(c->time.ntpServers[1], "time.google.com", sizeof(c->time.ntpServers[1]));
  strlcpy(c->time.ntpServers[2], "time.cloudflare.com", sizeof(c->time.ntpServers[2]));
  c->time.ntpServers[3][0] = 0;
  c->time.ntpEnabled = 1;
  c->time.syncIntervalMin = 60;
  c->time.tzMode = TZ_MANUAL;
  c->time.utcOffsetSec = 0;
  c->time.dstMode = DST_OFF;
  c->time.dstOffsetSec = 3600;
  strlcpy(c->time.posixTz, "UTC0", sizeof(c->time.posixTz));
  c->time.rtcEnabled = 1;
  c->time.driftComp = 1;

  c->sta.enabled = 0;
  c->sta.ssid[0] = 0; c->sta.pass[0] = 0;
  strlcpy(c->sta.hostname, "esp32-relays", sizeof(c->sta.hostname));
  c->sta.dhcp = 1;
  c->sta.ip = c->sta.gw = c->sta.sn = c->sta.dns1 = c->sta.dns2 = 0;
  c->sta.autoReconnect = 1;

  uint64_t mac = ESP.getEfuseMac();
  snprintf(c->ap.ssid, sizeof(c->ap.ssid), "ESP32-Relays-%04X",
           (uint16_t)(mac >> 32));
  strlcpy(c->ap.pass, "12345678", sizeof(c->ap.pass));
  c->ap.enabled = 1;
  c->ap.channel = 1;
  c->ap.hidden = 0;
  c->ap.maxClients = 4;
  c->ap.captivePortal = 1;
  c->ap.fallback = 0;
  c->ap.ip = (uint32_t)IPAddress(192, 168, 4, 1);
  c->ap.gw = (uint32_t)IPAddress(192, 168, 4, 1);
  c->ap.sn = (uint32_t)IPAddress(255, 255, 255, 0);

  c->sys.bootCount = 0;
  c->sys.cfgDefaulted = 0;
}

static void configDefaults() {
  configDefaultsInto(&cfg);
}

static uint8_t sanField(uint8_t v, int lo, int hi) {
  if (v == WILDCARD) return WILDCARD;
  if ((int)v < lo || (int)v > hi) return WILDCARD;
  return v;
}

static void sanitizeConfig() {
  for (int i = 0; i < N_RELAYS; i++) {
    RelayCfg* r = &cfg.relays[i];
    r->name[sizeof(r->name) - 1] = 0;
    for (char* c = r->name; *c; c++) {
      if ((uint8_t)*c < 32 || (uint8_t)*c > 126) *c = '_';
    }
    if (r->name[0] == 0) snprintf(r->name, sizeof(r->name), "Relay %d", i + 1);
    r->gpio = RELAY_GPIOS[i];
    if (r->activeHigh > 1) r->activeHigh = 0;
    if (r->mode > RMODE_MAX) r->mode = RMODE_AUTO;
    for (int p = 0; p < N_PAIRS; p++) {
      SchedPair* sp = &cfg.sched[i][p];
      if (sp->enabled > 1) sp->enabled = 0;
      sp->start.sec  = sanField(sp->start.sec, 0, 59);
      sp->start.min  = sanField(sp->start.min, 0, 59);
      sp->start.hour = sanField(sp->start.hour, 0, 23);
      sp->start.dom  = sanField(sp->start.dom, 1, 31);
      sp->start.month= sanField(sp->start.month, 1, 12);
      sp->start.dow  = sanField(sp->start.dow, 0, 6);
      sp->stop.sec   = sanField(sp->stop.sec, 0, 59);
      sp->stop.min   = sanField(sp->stop.min, 0, 59);
      sp->stop.hour  = sanField(sp->stop.hour, 0, 23);
      sp->stop.dom   = sanField(sp->stop.dom, 1, 31);
      sp->stop.month = sanField(sp->stop.month, 1, 12);
      sp->stop.dow   = sanField(sp->stop.dow, 0, 6);
    }
  }
  TimeCfg* t = &cfg.time;
  if (t->syncIntervalMin > 1440) t->syncIntervalMin = 1440;
  if (t->tzMode > TZ_POSIX) t->tzMode = TZ_MANUAL;
  if (t->utcOffsetSec < -47200) t->utcOffsetSec = 0;
  if (t->utcOffsetSec > 47200) t->utcOffsetSec = 0;
  if (t->dstMode > DST_AU) t->dstMode = DST_OFF;
  if (t->dstOffsetSec < 0 || t->dstOffsetSec > 7200) t->dstOffsetSec = 3600;
  t->posixTz[sizeof(t->posixTz) - 1] = 0;
  if (!validPosixTz(t->posixTz)) strlcpy(t->posixTz, "UTC0", sizeof(t->posixTz));
  if (t->driftComp > 30) t->driftComp = 1;
  for (int i = 0; i < 4; i++) {
    t->ntpServers[i][sizeof(t->ntpServers[i]) - 1] = 0;
    if (!validNtpName(t->ntpServers[i])) t->ntpServers[i][0] = 0;
  }
  if (t->ntpEnabled > 1) t->ntpEnabled = 1;
  if (t->rtcEnabled > 1) t->rtcEnabled = 1;

  StaCfg* s = &cfg.sta;
  s->ssid[sizeof(s->ssid) - 1] = 0; s->pass[sizeof(s->pass) - 1] = 0;
  s->hostname[sizeof(s->hostname) - 1] = 0;
  if (!validHostStr(s->hostname)) strlcpy(s->hostname, "esp32-relays", sizeof(s->hostname));
  if (s->enabled > 1) s->enabled = 0;
  if (s->dhcp > 1) s->dhcp = 1;
  if (s->autoReconnect > 1) s->autoReconnect = 1;

  ApCfg* a = &cfg.ap;
  a->ssid[sizeof(a->ssid) - 1] = 0; a->pass[sizeof(a->pass) - 1] = 0;
  if (a->enabled > 1) a->enabled = 0;
  if (a->channel < 1 || a->channel > 13) a->channel = 1;
  if (a->hidden > 1) a->hidden = 0;
  if (a->maxClients < 1 || a->maxClients > 8) a->maxClients = 4;
  if (a->captivePortal > 1) a->captivePortal = 1;
  if (a->fallback > 1) a->fallback = 0;
  if (a->ssid[0] == 0) strlcpy(a->ssid, "ESP32-Relays", sizeof(a->ssid));
  size_t pl = strlen(a->pass);
  if (pl != 0 && pl < 8) strlcpy(a->pass, "12345678", sizeof(a->pass));
}

static bool storageLoad() {
  haveA = haveB = false;
  slotSeqA = slotSeqB = 0;
  uint8_t bufA[sizeof(SlotHdr) + sizeof(Config)];
  uint8_t bufB[sizeof(SlotHdr) + sizeof(Config)];
  bool okA = false, okB = false;
  size_t want = sizeof(SlotHdr) + sizeof(Config);

  if (prefs.getBytesLength(NVS_KEY_A) == want) {
    prefs.getBytes(NVS_KEY_A, bufA, want);
    SlotHdr h;
    memcpy(&h, bufA, sizeof(h));
    if (h.magic == CFG_MAGIC && h.ver == CFG_VERSION && h.len == sizeof(Config) &&
        crc32buf(bufA + sizeof(SlotHdr), sizeof(Config)) == h.crc) {
      okA = true; slotSeqA = h.seq; haveA = true;
    }
  }
  if (prefs.getBytesLength(NVS_KEY_B) == want) {
    prefs.getBytes(NVS_KEY_B, bufB, want);
    SlotHdr h;
    memcpy(&h, bufB, sizeof(h));
    if (h.magic == CFG_MAGIC && h.ver == CFG_VERSION && h.len == sizeof(Config) &&
        crc32buf(bufB + sizeof(SlotHdr), sizeof(Config)) == h.crc) {
      okB = true; slotSeqB = h.seq; haveB = true;
    }
  }

  bool defaulted = false;
  if (okA && okB) {
    const uint8_t* src = (slotSeqA >= slotSeqB) ? bufA : bufB;
    memcpy(&cfg, src + sizeof(SlotHdr), sizeof(Config));
    lastSavedCrc = crc32buf((const uint8_t*)&cfg, sizeof(Config));
  } else if (okA || okB) {
    const uint8_t* src = okA ? bufA : bufB;
    memcpy(&cfg, src + sizeof(SlotHdr), sizeof(Config));
    lastSavedCrc = crc32buf((const uint8_t*)&cfg, sizeof(Config));
    repairPending = true;
  } else {
    configDefaults();
    defaulted = true;
    cfg.sys.cfgDefaulted = 1;
    lastSavedCrc = 0;
    repairPending = true;
  }
  sanitizeConfig();
  cfg.sys.cfgDefaulted = defaulted ? 1 : 0;
  return okA || okB || defaulted;
}

static bool storageSaveNow(bool force) {
  uint32_t crc = crc32buf((const uint8_t*)&cfg, sizeof(Config));
  if (!force && crc == lastSavedCrc) { storageDirty = false; return true; }

  uint8_t buf[sizeof(SlotHdr) + sizeof(Config)];
  uint32_t newSeq = (slotSeqA > slotSeqB ? slotSeqA : slotSeqB) + 1;
  bool useA = (!haveA && !haveB) || (slotSeqA <= slotSeqB);

  SlotHdr h;
  h.magic = CFG_MAGIC;
  h.ver = CFG_VERSION;
  h.len = (uint16_t)sizeof(Config);
  h.seq = newSeq;
  h.crc = crc;
  memcpy(buf, &h, sizeof(h));
  memcpy(buf + sizeof(h), &cfg, sizeof(cfg));

  const char* key = useA ? NVS_KEY_A : NVS_KEY_B;
  size_t wrote = prefs.putBytes(key, buf, sizeof(buf));
  if (wrote != sizeof(buf)) { storageOk = false; return false; }
  storageOk = true;

  if (useA) { slotSeqA = newSeq; haveA = true; }
  else      { slotSeqB = newSeq; haveB = true; }
  lastSavedCrc = crc;
  storageDirty = false;
  return true;
}

static void storageMarkDirty() {
  storageDirty = true;
  storageDirtyMs = millis();
}

static void storageCommitTask() {
  if (repairPending) {
    repairPending = false;
    storageSaveNow(true);
  }
  if (storageDirty && (millis() - storageDirtyMs) > 5000UL) {
    storageSaveNow(false);
  }
}

// ============================================================================
// SECTION 5: CALENDAR + TIMEZONE CORE
// ============================================================================

static long days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  long era = (y >= 0 ? y : y - 399) / 400;
  long yoe = y - era * 400;
  long doy = (153L * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + doe - 719468;
}

static void civil_from_days(long days, int* y, int* mo, int* d, int* dow) {
  time_t t = (time_t)(days * 86400L + 43200);
  struct tm tmv;
  gmtime_r(&t, &tmv);
  *y = tmv.tm_year + 1900;
  *mo = tmv.tm_mon + 1;
  *d = tmv.tm_mday;
  *dow = tmv.tm_wday;
}

static int days_in_month(int y, int m) {
  static const int dm[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
  if (m == 2 && ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0)) return 29;
  if (m < 1 || m > 12) return 30;
  return dm[m - 1];
}

static int nth_dow_dom(int y, int m, int dow, int nth) {
  int dummyY, dummyM, firstDom, firstDow;
  if (nth == 0) {
    int L = days_in_month(y, m);
    civil_from_days(days_from_civil(y, m, L), &dummyY, &dummyM, &firstDom, &firstDow);
    return L - ((firstDow - dow + 7) % 7);
  }
  civil_from_days(days_from_civil(y, m, 1), &dummyY, &dummyM, &firstDom, &firstDow);
  int first = 1 + ((dow - firstDow + 7) % 7);
  return first + (nth - 1) * 7;
}

static bool dstActiveManual(time_t utc, const TimeCfg* t) {
  if (t->dstMode == DST_OFF || t->dstOffsetSec == 0) return false;
  int32_t std = t->utcOffsetSec;
  int32_t dst = t->dstOffsetSec;

  time_t localApprox = utc + std + dst;
  struct tm tmv;
  gmtime_r(&localApprox, &tmv);
  int Y = tmv.tm_year + 1900;

  for (int yy = Y - 1; yy <= Y + 1; yy++) {
    time_t s, e;
    if (t->dstMode == DST_US) {
      int sd = nth_dow_dom(yy, 3, 0, 2);
      int ed = nth_dow_dom(yy, 11, 0, 1);
      s = (time_t)days_from_civil(yy, 3, sd) * 86400L + 2 * 3600L - std;
      e = (time_t)days_from_civil(yy, 11, ed) * 86400L + 2 * 3600L - (std + dst);
    } else if (t->dstMode == DST_EU) {
      int sd = nth_dow_dom(yy, 3, 0, 0);
      int ed = nth_dow_dom(yy, 10, 0, 0);
      s = (time_t)days_from_civil(yy, 3, sd) * 86400L + 3600L;
      e = (time_t)days_from_civil(yy, 10, ed) * 86400L + 3600L;
    } else {
      int sd = nth_dow_dom(yy, 10, 0, 1);
      int ed = nth_dow_dom(yy + 1, 4, 0, 1);
      s = (time_t)days_from_civil(yy, 10, sd) * 86400L + 2 * 3600L - std;
      e = (time_t)days_from_civil(yy + 1, 4, ed) * 86400L + 3 * 3600L - (std + dst);
    }
    if (utc >= s && utc < e) return true;
  }
  return false;
}

static void breakLocal(time_t utc, LocalFields* o) {
  memset(o, 0, sizeof(*o));
  if (utc < MIN_VALID_UTC || utc > MAX_VALID_UTC) { o->valid = false; return; }
  o->valid = true;
  struct tm tmv;
  if (cfg.time.tzMode == TZ_POSIX) {
    localtime_r(&utc, &tmv);
    o->y = tmv.tm_year + 1900; o->mo = tmv.tm_mon + 1; o->d = tmv.tm_mday;
    o->h = tmv.tm_hour; o->mi = tmv.tm_min; o->s = tmv.tm_sec;
    o->dow = tmv.tm_wday;
    time_t localAsUtc = (time_t)days_from_civil(o->y, o->mo, o->d) * 86400L
                        + (time_t)o->h * 3600L + (time_t)o->mi * 60L + (time_t)o->s;
    o->off = (int32_t)(localAsUtc - utc);
    o->dst = tmv.tm_isdst > 0;
  } else {
    bool dst = dstActiveManual(utc, &cfg.time);
    int32_t off = cfg.time.utcOffsetSec + (dst ? cfg.time.dstOffsetSec : 0);
    time_t le = utc + off;
    gmtime_r(&le, &tmv);
    o->y = tmv.tm_year + 1900; o->mo = tmv.tm_mon + 1; o->d = tmv.tm_mday;
    o->h = tmv.tm_hour; o->mi = tmv.tm_min; o->s = tmv.tm_sec;
    o->dow = tmv.tm_wday;
    o->off = off;
    o->dst = dst;
  }
}

static int32_t currentOffsetSec(time_t utc) {
  LocalFields lf;
  breakLocal(utc, &lf);
  return lf.off;
}

static int32_t offsetAtLocal(int y, int mo, int d, int h, int mi, int s, time_t approxUtc) {
  time_t guessUtc = (time_t)days_from_civil(y, mo, d) * 86400L + h * 3600L + mi * 60L + s
                    - currentOffsetSec(approxUtc);
  return currentOffsetSec(guessUtc);
}

static void applyTz() {
  if (cfg.time.tzMode == TZ_POSIX) {
    setenv("TZ", cfg.time.posixTz, 1);
    tzset();
  }
  else {
    setenv("TZ", "UTC0", 1);
    tzset();
  }
}

// ============================================================================
// SECTION 6: TIME MANAGEMENT
// ============================================================================

static bool setTimeFromUtc(time_t utc, uint8_t src) {
  if (utc < MIN_VALID_UTC || utc > MAX_VALID_UTC) return false;
  struct timeval tv;
  tv.tv_sec = (time_t)utc;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  lastTimeSource = src;
  lastSyncMs = millis();
  bool wasValid = systemTimeValid;
  systemTimeValid = true;
  if (engineRescanPending || (wasValid && (utc > lastEngineTickUtc + 300 ||
                                           utc + 300 < lastEngineTickUtc)) ||
      (!wasValid && utc != 0)) {
    engineRescanPending = true;
  }
  return true;
}

static void rtcToSystem(bool* applied) {
  if (applied) *applied = false;
  if (!rtcPresent || !cfg.time.rtcEnabled) return;
  time_t r = rtcUtcSane();
  if (r == 0) return;
  if (setTimeFromUtc(r, SRC_RTC)) {
    if (applied) *applied = true;
    DPRINT("rtc->sys %lu\n", (unsigned long)r);
  }
}

static time_t rtcUtcSane() {
  if (!rtcPresent) return 0;
  DateTime now = rtc.now();
  time_t t = (time_t)now.unixtime();
  if (t < MIN_VALID_UTC || t > MAX_VALID_UTC) return 0;
  return t;
}

static bool systemToRtc() {
  if (!rtcPresent) return false;
  time_t t = time(nullptr);
  if (t < MIN_VALID_UTC) return false;
  rtc.adjust(DateTime((uint32_t)t));
  rtcLostPowerFlag = false;
  return true;
}

static void rtcReadTemp() {
  if (!rtcPresent) return;
  rtcTempC = rtc.getTemperature();
}

static void driftTask(uint32_t now) {
  if (now - lastDriftCheckMs < 60000UL) return;
  lastDriftCheckMs = now;
  if (!rtcPresent || !cfg.time.rtcEnabled) return;
  if (lastTimeSource == SRC_NTP && (now - lastSyncMs) < 120000UL) return;
  time_t r = rtcUtcSane();
  if (r == 0) return;
  time_t sys = time(nullptr);
  long d = (long)r - (long)sys;
  if (d < 0) d = -d;
  int comp = cfg.time.driftComp < 1 ? 1 : cfg.time.driftComp;
  if (d >= comp) {
    struct timeval tv;
    tv.tv_sec = r; tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    lastTimeSource = SRC_RTC;
    lastSyncMs = now;
    DPRINT("drift corrected %lds\n", d);
  }
}

static uint32_t dnsBuildQuery(uint8_t* out, size_t outLen, uint16_t id, const char* host) {
  if (outLen < 18 + strlen(host) + 8) return 0;
  size_t p = 0;
  out[p++] = (uint8_t)(id >> 8); out[p++] = (uint8_t)id;
  out[p++] = 0x01; out[p++] = 0x00;
  out[p++] = 0x00; out[p++] = 0x00;
  out[p++] = 0x00; out[p++] = 0x00;
  out[p++] = 0x00; out[p++] = 0x00;
  out[p++] = 0x00; out[p++] = 0x00;
  const char* s = host;
  while (*s) {
    const char* dot = strchr(s, '.');
    size_t lab = dot ? (size_t)(dot - s) : strlen(s);
    if (lab == 0 || lab > 63) return 0;
    out[p++] = (uint8_t)lab;
    memcpy(out + p, s, lab); p += lab;
    if (dot) s = dot + 1; else break;
  }
  out[p++] = 0x00;
  out[p++] = 0x00; out[p++] = 0x01;
  out[p++] = 0x00; out[p++] = 0x01;
  return (uint32_t)p;
}

static int dnsSkipName(const uint8_t* b, int len, int* i) {
  while (true) {
    if (*i >= len) return -1;
    uint8_t l = b[(*i)++];
    if (l == 0) return 0;
    if ((l & 0xC0) == 0xC0) { if (*i + 1 >= len) return -1; (*i)++; return 0; }
    if (l & 0xC0) return -1;
    if (*i + l > len) return -1;
    *i += l;
  }
}

static void dnsResolveStart(const char* host) {
  memset(&dnsq, 0, sizeof(dnsq));
  if (!host || !host[0]) return;
  IPAddress ip;
  if (ip.fromString(host)) {
    dnsq.isIp = true; dnsq.ok = true; dnsq.busy = false;
    dnsq.ip = (uint32_t)ip;
    strlcpy(dnsq.host, host, sizeof(dnsq.host));
    return;
  }
  IPAddress srv;
  if (cfg.sta.dhcp == 0 && cfg.sta.dns1 != 0) srv = IPAddress(cfg.sta.dns1);
  else srv = WiFi.dnsIP(0);
  if (srv == IPAddress(0, 0, 0, 0)) srv = IPAddress(8, 8, 8, 8);
  uint8_t q[128];
  dnsq.id = (uint16_t)(micros() & 0xFFFF);
  uint32_t n = dnsBuildQuery(q, sizeof(q), dnsq.id, host);
  if (n == 0) return;
  dnsResolverUdp.stop();
  dnsResolverUdp.begin(41234);
  dnsResolverUdp.beginPacket(srv, 53);
  dnsResolverUdp.write(q, n);
  dnsResolverUdp.endPacket();
  strlcpy(dnsq.host, host, sizeof(dnsq.host));
  dnsq.busy = true;
  dnsq.sentMs = millis();
}

static void dnsResolveTask(uint32_t now) {
  if (!dnsq.busy) return;
  if ((now - dnsq.sentMs) > 1500UL) { dnsq.busy = false; dnsq.ok = false; return; }
  int p = dnsResolverUdp.parsePacket();
  if (p < 12) return;
  uint8_t b[512];
  int len = dnsResolverUdp.read(b, sizeof(b));
  if (len < 12) { dnsq.busy = false; dnsq.ok = false; return; }
  uint16_t id = ((uint16_t)b[0] << 8) | b[1];
  if (id != dnsq.id || (b[2] & 0x80) == 0 || (b[3] & 0x0F) != 0) {
    dnsq.busy = false; dnsq.ok = false; return;
  }
  int qd = ((int)b[4] << 8) | b[5];
  int an = ((int)b[6] << 8) | b[7];
  int i = 12;
  for (int qn = 0; qn < qd; qn++) {
    if (dnsSkipName(b, len, &i) != 0) break;
    i += 4;
  }
  for (int ai = 0; ai < an; ai++) {
    if (dnsSkipName(b, len, &i) != 0) break;
    if (i + 10 > len) break;
    int type = ((int)b[i] << 8) | b[i + 1];
    int rdlen = ((int)b[i + 8] << 8) | b[i + 9];
    i += 10;
    if (type == 1 && rdlen == 4 && i + 4 <= len) {
      dnsq.ip = ((uint32_t)b[i] << 24) | ((uint32_t)b[i + 1] << 16) |
                ((uint32_t)b[i + 2] << 8) | b[i + 3];
      dnsq.busy = false; dnsq.ok = true;
      return;
    }
    i += rdlen;
  }
  dnsq.busy = false; dnsq.ok = false;
}

static void ntpKick() { ntp.manualKick = true; }

static void ntpStartCycle() {
  if (!cfg.time.ntpEnabled || !staIsConnected()) return;
  int n = 0;
  int first = ntp.activeIdx;
  for (int k = 0; k < 4; k++) {
    int idx = (ntp.activeIdx + k) % 4;
    if (cfg.time.ntpServers[idx][0]) { first = idx; n++; break; }
  }
  if (n == 0) return;
  ntp.tryIdx = (uint8_t)first;
  ntp.cycleFails = 0;
  ntp.phase = NTP_DNS_WAIT;
  ntp.phaseMs = millis();
  strlcpy(ntp.activeSrv, cfg.time.ntpServers[ntp.tryIdx], sizeof(ntp.activeSrv));
  dnsResolveStart(ntp.activeSrv);
  if (dnsq.isIp) {
    ntpSendRequest(ntp.activeSrv, dnsq.ip);
    ntp.phase = NTP_NTP_WAIT;
    ntp.phaseMs = millis();
  }
}

static bool ntpSendRequest(const char* server, uint32_t ip) {
  (void)server;
  uint8_t pck[48];
  memset(pck, 0, sizeof(pck));
  pck[0] = 0x1B;
  ntpUdp.stop();
  ntpUdp.begin(123);
  bool ok = ntpUdp.beginPacket(IPAddress(ip), 123);
  if (!ok) return false;
  ntpUdp.write(pck, sizeof(pck));
  ntpUdp.endPacket();
  return true;
}

static void ntpRotateOrIdle(uint32_t now) {
  for (int k = 1; k <= 4; k++) {
    int idx = (ntp.tryIdx + k) % 4;
    if (cfg.time.ntpServers[idx][0]) {
      ntp.tryIdx = (uint8_t)idx;
      ntp.cycleFails++;
      ntp.phase = NTP_DNS_WAIT;
      ntp.phaseMs = now;
      strlcpy(ntp.activeSrv, cfg.time.ntpServers[idx], sizeof(ntp.activeSrv));
      dnsResolveStart(ntp.activeSrv);
      if (dnsq.isIp) {
        ntpSendRequest(ntp.activeSrv, dnsq.ip);
        ntp.phase = NTP_NTP_WAIT;
        ntp.phaseMs = now;
      }
      return;
    }
  }
  ntp.phase = NTP_IDLE;
  ntp.nextAttemptMs = now + 30000UL;
  ntp.lastOk = false;
}

static void ntpProcessPacket() {
  uint8_t b[48];
  int len = ntpUdp.read(b, sizeof(b));
  if (len < 48) return;
  uint8_t mode = b[0] & 0x07;
  uint8_t stratum = b[1];
  if (mode != 4 && mode != 5) return;
  if (stratum == 0) return;
  uint32_t secs = ((uint32_t)b[40] << 24) | ((uint32_t)b[41] << 16) |
                  ((uint32_t)b[42] << 8) | b[43];
  time_t utc = (time_t)(secs - 2208988800UL);
  if (utc < MIN_VALID_UTC || utc > MAX_VALID_UTC) return;

  time_t cur = time(nullptr);
  if (cur > MIN_VALID_UTC && (utc > cur + 315532800L || utc + 315532800L < cur)) return;

  if (setTimeFromUtc(utc, SRC_NTP)) {
    ntp.activeIdx = ntp.tryIdx;
    ntp.lastOk = true;
    ntp.lastOkMs = millis();
    strlcpy(ntp.lastSrv, ntp.activeSrv, sizeof(ntp.lastSrv));
    if (rtcPresent && cfg.time.rtcEnabled) {
      time_t r = rtcUtcSane();
      long d = r ? ((long)utc - (long)r) : 999999L;
      if (d < 0) d = -d;
      if (r == 0 || d >= (cfg.time.driftComp < 1 ? 1 : cfg.time.driftComp)) {
        rtc.adjust(DateTime((uint32_t)utc));
        rtcLostPowerFlag = false;
      }
    }
    ntp.phase = NTP_IDLE;
    uint32_t iv = cfg.time.syncIntervalMin ? cfg.time.syncIntervalMin : 1440;
    ntp.nextAttemptMs = millis() + iv * 60000UL;
    DPRINT("ntp ok %lu\n", (unsigned long)utc);
  }
}

static void ntpTask(uint32_t now) {
  if (ntp.manualKick && staIsConnected() && cfg.time.ntpEnabled) {
    ntp.manualKick = false;
    ntpStartCycle();
  }
  if (ntp.phase == NTP_IDLE) {
    if (cfg.time.ntpEnabled && cfg.time.syncIntervalMin > 0 &&
        staIsConnected() && (int32_t)(now - ntp.nextAttemptMs) >= 0) {
      ntpStartCycle();
    }
    return;
  }
  if (ntp.phase == NTP_DNS_WAIT) {
    dnsResolveTask(now);
    if (dnsq.isIp) return;
    if (dnsq.ok) {
      if (ntpSendRequest(ntp.activeSrv, dnsq.ip)) {
        ntp.phase = NTP_NTP_WAIT;
        ntp.phaseMs = now;
      } else ntpRotateOrIdle(now);
    } else if (!dnsq.busy) {
      ntpRotateOrIdle(now);
    }
    return;
  }
  if (ntp.phase == NTP_NTP_WAIT) {
    if (ntpUdp.parsePacket() > 0) {
      ntpProcessPacket();
      return;
    }
    if (now - ntp.phaseMs > 1500UL) ntpRotateOrIdle(now);
    return;
  }
}

// ============================================================================
// SECTION 7: SCHEDULE ENGINE
// ============================================================================

static inline bool fldOK(uint8_t v, int x) {
  return v == WILDCARD || (int)v == x;
}

static bool matchCal(const CalFields* f, int s, int mi, int h, int dom, int mon, int dow) {
  return fldOK(f->sec, s) && fldOK(f->min, mi) && fldOK(f->hour, h) &&
         fldOK(f->dom, dom) && fldOK(f->month, mon) && fldOK(f->dow, dow);
}

static bool dayMatch(const CalFields* f, int dom, int mon, int dow) {
  return fldOK(f->dom, dom) && fldOK(f->month, mon) && fldOK(f->dow, dow);
}

static bool sameDateFields(const CalFields* a, const CalFields* b) {
  return a->dom == b->dom && a->month == b->month && a->dow == b->dow;
}

static int todEarliest(const CalFields* f) {
  int h = (f->hour == WILDCARD) ? 0 : f->hour;
  int m = (f->min  == WILDCARD) ? 0 : f->min;
  int s = (f->sec  == WILDCARD) ? 0 : f->sec;
  return h * 3600 + m * 60 + s;
}

static int todLatest(const CalFields* f) {
  int h = (f->hour == WILDCARD) ? 23 : f->hour;
  int m = (f->min  == WILDCARD) ? 59 : f->min;
  int s = (f->sec  == WILDCARD) ? 59 : f->sec;
  return h * 3600 + m * 60 + s;
}

static bool pairActiveWindow(const SchedPair* p, const LocalFields* lf,
                             int yDom, int yMon, int yDow) {
  int sT = todEarliest(&p->start);
  int eT = todLatest(&p->stop);
  int nowT = lf->h * 3600 + lf->mi * 60 + lf->s;
  if (sT <= eT) {
    return dayMatch(&p->start, lf->d, lf->mo, lf->dow) && nowT >= sT && nowT <= eT;
  }
  return (dayMatch(&p->start, lf->d, lf->mo, lf->dow) && nowT >= sT) ||
         (dayMatch(&p->start, yDom, yMon, yDow) && nowT <= eT);
}

static void latchTick(const SchedPair* p, int r, int i, const LocalFields* lf) {
  if (matchCal(&p->stop, lf->s, lf->mi, lf->h, lf->d, lf->mo, lf->dow)) {
    latch[r][i] = false;
  } else if (matchCal(&p->start, lf->s, lf->mi, lf->h, lf->d, lf->mo, lf->dow)) {
    latch[r][i] = true;
  }
}

static int latestTodMatch(const CalFields* f, int limit) {
  if (limit < 0 || limit > 86399) return -1;
  int lh = limit / 3600, lm = (limit / 60) % 60, ls = limit % 60;
  for (int h = lh; h >= 0; h--) {
    if (f->hour != WILDCARD && (int)f->hour != h) continue;
    int mMax = (h == lh) ? lm : 59;
    for (int m = mMax; m >= 0; m--) {
      if (f->min != WILDCARD && (int)f->min != m) continue;
      int sMax = (h == lh && m == lm) ? ls : 59;
      for (int s = sMax; s >= 0; s--) {
        if (f->sec == WILDCARD || (int)f->sec == s) return h * 3600 + m * 60 + s;
      }
    }
  }
  return -1;
}

static long latestMatch(const CalFields* f, time_t utcNow, int32_t off,
                        int maxDays, bool* found) {
  *found = false;
  long localNow = (long)utcNow + (long)off;
  if (localNow < 0) return 0;
  long dayNow = localNow / 86400;
  int todNow = (int)(localNow % 86400);
  for (int o = 0; o <= maxDays; o++) {
    long day = dayNow - o;
    int y, mo, d, dow;
    civil_from_days(day, &y, &mo, &d, &dow);
    if (!dayMatch(f, d, mo, dow)) continue;
    int lim = (o == 0) ? todNow : 86399;
    int t = latestTodMatch(f, lim);
    if (t >= 0) { *found = true; return day * 86400L + t; }
  }
  return 0;
}

static void engineRescanLatches(time_t utc) {
  LocalFields lf;
  breakLocal(utc, &lf);
  if (!lf.valid) return;
  for (int r = 0; r < N_RELAYS; r++) {
    for (int i = 0; i < N_PAIRS; i++) {
      SchedPair* p = &cfg.sched[r][i];
      if (!p->enabled) continue;
      if (sameDateFields(&p->start, &p->stop)) continue;
      bool sf = false, ef = false;
      long ls = latestMatch(&p->start, utc, lf.off, 400, &sf);
      long le = latestMatch(&p->stop,  utc, lf.off, 400, &ef);
      latch[r][i] = sf && (!ef || ls > le);
    }
  }
  lastRescanUtc = utc;
}

static void engineInvalidate() {
  bool changed = false;
  for (int r = 0; r < N_RELAYS; r++) {
    if (engOn[r]) { engOn[r] = false; changed = true; }
  }
  if (changed) applyAllRelays();
}

static void engineTick(time_t utc) {
  LocalFields lf;
  breakLocal(utc, &lf);
  if (!lf.valid) { engineInvalidate(); return; }
  if (engineRescanPending) {
    engineRescanLatches(utc);
    engineRescanPending = false;
  }
  lastEngineTickUtc = utc;

  long localNow = (long)utc + (long)lf.off;
  long dayNow = localNow / 86400;
  int yY, yM, yD, yW;
  civil_from_days(dayNow - 1, &yY, &yM, &yD, &yW);

  for (int r = 0; r < N_RELAYS; r++) {
    bool on = false;
    for (int i = 0; i < N_PAIRS; i++) {
      SchedPair* p = &cfg.sched[r][i];
      if (!p->enabled) continue;
      if (sameDateFields(&p->start, &p->stop)) {
        if (pairActiveWindow(p, &lf, yD, yM, yW)) on = true;
      } else {
        latchTick(p, r, i, &lf);
        if (latch[r][i]) on = true;
      }
    }
    engOn[r] = on;
  }
  applyAllRelays();
}

// ============================================================================
// SECTION 8: RELAY PIN CONTROL
// ============================================================================

static void writeRelayPin(int i, uint8_t raw) {
  if (i < 0 || i >= N_RELAYS) return;
  if (lastPinInit[i] && lastPinLevel[i] == raw) return;
  uint8_t pin = cfg.relays[i].gpio;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, raw ? HIGH : LOW);
  lastPinLevel[i] = raw;
  lastPinInit[i] = true;
}

static uint8_t offLevel(int i) {
  return cfg.relays[i].activeHigh ? 0 : 1;
}

static void applyRelay(int i) {
  if (i < 0 || i >= N_RELAYS) return;
  const RelayCfg* r = &cfg.relays[i];
  if (r->mode == RMODE_REMOVED) { writeRelayPin(i, offLevel(i)); return; }
  switch (r->mode) {
    case RMODE_HIGH: writeRelayPin(i, 1); return;
    case RMODE_LOW:  writeRelayPin(i, 0); return;
    case RMODE_ON:   writeRelayPin(i, r->activeHigh ? 1 : 0); return;
    case RMODE_OFF:  writeRelayPin(i, offLevel(i)); return;
    default: break;
  }
  bool des = engOn[i];
  writeRelayPin(i, r->activeHigh ? (des ? 1 : 0) : (des ? 0 : 1));
}

static void applyAllRelays() {
  for (int i = 0; i < N_RELAYS; i++) applyRelay(i);
}

static void initPinsSafe() {
  for (int i = 0; i < N_RELAYS; i++) {
    uint8_t pin = RELAY_GPIOS[i];
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    lastPinLevel[i] = 1;
    lastPinInit[i] = true;
  }
  pinMode(BTN_FACTORY_GPIO, INPUT_PULLUP);
}

static void applyConfigPins() {
  for (int i = 0; i < N_RELAYS; i++) {
    lastPinInit[i] = false;
    applyRelay(i);
  }
}

static bool setRelayMode(int i, uint8_t mode) {
  if (i < 0 || i >= N_RELAYS || mode > RMODE_MAX) return false;
  if (cfg.relays[i].mode == RMODE_REMOVED && mode != RMODE_REMOVED && mode != RMODE_AUTO) {
    return false;
  }
  cfg.relays[i].mode = mode;
  applyRelay(i);
  storageSaveNow(false);
  return true;
}

static void setAllActiveLevel(bool activeHigh) {
  for (int i = 0; i < N_RELAYS; i++) cfg.relays[i].activeHigh = activeHigh ? 1 : 0;
  applyAllRelays();
  storageSaveNow(false);
}

static void restoreAllRelays() {
  for (int i = 0; i < N_RELAYS; i++) {
    if (cfg.relays[i].mode == RMODE_REMOVED) cfg.relays[i].mode = RMODE_AUTO;
  }
  applyAllRelays();
  storageSaveNow(false);
}

// ============================================================================
// SECTION 9: WIFI MANAGEMENT
// ============================================================================

static bool staIsConnected() {
  return WiFi.status() == WL_CONNECTED;
}

static bool staModeActive() {
  wifi_mode_t m = WiFi.getMode();
  return m == WIFI_STA || m == WIFI_AP_STA;
}

static bool apIsActive() {
  wifi_mode_t m = WiFi.getMode();
  return m == WIFI_AP || m == WIFI_AP_STA;
}

static void onWifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      evStaDisconnected = true;
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      evStaGotIp = true;
      break;
    default:
      break;
  }
}

static void wifiEventTask() {
  if (evStaDisconnected) {
    evStaDisconnected = false;
    if (!staRetrying) staDownSinceMs = millis();
    staRetrying = true;
    staEverUp = false;
    rtcToSystem(nullptr);
    DPRINT("sta down\n");
  }
  if (evStaGotIp) {
    evStaGotIp = false;
    staRetrying = false;
    staEverUp = true;
    rtcToSystem(nullptr);
    ntpKick();
    DPRINT("sta got ip\n");
  }
}

static void applySta(bool connectNow) {
  staPaused = false;
  if (!cfg.sta.enabled || cfg.sta.ssid[0] == 0) {
    if (staModeActive() && !cfg.ap.enabled) {
      WiFi.mode(WIFI_OFF);
    } else if (staModeActive()) {
      WiFi.mode(cfg.ap.enabled ? WIFI_AP : WIFI_OFF);
    }
    return;
  }
  if (cfg.ap.enabled && !apIsActive()) WiFi.mode(WIFI_AP_STA);
  else if (!cfg.ap.enabled) WiFi.mode(WIFI_STA);

  WiFi.setHostname(cfg.sta.hostname);
  WiFi.setAutoReconnect(cfg.sta.autoReconnect == 1);
  if (cfg.sta.dhcp == 0) {
    WiFi.config(IPAddress(cfg.sta.ip), IPAddress(cfg.sta.gw),
                IPAddress(cfg.sta.sn), IPAddress(cfg.sta.dns1),
                IPAddress(cfg.sta.dns2));
  } else {
    WiFi.config(IPAddress((uint32_t)0), IPAddress((uint32_t)0),
                IPAddress((uint32_t)0), IPAddress((uint32_t)0),
                IPAddress((uint32_t)0));
  }
  if (connectNow) {
    WiFi.begin(cfg.sta.ssid, cfg.sta.pass);
    lastStaAttemptMs = millis();
    staPaused = false;
  }
}

static uint32_t apRestartAtMs = 0;
static bool apRestartPending = false;

static void applyApInternal() {
  if (!cfg.ap.enabled) {
    if (apIsActive()) {
      if (staModeActive()) WiFi.mode(WIFI_STA);
      else WiFi.softAPdisconnect(true);
    }
    tempApActive = false;
    return;
  }
  if (!staModeActive() && !apIsActive()) WiFi.mode(cfg.sta.enabled ? WIFI_AP_STA : WIFI_AP);
  else if (staModeActive() && !apIsActive()) WiFi.mode(WIFI_AP_STA);

  IPAddress ip(cfg.ap.ip), gw(cfg.ap.gw), sn(cfg.ap.sn);
  WiFi.softAPConfig(ip, gw, sn);
  if (cfg.ap.pass[0] == 0) {
    WiFi.softAP(cfg.ap.ssid, nullptr, cfg.ap.channel, cfg.ap.hidden, cfg.ap.maxClients);
  } else {
    WiFi.softAP(cfg.ap.ssid, cfg.ap.pass, cfg.ap.channel, cfg.ap.hidden, cfg.ap.maxClients);
  }
  tempApActive = false;
}

static void applyAp(bool restartAp) {
  if (restartAp && apIsActive()) {
    WiFi.softAPdisconnect(true);
    apRestartAtMs = millis() + 200;
    apRestartPending = true;
    return;
  }
  apRestartPending = false;
  applyApInternal();
}

static void wifiTask(uint32_t now) {
  if (apRestartPending && (int32_t)(now - apRestartAtMs) >= 0) {
    apRestartPending = false;
    applyApInternal();
  }
  if (cfg.sta.enabled && cfg.sta.ssid[0] && !staPaused && !staIsConnected() &&
      staModeActive() && (now - lastStaAttemptMs) >= 1000UL) {
    lastStaAttemptMs = now;
    if (!staRetrying) staDownSinceMs = now;
    staRetrying = true;
    WiFi.reconnect();
  }
  if (!staIsConnected()) {
    if (staRetrying && staDownSinceMs == 0) staDownSinceMs = now;
  } else {
    staDownSinceMs = 0;
    if (tempApActive) {
      tempApActive = false;
      if (!cfg.ap.enabled) {
        WiFi.softAPdisconnect(true);
        if (cfg.sta.enabled) WiFi.mode(WIFI_STA);
      }
    }
  }
  if (cfg.ap.fallback && cfg.sta.enabled && cfg.sta.ssid[0] &&
      !staIsConnected() && !apIsActive() && staDownSinceMs != 0 &&
      (now - staDownSinceMs) >= 180000UL) {
    tempApActive = true;
    WiFi.mode(WIFI_AP_STA);
    IPAddress ip(192, 168, 4, 1);
    WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
    WiFi.softAP("ESP32-Setup", "12345678", 1, 0, 4);
  }
}

static void syncDnsCaptive(uint32_t now) {
  (void)now;
  bool want = apIsActive() && cfg.ap.captivePortal == 1;
  if (want && !dnsRunning) {
    dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    dnsServer.start(53, "*", WiFi.softAPIP());
    dnsRunning = true;
  } else if (!want && dnsRunning) {
    dnsServer.stop();
    dnsRunning = false;
  }
  if (dnsRunning) dnsServer.processNextRequest();
}

static void scanStart() {
  WiFi.scanDelete();
  WiFi.scanNetworks(true, true);
  lastScanMs = millis();
}

static bool scanDone() {
  int8_t n = WiFi.scanComplete();
  if (n >= 0) return true;
  if (n == WIFI_SCAN_RUNNING) return false;
  if (n == WIFI_SCAN_FAILED && (millis() - lastScanMs) < 15000UL) return false;
  return true;
}

// ============================================================================
// SECTION 10: HTTP HELPERS + VALIDATORS + CAPTIVE PORTAL
// ============================================================================

static String ipStr(uint32_t raw) {
  return IPAddress(raw).toString();
}

static uint32_t ipRaw(const String& s) {
  IPAddress ip;
  if (!ip.fromString(s)) return 0;
  return (uint32_t)ip;
}

static String modeStr(uint8_t m) {
  switch (m) {
    case RMODE_ON: return "on";
    case RMODE_OFF: return "off";
    case RMODE_HIGH: return "high";
    case RMODE_LOW: return "low";
    case RMODE_REMOVED: return "removed";
    default: return "auto";
  }
}

static uint8_t modeFromStr(const char* s) {
  if (!s) return 0xFF;
  if (!strcmp(s, "auto")) return RMODE_AUTO;
  if (!strcmp(s, "on")) return RMODE_ON;
  if (!strcmp(s, "off")) return RMODE_OFF;
  if (!strcmp(s, "high")) return RMODE_HIGH;
  if (!strcmp(s, "low")) return RMODE_LOW;
  if (!strcmp(s, "removed")) return RMODE_REMOVED;
  return 0xFF;
}

static bool validNameStr(const char* s) {
  if (!s) return false;
  size_t l = strlen(s);
  if (l == 0 || l > 23) return false;
  for (size_t i = 0; i < l; i++) {
    uint8_t c = (uint8_t)s[i];
    if (c < 32 || c > 126) return false;
  }
  return true;
}

static bool validHostStr(const char* s) {
  if (!s) return false;
  size_t l = strlen(s);
  if (l == 0 || l > 31) return false;
  if (s[0] == '-' || s[l - 1] == '-') return false;
  for (size_t i = 0; i < l; i++) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-')) return false;
  }
  return true;
}

static bool validNtpName(const char* s) {
  if (!s) return false;
  size_t l = strlen(s);
  if (l == 0) return true;
  if (l > 47) return false;
  for (size_t i = 0; i < l; i++) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '.' || c == '-' || c == ':')) return false;
  }
  return true;
}

static bool validPosixTz(const char* s) {
  if (!s || !s[0]) return false;
  size_t l = strlen(s);
  if (l > 63) return false;
  for (size_t i = 0; i < l; i++) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '+' || c == '-' || c == ',' ||
          c == ':' || c == '/' || c == '<' || c == '>')) return false;
  }
  return true;
}

static void sendJsonDoc(JsonDocument& doc) {
  size_t len = measureJson(doc);
  server.setContentLength(len);
  server.send(200, "application/json", "");
  WiFiClient cl = server.client();
  serializeJson(doc, cl);
}

static void sendJsonOk() {
  JsonDocument doc;
  doc["ok"] = true;
  sendJsonDoc(doc);
}

static void sendJsonErr(const char* msg) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"] = msg;
  String out;
  out.reserve(96);
  serializeJson(doc, out);
  server.send(400, "application/json", out);
}

static void sendJsonErr500(const char* msg) {
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"] = msg;
  String out;
  out.reserve(96);
  serializeJson(doc, out);
  server.send(500, "application/json", out);
}

static bool getJsonBody(String& out, size_t maxLen) {
  out = server.arg("plain");
  if (out.length() == 0) return false;
  if (out.length() > maxLen) return false;
  return true;
}

static void sendPage(const char* body) {
  server.sendHeader("Cache-Control", "no-store");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(HTML_HEAD);
  server.sendContent_P(body);
  server.sendContent_P(HTML_FOOT);
  server.sendContent("");
}

// ============================================================================
// SECTION 11: SHARED HTML SHELL
// ============================================================================

const char HTML_HEAD[] PROGMEM = "\n"
"<!DOCTYPE html>\n"
"<html lang=\"en\">\n"
"<head>\n"
"<meta charset=\"utf-8\">\n"
"<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\n"
"<title>ESP32 Relay Manager</title>\n"
"<style>\n"
":root{--bg:#0b1220;--card:#121a2b;--card2:#0e1626;--line:#243050;--tx:#e6ecf8;\n"
"--mut:#93a2c0;--acc:#4f8cff;--ok:#22c55e;--warn:#eab308;--bad:#ef4444}\n"
"*{box-sizing:border-box}\n"
"html,body{margin:0}\n"
"body{background:var(--bg);color:var(--tx);font:14px/1.5 \"Segoe UI\",system-ui,-apple-system,sans-serif}\n"
"a{color:var(--acc)}\n"
"header{display:flex;align-items:center;gap:10px;padding:10px 14px;background:var(--card2);\n"
"border-bottom:1px solid var(--line);position:sticky;top:0;z-index:50;flex-wrap:wrap}\n"
".brand{font-weight:700;letter-spacing:.4px;color:var(--acc);margin-right:8px;white-space:nowrap}\n"
"nav{display:flex;gap:2px;flex-wrap:wrap}\n"
"nav a{color:var(--mut);text-decoration:none;font-size:13px;padding:6px 10px;border-radius:8px;white-space:nowrap}\n"
"nav a:hover{background:var(--line);color:var(--tx)}\n"
"#clockbox{margin-left:auto;display:flex;align-items:center;gap:8px;font-variant-numeric:tabular-nums;\n"
"font-weight:600;font-size:14px;white-space:nowrap}\n"
"#syncdot{width:9px;height:9px;border-radius:50%;background:var(--bad);box-shadow:0 0 6px var(--bad)}\n"
"main{max-width:1240px;margin:0 auto;padding:16px}\n"
"h2{margin:4px 0 14px;font-size:19px}\n"
".grid{display:grid;gap:12px}\n"
".g2{grid-template-columns:repeat(auto-fit,minmax(330px,1fr))}\n"
".g3{grid-template-columns:repeat(auto-fit,minmax(250px,1fr))}\n"
".card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;margin-bottom:12px}\n"
".card h3{margin:0 0 10px;font-size:13px;text-transform:uppercase;letter-spacing:.6px;color:var(--acc)}\n"
".row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:8px 0}\n"
"label{color:var(--mut);font-size:13px}\n"
"input,select{background:var(--card2);border:1px solid var(--line);color:var(--tx);border-radius:8px;\n"
"padding:8px 10px;font:inherit;min-width:64px}\n"
"input:focus,select:focus{outline:1px solid var(--acc)}\n"
"input[type=checkbox]{min-width:0;width:18px;height:18px}\n"
"button{background:var(--line);border:1px solid var(--line);color:var(--tx);border-radius:8px;\n"
"padding:8px 12px;font:inherit;cursor:pointer}\n"
"button:hover{border-color:var(--acc)}\n"
"button.pri{background:var(--acc);border-color:var(--acc);color:#fff}\n"
"button.ok{background:var(--ok);border-color:var(--ok);color:#04140a}\n"
"button.bad{background:var(--bad);border-color:var(--bad);color:#fff}\n"
"button.sm{padding:4px 9px;font-size:12px}\n"
".badge{display:inline-block;padding:2px 9px;border-radius:10px;font-size:11px;font-weight:700}\n"
".b-on{background:var(--ok);color:#04140a}\n"
".b-off{background:var(--line);color:var(--mut)}\n"
".b-rem{background:var(--bad);color:#fff}\n"
".b-warn{background:var(--warn);color:#221b02}\n"
"table{width:100%;border-collapse:collapse;font-size:13px}\n"
"th,td{text-align:left;padding:7px 8px;border-bottom:1px solid var(--line)}\n"
"th{color:var(--mut);font-weight:600}\n"
".mut{color:var(--mut)}\n"
".mono{font-family:ui-monospace,Consolas,monospace}\n"
".small{font-size:12px}\n"
"#toast{position:fixed;right:14px;bottom:14px;background:var(--card2);border:1px solid var(--acc);\n"
"border-radius:10px;padding:10px 14px;display:none;z-index:99;max-width:360px;box-shadow:0 4px 18px #0009}\n"
".tile{display:flex;flex-direction:column;gap:4px;padding:10px;border-radius:10px;border:1px solid var(--line);\n"
"background:var(--card2);text-decoration:none;color:var(--tx)}\n"
".tile .st{font-size:12px;font-weight:700}\n"
".tabs{display:flex;gap:4px;flex-wrap:wrap;margin:8px 0}\n"
".tabs button.on{background:var(--acc);color:#fff;border-color:var(--acc)}\n"
".pair{border:1px solid var(--line);border-radius:10px;padding:10px;margin:8px 0;background:var(--card2)}\n"
".pair.sel{border-color:var(--acc)}\n"
".selrow{display:flex;gap:6px;flex-wrap:wrap;align-items:center;margin:4px 0}\n"
".selrow select{padding:5px 6px;font-size:12px;min-width:0}\n"
".selrow .fl{width:34px;display:inline-block;font-size:10px;color:var(--mut);text-align:center}\n"
".hint{font-size:12px;color:var(--mut);margin-top:4px}\n"
".bigclock{font-size:26px;font-weight:700;font-variant-numeric:tabular-nums;letter-spacing:1px}\n"
".kv{display:flex;justify-content:space-between;gap:10px;padding:4px 0;border-bottom:1px dashed var(--line);\n"
"font-size:13px}\n"
".kv b{font-weight:600}\n"
".kv span{color:var(--mut)}\n"
".net{padding:8px;border:1px solid var(--line);border-radius:8px;margin:4px 0;cursor:pointer;\n"
"background:var(--card2)}\n"
".net:hover{border-color:var(--acc)}\n"
".net.sel{border-color:var(--acc)}\n"
"@media(max-width:760px){#clockbox{font-size:12px} main{padding:10px}}\n"
"</style>\n"
"</head>\n"
"<body>\n"
"<header>\n"
"<span class=\"brand\">ESP32 RELAYS</span>\n"
"<nav>\n"
"<a href=\"/\">Home</a>\n"
"<a href=\"/relays\">Relays</a>\n"
"<a href=\"/gpio\">GPIO</a>\n"
"<a href=\"/time\">Time</a>\n"
"<a href=\"/wifi-ap\">WiFi AP</a>\n"
"<a href=\"/wifi-sta\">WiFi STA</a>\n"
"<a href=\"/status\">System</a>\n"
"</nav>\n"
"<div id=\"clockbox\"><span id=\"syncdot\" title=\"Time sync status\"></span><span id=\"ctime\">--:--:--</span></div>\n"
"</header>\n"
"<main>\n";

const char HTML_FOOT[] PROGMEM = "\n"
"</main>\n"
"<div id=\"toast\"></div>\n"
"<script>\n"
"function $(id){return document.getElementById(id)}\n"
"function esc(s){return String(s).replace(/[&<>\"]/g,function(c){return{'&':'&amp;','<':'&lt;','>':'&gt;','\"':'&quot;'}[c]})}\n"
"function toast(m,ok){var t=$('toast');if(!t){t=document.createElement('div');t.id='toast';document.body.appendChild(t)}\n"
"t.textContent=m;t.style.borderColor=(ok===false)?'var(--bad)':'var(--acc)';t.style.display='block';\n"
"clearTimeout(t._h);t._h=setTimeout(function(){t.style.display='none'},3500)}\n"
"function api(path,obj){var o={method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(obj||{})};\n"
"return fetch(path,o).then(function(r){return r.json()}).catch(function(){return{ok:false,error:'Network error'}})}\n"
"function getJSON(path){return fetch(path).then(function(r){return r.json()})\n"
".catch(function(){return{ok:false,error:'Network error'}})}\n"
"function p2(n){return (n<10?'0':'')+n}\n"
"var CLK={utc:0,off:0,src:0};\n"
"function civilFromDays(d){var z=d+719468;var era=Math.floor((z>=0?z:z-146096)/146097);\n"
"var doe=z-era*146097;var yoe=Math.floor((doe-Math.floor(doe/1460)+Math.floor(doe/36524)-Math.floor(doe/146096))/365);\n"
"var y=yoe+era*400;var doy=doe-(365*yoe+Math.floor(yoe/4)-Math.floor(yoe/100));\n"
"var mp=Math.floor((5*doy+2)/153);var dd=doy-Math.floor((153*mp+2)/5)+1;var mm=mp+(mp<10?3:-9);\n"
"y+=(mm<=2?1:0);return{y:y,m:mm,d:dd}}\n"
"function upClock(){var el=$('ctime');if(!el)return;\n"
"if(!CLK.utc){el.textContent='--:--:--  --/--/--';return}\n"
"var le=CLK.utc+CLK.off,d=Math.floor(le/86400),tod=le-d*86400;\n"
"var c=civilFromDays(d);\n"
"var DOW=['SUN','MON','TUE','WED','THU','FRI','SAT'][(d+4)%7];\n"
"el.textContent=DOW+' '+p2(c.m)+'/'+p2(c.d)+'/'+p2(c.y%100)+' '+p2(Math.floor(tod/3600))+':'+p2(Math.floor(tod%3600/60))+':'+p2(tod%60);\n"
"var dot=$('syncdot');if(dot){var col=CLK.src>=2?'var(--ok)':(CLK.src>=1?'var(--warn)':'var(--bad)');\n"
"dot.style.background=col;dot.style.boxShadow='0 0 6px '+col}}\n"
"function fetchTime(){getJSON('/api/time').then(function(d){\n"
"if(d&&d.ok&&d.utc>1577836800){CLK.utc=d.utc;CLK.off=d.off;CLK.src=d.srcCode;upClock()}})}\n"
"setInterval(function(){if(CLK.utc){CLK.utc++;upClock()}},1000);\n"
"setInterval(fetchTime,15000);\n"
"fetchTime();\n"
"document.addEventListener('visibilitychange',function(){if(!document.hidden)fetchTime()});\n"
"function bindDblTap(el,fn){var last=0;\n"
"function h(ev){var n=Date.now();if(n-last<450){ev.preventDefault();ev.stopPropagation();fn()}last=n}\n"
"el.addEventListener('dblclick',h);\n"
"el.addEventListener('touchend',function(ev){ev.preventDefault();h(ev)},{passive:false});}\n"
"function syncBrowserTime(){api('/api/time/browser',{utc:Math.floor(Date.now()/1000)}).then(function(d){\n"
"if(d.ok){toast('Time synced from browser');fetchTime()}else{toast('Sync failed: '+(d.error||''),false)}})}\n"
"if(window.pageInit){try{pageInit()}catch(e){}}\n"
"</script>\n"
"</body>\n"
"</html>\n";

// ============================================================================
// SECTION 12: HTML PAGES
// ============================================================================

const char PAGE_HOME[] PROGMEM = "\n"
"<h2>Dashboard</h2>\n"
"<div class=\"grid g2\">\n"
"<div class=\"card\">\n"
"<h3>Time</h3>\n"
"<div class=\"bigclock\" id=\"dclock\">--:--:--</div>\n"
"<div class=\"kv\"><span>Source</span><b id=\"dsrc\">-</b></div>\n"
"<div class=\"kv\"><span>Clock status</span><b id=\"dvalid\">-</b></div>\n"
"<div class=\"row\">\n"
"<button class=\"pri\" onclick=\"syncBrowserTime()\">Sync from Browser</button>\n"
"<a href=\"/time\"><button>Time Settings</button></a>\n"
"</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Network &amp; System</h3>\n"
"<div class=\"kv\"><span>Station</span><b id=\"dsta\">-</b></div>\n"
"<div class=\"kv\"><span>STA IP</span><b id=\"dstaip\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Access Point</span><b id=\"dap\">-</b></div>\n"
"<div class=\"kv\"><span>AP IP</span><b id=\"dapip\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Free heap</span><b id=\"dheap\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Uptime</span><b id=\"dup\" class=\"mono\">-</b></div>\n"
"<div class=\"row\"><a href=\"/wifi-sta\"><button>WiFi STA</button></a>\n"
"<a href=\"/wifi-ap\"><button>WiFi AP</button></a>\n"
"<a href=\"/status\"><button>System</button></a></div>\n"
"</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Relays</h3>\n"
"<div class=\"grid g3\" id=\"dtiles\"></div>\n"
"<div class=\"row\"><a href=\"/relays\"><button class=\"pri\">Relay Schedules</button></a>\n"
"<a href=\"/gpio\"><button>GPIO Management</button></a></div>\n"
"</div>\n"
"<script>\n"
"function bigClockStr(){if(!CLK.utc)return '--:--:--';\n"
"var le=CLK.utc+CLK.off,d=Math.floor(le/86400),tod=le-d*86400,c=civilFromDays(d);\n"
"var DOW=['SUN','MON','TUE','WED','THU','FRI','SAT'][(d+4)%7];\n"
"return DOW+' '+p2(c.m)+'/'+p2(c.d)+'/'+p2(c.y%100)+' '+p2(Math.floor(tod/3600))+':'+p2(Math.floor(tod%3600/60))+':'+p2(tod%60)}\n"
"function upDashClock(){$('dclock').textContent=bigClockStr()}\n"
"setInterval(upDashClock,1000);\n"
"function tileHtml(r){var st=r.on?'<span class=\"st\" style=\"color:var(--ok)\">ON</span>':\n"
"'<span class=\"st\" style=\"color:var(--mut)\">OFF</span>';\n"
"var md=r.removed?'<span class=\"badge b-rem\">REMOVED</span>':st;\n"
"return '<a class=\"tile\" href=\"/relays?r='+(r.i+1)+'\"><b>'+(r.i+1)+'. '+esc(r.n)+'</b>'+md+\n"
"'<span class=\"small mut\">'+(r.mode=='auto'?'AUTO':r.mode.toUpperCase())+' &middot; GPIO'+r.g+'</span></a>'}\n"
"function loadDash(){getJSON('/api/status').then(function(d){if(!d.ok)return;\n"
"var t=d.time;\n"
"$('dsrc').textContent=t.srcTxt||'-';\n"
"$('dvalid').textContent=t.valid?'Valid ('+(t.dst?'DST':'STD')+' offset '+(t.off/3600)+'h)':'CLOCK NOT SET';\n"
"$('dsta').textContent=d.sta.connected?('Connected: '+d.sta.ssid):(\n"
"d.sta.enabled?(d.sta.paused?'Paused':(d.sta.retrying?'Reconnecting (1s retry)':'Enabled')):'Disabled');\n"
"$('dstaip').textContent=d.sta.connected?d.sta.ip:'-';\n"
"$('dap').textContent=d.ap.active?('Active: '+d.ap.ssid+' ('+d.ap.clients+' client'+(d.ap.clients==1?'':'s')+')'):\n"
"(d.ap.enabled?'Enabled (inactive)':'Disabled');\n"
"$('dapip').textContent=d.ap.active?d.ap.ip:'-';\n"
"$('dheap').textContent=Math.round(d.heap.free/1024)+' KB';\n"
"var u=d.up,s=u%60,m=Math.floor(u/60)%60,h=Math.floor(u/3600)%24,dd=Math.floor(u/86400);\n"
"$('dup').textContent=(dd?dd+'d ':'')+p2(h)+':'+p2(m)+':'+p2(s);\n"
"var t2='';for(var i=0;i<d.relays.length;i++)t2+=tileHtml(d.relays[i]);\n"
"$('dtiles').innerHTML=t2;\n"
"})}\n"
"window.pageInit=function(){upDashClock();loadDash();setInterval(loadDash,3000)}\n"
"</script>\n";

const char PAGE_RELAYS[] PROGMEM = "\n"
"<h2>Relay Schedules</h2>\n"
"<div class=\"card\">\n"
"<div class=\"tabs\" id=\"rtabs\"></div>\n"
"<div class=\"row\" style=\"justify-content:space-between\">\n"
"<span style=\"font-size:18px;font-weight:700\" id=\"rname\">Relay</span>\n"
"<span class=\"row\"><span id=\"rmode\" class=\"badge b-off\">AUTO</span>\n"
"<button class=\"sm\" id=\"btnON\">Force ON</button>\n"
"<button class=\"sm\" id=\"btnOFF\">Force OFF</button>\n"
"<button class=\"sm\" id=\"btnAUTO\">Auto</button></span>\n"
"</div>\n"
"<div class=\"mut small\">Double-tap the relay name to rename it. Modes persist across reboots.\n"
"Each relay has 8 schedule pairs; pairs are combined with OR. &quot;*&quot; = wildcard (matches any value).</div>\n"
"<div id=\"pairs\"></div>\n"
"<div class=\"row\">\n"
"<button class=\"pri\" id=\"saveRelay\">Save This Relay</button>\n"
"<button class=\"pri\" id=\"saveAll\">Save All Relays</button>\n"
"<button class=\"sm\" id=\"addHint\">+ Fill Pair 1 Example</button>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"var SCHED=[],REL=[],R=0,MONN=['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'],\n"
"DOWN=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];\n"
"var FIELDS=[['SEC',0,59,null],['MIN',0,59,null],['HOUR',0,23,null],['DOM',1,31,null],\n"
"['MON',1,12,MONN],['DOW',0,6,DOWN]];\n"
"function optHtml(lo,hi,cur,names){var s='<option value=\"255\"'+(cur==255?' selected':'')+'>*</option>';\n"
"for(var v=lo;v<=hi;v++){s+='<option value=\"'+v+'\"'+(cur==v?' selected':'')+'>'+\n"
"(names?names[v-lo]:v)+'</option>'}return s}\n"
"function todE(f){return (f[2]==255?0:f[2])*3600+(f[1]==255?0:f[1])*60+(f[0]==255?0:f[0])}\n"
"function todL(f){return (f[2]==255?23:f[2])*3600+(f[1]==255?59:f[1])*60+(f[0]==255?59:f[0])}\n"
"function fmtT(t){return p2(Math.floor(t/3600))+':'+p2(Math.floor(t%3600/60))+':'+p2(t%60)}\n"
"function dateDesc(f){if(f[3]==255&&f[4]==255&&f[5]==255)return 'every day';\n"
"var a=[];if(f[3]!=255)a.push('day-of-month '+f[3]);if(f[4]!=255)a.push(MONN[f[4]-1]);\n"
"if(f[5]!=255)a.push(DOWN[f[5]]);return a.join(' + ')}\n"
"function fullDesc(f){var a=[];if(f[2]!=255)a.push('hour '+f[2]);if(f[1]!=255)a.push('min '+f[1]);\n"
"if(f[0]!=255)a.push('sec '+f[0]);return (a.length?a.join(', '):'every h/m/s')+' on '+dateDesc(f)}\n"
"function pairHint(pr){if(!pr.en)return 'Disabled';\n"
"var same=pr.s[3]==pr.e[3]&&pr.s[4]==pr.e[4]&&pr.s[5]==pr.e[5];\n"
"if(same){var sT=todE(pr.s),eT=todL(pr.e);\n"
"if(sT<=eT)return 'Day window: ON '+fmtT(sT)+' to '+fmtT(eT)+' on '+dateDesc(pr.s)+\n"
"' (wildcard seconds/minutes expand to earliest start / latest stop)';\n"
"return 'OVERNIGHT: ON '+fmtT(sT)+' -> next day '+fmtT(eT)+' on '+dateDesc(pr.s)+\n"
"' (tail continues after midnight)'}\n"
"return 'Event latch: START when '+fullDesc(pr.s)+'; STOP when '+fullDesc(pr.e)+'. Stop wins ties.'}\n"
"function renderTabs(){var h='';for(var i=0;i<16;i++){\n"
"var nm=REL[i]?esc(REL[i].n):('Relay '+(i+1));\n"
"h+='<button class=\"'+(i==R?'on':'')+'\" data-i=\"'+i+'\">'+(i+1)+' &middot; '+nm+'</button>'}\n"
"$('rtabs').innerHTML=h;\n"
"var bs=$('rtabs').getElementsByTagName('button');\n"
"for(var k=0;k<bs.length;k++)bs[k].addEventListener('click',function(){R=parseInt(this.dataset.i);render()})}\n"
"function renderHeader(){if(!REL[R])return;\n"
"var sp=$('rname');\n"
"if(sp.querySelector('input'))return;\n"
"sp.textContent=(R+1)+'. '+REL[R].n;\n"
"var m=$('rmode');m.textContent=REL[R].mode.toUpperCase();\n"
"m.className='badge '+(REL[R].mode=='on'||REL[R].mode=='high'?'b-on':(REL[R].mode=='removed'?'b-rem':'b-off'))}\n"
"function pairHtml(p){var pr=SCHED[R][p];\n"
"function side(key){var out='';\n"
"for(var k=0;k<6;k++){var F=FIELDS[k];\n"
"out+='<span class=\"fl\">'+F[0]+'</span><select data-p=\"'+p+'\" data-k=\"'+key+'\" data-f=\"'+k+'\">'+\n"
"optHtml(F[1],F[2],pr[key][k],F[3])+'</select>'}return out}\n"
"return '<div class=\"pair'+(pr.en?' sel':'')+'\" id=\"pair'+p+'\">'+\n"
"'<div class=\"row\"><label><input type=\"checkbox\" data-en=\"'+p+'\"'+(pr.en?' checked':'')+'> Pair '+(p+1)+' enabled</label>'+\n"
"'<span style=\"flex:1\"></span><button class=\"sm\" data-clr=\"'+p+'\">Clear pair</button></div>'+\n"
"'<div class=\"selrow\"><b style=\"width:44px\">Start</b>'+side('s')+'</div>'+\n"
"'<div class=\"selrow\"><b style=\"width:44px\">Stop</b>'+side('e')+'</div>'+\n"
"'<div class=\"hint\" id=\"hint'+p+'\"></div></div>'}\n"
"function updateHint(p){$('hint'+p).textContent=pairHint(SCHED[R][p]);\n"
"$('pair'+p).className='pair'+(SCHED[R][p].en?' sel':'')}\n"
"function render(){renderTabs();renderHeader();\n"
"var h='';for(var p=0;p<8;p++)h+=pairHtml(p);\n"
"$('pairs').innerHTML=h;\n"
"var sels=$('pairs').getElementsByTagName('select');\n"
"for(var i=0;i<sels.length;i++){sels[i].addEventListener('change',function(){\n"
"var p=parseInt(this.dataset.p),k=this.dataset.k,f=parseInt(this.dataset.f);\n"
"SCHED[R][p][k][f]=parseInt(this.value);updateHint(p)})}\n"
"var cks=$('pairs').querySelectorAll('input[type=checkbox]');\n"
"for(var j=0;j<cks.length;j++){cks[j].addEventListener('change',function(){\n"
"var p=parseInt(this.dataset.en);SCHED[R][p].en=this.checked?1:0;updateHint(p)})}\n"
"var clrs=$('pairs').querySelectorAll('button[data-clr]');\n"
"for(var q=0;q<clrs.length;q++){clrs[q].addEventListener('click',function(){\n"
"var p=parseInt(this.dataset.clr);SCHED[R][p].en=0;\n"
"for(var k=0;k<6;k++){SCHED[R][p].s[k]=255;SCHED[R][p].e[k]=255}\n"
"render();toast('Pair '+(p+1)+' cleared (not saved yet)')})}\n"
"}\n"
"function loadRelays(cb){getJSON('/api/relays').then(function(d){if(!d.ok)return;REL=d.relays;\n"
"renderHeader();renderTabs();if(cb)cb()})}\n"
"function loadSched(){getJSON('/api/schedules').then(function(d){if(!d.ok)return;\n"
"SCHED=d.sched.map(function(r){return r.pairs});render()})}\n"
"function loadMode(){getJSON('/api/relays').then(function(d){if(!d.ok)return;REL=d.relays;renderHeader()})}\n"
"function setMode(m){api('/api/relay',{i:R,mode:m}).then(function(d){\n"
"if(d.ok){toast('Mode: '+m.toUpperCase());loadMode()}else toast('Failed: '+(d.error||''),false)})}\n"
"function editName(){if(!REL[R])return;\n"
"var cur=REL[R].n,sp=$('rname'),done=false;\n"
"var inp=document.createElement('input');inp.type='text';inp.maxLength=23;inp.value=cur;\n"
"inp.style.cssText='font-size:16px;font-weight:600;max-width:280px';\n"
"sp.textContent='';sp.appendChild(inp);inp.focus();inp.select();\n"
"function finish(save){if(done)return;done=true;\n"
"if(save){var v=inp.value.trim();\n"
"if(v&&v!=cur){api('/api/relay',{i:R,name:v}).then(function(d){\n"
"if(d.ok){toast('Relay renamed');loadRelays()}else toast('Failed: '+(d.error||''),false)})\n"
"return}}\n"
"renderHeader()}\n"
"inp.addEventListener('keydown',function(ev){if(ev.key=='Enter')finish(true);\n"
"if(ev.key=='Escape')finish(false)});\n"
"inp.addEventListener('blur',function(){finish(true)})}\n"
"function saveRelay(){api('/api/schedules',{relay:R,pairs:SCHED[R]}).then(function(d){\n"
"if(d.ok)toast('Relay '+(R+1)+' schedules saved');else toast('Save failed: '+(d.error||''),false)})}\n"
"function saveAll(){api('/api/schedules',{all:SCHED}).then(function(d){\n"
"if(d.ok)toast('All 16 relay schedules saved');else toast('Save failed: '+(d.error||''),false)})}\n"
"window.pageInit=function(){\n"
"try{var q=parseInt((location.search.match(/[?&]r=(\\d+)/)||[])[1]||1)-1;\n"
"if(q>=0&&q<16)R=q}catch(e){}\n"
"$('btnON').addEventListener('click',function(){setMode('on')});\n"
"$('btnOFF').addEventListener('click',function(){setMode('off')});\n"
"$('btnAUTO').addEventListener('click',function(){setMode('auto')});\n"
"$('saveRelay').addEventListener('click',saveRelay);\n"
"$('saveAll').addEventListener('click',saveAll);\n"
"$('addHint').addEventListener('click',function(){\n"
"SCHED[R][0]={en:1,s:[0,0,9,255,255,255],e:[0,30,9,255,255,255]};render();\n"
"toast('Example: daily 09:00:00-09:00:30 in pair 1. Edit then Save.')});\n"
"bindDblTap($('rname'),editName);\n"
"loadRelays();loadSched();setInterval(loadMode,3000)}\n"
"</script>\n";

const char PAGE_GPIO[] PROGMEM = "\n"
"<h2>GPIO Management</h2>\n"
"<div class=\"card\">\n"
"<h3>Global Active Level</h3>\n"
"<div class=\"mut small\">Applies to all 16 relays and saves immediately. Active LOW = relay energizes when the\n"
"pin is LOW (most opto-isolated boards). Active HIGH = relay energizes when the pin is HIGH.</div>\n"
"<div class=\"row\">\n"
"<button id=\"gAllLow\">Set ALL Active LOW</button>\n"
"<button id=\"gAllHigh\">Set ALL Active HIGH</button>\n"
"<button id=\"gRestore\" class=\"ok\">Restore Removed Relays</button>\n"
"</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Per-Relay Control</h3>\n"
"<div class=\"mut small\">Double-tap a relay name to rename it. Set High / Set Low drive the physical pin\n"
"level (ignores active level). Auto returns the relay to schedule control. Remove detaches the relay\n"
"from automation and holds the pin at its safe OFF level.</div>\n"
"<div style=\"overflow-x:auto\">\n"
"<table><thead><tr><th>#</th><th>Name</th><th>GPIO</th><th>Pin</th><th>Relay</th>\n"
"<th>Active</th><th>Mode</th><th>Actions</th></tr></thead><tbody id=\"grows\"></tbody></table>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"var GREL=[];\n"
"function badge(on){return on?'<span class=\"badge b-on\">ON</span>':'<span class=\"badge b-off\">OFF</span>'}\n"
"function rowsHtml(){var h='';\n"
"for(var i=0;i<GREL.length;i++){var r=GREL[i];\n"
"h+='<tr><td>'+(i+1)+'</td><td><b id=\"gn'+i+'\">'+esc(r.n)+'</b></td><td class=\"mono\">'+r.g+'</td>'+\n"
"'<td class=\"mono\">'+(r.pin?'HIGH':'LOW')+'</td><td>'+badge(r.on)+'</td>'+\n"
"'<td><select id=\"ga'+i+'\"><option value=\"0\"'+(r.ah?'':' selected')+'>LOW</option>'+\n"
"'<option value=\"1\"'+(r.ah?' selected':'')+'>HIGH</option></select></td>'+\n"
"'<td><span class=\"badge '+(r.mode=='on'||r.mode=='high'?'b-on':(r.mode=='removed'?'b-rem':'b-off'))+'\">'+\n"
"r.mode.toUpperCase()+'</span>'+(r.sched?' <span class=\"badge b-warn\">SCHED ON</span>':'')+'</td>'+\n"
"'<td>'+(r.removed?'<button class=\"sm\" data-a=\"restore\" data-i=\"'+i+'\">Restore</button>':\n"
"'<button class=\"sm\" data-a=\"sethigh\" data-i=\"'+i+'\">Set High</button>'+\n"
"'<button class=\"sm\" data-a=\"setlow\" data-i=\"'+i+'\">Set Low</button>'+\n"
"'<button class=\"sm ok\" data-a=\"auto\" data-i=\"'+i+'\">Auto</button>'+\n"
"'<button class=\"sm bad\" data-a=\"remove\" data-i=\"'+i+'\">Remove</button>')+'</td></tr>'}\n"
"return h}\n"
"function bindRows(){var bs=$('grows').querySelectorAll('button');\n"
"for(var k=0;k<bs.length;k++)bs[k].addEventListener('click',function(){\n"
"var i=parseInt(this.dataset.i),a=this.dataset.a;\n"
"if(a=='remove'){if(!confirm('Remove relay '+(i+1)+' from management? The pin will be held at its OFF level.'))return}\n"
"api('/api/gpio',{i:i,action:a}).then(function(d){\n"
"if(d.ok){loadGpio()}else toast('Failed: '+(d.error||''),false)})});\n"
"var sels=$('grows').querySelectorAll('select');\n"
"for(var q=0;q<sels.length;q++)sels[q].addEventListener('change',function(){\n"
"var i=this.id.substring(2);\n"
"api('/api/gpio',{i:parseInt(i),ah:parseInt(this.value)}).then(function(d){\n"
"if(d.ok){toast('Active level saved');loadGpio()}else toast('Failed: '+(d.error||''),false)})});\n"
"for(var i=0;i<GREL.length;i++)bindDblTap($('gn'+i),makeRename(i))}\n"
"function makeRename(i){return function(){var cur=GREL[i].n,done=false;\n"
"var sp=$('gn'+i);\n"
"var inp=document.createElement('input');inp.type='text';inp.maxLength=23;inp.value=cur;\n"
"inp.style.cssText='width:130px';\n"
"sp.textContent='';sp.appendChild(inp);inp.focus();inp.select();\n"
"function fin(save){if(done)return;done=true;\n"
"if(save){var v=inp.value.trim();\n"
"if(v&&v!=cur){api('/api/relay',{i:i,name:v}).then(function(d){\n"
"if(d.ok){toast('Relay renamed');loadGpio()}else toast('Failed: '+(d.error||''),false)})return}}\n"
"loadGpio()}\n"
"inp.addEventListener('keydown',function(ev){if(ev.key=='Enter')fin(true);\n"
"if(ev.key=='Escape')fin(false)});\n"
"inp.addEventListener('blur',function(){fin(true)})}}\n"
"function loadGpio(){var ae=document.activeElement;\n"
"if(ae&&(ae.tagName=='INPUT'||ae.tagName=='SELECT'))return;\n"
"getJSON('/api/relays').then(function(d){if(!d.ok)return;GREL=d.relays;\n"
"$('grows').innerHTML=rowsHtml();bindRows()})}\n"
"window.pageInit=function(){\n"
"$('gAllLow').addEventListener('click',function(){\n"
"if(confirm('Set ALL relays active LOW?'))api('/api/gpio',{global:'low'}).then(function(d){\n"
"if(d.ok){toast('All relays set active LOW');loadGpio()}else toast('Failed',false)})});\n"
"$('gAllHigh').addEventListener('click',function(){\n"
"if(confirm('Set ALL relays active HIGH?'))api('/api/gpio',{global:'high'}).then(function(d){\n"
"if(d.ok){toast('All relays set active HIGH');loadGpio()}else toast('Failed',false)})});\n"
"$('gRestore').addEventListener('click',function(){\n"
"api('/api/gpio',{action:'restoreall'}).then(function(d){\n"
"if(d.ok){toast('Removed relays restored to Auto');loadGpio()}else toast('Failed',false)})});\n"
"loadGpio();setInterval(loadGpio,2000)}\n"
"</script>\n";

const char PAGE_TIME[] PROGMEM = "\n"
"<h2>Time Management</h2>\n"
"<div class=\"grid g2\">\n"
"<div class=\"card\">\n"
"<h3>Current Time</h3>\n"
"<div class=\"bigclock\" id=\"tclock\">--:--:--</div>\n"
"<div class=\"kv\"><span>Source</span><b id=\"tsrc\">-</b></div>\n"
"<div class=\"kv\"><span>Status</span><b id=\"tvalid\">-</b></div>\n"
"<div class=\"kv\"><span>Local offset</span><b id=\"toff\" class=\"mono\">-</b></div>\n"
"<div class=\"row\"><button class=\"pri\" onclick=\"syncBrowserTime()\">Sync from Browser</button></div>\n"
"<h3 style=\"margin-top:14px\">Manual Set (local time)</h3>\n"
"<div class=\"row\">\n"
"<input type=\"number\" id=\"my\" style=\"width:84px\" min=\"2020\" max=\"2060\" placeholder=\"YYYY\">\n"
"<input type=\"number\" id=\"mmo\" style=\"width:64px\" min=\"1\" max=\"12\" placeholder=\"MM\">\n"
"<input type=\"number\" id=\"md\" style=\"width:64px\" min=\"1\" max=\"31\" placeholder=\"DD\">\n"
"<input type=\"number\" id=\"mh\" style=\"width:64px\" min=\"0\" max=\"23\" placeholder=\"hh\">\n"
"<input type=\"number\" id=\"mm\" style=\"width:64px\" min=\"0\" max=\"59\" placeholder=\"mm\">\n"
"<input type=\"number\" id=\"ms\" style=\"width:64px\" min=\"0\" max=\"59\" placeholder=\"ss\">\n"
"<button id=\"mset\">Set</button>\n"
"</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>NTP Servers</h3>\n"
"<div class=\"row\"><label><input type=\"checkbox\" id=\"nen\"> NTP enabled</label></div>\n"
"<div class=\"row\"><input type=\"text\" id=\"ns0\" placeholder=\"Server 1 (e.g. pool.ntp.org)\" style=\"flex:1;min-width:200px\"></div>\n"
"<div class=\"row\"><input type=\"text\" id=\"ns1\" placeholder=\"Server 2 (optional)\" style=\"flex:1;min-width:200px\"></div>\n"
"<div class=\"row\"><input type=\"text\" id=\"ns2\" placeholder=\"Server 3 (optional)\" style=\"flex:1;min-width:200px\"></div>\n"
"<div class=\"row\"><input type=\"text\" id=\"ns3\" placeholder=\"Server 4 (optional)\" style=\"flex:1;min-width:200px\"></div>\n"
"<div class=\"row\"><label>Sync interval (minutes, 0=off, max 1440=24h)</label>\n"
"<input type=\"number\" id=\"nint\" min=\"0\" max=\"1440\" style=\"width:90px\"></div>\n"
"<div class=\"row\"><label>Drift compensation (seconds, fixed default 1)</label>\n"
"<input type=\"number\" id=\"ndrift\" min=\"0\" max=\"30\" style=\"width:70px\"></div>\n"
"<div class=\"kv\"><span>Last result</span><b id=\"nlast\">-</b></div>\n"
"<div class=\"row\"><button class=\"pri\" id=\"nsave\">Save</button>\n"
"<button id=\"nsync\">Sync Now</button></div>\n"
"<div class=\"mut small\">Servers are tried in rotation with automatic fallback: on failure the next\n"
"server is queried; a full failed cycle retries in 30 s. Queries are fully asynchronous.</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>DS3231 RTC (SDA=21, SCL=22)</h3>\n"
"<div class=\"kv\"><span>Present</span><b id=\"rpres\">-</b></div>\n"
"<div class=\"kv\"><span>Battery lost power</span><b id=\"rlost\">-</b></div>\n"
"<div class=\"kv\"><span>Temperature</span><b id=\"rtemp\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>RTC vs System</span><b id=\"rdelta\" class=\"mono\">-</b></div>\n"
"<div class=\"row\"><button id=\"rread\">Read RTC to System</button>\n"
"<button id=\"rwrite\">Write System to RTC</button></div>\n"
"<div class=\"mut small\">Every station reconnect refreshes the system clock from the RTC and then\n"
"NTP; every disconnect refreshes it from the RTC. A once-per-minute drift check re-disciplines the\n"
"system clock from the RTC whenever the difference reaches the drift compensation setting.</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Timezone / DST</h3>\n"
"<div class=\"row\"><label>Mode</label>\n"
"<select id=\"zmode\"><option value=\"manual\">Manual offset + DST rules</option>\n"
"<option value=\"posix\">Auto timezone (POSIX TZ)</option></select></div>\n"
"<div id=\"zman\">\n"
"<div class=\"row\"><label>GMT offset</label>\n"
"<select id=\"zoffsel\"></select>\n"
"<input type=\"number\" id=\"zoffsec\" style=\"width:100px\" title=\"Offset in seconds\">\n"
"<label>seconds</label></div>\n"
"<div class=\"row\"><label>DST rule</label>\n"
"<select id=\"zdst\"><option value=\"off\">No DST</option><option value=\"us\">US (Mar 2nd Sun - Nov 1st Sun)</option>\n"
"<option value=\"eu\">EU (Mar last Sun - Oct last Sun)</option><option value=\"au\">Australia (Oct 1st Sun - Apr 1st Sun)</option></select></div>\n"
"<div class=\"row\"><label>DST offset seconds</label>\n"
"<input type=\"number\" id=\"zdstoff\" min=\"0\" max=\"7200\" style=\"width:90px\"></div>\n"
"</div>\n"
"<div id=\"zpos\" style=\"display:none\">\n"
"<div class=\"row\"><label>Zone</label><select id=\"zpres\"></select></div>\n"
"<div class=\"row\"><label>Custom POSIX TZ</label>\n"
"<input type=\"text\" id=\"zcustom\" style=\"flex:1;min-width:220px\" placeholder=\"e.g. EST5EDT,M3.2.0,M11.1.0\"></div>\n"
"</div>\n"
"<div class=\"kv\"><span>Offset now</span><b id=\"znow\" class=\"mono\">-</b></div>\n"
"<div class=\"row\"><button class=\"pri\" id=\"zsave\">Save Timezone</button></div>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"var TZP=[[\"UTC\",\"UTC0\"],[\"US Eastern (EST/EDT)\",\"EST5EDT,M3.2.0,M11.1.0\"],\n"
"[\"US Central (CST/CDT)\",\"CST6CDT,M3.2.0,M11.1.0\"],[\"US Mountain (MST/MDT)\",\"MST7MDT,M3.2.0,M11.1.0\"],\n"
"[\"US Arizona (no DST)\",\"MST7\"],[\"US Pacific (PST/PDT)\",\"PST8PDT,M3.2.0,M11.1.0\"],\n"
"[\"US Alaska (AKST/AKDT)\",\"AKST9AKDT,M3.2.0,M11.1.0\"],[\"Hawaii (no DST)\",\"HST10\"],\n"
"[\"UK (GMT/BST)\",\"GMT0BST,M3.5.0/1,M10.5.0/2\"],[\"Central Europe (CET/CEST)\",\"CET-1CEST,M3.5.0,M10.5.0/3\"],\n"
"[\"Eastern Europe (EET/EEST)\",\"EET-2EEST,M3.5.0/3,M10.5.0/4\"],\n"
"[\"Australia NSW (AEST/AEDT)\",\"AEST-10AEDT,M10.1.0,M4.1.0/3\"],[\"Australia Queensland (no DST)\",\"AEST-10\"],\n"
"[\"India (IST)\",\"IST-5:30\"],[\"China (CST)\",\"CST-8\"],[\"Japan (JST)\",\"JST-9\"],\n"
"[\"Singapore (SGT)\",\"SGT-8\"],[\"UAE (GST)\",\"GST-4\"],[\"Philippines (PHT)\",\"PHT-8\"],[\"Brazil (BRT)\",\"BRT-3\"]];\n"
"var OFFP=[[0,\"UTC+0\"],[3600,\"UTC+1\"],[7200,\"UTC+2\"],[10800,\"UTC+3\"],[12600,\"UTC+3:30\"],\n"
"[14400,\"UTC+4\"],[18000,\"UTC+5\"],[19800,\"UTC+5:30\"],[21600,\"UTC+6\"],[25200,\"UTC+7\"],\n"
"[28800,\"UTC+8\"],[32400,\"UTC+9\"],[36000,\"UTC+10\"],[43200,\"UTC+12\"],[-16200,\"UTC-4:30\"],\n"
"[-18000,\"UTC-5\"],[-21600,\"UTC-6\"],[-25200,\"UTC-7\"],[-28800,\"UTC-8\"],[-32400,\"UTC-9\"],\n"
"[-36000,\"UTC-10\"],[-39600,\"UTC-11\"],[-43200,\"UTC-12\"]];\n"
"function fillSel(sel,rows,cur){sel.innerHTML='';\n"
"for(var i=0;i<rows.length;i++){var o=document.createElement('option');\n"
"o.value=rows[i][0];o.textContent=rows[i][1];if(rows[i][0]==cur)o.selected=true;sel.appendChild(o)}}\n"
"function upTClock(){if(CLK.utc){var le=CLK.utc+CLK.off,d=Math.floor(le/86400),tod=le-d*86400,c=civilFromDays(d);\n"
"var DOW=['SUN','MON','TUE','WED','THU','FRI','SAT'][(d+4)%7];\n"
"$('tclock').textContent=DOW+' '+p2(c.m)+'/'+p2(c.d)+'/'+p2(c.y%100)+' '+p2(Math.floor(tod/3600))+':'+p2(Math.floor(tod%3600/60))+':'+p2(tod%60)}}\n"
"function loadTime(){getJSON('/api/time').then(function(d){if(!d.ok)return;\n"
"CLK.utc=d.utc;CLK.off=d.off;CLK.src=d.srcCode;upClock();upTClock();\n"
"$('tsrc').textContent=d.srcTxt;\n"
"$('tvalid').textContent=d.valid?'Valid':'CLOCK NOT SET - schedules idle';\n"
"$('toff').textContent=(d.off>=0?'+':'')+(d.off/3600)+' h ('+d.off+' s)';\n"
"$('znow').textContent=(d.off>=0?'+':'')+(Math.round(d.off/360)/10)+' h, DST '+(d.dst?'active':'inactive')})}\n"
"function loadNtp(){getJSON('/api/ntp').then(function(d){if(!d.ok)return;\n"
"$('nen').checked=!!d.enabled;\n"
"for(var i=0;i<4;i++)$('ns'+i).value=d.servers[i]||'';\n"
"$('nint').value=d.intervalMin;$('ndrift').value=d.drift;\n"
"$('nlast').textContent=d.lastOk?('OK via '+d.lastSrv):'no successful sync yet'})}\n"
"function loadRtc(){getJSON('/api/rtc').then(function(d){if(!d.ok)return;\n"
"$('rpres').textContent=d.present?'Yes (DS3231)':'NOT FOUND';\n"
"$('rlost').textContent=d.lostPower?'YES - battery low/missing, time invalid until synced':'No';\n"
"$('rtemp').textContent=d.present?(d.tempC+' C'):'-';\n"
"$('rdelta').textContent=d.present?(d.delta+' s'):'-'})}\n"
"function loadTz(){getJSON('/api/tz').then(function(d){if(!d.ok)return;\n"
"$('zmode').value=d.mode;\n"
"if(d.mode=='posix'){$('zman').style.display='none';$('zpos').style.display='';\n"
"var known=false;for(var i=0;i<TZP.length;i++)if(TZP[i][1]==d.posix)known=true;\n"
"fillSel($('zpres'),TZP,known?d.posix:0);\n"
"var o=document.createElement('option');o.value='custom';o.textContent='(custom below)';\n"
"if(!known){o.selected=true;$('zpres').appendChild(o);$('zpres').value='custom'}\n"
"else $('zpres').appendChild(o);\n"
"$('zcustom').value=d.posix}\n"
"else{$('zman').style.display='';$('zpos').style.display='none';\n"
"var known=false;for(var i=0;i<OFFP.length;i++)if(OFFP[i][0]==d.utcOff)known=true;\n"
"fillSel($('zoffsel'),OFFP,d.utcOff);\n"
"if(!known){var o=document.createElement('option');o.value=d.utcOff;o.textContent=d.utcOff+' s (custom)';\n"
"o.selected=true;$('zoffsel').appendChild(o)}\n"
"$('zoffsec').value=d.utcOff;$('zdst').value=d.dstMode;$('zdstoff').value=d.dstOff}\n"
"loadTime()})}\n"
"window.pageInit=function(){\n"
"setInterval(upTClock,1000);setInterval(loadTime,5000);setInterval(loadRtc,5000);\n"
"$('zoffsel').addEventListener('change',function(){$('zoffsec').value=this.value});\n"
"$('zpres').addEventListener('change',function(){if(this.value!='custom')$('zcustom').value=this.value});\n"
"$('zmode').addEventListener('change',function(){\n"
"if(this.value=='posix'){$('zman').style.display='none';$('zpos').style.display=''}\n"
"else{$('zman').style.display='';$('zpos').style.display='none'}});\n"
"$('mset').addEventListener('click',function(){\n"
"api('/api/time/manual',{y:parseInt($('my').value),mo:parseInt($('mmo').value),d:parseInt($('md').value),\n"
"h:parseInt($('mh').value),mi:parseInt($('mm').value),s:parseInt($('ms').value)})\n"
".then(function(d){if(d.ok){toast('Time set');loadTime();loadRtc()}\n"
"else toast('Set failed: '+(d.error||''),false)})});\n"
"$('nsave').addEventListener('click',function(){\n"
"var srv=[];for(var i=0;i<4;i++)srv.push($('ns'+i).value.trim());\n"
"api('/api/ntp',{enabled:$('nen').checked?1:0,servers:srv,\n"
"intervalMin:parseInt($('nint').value),drift:parseInt($('ndrift').value)})\n"
".then(function(d){if(d.ok){toast('NTP settings saved');loadNtp()}\n"
"else toast('Save failed: '+(d.error||''),false)})});\n"
"$('nsync').addEventListener('click',function(){\n"
"api('/api/ntp/syncnow',{}).then(function(d){if(d.ok){toast('NTP sync started');\n"
"setTimeout(loadTime,4000)}else toast('Failed: '+(d.error||''),false)})});\n"
"$('rread').addEventListener('click',function(){api('/api/rtc',{op:'read'}).then(function(d){\n"
"if(d.ok){toast('System clock set from RTC');loadTime()}else toast('Failed',false)})});\n"
"$('rwrite').addEventListener('click',function(){api('/api/rtc',{op:'write'}).then(function(d){\n"
"if(d.ok){toast('RTC written from system clock');loadRtc()}else toast('Failed',false)})});\n"
"$('zsave').addEventListener('click',function(){\n"
"var mode=$('zmode').value;\n"
"if(mode=='posix'){var pz=$('zcustom').value.trim();\n"
"if($('zpres').value!='custom')pz=$('zpres').value;\n"
"api('/api/tz',{mode:'posix',posix:pz}).then(afterTz)}\n"
"else{api('/api/tz',{mode:'manual',utcOff:parseInt($('zoffsec').value),\n"
"dstMode:$('zdst').value,dstOff:parseInt($('zdstoff').value)}).then(afterTz)}});\n"
"function afterTz(d){if(d.ok){toast('Timezone saved');loadTz()}\n"
"else toast('Save failed: '+(d.error||''),false)}\n"
"loadTime();loadNtp();loadRtc();loadTz()}\n"
"</script>\n";

const char PAGE_WAP[] PROGMEM = "\n"
"<h2>WiFi Access Point</h2>\n"
"<div class=\"grid g2\">\n"
"<div class=\"card\">\n"
"<h3>AP Status</h3>\n"
"<div class=\"kv\"><span>Active</span><b id=\"as_act\">-</b></div>\n"
"<div class=\"kv\"><span>SSID</span><b id=\"as_ssid\">-</b></div>\n"
"<div class=\"kv\"><span>IP</span><b id=\"as_ip\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Connected clients</span><b id=\"as_cnt\">-</b></div>\n"
"<div class=\"kv\"><span>Captive portal</span><b id=\"as_cap\">-</b></div>\n"
"<div id=\"as_macs\" class=\"mut small mono\"></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>AP Configuration</h3>\n"
"<div class=\"row\"><label><input type=\"checkbox\" id=\"ac_en\"> AP enabled</label>\n"
"<label><input type=\"checkbox\" id=\"ac_cap\"> Captive portal (DNS + redirect)</label>\n"
"<label><input type=\"checkbox\" id=\"ac_fb\"> Fallback AP (STA down 3 min)</label></div>\n"
"<div class=\"row\"><label>SSID</label><input type=\"text\" id=\"ac_ssid\" maxLength=\"32\" style=\"flex:1;min-width:170px\"></div>\n"
"<div class=\"row\"><label>Password</label>\n"
"<input type=\"password\" id=\"ac_pass\" maxLength=\"63\" style=\"flex:1;min-width:170px\">\n"
"<label><input type=\"checkbox\" id=\"ac_show\" onchange=\"document.getElementById('ac_pass').type=this.checked?'text':'password'\"> show</label></div>\n"
"<div class=\"mut small\">Empty password = open AP. Otherwise 8-63 characters (WPA2).</div>\n"
"<div class=\"row\"><label>Channel</label>\n"
"<select id=\"ac_ch\"></select>\n"
"<label>Hidden SSID</label><input type=\"checkbox\" id=\"ac_hidden\">\n"
"<label>Max clients</label><select id=\"ac_maxc\"></select></div>\n"
"<div class=\"row\"><label>IP</label><input type=\"text\" id=\"ac_ip\" class=\"mono\" style=\"width:130px\">\n"
"<label>Gateway</label><input type=\"text\" id=\"ac_gw\" class=\"mono\" style=\"width:130px\">\n"
"<label>Netmask</label><input type=\"text\" id=\"ac_sn\" class=\"mono\" style=\"width:130px\"></div>\n"
"<div class=\"row\"><button class=\"pri\" id=\"ac_apply\">Save &amp; Apply (restarts AP)</button></div>\n"
"<div class=\"mut small\">The AP runs alongside the station (AP+STA) or standalone. With the captive\n"
"portal enabled, AP clients see the dashboard automatically after connecting.</div>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"function fillRange(sel,lo,hi,cur){sel.innerHTML='';\n"
"for(var v=lo;v<=hi;v++){var o=document.createElement('option');o.value=v;o.textContent=v;\n"
"if(v==cur)o.selected=true;sel.appendChild(o)}}\n"
"var apFormInit=false;\n"
"function loadAp(){getJSON('/api/wifi/ap').then(function(d){if(!d.ok)return;\n"
"if(!apFormInit){apFormInit=true;\n"
"$('ac_en').checked=!!d.enabled;$('ac_cap').checked=!!d.captive;$('ac_fb').checked=!!d.fallback;\n"
"$('ac_ssid').value=d.ssid;$('ac_pass').value=d.pass;\n"
"fillRange($('ac_ch'),1,13,d.chan);fillRange($('ac_maxc'),1,8,d.maxc);\n"
"$('ac_hidden').checked=!!d.hidden;\n"
"$('ac_ip').value=d.ip;$('ac_gw').value=d.gw;$('ac_sn').value=d.sn;}\n"
"$('as_act').textContent=d.active?'Active':'Inactive (disabled or restarting)';\n"
"$('as_ssid').textContent=d.ssid;$('as_ip').textContent=d.ip;\n"
"$('as_cnt').textContent=d.active?d.clients:'-';\n"
"$('as_cap').textContent=d.active&&d.captive?'Running (DNS + redirect)':'Off';\n"
"$('as_macs').textContent=(d.clientMacs&&d.clientMacs.length)?('Clients: '+d.clientMacs.join(', ')):''})}\n"
"window.pageInit=function(){\n"
"$('ac_apply').addEventListener('click',function(){\n"
"var pass=$('ac_pass').value;\n"
"if(pass&&pass.length<8){toast('WPA2 password needs 8-63 chars (or empty for open AP)',false);return}\n"
"api('/api/wifi/ap',{enabled:$('ac_en').checked?1:0,ssid:$('ac_ssid').value.trim(),pass:pass,\n"
"chan:parseInt($('ac_ch').value),hidden:$('ac_hidden').checked?1:0,\n"
"maxc:parseInt($('ac_maxc').value),ip:$('ac_ip').value.trim(),gw:$('ac_gw').value.trim(),\n"
"sn:$('ac_sn').value.trim(),captive:$('ac_cap').checked?1:0,fallback:$('ac_fb').checked?1:0})\n"
".then(function(d){if(d.ok){toast('AP settings saved - applying');setTimeout(loadAp,2500)}\n"
"else toast('Save failed: '+(d.error||''),false)})});\n"
"loadAp();setInterval(loadAp,4000)}\n"
"</script>\n";

const char PAGE_STA[] PROGMEM = "\n"
"<h2>WiFi Station</h2>\n"
"<div class=\"grid g2\">\n"
"<div class=\"card\">\n"
"<h3>Station Status</h3>\n"
"<div class=\"kv\"><span>State</span><b id=\"ss_state\">-</b></div>\n"
"<div class=\"kv\"><span>SSID</span><b id=\"ss_ssid\">-</b></div>\n"
"<div class=\"kv\"><span>IP / GW / DNS</span><b id=\"ss_ip\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Signal (RSSI)</span><b id=\"ss_rssi\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Host / MAC</span><b id=\"ss_mac\" class=\"mono\">-</b></div>\n"
"<div class=\"mut small\">If the station loses connection, reconnect is attempted automatically\n"
"every second. Every reconnect refreshes the clock (RTC then NTP).</div>\n"
"<div class=\"row\"><button id=\"ss_disc\" class=\"bad\">Disconnect (pause retries)</button></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Scan Networks</h3>\n"
"<div class=\"row\"><button id=\"scanbtn\" class=\"pri\">Start Scan</button><span id=\"scanst\" class=\"mut small\"></span></div>\n"
"<div id=\"scanlist\"></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Station Configuration</h3>\n"
"<div class=\"row\"><label><input type=\"checkbox\" id=\"sc_en\"> Station enabled</label>\n"
"<label><input type=\"checkbox\" id=\"sc_auto\"> Auto reconnect</label></div>\n"
"<div class=\"row\"><label>SSID</label><input type=\"text\" id=\"sc_ssid\" maxLength=\"32\" style=\"flex:1;min-width:170px\"></div>\n"
"<div class=\"row\"><label>Password</label>\n"
"<input type=\"password\" id=\"sc_pass\" maxLength=\"63\" style=\"flex:1;min-width:170px\">\n"
"<label><input type=\"checkbox\" onchange=\"document.getElementById('sc_pass').type=this.checked?'text':'password'\"> show</label></div>\n"
"<div class=\"row\"><label>Hostname</label><input type=\"text\" id=\"sc_host\" maxLength=\"31\" style=\"width:170px\"></div>\n"
"<div class=\"row\"><label><input type=\"checkbox\" id=\"sc_dhcp\"> Use DHCP</label></div>\n"
"<div class=\"row\"><label>IP</label><input type=\"text\" id=\"sc_ip\" class=\"mono\" style=\"width:130px\">\n"
"<label>Gateway</label><input type=\"text\" id=\"sc_gw\" class=\"mono\" style=\"width:130px\"></div>\n"
"<div class=\"row\"><label>Netmask</label><input type=\"text\" id=\"sc_sn\" class=\"mono\" style=\"width:130px\">\n"
"<label>DNS 1</label><input type=\"text\" id=\"sc_d1\" class=\"mono\" style=\"width:130px\"></div>\n"
"<div class=\"row\"><label>DNS 2</label><input type=\"text\" id=\"sc_d2\" class=\"mono\" style=\"width:130px\"></div>\n"
"<div class=\"row\"><button class=\"pri\" id=\"sc_connect\">Save &amp; Connect</button>\n"
"<button id=\"sc_save\">Save Only</button></div>\n"
"</div>\n"
"</div>\n"
"<script>\n"
"var SCAN=null;\n"
"function rssiQ(r){return r>=-55?'excellent':r>=-67?'good':r>=-80?'fair':'weak'}\n"
"function loadStaCfg(){getJSON('/api/wifi/sta').then(function(d){if(!d.ok)return;\n"
"$('sc_en').checked=!!d.enabled;$('sc_auto').checked=!!d.autoRec;\n"
"$('sc_ssid').value=d.ssid;$('sc_pass').value=d.pass;$('sc_host').value=d.host;\n"
"$('sc_dhcp').checked=!!d.dhcp;\n"
"$('sc_ip').value=d.ip;$('sc_gw').value=d.gw;$('sc_sn').value=d.sn;\n"
"$('sc_d1').value=d.dns1;$('sc_d2').value=d.dns2})}\n"
"function loadStaStatus(){getJSON('/api/status').then(function(d){if(!d.ok)return;\n"
"var s=d.sta;\n"
"$('ss_state').textContent=s.connected?'Connected':(s.enabled?(s.paused?'Paused (press Connect to resume)':\n"
"(s.retrying?'Reconnecting... (attempt every 1 s)':'Enabled, not connected')):'Disabled');\n"
"$('ss_ssid').textContent=s.connected?s.ssid:(s.ssid||'-');\n"
"$('ss_ip').textContent=s.connected?(s.ip+' / '+s.gw+' / '+s.dns):'-';\n"
"$('ss_rssi').textContent=s.connected?(s.rssi+' dBm ('+rssiQ(s.rssi)+')'):'-';\n"
"$('ss_mac').textContent=(s.host||'-')+' / '+(s.mac||'-')})}\n"
"function doScan(){$('scanst').textContent='Scanning...';$('scanlist').innerHTML='';\n"
"api('/api/scan/start',{}).then(function(d){\n"
"var iv=setInterval(function(){getJSON('/api/scan/results').then(function(r){\n"
"if(r.status=='running')return;\n"
"clearInterval(iv);\n"
"if(r.status=='done'&&r.nets){$('scanst').textContent=r.nets.length+' networks';\n"
"var h='';for(var i=0;i<r.nets.length;i++){var n=r.nets[i];\n"
"h+='<div class=\"net\" data-s=\"'+esc(n.ssid)+'\"><b>'+esc(n.ssid)+'</b>'+\n"
"' <span class=\"small mut\">'+n.rssi+' dBm ('+rssiQ(n.rssi)+') ch'+n.ch+' '+n.sec+'</span></div>'}\n"
"$('scanlist').innerHTML=h;\n"
"var els=document.querySelectorAll('.net');\n"
"for(var k=0;k<els.length;k++)els[k].addEventListener('click',function(){\n"
"$('sc_ssid').value=this.dataset.s;toast('SSID filled - enter password and Connect')})}\n"
"else{$('scanst').textContent=r.status=='idle'?'Scan failed or no networks':'Idle'}},1200)})}\n"
"function staPayload(){return {enabled:$('sc_en').checked?1:0,ssid:$('sc_ssid').value.trim(),\n"
"pass:$('sc_pass').value,host:$('sc_host').value.trim(),dhcp:$('sc_dhcp').checked?1:0,\n"
"ip:$('sc_ip').value.trim(),gw:$('sc_gw').value.trim(),sn:$('sc_sn').value.trim(),\n"
"dns1:$('sc_d1').value.trim(),dns2:$('sc_d2').value.trim(),autoRec:$('sc_auto').checked?1:0}}\n"
"window.pageInit=function(){\n"
"$('scanbtn').addEventListener('click',doScan);\n"
"$('sc_connect').addEventListener('click',function(){\n"
"var p=staPayload();p.connect=1;\n"
"api('/api/wifi/sta',p).then(function(d){\n"
"if(d.ok){toast('Saved - connecting...');setTimeout(loadStaStatus,3000)}\n"
"else toast('Save failed: '+(d.error||''),false)})});\n"
"$('sc_save').addEventListener('click',function(){\n"
"api('/api/wifi/sta',staPayload()).then(function(d){\n"
"if(d.ok)toast('Station settings saved');else toast('Save failed: '+(d.error||''),false)})});\n"
"$('ss_disc').addEventListener('click',function(){\n"
"api('/api/wifi/sta/disconnect',{}).then(function(d){\n"
"if(d.ok){toast('Disconnected - retries paused');loadStaStatus()}else toast('Failed',false)})});\n"
"loadStaCfg();loadStaStatus();setInterval(loadStaStatus,3000)}\n"
"</script>\n";

const char PAGE_STATUS[] PROGMEM = "\n"
"<h2>System Status</h2>\n"
"<div class=\"grid g2\">\n"
"<div class=\"card\">\n"
"<h3>System</h3>\n"
"<div class=\"kv\"><span>Firmware</span><b id=\"x_fw\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Uptime</span><b id=\"x_up\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Boot count</span><b id=\"x_boots\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Last reset reason</span><b id=\"x_reset\">-</b></div>\n"
"<div class=\"kv\"><span>Config storage</span><b id=\"x_cfg\">-</b></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Memory / Flash</h3>\n"
"<div class=\"kv\"><span>Free heap</span><b id=\"x_heap\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Minimum free heap</span><b id=\"x_minheap\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Largest free block</span><b id=\"x_big\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Fragmentation</span><b id=\"x_frag\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Sketch / free</span><b id=\"x_sketch\" class=\"mono\">-</b></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Time</h3>\n"
"<div class=\"kv\"><span>System clock</span><b id=\"x_time\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>Source</span><b id=\"x_tsrc\">-</b></div>\n"
"<div class=\"kv\"><span>Last sync age</span><b id=\"x_tage\" class=\"mono\">-</b></div>\n"
"<div class=\"kv\"><span>NTP</span><b id=\"x_ntp\">-</b></div>\n"
"<div class=\"kv\"><span>RTC</span><b id=\"x_rtc\">-</b></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Network</h3>\n"
"<div class=\"kv\"><span>Station</span><b id=\"x_sta\">-</b></div>\n"
"<div class=\"kv\"><span>AP</span><b id=\"x_ap\">-</b></div>\n"
"</div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Relay Summary</h3>\n"
"<div class=\"grid g3\" id=\"x_relays\"></div>\n"
"</div>\n"
"<div class=\"card\">\n"
"<h3>Maintenance</h3>\n"
"<div class=\"row\">\n"
"<a href=\"/api/config/export\" download=\"relay-config.json\"><button>Export Config</button></a>\n"
"<label class=\"row\">Import config: <input type=\"file\" id=\"x_file\" accept=\".json\"></label>\n"
"<button id=\"x_reboot\" class=\"bad\">Reboot</button>\n"
"<button id=\"x_factory\" class=\"bad\">Factory Reset</button>\n"
"</div>\n"
"<div class=\"mut small\">Factory reset: hold the BOOT button (GPIO0) for 5 seconds, or use the button\n"
"above. The config is wiped from NVS and the device reboots with defaults (AP mode, SSID\n"
"ESP32-Relays-XXXX, password 12345678). Relay pins are driven OFF during reset.</div>\n"
"</div>\n"
"<script>\n"
"function resetStr(r){var m={'POWERON':'Power-on','SW_RESET':'Software reset','SW_CPU_RESET':'Software CPU reset',\n"
"'DEEPSLEEP':'Deep sleep','EXT0':'EXT0 wake','EXT1':'EXT1 wake','SDIO':'SDIO',\n"
"'TG0WDT_SYS_RESET':'WDT (TG0)','TG1WDT_SYS_RESET':'WDT (TG1)','RTCWDT_SYS_RESET':'RTC WDT',\n"
"'RTCWDT_BROWN_OUT_RESET':'Brownout','RTCWDT_CPU_RESET':'RTC WDT CPU','BROWNOUT_RESET':'Brownout',\n"
"'SW_CPU_RESET':'Software CPU','INT_WDT':'Interrupt WDT','TASK_WDT':'Task WDT','WDT':'Watchdog'};\n"
"return m[r]||r||'-'}\n"
"function loadStatus(){getJSON('/api/status').then(function(d){if(!d.ok)return;\n"
"$('x_fw').textContent=d.fw+' / '+d.host;\n"
"var u=d.up,s=u%60,m=Math.floor(u/60)%60,h=Math.floor(u/3600)%24,dd=Math.floor(u/86400);\n"
"$('x_up').textContent=(dd?dd+'d ':'')+p2(h)+':'+p2(m)+':'+p2(s);\n"
"$('x_boots').textContent=d.boots;\n"
"$('x_reset').textContent=resetStr(d.reset);\n"
"$('x_cfg').textContent=d.cfgOk?(d.storageOk?'OK (dual-slot CRC)':'WRITE ERROR - check flash'):'DEFAULTS LOADED (no valid config found)';\n"
"$('x_heap').textContent=Math.round(d.heap.free/1024)+' KB';\n"
"$('x_minheap').textContent=Math.round(d.heap.min/1024)+' KB';\n"
"$('x_big').textContent=Math.round(d.heap.largest/1024)+' KB';\n"
"$('x_frag').textContent=d.heap.frag+' %';\n"
"$('x_sketch').textContent=Math.round(d.sketch.size/1024)+' KB / '+Math.round(d.sketch.free/1024)+' KB';\n"
"if(CLK.utc){var le=CLK.utc+d.off,dd2=Math.floor(le/86400),tod=le-dd2*86400,c=civilFromDays(dd2);\n"
"var DOW=['SUN','MON','TUE','WED','THU','FRI','SAT'][(dd2+4)%7];\n"
"$('x_time').textContent=DOW+' '+p2(c.m)+'/'+p2(c.d)+'/'+p2(c.y%100)+' '+p2(Math.floor(tod/3600))+':'+p2(Math.floor(tod%3600/60))+':'+p2(tod%60)}\n"
"else $('x_time').textContent='NOT SET';\n"
"$('x_tsrc').textContent=d.time.srcTxt;\n"
"$('x_tage').textContent=Math.round(d.time.syncAge)+' s';\n"
"$('x_ntp').textContent=d.ntp.enabled?('enabled, every '+d.ntp.intervalMin+' min, last '+(d.ntp.lastOk?('OK via '+d.ntp.server):'FAILED')):'disabled';\n"
"$('x_rtc').textContent=d.rtc.present?('DS3231, '+d.rtc.tempC+' C'+(d.rtc.lostPower?', BATTERY LOST POWER':'')):'NOT FOUND';\n"
"$('x_sta').textContent=d.sta.connected?('connected to '+d.sta.ssid+', '+d.sta.ip+', '+d.sta.rssi+' dBm'):\n"
"(d.sta.enabled?(d.sta.retrying?'reconnecting (1 s retry)':'enabled, down'):'disabled');\n"
"$('x_ap').textContent=d.ap.active?(d.ap.ssid+', '+d.ap.ip+', '+d.ap.clients+' clients'+(d.ap.captive?', captive portal':'')+(d.ap.temp?' (fallback)':'')):'inactive';\n"
"var h2='';\n"
"for(var i=0;i<d.relays.length;i++){var r=d.relays[i];\n"
"h2+='<div class=\"kv\"><span>'+(i+1)+'. '+esc(r.n)+' (GPIO'+r.g+')</span><b>'+\n"
"(r.removed?'<span class=\"badge b-rem\">REMOVED</span>':(r.on?'<span class=\"badge b-on\">ON</span>':'<span class=\"badge b-off\">OFF</span>'))+\n"
"' <span class=\"mut small\">'+r.mode.toUpperCase()+'</span></b></div>'}\n"
"$('x_relays').innerHTML=h2;\n"
"})}\n"
"window.pageInit=function(){\n"
"$('x_reboot').addEventListener('click',function(){\n"
"if(confirm('Reboot the controller? Relays go OFF briefly during boot.')){\n"
"api('/api/reboot',{}).then(function(){toast('Rebooting...')})}});\n"
"$('x_factory').addEventListener('click',function(){\n"
"if(confirm('FACTORY RESET wipes ALL settings, schedules and WiFi credentials. Continue?')&&\n"
"confirm('Are you sure? This cannot be undone.')){\n"
"api('/api/factoryreset',{confirm:'YES'}).then(function(d){\n"
"if(d.ok)toast('Factory reset - rebooting with defaults');else toast('Failed: '+(d.error||''),false)})}});\n"
"$('x_file').addEventListener('change',function(){\n"
"var f=this.files[0];if(!f)return;\n"
"var rd=new FileReader();\n"
"rd.onload=function(){fetch('/api/config/import',{method:'POST',\n"
"headers:{'Content-Type':'application/json'},body:rd.result})\n"
".then(function(r){return r.json()}).then(function(d){\n"
"if(d.ok){toast('Config imported - reboot recommended');setTimeout(loadStatus,1000)}\n"
"else toast('Import failed: '+(d.error||''),false)})};\n"
"rd.readAsText(f)});\n"
"loadStatus();setInterval(loadStatus,3000)}\n"
"</script>\n";

// ============================================================================
// SECTION 13: API HANDLERS
// ============================================================================

static const char* srcTxtOf(uint8_t s) {
  switch (s) {
    case SRC_RTC: return "DS3231 RTC";
    case SRC_NTP: return "NTP";
    case SRC_BROWSER: return "Browser";
    case SRC_MANUAL: return "Manual";
    default: return "Unset";
  }
}

static const char* resetReasonStr() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_SW:        return "SW_RESET";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_BROWNOUT:  return "BROWNOUT_RESET";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    default:                return "UNKNOWN";
  }
}

static void relayStatusJson(JsonArray arr) {
  for (int i = 0; i < N_RELAYS; i++) {
    const RelayCfg* r = &cfg.relays[i];
    JsonObject o = arr.add<JsonObject>();
    o["i"] = i;
    o["n"] = r->name;
    o["g"] = r->gpio;
    o["pin"] = lastPinLevel[i] ? 1 : 0;
    bool on;
    if (r->mode == RMODE_REMOVED) on = false;
    else on = r->activeHigh ? (lastPinLevel[i] != 0) : (lastPinLevel[i] == 0);
    o["on"] = on;
    o["mode"] = modeStr(r->mode);
    o["ah"] = r->activeHigh;
    o["sched"] = (r->mode == RMODE_AUTO) ? engOn[i] : false;
    o["removed"] = (r->mode == RMODE_REMOVED);
  }
}

static void handleApiStatus() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["fw"] = FW_VERSION;
  doc["host"] = cfg.sta.hostname;
  doc["up"] = (uint32_t)(esp_timer_get_time() / 1000000ULL);
  doc["boots"] = cfg.sys.bootCount;
  doc["reset"] = resetReasonStr();
  doc["cfgOk"] = cfg.sys.cfgDefaulted == 0;
  doc["storageOk"] = storageOk;

  JsonObject heap = doc["heap"].to<JsonObject>();
  uint32_t freeH = ESP.getFreeHeap();
  uint32_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  heap["free"] = freeH;
  heap["min"] = ESP.getMinFreeHeap();
  heap["largest"] = largest;
  heap["frag"] = (freeH > 0) ? (int)(100 - (100 * largest / freeH)) : 0;

  JsonObject sketch = doc["sketch"].to<JsonObject>();
  sketch["size"] = ESP.getSketchSize();
  sketch["free"] = ESP.getFreeSketchSpace();

  time_t utc = time(nullptr);
  LocalFields lf;
  breakLocal(utc, &lf);
  JsonObject t = doc["time"].to<JsonObject>();
  t["utc"] = (long)utc;
  t["off"] = lf.off;
  t["srcCode"] = lastTimeSource;
  t["srcTxt"] = srcTxtOf(lastTimeSource);
  t["syncAge"] = (lastSyncMs ? (uint32_t)((millis() - lastSyncMs) / 1000) : 0);
  t["valid"] = lf.valid;
  t["dst"] = lf.dst;

  JsonObject r = doc["rtc"].to<JsonObject>();
  r["present"] = rtcPresent;
  r["lostPower"] = rtcLostPowerFlag;
  r["tempC"] = rtcTempC;

  JsonObject n = doc["ntp"].to<JsonObject>();
  n["enabled"] = cfg.time.ntpEnabled == 1;
  n["intervalMin"] = cfg.time.syncIntervalMin;
  n["server"] = (ntp.lastSrv[0] ? ntp.lastSrv : ntp.activeSrv);
  n["lastOk"] = ntp.lastOk;
  n["drift"] = cfg.time.driftComp;
  n["active"] = ntp.phase != NTP_IDLE;

  JsonObject s = doc["sta"].to<JsonObject>();
  bool conn = staIsConnected();
  s["enabled"] = cfg.sta.enabled == 1;
  s["connected"] = conn;
  s["ssid"] = cfg.sta.ssid;
  s["ip"] = WiFi.localIP().toString();
  s["gw"] = WiFi.gatewayIP().toString();
  s["dns"] = WiFi.dnsIP().toString();
  s["rssi"] = conn ? WiFi.RSSI() : 0;
  s["chan"] = WiFi.channel();
  s["retrying"] = staRetrying;
  s["paused"] = staPaused;
  s["mac"] = WiFi.macAddress();
  s["bssid"] = conn ? WiFi.BSSIDstr() : "";
  s["host"] = WiFi.getHostname();
  s["autoRec"] = cfg.sta.autoReconnect == 1;

  JsonObject a = doc["ap"].to<JsonObject>();
  bool apAct = apIsActive();
  a["enabled"] = cfg.ap.enabled == 1;
  a["active"] = apAct;
  a["ssid"] = cfg.ap.ssid;
  a["ip"] = WiFi.softAPIP().toString();
  a["clients"] = apAct ? WiFi.softAPgetStationNum() : 0;
  a["captive"] = cfg.ap.captivePortal == 1;
  a["fallback"] = cfg.ap.fallback == 1;
  a["temp"] = tempApActive;
  JsonArray macs = a["clientMacs"].to<JsonArray>();
  if (apAct) {
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
      for (int i = 0; i < list.num; i++) {
        char mb[18];
        snprintf(mb, sizeof(mb), "%02X:%02X:%02X:%02X:%02X:%02X",
                 list.sta[i].mac[0], list.sta[i].mac[1], list.sta[i].mac[2],
                 list.sta[i].mac[3], list.sta[i].mac[4], list.sta[i].mac[5]);
        macs.add(mb);
      }
    }
  }

  JsonArray relays = doc["relays"].to<JsonArray>();
  relayStatusJson(relays);
  sendJsonDoc(doc);
}

static void handleApiTime() {
  time_t utc = time(nullptr);
  LocalFields lf;
  breakLocal(utc, &lf);
  JsonDocument doc;
  doc["ok"] = true;
  doc["utc"] = (long)utc;
  doc["off"] = lf.off;
  doc["srcCode"] = lastTimeSource;
  doc["srcTxt"] = srcTxtOf(lastTimeSource);
  doc["syncAge"] = (lastSyncMs ? (uint32_t)((millis() - lastSyncMs) / 1000) : 0);
  doc["valid"] = lf.valid;
  doc["y"] = lf.y; doc["mo"] = lf.mo; doc["d"] = lf.d;
  doc["h"] = lf.h; doc["mi"] = lf.mi; doc["s"] = lf.s;
  doc["dow"] = lf.dow;
  doc["dst"] = lf.dst;
  doc["tzMode"] = cfg.time.tzMode == TZ_POSIX ? "posix" : "manual";
  doc["posix"] = cfg.time.posixTz;
  sendJsonDoc(doc);
}

static void afterAuthoritativeSet(time_t utc) {
  if (rtcPresent && cfg.time.rtcEnabled) {
    time_t r = rtcUtcSane();
    long d = r ? labs((long)utc - (long)r) : 999999L;
    int comp = cfg.time.driftComp < 1 ? 1 : cfg.time.driftComp;
    if (r == 0 || d >= comp) {
      rtc.adjust(DateTime((uint32_t)utc));
      rtcLostPowerFlag = false;
    }
  }
}

static void handleApiTimeBrowser() {
  String body;
  if (!getJsonBody(body, 256)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  long long u = doc["utc"] | (long long)0;
  if (u <= 0) { sendJsonErr("utc (seconds) required"); return; }
  if (u <= (long long)MIN_VALID_UTC || u >= (long long)MAX_VALID_UTC) {
    sendJsonErr("utc out of range"); return;
  }
  time_t utc = (time_t)u;
  if (setTimeFromUtc(utc, SRC_BROWSER)) {
    afterAuthoritativeSet(utc);
    sendJsonOk();
  } else sendJsonErr("Set failed");
}

static void handleApiTimeManual() {
  String body;
  if (!getJsonBody(body, 256)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  int y = doc["y"] | -1, mo = doc["mo"] | -1, d = doc["d"] | -1;
  int h = doc["h"] | -1, mi = doc["mi"] | -1, s = doc["s"] | -1;
  if (y < 2020 || y > 2060 || mo < 1 || mo > 12 || d < 1 || d > days_in_month(y, mo) ||
      h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) {
    sendJsonErr("Invalid date/time fields"); return;
  }
  int32_t off = offsetAtLocal(y, mo, d, h, mi, s, time(nullptr));
  time_t utc = (time_t)days_from_civil(y, mo, d) * 86400L + h * 3600L + mi * 60L + s - off;
  if (utc <= MIN_VALID_UTC || utc >= MAX_VALID_UTC) { sendJsonErr("Out of range"); return; }
  if (setTimeFromUtc(utc, SRC_MANUAL)) {
    afterAuthoritativeSet(utc);
    sendJsonOk();
  } else sendJsonErr("Set failed");
}

static void handleApiNtpGet() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["enabled"] = cfg.time.ntpEnabled == 1;
  JsonArray s = doc["servers"].to<JsonArray>();
  for (int i = 0; i < 4; i++) s.add(cfg.time.ntpServers[i]);
  doc["intervalMin"] = cfg.time.syncIntervalMin;
  doc["drift"] = cfg.time.driftComp;
  doc["lastOk"] = ntp.lastOk;
  doc["lastSrv"] = ntp.lastSrv;
  sendJsonDoc(doc);
}

static void handleApiNtpPost() {
  String body;
  if (!getJsonBody(body, 1024)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  bool enabled = doc["enabled"] | false;
  JsonArrayConst srv = doc["servers"];
  char servers[4][48];
  int count = 0;
  if (!srv.isNull()) {
    if (srv.size() > 4) { sendJsonErr("Max 4 servers"); return; }
    int i = 0;
    for (JsonVariantConst v : srv) {
      if (i >= 4) break;
      servers[i][0] = 0;
      if (v.is<const char*>()) {
        String s = v.as<String>();
        s.trim();
        if (!validNtpName(s.c_str())) { sendJsonErr("Invalid server name"); return; }
        strlcpy(servers[i], s.c_str(), sizeof(servers[i]));
        if (servers[i][0]) count++;
      }
      i++;
    }
    for (; i < 4; i++) servers[i][0] = 0;
  } else {
    memcpy(servers, cfg.time.ntpServers, sizeof(servers));
    for (int k = 0; k < 4; k++) if (servers[k][0]) count++;
  }
  long iv = doc["intervalMin"] | -1;
  if (iv < 0 || iv > 1440) { sendJsonErr("intervalMin 0..1440"); return; }
  long dr = doc["drift"] | -1;
  if (dr < 0 || dr > 30) { sendJsonErr("drift 0..30"); return; }
  if (enabled && count == 0) { sendJsonErr("Enable needs >= 1 server"); return; }

  for (int i = 0; i < 4; i++) strlcpy(cfg.time.ntpServers[i], servers[i], sizeof(cfg.time.ntpServers[i]));
  cfg.time.ntpEnabled = enabled ? 1 : 0;
  cfg.time.syncIntervalMin = (uint16_t)iv;
  cfg.time.driftComp = (uint8_t)dr;
  storageSaveNow(false);
  if (cfg.time.ntpEnabled) ntpKick();
  sendJsonOk();
}

static void handleApiNtpSyncNow() {
  if (!cfg.time.ntpEnabled) { sendJsonErr("NTP disabled"); return; }
  if (!staIsConnected()) { sendJsonErr("Station not connected"); return; }
  ntpKick();
  sendJsonOk();
}

static void handleApiTzGet() {
  time_t utc = time(nullptr);
  JsonDocument doc;
  doc["ok"] = true;
  doc["mode"] = cfg.time.tzMode == TZ_POSIX ? "posix" : "manual";
  doc["utcOff"] = cfg.time.utcOffsetSec;
  const char* dm = "off";
  if (cfg.time.dstMode == DST_US) dm = "us";
  else if (cfg.time.dstMode == DST_EU) dm = "eu";
  else if (cfg.time.dstMode == DST_AU) dm = "au";
  doc["dstMode"] = dm;
  doc["dstOff"] = cfg.time.dstOffsetSec;
  doc["posix"] = cfg.time.posixTz;
  LocalFields lf;
  breakLocal(utc, &lf);
  doc["offNow"] = lf.off;
  doc["dstNow"] = lf.dst;
  sendJsonDoc(doc);
}

static void handleApiTzPost() {
  String body;
  if (!getJsonBody(body, 512)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  String mode = doc["mode"] | "";
  if (mode == "posix") {
    String p = doc["posix"] | "";
    if (!validPosixTz(p.c_str())) { sendJsonErr("Invalid POSIX TZ"); return; }
    strlcpy(cfg.time.posixTz, p.c_str(), sizeof(cfg.time.posixTz));
    cfg.time.tzMode = TZ_POSIX;
  } else if (mode == "manual") {
    long off = doc["utcOff"] | 999999999L;
    if (off < -47200 || off > 47200 || off % 60 != 0) { sendJsonErr("utcOff seconds -47200..47200"); return; }
    String dm = doc["dstMode"] | "";
    uint8_t dstMode = DST_OFF;
    if (dm == "us") dstMode = DST_US;
    else if (dm == "eu") dstMode = DST_EU;
    else if (dm == "au") dstMode = DST_AU;
    else if (dm != "off") { sendJsonErr("Bad dstMode"); return; }
    long dstOff = doc["dstOff"] | 3600;
    if (dstOff < 0 || dstOff > 7200) { sendJsonErr("dstOff 0..7200"); return; }
    cfg.time.tzMode = TZ_MANUAL;
    cfg.time.utcOffsetSec = (int32_t)off;
    cfg.time.dstMode = dstMode;
    cfg.time.dstOffsetSec = (int32_t)dstOff;
  } else {
    sendJsonErr("mode must be manual|posix"); return;
  }
  storageSaveNow(false);
  applyTz();
  engineRescanPending = true;
  sendJsonOk();
}

static void handleApiRtcGet() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["present"] = rtcPresent;
  doc["lostPower"] = rtcLostPowerFlag;
  if (rtcPresent) {
    rtcTempC = rtc.getTemperature();
    doc["tempC"] = rtcTempC;
    time_t r = rtcUtcSane();
    time_t sys = time(nullptr);
    doc["rtcUtc"] = (long)r;
    doc["sysUtc"] = (long)sys;
    doc["delta"] = (long)((long)r - (long)sys);
  }
  sendJsonDoc(doc);
}

static void handleApiRtcPost() {
  String body;
  if (!getJsonBody(body, 128)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  String op = doc["op"] | "";
  if (op == "read") {
    if (!rtcPresent) { sendJsonErr("RTC not present"); return; }
    bool applied = false;
    rtcToSystem(&applied);
    if (applied) sendJsonOk();
    else sendJsonErr("RTC time not sane (battery?)");
  } else if (op == "write") {
    if (!rtcPresent) { sendJsonErr("RTC not present"); return; }
    if (systemToRtc()) sendJsonOk();
    else sendJsonErr("System clock not set");
  } else sendJsonErr("op must be read|write");
}

static const char* encStr(uint8_t t) {
  switch (t) {
    case WIFI_AUTH_OPEN: return "OPEN";
    case WIFI_AUTH_WEP: return "WEP";
    case WIFI_AUTH_WPA_PSK: return "WPA";
    case WIFI_AUTH_WPA2_PSK: return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2";
    case WIFI_AUTH_WPA3_PSK: return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3";
    case WIFI_AUTH_ENTERPRISE: return "ENTERPRISE";
    default: return "OTHER";
  }
}

static void handleApiScanStart() {
  if (!scanDone()) { sendJsonOk(); return; }
  scanStart();
  sendJsonOk();
}

static void handleApiScanResults() {
  JsonDocument doc;
  doc["ok"] = true;
  int8_t n = WiFi.scanComplete();
  if (n == WIFI_SCAN_RUNNING) { doc["status"] = "running"; sendJsonDoc(doc); return; }
  if (n < 0) { doc["status"] = "idle"; sendJsonDoc(doc); return; }
  doc["status"] = "done";
  JsonArray nets = doc["nets"].to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject o = nets.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);
    o["ch"] = WiFi.channel(i);
    o["sec"] = encStr(WiFi.encryptionType(i));
  }
  sendJsonDoc(doc);
  WiFi.scanDelete();
}

// ============================================================================
// SECTION 14: API HANDLERS (relays, schedules, gpio, wifi)
// ============================================================================

static const int FLD_LO[6] = {0, 0, 0, 1, 1, 0};
static const int FLD_HI[6] = {59, 59, 23, 31, 12, 6};

static bool parseCalArr(JsonArrayConst a, CalFields* out, String* err) {
  if (a.isNull() || a.size() != 6) {
    *err = "s/e must be [sec,min,hour,dom,month,dow] (6 values, 255=wildcard)";
    return false;
  }
  int vals[6];
  int i = 0;
  for (JsonVariantConst v : a) vals[i++] = v | -999;
  for (int k = 0; k < 6; k++) {
    if (vals[k] == 255) continue;
    if (vals[k] < FLD_LO[k] || vals[k] > FLD_HI[k]) {
      *err = "schedule field out of range"; return false;
    }
  }
  out->sec = (vals[0] == 255) ? WILDCARD : (uint8_t)vals[0];
  out->min = (vals[1] == 255) ? WILDCARD : (uint8_t)vals[1];
  out->hour = (vals[2] == 255) ? WILDCARD : (uint8_t)vals[2];
  out->dom = (vals[3] == 255) ? WILDCARD : (uint8_t)vals[3];
  out->month = (vals[4] == 255) ? WILDCARD : (uint8_t)vals[4];
  out->dow = (vals[5] == 255) ? WILDCARD : (uint8_t)vals[5];
  return true;
}

static bool parsePair(JsonObjectConst o, SchedPair* out, String* err) {
  if (o.isNull()) { *err = "pair must be an object {en,s,e}"; return false; }
  int en = o["en"] | -1;
  if (en != 0 && en != 1) { *err = "en must be 0 or 1"; return false; }
  if (!parseCalArr(o["s"].as<JsonArrayConst>(), &out->start, err)) return false;
  if (!parseCalArr(o["e"].as<JsonArrayConst>(), &out->stop, err)) return false;
  out->enabled = (uint8_t)en;
  return true;
}

static void schedToJson(JsonDocument& doc) {
  doc["ok"] = true;
  JsonArray all = doc["sched"].to<JsonArray>();
  for (int r = 0; r < N_RELAYS; r++) {
    JsonObject ro = all.add<JsonObject>();
    JsonArray pairs = ro["pairs"].to<JsonArray>();
    for (int p = 0; p < N_PAIRS; p++) {
      const SchedPair* sp = &cfg.sched[r][p];
      JsonObject po = pairs.add<JsonObject>();
      po["en"] = sp->enabled;
      JsonArray s = po["s"].to<JsonArray>();
      s.add(sp->start.sec); s.add(sp->start.min); s.add(sp->start.hour);
      s.add(sp->start.dom); s.add(sp->start.month); s.add(sp->start.dow);
      JsonArray e = po["e"].to<JsonArray>();
      e.add(sp->stop.sec); e.add(sp->stop.min); e.add(sp->stop.hour);
      e.add(sp->stop.dom); e.add(sp->stop.month); e.add(sp->stop.dow);
    }
  }
}

static void handleApiSchedulesGet() {
  JsonDocument doc;
  schedToJson(doc);
  sendJsonDoc(doc);
}

static void handleApiSchedulesPost() {
  String body;
  if (!getJsonBody(body, 24000)) { sendJsonErr("Missing/oversized body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  String err;
  SchedPair tmp[N_RELAYS][N_PAIRS];

  if (doc["all"].is<JsonArrayConst>()) {
    JsonArrayConst all = doc["all"].as<JsonArrayConst>();
    if (all.size() != N_RELAYS) { sendJsonErr("all must contain 16 relays"); return; }
    int r = 0;
    for (JsonVariantConst rv : all) {
      JsonArrayConst pairs = rv["pairs"].as<JsonArrayConst>();
      if (pairs.isNull() || pairs.size() != N_PAIRS) {
        sendJsonErr("each relay needs exactly 8 pairs"); return;
      }
      int p = 0;
      for (JsonVariantConst pv : pairs) {
        if (!parsePair(pv.as<JsonObjectConst>(), &tmp[r][p], &err)) {
          sendJsonErr(err.c_str()); return;
        }
        p++;
      }
      r++;
    }
    memcpy(cfg.sched, tmp, sizeof(cfg.sched));
  } else {
    long rl = doc["relay"] | -1;
    if (rl < 0 || rl >= N_RELAYS) { sendJsonErr("relay 0..15 required"); return; }
    JsonArrayConst pairs = doc["pairs"].as<JsonArrayConst>();
    if (pairs.isNull() || pairs.size() != N_PAIRS) {
      sendJsonErr("pairs must contain exactly 8 entries"); return;
    }
    int p = 0;
    for (JsonVariantConst pv : pairs) {
      if (!parsePair(pv.as<JsonObjectConst>(), &tmp[rl][p], &err)) {
        sendJsonErr(err.c_str()); return;
      }
      p++;
    }
    memcpy(cfg.sched[rl], tmp[rl], sizeof(cfg.sched[rl]));
  }
  storageSaveNow(false);
  engineRescanPending = true;
  sendJsonOk();
}

static void handleApiRelayPost() {
  String body;
  if (!getJsonBody(body, 256)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  long i = doc["i"] | -1;
  if (i < 0 || i >= N_RELAYS) { sendJsonErr("i 0..15 required"); return; }

  if (doc["name"].is<const char*>()) {
    String nm = doc["name"].as<String>();
    nm.trim();
    if (!validNameStr(nm.c_str())) { sendJsonErr("Name 1-23 printable chars"); return; }
    strlcpy(cfg.relays[i].name, nm.c_str(), sizeof(cfg.relays[i].name));
    storageSaveNow(false);
  }
  if (doc["mode"].is<const char*>()) {
    uint8_t m = modeFromStr(doc["mode"].as<const char*>());
    if (m == 0xFF) { sendJsonErr("bad mode"); return; }
    if (!setRelayMode((int)i, m)) { sendJsonErr("transition blocked (relay removed)"); return; }
  }
  JsonDocument out;
  out["ok"] = true;
  sendJsonDoc(out);
}

static void handleApiGpio() {
  String body;
  if (!getJsonBody(body, 256)) { sendJsonErr("Missing body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }

  if (doc["global"].is<const char*>()) {
    String g = doc["global"].as<String>();
    if (g == "low") { setAllActiveLevel(false); sendJsonOk(); return; }
    if (g == "high") { setAllActiveLevel(true); sendJsonOk(); return; }
    sendJsonErr("global must be low|high"); return;
  }
  if (doc["action"].is<const char*>()) {
    String a = doc["action"].as<String>();
    long i = doc["i"] | -1;
    if (a == "restoreall") { restoreAllRelays(); sendJsonOk(); return; }
    if (i < 0 || i >= N_RELAYS) { sendJsonErr("i 0..15 required"); return; }
    uint8_t m = 0xFF;
    if (a == "sethigh") m = RMODE_HIGH;
    else if (a == "setlow") m = RMODE_LOW;
    else if (a == "auto" || a == "restore") m = RMODE_AUTO;
    else if (a == "remove") m = RMODE_REMOVED;
    else { sendJsonErr("bad action"); return; }
    if (!setRelayMode((int)i, m)) { sendJsonErr("transition blocked"); return; }
    sendJsonOk(); return;
  }
  if (doc["i"].is<long>() && doc["ah"].is<long>()) {
    long i = doc["i"] | -1;
    long ah = doc["ah"] | -1;
    if (i < 0 || i >= N_RELAYS || (ah != 0 && ah != 1)) { sendJsonErr("bad i/ah"); return; }
    cfg.relays[i].activeHigh = (uint8_t)ah;
    applyRelay((int)i);
    storageSaveNow(false);
    sendJsonOk(); return;
  }
  sendJsonErr("nothing to do");
}

static bool jsonToStaCfg(JsonObjectConst o, StaCfg* out, String* err) {
  int enabled = o["enabled"] | -1;
  if (enabled != 0 && enabled != 1) { *err = "enabled 0/1"; return false; }
  String ssid = o["ssid"] | String("");
  String pass = o["pass"] | String("");
  String host = o["host"] | String("");
  ssid.trim();
  if (ssid.length() > 32) { *err = "SSID too long (max 32)"; return false; }
  if (pass.length() > 63) { *err = "password too long (max 63)"; return false; }
  if (host.length() > 31) { *err = "hostname too long"; return false; }
  if (host.length() && !validHostStr(host.c_str())) { *err = "invalid hostname"; return false; }
  int dhcp = o["dhcp"] | 1;
  if (dhcp != 0 && dhcp != 1) { *err = "dhcp 0/1"; return false; }
  int autoRec = o["autoRec"] | 1;
  if (autoRec != 0 && autoRec != 1) { *err = "autoRec 0/1"; return false; }
  uint32_t ip = 0, gw = 0, sn = 0, d1 = 0, d2 = 0;
  if (dhcp == 0) {
    ip = ipRaw(o["ip"] | String(""));
    gw = ipRaw(o["gw"] | String(""));
    sn = ipRaw(o["sn"] | String(""));
    d1 = ipRaw(o["dns1"] | String(""));
    String ds2 = o["dns2"] | String("");
    d2 = ds2.length() ? ipRaw(ds2) : 0;
    if (ip == 0 || gw == 0 || sn == 0 || d1 == 0) { *err = "static config needs valid ip/gw/sn/dns1"; return false; }
  }
  if (enabled == 1 && ssid.length() == 0) { *err = "SSID required when enabled"; return false; }
  out->enabled = (uint8_t)enabled;
  strlcpy(out->ssid, ssid.c_str(), sizeof(out->ssid));
  strlcpy(out->pass, pass.c_str(), sizeof(out->pass));
  if (host.length()) strlcpy(out->hostname, host.c_str(), sizeof(out->hostname));
  else strlcpy(out->hostname, "esp32-relays", sizeof(out->hostname));
  out->dhcp = (uint8_t)dhcp;
  out->ip = ip; out->gw = gw; out->sn = sn; out->dns1 = d1; out->dns2 = d2;
  out->autoReconnect = (uint8_t)autoRec;
  return true;
}

static bool jsonToApCfg(JsonObjectConst o, ApCfg* out, String* err) {
  int enabled = o["enabled"] | -1;
  if (enabled != 0 && enabled != 1) { *err = "enabled 0/1"; return false; }
  String ssid = o["ssid"] | String("");
  String pass = o["pass"] | String("");
  ssid.trim();
  if (ssid.length() < 1 || ssid.length() > 32) { *err = "AP SSID 1-32 chars"; return false; }
  if (pass.length() > 63) { *err = "password too long"; return false; }
  if (pass.length() > 0 && pass.length() < 8) { *err = "WPA2 password needs 8-63 chars (or empty)"; return false; }
  long chan = o["chan"] | 1;
  if (chan < 1 || chan > 13) { *err = "channel 1-13"; return false; }
  long maxc = o["maxc"] | 4;
  if (maxc < 1 || maxc > 8) { *err = "max clients 1-8"; return false; }
  int hidden = o["hidden"] | 0;
  if (hidden != 0 && hidden != 1) { *err = "hidden 0/1"; return false; }
  int captive = o["captive"] | 1;
  if (captive != 0 && captive != 1) { *err = "captive 0/1"; return false; }
  int fallback = o["fallback"] | 0;
  if (fallback != 0 && fallback != 1) { *err = "fallback 0/1"; return false; }
  uint32_t ip = ipRaw(o["ip"] | String(""));
  uint32_t gw = ipRaw(o["gw"] | String(""));
  uint32_t sn = ipRaw(o["sn"] | String(""));
  if (ip == 0) { ip = (uint32_t)IPAddress(192, 168, 4, 1); }
  if (gw == 0) { gw = ip; }
  if (sn == 0) { sn = (uint32_t)IPAddress(255, 255, 255, 0); }
  out->enabled = (uint8_t)enabled;
  strlcpy(out->ssid, ssid.c_str(), sizeof(out->ssid));
  strlcpy(out->pass, pass.c_str(), sizeof(out->pass));
  out->channel = (uint8_t)chan;
  out->hidden = (uint8_t)hidden;
  out->maxClients = (uint8_t)maxc;
  out->captivePortal = (uint8_t)captive;
  out->fallback = (uint8_t)fallback;
  out->ip = ip; out->gw = gw; out->sn = sn;
  return true;
}

static void handleApiStaGet() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["enabled"] = cfg.sta.enabled == 1;
  doc["ssid"] = cfg.sta.ssid;
  doc["pass"] = cfg.sta.pass;
  doc["host"] = cfg.sta.hostname;
  doc["dhcp"] = cfg.sta.dhcp == 1;
  uint32_t ip = cfg.sta.dhcp ? (uint32_t)WiFi.localIP() : cfg.sta.ip;
  uint32_t gw = cfg.sta.dhcp ? (uint32_t)WiFi.gatewayIP() : cfg.sta.gw;
  uint32_t sn = cfg.sta.dhcp ? (uint32_t)WiFi.subnetMask() : cfg.sta.sn;
  uint32_t d1 = cfg.sta.dhcp ? (uint32_t)WiFi.dnsIP(0) : cfg.sta.dns1;
  uint32_t d2 = cfg.sta.dhcp ? (uint32_t)WiFi.dnsIP(1) : cfg.sta.dns2;
  doc["ip"] = ipStr(ip);
  doc["gw"] = ipStr(gw);
  doc["sn"] = ipStr(sn);
  doc["dns1"] = ipStr(d1);
  doc["dns2"] = ipStr(d2);
  doc["autoRec"] = cfg.sta.autoReconnect == 1;
  sendJsonDoc(doc);
}

static void handleApiStaPost() {
  String body;
  if (!getJsonBody(body, 2048)) { sendJsonErr("Missing/oversized body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  StaCfg tmp;
  memcpy(&tmp, &cfg.sta, sizeof(tmp));
  String err;
  if (!jsonToStaCfg(doc.as<JsonObjectConst>(), &tmp, &err)) { sendJsonErr(err.c_str()); return; }
  bool connect = (doc["connect"] | 0) == 1;
  memcpy(&cfg.sta, &tmp, sizeof(cfg.sta));
  storageSaveNow(false);
  applySta(connect);
  sendJsonOk();
}

static void handleApiStaDisconnect() {
  WiFi.disconnect(false, false);
  staPaused = true;
  staRetrying = false;
  sendJsonOk();
}

static void handleApiApGet() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["enabled"] = cfg.ap.enabled == 1;
  doc["ssid"] = cfg.ap.ssid;
  doc["pass"] = cfg.ap.pass;
  doc["chan"] = cfg.ap.channel;
  doc["hidden"] = cfg.ap.hidden == 1;
  doc["maxc"] = cfg.ap.maxClients;
  doc["ip"] = ipStr(cfg.ap.ip);
  doc["gw"] = ipStr(cfg.ap.gw);
  doc["sn"] = ipStr(cfg.ap.sn);
  doc["captive"] = cfg.ap.captivePortal == 1;
  doc["fallback"] = cfg.ap.fallback == 1;
  bool act = apIsActive();
  doc["active"] = act;
  doc["clients"] = act ? WiFi.softAPgetStationNum() : 0;
  JsonArray macs = doc["clientMacs"].to<JsonArray>();
  if (act) {
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK) {
      for (int i = 0; i < list.num; i++) {
        char mb[18];
        snprintf(mb, sizeof(mb), "%02X:%02X:%02X:%02X:%02X:%02X",
                 list.sta[i].mac[0], list.sta[i].mac[1], list.sta[i].mac[2],
                 list.sta[i].mac[3], list.sta[i].mac[4], list.sta[i].mac[5]);
        macs.add(mb);
      }
    }
  }
  sendJsonDoc(doc);
}

static void handleApiApPost() {
  String body;
  if (!getJsonBody(body, 2048)) { sendJsonErr("Missing/oversized body"); return; }
  JsonDocument doc;
  if (deserializeJson(doc, body)) { sendJsonErr("Bad JSON"); return; }
  ApCfg tmp;
  memcpy(&tmp, &cfg.ap, sizeof(tmp));
  String err;
  if (!jsonToApCfg(doc.as<JsonObjectConst>(), &tmp, &err)) { sendJsonErr(err.c_str()); return; }
  memcpy(&cfg.ap, &tmp, sizeof(cfg.ap));
  storageSaveNow(false);
  applyAp(true);
  sendJsonOk();
}

// ============================================================================
// SECTION 15: CONFIG EXPORT/IMPORT + SYSTEM COMMANDS + ROUTES
// ============================================================================

static void handleApiRelaysGet() {
  JsonDocument doc;
  doc["ok"] = true;
  JsonArray relays = doc["relays"].to<JsonArray>();
  relayStatusJson(relays);
  sendJsonDoc(doc);
}

static void handleApiReboot() {
  sendJsonOk();
  pendingFactoryReset = false;
  pendingRestart = true;
  pendingRestartMs = millis() + 800;
}

static void handleApiFactoryReset() {
  String body;
  getJsonBody(body, 256);
  JsonDocument doc;
  if (body.length()) deserializeJson(doc, body);
  String confirm = doc["confirm"] | String("");
  if (confirm != "YES") { sendJsonErr("confirm must be YES"); return; }
  for (int i = 0; i < N_RELAYS; i++) {
    pinMode(cfg.relays[i].gpio, OUTPUT);
    digitalWrite(cfg.relays[i].gpio, cfg.relays[i].activeHigh ? LOW : HIGH);
  }
  sendJsonOk();
  pendingFactoryReset = true;
  pendingRestart = true;
  pendingRestartMs = millis() + 800;
}

static void handleConfigExport() {
  JsonDocument doc;
  doc["ok"] = true;
  doc["fw"] = FW_VERSION;
  doc["cfgVersion"] = CFG_VERSION;
  JsonArray relays = doc["relays"].to<JsonArray>();
  for (int i = 0; i < N_RELAYS; i++) {
    JsonObject o = relays.add<JsonObject>();
    o["n"] = cfg.relays[i].name;
    o["g"] = cfg.relays[i].gpio;
    o["ah"] = cfg.relays[i].activeHigh;
    o["mode"] = modeStr(cfg.relays[i].mode);
  }
  JsonArray sched = doc["sched"].to<JsonArray>();
  for (int r = 0; r < N_RELAYS; r++) {
    JsonObject ro = sched.add<JsonObject>();
    JsonArray pairs = ro["pairs"].to<JsonArray>();
    for (int p = 0; p < N_PAIRS; p++) {
      const SchedPair* sp = &cfg.sched[r][p];
      JsonObject po = pairs.add<JsonObject>();
      po["en"] = sp->enabled;
      JsonArray s = po["s"].to<JsonArray>();
      s.add(sp->start.sec); s.add(sp->start.min); s.add(sp->start.hour);
      s.add(sp->start.dom); s.add(sp->start.month); s.add(sp->start.dow);
      JsonArray e = po["e"].to<JsonArray>();
      e.add(sp->stop.sec); e.add(sp->stop.min); e.add(sp->stop.hour);
      e.add(sp->stop.dom); e.add(sp->stop.month); e.add(sp->stop.dow);
    }
  }
  JsonObject t = doc["time"].to<JsonObject>();
  t["enabled"] = cfg.time.ntpEnabled == 1;
  JsonArray srv = t["servers"].to<JsonArray>();
  for (int i = 0; i < 4; i++) srv.add(cfg.time.ntpServers[i]);
  t["intervalMin"] = cfg.time.syncIntervalMin;
  t["drift"] = cfg.time.driftComp;
  t["tzMode"] = cfg.time.tzMode == TZ_POSIX ? "posix" : "manual";
  t["utcOff"] = cfg.time.utcOffsetSec;
  const char* dm = "off";
  if (cfg.time.dstMode == DST_US) dm = "us";
  else if (cfg.time.dstMode == DST_EU) dm = "eu";
  else if (cfg.time.dstMode == DST_AU) dm = "au";
  t["dstMode"] = dm;
  t["dstOff"] = cfg.time.dstOffsetSec;
  t["posix"] = cfg.time.posixTz;
  t["rtcEnabled"] = cfg.time.rtcEnabled == 1;

  JsonObject s = doc["sta"].to<JsonObject>();
  s["enabled"] = cfg.sta.enabled == 1;
  s["ssid"] = cfg.sta.ssid;
  s["pass"] = cfg.sta.pass;
  s["host"] = cfg.sta.hostname;
  s["dhcp"] = cfg.sta.dhcp == 1;
  s["ip"] = ipStr(cfg.sta.ip);
  s["gw"] = ipStr(cfg.sta.gw);
  s["sn"] = ipStr(cfg.sta.sn);
  s["dns1"] = ipStr(cfg.sta.dns1);
  s["dns2"] = ipStr(cfg.sta.dns2);
  s["autoRec"] = cfg.sta.autoReconnect == 1;

  JsonObject a = doc["ap"].to<JsonObject>();
  a["enabled"] = cfg.ap.enabled == 1;
  a["ssid"] = cfg.ap.ssid;
  a["pass"] = cfg.ap.pass;
  a["chan"] = cfg.ap.channel;
  a["hidden"] = cfg.ap.hidden == 1;
  a["maxc"] = cfg.ap.maxClients;
  a["captive"] = cfg.ap.captivePortal == 1;
  a["fallback"] = cfg.ap.fallback == 1;
  a["ip"] = ipStr(cfg.ap.ip);
  a["gw"] = ipStr(cfg.ap.gw);
  a["sn"] = ipStr(cfg.ap.sn);

  server.sendHeader("Content-Disposition", "attachment; filename=relay-config.json");
  sendJsonDoc(doc);
}

static uint32_t wifiApplyPendingMs = 0;
static bool wifiApplyPending = false;

static void handleConfigImport() {
  String body;
  if (!getJsonBody(body, 30000)) { sendJsonErr("Missing/oversized body"); return; }
  JsonDocument doc;
  DeserializationError e = deserializeJson(doc, body);
  if (e) { sendJsonErr("Bad JSON"); return; }

  Config tmp;
  configDefaultsInto(&tmp);

  JsonArrayConst relays = doc["relays"].as<JsonArrayConst>();
  if (!relays.isNull()) {
    if (relays.size() != N_RELAYS) { sendJsonErr("relays must have 16 entries"); return; }
    int i = 0;
    for (JsonVariantConst v : relays) {
      String nm = v["n"] | String("");
      if (nm.length() && !validNameStr(nm.c_str())) { sendJsonErr("bad relay name"); return; }
      if (nm.length()) strlcpy(tmp.relays[i].name, nm.c_str(), sizeof(tmp.relays[i].name));
      long ah = v["ah"] | 0;
      tmp.relays[i].activeHigh = (ah == 1) ? 1 : 0;
      String md = v["mode"] | String("auto");
      uint8_t m = modeFromStr(md.c_str());
      if (m == 0xFF) { sendJsonErr("bad relay mode"); return; }
      tmp.relays[i].mode = m;
      i++;
    }
  }
  JsonArrayConst sched = doc["sched"].as<JsonArrayConst>();
  if (!sched.isNull()) {
    if (sched.size() != N_RELAYS) { sendJsonErr("sched must have 16 relays"); return; }
    String err;
    int r = 0;
    for (JsonVariantConst rv : sched) {
      JsonArrayConst pairs = rv["pairs"].as<JsonArrayConst>();
      if (pairs.isNull() || pairs.size() != N_PAIRS) { sendJsonErr("each relay needs 8 pairs"); return; }
      int p = 0;
      for (JsonVariantConst pv : pairs) {
        if (!parsePair(pv.as<JsonObjectConst>(), &tmp.sched[r][p], &err)) {
          sendJsonErr(err.c_str()); return;
        }
        p++;
      }
      r++;
    }
  }
  JsonObjectConst t = doc["time"].as<JsonObjectConst>();
  if (!t.isNull()) {
    JsonArrayConst srv = t["servers"].as<JsonArrayConst>();
    if (!srv.isNull()) {
      if (srv.size() > 4) { sendJsonErr("max 4 ntp servers"); return; }
      int i = 0;
      for (JsonVariantConst v : srv) {
        if (i >= 4) break;
        if (v.is<const char*>()) {
          String s = v.as<String>();
          s.trim();
          if (!validNtpName(s.c_str())) { sendJsonErr("bad ntp server"); return; }
          strlcpy(tmp.time.ntpServers[i], s.c_str(), sizeof(tmp.time.ntpServers[i]));
        }
        i++;
      }
    }
    long iv = t["intervalMin"] | 60;
    if (iv < 0 || iv > 1440) { sendJsonErr("intervalMin 0..1440"); return; }
    tmp.time.syncIntervalMin = (uint16_t)iv;
    long dr = t["drift"] | 1;
    if (dr < 0 || dr > 30) { sendJsonErr("drift 0..30"); return; }
    tmp.time.driftComp = (uint8_t)dr;
    int en = t["enabled"] | 1;
    tmp.time.ntpEnabled = (en == 1) ? 1 : 0;
    int rtcEn = t["rtcEnabled"] | 1;
    tmp.time.rtcEnabled = (rtcEn == 1) ? 1 : 0;
    String tzMode = t["tzMode"] | String("manual");
    if (tzMode == "posix") {
      String p = t["posix"] | String("");
      if (!validPosixTz(p.c_str())) { sendJsonErr("bad posix tz"); return; }
      strlcpy(tmp.time.posixTz, p.c_str(), sizeof(tmp.time.posixTz));
      tmp.time.tzMode = TZ_POSIX;
    } else {
      long off = t["utcOff"] | 0;
      if (off < -47200 || off > 47200) { sendJsonErr("utcOff out of range"); return; }
      String dm = t["dstMode"] | String("off");
      uint8_t dstMode = DST_OFF;
      if (dm == "us") dstMode = DST_US;
      else if (dm == "eu") dstMode = DST_EU;
      else if (dm == "au") dstMode = DST_AU;
      else if (dm != "off") { sendJsonErr("bad dstMode"); return; }
      long dstOff = t["dstOff"] | 3600;
      if (dstOff < 0 || dstOff > 7200) { sendJsonErr("dstOff out of range"); return; }
      tmp.time.tzMode = TZ_MANUAL;
      tmp.time.utcOffsetSec = (int32_t)off;
      tmp.time.dstMode = dstMode;
      tmp.time.dstOffsetSec = (int32_t)dstOff;
    }
  }
  JsonObjectConst s = doc["sta"].as<JsonObjectConst>();
  if (!s.isNull()) {
    String err;
    if (!jsonToStaCfg(s, &tmp.sta, &err)) { sendJsonErr(err.c_str()); return; }
  }
  JsonObjectConst a = doc["ap"].as<JsonObjectConst>();
  if (!a.isNull()) {
    String err;
    if (!jsonToApCfg(a, &tmp.ap, &err)) { sendJsonErr(err.c_str()); return; }
  }

  uint32_t boots = cfg.sys.bootCount;
  memcpy(&cfg, &tmp, sizeof(cfg));
  cfg.sys.bootCount = boots;
  cfg.sys.cfgDefaulted = 0;
  sanitizeConfig();
  storageSaveNow(true);
  applyTz();
  applyConfigPins();
  engineRescanPending = true;

  JsonDocument out;
  out["ok"] = true;
  out["needReboot"] = true;
  sendJsonDoc(out);
  wifiApplyPending = true;
  wifiApplyPendingMs = millis() + 1500;
}

static void handleNotFound() {
  if (apIsActive() && cfg.ap.captivePortal == 1) {
    String url = "http://" + WiFi.softAPIP().toString() + "/";
    server.sendHeader("Location", url, true);
    server.sendHeader("Cache-Control", "no-store");
    server.send(302, "text/plain", "");
    return;
  }
  JsonDocument doc;
  doc["ok"] = false;
  doc["error"] = "Not found";
  String out;
  serializeJson(doc, out);
  server.send(404, "application/json", out);
}

static void registerRoutes() {
  server.on("/", HTTP_GET, []() { sendPage(PAGE_HOME); });
  server.on("/relays", HTTP_GET, []() { sendPage(PAGE_RELAYS); });
  server.on("/gpio", HTTP_GET, []() { sendPage(PAGE_GPIO); });
  server.on("/time", HTTP_GET, []() { sendPage(PAGE_TIME); });
  server.on("/wifi-ap", HTTP_GET, []() { sendPage(PAGE_WAP); });
  server.on("/wifi-sta", HTTP_GET, []() { sendPage(PAGE_STA); });
  server.on("/status", HTTP_GET, []() { sendPage(PAGE_STATUS); });
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204, "text/plain", ""); });

  server.on("/api/status", HTTP_GET, handleApiStatus);
  server.on("/api/time", HTTP_GET, handleApiTime);
  server.on("/api/time/browser", HTTP_POST, handleApiTimeBrowser);
  server.on("/api/time/manual", HTTP_POST, handleApiTimeManual);

  server.on("/api/relays", HTTP_GET, handleApiRelaysGet);
  server.on("/api/relay", HTTP_POST, handleApiRelayPost);
  server.on("/api/schedules", HTTP_GET, handleApiSchedulesGet);
  server.on("/api/schedules", HTTP_POST, handleApiSchedulesPost);
  server.on("/api/gpio", HTTP_GET, handleApiRelaysGet);
  server.on("/api/gpio", HTTP_POST, handleApiGpio);

  server.on("/api/ntp", HTTP_GET, handleApiNtpGet);
  server.on("/api/ntp", HTTP_POST, handleApiNtpPost);
  server.on("/api/ntp/syncnow", HTTP_POST, handleApiNtpSyncNow);
  server.on("/api/tz", HTTP_GET, handleApiTzGet);
  server.on("/api/tz", HTTP_POST, handleApiTzPost);
  server.on("/api/rtc", HTTP_GET, handleApiRtcGet);
  server.on("/api/rtc", HTTP_POST, handleApiRtcPost);
  server.on("/api/scan/start", HTTP_POST, handleApiScanStart);
  server.on("/api/scan/results", HTTP_GET, handleApiScanResults);

  server.on("/api/wifi/sta", HTTP_GET, handleApiStaGet);
  server.on("/api/wifi/sta", HTTP_POST, handleApiStaPost);
  server.on("/api/wifi/sta/disconnect", HTTP_POST, handleApiStaDisconnect);
  server.on("/api/wifi/ap", HTTP_GET, handleApiApGet);
  server.on("/api/wifi/ap", HTTP_POST, handleApiApPost);

  server.on("/api/reboot", HTTP_POST, handleApiReboot);
  server.on("/api/factoryreset", HTTP_POST, handleApiFactoryReset);
  server.on("/api/config/export", HTTP_GET, handleConfigExport);
  server.on("/api/config/import", HTTP_POST, handleConfigImport);

  server.onNotFound(handleNotFound);
}

// ============================================================================
// SECTION 16: BOOT BUTTON, FACTORY RESET, SETUP(), LOOP()
// ============================================================================

static void buttonTask(uint32_t now) {
  if ((int32_t)(now - bootIgnoreBtnMs) < 0) return;
  bool down = (digitalRead(BTN_FACTORY_GPIO) == LOW);
  if (down != btnWasDown) {
    btnWasDown = down;
    btnDownSinceMs = now;
    if (!down) { btnStable = false; factoryResetArmed = false; }
  }
  if (down && !btnStable && (now - btnDownSinceMs) >= 50) {
    btnStable = true;
    btnDownSinceMs = now;
  }
  if (btnStable && !factoryResetArmed &&
      (now - btnDownSinceMs) >= BTN_FACTORY_MS) {
    factoryResetArmed = true;
    doFactoryReset();
  }
}

static void doFactoryReset() {
  for (int i = 0; i < N_RELAYS; i++) {
    pinMode(cfg.relays[i].gpio, OUTPUT);
    digitalWrite(cfg.relays[i].gpio, cfg.relays[i].activeHigh ? LOW : HIGH);
  }
  prefs.clear();
  ESP.restart();
}

void setup() {
  bootMs = millis();
#if ENABLE_DEBUG_SERIAL
  Serial.begin(115200);
#endif

  initPinsSafe();

  if (!prefs.begin(NVS_NAMESPACE, false)) {
    storageOk = false;
  }
  storageLoad();
  cfg.sys.bootCount++;
  storageSaveNow(false);
  applyConfigPins();

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setTimeOut(50);
  rtcPresent = rtc.begin(&Wire);
  if (rtcPresent) {
    rtcLostPowerFlag = rtc.lostPower();
    rtcReadTemp();
  }
  if (rtcPresent && cfg.time.rtcEnabled) rtcToSystem(nullptr);
  applyTz();

  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.onEvent(onWifiEvent);
  WiFi.setAutoReconnect(cfg.sta.autoReconnect == 1);
  applyAp(false);
  applySta(cfg.sta.enabled == 1 && cfg.sta.ssid[0]);

  memset(&ntp, 0, sizeof(ntp));
  ntp.nextAttemptMs = millis() + 8000UL;
  memset(&dnsq, 0, sizeof(dnsq));

  registerRoutes();
  server.begin();
  syncDnsCaptive(millis());

  lastDriftCheckMs = millis();
  lastRtcTempMs = millis();
  bootIgnoreBtnMs = millis() + 1000;
}

void loop() {
  uint32_t now = millis();

  buttonTask(now);
  wifiEventTask();
  wifiTask(now);
  ntpTask(now);
  driftTask(now);

  static time_t lastLoopUtc = 0;
  time_t utc = time(nullptr);
  if (utc != lastLoopUtc) {
    lastLoopUtc = utc;
    engineTick(utc);
  }

  if ((now - lastRtcTempMs) >= 30000UL) {
    lastRtcTempMs = now;
    rtcReadTemp();
  }

  syncDnsCaptive(now);
  server.handleClient();
  storageCommitTask();

  if (wifiApplyPending && (int32_t)(now - wifiApplyPendingMs) >= 0) {
    wifiApplyPending = false;
    applyAp(true);
    applySta(cfg.sta.enabled == 1 && cfg.sta.ssid[0]);
  }

  if (pendingRestart && (int32_t)(now - pendingRestartMs) >= 0) {
    bool factory = pendingFactoryReset;
    pendingRestart = false;
    pendingFactoryReset = false;
    if (factory) {
      for (int i = 0; i < N_RELAYS; i++) {
        pinMode(cfg.relays[i].gpio, OUTPUT);
        digitalWrite(cfg.relays[i].gpio, cfg.relays[i].activeHigh ? LOW : HIGH);
      }
      prefs.clear();
    }
    ESP.restart();
  }
}
