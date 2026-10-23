/*
 * ESP32 Unified Relay Scheduler
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <RTClib.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <time.h>
#include <esp_wifi.h>
#include <esp_system.h>

static inline uint64_t monoMs() { return (uint64_t)esp_timer_get_time() / 1000ULL; }

// ------------------- CONFIG -------------------
static const uint8_t RELAY_COUNT = 16;
static const uint8_t RELAY_PINS[RELAY_COUNT] = {
  23, 32, 33, 25, 26, 27, 14, 13, 1, 3, 19, 18, 5, 4, 2, 15
};
static const uint8_t SCHED_PER_RELAY = 8;
static const uint32_t WIFI_RETRY_MS = 2000;
static const uint32_t NVS_MIN_WRITE_MS = 30000; 
static const uint32_t FACTORY_HOLD_MS = 5000;
static const uint8_t BOOT_BTN = 0;

// ------------------- DATA MODELS -------------------
struct Schedule {
  bool enabled;
  uint8_t sSec, sMin, sHour, sDay, sWday, sDom, sMon;
  uint8_t eSec, eMin, eHour, eDay, eWday, eDom, eMon;
  bool overnight;
  bool overnightTail;
  bool leapOnly;
  bool domFilter;
  bool wdayFilter;
  bool monFilter;
};

struct RelayConfig {
  bool activeLow;
  Schedule sched[SCHED_PER_RELAY];
  bool forced;
  bool forcedState;
};

struct AppConfig {
  char apSsid[33];
  char apPass[65];
  uint8_t apChannel;
  uint8_t apTxPower;
  bool apHidden;
  char staSsid[33];
  char staPass[65];
  bool staEnabled;
  char ntpServers[3][64];
  uint8_t ntpServerCount;
  uint32_t ntpSyncIntervalSec;
  int32_t gmtOffsetSec;
  bool autoTz;
  char tzRule[64];
  bool rtcEnabled;
  RelayConfig relays[RELAY_COUNT];
  uint32_t configVersion;
};

// ------------------- GLOBALS -------------------
AppConfig cfg;
RTC_DS3231 rtc;
WebServer server(80);
DNSServer dnsServer;
Preferences prefs;
bool rtcOK = false;
bool staConnected = false;
bool ntpSynced = false;
uint64_t lastWifiAttempt = 0;
uint64_t lastNtpAttempt = 0;
uint64_t lastNtpSync = 0;
uint8_t ntpServerIdx = 0;
uint64_t lastNvsWrite = 0;
uint64_t bootBtnDownAt = 0;
uint64_t bootMs = 0;
bool bootBtnHeld = false;
bool configDirty = false;
struct TimeNow {
  int year; uint8_t month, day, hour, minute, second, wday;
} nowT;

// ------------------- PROTOTYPES -------------------
void loadConfig();
void saveConfig(bool force);
void applyDefaults();
void setupRelays();
void applyRelay(uint8_t i);
void checkSchedules();
bool matchSchedule(const Schedule& s, const struct tm& t);
void setupWeb();
void handleRoot();
void handleRelayPage();
void handleGpioPage();
void handleTimePage();
void handleApPage();
void handleStaPage();
void handleStatusPage();
void handleApiRelay();
void handleApiGpio();
void handleApiTime();
void handleApiAp();
void handleApiSta();
void handleApiStatus();
void handleApiSched();
void handleCaptive();
void updateTime();
bool updateRtcFromSystem();
void updateSystemFromRtc();
void syncNtp();
void wifiTask();
void bootButtonTask();
String headerClock();
String pageHeader(const String& title);
String pageFooter();
String ipToString(IPAddress ip);
uint32_t crc32(const uint8_t* data, size_t len);
bool writeConfigAtomic(const AppConfig& c);
bool readConfigAtomic(AppConfig& c);

// ------------------- CONFIG PERSISTENCE -------------------
static const char* NVS_NS = "appcfg";
static const char* NVS_KEY = "cfg";

void applyDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  strncpy(cfg.apSsid, "ESP32-Relay", sizeof(cfg.apSsid)-1);
  strncpy(cfg.apPass, "12345678", sizeof(cfg.apPass)-1);
  cfg.apChannel = 6;
  cfg.apTxPower = 80;
  cfg.apHidden = false;
  cfg.staEnabled = false;
  strncpy(cfg.ntpServers[0], "pool.ntp.org", 63);
  strncpy(cfg.ntpServers[1], "time.nist.gov", 63);
  strncpy(cfg.ntpServers[2], "time.google.com", 63);
  cfg.ntpServerCount = 3;
  cfg.ntpSyncIntervalSec = 3600;
  cfg.gmtOffsetSec = 0;
  cfg.autoTz = false;
  strncpy(cfg.tzRule, "UTC0", sizeof(cfg.tzRule)-1);
  cfg.rtcEnabled = true;
  cfg.configVersion = 1;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    cfg.relays[i].activeLow = false;
    cfg.relays[i].forced = false;
    cfg.relays[i].forcedState = false;
    memset(cfg.relays[i].sched, 0, sizeof(cfg.relays[i].sched));
    for (uint8_t k = 0; k < SCHED_PER_RELAY; k++) {
      cfg.relays[i].sched[k].sWday = 255;
      cfg.relays[i].sched[k].eWday = 255;
    }
  }
}

uint32_t crc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
  }
  return ~crc;
}

bool writeConfigAtomic(const AppConfig& c) {
  size_t sz = sizeof(AppConfig);
  uint8_t* buf = (uint8_t*)malloc(sz + 4);
  if (!buf) return false;
  memcpy(buf, &c, sz);
  uint32_t crc = crc32((uint8_t*)&c, sz);
  memcpy(buf + sz, &crc, 4);
  prefs.begin(NVS_NS, false);
  size_t written = prefs.putBytes(NVS_KEY, buf, sz + 4);
  prefs.end();
  free(buf);
  return written == sz + 4;
}

bool readConfigAtomic(AppConfig& c) {
  prefs.begin(NVS_NS, true);
  size_t sz = prefs.getBytesLength(NVS_KEY);
  if (sz != sizeof(AppConfig) + 4) { prefs.end(); return false; }
  uint8_t* buf = (uint8_t*)malloc(sz);
  if (!buf) { prefs.end(); return false; }
  prefs.getBytes(NVS_KEY, buf, sz);
  prefs.end();
  uint32_t crcStored;
  memcpy(&crcStored, buf + sizeof(AppConfig), 4);
  memcpy(&c, buf, sizeof(AppConfig));
  uint32_t crcCalc = crc32((uint8_t*)&c, sizeof(AppConfig));
  free(buf);
  return crcStored == crcCalc;
}

void loadConfig() {
  bool ok = readConfigAtomic(cfg);
  if (!ok) {
    applyDefaults();
    writeConfigAtomic(cfg);
    return;
  }
  bool migrated = false;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    for (uint8_t k = 0; k < SCHED_PER_RELAY; k++) {
      Schedule& s = cfg.relays[i].sched[k];
      if (!s.wdayFilter && s.sWday != 255) { s.sWday = 255; migrated = true; }
      if (s.eWday == 0) { s.eWday = 255; migrated = true; }
    }
  }
  if (migrated) writeConfigAtomic(cfg);
}

void saveConfig(bool force) {
  uint64_t now = monoMs();
  if (!force && (now - lastNvsWrite) < (uint64_t)NVS_MIN_WRITE_MS) { configDirty = true; return; }
  cfg.configVersion++;
  if (writeConfigAtomic(cfg)) {
    lastNvsWrite = now;
    configDirty = false;
  } else {
    cfg.configVersion--;
  }
}

// ------------------- RELAYS -------------------
void applyRelay(uint8_t i) {
  bool state = cfg.relays[i].forced ? cfg.relays[i].forcedState : false;
  bool level = cfg.relays[i].activeLow ? !state : state;
  digitalWrite(RELAY_PINS[i], level ? HIGH : LOW);
}

void setupRelays() {
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    pinMode(RELAY_PINS[i], OUTPUT);
    bool level = cfg.relays[i].activeLow ? HIGH : LOW;
    digitalWrite(RELAY_PINS[i], level);
  }
}

// ------------------- SCHEDULER -------------------
bool matchSchedule(const Schedule& s, const struct tm& t) {
  if (!s.enabled) return false;
  if (s.sDay != 0 && t.tm_mday != s.sDay) return false;
  if (s.leapOnly && !((t.tm_year + 1900) % 4 == 0 && ((t.tm_year + 1900) % 100 != 0 || (t.tm_year + 1900) % 400 == 0))) return false;
  if (s.monFilter && s.sMon != 0 && (t.tm_mon + 1) != s.sMon) return false;
  if (s.domFilter && s.sDom != 0 && t.tm_mday != s.sDom) return false;
  if (s.wdayFilter && s.sWday != 255 && t.tm_wday != s.sWday) return false;
  int sh = s.sHour, sm = s.sMin, ss = s.sSec;
  int eh = s.eHour, em = s.eMin, es = s.eSec;
  bool overnight = s.overnight || (eh * 3600 + em * 60 + es <= sh * 3600 + sm * 60 + ss);
  if (!overnight) {
    int cur = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
    int st = sh * 3600 + sm * 60 + ss;
    int et = eh * 3600 + em * 60 + es;
    return cur >= st && cur < et;
  } else {
    int cur = t.tm_hour * 3600 + t.tm_min * 60 + t.tm_sec;
    int st = sh * 3600 + sm * 60 + ss;
    int et = eh * 3600 + em * 60 + es;
    if (cur >= st) {
      if (s.monFilter && s.sMon != 0 && (t.tm_mon + 1) != s.sMon) return false;
      if (s.domFilter && s.sDom != 0 && t.tm_mday != s.sDom) return false;
      if (s.wdayFilter && s.sWday != 255 && t.tm_wday != s.sWday) return false;
      return true;
    }
    struct tm prev = t;
    prev.tm_mday -= 1;
    prev.tm_hour = sh; prev.tm_min = sm; prev.tm_sec = ss;
    prev.tm_isdst = -1;
    time_t prevEpoch = mktime(&prev);
    if (prevEpoch == (time_t)-1) return false;
    if (s.monFilter && s.sMon != 0 && (prev.tm_mon + 1) != s.sMon) return false;
    if (s.domFilter && s.sDom != 0 && prev.tm_mday != s.sDom) return false;
    if (s.wdayFilter && s.sWday != 255 && prev.tm_wday != s.sWday) return false;
    return cur < et;
  }
}

void checkSchedules() {
  if (!ntpSynced && !rtcOK) return;
  struct tm t;
  if (!getLocalTime(&t, 0)) return;
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    if (cfg.relays[i].forced) continue;
    bool on = false;
    for (uint8_t k = 0; k < SCHED_PER_RELAY; k++) {
      if (matchSchedule(cfg.relays[i].sched[k], t)) { on = true; break; }
    }
    bool level = cfg.relays[i].activeLow ? !on : on;
    digitalWrite(RELAY_PINS[i], level ? HIGH : LOW);
  }
}

// ------------------- TIME -------------------
void updateTime() {
  struct tm t;
  if (getLocalTime(&t, 0)) {
    nowT.year = t.tm_year + 1900;
    nowT.month = t.tm_mon + 1;
    nowT.day = t.tm_mday;
    nowT.hour = t.tm_hour;
    nowT.minute = t.tm_min;
    nowT.second = t.tm_sec;
    nowT.wday = t.tm_wday;
  }
}

bool updateRtcFromSystem() {
  if (!rtcOK) return false;
  struct tm t;
  if (!getLocalTime(&t, 0)) return false;
  rtc.adjust(DateTime(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec));
  return true;
}

void updateSystemFromRtc() {
  if (!rtcOK) return;
  DateTime n = rtc.now();
  if (n.year() < 2000) return;
  struct tm t = {0};
  t.tm_year = n.year() - 1900;
  t.tm_mon = n.month() - 1;
  t.tm_mday = n.day();
  t.tm_hour = n.hour();
  t.tm_min = n.minute();
  t.tm_sec = n.second();
  t.tm_isdst = -1;
  time_t tt = mktime(&t);
  if (tt == (time_t)-1) return;
  struct timeval tv = { tt, 0 };
  settimeofday(&tv, nullptr);
}

void syncNtp() {
  if (cfg.ntpServerCount == 0) return;
  const char* srv = cfg.ntpServers[ntpServerIdx % cfg.ntpServerCount];
  if (cfg.autoTz && cfg.tzRule[0]) {
    setenv("TZ", cfg.tzRule, 1);
    tzset();
    configTzTime(cfg.tzRule, srv);
  } else {
    configTime(cfg.gmtOffsetSec, 0, srv);
  }
  lastNtpAttempt = monoMs();
}

// ------------------- WIFI -------------------
void wifiTask() {
  if (!cfg.staEnabled) return;
  uint64_t now = monoMs();
  if (WiFi.status() != WL_CONNECTED) {
    if (now - lastWifiAttempt >= (uint64_t)WIFI_RETRY_MS) {
      lastWifiAttempt = now;
      WiFi.disconnect(false, false);
      WiFi.begin(cfg.staSsid, cfg.staPass);
    }
    staConnected = false;
  } else {
    if (!staConnected) {
      staConnected = true;
      updateSystemFromRtc(); 
      syncNtp();             
    }
    if (!ntpSynced) {
      uint64_t elapsed = now - lastNtpAttempt;
      uint64_t baseMs  = (uint64_t)cfg.ntpSyncIntervalSec * 1000ULL;
      if (elapsed >= baseMs) {
        if (cfg.ntpServerCount > 1) ntpServerIdx++;
        syncNtp();
      }
    }
    if (!ntpSynced && time(nullptr) > 1600000000) {
      struct tm tmp;
      if (getLocalTime(&tmp, 0)) {
        ntpSynced = true;
        updateRtcFromSystem();
        lastNtpSync = now;
      }
    }
    if (ntpSynced && (now - lastNtpSync) >= (uint64_t)cfg.ntpSyncIntervalSec * 1000ULL) {
      lastNtpSync = now;
      syncNtp();
    }
  }
}

// ------------------- BOOT BUTTON -------------------
void bootButtonTask() {
  static uint64_t lastEdge = 0;
  static int lastRead = HIGH;
  int r = digitalRead(BOOT_BTN);
  uint64_t now = monoMs();
  if (r != lastRead && (now - lastEdge) >= 50) { lastEdge = now; lastRead = r; }
  if (lastRead == LOW) {
    if (!bootBtnHeld) { bootBtnHeld = true; bootBtnDownAt = now; }
    else if (now - bootBtnDownAt >= (uint64_t)FACTORY_HOLD_MS) {
      WiFi.disconnect(true, true);
      WiFi.softAPdisconnect(true);
      server.stop();
      dnsServer.stop();
      prefs.begin(NVS_NS, false);
      prefs.clear();
      prefs.end();
      delay(200);
      ESP.restart();
    }
  } else {
    bootBtnHeld = false;
  }
}

// ------------------- WEB HELPERS -------------------
String ipToString(IPAddress ip) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
  return String(buf);
}

String headerClock() {
  static const char* WD[] = {"SUN","MON","TUE","WED","THU","FRI","SAT"};
  struct tm t;
  if (!getLocalTime(&t, 0)) return "--- --/--/-- --:--:--";
  char buf[64];
  snprintf(buf, sizeof(buf), "%s %02d/%02d/%02d %02d:%02d:%02d",
           WD[t.tm_wday % 7], t.tm_mon + 1, t.tm_mday, (t.tm_year + 1900) % 100,
           t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

String pageHeader(const String& title) {
  String s;
  s.reserve(1200);
  s += "<!doctype html><html><head><meta charset='utf-8'>";
  s += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  s += "<title>" + title + "</title>";
  s += "<style>body{font-family:Arial;margin:8px;background:#111;color:#eee}"
       "a{color:#4af;margin-right:8px}.hdr{display:flex;justify-content:space-between;"
       "align-items:center;background:#222;padding:6px;border-radius:6px}"
       "table{border-collapse:collapse;width:100%}td,th{border:1px solid #444;padding:2px}"
       "input,select{background:#222;color:#eee;border:1px solid #555}"
       "button{background:#2a5;color:#fff;border:0;padding:4px 8px;border-radius:4px;margin:1px}"
       ".off{background:#a33}.on{background:#2a5}</style></head><body>";
  s += "<div class='hdr'><div><b>" + title + "</b><br>";
  s += "<a href='/'>Home</a><a href='/relay'>Relay</a><a href='/gpio'>GPIO</a>"
       "<a href='/time'>Time</a><a href='/ap'>AP</a><a href='/sta'>STA</a>"
       "<a href='/status'>Status</a></div>";
  s += "<div id='clk'>" + headerClock() + "</div></div>";
  return s;
}

String pageFooter() {
  return "<script>"
         "function clkTick(){fetch('/api/clock',{cache:'no-store'})"
         ".then(r=>r.text()).then(t=>{document.getElementById('clk').textContent=t;})"
         ".catch(()=>{});}"
         "setInterval(clkTick,1000);"
         "document.addEventListener('visibilitychange',()=>{if(!document.hidden)clkTick();});"
         "</script></body></html>";
}

// ------------------- PAGES -------------------
void handleRoot() {
  String h;
  h.reserve(2000);
  h = pageHeader("Dashboard");
  h += "<p>ESP32 Relay Controller</p><ul>";
  h += "<li><a href='/relay'>Relay Management (16)</a></li>";
  h += "<li><a href='/gpio'>GPIO Management</a></li>";
  h += "<li><a href='/time'>Time Management</a></li>";
  h += "<li><a href='/ap'>WiFi AP Management</a></li>";
  h += "<li><a href='/sta'>WiFi STA Management</a></li>";
  h += "<li><a href='/status'>System Status</a></li></ul>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleRelayPage() {
  String h;
  h.reserve(38000);
  h = pageHeader("Relay Management");
  h += "<p>Double-tap relay name to edit schedules. Each relay has 8 schedule pairs.</p>";
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    h += "<div style='border:1px solid #444;margin:4px;padding:4px'>";
    h += "<b ondblclick=\"document.getElementById('r"+String(i)+"').style.display='block'\">Relay " + String(i) + " (GPIO " + String(RELAY_PINS[i]) + ")</b>";
    h += "<div id='r"+String(i)+"' style='display:none'>";
    h += "<button onclick=\"fetch('/api/relay?i="+String(i)+"&act=on')\">ON</button>";
    h += "<button onclick=\"fetch('/api/relay?i="+String(i)+"&act=off')\">OFF</button>";
    h += "<button onclick=\"fetch('/api/relay?i="+String(i)+"&act=auto')\">AUTO</button>";
    for (uint8_t k = 0; k < SCHED_PER_RELAY; k++) {
      h += "<div style='border-top:1px dashed #555;padding:2px'>Schedule " + String(k) + "<br>";
      h += "<label><input type='checkbox' id='e"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].enabled?" checked":"")+"> enabled</label><br>";
      h += "S: <input size=2 id='ss"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sSec)+"'>";
      h += "<input size=2 id='sm"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sMin)+"'>";
      h += "<input size=2 id='sh"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sHour)+"'>";
      h += "<input size=2 id='sd"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sDom)+"'>";
      h += "<input size=2 id='sw"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sWday==255?String(""):String(cfg.relays[i].sched[k].sWday))+"'>";
      h += "<input size=2 id='smo"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].sMon)+"'><br>";
      h += "E: <input size=2 id='es"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eSec)+"'>";
      h += "<input size=2 id='em"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eMin)+"'>";
      h += "<input size=2 id='eh"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eHour)+"'>";
      h += "<input size=2 id='ed"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eDom)+"'>";
      h += "<input size=2 id='ew"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eWday==255?String(""):String(cfg.relays[i].sched[k].eWday))+"'>";
      h += "<input size=2 id='emo"+String(i)+"_"+String(k)+"' value='"+String(cfg.relays[i].sched[k].eMon)+"'><br>";
      h += "<label><input type='checkbox' id='on"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].overnight?" checked":"")+"> overnight</label>";
      h += "<label><input type='checkbox' id='ot"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].overnightTail?" checked":"")+"> overnight tail</label>";
      h += "<label><input type='checkbox' id='lp"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].leapOnly?" checked":"")+"> leap only</label>";
      h += "<label><input type='checkbox' id='df"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].domFilter?" checked":"")+"> dom filter</label>";
      h += "<label><input type='checkbox' id='wf"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].wdayFilter?" checked":"")+"> wday filter</label>";
      h += "<label><input type='checkbox' id='mf"+String(i)+"_"+String(k)+"'"+(cfg.relays[i].sched[k].monFilter?" checked":"")+"> mon filter</label>";
      h += "<button onclick=\"saveSched("+String(i)+","+String(k)+")\">Save</button>";
      h += "</div>";
    }
    h += "</div></div>";
  }
  h += "<script>async function saveSched(i,k){const g=id=>document.getElementById(id).value;"
       "const c=id=>document.getElementById(id).checked;"
       "const q=`i=${i}&k=${k}&en=${c('e'+i+'_'+k)}&ss=${g('ss'+i+'_'+k)}&sm=${g('sm'+i+'_'+k)}&sh=${g('sh'+i+'_'+k)}"
       "&sd=${g('sd'+i+'_'+k)}&sw=${g('sw'+i+'_'+k)===''?255:g('sw'+i+'_'+k)}&smo=${g('smo'+i+'_'+k)}"
       "&es=${g('es'+i+'_'+k)}&em=${g('em'+i+'_'+k)}&eh=${g('eh'+i+'_'+k)}&ed=${g('ed'+i+'_'+k)}&ew=${g('ew'+i+'_'+k)===''?255:g('ew'+i+'_'+k)}&emo=${g('emo'+i+'_'+k)}"
       "&on=${c('on'+i+'_'+k)}&ot=${c('ot'+i+'_'+k)}&lp=${c('lp'+i+'_'+k)}"
       "&df=${c('df'+i+'_'+k)}&wf=${c('wf'+i+'_'+k)}&mf=${c('mf'+i+'_'+k)}`;"
       "await fetch('/api/sched',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:q});"
       "alert('Saved');}</script>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleGpioPage() {
  String h;
  h.reserve(4000);
  h = pageHeader("GPIO Management");
  h += "<button onclick=\"fetch('/api/gpio?act=allhigh')\">All ON</button>";
  h += "<button onclick=\"fetch('/api/gpio?act=alllow')\">All OFF</button>";
  h += "<table><tr><th>#</th><th>GPIO</th><th>Active</th><th>Set</th><th>Remove</th></tr>";
  for (uint8_t i = 0; i < RELAY_COUNT; i++) {
    h += "<tr><td>"+String(i)+"</td><td>"+String(RELAY_PINS[i])+"</td>";
    h += "<td><select id='al"+String(i)+"' onchange=\"fetch('/api/gpio?act=active&i="+String(i)+"&v='+this.value)\">";
    h += String("<option value='0'") + (cfg.relays[i].activeLow ? "" : " selected") + ">Active HIGH</option>";
    h += String("<option value='1'") + (cfg.relays[i].activeLow ? " selected" : "") + ">Active LOW</option></select></td>";
    h += "<td><button onclick=\"fetch('/api/gpio?act=sethigh&i="+String(i)+"')\">Force ON</button></td>";
    h += "<td><button class='off' onclick=\"fetch('/api/gpio?act=remove&i="+String(i)+"')\">Remove</button></td></tr>";
  }
  h += "</table>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleTimePage() {
  String h;
  h.reserve(3500);
  h = pageHeader("Time Management");
  h += "<form id='f'>";
  h += "NTP1 <input name='n1' value='"+String(cfg.ntpServers[0])+"'><br>";
  h += "NTP2 <input name='n2' value='"+String(cfg.ntpServers[1])+"'><br>";
  h += "NTP3 <input name='n3' value='"+String(cfg.ntpServers[2])+"'><br>";
  h += "Sync interval (s, up to 86400) <input name='iv' value='"+String(cfg.ntpSyncIntervalSec)+"'><br>";
  h += "GMT offset (s) <input name='off' value='"+String(cfg.gmtOffsetSec)+"'><br>";
  h += String("Auto TZ <input type='checkbox' name='atz'") + (cfg.autoTz ? " checked" : "") + "><br>";
  h += "TZ rule <input name='tz' value='"+String(cfg.tzRule)+"'><br>";
  h += "<button type='button' onclick='saveTime()'>Save</button>";
  h += "<button type='button' onclick=\"fetch('/api/time?act=ntp')\">Sync NTP now</button>";
  h += "<button type='button' onclick='syncBrowser()'>Sync Browser</button>";
  h += "<button type='button' onclick=\"fetch('/api/time?act=rtc2sys')\">RTC → System</button>";
  h += "<button type='button' onclick=\"fetch('/api/time?act=sys2rtc')\">System → RTC</button>";
  h += "</form><script>"
       "async function saveTime(){const f=new FormData(document.getElementById('f'));"
       "await fetch('/api/time',{method:'POST',body:new URLSearchParams(f)});alert('Saved');}"
       "async function syncBrowser(){const now=Math.floor(Date.now()/1000);"
       "await fetch('/api/time?act=browser&t='+now);alert('Browser time set');}"
       "</script>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleApPage() {
  String h;
  h.reserve(2000);
  h = pageHeader("WiFi AP Management");
  h += "<form id='f'>";
  h += "SSID <input name='ssid' value='"+String(cfg.apSsid)+"'><br>";
  h += "Password <input name='pass' value='"+String(cfg.apPass)+"'><br>";
  h += "Channel (1..13) <input name='ch' value='"+String(cfg.apChannel)+"'><br>";
  h += "TX power (8..84, 0.25 dBm/step) <input name='tx' value='"+String(cfg.apTxPower)+"'><br>";
  h += String("Hidden <input type='checkbox' name='hid'") + (cfg.apHidden ? " checked" : "") + "><br>";
  h += "<button type='button' onclick='saveAp()'>Save</button></form>";
  h += "<script>async function saveAp(){const f=new FormData(document.getElementById('f'));"
       "const r=await fetch('/api/ap',{method:'POST',body:new URLSearchParams(f)});"
       "if(r.ok)alert('Saved');else alert(await r.text());}</script>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleStaPage() {
  String h;
  h.reserve(2000);
  h = pageHeader("WiFi STA Management");
  h += "<form id='f'>";
  h += "SSID <input name='ssid' value='"+String(cfg.staSsid)+"'><br>";
  h += "Password <input name='pass' value='"+String(cfg.staPass)+"'><br>";
  h += String("Enabled <input type='checkbox' name='en'") + (cfg.staEnabled ? " checked" : "") + "><br>";
  h += "<button type='button' onclick='saveSta()'>Save</button></form>";
  h += "<script>async function saveSta(){const f=new FormData(document.getElementById('f'));"
       "await fetch('/api/sta',{method:'POST',body:new URLSearchParams(f)});alert('Saved');}</script>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

void handleStatusPage() {
  String h;
  h.reserve(2000);
  h = pageHeader("System Status");
  h += "<p>Heap: " + String(ESP.getFreeHeap()) + " bytes</p>";
  h += "<p>Uptime: " + String((monoMs() - bootMs) / 1000ULL) + " s</p>";
  h += "<p>WiFi: " + String(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected") + "</p>";
  h += "<p>IP: " + ipToString(WiFi.localIP()) + "</p>";
  h += "<p>RTC: " + String(rtcOK ? "OK" : "not found") + "</p>";
  h += "<p>NTP synced: " + String(ntpSynced ? "yes" : "no") + "</p>";
  h += "<p>Config version: " + String(cfg.configVersion) + "</p>";
  h += pageFooter();
  server.send(200, "text/html", h);
}

// ------------------- API -------------------
void handleApiRelay() {
  if (!server.hasArg("i")) { server.send(400); return; }
  uint8_t i = server.arg("i").toInt();
  if (i >= RELAY_COUNT) { server.send(400); return; }
  String act = server.arg("act");
  if (act == "on") { cfg.relays[i].forced = true; cfg.relays[i].forcedState = true; applyRelay(i); }
  else if (act == "off") { cfg.relays[i].forced = true; cfg.relays[i].forcedState = false; applyRelay(i); }
  else if (act == "auto") { cfg.relays[i].forced = false; checkSchedules(); }
  saveConfig(false);
  server.send(200, "text/plain", "ok");
}

void handleApiGpio() {
  String act = server.arg("act");
  if (act == "allhigh") {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) { cfg.relays[i].forced = true; cfg.relays[i].forcedState = true; applyRelay(i); }
  } else if (act == "alllow") {
    for (uint8_t i = 0; i < RELAY_COUNT; i++) { cfg.relays[i].forced = true; cfg.relays[i].forcedState = false; applyRelay(i); }
  } else if (act == "sethigh") {
    uint8_t i = server.arg("i").toInt(); if (i < RELAY_COUNT) { cfg.relays[i].forced = true; cfg.relays[i].forcedState = true; applyRelay(i); }
  } else if (act == "setlow") {
    uint8_t i = server.arg("i").toInt(); if (i < RELAY_COUNT) { cfg.relays[i].forced = true; cfg.relays[i].forcedState = false; applyRelay(i); }
  } else if (act == "remove") {
    uint8_t i = server.arg("i").toInt(); if (i < RELAY_COUNT) { cfg.relays[i].forced = false; checkSchedules(); }
  } else if (act == "active") {
    uint8_t i = server.arg("i").toInt();
    if (i < RELAY_COUNT) {
      cfg.relays[i].activeLow = (server.arg("v").toInt() != 0);
      if (cfg.relays[i].forced) applyRelay(i);
      else checkSchedules();
    } else { server.send(400, "text/plain", "bad i"); return; }
  }
  saveConfig(false);
  server.send(200, "text/plain", "ok");
}

void handleApiTime() {
  if (server.method() == HTTP_POST) {
    for (int k = 0; k < 3; k++) memset(cfg.ntpServers[k], 0, sizeof(cfg.ntpServers[k]));
    strncpy(cfg.ntpServers[0], server.arg("n1").c_str(), sizeof(cfg.ntpServers[0]) - 1);
    strncpy(cfg.ntpServers[1], server.arg("n2").c_str(), sizeof(cfg.ntpServers[1]) - 1);
    strncpy(cfg.ntpServers[2], server.arg("n3").c_str(), sizeof(cfg.ntpServers[2]) - 1);
    long ivRaw = server.arg("iv").toInt();
    uint32_t iv;
    if (ivRaw < 60L)          iv = 60;
    else if (ivRaw > 86400L)  iv = 86400;
    else                      iv = (uint32_t)ivRaw;
    cfg.ntpSyncIntervalSec = iv;
    {
      long off = server.arg("off").toInt();
      if (off < -50400L || off > 50400L) off = 0;
      cfg.gmtOffsetSec = (int32_t)off;
    }
    cfg.autoTz = server.hasArg("atz");
    memset(cfg.tzRule, 0, sizeof(cfg.tzRule));
    strncpy(cfg.tzRule, server.arg("tz").c_str(), sizeof(cfg.tzRule) - 1);
    saveConfig(true);
    if (cfg.autoTz && cfg.tzRule[0]) {
      setenv("TZ", cfg.tzRule, 1);
      tzset();
      configTzTime(cfg.tzRule, cfg.ntpServers[0]);
    } else {
      configTime(cfg.gmtOffsetSec, 0, cfg.ntpServers[0]);
    }
    server.send(200, "text/plain", "ok");
    return;
  }
  String act = server.arg("act");
  if (act == "ntp") { syncNtp(); }
  else if (act == "browser") {
    int64_t t = strtoll(server.arg("t").c_str(), nullptr, 10);
    if (t < 1600000000LL || t > 4000000000LL) { server.send(400, "text/plain", "bad time"); return; }
    struct timeval tv = { (time_t)t, 0 };
    settimeofday(&tv, nullptr);
    updateRtcFromSystem();
  }
  else if (act == "rtc2sys") updateSystemFromRtc();
  else if (act == "sys2rtc") updateRtcFromSystem();
  server.send(200, "text/plain", "ok");
}

void handleApiAp() {
  {
    size_t plen = server.arg("pass").length();
    if (plen > 0 && plen < 8) { server.send(400, "text/plain", "AP password must be empty or >= 8 chars"); return; }
    if (server.arg("ssid").length() == 0) { server.send(400, "text/plain", "AP SSID required"); return; }
    int tx = server.arg("tx").toInt();
    if (tx < 8 || tx > 84) { server.send(400, "text/plain", "TX power 8..84"); return; }
    int ch = server.arg("ch").toInt();
    if (ch < 1 || ch > 13) { server.send(400, "text/plain", "channel 1..13"); return; }
  }
  memset(cfg.apSsid, 0, sizeof(cfg.apSsid));
  memset(cfg.apPass, 0, sizeof(cfg.apPass));
  strncpy(cfg.apSsid, server.arg("ssid").c_str(), sizeof(cfg.apSsid) - 1);
  strncpy(cfg.apPass, server.arg("pass").c_str(), sizeof(cfg.apPass) - 1);
  cfg.apChannel = server.arg("ch").toInt();
  cfg.apTxPower = server.arg("tx").toInt();
  cfg.apHidden = server.hasArg("hid");
  saveConfig(true);
  WiFi.softAP(cfg.apSsid, cfg.apPass, cfg.apChannel, cfg.apHidden);
  esp_wifi_set_max_tx_power(cfg.apTxPower);
  server.send(200, "text/plain", "ok");
}

void handleApiSta() {
  {
    size_t plen = server.arg("pass").length();
    if (plen > 0 && plen < 8) { server.send(400, "text/plain", "STA password must be empty or >= 8 chars"); return; }
    if (server.arg("ssid").length() == 0 && server.hasArg("en")) { server.send(400, "text/plain", "STA SSID required"); return; }
  }
  memset(cfg.staSsid, 0, sizeof(cfg.staSsid));
  memset(cfg.staPass, 0, sizeof(cfg.staPass));
  strncpy(cfg.staSsid, server.arg("ssid").c_str(), sizeof(cfg.staSsid) - 1);
  strncpy(cfg.staPass, server.arg("pass").c_str(), sizeof(cfg.staPass) - 1);
  cfg.staEnabled = server.hasArg("en");
  saveConfig(true);
  WiFi.disconnect(false, false);
  if (cfg.staEnabled) WiFi.begin(cfg.staSsid, cfg.staPass);
  server.send(200, "text/plain", "ok");
}

void handleApiStatus() {
  DynamicJsonDocument d(512);
  d["heap"] = ESP.getFreeHeap();
  d["uptime"] = (uint64_t)((monoMs() - bootMs) / 1000ULL);
  d["wifi"] = (WiFi.status() == WL_CONNECTED);
  d["ip"] = ipToString(WiFi.localIP());
  String s; serializeJson(d, s);
  server.send(200, "application/json", s);
}

void handleApiSched() {
  if (!server.hasArg("i") || !server.hasArg("k")) { server.send(400); return; }
  uint8_t i = server.arg("i").toInt();
  uint8_t k = server.arg("k").toInt();
  if (i >= RELAY_COUNT || k >= SCHED_PER_RELAY) { server.send(400); return; }
  Schedule& s = cfg.relays[i].sched[k];
  s.enabled = server.hasArg("en");
  s.sSec = server.arg("ss").toInt();
  s.sMin = server.arg("sm").toInt();
  s.sHour = server.arg("sh").toInt();
  s.sDom = server.arg("sd").toInt();
  { int v = server.arg("sw").toInt(); s.sWday = (v < 0 || v > 6) ? 255 : (uint8_t)v; }
  s.sMon = server.arg("smo").toInt();
  s.eSec = server.arg("es").toInt();
  s.eMin = server.arg("em").toInt();
  s.eHour = server.arg("eh").toInt();
  s.eDom = server.arg("ed").toInt();
  { int v = server.arg("ew").toInt(); s.eWday = (v < 0 || v > 6) ? 255 : (uint8_t)v; }
  s.eMon = server.arg("emo").toInt();
  s.overnight = server.hasArg("on");
  s.overnightTail = server.hasArg("ot");
  s.leapOnly = server.hasArg("lp");
  s.domFilter = server.hasArg("df");
  s.wdayFilter = server.hasArg("wf");
  s.monFilter = server.hasArg("mf");
  if (s.sSec  > 59) s.sSec  = 59;
  if (s.sMin  > 59) s.sMin  = 59;
  if (s.sHour > 23) s.sHour = 23;
  if (s.eSec  > 59) s.eSec  = 59;
  if (s.eMin  > 59) s.eMin  = 59;
  if (s.eHour > 23) s.eHour = 23;
  int st = s.sHour * 3600 + s.sMin * 60 + s.sSec;
  int et = s.eHour * 3600 + s.eMin * 60 + s.eSec;
  if (!s.overnight && st == et) {
    server.send(400, "text/plain", "start equals end");
    return;
  }
  saveConfig(true);
  server.send(200, "text/plain", "ok");
}

void handleCaptive() {
  IPAddress clientIp = server.client().remoteIP();
  IPAddress apIp = WiFi.softAPIP();
  bool onAp = (clientIp[0] == apIp[0] && clientIp[1] == apIp[1] && clientIp[2] == apIp[2]);
  if (!onAp) { server.send(404, "text/plain", "not found"); return; }
  server.sendHeader("Location", "http://" + ipToString(apIp) + "/", true);
  server.send(302, "text/plain", "");
}

// ------------------- WEB SETUP -------------------
void setupWeb() {
  server.on("/", handleRoot);
  server.on("/relay", handleRelayPage);
  server.on("/gpio", handleGpioPage);
  server.on("/time", handleTimePage);
  server.on("/ap", handleApPage);
  server.on("/sta", handleStaPage);
  server.on("/status", handleStatusPage);
  server.on("/api/relay", handleApiRelay);
  server.on("/api/gpio", handleApiGpio);
  server.on("/api/time", handleApiTime);
  server.on("/api/ap", handleApiAp);
  server.on("/api/sta", handleApiSta);
  server.on("/api/status", handleApiStatus);
  server.on("/api/sched", handleApiSched);
  server.on("/api/clock", [](){ server.send(200, "text/plain", headerClock()); });
  server.on("/generate_204", [](){ server.send(204, "text/plain", ""); });
  server.on("/gen_204", [](){ server.send(204, "text/plain", ""); });
  server.on("/hotspot-detect.html", handleCaptive);
  server.on("/library/test/success.html", handleCaptive);
  server.on("/connecttest.txt", handleCaptive);
  server.on("/redirect", handleCaptive);
  server.on("/ncsi.txt", handleCaptive);
  server.onNotFound(handleCaptive);
  server.begin();
}

// ------------------- SETUP -------------------
void setup() {
  pinMode(BOOT_BTN, INPUT_PULLUP);
  bootMs = monoMs();
  loadConfig();
  setupRelays();
  Wire.begin(21, 22);
  rtcOK = rtc.begin();
  if (rtcOK) updateSystemFromRtc();
  if (time(nullptr) < 1600000000) {
    struct tm seed = {0};
    seed.tm_year = 2024 - 1900; seed.tm_mon = 0; seed.tm_mday = 1;
    seed.tm_hour = 0; seed.tm_min = 0; seed.tm_sec = 0; seed.tm_isdst = -1;
    time_t t = mktime(&seed);
    struct timeval tv = { t, 0 };
    settimeofday(&tv, nullptr);
  }
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(cfg.apSsid, cfg.apPass, cfg.apChannel, cfg.apHidden);
  esp_wifi_set_max_tx_power(cfg.apTxPower);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  if (cfg.staEnabled) WiFi.begin(cfg.staSsid, cfg.staPass);
  dnsServer.start(53, "*", WiFi.softAPIP());
  setupWeb();
  if (MDNS.begin("esp32relay")) MDNS.addService("http", "tcp", 80);
  syncNtp();
}

// ------------------- LOOP  ---------------------
void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  wifiTask();
  bootButtonTask();
  static uint64_t lastScheduleTick = 0;
  uint64_t now = monoMs();
  if (now - lastScheduleTick >= 1000) {
    lastScheduleTick = now;
    updateTime();
    checkSchedules();
  }
  if (configDirty && (now - lastNvsWrite) >= (uint64_t)NVS_MIN_WRITE_MS) saveConfig(false);
  delay(1);
}
