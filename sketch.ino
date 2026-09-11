#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <PubSubClient.h>
#include <time.h>
#include <Wire.h>
#include <RTClib.h>

const char* DEFAULT_AP_SSID = "ESP32_16CH_Relay_Controller";
const char* DEFAULT_AP_PASS = "12345678";
const char* DEFAULT_STA_HOST = "esp32-16ch";
const char* DEFAULT_MQTT_BASE = "esp32_16ch";
const char* DEFAULT_NTP_1 = "pool.ntp.org";
const char* DEFAULT_NTP_2 = "time.google.com";

const int DEFAULT_RELAY_PINS[16] = {32, 33, 25, 26, 27, 14, 13, 23, 1, 3, 19, 18, 5, 4, 2, 15};

const int VALID_PINS[] = {1, 2, 3, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
const int VALID_PINS_COUNT = sizeof(VALID_PINS) / sizeof(VALID_PINS[0]);

const IPAddress AP_DEFAULT_IP(192, 168, 4, 1);
const IPAddress AP_DEFAULT_GW(192, 168, 4, 1);
const IPAddress AP_DEFAULT_MASK(255, 255, 255, 0);

const unsigned long STA_ONLY_BOOT_TIMEOUT_MS    = 60000UL;
const unsigned long STA_RUNTIME_DROP_TIMEOUT_MS = 120000UL;
const unsigned long STA_ONLY_DROP_GRACE_MS      = 10000UL;
const unsigned long STA_RETRY_INTERVAL_MS       = 30000UL;
const unsigned long MQTT_BACKOFF_MIN_MS         = 2000UL;
const unsigned long MQTT_BACKOFF_MAX_MS         = 60000UL;

const int  SCHED_COUNT = 8;
const uint8_t ANY8  = 0xFF;
const uint32_t ANY32 = 0xFFFFFFFFUL;

const int RTC_SDA_PIN = 21;
const int RTC_SCL_PIN = 22;

enum SchedMode : uint8_t {
  MODE_NORMAL         = 0,
  MODE_OVERNIGHT      = 1,
  MODE_OVERNIGHT_TAIL = 2
};

struct Schedule {
  bool     enabled;
  uint8_t  mode;
  uint16_t relayMask;
  uint8_t  startSec,  startMin,  startHour;
  uint8_t  stopSec,   stopMin,   stopHour;
  uint32_t startDowMask;
  uint32_t stopDowMask;
  uint32_t domMask;
  uint16_t monthMask;
  uint32_t tailSeconds;
  char     name[24];
  bool     matchedPrev;
  bool     tailArmed;
  time_t   tailArmedAt;
};

WebServer server(80);
DNSServer dnsServer;
Preferences prefsRelay;
Preferences prefsAP;
Preferences prefsSTA;
Preferences prefsMQTT;
Preferences prefsSched;
Preferences prefsTime;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
RTC_DS3231 rtc;

bool   relayState[16];
bool   relayActiveHigh[16];
int    relayPins[16];
String relayNames[16];

bool     apEnabled      = true;
String   apSSID         = DEFAULT_AP_SSID;
String   apPassword     = DEFAULT_AP_PASS;
int      apChannel      = 1;
int      apMaxClients   = 4;
bool     apHidden       = false;

bool     staEnabled     = false;
String   staSSID        = "";
String   staPassword    = "";
String   staHostname    = DEFAULT_STA_HOST;
bool     staStatic      = false;
IPAddress staIP;
IPAddress staGW;
IPAddress staSN;
IPAddress staDNS;

bool     mqttEnabled   = false;
String   mqttServer    = "";
uint16_t mqttPort      = 1883;
String   mqttUser      = "";
String   mqttPass      = "";
String   mqttBaseTopic = DEFAULT_MQTT_BASE;
String   mqttClientId  = "";

unsigned long lastMqttReconnect = 0;
unsigned long mqttBackoff       = MQTT_BACKOFF_MIN_MS;

volatile bool pendingState[16] = {false};
volatile bool pendingAll       = false;

Schedule schedules[SCHED_COUNT];
time_t   lastSchedTick = 0;
bool     timeSynced    = false;

int      tzOffsetMinutes = -480;
String   ntpServer1      = DEFAULT_NTP_1;
String   ntpServer2      = DEFAULT_NTP_2;
bool     rtcEnabled      = false;
bool     rtcPresent      = false;
bool     ntpSynced       = false;
time_t   lastRtcCheck    = 0;
bool     rtcValidAtBoot  = false;

bool          staOnlyMode          = false;
unsigned long staOnlyDeadline      = 0;
unsigned long staOnlyConnectedAt   = 0;
unsigned long staRuntimeDropStart  = 0;
bool          wasStaConnected      = false;
unsigned long lastStaRetry         = 0;
bool          scanInProgress       = false;

void loadRelayPrefs();
void loadAPPrefs();
void loadSTAPrefs();
void loadMQTTPrefs();
void loadSchedPrefs();
void saveSchedPrefs();
void loadTimePrefs();
void saveTimePrefs();
void initRelays();
void applyWifiMode();
void staWatchdog();
void scheduleTick();
void applyTimeConfig();
void initRTC();
void rtcMaintenance();
bool i2cPinsFree();

String jsonEscape(const String& s);
String sanitizeId(const String& s);
bool   parseIP(const String& s, IPAddress& out);
bool   isValidPin(int pin);
inline void writeRelay(int i);
String staStatusString();
String buildTZString(int offsetMinutes);
String formatLocalTime();

void publishRelayState(int idx);
void publishAllStates();
void publishName(int idx);
void publishAllNames();
void publishDiscoveryOne(int i);
void publishDiscovery();
void mqttCallback(char* topic, byte* payload, unsigned int length);
void mqttConnect();
void mqttLoop();
void flushPendingPublishes();

void handleRoot();
void handleToggle();
void handleRename();
void handleWifiApPage();
void handleWifiStaPage();
void handleWifiScan();
void handleSetAp();
void handleSetSta();
void handleGpioPage();
void handleGPIOSet();
void handleMqttPage();
void handleSetMqtt();
void handleSchedulePage();
void handleSchedList();
void handleSchedSet();
void handleSchedDel();
void handleSchedTest();
void handleTimePage();
void handleSetTime();
void handleRtcSet();
void handleNtpNow();
void handleNotFound();

String getMainHTML();
String getWifiApHTML();
String getWifiStaHTML();
String getGpioHTML();
String getMqttHTML();
String getScheduleHTML();
String getTimeHTML();

String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 4);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') out += '\\';
    out += c;
  }
  return out;
}

String sanitizeId(const String& s) {
  String out;
  out.reserve(s.length());
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum((unsigned char)c) || c == '_' || c == '-') out += c;
    else out += '_';
  }
  return out;
}

bool parseIP(const String& s, IPAddress& out) {
  int parts[4] = {0, 0, 0, 0};
  int idx = 0, val = 0, digits = 0;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '.') {
      if (digits == 0) return false;
      parts[idx++] = val;
      if (idx > 3) return false;
      val = 0; digits = 0;
    } else if (isDigit(c)) {
      val = val * 10 + (c - '0');
      if (val > 255) return false;
      digits++;
      if (digits > 3) return false;
    } else {
      return false;
    }
  }
  if (digits == 0 || idx != 3) return false;
  parts[3] = val;
  out = IPAddress(parts[0], parts[1], parts[2], parts[3]);
  return true;
}

bool isValidPin(int pin) {
  for (int i = 0; i < VALID_PINS_COUNT; i++) {
    if (VALID_PINS[i] == pin) return true;
  }
  return false;
}

inline void writeRelay(int i) {
  bool level = relayActiveHigh[i] ? relayState[i] : !relayState[i];
  digitalWrite(relayPins[i], level ? HIGH : LOW);
}

String staStatusString() {
  switch (WiFi.status()) {
    case WL_IDLE_STATUS:     return "Idle";
    case WL_NO_SSID_AVAIL:   return "SSID not found";
    case WL_SCAN_COMPLETED:  return "Scan done";
    case WL_CONNECTED:       return "Connected";
    case WL_CONNECT_FAILED:  return "Connect failed";
    case WL_CONNECTION_LOST: return "Connection lost";
    case WL_DISCONNECTED:    return "Disconnected";
    default:                 return "Unknown";
  }
}

String buildTZString(int offsetMinutes) {
  int sign = offsetMinutes >= 0 ? -1 : 1;
  int absMin = offsetMinutes < 0 ? -offsetMinutes : offsetMinutes;
  int h = absMin / 60;
  int m = absMin % 60;
  char buf[20];
  if (m == 0) snprintf(buf, sizeof(buf), "GMT%+d", sign * h);
  else        snprintf(buf, sizeof(buf), "GMT%+d:%02d", sign * h, m);
  return String(buf);
}

String formatLocalTime() {
  time_t now = time(nullptr);
  if (now < 1700000000) return "not synced";
  struct tm t;
  localtime_r(&now, &t);
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

bool i2cPinsFree() {
  for (int i = 0; i < 16; i++) {
    if (relayPins[i] == RTC_SDA_PIN || relayPins[i] == RTC_SCL_PIN) return false;
  }
  return true;
}

void loadRelayPrefs() {
  prefsRelay.begin("relays", true);
  for (int i = 0; i < 16; i++) {
    int p = prefsRelay.getInt(("p" + String(i)).c_str(), DEFAULT_RELAY_PINS[i]);
    if (!isValidPin(p)) p = DEFAULT_RELAY_PINS[i];
    relayPins[i] = p;
    relayActiveHigh[i] = prefsRelay.getBool(("a" + String(i)).c_str(), false);
    relayState[i]      = prefsRelay.getBool(String(i).c_str(), false);
    relayNames[i]      = prefsRelay.getString(("n" + String(i)).c_str(),
                                              "Relay " + String(i + 1));
    if (relayNames[i].length() == 0) relayNames[i] = "Relay " + String(i + 1);
  }
  prefsRelay.end();
}

void loadAPPrefs() {
  prefsAP.begin("wifi_ap", true);
  apEnabled    = prefsAP.getBool  ("en",   true);
  apSSID       = prefsAP.getString("ssid", DEFAULT_AP_SSID);
  apPassword   = prefsAP.getString("pass", DEFAULT_AP_PASS);
  apChannel    = prefsAP.getInt   ("chan", 1);
  apMaxClients = prefsAP.getInt   ("maxc", 4);
  apHidden     = prefsAP.getBool  ("hidden", false);
  prefsAP.end();

  if (apSSID.length() == 0) apSSID = DEFAULT_AP_SSID;
  if (apChannel < 1 || apChannel > 13) apChannel = 1;
  if (apMaxClients < 1 || apMaxClients > 8) apMaxClients = 4;
  if (apPassword.length() != 0 && (apPassword.length() < 8 || apPassword.length() > 63)) {
    apPassword = DEFAULT_AP_PASS;
  }
}

void loadSTAPrefs() {
  prefsSTA.begin("wifi_sta", true);
  staEnabled  = prefsSTA.getBool  ("en",   false);
  staSSID     = prefsSTA.getString("ssid", "");
  staPassword = prefsSTA.getString("pass", "");
  staHostname = prefsSTA.getString("host", DEFAULT_STA_HOST);
  staStatic   = prefsSTA.getBool  ("static", false);
  String sIP  = prefsSTA.getString("ip",  "");
  String sGW  = prefsSTA.getString("gw",  "");
  String sSN  = prefsSTA.getString("sn",  "");
  String sDNS = prefsSTA.getString("dns", "");
  prefsSTA.end();

  if (staHostname.length() == 0) staHostname = DEFAULT_STA_HOST;
  if (staStatic) {
    if (!parseIP(sIP, staIP))   staStatic = false;
    if (!parseIP(sGW, staGW))   staStatic = false;
    if (!parseIP(sSN, staSN))   staStatic = false;
    if (!parseIP(sDNS, staDNS)) staDNS = staGW;
  }
}

void loadMQTTPrefs() {
  prefsMQTT.begin("mqtt", true);
  mqttEnabled   = prefsMQTT.getBool  ("en",   false);
  mqttServer    = prefsMQTT.getString("srv",  "");
  mqttPort      = prefsMQTT.getUShort("port", 1883);
  mqttUser      = prefsMQTT.getString("usr",  "");
  mqttPass      = prefsMQTT.getString("pw",   "");
  mqttBaseTopic = prefsMQTT.getString("base", DEFAULT_MQTT_BASE);
  prefsMQTT.end();

  if (mqttBaseTopic.length() == 0) mqttBaseTopic = DEFAULT_MQTT_BASE;
  if (mqttPort == 0) mqttPort = 1883;
}

void loadSchedPrefs() {
  prefsSched.begin("sched", true);
  for (int i = 0; i < SCHED_COUNT; i++) {
    String key = "s" + String(i);
    size_t blobLen = prefsSched.getBytesLength(key.c_str());
    if (blobLen == sizeof(Schedule)) {
      prefsSched.getBytes(key.c_str(), &schedules[i], sizeof(Schedule));
    } else {
      memset(&schedules[i], 0, sizeof(Schedule));
      schedules[i].enabled     = false;
      schedules[i].mode        = MODE_NORMAL;
      schedules[i].relayMask   = 0;
      schedules[i].startSec    = 0;
      schedules[i].startMin    = 0;
      schedules[i].startHour   = 0;
      schedules[i].stopSec     = 0;
      schedules[i].stopMin     = 0;
      schedules[i].stopHour    = 0;
      schedules[i].startDowMask= ANY32;
      schedules[i].stopDowMask = ANY32;
      schedules[i].domMask     = ANY32;
      schedules[i].monthMask   = 0x0FFF;
      schedules[i].tailSeconds = 0;
      snprintf(schedules[i].name, sizeof(schedules[i].name), "Schedule %d", i + 1);
    }
    schedules[i].matchedPrev = false;
    schedules[i].tailArmed   = false;
    schedules[i].tailArmedAt = 0;
  }
  prefsSched.end();
}

void saveSchedPrefs() {
  prefsSched.begin("sched", false);
  for (int i = 0; i < SCHED_COUNT; i++) {
    String key = "s" + String(i);
    prefsSched.putBytes(key.c_str(), &schedules[i], sizeof(Schedule));
  }
  prefsSched.end();
}

void loadTimePrefs() {
  prefsTime.begin("time", true);
  tzOffsetMinutes = prefsTime.getInt   ("tz",    -480);
  ntpServer1      = prefsTime.getString("ntp1",  DEFAULT_NTP_1);
  ntpServer2      = prefsTime.getString("ntp2",  DEFAULT_NTP_2);
  rtcEnabled      = prefsTime.getBool  ("rtc",   false);
  prefsTime.end();

  if (tzOffsetMinutes < -720 || tzOffsetMinutes > 840) tzOffsetMinutes = -480;
  if (ntpServer1.length() == 0) ntpServer1 = DEFAULT_NTP_1;
  if (ntpServer2.length() == 0) ntpServer2 = DEFAULT_NTP_2;
}

void saveTimePrefs() {
  prefsTime.begin("time", false);
  prefsTime.putInt   ("tz",   tzOffsetMinutes);
  prefsTime.putString("ntp1", ntpServer1);
  prefsTime.putString("ntp2", ntpServer2);
  prefsTime.putBool  ("rtc",  rtcEnabled);
  prefsTime.end();
}

void initRelays() {
  for (int i = 0; i < 16; i++) {
    pinMode(relayPins[i], OUTPUT);
    writeRelay(i);
  }
}

void applyTimeConfig() {
  String tz = buildTZString(tzOffsetMinutes);
  configTzTime(tz.c_str(), ntpServer1.c_str(), ntpServer2.c_str());
}

void initRTC() {
  rtcPresent = false;
  rtcValidAtBoot = false;
  if (!rtcEnabled) return;
  if (!i2cPinsFree()) return;

  Wire.begin(RTC_SDA_PIN, RTC_SCL_PIN);
  if (!rtc.begin(&Wire)) return;
  rtcPresent = true;

  if (rtc.lostPower()) return;

  DateTime now = rtc.now();
  if (now.year() < 2024 || now.year() > 2099) return;

  struct timeval tv;
  tv.tv_sec  = now.unixtime();
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
  rtcValidAtBoot = true;
}

void rtcMaintenance() {
  time_t now = time(nullptr);

  if (now > 1700000000) {
    if (!ntpSynced) {
      ntpSynced = true;
      if (rtcEnabled && rtcPresent) {
        DateTime dt((uint32_t)now);
        rtc.adjust(dt);
      }
    }
  } else {
    ntpSynced = false;
  }

  if (!ntpSynced && rtcEnabled && rtcPresent) {
    if (now - lastRtcCheck >= 60) {
      lastRtcCheck = now;
      DateTime rnow = rtc.now();
      if (rnow.year() >= 2024 && rnow.year() <= 2099) {
        time_t rsec = rnow.unixtime();
        time_t diff = rsec - now;
        if (diff < 0) diff = -diff;
        if (diff > 5) {
          struct timeval tv;
          tv.tv_sec  = rsec;
          tv.tv_usec = 0;
          settimeofday(&tv, nullptr);
        }
      }
    }
  }
}

void publishRelayState(int idx) {
  if (!mqttClient.connected()) return;
  String topic = mqttBaseTopic + "/relay/" + String(idx) + "/state";
  const char* payload = relayState[idx] ? "ON" : "OFF";
  mqttClient.publish(topic.c_str(), payload, true);
}

void publishAllStates() {
  if (!mqttClient.connected()) return;
  for (int i = 0; i < 16; i++) publishRelayState(i);

  String topic = mqttBaseTopic + "/status";
  String json = "{\"relays\":[";
  for (int i = 0; i < 16; i++) {
    json += relayState[i] ? "true" : "false";
    if (i < 15) json += ",";
  }
  json += "]}";
  mqttClient.publish(topic.c_str(), json.c_str(), true);
}

void publishName(int idx) {
  if (!mqttClient.connected()) return;
  String topic = mqttBaseTopic + "/relay/" + String(idx) + "/name";
  mqttClient.publish(topic.c_str(), relayNames[idx].c_str(), true);
}

void publishAllNames() {
  for (int i = 0; i < 16; i++) publishName(i);
}

void publishDiscoveryOne(int i) {
  if (!mqttClient.connected()) return;

  String idBase     = sanitizeId(mqttBaseTopic);
  String availTopic = mqttBaseTopic + "/availability";
  String stateTopic = mqttBaseTopic + "/relay/" + String(i) + "/state";
  String cmdTopic   = mqttBaseTopic + "/relay/" + String(i) + "/set";
  String cfgTopic   = "homeassistant/switch/" + idBase + "_relay" + String(i) + "/config";

  String name = relayNames[i].length() > 0 ? relayNames[i] : ("Relay " + String(i + 1));

  String payload = "{";
  payload += "\"name\":\"" + jsonEscape(name) + "\",";
  payload += "\"uniq_id\":\"" + idBase + "_relay" + String(i) + "\",";
  payload += "\"stat_t\":\"" + stateTopic + "\",";
  payload += "\"cmd_t\":\"" + cmdTopic + "\",";
  payload += "\"avty_t\":\"" + availTopic + "\",";
  payload += "\"pl_on\":\"ON\",\"pl_off\":\"OFF\",";
  payload += "\"stat_on\":\"ON\",\"stat_off\":\"OFF\",";
  payload += "\"dev\":{";
  payload +=   "\"ids\":[\"" + idBase + "\"],";
  payload +=   "\"name\":\"ESP32 16CH Relay\",";
  payload +=   "\"mdl\":\"ESP32-16CH\",";
  payload +=   "\"mf\":\"Custom\"";
  payload += "}";
  payload += "}";

  if (payload.length() > mqttClient.getBufferSize() - 5) {
    return;
  }
  mqttClient.publish(cfgTopic.c_str(), payload.c_str(), true);
}

void publishDiscovery() {
  for (int i = 0; i < 16; i++) publishDiscoveryOne(i);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t(topic);
  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];
  msg.trim();

  String prefix = mqttBaseTopic + "/relay/";
  if (!t.startsWith(prefix) || !t.endsWith("/set")) return;

  String mid = t.substring(prefix.length(), t.length() - 4);
  if (mid.indexOf('/') >= 0) return;
  int idx = mid.toInt();
  if (idx < 0 || idx >= 16) return;

  bool newState = relayState[idx];
  if (msg.equalsIgnoreCase("ON") || msg == "1" || msg.equalsIgnoreCase("true")) {
    newState = true;
  } else if (msg.equalsIgnoreCase("OFF") || msg == "0" || msg.equalsIgnoreCase("false")) {
    newState = false;
  } else if (msg.equalsIgnoreCase("TOGGLE")) {
    newState = !relayState[idx];
  } else {
    return;
  }

  relayState[idx] = newState;
  writeRelay(idx);
  prefsRelay.begin("relays", false);
  prefsRelay.putBool(String(idx).c_str(), newState);
  prefsRelay.end();
  pendingState[idx] = true;
}

void mqttConnect() {
  if (!mqttEnabled || mqttServer.length() == 0) return;
  if (mqttClient.connected()) return;

  unsigned long now = millis();
  if (now - lastMqttReconnect < mqttBackoff) return;
  lastMqttReconnect = now;

  mqttClient.setServer(mqttServer.c_str(), mqttPort);

  String availTopic = mqttBaseTopic + "/availability";
  const char* lwtPayload = "offline";

  bool ok;
  if (mqttUser.length() > 0) {
    ok = mqttClient.connect(mqttClientId.c_str(),
                            mqttUser.c_str(), mqttPass.c_str(),
                            availTopic.c_str(), 0, true, lwtPayload);
  } else {
    ok = mqttClient.connect(mqttClientId.c_str(),
                            availTopic.c_str(), 0, true, lwtPayload);
  }

  if (ok) {
    mqttBackoff = MQTT_BACKOFF_MIN_MS;

    mqttClient.publish(availTopic.c_str(), "online", true);

    String subTopic = mqttBaseTopic + "/relay/+/set";
    mqttClient.subscribe(subTopic.c_str());

    delay(50);
    mqttClient.publish(availTopic.c_str(), "online", true);

    publishAllStates();
    publishAllNames();
    publishDiscovery();
  } else {
    mqttBackoff = min(mqttBackoff * 2, MQTT_BACKOFF_MAX_MS);
  }
}

void flushPendingPublishes() {
  if (!mqttClient.connected()) return;
  if (pendingAll) {
    pendingAll = false;
    publishAllStates();
    for (int i = 0; i < 16; i++) pendingState[i] = false;
    return;
  }
  for (int i = 0; i < 16; i++) {
    if (pendingState[i]) {
      pendingState[i] = false;
      publishRelayState(i);
    }
  }
}

void mqttLoop() {
  if (!mqttEnabled || mqttServer.length() == 0) return;
  if (!mqttClient.connected()) {
    mqttConnect();
  } else {
    mqttClient.loop();
    flushPendingPublishes();
  }
}

static bool matchField(uint8_t field, int value) {
  return (field == ANY8) || (field == (uint8_t)value);
}

static bool matchStart(const Schedule& s, const struct tm& t) {
  if (!matchField(s.startSec,  t.tm_sec))  return false;
  if (!matchField(s.startMin,  t.tm_min))  return false;
  if (!matchField(s.startHour, t.tm_hour)) return false;
  if (!(s.startDowMask & (1UL << t.tm_wday))) return false;
  if (!(s.domMask     & (1UL << (t.tm_mday - 1)))) return false;
  if (!(s.monthMask   & (1U  << t.tm_mon))) return false;
  return true;
}

static bool matchStop(const Schedule& s, const struct tm& t) {
  if (!matchField(s.stopSec,  t.tm_sec))  return false;
  if (!matchField(s.stopMin,  t.tm_min))  return false;
  if (!matchField(s.stopHour, t.tm_hour)) return false;
  if (!(s.stopDowMask & (1UL << t.tm_wday))) return false;
  if (!(s.domMask     & (1UL << (t.tm_mday - 1)))) return false;
  if (!(s.monthMask   & (1U  << t.tm_mon))) return false;
  return true;
}

static void applyRelayMask(uint16_t mask, bool on) {
  for (int i = 0; i < 16; i++) {
    if (mask & (1U << i)) {
      if (relayState[i] != on) {
        relayState[i] = on;
        writeRelay(i);
        pendingState[i] = true;
      }
    }
  }
  prefsRelay.begin("relays", false);
  for (int i = 0; i < 16; i++) {
    if (mask & (1U << i)) prefsRelay.putBool(String(i).c_str(), relayState[i]);
  }
  prefsRelay.end();
}

void scheduleTick() {
  time_t now = time(nullptr);
  if (now < 1700000000) return;
  timeSynced = true;

  if (now <= lastSchedTick) return;
  lastSchedTick = now;

  struct tm t;
  localtime_r(&now, &t);

  for (int i = 0; i < SCHED_COUNT; i++) {
    Schedule& s = schedules[i];
    if (!s.enabled || s.relayMask == 0) continue;

    bool startHit = matchStart(s, t);
    bool stopHit  = matchStop(s, t);

    if (s.mode == MODE_NORMAL) {
      if (startHit && !s.matchedPrev) {
        applyRelayMask(s.relayMask, true);
      }
      static bool stopPrev[SCHED_COUNT] = {false};
      if (stopHit && !stopPrev[i]) {
        applyRelayMask(s.relayMask, false);
      }
      stopPrev[i] = stopHit;

    } else {
      int startMinOfDay = (s.startHour == ANY8 ? 0 : s.startHour) * 60
                        + (s.startMin  == ANY8 ? 0 : s.startMin);
      int stopMinOfDay  = (s.stopHour  == ANY8 ? 0 : s.stopHour)  * 60
                        + (s.stopMin   == ANY8 ? 0 : s.stopMin);
      int nowMinOfDay   = t.tm_hour * 60 + t.tm_min;

      bool inWindow;
      if (startMinOfDay == stopMinOfDay) inWindow = false;
      else if (startMinOfDay < stopMinOfDay)
        inWindow = (nowMinOfDay >= startMinOfDay && nowMinOfDay < stopMinOfDay);
      else
        inWindow = (nowMinOfDay >= startMinOfDay || nowMinOfDay < stopMinOfDay);

      if (inWindow && !s.matchedPrev) {
        applyRelayMask(s.relayMask, true);
        if (s.mode == MODE_OVERNIGHT_TAIL) {
          s.tailArmed   = false;
          s.tailArmedAt = 0;
        }
      } else if (!inWindow && s.matchedPrev) {
        if (s.mode == MODE_OVERNIGHT_TAIL && s.tailSeconds > 0) {
          s.tailArmed   = true;
          s.tailArmedAt = now;
        } else {
          applyRelayMask(s.relayMask, false);
        }
      }

      if (s.mode == MODE_OVERNIGHT_TAIL && s.tailArmed) {
        if (now - s.tailArmedAt >= (time_t)s.tailSeconds) {
          applyRelayMask(s.relayMask, false);
          s.tailArmed   = false;
          s.tailArmedAt = 0;
        }
      }

      s.matchedPrev = inWindow;
    }
  }
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Relay Control</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; margin: 0; padding: 20px; background: #e9ecef; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(140px, 1fr)); gap: 15px; max-width: 900px; margin: 0 auto; }
    .card { background: white; border-radius: 12px; padding: 15px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); display: flex; flex-direction: column; align-items: center; }
    .btn { padding: 15px 0; width: 100%; font-size: 18px; border: none; border-radius: 8px; cursor: pointer; color: white; transition: 0.2s; font-weight: bold; margin-top: 10px; }
    .btn-on { background: #28a745; box-shadow: 0 4px #1e7e34; }
    .btn-on:active { transform: translateY(4px); box-shadow: none; }
    .btn-off { background: #dc3545; box-shadow: 0 4px #bd2130; }
    .btn-off:active { transform: translateY(4px); box-shadow: none; }
    .name-box { font-size: 14px; color: #495057; cursor: pointer; padding: 5px 10px; border-radius: 4px; background: #f8f9fa; width: 100%; box-sizing: border-box; border: 1px solid transparent; transition: 0.2s; text-align: center; }
    .name-box:hover { border-color: #adb5bd; background: #e9ecef; }
    .name-input { font-size: 14px; padding: 5px 10px; border-radius: 4px; border: 1px solid #007bff; outline: none; width: 100%; box-sizing: border-box; text-align: center; font-family: inherit; }
    .status { margin-top: 20px; font-size: 14px; color: #6c757d; }
    .nav { margin-top: 15px; display: flex; gap: 10px; justify-content: center; flex-wrap: wrap; }
    .nav-btn { display: inline-block; padding: 10px 20px; color: white; text-decoration: none; border-radius: 6px; font-size: 14px; font-weight: bold; }
    .nav-btn.wifi  { background: #007bff; }
    .nav-btn.wifi:hover  { background: #0056b3; }
    .nav-btn.sta   { background: #17a2b8; }
    .nav-btn.sta:hover   { background: #117a8b; }
    .nav-btn.gpio  { background: #6f42c1; }
    .nav-btn.gpio:hover  { background: #59359c; }
    .nav-btn.mqtt  { background: #fd7e14; }
    .nav-btn.mqtt:hover  { background: #e36a0a; }
    .nav-btn.sched { background: #20c997; }
    .nav-btn.sched:hover { background: #17a589; }
    .nav-btn.time  { background: #e83e8c; }
    .nav-btn.time:hover  { background: #c8236f; }
    .info { margin-top: 15px; font-size: 13px; color: #495057; background: white; padding: 10px 15px; border-radius: 8px; max-width: 600px; margin-left: auto; margin-right: auto; box-shadow: 0 2px 4px rgba(0,0,0,0.06); text-align: left; }
    .info b { color: #212529; }
  </style>
</head>
<body>
  <div class="grid">
    %BUTTONS%
  </div>
  <div class="status">Device is running</div>
  <div class="nav">
    <a href="/wifi/ap"  class="nav-btn wifi">&#9881; WiFi AP</a>
    <a href="/wifi/sta" class="nav-btn sta">&#128225; WiFi STA</a>
    <a href="/gpio"     class="nav-btn gpio">&#9881; GPIO</a>
    <a href="/mqtt"     class="nav-btn mqtt">&#9881; MQTT</a>
    <a href="/schedule" class="nav-btn sched">&#128337; Schedule</a>
    <a href="/time"     class="nav-btn time">&#128340; Time</a>
  </div>
  <div class="info">%INFO%</div>

  <script>
    function editName(id, currentName) {
      var box = document.getElementById('name-' + id);
      var input = document.createElement('input');
      input.type = 'text';
      input.className = 'name-input';
      input.value = currentName;
      input.id = 'input-' + id;

      box.parentNode.replaceChild(input, box);
      input.focus();
      input.select();

      input.addEventListener('blur', function() {
        saveName(id, input.value);
      });
      input.addEventListener('keydown', function(e) {
        if (e.key === 'Enter') input.blur();
      });
    }

    function saveName(id, newName) {
      fetch('/rename?relay=' + id + '&name=' + encodeURIComponent(newName))
      .then(response => response.text())
      .then(data => { location.reload(); });
    }
  </script>
</body>
</html>
)rawliteral";

const char WIFI_AP_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WiFi AP Settings</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #e9ecef; }
    .panel { max-width: 480px; margin: 0 auto; padding: 22px; background: white; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    h3 { margin-top: 0; color: #343a40; text-align: center; }
    label { display: block; text-align: left; font-size: 13px; color: #495057; margin-bottom: 4px; margin-top: 10px; }
    input[type=text], input[type=password], input[type=number] {
      padding: 10px; width: 100%; border: 1px solid #ced4da; border-radius: 4px;
      font-size: 14px; box-sizing: border-box;
    }
    .row { display: flex; gap: 10px; }
    .row > div { flex: 1; }
    .chk { display: flex; align-items: center; gap: 8px; margin: 12px 0; font-size: 14px; }
    .chk input { width: auto; margin: 0; }
    button { padding: 12px 20px; border: none; border-radius: 4px; background: #007bff; color: white; cursor: pointer; font-size: 14px; font-weight: bold; width: 100%; margin-top: 15px; }
    button:hover { background: #0056b3; }
    a.back { display: block; text-align: center; margin-top: 15px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
    .note { font-size: 12px; color: #6c757d; margin-top: 12px; background: #f8f9fa; padding: 10px; border-radius: 6px; line-height: 1.5; }
    .warn { font-size: 12px; color: #856404; background: #fff3cd; padding: 10px; border-radius: 6px; margin-top: 12px; line-height: 1.5; }
  </style>
</head>
<body>
  <div class="panel">
    <h3>&#9881; WiFi Access Point</h3>
    %WARNING%
    <div class="chk">
      <input type="checkbox" id="ap_en" %AP_EN%>
      <label for="ap_en" style="margin:0;">Enable Access Point</label>
    </div>
    <label>AP SSID</label>
    <input type="text" id="ap_ssid" value="%AP_SSID%" maxlength="32">
    <label>AP Password (8–63 chars, blank for open)</label>
    <input type="password" id="ap_pass" value="%AP_PASS%" maxlength="63">
    <div class="row">
      <div>
        <label>Channel (1–13)</label>
        <input type="number" id="ap_chan" value="%AP_CHAN%" min="1" max="13">
      </div>
      <div>
        <label>Max Clients (1–8)</label>
        <input type="number" id="ap_maxc" value="%AP_MAXC%" min="1" max="8">
      </div>
    </div>
    <div class="chk">
      <input type="checkbox" id="ap_hidden" %AP_HIDDEN%>
      <label for="ap_hidden" style="margin:0;">Hide SSID</label>
    </div>
    <button onclick="saveAp()">Save &amp; Reboot</button>
    <div class="note">
      <b>Current AP:</b> %AP_INFO%<br>
      <b>Note:</b> When STA is connected, the AP radio follows the STA channel. The channel setting is ignored until STA disconnects.
    </div>
    <a class="back" href="/">&larr; Back to Control</a>
  </div>

  <script>
    function saveAp() {
      var en    = document.getElementById('ap_en').checked ? '1' : '0';
      var ssid  = document.getElementById('ap_ssid').value;
      var pass  = document.getElementById('ap_pass').value;
      var chan  = document.getElementById('ap_chan').value;
      var maxc  = document.getElementById('ap_maxc').value;
      var hid   = document.getElementById('ap_hidden').checked ? '1' : '0';

      if (en === '1') {
        if (ssid.trim().length === 0) { alert('AP SSID cannot be empty.'); return; }
        if (pass.length !== 0 && (pass.length < 8 || pass.length > 63)) {
          alert('AP password must be 8–63 characters, or blank for open.'); return;
        }
      }
      if (en === '0' && %STA_CONNECTED% !== 1) {
        alert('Cannot disable AP while STA is disconnected — device would be unreachable.');
        return;
      }

      var url = '/setap?en=' + en
              + '&ssid=' + encodeURIComponent(ssid)
              + '&pass=' + encodeURIComponent(pass)
              + '&chan=' + encodeURIComponent(chan)
              + '&maxc=' + encodeURIComponent(maxc)
              + '&hid='  + hid;
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          alert('AP settings saved. Rebooting...');
          setTimeout(function(){ window.location.href = '/'; }, 2000);
        } else {
          alert('Error: ' + t);
        }
      });
    }
  </script>
</body>
</html>
)rawliteral";

const char WIFI_STA_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WiFi Station Settings</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 0; padding: 20px; background: #e9ecef; }
    .panel { max-width: 480px; margin: 0 auto; padding: 22px; background: white; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    h3 { margin-top: 0; color: #343a40; text-align: center; }
    label { display: block; text-align: left; font-size: 13px; color: #495057; margin-bottom: 4px; margin-top: 10px; }
    input[type=text], input[type=password] {
      padding: 10px; width: 100%; border: 1px solid #ced4da; border-radius: 4px;
      font-size: 14px; box-sizing: border-box;
    }
    .row { display: flex; gap: 10px; }
    .row > div { flex: 1; }
    .chk { display: flex; align-items: center; gap: 8px; margin: 12px 0; font-size: 14px; }
    .chk input { width: auto; margin: 0; }
    button { padding: 12px 20px; border: none; border-radius: 4px; background: #17a2b8; color: white; cursor: pointer; font-size: 14px; font-weight: bold; width: 100%; margin-top: 15px; }
    button:hover { background: #117a8b; }
    a.back { display: block; text-align: center; margin-top: 15px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
    .note { font-size: 12px; color: #6c757d; margin-top: 12px; background: #f8f9fa; padding: 10px; border-radius: 6px; line-height: 1.5; }
    .stat { font-size: 13px; background: #e7f3ff; border-left: 4px solid #17a2b8; padding: 10px; border-radius: 4px; margin-bottom: 12px; line-height: 1.6; }
    .stat b { color: #0c5460; }
    select { padding: 10px; width: 100%; border: 1px solid #ced4da; border-radius: 4px; font-size: 14px; box-sizing: border-box; background: white; }
    .scan-btn { background: #6c757d; margin-top: 8px; padding: 8px; font-size: 13px; }
    .scan-btn:hover { background: #5a6268; }
  </style>
</head>
<body>
  <div class="panel">
    <h3>&#128225; WiFi Station</h3>
    <div class="stat">
      <b>Status:</b> %STA_STATUS%<br>
      <b>IP:</b> %STA_IP%<br>
      <b>RSSI:</b> %STA_RSSI%<br>
      <b>MAC:</b> %STA_MAC%
    </div>

    <div class="chk">
      <input type="checkbox" id="sta_en" %STA_EN%>
      <label for="sta_en" style="margin:0;">Enable Station (connect to a router)</label>
    </div>

    <label>Network SSID</label>
    <input type="text" id="sta_ssid" value="%STA_SSID%" maxlength="32">
    <button class="scan-btn" onclick="scanNetworks()">&#128269; Scan Networks</button>
    <div id="scan_results"></div>

    <label>Password (leave blank to keep current, 0 or 8–63 chars)</label>
    <input type="password" id="sta_pass" value="" maxlength="63" placeholder="%STA_PASS_PLACEHOLDER%">

    <label>Hostname</label>
    <input type="text" id="sta_host" value="%STA_HOST%" maxlength="32">

    <div class="chk">
      <input type="checkbox" id="sta_static" %STA_STATIC% onchange="toggleStatic()">
      <label for="sta_static" style="margin:0;">Use Static IP</label>
    </div>

    <div id="static_fields" style="display:%STATIC_DISPLAY%;">
      <label>IP Address</label>
      <input type="text" id="sta_ip" value="%STA_IP_VAL%" placeholder="192.168.1.50">
      <label>Gateway</label>
      <input type="text" id="sta_gw" value="%STA_GW%" placeholder="192.168.1.1">
      <label>Subnet Mask</label>
      <input type="text" id="sta_sn" value="%STA_SN%" placeholder="255.255.255.0">
      <label>DNS Server</label>
      <input type="text" id="sta_dns" value="%STA_DNS%" placeholder="192.168.1.1">
    </div>

    <button onclick="saveSta()">Save &amp; Reboot</button>
    <div class="note">
      <b>Note:</b> If you disable AP and STA fails to connect, the device will automatically
      re-enable the AP after 60 seconds so it stays reachable.
    </div>
    <a class="back" href="/">&larr; Back to Control</a>
  </div>

  <script>
    function toggleStatic() {
      var on = document.getElementById('sta_static').checked;
      document.getElementById('static_fields').style.display = on ? 'block' : 'none';
    }

    function scanNetworks() {
      var box = document.getElementById('scan_results');
      box.innerHTML = '<div style="font-size:13px;color:#6c757d;margin-top:6px;">Scanning…</div>';
      fetch('/wifi/scan').then(r => r.json()).then(list => {
        if (!list.length) { box.innerHTML = '<div style="font-size:13px;color:#6c757d;margin-top:6px;">No networks found.</div>'; return; }
        var html = '<select id="scan_select" onchange="pickSsid()"><option value="">-- pick a network --</option>';
        list.forEach(function(n) {
          html += '<option value="' + n.ssid.replace(/"/g,'&quot;') + '">' + n.ssid + ' (' + n.rssi + ' dBm)</option>';
        });
        html += '</select>';
        box.innerHTML = html;
      }).catch(e => {
        box.innerHTML = '<div style="font-size:13px;color:#dc3545;margin-top:6px;">Scan failed.</div>';
      });
    }

    function pickSsid() {
      var sel = document.getElementById('scan_select');
      if (sel.value) document.getElementById('sta_ssid').value = sel.value;
    }

    function saveSta() {
      var en     = document.getElementById('sta_en').checked ? '1' : '0';
      var ssid   = document.getElementById('sta_ssid').value;
      var pass   = document.getElementById('sta_pass').value;
      var host   = document.getElementById('sta_host').value;
      var staticOn = document.getElementById('sta_static').checked ? '1' : '0';
      var ip     = document.getElementById('sta_ip').value;
      var gw     = document.getElementById('sta_gw').value;
      var sn     = document.getElementById('sta_sn').value;
      var dns    = document.getElementById('sta_dns').value;

      if (en === '1' && ssid.trim().length === 0) {
        alert('STA SSID cannot be empty when STA is enabled.'); return;
      }
      if (pass.length !== 0 && (pass.length < 8 || pass.length > 63)) {
        alert('STA password must be 0 (open) or 8–63 characters.'); return;
      }

      var url = '/setsta?en=' + en
              + '&ssid=' + encodeURIComponent(ssid)
              + '&pass=' + encodeURIComponent(pass)
              + '&host=' + encodeURIComponent(host)
              + '&static=' + staticOn
              + '&ip='   + encodeURIComponent(ip)
              + '&gw='   + encodeURIComponent(gw)
              + '&sn='   + encodeURIComponent(sn)
              + '&dns='  + encodeURIComponent(dns);
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          alert('STA settings saved. Rebooting...');
          setTimeout(function(){ window.location.href = '/'; }, 2000);
        } else {
          alert('Error: ' + t);
        }
      });
    }
  </script>
</body>
</html>
)rawliteral";

const char GPIO_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>GPIO Settings</title>
  <style>
    body { font-family: Arial, sans-serif; padding: 20px; background: #e9ecef; margin: 0; }
    h2 { text-align: center; color: #343a40; }
    .msg { text-align: center; color: #28a745; min-height: 22px; font-size: 14px; font-weight: bold; margin-bottom: 10px; }
    .msg.error { color: #dc3545; }
    .warn { text-align: center; color: #856404; background: #fff3cd; padding: 10px; border-radius: 6px; max-width: 620px; margin: 0 auto 12px auto; font-size: 13px; }
    .row { display: grid; grid-template-columns: 90px 1fr 1fr 90px; gap: 10px;
           align-items: center; background: white; padding: 10px 15px; border-radius: 8px;
           margin-bottom: 8px; max-width: 620px; margin-left: auto; margin-right: auto;
           box-shadow: 0 2px 4px rgba(0,0,0,0.08); }
    .row b { font-size: 14px; color: #495057; }
    select { padding: 8px; border-radius: 4px; border: 1px solid #ced4da; font-size: 14px; background: white; width: 100%; box-sizing: border-box; }
    select option:disabled { color: #adb5bd; }
    button { padding: 8px 12px; border-radius: 4px; background: #6f42c1; color: white; border: none; cursor: pointer; font-size: 14px; font-weight: bold; }
    button:hover { background: #59359c; }
    a.back { display: block; text-align: center; margin-top: 20px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
  </style>
</head>
<body>
  <h2>&#9881; GPIO Configuration</h2>
  <div class="msg" id="msg"></div>
  %RTC_WARN%
  <div id="rows">%ROWS%</div>
  <a class="back" href="/">&larr; Back to Control</a>

  <script>
    function saveGpio(idx) {
      var pin = document.getElementById('pin-' + idx).value;
      var pol = document.getElementById('pol-' + idx).value;
      var msg = document.getElementById('msg');
      msg.className = 'msg';
      msg.textContent = 'Saving...';
      fetch('/setgpio?relay=' + idx + '&pin=' + pin + '&activeHigh=' + pol)
        .then(r => r.text())
        .then(t => {
          if (t === 'OK') {
            msg.className = 'msg';
            msg.textContent = 'Saved Relay ' + (idx + 1);
          } else {
            msg.className = 'msg error';
            msg.textContent = 'Error: ' + t;
          }
          setTimeout(function() { msg.textContent = ''; }, 3000);
        })
        .catch(e => {
          msg.className = 'msg error';
          msg.textContent = 'Network error';
        });
    }
  </script>
</body>
</html>
)rawliteral";

const char MQTT_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>MQTT Settings</title>
  <style>
    body { font-family: Arial, sans-serif; text-align: center; padding: 20px; background: #e9ecef; margin: 0; }
    .panel { max-width: 480px; margin: 0 auto; padding: 22px; background: white; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    h3 { margin-top: 0; color: #343a40; }
    label { display: block; text-align: left; font-size: 13px; color: #495057; margin-bottom: 4px; margin-top: 10px; }
    input[type=text], input[type=password], input[type=number] {
      padding: 10px; width: 100%; border: 1px solid #ced4da; border-radius: 4px;
      font-size: 14px; box-sizing: border-box;
    }
    .row { display: flex; gap: 10px; }
    .row > div { flex: 1; }
    .chk { display: flex; align-items: center; gap: 8px; margin: 12px 0; font-size: 14px; justify-content: flex-start; }
    .chk input { width: auto; margin: 0; }
    button { padding: 12px 20px; border: none; border-radius: 4px; background: #fd7e14; color: white; cursor: pointer; font-size: 14px; font-weight: bold; width: 100%; margin-top: 15px; }
    button:hover { background: #e36a0a; }
    a.back { display: block; text-align: center; margin-top: 15px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
    .note { font-size: 12px; color: #6c757d; margin-top: 12px; background: #f8f9fa; padding: 10px; border-radius: 6px; line-height: 1.5; text-align: left; }
    .warn { font-size: 12px; color: #856404; background: #fff3cd; padding: 10px; border-radius: 6px; margin-top: 12px; line-height: 1.5; text-align: left; }
  </style>
</head>
<body>
  <div class="panel">
    <h3>&#9881; MQTT Settings</h3>
    %MQTT_WARN%
    <div class="chk">
      <input type="checkbox" id="m_en" %MQTT_EN%>
      <label for="m_en" style="margin:0;">Enable MQTT</label>
    </div>
    <label>Broker Address</label>
    <input type="text" id="m_srv" value="%MQTT_SRV%" placeholder="192.168.1.10 or broker.local">
    <div class="row">
      <div>
        <label>Port (1883/1884)</label>
        <input type="number" id="m_port" value="%MQTT_PORT%" placeholder="1883">
      </div>
      <div>
        <label>Base Topic</label>
        <input type="text" id="m_base" value="%MQTT_BASE%" placeholder="esp32_16ch">
      </div>
    </div>
    <label>Username (optional)</label>
    <input type="text" id="m_usr" value="%MQTT_USR%" placeholder="">
    <label>Password (optional, leave blank to keep)</label>
    <input type="password" id="m_pw" value="" placeholder="%MQTT_PW_PLACEHOLDER%">
    <button onclick="saveMqtt()">Save &amp; Reboot</button>
    <div class="note">
      <b>Topics (base = the Base Topic above):</b><br>
      &bull; <code>&lt;base&gt;/availability</code> &mdash; online/offline (retained)<br>
      &bull; <code>&lt;base&gt;/status</code> &mdash; JSON of all relays<br>
      &bull; <code>&lt;base&gt;/relay/N/state</code> &mdash; ON/OFF (retained)<br>
      &bull; <code>&lt;base&gt;/relay/N/name</code> &mdash; relay name (retained)<br>
      &bull; <code>&lt;base&gt;/relay/N/set</code> &mdash; subscribe: ON/OFF/1/0/TOGGLE<br>
      <b>Home Assistant:</b> discovery messages are published automatically.
    </div>
    <a class="back" href="/">&larr; Back to Control</a>
  </div>

  <script>
    function saveMqtt() {
      var en   = document.getElementById('m_en').checked ? '1' : '0';
      var srv  = document.getElementById('m_srv').value.trim();
      var port = document.getElementById('m_port').value.trim() || '1883';
      var base = document.getElementById('m_base').value.trim() || 'esp32_16ch';
      var usr  = document.getElementById('m_usr').value;
      var pw   = document.getElementById('m_pw').value;

      if (en === '1' && srv === '') {
        alert('Broker address required when MQTT is enabled.'); return;
      }
      if (base.indexOf('+') >= 0 || base.indexOf('#') >= 0 || base.indexOf(' ') >= 0) {
        alert('Base topic cannot contain +, # or spaces.'); return;
      }
      if (port === '8883') {
        alert('TLS (8883) not supported. Use 1883 or 1884.'); return;
      }
      var url = '/setmqtt?en=' + en
              + '&srv='  + encodeURIComponent(srv)
              + '&port=' + encodeURIComponent(port)
              + '&base=' + encodeURIComponent(base)
              + '&usr='  + encodeURIComponent(usr)
              + '&pw='   + encodeURIComponent(pw);
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          alert('MQTT settings saved. Rebooting...');
          setTimeout(function(){ window.location.href = '/'; }, 2000);
        } else {
          alert('Error: ' + t);
        }
      });
    }
  </script>
</body>
</html>
)rawliteral";

const char SCHEDULE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Schedule Settings</title>
  <style>
    body { font-family: Arial, sans-serif; padding: 20px; background: #e9ecef; margin: 0; }
    h2 { text-align: center; color: #343a40; }
    .msg { text-align: center; color: #28a745; min-height: 22px; font-size: 14px; font-weight: bold; margin-bottom: 10px; }
    .msg.error { color: #dc3545; }
    .slot { background: white; padding: 15px; border-radius: 8px; margin-bottom: 12px;
            max-width: 900px; margin-left: auto; margin-right: auto;
            box-shadow: 0 2px 4px rgba(0,0,0,0.08); }
    .slot h4 { margin: 0 0 10px 0; color: #495057; font-size: 15px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(120px, 1fr)); gap: 8px; }
    label { display: block; font-size: 12px; color: #6c757d; margin-bottom: 2px; }
    input[type=text], input[type=number], select {
      padding: 6px; border-radius: 4px; border: 1px solid #ced4da;
      font-size: 13px; background: white; width: 100%; box-sizing: border-box;
    }
    .chk { display: flex; align-items: center; gap: 6px; font-size: 13px; margin: 6px 0; }
    .chk input { width: auto; margin: 0; }
    .relays { display: flex; flex-wrap: wrap; gap: 6px; margin: 6px 0; }
    .relays label { display: flex; align-items: center; gap: 3px; font-size: 12px; background: #f8f9fa; padding: 3px 6px; border-radius: 3px; cursor: pointer; }
    .relays input { width: auto; margin: 0; }
    .btnrow { display: flex; gap: 8px; margin-top: 10px; }
    .btnrow button { flex: 1; padding: 8px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; font-size: 13px; color: white; }
    .btn-save { background: #20c997; }
    .btn-save:hover { background: #17a589; }
    .btn-del { background: #dc3545; }
    .btn-del:hover { background: #bd2130; }
    .btn-test { background: #6c757d; }
    .btn-test:hover { background: #5a6268; }
    a.back { display: block; text-align: center; margin-top: 20px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
    .dow { display: flex; flex-wrap: wrap; gap: 4px; }
    .dow label { display: flex; align-items: center; gap: 3px; font-size: 11px; background: #f8f9fa; padding: 3px 5px; border-radius: 3px; cursor: pointer; }
    .dow input { width: auto; margin: 0; }
    .hint { font-size: 11px; color: #6c757d; margin-top: 6px; line-height: 1.4; }
  </style>
</head>
<body>
  <h2>&#128337; Schedule Configuration</h2>
  <div class="msg" id="msg"></div>
  <div id="slots">%SLOTS%</div>
  <a class="back" href="/">&larr; Back to Control</a>

  <script>
    var DOW_LABELS = ['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
    var MON_LABELS = ['Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'];

    function saveSlot(idx) {
      var en   = document.getElementById('en-' + idx).checked ? '1' : '0';
      var name = document.getElementById('name-' + idx).value;
      var mode = document.getElementById('mode-' + idx).value;
      var sH   = document.getElementById('sh-' + idx).value;
      var sM   = document.getElementById('sm-' + idx).value;
      var sS   = document.getElementById('ss-' + idx).value;
      var eH   = document.getElementById('eh-' + idx).value;
      var eM   = document.getElementById('em-' + idx).value;
      var eS   = document.getElementById('es-' + idx).value;
      var tail = document.getElementById('tail-' + idx).value;

      var relayMask = 0;
      for (var r = 0; r < 16; r++) {
        var cb = document.getElementById('r-' + idx + '-' + r);
        if (cb && cb.checked) relayMask |= (1 << r);
      }

      var sDow = 0, eDow = 0, dom = 0, mon = 0;
      for (var d = 0; d < 7; d++) {
        if (document.getElementById('sdow-' + idx + '-' + d).checked) sDow |= (1 << d);
        if (document.getElementById('edow-' + idx + '-' + d).checked) eDow |= (1 << d);
      }
      for (var dd = 0; dd < 31; dd++) {
        if (document.getElementById('dom-' + idx + '-' + dd).checked) dom |= (1 << dd);
      }
      for (var mm = 0; mm < 12; mm++) {
        if (document.getElementById('mon-' + idx + '-' + mm).checked) mon |= (1 << mm);
      }

      var url = '/schedset?idx=' + idx
              + '&en=' + en
              + '&name=' + encodeURIComponent(name)
              + '&mode=' + mode
              + '&mask=' + relayMask
              + '&sh=' + sH + '&sm=' + sM + '&ss=' + sS
              + '&eh=' + eH + '&em=' + eM + '&es=' + eS
              + '&sdow=' + sDow + '&edow=' + eDow
              + '&dom=' + dom + '&mon=' + mon
              + '&tail=' + tail;
      var msg = document.getElementById('msg');
      msg.className = 'msg';
      msg.textContent = 'Saving...';
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          msg.className = 'msg';
          msg.textContent = 'Saved slot ' + (idx + 1);
        } else {
          msg.className = 'msg error';
          msg.textContent = 'Error: ' + t;
        }
        setTimeout(function() { msg.textContent = ''; }, 3000);
      }).catch(e => {
        msg.className = 'msg error';
        msg.textContent = 'Network error';
      });
    }

    function delSlot(idx) {
      if (!confirm('Clear slot ' + (idx + 1) + '?')) return;
      fetch('/scheddel?idx=' + idx).then(r => r.text()).then(t => {
        if (t === 'OK') location.reload();
      });
    }

    function testSlot(idx) {
      fetch('/schedtest?idx=' + idx).then(r => r.text()).then(t => {
        var msg = document.getElementById('msg');
        msg.className = 'msg';
        msg.textContent = t === 'OK' ? 'Test fired slot ' + (idx + 1) : 'Test error: ' + t;
        setTimeout(function() { msg.textContent = ''; }, 3000);
      });
    }
  </script>
</body>
</html>
)rawliteral";

const char TIME_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Time Settings</title>
  <style>
    body { font-family: Arial, sans-serif; padding: 20px; background: #e9ecef; margin: 0; }
    .panel { max-width: 520px; margin: 0 auto; padding: 22px; background: white; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.1); }
    h3 { margin-top: 0; color: #343a40; text-align: center; }
    label { display: block; text-align: left; font-size: 13px; color: #495057; margin-bottom: 4px; margin-top: 10px; }
    input[type=text], input[type=number], select {
      padding: 10px; width: 100%; border: 1px solid #ced4da; border-radius: 4px;
      font-size: 14px; box-sizing: border-box;
    }
    .row { display: flex; gap: 10px; }
    .row > div { flex: 1; }
    .chk { display: flex; align-items: center; gap: 8px; margin: 12px 0; font-size: 14px; }
    .chk input { width: auto; margin: 0; }
    button { padding: 12px 20px; border: none; border-radius: 4px; background: #e83e8c; color: white; cursor: pointer; font-size: 14px; font-weight: bold; width: 100%; margin-top: 15px; }
    button:hover { background: #c8236f; }
    .btn-sec { background: #6c757d; margin-top: 8px; }
    .btn-sec:hover { background: #5a6268; }
    a.back { display: block; text-align: center; margin-top: 15px; color: #007bff; text-decoration: none; font-size: 14px; }
    a.back:hover { text-decoration: underline; }
    .stat { font-size: 13px; background: #fde8f2; border-left: 4px solid #e83e8c; padding: 10px; border-radius: 4px; margin-bottom: 12px; line-height: 1.6; }
    .stat b { color: #8b1a52; }
    .note { font-size: 12px; color: #6c757d; margin-top: 12px; background: #f8f9fa; padding: 10px; border-radius: 6px; line-height: 1.5; }
    .warn { font-size: 12px; color: #856404; background: #fff3cd; padding: 10px; border-radius: 6px; margin-top: 12px; line-height: 1.5; }
    .msg { text-align: center; color: #28a745; min-height: 22px; font-size: 13px; font-weight: bold; margin-bottom: 8px; }
    .msg.error { color: #dc3545; }
  </style>
</head>
<body>
  <div class="panel">
    <h3>&#128340; Time Settings</h3>
    <div class="msg" id="msg"></div>
    <div class="stat">
      <b>Current Time:</b> %CUR_TIME%<br>
      <b>Time Source:</b> %SOURCE%<br>
      <b>NTP:</b> %NTP_STATUS%<br>
      <b>DS3231:</b> %RTC_STATUS%<br>
      <b>RTC Time:</b> %RTC_TIME%
    </div>
    %RTC_WARN%

    <label>GMT Offset</label>
    <select id="tzsel" onchange="onTzChange()">
      <option value="-720">UTC−12:00</option>
      <option value="-660">UTC−11:00</option>
      <option value="-600">UTC−10:00</option>
      <option value="-540">UTC−09:00</option>
      <option value="-480">UTC−08:00</option>
      <option value="-420">UTC−07:00</option>
      <option value="-360">UTC−06:00</option>
      <option value="-300">UTC−05:00</option>
      <option value="-240">UTC−04:00</option>
      <option value="-210">UTC−03:30</option>
      <option value="-180">UTC−03:00</option>
      <option value="-120">UTC−02:00</option>
      <option value="-60">UTC−01:00</option>
      <option value="0">UTC±00:00</option>
      <option value="60">UTC+01:00</option>
      <option value="120">UTC+02:00</option>
      <option value="180">UTC+03:00</option>
      <option value="210">UTC+03:30</option>
      <option value="240">UTC+04:00</option>
      <option value="270">UTC+04:30</option>
      <option value="300">UTC+05:00</option>
      <option value="330">UTC+05:30</option>
      <option value="345">UTC+05:45</option>
      <option value="360">UTC+06:00</option>
      <option value="390">UTC+06:30</option>
      <option value="420">UTC+07:00</option>
      <option value="480">UTC+08:00</option>
      <option value="525">UTC+08:45</option>
      <option value="540">UTC+09:00</option>
      <option value="570">UTC+09:30</option>
      <option value="600">UTC+10:00</option>
      <option value="630">UTC+10:30</option>
      <option value="660">UTC+11:00</option>
      <option value="720">UTC+12:00</option>
      <option value="765">UTC+12:45</option>
      <option value="780">UTC+13:00</option>
      <option value="840">UTC+14:00</option>
    </select>
    <label>Custom Offset (minutes, optional override)</label>
    <input type="number" id="tzcustom" value="%TZ_VAL%" min="-720" max="840">

    <label>NTP Server 1</label>
    <input type="text" id="ntp1" value="%NTP1%" maxlength="63">
    <label>NTP Server 2</label>
    <input type="text" id="ntp2" value="%NTP2%" maxlength="63">

    <div class="chk">
      <input type="checkbox" id="rtc_en" %RTC_EN%>
      <label for="rtc_en" style="margin:0;">Enable DS3231 RTC</label>
    </div>
    <div class="note">
      DS3231 uses GPIO 21 (SDA) and GPIO 22 (SCL). Relays cannot use these pins while RTC is enabled.
    </div>

    <button onclick="saveTime()">Save Time Settings</button>
    <button class="btn-sec" onclick="ntpNow()">Sync NTP Now</button>

    <label style="margin-top:20px;">Manual Date &amp; Time (for offline first-boot)</label>
    <div class="row">
      <div><label>Year</label><input type="number" id="m_y" value="%M_Y%" min="2024" max="2099"></div>
      <div><label>Month</label><input type="number" id="m_mo" value="%M_MO%" min="1" max="12"></div>
      <div><label>Day</label><input type="number" id="m_d" value="%M_D%" min="1" max="31"></div>
    </div>
    <div class="row">
      <div><label>Hour</label><input type="number" id="m_h" value="%M_H%" min="0" max="23"></div>
      <div><label>Minute</label><input type="number" id="m_mi" value="%M_MI%" min="0" max="59"></div>
      <div><label>Second</label><input type="number" id="m_s" value="%M_S%" min="0" max="59"></div>
    </div>
    <button class="btn-sec" onclick="setManual()">Set System &amp; RTC Time</button>

    <a class="back" href="/">&larr; Back to Control</a>
  </div>

  <script>
    function onTzChange() {
      var v = document.getElementById('tzsel').value;
      document.getElementById('tzcustom').value = v;
    }

    function saveTime() {
      var tz   = document.getElementById('tzcustom').value;
      var ntp1 = document.getElementById('ntp1').value.trim();
      var ntp2 = document.getElementById('ntp2').value.trim();
      var rtc  = document.getElementById('rtc_en').checked ? '1' : '0';
      if (ntp1 === '') { alert('NTP server 1 required.'); return; }
      var url = '/settime?tz=' + encodeURIComponent(tz)
              + '&ntp1=' + encodeURIComponent(ntp1)
              + '&ntp2=' + encodeURIComponent(ntp2)
              + '&rtc=' + rtc;
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          alert('Saved. Rebooting...');
          setTimeout(function(){ location.reload(); }, 2000);
        } else {
          alert('Error: ' + t);
        }
      });
    }

    function ntpNow() {
      var msg = document.getElementById('msg');
      msg.className = 'msg';
      msg.textContent = 'Syncing NTP...';
      fetch('/ntpnow').then(r => r.text()).then(t => {
        if (t === 'OK') {
          msg.textContent = 'NTP synced.';
        } else {
          msg.className = 'msg error';
          msg.textContent = 'NTP failed: ' + t;
        }
        setTimeout(function(){ msg.textContent = ''; location.reload(); }, 2000);
      });
    }

    function setManual() {
      var y  = document.getElementById('m_y').value;
      var mo = document.getElementById('m_mo').value;
      var d  = document.getElementById('m_d').value;
      var h  = document.getElementById('m_h').value;
      var mi = document.getElementById('m_mi').value;
      var s  = document.getElementById('m_s').value;
      var msg = document.getElementById('msg');
      msg.className = 'msg';
      msg.textContent = 'Setting...';
      var url = '/rtcset?y=' + y + '&mo=' + mo + '&d=' + d
              + '&h=' + h + '&mi=' + mi + '&s=' + s;
      fetch(url).then(r => r.text()).then(t => {
        if (t === 'OK') {
          msg.textContent = 'Time set.';
        } else {
          msg.className = 'msg error';
          msg.textContent = 'Error: ' + t;
        }
        setTimeout(function(){ msg.textContent = ''; location.reload(); }, 2000);
      });
    }

    window.addEventListener('load', function(){
      var cur = document.getElementById('tzcustom').value;
      var sel = document.getElementById('tzsel');
      for (var i = 0; i < sel.options.length; i++) {
        if (sel.options[i].value === cur) { sel.selectedIndex = i; break; }
      }
    });
  </script>
</body>
</html>
)rawliteral";

String getMainHTML() {
  String html = String(FPSTR(INDEX_HTML));
  String buttonHTML = "";

  for (int i = 0; i < 16; i++) {
    String btnClass = relayState[i] ? "btn btn-on" : "btn btn-off";
    String label    = relayState[i] ? "ON" : "OFF";
    String name     = relayNames[i];

    buttonHTML += "<div class='card'>";
    buttonHTML += "<div id='name-" + String(i) + "' class='name-box' onclick=\"editName(" + String(i) + ", '" + name + "')\">" + name + "</div>";
    buttonHTML += "<button class='" + btnClass + "' onclick=\"fetch('/toggle?relay=" + String(i) + "').then(() => location.reload())\">" + label + "</button>";
    buttonHTML += "</div>";
  }

  String info = "";
  info += "<b>AP:</b> " + String(apEnabled ? "ON" : "OFF") + " — ";
  if (apEnabled) info += "http://" + WiFi.softAPIP().toString();
  else           info += "(disabled)";
  info += "<br><b>STA:</b> ";
  if (staEnabled) {
    info += staStatusString();
    if (WiFi.status() == WL_CONNECTED) {
      info += " — http://" + WiFi.localIP().toString();
    }
  } else {
    info += "disabled";
  }
  info += "<br><b>MQTT:</b> ";
  if (mqttEnabled) {
    info += mqttClient.connected() ? "connected" : "disconnected";
    info += " — " + mqttServer + ":" + String(mqttPort);
  } else {
    info += "disabled";
  }
  info += "<br><b>Time:</b> " + formatLocalTime();
  info += "<br><b>Source:</b> ";
  if (ntpSynced) info += "NTP";
  else if (rtcEnabled && rtcPresent) info += "DS3231";
  else info += "none";
  info += "<br><b>GMT Offset:</b> " + String(tzOffsetMinutes) + " min";

  html.replace("%BUTTONS%", buttonHTML);
  html.replace("%INFO%", info);
  return html;
}

String getWifiApHTML() {
  String html = String(FPSTR(WIFI_AP_HTML));

  String warning = "";
  if (staEnabled && WiFi.status() != WL_CONNECTED) {
    warning = "<div class='warn'><b>STA is not connected.</b> AP cannot be disabled until STA connects, otherwise the device becomes unreachable.</div>";
  }
  html.replace("%WARNING%", warning);

  html.replace("%AP_EN%", apEnabled ? "checked" : "");
  html.replace("%AP_SSID%", apSSID);
  html.replace("%AP_PASS%", apPassword);
  html.replace("%AP_CHAN%", String(apChannel));
  html.replace("%AP_MAXC%", String(apMaxClients));
  html.replace("%AP_HIDDEN%", apHidden ? "checked" : "");
  html.replace("%STA_CONNECTED%", (WiFi.status() == WL_CONNECTED) ? "1" : "0");

  String info = "";
  if (apEnabled) {
    info += "SSID <b>" + apSSID + "</b> on http://" + WiFi.softAPIP().toString();
    info += " — channel " + String(WiFi.channel());
  } else {
    info += "disabled";
  }
  html.replace("%AP_INFO%", info);
  return html;
}

String getWifiStaHTML() {
  String html = String(FPSTR(WIFI_STA_HTML));

  html.replace("%STA_EN%", staEnabled ? "checked" : "");
  html.replace("%STA_SSID%", staSSID);
  html.replace("%STA_HOST%", staHostname);
  html.replace("%STA_STATIC%", staStatic ? "checked" : "");
  html.replace("%STATIC_DISPLAY%", staStatic ? "block" : "none");
  html.replace("%STA_IP_VAL%", staStatic ? staIP.toString()  : "");
  html.replace("%STA_GW%",     staStatic ? staGW.toString()  : "");
  html.replace("%STA_SN%",     staStatic ? staSN.toString()  : "");
  html.replace("%STA_DNS%",    staStatic ? staDNS.toString() : "");
  html.replace("%STA_PASS_PLACEHOLDER%", staPassword.length() ? "••••••••" : "(blank = open)");

  String status = staEnabled ? staStatusString() : "Disabled";
  html.replace("%STA_STATUS%", status);

  String ip = "—";
  if (staEnabled && WiFi.status() == WL_CONNECTED) ip = WiFi.localIP().toString();
  html.replace("%STA_IP%", ip);

  String rssi = "—";
  if (staEnabled && WiFi.status() == WL_CONNECTED) rssi = String(WiFi.RSSI()) + " dBm";
  html.replace("%STA_RSSI%", rssi);

  String mac = WiFi.macAddress();
  html.replace("%STA_MAC%", mac);
  return html;
}

String getGpioHTML() {
  String html = String(FPSTR(GPIO_HTML));
  String rows = "";

  String rtcWarn = "";
  if (rtcEnabled) {
    rtcWarn = "<div class='warn'>DS3231 RTC is enabled: GPIO 21 and 22 are reserved. Relays cannot use these pins.</div>";
  }
  html.replace("%RTC_WARN%", rtcWarn);

  for (int i = 0; i < 16; i++) {
    rows += "<div class='row'>";
    rows += "<b>Relay " + String(i + 1) + "</b>";

    rows += "<select id='pin-" + String(i) + "'>";
    for (int j = 0; j < VALID_PINS_COUNT; j++) {
      int p = VALID_PINS[j];
      bool reserved = rtcEnabled && (p == RTC_SDA_PIN || p == RTC_SCL_PIN);
      rows += "<option value='" + String(p) + "'";
      if (p == relayPins[i]) rows += " selected";
      if (reserved) rows += " disabled";
      rows += ">GPIO " + String(p);
      if (reserved) rows += " (RTC)";
      rows += "</option>";
    }
    rows += "</select>";

    rows += "<select id='pol-" + String(i) + "'>";
    rows += "<option value='0'";
    if (!relayActiveHigh[i]) rows += " selected";
    rows += ">Active LOW</option>";
    rows += "<option value='1'";
    if (relayActiveHigh[i]) rows += " selected";
    rows += ">Active HIGH</option>";
    rows += "</select>";

    rows += "<button onclick='saveGpio(" + String(i) + ")'>Save</button>";
    rows += "</div>";
  }

  html.replace("%ROWS%", rows);
  return html;
}

String getMqttHTML() {
  String html = String(FPSTR(MQTT_HTML));

  html.replace("%MQTT_EN%", mqttEnabled ? "checked" : "");
  html.replace("%MQTT_SRV%", mqttServer);
  html.replace("%MQTT_PORT%", String(mqttPort));
  html.replace("%MQTT_BASE%", mqttBaseTopic);
  html.replace("%MQTT_USR%", mqttUser);
  html.replace("%MQTT_PW_PLACEHOLDER%", mqttPass.length() ? "••••••••" : "");

  String warn = "";
  if (mqttEnabled && (!staEnabled || WiFi.status() != WL_CONNECTED)) {
    warn = "<div class='warn'><b>MQTT requires a working STA connection.</b> Currently STA is ";
    warn += !staEnabled ? "disabled." : "not connected.";
    warn += " MQTT will not be able to reach the broker until STA connects.</div>";
  }
  html.replace("%MQTT_WARN%", warn);
  return html;
}

String getScheduleHTML() {
  String html = String(FPSTR(SCHEDULE_HTML));
  String slots = "";

  const char* DOW_LABELS[7] = {"Sun","Mon","Tue","Wed","Thu","Fri","Sat"};
  const char* MON_LABELS[12] = {"Jan","Feb","Mar","Apr","May","Jun",
                                "Jul","Aug","Sep","Oct","Nov","Dec"};

  for (int i = 0; i < SCHED_COUNT; i++) {
    Schedule& s = schedules[i];
    slots += "<div class='slot'>";
    slots += "<h4>Slot " + String(i + 1) + "</h4>";

    slots += "<div class='grid'>";
    slots += "<div><label>Enabled</label><input type='checkbox' id='en-" + String(i) + "'";
    if (s.enabled) slots += " checked";
    slots += "></div>";
    slots += "<div><label>Name</label><input type='text' id='name-" + String(i) + "' value='";
    slots += jsonEscape(String(s.name));
    slots += "' maxlength='23'></div>";
    slots += "<div><label>Mode</label><select id='mode-" + String(i) + "'>";
    slots += "<option value='0'";
    if (s.mode == MODE_NORMAL) slots += " selected";
    slots += ">Normal</option>";
    slots += "<option value='1'";
    if (s.mode == MODE_OVERNIGHT) slots += " selected";
    slots += ">Overnight</option>";
    slots += "<option value='2'";
    if (s.mode == MODE_OVERNIGHT_TAIL) slots += " selected";
    slots += ">Overnight + Tail</option>";
    slots += "</select></div>";
    slots += "</div>";

    slots += "<div class='grid'>";
    slots += "<div><label>Start H</label><input type='number' id='sh-" + String(i) + "' min='0' max='23' value='" + (s.startHour == ANY8 ? "" : String(s.startHour)) + "' placeholder='*'></div>";
    slots += "<div><label>Start M</label><input type='number' id='sm-" + String(i) + "' min='0' max='59' value='" + (s.startMin  == ANY8 ? "" : String(s.startMin))  + "' placeholder='*'></div>";
    slots += "<div><label>Start S</label><input type='number' id='ss-" + String(i) + "' min='0' max='59' value='" + (s.startSec  == ANY8 ? "" : String(s.startSec))  + "' placeholder='*'></div>";
    slots += "<div><label>Stop H</label><input type='number' id='eh-" + String(i) + "' min='0' max='23' value='" + (s.stopHour  == ANY8 ? "" : String(s.stopHour))  + "' placeholder='*'></div>";
    slots += "<div><label>Stop M</label><input type='number' id='em-" + String(i) + "' min='0' max='59' value='" + (s.stopMin   == ANY8 ? "" : String(s.stopMin))   + "' placeholder='*'></div>";
    slots += "<div><label>Stop S</label><input type='number' id='es-" + String(i) + "' min='0' max='59' value='" + (s.stopSec   == ANY8 ? "" : String(s.stopSec))   + "' placeholder='*'></div>";
    slots += "<div><label>Tail (sec)</label><input type='number' id='tail-" + String(i) + "' min='0' max='86400' value='" + String(s.tailSeconds) + "'></div>";
    slots += "</div>";

    slots += "<label style='margin-top:8px;'>Relays</label><div class='relays'>";
    for (int r = 0; r < 16; r++) {
      slots += "<label><input type='checkbox' id='r-" + String(i) + "-" + String(r) + "'";
      if (s.relayMask & (1U << r)) slots += " checked";
      slots += ">" + String(r + 1) + "</label>";
    }
    slots += "</div>";

    slots += "<label style='margin-top:8px;'>Start Days-of-Week</label><div class='dow'>";
    for (int d = 0; d < 7; d++) {
      slots += "<label><input type='checkbox' id='sdow-" + String(i) + "-" + String(d) + "'";
      if (s.startDowMask & (1UL << d)) slots += " checked";
      slots += ">" + String(DOW_LABELS[d]) + "</label>";
    }
    slots += "</div>";

    slots += "<label style='margin-top:8px;'>Stop Days-of-Week</label><div class='dow'>";
    for (int d = 0; d < 7; d++) {
      slots += "<label><input type='checkbox' id='edow-" + String(i) + "-" + String(d) + "'";
      if (s.stopDowMask & (1UL << d)) slots += " checked";
      slots += ">" + String(DOW_LABELS[d]) + "</label>";
    }
    slots += "</div>";

    slots += "<label style='margin-top:8px;'>Days of Month (none checked = any)</label><div class='dow'>";
    for (int dd = 0; dd < 31; dd++) {
      slots += "<label><input type='checkbox' id='dom-" + String(i) + "-" + String(dd) + "'";
      if (s.domMask & (1UL << dd)) slots += " checked";
      slots += ">" + String(dd + 1) + "</label>";
    }
    slots += "</div>";

    slots += "<label style='margin-top:8px;'>Months (none checked = any)</label><div class='dow'>";
    for (int mm = 0; mm < 12; mm++) {
      slots += "<label><input type='checkbox' id='mon-" + String(i) + "-" + String(mm) + "'";
      if (s.monthMask & (1U << mm)) slots += " checked";
      slots += ">" + String(MON_LABELS[mm]) + "</label>";
    }
    slots += "</div>";

    slots += "<div class='btnrow'>";
    slots += "<button class='btn-save' onclick='saveSlot(" + String(i) + ")'>Save</button>";
    slots += "<button class='btn-test' onclick='testSlot(" + String(i) + ")'>Test</button>";
    slots += "<button class='btn-del'  onclick='delSlot(" + String(i) + ")'>Clear</button>";
    slots += "</div>";

    slots += "<div class='hint'>Leave a time field blank to mean &quot;any&quot;. For overnight, set Stop earlier than Start (e.g. Start 22:00, Stop 06:00). Tail only applies to Overnight + Tail mode.</div>";

    slots += "</div>";
  }

  html.replace("%SLOTS%", slots);
  return html;
}

String getTimeHTML() {
  String html = String(FPSTR(TIME_HTML));

  html.replace("%CUR_TIME%", formatLocalTime());

  String src = "none";
  if (ntpSynced) src = "NTP";
  else if (rtcEnabled && rtcPresent) src = "DS3231";
  html.replace("%SOURCE%", src);

  html.replace("%NTP_STATUS%", ntpSynced ? "synced" : "not synced");

  String rtcStatus = "disabled";
  if (rtcEnabled) {
    if (!i2cPinsFree()) rtcStatus = "conflict with relay pins";
    else if (rtcPresent) rtcStatus = "present";
    else rtcStatus = "not detected";
  }
  html.replace("%RTC_STATUS%", rtcStatus);

  String rtcTime = "—";
  if (rtcEnabled && rtcPresent) {
    DateTime n = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             n.year(), n.month(), n.day(), n.hour(), n.minute(), n.second());
    rtcTime = String(buf);
  }
  html.replace("%RTC_TIME%", rtcTime);

  String rtcWarn = "";
  if (rtcEnabled && !i2cPinsFree()) {
    rtcWarn = "<div class='warn'>DS3231 is enabled but GPIO 21/22 are assigned to relays. Reassign those relays or disable RTC.</div>";
  } else if (rtcEnabled && !rtcPresent) {
    rtcWarn = "<div class='warn'>DS3231 enabled but not detected on I2C. Check wiring (SDA=21, SCL=22) and power.</div>";
  } else if (rtcEnabled && rtcPresent && !rtcValidAtBoot) {
    rtcWarn = "<div class='warn'>DS3231 detected but its time is invalid (power loss). Use Manual Date &amp; Time below to set it.</div>";
  }
  html.replace("%RTC_WARN%", rtcWarn);

  html.replace("%TZ_VAL%", String(tzOffsetMinutes));
  html.replace("%NTP1%", ntpServer1);
  html.replace("%NTP2%", ntpServer2);
  html.replace("%RTC_EN%", rtcEnabled ? "checked" : "");

  time_t now = time(nullptr);
  if (now > 1700000000) {
    struct tm t;
    localtime_r(&now, &t);
    html.replace("%M_Y%",  String(t.tm_year + 1900));
    html.replace("%M_MO%", String(t.tm_mon + 1));
    html.replace("%M_D%",  String(t.tm_mday));
    html.replace("%M_H%",  String(t.tm_hour));
    html.replace("%M_MI%", String(t.tm_min));
    html.replace("%M_S%",  String(t.tm_sec));
  } else {
    html.replace("%M_Y%",  "2024");
    html.replace("%M_MO%", "1");
    html.replace("%M_D%",  "1");
    html.replace("%M_H%",  "0");
    html.replace("%M_MI%", "0");
    html.replace("%M_S%",  "0");
  }

  return html;
}

void handleRoot() {
  server.send(200, "text/html", getMainHTML());
}

void handleToggle() {
  if (server.hasArg("relay")) {
    int relayIndex = server.arg("relay").toInt();
    if (relayIndex >= 0 && relayIndex < 16) {
      relayState[relayIndex] = !relayState[relayIndex];
      writeRelay(relayIndex);

      prefsRelay.begin("relays", false);
      prefsRelay.putBool(String(relayIndex).c_str(), relayState[relayIndex]);
      prefsRelay.end();

      pendingState[relayIndex] = true;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad Request");
}

void handleRename() {
  if (server.hasArg("relay") && server.hasArg("name")) {
    int relayIndex = server.arg("relay").toInt();
    String newName = server.arg("name");
    newName.trim();
    if (newName.length() == 0) newName = "Relay " + String(relayIndex + 1);

    if (relayIndex >= 0 && relayIndex < 16) {
      relayNames[relayIndex] = newName;

      prefsRelay.begin("relays", false);
      prefsRelay.putString(("n" + String(relayIndex)).c_str(), newName);
      prefsRelay.end();

      if (mqttClient.connected()) {
        publishName(relayIndex);
        publishDiscoveryOne(relayIndex);
      }
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "Bad Request");
}

void handleWifiApPage() {
  server.send(200, "text/html", getWifiApHTML());
}

void handleWifiStaPage() {
  server.send(200, "text/html", getWifiStaHTML());
}

void handleWifiScan() {
  if (scanInProgress) {
    server.send(429, "text/plain", "Scan in progress");
    return;
  }
  scanInProgress = true;

  int n = WiFi.scanNetworks(false, false);

  String json = "[";
  for (int i = 0; i < n; i++) {
    if (i) json += ",";
    json += "{\"ssid\":\"" + jsonEscape(WiFi.SSID(i)) + "\",";
    json += "\"rssi\":" + String(WiFi.RSSI(i)) + ",";
    json += "\"enc\":" + String(WiFi.encryptionType(i)) + "}";
  }
  json += "]";

  WiFi.scanDelete();
  scanInProgress = false;

  server.send(200, "application/json", json);
}

void handleSetAp() {
  if (!server.hasArg("en") || !server.hasArg("ssid")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  bool en     = server.arg("en") == "1";
  String ssid = server.arg("ssid"); ssid.trim();
  String pass = server.arg("pass");
  int chan    = server.arg("chan").toInt();
  int maxc    = server.arg("maxc").toInt();
  bool hid    = server.arg("hid") == "1";

  if (ssid.length() == 0 || ssid.length() > 32) {
    server.send(400, "text/plain", "SSID must be 1–32 chars"); return;
  }
  if (pass.length() != 0 && (pass.length() < 8 || pass.length() > 63)) {
    server.send(400, "text/plain", "Password must be 0 or 8–63 chars"); return;
  }
  if (chan < 1 || chan > 13) chan = 1;
  if (maxc < 1 || maxc > 8)  maxc = 4;

  if (!en && WiFi.status() != WL_CONNECTED) {
    server.send(400, "text/plain", "Cannot disable AP while STA is disconnected");
    return;
  }

  prefsAP.begin("wifi_ap", false);
  prefsAP.putBool  ("en",     en);
  prefsAP.putString("ssid",   ssid);
  prefsAP.putString("pass",   pass);
  prefsAP.putInt   ("chan",   chan);
  prefsAP.putInt   ("maxc",   maxc);
  prefsAP.putBool  ("hidden", hid);
  prefsAP.end();

  server.send(200, "text/plain", "OK");
  delay(1000);
  ESP.restart();
}

void handleSetSta() {
  if (!server.hasArg("en") || !server.hasArg("ssid")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  bool en     = server.arg("en") == "1";
  String ssid = server.arg("ssid"); ssid.trim();
  String pass = server.arg("pass");
  String host = server.arg("host"); host.trim();
  bool staticOn = server.hasArg("static") && server.arg("static") == "1";
  String ip   = server.arg("ip");
  String gw   = server.arg("gw");
  String sn   = server.arg("sn");
  String dns  = server.arg("dns");

  if (en) {
    if (ssid.length() == 0 || ssid.length() > 32) {
      server.send(400, "text/plain", "SSID must be 1–32 chars"); return;
    }
    if (pass.length() != 0 && (pass.length() < 8 || pass.length() > 63)) {
      server.send(400, "text/plain", "Password must be 0 or 8–63 chars"); return;
    }
    if (staticOn) {
      IPAddress tIP, tGW, tSN, tDNS;
      if (!parseIP(ip,  tIP))  { server.send(400, "text/plain", "Invalid static IP");  return; }
      if (!parseIP(gw,  tGW))  { server.send(400, "text/plain", "Invalid gateway");    return; }
      if (!parseIP(sn,  tSN))  { server.send(400, "text/plain", "Invalid subnet");     return; }
      if (!parseIP(dns, tDNS)) { server.send(400, "text/plain", "Invalid DNS");        return; }
    }
  }

  if (host.length() == 0) host = DEFAULT_STA_HOST;
  if (host.length() > 32) host = host.substring(0, 32);

  prefsSTA.begin("wifi_sta", false);
  prefsSTA.putBool  ("en",     en);
  prefsSTA.putString("ssid",   ssid);
  if (pass.length() > 0) {
    prefsSTA.putString("pass", pass);
  } else if (!en) {
  }
  prefsSTA.putString("host",   host);
  prefsSTA.putBool  ("static", staticOn);
  if (staticOn) {
    prefsSTA.putString("ip",  ip);
    prefsSTA.putString("gw",  gw);
    prefsSTA.putString("sn",  sn);
    prefsSTA.putString("dns", dns);
  }
  prefsSTA.end();

  server.send(200, "text/plain", "OK");
  delay(1000);
  ESP.restart();
}

void handleGpioPage() {
  server.send(200, "text/html", getGpioHTML());
}

void handleGPIOSet() {
  if (!server.hasArg("relay") || !server.hasArg("pin")) {
    server.send(400, "text/plain", "Bad Request");
    return;
  }

  int idx = server.arg("relay").toInt();
  int newPin = server.arg("pin").toInt();
  bool activeHigh = server.hasArg("activeHigh") && server.arg("activeHigh") == "1";

  if (idx < 0 || idx >= 16) {
    server.send(400, "text/plain", "Bad relay index"); return;
  }
  if (!isValidPin(newPin)) {
    server.send(400, "text/plain", "Invalid GPIO"); return;
  }
  if (rtcEnabled && (newPin == RTC_SDA_PIN || newPin == RTC_SCL_PIN)) {
    server.send(400, "text/plain", "GPIO reserved for DS3231"); return;
  }
  for (int i = 0; i < 16; i++) {
    if (i != idx && relayPins[i] == newPin) {
      server.send(400, "text/plain", "GPIO already in use by Relay " + String(i + 1));
      return;
    }
  }

  if (relayPins[idx] != newPin) {
    pinMode(relayPins[idx], INPUT);
    relayPins[idx] = newPin;
  }
  relayActiveHigh[idx] = activeHigh;

  pinMode(relayPins[idx], OUTPUT);
  writeRelay(idx);

  prefsRelay.begin("relays", false);
  prefsRelay.putInt (("p" + String(idx)).c_str(), newPin);
  prefsRelay.putBool(("a" + String(idx)).c_str(), activeHigh);
  prefsRelay.end();

  server.send(200, "text/plain", "OK");
}

void handleMqttPage() {
  server.send(200, "text/html", getMqttHTML());
}

void handleSetMqtt() {
  if (!server.hasArg("en") || !server.hasArg("srv")) {
    server.send(400, "text/plain", "Bad Request"); return;
  }
  bool en = server.arg("en") == "1";
  String srv  = server.arg("srv"); srv.trim();
  uint16_t port = server.arg("port").toInt();
  if (port == 0) port = 1883;
  String base = server.arg("base"); base.trim();
  if (base.length() == 0) base = DEFAULT_MQTT_BASE;
  String usr = server.arg("usr");
  String pw  = server.arg("pw");

  if (base.indexOf('+') >= 0 || base.indexOf('#') >= 0 || base.indexOf(' ') >= 0) {
    server.send(400, "text/plain", "Base topic cannot contain +, # or spaces"); return;
  }
  if (port == 8883) {
    server.send(400, "text/plain", "TLS (8883) not supported — use 1883/1884"); return;
  }
  if (en && srv.length() == 0) {
    server.send(400, "text/plain", "Broker required"); return;
  }

  prefsMQTT.begin("mqtt", false);
  prefsMQTT.putBool  ("en",   en);
  prefsMQTT.putString("srv",  srv);
  prefsMQTT.putUShort("port", port);
  prefsMQTT.putString("base", base);
  prefsMQTT.putString("usr",  usr);
  if (pw.length() > 0) {
    prefsMQTT.putString("pw", pw);
  }
  prefsMQTT.end();

  server.send(200, "text/plain", "OK");
  delay(1000);
  ESP.restart();
}

void handleSchedulePage() {
  server.send(200, "text/html", getScheduleHTML());
}

void handleSchedList() {
  String json = "[";
  for (int i = 0; i < SCHED_COUNT; i++) {
    Schedule& s = schedules[i];
    if (i) json += ",";
    json += "{";
    json += "\"idx\":" + String(i) + ",";
    json += "\"en\":" + String(s.enabled ? "true" : "false") + ",";
    json += "\"mode\":" + String(s.mode) + ",";
    json += "\"mask\":" + String(s.relayMask) + ",";
    json += "\"name\":\"" + jsonEscape(String(s.name)) + "\"";
    json += "}";
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleSchedSet() {
  if (!server.hasArg("idx")) { server.send(400, "text/plain", "Bad Request"); return; }
  int idx = server.arg("idx").toInt();
  if (idx < 0 || idx >= SCHED_COUNT) { server.send(400, "text/plain", "Bad idx"); return; }

  Schedule& s = schedules[idx];
  s.enabled = server.arg("en") == "1";

  String nm = server.arg("name");
  nm.trim();
  if (nm.length() == 0) nm = "Schedule " + String(idx + 1);
  if (nm.length() > 23) nm = nm.substring(0, 23);
  snprintf(s.name, sizeof(s.name), "%s", nm.c_str());

  s.mode = (uint8_t)server.arg("mode").toInt();
  if (s.mode > 2) s.mode = 0;

  s.relayMask = 0;
  if (server.hasArg("mask")) {
    s.relayMask = (uint16_t)strtoul(server.arg("mask").c_str(), nullptr, 10);
  }

  auto parseField = [&](const char* key, uint8_t maxVal) -> uint8_t {
    if (!server.hasArg(key)) return ANY8;
    String v = server.arg(key);
    v.trim();
    if (v.length() == 0) return ANY8;
    long n = v.toInt();
    if (n < 0 || n > maxVal) return ANY8;
    return (uint8_t)n;
  };

  s.startHour = parseField("sh", 23);
  s.startMin  = parseField("sm", 59);
  s.startSec  = parseField("ss", 59);
  s.stopHour  = parseField("eh", 23);
  s.stopMin   = parseField("em", 59);
  s.stopSec   = parseField("es", 59);

  s.startDowMask = 0;
  s.stopDowMask  = 0;
  if (server.hasArg("sdow")) {
    s.startDowMask = (uint32_t)strtoul(server.arg("sdow").c_str(), nullptr, 10);
  }
  if (server.hasArg("edow")) {
    s.stopDowMask = (uint32_t)strtoul(server.arg("edow").c_str(), nullptr, 10);
  }
  if (s.startDowMask == 0) s.startDowMask = ANY32;
  if (s.stopDowMask  == 0) s.stopDowMask  = ANY32;

  s.domMask = 0;
  if (server.hasArg("dom")) {
    s.domMask = (uint32_t)strtoul(server.arg("dom").c_str(), nullptr, 10);
  }
  if (s.domMask == 0) s.domMask = ANY32;

  s.monthMask = 0;
  if (server.hasArg("mon")) {
    s.monthMask = (uint16_t)strtoul(server.arg("mon").c_str(), nullptr, 10);
  }
  if (s.monthMask == 0) s.monthMask = 0x0FFF;

  s.tailSeconds = 0;
  if (server.hasArg("tail")) {
    long t = server.arg("tail").toInt();
    if (t < 0) t = 0;
    if (t > 86400) t = 86400;
    s.tailSeconds = (uint32_t)t;
  }

  s.matchedPrev = false;
  s.tailArmed   = false;
  s.tailArmedAt = 0;

  saveSchedPrefs();
  server.send(200, "text/plain", "OK");
}

void handleSchedDel() {
  if (!server.hasArg("idx")) { server.send(400, "text/plain", "Bad Request"); return; }
  int idx = server.arg("idx").toInt();
  if (idx < 0 || idx >= SCHED_COUNT) { server.send(400, "text/plain", "Bad idx"); return; }

  Schedule& s = schedules[idx];
  memset(&s, 0, sizeof(Schedule));
  s.enabled     = false;
  s.mode        = MODE_NORMAL;
  s.relayMask   = 0;
  s.startDowMask= ANY32;
  s.stopDowMask = ANY32;
  s.domMask     = ANY32;
  s.monthMask   = 0x0FFF;
  snprintf(s.name, sizeof(s.name), "Schedule %d", idx + 1);

  saveSchedPrefs();
  server.send(200, "text/plain", "OK");
}

void handleSchedTest() {
  if (!server.hasArg("idx")) { server.send(400, "text/plain", "Bad Request"); return; }
  int idx = server.arg("idx").toInt();
  if (idx < 0 || idx >= SCHED_COUNT) { server.send(400, "text/plain", "Bad idx"); return; }

  Schedule& s = schedules[idx];
  if (s.relayMask == 0) { server.send(400, "text/plain", "No relays selected"); return; }

  bool anyOn = false;
  for (int i = 0; i < 16; i++) {
    if (s.relayMask & (1U << i)) {
      if (relayState[i]) { anyOn = true; break; }
    }
  }
  applyRelayMask(s.relayMask, !anyOn);

  server.send(200, "text/plain", "OK");
}

void handleTimePage() {
  server.send(200, "text/html", getTimeHTML());
}

void handleSetTime() {
  if (!server.hasArg("tz") || !server.hasArg("ntp1")) {
    server.send(400, "text/plain", "Bad Request"); return;
  }

  int tz = server.arg("tz").toInt();
  if (tz < -720 || tz > 840) tz = -480;

  String n1 = server.arg("ntp1"); n1.trim();
  String n2 = server.arg("ntp2"); n2.trim();
  if (n1.length() == 0 || n1.length() > 63) { server.send(400, "text/plain", "Invalid NTP1"); return; }
  if (n2.length() > 63) { server.send(400, "text/plain", "Invalid NTP2"); return; }
  if (n2.length() == 0) n2 = DEFAULT_NTP_2;

  bool rtc = server.hasArg("rtc") && server.arg("rtc") == "1";

  if (rtc && !i2cPinsFree()) {
    server.send(400, "text/plain", "GPIO 21/22 in use by relays — cannot enable RTC"); return;
  }

  tzOffsetMinutes = tz;
  ntpServer1 = n1;
  ntpServer2 = n2;
  rtcEnabled = rtc;
  saveTimePrefs();

  server.send(200, "text/plain", "OK");
  delay(1000);
  ESP.restart();
}

void handleRtcSet() {
  if (!server.hasArg("y") || !server.hasArg("mo") || !server.hasArg("d") ||
      !server.hasArg("h") || !server.hasArg("mi") || !server.hasArg("s")) {
    server.send(400, "text/plain", "Bad Request"); return;
  }

  int y  = server.arg("y").toInt();
  int mo = server.arg("mo").toInt();
  int d  = server.arg("d").toInt();
  int h  = server.arg("h").toInt();
  int mi = server.arg("mi").toInt();
  int s  = server.arg("s").toInt();

  if (y < 2024 || y > 2099) { server.send(400, "text/plain", "Year out of range"); return; }
  if (mo < 1 || mo > 12)    { server.send(400, "text/plain", "Month out of range"); return; }
  if (d < 1 || d > 31)      { server.send(400, "text/plain", "Day out of range"); return; }
  if (h < 0 || h > 23)      { server.send(400, "text/plain", "Hour out of range"); return; }
  if (mi < 0 || mi > 59)    { server.send(400, "text/plain", "Minute out of range"); return; }
  if (s < 0 || s > 59)      { server.send(400, "text/plain", "Second out of range"); return; }

  struct tm tm = {};
  tm.tm_year = y - 1900;
  tm.tm_mon  = mo - 1;
  tm.tm_mday = d;
  tm.tm_hour = h;
  tm.tm_min  = mi;
  tm.tm_sec  = s;
  tm.tm_isdst = -1;

  time_t utc = mktime(&tm);
  utc -= tzOffsetMinutes * 60;

  struct timeval tv;
  tv.tv_sec  = utc;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);

  if (rtcEnabled && rtcPresent) {
    DateTime dt((uint32_t)utc);
    rtc.adjust(dt);
  }

  ntpSynced = false;
  server.send(200, "text/plain", "OK");
}

void handleNtpNow() {
  if (WiFi.status() != WL_CONNECTED) {
    server.send(400, "text/plain", "STA not connected"); return;
  }
  applyTimeConfig();
  for (int i = 0; i < 20; i++) {
    delay(250);
    time_t now = time(nullptr);
    if (now > 1700000000) {
      if (rtcEnabled && rtcPresent) {
        DateTime dt((uint32_t)now);
        rtc.adjust(dt);
      }
      ntpSynced = true;
      server.send(200, "text/plain", "OK");
      return;
    }
  }
  server.send(400, "text/plain", "NTP timeout");
}

void handleNotFound() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void applyWifiMode() {
  bool apOn  = apEnabled;
  bool staOn = staEnabled;

  if (!apOn && !staOn) {
    apOn = true;
    apEnabled = true;
  }

  bool staOnlyRequested = (!apOn && staOn);
  if (staOnlyRequested) {
    apOn = true;
  }

  if (apOn && staOn)      WiFi.mode(WIFI_AP_STA);
  else if (apOn)          WiFi.mode(WIFI_AP);
  else                    WiFi.mode(WIFI_STA);

  if (apOn) {
    WiFi.softAPConfig(AP_DEFAULT_IP, AP_DEFAULT_GW, AP_DEFAULT_MASK);
    if (apPassword.length() >= 8) {
      WiFi.softAP(apSSID.c_str(), apPassword.c_str(), apChannel, apHidden, apMaxClients);
    } else {
      WiFi.softAP(apSSID.c_str(), nullptr, apChannel, apHidden, apMaxClients);
    }
  }

  if (staOn) {
    WiFi.setHostname(staHostname.c_str());
    if (staStatic) {
      WiFi.config(staIP, staGW, staSN, staDNS);
    }
    if (staPassword.length() >= 8) {
      WiFi.begin(staSSID.c_str(), staPassword.c_str());
    } else {
      WiFi.begin(staSSID.c_str());
    }
  }

  if (apOn) {
    dnsServer.start(53, "*", WiFi.softAPIP());
  }

  staOnlyMode        = staOnlyRequested;
  staOnlyDeadline    = millis() + STA_ONLY_BOOT_TIMEOUT_MS;
  staOnlyConnectedAt = 0;
  wasStaConnected    = (WiFi.status() == WL_CONNECTED);
  staRuntimeDropStart = 0;
  lastStaRetry       = millis();
}

void staWatchdog() {
  if (!staEnabled) return;

  wl_status_t st = WiFi.status();
  bool connected = (st == WL_CONNECTED);

  if (!connected &&
      (st == WL_NO_SSID_AVAIL || st == WL_CONNECT_FAILED || st == WL_CONNECTION_LOST) &&
      millis() - lastStaRetry > STA_RETRY_INTERVAL_MS) {
    lastStaRetry = millis();
    WiFi.disconnect();
    if (staPassword.length() >= 8) {
      WiFi.begin(staSSID.c_str(), staPassword.c_str());
    } else {
      WiFi.begin(staSSID.c_str());
    }
  }

  if (staOnlyMode) {
    if (connected) {
      if (staOnlyConnectedAt == 0) staOnlyConnectedAt = millis();
      if (millis() - staOnlyConnectedAt > STA_ONLY_DROP_GRACE_MS) {
        dnsServer.stop();
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        apEnabled = false;
        prefsAP.begin("wifi_ap", false);
        prefsAP.putBool("en", false);
        prefsAP.end();
        staOnlyMode = false;
      }
    } else if (millis() > staOnlyDeadline) {
      apEnabled = true;
      prefsAP.begin("wifi_ap", false);
      prefsAP.putBool("en", true);
      prefsAP.end();

      WiFi.mode(WIFI_AP_STA);
      WiFi.softAPConfig(AP_DEFAULT_IP, AP_DEFAULT_GW, AP_DEFAULT_MASK);
      if (apPassword.length() >= 8) {
        WiFi.softAP(apSSID.c_str(), apPassword.c_str(), apChannel, apHidden, apMaxClients);
      } else {
        WiFi.softAP(apSSID.c_str(), nullptr, apChannel, apHidden, apMaxClients);
      }
      dnsServer.start(53, "*", WiFi.softAPIP());
      staOnlyMode = false;
    }
  }

  if (!staOnlyMode) {
    if (wasStaConnected && !connected) {
      if (staRuntimeDropStart == 0) staRuntimeDropStart = millis();
      if (millis() - staRuntimeDropStart > STA_RUNTIME_DROP_TIMEOUT_MS) {
        if (!apEnabled) {
          apEnabled = true;
          prefsAP.begin("wifi_ap", false);
          prefsAP.putBool("en", true);
          prefsAP.end();

          WiFi.mode(WIFI_AP_STA);
          WiFi.softAPConfig(AP_DEFAULT_IP, AP_DEFAULT_GW, AP_DEFAULT_MASK);
          if (apPassword.length() >= 8) {
            WiFi.softAP(apSSID.c_str(), apPassword.c_str(), apChannel, apHidden, apMaxClients);
          } else {
            WiFi.softAP(apSSID.c_str(), nullptr, apChannel, apHidden, apMaxClients);
          }
          dnsServer.start(53, "*", WiFi.softAPIP());
        }
        staRuntimeDropStart = 0;
      }
    } else if (connected) {
      wasStaConnected = true;
      staRuntimeDropStart = 0;
    }
  }
}

void setup() {
  loadRelayPrefs();
  loadAPPrefs();
  loadSTAPrefs();
  loadMQTTPrefs();
  loadSchedPrefs();
  loadTimePrefs();

  initRelays();

  initRTC();

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  char cid[64];
  snprintf(cid, sizeof(cid), "esp32_16ch_%02X%02X%02X", mac[3], mac[4], mac[5]);
  mqttClientId = String(cid);

  mqttClient.setBufferSize(2048);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setKeepAlive(30);
  espClient.setTimeout(2);

  applyWifiMode();

  esp_wifi_set_ps(WIFI_PS_NONE);

  applyTimeConfig();

  delay(500);

  server.on("/",           handleRoot);
  server.on("/toggle",     handleToggle);
  server.on("/rename",     handleRename);
  server.on("/wifi/ap",    handleWifiApPage);
  server.on("/wifi/sta",   handleWifiStaPage);
  server.on("/wifi/scan",  handleWifiScan);
  server.on("/setap",      handleSetAp);
  server.on("/setsta",     handleSetSta);
  server.on("/gpio",       handleGpioPage);
  server.on("/setgpio",    handleGPIOSet);
  server.on("/mqtt",       handleMqttPage);
  server.on("/setmqtt",    handleSetMqtt);
  server.on("/schedule",   handleSchedulePage);
  server.on("/schedlist",  handleSchedList);
  server.on("/schedset",   handleSchedSet);
  server.on("/scheddel",   handleSchedDel);
  server.on("/schedtest",  handleSchedTest);
  server.on("/time",       handleTimePage);
  server.on("/settime",    handleSetTime);
  server.on("/rtcset",     handleRtcSet);
  server.on("/ntpnow",     handleNtpNow);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  mqttLoop();
  staWatchdog();
  rtcMaintenance();
  scheduleTick();
}
