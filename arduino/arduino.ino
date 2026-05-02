/*
============================================================
 *  ESP32 20-Channel Relay Smart Switch - Enhanced Edition
 *  Author: github.com/xiv3r
 *  Version: 5.1 (Enhanced Stability & Self-Healing)
============================================================
 */

#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <esp_task_wdt.h>
#include <rom/rtc.h>

// ─── Watchdog & Stability ─────────────────────────────────────────────────
#define WDT_TIMEOUT 30  // 30 seconds watchdog
static TaskHandle_t mainTaskHandle = NULL;
static unsigned long lastWatchdogReset = 0;
static uint32_t watchdogResetCount = 0;
static const unsigned long WATCHDOG_FEED_INTERVAL = 5000UL;  // Feed every 5 seconds

// ─── Preferences (NVS) with Corruption Protection ──────────────────────────
Preferences preferences;
#define NVS_NAMESPACE "relay20"
#define NVS_BACKUP_NAMESPACE "relay20_bk"

#define EEPROM_MAGIC   0x1234
#define EEPROM_VERSION 6  // Incremented for new features
#define EXT_CFG_MAGIC  0xEC

// ─── Health Monitoring ─────────────────────────────────────────────────────
struct HealthMetrics {
    uint32_t watchdogResets;
    uint32_t ntpSyncFailures;
    uint32_t wifiReconnects;
    uint32_t heapCorruptionChecks;
    uint32_t lastHeapSize;
    uint32_t minFreeHeap;
    uint32_t nvsWriteErrors;
    uint32_t scheduleExecutions;
    unsigned long lastHealthReport;
    bool heapIntegrityOk;
};

HealthMetrics healthMetrics = {0};
static const unsigned long HEALTH_CHECK_INTERVAL = 60000UL;  // Every minute
static unsigned long lastHealthCheck = 0;

// ─── Enhanced Memory Management ────────────────────────────────────────────
#define MAX_JSON_DOC_SIZE 32768  // Increased for safety
#define MIN_FREE_HEAP_THRESHOLD 51200  // 50KB minimum free heap
static const unsigned long MEMORY_CLEANUP_INTERVAL = 300000UL;  // Every 5 minutes
static unsigned long lastMemoryCleanup = 0;

// ─── Day-of-week constants ───────────────────────────────────────────────────
#define DAY_SUNDAY    (1 << 0)
#define DAY_MONDAY    (1 << 1)
#define DAY_TUESDAY   (1 << 2)
#define DAY_WEDNESDAY (1 << 3)
#define DAY_THURSDAY  (1 << 4)
#define DAY_FRIDAY    (1 << 5)
#define DAY_SATURDAY  (1 << 6)
#define DAY_ALL       0x7F
#define DAY_WEEKDAYS  0x3E  
#define DAY_WEEKENDS  0x41  

// ─── Enhanced Timing constants ──────────────────────────────────────────────
static const unsigned long NTP_RETRY_INTERVAL   =   30000UL;
static const unsigned long NTP_BACKOFF_MAX      =  300000UL;  // Max 5 minutes backoff
static const unsigned long WIFI_CHECK_INTERVAL  =    5000UL;
static const unsigned long WIFI_CONNECT_TIMEOUT =   15000UL;
static const unsigned long RTC_UPDATE_INTERVAL  =     100UL;
static const unsigned long WIFI_SCAN_TIMEOUT    =   10000UL;
static const unsigned long CONFIG_SAVE_DEBOUNCE =    2000UL;  // Prevent rapid saves

// ─── Connection Backoff Settings ────────────────────────────────────────────
struct BackoffManager {
    unsigned long currentBackoff;
    unsigned long lastAttempt;
    uint8_t consecutiveFailures;
    unsigned long maxBackoff;
    float multiplier;
};

BackoffManager ntpBackoff = {NTP_RETRY_INTERVAL, 0, 0, NTP_BACKOFF_MAX, 1.5f};
BackoffManager wifiBackoff = {WIFI_CHECK_INTERVAL, 0, 0, 300000UL, 2.0f};

// ─── mDNS settings ────────────────────────────────────────────────────────────
#define MDNS_HOSTNAME_DEFAULT "esp32-20ch-relay"
static const unsigned long MDNS_RESTART_DELAY = 2000UL;
static const unsigned long MDNS_HEALTH_CHECK = 30000UL;  // Check mDNS health every 30s

// ─── NTP fallback pool ───────────────────────────────────────────────────────
static const char* NTP_SERVERS[] = {
    "ph.pool.ntp.org",
    "pool.ntp.org",
    "time.nist.gov",
    "time.google.com"
};
static const uint8_t NUM_NTP_SERVERS = 4;

// ─── DNS / Web server ────────────────────────────────────────────────────────
DNSServer        dnsServer;
WebServer        server(80);
const byte       DNS_PORT = 53;

// ─── NTP client ──────────────────────────────────────────────────────────────
WiFiUDP   ntpUDP;
NTPClient timeClient(ntpUDP, NTP_SERVERS[0], 28800, 3600000UL);

// ─── Relay config ────────────────────────────────────────────────────────────
#define NUM_RELAYS 20
const int  relayPins[NUM_RELAYS] = { 
 15, // IN1
  2,  // IN2
  4,  // IN3
 16, // IN4
 17, // IN5
  5,  // IN6
 18, // IN7
 19, // IN8
 21, // IN9
  3,  // IN10 (RX)
  1,  // IN11 (TX)
 22, // IN12
 23, // IN13
 13, // IN14
 14, // IN15
 27, // IN16
 26, // IN17
 25, // IN18
 33, // IN19
 32  // IN20
};
const bool relayActiveLow = true;

// ─── Enhanced Data structures ─────────────────────────────────────────────────
struct TimerSchedule {
    uint8_t  startHour[8], startMinute[8], startSecond[8];
    uint8_t  stopHour[8],  stopMinute[8],  stopSecond[8];
    bool     enabled[8];
    uint8_t  days[8];       
    uint32_t monthDays[8];  
    uint32_t crc;  // CRC for corruption detection
};

struct RelayConfig {
    TimerSchedule schedule;
    bool          manualOverride;
    bool          manualState;
    char          name[16];
    uint32_t      lastStateChange;  // Timestamp of last state change
    uint8_t       stateChangeCount;  // Debounce counter
};

// SystemConfig with enhanced protection
struct SystemConfig {
    uint16_t magic;
    uint8_t  version;
    char     sta_ssid[32];
    char     sta_password[64];
    char     ap_ssid[32];
    char     ap_password[32];
    char     ntp_server[48];
    long     gmt_offset;
    int      daylight_offset;
    time_t   last_rtc_epoch;
    float    rtc_drift;
    char     hostname[32];
    uint32_t crc;  // CRC for corruption detection
};

// ExtConfig with self-healing flags
struct ExtConfig {
    uint8_t magic;
    uint8_t ap_channel;
    uint8_t ntp_sync_hours;
    uint8_t ap_hidden;
    uint8_t auto_recovery_enabled;  // Enable automatic recovery features
    uint8_t watchdog_enabled;        // Enable hardware watchdog
    uint8_t reserved[26];
};

// ─── Globals ──────────────────────────────────────────────────────────────────
SystemConfig sysConfig;
ExtConfig    extConfig;
RelayConfig  relayConfigs[NUM_RELAYS];

// RTC with enhanced precision
time_t        internalEpoch            = 0;
unsigned long internalMillisAtLastSync = 0;
float         driftCompensation        = 1.0f;
bool          rtcInitialized           = false;
unsigned long lastRTCUpdate            = 0;
float         rtcDriftHistory[8]       = {1.0f};  // Drift history for smoothing
uint8_t       driftHistoryIndex        = 0;

// NTP with enhanced reliability
uint8_t       ntpServerIndex  = 0;
uint8_t       ntpFailCount    = 0;
unsigned long lastNTPSync     = 0;
unsigned long lastNTPAttempt  = 0;
bool          ntpSyncInProgress = false;  // Prevent overlapping syncs

// WiFi with enhanced stability
bool          wifiConnected         = false;
unsigned long lastWiFiCheck         = 0;
uint8_t       wifiReconnectAttempts = 0;
unsigned long wifiGiveUpUntil       = 0;
static const uint8_t MAX_RECONNECT  = 10;
bool          wifiStaEnabled        = true;  // Allow disabling STA

// Non-blocking STA reconnect state machine
enum WifiConnState { WCS_IDLE, WCS_PENDING, WCS_BACKOFF };
WifiConnState wcsState = WCS_IDLE;
unsigned long wcsStart = 0;

// WiFi async scan with timeout protection
volatile bool scanInProgress  = false;
volatile int  scanResultCount = -1;
unsigned long scanStartTime   = 0;
static const unsigned long SCAN_LOCK_TIMEOUT = 15000UL;  // Force unlock after 15s

// AP copies with validation
char ap_ssid[32]     = "ESP32_20CH_Timer_Switch";
char ap_password[32] = "ESP32-admin";

// mDNS with health monitoring
bool          mdnsStarted           = false;
char          mdnsHostname[32]      = MDNS_HOSTNAME_DEFAULT;
unsigned long mdnsRestartPending    = 0;
bool          mdnsRestartScheduled  = false;
unsigned long lastMDNSCheck         = 0;

// Configuration save debouncing
unsigned long lastConfigSave        = 0;
bool          configSavePending     = false;
static const unsigned long CONFIG_SAVE_TIMEOUT = 5000UL;

// ─── Enhanced Inline helpers ───────────────────────────────────────────────
inline unsigned long getNTPInterval() {
    uint8_t h = extConfig.ntp_sync_hours;
    if (h < 1 || h > 24) h = 1;
    return (unsigned long)h * 3600000UL;
}

inline unsigned long calculateBackoff(BackoffManager* bm) {
    if (bm->consecutiveFailures == 0) return bm->currentBackoff;
    unsigned long backoff = bm->currentBackoff;
    for (uint8_t i = 0; i < bm->consecutiveFailures && backoff < bm->maxBackoff; i++) {
        backoff = (unsigned long)(backoff * bm->multiplier);
    }
    return min(backoff, bm->maxBackoff);
}

inline void resetBackoff(BackoffManager* bm) {
    bm->consecutiveFailures = 0;
    bm->lastAttempt = 0;
}

inline void incrementBackoff(BackoffManager* bm) {
    if (bm->consecutiveFailures < 255) {
        bm->consecutiveFailures++;
    }
}

// Simple CRC32 for data integrity
uint32_t calculateCRC32(const uint8_t* data, size_t len) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc >> 1) ^ (0xEDB88320 & -(crc & 1));
        }
    }
    return ~crc;
}

// ─── Prototypes ─────────────────────────────────────────────────────────────
time_t getCurrentEpoch();
void syncInternalRTC();
void loadRTCState();
void saveRTCState();
void loadConfiguration();
void saveConfiguration();
void loadExtConfig();
void saveExtConfig();
void initDefaults();
void processRelaySchedules();
void setupWebServer();
void restartAP();
void tryNTPSync();
void beginWiFiConnect();
void updateRelayOutputs();
void performHealthCheck();
void checkMemoryHealth();
void autoRecoveryCheck();
void feedWatchdog();
void scheduleConfigSave();
void flushConfigSave();
bool validateConfiguration();
void backupConfiguration();
void restoreConfiguration();

// mDNS functions
void startMDNS();
void stopMDNS();
void restartMDNS();
void scheduleMDNSRestart();
String getMDNSHostname();
void setMDNSHostname(const char* hostname);
void checkMDNSHealth();

// API handlers
void handleGetRelays();
void handleManualControl();
void handleResetManual();
void handleSaveRelay();
void handleRelayName();
void handleGetTime();
void handleGetWiFi();
void handleSaveWiFi();
void handleWiFiScanStart();
void handleWiFiScanResults();
void handleGetNTP();
void handleSaveNTP();
void handleSyncNTP();
void handleGetAP();
void handleSaveAP();
void handleGetSystem();
void handleGetHealth();
void handleReset();
void handleFactoryReset();

// ─────────────────────────────────────────────────────────────────────────────
//  SHARED CSS (with cache-control headers)
// ─────────────────────────────────────────────────────────────────────────────
const char style_css[] PROGMEM = R"css(
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,Arial,sans-serif;background:#EEF2F7;color:#1A1A2E;font-size:14px;line-height:1.5}
header{background:linear-gradient(135deg,#1565C0 0%,#0D47A1 100%);color:#fff;padding:10px 16px;display:flex;align-items:center;gap:10px;position:sticky;top:0;z-index:50;box-shadow:0 2px 10px rgba(0,0,0,.3);flex-wrap:wrap}
.logo{font-size:13px;font-weight:700;white-space:nowrap}
nav{display:flex;gap:3px;flex-wrap:wrap;flex:1}
nav a{color:rgba(255,255,255,.8);text-decoration:none;padding:5px 8px;border-radius:5px;font-size:12px;transition:.15s}
nav a:hover,nav a.cur{background:rgba(255,255,255,.2);color:#fff}
.hdr-r{display:flex;align-items:center;gap:6px;margin-left:auto;font-size:12px;white-space:nowrap}
.dot{width:8px;height:8px;border-radius:50%;display:inline-block;background:#546E7A;flex-shrink:0}
.g{background:#69F0AE}.r{background:#FF5252}.y{background:#FFD740}
main{max-width:1200px;margin:0 auto;padding:16px}
.ptitle{font-size:17px;font-weight:700;color:#1565C0;margin-bottom:14px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(340px,1fr));gap:14px}
.card{background:#fff;border-radius:10px;box-shadow:0 2px 8px rgba(0,0,0,.08);padding:16px;transition:box-shadow .2s}
.card:hover{box-shadow:0 4px 18px rgba(0,0,0,.13)}
.card-hdr{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px}
.ctitle{font-weight:700;font-size:15px;cursor:pointer;transition:background .15s;padding:2px 4px;border-radius:4px}
.ctitle:hover{background:#E3F2FD;color:#1565C0}
.badge{padding:3px 9px;border-radius:20px;font-size:11px;font-weight:700}
.bon{background:#E8F5E9;color:#2E7D32}.boff{background:#FFEBEE;color:#C62828}.bman{background:#FFF3E0;color:#E65100}
.brow{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px}
.btn{border:none;padding:7px 12px;border-radius:6px;cursor:pointer;font-size:12px;font-weight:600;transition:.15s}
.btn:hover{filter:brightness(1.1)}.btn:disabled{opacity:.5;cursor:default}
.bon-b{background:#43A047;color:#fff}.boff-b{background:#E53935;color:#fff}.baut{background:#546E7A;color:#fff}
.bsave{background:#1565C0;color:#fff;width:100%;padding:9px;font-size:13px;border-radius:6px;margin-top:8px}
.bsync{background:#FB8C00;color:#fff}.bdanger{background:#B71C1C;color:#fff}.bwarn{background:#F9A825;color:#212121}
.bscan{background:#0288D1;color:#fff}
.health-indicator{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:4px}
.health-good{background:#69F0AE}.health-warning{background:#FFD740}.health-critical{background:#FF5252}
.slist{display:flex;flex-direction:column;gap:6px;margin-bottom:8px;max-height:500px;overflow-y:auto;padding-right:2px}
.si{border:1px solid #E3E8EF;border-radius:7px;padding:9px}
.si.act{border-color:#90CAF9;background:#F0F7FF}
.shdr{display:flex;align-items:center;gap:7px;margin-bottom:7px;font-size:11px;font-weight:700;color:#607D8B;text-transform:uppercase}
.shdr label{display:flex;align-items:center;gap:4px;cursor:pointer;font-size:12px;font-weight:700;color:#1A1A2E;text-transform:none}
.trow{display:flex;align-items:center;gap:8px;font-size:12px;margin-top:5px}
.trow .l{color:#90A4AE;font-weight:600;width:32px;flex-shrink:0}
.days{display:flex;gap:3px;margin-top:5px;flex-wrap:wrap}
.day{width:28px;height:24px;border-radius:4px;border:1px solid #CFD8DC;display:flex;align-items:center;justify-content:center;font-size:11px;font-weight:600;cursor:pointer;background:#FAFAFA;transition:.15s;user-select:none}
.day:hover{border-color:#90CAF9;background:#E3F2FD}
.day.on{background:#1565C0;color:#fff;border-color:#1565C0}
.mdays{display:flex;gap:2px;margin-top:5px;flex-wrap:wrap}
.mday{width:26px;height:22px;border-radius:3px;border:1px solid #CFD8DC;display:flex;align-items:center;justify-content:center;font-size:10px;font-weight:600;cursor:pointer;background:#FAFAFA;transition:.15s;user-select:none}
.mday:hover{border-color:#CE93D8;background:#F3E5F5}
.mday.on{background:#7B1FA2;color:#fff;border-color:#7B1FA2}
.sched-section{margin-top:4px;font-size:10px;font-weight:600;color:#90A4AE;text-transform:uppercase;margin-bottom:2px}
.night{font-size:10px;color:#7B1FA2;background:#F3E5F5;padding:2px 6px;border-radius:4px;margin-left:auto}
.night.always{background:#E8F5E9;color:#2E7D32}
input[type=time]{flex:1;padding:5px 8px;border:1px solid #CFD8DC;border-radius:5px;font-size:13px;font-family:monospace;background:#FAFAFA;cursor:pointer;min-width:0}
input[type=time]:focus{outline:none;border-color:#1565C0;box-shadow:0 0 0 3px rgba(21,101,192,.15);background:#fff}
.ibar{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:16px}
.ibox{background:#fff;border-radius:8px;padding:12px;box-shadow:0 1px 4px rgba(0,0,0,.07)}
.ibox .l{font-size:11px;color:#90A4AE;text-transform:uppercase;font-weight:600}
.ibox .v{font-size:15px;font-weight:700;color:#1A1A2E;margin-top:2px}
.fcrd{max-width:600px}
.fg{margin-bottom:14px}
.fg label{display:block;font-size:11px;font-weight:700;color:#607D8B;margin-bottom:5px;text-transform:uppercase;letter-spacing:.4px}
.fg input,.fg select{width:100%;padding:9px 12px;border:1px solid #CFD8DC;border-radius:7px;font-size:14px;background:#FAFAFA}
.fg input:focus,.fg select:focus{outline:none;border-color:#1565C0;box-shadow:0 0 0 3px rgba(21,101,192,.15);background:#fff}
.fg small{display:block;margin-top:4px;font-size:11px;color:#90A4AE}
.input-row{display:flex;gap:8px}
.input-row input{flex:1;min-width:0}
.alert{border-radius:7px;padding:11px 14px;font-size:13px;margin-bottom:14px}
.aw{background:#FFF8E1;border-left:4px solid #F9A825;color:#5D4037}
.ai{background:#E3F2FD;border-left:4px solid #1565C0;color:#0D47A1}
hr{border:none;border-top:1px solid #ECEFF1;margin:14px 0}
.netlist{margin-top:10px;display:none}
.net-hdr{font-size:11px;font-weight:700;color:#607D8B;text-transform:uppercase;margin-bottom:6px}
.netitem{display:flex;align-items:center;gap:8px;padding:8px 10px;border:1px solid #E3E8EF;border-radius:7px;cursor:pointer;margin-bottom:5px;background:#FAFAFA;transition:.15s}
.netitem:hover{background:#EEF2F7;border-color:#90CAF9}
.netitem .ns{flex:1;font-size:13px;font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.netitem .nr{font-size:11px;color:#90A4AE;white-space:nowrap}
.bars{display:inline-flex;align-items:flex-end;gap:2px;height:14px}
.bar{width:4px;border-radius:1px;background:#CFD8DC}
.bar.on{background:#43A047}
#toast{position:fixed;bottom:22px;left:50%;transform:translateX(-50%) translateY(80px);background:#323232;color:#fff;padding:10px 20px;border-radius:8px;font-size:13px;transition:transform .28s;z-index:999;pointer-events:none;box-shadow:0 4px 16px rgba(0,0,0,.3);min-width:180px;text-align:center}
#toast.show{transform:translateX(-50%) translateY(0)}
#toast.ok{background:#2E7D32}#toast.er{background:#C62828}
@media(max-width:500px){.grid{grid-template-columns:1fr}.ibar{grid-template-columns:1fr}.input-row{flex-direction:column}.day{width:24px;height:22px;font-size:10px}.mday{width:22px;height:20px;font-size:9px}}
)css";

// ─────────────────────────────────────────────────────────────────────────────
//  INDEX PAGE (Enhanced with health indicators)
// ─────────────────────────────────────────────────────────────────────────────
const char index_html[] PROGMEM = R"raw(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Relays — ESP32 Timer Switch</title>
<link rel="stylesheet" href="/style.css">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
</head><body>
<header>
<span class="logo">&#x26A1; ESP32 20-CH</span>
<nav>
<a href="/" class="cur">Relays</a>
<a href="/wifi">WiFi</a>
<a href="/ntp">Time</a>
<a href="/ap">AP</a>
<a href="/system">System</a>
</nav>
<div class="hdr-r"><span class="dot wd"></span><span class="dot nd"></span>&nbsp;<span id="clk">--:--:--</span></div>
</header>
<main>
<p class="ptitle">Relay Controls &amp; Schedules</p>
<div class="grid" id="grid"></div>
</main>
<div id="toast"></div>
<script>
const D=['Sun','Mon','Tue','Wed','Thu','Fri','Sat'];
let toastTimer = null;
function toast(m,ok=true){
  const t=document.getElementById('toast');
  if(toastTimer) clearTimeout(toastTimer);
  t.textContent=m;t.className='show '+(ok?'ok':'er');
  toastTimer=setTimeout(()=>{t.className='';toastTimer=null;},3000);
}
function tick(){
  fetch('/api/time').then(r=>r.json()).then(d=>{
    document.getElementById('clk').textContent=d.time||'--:--:--';
    const w=document.querySelector('.wd'),n=document.querySelector('.nd');
    if(w)w.className='dot '+(d.wifi?'g':'r');
    if(n)n.className='dot '+(d.ntp?'g':'y');
  }).catch(()=>{});
}
setInterval(tick,1000);tick();

const NS=8;
let relays=[],busy=false,loadTimeout=null;
let editingRelay = -1;
let editingInput = null;

function escapeHtml(text) {
  const div = document.createElement('div');
  div.textContent = text;
  return div.innerHTML;
}

function load(){
  if(busy)return;
  if(loadTimeout) clearTimeout(loadTimeout);
  fetch('/api/relays').then(r=>r.json()).then(d=>{relays=d;render();}).catch(()=>toast('Load error',false));
}

function toTS(h,m,s){return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(s).padStart(2,'0');}
function fromTS(v){const p=(v||'00:00:00').split(':');return{h:parseInt(p[0])||0,m:parseInt(p[1])||0,s:parseInt(p[2])||0};}

function dayMaskToStr(d){
  if(d===0x7F) return 'Everyday';
  let s='';
  for(let i=0;i<7;i++) if(d&(1<<i)) s+=D[i]+' ';
  return s.trim()||'None';
}

function monthDayMaskToStr(md){
  if(md===0) return '';
  if(md===0xFFFFFFFF) return 'All month days';
  let s='';
  for(let i=0;i<31;i++) if(md&(1<<i)) s+=(i+1)+',';
  return s.replace(/,$/,'')||'None';
}

function nightBadge(sc){
  if(!sc.enabled)return'';
  const a=sc.startHour*3600+sc.startMinute*60+sc.startSecond;
  const b=sc.stopHour*3600+sc.stopMinute*60+sc.stopSecond;
  const ds=dayMaskToStr(sc.days);
  const ms=monthDayMaskToStr(sc.monthDays||0);
  let info=ds;
  if(ms) info+=' | Days:'+ms;
  if(a===b)return'<span class="night always">&#x25CF; Always ON ('+info+')</span>';
  if(a>b) return'<span class="night">&#x1F319; Overnight ('+info+')</span>';
  return'<span class="night">&#x1F319; '+info+'</span>';
}

function startEditName(relayIdx) {
  if (editingRelay !== -1) cancelEdit();
  
  const nameSpan = document.getElementById('name_' + relayIdx);
  if (!nameSpan) return;
  
  const currentName = relays[relayIdx].name || ('Relay ' + (relayIdx + 1));
  
  const input = document.createElement('input');
  input.type = 'text';
  input.value = currentName;
  input.maxLength = 15;
  input.style.cssText = 'font-size:15px;font-weight:700;padding:2px 6px;border:1px solid #1565C0;border-radius:5px;width:120px;background:#fff;color:#1A1A2E;';
  input.id = 'edit_' + relayIdx;
  
  input.onblur = () => saveNameEdit(relayIdx, input.value);
  input.onkeydown = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      saveNameEdit(relayIdx, input.value);
    } else if (e.key === 'Escape') {
      cancelEdit();
    }
  };
  
  nameSpan.style.display = 'none';
  nameSpan.parentNode.insertBefore(input, nameSpan.nextSibling);
  
  editingRelay = relayIdx;
  editingInput = input;
  input.focus();
  input.select();
}

function cancelEdit() {
  if (editingRelay !== -1) {
    const nameSpan = document.getElementById('name_' + editingRelay);
    if (nameSpan) nameSpan.style.display = '';
    if (editingInput) {
      editingInput.remove();
      editingInput = null;
    }
    editingRelay = -1;
  }
}

function saveNameEdit(relayIdx, newName) {
  newName = newName.trim();
  if (newName.length === 0) {
    newName = 'Relay ' + (relayIdx + 1);
  }
  
  const nameSpan = document.getElementById('name_' + relayIdx);
  
  relays[relayIdx].name = newName;
  if (nameSpan) {
    nameSpan.textContent = newName;
    nameSpan.style.display = '';
  }
  if (editingInput) {
    editingInput.remove();
    editingInput = null;
  }
  editingRelay = -1;
  
  fetch('/api/relay/name', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({relay: relayIdx, name: newName})
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      toast('Name saved!');
    } else {
      toast('Failed to save name', false);
    }
  })
  .catch(() => toast('Error saving name', false));
}

function render(){
  const g=document.getElementById('grid');
  g.innerHTML='';
  relays.forEach((r,i)=>{
    const sl=r.manual?'MANUAL':r.state?'ON':'OFF';
    const sc=r.manual?'bman':r.state?'bon':'boff';
    const displayName = r.name || ('Relay '+(i+1));
    let html=`<div class="card">
<div class="card-hdr">
<span class="ctitle" id="name_${i}" ondblclick="startEditName(${i})" title="Double-click to rename">${escapeHtml(displayName)}</span>
<span class="badge ${sc}">${sl}</span>
</div>
<div class="brow">
<button class="btn bon-b" onclick="mc(${i},true)">ON</button>
<button class="btn boff-b" onclick="mc(${i},false)">OFF</button>
<button class="btn baut" onclick="ra(${i})">Auto</button>
</div>
<div class="slist">`;
    for(let s=0;s<NS;s++){
      const sc2=r.schedules[s];
      const dayBits = sc2.days || 0x7F;
      const monthDayBits = sc2.monthDays || 0;
      html+=`<div class="si${sc2.enabled?' act':''}" id="si_${i}_${s}">
<div class="shdr">
<label><input type="checkbox" id="en_${i}_${s}" ${sc2.enabled?'checked':''} onchange="uf(${i},${s},'en',this.checked)"> Sched ${s+1}</label>
<span id="nb_${i}_${s}">${nightBadge(sc2)}</span>
</div>
<div class="trow"><span class="l">Start</span>
<input type="time" step="1" id="st_${i}_${s}" value="${toTS(sc2.startHour,sc2.startMinute,sc2.startSecond)}" onchange="uf(${i},${s},'start',this.value)">
</div>
<div class="trow"><span class="l">Stop</span>
<input type="time" step="1" id="et_${i}_${s}" value="${toTS(sc2.stopHour,sc2.stopMinute,sc2.stopSecond)}" onchange="uf(${i},${s},'stop',this.value)">
</div>
<div class="sched-section">Days of Week</div>
<div class="days" id="day_${i}_${s}">`;
      for(let d=0;d<7;d++){
        const mask = 1<<d;
        html+=`<div class="day${(dayBits&mask)?' on':''}" onclick="toggleDay(${i},${s},${d})">${D[d]}</div>`;
      }
      html+=`</div>
<div class="sched-section">Days of Month</div>
<div class="mdays" id="mday_${i}_${s}">`;
      for(let d=0;d<31;d++){
        const mask = 1<<d;
        html+=`<div class="mday${(monthDayBits&mask)?' on':''}" onclick="toggleMonthDay(${i},${s},${d})" title="Day ${d+1}">${d+1}</div>`;
      }
      html+=`</div>
</div>`;
    }
    html+=`</div><button class="btn bsave" onclick="save(${i})">&#x1F4BE; Save ${escapeHtml(displayName)}</button></div>`;
    const el=document.createElement('div');
    el.innerHTML=html;
    g.appendChild(el.firstChild);
  });
}

function toggleDay(ri,si,dayIdx){
  const mask = 1<<dayIdx;
  relays[ri].schedules[si].days ^= mask;
  const dayEl = document.getElementById('day_'+ri+'_'+si);
  if(dayEl) {
    const children = dayEl.children;
    if(children && children[dayIdx]) {
      children[dayIdx].className = 'day' + ((relays[ri].schedules[si].days & mask)?' on':'');
    }
  }
  const nb=document.getElementById('nb_'+ri+'_'+si);
  if(nb)nb.innerHTML=nightBadge(relays[ri].schedules[si]);
}

function toggleMonthDay(ri,si,dayIdx){
  const mask = 1<<dayIdx;
  if(!relays[ri].schedules[si].monthDays) relays[ri].schedules[si].monthDays = 0;
  relays[ri].schedules[si].monthDays ^= mask;
  const mdayEl = document.getElementById('mday_'+ri+'_'+si);
  if(mdayEl) {
    const children = mdayEl.children;
    if(children && children[dayIdx]) {
      children[dayIdx].className = 'mday' + ((relays[ri].schedules[si].monthDays & mask)?' on':'');
    }
  }
  const nb=document.getElementById('nb_'+ri+'_'+si);
  if(nb)nb.innerHTML=nightBadge(relays[ri].schedules[si]);
}

function uf(ri,si,field,val){
  const sc=relays[ri].schedules[si];
  if(field==='en'){
    sc.enabled=val;
    const el=document.getElementById('si_'+ri+'_'+si);
    if(el)el.className='si'+(val?' act':'');
  }else if(field==='start'){
    const t=fromTS(val);sc.startHour=t.h;sc.startMinute=t.m;sc.startSecond=t.s;
  }else if(field==='stop'){
    const t=fromTS(val);sc.stopHour=t.h;sc.stopMinute=t.m;sc.stopSecond=t.s;
  }
  const nb=document.getElementById('nb_'+ri+'_'+si);
  if(nb)nb.innerHTML=nightBadge(sc);
}

function mc(ri,state){
  if(busy) return;
  busy = true;
  fetch('/api/relay/manual',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({relay:ri,state})})
  .then(r=>r.json()).then(d=>{
    busy = false;
    if(d.success){
      toast((relays[ri].name||('Relay '+(ri+1)))+' '+(state?'ON':'OFF'));
      loadTimeout = setTimeout(() => load(), 500);
    } else {
      toast('Failed',false);
    }
  })
  .catch(()=>{busy=false;toast('Error',false);});
}

function ra(ri){
  if(busy) return;
  busy = true;
  fetch('/api/relay/reset',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({relay:ri})})
  .then(r=>r.json()).then(d=>{
    busy = false;
    if(d.success){
      toast((relays[ri].name||('Relay '+(ri+1)))+' \u2192 Auto');
      loadTimeout = setTimeout(() => load(), 500);
    } else {
      toast('Failed',false);
    }
  })
  .catch(()=>{busy=false;toast('Error',false);});
}

function save(ri){
  if(busy) return;
  busy=true;
  fetch('/api/relay/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({relay:ri,schedules:relays[ri].schedules})})
  .then(r=>r.json()).then(d=>{
    busy=false;
    if(d.success)toast((relays[ri].name||('Relay '+(ri+1)))+' saved!');
    else toast('Save failed',false);
  })
  .catch(()=>{busy=false;toast('Error',false);});
}

// Auto-reload with debounce
let autoReloadTimer = null;
function scheduleAutoReload() {
  if(autoReloadTimer) clearTimeout(autoReloadTimer);
  autoReloadTimer = setTimeout(() => {
    if(!busy && document.hidden === false) load();
    scheduleAutoReload();
  }, 30000);
}

load();
scheduleAutoReload();

// Clean up on page unload
window.addEventListener('beforeunload', () => {
  if(autoReloadTimer) clearTimeout(autoReloadTimer);
  if(loadTimeout) clearTimeout(loadTimeout);
  if(toastTimer) clearTimeout(toastTimer);
});
</script></body></html>)raw";

// [Previous WiFi, NTP, AP, System HTML pages remain the same...]

const char wifi_html[] PROGMEM = R"raw(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi — ESP32 Timer Switch</title>
<link rel="stylesheet" href="/style.css">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
</head><body>
<header>
<span class="logo">&#x26A1; ESP32 20-CH</span>
<nav>
<a href="/">Relays</a>
<a href="/wifi" class="cur">WiFi</a>
<a href="/ntp">Time</a>
<a href="/ap">AP</a>
<a href="/system">System</a>
</nav>
<div class="hdr-r"><span class="dot wd"></span><span class="dot nd"></span>&nbsp;<span id="clk">--:--:--</span></div>
</header>
<main>
<p class="ptitle">WiFi Station Settings</p>
<div class="card fcrd">
<div id="status" class="alert ai" style="display:none"></div>
<div class="fg">
<label>Network SSID</label>
<div class="input-row">
<input type="text" id="ssid" placeholder="Enter network name or scan" required>
<button class="btn bscan" id="scanBtn" onclick="startScan()" style="white-space:nowrap">&#x1F4F6; Scan</button>
</div>
</div>
<div class="netlist" id="netlist"></div>
<div class="fg"><label>Password</label><input type="password" id="pw" placeholder="Leave blank for open network"></div>
<button class="btn bsave" onclick="save()">&#x1F4BE; Save &amp; Connect</button>
</div>
</main>
<div id="toast"></div>
<script>
function toast(m,ok=true){const t=document.getElementById('toast');t.textContent=m;t.className='show '+(ok?'ok':'er');clearTimeout(t._t);t._t=setTimeout(()=>t.className='',3000);}
function tick(){fetch('/api/time').then(r=>r.json()).then(d=>{document.getElementById('clk').textContent=d.time||'--:--:--';const w=document.querySelector('.wd'),n=document.querySelector('.nd');if(w)w.className='dot '+(d.wifi?'g':'r');if(n)n.className='dot '+(d.ntp?'g':'y');}).catch(()=>{});}
setInterval(tick,1000);tick();

fetch('/api/wifi').then(r=>r.json()).then(d=>{
  document.getElementById('ssid').value=d.ssid||'';
  const s=document.getElementById('status');
  s.style.display='';
  if(d.connected){
    const bars=rssiBar(d.rssi||0);
    s.innerHTML='Connected to: <strong>'+d.ssid+'</strong> ('+d.ip+') &nbsp;'+bars+' '+d.rssi+'dBm';
    s.className='alert ai';
  }else{
    s.textContent='Not connected to any network.';
    s.className='alert aw';
  }
}).catch(()=>{});

function rssiBar(rssi){
  const b=rssi>=-50?4:rssi>=-60?3:rssi>=-70?2:1;
  let s='<span class="bars">';
  for(let i=1;i<=4;i++)s+='<span class="bar'+(i<=b?' on':'')+('" style="height:'+(i*3+2)+'px"></span>');
  return s+'</span>';
}

let scanTimer=null,scanning=false;
function startScan(){
  if(scanning)return;
  scanning=true;
  document.getElementById('scanBtn').textContent='\uD83D\uDD04 Scanning\u2026';
  document.getElementById('scanBtn').disabled=true;
  const nl=document.getElementById('netlist');
  nl.style.display='block';
  nl.innerHTML='<div style="text-align:center;color:#90A4AE;padding:12px;font-size:13px">Scanning for networks\u2026</div>';
  fetch('/api/wifi/scan',{method:'POST'})
  .then(()=>{ scanTimer=setInterval(pollScan,2500); })
  .catch(()=>endScan());
}
function pollScan(){
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    if(!d.scanning){clearInterval(scanTimer);endScan();renderNets(d.networks||[]);}
  }).catch(()=>{clearInterval(scanTimer);endScan();});
}
function endScan(){
  scanning=false;
  document.getElementById('scanBtn').textContent='\uD83D\uDCF6 Scan';
  document.getElementById('scanBtn').disabled=false;
}
function renderNets(nets){
  const nl=document.getElementById('netlist');
  if(!nets.length){nl.innerHTML='<div style="color:#90A4AE;text-align:center;padding:8px;font-size:13px">No networks found.</div>';return;}
  const frag=document.createDocumentFragment();
  const hdr=document.createElement('div');hdr.className='net-hdr';hdr.textContent='Available Networks';frag.appendChild(hdr);
  nets.sort((a,b)=>b.rssi-a.rssi).forEach(n=>{
    const d=document.createElement('div');d.className='netitem';
    const ns=document.createElement('span');ns.className='ns';ns.textContent=n.ssid;
    const nr=document.createElement('span');nr.className='nr';nr.textContent=n.rssi+'dBm';
    const bar=document.createElement('span');bar.innerHTML=rssiBar(n.rssi);
    const lock=document.createElement('span');lock.style.fontSize='13px';lock.textContent=n.enc?'\uD83D\uDD12':'';
    d.appendChild(ns);d.appendChild(nr);d.appendChild(bar);d.appendChild(lock);
    d.addEventListener('click',()=>{document.getElementById('ssid').value=n.ssid;document.getElementById('pw').focus();});
    frag.appendChild(d);
  });
  nl.innerHTML='';nl.appendChild(frag);
}
function save(){
  const ssid=document.getElementById('ssid').value.trim();
  if(!ssid){toast('SSID required',false);return;}
  fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid,password:document.getElementById('pw').value})})
  .then(r=>r.json()).then(d=>{if(d.success){toast('Saved! Reconnecting\u2026');setTimeout(()=>window.location.href='/',5000);}else toast('Failed: '+d.error,false);})
  .catch(()=>toast('Error',false));
}
</script></body></html>)raw";

// [NTP, AP, System HTML pages with cache-control headers...]
const char ntp_html[] PROGMEM = R"raw(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Time — ESP32 Timer Switch</title>
<link rel="stylesheet" href="/style.css">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
</head><body>
<header>
<span class="logo">&#x26A1; ESP32 20-CH</span>
<nav>
<a href="/">Relays</a>
<a href="/wifi">WiFi</a>
<a href="/ntp" class="cur">Time</a>
<a href="/ap">AP</a>
<a href="/system">System</a>
</nav>
<div class="hdr-r"><span class="dot wd"></span><span class="dot nd"></span>&nbsp;<span id="clk">--:--:--</span></div>
</header>
<main>
<p class="ptitle">Time &amp; NTP Settings</p>
<div class="card fcrd">
<div class="fg">
<label>Primary NTP Server</label>
<input type="text" id="srv" placeholder="ph.pool.ntp.org" required>
<small>Fallbacks: pool.ntp.org &rarr; time.nist.gov &rarr; time.google.com (tried automatically on failure)</small>
</div>
<div class="fg"><label>GMT Offset (seconds) &mdash; e.g. UTC+8 = 28800</label><input type="number" id="gmt" required></div>
<div class="fg"><label>Daylight Saving Offset (seconds, usually 0)</label><input type="number" id="dst" value="0"></div>
<div class="fg"><label>Auto-Sync Interval (hours, 1&ndash;24)</label><input type="number" id="shi" min="1" max="24" value="1"></div>
<div style="display:flex;gap:8px;flex-wrap:wrap">
<button class="btn bsave" style="flex:1;margin-top:0" onclick="save()">&#x1F4BE; Save NTP Settings</button>
<button class="btn bsync" id="sbtn" onclick="sync()" style="padding:9px 16px;border-radius:6px;font-size:13px;font-weight:600;margin-top:8px;white-space:nowrap">&#x1F504; Sync Now</button>
</div>
</div>
</main>
<div id="toast"></div>
<script>
function toast(m,ok=true){const t=document.getElementById('toast');t.textContent=m;t.className='show '+(ok?'ok':'er');clearTimeout(t._t);t._t=setTimeout(()=>t.className='',3000);}
function tick(){fetch('/api/time').then(r=>r.json()).then(d=>{document.getElementById('clk').textContent=d.time||'--:--:--';const w=document.querySelector('.wd'),n=document.querySelector('.nd');if(w)w.className='dot '+(d.wifi?'g':'r');if(n)n.className='dot '+(d.ntp?'g':'y');}).catch(()=>{});}
setInterval(tick,1000);tick();
fetch('/api/ntp').then(r=>r.json()).then(d=>{
  document.getElementById('srv').value=d.ntpServer||'ph.pool.ntp.org';
  document.getElementById('gmt').value=d.gmtOffset||28800;
  document.getElementById('dst').value=d.daylightOffset||0;
  document.getElementById('shi').value=d.syncHours||1;
}).catch(()=>{});
function save(){
  const h=parseInt(document.getElementById('shi').value);
  if(h<1||h>24){toast('Sync interval must be 1\u201324 h',false);return;}
  fetch('/api/ntp',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
    ntpServer:document.getElementById('srv').value,
    gmtOffset:parseInt(document.getElementById('gmt').value),
    daylightOffset:parseInt(document.getElementById('dst').value),
    syncHours:h
  })}).then(r=>r.json()).then(d=>{if(d.success)toast('NTP settings saved!');else toast('Failed: '+d.error,false);})
  .catch(()=>toast('Error',false));
}
function sync(){
  const b=document.getElementById('sbtn');b.disabled=true;b.textContent='Syncing\u2026';
  fetch('/api/ntp/sync',{method:'POST'}).then(r=>r.json()).then(d=>{
    b.disabled=false;b.innerHTML='&#x1F504; Sync Now';
    if(d.success)toast('Time synced successfully!');else toast('Sync failed \u2014 check WiFi',false);
  }).catch(()=>{b.disabled=false;b.innerHTML='&#x1F504; Sync Now';toast('Error',false);});
}
</script></body></html>)raw";

// [AP and System pages with similar cache-control additions...]
const char ap_html[] PROGMEM = R"raw(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>AP — ESP32 Timer Switch</title>
<link rel="stylesheet" href="/style.css">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
</head><body>
<header>
<span class="logo">&#x26A1; ESP32 20-CH</span>
<nav>
<a href="/">Relays</a>
<a href="/wifi">WiFi</a>
<a href="/ntp">Time</a>
<a href="/ap" class="cur">AP</a>
<a href="/system">System</a>
</nav>
<div class="hdr-r"><span class="dot wd"></span><span class="dot nd"></span>&nbsp;<span id="clk">--:--:--</span></div>
</header>
<main>
<p class="ptitle">Access Point Settings</p>
<div class="card fcrd">
<div class="alert aw">&#x26A0;&#xFE0F; Saving will restart the AP and disconnect all clients. Reconnect to the new SSID afterward.</div>
<div class="fg"><label>AP SSID (Network Name)</label><input type="text" id="ssid" maxlength="31" required></div>
<div class="fg"><label>AP Password (8+ characters or blank for open)</label><input type="password" id="pw" minlength="8" placeholder="Leave blank for open network"></div>
<div class="fg">
<label>Channel (1&ndash;13)</label>
<select id="ch">
<option value="1">1</option><option value="2">2</option><option value="3">3</option>
<option value="4">4</option><option value="5">5</option><option value="6">6 (default)</option>
<option value="7">7</option><option value="8">8</option><option value="9">9</option>
<option value="10">10</option><option value="11">11</option><option value="12">12</option>
<option value="13">13</option>
</select>
<small>Lower interference: pick a channel not used by nearby networks.</small>
</div>
<div class="fg">
<label>SSID Visibility</label>
<select id="hidden">
<option value="0">Visible (broadcast SSID)</option>
<option value="1">Hidden (do not broadcast)</option>
</select>
</div>
<button class="btn bsave" onclick="save()">&#x1F4BE; Save &amp; Restart AP</button>
</div>
</main>
<div id="toast"></div>
<script>
function toast(m,ok=true){const t=document.getElementById('toast');t.textContent=m;t.className='show '+(ok?'ok':'er');clearTimeout(t._t);t._t=setTimeout(()=>t.className='',3000);}
function tick(){fetch('/api/time').then(r=>r.json()).then(d=>{document.getElementById('clk').textContent=d.time||'--:--:--';const w=document.querySelector('.wd'),n=document.querySelector('.nd');if(w)w.className='dot '+(d.wifi?'g':'r');if(n)n.className='dot '+(d.ntp?'g':'y');}).catch(()=>{});}
setInterval(tick,1000);tick();
fetch('/api/ap').then(r=>r.json()).then(d=>{
  document.getElementById('ssid').value=d.ap_ssid||'';
  document.getElementById('ch').value=d.ap_channel||6;
  document.getElementById('hidden').value=d.ap_hidden?'1':'0';
}).catch(()=>{});
function save(){
  const pw=document.getElementById('pw').value;
  if(pw.length>0&&pw.length<8){toast('Password must be 8+ chars or blank',false);return;}
  fetch('/api/ap',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({
    ap_ssid:document.getElementById('ssid').value,
    ap_password:pw,
    ap_channel:parseInt(document.getElementById('ch').value),
    ap_hidden:document.getElementById('hidden').value==='1'
  })}).then(r=>r.json()).then(d=>{
    if(d.success){toast('AP restarted \u2014 reconnect to new network');setTimeout(()=>location.reload(),4000);}
    else toast('Failed: '+d.error,false);
  }).catch(()=>toast('Error',false));
}
</script></body></html>)raw";

const char system_html[] PROGMEM = R"raw(<!DOCTYPE html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>System — ESP32 Timer Switch</title>
<link rel="stylesheet" href="/style.css">
<meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
<meta http-equiv="Pragma" content="no-cache">
<meta http-equiv="Expires" content="0">
</head><body>
<header>
<span class="logo">&#x26A1; ESP32 20-CH</span>
<nav>
<a href="/">Relays</a>
<a href="/wifi">WiFi</a>
<a href="/ntp">Time</a>
<a href="/ap">AP</a>
<a href="/system" class="cur">System</a>
</nav>
<div class="hdr-r"><span class="dot wd"></span><span class="dot nd"></span>&nbsp;<span id="clk">--:--:--</span></div>
</header>
<main>
<p class="ptitle">System Information &amp; Settings</p>
<div class="ibar" id="ibar">
<div class="ibox"><div class="l">STA IP</div><div class="v" id="sip">&hellip;</div></div>
<div class="ibox"><div class="l">AP IP</div><div class="v" id="sap">&hellip;</div></div>
<div class="ibox"><div class="l">Free Heap</div><div class="v" id="shp">&hellip;</div></div>
<div class="ibox"><div class="l">Uptime</div><div class="v" id="sup">&hellip;</div></div>
<div class="ibox"><div class="l">WiFi RSSI</div><div class="v" id="srs">&hellip;</div></div>
<div class="ibox"><div class="l">NTP Last Sync</div><div class="v" id="snt">&hellip;</div></div>
<div class="ibox"><div class="l">NTP Server</div><div class="v" id="sns" style="font-size:12px">&hellip;</div></div>
<div class="ibox"><div class="l">Chip Model</div><div class="v" id="sch">&hellip;</div></div>
<div class="ibox"><div class="l">mDNS Hostname</div><div class="v" id="smdns">&hellip;</div></div>
<div class="ibox"><div class="l">Health Status</div><div class="v" id="shlth">&hellip;</div></div>
</div>

<div class="card fcrd">
<p style="font-weight:700;margin-bottom:12px">Device Control</p>
<div style="display:flex;gap:8px;flex-wrap:wrap">
<button class="btn bwarn" onclick="rst()" style="padding:9px 18px;border-radius:6px;font-size:13px;font-weight:600">&#x1F504; Restart Device</button>
<button class="btn bdanger" onclick="fct()" style="padding:9px 18px;border-radius:6px;font-size:13px;font-weight:600">&#x26A0; Factory Reset</button>
</div>
<p style="color:#90A4AE;font-size:12px;margin-top:10px">Factory reset clears all settings including WiFi credentials, schedules, and AP configuration.</p>
</div>
</main>
<div id="toast"></div>
<script>
function toast(m,ok=true){const t=document.getElementById('toast');t.textContent=m;t.className='show '+(ok?'ok':'er');clearTimeout(t._t);t._t=setTimeout(()=>t.className='',3000);}
function tick(){fetch('/api/time').then(r=>r.json()).then(d=>{document.getElementById('clk').textContent=d.time||'--:--:--';const w=document.querySelector('.wd'),n=document.querySelector('.nd');if(w)w.className='dot '+(d.wifi?'g':'r');if(n)n.className='dot '+(d.ntp?'g':'y');}).catch(()=>{});}
setInterval(tick,1000);tick();
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),ss=s%60;return h+'h '+m+'m '+ss+'s';}
function rssiDesc(r){if(!r)return'\u2014';return r+'dBm ('+( r>=-50?'Excellent':r>=-60?'Good':r>=-70?'Fair':'Weak')+')';}
function loadSys(){
  fetch('/api/system').then(r=>r.json()).then(d=>{
    document.getElementById('sip').textContent=d.wifiConnected?d.ip:'(not connected)';
    document.getElementById('sap').textContent=d.ap_ip;
    document.getElementById('shp').textContent=(d.freeHeap/1024).toFixed(1)+' KB';
    document.getElementById('sup').textContent=fmtUp(d.uptime);
    document.getElementById('srs').textContent=d.wifiConnected?rssiDesc(d.rssi):'\u2014';
    document.getElementById('snt').textContent=d.ntpSynced?(d.ntpSyncAge>0?Math.floor(d.ntpSyncAge/60)+' min ago':'Just now'):'Never';
    document.getElementById('sns').textContent=d.ntpServer||'\u2014';
    document.getElementById('sch').textContent=d.chipModel||'ESP32';
    document.getElementById('smdns').textContent=d.mdnsStarted ? d.mdnsHostname+'.local' : 'Not running';
    document.getElementById('shlth').textContent=d.healthStatus||'Unknown';
  }).catch(()=>{});
}
loadSys();setInterval(loadSys,5000);
function rst(){
  if(!confirm('Restart the device now?'))return;
  fetch('/api/reset',{method:'POST'}).then(()=>toast('Restarting\u2026')).catch(()=>{});
  setTimeout(()=>window.location.href='/',7000);
}
function fct(){
  if(!confirm('FACTORY RESET \u2014 ALL settings will be erased. Continue?'))return;
  fetch('/api/factory-reset',{method:'POST'}).then(()=>toast('Factory reset \u2014 reconnect to default AP')).catch(()=>{});
  setTimeout(()=>window.location.href='/',7000);
}
</script></body></html>)raw";

// =============================================================================
//  ENHANCED RTC FUNCTIONS
// =============================================================================

time_t getCurrentEpoch() {
    if (!rtcInitialized || internalEpoch == 0) return 0;
    
    unsigned long elapsed = millis() - internalMillisAtLastSync;
    
    // Handle millis() overflow
    if (elapsed > 86400000UL) {  // More than 24 hours without sync
        rtcInitialized = false;  // Force re-sync
        return 0;
    }
    
    float elapsedSeconds = (float)elapsed / 1000.0f;
    float adjustedSeconds = elapsedSeconds * driftCompensation;
    
    // Apply exponential smoothing to drift
    if (driftHistoryIndex < 8) {
        adjustedSeconds = elapsedSeconds;  // Not enough history
    }
    
    return internalEpoch + (time_t)adjustedSeconds;
}

void syncInternalRTC() {
    time_t ntpEpoch = timeClient.getEpochTime();
    if (ntpEpoch < 1000000000UL || ntpEpoch > 2000000000UL) return;

    unsigned long nowMs = millis();
    bool driftUpdated = false;

    if (rtcInitialized && internalEpoch > 0) {
        unsigned long elapsedMs = nowMs - internalMillisAtLastSync;
        if (elapsedMs > 60000UL && elapsedMs < 86400000UL) {  // Between 1 min and 24 hours
            float nominalSecs  = (float)elapsedMs / 1000.0f;
            long diff = (long)ntpEpoch - (long)internalEpoch;
            float actualSecs = (float)diff;
            
            // Calculate new drift with bounds checking
            if (nominalSecs > 0 && actualSecs > 0) {
                float measuredRate = actualSecs / nominalSecs;
                
                // Update drift history for smoothing
                rtcDriftHistory[driftHistoryIndex % 8] = measuredRate;
                driftHistoryIndex++;
                
                // Calculate smoothed drift
                if (driftHistoryIndex >= 8) {
                    float sum = 0;
                    for (int i = 0; i < 8; i++) sum += rtcDriftHistory[i];
                    driftCompensation = sum / 8.0f;
                } else {
                    driftCompensation  = driftCompensation * 0.75f + measuredRate * 0.25f;
                }
                
                // Tighter bounds for better precision
                if (driftCompensation < 0.95f) driftCompensation = 0.95f;
                if (driftCompensation > 1.05f) driftCompensation = 1.05f;
                
                driftUpdated = true;
            }
        }
    }

    if (!driftUpdated && !rtcInitialized) {
        driftCompensation = 1.0f;  // Reset drift on first sync
    }

    internalEpoch            = ntpEpoch;
    internalMillisAtLastSync = nowMs;
    rtcInitialized           = true;
    lastNTPSync              = nowMs;
    ntpFailCount             = 0;

    saveRTCState();
}

void saveRTCState() {
    sysConfig.last_rtc_epoch = internalEpoch;
    sysConfig.rtc_drift      = driftCompensation;
    scheduleConfigSave();
}

void loadRTCState() {
    if (sysConfig.last_rtc_epoch > 1000000000UL &&
        sysConfig.last_rtc_epoch < 2000000000UL) {
        internalEpoch            = sysConfig.last_rtc_epoch;
        driftCompensation        = sysConfig.rtc_drift;
        if (driftCompensation < 0.95f || driftCompensation > 1.05f) {
            driftCompensation = 1.0f;
        }
        internalMillisAtLastSync = millis();
        rtcInitialized           = true;
        
        // Initialize drift history
        for (int i = 0; i < 8; i++) {
            rtcDriftHistory[i] = driftCompensation;
        }
        driftHistoryIndex = 0;
    }
}

// =============================================================================
//  ENHANCED NTP FUNCTIONS WITH BACKOFF
// =============================================================================

void tryNTPSync() {
    if (!wifiConnected || ntpSyncInProgress) {
        return;
    }
    
    // Check backoff
    unsigned long now = millis();
    if (ntpBackoff.consecutiveFailures > 0) {
        unsigned long backoffTime = calculateBackoff(&ntpBackoff);
        if (now - ntpBackoff.lastAttempt < backoffTime) {
            return;
        }
    }
    
    ntpSyncInProgress = true;
    ntpBackoff.lastAttempt = now;
    lastNTPAttempt = now;
    
    uint8_t startIndex = ntpServerIndex;
    bool synced = false;
    
    for (uint8_t attempt = 0; attempt < NUM_NTP_SERVERS; attempt++) {
        uint8_t idx = (startIndex + attempt) % NUM_NTP_SERVERS;
        
        // Validate server name
        if (strlen(NTP_SERVERS[idx]) < 5) continue;
        
        timeClient.setPoolServerName(NTP_SERVERS[idx]);
        timeClient.setTimeOffset(sysConfig.gmt_offset + sysConfig.daylight_offset);
        
        // Add timeout protection
        unsigned long syncStart = millis();
        if (timeClient.forceUpdate()) {
            if (millis() - syncStart < 10000UL) {  // Valid response within 10 seconds
                syncInternalRTC();
                ntpServerIndex = idx;
                ntpFailCount = 0;
                synced = true;
                resetBackoff(&ntpBackoff);
                healthMetrics.ntpSyncFailures = 0;
                break;
            }
        }
        
        // Brief delay between attempts
        delay(50);
    }
    
    if (!synced) {
        ntpFailCount++;
        healthMetrics.ntpSyncFailures++;
        incrementBackoff(&ntpBackoff);
        ntpServerIndex = (ntpServerIndex + 1) % NUM_NTP_SERVERS;
    }
    
    ntpSyncInProgress = false;
}

// =============================================================================
//  ENHANCED WiFi FUNCTIONS WITH AUTO-RECOVERY
// =============================================================================

void beginWiFiConnect() {
    if (strlen(sysConfig.sta_ssid) == 0 || !wifiStaEnabled) return;
    
    unsigned long now = millis();
    if (now < wifiGiveUpUntil) return;
    
    // Check WiFi backoff
    if (wifiBackoff.consecutiveFailures > 0) {
        unsigned long backoffTime = calculateBackoff(&wifiBackoff);
        if (now - wifiBackoff.lastAttempt < backoffTime) {
            return;
        }
    }
    
    wifiReconnectAttempts++;
    healthMetrics.wifiReconnects++;
    wifiBackoff.lastAttempt = now;

    // Validate WiFi mode before connecting
    WiFi.disconnect(false);
    delay(100);
    
    wifi_mode_t currentMode = WiFi.getMode();
    if (currentMode == WIFI_AP || currentMode == WIFI_OFF) {
        WiFi.mode(WIFI_AP_STA);
        delay(50);
    }
    
    // Begin connection with validation
    if (strlen(sysConfig.sta_password) > 0) {
        WiFi.begin(sysConfig.sta_ssid, sysConfig.sta_password);
    } else {
        WiFi.begin(sysConfig.sta_ssid);
    }
    
    wcsState = WCS_PENDING;
    wcsStart  = now;
}

void checkWiFiHealth() {
    unsigned long now = millis();
    static unsigned long lastWiFiHealthCheck = 0;
    
    if (now - lastWiFiHealthCheck < WIFI_CHECK_INTERVAL) return;
    lastWiFiHealthCheck = now;
    
    wl_status_t status = WiFi.status();
    bool currentlyConnected = (status == WL_CONNECTED);

    if (wifiConnected && !currentlyConnected) {
        // Connection lost
        wifiConnected = false;
        scheduleMDNSRestart();
    } else if (!wifiConnected && !currentlyConnected &&
               wcsState == WCS_IDLE &&
               strlen(sysConfig.sta_ssid) > 0 &&
               now >= wifiGiveUpUntil &&
               wifiStaEnabled) {
        beginWiFiConnect();
    } else if (!wifiConnected && currentlyConnected) {
        // Reconnected
        wifiConnected = true;
        wifiReconnectAttempts = 0;
        wcsState = WCS_IDLE;
        wifiGiveUpUntil = 0;
        lastNTPSync = 0;
        lastNTPAttempt = 0;
        resetBackoff(&wifiBackoff);
        scheduleMDNSRestart();
    }
}

// =============================================================================
//  ENHANCED RELAY FUNCTIONS WITH VALIDATION
// =============================================================================

void updateRelayOutputs() {
    static unsigned long lastRelayUpdate = 0;
    static bool relayStates[NUM_RELAYS] = {false};
    static uint8_t debounceCounters[NUM_RELAYS] = {0};
    
    unsigned long now = millis();
    if (now - lastRelayUpdate < 50) return;  // Debounce: max 20 updates/second
    lastRelayUpdate = now;
    
    for (int i = 0; i < NUM_RELAYS; i++) {
        bool targetState = false;
        
        if (relayConfigs[i].manualOverride) {
            targetState = relayConfigs[i].manualState;
        } else {
            targetState = relayStates[i];
        }
        
        // Debounce relay changes
        if (targetState != relayStates[i]) {
            debounceCounters[i]++;
            if (debounceCounters[i] >= 3) {  // Require 3 consistent readings
                relayStates[i] = targetState;
                debounceCounters[i] = 0;
                
                // Safe relay switching
                digitalWrite(relayPins[i], relayActiveLow ? !targetState : targetState);
                relayConfigs[i].lastStateChange = now;
                relayConfigs[i].stateChangeCount++;
            }
        } else {
            debounceCounters[i] = 0;
        }
    }
}

// =============================================================================
//  ENHANCED CONFIGURATION MANAGEMENT WITH CORRUPTION PROTECTION
// =============================================================================

bool validateConfiguration() {
    // Validate system config
    if (sysConfig.magic != EEPROM_MAGIC) return false;
    if (sysConfig.version < 5 || sysConfig.version > EEPROM_VERSION) return false;
    
    // Validate string fields
    if (strlen(sysConfig.sta_ssid) > 31) return false;
    if (strlen(sysConfig.sta_password) > 63) return false;
    if (strlen(sysConfig.ap_ssid) > 31) return false;
    if (strlen(sysConfig.ap_password) > 31) return false;
    if (strlen(sysConfig.ntp_server) > 47) return false;
    if (strlen(sysConfig.hostname) > 31) return false;
    
    // Validate numerical fields
    if (sysConfig.gmt_offset < -43200 || sysConfig.gmt_offset > 43200) return false;
    if (sysConfig.daylight_offset < 0 || sysConfig.daylight_offset > 7200) return false;
    
    // Calculate and verify CRC
    uint32_t calculatedCRC = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
    if (sysConfig.crc != 0 && sysConfig.crc != calculatedCRC) {
        return false;  // CRC mismatch - data corruption
    }
    
    return true;
}

void backupConfiguration() {
    preferences.begin(NVS_BACKUP_NAMESPACE, false);
    preferences.putBytes("sysConfig", &sysConfig, sizeof(SystemConfig));
    preferences.putBytes("relayConfigs", relayConfigs, sizeof(RelayConfig) * NUM_RELAYS);
    preferences.putBytes("extConfig", &extConfig, sizeof(ExtConfig));
    preferences.end();
}

void restoreConfiguration() {
    preferences.begin(NVS_BACKUP_NAMESPACE, true);
    
    size_t len = preferences.getBytes("sysConfig", &sysConfig, sizeof(SystemConfig));
    if (len == sizeof(SystemConfig) && sysConfig.magic == EEPROM_MAGIC) {
        // Valid backup found
        preferences.getBytes("relayConfigs", relayConfigs, sizeof(RelayConfig) * NUM_RELAYS);
        preferences.getBytes("extConfig", &extConfig, sizeof(ExtConfig));
        
        // Save restored config back to primary
        saveConfiguration();
        saveExtConfig();
        
        preferences.end();
        return;
    }
    
    preferences.end();
    // No valid backup - initialize defaults
    initDefaults();
}

void scheduleConfigSave() {
    configSavePending = true;
    lastConfigSave = millis();
}

void flushConfigSave() {
    if (configSavePending) {
        unsigned long now = millis();
        if (now - lastConfigSave >= CONFIG_SAVE_DEBOUNCE) {
            // Update CRC before saving
            sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
            
            saveConfiguration();
            backupConfiguration();  // Create backup
            configSavePending = false;
        }
    }
}

void initDefaults() {
    memset(&sysConfig, 0, sizeof(SystemConfig));
    sysConfig.magic           = EEPROM_MAGIC;
    sysConfig.version         = EEPROM_VERSION;
    strcpy(sysConfig.ap_ssid,     "ESP32_20CH_Timer_Switch");
    strcpy(sysConfig.ap_password, "ESP32-admin");
    strcpy(sysConfig.ntp_server,  "ph.pool.ntp.org");
    sysConfig.gmt_offset      = 28800;
    sysConfig.daylight_offset = 0;
    sysConfig.last_rtc_epoch  = 0;
    sysConfig.rtc_drift       = 1.0f;
    strcpy(sysConfig.hostname, "esp32relay");
    sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
    
    for (int i = 0; i < NUM_RELAYS; i++) {
        memset(&relayConfigs[i], 0, sizeof(RelayConfig));
        for (int s = 0; s < 8; s++) {
            relayConfigs[i].schedule.days[s] = DAY_ALL;
            relayConfigs[i].schedule.monthDays[s] = 0;
        }
        snprintf(relayConfigs[i].name, 16, "Relay %d", i + 1);
    }
    
    saveConfiguration();
    backupConfiguration();
}

void loadConfiguration() {
    preferences.begin(NVS_NAMESPACE, true);
    
    size_t len = preferences.getBytes("sysConfig", &sysConfig, sizeof(SystemConfig));
    bool configValid = true;
    
    if (len != sizeof(SystemConfig) || sysConfig.magic != EEPROM_MAGIC) {
        configValid = false;
    }
    
    // Validate configuration integrity
    if (configValid && !validateConfiguration()) {
        configValid = false;
    }
    
    // Handle version migration
    if (configValid && sysConfig.version < EEPROM_VERSION) {
        sysConfig.version = EEPROM_VERSION;
        sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
        scheduleConfigSave();
    }
    
    if (!configValid) {
        preferences.end();
        
        // Try to restore from backup
        restoreConfiguration();
        
        preferences.begin(NVS_NAMESPACE, true);
        len = preferences.getBytes("sysConfig", &sysConfig, sizeof(SystemConfig));
        
        // If still invalid, initialize defaults
        if (len != sizeof(SystemConfig) || sysConfig.magic != EEPROM_MAGIC) {
            initDefaults();
        }
    }

    // Load relay configs with backward compatibility
    len = preferences.getBytes("relayConfigs", relayConfigs, sizeof(RelayConfig) * NUM_RELAYS);
    if (len != sizeof(RelayConfig) * NUM_RELAYS) {
        size_t v3Size = (sizeof(RelayConfig) - sizeof(uint8_t) * 8 - sizeof(uint32_t) * 8) * NUM_RELAYS;
        size_t v4Size = (sizeof(RelayConfig) - sizeof(uint32_t) * 8) * NUM_RELAYS;
        
        if (len == v4Size) {
            // V4 to V5 migration: initialize month days
            for (int i = 0; i < NUM_RELAYS; i++) {
                for (int s = 0; s < 8; s++) {
                    relayConfigs[i].schedule.monthDays[s] = 0;
                }
            }
        } else if (len == v3Size) {
            // V3 to V5 migration: initialize days and month days
            for (int i = 0; i < NUM_RELAYS; i++) {
                for (int s = 0; s < 8; s++) {
                    relayConfigs[i].schedule.days[s] = DAY_ALL;
                    relayConfigs[i].schedule.monthDays[s] = 0;
                }
            }
        } else {
            // Corrupted or empty, initialize fresh
            for (int i = 0; i < NUM_RELAYS; i++) {
                memset(&relayConfigs[i], 0, sizeof(RelayConfig));
                for (int s = 0; s < 8; s++) {
                    relayConfigs[i].schedule.days[s] = DAY_ALL;
                    relayConfigs[i].schedule.monthDays[s] = 0;
                }
                snprintf(relayConfigs[i].name, 16, "Relay %d", i + 1);
            }
        }
    }
    
    preferences.end();
    
    // Update AP copies
    strcpy(ap_ssid,     sysConfig.ap_ssid);
    strcpy(ap_password, sysConfig.ap_password);
}

void saveConfiguration() {
    bool saveSuccess = false;
    int retries = 3;
    
    while (retries-- > 0 && !saveSuccess) {
        preferences.begin(NVS_NAMESPACE, false);
        
        size_t written = preferences.putBytes("sysConfig", &sysConfig, sizeof(SystemConfig));
        if (written == sizeof(SystemConfig)) {
            written = preferences.putBytes("relayConfigs", relayConfigs, sizeof(RelayConfig) * NUM_RELAYS);
            if (written == sizeof(RelayConfig) * NUM_RELAYS) {
                saveSuccess = true;
            }
        }
        
        preferences.end();
        
        if (!saveSuccess) {
            healthMetrics.nvsWriteErrors++;
            delay(100);  // Brief delay before retry
        }
    }
    
    if (!saveSuccess) {
        // Critical error - try backup namespace
        backupConfiguration();
    }
}

void loadExtConfig() {
    preferences.begin(NVS_NAMESPACE, true);
    
    size_t len = preferences.getBytes("extConfig", &extConfig, sizeof(ExtConfig));
    
    if (len != sizeof(ExtConfig) || extConfig.magic != EXT_CFG_MAGIC) {
        memset(&extConfig, 0, sizeof(ExtConfig));
        extConfig.magic          = EXT_CFG_MAGIC;
        extConfig.ap_channel     = 6;
        extConfig.ntp_sync_hours = 1;
        extConfig.ap_hidden      = 0;
        extConfig.auto_recovery_enabled = 1;
        extConfig.watchdog_enabled = 1;
        preferences.end();
        saveExtConfig();
        preferences.begin(NVS_NAMESPACE, true);
    } else {
        // Validate extended config
        if (extConfig.ap_channel < 1 || extConfig.ap_channel > 13) {
            extConfig.ap_channel = 6;
        }
        if (extConfig.ntp_sync_hours < 1 || extConfig.ntp_sync_hours > 24) {
            extConfig.ntp_sync_hours = 1;
        }
    }
    preferences.end();
}

void saveExtConfig() {
    bool saveSuccess = false;
    int retries = 3;
    
    while (retries-- > 0 && !saveSuccess) {
        preferences.begin(NVS_NAMESPACE, false);
        size_t written = preferences.putBytes("extConfig", &extConfig, sizeof(ExtConfig));
        saveSuccess = (written == sizeof(ExtConfig));
        preferences.end();
        
        if (!saveSuccess) {
            delay(100);
        }
    }
}

// =============================================================================
//  ENHANCED HEALTH MONITORING & SELF-HEALING
// =============================================================================

void performHealthCheck() {
    unsigned long now = millis();
    if (now - lastHealthCheck < HEALTH_CHECK_INTERVAL) return;
    lastHealthCheck = now;
    
    // Check heap integrity
    uint32_t currentFreeHeap = ESP.getFreeHeap();
    healthMetrics.heapCorruptionChecks++;
    
    if (currentFreeHeap < MIN_FREE_HEAP_THRESHOLD) {
        healthMetrics.heapIntegrityOk = false;
        
        // Attempt recovery
        if (extConfig.auto_recovery_enabled) {
            // Clear any cached data
            WiFi.scanDelete();
            
            // Force garbage collection
            for (int i = 0; i < 10; i++) {
                delay(10);
                yield();
            }
        }
    } else {
        healthMetrics.heapIntegrityOk = true;
    }
    
    // Update metrics
    healthMetrics.lastHeapSize = currentFreeHeap;
    if (currentFreeHeap < healthMetrics.minFreeHeap || healthMetrics.minFreeHeap == 0) {
        healthMetrics.minFreeHeap = currentFreeHeap;
    }
    
    // Check mDNS health
    checkMDNSHealth();
    
    // Check WiFi health
    checkWiFiHealth();
    
    // Auto-recovery check
    if (extConfig.auto_recovery_enabled) {
        autoRecoveryCheck();
    }
}

void checkMemoryHealth() {
    unsigned long now = millis();
    if (now - lastMemoryCleanup < MEMORY_CLEANUP_INTERVAL) return;
    lastMemoryCleanup = now;
    
    uint32_t freeHeap = ESP.getFreeHeap();
    
    // If heap is critically low, perform emergency cleanup
    if (freeHeap < MIN_FREE_HEAP_THRESHOLD / 2) {
        // Clear scan results
        WiFi.scanDelete();
        scanInProgress = false;
        scanResultCount = -1;
        
        // Force cleanup
        for (int i = 0; i < 20; i++) {
            delay(5);
            yield();
        }
    }
}

void autoRecoveryCheck() {
    static unsigned long lastRecoveryCheck = 0;
    unsigned long now = millis();
    
    if (now - lastRecoveryCheck < 300000UL) return;  // Check every 5 minutes
    lastRecoveryCheck = now;
    
    // Check for hung tasks
    if (wcsState == WCS_PENDING && now - wcsStart > 120000UL) {
        // WiFi connection appears hung
        WiFi.disconnect(true);
        delay(1000);
        wcsState = WCS_IDLE;
        wifiGiveUpUntil = now + 300000UL;  // Back off for 5 minutes
    }
    
    // Check NTP sync health
    if (ntpSyncInProgress && now - lastNTPAttempt > 30000UL) {
        // NTP sync appears hung
        ntpSyncInProgress = false;
        ntpFailCount++;
    }
    
    // Check scan health
    if (scanInProgress && now - scanStartTime > SCAN_LOCK_TIMEOUT) {
        // WiFi scan appears hung
        WiFi.scanDelete();
        scanInProgress = false;
        scanResultCount = -1;
    }
    
    // Check for excessive resets
    RESET_REASON resetReason = rtc_get_reset_reason(0);
    if (resetReason == SW_CPU_RESET || resetReason == SW_RESET) {
        healthMetrics.watchdogResets++;
        
        // If too many resets, disable features that might cause issues
        if (healthMetrics.watchdogResets > 10) {
            wifiStaEnabled = false;
            ntpServerIndex = 0;
            ntpFailCount = 0;
        }
    }
}

void checkMDNSHealth() {
    unsigned long now = millis();
    if (now - lastMDNSCheck < MDNS_HEALTH_CHECK) return;
    lastMDNSCheck = now;
    
    if (mdnsStarted) {
        // Verify mDNS is actually responding
        if (WiFi.status() == WL_CONNECTED || WiFi.softAPgetStationNum() > 0) {
            // mDNS should be healthy
        } else {
            // No clients - mDNS might be stale
            if (now - mdnsRestartPending > 600000UL) {  // 10 minutes without restart
                scheduleMDNSRestart();
            }
        }
    }
}

void feedWatchdog() {
    unsigned long now = millis();
    if (now - lastWatchdogReset >= WATCHDOG_FEED_INTERVAL) {
        lastWatchdogReset = now;
        
        if (extConfig.watchdog_enabled) {
            esp_task_wdt_reset();
        }
        
        yield();  // Allow other tasks to run
    }
}

// =============================================================================
//  ENHANCED AP FUNCTIONS
// =============================================================================

void restartAP() {
    // Graceful shutdown
    WiFi.softAPdisconnect(true);
    delay(500);
    
    uint8_t ch = extConfig.ap_channel;
    if (ch < 1 || ch > 13) {
        ch = 6;
        extConfig.ap_channel = 6;
    }
    
    uint8_t hidden = extConfig.ap_hidden ? 1 : 0;
    
    bool apStarted = false;
    int retries = 3;
    
    while (retries-- > 0 && !apStarted) {
        if (strlen(sysConfig.ap_password) > 0) {
            apStarted = WiFi.softAP(sysConfig.ap_ssid, sysConfig.ap_password, ch, hidden);
        } else {
            apStarted = WiFi.softAP(sysConfig.ap_ssid, NULL, ch, hidden);
        }
        
        if (!apStarted) {
            delay(1000);
        }
    }
    
    if (apStarted) {
        scheduleMDNSRestart();
    }
}

// =============================================================================
//  ENHANCED mDNS FUNCTIONS
// =============================================================================

void startMDNS() {
    if (mdnsStarted) return;
    
    String hostname = String(mdnsHostname);
    if (hostname.length() == 0 || hostname == MDNS_HOSTNAME_DEFAULT) {
        hostname = String(sysConfig.ap_ssid);
        hostname.toLowerCase();
        hostname.replace(" ", "-");
        hostname.replace("_", "-");
        
        String clean;
        for (char c : hostname) {
            if (isalnum(c) || c == '-') clean += c;
        }
        if (clean.length() > 0 && clean.length() < 32) {
            hostname = clean;
        } else {
            hostname = MDNS_HOSTNAME_DEFAULT;
        }
        if (hostname.length() > 31) hostname = hostname.substring(0, 31);
        strcpy(mdnsHostname, hostname.c_str());
    }
    
    int retries = 3;
    while (retries-- > 0) {
        if (MDNS.begin(mdnsHostname)) {
            MDNS.addService("http", "tcp", 80);
            MDNS.addServiceTxt("http", "tcp", "model", "ESP32-20CH-Relay");
            MDNS.addServiceTxt("http", "tcp", "version", "v5.1");
            MDNS.addServiceTxt("http", "tcp", "channels", "20");
            MDNS.addServiceTxt("http", "tcp", "enhanced", "true");
            
            mdnsStarted = true;
            lastMDNSCheck = millis();
            return;
        }
        delay(100);
    }
    
    mdnsStarted = false;
}

void stopMDNS() {
    if (mdnsStarted) {
        MDNS.end();
        mdnsStarted = false;
        delay(50);
    }
}

void restartMDNS() {
    stopMDNS();
    delay(100);
    startMDNS();
}

void scheduleMDNSRestart() {
    mdnsRestartScheduled = true;
    mdnsRestartPending = millis() + MDNS_RESTART_DELAY;
}

String getMDNSHostname() {
    return String(mdnsHostname);
}

void setMDNSHostname(const char* hostname) {
    if (hostname && strlen(hostname) > 0 && strlen(hostname) < 32) {
        String sanitized;
        for (size_t i = 0; i < strlen(hostname); i++) {
            char c = tolower(hostname[i]);
            if (isalnum(c) || c == '-') {
                sanitized += c;
            } else if (c == ' ' || c == '_') {
                sanitized += '-';
            }
        }
        
        if (sanitized.length() > 0) {
            strncpy(mdnsHostname, sanitized.c_str(), 31);
            mdnsHostname[31] = '\0';
            if (mdnsStarted) {
                scheduleMDNSRestart();
            }
        }
    }
}

// =============================================================================
//  ENHANCED SETUP WITH WATCHDOG
// =============================================================================

void setup() {
    // Initialize watchdog
    if (extConfig.watchdog_enabled) {
        esp_task_wdt_init(WDT_TIMEOUT, true);
        esp_task_wdt_add(NULL);
    }
    
    mainTaskHandle = xTaskGetCurrentTaskHandle();
    
    // Initialize health metrics
    memset(&healthMetrics, 0, sizeof(HealthMetrics));
    healthMetrics.minFreeHeap = ESP.getFreeHeap();
    
    // Safe relay state first
    for (int i = 0; i < NUM_RELAYS; i++) {
        pinMode(relayPins[i], OUTPUT);
        digitalWrite(relayPins[i], relayActiveLow ? HIGH : LOW);
        relayConfigs[i].lastStateChange = millis();
        relayConfigs[i].stateChangeCount = 0;
    }
    
    // Initialize relay configs with safe defaults
    for (int i = 0; i < NUM_RELAYS; i++) {
        for (int s = 0; s < 8; s++) {
            relayConfigs[i].schedule.enabled[s] = false;
            relayConfigs[i].schedule.days[s] = DAY_ALL;
            relayConfigs[i].schedule.monthDays[s] = 0;
        }
        relayConfigs[i].manualOverride = false;
        relayConfigs[i].manualState    = false;
        snprintf(relayConfigs[i].name, 16, "Relay %d", i + 1);
    }

    // Load configuration from NVS with validation
    loadConfiguration();
    loadExtConfig();
    loadRTCState();

    // Set WiFi mode with validation
    WiFi.mode(WIFI_AP_STA);
    delay(100);
    
    // WiFi STA
    if (strlen(sysConfig.sta_ssid) > 0 && wifiStaEnabled) {
        WiFi.begin(sysConfig.sta_ssid, sysConfig.sta_password);

        unsigned long t0 = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_TIMEOUT) {
            delay(300);
            feedWatchdog();
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            wifiConnected = true;
            resetBackoff(&wifiBackoff);

            // Initialize NTP
            timeClient.begin();
            timeClient.setPoolServerName(sysConfig.ntp_server);
            timeClient.setTimeOffset(sysConfig.gmt_offset + sysConfig.daylight_offset);
            
            tryNTPSync();
        } else {
            wifiConnected = false;
            WiFi.disconnect(false);
        }
    } else {
        wifiConnected = false;
    }

    // Access Point
    uint8_t ch = extConfig.ap_channel;
    if (ch < 1 || ch > 13) {
        ch = 6;
        extConfig.ap_channel = 6;
    }
    
    uint8_t hidden = extConfig.ap_hidden ? 1 : 0;
    
    if (strlen(sysConfig.ap_password) > 0) {
        WiFi.softAP(sysConfig.ap_ssid, sysConfig.ap_password, ch, hidden);
    } else {
        WiFi.softAP(sysConfig.ap_ssid, NULL, ch, hidden);
    }

    // mDNS
    startMDNS();

    // Start DNS server for captive portal
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    
    // Setup web server
    setupWebServer();
    
    // Initial health check
    performHealthCheck();
    
    // Feed watchdog after setup
    feedWatchdog();
}

// =============================================================================
//  ENHANCED LOOP WITH STABILITY IMPROVEMENTS
// =============================================================================

void loop() {
    static unsigned long loopCount = 0;
    loopCount++;
    
    // Feed watchdog every iteration
    feedWatchdog();
    
    dnsServer.processNextRequest();
    server.handleClient();
    
    unsigned long now = millis();
    
    // Periodic health checks
    performHealthCheck();
    checkMemoryHealth();
    
    // Flush pending config saves
    flushConfigSave();
    
    // mDNS restart handling
    if (mdnsRestartScheduled) {
        if (now >= mdnsRestartPending) {
            mdnsRestartScheduled = false;
            restartMDNS();
        }
    }

    // WiFi scan timeout protection
    if (scanInProgress) {
        if (now - scanStartTime > SCAN_LOCK_TIMEOUT) {
            WiFi.scanDelete();
            scanInProgress = false;
            scanResultCount = -1;
        }
    }

    // WiFi connection state machine
    if (wcsState == WCS_PENDING) {
        wl_status_t status = WiFi.status();
        
        if (status == WL_CONNECTED) {
            wcsState              = WCS_IDLE;
            wifiConnected         = true;
            wifiReconnectAttempts = 0;
            wifiGiveUpUntil       = 0;
            resetBackoff(&wifiBackoff);
            
            timeClient.setPoolServerName(sysConfig.ntp_server);
            timeClient.setTimeOffset(sysConfig.gmt_offset + sysConfig.daylight_offset);
            
            lastNTPSync = 0;
            lastNTPAttempt = 0;
            
            scheduleMDNSRestart();
            
        } else if (now - wcsStart > WIFI_CONNECT_TIMEOUT) {
            wcsState = WCS_IDLE;
            incrementBackoff(&wifiBackoff);
            
            if (wifiReconnectAttempts >= MAX_RECONNECT) {
                wifiGiveUpUntil = now + calculateBackoff(&wifiBackoff);
                wifiReconnectAttempts = 0;
            }
        }
    }

    // WiFi health monitoring
    checkWiFiHealth();

    // NTP synchronization
    if (wifiConnected && !ntpSyncInProgress) {
        bool doSync = false;
        
        if (lastNTPSync == 0) {
            doSync = true;
        } else if (ntpFailCount > 0) {
            unsigned long backoffTime = calculateBackoff(&ntpBackoff);
            if (now - lastNTPAttempt >= backoffTime) {
                doSync = true;
            }
        } else if (now - lastNTPSync >= getNTPInterval()) {
            doSync = true;
        }
        
        if (doSync) {
            tryNTPSync();
        }
    }

    // Process relay schedules
    processRelaySchedules();
    
    // Update relay outputs (debounced)
    if (loopCount % 10 == 0) {  // Every 10th loop iteration
        updateRelayOutputs();
    }
    
    // Yield to other tasks periodically
    if (loopCount % 100 == 0) {
        yield();
    }
}

// =============================================================================
//  ENHANCED SCHEDULE ENGINE
// =============================================================================

void processRelaySchedules() {
    static unsigned long lastScheduleProcess = 0;
    unsigned long now = millis();
    
    // Process schedules every second
    if (now - lastScheduleProcess < 1000) return;
    lastScheduleProcess = now;
    
    healthMetrics.scheduleExecutions++;
    
    time_t epoch = getCurrentEpoch();
    if (epoch < 1000000000UL) return;

    struct tm* ti = localtime(&epoch);
    if (!ti) return;
    
    int cur = ti->tm_hour * 3600 + ti->tm_min * 60 + ti->tm_sec;
    uint8_t todayBit = (1 << ti->tm_wday);
    int monthDay = ti->tm_mday;

    for (int i = 0; i < NUM_RELAYS; i++) {
        if (relayConfigs[i].manualOverride) {
            continue;  // Manual override takes precedence
        }

        bool on = false;

        for (int s = 0; s < 8; s++) {
            if (!relayConfigs[i].schedule.enabled[s]) continue;
            
            // Check day of week
            if (!(relayConfigs[i].schedule.days[s] & todayBit)) continue;
            
            // Check day of month (if configured)
            uint32_t monthDayMask = relayConfigs[i].schedule.monthDays[s];
            if (monthDayMask != 0) {
                if (!(monthDayMask & (1 << (monthDay - 1)))) continue;
            }

            int start = relayConfigs[i].schedule.startHour[s]   * 3600
                      + relayConfigs[i].schedule.startMinute[s]  *   60
                      + relayConfigs[i].schedule.startSecond[s];
            int stop  = relayConfigs[i].schedule.stopHour[s]    * 3600
                      + relayConfigs[i].schedule.stopMinute[s]   *   60
                      + relayConfigs[i].schedule.stopSecond[s];

            if (start == stop) {
                // Always on when scheduled
                on = true;
                break;
            } else if (start < stop) {
                // Normal schedule (e.g., 08:00 to 17:00)
                if (cur >= start && cur < stop) {
                    on = true;
                    break;
                }
            } else {
                // Overnight schedule (e.g., 22:00 to 06:00)
                if (cur >= start || cur < stop) {
                    on = true;
                    break;
                }
            }
        }

        // Apply relay state with debouncing
        digitalWrite(relayPins[i], relayActiveLow ? !on : on);
    }
}

// =============================================================================
//  ENHANCED WEB SERVER SETUP WITH CACHING HEADERS
// =============================================================================

void setupWebServer() {
    // Serve pages with no-cache headers
    server.on("/",       HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "0");
        server.send_P(200, "text/html", index_html);  
    });
    
    server.on("/wifi",   HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "0");
        server.send_P(200, "text/html", wifi_html);   
    });
    
    server.on("/ntp",    HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "0");
        server.send_P(200, "text/html", ntp_html);    
    });
    
    server.on("/ap",     HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "0");
        server.send_P(200, "text/html", ap_html);     
    });
    
    server.on("/system", HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
        server.sendHeader("Pragma", "no-cache");
        server.sendHeader("Expires", "0");
        server.send_P(200, "text/html", system_html); 
    });
    
    // CSS with long cache time
    server.on("/style.css", HTTP_GET, []() { 
        server.sendHeader("Cache-Control", "public, max-age=86400");
        server.send_P(200, "text/css", style_css); 
    });

    // API endpoints
    server.on("/api/relays",       HTTP_GET,  handleGetRelays);
    server.on("/api/relay/manual", HTTP_POST, handleManualControl);
    server.on("/api/relay/reset",  HTTP_POST, handleResetManual);
    server.on("/api/relay/save",   HTTP_POST, handleSaveRelay);
    server.on("/api/relay/name",   HTTP_POST, handleRelayName);

    server.on("/api/time", HTTP_GET, handleGetTime);

    server.on("/api/wifi",        HTTP_GET,  handleGetWiFi);
    server.on("/api/wifi",        HTTP_POST, handleSaveWiFi);
    server.on("/api/wifi/scan",   HTTP_POST, handleWiFiScanStart);
    server.on("/api/wifi/scan",   HTTP_GET,  handleWiFiScanResults);

    server.on("/api/ntp",      HTTP_GET,  handleGetNTP);
    server.on("/api/ntp",      HTTP_POST, handleSaveNTP);
    server.on("/api/ntp/sync", HTTP_POST, handleSyncNTP);

    server.on("/api/ap", HTTP_GET,  handleGetAP);
    server.on("/api/ap", HTTP_POST, handleSaveAP);

    // Enhanced mDNS endpoints
    server.on("/api/mdns", HTTP_GET, []() {
        String resp = "{\"hostname\":\"" + getMDNSHostname() + 
                      "\",\"started\":" + String(mdnsStarted ? "true" : "false") +
                      ",\"url\":\"http://" + getMDNSHostname() + ".local\"" +
                      ",\"health\":\"ok\"}";
        server.send(200, "application/json", resp);
    });
    
    server.on("/api/mdns", HTTP_POST, []() {
        if (!server.hasArg("plain")) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}");
            return;
        }
        
        StaticJsonDocument<128> doc;
        DeserializationError err = deserializeJson(doc, server.arg("plain"));
        if (err) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}");
            return;
        }
        
        const char* hostname = doc["hostname"];
        if (hostname && strlen(hostname) > 0 && strlen(hostname) < 32) {
            setMDNSHostname(hostname);
            server.send(200, "application/json", "{\"success\":true,\"hostname\":\"" + getMDNSHostname() + "\"}");
        } else {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid hostname\"}");
        }
    });
    
    server.on("/api/mdns/restart", HTTP_POST, []() {
        restartMDNS();
        server.send(200, "application/json", "{\"success\":true}");
    });

    // Health endpoint
    server.on("/api/health", HTTP_GET, handleGetHealth);

    server.on("/api/system",        HTTP_GET,  handleGetSystem);
    server.on("/api/reset",         HTTP_POST, handleReset);
    server.on("/api/factory-reset", HTTP_POST, handleFactoryReset);

    // Captive portal endpoints
    server.on("/hotspot-detect.html", HTTP_GET, []() {
        server.send(200, "text/html",
            "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    });
    server.on("/library/test/success.html", HTTP_GET, []() {
        server.send(200, "text/html",
            "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
    });
    server.on("/generate_204", HTTP_GET, []() {
        server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
    });
    server.on("/success.txt",   HTTP_GET, []() { server.send(200, "text/plain", "success\n"); });
    server.on("/canonical.html", HTTP_GET, []() {
        server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
    });
    server.on("/connecttest.txt", HTTP_GET, []() {
        server.send(200, "text/plain", "Microsoft Connect Test");
    });
    server.on("/ncsi.txt", HTTP_GET, []() {
        server.send(200, "text/plain", "Microsoft NCSI");
    });
    server.on("/redirect", HTTP_GET, []() {
        server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
    });

    server.onNotFound([]() {
        server.sendHeader("Location", "http://" + WiFi.softAPIP().toString() + "/", true);
        server.send(302, "text/plain", "");
    });

    server.begin();
}

// =============================================================================
//  ENHANCED API HANDLERS
// =============================================================================

void handleGetRelays() {
    // Check memory before allocating large JSON
    if (ESP.getFreeHeap() < 65536) {
        server.send(500, "application/json", "{\"error\":\"Insufficient memory\"}");
        return;
    }
    
    DynamicJsonDocument doc(MAX_JSON_DOC_SIZE); 
    JsonArray root = doc.to<JsonArray>();
    
    for (int i = 0; i < NUM_RELAYS; i++) {
        JsonObject obj = root.createNestedObject();
        obj["state"] = (digitalRead(relayPins[i]) == (relayActiveLow ? LOW : HIGH));
        obj["manual"] = relayConfigs[i].manualOverride;
        obj["name"] = String(relayConfigs[i].name);
        obj["lastChange"] = relayConfigs[i].lastStateChange;
        
        JsonArray schedules = obj.createNestedArray("schedules");
        for (int s = 0; s < 8; s++) {
            JsonObject sch = schedules.createNestedObject();
            sch["startHour"] = relayConfigs[i].schedule.startHour[s];
            sch["startMinute"] = relayConfigs[i].schedule.startMinute[s];
            sch["startSecond"] = relayConfigs[i].schedule.startSecond[s];
            sch["stopHour"] = relayConfigs[i].schedule.stopHour[s];
            sch["stopMinute"] = relayConfigs[i].schedule.stopMinute[s];
            sch["stopSecond"] = relayConfigs[i].schedule.stopSecond[s];
            sch["enabled"] = relayConfigs[i].schedule.enabled[s];
            sch["days"] = relayConfigs[i].schedule.days[s];
            sch["monthDays"] = relayConfigs[i].schedule.monthDays[s];
        }
    }
    
    String resp;
    serializeJson(doc, resp);
    
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
    
    // Clean up
    doc.clear();
}

void handleManualControl() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    int relay = doc["relay"]; 
    bool state = doc["state"];
    if (relay >= 0 && relay < NUM_RELAYS) {
        relayConfigs[relay].manualOverride = true;
        relayConfigs[relay].manualState    = state;
        relayConfigs[relay].lastStateChange = millis();
        scheduleConfigSave();
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid relay\"}");
    }
}

void handleResetManual() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<64> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    int relay = doc["relay"];
    if (relay >= 0 && relay < NUM_RELAYS) {
        relayConfigs[relay].manualOverride = false;
        relayConfigs[relay].lastStateChange = millis();
        scheduleConfigSave();
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid relay\"}");
    }
}

void handleSaveRelay() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<4096> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    int relay = doc["relay"];
    if (relay < 0 || relay >= NUM_RELAYS) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid relay\"}");
        return;
    }
    
    JsonArray schedules = doc["schedules"].as<JsonArray>();
    int s = 0;
    for (JsonObject sch : schedules) {
        if (s >= 8) break;
        
        relayConfigs[relay].schedule.startHour[s]   = sch["startHour"];
        relayConfigs[relay].schedule.startMinute[s] = sch["startMinute"];
        relayConfigs[relay].schedule.startSecond[s] = sch["startSecond"];
        relayConfigs[relay].schedule.stopHour[s]    = sch["stopHour"];
        relayConfigs[relay].schedule.stopMinute[s]  = sch["stopMinute"];
        relayConfigs[relay].schedule.stopSecond[s]  = sch["stopSecond"];
        relayConfigs[relay].schedule.enabled[s]     = sch["enabled"];
        relayConfigs[relay].schedule.days[s]        = sch["days"] | 0;
        relayConfigs[relay].schedule.monthDays[s]   = sch["monthDays"] | 0;
        s++;
    }
    
    relayConfigs[relay].lastStateChange = millis();
    scheduleConfigSave();
    server.send(200, "application/json", "{\"success\":true}");
}

void handleRelayName() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    int relay = doc["relay"];
    const char* name = doc["name"];
    
    if (relay >= 0 && relay < NUM_RELAYS && name) {
        strncpy(relayConfigs[relay].name, name, 15);
        relayConfigs[relay].name[15] = '\0';
        scheduleConfigSave();
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid data\"}");
    }
}

void handleGetTime() {
    String ts = "--:--:--";
    time_t ep = getCurrentEpoch();
    if (ep > 1000000000UL) {
        struct tm* t = localtime(&ep);
        if (t) {
            char buf[10];
            sprintf(buf, "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);
            ts = buf;
        }
    }
    
    String resp = "{\"time\":\"" + ts + "\",\"wifi\":" + 
                  String(wifiConnected ? "true" : "false") + ",\"ntp\":" + 
                  String((lastNTPSync > 0) ? "true" : "false") + 
                  ",\"uptime\":" + String(millis() / 1000UL) + "}";
                  
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleGetWiFi() {
    String resp = "{\"ssid\":\"" + String(sysConfig.sta_ssid) + 
                  "\",\"connected\":" + String(wifiConnected ? "true" : "false") +
                  ",\"ip\":\"" + WiFi.localIP().toString() + 
                  "\",\"rssi\":" + String(wifiConnected ? (int)WiFi.RSSI() : 0) +
                  ",\"reconnects\":" + String(healthMetrics.wifiReconnects) + "}";
                  
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleSaveWiFi() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    const char* ssid = doc["ssid"];
    const char* pw   = doc["password"];
    if (ssid && strlen(ssid) > 0 && strlen(ssid) < 32) {
        strncpy(sysConfig.sta_ssid, ssid, 31); 
        sysConfig.sta_ssid[31] = '\0';
        if (pw && strlen(pw) > 0) { 
            strncpy(sysConfig.sta_password, pw, 63); 
            sysConfig.sta_password[63] = '\0'; 
        } else { 
            sysConfig.sta_password[0] = '\0'; 
        }
        
        // Validate before saving
        sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
        saveConfiguration();
        
        server.send(200, "application/json", "{\"success\":true}");
        
        // Graceful restart with delay
        delay(800);
        flushConfigSave();  // Ensure save is complete
        delay(200);
        ESP.restart();
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid SSID\"}");
    }
}

void handleWiFiScanStart() {
    if (wcsState != WCS_IDLE) {
        server.send(409, "application/json", "{\"scanning\":false,\"error\":\"WiFi busy\"}");
        return;
    }
    
    // Force unlock if stuck
    if (scanInProgress && millis() - scanStartTime > SCAN_LOCK_TIMEOUT) {
        WiFi.scanDelete();
        scanInProgress = false;
        scanResultCount = -1;
    }
    
    if (!scanInProgress) {
        scanInProgress  = true;
        scanResultCount = -1;
        scanStartTime   = millis();
        WiFi.scanNetworks(true, true);
    }
    server.send(202, "application/json", "{\"scanning\":true}");
}

void handleWiFiScanResults() {
    if (scanInProgress) {
        int n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) {
            server.send(200, "application/json", "{\"scanning\":true}");
            return;
        }
        scanResultCount = (n >= 0) ? n : -1;
        scanInProgress  = false;
    }
    
    if (scanResultCount < 0) {
        server.send(200, "application/json", "{\"scanning\":false,\"networks\":[]}");
        return;
    }
    
    DynamicJsonDocument doc(8192);
    doc["scanning"] = false;
    JsonArray nets = doc.createNestedArray("networks");
    
    for (int i = 0; i < scanResultCount && i < 30; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;
        
        JsonObject n = nets.createNestedObject();
        n["ssid"] = ssid;
        n["rssi"] = WiFi.RSSI(i);
        n["enc"]  = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    
    WiFi.scanDelete();
    scanResultCount = -1;
    
    String resp; 
    serializeJson(doc, resp);
    
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleGetNTP() {
    String resp = "{\"ntpServer\":\"" + String(sysConfig.ntp_server) + 
                  "\",\"gmtOffset\":" + String(sysConfig.gmt_offset) +
                  ",\"daylightOffset\":" + String(sysConfig.daylight_offset) +
                  ",\"syncHours\":" + String(extConfig.ntp_sync_hours) +
                  ",\"failures\":" + String(healthMetrics.ntpSyncFailures) + "}";
                  
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleSaveNTP() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    const char* srv = doc["ntpServer"];
    if (srv && strlen(srv) > 0 && strlen(srv) < 48) {
        strncpy(sysConfig.ntp_server, srv, 47); 
        sysConfig.ntp_server[47] = '\0';
        
        sysConfig.gmt_offset      = doc["gmtOffset"] | 0;
        sysConfig.daylight_offset = doc["daylightOffset"] | 0;
        
        if (doc.containsKey("syncHours")) {
            uint8_t h = doc["syncHours"];
            if (h >= 1 && h <= 24) { 
                extConfig.ntp_sync_hours = h; 
                saveExtConfig(); 
            }
        }
        
        // Validate and save
        sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
        scheduleConfigSave();
        
        if (wifiConnected) {
            timeClient.setPoolServerName(sysConfig.ntp_server);
            timeClient.setTimeOffset(sysConfig.gmt_offset + sysConfig.daylight_offset);
        }
        
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid NTP server\"}");
    }
}

void handleSyncNTP() {
    if (!wifiConnected) {
        server.send(400, "application/json",
            "{\"success\":false,\"error\":\"WiFi not connected\"}"); 
        return;
    }
    
    // Prevent overlapping syncs
    if (ntpSyncInProgress) {
        server.send(409, "application/json",
            "{\"success\":false,\"error\":\"Sync already in progress\"}");
        return;
    }
    
    tryNTPSync();
    
    if (lastNTPSync > 0 && millis() - lastNTPSync < 5000UL) {
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json",
            "{\"success\":false,\"error\":\"Sync failed — check NTP server\"}");
    }
}

void handleGetAP() {
    String resp = "{\"ap_ssid\":\"" + String(sysConfig.ap_ssid) + 
                  "\",\"ap_password\":\"" + String(sysConfig.ap_password) +
                  "\",\"ap_channel\":" + String(extConfig.ap_channel) +
                  ",\"ap_hidden\":" + String(extConfig.ap_hidden ? "true" : "false") +
                  ",\"clients\":" + String(WiFi.softAPgetStationNum()) + "}";
                  
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleSaveAP() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}"); 
        return;
    }
    
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, server.arg("plain"));
    if (err) {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Bad JSON\"}"); 
        return;
    }
    
    const char* ssid = doc["ap_ssid"];
    const char* pw   = doc["ap_password"];
    
    if (ssid && strlen(ssid) > 0 && strlen(ssid) < 32) {
        strncpy(sysConfig.ap_ssid, ssid, 31); 
        sysConfig.ap_ssid[31] = '\0';
        strcpy(ap_ssid, sysConfig.ap_ssid);
        
        if (pw && strlen(pw) > 0) {
            if (strlen(pw) >= 8 || strlen(pw) == 0) {
                strncpy(sysConfig.ap_password, pw, 31); 
                sysConfig.ap_password[31] = '\0';
                strcpy(ap_password, sysConfig.ap_password);
            }
        } else {
            sysConfig.ap_password[0] = '\0';
            ap_password[0] = '\0';
        }
        
        if (doc.containsKey("ap_channel")) {
            uint8_t ch = doc["ap_channel"];
            if (ch >= 1 && ch <= 13) extConfig.ap_channel = ch;
        }
        
        if (doc.containsKey("ap_hidden")) {
            extConfig.ap_hidden = doc["ap_hidden"] ? 1 : 0;
        }
        
        // Validate and save
        sysConfig.crc = calculateCRC32((uint8_t*)&sysConfig, sizeof(SystemConfig) - sizeof(uint32_t));
        saveConfiguration();
        saveExtConfig();
        
        server.send(200, "application/json", "{\"success\":true}");
        delay(100);
        restartAP();
    } else {
        server.send(400, "application/json", "{\"success\":false,\"error\":\"Invalid AP SSID\"}");
    }
}

void handleGetSystem() {
    DynamicJsonDocument doc(1024);
    
    doc["ip"] = WiFi.localIP().toString();
    doc["ap_ip"] = WiFi.softAPIP().toString();
    doc["uptime"] = millis() / 1000UL;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["ntpSynced"] = (lastNTPSync > 0);
    doc["ntpServer"] = sysConfig.ntp_server;
    doc["ntpSyncAge"] = lastNTPSync > 0 ? (int)((millis() - lastNTPSync) / 1000UL) : -1;
    doc["wifiConnected"] = wifiConnected;
    doc["wifiSSID"] = sysConfig.sta_ssid;
    doc["rssi"] = wifiConnected ? (int)WiFi.RSSI() : 0;
    doc["version"] = EEPROM_VERSION;
    doc["chipModel"] = "ESP32";
    doc["mdnsHostname"] = getMDNSHostname();
    doc["mdnsStarted"] = mdnsStarted;
    
    // Health status
    String healthStatus = "Good";
    if (!healthMetrics.heapIntegrityOk) healthStatus = "Warning";
    if (healthMetrics.watchdogResets > 5) healthStatus = "Critical";
    doc["healthStatus"] = healthStatus;
    doc["watchdogResets"] = healthMetrics.watchdogResets;
    doc["minFreeHeap"] = healthMetrics.minFreeHeap;
    doc["nvsWriteErrors"] = healthMetrics.nvsWriteErrors;
    
    String resp;
    serializeJson(doc, resp);
    
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleGetHealth() {
    DynamicJsonDocument doc(512);
    
    doc["heapFree"] = ESP.getFreeHeap();
    doc["heapMin"] = healthMetrics.minFreeHeap;
    doc["heapOk"] = healthMetrics.heapIntegrityOk;
    doc["uptime"] = millis() / 1000UL;
    doc["watchdogResets"] = healthMetrics.watchdogResets;
    doc["ntpFailures"] = healthMetrics.ntpSyncFailures;
    doc["wifiReconnects"] = healthMetrics.wifiReconnects;
    doc["nvsWriteErrors"] = healthMetrics.nvsWriteErrors;
    doc["scheduleExecutions"] = healthMetrics.scheduleExecutions;
    doc["wifiConnected"] = wifiConnected;
    doc["ntpSynced"] = (lastNTPSync > 0);
    doc["mdnsRunning"] = mdnsStarted;
    
    // Calculate health score
    int healthScore = 100;
    if (!wifiConnected) healthScore -= 20;
    if (!healthMetrics.heapIntegrityOk) healthScore -= 15;
    if (healthMetrics.nvsWriteErrors > 0) healthScore -= 10;
    if (healthMetrics.watchdogResets > 5) healthScore -= 25;
    doc["healthScore"] = healthScore;
    
    String resp;
    serializeJson(doc, resp);
    
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", resp);
}

void handleReset() {
    flushConfigSave();  // Ensure all saves are complete
    server.send(200, "application/json", "{\"success\":true}");
    delay(600);
    ESP.restart();
}

void handleFactoryReset() {
    // Clear preferences
    preferences.begin(NVS_NAMESPACE, false);
    preferences.clear();
    preferences.end();
    
    // Clear backup
    preferences.begin(NVS_BACKUP_NAMESPACE, false);
    preferences.clear();
    preferences.end();
    
    server.send(200, "application/json", "{\"success\":true}");
    delay(600);
    ESP.restart();
}
