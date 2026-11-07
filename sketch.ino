#include <WiFi.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <DNSServer.h>

// ---------- CONFIGURATION ----------
const char* AP_SSID = "ESP32_WIFI_BEACON";             
const char* AP_PASS = "12345678";      
#define LED_PIN 2

// ---------- RUNTIME STATE ----------
WebServer server(80);
DNSServer dnsServer;
const byte DNS_PORT = 53;

volatile bool  broadcasting  = false;
volatile int   totalWifi     = 1000;
volatile int   wifiChannel   = 1;
volatile unsigned long sentCount = 0;
char wifiName[32] = "SARTZ"; 

uint8_t packet[128] = {
  0x80, 0x00, 0x00, 0x00,
  0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  
  0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x64, 0x00, 0x01, 0x04,
  0x00, 0x00
};

// ---------- HTML PORTAL ----------
const char PORTAL_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>SARTZ BEACON WIFI SPAMMER</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Courier New', Courier, monospace; }
  body { background-color: #000; color: #fff; display: flex; justify-content: center; align-items: center; min-height: 100vh; padding: 15px; }
  .container { width: 100%; max-width: 450px; display: flex; flex-direction: column; justify-content: center; }
  .header { border: 1px solid #fff; padding: 15px; text-align: center; font-size: 18px; font-weight: bold; letter-spacing: 2px; margin-bottom: 20px; color: #fff; text-shadow: 0 0 10px #fff; box-shadow: 0 0 15px rgba(255,255,255,0.2); background: rgba(255,255,255,0.05); }
  .status-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px; }
  .status-item { background: #080808; border: 1px solid #333; padding: 10px; font-size: 12px; text-align: center; border-radius: 4px; }
  .status-item span { color: #fff; font-weight: bold; text-shadow: 0 0 5px #fff; }
  .temp-bar { display: flex; justify-content: center; font-size: 14px; font-weight: bold; margin-bottom: 25px; background: #080808; border: 1px solid #333; padding: 10px; border-radius: 4px; }
  .temp-text { color: #fff; transition: 0.3s; text-shadow: 0 0 5px #fff; }
  .temp-text.hot { color: #ff0000; text-shadow: 0 0 10px #ff0000; animation: blink 1s infinite; border-color: #ff0000;}
  @keyframes blink { 50% { opacity: 0.5; } }
  .form-box { background: #050505; border-radius: 8px; padding: 25px 20px; border: 1px solid #fff; box-shadow: 0 0 20px rgba(255,255,255,0.1); }
  .input-group { margin-bottom: 20px; }
  input { width: 100%; padding: 12px 10px; background: transparent; border: none; border-bottom: 2px solid #555; color: #fff; font-size: 15px; outline: none; transition: 0.2s; }
  input:focus { border-bottom: 2px solid #fff; box-shadow: 0 10px 10px -10px #fff; }
  input::placeholder { color: #888; }
  .error-msg { color: #ff0000; font-size: 11px; margin-top: 5px; display: none; font-weight: bold; text-shadow: 0 0 5px #ff0000; }
  .btn { width: 100%; padding: 15px; border: 1px solid #fff; border-radius: 4px; font-size: 14px; font-weight: bold; letter-spacing: 1.5px; cursor: pointer; margin-top: 15px; text-transform: uppercase; transition: 0.2s; background: transparent; color: #fff; }
  .btn-start { box-shadow: 0 0 10px rgba(255,255,255,0.2); }
  .btn-start:hover, .btn-start:active { background: #fff; color: #000; box-shadow: 0 0 25px #fff; }
  .btn-stop { border-color: #ff0000; color: #ff0000; margin-top: 15px; box-shadow: 0 0 10px rgba(255,0,0,0.2); }
  .btn-stop:hover, .btn-stop:active { background: #ff0000; color: #000; box-shadow: 0 0 25px #ff0000; }
  .btn:disabled { opacity: 0.3; cursor: not-allowed; border-color: #333; color: #333; box-shadow: none; }
</style>
</head>
<body>
<div class="container">
  <div class="header">SARTZ BEACON WIFI SPAMMER</div>
  <div class="status-grid">
    <div class="status-item">STATUS<br><span id="statusText">NOT ACTIVE</span></div>
    <div class="status-item">PACKET SENT<br><span id="packetText">0</span></div>
    <div class="status-item" style="grid-column: span 2;">NAME SENT<br><span id="nameSentText">---</span></div>
  </div>
  <div class="temp-bar"><span class="temp-text" id="tempText">ESP TEMP: -- °C</span></div>
  <div class="form-box">
    <div class="input-group">
      <input type="text" id="wifiName" maxlength="31" placeholder="Enter Wifi name (SSID)..." oninput="hideError('errName')">
      <div class="error-msg" id="errName">⚠ Wifi Name is required!</div>
    </div>
    <div class="input-group">
      <input type="number" id="totalWifi" placeholder="Enter total wifi (1 to 1000)..." oninput="hideError('errTotal')">
      <div class="error-msg" id="errTotal">⚠ Enter Total WiFi (1-1000)!</div>
    </div>
    <div class="input-group">
      <input type="number" id="channel" placeholder="Enter channel (1 to 14)..." oninput="hideError('errChannel')">
      <div class="error-msg" id="errChannel">⚠ Enter Channel (1-14)!</div>
    </div>
    <button class="btn btn-start" id="startBtn" onclick="startBroadcast()">START BROADCAST</button>
    <button class="btn btn-stop" id="stopBtn" onclick="stopBroadcast()" disabled>STOP BROADCAST</button>
  </div>
</div>
<script>
  let updateTimer = null;
  async function hideError(id) { document.getElementById(id).style.display = 'none'; }
  
  async function startBroadcast() {
    let nameVal = document.getElementById('wifiName').value.trim();
    let totalVal = document.getElementById('totalWifi').value.trim();
    let chVal = document.getElementById('channel').value.trim();
    
    hideError('errName'); hideError('errTotal'); hideError('errChannel');
    let hasError = false;
    if (nameVal === "" || nameVal.includes('"')) { document.getElementById('errName').style.display = 'block'; hasError = true; }
    let totalNum = parseInt(totalVal);
    if (totalVal === "" || isNaN(totalNum) || totalNum < 1 || totalNum > 1000) { document.getElementById('errTotal').style.display = 'block'; hasError = true; }
    let chNum = parseInt(chVal);
    if (chVal === "" || isNaN(chNum) || chNum < 1 || chNum > 14) { document.getElementById('errChannel').style.display = 'block'; hasError = true; }
    if (hasError) return;

    document.getElementById('statusText').textContent = 'ACTIVE';
    document.getElementById('nameSentText').textContent = nameVal;
    document.getElementById('startBtn').disabled = true;
    document.getElementById('stopBtn').disabled = false;
    document.getElementById('packetText').textContent = '0';
    
    const payload = { name: nameVal, total: totalNum, channel: chNum };
    try {
      const res = await fetch('/start', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(payload) });
      if (!res.ok) throw new Error('start failed');
    } catch (e) {
      document.getElementById('statusText').textContent = 'NOT ACTIVE';
      document.getElementById('nameSentText').textContent = '---';
      document.getElementById('startBtn').disabled = false;
      document.getElementById('stopBtn').disabled = true;
      document.getElementById('packetText').textContent = '0';
      return;
    }
    
    if(updateTimer) clearInterval(updateTimer);
    updateTimer = setInterval(updateStatus, 500);
  }

  async function stopBroadcast() {
    document.getElementById('statusText').textContent = 'NOT ACTIVE';
    document.getElementById('nameSentText').textContent = '---';
    document.getElementById('startBtn').disabled = false;
    document.getElementById('stopBtn').disabled = true;
    document.getElementById('packetText').textContent = '0';
    if(updateTimer) { clearInterval(updateTimer); updateTimer = null; }
    await fetch('/stop', { method: 'POST' });
  }

  async function updateStatus() {
    try {
      const res = await fetch('/status');
      const data = await res.json();
      document.getElementById('packetText').textContent = data.sent;
    } catch(e) {}
  }

  async function updateTemp() {
    try {
      const res = await fetch('/temp');
      const data = await res.json();
      const tempEl = document.getElementById('tempText');
      tempEl.textContent = 'ESP TEMP: ' + data.temp + ' °C';
      if (data.temp >= 65) tempEl.classList.add('hot');
      else tempEl.classList.remove('hot');
    } catch(e) {}
  }
  async function syncOnLoad() {
    try {
      const res = await fetch('/status');
      const data = await res.json();
      if (data.active) {
        document.getElementById('statusText').textContent = 'ACTIVE';
        document.getElementById('startBtn').disabled = true;
        document.getElementById('stopBtn').disabled = false;
        document.getElementById('packetText').textContent = data.sent;
        if (updateTimer) clearInterval(updateTimer);
        updateTimer = setInterval(updateStatus, 500);
      }
    } catch(e) {}
  }
  window.onload = () => {
    setInterval(updateTemp, 2000);
    syncOnLoad();
  };
</script>
</body>
</html>
)rawliteral";

// ---------- BROADCAST WORKER ----------
void broadcastPackets() {
  if (!broadcasting) return;
  if (totalWifi <= 0) totalWifi = 1;

  for (uint16_t a = 0; a < totalWifi && broadcasting; a++) {
    packet[10] = packet[16] = (random(256) & 0xFE) | 0x02;
    packet[11] = packet[17] = random(256);

    char ssidBuffer[32];
    snprintf(ssidBuffer, sizeof(ssidBuffer), "%s %d", wifiName, a);
    int ssidLen = strlen(ssidBuffer);

    packet[37] = ssidLen;
    memcpy(&packet[38], ssidBuffer, ssidLen);

    int pos = 38 + ssidLen;
    packet[pos++] = 0x01;
    packet[pos++] = 0x04;
    packet[pos++] = 0x82;
    packet[pos++] = 0x84;
    packet[pos++] = 0x8B;
    packet[pos++] = 0x96;
    packet[pos++] = 0x03;
    packet[pos++] = 0x01;
    packet[pos++] = (uint8_t)wifiChannel;

    if (esp_wifi_80211_tx(WIFI_IF_AP, packet, pos, false) == ESP_OK) {
      sentCount++;
    }

    if ((a % 50) == 0) {
      server.handleClient();
      dnsServer.processNextRequest();
      yield();
    }
    delay(1);
  }
}

// ---------- WEB HANDLERS ----------
void handleNotFound() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void handleRoot() {
  server.send(200, "text/html", PORTAL_HTML);
}

void handleStart() {
  if (broadcasting) { server.send(409, "application/json", "{\"ok\":false,\"err\":\"busy\"}"); return; }
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"ok\":false}"); return; }
  String body = server.arg("plain");
  
  int n1 = body.indexOf("\"name\":\"");
  int n2 = body.lastIndexOf("\"total\":");
  int n3 = body.lastIndexOf("\"channel\":");

  if (n1 >= 0) {
    int start = n1 + 8;
    int end   = body.indexOf('"', start);
    if (end > start) {
      String tempName = body.substring(start, end);
      tempName.toCharArray(wifiName, sizeof(wifiName));
    }
  }
  if (n2 >= 0) {
    totalWifi = body.substring(n2 + 8).toInt();
    if (totalWifi > 1000) totalWifi = 1000;
    if (totalWifi < 1) totalWifi = 1;
  }
  if (n3 >= 0) {
    wifiChannel = body.substring(n3 + 10).toInt();
    if (wifiChannel < 1 || wifiChannel > 14) wifiChannel = 1;
    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  }

  sentCount = 0;
  broadcasting = true;
  digitalWrite(LED_PIN, HIGH);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStop() {
  broadcasting = false;
  sentCount = 0; 
  digitalWrite(LED_PIN, LOW);
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleStatus() {
  char buf[96];
  snprintf(buf, sizeof(buf), "{\"active\":%s,\"sent\":%lu}", broadcasting ? "true" : "false", sentCount);
  server.send(200, "application/json", buf);
}

void handleTemp() {
  float temp = temperatureRead();
  if (isnan(temp) || temp < -40.0f || temp > 150.0f) temp = 0.0f;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"temp\": %.1f}", temp);
  server.send(200, "application/json", buf);
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_start();
  
  randomSeed(esp_random());
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/start", HTTP_POST, handleStart);
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/temp", HTTP_GET, handleTemp); 
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
  server.handleClient();
  
  if (broadcasting) {
    broadcastPackets();
  }
}
