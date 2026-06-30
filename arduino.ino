// ============================================
// Vending Machine Controller
// Hardware: ESP32 Dev Module
// Framework: Arduino ESP32 Core 3.3.7
// ESP-IDF: v5.5.2
// ============================================

#include <WiFi.h>
#include <WebServer.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <Update.h>
#include <mbedtls/md.h>

// ========================================
// GPIO DEFINITIONS
// ========================================
#define BUTTON1_PIN         25
#define BUTTON2_PIN         26
#define BUTTON3_PIN         27
#define BUTTON4_PIN         14
#define SENSOR_PIN          15
#define SENSOR_LED_PIN      2
#define LED_FRAME_PIN       12
#define RELAY1_PIN          16
#define RELAY2_PIN          17
#define RELAY3_PIN          18
#define RELAY4_PIN          23
#define BUZZER_PIN          5
#define COIN_PIN            19
#define SEG_SER_PIN         13  // 74HC595 Serial Data
#define SEG_SCLK_PIN        32  // 74HC595 Shift Clock
#define SEG_RCLK_PIN        33  // 74HC595 Latch Clock
#define SEG_DIGIT1_PIN      21  // Digit 1 common (transistor)
#define SEG_DIGIT2_PIN      22  // Digit 2 common (transistor)

// ========================================
// CONSTANTS
// ========================================
#define NUM_SERVICES         4
#define NUM_DIGITS           2
#define DEBOUNCE_MS          50
#define COIN_PULSE_VALUE     1
#define MAX_CREDITS          99
#define DISPLAY_REFRESH_US   5000
#define STATUS_UPDATE_MS     300
#define COIN_SHORT_THRESHOLD 10
#define COIN_SHORT_WINDOW_MS 1000
#define DEFAULT_SERVICE_TIME 30000
#define MIN_SERVICE_TIME     5000
#define MAX_SERVICE_TIME     120000
#define WIFI_TIMEOUT_MS      30000
#define AP_MODE_TIMEOUT_MS   300000

// ========================================
// SERVICE CONFIGURATION STRUCTURE
// ========================================
struct ServiceConfig {
    const char* name;
    const char* code;
    uint8_t relayPin;
    uint8_t buttonPin;
    unsigned long duration;
    unsigned int cost;
    bool enabled;
};

// ========================================
// GLOBAL OBJECTS
// ========================================
WebServer server(80);
Preferences preferences;

// ========================================
// GLOBAL VARIABLES
// ========================================
// Service configuration
ServiceConfig services[NUM_SERVICES] = {
    {"Service 1", "SVC1", RELAY1_PIN, BUTTON1_PIN, DEFAULT_SERVICE_TIME, 1, true},
    {"Service 2", "SVC2", RELAY2_PIN, BUTTON2_PIN, DEFAULT_SERVICE_TIME, 1, true},
    {"Service 3", "SVC3", RELAY3_PIN, BUTTON3_PIN, DEFAULT_SERVICE_TIME, 1, true},
    {"Service 4", "SVC4", RELAY4_PIN, BUTTON4_PIN, DEFAULT_SERVICE_TIME, 1, true}
};

// Button handling
bool buttonStates[NUM_SERVICES] = {false};
bool lastButtonReading[NUM_SERVICES] = {false};
unsigned long lastButtonDebounce[NUM_SERVICES] = {0};
bool buttonPressed[NUM_SERVICES] = {false};

// Sensor
bool sensorState = false;
bool lastSensorState = false;
unsigned long sensorTriggerTime = 0;
unsigned int sensorTriggerCount = 0;

// Relays
bool relayStates[NUM_SERVICES] = {false};
unsigned long relayTimingStart[NUM_SERVICES] = {0};
int relayTimingDuration[NUM_SERVICES] = {500};
bool relayTimingActive[NUM_SERVICES] = {false};

// Coin acceptor
volatile unsigned int coinPulses = 0;
volatile unsigned long lastCoinPulseTime = 0;
bool coinShorted = false;
unsigned int coinPulseCountWindow = 0;
unsigned long coinWindowStart = 0;
int totalCredits = 0;
int lifetimeCoins = 0;

// Service states
enum ServiceState {
    STATE_IDLE = 0,
    STATE_ACTIVE = 1,
    STATE_PAUSED = 2,
    STATE_COMPLETED = 3,
    STATE_ERROR = 4
};
ServiceState serviceStates[NUM_SERVICES] = {STATE_IDLE};
unsigned long serviceStartTime[NUM_SERVICES] = {0};
unsigned long serviceElapsedTime[NUM_SERVICES] = {0};
unsigned long serviceDuration[NUM_SERVICES] = {DEFAULT_SERVICE_TIME};
unsigned int serviceCount[NUM_SERVICES] = {0};

// Buzzer
bool buzzerActive = false;
int buzzerPattern = 0;
unsigned long buzzerStartTime = 0;
unsigned long buzzerLastToggle = 0;
bool buzzerState = false;

// Display (multiplexed 7-segment)
int currentDisplayMode = 0;  // 0=CRED, 1-4=SVC1-4
int displayValue = 0;
bool displayNeedsUpdate = true;
unsigned long lastDisplayRefresh = 0;
int currentDigit = 0;
const uint8_t segmentMap[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

// Diagnostic mode
bool diagnosticMode = false;
bool diagnosticBlinkState = false;
unsigned long diagnosticBlinkTimer = 0;

// WiFi
String wifiSSID = "";
String wifiPassword = "";
String apSSID = "VENDING_MACHINE";
String apPassword = "12345678";
String hostname = "vending-machine";
bool wifiConnected = false;
bool apActive = false;
unsigned long wifiConnectStart = 0;
int wifiReconnectAttempts = 0;

// Web server
String authUsername = "admin";
String authPassword = "admin";
bool authRequired = true;
String sessionToken = "";
unsigned long lastWebRequest = 0;

// System
unsigned long systemStartTime = 0;
unsigned long lastStatusLog = 0;
bool systemError = false;
String systemErrorMessage = "";
int watchdogTimer = 0;

// Timer for non-blocking operations
unsigned long loopTimer = 0;
unsigned long statusUpdateTimer = 0;
unsigned long displayTimer = 0;

// ========================================
// SETUP
// ========================================
void setup() {
    systemStartTime = millis();
    
    Serial.begin(115200);
    delay(100);
    Serial.println();
    Serial.println("========================================");
    Serial.println("N&R Solartech Sol - Vending Machine");
    Serial.println("FW: arduino-lib-builder v8cabf2c");
    Serial.println("ESP-IDF: v5.5.2");
    Serial.println("========================================");
    
    // Initialize all subsystems
    initGPIO();
    playBootSound();
    initDisplay();
    initSPIFFS();
    loadConfiguration();
    initWiFi();
    initWebServer();
    initCoinAcceptor();
    showStartupDisplay();
    
    Serial.println("[SYSTEM] Initialization complete");
    Serial.printf("[SYSTEM] Free heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("[SYSTEM] Chip: %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("[SYSTEM] CPU Freq: %d MHz\n", ESP.getCpuFreqMHz());
}

// ========================================
// MAIN LOOP
// ========================================
void loop() {
    unsigned long currentMillis = millis();
    
    // Core functions - run every iteration
    server.handleClient();
    readButtons();
    readSensor();
    handleCoinAcceptor();
    updateDisplayMultiplex();
    
    // Timed functions
    if (currentMillis - loopTimer >= 10) {
        loopTimer = currentMillis;
        handleRelayTiming();
        handleBuzzer();
        handleServices();
        handleDiagnosticMode();
    }
    
    // Status updates (every 300ms)
    if (currentMillis - statusUpdateTimer >= STATUS_UPDATE_MS) {
        statusUpdateTimer = currentMillis;
        updateSystemStatus();
        checkWiFiConnection();
    }
    
    // Periodic display refresh
    if (currentMillis - displayTimer >= 100) {
        displayTimer = currentMillis;
        if (displayNeedsUpdate) {
            updateDisplayValue();
        }
    }
    
    // Serial debug output
    if (currentMillis - lastStatusLog >= 60000) {
        lastStatusLog = currentMillis;
        logSystemStatus();
    }
    
    // Watchdog
    watchdogTimer++;
    if (watchdogTimer > 1000000) {
        watchdogTimer = 0;
        checkSystemHealth();
    }
}

// ========================================
// GPIO INITIALIZATION
// ========================================
void initGPIO() {
    Serial.println("[GPIO] Initializing pins...");
    
    // Configure button inputs with pullup resistors
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLUP);
    pinMode(BUTTON3_PIN, INPUT_PULLUP);
    pinMode(BUTTON4_PIN, INPUT_PULLUP);
    
    // Configure sensor input
    pinMode(SENSOR_PIN, INPUT);
    
    // Configure outputs
    pinMode(SENSOR_LED_PIN, OUTPUT);
    pinMode(LED_FRAME_PIN, OUTPUT);
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);
    pinMode(RELAY4_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    
    // Configure display pins
    pinMode(SEG_SER_PIN, OUTPUT);
    pinMode(SEG_SCLK_PIN, OUTPUT);
    pinMode(SEG_RCLK_PIN, OUTPUT);
    pinMode(SEG_DIGIT1_PIN, OUTPUT);
    pinMode(SEG_DIGIT2_PIN, OUTPUT);
    
    // Set initial states
    digitalWrite(SENSOR_LED_PIN, LOW);
    digitalWrite(LED_FRAME_PIN, LOW);
    allRelaysOff();
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(SEG_DIGIT1_PIN, LOW);
    digitalWrite(SEG_DIGIT2_PIN, LOW);
    
    // Configure coin acceptor interrupt
    pinMode(COIN_PIN, INPUT_PULLUP);
    
    Serial.println("[GPIO] Initialization complete");
}

// ========================================
// BOOT SOUND
// ========================================
void playBootSound() {
    // Quick ascending tone sequence
    for (int i = 0; i < 3; i++) {
        tone(BUZZER_PIN, 1000 + (i * 500), 100);
        delay(120);
    }
    noTone(BUZZER_PIN);
}

// ========================================
// DISPLAY INITIALIZATION
// ========================================
void initDisplay() {
    // Clear all segments
    digitalWrite(SEG_RCLK_PIN, LOW);
    shiftOut(SEG_SER_PIN, SEG_SCLK_PIN, MSBFIRST, 0x00);
    shiftOut(SEG_SER_PIN, SEG_SCLK_PIN, MSBFIRST, 0x00);
    digitalWrite(SEG_RCLK_PIN, HIGH);
    
    // Turn off digits
    digitalWrite(SEG_DIGIT1_PIN, LOW);
    digitalWrite(SEG_DIGIT2_PIN, LOW);
}

// ========================================
// STARTUP DISPLAY SEQUENCE
// ========================================
void showStartupDisplay() {
    // Show "88" test
    sendToShiftRegister(0xFF);
    digitalWrite(SEG_DIGIT1_PIN, HIGH);
    digitalWrite(SEG_DIGIT2_PIN, HIGH);
    delay(1000);
    
    // Clear display
    sendToShiftRegister(0x00);
    digitalWrite(SEG_DIGIT1_PIN, LOW);
    digitalWrite(SEG_DIGIT2_PIN, LOW);
    delay(500);
    
    // Show current credits
    displayValue = totalCredits;
    displayNeedsUpdate = true;
}

// ========================================
// SPIFFS INITIALIZATION
// ========================================
void initSPIFFS() {
    Serial.println("[SPIFFS] Mounting filesystem...");
    
    if (!SPIFFS.begin(true)) {
        Serial.println("[SPIFFS] Mount failed, formatting...");
        if (SPIFFS.format()) {
            Serial.println("[SPIFFS] Format complete");
            if (SPIFFS.begin(true)) {
                Serial.println("[SPIFFS] Mounted after format");
                createDefaultWebFiles();
            }
        } else {
            Serial.println("[SPIFFS] Format failed!");
            systemError = true;
            systemErrorMessage = "SPIFFS initialization failed";
        }
    } else {
        Serial.println("[SPIFFS] Mounted successfully");
        Serial.printf("[SPIFFS] Total: %d bytes\n", SPIFFS.totalBytes());
        Serial.printf("[SPIFFS] Used: %d bytes\n", SPIFFS.usedBytes());
        Serial.printf("[SPIFFS] Free: %d bytes\n", SPIFFS.totalBytes() - SPIFFS.usedBytes());
        
        // List files
        listSPIFFSFiles();
    }
}

// ========================================
// LIST SPIFFS FILES
// ========================================
void listSPIFFSFiles() {
    Serial.println("[SPIFFS] Files:");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        Serial.printf("  %s (%d bytes)\n", file.name(), file.size());
        file = root.openNextFile();
    }
}

// ========================================
// CREATE DEFAULT WEB FILES
// ========================================
void createDefaultWebFiles() {
    Serial.println("[SPIFFS] Creating default web files...");
    
    // Create dashboard HTML
    File file = SPIFFS.open("/index.html", FILE_WRITE);
    if (file) {
        file.print(getDashboardHTML());
        file.close();
        Serial.println("[SPIFFS] Created /index.html");
    }
    
    // Create diagnostic page
    file = SPIFFS.open("/diagnostic.html", FILE_WRITE);
    if (file) {
        file.print(getDiagnosticHTML());
        file.close();
        Serial.println("[SPIFFS] Created /diagnostic.html");
    }
    
    // Create config page
    file = SPIFFS.open("/config.html", FILE_WRITE);
    if (file) {
        file.print(getConfigHTML());
        file.close();
        Serial.println("[SPIFFS] Created /config.html");
    }
    
    // Create login page
    file = SPIFFS.open("/login.html", FILE_WRITE);
    if (file) {
        file.print(getLoginHTML());
        file.close();
        Serial.println("[SPIFFS] Created /login.html");
    }
}

// ========================================
// CONFIGURATION MANAGEMENT
// ========================================
void loadConfiguration() {
    Serial.println("[CONFIG] Loading configuration...");
    
    preferences.begin("vending", false);
    
    // Load WiFi settings
    wifiSSID = preferences.getString("wifi_ssid", "");
    wifiPassword = preferences.getString("wifi_pass", "");
    apSSID = preferences.getString("ap_ssid", "VENDING_MACHINE");
    apPassword = preferences.getString("ap_pass", "12345678");
    hostname = preferences.getString("hostname", "vending-machine");
    
    // Load authentication
    authUsername = preferences.getString("auth_user", "admin");
    authPassword = preferences.getString("auth_pass", "admin");
    authRequired = preferences.getBool("auth_req", true);
    
    // Load service configuration
    for (int i = 0; i < NUM_SERVICES; i++) {
        String prefix = "svc" + String(i);
        services[i].duration = preferences.getULong((prefix + "_dur").c_str(), DEFAULT_SERVICE_TIME);
        services[i].cost = preferences.getUInt((prefix + "_cost").c_str(), 1);
        services[i].enabled = preferences.getBool((prefix + "_en").c_str(), true);
    }
    
    // Load counters
    totalCredits = preferences.getInt("credits", 0);
    lifetimeCoins = preferences.getInt("life_coins", 0);
    for (int i = 0; i < NUM_SERVICES; i++) {
        serviceCount[i] = preferences.getUInt(("svc" + String(i) + "_cnt").c_str(), 0);
    }
    
    preferences.end();
    
    Serial.println("[CONFIG] Configuration loaded");
    Serial.printf("[CONFIG] WiFi SSID: %s\n", wifiSSID.isEmpty() ? "(not set)" : wifiSSID.c_str());
    Serial.printf("[CONFIG] Credits: %d\n", totalCredits);
}

void saveConfiguration() {
    Serial.println("[CONFIG] Saving configuration...");
    
    preferences.begin("vending", false);
    
    // Save WiFi settings
    preferences.putString("wifi_ssid", wifiSSID);
    preferences.putString("wifi_pass", wifiPassword);
    preferences.putString("ap_ssid", apSSID);
    preferences.putString("ap_pass", apPassword);
    preferences.putString("hostname", hostname);
    
    // Save authentication
    preferences.putString("auth_user", authUsername);
    preferences.putString("auth_pass", authPassword);
    preferences.putBool("auth_req", authRequired);
    
    // Save service configuration
    for (int i = 0; i < NUM_SERVICES; i++) {
        String prefix = "svc" + String(i);
        preferences.putULong((prefix + "_dur").c_str(), services[i].duration);
        preferences.putUInt((prefix + "_cost").c_str(), services[i].cost);
        preferences.putBool((prefix + "_en").c_str(), services[i].enabled);
    }
    
    // Save counters
    preferences.putInt("credits", totalCredits);
    preferences.putInt("life_coins", lifetimeCoins);
    for (int i = 0; i < NUM_SERVICES; i++) {
        preferences.putUInt(("svc" + String(i) + "_cnt").c_str(), serviceCount[i]);
    }
    
    preferences.end();
    
    Serial.println("[CONFIG] Configuration saved");
}

// ========================================
// WIFI INITIALIZATION
// ========================================
void initWiFi() {
    Serial.println("[WiFi] Initializing...");
    
    WiFi.mode(WIFI_AP_STA);
    WiFi.setHostname(hostname.c_str());
    
    // Start AP mode first for immediate access
    WiFi.softAP(apSSID.c_str(), apPassword.c_str());
    apActive = true;
    Serial.printf("[WiFi] AP Mode: %s\n", apSSID.c_str());
    Serial.printf("[WiFi] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
    
    // Try to connect to configured network
    if (!wifiSSID.isEmpty()) {
        Serial.printf("[WiFi] Connecting to: %s\n", wifiSSID.c_str());
        WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
        wifiConnectStart = millis();
        wifiReconnectAttempts = 0;
    }
}

void checkWiFiConnection() {
    static bool lastWiFiStatus = false;
    
    wifiConnected = (WiFi.status() == WL_CONNECTED);
    
    // Status change detection
    if (wifiConnected != lastWiFiStatus) {
        lastWiFiStatus = wifiConnected;
        if (wifiConnected) {
            Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
            wifiReconnectAttempts = 0;
        } else {
            Serial.println("[WiFi] Disconnected");
        }
    }
    
    // Reconnection logic
    if (!wifiConnected && !wifiSSID.isEmpty() && wifiReconnectAttempts < 10) {
        if (millis() - wifiConnectStart > WIFI_TIMEOUT_MS) {
            Serial.println("[WiFi] Connection timeout, retrying...");
            WiFi.reconnect();
            wifiConnectStart = millis();
            wifiReconnectAttempts++;
        }
    }
}

// ========================================
// WEB SERVER INITIALIZATION
// ========================================
void initWebServer() {
    Serial.println("[WEB] Initializing server...");
    
    // CORS headers for all responses
    server.enableCORS(true);
    
    // Main pages
    server.on("/", HTTP_GET, handleRoot);
    server.on("/index.html", HTTP_GET, handleRoot);
    server.on("/diagnostic", HTTP_GET, handleDiagnosticPage);
    server.on("/config", HTTP_GET, handleConfigPage);
    server.on("/login", HTTP_GET, handleLoginPage);
    
    // Authentication
    server.on("/api/login", HTTP_POST, handleApiLogin);
    server.on("/api/logout", HTTP_GET, handleApiLogout);
    
    // Dashboard API
    server.on("/api/status", HTTP_GET, handleApiStatus);
    server.on("/api/dashboard", HTTP_GET, handleApiDashboard);
    
    // Service API
    server.on("/api/service/start", HTTP_POST, handleApiServiceStart);
    server.on("/api/service/stop", HTTP_POST, handleApiServiceStop);
    server.on("/api/service/config", HTTP_POST, handleApiServiceConfig);
    
    // Credit API
    server.on("/api/credits/add", HTTP_POST, handleApiAddCredits);
    server.on("/api/credits/reset", HTTP_POST, handleApiResetCredits);
    
    // Diagnostic API
    server.on("/api/diag/status", HTTP_GET, handleApiDiagStatus);
    server.on("/api/diag/test", HTTP_POST, handleApiDiagTest);
    
    // Configuration API
    server.on("/api/config/get", HTTP_GET, handleApiGetConfig);
    server.on("/api/config/set", HTTP_POST, handleApiSetConfig);
    server.on("/api/config/wifi", HTTP_POST, handleApiWiFiConfig);
    
    // System API
    server.on("/api/system/info", HTTP_GET, handleApiSystemInfo);
    server.on("/api/system/reset", HTTP_POST, handleApiSystemReset);
    server.on("/api/system/reboot", HTTP_POST, handleApiSystemReboot);
    
    // Firmware update
    server.on("/api/update", HTTP_POST, 
        []() { server.send(200, "text/plain", "OK"); },
        handleApiFirmwareUpdate
    );
    
    // Static files from SPIFFS
    server.serveStatic("/", SPIFFS, "/");
    
    // 404 handler
    server.onNotFound(handleNotFound);
    
    // Start server
    server.begin();
    Serial.println("[WEB] Server started on port 80");
}

// ========================================
// COIN ACCEPTOR INITIALIZATION
// ========================================
void initCoinAcceptor() {
    attachInterrupt(digitalPinToInterrupt(COIN_PIN), coinISR, FALLING);
    Serial.printf("[COIN] Acceptor initialized on GPIO%d\n", COIN_PIN);
}

// ========================================
// COIN INTERRUPT SERVICE ROUTINE
// ========================================
void IRAM_ATTR coinISR() {
    unsigned long now = millis();
    
    // Debounce (50ms)
    if (now - lastCoinPulseTime > 50) {
        coinPulses++;
        lastCoinPulseTime = now;
        
        // Track for short circuit detection
        if (now - coinWindowStart < COIN_SHORT_WINDOW_MS) {
            coinPulseCountWindow++;
        } else {
            coinWindowStart = now;
            coinPulseCountWindow = 1;
        }
    }
}

// ========================================
// COIN ACCEPTOR HANDLER
// ========================================
void handleCoinAcceptor() {
    // Short circuit detection
    if (coinPulseCountWindow > COIN_SHORT_THRESHOLD) {
        if (!coinShorted) {
            coinShorted = true;
            Serial.println("[COIN] SHORT CIRCUIT DETECTED!");
        }
        coinPulseCountWindow = 0;
        coinPulses = 0;
        return;
    }
    
    // Clear short circuit flag if pulses return to normal
    if (coinShorted && coinPulseCountWindow == 0) {
        coinShorted = false;
        Serial.println("[COIN] Short circuit cleared");
    }
    
    // Process accumulated pulses
    if (coinPulses > 0 && !coinShorted) {
        unsigned int pulses = coinPulses;
        coinPulses = 0;
        
        int creditsToAdd = pulses * COIN_PULSE_VALUE;
        totalCredits += creditsToAdd;
        lifetimeCoins += pulses;
        
        // Cap credits
        if (totalCredits > MAX_CREDITS) {
            totalCredits = MAX_CREDITS;
        }
        
        Serial.printf("[COIN] %d pulse(s) -> %d credit(s) (Total: %d)\n", 
                     pulses, creditsToAdd, totalCredits);
        
        // Play coin sound
        playBuzzerPattern(2);
        
        // Update display
        displayValue = totalCredits;
        displayNeedsUpdate = true;
        
        // Save state
        saveConfiguration();
    }
}

// ========================================
// BUTTON READING
// ========================================
void readButtons() {
    int buttonPins[NUM_SERVICES] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
    
    for (int i = 0; i < NUM_SERVICES; i++) {
        // Read button (active low due to pullup)
        bool reading = !digitalRead(buttonPins[i]);
        
        // Check for change
        if (reading != lastButtonReading[i]) {
            lastButtonDebounce[i] = millis();
        }
        
        // Debounce check
        if ((millis() - lastButtonDebounce[i]) > DEBOUNCE_MS) {
            // State has changed
            if (reading != buttonStates[i]) {
                buttonStates[i] = reading;
                
                // Button pressed
                if (reading) {
                    buttonPressed[i] = true;
                    Serial.printf("[BUTTON] Button %d pressed\n", i + 1);
                }
            }
        }
        
        lastButtonReading[i] = reading;
    }
    
    // Process button presses
    for (int i = 0; i < NUM_SERVICES; i++) {
        if (buttonPressed[i]) {
            buttonPressed[i] = false;
            handleButtonPress(i);
        }
    }
}

void handleButtonPress(int serviceIndex) {
    if (diagnosticMode) {
        // In diagnostic mode, buttons are handled by web interface
        return;
    }
    
    if (!services[serviceIndex].enabled) {
        Serial.printf("[SERVICE] Service %d is disabled\n", serviceIndex + 1);
        playBuzzerPattern(4); // Error sound
        return;
    }
    
    if (serviceStates[serviceIndex] == STATE_ACTIVE) {
        // Service already running
        Serial.printf("[SERVICE] Service %d is already active\n", serviceIndex + 1);
        return;
    }
    
    if (totalCredits >= services[serviceIndex].cost) {
        // Deduct credits and start service
        totalCredits -= services[serviceIndex].cost;
        startService(serviceIndex);
        displayValue = totalCredits;
        displayNeedsUpdate = true;
        saveConfiguration();
    } else {
        // Insufficient credits
        Serial.printf("[SERVICE] Insufficient credits for Service %d (need %d, have %d)\n",
                     serviceIndex + 1, services[serviceIndex].cost, totalCredits);
        playBuzzerPattern(4); // Error sound
        flashDisplay(3); // Flash to indicate error
    }
}

// ========================================
// SENSOR READING
// ========================================
void readSensor() {
    bool newState = digitalRead(SENSOR_PIN);
    
    if (newState != lastSensorState) {
        lastSensorState = newState;
        
        if (newState) {
            sensorState = true;
            sensorTriggerTime = millis();
            sensorTriggerCount++;
            digitalWrite(SENSOR_LED_PIN, HIGH);
            Serial.println("[SENSOR] Triggered");
        } else {
            sensorState = false;
            digitalWrite(SENSOR_LED_PIN, LOW);
            Serial.println("[SENSOR] Released");
        }
    }
}

// ========================================
// SERVICE CONTROL
// ========================================
void startService(int serviceIndex) {
    if (serviceIndex < 0 || serviceIndex >= NUM_SERVICES) return;
    
    serviceStates[serviceIndex] = STATE_ACTIVE;
    serviceStartTime[serviceIndex] = millis();
    serviceElapsedTime[serviceIndex] = 0;
    
    // Activate relay
    digitalWrite(services[serviceIndex].relayPin, HIGH);
    relayStates[serviceIndex] = true;
    
    // Play start sound
    playBuzzerPattern(3);
    
    // Update display
    currentDisplayMode = serviceIndex + 1;
    displayNeedsUpdate = true;
    
    // Enable LED frame if any service is active
    updateLEDFrame();
    
    Serial.printf("[SERVICE] Service %d started (Duration: %lu ms)\n",
                 serviceIndex + 1, services[serviceIndex].duration);
}

void stopService(int serviceIndex) {
    if (serviceIndex < 0 || serviceIndex >= NUM_SERVICES) return;
    
    serviceStates[serviceIndex] = STATE_COMPLETED;
    
    // Deactivate relay
    digitalWrite(services[serviceIndex].relayPin, LOW);
    relayStates[serviceIndex] = false;
    
    // Update counters
    serviceCount[serviceIndex]++;
    
    // Reset display to credits
    currentDisplayMode = 0;
    displayValue = totalCredits;
    displayNeedsUpdate = true;
    
    // Update LED frame
    updateLEDFrame();
    
    // Save updated counters
    saveConfiguration();
    
    Serial.printf("[SERVICE] Service %d completed (Count: %u)\n",
                 serviceIndex + 1, serviceCount[serviceIndex]);
}

void handleServices() {
    for (int i = 0; i < NUM_SERVICES; i++) {
        if (serviceStates[i] == STATE_ACTIVE) {
            serviceElapsedTime[i] = millis() - serviceStartTime[i];
            
            // Check if service time has expired
            if (serviceElapsedTime[i] >= services[i].duration) {
                stopService(i);
            }
            
            // Update display if this service is being shown
            if (currentDisplayMode == i + 1) {
                unsigned long remaining = services[i].duration - serviceElapsedTime[i];
                displayValue = (remaining + 999) / 1000; // Convert to seconds
                if (displayValue > 99) displayValue = 99;
            }
        }
    }
    
    // Check if all services completed and reset display
    bool anyActive = false;
    for (int i = 0; i < NUM_SERVICES; i++) {
        if (serviceStates[i] == STATE_ACTIVE) {
            anyActive = true;
            break;
        }
    }
    
    if (!anyActive && currentDisplayMode > 0) {
        currentDisplayMode = 0;
        displayValue = totalCredits;
        displayNeedsUpdate = true;
    }
}

void updateLEDFrame() {
    bool anyActive = false;
    for (int i = 0; i < NUM_SERVICES; i++) {
        if (serviceStates[i] == STATE_ACTIVE) {
            anyActive = true;
            break;
        }
    }
    digitalWrite(LED_FRAME_PIN, anyActive ? HIGH : LOW);
}

// ========================================
// RELAY CONTROL
// ========================================
void allRelaysOff() {
    for (int i = 0; i < NUM_SERVICES; i++) {
        digitalWrite(services[i].relayPin, LOW);
        relayStates[i] = false;
        relayTimingActive[i] = false;
    }
    updateLEDFrame();
}

void relayAllSequence(int delayMs) {
    Serial.printf("[RELAY] Sequential test with %d ms delay\n", delayMs);
    
    for (int i = 0; i < NUM_SERVICES; i++) {
        relayTimingActive[i] = true;
        relayTimingStart[i] = millis();
        relayTimingDuration[i] = delayMs;
    }
}

void handleRelayTiming() {
    for (int i = 0; i < NUM_SERVICES; i++) {
        if (relayTimingActive[i]) {
            unsigned long elapsed = millis() - relayTimingStart[i];
            
            if (elapsed < relayTimingDuration[i]) {
                digitalWrite(services[i].relayPin, HIGH);
            } else if (elapsed < relayTimingDuration[i] * 2) {
                digitalWrite(services[i].relayPin, LOW);
            } else {
                relayTimingActive[i] = false;
            }
        }
    }
}

// ========================================
// BUZZER CONTROL
// ========================================
void playBuzzerPattern(int pattern) {
    buzzerPattern = pattern;
    buzzerActive = true;
    buzzerStartTime = millis();
    buzzerLastToggle = 0;
    buzzerState = false;
    digitalWrite(BUZZER_PIN, LOW);
}

void handleBuzzer() {
    if (!buzzerActive) return;
    
    unsigned long elapsed = millis() - buzzerStartTime;
    
    switch (buzzerPattern) {
        case 1: // Simple beep - 200ms
            if (elapsed < 200) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerActive = false;
            }
            break;
            
        case 2: // Coin sound - two quick beeps
            if (elapsed < 80) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else if (elapsed < 160) {
                digitalWrite(BUZZER_PIN, LOW);
            } else if (elapsed < 240) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else if (elapsed < 320) {
                digitalWrite(BUZZER_PIN, LOW);
            } else {
                buzzerActive = false;
            }
            break;
            
        case 3: // Start sound - long beep 500ms
            if (elapsed < 500) {
                digitalWrite(BUZZER_PIN, HIGH);
            } else {
                digitalWrite(BUZZER_PIN, LOW);
                buzzerActive = false;
            }
            break;
            
        case 4: // Error sound - three rapid beeps
            {
                int phase = (elapsed / 150) % 6;
                if (phase < 3 && (elapsed % 150) < 75) {
                    digitalWrite(BUZZER_PIN, HIGH);
                } else {
                    digitalWrite(BUZZER_PIN, LOW);
                }
                if (elapsed > 600) {
                    buzzerActive = false;
                }
            }
            break;
            
        default:
            buzzerActive = false;
            digitalWrite(BUZZER_PIN, LOW);
            break;
    }
}

// ========================================
// DISPLAY CONTROL
// ========================================
void updateDisplayValue() {
    if (currentDisplayMode == 0) {
        displayValue = totalCredits;
    }
    displayNeedsUpdate = false;
}

void updateDisplayMultiplex() {
    static unsigned long lastMicros = 0;
    unsigned long now = micros();
    
    if (now - lastMicros >= DISPLAY_REFRESH_US) {
        lastMicros = now;
        
        // Turn off current digit
        digitalWrite(SEG_DIGIT1_PIN, LOW);
        digitalWrite(SEG_DIGIT2_PIN, LOW);
        
        // Alternate between digit 1 and 2
        currentDigit = !currentDigit;
        
        int value = displayValue;
        if (value < 0) value = 0;
        if (value > 99) value = 99;
        
        if (currentDigit == 0) {
            // Tens digit
            int digit = (value / 10) % 10;
            if (value < 10 && currentDisplayMode == 0) {
                // Blank leading zero for credits
                sendToShiftRegister(0x00);
            } else {
                sendToShiftRegister(segmentMap[digit]);
            }
            digitalWrite(SEG_DIGIT1_PIN, HIGH);
        } else {
            // Ones digit
            int digit = value % 10;
            sendToShiftRegister(segmentMap[digit]);
            digitalWrite(SEG_DIGIT2_PIN, HIGH);
        }
    }
}

void sendToShiftRegister(uint8_t data) {
    digitalWrite(SEG_RCLK_PIN, LOW);
    shiftOut(SEG_SER_PIN, SEG_SCLK_PIN, MSBFIRST, data);
    digitalWrite(SEG_RCLK_PIN, HIGH);
}

void flashDisplay(int times) {
    for (int i = 0; i < times; i++) {
        digitalWrite(SEG_DIGIT1_PIN, LOW);
        digitalWrite(SEG_DIGIT2_PIN, LOW);
        delay(200);
        // Display will resume on next multiplex cycle
        delay(200);
    }
}

// ========================================
// DIAGNOSTIC MODE
// ========================================
void handleDiagnosticMode() {
    if (!diagnosticMode) return;
    
    // Blink sensor LED in diagnostic mode
    if (millis() - diagnosticBlinkTimer > 500) {
        diagnosticBlinkTimer = millis();
        diagnosticBlinkState = !diagnosticBlinkState;
        digitalWrite(SENSOR_LED_PIN, diagnosticBlinkState ? HIGH : LOW);
    }
}

// ========================================
// SYSTEM FUNCTIONS
// ========================================
void updateSystemStatus() {
    // Update any system-level status indicators
}

void checkSystemHealth() {
    // Check free heap
    if (ESP.getFreeHeap() < 10000) {
        Serial.printf("[WARNING] Low memory: %d bytes free\n", ESP.getFreeHeap());
    }
    
    // Check WiFi
    if (!wifiConnected && !apActive) {
        Serial.println("[WARNING] No network connectivity");
    }
}

void logSystemStatus() {
    Serial.println("========================================");
    Serial.println("[STATUS] System Status");
    Serial.printf("  Uptime: %lu seconds\n", (millis() - systemStartTime) / 1000);
    Serial.printf("  Free Heap: %d bytes\n", ESP.getFreeHeap());
    Serial.printf("  WiFi: %s\n", wifiConnected ? "Connected" : "Disconnected");
    Serial.printf("  AP: %s\n", apActive ? "Active" : "Inactive");
    Serial.printf("  Credits: %d\n", totalCredits);
    Serial.printf("  Lifetime Coins: %d\n", lifetimeCoins);
    Serial.printf("  Coin Shorted: %s\n", coinShorted ? "YES" : "No");
    Serial.printf("  Sensor: %s\n", sensorState ? "HIGH" : "LOW");
    Serial.printf("  Diagnostic Mode: %s\n", diagnosticMode ? "ON" : "OFF");
    
    for (int i = 0; i < NUM_SERVICES; i++) {
        Serial.printf("  Service %d: %s (Count: %u)\n", 
                     i + 1, 
                     serviceStates[i] == STATE_ACTIVE ? "ACTIVE" : "IDLE",
                     serviceCount[i]);
    }
    Serial.println("========================================");
}

// ========================================
// WEB PAGE HANDLERS
// ========================================
void handleRoot() {
    if (authRequired && !isAuthenticated()) {
        server.sendHeader("Location", "/login");
        server.send(302, "text/plain", "");
        return;
    }
    
    File file = SPIFFS.open("/index.html", "r");
    if (file) {
        server.streamFile(file, "text/html");
        file.close();
    } else {
        server.send(200, "text/html", getDashboardHTML());
    }
}

void handleDiagnosticPage() {
    if (authRequired && !isAuthenticated()) {
        server.sendHeader("Location", "/login");
        server.send(302, "text/plain", "");
        return;
    }
    
    diagnosticMode = true;
    
    File file = SPIFFS.open("/diagnostic.html", "r");
    if (file) {
        server.streamFile(file, "text/html");
        file.close();
    } else {
        server.send(200, "text/html", getDiagnosticHTML());
    }
}

void handleConfigPage() {
    if (authRequired && !isAuthenticated()) {
        server.sendHeader("Location", "/login");
        server.send(302, "text/plain", "");
        return;
    }
    
    File file = SPIFFS.open("/config.html", "r");
    if (file) {
        server.streamFile(file, "text/html");
        file.close();
    } else {
        server.send(200, "text/html", getConfigHTML());
    }
}

void handleLoginPage() {
    File file = SPIFFS.open("/login.html", "r");
    if (file) {
        server.streamFile(file, "text/html");
        file.close();
    } else {
        server.send(200, "text/html", getLoginHTML());
    }
}

void handleNotFound() {
    server.send(404, "text/plain", "404: Not Found");
}

// ========================================
// API HANDLERS
// ========================================
void handleApiLogin() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));
    
    String username = doc["username"] | "";
    String password = doc["password"] | "";
    
    if (username == authUsername && password == authPassword) {
        // Generate simple session token
        sessionToken = String(ESP.getEfuseMac()) + String(millis());
        server.send(200, "application/json", 
                   "{\"success\":true,\"token\":\"" + sessionToken + "\"}");
        Serial.println("[AUTH] Login successful");
    } else {
        server.send(401, "application/json", "{\"error\":\"Invalid credentials\"}");
        Serial.println("[AUTH] Login failed");
    }
}

void handleApiLogout() {
    sessionToken = "";
    server.send(200, "application/json", "{\"success\":true}");
}

bool isAuthenticated() {
    if (!authRequired) return true;
    
    // Check for session token in header or cookie
    if (server.hasHeader("Authorization")) {
        String auth = server.header("Authorization");
        if (auth == "Bearer " + sessionToken && !sessionToken.isEmpty()) {
            return true;
        }
    }
    return false;
}

void handleApiStatus() {
    DynamicJsonDocument doc(1024);
    
    doc["uptime"] = (millis() - systemStartTime) / 1000;
    doc["credits"] = totalCredits;
    doc["sensor"] = sensorState;
    doc["coinShorted"] = coinShorted;
    doc["diagnosticMode"] = diagnosticMode;
    doc["wifiConnected"] = wifiConnected;
    doc["freeHeap"] = ESP.getFreeHeap();
    
    JsonArray svcArray = doc.createNestedArray("services");
    for (int i = 0; i < NUM_SERVICES; i++) {
        JsonObject svc = svcArray.createNestedObject();
        svc["id"] = i;
        svc["name"] = services[i].name;
        svc["code"] = services[i].code;
        svc["state"] = serviceStates[i];
        svc["enabled"] = services[i].enabled;
        svc["cost"] = services[i].cost;
        svc["count"] = serviceCount[i];
        
        if (serviceStates[i] == STATE_ACTIVE) {
            svc["elapsed"] = serviceElapsedTime[i];
            svc["remaining"] = services[i].duration - serviceElapsedTime[i];
            svc["duration"] = services[i].duration;
        }
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiDashboard() {
    DynamicJsonDocument doc(512);
    
    doc["credits"] = totalCredits;
    doc["lifetimeCoins"] = lifetimeCoins;
    doc["sensor"] = sensorState;
    doc["ledFrame"] = digitalRead(LED_FRAME_PIN);
    
    JsonArray svcArray = doc.createNestedArray("services");
    for (int i = 0; i < NUM_SERVICES; i++) {
        JsonObject svc = svcArray.createNestedObject();
        svc["name"] = services[i].name;
        svc["active"] = (serviceStates[i] == STATE_ACTIVE);
        svc["enabled"] = services[i].enabled;
        if (serviceStates[i] == STATE_ACTIVE) {
            svc["remaining"] = (services[i].duration - serviceElapsedTime[i]) / 1000;
        }
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiServiceStart() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(128);
    deserializeJson(doc, server.arg("plain"));
    
    int serviceId = doc["service"] | -1;
    
    if (serviceId < 0 || serviceId >= NUM_SERVICES) {
        server.send(400, "application/json", "{\"error\":\"Invalid service ID\"}");
        return;
    }
    
    if (!services[serviceId].enabled) {
        server.send(400, "application/json", "{\"error\":\"Service disabled\"}");
        return;
    }
    
    if (serviceStates[serviceId] == STATE_ACTIVE) {
        server.send(400, "application/json", "{\"error\":\"Service already active\"}");
        return;
    }
    
    if (totalCredits >= services[serviceId].cost) {
        totalCredits -= services[serviceId].cost;
        startService(serviceId);
        displayValue = totalCredits;
        displayNeedsUpdate = true;
        saveConfiguration();
        
        DynamicJsonDocument resp(128);
        resp["success"] = true;
        resp["credits"] = totalCredits;
        String response;
        serializeJson(resp, response);
        server.send(200, "application/json", response);
    } else {
        server.send(400, "application/json", "{\"error\":\"Insufficient credits\"}");
    }
}

void handleApiServiceStop() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(128);
    deserializeJson(doc, server.arg("plain"));
    
    int serviceId = doc["service"] | -1;
    
    if (serviceId >= 0 && serviceId < NUM_SERVICES) {
        if (serviceStates[serviceId] == STATE_ACTIVE) {
            stopService(serviceId);
            server.send(200, "application/json", "{\"success\":true}");
        } else {
            server.send(400, "application/json", "{\"error\":\"Service not active\"}");
        }
    } else {
        server.send(400, "application/json", "{\"error\":\"Invalid service\"}");
    }
}

void handleApiServiceConfig() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));
    
    int serviceId = doc["service"] | -1;
    
    if (serviceId >= 0 && serviceId < NUM_SERVICES) {
        if (doc.containsKey("duration")) {
            unsigned long dur = doc["duration"];
            if (dur >= MIN_SERVICE_TIME && dur <= MAX_SERVICE_TIME) {
                services[serviceId].duration = dur;
            }
        }
        if (doc.containsKey("cost")) {
            services[serviceId].cost = doc["cost"];
        }
        if (doc.containsKey("enabled")) {
            services[serviceId].enabled = doc["enabled"];
        }
        if (doc.containsKey("name")) {
            services[serviceId].name = strdup(doc["name"].as<const char*>());
        }
        
        saveConfiguration();
        server.send(200, "application/json", "{\"success\":true}");
    } else {
        server.send(400, "application/json", "{\"error\":\"Invalid service\"}");
    }
}

void handleApiAddCredits() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(128);
    deserializeJson(doc, server.arg("plain"));
    
    int credits = doc["credits"] | 0;
    
    if (credits > 0) {
        totalCredits += credits;
        if (totalCredits > MAX_CREDITS) totalCredits = MAX_CREDITS;
        
        displayValue = totalCredits;
        displayNeedsUpdate = true;
        playBuzzerPattern(2);
        saveConfiguration();
        
        DynamicJsonDocument resp(64);
        resp["credits"] = totalCredits;
        String response;
        serializeJson(resp, response);
        server.send(200, "application/json", response);
    } else {
        server.send(400, "application/json", "{\"error\":\"Invalid amount\"}");
    }
}

void handleApiResetCredits() {
    totalCredits = 0;
    displayValue = 0;
    displayNeedsUpdate = true;
    saveConfiguration();
    server.send(200, "application/json", "{\"success\":true,\"credits\":0}");
}

void handleApiDiagStatus() {
    DynamicJsonDocument doc(512);
    
    // Button states
    JsonArray buttons = doc.createNestedArray("buttons");
    int buttonPins[NUM_SERVICES] = {BUTTON1_PIN, BUTTON2_PIN, BUTTON3_PIN, BUTTON4_PIN};
    for (int i = 0; i < NUM_SERVICES; i++) {
        buttons.add(!digitalRead(buttonPins[i]));
    }
    
    // Sensor
    doc["sensor"] = digitalRead(SENSOR_PIN);
    
    // Relays
    JsonArray relays = doc.createNestedArray("relays");
    for (int i = 0; i < NUM_SERVICES; i++) {
        relays.add(digitalRead(services[i].relayPin));
    }
    
    // Coin
    doc["coinPulses"] = coinPulses;
    doc["coinShorted"] = coinShorted;
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiDiagTest() {
    if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Missing body");
        return;
    }
    
    String body = server.arg("plain");
    DynamicJsonDocument doc(256);
    deserializeJson(doc, body);
    
    String type = doc["type"];
    int value = doc["value"];
    
    Serial.printf("[DIAG] Test - type: %s, value: %d\n", type.c_str(), value);
    
    if (type == "sled") {
        if (value == 0) digitalWrite(SENSOR_LED_PIN, LOW);
        else if (value == 1) digitalWrite(SENSOR_LED_PIN, HIGH);
        else if (value == 2) {
            // Blink mode handled in diagnostic loop
        }
    }
    else if (type == "ledf") {
        digitalWrite(LED_FRAME_PIN, value);
    }
    else if (type == "relayon") {
        if (value >= 0 && value < NUM_SERVICES) {
            relayStates[value] = !relayStates[value];
            digitalWrite(services[value].relayPin, relayStates[value]);
        }
    }
    else if (type == "relayall") {
        relayAllSequence(value);
    }
    else if (type == "relay") {
        if (value == -1) allRelaysOff();
    }
    else if (type == "buz") {
        playBuzzerPattern(value);
    }
    else if (type == "coinrst") {
        coinPulses = 0;
        coinPulseCountWindow = 0;
        coinShorted = false;
    }
    else if (type == "segd") {
        if (value == -1) {
            for (int i = 0; i < NUM_SERVICES; i++) {
                // Clear individual displays
            }
        } else if (value >= 0 && value <= 4) {
            currentDisplayMode = value;
            if (value == 0) displayValue = totalCredits;
            else displayValue = value;
            displayNeedsUpdate = true;
        }
    }
    else if (type == "exit") {
        diagnosticMode = false;
        digitalWrite(SENSOR_LED_PIN, sensorState);
        updateLEDFrame();
        currentDisplayMode = 0;
        displayValue = totalCredits;
        displayNeedsUpdate = true;
    }
    
    server.send(200, "text/plain", "OK");
}

void handleApiGetConfig() {
    DynamicJsonDocument doc(1024);
    
    // WiFi config
    doc["wifi_ssid"] = wifiSSID;
    doc["wifi_connected"] = wifiConnected;
    doc["ap_ssid"] = apSSID;
    doc["hostname"] = hostname;
    doc["auth_required"] = authRequired;
    
    // Service config
    JsonArray svcArray = doc.createNestedArray("services");
    for (int i = 0; i < NUM_SERVICES; i++) {
        JsonObject svc = svcArray.createNestedObject();
        svc["name"] = services[i].name;
        svc["code"] = services[i].code;
        svc["duration"] = services[i].duration;
        svc["cost"] = services[i].cost;
        svc["enabled"] = services[i].enabled;
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiSetConfig() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(512);
    deserializeJson(doc, server.arg("plain"));
    
    if (doc.containsKey("auth_required")) {
        authRequired = doc["auth_required"];
    }
    if (doc.containsKey("hostname")) {
        hostname = doc["hostname"].as<String>();
    }
    
    saveConfiguration();
    server.send(200, "application/json", "{\"success\":true}");
}

void handleApiWiFiConfig() {
    if (!server.hasArg("plain")) {
        server.send(400, "application/json", "{\"error\":\"Missing body\"}");
        return;
    }
    
    DynamicJsonDocument doc(256);
    deserializeJson(doc, server.arg("plain"));
    
    wifiSSID = doc["ssid"] | "";
    wifiPassword = doc["password"] | "";
    
    saveConfiguration();
    
    // Reconnect with new credentials
    WiFi.disconnect();
    delay(500);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    wifiConnectStart = millis();
    
    server.send(200, "application/json", "{\"success\":true}");
}

void handleApiSystemInfo() {
    DynamicJsonDocument doc(256);
    
    doc["uptime"] = (millis() - systemStartTime) / 1000;
    doc["freeHeap"] = ESP.getFreeHeap();
    doc["chipModel"] = ESP.getChipModel();
    doc["chipRevision"] = ESP.getChipRevision();
    doc["cpuFreq"] = ESP.getCpuFreqMHz();
    doc["sdkVersion"] = ESP.getSdkVersion();
    doc["flashSize"] = ESP.getFlashChipSize();
    doc["sketchSize"] = ESP.getSketchSize();
    doc["freeSketchSpace"] = ESP.getFreeSketchSpace();
    doc["spiffsTotal"] = SPIFFS.totalBytes();
    doc["spiffsUsed"] = SPIFFS.usedBytes();
    
    if (wifiConnected) {
        doc["wifiIP"] = WiFi.localIP().toString();
        doc["wifiRSSI"] = WiFi.RSSI();
    }
    
    String response;
    serializeJson(doc, response);
    server.send(200, "application/json", response);
}

void handleApiSystemReset() {
    // Reset all counters
    totalCredits = 0;
    lifetimeCoins = 0;
    for (int i = 0; i < NUM_SERVICES; i++) {
        serviceCount[i] = 0;
        if (serviceStates[i] == STATE_ACTIVE) {
            stopService(i);
        }
    }
    displayValue = 0;
    displayNeedsUpdate = true;
    saveConfiguration();
    
    server.send(200, "application/json", "{\"success\":true}");
}

void handleApiSystemReboot() {
    server.send(200, "application/json", "{\"success\":true,\"message\":\"Rebooting...\"}");
    delay(500);
    ESP.restart();
}

void handleApiFirmwareUpdate() {
    HTTPUpload& upload = server.upload();
    
    if (upload.status == UPLOAD_FILE_START) {
        Serial.printf("[UPDATE] Starting firmware update: %s\n", upload.filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            Update.printError(Serial);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            Serial.printf("[UPDATE] Success: %u bytes\n", upload.totalSize);
            server.send(200, "text/plain", "Update successful! Rebooting...");
            delay(1000);
            ESP.restart();
        } else {
            Update.printError(Serial);
            server.send(500, "text/plain", "Update failed");
        }
    }
}

// ========================================
// HTML PAGES
// ========================================
String getDashboardHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
    <title>N&R Solartech - Vending Machine</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:system-ui,-apple-system,sans-serif;background:linear-gradient(135deg,#0f172a 0%,#1e293b 100%);color:#e2e8f0;min-height:100vh;padding:16px}
        .header{text-align:center;padding:20px 0;border-bottom:1px solid #334155;margin-bottom:20px}
        .header h1{color:#f59e0b;font-size:24px;font-weight:700;margin-bottom:4px}
        .header .subtitle{color:#94a3b8;font-size:12px}
        .credits-card{background:linear-gradient(135deg,#1e293b,#0f172a);border:2px solid #f59e0b;border-radius:16px;padding:24px;text-align:center;margin-bottom:20px;box-shadow:0 4px 20px rgba(245,158,11,0.1)}
        .credits-value{font-size:64px;font-weight:800;color:#f59e0b;line-height:1}
        .credits-label{color:#94a3b8;font-size:14px;margin-top:8px;text-transform:uppercase;letter-spacing:2px}
        .services-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:20px}
        .service-card{background:#1e293b;border-radius:12px;padding:16px;text-align:center;border:2px solid #334155;transition:all 0.2s;cursor:pointer}
        .service-card:active{transform:scale(0.96)}
        .service-card.active{border-color:#22c55e;background:#1a2e1f;box-shadow:0 0 20px rgba(34,197,94,0.2)}
        .service-card.disabled{border-color:#7f1d1d;background:#1f1414;opacity:0.5}
        .service-icon{font-size:36px;margin-bottom:8px}
        .service-name{font-weight:700;font-size:16px;margin-bottom:4px}
        .service-cost{color:#94a3b8;font-size:12px}
        .service-timer{color:#22c55e;font-size:20px;font-weight:700;margin-top:8px;display:none}
        .service-card.active .service-timer{display:block}
        .status-bar{display:flex;justify-content:space-around;align-items:center;padding:12px;background:#1e293b;border-radius:12px;margin-bottom:20px}
        .status-item{text-align:center}
        .status-dot{width:10px;height:10px;border-radius:50%;display:inline-block;margin-right:4px}
        .status-dot.on{background:#22c55e;box-shadow:0 0 8px #22c55e}
        .status-dot.off{background:#ef4444;box-shadow:0 0 8px #ef4444}
        .nav-bar{display:flex;gap:8px;justify-content:center;flex-wrap:wrap}
        .nav-btn{padding:10px 20px;border:none;border-radius:8px;font-size:12px;font-weight:600;cursor:pointer;text-decoration:none;transition:all 0.2s;color:#fff}
        .nav-btn.primary{background:#0ea5e9}
        .nav-btn.secondary{background:#334155}
        .nav-btn.danger{background:#ef4444}
        .nav-btn:active{transform:scale(0.95)}
        .sensor-alert{background:#7f1d1d;border:1px solid #ef4444;border-radius:8px;padding:8px 12px;text-align:center;margin-top:8px;display:none;color:#fca5a5;font-size:12px}
    </style>
</head>
<body>
    <div class="header">
        <h1>N&R Solartech Sol</h1>
        <div class="subtitle">Vending Machine Controller</div>
    </div>
    
    <div class="credits-card">
        <div class="credits-value" id="credits">0</div>
        <div class="credits-label">Credits Available</div>
    </div>
    
    <div class="services-grid" id="services"></div>
    
    <div class="status-bar">
        <div class="status-item">
            <span class="status-dot off" id="sensorDot"></span>
            <span style="font-size:12px">Sensor</span>
        </div>
        <div class="status-item">
            <span class="status-dot off" id="wifiDot"></span>
            <span style="font-size:12px">WiFi</span>
        </div>
        <div class="status-item">
            <span style="font-size:12px;color:#94a3b8" id="uptime">--</span>
        </div>
    </div>
    
    <div class="sensor-alert" id="sensorAlert">Sensor Triggered!</div>
    
    <div class="nav-bar">
        <button class="nav-btn primary" onclick="window.location.href='/diagnostic'">Diagnostics</button>
        <button class="nav-btn secondary" onclick="window.location.href='/config'">Settings</button>
        <button class="nav-btn danger" onclick="logout()">Logout</button>
    </div>

    <script>
        const NUM_SERVICES = 4;
        let serviceData = [];
        let updateTimer = null;
        
        function init() {
            let html = '';
            for (let i = 0; i < NUM_SERVICES; i++) {
                html += `
                <div class="service-card" id="svc${i}" onclick="startService(${i})">
                    <div class="service-icon">🔄</div>
                    <div class="service-name" id="svcName${i}">Service ${i+1}</div>
                    <div class="service-cost" id="svcCost${i}">1 Credit</div>
                    <div class="service-timer" id="svcTimer${i}">--</div>
                </div>`;
            }
            document.getElementById('services').innerHTML = html;
            updateDashboard();
            updateTimer = setInterval(updateDashboard, 1000);
        }
        
        function updateDashboard() {
            fetch('/api/dashboard')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('credits').textContent = d.credits;
                    
                    for (let i = 0; i < NUM_SERVICES; i++) {
                        const svc = d.services[i];
                        const card = document.getElementById('svc' + i);
                        document.getElementById('svcName' + i).textContent = svc.name;
                        document.getElementById('svcCost' + i).textContent = svc.enabled ? svc.cost + ' Credit(s)' : 'Disabled';
                        
                        card.className = 'service-card';
                        if (svc.active) {
                            card.classList.add('active');
                            const mins = Math.floor(svc.remaining / 60);
                            const secs = svc.remaining % 60;
                            document.getElementById('svcTimer' + i).textContent = 
                                String(mins).padStart(2,'0') + ':' + String(secs).padStart(2,'0');
                        }
                        if (!svc.enabled) {
                            card.classList.add('disabled');
                        }
                    }
                    
                    // Update status dots
                    const sensorDot = document.getElementById('sensorDot');
                    sensorDot.className = 'status-dot ' + (d.sensor ? 'on' : 'off');
                    
                    const alert = document.getElementById('sensorAlert');
                    alert.style.display = d.sensor ? 'block' : 'none';
                })
                .catch(e => console.error('Update failed:', e));
        }
        
        function startService(id) {
            fetch('/api/service/start', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({service: id})
            })
            .then(r => r.json())
            .then(d => {
                if (!d.success && d.error) {
                    alert(d.error);
                }
            });
        }
        
        function logout() {
            if (confirm('Are you sure you want to logout?')) {
                fetch('/api/logout').then(() => window.location.reload());
            }
        }
        
        init();
    </script>
</body>
</html>
    )rawliteral";
}

String getDiagnosticHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
    <title>Diagnostic Mode</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;font-size:12px;padding:8px;max-height:100vh;overflow:hidden}
        .h{text-align:center;color:#f59e0b;font-size:16px;font-weight:700;margin-bottom:8px}
        .g{display:grid;grid-template-columns:1fr 1fr;gap:6px}
        .c{background:#1e293b;border-radius:8px;padding:8px}
        .ct{color:#38bdf8;font-size:11px;font-weight:600;margin-bottom:6px;border-bottom:1px solid #334155;padding-bottom:4px}
        .r{display:flex;gap:6px;flex-wrap:wrap;justify-content:center}
        .b{padding:10px 14px;border:none;border-radius:6px;font-size:12px;font-weight:600;cursor:pointer;min-width:48px;transition:all 0.1s}
        .bp{background:#0ea5e9;color:#fff}
        .bg{background:#22c55e;color:#fff}
        .br{background:#ef4444;color:#fff}
        .by{background:#f59e0b;color:#fff}
        .bon{background:#22c55e;color:#fff;box-shadow:0 0 8px #22c55e}
        .b:active{transform:scale(.95)}
        .i{display:inline-flex;align-items:center;justify-content:center;width:36px;height:36px;border-radius:50%;margin:3px;font-weight:700;font-size:12px}
        .ion{background:#22c55e;color:#fff;box-shadow:0 0 10px #22c55e}
        .ioff{background:#334155;color:#64748b}
        .st{font-size:10px;color:#94a3b8;text-align:center;margin-top:4px}
        .sterr{font-size:10px;color:#ef4444;text-align:center;margin-top:4px;font-weight:bold}
        .bk{margin-top:8px;text-align:center}
        .bk a{color:#38bdf8;font-size:11px}
        .sel{padding:6px;border-radius:4px;background:#0f172a;color:#e2e8f0;border:1px solid #334155;font-size:11px}
    </style>
</head>
<body>
    <div class="h">DIAGNOSTIC MODE (7-SEG)</div>
    <div class="g">
        <div class="c">
            <div class="ct">BUTTONS GPIO 25,26,27,14</div>
            <div class="r" id="btns"></div>
            <div class="st">Press physical button</div>
        </div>
        <div class="c">
            <div class="ct">SENSOR GPIO15</div>
            <div class="r"><span class="i ioff" id="sens">--</span></div>
            <div class="st" id="sensv">Waiting...</div>
        </div>
        <div class="c">
            <div class="ct">SENSOR LED GPIO2</div>
            <div class="r">
                <button class="b bg" onclick="tst('sled',1)">ON</button>
                <button class="b br" onclick="tst('sled',0)">OFF</button>
                <button class="b by" onclick="tst('sled',2)">BLINK</button>
            </div>
        </div>
        <div class="c">
            <div class="ct">LED FRAME GPIO12</div>
            <div class="r">
                <button class="b bg" onclick="tst('ledf',1)">ON</button>
                <button class="b br" onclick="tst('ledf',0)">OFF</button>
            </div>
        </div>
        <div class="c">
            <div class="ct">RELAYS GPIO 16,17,18,23</div>
            <div class="r" id="relays"></div>
            <div class="r" style="margin-top:6px">
                <select class="sel" id="reldly">
                    <option value="300">300ms</option>
                    <option value="500" selected>500ms</option>
                    <option value="1000">1s</option>
                </select>
                <button class="b by" onclick="relayAll()">ALL</button>
                <button class="b br" onclick="relayOff()">OFF</button>
            </div>
        </div>
        <div class="c">
            <div class="ct">BUZZER GPIO5</div>
            <div class="r">
                <button class="b bp" onclick="tst('buz',1)">BEEP</button>
                <button class="b bg" onclick="tst('buz',2)">COIN</button>
                <button class="b by" onclick="tst('buz',3)">START</button>
                <button class="b br" onclick="tst('buz',4)">ERROR</button>
            </div>
        </div>
        <div class="c">
            <div class="ct">COIN ACCEPTOR GPIO19</div>
            <div class="r"><span style="font-size:22px;font-weight:700;color:#22c55e" id="coin">0</span></div>
            <div class="st" id="coinst">Insert coin to test</div>
            <div class="r" style="margin-top:4px"><button class="b br" onclick="tst('coinrst',0)">RESET</button></div>
        </div>
        <div class="c">
            <div class="ct">INDIVIDUAL DISPLAYS</div>
            <div class="r">
                <button class="b bp" onclick="tst('segd',0)">CRED</button>
                <button class="b bp" onclick="tst('segd',1)">SVC1</button>
                <button class="b bp" onclick="tst('segd',2)">SVC2</button>
                <button class="b bp" onclick="tst('segd',3)">SVC3</button>
                <button class="b bp" onclick="tst('segd',4)">SVC4</button>
                <button class="b br" onclick="tst('segd',-1)">CLR</button>
            </div>
            <div class="st">Test display / Clear all</div>
        </div>
    </div>
    <div class="bk"><a href="/" onclick="exitDiag()">Back to Dashboard</a></div>
    <script>
        var NS=4,rs=[];
        function $(i){return document.getElementById(i)}
        function init(){
            var b='',r='';
            for(var i=0;i<NS;i++){
                b+='<span class="i ioff" id="b'+i+'">'+(i+1)+'</span>';
                r+='<button class="b bp" id="rb'+i+'" onclick="relay('+i+')">R'+(i+1)+'</button>';
                rs[i]=0
            }
            $('btns').innerHTML=b;
            $('relays').innerHTML=r
        }
        function ld(){
            fetch('/api/diag/status').then(r=>r.json()).then(d=>{
                for(var i=0;i<NS;i++){
                    $('b'+i).className=d.buttons[i]?'i ion':'i ioff';
                    if(d.relays){rs[i]=d.relays[i];$('rb'+i).className=d.relays[i]?'b bon':'b bp'}
                }
                $('sens').className=d.sensor?'i ion':'i ioff';
                $('sens').textContent=d.sensor?'HI':'LO';
                $('sensv').textContent='GPIO15: '+(d.sensor?'HIGH':'LOW');
                $('coin').textContent=d.coinPulses;
                if(d.coinShorted){
                    $('coin').style.color='#ef4444';
                    $('coinst').className='sterr';
                    $('coinst').textContent='SHORT DETECTED! Check wiring'
                }else{
                    $('coin').style.color='#22c55e';
                    $('coinst').className='st';
                    $('coinst').textContent='Insert coin to test'
                }
            }).catch(e=>0)
        }
        function tst(t,v){
            fetch('/api/diag/test',{
                method:'POST',
                headers:{'Content-Type':'application/json'},
                body:JSON.stringify({type:t,value:v})
            })
        }
        function relay(i){
            rs[i]=!rs[i];
            $('rb'+i).className=rs[i]?'b bon':'b bp';
            tst('relayon',i)
        }
        function relayAll(){tst('relayall',+$('reldly').value)}
        function relayOff(){
            for(var i=0;i<NS;i++){rs[i]=0;$('rb'+i).className='b bp'}
            tst('relay',-1)
        }
        function exitDiag(){tst('exit',0)}
        init();ld();setInterval(ld,300)
    </script>
</body>
</html>
    )rawliteral";
}

String getConfigHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Settings - Vending Machine</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;padding:16px;font-size:14px}
        h2{color:#f59e0b;margin-bottom:16px;text-align:center}
        .section{background:#1e293b;border-radius:12px;padding:16px;margin-bottom:16px}
        .section h3{color:#38bdf8;font-size:14px;margin-bottom:12px;border-bottom:1px solid #334155;padding-bottom:8px}
        .form-group{margin-bottom:12px}
        label{display:block;color:#94a3b8;font-size:12px;margin-bottom:4px}
        input,select{padding:10px;border-radius:6px;border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:14px;width:100%}
        button{padding:10px 20px;border:none;border-radius:6px;font-weight:600;cursor:pointer;font-size:14px}
        .btn-save{background:#22c55e;color:#fff;width:100%;margin-top:8px}
        .btn-back{background:#334155;color:#fff;margin-top:8px}
        button:active{transform:scale(0.95)}
    </style>
</head>
<body>
    <h2>Settings</h2>
    
    <div class="section">
        <h3>WiFi Configuration</h3>
        <div class="form-group">
            <label>SSID</label>
            <input type="text" id="wifiSSID" placeholder="Network name">
        </div>
        <div class="form-group">
            <label>Password</label>
            <input type="password" id="wifiPass" placeholder="Network password">
        </div>
        <button class="btn-save" onclick="saveWiFi()">Save WiFi</button>
    </div>
    
    <div class="section" id="serviceConfig">
        <h3>Service Configuration</h3>
    </div>
    
    <button class="btn-back" onclick="window.location.href='/'">Back to Dashboard</button>

    <script>
        function loadConfig() {
            fetch('/api/config/get')
                .then(r => r.json())
                .then(d => {
                    document.getElementById('wifiSSID').value = d.wifi_ssid || '';
                    
                    let html = '';
                    d.services.forEach((svc, i) => {
                        html += `
                        <h3>${svc.name} (${svc.code})</h3>
                        <div class="form-group">
                            <label>Duration (ms)</label>
                            <input type="number" id="dur${i}" value="${svc.duration}" min="5000" max="120000" step="1000">
                        </div>
                        <div class="form-group">
                            <label>Cost (credits)</label>
                            <input type="number" id="cost${i}" value="${svc.cost}" min="1" max="10">
                        </div>
                        <div class="form-group">
                            <label>Enabled</label>
                            <select id="en${i}">
                                <option value="1" ${svc.enabled?'selected':''}>Yes</option>
                                <option value="0" ${!svc.enabled?'selected':''}>No</option>
                            </select>
                        </div>
                        <button class="btn-save" onclick="saveService(${i})">Save ${svc.name}</button>`;
                    });
                    document.getElementById('serviceConfig').innerHTML += html;
                });
        }
        
        function saveWiFi() {
            fetch('/api/config/wifi', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    ssid: document.getElementById('wifiSSID').value,
                    password: document.getElementById('wifiPass').value
                })
            }).then(r => r.json()).then(d => {
                alert(d.success ? 'WiFi saved!' : 'Error saving WiFi');
            });
        }
        
        function saveService(id) {
            fetch('/api/service/config', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    service: id,
                    duration: parseInt(document.getElementById('dur'+id).value),
                    cost: parseInt(document.getElementById('cost'+id).value),
                    enabled: document.getElementById('en'+id).value == '1'
                })
            }).then(r => r.json()).then(d => {
                alert(d.success ? 'Service saved!' : 'Error saving service');
            });
        }
        
        loadConfig();
    </script>
</body>
</html>
    )rawliteral";
}

String getLoginHTML() {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Login - Vending Machine</title>
    <style>
        *{box-sizing:border-box;margin:0;padding:0}
        body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;display:flex;align-items:center;justify-content:center;min-height:100vh;padding:20px}
        .login-box{background:#1e293b;border-radius:16px;padding:32px;width:100%;max-width:360px;text-align:center}
        h1{color:#f59e0b;font-size:20px;margin-bottom:24px}
        input{padding:12px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:14px;width:100%;margin-bottom:12px}
        button{padding:12px;border:none;border-radius:8px;background:#0ea5e9;color:#fff;font-weight:600;font-size:14px;width:100%;cursor:pointer}
        button:active{transform:scale(0.95)}
        .error{color:#ef4444;font-size:12px;margin-top:8px;display:none}
    </style>
</head>
<body>
    <div class="login-box">
        <h1>N&R Solartech Sol</h1>
        <input type="text" id="username" placeholder="Username">
        <input type="password" id="password" placeholder="Password">
        <button onclick="doLogin()">Login</button>
        <div class="error" id="error">Invalid credentials</div>
    </div>
    <script>
        function doLogin() {
            fetch('/api/login', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({
                    username: document.getElementById('username').value,
                    password: document.getElementById('password').value
                })
            })
            .then(r => r.json())
            .then(d => {
                if (d.success) {
                    window.location.href = '/';
                } else {
                    document.getElementById('error').style.display = 'block';
                }
            });
        }
    </script>
</body>
</html>
    )rawliteral";
}
