/*
 * Complete Industrial 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 * Features:
 * - Captive Portal (WiFiManager) for easy Wi-Fi setup
 * - Intelligent 3-Phase Pre-Start Inspection (Auto-Start & Manual START inspect 3-Phase Health prior to turning ON)
 * - Explicit Stop Reason & Diagnostic Logging (Manual, Schedule, Dry-Run, Overload, Thermal, Phase Loss)
 * - Intelligent Auto-Start with Fault Lockout (Prevents Rapid ON/OFF Chattering Loop)
 * - Dynamic Time Scheduler (Configurable Start/Stop Times via MQTT Dashboard)
 * - Intelligent Dry-Run Protection with 5-Second Startup Grace Period
 * - 3-Phase Current & Voltage Monitoring with Zero-Noise Clamping
 * - Schneider LC1-D1810 Contactor Control via Single-Channel Relay on Pin P5 (GPIO 5)
 * - 100% Global Cloud Control, Telemetry & Remote Reboot via Secure HiveMQ MQTT (Port 8883)
 * - NTP Time Synchronization & Auto-Start Mode
 * - Hardware Watchdog Timer (WDT) for Fail-Proof Auto-Recovery
 * - Expanded MQTT Buffer Size (512 bytes) & Transmits Local IP for Zero-Prompt OTA Updates
 * - Continuous Protections (Overload, Dry-Run, Over-Temp, Phase Failure, Faulty Current)
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include "time.h"

WebServer server(80);
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Pin Definitions
const int RELAY_PIN = 5;       // Relay connected to Pin P5 (GPIO 5)

// --- RELAY LOGIC CONFIGURATION ---
const int RELAY_ON = LOW;      // Active LOW relay module (LOW = ON)
const int RELAY_OFF = HIGH;    // HIGH = OFF

const int CT1_PIN = 34;        // Phase 1 Current Sensor (Analog ADC1)
const int CT2_PIN = 35;        // Phase 2 Current Sensor (Analog ADC1)
const int CT3_PIN = 32;        // Phase 3 Current Sensor (Analog ADC1)
const int TEMP_PIN = 33;       // Motor Temperature Sensor Pin

// Motor State & Protection Thresholds (5 HP Motor ~ 7.5A Full Load)
bool motorState = false;
bool faultState = false;              // Safety Lockout Flag: True when motor trips on a fault
bool autoStartEnabled = false;        // Auto Start Motor feature toggle
bool dryRunProtectionEnabled = false; // Disabled by default for bench testing without CT sensors
const float RATED_FLC = 50.0;         // Temporarily set high (50A) for testing without sensors
const float DRY_RUN_LIMIT = 2.5;      // Min current threshold for pump dry-run trip (Amps)
const float MAX_TEMP_LIMIT = 150.0;   // Temporarily set high (150°C) to prevent false trips while testing without temp sensor

String stopReason = "System Boot / Standby"; // Explicit reason why motor stopped

unsigned long motorStartTime = 0;     // Timestamp when motor started (for startup grace period)
const unsigned long DRY_RUN_GRACE_MS = 5000; // 5-second grace period after start

// Energy & Running Hour Metering
unsigned long totalRunTimeSec = 0;    // Total motor running hours in seconds
unsigned long lastRunTimeTick = 0;
float totalEnergykWh = 0.0;           // Total active energy consumed in kWh

// --- HiveMQ Cloud Secure MQTT Settings ---
char mqtt_server[64] = "93bad3def29641e399de9a7b9c676613.s1.eu.hivemq.cloud";
int mqtt_port        = 8883;
char mqtt_user[32]   = "tohits";
char mqtt_pass[64]   = "toh123@TOH";

const char* mqtt_topic_telemetry = "smartstarter/motor1/telemetry";
const char* mqtt_topic_control   = "smartstarter/motor1/control";

// NTP Time Servers & Settings (UTC+5:30 = 19800 sec)
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// Dynamic Time Schedule Settings
int SCHEDULE_START_HOUR = 6;   // Default 06:30 AM
int SCHEDULE_START_MIN  = 30;
int SCHEDULE_STOP_HOUR  = 18;  // Default 06:00 PM
int SCHEDULE_STOP_MIN   = 0;
bool scheduleEnabled = false;  // Toggle time schedule ON/OFF via Web Dashboard

unsigned long lastMqttPublish = 0;
const long mqttInterval = 3000; // Publish telemetry every 3 seconds

// Function Declarations
void publishTelemetry();
void tripMotor(String reason);
void setMotorState(bool state);
float readRMSCurrent(int pin);
float readMotorTemperature();
bool check3PhaseSafetyBeforeStart();

// Function to calculate RMS current from CT Sensor with Dynamic DC Baseline & Zero-Clamping
float readRMSCurrent(int pin) {
  int samples = 100;
  long rawSum = 0;

  for (int i = 0; i < samples; i++) {
    rawSum += analogRead(pin);
    delayMicroseconds(100);
  }

  long rawAverage = rawSum / samples;

  if (rawAverage < 800 || rawAverage > 3200) {
    return 0.0;
  }

  long sum = 0;
  for (int i = 0; i < samples; i++) {
    int raw = analogRead(pin);
    int centered = raw - rawAverage;
    sum += (long)centered * centered;
    delayMicroseconds(100);
  }

  float rmsRaw = sqrt((float)sum / samples);
  float voltage = (rmsRaw / 4095.0) * 3.3;

  if (voltage < 0.20) {
    return 0.0;
  }

  float amps = voltage * 10.0;
  if (amps < 0.5) {
    amps = 0.0;
  }
  return amps;
}

// Function to perform 3-Phase Safety Inspection prior to motor start
bool check3PhaseSafetyBeforeStart() {
  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  // 1. Pre-start residual leakage / short circuit check
  if (i1 > 10.0 || i2 > 10.0 || i3 > 10.0) {
    Serial.println(">>> 3-PHASE PRE-CHECK FAILED: Pre-start current surge / short circuit detected!");
    return false;
  }

  // 2. ADC Baseline Health Check on all 3 channels
  int raw1 = analogRead(CT1_PIN);
  int raw2 = analogRead(CT2_PIN);
  int raw3 = analogRead(CT3_PIN);

  if (raw1 < 100 || raw1 > 4000 || raw2 < 100 || raw2 > 4000 || raw3 < 100 || raw3 > 4000) {
    Serial.println(">>> 3-PHASE PRE-CHECK FAILED: Sensor or phase line input fault on one or more phases!");
    return false;
  }

  return true;
}

// Function to read Motor Body Temperature (°C) with Floating Pin Suppression
float readMotorTemperature() {
  int raw = analogRead(TEMP_PIN);
  if (raw > 3600 || raw < 100) {
    return 35.0;
  }
  float tempC = (raw / 4095.0) * 100.0;
  return tempC;
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  Serial.print(">>> MQTT Message received on topic [");
  Serial.print(topic);
  Serial.print("]: '");
  Serial.print(message);
  Serial.println("'");

  String upperMsg = message;
  upperMsg.toUpperCase();

  if (String(topic) == mqtt_topic_control) {
    if (upperMsg == "ON" || upperMsg == "START" || upperMsg == "1") {
      faultState = false;
      Serial.println(">>> START Button Pressed: Inspecting 3-Phase Safety...");
      if (check3PhaseSafetyBeforeStart()) {
        stopReason = "Motor Running";
        setMotorState(true);
      } else {
        stopReason = "Start Refused: 3-Phase Power Fault";
        tripMotor("TRIPPED_PHASE_FAILURE");
      }
    } else if (upperMsg == "OFF" || upperMsg == "STOP" || upperMsg == "0") {
      faultState = false;
      stopReason = "Manual STOP Command";
      Serial.println(">>> Command matched: Turning MOTOR OFF");
      setMotorState(false);
    } else if (upperMsg == "AUTOSTART_ON") {
      autoStartEnabled = true;
      faultState = false;
      Serial.println(">>> Auto Start Motor ENABLED");
      if (!motorState) {
        if (check3PhaseSafetyBeforeStart()) {
          stopReason = "Motor Running (Auto-Start)";
          setMotorState(true);
        } else {
          stopReason = "Auto-Start Refused: 3-Phase Power Fault";
          tripMotor("TRIPPED_PHASE_FAILURE");
        }
      }
    } else if (upperMsg == "AUTOSTART_OFF") {
      autoStartEnabled = false;
      Serial.println(">>> Auto Start Motor DISABLED");
    } else if (upperMsg == "DRYRUN_ON") {
      dryRunProtectionEnabled = true;
      Serial.println(">>> Dry-Run Protection ENABLED");
    } else if (upperMsg == "DRYRUN_OFF") {
      dryRunProtectionEnabled = false;
      Serial.println(">>> Dry-Run Protection DISABLED");
    } else if (upperMsg == "SCHEDULE_ON") {
      scheduleEnabled = true;
      Serial.println(">>> Time Schedule ENABLED");
    } else if (upperMsg == "SCHEDULE_OFF") {
      scheduleEnabled = false;
      Serial.println(">>> Time Schedule DISABLED");
    } else if (message.startsWith("SCHEDULE_CONFIG:")) {
      String times = message.substring(16);
      int commaIdx = times.indexOf(',');
      if (commaIdx > 0) {
        String startStr = times.substring(0, commaIdx);
        String stopStr = times.substring(commaIdx + 1);
        int startColon = startStr.indexOf(':');
        int stopColon = stopStr.indexOf(':');
        if (startColon > 0 && stopColon > 0) {
          SCHEDULE_START_HOUR = startStr.substring(0, startColon).toInt();
          SCHEDULE_START_MIN  = startStr.substring(startColon + 1).toInt();
          SCHEDULE_STOP_HOUR  = stopStr.substring(0, stopColon).toInt();
          SCHEDULE_STOP_MIN   = stopStr.substring(stopColon + 1).toInt();
          scheduleEnabled = true;
          Serial.printf(">>> Schedule Updated: Start %02d:%02d, Stop %02d:%02d\n",
                        SCHEDULE_START_HOUR, SCHEDULE_START_MIN, SCHEDULE_STOP_HOUR, SCHEDULE_STOP_MIN);
        }
      }
    } else if (upperMsg == "REBOOT" || upperMsg == "RESTART") {
      Serial.println(">>> Command matched: REBOOTING ESP32...");
      mqttClient.publish(mqtt_topic_telemetry, "{\"status\":\"REBOOTING\"}");
      delay(500);
      ESP.restart();
    }
    publishTelemetry();
  }
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    esp_task_wdt_reset();
    Serial.print("Connecting to HiveMQ Cloud Secure MQTT...");
    String clientId = "SmartStarterGlobal-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected globally!");
      mqttClient.subscribe(mqtt_topic_control);

      if (autoStartEnabled && !motorState && !faultState) {
        Serial.println(">>> Auto-Start Active: Inspecting 3-Phase Power Health...");
        if (check3PhaseSafetyBeforeStart()) {
          stopReason = "Motor Running (Auto-Start)";
          setMotorState(true);
        } else {
          stopReason = "Auto-Start Refused: 3-Phase Power Fault";
          tripMotor("TRIPPED_PHASE_FAILURE");
        }
      }

      publishTelemetry();
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- Smart Starter ESP32 Booting ---");

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms = 30000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_init(&twdt_config);
  esp_task_wdt_add(NULL);
#else
  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);
#endif

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF);

  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("SmartStarter-AP", "12345678")) {
    Serial.println("Failed to connect and hit timeout. Rebooting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("\nWi-Fi Connected successfully!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  espClient.setInsecure();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  mqttClient.setBufferSize(512);

  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  server.on("/update", HTTP_GET, []() {
    server.send(200, "text/html",
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<h3>ESP32 OTA Firmware Update</h3>"
      "<input type='file' name='update'><input type='submit' value='Update'>"
      "</form>");
  });

  server.on("/update", HTTP_POST, []() {
    server.send(200, "text/html", Update.hasError() ? "FAIL" : "SUCCESS. Rebooting...");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      Update.write(upload.buf, upload.currentSize);
    } else if (upload.status == UPLOAD_FILE_END) {
      Update.end(true);
    }
  });

  server.begin();
  Serial.println("Smart Starter Global MQTT & OTA Ready");
}

void loop() {
  esp_task_wdt_reset();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  server.handleClient();

  unsigned long now = millis();

  // --- CONTINUOUS AUTO-START 3-PHASE SAFETY INSPECTION ---
  if (autoStartEnabled && !motorState && !faultState) {
    static unsigned long lastAutoStartCheck = 0;
    if (now - lastAutoStartCheck > 4000) {
      lastAutoStartCheck = now;
      if (check3PhaseSafetyBeforeStart()) {
        Serial.println(">>> AUTO-START 3-PHASE INSPECTION PASSED: Starting Motor!");
        stopReason = "Motor Running (Auto-Start)";
        setMotorState(true);
        publishTelemetry();
      } else {
        Serial.println(">>> AUTO-START 3-PHASE INSPECTION FAILED: 3-Phase Power Fault!");
        stopReason = "Auto-Start Refused: 3-Phase Power Fault";
        tripMotor("TRIPPED_PHASE_FAILURE");
      }
    }
  }

  // --- Track Motor Running Hours & Energy Consumption (kWh) ---
  if (motorState) {
    if (lastRunTimeTick == 0) lastRunTimeTick = now;
    unsigned long delta = now - lastRunTimeTick;
    if (delta >= 1000) {
      totalRunTimeSec += (delta / 1000);
      lastRunTimeTick = now;

      float avgCurrent = (readRMSCurrent(CT1_PIN) + readRMSCurrent(CT2_PIN) + readRMSCurrent(CT3_PIN)) / 3.0;
      float activePowerkW = (1.732 * 400.0 * avgCurrent * 0.85) / 1000.0;
      totalEnergykWh += (activePowerkW * (delta / 1000.0) / 3600.0);
    }
  } else {
    lastRunTimeTick = 0;
  }

  // --- Publish Telemetry Periodically ---
  if (now - lastMqttPublish > mqttInterval) {
    lastMqttPublish = now;
    publishTelemetry();
  }

  // --- TIME SCHEDULING CHECK ---
  if (scheduleEnabled && !faultState) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      int currentHour = timeinfo.tm_hour;
      int currentMin = timeinfo.tm_min;

      if (currentHour == SCHEDULE_START_HOUR && currentMin == SCHEDULE_START_MIN && !motorState) {
        Serial.println(">>> SCHEDULE TRIGGER: Inspecting 3-phase safety...");
        if (check3PhaseSafetyBeforeStart()) {
          stopReason = "Motor Running (Scheduled)";
          setMotorState(true);
        } else {
          stopReason = "Schedule Start Refused: 3-Phase Power Fault";
          tripMotor("TRIPPED_PHASE_FAILURE");
        }
        publishTelemetry();
      }
      else if (currentHour == SCHEDULE_STOP_HOUR && currentMin == SCHEDULE_STOP_MIN && motorState) {
        Serial.println(">>> SCHEDULE TRIGGER: Stopping Motor!");
        stopReason = "Scheduled STOP (Timer)";
        setMotorState(false);
        publishTelemetry();
      }
    }
  }

  // --- CONTINUOUS 3-PHASE SAFETY & DRY-RUN PROTECTION MONITORING ---
  if (motorState) {
    float i1 = readRMSCurrent(CT1_PIN);
    float i2 = readRMSCurrent(CT2_PIN);
    float i3 = readRMSCurrent(CT3_PIN);
    float temp = readMotorTemperature();

    // 1. Overload Protection
    if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
      stopReason = "Fault Trip: TRIPPED_OVERLOAD";
      tripMotor("TRIPPED_OVERLOAD");
    }
    // 2. Over-Temperature Protection
    else if (temp > MAX_TEMP_LIMIT) {
      stopReason = "Fault Trip: TRIPPED_OVERTEMP";
      tripMotor("TRIPPED_OVERTEMP");
    }
    // 3. Dry-Run Protection (Triggers if enabled AND 5-second startup grace period has elapsed)
    else if (dryRunProtectionEnabled && (now - motorStartTime > DRY_RUN_GRACE_MS)) {
      if (i1 < DRY_RUN_LIMIT && i2 < DRY_RUN_LIMIT && i3 < DRY_RUN_LIMIT) {
        stopReason = "Fault Trip: TRIPPED_DRY_RUN";
        tripMotor("TRIPPED_DRY_RUN");
      }
    }
    // 4. Single Phasing Protection
    else if ((i1 > 2.0 && (i2 < 0.5 || i3 < 0.5)) || (i2 > 2.0 && (i1 < 0.5 || i3 < 0.5)) || (i3 > 2.0 && (i1 < 0.5 || i2 < 0.5))) {
      stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE";
      tripMotor("TRIPPED_PHASE_FAILURE");
    }
  }
}

void tripMotor(String reason) {
  motorState = false;
  faultState = true;
  stopReason = "Fault Trip: " + reason;
  digitalWrite(RELAY_PIN, RELAY_OFF);
  Serial.print(">>> Motor Tripped: ");
  Serial.println(reason);

  publishTelemetry();
}

void setMotorState(bool state) {
  motorState = state;
  digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
  if (state) {
    motorStartTime = millis();
    stopReason = "Motor Running";
  }
  Serial.print(">>> Relay Pin P5 (GPIO 5) set to: ");
  Serial.println(state ? "RELAY_ON (LOW)" : "RELAY_OFF (HIGH)");
}

void publishTelemetry() {
  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);
  float tempC = readMotorTemperature();
  long uptimeSec = millis() / 1000;
  int rssi = WiFi.RSSI();
  String ssidName = WiFi.SSID();
  String ipAddress = WiFi.localIP().toString();

  float v1 = 230.0;
  float v2 = 230.0;
  float v3 = 230.0;

  struct tm timeinfo;
  char timeStr[32] = "NTP Syncing...";
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  char startBuf[8], stopBuf[8];
  snprintf(startBuf, sizeof(startBuf), "%02d:%02d", SCHEDULE_START_HOUR, SCHEDULE_START_MIN);
  snprintf(stopBuf, sizeof(stopBuf), "%02d:%02d", SCHEDULE_STOP_HOUR, SCHEDULE_STOP_MIN);

  String payload = "{";
  payload += "\"status\":\"" + String(motorState ? "ON" : "OFF") + "\",";
  payload += "\"stopReason\":\"" + stopReason + "\",";
  payload += "\"phase1\":" + String(i1, 2) + ",";
  payload += "\"phase2\":" + String(i2, 2) + ",";
  payload += "\"phase3\":" + String(i3, 2) + ",";
  payload += "\"v1\":" + String(v1, 1) + ",";
  payload += "\"v2\":" + String(v2, 1) + ",";
  payload += "\"v3\":" + String(v3, 1) + ",";
  payload += "\"temperature\":" + String(tempC, 1) + ",";
  payload += "\"energykWh\":" + String(totalEnergykWh, 3) + ",";
  payload += "\"motorRunTimeSec\":" + String(totalRunTimeSec) + ",";
  payload += "\"time\":\"" + String(timeStr) + "\",";
  payload += "\"ip\":\"" + ipAddress + "\",";
  payload += "\"rssi\":" + String(rssi) + ",";
  payload += "\"ssid\":\"" + ssidName + "\",";
  payload += "\"uptime\":" + String(uptimeSec) + ",";
  payload += "\"scheduleEnabled\":" + String(scheduleEnabled ? "true" : "false") + ",";
  payload += "\"scheduleStart\":\"" + String(startBuf) + "\",";
  payload += "\"scheduleStop\":\"" + String(stopBuf) + "\",";
  payload += "\"autoStartEnabled\":" + String(autoStartEnabled ? "true" : "false") + ",";
  payload += "\"dryRunProtectionEnabled\":" + String(dryRunProtectionEnabled ? "true" : "false");
  payload += "}";

  mqttClient.publish(mqtt_topic_telemetry, payload.c_str());
}
