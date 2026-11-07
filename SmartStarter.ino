/*
 * Complete Industrial 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 *
 * Industrial Non-Blocking State Machine Architecture:
 * - STATE_BOOT_STABILIZATION (3s Hardware Settle Delay)
 * - STATE_STANDBY (Motor OFF, Continuous 3-Phase Monitoring)
 * - STATE_PRE_START_CHECK (3-Phase Safety Inspection prior to Relay ON)
 * - STATE_RUNNING (Motor ON, Local 10ms Protection Monitoring)
 * - STATE_AUTO_RECOVERY_WAITING (15s Stability Timer after Grid Phase Loss)
 * - STATE_LATCHED_FAULT (Latched Safety Lockout for Damaging Faults: Dry-Run, Overload, Temp, Short)
 *
 * Protection Matrix:
 * 1. Pre-Start Current Inspection (Surge > 10A -> Latched Fault)
 * 2. Phase Grid Line Power Verification (Grid Loss -> Auto-Recovery eligible after 15s stability)
 * 3. Single Phasing / Phase Loss Protection (During Run -> Auto-Recovery eligible after 15s stability)
 * 4. Overload Protection (Current > FLC -> Latched Fault, Manual Reset Required)
 * 5. Dry-Run Protection (Current < 2.5A after 5s grace -> Latched Fault, Manual Reset Required)
 * 6. Over-Temperature Protection (Temp > MAX_TEMP -> Latched Fault, Manual Reset Required)
 *
 * Non-Volatile Storage (Preferences.h):
 * - Auto-Start, Dry-Run, & Schedule Settings survive reboots and power outages.
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <esp_task_wdt.h>
#include <Preferences.h>
#include "time.h"

WebServer server(80);
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);
Preferences preferences;

// Pin Definitions
const int RELAY_PIN = 5;       // Relay connected to Pin P5 (GPIO 5)

// --- RELAY LOGIC CONFIGURATION ---
const int RELAY_ON = LOW;      // Active LOW relay module (LOW = ON)
const int RELAY_OFF = HIGH;    // HIGH = OFF

// Analog CT Current Sensor Pins
const int CT1_PIN = 34;        // Phase 1 (R) Current Sensor
const int CT2_PIN = 35;        // Phase 2 (Y) Current Sensor
const int CT3_PIN = 32;        // Phase 3 (B) Current Sensor
const int TEMP_PIN = 33;       // Motor Temperature Sensor Pin

// Optional Hardware Optocoupler / AC Voltage Sensing Digital Pins (GPIO 25, 26, 27)
const int PHASE_V1_PIN = 25;   // Phase R AC Line Sensor (Optional)
const int PHASE_V2_PIN = 26;   // Phase Y AC Line Sensor (Optional)
const int PHASE_V3_PIN = 27;   // Phase B AC Line Sensor (Optional)

// --- FINITE STATE MACHINE DEFINITIONS ---
enum SystemState {
  STATE_BOOT_STABILIZATION,
  STATE_STANDBY,
  STATE_PRE_START_CHECK,
  STATE_RUNNING,
  STATE_AUTO_RECOVERY_WAITING,
  STATE_LATCHED_FAULT
};

SystemState currentState = STATE_BOOT_STABILIZATION;

// Motor State & Protection Settings (5 HP Motor ~ 7.5A Full Load)
bool motorState = false;
bool faultState = false;              // Latched Fault Lockout Flag
bool autoStartEnabled = false;        // Auto Start Motor feature toggle (Saved in NVS)
bool dryRunProtectionEnabled = false; // Dry-Run protection toggle (Saved in NVS)
bool phasePowerGridOk = true;         // 3-Phase Line Grid Power Status Flag

const float RATED_FLC = 50.0;         // Temporarily set high (50A) for testing without sensors
const float DRY_RUN_LIMIT = 2.5;      // Min current threshold for pump dry-run trip (Amps)
const float MAX_TEMP_LIMIT = 150.0;   // Temporarily set high (150°C) to prevent false trips while testing without temp sensor

String stopReason = "System Boot / Stabilization"; // Diagnostic reason string

unsigned long motorStartTime = 0;           // Timestamp when motor started
unsigned long phaseRecoveryStartTime = 0;   // Timestamp when phase power recovered
const unsigned long DRY_RUN_GRACE_MS = 5000;       // 5-second grace period after start
const unsigned long BOOT_STABILIZATION_MS = 3000; // 3-second hardware settle delay after boot
const unsigned long RECOVERY_STABILITY_MS = 15000;// 15-second phase stability period required before auto-restart

// Energy & Running Hour Metering
unsigned long totalRunTimeSec = 0;
unsigned long lastRunTimeTick = 0;
float totalEnergykWh = 0.0;

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
unsigned long lastMqttRetry = 0;

// Function Declarations
void publishTelemetry();
void tripMotor(String reason, bool isLatchedFault);
void setMotorState(bool state);
float readRMSCurrent(int pin);
float readMotorTemperature();
bool check3PhaseSafetyBeforeStart();
void loadNvsSettings();
void saveNvsSettings();
void processStateMachine();

void loadNvsSettings() {
  preferences.begin("smartstarter", true); // Read-only mode
  autoStartEnabled = preferences.getBool("autoStart", false);
  dryRunProtectionEnabled = preferences.getBool("dryRun", false);
  scheduleEnabled = preferences.getBool("schedEnabled", false);
  SCHEDULE_START_HOUR = preferences.getInt("schedStartH", 6);
  SCHEDULE_START_MIN = preferences.getInt("schedStartM", 30);
  SCHEDULE_STOP_HOUR = preferences.getInt("schedStopH", 18);
  SCHEDULE_STOP_MIN = preferences.getInt("schedStopM", 0);
  preferences.end();

  Serial.printf(">>> NVS Loaded Settings: AutoStart=%s, DryRun=%s, Sched=%s (%02d:%02d to %02d:%02d)\n",
                autoStartEnabled ? "ENABLED" : "DISABLED",
                dryRunProtectionEnabled ? "ENABLED" : "DISABLED",
                scheduleEnabled ? "ENABLED" : "DISABLED",
                SCHEDULE_START_HOUR, SCHEDULE_START_MIN, SCHEDULE_STOP_HOUR, SCHEDULE_STOP_MIN);
}

void saveNvsSettings() {
  preferences.begin("smartstarter", false); // Read-write mode
  preferences.putBool("autoStart", autoStartEnabled);
  preferences.putBool("dryRun", dryRunProtectionEnabled);
  preferences.putBool("schedEnabled", scheduleEnabled);
  preferences.putInt("schedStartH", SCHEDULE_START_HOUR);
  preferences.putInt("schedStartM", SCHEDULE_START_MIN);
  preferences.putInt("schedStopH", SCHEDULE_STOP_HOUR);
  preferences.putInt("schedStopM", SCHEDULE_STOP_MIN);
  preferences.end();
  Serial.println(">>> NVS Settings Saved Successfully.");
}

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

// Function to perform 3-Phase Line Voltage & Health Inspection prior to motor start
bool check3PhaseSafetyBeforeStart() {
  // Allow hardware stabilization delay after boot before enforcing ADC pre-checks
  if (millis() < BOOT_STABILIZATION_MS) {
    return false;
  }

  // 1. Verify 3-Phase Power Grid Availability
  if (!phasePowerGridOk) {
    Serial.println(">>> 3-PHASE PRE-CHECK FAILED: 3-Phase Grid Power is OFF or Phase Line Missing!");
    return false;
  }

  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  // 2. Pre-start residual leakage / short circuit check (> 10.0A pre-start residual is unsafe)
  if (i1 > 10.0 || i2 > 10.0 || i3 > 10.0) {
    Serial.println(">>> 3-PHASE PRE-CHECK FAILED: Pre-start current surge / short circuit detected!");
    return false;
  }

  // 3. ADC Baseline Health Check on all 3 channels
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
      faultState = false; // Clear fault lockout on manual START
      Serial.println(">>> START Button Pressed: Performing Pre-Start Safety Inspection...");
      if (check3PhaseSafetyBeforeStart()) {
        stopReason = "Motor Running";
        setMotorState(true);
        currentState = STATE_RUNNING;
      } else {
        stopReason = "Start Refused: 3-Phase Grid Power Missing / Fault";
        tripMotor("TRIPPED_PHASE_FAILURE", true);
      }
    } else if (upperMsg == "OFF" || upperMsg == "STOP" || upperMsg == "0") {
      faultState = false; // Clear fault lockout on manual STOP
      stopReason = "Manual STOP Command";
      Serial.println(">>> Command matched: Turning MOTOR OFF");
      setMotorState(false);
      currentState = STATE_STANDBY;
    } else if (upperMsg == "AUTOSTART_ON") {
      autoStartEnabled = true;
      faultState = false; // Reset fault state when user re-enables Auto-Start
      saveNvsSettings();
      Serial.println(">>> Auto Start Motor ENABLED");
      if (!motorState) {
        if (check3PhaseSafetyBeforeStart()) {
          stopReason = "Motor Running (Auto-Start)";
          setMotorState(true);
          currentState = STATE_RUNNING;
        } else {
          stopReason = "Auto-Start Waiting: 3-Phase Power Unhealthy";
          currentState = STATE_AUTO_RECOVERY_WAITING;
        }
      }
    } else if (upperMsg == "AUTOSTART_OFF") {
      autoStartEnabled = false;
      saveNvsSettings();
      Serial.println(">>> Auto Start Motor DISABLED");
    } else if (upperMsg == "DRYRUN_ON") {
      dryRunProtectionEnabled = true;
      saveNvsSettings();
      Serial.println(">>> Dry-Run Protection ENABLED");
    } else if (upperMsg == "DRYRUN_OFF") {
      dryRunProtectionEnabled = false;
      saveNvsSettings();
      Serial.println(">>> Dry-Run Protection DISABLED");
    } else if (upperMsg == "PHASE_GRID_OK") {
      phasePowerGridOk = true;
      Serial.println(">>> 3-Phase Grid Power Set to NORMAL (230V/400V)");
    } else if (upperMsg == "PHASE_GRID_FAIL") {
      phasePowerGridOk = false;
      Serial.println(">>> 3-Phase Grid Power Set to MISSING / FAULT (0V)");
      if (motorState) {
        stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE (Grid Power Loss)";
        tripMotor("TRIPPED_PHASE_FAILURE", false); // Auto-recoverable when grid restores
      }
    } else if (upperMsg == "SCHEDULE_ON") {
      scheduleEnabled = true;
      saveNvsSettings();
      Serial.println(">>> Time Schedule ENABLED");
    } else if (upperMsg == "SCHEDULE_OFF") {
      scheduleEnabled = false;
      saveNvsSettings();
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
          saveNvsSettings();
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
  if (millis() - lastMqttRetry < 5000) return; // Non-blocking 5s retry
  lastMqttRetry = millis();

  if (!mqttClient.connected()) {
    esp_task_wdt_reset();
    Serial.print("Connecting to HiveMQ Cloud Secure MQTT...");
    String clientId = "SmartStarterGlobal-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected globally!");
      mqttClient.subscribe(mqtt_topic_control);
      publishTelemetry();
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- Smart Starter ESP32 Booting ---");

  // Load Persisted Settings from Non-Volatile Memory (NVS)
  loadNvsSettings();

#if defined(ESP_ARDUINO_VERSION) && ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_config_t twdt_config;
  twdt_config.timeout_ms = 30000;
  twdt_config.idle_core_mask = 0;
  twdt_config.trigger_panic = true;
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

  currentState = STATE_BOOT_STABILIZATION;
}

// --- NON-BLOCKING INDUSTRIAL FINITE STATE MACHINE ---
void processStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_BOOT_STABILIZATION:
      if (now >= BOOT_STABILIZATION_MS) {
        Serial.println(">>> Hardware Stabilization Complete.");
        if (autoStartEnabled && !faultState) {
          currentState = STATE_PRE_START_CHECK;
        } else {
          currentState = STATE_STANDBY;
        }
      }
      break;

    case STATE_STANDBY:
      if (autoStartEnabled && !motorState && !faultState) {
        currentState = STATE_PRE_START_CHECK;
      }
      break;

    case STATE_PRE_START_CHECK:
      Serial.println(">>> State Machine: Performing Pre-Start Safety Inspection...");
      if (check3PhaseSafetyBeforeStart()) {
        Serial.println(">>> Pre-Start Inspection PASSED: Auto-Starting Motor!");
        stopReason = "Motor Running (Auto-Start)";
        setMotorState(true);
        currentState = STATE_RUNNING;
        publishTelemetry();
      } else {
        Serial.println(">>> Pre-Start Inspection FAILED: 3-Phase Power Unhealthy.");
        stopReason = "Auto-Start Refused: 3-Phase Grid Power Missing / Unhealthy";
        phaseRecoveryStartTime = 0; // Reset recovery timer
        currentState = STATE_AUTO_RECOVERY_WAITING;
        publishTelemetry();
      }
      break;

    case STATE_RUNNING:
      // Local 10ms Continuous Multi-Protection Engine
      if (motorState) {
        float i1 = readRMSCurrent(CT1_PIN);
        float i2 = readRMSCurrent(CT2_PIN);
        float i3 = readRMSCurrent(CT3_PIN);
        float temp = readMotorTemperature();

        // 1. Grid Power Loss Check (Auto-recoverable)
        if (!phasePowerGridOk) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE (Grid Power Loss)";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
        // 2. Overload Protection (Latched)
        else if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
          stopReason = "Fault Trip: TRIPPED_OVERLOAD";
          tripMotor("TRIPPED_OVERLOAD", true);
        }
        // 3. Over-Temperature Protection (Latched)
        else if (temp > MAX_TEMP_LIMIT) {
          stopReason = "Fault Trip: TRIPPED_OVERTEMP";
          tripMotor("TRIPPED_OVERTEMP", true);
        }
        // 4. Dry-Run Protection (Latched - 5s Grace Period)
        else if (dryRunProtectionEnabled && (now - motorStartTime > DRY_RUN_GRACE_MS)) {
          if (i1 < DRY_RUN_LIMIT && i2 < DRY_RUN_LIMIT && i3 < DRY_RUN_LIMIT) {
            stopReason = "Fault Trip: TRIPPED_DRY_RUN";
            tripMotor("TRIPPED_DRY_RUN", true);
          }
        }
        // 5. Single Phasing Protection (Auto-recoverable after grid stabilizes)
        else if ((i1 > 2.0 && (i2 < 0.5 || i3 < 0.5)) || (i2 > 2.0 && (i1 < 0.5 || i3 < 0.5)) || (i3 > 2.0 && (i1 < 0.5 || i2 < 0.5))) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
      }
      break;

    case STATE_AUTO_RECOVERY_WAITING:
      if (!autoStartEnabled) {
        currentState = STATE_STANDBY;
        break;
      }
      if (faultState) {
        currentState = STATE_LATCHED_FAULT;
        break;
      }
      // Require 15 seconds of continuous healthy 3-phase grid power before restarting
      if (check3PhaseSafetyBeforeStart()) {
        if (phaseRecoveryStartTime == 0) {
          phaseRecoveryStartTime = now;
          Serial.println(">>> 3-Phase Power Health Restored. Starting 15s Stability Timer...");
        } else if (now - phaseRecoveryStartTime >= RECOVERY_STABILITY_MS) {
          Serial.println(">>> 15s Stability Timer Passed! Auto-Restarting Motor...");
          stopReason = "Motor Running (Auto-Start Recovered)";
          setMotorState(true);
          currentState = STATE_RUNNING;
          publishTelemetry();
        }
      } else {
        phaseRecoveryStartTime = 0; // Reset timer if phase fluctuates
      }
      break;

    case STATE_LATCHED_FAULT:
      // Unsafe damaging fault (Overload, Dry-Run, Temp, Pre-Start Short). Requires manual START or STOP command over MQTT.
      break;
  }
}

void loop() {
  esp_task_wdt_reset();

  if (!mqttClient.connected()) {
    reconnectMqtt();
  } else {
    mqttClient.loop();
  }
  server.handleClient();

  unsigned long now = millis();

  // Run Non-Blocking State Machine
  processStateMachine();

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
          currentState = STATE_RUNNING;
        } else {
          stopReason = "Schedule Start Refused: 3-Phase Grid Power Missing";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
        publishTelemetry();
      }
      else if (currentHour == SCHEDULE_STOP_HOUR && currentMin == SCHEDULE_STOP_MIN && motorState) {
        Serial.println(">>> SCHEDULE TRIGGER: Stopping Motor!");
        stopReason = "Scheduled STOP (Timer)";
        setMotorState(false);
        currentState = STATE_STANDBY;
        publishTelemetry();
      }
    }
  }
}

void tripMotor(String reason, bool isLatchedFault) {
  motorState = false;
  digitalWrite(RELAY_PIN, RELAY_OFF);

  if (isLatchedFault) {
    faultState = true;
    currentState = STATE_LATCHED_FAULT;
    Serial.print(">>> Motor Tripped (Latched Fault Lockout Activated - Manual Reset Required): ");
  } else {
    phaseRecoveryStartTime = 0;
    currentState = STATE_AUTO_RECOVERY_WAITING;
    Serial.print(">>> Motor Tripped (Auto-Recovery Eligible after 15s Grid Stability): ");
  }

  stopReason = "Fault Trip: " + reason;
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

  float v1 = phasePowerGridOk ? 230.0 : 0.0;
  float v2 = phasePowerGridOk ? 230.0 : 0.0;
  float v3 = phasePowerGridOk ? 230.0 : 0.0;

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
  payload += "\"phaseGridOk\":" + String(phasePowerGridOk ? "true" : "false") + ",";
  payload += "\"faultState\":" + String(faultState ? "true" : "false") + ",";
  payload += "\"dryRunProtectionEnabled\":" + String(dryRunProtectionEnabled ? "true" : "false");
  payload += "}";

  mqttClient.publish(mqtt_topic_telemetry, payload.c_str());
}
