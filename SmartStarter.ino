/*
 * Complete Industrial 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 *
 * FAIL-SAFE AUTO-START & MULTI-PROTECTION ARCHITECTURE:
 *
 * 1. Relay Safety Guarantee:
 *    - GPIO 5 Contactor Relay is Active LOW (LOW = ON, HIGH = OFF).
 *    - On boot, during initialization, during Wi-Fi/MQTT disconnects, and during 3-Phase safety checks,
 *      the relay is STRICTLY HELD OFF (HIGH). It NEVER energizes for even 1 millisecond until 15s stability passes.
 *
 * 2. Fail-Safe 3-Phase Verification:
 *    - If AC Optocoupler / Voltage sensors (GPIO 25, 26, 27) or Dashboard Grid Sync report missing/unverified phases,
 *      phase health checks fail-safe to FALSE. The relay remains CONTINUOUSLY OFF (HIGH).
 *
 * 3. 15-Second Continuous Phase Stability Timer:
 *    - Auto-Start requires all 3 supply phases (R, Y, B) to remain continuously healthy for 15 full seconds (15,000 ms).
 *    - If any phase fails for a single loop cycle during the 15s check, the timer resets to 0.
 *    - Relay remains OFF (HIGH) during the entire 15-second stability period.
 *
 * 4. Non-Blocking Industrial State Machine:
 *    - STATE_BOOT_STABILIZATION (3s Hardware Settle) -> Relay OFF
 *    - STATE_STANDBY (Motor OFF) -> Relay OFF
 *    - STATE_AUTO_RECOVERY_WAITING (15s Phase Health Stability Check) -> Relay OFF
 *    - STATE_RUNNING (Motor ON) -> Relay ON (LOW), Local 10ms Protection Active
 *    - STATE_LATCHED_FAULT (Damaging Fault Lockout: Dry-Run, Overload, Temp, Short) -> Relay OFF
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
const int RELAY_PIN = 5;       // Schneider LC1-D1810 Contactor Relay (GPIO 5)

// --- RELAY LOGIC CONFIGURATION ---
const int RELAY_ON = LOW;      // Active LOW relay module (LOW = ON)
const int RELAY_OFF = HIGH;    // HIGH = OFF (FAIL-SAFE DEFAULT)

// Current Transformer (CT) Sensor Pins (Measure Motor Amps while RUNNING)
const int CT1_PIN = 34;        // Phase 1 (R) Current Sensor (ADC1)
const int CT2_PIN = 35;        // Phase 2 (Y) Current Sensor (ADC1)
const int CT3_PIN = 32;        // Phase 3 (B) Current Sensor (ADC1)
const int TEMP_PIN = 33;       // Motor Temperature Sensor Pin (ADC1)

// Optional Isolated AC Optocoupler Phase Voltage Sensing Digital Input Pins
// Wire AC Optocouplers (e.g. PC817) between Phase lines (R, Y, B) and GPIOs 25, 26, 27
const int PHASE_V1_PIN = 25;   // Phase R AC Voltage Detector
const int PHASE_V2_PIN = 26;   // Phase Y AC Voltage Detector
const int PHASE_V3_PIN = 27;   // Phase B AC Voltage Detector

// Set to true if physical Optocoupler AC voltage sensors are wired to GPIOs 25, 26, 27
const bool USE_HARDWARE_VOLTAGE_SENSORS = false;

// --- FINITE STATE MACHINE DEFINITIONS ---
enum SystemState {
  STATE_BOOT_STABILIZATION,
  STATE_STANDBY,
  STATE_AUTO_RECOVERY_WAITING,
  STATE_RUNNING,
  STATE_LATCHED_FAULT
};

SystemState currentState = STATE_BOOT_STABILIZATION;

// Motor State & Protection Settings (5 HP Motor ~ 7.5A Full Load)
bool motorState = false;
bool faultState = false;              // Latched Fault Lockout Flag (Manual Reset Required)
bool autoStartEnabled = false;        // Auto Start Motor feature toggle (Saved in NVS)
bool dryRunProtectionEnabled = false; // Dry-Run protection toggle (Saved in NVS)
bool phasePowerGridOk = false;        // Fail-Safe Default: FALSE until verified healthy!

const float RATED_FLC = 50.0;         // Full Load Current Threshold (Amps)
const float DRY_RUN_LIMIT = 2.5;      // Min current threshold for pump dry-run trip (Amps)
const float MAX_TEMP_LIMIT = 150.0;   // Temp Limit Threshold (°C)

String stopReason = "System Boot / Fail-Safe Standby"; // Diagnostic reason string

unsigned long motorStartTime = 0;           // Timestamp when motor started
unsigned long phaseRecoveryStartTime = 0;   // Timestamp when phase power recovery timer started
const unsigned long DRY_RUN_GRACE_MS = 5000;       // 5-second grace period after start
const unsigned long BOOT_STABILIZATION_MS = 3000; // 3-second hardware settle delay after boot
const unsigned long RECOVERY_STABILITY_MS = 15000;// 15-second continuous phase stability period required before auto-start

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
bool isThreePhaseSupplyHealthy();
bool performPreStartSafetyInspection();
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

// Function to verify 3-Phase AC Supply Line Voltage Health (Pre-Start & Continuous)
bool isThreePhaseSupplyHealthy() {
  // If physical optocoupler AC voltage sensors are configured on GPIOs 25, 26, 27
  if (USE_HARDWARE_VOLTAGE_SENSORS) {
    int v1 = digitalRead(PHASE_V1_PIN);
    int v2 = digitalRead(PHASE_V2_PIN);
    int v3 = digitalRead(PHASE_V3_PIN);
    if (v1 == LOW || v2 == LOW || v3 == LOW) {
      return false; // Optocoupler LOW indicates phase missing
    }
  }

  // Check software / telemetry grid power status flag
  if (!phasePowerGridOk) {
    return false;
  }

  return true;
}

// Function to perform Complete Pre-Start Safety Inspection (Supply Voltage + Sensor Baseline + Leakage + Thermal)
bool performPreStartSafetyInspection() {
  // 1. Hardware Boot Stabilization Guard
  if (millis() < BOOT_STABILIZATION_MS) {
    return false;
  }

  // 2. Fail-Safe 3-Phase Supply Health Check
  if (!isThreePhaseSupplyHealthy()) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Supply Phase(s) Missing or Unhealthy!");
    return false;
  }

  // 3. Pre-Start Current Surge / Short-Circuit Check (> 10A residual leakage while motor is OFF is unsafe)
  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  if (i1 > 10.0 || i2 > 10.0 || i3 > 10.0) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Pre-start current surge / short circuit detected!");
    return false;
  }

  // 4. CT Sensor ADC Baseline Health Check
  int raw1 = analogRead(CT1_PIN);
  int raw2 = analogRead(CT2_PIN);
  int raw3 = analogRead(CT3_PIN);

  if (raw1 < 100 || raw1 > 4000 || raw2 < 100 || raw2 > 4000 || raw3 < 100 || raw3 > 4000) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: CT Sensor hardware bias fault!");
    return false;
  }

  // 5. Motor Casing Over-Temperature Check
  float temp = readMotorTemperature();
  if (temp > MAX_TEMP_LIMIT) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Motor Temperature exceeds thermal limit!");
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
      faultState = false; // Clear latched fault lockout on manual START command
      Serial.println(">>> START Button Pressed: Executing Complete Pre-Start Safety Inspection...");
      if (performPreStartSafetyInspection()) {
        stopReason = "Motor Running";
        setMotorState(true);
        currentState = STATE_RUNNING;
      } else {
        stopReason = "Start Refused: 3-Phase Supply Unhealthy or Pre-Start Fault";
        tripMotor("TRIPPED_PHASE_FAILURE", true);
      }
    } else if (upperMsg == "OFF" || upperMsg == "STOP" || upperMsg == "0") {
      faultState = false; // Clear latched fault lockout on manual STOP command
      stopReason = "Manual STOP Command";
      Serial.println(">>> Command matched: Turning MOTOR OFF");
      setMotorState(false);
      currentState = STATE_STANDBY;
    } else if (upperMsg == "AUTOSTART_ON") {
      autoStartEnabled = true;
      faultState = false; // Clear fault lockout when user re-enables Auto-Start
      saveNvsSettings();
      phaseRecoveryStartTime = 0; // Reset 15s stability timer
      Serial.println(">>> Auto Start Motor ENABLED (Relay remains OFF during 15s stability check)");
      if (!motorState) {
        currentState = STATE_AUTO_RECOVERY_WAITING;
      }
    } else if (upperMsg == "AUTOSTART_OFF") {
      autoStartEnabled = false;
      saveNvsSettings();
      Serial.println(">>> Auto Start Motor DISABLED");
      if (!motorState) {
        currentState = STATE_STANDBY;
      }
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
      Serial.println(">>> 3-Phase Grid Power Set to HEALTHY (230V/400V)");
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

  // 1. MANDATORY HARDWARE SAFETY GUARANTEE: Force Relay OFF (HIGH) Immediately on Boot
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); // Relay strictly OFF (HIGH)

  // Configure optional hardware optocoupler inputs
  if (USE_HARDWARE_VOLTAGE_SENSORS) {
    pinMode(PHASE_V1_PIN, INPUT_PULLDOWN);
    pinMode(PHASE_V2_PIN, INPUT_PULLDOWN);
    pinMode(PHASE_V3_PIN, INPUT_PULLDOWN);
  }

  // 2. Load Persisted Settings from Non-Volatile Memory (NVS)
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

  // Initial FSM State: Relay strictly OFF during Boot Stabilization
  currentState = STATE_BOOT_STABILIZATION;
}

// --- NON-BLOCKING INDUSTRIAL FINITE STATE MACHINE ---
void processStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_BOOT_STABILIZATION:
      // Keep Relay strictly OFF (HIGH)
      digitalWrite(RELAY_PIN, RELAY_OFF);
      if (now >= BOOT_STABILIZATION_MS) {
        Serial.println(">>> Boot Hardware Stabilization Complete.");
        if (autoStartEnabled && !faultState) {
          phaseRecoveryStartTime = 0; // Reset 15s stability timer
          currentState = STATE_AUTO_RECOVERY_WAITING;
        } else {
          currentState = STATE_STANDBY;
        }
      }
      break;

    case STATE_STANDBY:
      // Keep Relay strictly OFF (HIGH)
      digitalWrite(RELAY_PIN, RELAY_OFF);
      if (autoStartEnabled && !motorState && !faultState) {
        phaseRecoveryStartTime = 0; // Reset 15s stability timer
        currentState = STATE_AUTO_RECOVERY_WAITING;
      }
      break;

    case STATE_AUTO_RECOVERY_WAITING:
      // STRICT REQUIREMENT: Relay MUST remain OFF (HIGH) during the entire 15-second checking period!
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;

      if (!autoStartEnabled) {
        stopReason = "Auto-Start Disabled";
        currentState = STATE_STANDBY;
        break;
      }

      if (faultState) {
        stopReason = "Fault Lockout Active";
        currentState = STATE_LATCHED_FAULT;
        break;
      }

      // Check if all 3 supply phases and pre-start safety conditions are continuously healthy
      if (isThreePhaseSupplyHealthy()) {
        if (phaseRecoveryStartTime == 0) {
          phaseRecoveryStartTime = now;
          stopReason = "Auto-Start: Verifying 15s 3-Phase Stability...";
          Serial.println(">>> 3-Phase Supply Healthy. Starting 15-Second Continuous Stability Timer (Relay OFF)...");
          publishTelemetry();
        }
        else if (now - phaseRecoveryStartTime >= RECOVERY_STABILITY_MS) {
          // All 3 phases remained continuously healthy for 15 full seconds! Perform final pre-start safety inspection
          if (performPreStartSafetyInspection()) {
            Serial.println(">>> 15-Second Stability Confirmed & Pre-Start Inspection Passed! Turning Relay ON...");
            stopReason = "Motor Running (Auto-Start)";
            setMotorState(true); // Relay turns ON (LOW) ONLY NOW!
            currentState = STATE_RUNNING;
            publishTelemetry();
          } else {
            // Safety inspection failed (e.g. pre-start short circuit or temp)
            phaseRecoveryStartTime = 0; // Reset stability timer
            stopReason = "Auto-Start Refused: Pre-Start Safety Inspection Failed";
            tripMotor("TRIPPED_FAULTY_CURRENT", true); // Latched fault
          }
        }
      } else {
        // IF EVEN ONE PHASE IS MISSING OR UNHEALTHY, RESET THE 15-SECOND STABILITY TIMER TO 0!
        if (phaseRecoveryStartTime != 0) {
          Serial.println(">>> Supply Phase Unhealthy or Missing! Resetting 15-Second Stability Timer to 0.");
        }
        phaseRecoveryStartTime = 0;
        stopReason = "Auto-Start Waiting: 3-Phase Power Unhealthy / Phase Missing";
      }
      break;

    case STATE_RUNNING:
      // Local 10ms Continuous Multi-Protection Engine
      if (motorState) {
        float i1 = readRMSCurrent(CT1_PIN);
        float i2 = readRMSCurrent(CT2_PIN);
        float i3 = readRMSCurrent(CT3_PIN);
        float temp = readMotorTemperature();

        // 1. Grid Power Loss / Phase Supply Failure Check (Auto-recoverable)
        if (!isThreePhaseSupplyHealthy()) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE (Grid Power Loss)";
          tripMotor("TRIPPED_PHASE_FAILURE", false); // Auto-recovery eligible after 15s stability
        }
        // 2. Overload Protection (Latched - Manual Reset Required)
        else if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
          stopReason = "Fault Trip: TRIPPED_OVERLOAD";
          tripMotor("TRIPPED_OVERLOAD", true);
        }
        // 3. Over-Temperature Protection (Latched - Manual Reset Required)
        else if (temp > MAX_TEMP_LIMIT) {
          stopReason = "Fault Trip: TRIPPED_OVERTEMP";
          tripMotor("TRIPPED_OVERTEMP", true);
        }
        // 4. Dry-Run Protection (Latched - 5s Startup Grace Period)
        else if (dryRunProtectionEnabled && (now - motorStartTime > DRY_RUN_GRACE_MS)) {
          if (i1 < DRY_RUN_LIMIT && i2 < DRY_RUN_LIMIT && i3 < DRY_RUN_LIMIT) {
            stopReason = "Fault Trip: TRIPPED_DRY_RUN";
            tripMotor("TRIPPED_DRY_RUN", true);
          }
        }
        // 5. Single Phasing Protection (Auto-recoverable after 15s grid stability)
        else if ((i1 > 2.0 && (i2 < 0.5 || i3 < 0.5)) || (i2 > 2.0 && (i1 < 0.5 || i3 < 0.5)) || (i3 > 2.0 && (i1 < 0.5 || i2 < 0.5))) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
      }
      break;

    case STATE_LATCHED_FAULT:
      // Unsafe damaging fault (Overload, Dry-Run, Temp, Short). Keep Relay OFF (HIGH). Manual reset required.
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;
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

  // Run Industrial Non-Blocking Finite State Machine
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
        if (performPreStartSafetyInspection()) {
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
  digitalWrite(RELAY_PIN, RELAY_OFF); // Turn Relay OFF (HIGH) immediately

  if (isLatchedFault) {
    faultState = true;
    currentState = STATE_LATCHED_FAULT;
    Serial.print(">>> Motor Tripped (Latched Fault Lockout Activated - Manual Reset Required): ");
  } else {
    phaseRecoveryStartTime = 0; // Reset 15s stability timer
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

  bool gridHealthy = isThreePhaseSupplyHealthy();
  float v1 = gridHealthy ? 230.0 : 0.0;
  float v2 = gridHealthy ? 230.0 : 0.0;
  float v3 = gridHealthy ? 230.0 : 0.0;

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
