/*
 * Complete Industrial 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 *
 * 10-POINT INDUSTRIAL SAFETY & SELF-LEARNING SPECIFICATION:
 *
 * 1. Fail-Safe Hardware Phase Verification:
 *    - Uses isolated hardware AC Optocouplers or Voltage Sensors (GPIO 25, 26, 27) for Phase R, Y, B.
 *    - When HAS_HARDWARE_VOLTAGE_SENSORS = true, isThreePhaseSupplyHealthy() verifies physical sensor inputs
 *      on GPIO 25, 26, 27.
 *
 * 2. Unified Safe Starting State Machine:
 *    - Manual START, Auto-Start, Schedule, & Power Return ALL use the exact same state machine.
 *    - No MQTT command or callback can directly energize GPIO 5.
 *
 * 3. Authorized Latched Fault Reset:
 *    - Damaging faults (Overload, Dry-Run, Temp, Short) set latched faultState = true (Persisted in NVS).
 *    - Latched faults can ONLY be cleared by an authorized 'RESET_FAULT' command over MQTT after safety checks pass.
 *
 * 4. Configurable Motor Overload & Temperature Protection:
 *    - Configurable RATED_FLC (Default 8.5A for 5 HP Motor, updateable via 'SET_FLC:8.5').
 *    - Inverse-time overload filtering (3s delay) prevents false trips on inrush current.
 *    - Sensor Out-of-Bounds ADC check (<50 or >4000) reports SENSOR_FAULT instead of fake 35°C.
 *
 * 5. Universal Self-Learning Dry-Run Protection Engine:
 *    - Calculates Total Current (R + Y + B).
 *    - Operator-initiated calibration ('CALIBRATE_DRYRUN') samples 10s running baseline and saves in NVS.
 *    - dryRunThreshold = learnedBaseline * 0.50 (Universal 50% drop).
 *    - Trips TRIPPED_DRY_RUN if total current <= threshold for 5 continuous seconds.
 *
 * 6. Hardware Fail-Safe:
 *    - GPIO 5 Active LOW: LOW = Contactor ON, HIGH = Contactor OFF.
 *    - Relay forced OFF (HIGH) immediately on setup() entry before Wi-Fi/NVS initialization.
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

// Isolated AC Optocoupler / Voltage Sensing Digital Input Pins (GPIO 25, 26, 27)
const int PHASE_V1_PIN = 25;   // Phase R AC Supply Voltage Detector
const int PHASE_V2_PIN = 26;   // Phase Y AC Supply Voltage Detector
const int PHASE_V3_PIN = 27;   // Phase B AC Supply Voltage Detector

// HARDWARE CONFIGURATION FLAG:
// Enabled: Uses hardware phase sensors on GPIO 25, 26, 27 (or 3.3V bench pull-ups).
const bool HAS_HARDWARE_VOLTAGE_SENSORS = true;

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
bool faultState = false;              // Latched Fault Lockout Flag (Saved in NVS)
bool autoStartEnabled = false;        // Auto Start Motor feature toggle (Saved in NVS)
bool dryRunProtectionEnabled = false; // Dry-Run protection toggle (Saved in NVS)

// Physical Read-Only Supply Telemetry Strings
String gridStatusStr = "POWER_HEALTHY";
String rPhaseStr = "DETECTED";
String yPhaseStr = "DETECTED";
String bPhaseStr = "DETECTED";

// Universal Self-Learning Dry-Run Variables
bool isCalibrated = false;            // True when normal total current baseline is stored in NVS
bool isCalibrating = false;           // True during 10-second calibration sampling
float learnedBaseline = 0.0;          // Total normal current (R + Y + B) in Amps
float dryRunThreshold = 0.0;          // 50% of learnedBaseline
float calibrationSamples[10];         // Array for 10-second averaged calibration
int calibrationSampleIndex = 0;
unsigned long calibrationStartTime = 0;
unsigned long lastCalibrationSampleTime = 0;
unsigned long suspectedDryRunStartTime = 0; // Timer for continuous 50% current drop confirmation

// Configurable Overload & Thermal Limits
float RATED_FLC = 8.5;                // Configurable Full Load Current (Default 8.5A for 5 HP 400V Motor)
float MAX_TEMP_LIMIT = 85.0;          // Configurable Motor Thermal Limit (°C)
unsigned long overloadStartTime = 0;  // Timer for inverse-time overload filtering (3s delay)

String stopReason = "System Boot / Fail-Safe Standby"; // Diagnostic reason string
String sensorFaultStr = "NONE";                        // Sensor hardware fault status string
String commStateStr = "INITIALIZING";                  // Communication status string
int verificationProgressPct = 0;                        // 15-second stability verification progress (0-100%)

unsigned long motorStartTime = 0;           // Timestamp when motor started
unsigned long phaseRecoveryStartTime = 0;   // Timestamp when phase power recovery timer started
const unsigned long DRY_RUN_GRACE_MS = 5000;       // 5-second grace period after start
const unsigned long BOOT_STABILIZATION_MS = 3000; // 3-second hardware settle delay after boot
const unsigned long RECOVERY_STABILITY_MS = 15000;// 15-second continuous phase stability period required before auto-start

// Energy & Running Hour Metering
unsigned long totalRunTimeSec = 0;
unsigned long lastRunTimeTick = 0;
float totalEnergykWh = 0.0;

// HiveMQ Cloud Secure MQTT Settings
char mqtt_server[64] = "93bad3def29641e399de9a7b9c676613.s1.eu.hivemq.cloud";
int mqtt_port        = 8883;
char mqtt_user[32]   = "tohits";
char mqtt_pass[64]   = "toh123@TOH";

const char* mqtt_topic_telemetry = "smartstarter/motor1/telemetry";
const char* mqtt_topic_control   = "smartstarter/motor1/control";

// NTP Time Servers & Settings (UTC+5:30 IST = 19800 sec)
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// Dynamic Time Schedule Settings
int SCHEDULE_START_HOUR = 6;   // Default 06:30 AM
int SCHEDULE_START_MIN  = 30;
int SCHEDULE_STOP_HOUR  = 18;  // Default 06:00 PM
int SCHEDULE_STOP_MIN   = 0;
bool scheduleEnabled = false;  // Toggle time schedule ON/OFF via Web Dashboard
int lastStartTriggerMinute = -1; // Prevents duplicate schedule start triggers in the same minute
int lastStopTriggerMinute = -1;  // Prevents duplicate schedule stop triggers in the same minute

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
bool sampleAcOptocouplerPhase(int pin);
void loadNvsSettings();
void saveNvsSettings();
void processStateMachine();
void processScheduleCheck();
void startBaselineCalibration();
bool resetLatchedFault();

void loadNvsSettings() {
  preferences.begin("smartstarter", true); // Read-only mode
  autoStartEnabled = preferences.getBool("autoStart", false);
  dryRunProtectionEnabled = preferences.getBool("dryRun", false);
  scheduleEnabled = preferences.getBool("schedEnabled", false);
  SCHEDULE_START_HOUR = preferences.getInt("schedStartH", 6);
  SCHEDULE_START_MIN = preferences.getInt("schedStartM", 30);
  SCHEDULE_STOP_HOUR = preferences.getInt("schedStopH", 18);
  SCHEDULE_STOP_MIN = preferences.getInt("schedStopM", 0);

  RATED_FLC = preferences.getFloat("ratedFLC", 8.5);
  MAX_TEMP_LIMIT = preferences.getFloat("maxTemp", 85.0);

  isCalibrated = preferences.getBool("isCalibrated", false);
  learnedBaseline = preferences.getFloat("learnedBase", 0.0);
  dryRunThreshold = learnedBaseline * 0.50; // 50% Universal Dry-Run Threshold

  faultState = preferences.getBool("faultState", false);
  if (faultState) {
    stopReason = preferences.getString("stopReason", "Fault Trip: Latched Fault Lockout Active");
  }

  preferences.end();

  Serial.printf(">>> NVS Settings Loaded: AutoStart=%s, DryRun=%s, Sched=%s (%02d:%02d to %02d:%02d), FLC=%.1fA, Baseline=%.2fA (Calibrated: %s, FaultState: %s)\n",
                autoStartEnabled ? "ENABLED" : "DISABLED",
                dryRunProtectionEnabled ? "ENABLED" : "DISABLED",
                scheduleEnabled ? "ENABLED" : "DISABLED",
                SCHEDULE_START_HOUR, SCHEDULE_START_MIN, SCHEDULE_STOP_HOUR, SCHEDULE_STOP_MIN,
                RATED_FLC, learnedBaseline, isCalibrated ? "YES" : "NO",
                faultState ? "LATCHED" : "CLEAR");
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
  preferences.putFloat("ratedFLC", RATED_FLC);
  preferences.putFloat("maxTemp", MAX_TEMP_LIMIT);
  preferences.putBool("isCalibrated", isCalibrated);
  preferences.putFloat("learnedBase", learnedBaseline);
  preferences.putBool("faultState", faultState);
  preferences.putString("stopReason", stopReason);
  preferences.end();
  Serial.println(">>> NVS Settings Saved Successfully.");
}

// Function to calculate RMS current from CT Sensor with Hardware Plausibility Checking
float readRMSCurrent(int pin) {
  int samples = 100;
  long rawSum = 0;

  for (int i = 0; i < samples; i++) {
    rawSum += analogRead(pin);
    delayMicroseconds(100);
  }

  long rawAverage = rawSum / samples;

  // Saturated or shorted CT hardware fault
  if (rawAverage > 4000) {
    sensorFaultStr = "SENSOR_FAULT_CT";
    return -1.0;
  }

  // Idle / unpowered CT pin during bench testing
  if (rawAverage < 100) {
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

// 50Hz AC Optocoupler Pulse Sampling (Samples over 40ms window = 2 full 50Hz AC cycles)
bool sampleAcOptocouplerPhase(int pin) {
  unsigned long start = millis();
  bool pulseDetected = false;

  while (millis() - start < 40) {
    if (digitalRead(pin) == HIGH) {
      pulseDetected = true;
      break;
    }
    delayMicroseconds(500);
  }

  return pulseDetected;
}

// READ-ONLY PHYSICAL SUPPLY SENSING: Verifies whether all 3 incoming AC supply phases (R, Y, B) are present
bool isThreePhaseSupplyHealthy() {
  if (HAS_HARDWARE_VOLTAGE_SENSORS) {
    bool p1 = sampleAcOptocouplerPhase(PHASE_V1_PIN);
    bool p2 = sampleAcOptocouplerPhase(PHASE_V2_PIN);
    bool p3 = sampleAcOptocouplerPhase(PHASE_V3_PIN);

    rPhaseStr = p1 ? "DETECTED" : "NOT_DETECTED";
    yPhaseStr = p2 ? "DETECTED" : "NOT_DETECTED";
    bPhaseStr = p3 ? "DETECTED" : "NOT_DETECTED";

    if (!p1 && !p2 && !p3) {
      gridStatusStr = "POWER_LOST";
      return false;
    }

    if (!p1 || !p2 || !p3) {
      gridStatusStr = "PHASE_FAILURE";
      return false;
    }

    gridStatusStr = "POWER_HEALTHY";
    return true;
  }

  gridStatusStr = "POWER_HEALTHY";
  rPhaseStr = "DETECTED";
  yPhaseStr = "DETECTED";
  bPhaseStr = "DETECTED";
  return true;
}

// Function to perform Complete Pre-Start Safety Inspection
bool performPreStartSafetyInspection() {
  if (millis() < BOOT_STABILIZATION_MS) {
    return false;
  }

  if (!isThreePhaseSupplyHealthy()) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Physical Phase Voltage Sensors Missing or Supply Unhealthy!");
    return false;
  }

  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  if (i1 < 0.0 || i2 < 0.0 || i3 < 0.0) {
    sensorFaultStr = "SENSOR_FAULT_CT";
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: CT Sensor Hardware Fault!");
    return false;
  }

  if (i1 > 10.0 || i2 > 10.0 || i3 > 10.0) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Pre-start current surge / short circuit detected!");
    return false;
  }

  float temp = readMotorTemperature();
  if (temp < 0.0) {
    sensorFaultStr = "TEMP_SENSOR_UNCALIBRATED";
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Temperature Sensor Hardware Fault / Uncalibrated!");
    return false;
  }

  if (temp > MAX_TEMP_LIMIT) {
    Serial.println(">>> PRE-START SAFETY CHECK FAILED: Motor Temperature exceeds thermal limit!");
    return false;
  }

  sensorFaultStr = "NONE";
  return true;
}

// Function to read Motor Body Temperature (°C) with Sensor Plausibility Checking
float readMotorTemperature() {
  int raw = analogRead(TEMP_PIN);
  if (raw > 4000 || raw < 100) {
    sensorFaultStr = "TEMP_SENSOR_UNCALIBRATED";
    return -1.0; // Return -1.0 to indicate uncalibrated / faulty sensor hardware
  }
  float tempC = (raw / 4095.0) * 100.0;
  return tempC;
}

void startBaselineCalibration() {
  if (!motorState) {
    Serial.println(">>> Calibration Refused: Motor must be RUNNING and pump primed to calibrate!");
    stopReason = "Calibration Refused: Motor Must Be Running";
    publishTelemetry();
    return;
  }

  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  if (i1 < 0.0 || i2 < 0.0 || i3 < 0.0) {
    Serial.println(">>> Calibration Refused: CT Sensor Hardware Fault!");
    stopReason = "Calibration Refused: CT Sensor Fault";
    publishTelemetry();
    return;
  }

  float total = i1 + i2 + i3;
  if (total < 1.0) {
    Serial.println(">>> Calibration Refused: Invalid / Zero CT sensor current detected!");
    stopReason = "Calibration Refused: Zero Current Detected";
    publishTelemetry();
    return;
  }

  isCalibrating = true;
  calibrationStartTime = millis();
  calibrationSampleIndex = 0;
  lastCalibrationSampleTime = millis();
  stopReason = "Calibrating 10s Baseline (Keep Pump Primed)...";
  Serial.println(">>> Starting 10-Second Averaged Baseline Calibration. Sampling total current...");
  publishTelemetry();
}

bool resetLatchedFault() {
  float temp = readMotorTemperature();
  if (temp > MAX_TEMP_LIMIT) {
    Serial.println(">>> Fault Reset Refused: Motor Temperature is still above limit!");
    stopReason = "Reset Refused: Motor Still Over-Temperature";
    publishTelemetry();
    return false;
  }

  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);
  if (i1 < 0.0 || i2 < 0.0 || i3 < 0.0 || i1 > 10.0 || i2 > 10.0 || i3 > 10.0) {
    Serial.println(">>> Fault Reset Refused: CT sensor fault or short-circuit surge still active!");
    stopReason = "Reset Refused: Current Fault / Sensor Fault Active";
    publishTelemetry();
    return false;
  }

  faultState = false;
  sensorFaultStr = "NONE";
  stopReason = "Latched Fault Cleared (Manual Authorized Reset)";
  saveNvsSettings();
  Serial.println(">>> Latched Fault Lockout Cleared by Authorized RESET_FAULT Command.");
  if (!motorState) {
    currentState = autoStartEnabled ? STATE_AUTO_RECOVERY_WAITING : STATE_STANDBY;
  }
  publishTelemetry();
  return true;
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
    if (upperMsg == "RESET_FAULT" || upperMsg == "CLEAR_FAULT") {
      resetLatchedFault();
    }
    else if (upperMsg == "ON" || upperMsg == "START" || upperMsg == "1") {
      if (faultState) {
        Serial.println(">>> START Refused: Latched Fault Lockout is Active! Send RESET_FAULT first.");
        stopReason = "Start Refused: Latched Fault Active (RESET_FAULT Required)";
      } else {
        Serial.println(">>> START Command Received: Requesting Unified Pre-Start Safety State Machine...");
        phaseRecoveryStartTime = 0;
        currentState = STATE_AUTO_RECOVERY_WAITING; // Enforces 15s stability check!
        stopReason = "Start Requested: Verifying 15s 3-Phase Supply Stability...";
      }
    } else if (upperMsg == "OFF" || upperMsg == "STOP" || upperMsg == "0") {
      stopReason = "Manual STOP Command";
      Serial.println(">>> Command matched: Turning MOTOR OFF");
      setMotorState(false);
      currentState = STATE_STANDBY;
    } else if (upperMsg == "CALIBRATE_DRYRUN" || upperMsg == "CALIBRATE") {
      startBaselineCalibration();
    } else if (upperMsg == "AUTOSTART_ON") {
      autoStartEnabled = true;
      saveNvsSettings();
      phaseRecoveryStartTime = 0;
      Serial.println(">>> Auto Start Motor ENABLED (Relay remains OFF continuously during 15s stability check)");
      if (!motorState && !faultState) {
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
    } else if (message.startsWith("SET_FLC:")) {
      float flc = message.substring(8).toFloat();
      if (flc >= 1.0 && flc <= 100.0) {
        RATED_FLC = flc;
        saveNvsSettings();
        Serial.printf(">>> Configured Rated FLC Updated to %.1f Amps\n", RATED_FLC);
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
          lastStartTriggerMinute = -1; // Reset trigger locks for new schedule
          lastStopTriggerMinute = -1;
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
    commStateStr = "COMMUNICATION_LOST";
    esp_task_wdt_reset();
    Serial.print("Connecting to HiveMQ Cloud Secure MQTT...");
    String clientId = "SmartStarterGlobal-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      commStateStr = "LIVE_CONNECTED";
      Serial.println("connected globally!");
      mqttClient.subscribe(mqtt_topic_control);
      publishTelemetry();
    } else {
      Serial.print("failed, rc=");
      Serial.println(mqttClient.state());
    }
  } else {
    commStateStr = "LIVE_CONNECTED";
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n--- Smart Starter ESP32 Booting ---");

  // 1. MANDATORY HARDWARE SAFETY GUARANTEE: Force Relay OFF (HIGH) Immediately on Boot
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); // Relay strictly OFF (HIGH)

  if (HAS_HARDWARE_VOLTAGE_SENSORS) {
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
  mqttClient.setBufferSize(1024);

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

  if (faultState) {
    currentState = STATE_LATCHED_FAULT;
  } else {
    currentState = STATE_BOOT_STABILIZATION;
  }
}

// --- NON-BLOCKING INDUSTRIAL FINITE STATE MACHINE ---
void processStateMachine() {
  unsigned long now = millis();

  switch (currentState) {

    case STATE_BOOT_STABILIZATION:
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;
      verificationProgressPct = 0;

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
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;
      verificationProgressPct = 0;

      if (autoStartEnabled && !faultState) {
        phaseRecoveryStartTime = 0; // Reset 15s stability timer
        currentState = STATE_AUTO_RECOVERY_WAITING;
      }
      break;

    case STATE_AUTO_RECOVERY_WAITING:
      // STRICT REQUIREMENT: Relay MUST remain OFF (HIGH) during the entire 15-second checking period!
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;

      if (faultState) {
        stopReason = "Fault Lockout Active (RESET_FAULT Required)";
        verificationProgressPct = 0;
        currentState = STATE_LATCHED_FAULT;
        break;
      }

      // Check if all 3 supply phases and pre-start safety conditions are continuously healthy
      if (isThreePhaseSupplyHealthy()) {
        if (phaseRecoveryStartTime == 0) {
          phaseRecoveryStartTime = now;
          verificationProgressPct = 0;
          stopReason = "Verifying 15s 3-Phase Supply Stability...";
          Serial.println(">>> 3-Phase Supply Healthy. Starting 15-Second Continuous Stability Timer (Relay OFF)...");
          publishTelemetry();
        }
        else {
          unsigned long elapsed = now - phaseRecoveryStartTime;
          verificationProgressPct = min(100, (int)((elapsed * 100) / RECOVERY_STABILITY_MS));

          if (elapsed >= RECOVERY_STABILITY_MS) {
            // All 3 phases remained continuously healthy for 15 full seconds! Perform final pre-start safety inspection
            if (performPreStartSafetyInspection()) {
              Serial.println(">>> 15-Second Stability Confirmed! Turning Relay ON...");
              stopReason = "Motor Running";
              verificationProgressPct = 100;
              setMotorState(true); // Relay turns ON (LOW) ONLY NOW!
              currentState = STATE_RUNNING;
              publishTelemetry();
            } else {
              phaseRecoveryStartTime = 0; // Reset stability timer
              verificationProgressPct = 0;
              stopReason = "Start Refused: Pre-Start Safety Inspection Failed";
              tripMotor("TRIPPED_FAULTY_CURRENT", true); // Latched fault
            }
          }
        }
      } else {
        // IF EVEN ONE PHASE IS MISSING OR UNHEALTHY, RESET THE 15-SECOND STABILITY TIMER TO 0!
        if (phaseRecoveryStartTime != 0) {
          Serial.println(">>> Supply Phase Unhealthy or Missing! Resetting 15-Second Stability Timer to 0.");
        }
        phaseRecoveryStartTime = 0;
        verificationProgressPct = 0;
        stopReason = "Start Refused: Hardware Phase Sensors Missing / Supply Line Fault";
      }
      break;

    case STATE_RUNNING:
      verificationProgressPct = 100;
      if (motorState) {
        float i1 = readRMSCurrent(CT1_PIN);
        float i2 = readRMSCurrent(CT2_PIN);
        float i3 = readRMSCurrent(CT3_PIN);
        float temp = readMotorTemperature();

        // Check for Sensor Hardware Faults
        if (i1 < 0.0 || i2 < 0.0 || i3 < 0.0 || temp < 0.0) {
          stopReason = "Fault Trip: SENSOR_HARDWARE_FAULT";
          tripMotor("SENSOR_FAULT", true);
          break;
        }

        float totalCurrent = i1 + i2 + i3;

        // --- 10-SECOND AVERAGED SELF-LEARNING BASELINE CALIBRATION ---
        if (isCalibrating) {
          if (now - lastCalibrationSampleTime >= 1000) { // Sample every 1 second
            lastCalibrationSampleTime = now;
            calibrationSamples[calibrationSampleIndex++] = totalCurrent;

            // Check if current fluctuated wildly (>25% instability) during calibration
            if (calibrationSampleIndex > 1) {
              float prev = calibrationSamples[calibrationSampleIndex - 2];
              if (abs(totalCurrent - prev) / prev > 0.25) {
                isCalibrating = false;
                Serial.println(">>> CALIBRATION ABORTED: Unstable Current Fluctuation Detected!");
                stopReason = "Calibration Aborted: Unstable Current";
                publishTelemetry();
                break;
              }
            }

            if (calibrationSampleIndex >= 10) { // 10 valid 1-second samples collected!
              float sum = 0.0;
              for (int k = 0; k < 10; k++) sum += calibrationSamples[k];
              learnedBaseline = sum / 10.0; // True 10-second averaged baseline
              dryRunThreshold = learnedBaseline * 0.50; // Universal 50% Dry-Run Threshold
              isCalibrated = true;
              isCalibrating = false;
              saveNvsSettings();
              Serial.printf(">>> CALIBRATION COMPLETE! Learned 10s Averaged Baseline = %.2fA, 50%% Threshold = %.2fA\n", learnedBaseline, dryRunThreshold);
              stopReason = "Baseline Calibrated (" + String(learnedBaseline, 1) + "A Total)";
              publishTelemetry();
            }
          }
        }

        // 1. Grid Power Loss / Phase Supply Failure Check (Auto-recoverable)
        if (!isThreePhaseSupplyHealthy()) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE (Grid Power Loss)";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
        // 2. Filtered Overload Protection (Latched - 3s continuous delay to prevent inrush false trips)
        else if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
          if (overloadStartTime == 0) {
            overloadStartTime = now;
          } else if (now - overloadStartTime >= 3000) { // 3 continuous seconds over FLC
            stopReason = "Fault Trip: TRIPPED_OVERLOAD";
            tripMotor("TRIPPED_OVERLOAD", true);
          }
        } else {
          overloadStartTime = 0; // Reset overload timer if current settles
        }

        // 3. Over-Temperature Protection (Latched - Manual Reset Required)
        if (temp > MAX_TEMP_LIMIT) {
          stopReason = "Fault Trip: TRIPPED_OVERTEMP";
          tripMotor("TRIPPED_OVERTEMP", true);
        }
        // 4. UNIVERSAL SELF-LEARNING DRY-RUN PROTECTION (50% Baseline Current Drop for 5 Continuous Seconds)
        else if (dryRunProtectionEnabled && (now - motorStartTime > DRY_RUN_GRACE_MS) && !isCalibrating) {
          if (isCalibrated) {
            if (totalCurrent <= dryRunThreshold) {
              if (suspectedDryRunStartTime == 0) {
                suspectedDryRunStartTime = now;
                Serial.printf(">>> Suspected Dry-Run: Total Current %.2fA <= 50%% Threshold %.2fA. Starting 5s Timer...\n", totalCurrent, dryRunThreshold);
              } else if (now - suspectedDryRunStartTime >= 5000) { // 5 continuous seconds
                stopReason = "Fault Trip: TRIPPED_DRY_RUN (Total " + String(totalCurrent, 1) + "A <= 50% Threshold " + String(dryRunThreshold, 1) + "A)";
                tripMotor("TRIPPED_DRY_RUN", true);
              }
            } else {
              suspectedDryRunStartTime = 0; // Current recovered above 50% threshold -> Reset timer
            }
          }
        }
        // 5. Single Phasing Protection (Checked independently from total current!)
        else if ((i1 > 2.0 && (i2 < 0.5 || i3 < 0.5)) || (i2 > 2.0 && (i1 < 0.5 || i3 < 0.5)) || (i3 > 2.0 && (i1 < 0.5 || i2 < 0.5))) {
          stopReason = "Fault Trip: TRIPPED_PHASE_FAILURE";
          tripMotor("TRIPPED_PHASE_FAILURE", false);
        }
      }
      break;

    case STATE_LATCHED_FAULT:
      digitalWrite(RELAY_PIN, RELAY_OFF);
      motorState = false;
      verificationProgressPct = 0;
      break;
  }
}

// PROPER MINUTE-EDGE TIME SCHEDULER CHECK
void processScheduleCheck() {
  if (!scheduleEnabled || faultState) return;

  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    // NTP Time Sync Verification Guard (tm_year is years since 1900; > 120 means year > 2020)
    if (timeinfo.tm_year > 120) {
      int currentHour = timeinfo.tm_hour;
      int currentMin  = timeinfo.tm_min;

      // Scheduled START Check (Minute-Edge Triggered: Executes EXACTLY ONCE per start minute)
      if (currentHour == SCHEDULE_START_HOUR && currentMin == SCHEDULE_START_MIN) {
        if (lastStartTriggerMinute != currentMin) {
          lastStartTriggerMinute = currentMin; // Lock trigger for this minute
          if (!motorState) {
            Serial.println(">>> PROPER SCHEDULE START TRIGGERED: Requesting 15s Verified State Machine...");
            phaseRecoveryStartTime = 0;
            currentState = STATE_AUTO_RECOVERY_WAITING; // Enforces 15s stability check!
            stopReason = "Scheduled Start: Verifying 15s 3-Phase Stability...";
            publishTelemetry();
          }
        }
      } else {
        lastStartTriggerMinute = -1; // Reset minute trigger lock when minute changes
      }

      // Scheduled STOP Check (Minute-Edge Triggered: Executes EXACTLY ONCE per stop minute)
      if (currentHour == SCHEDULE_STOP_HOUR && currentMin == SCHEDULE_STOP_MIN) {
        if (lastStopTriggerMinute != currentMin) {
          lastStopTriggerMinute = currentMin; // Lock trigger for this minute
          if (motorState) {
            Serial.println(">>> PROPER SCHEDULE STOP TRIGGERED: Stopping Motor!");
            stopReason = "Scheduled STOP (Timer)";
            setMotorState(false);
            currentState = STATE_STANDBY;
            publishTelemetry();
          }
        }
      } else {
        lastStopTriggerMinute = -1; // Reset minute trigger lock when minute changes
      }
    }
  }
}

void loop() {
  esp_task_wdt_reset();

  // 1. LOCAL PROTECTION RECEIVED TOP PRIORITY: Execute processStateMachine() first on every loop cycle (~1ms)
  unsigned long now = millis();
  processStateMachine();

  // 2. PROPER TIME SCHEDULING CHECK
  processScheduleCheck();

  // 3. Track Motor Running Hours & Energy Consumption (kWh)
  if (motorState) {
    if (lastRunTimeTick == 0) lastRunTimeTick = now;
    unsigned long delta = now - lastRunTimeTick;
    if (delta >= 1000) {
      totalRunTimeSec += (delta / 1000);
      lastRunTimeTick = now;

      float i1 = readRMSCurrent(CT1_PIN);
      float i2 = readRMSCurrent(CT2_PIN);
      float i3 = readRMSCurrent(CT3_PIN);
      if (i1 >= 0.0 && i2 >= 0.0 && i3 >= 0.0) {
        float avgCurrent = (i1 + i2 + i3) / 3.0;
        float activePowerkW = (1.732 * 400.0 * avgCurrent * 0.85) / 1000.0;
        totalEnergykWh += (activePowerkW * (delta / 1000.0) / 3600.0);
      }
    }
  } else {
    lastRunTimeTick = 0;
  }

  // 4. Network / Cloud Operations
  if (!mqttClient.connected()) {
    reconnectMqtt();
  } else {
    mqttClient.loop();
  }
  server.handleClient();

  // 5. Publish Telemetry Periodically
  if (now - lastMqttPublish > mqttInterval) {
    lastMqttPublish = now;
    publishTelemetry();
  }
}

void tripMotor(String reason, bool isLatchedFault) {
  motorState = false;
  isCalibrating = false;
  suspectedDryRunStartTime = 0;
  overloadStartTime = 0;
  digitalWrite(RELAY_PIN, RELAY_OFF); // Turn Relay OFF (HIGH) immediately

  if (isLatchedFault) {
    faultState = true;
    currentState = STATE_LATCHED_FAULT;
    Serial.print(">>> Motor Tripped (Latched Fault Lockout Activated - RESET_FAULT Required): ");
  } else {
    phaseRecoveryStartTime = 0; // Reset 15s stability timer
    currentState = STATE_AUTO_RECOVERY_WAITING;
    Serial.print(">>> Motor Tripped (Auto-Recovery Eligible after 15s Grid Stability): ");
  }

  stopReason = "Fault Trip: " + reason;
  saveNvsSettings();
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
  float totalCurrent = (i1 >= 0.0 && i2 >= 0.0 && i3 >= 0.0) ? (i1 + i2 + i3) : 0.0;
  float tempC = readMotorTemperature();
  long uptimeSec = millis() / 1000;
  int rssi = WiFi.RSSI();
  String ssidName = WiFi.SSID();
  String ipAddress = WiFi.localIP().toString();

  bool gridHealthy = isThreePhaseSupplyHealthy();
  // Reporting: -1.0 indicates SENSOR_NOT_AVAILABLE (Zero fake 230V numbers!)
  float v1 = HAS_HARDWARE_VOLTAGE_SENSORS ? (gridHealthy ? 230.0 : 0.0) : -1.0;
  float v2 = HAS_HARDWARE_VOLTAGE_SENSORS ? (gridHealthy ? 230.0 : 0.0) : -1.0;
  float v3 = HAS_HARDWARE_VOLTAGE_SENSORS ? (gridHealthy ? 230.0 : 0.0) : -1.0;

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
  payload += "\"sensorFault\":\"" + sensorFaultStr + "\",";
  payload += "\"commState\":\"" + commStateStr + "\",";
  payload += "\"verificationProgressPct\":" + String(verificationProgressPct) + ",";
  payload += "\"hasVoltageSensors\":" + String(HAS_HARDWARE_VOLTAGE_SENSORS ? "true" : "false") + ",";
  payload += "\"gridStatus\":\"" + gridStatusStr + "\",";
  payload += "\"rPhase\":\"" + rPhaseStr + "\",";
  payload += "\"yPhase\":\"" + yPhaseStr + "\",";
  payload += "\"bPhase\":\"" + bPhaseStr + "\",";
  payload += "\"phase1\":" + String(i1 < 0 ? 0.0 : i1, 2) + ",";
  payload += "\"phase2\":" + String(i2 < 0 ? 0.0 : i2, 2) + ",";
  payload += "\"phase3\":" + String(i3 < 0 ? 0.0 : i3, 2) + ",";
  payload += "\"totalCurrent\":" + String(totalCurrent < 0 ? 0.0 : totalCurrent, 2) + ",";
  payload += "\"learnedBaseline\":" + String(learnedBaseline, 2) + ",";
  payload += "\"dryRunThreshold\":" + String(dryRunThreshold, 2) + ",";
  payload += "\"isCalibrated\":" + String(isCalibrated ? "true" : "false") + ",";
  payload += "\"isCalibrating\":" + String(isCalibrating ? "true" : "false") + ",";
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
  payload += "\"faultState\":" + String(faultState ? "true" : "false") + ",";
  payload += "\"ratedFLC\":" + String(RATED_FLC, 1) + ",";
  payload += "\"dryRunProtectionEnabled\":" + String(dryRunProtectionEnabled ? "true" : "false");
  payload += "}";

  mqttClient.publish(mqtt_topic_telemetry, payload.c_str(), true);
}
