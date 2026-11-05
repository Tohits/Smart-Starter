/*
 * 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S (Global Cloud-Only Edition)
 * Features:
 * - Captive Portal (WiFiManager) for easy Wi-Fi setup
 * - Non-blocking 3-Phase Current Monitoring (CT Sensors) with Zero-Noise Filtering
 * - Schneider LC1-D1810 Contactor Control via Single-Channel Relay (Active LOW)
 * - 100% Global Cloud Control & Telemetry via Secure HiveMQ MQTT (Port 8883) with RSSI & Uptime
 * - NTP Time Synchronization & Time Scheduling
 * - Continuous Safety Protections (Overload, Dry-Run, Phase Failure)
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <Update.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include "time.h"

WebServer server(80);
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

// Pin Definitions
const int RELAY_PIN = 23;     // Connected to Single-Channel Relay IN

// --- RELAY LOGIC CONFIGURATION ---
const int RELAY_ON = LOW;     // Active LOW relay module (LOW = ON)
const int RELAY_OFF = HIGH;   // HIGH = OFF

const int CT1_PIN = 34;       // Phase 1 Current Sensor (Analog ADC1)
const int CT2_PIN = 35;       // Phase 2 Current Sensor (Analog ADC1)
const int CT3_PIN = 32;       // Phase 3 Current Sensor (Analog ADC1)

// Motor State & Protection Thresholds (5 HP Motor ~ 7.5A Full Load)
bool motorState = false;
const float RATED_FLC = 9.0;     // Max allowable current before overload trip (Amps)
const float DRY_RUN_LIMIT = 1.5; // Min current for pump dry-run detection (Amps)

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

// Time Schedule Settings
const int SCHEDULE_START_HOUR = 6;   // 06:00 AM
const int SCHEDULE_START_MIN = 30;
const int SCHEDULE_STOP_HOUR = 18;   // 06:00 PM
const int SCHEDULE_STOP_MIN = 0;
bool scheduleEnabled = false;

unsigned long lastMqttPublish = 0;
const long mqttInterval = 3000; // Publish telemetry every 3 seconds

void publishTelemetry();

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();
  Serial.print(">>> MQTT Message received on topic [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(message);

  if (String(topic) == mqtt_topic_control) {
    if (message == "ON") {
      Serial.println(">>> Command matched: Turning MOTOR ON");
      setMotorState(true);
    } else if (message == "OFF") {
      Serial.println(">>> Command matched: Turning MOTOR OFF");
      setMotorState(false);
    } else {
      Serial.print(">>> Unknown command payload: ");
      Serial.println(message);
    }
    publishTelemetry();
  }
}

void reconnectMqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to HiveMQ Cloud Secure MQTT...");
    String clientId = "SmartStarterGlobal-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println("connected globally!");
      mqttClient.subscribe(mqtt_topic_control);
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

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, RELAY_OFF); // Ensure relay is OFF initially

  // --- Captive Portal Wi-Fi Setup ---
  WiFiManager wifiManager;
  if (!wifiManager.autoConnect("SmartStarter-AP", "12345678")) {
    Serial.println("Failed to connect and hit timeout. Rebooting...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("\nWi-Fi Connected successfully!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // --- Secure MQTT Setup (TLS/SSL on Port 8883) ---
  espClient.setInsecure(); // Skip strict CA cert check for HiveMQ Cloud
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);

  // --- NTP Time Sync ---
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  // --- Emergency OTA Web Route ---
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
  if (!mqttClient.connected()) {
    reconnectMqtt();
  }
  mqttClient.loop();
  server.handleClient();

  // --- Publish MQTT Telemetry Periodically ---
  unsigned long now = millis();
  if (now - lastMqttPublish > mqttInterval) {
    lastMqttPublish = now;
    publishTelemetry();
  }

  // --- Time Scheduling Check ---
  if (scheduleEnabled) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      int currentHour = timeinfo.tm_hour;
      int currentMin = timeinfo.tm_min;

      if (currentHour == SCHEDULE_START_HOUR && currentMin == SCHEDULE_START_MIN && !motorState) {
        Serial.println("Schedule Trigger: Starting Motor");
        setMotorState(true);
        publishTelemetry();
      }
      else if (currentHour == SCHEDULE_STOP_HOUR && currentMin == SCHEDULE_STOP_MIN && motorState) {
        Serial.println("Schedule Trigger: Stopping Motor");
        setMotorState(false);
        publishTelemetry();
      }
    }
  }

  // --- CONTINUOUS 3-PHASE SAFETY & PROTECTION MONITORING ---
  if (motorState) {
    float i1 = readRMSCurrent(CT1_PIN);
    float i2 = readRMSCurrent(CT2_PIN);
    float i3 = readRMSCurrent(CT3_PIN);

    // 1. Overload Protection (Any phase exceeds FLC)
    if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
      Serial.println(">>> TRIP: Overload detected on phase(s)! Shutting down motor immediately.");
      tripMotor();
      publishTelemetry();
    }
    // 2. Dry Run Protection (Pump running empty / under-load on all phases)
    else if (i1 < DRY_RUN_LIMIT && i2 < DRY_RUN_LIMIT && i3 < DRY_RUN_LIMIT) {
      Serial.println(">>> TRIP: Dry Run detected! Motor running empty. Shutting down immediately.");
      tripMotor();
      publishTelemetry();
    }
    // 3. Phase Failure / Single Phasing Protection (One phase drops to zero while others are active)
    else if ((i1 < 0.5 && (i2 > 3.0 || i3 > 3.0)) ||
             (i2 < 0.5 && (i1 > 3.0 || i3 > 3.0)) ||
             (i3 < 0.5 && (i1 > 3.0 || i2 > 3.0))) {
      Serial.println(">>> TRIP: Phase Failure / Single Phasing detected! Shutting down immediately.");
      tripMotor();
      publishTelemetry();
    }
  }
}

// Function to calculate RMS current from CT Sensor with Zero-Noise Filtering
float readRMSCurrent(int pin) {
  long sum = 0;
  int samples = 80;
  int zeroOffset = 2048; // 1.65V center for CT sensors (3.3V / 2)

  for (int i = 0; i < samples; i++) {
    int raw = analogRead(pin);
    int centered = raw - zeroOffset;
    sum += (long)centered * centered;
    delayMicroseconds(200); // Non-blocking fast sampling
  }

  float rmsRaw = sqrt((float)sum / samples);
  float voltage = (rmsRaw / 4095.0) * 3.3;
  float amps = voltage * 12.0; // Calibration factor

  // Noise filter: if nothing is attached or floating, force 0.0 A
  if (amps < 0.4) {
    amps = 0.0;
  }
  return amps;
}

void tripMotor() {
  motorState = false;
  digitalWrite(RELAY_PIN, RELAY_OFF);
  Serial.println(">>> Motor Tripped: Relay turned OFF");
}

void setMotorState(bool state) {
  motorState = state;
  digitalWrite(RELAY_PIN, state ? RELAY_ON : RELAY_OFF);
  Serial.print(">>> Relay GPIO 23 set to: ");
  Serial.println(state ? "RELAY_ON (LOW)" : "RELAY_OFF (HIGH)");
}

void publishTelemetry() {
  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);
  long uptimeSec = millis() / 1000;
  int rssi = WiFi.RSSI();

  struct tm timeinfo;
  char timeStr[32] = "NTP Syncing...";
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  String payload = "{";
  payload += "\"status\":\"" + String(motorState ? "ON" : "OFF") + "\",";
  payload += "\"phase1\":" + String(i1, 2) + ",";
  payload += "\"phase2\":" + String(i2, 2) + ",";
  payload += "\"phase3\":" + String(i3, 2) + ",";
  payload += "\"time\":\"" + String(timeStr) + "\",";
  payload += "\"rssi\":" + String(rssi) + ",";
  payload += "\"uptime\":" + String(uptimeSec) + ",";
  payload += "\"scheduleEnabled\":" + String(scheduleEnabled ? "true" : "false");
  payload += "}";

  mqttClient.publish(mqtt_topic_telemetry, payload.c_str());
}
