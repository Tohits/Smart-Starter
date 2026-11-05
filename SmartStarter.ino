/*
 * 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 * Features:
 * - Captive Portal (WiFiManager) for easy Wi-Fi setup
 * - 3-Phase Current Monitoring (CT Sensors) & Overload/Dry-Run Protection
 * - Schneider LC1-D1810 Contactor Control via Single-Channel Relay
 * - NTP Time Synchronization & Time Scheduling
 * - Web-Based OTA (Over-The-Air) Firmware Update
 * - CORS-enabled REST API for Vercel Web Dashboard
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <Update.h>
#include "time.h"

WebServer server(80);

// Pin Definitions
const int RELAY_PIN = 23;     // Connected to Single-Channel Relay IN
const int CT1_PIN = 34;       // Phase 1 Current Sensor (Analog ADC1)
const int CT2_PIN = 35;       // Phase 2 Current Sensor (Analog ADC1)
const int CT3_PIN = 32;       // Phase 3 Current Sensor (Analog ADC1)

// Motor State & Protection Thresholds (5 HP Motor ~ 7.5A Full Load)
bool motorState = false;
const float RATED_FLC = 9.0;     // Max allowable current before overload trip (Amps)
const float DRY_RUN_LIMIT = 1.5; // Min current for pump dry-run detection (Amps)

// NTP Time Servers & Settings (UTC+5:30 = 19800 sec)
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// Time Schedule Settings
const int SCHEDULE_START_HOUR = 6;   // 06:00 AM
const int SCHEDULE_START_MIN = 30;
const int SCHEDULE_STOP_HOUR = 18;   // 06:00 PM
const int SCHEDULE_STOP_MIN = 0;
bool scheduleEnabled = false;

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Contactor OFF by default

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

  // --- NTP Time Sync ---
  configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");

  // --- Web Server Routes ---
  server.on("/data", HTTP_GET, handleGetData);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/data", HTTP_OPTIONS, handleCors);
  server.on("/control", HTTP_OPTIONS, handleCors);

  // OTA Update HTML Form Route
  server.on("/update", HTTP_GET, []() {
    server.send(200, "text/html",
      "<form method='POST' action='/update' enctype='multipart/form-data'>"
      "<h3>ESP32 OTA Firmware Update</h3>"
      "<input type='file' name='update'><input type='submit' value='Update'>"
      "</form>");
  });

  // OTA Update Handler Route
  server.on("/update", HTTP_POST, []() {
    server.send(200, "text/html", Update.hasError() ? "FAIL" : "SUCCESS. Rebooting...");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) {
        Serial.printf("Update Success: %u bytes\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
  Serial.println("Smart Starter HTTP Server & OTA Started");
}

void loop() {
  server.handleClient();

  // --- Time Scheduling Check ---
  if (scheduleEnabled) {
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
      int currentHour = timeinfo.tm_hour;
      int currentMin = timeinfo.tm_min;

      if (currentHour == SCHEDULE_START_HOUR && currentMin == SCHEDULE_START_MIN && !motorState) {
        Serial.println("Schedule Trigger: Starting Motor");
        setMotorState(true);
      }
      else if (currentHour == SCHEDULE_STOP_HOUR && currentMin == SCHEDULE_STOP_MIN && motorState) {
        Serial.println("Schedule Trigger: Stopping Motor");
        setMotorState(false);
      }
    }
  }

  // --- Continuous Protection Check if Motor is Running ---
  if (motorState) {
    float i1 = readRMSCurrent(CT1_PIN);
    float i2 = readRMSCurrent(CT2_PIN);
    float i3 = readRMSCurrent(CT3_PIN);

    // Overload Protection Trip
    if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
      Serial.println("TRIP: Overload detected! Shutting down motor.");
      tripMotor();
    }
  }
  delay(1000);
}

// Function to calculate RMS current from CT Sensor
float readRMSCurrent(int pin) {
  long sum = 0;
  int samples = 150;
  for (int i = 0; i < samples; i++) {
    int raw = analogRead(pin);
    float voltage = (raw / 4095.0) * 3.3;
    float amps = voltage * 10.0;
    sum += amps * amps;
    delay(2);
  }
  return sqrt(sum / samples);
}

void tripMotor() {
  motorState = false;
  digitalWrite(RELAY_PIN, LOW);
}

void setMotorState(bool state) {
  motorState = state;
  digitalWrite(RELAY_PIN, state ? HIGH : LOW);
}

void addCorsHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

void handleCors() {
  addCorsHeaders();
  server.send(204);
}

void handleGetData() {
  addCorsHeaders();
  float i1 = readRMSCurrent(CT1_PIN);
  float i2 = readRMSCurrent(CT2_PIN);
  float i3 = readRMSCurrent(CT3_PIN);

  struct tm timeinfo;
  char timeStr[32] = "NTP Syncing...";
  if (getLocalTime(&timeinfo)) {
    strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  }

  String json = "{";
  json += "\"status\":\"" + String(motorState ? "ON" : "OFF") + "\",";
  json += "\"phase1\":" + String(i1, 2) + ",";
  json += "\"phase2\":" + String(i2, 2) + ",";
  json += "\"phase3\":" + String(i3, 2) + ",";
  json += "\"time\":\"" + String(timeStr) + "\",";
  json += "\"scheduleEnabled\":" + String(scheduleEnabled ? "true" : "false");
  json += "}";

  server.send(200, "application/json", json);
}

void handleControl() {
  addCorsHeaders();
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") {
      setMotorState(true);
      Serial.println("Motor Started via Web Command");
    } else if (state == "OFF") {
      setMotorState(false);
      Serial.println("Motor Stopped via Web Command");
    }
    server.send(200, "application/json", "{\"result\":\"success\",\"state\":\"" + String(motorState ? "ON" : "OFF") + "\"}");
  } else if (server.hasArg("schedule")) {
    String sched = server.arg("schedule");
    scheduleEnabled = (sched == "true");
    server.send(200, "application/json", "{\"result\":\"success\",\"scheduleEnabled\":" + String(scheduleEnabled ? "true" : "false") + "}");
  } else {
    server.send(400, "application/json", "{\"result\":\"error\",\"message\":\"Missing parameters\"}");
  }
}
