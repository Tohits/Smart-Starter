/*
 * 5 HP 3-Phase Smart Motor Starter Firmware for ESP32S
 * Components:
 * - ESP32S Microcontroller
 * - 3x Current Sensors (CT Sensors on Phase 1, Phase 2, Phase 3)
 * - 1x Single-Channel Relay Module (connected to Contactor Coil A1)
 * - 1x Schneider LC1-D1810 AC Contactor (18A AC-3)
 */

#include <WiFi.h>
#include <WebServer.h>

// Replace with your Wi-Fi credentials
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

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

void setup() {
  Serial.begin(115200);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Contactor OFF by default

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWi-Fi Connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // Web Server Routes & CORS Handlers
  server.on("/data", HTTP_GET, handleGetData);
  server.on("/control", HTTP_POST, handleControl);
  server.on("/data", HTTP_OPTIONS, handleCors);
  server.on("/control", HTTP_OPTIONS, handleCors);

  server.begin();
  Serial.println("Smart Starter HTTP Server Started");
}

void loop() {
  server.handleClient();

  // Continuous Protection Check if Motor is Running
  if (motorState) {
    float i1 = readRMSCurrent(CT1_PIN);
    float i2 = readRMSCurrent(CT2_PIN);
    float i3 = readRMSCurrent(CT3_PIN);

    // Overload or Dry Run Protection Trip
    if (i1 > RATED_FLC || i2 > RATED_FLC || i3 > RATED_FLC) {
      Serial.println("TRIP: Overload detected! Shutting down motor.");
      tripMotor();
    } else if (i1 < DRY_RUN_LIMIT && i2 < DRY_RUN_LIMIT && i3 < DRY_RUN_LIMIT) {
      // Optional dry-run check
      // Serial.println("TRIP: Dry Run detected! Shutting down motor.");
      // tripMotor();
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
    // Convert ADC reading to voltage (ESP32 12-bit ADC = 4095, 3.3V ref)
    float voltage = (raw / 4095.0) * 3.3;
    // Calibration factor depending on your CT sensor & burden resistor
    float amps = voltage * 10.0;
    sum += amps * amps;
    delay(2);
  }
  float rms = sqrt(sum / samples);
  return rms;
}

void tripMotor() {
  motorState = false;
  digitalWrite(RELAY_PIN, LOW); // De-energize relay -> release contactor
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

  String json = "{";
  json += "\"status\":\"" + String(motorState ? "ON" : "OFF") + "\",";
  json += "\"phase1\":" + String(i1, 2) + ",";
  json += "\"phase2\":" + String(i2, 2) + ",";
  json += "\"phase3\":" + String(i3, 2);
  json += "}";

  server.send(200, "application/json", json);
}

void handleControl() {
  addCorsHeaders();
  if (server.hasArg("state")) {
    String state = server.arg("state");
    if (state == "ON") {
      motorState = true;
      digitalWrite(RELAY_PIN, HIGH); // Energize relay -> pull in contactor LC1-D1810
      Serial.println("Motor Started via Web Command");
    } else if (state == "OFF") {
      motorState = false;
      digitalWrite(RELAY_PIN, LOW);  // Drop contactor
      Serial.println("Motor Stopped via Web Command");
    }
    server.send(200, "application/json", "{\"result\":\"success\",\"state\":\"" + String(motorState ? "ON" : "OFF") + "\"}");
  } else {
    server.send(400, "application/json", "{\"result\":\"error\",\"message\":\"Missing state parameter\"}");
  }
}
// End of file
