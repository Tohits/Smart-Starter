# 5 HP Smart Motor Starter (ESP32)

A smart IoT-enabled 3-phase motor starter and monitoring system powered by an **ESP32S**, 3 current sensors (CT sensors), a single-channel relay module, and a **Schneider LC1-D1810** AC contactor.

---

## 🛠️ Hardware Components

1. **Microcontroller**: ESP32S (ESP-WROOM-32)
2. **Current Sensors (x3)**: Non-invasive AC Current Transformers (e.g., SCT-013) on Phases 1, 2, and 3
3. **Switching Relay**: Single-Channel Relay Module (Optocoupler isolated)
4. **Contactor**: Schneider Electric TeSys D **LC1-D1810** (18A AC-3, 1NO auxiliary contact)
5. **Motor**: 3-Phase 5 HP Motor (~3.7 kW / ~7.5A Full Load Current)

---

## ⚡ Wiring Guide

| Component | Pin / Terminal | Connection |
| :--- | :--- | :--- |
| **Relay Module** | VCC / GND | ESP32 `5V` / `GND` |
| **Relay Module** | IN | ESP32 `GPIO 23` |
| **Contactor (LC1-D1810)** | Terminal A1 | Relay **NO** (Normally Open) |
| **Contactor (LC1-D1810)** | Terminal A2 | Neutral (230V AC) |
| **Relay COM** | Common | Live (230V AC Supply) |
| **CT Sensor 1** | Output | ESP32 `GPIO 34` (Analog ADC1) |
| **CT Sensor 2** | Output | ESP32 `GPIO 35` (Analog ADC1) |
| **CT Sensor 3** | Output | ESP32 `GPIO 32` (Analog ADC1) |

---

## 📡 Firmware Features

- **Wi-Fi Connectivity**: Connects to your local Wi-Fi network.
- **Real-Time 3-Phase Monitoring**: Samples analog pins and calculates RMS current across all 3 phases.
- **Overload & Dry-Run Protection**: Automatically trips (disengages relay) if phase current exceeds rated FLC (9.0A for 5 HP) or drops below dry-run threshold.
- **REST API Endpoints**:
  - `GET /data`: Returns JSON containing motor status (`ON`/`OFF`) and phase currents (`phase1`, `phase2`, `phase3`).
  - `POST /control?state=ON` or `OFF`: Remotely starts or stops the motor contactor.

---

## 🚀 Getting Started

1. Open `SmartStarter.ino` in the **Arduino IDE**.
2. Install the **ESP32 Board Package** and required libraries (`WebServer`).
3. Update your Wi-Fi credentials in `SmartStarter.ino`:
   ```cpp
   const char* ssid = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";
   ```
4. Compile and upload to your ESP32S.
5. Check the Arduino Serial Monitor for the assigned IP address and access `http://<ESP32_IP>/data`.
