# 5 HP Smart Motor Starter (Full-Stack IoT System)

A complete IoT-enabled 3-phase motor starter and monitoring system powered by an **ESP32S**, 3 current sensors (CT sensors), a single-channel relay module, and a **Schneider LC1-D1810** AC contactor. 

This repository includes:
1. **ESP32 Firmware** (`SmartStarter.ino`)
2. **Node.js Cloud Backend** (`server.js` - Render ready)
3. **Web Frontend Dashboard** (`index.html` - Vercel ready)

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

## 🌐 Hosting Architecture

```text
 [ESP32 Smart Starter] ---> (HTTP POST /api/telemetry) ---> [ Node.js Backend on Render ]
          ^                                                              |
          |                                                       (HTTP GET /api/data)
  (HTTP GET /api/command)                                                |
          |                                                              v
 [ Vercel Frontend Dashboard ] <--- (User clicks START/STOP) --- [ Browser / Phone ]
```

### 1. Backend API (Render)
- Deploy this repository to **Render** as a Web Service.
- Render will automatically run `npm install` and `npm start` using `render.yaml`.
- Provides cloud endpoints:
  - `POST /api/telemetry` (ESP32 updates currents/status)
  - `GET /api/data` (Frontend fetches latest telemetry)
  - `POST /api/control` (Frontend queues Start/Stop command)
  - `GET /api/command` (ESP32 polls pending commands)

### 2. Frontend Dashboard (Vercel)
- Import this repository into **Vercel** as a static site.
- Point the frontend to your Render API URL.

---

## 🚀 Getting Started

1. **Upload Firmware**: Open `SmartStarter.ino` in the Arduino IDE, set your Wi-Fi credentials and your Render API URL, and flash it to your ESP32S.
2. **Deploy Backend**: Push to GitHub and deploy on Render.
3. **Deploy Frontend**: Deploy on Vercel and monitor/control your 5 HP motor from anywhere in the world!
