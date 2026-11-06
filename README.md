# ⚙️ Smart Motor Starter (ESP32 IoT Solution)

An industrial, IoT-enabled 3-phase motor starter, energy meter, and protection system powered by an **ESP32S**, 3 CT current sensors, a single-channel optocoupler relay module, and a **Schneider LC1-D1810** AC contactor. It features 100% global cloud control via **HiveMQ Cloud MQTT** and a Progressive Web App (PWA) dashboard deployed on **Vercel**.

---

## 💡 Working Principle

1. **User Action**: The user sends a `START` command through the mobile PWA application dashboard.
2. **Wireless Reception**: The ESP32 receives the encrypted command via Wi-Fi over secure MQTT WebSockets (`HiveMQ Cloud`).
3. **Safety Health Check**: The ESP32 instantly evaluates protection conditions (3-phase voltages, currents, and motor temperature).
4. **Isolated Contactor Control**: If all conditions are healthy, the ESP32 operates an optically isolated control interface (Relay on Pin P5 / GPIO 5).
5. **Motor Operation**: The relay energizes the coil (`A1`/`A2`) of the **Schneider LC1-D1810** contactor, switching ON 3-phase power to the motor.
6. **Continuous Parametric Sensing**: CT sensors and temperature probes continuously monitor motor current, voltages, running hours, and energy consumption ($kWh$).
7. **Automated Protection & Alerting**: If a fault occurs (overload, dry-run, phase failure, or thermal over-temp), the protection circuit immediately trips the starter relay and dispatches native mobile push notifications to the user's phone.

---

## 🛠️ Hardware Components

1. **Microcontroller**: ESP32S (ESP-WROOM-32 with Hardware Watchdog Timer)
2. **Current Sensors (x3)**: Non-invasive AC Current Transformers (SCT-013) on Phases 1 (R), 2 (Y), and 3 (B)
3. **Switching Relay**: Single-Channel 5V Relay Module (Optocoupler isolated, Active LOW)
4. **Power Contactor**: Schneider Electric TeSys D **LC1-D1810** (18A AC-3 rating)
5. **Motor**: 3-Phase 5 HP Motor (~3.7 kW / ~7.5A Full Load Current)

---

## ⚡ Hardware Wiring Guide

| Component | Pin / Terminal | Connection |
| :--- | :--- | :--- |
| **Relay Module** | VCC / GND | ESP32 `5V` / `GND` |
| **Relay Module** | IN | ESP32 `GPIO 5` (Pin P5) |
| **Contactor (LC1-D1810)** | Terminal A1 | Relay **NO** (Normally Open) |
| **Contactor (LC1-D1810)** | Terminal A2 | Neutral (230V AC) |
| **Relay COM** | Common | Live (230V AC Supply) |
| **CT Sensor 1 (Phase R)** | Signal | ESP32 `GPIO 34` (Analog ADC1) |
| **CT Sensor 2 (Phase Y)** | Signal | ESP32 `GPIO 35` (Analog ADC1) |
| **CT Sensor 3 (Phase B)** | Signal | ESP32 `GPIO 32` (Analog ADC1) |
| **Temperature Sensor** | Signal | ESP32 `GPIO 33` (Analog ADC1) |

---

## 🌐 PWA Web Frontend Dashboard (`index.html`)

Features a responsive Tailwind CSS PWA dashboard deployed on **Vercel** with:
- **Global MQTT Control**: 🟢 **START** & 🔴 **STOP** motor from anywhere in the world.
- **Real-Time Telemetry**: 3-Phase Current ($A$), Voltage ($V$), Motor Temp ($°C$), Energy ($kWh$), and Running Hours.
- **Auto-Start Mode**: Automatically restarts the motor when grid power & Wi-Fi return if enabled.
- **Native Mobile Push Notifications**: Triggers native system notifications and vibration alerts on phone for fault trips.
- **Auto Day/Night Mode**: Automatic theme adjustment based on daylight hours or manual toggle.
- **Single-Screen Non-Scrolling Viewport**: Tailored to fit mobile and desktop screens compactly.

---

## 🚀 Getting Started

1. Open `SmartStarter.ino` in the **Arduino IDE**.
2. Install required libraries via **Tools > Manage Libraries**:
   - `WiFiManager` (by tzapu)
   - `PubSubClient` (by Nick O'Leary)
3. Upload `SmartStarter.ino` to your ESP32S.
4. Connect to the `SmartStarter-AP` Wi-Fi hotspot on your phone to set your home/farm Wi-Fi credentials.
5. Deploy `index.html` to **Vercel** or access your live Vercel URL!
