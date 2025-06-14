# 🌞 EpEver Solar Monitor

Monitor your EPEVER solar charge controller via ESP8266, MQTT, and Home Assistant – **fully local and all sunshine,** no vendor cloud required.**


![image](https://github.com/user-attachments/assets/5a68283e-1a5d-4f7e-b3dd-24524c0ed6d9)






> 🔗 [Buy the Charge Controller](https://amzn.eu/d/51auldm)
> 🔗 [Buy the DS485Max Controller](https://www.amazon.co.uk/dp/B07B667STP)


Monitor your off-grid solar system in real-time with a WiFi-enabled, Arduino-powered NodeMCU ESP8266. This firmware speaks fluent **Modbus over RS485**, publishes readings via **MQTT**, and auto-discovers in **Home Assistant** - no cloud, no fuss, just data.


---

## ⚡️ Highlights

✅ Polls Modbus registers from EPEVER MPPT via RS485 (MAX485)

✅ Sends clean JSON over MQTT

✅ Works out-of-the-box with Home Assistant MQTT Discovery

✅ OTA firmware updates

✅ Optional Deep Sleep for ultra-low power

---

## 🧠 How It Works

The ESP8266 queries your EPEVER solar charge controller via RS485 using the Modbus protocol. It extracts real-time data like voltages, amps, SOC, and temperatures - then sends it wirelessly over MQTT to your broker (e.g., Mosquitto). If you’re running Home Assistant, it auto-discovers and configures the entities.

No YAML, no cables, no vendor cloud.

---

## 🪛 Hardware Wiring

Use an RJ45/Ethernet cable to neatly bridge the EPEVER RS485 port, MAX485 breakout board, and your NodeMCU ESP8266.

### Ethernet Cable Mapping (T568B standard)

```
Green/White  → RS485MAX B
Blue/White   → RS485MAX A
Brown/White  → NodeMCU GND
Orange       → NodeMCU VIN 5v
```

### Full Connection Table

| Device        | Pin           | Connects To       |
|---------------|---------------|-------------------|
| EPEVER        | A (+)         | MAX485 A          |
|               | B (−)         | MAX485 B          |
|               | GND           | MAX485 GND        |
| MAX485        | RO            | NodeMCU RX (D7)   |
|               | DI            | NodeMCU TX (D6)   |
|               | RE            | NodeMCU D1        |
|               | DE            | NodeMCU D2        |
|               | VCC           | NodeMCU 3.3V      |
| NodeMCU       | GND           | Common Ground     |

> 💡 **Pro tip**: Use stranded Cat5/6 for flexibility and reduced noise.

---

## 📦 Dependencies

Install these via Arduino Library Manager:

- `ModbusMaster` by Doc Walker
- `PubSubClient` by Nick O'Leary
- `ESP8266WiFi` (built-in)
- `ArduinoOTA` (built-in)
- `SoftwareSerial`

---
## 🔧 Prerequisites

- Arduino IDE with ESP8266 board support installed
- USB-to-MicroUSB cable for uploading firmware
- Local MQTT broker (e.g. Mosquitto)
- Basic knowledge of Home Assistant (for integration)
---
## 🧰 Getting Started

1. Clone the repo and open `EpEverSolarMonitor.ino`
2. Copy `secrets_template.h` → `secrets.h` and fill in:
```cpp
const char* hostname    = "espSolarChargerOTA";
const char* mqtt_user   = "your_user";
const char* mqtt_pass   = "your_pass";
const char* mqtt_server = "192.168.1.x";
const int   mqtt_port   = 1883;
etc
```
3. Upload the sketch via USB
4. Device connects to WiFi, starts publishing via MQTT, and auto-registers in Home Assistant

---

## 📈 Published Metrics

These show up in MQTT as JSON:

- `pv_volt`, `pv_amps`, `pv_power`
- `batt_volt`, `batt_amps`, `batt_power`, `batt_charge`, `batt_soc`, `batt_temp`
- `load_volt`, `load_amps`, `load_power`

**Topic**: `epever/state`

---

## 🏠 Home Assistant Integration

When `#define ENABLE_DISCOVERY` is set:
- Entities auto-register under `homeassistant/sensor/epever_*`
- Includes correct `device_class`, `unit_of_measurement`, and `unique_id`
- No YAML required

---

## 🛌 Deep Sleep Support

Low power deployment? Wire **GPIO16 (D0)** to **RST** to enable wake-from-deep-sleep.

Control sleep via MQTT:
- Topic: `mortylabs/solar/deepsleep`
- Payload: `1`

---

## 📸 Screenshot

![image](https://github.com/user-attachments/assets/605d44dd-83b6-4912-8ff0-1fb569996b49)


---

MIT License - Made with ☀️ by MortyLabs

[![MIT License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/platform-ESP8266-blue.svg)](https://arduino-esp8266.readthedocs.io/)
[![Made with Arduino](https://img.shields.io/badge/made%20with-Arduino-00979D.svg)](https://www.arduino.cc/)

