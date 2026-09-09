# Queyk IoT — Seismic Edge Detection & Telemetry System

[![Platform](https://img.shields.io/badge/Platform-ESP32%20%7C%20WisBlock-blue.svg)](https://docs.rakwireless.com/)
[![Sensor](https://img.shields.io/badge/Sensor-Omron%20D7S%20%2F%20RAK12027-orange.svg)](https://store.rakwireless.com/products/rak12027-d7s-seismic-sensor)
[![Framework](https://img.shields.io/badge/Framework-Arduino-00979C.svg)](https://www.arduino.cc/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](file:///home/luis/dev/projects/QUEYK/queyk-iot/LICENSE)

An open-source Arduino firmware for ESP32 and RAKwireless WisBlock hardware designed for real-time seismic sensing, edge magnitude calculation, local buzzer alerts, and HTTP cloud telemetry reporting.

---

## 1. Overview & Key Capabilities

**Queyk IoT** is an IoT edge client that interfaces with an Omron D7S seismic sensor (via the RAK12027 module) to monitor Spectral Intensity (SI) and Peak Ground Acceleration (PGA). It handles on-device earthquake detection, triggers a local acoustic buzzer alarm, and dispatches HTTP alerts and periodic 5-minute rolling telemetry over Wi-Fi.

### Key Capabilities

- **Real-Time Seismic Sensing**: Samples Spectral Intensity (SI) and Peak Ground Acceleration (PGA) from the D7S sensor over I2C (`0x55`).
- **Edge Magnitude Calculation**: Converts Spectral Intensity into Richter-equivalent seismic magnitude ($M$) using an empirical piecewise model.
- **Multi-Channel Alert Dispatching**:
  - **Local Buzzer Alarm**: Plays an alternating dual-pitch alarm sequence on `WB_IO5` upon detecting SI $\ge 0.5$, followed by a clearance tone after hold time.
  - **Immediate Webhook / Email Trigger**: Fires an HTTP POST request to `emailURL` immediately for events $\ge M2.0$.
  - **Incident Summary Dispatch**: Posts measured magnitude and duration to `earthquakeURL` once the seismic event clears.
- **Periodic 5-Minute Telemetry**: Calculates rolling statistical summaries (Average, Minimum, and Maximum SI, simulated battery level, Wi-Fi RSSI) and posts JSON data to `serverURL`.
- **Built-in Fault Tolerance & Recovery**:
  - Power cycling routine via `WB_IO2` during boot and reconnect.
  - Multi-speed I2C initialization ($100\,\text{kHz}$, $400\,\text{kHz}$, $50\,\text{kHz}$).
  - Automatic consecutive invalid reading tracking with 60-second reinitialization attempts.
  - Hourly buzzer health check beeps.

---

## 2. Architecture / How it Works

The firmware executes an event evaluation loop balancing real-time seismic detection, acoustic alarm management, and periodic telemetry reporting.

```mermaid
flowchart TD
    subgraph Initialization ["Startup & Hardware Setup"]
        A[Device Startup] --> B[I2C Bus Scan & Init]
        B --> C[Power Cycle Sensor via WB_IO2]
        C --> D[Multi-Speed I2C Handshake 100/400/50 kHz]
        D --> E[Buzzer Diagnostic Test]
        E --> F[Connect Wi-Fi]
    end

    subgraph MainLoop ["Main Execution Loop (~3s cycle)"]
        F --> G{Sensor Ready?}
        G -- No --> H[Retry Reconnection Every 60s]
        H --> G
        G -- Yes --> I[Read SI & PGA from D7S]
        I --> J{Valid Reading?}
        J -- No --> K[Increment Invalid Count]
        K --> L{Exceeded 5 Counts?}
        L -- Yes --> M[Mark Sensor Not Ready]
        L -- No --> N[Wait Next Loop]
        J -- Yes --> O[Push to 100-Sample Circular Buffer]
    end

    subgraph EarthquakePipeline ["Seismic Event Pipeline"]
        O --> P{SI >= 0.5 & Not Active?}
        P -- Yes --> Q[Trigger Earthquake Alert State]
        Q --> R[Convert SI to Richter Magnitude]
        Q --> S[Activate Local Buzzer Alarm Pattern]
        Q --> T{Magnitude >= 2.0?}
        T -- Yes --> U[POST Emergency Alert to Email URL]
        T -- No --> V[Log Minor Event]
        P -- No --> W{Earthquake Active?}
        W -- Yes --> X{Hold Duration >= 5s?}
        X -- Yes --> Y[Sound Ascending Success Tone]
        Y --> Z[POST Event Magnitude & Duration to Earthquake URL]
        Z --> AA[Reset Event State]
    end

    subgraph TelemetryPipeline ["5-Minute Telemetry Pipeline"]
        O --> AB{Interval >= 5 min?}
        AB -- Yes --> AC[Calculate Rolling Avg, Min, Max SI]
        AC --> AD[Compile Battery Level & RSSI]
        AD --> AE[POST Payload to Backend Ingestion URL]
        AE --> AF[Flush Buffer & Reset Window Timer]
    end
```

### Empirical Magnitude Conversion Model

The firmware maps Spectral Intensity ($SI$ in $\text{m/s}$ or $\text{kine}$) to estimated local earthquake magnitude ($M$) according to the following piecewise model:

$$ \text{Magnitude}(SI) = \begin{cases}
1.0 + (SI \times 2.0) & SI \le 0.5 \\
2.0 + ((SI - 0.5) \times 1.0) & 0.5 < SI \le 1.5 \\
3.0 + \left(\frac{SI - 1.5}{1.5}\right) & 1.5 < SI \le 3.0 \\
4.0 + \left(\frac{SI - 3.0}{3.0}\right) & 3.0 < SI \le 6.0 \\
5.0 + \left(\frac{SI - 6.0}{4.0}\right) & SI > 6.0
\end{cases}$$

---

## 3. Tech Stack

- **Target Architecture**: Espressif ESP32 Core / RAKwireless WisBlock Core (e.g., RAK11200 / ESP32)
- **Primary Language**: C++ (Arduino Core)
- **Sensors & Peripherals**:
  - [Omron D7S](https://components.omron.com/us-en/products/sensors/D7S) / [RAK12027](https://store.rakwireless.com/products/rak12027-d7s-seismic-sensor) Seismic Sensor (I2C address: `0x55`)
  - Piezo Buzzer (`WB_IO5`)
  - Sensor Power Control Pin (`WB_IO2`)
- **Core Dependencies**:
  - [`RAK12027_D7S.h`](https://github.com/RAKWireless/RAK12027-D7S): Hardware driver for Omron D7S
  - [`WiFi.h`](https://github.com/espressif/arduino-esp32/tree/master/libraries/WiFi): ESP32 Wi-Fi station library
  - [`HTTPClient.h`](https://github.com/espressif/arduino-esp32/tree/master/libraries/HTTPClient): HTTP client library with Bearer token authentication
  - [`Wire.h`](https://github.com/espressif/arduino-esp32/tree/master/libraries/Wire): I2C communication library

---

## 4. Project Structure

```
queyk-iot/
├── LICENSE              # MIT License
├── README.md            # Project technical documentation
└── iot.ino              # Main Arduino firmware source code
```

### Module Descriptions

- [`iot.ino`](file:///home/luis/dev/projects/QUEYK/queyk-iot/iot.ino):
  - **Configuration & Constants**: Network credentials, API endpoints, auth token headers, and pin configurations.
  - **I2C Scanner & Power Management**: Bus scanning (`scanI2C()`) and sensor power cycling (`WB_IO2`).
  - **Sensor Engine**: Multi-frequency I2C initialization, status polling (`D7S.isReady()`), and SI/PGA reading acquisition.
  - **Statistical Buffer**: Circular buffer storing up to 100 historical readings for rolling average, min, and max computations.
  - **Alerting & Actuation**: Piezo buzzer tone routines (`soundEarthquakeAlarm()`, `soundSuccessTone()`) and HTTP POST alert dispatchers.

---

## 5. Getting Started

### Prerequisites

1. **Hardware**:
   - ESP32 development board or RAKwireless WisBlock Base + Core (RAK5005-O + RAK11200 / RAK4631).
   - RAK12027 (Omron D7S) seismic sensor module mounted flat and level.
   - Buzzer connected to `WB_IO5` (or configured GPIO).
2. **Software**:
   - [Arduino IDE](https://www.arduino.cc/en/software) (v2.0+) or [PlatformIO](https://platformio.org/).
   - ESP32 Board Package installed in Arduino Board Manager.
   - `RAK12027_D7S` library installed via Library Manager.

### Configuration Variables

Open [`iot.ino`](file:///home/luis/dev/projects/QUEYK/queyk-iot/iot.ino) and configure your Wi-Fi and backend settings:

| Variable | Type | Description | Default / Example |
| :--- | :--- | :--- | :--- |
| `ssid` | `const char[]` | Wi-Fi SSID | `"WIFI_SSID"` |
| `pass` | `const char[]` | Wi-Fi password | `"WIFI_PASSWORD"` |
| `serverURL` | `const char*` | HTTP endpoint for 5-minute statistical telemetry | `"https://api.example.com/v1/telemetry"` |
| `emailURL` | `const char*` | HTTP webhook for immediate email alerts | `"https://api.example.com/v1/alerts/email"` |
| `earthquakeURL` | `const char*` | HTTP endpoint for earthquake completion reports | `"https://api.example.com/v1/alerts/earthquake"` |
| `authToken` | `const char*` | Bearer auth token for HTTP headers | `"AUTH_TOKEN"` |
| `tokenType` | `const char*` | Header identifier for token type | `"Bearer"` / `"TOKEN_TYPE"` |
| `BUZZER_PIN` | `const int` | Buzzer GPIO output pin | `WB_IO5` |

### Flashing via Arduino IDE

1. Connect the ESP32 / WisBlock board via USB.
2. Select your board in **Tools > Board** (e.g., *ESP32 Dev Module* or *WisBlock Core*).
3. Select the serial port under **Tools > Port**.
4. Click **Upload**.
5. Open **Tools > Serial Monitor** at `115200` baud.

---

## 6. Usage & Telemetry Reference

### Serial Diagnostics Output

When booted, the board outputs startup diagnostics and real-time status:

```text
🔄 === DEVICE STARTUP ===
Board: ESP32-based device
🔧 Initializing I2C...
🔍 Scanning I2C bus...
I2C device found at address 0x55
✅ Found 1 I2C device(s)
⚡ Configuring power control...
🔊 Configuring buzzer...
🔊 Testing buzzer at startup...
🔊 Buzzer test complete
Power cycle 1/3
...
Connecting to WiFi: MyHomeNetwork
✅ WiFi connected: 192.168.1.145
🔧 Trying different I2C configurations...
Trying I2C speed: 100000 Hz
🔧 Attempting D7S initialization...
✅ D7S responds to I2C
✅ D7S.begin() successful
Waiting for sensor ready...
✅ Sensor READY!
✅ Sensor initialized at 100000 Hz
======================

📊 Normal | SI: 0.002 | Magnitude: M1.0 | Rolling Avg: 0.002 | Min: 0.000 | Max: 0.002 | Source: REAL | Email: READY
```

### JSON Payloads

#### 1. Periodic 5-Minute Telemetry (`POST serverURL`)
```json
{
  "siAverage": 0.014,
  "siMinimum": 0.000,
  "siMaximum": 0.082,
  "battery": 99.7,
  "signalStrength": "-64"
}
```

#### 2. Immediate Earthquake Trigger (`POST emailURL`)
```json
{
  "magnitude": 3.4
}
```

#### 3. Event Completion Report (`POST earthquakeURL`)
```json
{
  "magnitude": 3.4,
  "duration": 6
}
```

---

## 7. Troubleshooting & Error Handling

| Issue / Symptom | Potential Cause | Remediation Step |
| :--- | :--- | :--- |
| `❌ No response from D7S at address 0x55` | Sensor unseated or power rail off | Check that `WB_IO2` power control is driven HIGH and the module is seated securely in the I2C socket. |
| `❌ Sensor TIMEOUT` | Sensor not stable during startup | Mount the sensor on a flat, vibration-free surface during the startup calibration phase (`D7S.initialize()`). |
| `⚠️ Invalid sensor reading #N` | I2C line noise or loose jumper wires | The firmware attempts recovery after 5 consecutive invalid readings. Try lowering I2C clock speed to `50000 Hz`. |
| `📱 Offline - data not sent` | Wi-Fi disconnected | Verify SSID/password and ensure 2.4 GHz network availability. The device retries transmission on subsequent loops. |
| `⚠️ API Error 401 / 403` | Unauthorized endpoint access | Verify `authToken` and `tokenType` values in the configuration header. |
| Buzzer silent during test | Pin mismatch | Check if `BUZZER_PIN` matches the connected pin on your specific board revision. |

---

## License

This project is open-source software licensed under the [MIT License](file:///home/luis/dev/projects/QUEYK/queyk-iot/LICENSE).
$$
