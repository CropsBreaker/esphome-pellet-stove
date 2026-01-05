# ESP32 Pellet Stove Controller

## Project Overview
This project implements a hybrid control system for a pellet stove using an ESP32. The system leverages a **PID Controller** to manage water temperature by modulating the pellet feed through a **Smart Bucket** algorithm.

**Control Hybridization:** The stove is not fully controlled by the ESP. A secondary **PLC** handles the high-voltage sequencing (exhaust vent and igniter/sparkplug) and dictates the master "ON" state via the `physical_sw` binary sensor.



## System Architecture
* **Safety & Power Layer (PLC):** Managed via 220V logic. When active, it powers the vent and the igniter (for approx. 10 min) and closes the ESP32 clean contact.
* **Control Layer (ESP32):** Reads temperature data and performs the PID calculation to modulate fuel delivery (auger).

## Hardware Components & Rationale

* **MCU: ESP32-WROOM (4MB Flash):** *
    * *Rationale:* The code is heavy (approx. 900 KB). 4MB is required to support **OTA (Over-The-Air)** updates comfortably by allowing enough space for the twin-partition switch.
* **Exhaust Sensor: K-type Thermocouple + MAX6675:**
    * *Rationale:* Ranges well for high temperatures and is very cheap compared to industrial probes.
* **Auger Actuator: Solid State Relay (SSR-40DA):**
    * *Rationale:* Preferred over a classic mechanical relay because the auger requires constant pulse-width modulation. An SSR is needed to last for the high number of "clicks" without mechanical wear.
* **PLC Interface: APKLVSR 3V Relay:**
    * *Rationale:* Used to bridge the ESP32 logic to the PLC for web-based remote start.
* **Water Sensor: Ds18B20:**

## GPIO Pinout Table

| Component | ESP32 Pin | Protocol / Mode | Description |
|:---|:---:|:---:|:---|
| **MAX6675 CLK** | GPIO 18 | SPI | Clock for Thermocouple |
| **MAX6675 MISO** | GPIO 19 | SPI | Data from Thermocouple |
| **MAX6675 CS** | GPIO 21 | SPI | Chip Select for Smoke Sensor |
| **Ds18B20 Data** | GPIO 4 | 1-Wire | Water Temperature Sensor Bus |
| **Auger (Coclea)** | GPIO 26 | Output (SSR) | Pulse control for pellet feed |
| **PLC Command** | GPIO 33 | Output (Relay) | Web-triggered start for PLC |
| **Physical Sw** | GPIO 16 | Input (PD) | Input status from external PLC |
| **LED OK** | GPIO 27 | Output | RUN mode active |
| **LED Warning** | GPIO 13 | Output | Boot/Startup sequence |
| **LED Error** | GPIO 14 | Output | Critical Safety Error |

## Control Logic Details

### PID & Smart Bucket
The $K_p$, $K_i$, and $K_d$ parameters are currently generic as the system must adapt to different water volumes. To bridge the gap between continuous PID logic and discrete hardware, we use a **Smart Bucket**:
1.  **Accumulation:** The PID output is integrated over time into a virtual "bucket".
2.  **Actuation:** Once the bucket reaches **0.8s** (minimum safe time for motor/relay), it triggers a pulse.
3.  **Efficiency:** This prevents the motor from jittering and ensures fuel delivery is proportional to the calculated need.



## Future Roadmap
- [ ] **Translation System:** Externalize status strings via JSON or C++ Map.
- [ ] **Specific Documentation:** Inline Doxygen-style comments for the PID loop.
- [ ] **PID Calibration:** Data logging for different water volume presets.
