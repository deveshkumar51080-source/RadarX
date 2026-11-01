# R.A.D.A.R-X (Robotic Adaptive Detection and Response System)

![Project Status](https://img.shields.io/badge/Status-Active-success)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Framework](https://img.shields.io/badge/Framework-Arduino-cyan)

R.A.D.A.R-X is an intelligent, gesture-driven robotic platform powered by an ESP32 microcontroller. It merges multi-modal sensor fusion with deterministic safety overrides, allowing contactless hand-gesture navigation while autonomously preventing collisions and identifying stationary human presence in real-time.

## 🧠 Core Features
* **Contactless Gesture Navigation:** Decodes hand motions (directional swipes) using the APDS9960 sensor via I2C to dictate real-time motor maneuvers.
* **Multi-Modal Spatial Awareness:** Fuses 24GHz mmWave radar (HLK-LD2410), laser Time-of-Flight LiDAR (VL53L0X), ultrasonic ranging (HC-SR04), and infrared proximity to eliminate sensor blind spots.
* **Autonomous Safety Override:** Implements a three-tier risk management architecture (**Safe** / **Warning** / **Danger**). Firmware overrides user input and disconnects motor power via a relay kill-switch if obstructions breach the <15cm threshold.
* **Adaptive Actuation:** Regulates dual DC motors through an L298N H-Bridge using pulse-width modulation (PWM) for speed calibration.

## 🛠️ Hardware Requirements
* **Microcontroller:** ESP32 (38-Pin Dev Module)
* **Sensors:**
  * APDS9960 (Gesture/Color/Proximity)
  * HLK-LD2410 (24GHz mmWave Radar)
  * VL53L0X (ToF LiDAR)
  * HC-SR04 (Ultrasonic)
  * Digital IR Proximity Sensors
* **Actuators & Power:**
  * L298N Motor Driver + 2x DC BO Motors
  * LM2596 Buck Converter (12V to 5V Step-Down)
  * 5V Relay Module (Hardware Kill-Switch)
  * 3x LEDs (Red/Yellow/Green) + Piezo Buzzer

## 📦 Software Dependencies
Install the following libraries via the Arduino IDE Library Manager:
* `Adafruit_APDS9960` by Adafruit
* `Adafruit_VL53L0X` by Adafruit
* `NewPing` by Tim Eckel
* `ld2410` by Nick Reynolds

## ⚙️ Pin Configuration (ESP32)
| Component | Function | ESP32 GPIO |
| :--- | :--- | :--- |
| **I2C Bus** | SDA / SCL | 21 / 22 |
| **L298N** | ENA, IN1, IN2 | 12, 13, 14 |
| **L298N** | ENB, IN3, IN4 | 27, 26, 25 |
| **Ultrasonic** | TRIG / ECHO | 5 / 18 |
| **mmWave** | Digital OUT | 19 |
| **IR Sensors** | NEAR / FAR | 33 / 34 |
| **Safety** | Relay (Kill-Switch) | 16 |
| **Indicators** | Red / Yellow / Green | 17 / 32 / 4 |
| **Indicators**| Buzzer | 23 |

## 🚀 Installation & Usage
1. Clone this repository.
2. Wire the hardware according to the pin configuration table above. *Note: Ensure all grounds (ESP32, Sensors, L298N, Battery) are tied together.*
3. Open `RADAR_X.ino` in the Arduino IDE.
4. Select `ESP32 Dev Module` from the Boards menu and compile/upload.
5. Open Serial Monitor at `115200` baud to monitor sensor fusion states in real-time.
