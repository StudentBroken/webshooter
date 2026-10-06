# 🕸️ Spider-Man Web-Shooter Combat & Servo Controller

Standalone PlatformIO firmware for an **ESP32-C3 SuperMini** controlling:
1. **Cinematic Extension/Retraction Mechanism**: Single servo motor with Iron Man missile pod kinematics (`0°` = Deployed, `90°` = Retracted).
2. **25W Electromagnet Firing System**: Controlled via a single-direction brushed motor controller / ESC on `GPIO 3` (clean boot, no power-on twitching).
3. **Tactical Arming Audio Engine**: Piezo/Active Buzzer on `GPIO 21` outputting a continuous cinematic armed pulse sequence.

---

## 📋 Hardware Setup & Pinout

| Peripheral | ESP32-C3 Pin | Description |
| :--- | :--- | :--- |
| **Extension Servo Signal** | `GPIO 10` | 14-Bit Native LEDC Timer (Channel 0, 50Hz) |
| **Electromagnet Brushed ESC** | `GPIO 3` | 50Hz RC Servo Pulses (Idle: 544µs / 0°, Fire: 2200µs) |
| **Cinematic Active Buzzer** | `GPIO 21` | Active Buzzer (Digital DC HIGH = Sound ON) |
| **Addressable RGB LED** | `GPIO 0` | WS2812 Single Pixel (Apple breathing / Firing red) |
| **Battery ADC** | `GPIO 4` | 100k/100k Divider Voltage Telemetry |
| **Local Webshooter Button** | `GPIO 9` | Onboard BOOT Button (`INPUT_PULLUP`, Active LOW) |
| **Remote Gimbal Button** | `GPIO 8` | Dedicated Gimbal Trigger Button (`INPUT_PULLUP`, Active LOW) |
| **Power (+5V & GND)** | `5V` / `GND` | Common power rail |

```
                       +------------------------+
                       |   ESP32-C3 SuperMini   |
                       |                        |
                       |       [ USB-C ]        |
                       |                        |
                       | GND                5V  |--- 5V Power Rail
                       | GPIO 0 (WS2812)   3V3 |
                       | GPIO 1            GND  |--- Common Ground
                       | GPIO 2          GPIO10 |--- Extension Servo Signal (GPIO 10)
                       | GPIO 3 (ESC)    GPIO9  |--- Local Webshooter BOOT Button
                       | GPIO 4 (BAT)    GPIO8  |--- Dedicated Gimbal Button (GPIO 8)
                       | GPIO 5          GPIO20 |
                       |                 GPIO21 |--- Buzzer (GPIO 21)
                       +------------------------+
```

---

## ⚡ Combat Operation & Button Logic (GPIO 9)

**Core Rule**:
* **OPEN (`0°`) = ARMED** (Jarvis targeting stance active)
* **CLOSED (`90°`) = DISARMED / STOWED** (Silent, safe)

---

### ⏱️ Button Timing Rules

#### 1. When CLOSED (`90°` / Stowed):
* **Press & Release (< 500ms)**:
  * Smoothly **OPENS mechanism to `0°`** (Iron Man deployment curve).
  * System is now **ARMED**: Mechanism opens silently into calm targeting stance (faint single beacon pip every 2.5s).

#### 2. When ARMED / OPEN (`0°`):
* **Press & Release (< 500ms)**:
  * **IMMEDIATELY DISARMS & FOLDS BACK to `90°`** (slower, deliberate 850ms motion)!
  * Plays 3 subtle, quiet micro-peeps (10ms pulses) distributed smoothly while retracting, beacon stops, LED turns OFF.
* **Press & Hold ($\ge 500\text{ms}$)**:
  * **FIRES the 25W Electromagnet on `GPIO 3`** (full 100% power / 2200µs for 1.5s)!
  * **Delayed Projectile Sync Sound**: Waits 500ms after coil energizes (compensating for electromagnet/latch lag) and plays an explosive coil discharge crack & magnetic ring-down (`CRACK-tk-tk`).
  * **Crucial**: After the 1.5s fire finishes, the mechanism **KEEPS OPEN at `0°`** to ensure the projectile has fully shot out with zero obstruction!

#### 3. Post-Fire (Mechanism Kept Open):
* **Press & Release (< 500ms)**:
  * **IMMEDIATELY DISARMS & FOLDS BACK to `90°`** (slower 850ms motion with 3 soft micro-peeps)!
  * Mechanism stows flush, ready for the next reload.

---

## 🎬 Cinematic Motion Engine (Iron Man Weapon Pod Curve)

Unlike standard abrupt RC servo jumps, the extension servo runs an accelerated trajectory generator:
* **Deploy Motion (`90°` $\to$ `0°`)**: Rapid launch with mechanical latch overshoot that cleanly settles at `0.0°`.
* **Retract Motion (`0°` $\to$ `90°`)**: Smooth, aerodynamic tuck back into `90.0°`.
* **Auto-Release**: Cuts servo PWM to 0V after 800ms of settling when retracted to eliminate hum, heat, and battery drain. (Maintains continuous holding torque when deployed).

---

## 💻 Serial Monitor Commands (115200 Baud)

Open the Serial Monitor and send any single-key command:
* **`a`**: Toggle Open/Armed <---> Closed/Disarmed
* **`b`** or **`t`**: Simulate button press (Servo transit only)
* **`0`**: Deploy to 0° (Armed)
* **`9`**: Retract to 90° (Disarmed)
* **`s`**: Run **Cinematic Showcase Cycle** (Deploy $\to$ Hold 1.2s $\to$ Retract)
* **`1`-`4`**: Switch motion profile (1=Iron Man, 2=SmootherStep, 3=Cubic, 4=Instant)
* **`o`**: Toggle Mechanical Overshoot on/off (`2.5°` $\leftrightarrow$ `0.0°`)
* **`+` / `-`**: Adjust duration in real-time ($\pm 50\text{ms}$)
* **`?`** or **`h`**: Print real-time status and telemetry
*(Note: Firing via serial is permanently disabled to prevent 25W USB brownout)*

---

## 🔨 Building & Flashing

From the `webshooter` directory:
```bash
# Compile firmware
pio run

# Flash to ESP32-C3
pio run --target upload

# Open Serial Monitor
pio device monitor
```

---

## 📡 Wireless ESP-NOW Communication (Master Mode)

The Webshooter acts as the **Master Controller** communicating directly to the **Shoulder Mounted Gimbal (Slave)** via ultra-low-latency ESP-NOW peer-to-peer Wi-Fi (Channel 1, broadcast `FF:FF:FF:FF:FF:FF`).

### 🎮 Dual-Button Controls:

#### **Button 1 (GPIO 9 - Local Webshooter)**:
* **Single Tap (< 400ms) when Closed**: Deploys webshooter to 0° (Armed).
* **Single Tap (< 400ms) when Armed**: Disarms & retracts webshooter to 90°.
* **Hold ($\ge 400\text{ms}$) when Armed**: Fires local 25W electromagnet.
* **Hold ($\ge 800\text{ms}$) when Closed**: Toggles stealth mode LED.

#### **Button 2 (GPIO 8 - Remote Gimbal Control)**:
* **When Gimbal is Disarmed**:
  * **Single Tap (< 400ms)**: Sends `deploy()` (`CMD_DEPLOY`) -> Arms and deploys shoulder gimbal.
  * **Long Press ($\ge 1000\text{ms}$)**: Sends `calibrate()` (`CMD_CALIBRATE`) -> Triggers full sweep & calibration.
* **When Gimbal is Armed**:
  * **Single Tap (< 400ms)**: Sends `disarm()` (`CMD_DISARM`) -> Retracts & disarms shoulder gimbal.
  * **Hold ($\ge 400\text{ms}$)**: Sends `fire()` (`CMD_FIRE`) -> Activates gimbal electromagnet continuous firing until released, with a **hard 2.0s safety cutoff** to protect coils from overheating.
  * **Release**: Sends `stop_fire()` (`CMD_STOP_FIRE`) -> Immediately turns off electromagnet.

