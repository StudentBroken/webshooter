#pragma once
#include <Arduino.h>

// ==============================================================================
// Spider-Man Web-Shooter Controller - Configuration
// Board: ESP32-C3 SuperMini
// ==============================================================================

// ------------------------------------------------------------------------------
// Pin Assignments (ESP32-C3 SuperMini)
// ------------------------------------------------------------------------------
#define PIN_BATTERY_ADC 4   // GPIO 4: Battery Voltage monitoring (ADC1_CHANNEL_4, 100k-100k divider)   // GPIO 2: Battery Voltage monitoring (100k-100k divider)
#define BATTERY_DIVIDER_RATIO 2.0f
#define TELEMETRY_BROADCAST_MS 300 // ESP-NOW status ping interval

// ------------------------------------------------------------------------------
// Addressable WS2812 RGB LED (GPIO 0, Single Pixel)
// ------------------------------------------------------------------------------
#define PIN_RGB_LED          0   // GPIO 0: Single WS2812 Addressable RGB LED Pixel
#define RGB_MIN_BRIGHTNESS   12  // Floor brightness (approx 5%): never drops to 0%, always glowing
#define RGB_MAX_BRIGHTNESS   220 // Maximum brightness clamp (0-255)
#define RGB_BREATH_PERIOD_MS 3600 // 3.6s Apple-style breathing cycle (16.6 breaths/min)

#define PIN_SERVO 10 // GPIO 10: Extension Servo PWM Signal (LEDC Channel 0)
#define PIN_FIRE_ESC                                                           \
  3 // GPIO 3:  25W Electromagnet Brushed ESC (LEDC Channel 1, Clean Boot - No
    // Pullup)
#define PIN_BUZZER                                                             \
  21 // GPIO 21: Active Buzzer (DC HIGH = Sound ON, DC LOW = Silent)
#define BUZZER_ARMED_BEACON_MS                                                 \
  2500 // Faint slow beacon pip interval while armed (2.5s)
#define BUZZER_BEACON_PULSE_MS                                                 \
  8 // Subtle, soft micro-pip pulse duration (8ms = less loud)
#define BUZZER_FIRE_SNAP_MS                                                    \
  20 // Instant discharge pop pulse duration on fire (20ms)

// ------------------------------------------------------------------------------
// Dual Button Assignments (ESP32-C3 SuperMini)
// ------------------------------------------------------------------------------
#define PIN_BUTTON                  9       // GPIO 9:  Local Webshooter BOOT Button (Active LOW)
#define PIN_BUTTON_GIMBAL           8       // GPIO 8:  Dedicated Shoulder Mounted Gimbal Button (Active LOW)
#define BUTTON_ACTIVE_LEVEL         LOW     // LOW when pressed / grounded
#define BUTTON_DEBOUNCE_MS          35      // Debounce window (ms)
#define BUTTON_FIRE_HOLD_MS         400     // Snappy Hold >= 400ms to fire (reliable tap window for retract/disarm)
#define DISARMED_LED_HOLD_MS        800     // Hold >= 800ms while disarmed to toggle stealth mode
#define GIMBAL_LONG_PRESS_MS        1000    // Hold >= 1000ms while disarmed to calibrate
#define GIMBAL_MAX_FIRE_MS          2000    // Max 2.0s safety cutoff; fires while held until released

// ------------------------------------------------------------------------------
// Servo Geometry & Motion Limits (0° = Deployed / Fired, 90° = Retracted /
// Reloaded)
// ------------------------------------------------------------------------------
#define SERVO_POS_DEPLOYED_ANGLE 0 // Position B (Fired / Deployed):   0 Degrees
#define SERVO_POS_RETRACTED_ANGLE                                              \
  90 // Position A (Default / Retracted): 90 Degrees

#define SERVO_POS_A_ANGLE SERVO_POS_RETRACTED_ANGLE // 90° (Boot Home)
#define SERVO_POS_B_ANGLE SERVO_POS_DEPLOYED_ANGLE  // 0°  (Fired)

#define SERVO_MIN_PULSE_US 544  // Microseconds for 0 degrees
#define SERVO_MAX_PULSE_US 2400 // Microseconds for 180 degrees

// ------------------------------------------------------------------------------
// Cinematic Motion Profiling (Iron Man Shoulder Missile Pod Dynamics)
// ------------------------------------------------------------------------------
// Motion Profiles:
//   1 = PROFILE_IRON_MAN     : Cinematic punch, fast mid-stroke, mechanical
//   overshoot & latch on deploy, smooth retract. 2 = PROFILE_SMOOTHERSTEP : Ken
//   Perlin 5th-order polynomial (zero overshoot, buttery fluid motion). 3 =
//   PROFILE_CUBIC        : Classical S-curve cubic ease-in-out. 4 =
//   PROFILE_INSTANT      : Instant snap (raw servo motor speed).
#define PROFILE_IRON_MAN 1
#define PROFILE_SMOOTHERSTEP 2
#define PROFILE_CUBIC 3
#define PROFILE_INSTANT 4

#define DEFAULT_SERVO_PROFILE PROFILE_IRON_MAN

// Frame Refresh Rate: Synchronized with 50Hz RC servo PWM period (20ms)
// Crucial: Rate-limiting updates to 20ms prevents LEDC hardware register
// flooding and eliminates choppiness/jitter!
#define SERVO_FRAME_INTERVAL_MS 20 // 20ms = 50Hz exact servo frame

// Pacing & Durations (in milliseconds)
// Deploy: Smooth, deliberate extension (not too fast, cinematic weapon pod
// deployment) Retract: Controlled, clean tuck back flush into housing
#define SERVO_DEPLOY_TIME_MS 450  // Smooth, controlled extension (not too fast)
#define SERVO_RETRACT_TIME_MS 450 // Slower, deliberate retraction

// Mechanical Latch Overshoot (Degrees):
// Simulates the physical inertial recoil and pneumatic latch of Iron Man's
// missile pod. Deploys slightly past target (e.g. to 92.5°) then crisply
// settles back to 90.0°. Set to 0.0f if your mechanism has a rigid 90° physical
// stop.
#define SERVO_DEPLOY_OVERSHOOT_DEG 2.5f

// Auto-Release PWM after movement (Extension Servo ONLY):
// Cuts PWM to flat 0V only when RETRACTED (90°) to prevent motor hum, coil
// heating, and battery drain. When DEPLOYED (0°), servo maintains continuous
// active holding torque so the mechanism stays rigid!
#define SERVO_AUTO_RELEASE_MS                                                  \
  800 // Milliseconds of holding torque after retracting (0 = always on)

// ------------------------------------------------------------------------------
// 25W Electromagnet Firing System (GPIO 3 Single-Direction Brushed ESC)
// ------------------------------------------------------------------------------
// This ESC operates on standard 50Hz RC servo pulses:
//   Idle:   544us (0 degrees / servo.write(0)) -> ESC is armed at 0% throttle,
//   0V out Firing: 2200us (Full throttle / servo.write(180)) -> ESC outputs
//   full 25W power
// Note: Must remain actively clocked at 50Hz (never cut to flat 0V) so ESC
// never loses signal!
#define FIRE_LEDC_CHANNEL 1  // Dedicated LEDC Timer Channel 1
#define FIRE_PWM_IDLE_US 544 // 0% Throttle / 0 Degrees (ESC Armed & Safe)
#define FIRE_PWM_ACTIVE_US       2200 // 100% Throttle (Full 25W saturation power to electromagnet)
#define FIRE_PULSE_DURATION_MS   8000 // 8.0s maximum safety cutoff; stays energized while held until release
#define FIRE_MAX_SAFETY_MS       8000 // 8.0s maximum safety limit
#define FIRE_SOUND_DELAY_MS                                                    \
  500 // 500ms delay after electromagnet energizes before shot sound (projectile
      // sync)
#define AUTO_DISARM_AFTER_FIRE                                                 \
  false // Keep open after firing until user presses button one more time
#define AUTO_RETRACT_AFTER_FIRE                                                \
  false // Keep open after firing to ensure projectile clears safely

// ------------------------------------------------------------------------------
// Serial Telemetry
// ------------------------------------------------------------------------------
#define SERIAL_BAUD_RATE 115200
