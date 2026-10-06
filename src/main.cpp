#include "config.h"
#include <esp32-hal-rmt.h>
#include "esp_now_protocol.h"
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <Preferences.h>

Preferences prefs;
bool disarmedLedEnabled = true;

// ==============================================================================
// Hardware Native ESP32 LEDC PWM Definitions
// ==============================================================================
#define SERVO_LEDC_CHANNEL 0 // Channel 0: Extension Servo (GPIO 10)
#define FIRE_LEDC_CHANNEL 1  // Channel 1: Electromagnet ESC (GPIO 3)
#define LEDC_FREQ 50         // 50Hz (20ms RC standard period)
#define LEDC_RESOLUTION 14   // 14-bit resolution: 0 - 16383 counts

// ==============================================================================
// State Variables & Motion Engine
// ==============================================================================
// Angles: Webshooter starts retracted at 90°
float currentAngle = SERVO_POS_RETRACTED_ANGLE;
float startAngle = SERVO_POS_RETRACTED_ANGLE;
float targetAngle = SERVO_POS_RETRACTED_ANGLE;

// Motion Timing
bool isMoving = false;
unsigned long motionStartTime = 0;
unsigned long currentMotionDuration = SERVO_DEPLOY_TIME_MS;
unsigned long deployDurationMs = SERVO_DEPLOY_TIME_MS;
unsigned long retractDurationMs = SERVO_RETRACT_TIME_MS;
unsigned long lastMotionEndTime = 0;
unsigned long lastServoUpdateMs = 0; // 50Hz frame rate limiter

// Retraction Audio (3 short micro-peeps while moving)
int retractBeepStage = 0;
unsigned long retractBeepPulseEndTime = 0;

// Profile & Tuning Settings
int activeProfile = DEFAULT_SERVO_PROFILE;
float currentOvershootDeg = SERVO_DEPLOY_OVERSHOOT_DEG;

// Power Management (Servo only)
bool pwmActive = false;
bool autoReleaseEnabled = (SERVO_AUTO_RELEASE_MS > 0);
uint32_t lastServoDuty = 0xFFFFFFFF;

// ------------------------------------------------------------------------------
// Firing System State (GPIO 3 25W Electromagnet Brushed ESC)
// ------------------------------------------------------------------------------
bool isArmed = false;
bool isFiring = false;
bool hasFired = false;
bool fireSoundPlayed = false;
unsigned long fireStartTime = 0;
uint32_t fireIdlePulseUs =
    FIRE_PWM_IDLE_US; // 544us default (0 degrees / 0% throttle)
uint32_t fireActivePulseUs =
    FIRE_PWM_ACTIVE_US; // 2200us default (full throttle / 100% power)

// ------------------------------------------------------------------------------
// Button State Machine: Local Webshooter (GPIO 9 - Active LOW)
// ------------------------------------------------------------------------------
bool lastButtonRaw = HIGH;
bool buttonIsPressed = false;
unsigned long lastDebounceTime = 0;
unsigned long buttonPressStartTime = 0;
bool holdActionHandled = false;

// ------------------------------------------------------------------------------
// Gimbal Button State Machine: Dedicated Gimbal (GPIO 8 - Active LOW) & ESP-NOW
// ------------------------------------------------------------------------------
bool lastGimbalButtonRaw = HIGH;
bool gimbalButtonIsPressed = false;
unsigned long lastGimbalDebounceTime = 0;
unsigned long gimbalPressStartTime = 0;
bool gimbalHoldHandled = false;

bool gimbalArmed = false;
bool gimbalZeroed = false;
bool gimbalFiring = false;
unsigned long gimbalFireStartTime = 0;
unsigned long lastGimbalKeepAliveMs = 0;
uint32_t gimbalSequence = 0;
unsigned long lastTelemetryBroadcastMs = 0;
float webshooterVbatFilteredMv = 0.0f;

void initEspNowMaster();
bool sendGimbalCommand(GimbalCommand cmd, uint16_t duration_ms = 0);
void processGimbalButton();

// ------------------------------------------------------------------------------
// Cinematic Audio Engine (GPIO 21 Active Buzzer - Pure Digital DC)
// ------------------------------------------------------------------------------
unsigned long lastArmedBeepCycleMs = 0;
int armedBeepStep = 0;

// Non-blocking Demo Showcase ('s' command)
enum DemoState { DEMO_OFF, DEMO_DEPLOYING, DEMO_HOLDING, DEMO_RETRACTING };
DemoState demoState = DEMO_OFF;
unsigned long demoHoldStartTime = 0;

// ==============================================================================
// Function Prototypes
// ==============================================================================
void initHardware();
void writeServoPulseDirect(float angle);
void releaseServoSignal();
void writeFirePulseUs(uint32_t pulse_us);
void fireProjectile();

void startServoMotion(float newTargetAngle);
void updateServoMotion();
void toggleServoPosition();

void processButton();
void updateArmedBeeper();
void updateRetractAudio();
void playFireSound();
void setBuzzer(bool on);

void handleSerialCommands();
void printHelpAndStatus();
void setLed(bool on);
void ws2812WritePixel(uint8_t r, uint8_t g, uint8_t b);
void triggerRgbFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t duration_ms);
void updateRgbLed();

float calculateEasedT(float t, bool deploying);
float smoothStep(float t);
float cubicEaseInOut(float t);
float smootherStep(float t);

// ==============================================================================
// Apple-Style WS2812 RGB LED Engine (GPIO 0, Single Pixel)
// Non-linear perceptual eye smoothness via CIE 1931 lightness curve + exp(sin(t))
// ==============================================================================
const uint8_t cie1931_table[256] PROGMEM = {
    0,   0,   0,   0,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,
    2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   4,
    4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   6,   6,   6,   6,   6,   7,
    7,   7,   7,   8,   8,   8,   8,   9,   9,   9,  10,  10,  10,  10,  11,  11,
   11,  12,  12,  12,  13,  13,  13,  14,  14,  15,  15,  15,  16,  16,  17,  17,
   17,  18,  18,  19,  19,  20,  20,  21,  21,  22,  22,  23,  23,  24,  24,  25,
   25,  26,  26,  27,  28,  28,  29,  29,  30,  31,  31,  32,  32,  33,  34,  34,
   35,  36,  37,  37,  38,  39,  39,  40,  41,  42,  43,  43,  44,  45,  46,  47,
   47,  48,  49,  50,  51,  52,  53,  54,  54,  55,  56,  57,  58,  59,  60,  61,
   62,  63,  64,  65,  66,  67,  68,  70,  71,  72,  73,  74,  75,  76,  77,  79,
   80,  81,  82,  83,  85,  86,  87,  88,  90,  91,  92,  94,  95,  96,  98,  99,
  100, 102, 103, 105, 106, 108, 109, 110, 112, 113, 115, 116, 118, 120, 121, 123,
  124, 126, 128, 129, 131, 132, 134, 136, 138, 139, 141, 143, 145, 146, 148, 150,
  152, 154, 155, 157, 159, 161, 163, 165, 167, 169, 171, 173, 175, 177, 179, 181,
  183, 185, 187, 189, 191, 193, 196, 198, 200, 202, 204, 207, 209, 211, 214, 216,
  218, 220, 223, 225, 228, 230, 232, 235, 237, 240, 242, 245, 247, 250, 252, 255
};

unsigned long rgbFlashEndTime = 0;
uint8_t flashR = 0, flashG = 0, flashB = 0;

void triggerRgbFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t duration_ms) {
  flashR = r;
  flashG = g;
  flashB = b;
  rgbFlashEndTime = millis() + duration_ms;
}

void ws2812WritePixel(uint8_t r, uint8_t g, uint8_t b) {
  rmt_data_t led_data[24];
  static rmt_obj_t* rmt_send = NULL;
  static bool initialized = false;

  if (!initialized) {
    rmt_send = rmtInit(PIN_RGB_LED, RMT_TX_MODE, RMT_MEM_64);
    if (rmt_send == NULL) return;
    rmtSetTick(rmt_send, 100);
    initialized = true;
  }

  uint8_t colors[3] = {r, b, g}; // Hardware RBG diode format: Byte 0=R, Byte 1=B, Byte 2=G
  int idx = 0;
  for (int p = 0; p < 3; p++) {
    uint8_t val = colors[p];
    for (int bit = 7; bit >= 0; bit--) {
      if (val & (1 << bit)) {
        led_data[idx].level0 = 1;
        led_data[idx].duration0 = 8; // 0.8us
        led_data[idx].level1 = 0;
        led_data[idx].duration1 = 4; // 0.4us
      } else {
        led_data[idx].level0 = 1;
        led_data[idx].duration0 = 4; // 0.4us
        led_data[idx].level1 = 0;
        led_data[idx].duration1 = 8; // 0.8us
      }
      idx++;
    }
  }
  rmtWriteBlocking(rmt_send, led_data, 24);
}

void updateRgbLed() {
  unsigned long now = millis();

  // Rate-limit ALL WS2812 updates to 50Hz (every 20ms)
  // This guarantees the WS2812 data line stays LOW for ~20ms between frames,
  // providing the necessary >50us reset latch pulse for all modes including firing and flash!
  static unsigned long lastLedFrameMs = 0;
  if (now - lastLedFrameMs < 20) {
    return;
  }
  lastLedFrameMs = now;

  // 1. Temporary high-priority visual flash
  if (now < rgbFlashEndTime) {
    ws2812WritePixel(flashR, flashG, flashB);
    return;
  }

  // 2. Firing mode: Solid bright intense TRUE RED immediately while local webshooter is firing!
  if (isFiring) {
    ws2812WritePixel(255, 0, 0); // Solid True Red
    return;
  }

  // 3. Apple Sleep Breathing Waveform calculation:
  //    f(x) = (exp(sin(phase)) - 1/e) * 255 / (e - 1/e)
  //    phase = 2*PI * (now % period) / period - PI/2
  float phase = (2.0f * (float)PI * (float)(now % RGB_BREATH_PERIOD_MS) / (float)RGB_BREATH_PERIOD_MS) - (0.5f * (float)PI);
  float rawNorm = (expf(sinf(phase)) - 0.36787944f) / 2.3504024f; // Normalized 0.0 to 1.0
  rawNorm = constrain(rawNorm, 0.0f, 1.0f);

  uint8_t index = (uint8_t)(rawNorm * 255.0f);
  uint8_t correctedBrightness = pgm_read_byte(&cie1931_table[index]);

  // Scale between minimum floor (never drops to 0%, always glowing) and maximum brightness
  uint8_t level = RGB_MIN_BRIGHTNESS + (uint8_t)(((uint32_t)correctedBrightness * (RGB_MAX_BRIGHTNESS - RGB_MIN_BRIGHTNESS)) / 255);

  // 4. Color states:
  //    - Deployed / Armed (0°): Tactical Breathing BLUE (always active when deployed!)
  //    - Retracted / Disarmed (90°): Standby Breathing WHITE if enabled, else OFF (Stealth mode)
  if (isArmed || targetAngle == SERVO_POS_DEPLOYED_ANGLE) {
    ws2812WritePixel(0, 0, level); // Breathing Blue (always on when deployed!)
  } else {
    if (disarmedLedEnabled) {
      ws2812WritePixel(level, level, level); // Breathing White
    } else {
      ws2812WritePixel(0, 0, 0); // Stealth Mode: LED off while disarmed
    }
  }
}

// ==============================================================================
// Setup
// ==============================================================================
void setup() {
  // 1. CRITICAL: Immediately initialize Hardware Peripherals at millisecond 0
  // so GPIO 3 is sending continuous 50Hz 544us (0% throttle) before ANY delay!
  initHardware();

  // Load persistent user preference for disarmed LED (default: ON)
  prefs.begin("webshooter", false);
  disarmedLedEnabled = prefs.getBool("disLed", true);

  // Initialize WS2812 RGB LED (starts soft white at boot)
  ws2812WritePixel(40, 40, 40);

  // 2. Pin Configurations
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_GIMBAL, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  setBuzzer(false);

  // 3. Start USB Serial
  Serial.begin(SERIAL_BAUD_RATE);

  // 3b. Initialize ESP-NOW Master Broadcast
  initEspNowMaster();

  // 3c. Battery ADC Config (GPIO 4, 11dB attenuation)
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db);

  // 4. Cold-boot Home sequence: direct lock to Retracted position (90°)
  writeServoPulseDirect(SERVO_POS_RETRACTED_ANGLE);
  currentAngle = SERVO_POS_RETRACTED_ANGLE;
  startAngle = SERVO_POS_RETRACTED_ANGLE;
  targetAngle = SERVO_POS_RETRACTED_ANGLE;
  isMoving = false;
  lastMotionEndTime = millis();

  delay(600); // Allow physical servo movement to complete on cold start

  if (autoReleaseEnabled) {
    releaseServoSignal();
  }

  printHelpAndStatus();
}

// ==============================================================================
// Main Loop
// ==============================================================================
void loop() {
  // 1. Process serial commands
  handleSerialCommands();

  // 2. Process physical buttons:
  //    GPIO 9: Local Webshooter BOOT Button
  //    GPIO 8: Dedicated Shoulder Mounted Gimbal Button
  processButton();
  processGimbalButton();

  // 3. Update continuous non-blocking motion trajectory (50Hz = 20ms)
  unsigned long now = millis();
  if (now - lastServoUpdateMs >= SERVO_FRAME_INTERVAL_MS) {
    lastServoUpdateMs = now;
    updateServoMotion();
  }

  // 3b. Non-blocking audio cadence while retracting (3 short, soft micro-peeps)
  updateRetractAudio();

  // 4. Update 25W Electromagnet Firing Timer
  if (isFiring) {
    // Delayed cinematic shot sound (synchronized with projectile launch at
    // 500ms)
    if (!fireSoundPlayed && (now - fireStartTime >= FIRE_SOUND_DELAY_MS)) {
      fireSoundPlayed = true;
      playFireSound();
    }

    // Hard safety cutoff (8.0s) to prevent coil overheating if held too long or button stuck
    if (now - fireStartTime >= FIRE_MAX_SAFETY_MS) {
      writeFirePulseUs(fireIdlePulseUs); // Return to 544us (0% throttle)
      isFiring = false;
      setBuzzer(false);
      Serial.println(
          "\n============================================================");
      Serial.printf("  [FIRE] Electromagnet pulse completed (%luus idle).\n",
                    fireIdlePulseUs);
      Serial.println("  [STANCE] Mechanism KEPT OPEN (0°) to ensure projectile "
                     "has fully shot out.");
      Serial.println(
          "           Press button one more time to FOLD BACK (90°) & DISARM.");
      Serial.println(
          "============================================================");
      setLed(true); // Keep LED on while open
    }
  }

  // 5. Update Cinematic Armed Active Buzzer & LED Strobe Sequence
  updateArmedBeeper();

  // 5b. Update Apple-Style WS2812 RGB LED Engine (GPIO 0)
  updateRgbLed();

  // 6. Update Demo Showcase Cycle if active
  if (demoState != DEMO_OFF) {
    if (demoState == DEMO_DEPLOYING && !isMoving) {
      demoHoldStartTime = millis();
      demoState = DEMO_HOLDING;
      Serial.printf(
          "[DEMO] Deployed (%d°)! Holding firing position for 1.2s...\n",
          SERVO_POS_DEPLOYED_ANGLE);
    } else if (demoState == DEMO_HOLDING) {
      if (millis() - demoHoldStartTime >= 1200) {
        Serial.printf("[DEMO] Retracting mechanism smoothly to %d°...\n",
                      SERVO_POS_RETRACTED_ANGLE);
        startServoMotion(SERVO_POS_RETRACTED_ANGLE);
        setLed(false);
        demoState = DEMO_RETRACTING;
      }
    } else if (demoState == DEMO_RETRACTING && !isMoving) {
      demoState = DEMO_OFF;
      Serial.println("[DEMO] Cinematic cycle finished successfully!");
    }
  }

  // 7. Handle Auto-Release timeout (Extension Servo ONLY - cuts PWM only when
  // RETRACTED) When DEPLOYED (0°), keep active holding torque constantly
  // engaged so the barrel stays rigid!
  if (autoReleaseEnabled && pwmActive && !isMoving) {
    if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
      if (millis() - lastMotionEndTime >= SERVO_AUTO_RELEASE_MS) {
        releaseServoSignal();
        Serial.println("[POWER] Servo auto-release: PWM cut to 0V (retracted "
                       "idle, 0 heat/hum).");
      }
    }
  }

  // 8. Periodic Telemetry Broadcast to Gimbal Pod (every 300ms)
  if (now - lastTelemetryBroadcastMs >= TELEMETRY_BROADCAST_MS) {
    lastTelemetryBroadcastMs = now;
    uint32_t rawMv = (uint32_t)(analogReadMilliVolts(PIN_BATTERY_ADC) * BATTERY_DIVIDER_RATIO);
    if (webshooterVbatFilteredMv < 100.0f) {
      webshooterVbatFilteredMv = rawMv;
    } else {
      webshooterVbatFilteredMv = webshooterVbatFilteredMv * 0.85f + (float)rawMv * 0.15f;
    }

    WebshooterTelemetry telem;
    telem.command = CMD_TELEMETRY;
    telem.vbat_mv = (uint16_t)webshooterVbatFilteredMv;
    telem.webshooter_pos = (currentAngle <= 20.0f) ? 0 : 1; // 0 = Deployed, 1 = Retracted
    telem.gimbal_armed = gimbalArmed ? 1 : 0;
    telem.gimbal_firing = gimbalFiring ? 1 : 0;
    telem.breath_phase_ms = (uint16_t)(millis() % RGB_BREATH_PERIOD_MS);
    telem.sequence = ++gimbalSequence;

    uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_send(broadcastAddr, (uint8_t *)&telem, sizeof(telem));
  }

  // Rate-limit loop for servo stability & RTOS idle breathing
  delay(10);
}

// ==============================================================================
// ==============================================================================
// Hardware Native PWM Setup (LEDC)
// ==============================================================================
void initHardware() {
  // 1. Extension Servo on Channel 0 (GPIO 10)
  ledcSetup(SERVO_LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
  ledcAttachPin(PIN_SERVO, SERVO_LEDC_CHANNEL);

  // 2. 25W Electromagnet Brushed ESC on Channel 1 (GPIO 3)
  // Ensure unhold, cleanly reset pin from any boot/safety holds
  gpio_hold_dis((gpio_num_t)PIN_FIRE_ESC);
  gpio_reset_pin((gpio_num_t)PIN_FIRE_ESC);

  ledcSetup(FIRE_LEDC_CHANNEL, LEDC_FREQ, LEDC_RESOLUTION);
  writeFirePulseUs(fireIdlePulseUs); // Pre-load 544us 0% throttle pulse to arm ESC
  ledcAttachPin(PIN_FIRE_ESC, FIRE_LEDC_CHANNEL);
  writeFirePulseUs(fireIdlePulseUs);

  Serial.printf(
      "[ESC] Initialized GPIO %d at 50Hz %luus (0 degrees / 0%% throttle armed).\n",
      PIN_FIRE_ESC, (unsigned long)fireIdlePulseUs);
}
void writeServoPulseDirect(float angle) {
  angle = constrain(angle, -5.0f, 185.0f);
  float pulse_us =
      (float)SERVO_MIN_PULSE_US +
      (angle / 180.0f) * (float)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US);
  pulse_us = constrain(pulse_us, 500.0f, 2500.0f);

  uint32_t duty = (uint32_t)(((uint64_t)pulse_us * 16384ULL) / 20000ULL);

  if (duty != lastServoDuty) {
    ledcWrite(SERVO_LEDC_CHANNEL, duty);
    lastServoDuty = duty;
  }
  pwmActive = true;
}

void releaseServoSignal() {
  ledcWrite(SERVO_LEDC_CHANNEL, 0);
  lastServoDuty = 0;
  pwmActive = false;
}

void writeFirePulseUs(uint32_t pulse_us) {
  pulse_us = constrain(pulse_us, 400, 2500);
  uint32_t duty = (uint32_t)(((uint64_t)pulse_us * 16384ULL) / 20000ULL);
  ledcWrite(FIRE_LEDC_CHANNEL, duty);
}

// ==============================================================================
// Firing Action (25W Electromagnet via GPIO 3 Brushed ESC)
// ==============================================================================
void fireProjectile() {
  if (isFiring) return;
  isFiring = true;
  hasFired = true;
  fireSoundPlayed = false;
  fireStartTime = millis();
  writeFirePulseUs(fireActivePulseUs); // 2200us (full throttle / 25W full power)
  Serial.println("\n============================================================");
  Serial.printf("  [FIRE] 25W Electromagnet ENERGIZED (%luus full power)!\n", (unsigned long)fireActivePulseUs);
  Serial.printf("  Duration: %d ms | Discharge sound sync at: %d ms\n", FIRE_PULSE_DURATION_MS, FIRE_SOUND_DELAY_MS);
  Serial.println("============================================================");
}

// ==============================================================================
// Cinematic Active Buzzer Engine (GPIO 21 - Pure Digital DC)
//   - Stance: Calm, faint single beacon pip every 2.5s while aiming
//   - Fire:   500ms delayed coilgun discharge crack & magnetic ring-down
//   - Deploy & Retract: Completely silent (pure mechanical servo motion)
// ==============================================================================
void setBuzzer(bool on) { digitalWrite(PIN_BUZZER, on ? HIGH : LOW); }

void updateRetractAudio() {
  // Turn off active peep pulse when its micro-duration expires
  if (retractBeepPulseEndTime > 0 && millis() >= retractBeepPulseEndTime) {
    if (!isFiring)
      setBuzzer(false);
    retractBeepPulseEndTime = 0;
  }

  if (!isMoving || targetAngle != SERVO_POS_RETRACTED_ANGLE) {
    return;
  }

  unsigned long elapsed = millis() - motionStartTime;
  float progress = (float)elapsed / (float)currentMotionDuration;

  // 2 short, soft micro-peeps while retracting (50% and 100%):
  // Beep 1: at 50% progress (mid-stroke)
  if (retractBeepStage == 0 && progress >= 0.25f) {
    retractBeepStage = 1;
    setBuzzer(true);
    retractBeepPulseEndTime = millis() + 10;
  }
}

void playFireSound() {
  // Cinematic Coilgun Discharge & Magnetic Ring-Down:
  // (Triggered at 500ms delay to sync with physical projectile launch)
  // 1. Primary explosive kinetic crack
  setBuzzer(true);
  delay(18);
  setBuzzer(false);
  delay(10);
  // 2. High-power magnetic coil ring-down decay
  setBuzzer(true);
  delay(6);
  setBuzzer(false);
  delay(8);
  setBuzzer(true);
  delay(4);
  setBuzzer(false);
}

void updateArmedBeeper() {
  // Only beep when open, armed, not firing, and before shooting
  if (!isArmed || isFiring || hasFired) {
    if (armedBeepStep != 0) {
      if (!isFiring)
        setBuzzer(false);
      armedBeepStep = 0;
    }
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - lastArmedBeepCycleMs;

  // Jarvis Targeting Stance: faint, slow single beacon pip every 2500ms (2.5s)
  // 0ms:  Crisp micro-pip ON (12ms)
  // 12ms: Silent targeting stance (2488ms calm focus while aiming)
  if (elapsed >= BUZZER_ARMED_BEACON_MS) {
    lastArmedBeepCycleMs = now;
    armedBeepStep = 1;
    setBuzzer(true);
  } else if (elapsed >= BUZZER_BEACON_PULSE_MS && armedBeepStep == 1) {
    armedBeepStep = 0;
    setBuzzer(false);
  }
}

// ==============================================================================
// Cinematic Motion Engine (Extension Servo)
// ==============================================================================
void startServoMotion(float newTargetAngle) {
  newTargetAngle = constrain(newTargetAngle, -5.0f, 185.0f);

  if (!isMoving && fabs(currentAngle - newTargetAngle) < 0.2f) {
    return;
  }

  startAngle = currentAngle;
  targetAngle = newTargetAngle;
  bool deploying = (targetAngle == SERVO_POS_DEPLOYED_ANGLE);

  currentMotionDuration = deploying ? deployDurationMs : retractDurationMs;
  motionStartTime = millis();
  isMoving = true;
  pwmActive = true;
  retractBeepStage = 0;
  retractBeepPulseEndTime = 0;

  if (activeProfile == PROFILE_INSTANT) {
    currentAngle = targetAngle;
    writeServoPulseDirect(currentAngle);
    isMoving = false;
    lastMotionEndTime = millis();
    Serial.printf("[MOTION] Instant snap to: %.1f°\n", currentAngle);
  }
}

void updateServoMotion() {
  if (!isMoving)
    return;

  unsigned long elapsed = millis() - motionStartTime;
  float t = (float)elapsed / (float)currentMotionDuration;

  if (t >= 1.0f) {
    currentAngle = targetAngle;
    writeServoPulseDirect(currentAngle);
    isMoving = false;
    lastMotionEndTime = millis(); // Auto-release countdown starts NOW
    Serial.printf("[MOTION] Destination reached: %.1f°\n", currentAngle);

    // Beep 2: at 100% progress (destination reached / locked at 90° stowed
    // position)
    if (targetAngle == SERVO_POS_RETRACTED_ANGLE && retractBeepStage == 1) {
      retractBeepStage = 2;
      setBuzzer(true);
      retractBeepPulseEndTime = millis() + 12;
    } else {
      if (!isFiring && retractBeepPulseEndTime == 0)
        setBuzzer(false);
    }
    return;
  }

  bool deploying = (targetAngle == SERVO_POS_DEPLOYED_ANGLE);
  float easedT = calculateEasedT(t, deploying);

  currentAngle = startAngle + easedT * (targetAngle - startAngle);
  writeServoPulseDirect(currentAngle);
}

// ==============================================================================
// Kinematic Curves & Easing Functions
// ==============================================================================
float calculateEasedT(float t, bool deploying) {
  t = constrain(t, 0.0f, 1.0f);

  switch (activeProfile) {
  case PROFILE_IRON_MAN: {
    if (deploying) {
      // Deploying (90° -> 0°): Smoothstep with mechanical overshoot latch
      float base = smoothStep(t);
      float bump = 0.0f;

      float delta = fabs(targetAngle - startAngle);
      if (currentOvershootDeg > 0.0f && delta > 5.0f && t >= 0.60f) {
        float p = (t - 0.60f) / 0.40f;
        float bell = 16.0f * (p * p) * ((1.0f - p) * (1.0f - p));
        bump = bell * (currentOvershootDeg / delta);
      }
      return base + bump;
    } else {
      // Retracting (0° -> 90°): Smooth clean aerodynamic tuck
      return smoothStep(t);
    }
  }

  case PROFILE_SMOOTHERSTEP:
    return smootherStep(t);

  case PROFILE_CUBIC:
    return cubicEaseInOut(t);

  case PROFILE_INSTANT:
  default:
    return 1.0f;
  }
}

float smoothStep(float t) { return t * t * (3.0f - 2.0f * t); }

float cubicEaseInOut(float t) {
  return (t < 0.5f) ? (4.0f * t * t * t)
                    : (1.0f - powf(-2.0f * t + 2.0f, 3.0f) * 0.5f);
}

float smootherStep(float t) {
  return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// ==============================================================================
// Toggle Deploy (0°) <---> Retract (90°)
// ==============================================================================
void toggleServoPosition() {
  demoState = DEMO_OFF;

  if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
    Serial.println(
        "\n------------------------------------------------------------");
    Serial.printf("[ACTION] Quick Press -> EXTENDING (%d° Deployed)\n",
                  SERVO_POS_DEPLOYED_ANGLE);
    Serial.println(
        "------------------------------------------------------------");
    setLed(true);
    startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
  } else {
    Serial.println(
        "\n------------------------------------------------------------");
    Serial.printf("[ACTION] Quick Press -> RETRACTING (%d° Reloaded)\n",
                  SERVO_POS_RETRACTED_ANGLE);
    Serial.println(
        "------------------------------------------------------------");
    setLed(false);
    startServoMotion(SERVO_POS_RETRACTED_ANGLE);
  }
}

// ==============================================================================
// Button Processing with Tap vs Hold Detection (GPIO 9 BOOT Button)
// ==============================================================================
// ==============================================================================
// Unified Button Processing: Single Button Synced Controller (GPIO 9 BOOT Button)
// Controls BOTH Webshooter Extension Servo AND Shoulder Mounted Gimbal:
//   - Disarmed state:
//       * Hold >= 1000ms: Zero Gimbal Yaw (Calibrate) -> Buzzer confirm chirp
//       * Quick tap (< 500ms): DEPLOY BOTH (Webshooter opens to 0° + Gimbal unfolds)
//                              * Safety Interlock: Blocked with reject beep if not zeroed!
//   - Armed state:
//       * Quick tap (< 500ms): DISARM BOTH (Webshooter folds to 90° + Gimbal retracts)
//       * Hold >= 500ms: FIRE Shoulder Gimbal Electromagnet (via ESP-NOW, max 2.0s)
// ==============================================================================
// ==============================================================================
// Button 1 Processing: Local Webshooter Extension Servo (GPIO 9 BOOT Button)
// Rules:
//   - Disarmed state (90° Closed): Quick tap (< 500ms) -> DEPLOYS to 0° (Armed)
//   - Armed state (0° Open):       Quick tap (< 500ms) -> RETRACTS to 90° (Disarmed)
//   - Hold >= 500ms while Open:    Local ESC safety block (permanent disable)
// ==============================================================================
void processButton() {
  bool raw = digitalRead(PIN_BUTTON);
  unsigned long now = millis();

  // Debounce check
  if (raw != lastButtonRaw) {
    lastDebounceTime = now;
  }

  if ((now - lastDebounceTime) > BUTTON_DEBOUNCE_MS) {
    bool isPressed = (raw == BUTTON_ACTIVE_LEVEL); // LOW = Pressed

    if (isPressed != buttonIsPressed) {
      buttonIsPressed = isPressed;

      if (buttonIsPressed) {
        // --- BUTTON PRESSED DOWN ---
        buttonPressStartTime = now;
        holdActionHandled = false;
      } else {
        // --- BUTTON RELEASED UP ---
        if (isFiring) {
          // Stop firing immediately when button is released!
          writeFirePulseUs(fireIdlePulseUs);
          isFiring = false;
          setBuzzer(false);
          Serial.println("[WEBSHOOTER] Fire button released -> Electromagnet turned OFF.");
          // Webshooter remains extended at 0°! Never retract after firing!
        } else if (!holdActionHandled) {
          unsigned long pressDuration = now - buttonPressStartTime;

          if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
            // Disarmed (90°): Tap (< 500ms) DEPLOYS & ARMS Webshooter
            Serial.println("============================================================");
            Serial.printf("  [WEBSHOOTER] Quick Tap (%lu ms) -> EXTENDING TO 0 deg (ARMED)!\n", pressDuration);
            Serial.println("  Hold button while open to FIRE electromagnet.");
            Serial.println("  Tap button while open to RETRACT & DISARM.");
            Serial.println("============================================================");
            isArmed = true;
            hasFired = false;
            lastArmedBeepCycleMs = millis();
            startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
            setLed(true);
          } else if (targetAngle == SERVO_POS_DEPLOYED_ANGLE) {
            // Deployed (0°): Quick Tap (< 150ms) without fire hold -> RETRACTS & DISARMS
            Serial.println("============================================================");
            Serial.printf("  [WEBSHOOTER] Quick Tap (%lu ms) -> RETRACTING & FOLDING BACK (90 deg)\n", pressDuration);
            Serial.println("============================================================");
            isArmed = false;
            hasFired = false;
            startServoMotion(SERVO_POS_RETRACTED_ANGLE);
            setLed(false);
          }
        }
      }
    }

    // --- BUTTON HOLD PROCESSING (While button is held down) ---
    if (buttonIsPressed && !holdActionHandled) {
      if (targetAngle == SERVO_POS_DEPLOYED_ANGLE || isArmed) {
        // Deployed / Extending: Snappy hold >= BUTTON_FIRE_HOLD_MS (100ms) FIRES electromagnet!
        // Unblocked by isMoving so fire responds instantly without delay
        if (now - buttonPressStartTime >= BUTTON_FIRE_HOLD_MS) {
          holdActionHandled = true;
          fireProjectile();
        }
      } else if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
        // Disarmed stance (90°): Long hold >= DISARMED_LED_HOLD_MS (500ms) toggles disarmed LED
        // Never moves the servo!
        if (now - buttonPressStartTime >= DISARMED_LED_HOLD_MS) {
          holdActionHandled = true;
          disarmedLedEnabled = !disarmedLedEnabled;
          prefs.putBool("disLed", disarmedLedEnabled);
          if (disarmedLedEnabled) {
            Serial.println("[WEBSHOOTER] Disarmed LED toggled: ON (White Breathing)");
            setBuzzer(true); delay(25); setBuzzer(false); delay(30); setBuzzer(true); delay(25); setBuzzer(false);
          } else {
            Serial.println("[WEBSHOOTER] Disarmed LED toggled: OFF (Stealth Mode)");
            setBuzzer(true); delay(45); setBuzzer(false);
            ws2812WritePixel(0, 0, 0);
          }
        }
      }
    }
  }

  lastButtonRaw = raw;
}

// ==============================================================================
// ESP-NOW Master Implementation (Webshooter -> Shoulder Mounted Gimbal)
// ==============================================================================
void initEspNowMaster() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_err_t chanErr = esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (chanErr != ESP_OK) {
    Serial.printf("[ESP-NOW] Warning: Set Wi-Fi channel failed: %d\n", chanErr);
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Master initialization failed!");
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memset(&peerInfo, 0, sizeof(peerInfo));
  memset(peerInfo.peer_addr, 0xFF, 6); // Broadcast to all Gimbal receivers
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ESP-NOW] Failed to add broadcast peer!");
  } else {
    Serial.println("[ESP-NOW] Master Ready! (Broadcast mode on Wi-Fi Channel 1)");
  }
}

bool sendGimbalCommand(GimbalCommand cmd, uint16_t duration_ms) {
  uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  GimbalMessage msg;
  msg.command = (uint8_t)cmd;
  msg.duration_ms = duration_ms;
  msg.sequence = ++gimbalSequence;

  esp_err_t res = esp_now_send(broadcastAddr, (uint8_t *)&msg, sizeof(msg));
  if (res != ESP_OK) {
    Serial.printf("[ESP-NOW] Send command %d failed: 0x%X\n", cmd, res);
  }
  return (res == ESP_OK);
}

// ==============================================================================
// Gimbal Button Handler (GPIO 8 - Dedicated Shoulder Mounted Gimbal Trigger)
// Rules:
//   - Disarmed state:
//       * Single press (tap): DEPLOY (arm)
//       * Long press (>= 1000ms): CALIBRATE
//   - Armed state:
//       * Single press (tap): DISARM (retract)
//       * Hold (>= 400ms): FIRE electromagnet until release (Max 2.0s overheat cutoff)
// ==============================================================================
void processGimbalButton() {
  bool raw = digitalRead(PIN_BUTTON_GIMBAL);
  unsigned long now = millis();

  if (raw != lastGimbalButtonRaw) {
    lastGimbalDebounceTime = now;
  }

  if ((now - lastGimbalDebounceTime) > BUTTON_DEBOUNCE_MS) {
    bool isPressed = (raw == BUTTON_ACTIVE_LEVEL);

    if (isPressed != gimbalButtonIsPressed) {
      gimbalButtonIsPressed = isPressed;

      if (gimbalButtonIsPressed) {
        // Button Pressed Down
        gimbalPressStartTime = now;
        gimbalHoldHandled = false;
      } else {
        // Button Released Up
        if (gimbalFiring) {
          // User let go while firing!
          gimbalFiring = false;
          sendGimbalCommand(CMD_STOP_FIRE);
          Serial.println("\n[GIMBAL] Fire button released -> STOP_FIRE sent.");
        } else if (!gimbalHoldHandled) {
          // Single Tap: released before any hold triggered
          if (!gimbalArmed) {
            // Disarmed state -> Single press = DEPLOY (arm)
            gimbalArmed = true;
            sendGimbalCommand(CMD_DEPLOY);
            Serial.println("============================================================");
            Serial.println("  [GIMBAL] Single Tap -> DEPLOY (ARM) sent via ESP-NOW!");
            Serial.println("============================================================");
          } else {
            // Armed state -> Single press = DISARM (retract)
            gimbalArmed = false;
            sendGimbalCommand(CMD_DISARM);
            Serial.println("\n============================================================");
            Serial.println("  [GIMBAL] Single Tap -> DISARM sent via ESP-NOW!");
            Serial.println("============================================================");
          }
        }
      }
    }

    // Button HOLD processing while pressed
    if (gimbalButtonIsPressed) {
      if (gimbalArmed) {
        // When armed: Hold >= BUTTON_FIRE_HOLD_MS (400ms) fires electromagnet until release (max 2.0s)
        if (!gimbalFiring && !gimbalHoldHandled && (now - gimbalPressStartTime >= BUTTON_FIRE_HOLD_MS)) {
          gimbalFiring = true;
          gimbalHoldHandled = true;
          gimbalFireStartTime = now;
          lastGimbalKeepAliveMs = now;
          sendGimbalCommand(CMD_FIRE, GIMBAL_MAX_FIRE_MS);
          Serial.println("\n============================================================");
          Serial.println("  [GIMBAL] HELD WHILE ARMED -> FIRING ELECTROMAGNET (Max 2.0s)!");
          Serial.println("============================================================");
        } else if (gimbalFiring) {
          // Hard 2.0-second safety cutoff to prevent coil overheating
          if (now - gimbalFireStartTime >= GIMBAL_MAX_FIRE_MS) {
            gimbalFiring = false;
            sendGimbalCommand(CMD_STOP_FIRE);
            Serial.println("\n[GIMBAL] 2.0s limit reached! Electromagnet cut off to prevent overheating.");
          } else if (now - lastGimbalKeepAliveMs >= 80) {
            // Periodic keep-alive packet while holding fire
            lastGimbalKeepAliveMs = now;
            sendGimbalCommand(CMD_FIRE, GIMBAL_MAX_FIRE_MS);
          }
        }
      } else {
        // When disarmed: Long press >= GIMBAL_LONG_PRESS_MS (1000ms) triggers CALIBRATE
        if (!gimbalHoldHandled && (now - gimbalPressStartTime >= GIMBAL_LONG_PRESS_MS)) {
          gimbalHoldHandled = true;
          sendGimbalCommand(CMD_CALIBRATE);
          setBuzzer(true); delay(60); setBuzzer(false); // Immediate audio confirmation chirp
          Serial.println("\n============================================================");
          Serial.println("  [GIMBAL] Long Press Disarmed -> CALIBRATE sent via ESP-NOW!");
          Serial.println("============================================================");
        }
      }
    }
  }

  lastGimbalButtonRaw = raw;
}

// ==============================================================================
// Serial Commands for Testing, Tuning & Calibration
// ==============================================================================
void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();

    switch (c) {
    case 'f':
    case 'F':
      if (targetAngle == SERVO_POS_DEPLOYED_ANGLE && !isMoving) {
        fireProjectile();
      } else {
        Serial.println("\n[SAFETY] Cannot fire while retracted! Deploy first ('0' or button tap).");
      }
      break;

    case 'a':
    case 'A':
      if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
        isArmed = true;
        hasFired = false;
        lastArmedBeepCycleMs = millis();
        startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
        setLed(true);
        Serial.println("\n[SERIAL] Mechanism OPENED (0°) & System ARMED!");
      } else {
        isArmed = false;
        hasFired = false;
        startServoMotion(SERVO_POS_RETRACTED_ANGLE);
        setLed(false);
        Serial.println(
            "\n[SERIAL] Mechanism FOLDED BACK (90°) & System DISARMED!");
      }
      break;

    case 'b':
    case 'B':
    case 't':
    case 'T':
      // Serial button simulation for SERVO ONLY - never fires 25W magnet over
      // USB!
      if (targetAngle == SERVO_POS_RETRACTED_ANGLE) {
        isArmed = true;
        hasFired = false;
        lastArmedBeepCycleMs = millis();
        startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
        setLed(true);
        Serial.println("\n[SERIAL TEST] Opened to 0° (Armed)");
      } else {
        isArmed = false;
        hasFired = false;
        startServoMotion(SERVO_POS_RETRACTED_ANGLE);
        setLed(false);
        Serial.println("\n[SERIAL TEST] Folded back to 90° (Disarmed)");
      }
      break;

    case ' ':
      if (targetAngle == SERVO_POS_DEPLOYED_ANGLE && !isMoving) {
        Serial.println("\n[SERIAL] Local Webshooter Fire pulse triggered!");
        fireProjectile();
      } else {
        Serial.println("\n[SAFETY] Cannot fire while retracted! Deploy first ('0' or button tap).");
      }
      break;

    case '0':
      demoState = DEMO_OFF;
      Serial.printf("\n[SERIAL] Deploying to %d° (Fired)...\n",
                    SERVO_POS_DEPLOYED_ANGLE);
      setLed(true);
      startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
      break;

    case '9':
      demoState = DEMO_OFF;
      Serial.printf("\n[SERIAL] Retracting to %d° (Reloaded)...\n",
                    SERVO_POS_RETRACTED_ANGLE);
      setLed(false);
      startServoMotion(SERVO_POS_RETRACTED_ANGLE);
      break;

    case 's':
    case 'S':
      Serial.println(
          "\n[DEMO] Starting Iron Man Cinematic Webshooter Showcase Cycle...");
      setLed(true);
      startServoMotion(SERVO_POS_DEPLOYED_ANGLE);
      demoState = DEMO_DEPLOYING;
      break;

    case '1':
      activeProfile = PROFILE_IRON_MAN;
      Serial.println("\n[PROFILE] Switched to: (1) IRON MAN (Cinematic Pod "
                     "Launch + Mechanical Latch Overshoot)");
      break;

    case '2':
      activeProfile = PROFILE_SMOOTHERSTEP;
      Serial.println("\n[PROFILE] Switched to: (2) SMOOTHERSTEP (Ken Perlin "
                     "5th-Order, Zero Overshoot)");
      break;

    case '3':
      activeProfile = PROFILE_CUBIC;
      Serial.println(
          "\n[PROFILE] Switched to: (3) CUBIC EASE-IN-OUT (Symmetric S-Curve)");
      break;

    case '4':
      activeProfile = PROFILE_INSTANT;
      Serial.println(
          "\n[PROFILE] Switched to: (4) INSTANT SNAP (Raw Servo Speed)");
      break;

    case 'o':
    case 'O':
      if (currentOvershootDeg > 0.0f) {
        currentOvershootDeg = 0.0f;
        Serial.println(
            "\n[OVERSHOOT] Mechanical Latch Overshoot: DISABLED (0.0°)");
      } else {
        currentOvershootDeg = SERVO_DEPLOY_OVERSHOOT_DEG;
        Serial.printf(
            "\n[OVERSHOOT] Mechanical Latch Overshoot: ENABLED (%.1f°)\n",
            currentOvershootDeg);
      }
      break;

    case '+':
    case '=':
      if (deployDurationMs > 100)
        deployDurationMs -= 50;
      if (retractDurationMs > 100)
        retractDurationMs -= 50;
      Serial.printf("\n[SPEED] Faster motion! Deploy: %lums | Retract: %lums\n",
                    deployDurationMs, retractDurationMs);
      break;

    case '-':
    case '_':
      if (deployDurationMs < 3000)
        deployDurationMs += 50;
      if (retractDurationMs < 3000)
        retractDurationMs += 50;
      Serial.printf("\n[SPEED] Slower motion! Deploy: %lums | Retract: %lums\n",
                    deployDurationMs, retractDurationMs);
      break;

    case '[':
      if (fireIdlePulseUs > 400)
        fireIdlePulseUs -= 25;
      writeFirePulseUs(fireIdlePulseUs);
      Serial.printf("\n[CALIBRATION] ESC Idle Pulse: %lu us\n",
                    fireIdlePulseUs);
      break;

    case ']':
      if (fireIdlePulseUs < 1200)
        fireIdlePulseUs += 25;
      writeFirePulseUs(fireIdlePulseUs);
      Serial.printf("\n[CALIBRATION] ESC Idle Pulse: %lu us\n",
                    fireIdlePulseUs);
      break;

    case '{':
      if (fireActivePulseUs > 1400)
        fireActivePulseUs -= 50;
      Serial.printf("\n[CALIBRATION] ESC Active Fire Pulse: %lu us\n",
                    fireActivePulseUs);
      break;

    case '}':
      if (fireActivePulseUs < 2500)
        fireActivePulseUs += 50;
      Serial.printf("\n[CALIBRATION] ESC Active Fire Pulse: %lu us\n",
                    fireActivePulseUs);
      break;

    case 'r':
    case 'R':
      demoState = DEMO_OFF;
      isMoving = false;
      Serial.println("\n[SERIAL] Cutting Servo PWM immediately (release).");
      releaseServoSignal();
      break;

    case 'p':
    case 'P':
      autoReleaseEnabled = !autoReleaseEnabled;
      Serial.printf("\n[CONFIG] Auto-Release PWM: %s\n",
                    autoReleaseEnabled
                        ? "ENABLED (Silent after motion + 800ms hold)"
                        : "DISABLED (Continuous Holding Torque)");
      if (!autoReleaseEnabled) {
        writeServoPulseDirect(currentAngle);
      }
      break;

    case 'c':
    case 'C':
      gimbalZeroed = true;
      sendGimbalCommand(CMD_CALIBRATE);
      Serial.println("\n[SERIAL] Sent CMD_CALIBRATE to Shoulder Mounted Gimbal.");
      break;

    case 'd':
    case 'D':
      gimbalArmed = true;
      sendGimbalCommand(CMD_DEPLOY);
      Serial.println("\n[SERIAL] Sent CMD_DEPLOY (ARM) to Shoulder Mounted Gimbal.");
      break;

    case 'x':
    case 'X':
      gimbalArmed = false;
      sendGimbalCommand(CMD_DISARM);
      Serial.println("\n[SERIAL] Sent CMD_DISARM to Shoulder Mounted Gimbal.");
      break;

    case '?':
    case 'h':
    case 'H':
      printHelpAndStatus();
      break;

    default:
      break;
    }
  }
}

// ==============================================================================
// LED Helper (GPIO 8 is dedicated to Gimbal Button - LED no-op)
// ==============================================================================
void setLed(bool on) {
  // GPIO 8 is reserved exclusively for the Shoulder Mounted Gimbal Trigger
}

// ==============================================================================
// Telemetry & Help
// ==============================================================================
void printHelpAndStatus() {
  const char *profileNames[] = {
      "Unknown", "IRON MAN (Cinematic Pod Launch + Mechanical Latch)",
      "SMOOTHERSTEP (Ken Perlin 5th-Order, Zero Overshoot)",
      "CUBIC EASE-IN-OUT (Symmetric S-Curve)",
      "INSTANT SNAP (Raw Motor Speed)"};

  Serial.println(
      "\n============================================================");
  Serial.println("  SPIDER-MAN WEBSHOOTER - ADVANCED COMBAT FIRING CONTROLLER");
  Serial.println(
      "============================================================");
  Serial.printf("  Current Angle:       %.1f°\n", currentAngle);
  Serial.printf("  Armed Status:        %s\n",
                isArmed ? "*** ARMED (READY TO FIRE) ***" : "DISARMED (Safe)");
  Serial.printf("  Active Profile:      [%d] %s\n", activeProfile,
                profileNames[constrain(activeProfile, 0, 4)]);
  Serial.printf("  Deploy (0°) Pacing:  %lu ms\n", deployDurationMs);
  Serial.printf("  Retract (90°) Pacing:%lu ms\n", retractDurationMs);
  Serial.printf("  Overshoot Latch:     %.1f°\n", currentOvershootDeg);
  Serial.printf("  ESC Firing Pulses:   Idle: %lu us (0 deg) | Active: %lu us "
                "(Full 25W)\n",
                fireIdlePulseUs, fireActivePulseUs);
  Serial.printf("  Firing Duration:     %d ms (%.1fs)\n",
                FIRE_PULSE_DURATION_MS,
                (float)FIRE_PULSE_DURATION_MS / 1000.0f);
  Serial.printf("  Active Buzzer Pin:   GPIO %d (Digital DC HIGH = Sound ON)\n",
                PIN_BUZZER);
  Serial.println(
      "------------------------------------------------------------");
  Serial.println("  [CONTROLS]:");
  Serial.println(
      "   - WHEN CLOSED (90°): Tap button to OPEN (0°) & ARM SYSTEM");
  Serial.println("   - WHEN ARMED (0°):");
  Serial.println("       * HOLD >= 500ms: FIRE ELECTROMAGNET (1.5s Full Power "
                 "- KEEPS OPEN!)");
  Serial.println(
      "       * TAP < 500ms:   IMMEDIATELY DISARMS & FOLDS BACK (90°)");
  Serial.println(
      "   - POST-FIRE (Kept Open): Tap < 500ms to DISARM & FOLD BACK (90°)");
  Serial.println("   'f'                 - Fire electromagnet immediately");
  Serial.println("   'a'                 - Toggle Armed / Disarmed state");
  Serial.println("   '0' / '9'           - Direct Extend (0°) / Retract (90°)");
  Serial.println("   '[' / ']'           - Trim ESC Idle pulse (±25us)");
  Serial.println("   '{' / '}'           - Trim ESC Fire pulse (±50us)");
  Serial.println("   '1'-'4'             - Select motion curve");
  Serial.println("   'o'                 - Toggle mechanical latch overshoot");
  Serial.println("   '+' / '-'           - Fine-tune motion duration (±50ms)");
  Serial.println("   '?' or 'h'          - Print this telemetry menu");
  Serial.println(
      "============================================================\n");
  Serial.println("[READY] System initialized and safe.");
}
