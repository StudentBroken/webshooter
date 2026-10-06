#pragma once
#include <Arduino.h>

// ==============================================================================
// ESP-NOW Protocol: Webshooter (Master) <--> Shoulder Mounted Gimbal (Slave)
// ==============================================================================
#define ESPNOW_WIFI_CHANNEL         1       // Fixed 2.4GHz Wi-Fi Channel
#define GIMBAL_MAX_FIRE_DURATION_MS 2000    // 2.0s hardware safety cutoff to prevent coil overheating

enum GimbalCommand : uint8_t {
    CMD_NONE         = 0,
    CMD_CALIBRATE    = 1,  // Long press while disarmed -> calibrate gimbal
    CMD_DEPLOY       = 2,  // Single tap while disarmed -> deploy (arm) gimbal
    CMD_DISARM       = 3,  // Single tap while armed    -> disarm (retract) gimbal
    CMD_FIRE         = 4,  // Held while armed          -> energize electromagnet
    CMD_STOP_FIRE    = 5,  // Released / timeout        -> de-energize electromagnet
    CMD_TELEMETRY    = 6   // Webshooter periodic telemetry broadcast
};

typedef struct __attribute__((packed)) {
    uint8_t command;        // GimbalCommand
    uint16_t duration_ms;   // Safety duration parameter (ms)
    uint32_t sequence;      // Monotonically increasing sequence number
} GimbalMessage;

typedef struct __attribute__((packed)) {
    uint8_t command;        // CMD_TELEMETRY (6)
    uint16_t vbat_mv;       // Battery voltage in millivolts (e.g. 4150 for 4.15V)
    uint8_t webshooter_pos; // 0 = Deployed (0°), 1 = Retracted (90°)
    uint8_t gimbal_armed;   // 0 = Disarmed, 1 = Armed
    uint8_t gimbal_firing;  // 0 = Idle, 1 = Firing
    uint16_t breath_phase_ms; // Current Apple breathing phase for synchronized rhythm
    uint32_t sequence;      // Monotonically increasing sequence number
} WebshooterTelemetry;

