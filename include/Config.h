#pragma once
/**
 * Config.h — Hardware pins, defaults, NVS persistence
 * Branch: feature/multi-pen
 *
 * Fysetc E4 pin assignments (ESP32-WROOM-32):
 * ┌──────────────────┬──────────┬────────────────────────────────────────┐
 * │ Function         │ GPIO     │ Notes                                  │
 * ├──────────────────┼──────────┼────────────────────────────────────────┤
 * │ X STEP           │ 27       │ Left cord motor step                   │
 * │ X DIR            │ 26       │ Left cord motor direction              │
 * │ Y STEP           │ 33       │ Right cord motor step                  │
 * │ Y DIR            │ 32       │ Right cord motor direction             │
 * │ ENABLE (shared)  │ 25       │ Active LOW, all motors                 │
 * │ TMC2209 UART TX  │ 17       │ Shared UART bus to drivers             │
 * │ TMC2209 UART RX  │ 16       │ Shared UART bus from drivers           │
 * │ E-STOP           │ 34       │ Input-only, active LOW                 │
 * ├──────────────────┼──────────┼────────────────────────────────────────┤
 * │ Servo 0 (Pen 0)  │  2       │ HEAT_E0 header — must be LOW at boot   │
 * │ Servo 1 (Pen 1)  │  4       │ HEAT_BED header                        │
 * │ Servo 2 (Pen 2)  │ 13       │ FAN_E0 header                          │
 * │ Servo 3 (Pen 3)  │ 14       │ Z endstop header (endstop unused)      │
 * └──────────────────┴──────────┴────────────────────────────────────────┘
 *
 * Servo power: use 5V pins on the board power header, NOT the heater
 * outputs (those are switched MOSFETs at 12/24V).
 *
 * Square gondola arrangement (top view):
 *
 *       ┌───────────────┐
 *       │  Pen0   Pen1  │
 *       │               │
 *       │  Pen3   Pen2  │
 *       └───────────────┘
 *
 * Tool offsets (offsetX, offsetY) are the position of each pen tip
 * relative to the gondola centre in mm.  Calibrate with M218 T0-T3.
 */

#include <Arduino.h>
#include <Preferences.h>

// ── Stepper pins ───────────────────────────────────────────────
constexpr int PIN_X_STEP  = 27;
constexpr int PIN_X_DIR   = 26;
constexpr int PIN_Y_STEP  = 33;
constexpr int PIN_Y_DIR   = 32;
constexpr int PIN_EN      = 25;
constexpr int PIN_UART_TX = 17;
constexpr int PIN_UART_RX = 16;
constexpr int PIN_ESTOP   = 34;

// ── Servo pins — one per pen ───────────────────────────────────
constexpr int PIN_SERVO[4] = { 2, 4, 13, 14 };

// ── Number of pens ─────────────────────────────────────────────
constexpr int NUM_PENS = 4;

// ── Default servo angles (same for all pens, individually tunable) ──
constexpr int SERVO_UP_ANGLE   = 60;
constexpr int SERVO_DOWN_ANGLE = 120;

// ── Motion defaults ────────────────────────────────────────────
namespace Defaults {
    constexpr float MACHINE_WIDTH  = 700.0f;
    constexpr float MACHINE_HEIGHT = 900.0f;
    constexpr float HOME_X         = MACHINE_WIDTH / 2.0f;
    constexpr float HOME_Y         = 200.0f;
    constexpr float STEPS_PER_MM  = 80.0f;
    constexpr float MAX_FEEDRATE   = 150.0f;
    constexpr float HOME_FEEDRATE  = 50.0f;
    constexpr float ACCELERATION   = 400.0f;
    constexpr float JUNCTION_SPEED = 5.0f;
    constexpr uint16_t RUN_CURRENT = 600;
    constexpr uint16_t HOLD_CURRENT= 200;
    constexpr uint8_t  MICROSTEPS  = 16;

    // Tool offsets — pens in a square, 10mm from centre each axis
    // Adjust these to match your actual gondola geometry.
    //        Pen0        Pen1        Pen2        Pen3
    constexpr float TOOL_OFFSET_X[4] = { -10,  10,  10, -10 };
    constexpr float TOOL_OFFSET_Y[4] = { -10, -10,  10,  10 };
}

// ── Per-pen configuration ──────────────────────────────────────
struct PenConfig {
    int   servoUp   = SERVO_UP_ANGLE;
    int   servoDown = SERVO_DOWN_ANGLE;
    float offsetX   = 0.0f;   // mm from gondola centre
    float offsetY   = 0.0f;
    bool  enabled   = true;   // can disable pens not loaded
};

// ── Runtime machine config ─────────────────────────────────────
struct MachineConfig {
    float    machineWidth  = Defaults::MACHINE_WIDTH;
    float    machineHeight = Defaults::MACHINE_HEIGHT;
    float    stepsPerMm    = Defaults::STEPS_PER_MM;
    float    maxFeedrate   = Defaults::MAX_FEEDRATE;
    float    acceleration  = Defaults::ACCELERATION;
    uint16_t runCurrent    = Defaults::RUN_CURRENT;
    uint16_t holdCurrent   = Defaults::HOLD_CURRENT;
    uint8_t  microsteps    = Defaults::MICROSTEPS;
    PenConfig pens[NUM_PENS];
};

namespace Config {
    extern MachineConfig cfg;
    void load();
    void save();
    void reset();
    void print();
}
