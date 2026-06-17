#pragma once
/**
 * Config.h — Hardware pins, default parameters, EEPROM persistence
 *
 * Fysetc E4 pin assignments (ESP32-WROOM-32):
 * ┌──────────────┬──────────┬───────────────────────────────────┐
 * │ Function     │ GPIO     │ Notes                             │
 * ├──────────────┼──────────┼───────────────────────────────────┤
 * │ X STEP       │ 27       │ Left motor step                   │
 * │ X DIR        │ 26       │ Left motor direction              │
 * │ X EN         │ 25       │ Active LOW enable                 │
 * │ Y STEP       │ 33       │ Right motor step                  │
 * │ Y DIR        │ 32       │ Right motor direction             │
 * │ Y EN         │ 25       │ Shared enable (all motors)        │
 * │ UART TX→drv  │ 17       │ TMC2209 UART (shared bus)         │
 * │ UART RX←drv  │ 16       │ TMC2209 UART (shared bus)         │
 * │ Servo PWM    │ 2        │ FAN1 / LED pin repurposed         │
 * │ E-STOP       │ 34       │ Pull-up; active LOW               │
 * └──────────────┴──────────┴───────────────────────────────────┘
 */

#include <Arduino.h>
#include <Preferences.h>

// ── Pin definitions ────────────────────────────────────────────
constexpr int PIN_X_STEP  = 27;
constexpr int PIN_X_DIR   = 26;
constexpr int PIN_Y_STEP  = 33;
constexpr int PIN_Y_DIR   = 32;
constexpr int PIN_EN      = 25;   // active LOW, shared
constexpr int PIN_UART_TX = 17;   // to TMC2209 PDN/UART
constexpr int PIN_UART_RX = 16;
constexpr int PIN_SERVO   = 2;
constexpr int PIN_ESTOP   = 34;

// ── Servo angles ───────────────────────────────────────────────
constexpr int SERVO_UP_ANGLE   = 60;   // pen lifted
constexpr int SERVO_DOWN_ANGLE = 120;  // pen on paper

// ── Motion defaults (overridable via M-codes / EEPROM) ─────────
namespace Defaults {
    // Machine geometry (mm)
    constexpr float MACHINE_WIDTH   = 700.0f;  // distance between anchors
    constexpr float MACHINE_HEIGHT  = 900.0f;  // canvas vertical extent
    constexpr float HOME_X          = MACHINE_WIDTH  / 2.0f;
    constexpr float HOME_Y          = 200.0f;  // home offset from top

    // Mechanical
    constexpr float STEPS_PER_MM    = 80.0f;   // 200step/rev × 16µstep ÷ 40mm/rev GT2
    constexpr float MAX_FEEDRATE     = 150.0f;  // mm/s
    constexpr float HOME_FEEDRATE    = 50.0f;
    constexpr float ACCELERATION     = 400.0f;  // mm/s²
    constexpr float JUNCTION_SPEED   = 5.0f;    // mm/s cornering

    // TMC2209 current (mA RMS)
    constexpr uint16_t RUN_CURRENT   = 600;
    constexpr uint16_t HOLD_CURRENT  = 200;
    constexpr uint8_t  MICROSTEPS    = 16;
}

// ── Runtime config struct (lives in RAM, persisted to NVS) ─────
struct MachineConfig {
    float  machineWidth   = Defaults::MACHINE_WIDTH;
    float  machineHeight  = Defaults::MACHINE_HEIGHT;
    float  stepsPerMm     = Defaults::STEPS_PER_MM;
    float  maxFeedrate    = Defaults::MAX_FEEDRATE;
    float  acceleration   = Defaults::ACCELERATION;
    uint16_t runCurrent   = Defaults::RUN_CURRENT;
    uint16_t holdCurrent  = Defaults::HOLD_CURRENT;
    uint8_t  microsteps   = Defaults::MICROSTEPS;
    int    servoUp        = SERVO_UP_ANGLE;
    int    servoDown      = SERVO_DOWN_ANGLE;
};

// ── Config namespace (singleton-ish) ──────────────────────────
namespace Config {
    extern MachineConfig cfg;

    void load();
    void save();
    void reset();
    void print();
}
