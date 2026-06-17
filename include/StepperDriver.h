#pragma once
/**
 * StepperDriver.h — Low-level stepper control for Fysetc E4
 *
 * Uses TMCStepper library for TMC2209 UART configuration,
 * then direct GPIO step/dir pulses for maximum timing accuracy
 * (hardware timer ISR driven, no AccelStepper dependency).
 *
 * Both TMC2209 chips share the same UART bus; they are addressed
 * by their MS1/MS2 address pins (0 for left, 1 for right).
 */

#include <Arduino.h>
#include <TMCStepper.h>   // https://github.com/teemuatlut/TMCStepper
#include "Config.h"

// TMC2209 UART addresses (set by MS1/MS2 on E4 board)
constexpr uint8_t TMC_ADDR_LEFT  = 0;
constexpr uint8_t TMC_ADDR_RIGHT = 1;
constexpr float   TMC_RSENSE     = 0.11f;  // E4 sense resistor

class StepperDriver {
public:
    void begin() {
        // ── GPIO setup ─────────────────────────────────────────
        pinMode(PIN_X_STEP, OUTPUT);
        pinMode(PIN_X_DIR,  OUTPUT);
        pinMode(PIN_Y_STEP, OUTPUT);
        pinMode(PIN_Y_DIR,  OUTPUT);
        pinMode(PIN_EN,     OUTPUT);
        disable();   // motors off until needed

        // ── TMC2209 UART init ──────────────────────────────────
        Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX);

        _driverL = new TMC2209Stepper(&Serial2, TMC_RSENSE, TMC_ADDR_LEFT);
        _driverR = new TMC2209Stepper(&Serial2, TMC_RSENSE, TMC_ADDR_RIGHT);

        configureTMC(_driverL);
        configureTMC(_driverR);

        Serial.println(F("  StepperDriver: TMC2209 configured"));
    }

    // ── Enable / disable ───────────────────────────────────────
    void enable()  { digitalWrite(PIN_EN, LOW);  _enabled = true;  }
    void disable() { digitalWrite(PIN_EN, HIGH); _enabled = false; }
    bool isEnabled() const { return _enabled; }

    // ── Single-step pulse (called from ISR or planner) ─────────
    // IRAM_ATTR ensures code lives in IRAM for ISR safety
    IRAM_ATTR void stepLeft(bool dir) {
        digitalWrite(PIN_X_DIR, dir ? HIGH : LOW);
        digitalWrite(PIN_X_STEP, HIGH);
        delayMicroseconds(2);
        digitalWrite(PIN_X_STEP, LOW);
    }

    IRAM_ATTR void stepRight(bool dir) {
        digitalWrite(PIN_Y_DIR, dir ? HIGH : LOW);
        digitalWrite(PIN_Y_STEP, HIGH);
        delayMicroseconds(2);
        digitalWrite(PIN_Y_STEP, LOW);
    }

    // ── Reconfigure current (called after config change) ───────
    void updateCurrent() {
        if (_driverL) {
            _driverL->rms_current(Config::cfg.runCurrent);
            _driverR->rms_current(Config::cfg.runCurrent);
        }
    }

    // ── Driver accessors (for StallHoming) ───────────────────
    TMC2209Stepper* driverL() { return _driverL; }
    TMC2209Stepper* driverR() { return _driverR; }

    // ── Diagnostics ───────────────────────────────────────────
    void printStatus() {
        if (!_driverL) return;
        Serial.printf("  TMC Left  OT:%d OTP:%d S2G:%d S2VS:%d STST:%d\n",
            _driverL->ot(), _driverL->otpw(),
            _driverL->s2g(), _driverL->s2vs(), _driverL->stst());
        Serial.printf("  TMC Right OT:%d OTP:%d S2G:%d S2VS:%d STST:%d\n",
            _driverR->ot(), _driverR->otpw(),
            _driverR->s2g(), _driverR->s2vs(), _driverR->stst());
    }

private:
    TMC2209Stepper* _driverL = nullptr;
    TMC2209Stepper* _driverR = nullptr;
    bool _enabled = false;

    void configureTMC(TMC2209Stepper* drv) {
        drv->begin();
        drv->toff(5);
        drv->blank_time(24);
        drv->rms_current(Config::cfg.runCurrent);
        drv->microsteps(Config::cfg.microsteps);
        drv->intpol(true);    // interpolate to 256 µsteps internally
        drv->en_spreadCycle(false);  // StealthChop for quiet operation
        drv->pwm_autoscale(true);
        // StallGuard / CoolStep off for plotter (no load sensing needed)
        drv->TCOOLTHRS(0);
        drv->semin(0);
    }
};
