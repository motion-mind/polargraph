#pragma once
/**
 * PenServo.h — Four independent pen servos
 * Branch: feature/multi-pen
 *
 * Manages 4 servos (one per pen) on GPIO 2, 4, 13, 14.
 * Only one pen can be down at a time — selectPen() raises the
 * current pen before lowering the new one, preventing ink smear
 * from two pens touching the paper simultaneously.
 *
 * LEDC channel allocation (ESP32Servo uses LEDC internally):
 *   Servo 0 → LEDC channel 0  (timer 0)
 *   Servo 1 → LEDC channel 1  (timer 1)
 *   Servo 2 → LEDC channel 2  (timer 2)
 *   Servo 3 → LEDC channel 3  (timer 3)
 * All four share 50Hz PWM, standard servo pulse range 500–2400µs.
 */

#include <Arduino.h>
#include <ESP32Servo.h>
#include "Config.h"

class PenServo {
public:
    void begin() {
        // Allocate one LEDC timer per servo
        for (int i = 0; i < NUM_PENS; i++) {
            ESP32PWM::allocateTimer(i);
            _servos[i].setPeriodHertz(50);
            _servos[i].attach(PIN_SERVO[i], 500, 2400);
        }
        // All pens up at startup
        allUp();
        Serial.printf("  PenServo: %d pens initialised on GPIO %d,%d,%d,%d\n",
            NUM_PENS,
            PIN_SERVO[0], PIN_SERVO[1], PIN_SERVO[2], PIN_SERVO[3]);
    }

    // ── Raise all pens ────────────────────────────────────────
    void allUp() {
        for (int i = 0; i < NUM_PENS; i++) {
            _servos[i].write(Config::cfg.pens[i].servoUp);
        }
        delay(SETTLE_MS);
        _activePen = -1;   // no pen selected
        _isDown    = false;
    }

    // ── Select active pen (raises current, does NOT lower new one yet) ──
    // Call this during a tool-change move (pen travels to new position
    // with all pens up). Call penDown() when ready to draw.
    void selectPen(int pen) {
        if (pen < 0 || pen >= NUM_PENS) return;
        if (_isDown) penUp();           // raise current pen first
        _activePen = pen;
        Serial.printf("  Pen %d selected\n", pen);
    }

    // ── Lower the active pen ──────────────────────────────────
    void penDown() {
        if (_activePen < 0 || _activePen >= NUM_PENS) return;
        if (!_isDown) {
            _servos[_activePen].write(Config::cfg.pens[_activePen].servoDown);
            delay(SETTLE_MS);
            _isDown = true;
        }
    }

    // ── Raise the active pen ──────────────────────────────────
    void penUp() {
        if (_activePen >= 0 && _activePen < NUM_PENS && _isDown) {
            _servos[_activePen].write(Config::cfg.pens[_activePen].servoUp);
            delay(SETTLE_MS);
            _isDown = false;
        }
    }

    // ── Direct angle control (for calibration) ────────────────
    void setAngle(int pen, int angle) {
        if (pen < 0 || pen >= NUM_PENS) return;
        _servos[pen].write(angle);
        delay(SETTLE_MS);
    }

    // ── Accessors ─────────────────────────────────────────────
    int  activePen() const { return _activePen; }
    bool isDown()    const { return _isDown; }
    bool isUp()      const { return !_isDown; }  // matches old single-pen API

private:
    Servo _servos[NUM_PENS];
    int   _activePen = 0;
    bool  _isDown    = false;

    static constexpr int SETTLE_MS = 120;
};
