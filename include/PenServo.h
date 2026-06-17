#pragma once
/**
 * PenServo.h — Servo-driven pen lift
 *
 * Uses ESP32Servo library (ledc-based, no analogWrite needed).
 * A small delay after moving lets the servo settle before drawing.
 */

#include <Arduino.h>
#include <ESP32Servo.h>

class PenServo {
public:
    void begin(int pin, int upAngle, int downAngle) {
        _pin       = pin;
        _upAngle   = upAngle;
        _downAngle = downAngle;

        // Allocate a PWM timer for the servo
        ESP32PWM::allocateTimer(3);
        _servo.setPeriodHertz(50);        // standard 50Hz servo
        _servo.attach(pin, 500, 2400);    // µs pulse range

        penUp();
        Serial.printf("  PenServo: pin %d  up=%d° down=%d°\n",
                      pin, upAngle, downAngle);
    }

    void penUp() {
        if (!_isUp) {
            _servo.write(_upAngle);
            delay(SETTLE_MS);
            _isUp = true;
        }
    }

    void penDown() {
        if (_isUp) {
            _servo.write(_downAngle);
            delay(SETTLE_MS);
            _isUp = false;
        }
    }

    void setAngle(int angle) {
        _servo.write(angle);
        delay(SETTLE_MS);
        _isUp = (angle == _upAngle);
    }

    bool isUp() const { return _isUp; }

    void setUpAngle(int a)   { _upAngle   = a; }
    void setDownAngle(int a) { _downAngle = a; }

private:
    Servo _servo;
    int   _pin       = -1;
    int   _upAngle   = 60;
    int   _downAngle = 120;
    bool  _isUp      = true;

    static constexpr int SETTLE_MS = 120;  // ms to let servo reach position
};
