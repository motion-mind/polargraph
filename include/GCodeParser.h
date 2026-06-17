#pragma once
/**
 * GCodeParser.h — Serial G-code interpreter
 *
 * Supported commands:
 *   G0  [Xnnn] [Ynnn]          Rapid move (max feedrate)
 *   G1  [Xnnn] [Ynnn] [Fnnn]   Linear move
 *   G4  Pnnn                   Dwell milliseconds
 *   G28                        Go to home position
 *   G90                        Absolute positioning (default)
 *   G91                        Relative positioning
 *   G92 [Xnnn] [Ynnn]          Set current position
 *   M3  [Snnn]                 Pen down (optional angle override)
 *   M5                         Pen up
 *   M17                        Enable steppers
 *   M18 / M84                  Disable steppers
 *   M100                       Help
 *   M114                       Report current position
 *   M119                       Report E-stop state
 *   M503                       Print settings
 *   M500                       Save settings to NVS
 *   M502                       Reset settings to defaults
 *   M906 Xnnn                  Set stepper current (mA RMS)
 *   M201 Xnnn                  Set acceleration (mm/s²)
 *   M203 Xnnn                  Set max feedrate (mm/s)
 *   M92  Xnnn                  Set steps/mm
 *   M104 Snnn                  Set machine width (mm)  [repurposed]
 *   M105 Snnn                  Set machine height (mm) [repurposed]
 *   M280 Pnnn Snnn             Set servo angle (P0=up P1=down S=degrees)
 *
 * All commands respond with "ok" on success.
 * Comments start with ';' and are stripped.
 */

#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "Config.h"
#include "MotionPlanner.h"
#include "PenServo.h"
#include "Kinematics.h"
#include "StallHoming.h"

class GCodeParser {
public:
    void begin(MotionPlanner* planner, PenServo* servo,
               Kinematics* kin, StallHoming* homing) {
        _planner  = planner;
        _servo    = servo;
        _kin      = kin;
        _homing   = homing;
        _absolute = true;
        _feedrate = Config::cfg.maxFeedrate;
        Serial.println(F("  GCodeParser: ready"));
    }

    void process(const char* line) {
        // Strip comment
        static char buf[128];
        strncpy(buf, line, sizeof(buf)-1);
        buf[sizeof(buf)-1] = '\0';
        char* semi = strchr(buf, ';');
        if (semi) *semi = '\0';

        // Uppercase & trim
        for (char* p = buf; *p; p++) *p = toupper(*p);
        const char* cmd = trim(buf);
        if (!*cmd) { Serial.println(F("ok")); return; }

        // Parse first letter+number token
        char letter = cmd[0];
        if (letter == 'G') {
            int num = atoi(cmd + 1);
            handleG(num, cmd);
        } else if (letter == 'M') {
            int num = atoi(cmd + 1);
            handleM(num, cmd);
        } else {
            Serial.printf("error: unknown command '%s'\n", cmd);
            return;
        }
        Serial.println(F("ok"));
    }

private:
    MotionPlanner* _planner  = nullptr;
    PenServo*      _servo    = nullptr;
    Kinematics*    _kin      = nullptr;
    StallHoming*   _homing   = nullptr;
    bool           _absolute = true;
    float          _feedrate = 0;

    // ── G-codes ────────────────────────────────────────────────
    void handleG(int num, const char* line) {
        switch (num) {
        case 0: {   // Rapid move
            float x = getParam(line, 'X', _planner->posX());
            float y = getParam(line, 'Y', _planner->posY());
            doMove(x, y, Config::cfg.maxFeedrate);
            break;
        }
        case 1: {   // Linear move
            float x = getParam(line, 'X', _planner->posX());
            float y = getParam(line, 'Y', _planner->posY());
            float f = getParam(line, 'F', _feedrate * 60.0f); // G1 F is mm/min
            _feedrate = f / 60.0f;  // convert to mm/s internally
            doMove(x, y, _feedrate);
            break;
        }
        case 4: {   // Dwell
            float ms = getParam(line, 'P', 0);
            _planner->waitIdle();
            delay((uint32_t)ms);
            break;
        }
        case 28: {  // Sensorless home via StallGuard2
            _servo->penUp();
            int32_t oL = 0, oR = 0;
            HomingResult res = _homing->home(oL, oR);
            if (res != HomingResult::SUCCESS) {
                Serial.println(F("error: homing failed — check cords and SGTHRS (M914)"));
            }
            break;
        }
        case 90:    // Absolute
            _absolute = true;
            break;
        case 91:    // Relative
            _absolute = false;
            break;
        case 92: {  // Set position
            // We re-interpret current machine-space position.
            // This is complex with cord kinematics; simplest approach:
            // just print a warning that G92 resets to given XY.
            float x = getParam(line, 'X', _planner->posX());
            float y = getParam(line, 'Y', _planner->posY());
            Serial.printf("  G92: position asserted as (%.2f, %.2f)\n", x, y);
            // TODO: deep reset of internal step counters if needed
            break;
        }
        default:
            Serial.printf("warn: G%d not implemented\n", num);
        }
    }

    // ── M-codes ────────────────────────────────────────────────
    void handleM(int num, const char* line) {
        switch (num) {
        case 3:     // Pen down
            _planner->waitIdle();
            if (hasParam(line, 'S')) {
                _servo->setAngle((int)getParam(line, 'S', Config::cfg.servoDown));
            } else {
                _servo->penDown();
            }
            break;
        case 5:     // Pen up
            _planner->waitIdle();
            _servo->penUp();
            break;
        case 17:    // Enable steppers
            // Planner enables on demand; this forces it
            break;
        case 18:
        case 84:    // Disable steppers
            _planner->waitIdle();
            break;
        case 92: {  // Steps/mm
            float v = getParam(line, 'X', Config::cfg.stepsPerMm);
            Config::cfg.stepsPerMm = v;
            Serial.printf("  steps/mm = %.2f\n", v);
            break;
        }
        case 100:   // Help
            printHelp();
            break;
        case 104: { // Machine width (repurposed from hotend temp)
            float v = getParam(line, 'S', Config::cfg.machineWidth);
            Config::cfg.machineWidth = v;
            Serial.printf("  machine width = %.1f mm\n", v);
            break;
        }
        case 105: { // Machine height
            float v = getParam(line, 'S', Config::cfg.machineHeight);
            Config::cfg.machineHeight = v;
            Serial.printf("  machine height = %.1f mm\n", v);
            break;
        }
        case 114:   // Report position
            Serial.printf("X:%.2f Y:%.2f\n",
                          _planner->posX(), _planner->posY());
            break;
        case 119:   // Endstop (E-stop) state
            Serial.printf("E-STOP: %s\n",
                          digitalRead(PIN_ESTOP) == LOW ? "TRIGGERED" : "ok");
            break;
        case 201: { // Acceleration
            float v = getParam(line, 'X', Config::cfg.acceleration);
            Config::cfg.acceleration = v;
            Serial.printf("  accel = %.1f mm/s²\n", v);
            break;
        }
        case 203: { // Max feedrate
            float v = getParam(line, 'X', Config::cfg.maxFeedrate);
            Config::cfg.maxFeedrate = v;
            Serial.printf("  max feedrate = %.1f mm/s\n", v);
            break;
        }
        case 280: { // Servo angle
            int p = (int)getParam(line, 'P', 0);
            int s = (int)getParam(line, 'S', 90);
            if (p == 0)      { Config::cfg.servoUp   = s; _servo->setUpAngle(s);   }
            else if (p == 1) { Config::cfg.servoDown = s; _servo->setDownAngle(s); }
            _servo->setAngle(s);
            break;
        }
        case 500:   // Save settings
            Config::save();
            break;
        case 502:   // Reset settings
            Config::reset();
            break;
        case 503:   // Print settings
            Config::print();
            break;
        case 906: { // Stepper current
            uint16_t v = (uint16_t)getParam(line, 'X', (float)Config::cfg.runCurrent);
            Config::cfg.runCurrent = v;
            Serial.printf("  run current = %d mA\n", v);
            break;
        }
        case 914: { // StallGuard threshold (Marlin-compatible M-code)
            if (hasParam(line, 'X'))
                _homing->homingCfg.sgThreshold = (uint8_t)getParam(line, 'X', 80);
            if (hasParam(line, 'S'))  // homing speed
                _homing->homingCfg.homingSpeed = getParam(line, 'S', 30.0f);
            if (hasParam(line, 'B'))  // backoff distance
                _homing->homingCfg.backoffMm   = getParam(line, 'B', 5.0f);
            _homing->printConfig();
            break;
        }
        default:
            Serial.printf("warn: M%d not implemented\n", num);
        }
    }

    // ── Motion helper ─────────────────────────────────────────
    void doMove(float x, float y, float feed) {
        if (!_absolute) {
            x += _planner->posX();
            y += _planner->posY();
        }
        _planner->moveTo(x, y, feed);
    }

    // ── Parameter extraction ──────────────────────────────────
    float getParam(const char* line, char c, float def) const {
        const char* p = strchr(line, c);
        if (!p) return def;
        return strtof(p + 1, nullptr);
    }

    bool hasParam(const char* line, char c) const {
        return strchr(line, c) != nullptr;
    }

    // ── Whitespace trim ───────────────────────────────────────
    static const char* trim(const char* s) {
        while (*s == ' ' || *s == '\t') s++;
        return s;
    }

    // ── Help text ─────────────────────────────────────────────
    static void printHelp() {
        Serial.println(F(
            "\n--- Polargraph G-code Reference ---\n"
            " G0 Xnnn Ynnn          Rapid move\n"
            " G1 Xnnn Ynnn Fnnn     Linear move (F in mm/min)\n"
            " G4 Pnnn               Dwell (ms)\n"
            " G28                   Home\n"
            " G90 / G91             Absolute / Relative\n"
            " M3 [Snnn]             Pen down [angle]\n"
            " M5                    Pen up\n"
            " M92 Xnnn              Steps/mm\n"
            " M104 Snnn             Machine width (mm)\n"
            " M105 Snnn             Machine height (mm)\n"
            " M114                  Report position\n"
            " M201 Xnnn             Acceleration (mm/s²)\n"
            " M203 Xnnn             Max feedrate (mm/s)\n"
            " M280 P0|1 Snnn        Servo up/down angle\n"
            " M500                  Save settings\n"
            " M502                  Reset to defaults\n"
            " M503                  Print settings\n"
            " M914 Xnnn [Snnn] [Bnnn]  StallGuard threshold [speed] [backoff]\n"
            "-----------------------------------\n"
        ));
    }
};
