#pragma once
/**
 * GCodeParser.h — G-code interpreter, multi-pen branch
 *
 * New / changed commands vs single-pen:
 *   T0 / T1 / T2 / T3        Select active pen (raises current, selects new)
 *   M3  [Snnn]               Pen down (active pen)
 *   M5                       Pen up   (active pen)
 *   M7                       All pens up (emergency)
 *   M218 Tnnn Xnnn Ynnn      Set tool offset for pen n
 *   M280 Tnnn P0|1 Snnn      Servo angle: T=pen, P0=up angle, P1=down angle
 *   M503                     Print all settings including per-pen config
 *
 * Everything else unchanged from single-pen version.
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
        _planner   = planner;
        _servo     = servo;
        _kin       = kin;
        _homing    = homing;
        _absolute  = true;
        _feedrate  = Config::cfg.maxFeedrate;
        _activePen = 0;
        Serial.println(F("  GCodeParser: ready (multi-pen)"));
    }

    void process(const char* line) {
        static char buf[128];
        strncpy(buf, line, sizeof(buf)-1);
        buf[sizeof(buf)-1] = '\0';
        char* semi = strchr(buf, ';');
        if (semi) *semi = '\0';
        for (char* p = buf; *p; p++) *p = toupper((unsigned char)*p);
        const char* cmd = trim(buf);
        if (!*cmd) { Serial.println(F("ok")); return; }

        char letter = cmd[0];
        if (letter == 'T') {
            // Tool selection: T0 T1 T2 T3
            int t = atoi(cmd + 1);
            handleToolChange(t);
        } else if (letter == 'G') {
            handleG(atoi(cmd + 1), cmd);
        } else if (letter == 'M') {
            handleM(atoi(cmd + 1), cmd);
        } else {
            Serial.printf("error: unknown '%s'\n", cmd);
            return;
        }
        Serial.println(F("ok"));
    }

private:
    MotionPlanner* _planner   = nullptr;
    PenServo*      _servo     = nullptr;
    Kinematics*    _kin       = nullptr;
    StallHoming*   _homing    = nullptr;
    bool           _absolute  = true;
    float          _feedrate  = 0;
    int            _activePen = 0;

    // ── Tool change ───────────────────────────────────────────
    void handleToolChange(int t) {
        if (t < 0 || t >= NUM_PENS) {
            Serial.printf("error: T%d out of range (0-%d)\n", t, NUM_PENS-1);
            return;
        }
        if (!Config::cfg.pens[t].enabled) {
            Serial.printf("warn: pen %d is disabled\n", t);
        }
        _planner->waitIdle();
        _servo->selectPen(t);
        _activePen = t;
        Serial.printf("  Tool: pen %d selected  offset=(%.1f,%.1f)mm\n",
            t,
            Config::cfg.pens[t].offsetX,
            Config::cfg.pens[t].offsetY);
    }

    // ── G-codes ───────────────────────────────────────────────
    void handleG(int num, const char* line) {
        switch (num) {
        case 0: {
            float x = getParam(line,'X', effectivePos('X'));
            float y = getParam(line,'Y', effectivePos('Y'));
            doMove(x, y, Config::cfg.maxFeedrate);
            break;
        }
        case 1: {
            float x = getParam(line,'X', effectivePos('X'));
            float y = getParam(line,'Y', effectivePos('Y'));
            float f = getParam(line,'F', _feedrate * 60.0f);
            _feedrate = f / 60.0f;
            doMove(x, y, _feedrate);
            break;
        }
        case 4:
            _planner->waitIdle();
            delay((uint32_t)getParam(line,'P',0));
            break;
        case 28: {
            _servo->allUp();
            int32_t oL=0, oR=0;
            HomingResult res = _homing->home(oL, oR);
            if (res != HomingResult::SUCCESS)
                Serial.println(F("error: homing failed"));
            break;
        }
        case 90: _absolute = true;  break;
        case 91: _absolute = false; break;
        case 92:
            Serial.printf("  G92: position asserted (%.2f, %.2f)\n",
                getParam(line,'X',_planner->posX()),
                getParam(line,'Y',_planner->posY()));
            break;
        default:
            Serial.printf("warn: G%d not implemented\n", num);
        }
    }

    // ── M-codes ───────────────────────────────────────────────
    void handleM(int num, const char* line) {
        switch (num) {
        case 3:  // Pen down (active pen)
            _planner->waitIdle();
            if (hasParam(line,'S'))
                _servo->setAngle(_activePen, (int)getParam(line,'S',90));
            else
                _servo->penDown();
            break;
        case 5:  // Pen up (active pen)
            _planner->waitIdle();
            _servo->penUp();
            break;
        case 7:  // All pens up (emergency)
            _planner->waitIdle();
            _servo->allUp();
            break;
        case 92: {
            float v = getParam(line,'X', Config::cfg.stepsPerMm);
            Config::cfg.stepsPerMm = v;
            Serial.printf("  steps/mm = %.2f\n", v);
            break;
        }
        case 100: printHelp(); break;
        case 104: {
            float v = getParam(line,'S', Config::cfg.machineWidth);
            Config::cfg.machineWidth = v;
            Serial.printf("  machine width = %.1f mm\n", v);
            break;
        }
        case 105: {
            float v = getParam(line,'S', Config::cfg.machineHeight);
            Config::cfg.machineHeight = v;
            Serial.printf("  machine height = %.1f mm\n", v);
            break;
        }
        case 114:
            Serial.printf("X:%.2f Y:%.2f  Pen:%d\n",
                _planner->posX(), _planner->posY(), _activePen);
            break;
        case 201: {
            float v = getParam(line,'X', Config::cfg.acceleration);
            Config::cfg.acceleration = v;
            Serial.printf("  accel = %.1f mm/s²\n", v);
            break;
        }
        case 203: {
            float v = getParam(line,'X', Config::cfg.maxFeedrate);
            Config::cfg.maxFeedrate = v;
            Serial.printf("  max feedrate = %.1f mm/s\n", v);
            break;
        }
        case 218: {
            // M218 T<pen> X<offsetX> Y<offsetY>
            int t = (int)getParam(line,'T', (float)_activePen);
            if (t >= 0 && t < NUM_PENS) {
                if (hasParam(line,'X')) Config::cfg.pens[t].offsetX = getParam(line,'X',0);
                if (hasParam(line,'Y')) Config::cfg.pens[t].offsetY = getParam(line,'Y',0);
                Serial.printf("  Pen %d offset = (%.2f, %.2f) mm\n",
                    t, Config::cfg.pens[t].offsetX, Config::cfg.pens[t].offsetY);
            }
            break;
        }
        case 280: {
            // M280 T<pen> P0|1 S<angle>
            int t = (int)getParam(line,'T', (float)_activePen);
            int p = (int)getParam(line,'P', 0);
            int s = (int)getParam(line,'S', 90);
            if (t >= 0 && t < NUM_PENS) {
                if (p == 0) { Config::cfg.pens[t].servoUp   = s; }
                else        { Config::cfg.pens[t].servoDown  = s; }
                _servo->setAngle(t, s);
            }
            break;
        }
        case 500: Config::save();  break;
        case 502: Config::reset(); break;
        case 503: Config::print(); break;
        case 906: {
            uint16_t v = (uint16_t)getParam(line,'X',(float)Config::cfg.runCurrent);
            Config::cfg.runCurrent = v;
            Serial.printf("  run current = %d mA\n", v);
            break;
        }
        case 914: {
            if (hasParam(line,'X')) _homing->homingCfg.sgThreshold = (uint8_t)getParam(line,'X',80);
            if (hasParam(line,'S')) _homing->homingCfg.homingSpeed = getParam(line,'S',30.0f);
            if (hasParam(line,'B')) _homing->homingCfg.backoffMm   = getParam(line,'B',5.0f);
            _homing->printConfig();
            break;
        }
        default:
            Serial.printf("warn: M%d not implemented\n", num);
        }
    }

    // ── Motion helper (applies active pen's tool offset) ──────
    void doMove(float x, float y, float feed) {
        if (!_absolute) { x += _planner->posX(); y += _planner->posY(); }
        // Apply tool offset so the correct pen tip lands at (x,y)
        float tx = x + Config::cfg.pens[_activePen].offsetX;
        float ty = y + Config::cfg.pens[_activePen].offsetY;
        _planner->moveTo(tx, ty, feed);
    }

    // Current effective position corrected for active pen offset
    float effectivePos(char axis) const {
        if (axis == 'X')
            return _planner->posX() - Config::cfg.pens[_activePen].offsetX;
        return _planner->posY() - Config::cfg.pens[_activePen].offsetY;
    }

    float getParam(const char* line, char c, float def) const {
        const char* p = strchr(line, c);
        if (!p) return def;
        return strtof(p+1, nullptr);
    }
    bool hasParam(const char* line, char c) const {
        return strchr(line, c) != nullptr;
    }
    static const char* trim(const char* s) {
        while (*s==' '||*s=='\t') s++;
        return s;
    }

    static void printHelp() {
        Serial.println(F(
            "\n--- Polargraph Multi-Pen G-code ---\n"
            " T0 / T1 / T2 / T3    Select pen\n"
            " G0 Xnnn Ynnn         Rapid move\n"
            " G1 Xnnn Ynnn Fnnn    Linear move (F=mm/min)\n"
            " G4 Pnnn              Dwell (ms)\n"
            " G28                  Home (sensorless)\n"
            " G90 / G91            Absolute / Relative\n"
            " M3  [Snnn]           Pen down\n"
            " M5                   Pen up\n"
            " M7                   All pens up\n"
            " M114                 Report position + active pen\n"
            " M218 Tnnn Xnnn Ynnn  Set tool offset\n"
            " M280 Tnnn P0|1 Snnn  Servo angle\n"
            " M500 / M502 / M503   Save / Reset / Print settings\n"
            " M914 [Xnnn][Snnn][Bnnn] StallGuard config\n"
            "-----------------------------------\n"
        ));
    }
};
