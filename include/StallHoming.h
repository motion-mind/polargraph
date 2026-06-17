#pragma once
/**
 * StallHoming.h — Current-sensing sensorless endstops via TMC2209 StallGuard2
 *
 * How it works:
 * ─────────────
 * The TMC2209's StallGuard2 (SG2) measures back-EMF amplitude vs.
 * coil current to estimate motor load.  When the cord reaches maximum
 * extension (all cord paid out) or minimum (fully wound), the gondola
 * is effectively blocked: load spikes → SG2 result drops to 0 → we
 * treat that as an endstop trigger.
 *
 * For a polargraph we home each cord independently:
 *
 *   Phase 1 — RETRACT both cords simultaneously at low speed until
 *              each motor stalls (cords taut against anchor hardware).
 *              Record step counts as cord-zero references.
 *
 *   Phase 2 — PAY OUT each cord by a known offset distance so the
 *              gondola hangs at the defined home position.
 *
 *   Phase 3 — Assert the home coordinates in the MotionPlanner.
 *
 * StallGuard thresholds are tunable via M-code (M914) and stored in NVS.
 *
 * Important caveats:
 *   - StallGuard only works above a minimum velocity (TCOOLTHRS).
 *     Below that speed, spreadCycle must be active.  We switch modes.
 *   - Run at ≈ 20–40% of max feedrate for reliable stall detection.
 *   - After a stall detection, motors must be re-enabled before moving.
 *   - Each polargraph geometry is different; stall sensitivity (SGTHRS)
 *     needs calibration for your cord tension & motor current.
 */

#include <Arduino.h>
#include <TMCStepper.h>
#include "Config.h"
#include "Kinematics.h"
#include "StepperDriver.h"

// ── Homing config (tunable, stored in NVS) ────────────────────
struct HomingConfig {
    uint8_t  sgThreshold    = 80;    // StallGuard threshold (0–255); higher = less sensitive
    float    homingSpeed    = 30.0f; // mm/s during stall-search
    float    backoffMm      = 5.0f;  // mm to back off after stall before cord-zero
    float    homeCordOffsetL = 0.0f; // extra mm on left cord from zero to home (calibrated)
    float    homeCordOffsetR = 0.0f; // extra mm on right cord from zero to home
    uint32_t stallDebounce  = 8;     // consecutive low-SG readings before accepting stall
    bool     invertLeft     = false; // flip stall direction if motor is mirrored
    bool     invertRight    = false;
};

// ── Homing result ─────────────────────────────────────────────
enum class HomingResult {
    SUCCESS,
    STALL_TIMEOUT_L,
    STALL_TIMEOUT_R,
    DRIVER_ERROR
};

class StallHoming {
public:
    HomingConfig homingCfg;

    void begin(TMC2209Stepper* driverL, TMC2209Stepper* driverR,
               StepperDriver* drv, Kinematics* kin)
    {
        _driverL = driverL;
        _driverR = driverR;
        _drv     = drv;
        _kin     = kin;

        loadConfig();
        Serial.println(F("  StallHoming: ready (SG2 sensorless)"));
    }

    // ── Main homing sequence ──────────────────────────────────
    // Returns SUCCESS and updates offsetStepsL/R on success.
    // Call planner->assertPosition(homeX, homeY) afterwards.
    HomingResult home(int32_t& offsetStepsL, int32_t& offsetStepsR) {
        Serial.println(F("[Home] Starting StallGuard2 homing sequence"));
        _drv->enable();
        delay(100);

        // ── Switch to SpreadCycle for reliable high-load sensing ──
        // StealthChop masks the load signal; spreadCycle exposes it.
        enableStallGuard(true);
        delay(50);

        // ── Phase 1: retract both cords to stall ─────────────
        Serial.println(F("[Home] Phase 1: retracting cords to stall..."));

        bool stalledL = false, stalledR = false;
        int32_t stepsL = 0, stepsR = 0;
        const int32_t maxSteps = (int32_t)(Config::cfg.machineWidth * 1.5f
                                           * Config::cfg.stepsPerMm);
        const uint32_t stepInterval = speedToInterval(homingCfg.homingSpeed);

        uint32_t debounceL = 0, debounceR = 0;

        for (int32_t i = 0; i < maxSteps; i++) {
            if (!stalledL) {
                _drv->stepLeft(!homingCfg.invertLeft);   // retract = shorten cord
                stepsL++;
            }
            if (!stalledR) {
                _drv->stepRight(!homingCfg.invertRight);
                stepsR++;
            }
            delayMicroseconds(stepInterval);

            // Read SG result every 16 steps (UART is slow)
            if (i % 16 == 0) {
                if (!stalledL) {
                    uint16_t sg = _driverL->SG_RESULT();
                    if (sg == 0) { debounceL++; } else { debounceL = 0; }
                    if (debounceL >= homingCfg.stallDebounce) {
                        stalledL = true;
                        Serial.printf("[Home] Left stall at %d steps\n", stepsL);
                    }
                }
                if (!stalledR) {
                    uint16_t sg = _driverR->SG_RESULT();
                    if (sg == 0) { debounceR++; } else { debounceR = 0; }
                    if (debounceR >= homingCfg.stallDebounce) {
                        stalledR = true;
                        Serial.printf("[Home] Right stall at %d steps\n", stepsR);
                    }
                }
                if (stalledL && stalledR) break;
            }
        }

        if (!stalledL) { enableStallGuard(false); return HomingResult::STALL_TIMEOUT_L; }
        if (!stalledR) { enableStallGuard(false); return HomingResult::STALL_TIMEOUT_R; }

        // Brief pause — motors may have taken a false step at stall
        delay(200);

        // ── Phase 2: back off from stall ──────────────────────
        Serial.println(F("[Home] Phase 2: backing off..."));
        int32_t backoffSteps = (int32_t)(homingCfg.backoffMm * Config::cfg.stepsPerMm);
        for (int32_t i = 0; i < backoffSteps; i++) {
            _drv->stepLeft(homingCfg.invertLeft);    // pay out = lengthen cord
            _drv->stepRight(homingCfg.invertRight);
            delayMicroseconds(stepInterval * 2);     // slower on backoff
        }
        delay(100);

        // ── Phase 3: restore StealthChop ─────────────────────
        enableStallGuard(false);

        // ── Phase 4: compute cord offsets to home position ────
        // Home position cord lengths (from anchor geometry)
        float homeX, homeY;
        _kin->homePosition(homeX, homeY);
        int32_t homeCordL, homeCordR;
        _kin->cartesianToSteps(homeX, homeY, homeCordL, homeCordR);

        // Cord zero = stall point (fully retracted) + backoff.
        // The cord offsets tell us how many steps from zero = home.
        // These are stored so the planner can set its internal origin.
        int32_t zeroL = backoffSteps + (int32_t)(homingCfg.homeCordOffsetL
                                                  * Config::cfg.stepsPerMm);
        int32_t zeroR = backoffSteps + (int32_t)(homingCfg.homeCordOffsetR
                                                  * Config::cfg.stepsPerMm);

        // Steps still needed to reach home from current (post-backoff) position
        offsetStepsL = homeCordL - zeroL;
        offsetStepsR = homeCordR - zeroR;

        Serial.printf("[Home] Cord offsets to home: L=%d R=%d steps\n",
                      offsetStepsL, offsetStepsR);

        // ── Phase 5: drive to home ────────────────────────────
        Serial.println(F("[Home] Phase 5: moving to home position..."));
        driveSteps(offsetStepsL, offsetStepsR,
                   speedToInterval(homingCfg.homingSpeed));

        Serial.printf("[Home] Complete. Home = (%.1f, %.1f) mm\n", homeX, homeY);
        return HomingResult::SUCCESS;
    }

    // ── Calibration helper: run until stall, report SG readings ──
    void calibrateSensitivity() {
        Serial.println(F("[SG Cal] Move motor slowly; watch SG values. Ctrl+C to stop."));
        enableStallGuard(true);
        uint32_t iv = speedToInterval(homingCfg.homingSpeed * 0.5f);
        for (int i = 0; i < 5000; i++) {
            _drv->stepLeft(false);
            delayMicroseconds(iv);
            if (i % 64 == 0) {
                Serial.printf("  SG L=%4u  SG R=%4u\n",
                    _driverL->SG_RESULT(), _driverR->SG_RESULT());
            }
        }
        enableStallGuard(false);
    }

    // ── Read live SG values (for web dashboard) ───────────────
    uint16_t sgLeft()  { return _driverL ? _driverL->SG_RESULT() : 0; }
    uint16_t sgRight() { return _driverR ? _driverR->SG_RESULT() : 0; }
    bool     isStallL(){ return sgLeft()  == 0; }
    bool     isStallR(){ return sgRight() == 0; }

    // ── NVS persistence ───────────────────────────────────────
    void saveConfig() {
        Preferences p; p.begin("homing", false);
        p.putUChar("sgt",  homingCfg.sgThreshold);
        p.putFloat("spd",  homingCfg.homingSpeed);
        p.putFloat("bof",  homingCfg.backoffMm);
        p.putFloat("ofl",  homingCfg.homeCordOffsetL);
        p.putFloat("ofr",  homingCfg.homeCordOffsetR);
        p.putUInt("dbn",   homingCfg.stallDebounce);
        p.end();
        Serial.println(F("ok — homing config saved"));
    }

    void loadConfig() {
        Preferences p; p.begin("homing", true);
        homingCfg.sgThreshold     = p.getUChar("sgt", 80);
        homingCfg.homingSpeed     = p.getFloat("spd", 30.0f);
        homingCfg.backoffMm       = p.getFloat("bof", 5.0f);
        homingCfg.homeCordOffsetL = p.getFloat("ofl", 0.0f);
        homingCfg.homeCordOffsetR = p.getFloat("ofr", 0.0f);
        homingCfg.stallDebounce   = p.getUInt("dbn",  8);
        p.end();
    }

    void printConfig() {
        Serial.printf("  SG threshold  : %d\n",   homingCfg.sgThreshold);
        Serial.printf("  Homing speed  : %.1f mm/s\n", homingCfg.homingSpeed);
        Serial.printf("  Back-off dist : %.1f mm\n",   homingCfg.backoffMm);
        Serial.printf("  Cord offset L : %.1f mm\n",   homingCfg.homeCordOffsetL);
        Serial.printf("  Cord offset R : %.1f mm\n",   homingCfg.homeCordOffsetR);
        Serial.printf("  Stall debounce: %d reads\n",  homingCfg.stallDebounce);
    }

private:
    TMC2209Stepper* _driverL = nullptr;
    TMC2209Stepper* _driverR = nullptr;
    StepperDriver*  _drv     = nullptr;
    Kinematics*     _kin     = nullptr;

    // Switch TMC2209 between StealthChop (quiet) and SpreadCycle (SG-visible)
    void enableStallGuard(bool on) {
        if (!_driverL) return;
        if (on) {
            // SpreadCycle: load is visible in sg_result
            _driverL->en_spreadCycle(true);
            _driverR->en_spreadCycle(true);
            // TCOOLTHRS: SG active above this velocity threshold
            // Value in internal clock ticks; 200 ≈ reasonable for ~30mm/s
            _driverL->TCOOLTHRS(200);
            _driverR->TCOOLTHRS(200);
            _driverL->SGTHRS(homingCfg.sgThreshold);
            _driverR->SGTHRS(homingCfg.sgThreshold);
        } else {
            // Restore StealthChop for quiet plotting
            _driverL->en_spreadCycle(false);
            _driverR->en_spreadCycle(false);
            _driverL->TCOOLTHRS(0);
            _driverR->TCOOLTHRS(0);
            _driverL->pwm_autoscale(true);
            _driverR->pwm_autoscale(true);
        }
    }

    // µs per step → speed in mm/s
    uint32_t speedToInterval(float mmPerSec) const {
        if (mmPerSec <= 0) mmPerSec = 10.0f;
        return (uint32_t)(1e6f / (mmPerSec * Config::cfg.stepsPerMm));
    }

    // Synchronised multi-step drive (Bresenham, blocking) for homing moves
    // stepsL/stepsR are signed: positive = pay out, negative = retract
    void driveSteps(int32_t stepsL, int32_t stepsR, uint32_t interval) {
        int32_t absL = abs(stepsL), absR = abs(stepsR);
        int32_t dominant = max(absL, absR);
        if (dominant == 0) return;

        bool dirL = stepsL >= 0;
        bool dirR = stepsR >= 0;
        int32_t errL = dominant / 2;
        int32_t errR = dominant / 2;

        for (int32_t i = 0; i < dominant; i++) {
            errL -= absL;
            if (errL < 0) { errL += dominant; _drv->stepLeft(dirL); }
            errR -= absR;
            if (errR < 0) { errR += dominant; _drv->stepRight(dirR); }
            delayMicroseconds(interval);
        }
    }
};
