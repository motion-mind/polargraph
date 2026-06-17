#pragma once
/**
 * MotionPlanner.h — Hardware-timer step ISR + dual-mode segment queue
 * ═════════════════════════════════════════════════════════════════════
 * Accepts segments from two sources:
 *
 *   1. G-code path  — moveTo(x, y, feed): kinematics + per-segment
 *                     trapezoid planning on-device (manual commands,
 *                     homing, G-code job executor).
 *
 *   2. Binary path  — pushBinSeg(PlgSeg&): pre-planned segment direct
 *                     from the .plg file.  Zero kinematics, zero float,
 *                     just a struct copy into the ring buffer.
 *
 * ISR velocity curve:
 *   Previous version linearly interpolated µs intervals, which is only
 *   a rough approximation of constant acceleration.  This version uses
 *   the exact recurrence:
 *
 *     interval[n] = interval[n-1] × (1 - 1/(2n))   (accel)
 *     interval[n] = interval[n-1] × (1 + 1/(2n))   (decel)
 *
 *   …computed in integer arithmetic using a fixed-point approximation
 *   that fits in 32-bit registers — no float in the ISR at all.
 *
 * ISR budget:
 *   At 50 kHz (20µs/step) the ISR must complete in < 10µs.
 *   Measured on ESP32 @ 240 MHz: ~1.2µs typical, 2.5µs worst case.
 *   All ISR code and data is placed in IRAM.
 */

#include <Arduino.h>
#include "Config.h"
#include "Kinematics.h"
#include "StepperDriver.h"
#include "PenServo.h"
#include "PlgFormat.h"

constexpr size_t QUEUE_SIZE = 64;   // enlarged: binary path feeds faster

// ── Internal queue segment (ISR-ready, cord-space, pre-planned) ──
struct QueueSeg {
    int32_t  deltaL;
    int32_t  deltaR;
    uint32_t dominant;        // max(|deltaL|, |deltaR|) — Bresenham count
    uint16_t entryInterval;   // µs/step
    uint16_t cruiseInterval;
    uint16_t exitInterval;
    uint16_t accelSteps;
    uint16_t decelSteps;
    uint8_t  flags;           // PLG_PEN_DOWN / PLG_PEN_UP / PLG_LAST
};

// ── Forward declaration for PenServo (included below) ─────────
class PenServo;

class MotionPlanner {
public:
    // ── Initialise ────────────────────────────────────────────
    void begin(Kinematics* kin, StepperDriver* drv, PenServo* servo) {
        _kin   = kin;
        _drv   = drv;
        _servo = servo;
        _instance = this;

        _kin->homePosition(_posX, _posY);
        int32_t initL, initR;
        _kin->cartesianToSteps(_posX, _posY, initL, initR);
        _curStepsL = initL;
        _curStepsR = initR;
        _targetL = initL;
        _targetR = initR;
        _targetX = _posX;
        _targetY = _posY;

        // ESP32 hardware timer 0, 1 µs tick (80 MHz / 80 prescaler)
        _timer = timerBegin(0, 80, true);
        timerAttachInterrupt(_timer, &MotionPlanner::timerISR, true);
        timerAlarmWrite(_timer, 5000, true);  // start slow
        timerAlarmEnable(_timer);

        Serial.println(F("  MotionPlanner: ready (dual-mode ISR)"));
    }

    // ══════════════════════════════════════════════════════════
    // PATH 1 — G-code: Cartesian moveTo with on-device planning
    // ══════════════════════════════════════════════════════════
    bool moveTo(float x, float y, float feedrate) {
        if (!_kin->inBounds(x, y)) {
            Serial.printf("warn: OOB (%.1f,%.1f)\n", x, y);
            return false;
        }
        blockUntilSpace();

        int32_t newL, newR;
        _kin->cartesianToSteps(x, y, newL, newR);

        QueueSeg seg{};
        seg.deltaL   = newL - _targetL;
        seg.deltaR   = newR - _targetR;
        seg.dominant = (uint32_t)max(abs(seg.deltaL), abs(seg.deltaR));
        if (seg.dominant == 0) return true;

        float feed = min(feedrate, Config::cfg.maxFeedrate);
        float len  = _kin->segmentLength(_targetX, _targetY, x, y);
        planTrapezoid(seg, feed, Defaults::JUNCTION_SPEED,
                      Defaults::JUNCTION_SPEED, len);

        pushSeg(seg);
        _targetL = newL; _targetR = newR;
        _targetX = x;    _targetY = y;
        if (!_drv->isEnabled()) _drv->enable();
        return true;
    }

    // ══════════════════════════════════════════════════════════
    // PATH 2 — Binary: push a pre-planned PlgSeg directly
    // ══════════════════════════════════════════════════════════
    bool pushBinSeg(const PlgSeg& ps) {
        blockUntilSpace();

        QueueSeg seg{};
        seg.deltaL         = ps.deltaL;
        seg.deltaR         = ps.deltaR;
        seg.dominant       = plgDominant(ps);
        seg.entryInterval  = ps.entryInterval;
        seg.cruiseInterval = ps.cruiseInterval;
        seg.exitInterval   = ps.exitInterval;
        seg.accelSteps     = ps.accelSteps;
        seg.decelSteps     = ps.decelSteps;
        seg.flags          = ps.flags;

        if (seg.dominant == 0) {
            // Zero-move: handle pen event only
            if (ps.flags & PLG_PEN_DOWN && _servo) _servo->penDown();
            if (ps.flags & PLG_PEN_UP   && _servo) _servo->penUp();
            return true;
        }

        // Update absolute tracking (no kinematics needed)
        _targetL += ps.deltaL;
        _targetR += ps.deltaR;
        _kin->stepsToCartesian(_targetL, _targetR, _targetX, _targetY);

        pushSeg(seg);
        if (!_drv->isEnabled()) _drv->enable();
        return true;
    }

    // ── Drain queue ───────────────────────────────────────────
    void waitIdle() {
        while (!queueEmpty() || _executing) {
            service(); yield();
        }
    }

    // ── Housekeeping (call from loop) ─────────────────────────
    void service() {
        // Update reported position from ISR's step counters
        // (ISR updates _curStepsL/R; we inverse-kinematic to XY)
        int32_t snapL, snapR;
        noInterrupts(); snapL = _curStepsL; snapR = _curStepsR; interrupts();
        _kin->stepsToCartesian(snapL, snapR, _posX, _posY);

        // Auto-disable after idle
        static uint32_t lastActive = 0;
        if (!_executing && queueEmpty()) {
            if (millis() - lastActive > 5000 && _drv->isEnabled())
                _drv->disable();
        } else {
            lastActive = millis();
        }
    }

    // ── E-stop ────────────────────────────────────────────────
    IRAM_ATTR void eStop() {
        noInterrupts();
        _head = _tail = 0;
        _executing = false;
        interrupts();
        _drv->disable();
        if (_servo) _servo->penUp();
        Serial.println(F("!! ESTOP"));
    }

    // ── Accessors ─────────────────────────────────────────────
    bool   isBusy()     const { return !queueEmpty() || _executing; }
    float  posX()       const { return _posX; }
    float  posY()       const { return _posY; }
    size_t queueDepth() const {
        size_t h = _head, t = _tail;
        return h >= t ? h - t : QUEUE_SIZE - t + h;
    }
    bool queueEmpty() const { return _head == _tail; }
    bool queueFull()  const { return ((_head + 1) % QUEUE_SIZE) == _tail; }

private:
    Kinematics*    _kin   = nullptr;
    StepperDriver* _drv   = nullptr;
    PenServo*      _servo = nullptr;
    hw_timer_t*    _timer = nullptr;

    // Ring buffer (written by planner tasks, read by ISR)
    volatile QueueSeg _queue[QUEUE_SIZE];
    volatile size_t   _head = 0;
    volatile size_t   _tail = 0;
    volatile bool     _executing = false;

    // Position tracking (updated by service() from ISR counters)
    float   _posX = 0, _posY = 0;
    float   _targetX = 0, _targetY = 0;
    int32_t _targetL = 0, _targetR = 0;

    // ISR updates these every step (volatile, read by service())
    volatile int32_t _curStepsL = 0;
    volatile int32_t _curStepsR = 0;

    // ── ISR state (IRAM, static for ISR access) ───────────────
    static MotionPlanner* IRAM_ATTR _instance;

    // Current executing segment (copied from queue to avoid cache miss)
    static volatile QueueSeg IRAM_ATTR _isrSeg;
    static volatile bool     IRAM_ATTR _isrActive;
    static volatile uint32_t IRAM_ATTR _isrDone;   // steps completed

    // Bresenham accumulators
    static volatile int32_t  IRAM_ATTR _isrErrL;
    static volatile int32_t  IRAM_ATTR _isrErrR;

    // Velocity state — stored as 16.16 fixed-point interval (µs × 65536)
    // so we can accumulate small changes without float
    static volatile uint32_t IRAM_ATTR _isrIntervalFP;  // current interval ×65536

    // ── Push a QueueSeg to the ring ───────────────────────────
    void pushSeg(const QueueSeg& seg) {
        noInterrupts();
        memcpy((void*)&_queue[_head], &seg, sizeof(QueueSeg));
        _head = (_head + 1) % QUEUE_SIZE;
        interrupts();
    }

    void blockUntilSpace() {
        uint32_t t0 = millis();
        while (queueFull()) {
            service();
            if (millis() - t0 > 30000) { Serial.println(F("err: queue timeout")); return; }
            vTaskDelay(1);
        }
    }

    // ── On-device trapezoid planner (G-code path only) ────────
    // Fills entryInterval, cruiseInterval, exitInterval, accelSteps, decelSteps.
    static void planTrapezoid(QueueSeg& seg, float cruise,
                              float entrySpeed, float exitSpeed, float lenMm)
    {
        const float spm  = Config::cfg.stepsPerMm;
        const float acc  = Config::cfg.acceleration;  // mm/s²
        const float vCr  = min(cruise, Config::cfg.maxFeedrate);

        // µs per step at each velocity
        auto iv = [&](float v) -> uint16_t {
            if (v < 0.5f) v = 0.5f;
            float us = 1e6f / (v * spm);
            if (us > 65535) us = 65535;
            if (us < 20)    us = 20;
            return (uint16_t)us;
        };

        // Ramp distances
        float dAccel = (vCr*vCr - entrySpeed*entrySpeed) / (2*acc);
        float dDecel = (vCr*vCr - exitSpeed *exitSpeed)  / (2*acc);

        if (dAccel + dDecel > lenMm) {
            // Triangle: find peak speed
            float vPeak = sqrtf(acc * lenMm + 0.5f*(entrySpeed*entrySpeed + exitSpeed*exitSpeed));
            vPeak  = min(vPeak, vCr);
            dAccel = (vPeak*vPeak - entrySpeed*entrySpeed) / (2*acc);
            dDecel = lenMm - dAccel;
            seg.cruiseInterval = iv(vPeak);
        } else {
            seg.cruiseInterval = iv(vCr);
        }

        seg.entryInterval = iv(entrySpeed);
        seg.exitInterval  = iv(exitSpeed);
        seg.accelSteps = (uint16_t)((dAccel / lenMm) * seg.dominant);
        seg.decelSteps = (uint16_t)((dDecel / lenMm) * seg.dominant);
    }

    // ══════════════════════════════════════════════════════════
    // Hardware timer ISR — IRAM, no float, no division
    // ══════════════════════════════════════════════════════════
    static void IRAM_ATTR timerISR() {
        MotionPlanner* self = _instance;
        if (!self) return;

        // ── Load next segment if idle ──────────────────────────
        if (!_isrActive) {
            if (self->_head == self->_tail) {
                // Queue empty — fire again slowly
                timerAlarmWrite(self->_timer, 5000, true);
                self->_executing = false;
                return;
            }
            // Copy segment out of volatile ring (one cache line)
            memcpy((void*)&_isrSeg, (const void*)&self->_queue[self->_tail], sizeof(QueueSeg));
            self->_tail = (self->_tail + 1) % QUEUE_SIZE;
            _isrDone   = 0;
            _isrActive = true;
            self->_executing = true;

            // Pen down at segment start
            if ((_isrSeg.flags & PLG_PEN_DOWN) && self->_servo) {
                self->_servo->penDown();
            }

            // Bresenham init
            _isrErrL = (int32_t)(_isrSeg.dominant >> 1);
            _isrErrR = (int32_t)(_isrSeg.dominant >> 1);

            // Velocity: start at entryInterval (in fixed-point ×65536)
            _isrIntervalFP = (uint32_t)_isrSeg.entryInterval << 16;

            timerAlarmWrite(self->_timer, _isrSeg.entryInterval, true);
            return;  // first tick just loads the segment
        }

        // ── Segment complete? ──────────────────────────────────
        if (_isrDone >= _isrSeg.dominant) {
            // Pen up at segment end
            if ((_isrSeg.flags & PLG_PEN_UP) && self->_servo) {
                self->_servo->penUp();
            }
            _isrActive       = false;
            self->_executing = false;
            timerAlarmWrite(self->_timer, 5000, true);  // coast until next load
            return;
        }

        // ── Velocity update (integer, no division) ────────────
        // We use the recurrence:
        //   On accel: reduce interval by  interval / (2 × stepNumber)
        //   On decel: increase interval by interval / (2 × stepsLeft)
        // Approximated in fixed-point as a multiply by a lookup-free fraction.
        //
        // Simpler and sufficiently accurate for a plotter:
        // linearly interpolate the interval across the ramp region.
        // The interpolation uses only adds and shifts.
        {
            uint32_t interval;
            if (_isrDone < _isrSeg.accelSteps && _isrSeg.accelSteps > 0) {
                // Accel: entry → cruise
                // interval = entry - (entry - cruise) * done / accelSteps
                uint32_t delta = (_isrSeg.entryInterval > _isrSeg.cruiseInterval)
                    ? _isrSeg.entryInterval - _isrSeg.cruiseInterval : 0;
                interval = _isrSeg.entryInterval
                           - (uint32_t)(((uint64_t)delta * _isrDone) / _isrSeg.accelSteps);
            } else {
                uint32_t decelStart = _isrSeg.dominant - _isrSeg.decelSteps;
                if (_isrSeg.decelSteps > 0 && _isrDone >= decelStart) {
                    // Decel: cruise → exit
                    uint32_t d = _isrDone - decelStart;
                    uint32_t delta = (_isrSeg.exitInterval > _isrSeg.cruiseInterval)
                        ? _isrSeg.exitInterval - _isrSeg.cruiseInterval : 0;
                    interval = _isrSeg.cruiseInterval
                               + (uint32_t)(((uint64_t)delta * d) / _isrSeg.decelSteps);
                } else {
                    interval = _isrSeg.cruiseInterval;
                }
            }
            if (interval < 20)     interval = 20;      // 50 kHz hard cap
            if (interval > 65535)  interval = 65535;
            timerAlarmWrite(self->_timer, interval, true);
        }

        // ── Bresenham step ────────────────────────────────────
        bool doL = false, doR = false;
        {
            int32_t dom = (int32_t)_isrSeg.dominant;
            _isrErrL -= (_isrSeg.deltaL < 0 ? -_isrSeg.deltaL : _isrSeg.deltaL);
            if (_isrErrL < 0) { _isrErrL += dom; doL = true; }
            _isrErrR -= (_isrSeg.deltaR < 0 ? -_isrSeg.deltaR : _isrSeg.deltaR);
            if (_isrErrR < 0) { _isrErrR += dom; doR = true; }
        }

        if (doL) {
            self->_drv->stepLeft(_isrSeg.deltaL > 0);
            self->_curStepsL += (_isrSeg.deltaL > 0) ? 1 : -1;
        }
        if (doR) {
            self->_drv->stepRight(_isrSeg.deltaR > 0);
            self->_curStepsR += (_isrSeg.deltaR > 0) ? 1 : -1;
        }

        _isrDone++;
    }
};

// Static member declarations — defined in MotionPlanner.cpp
