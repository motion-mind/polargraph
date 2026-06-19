#include "MotionPlanner.h"

// ── Static member definitions ─────────────────────────────────
// These must be in a .cpp to get a single definition.
// IRAM_ATTR on data places them in IRAM (fast access from ISR).
MotionPlanner* IRAM_ATTR MotionPlanner::_instance     = nullptr;
volatile QueueSeg IRAM_ATTR MotionPlanner::_isrSeg    = {};
volatile bool     IRAM_ATTR MotionPlanner::_isrActive  = false;
volatile uint32_t IRAM_ATTR MotionPlanner::_isrDone    = 0;
volatile int32_t  IRAM_ATTR MotionPlanner::_isrErrL    = 0;
volatile int32_t  IRAM_ATTR MotionPlanner::_isrErrR    = 0;
volatile uint32_t IRAM_ATTR MotionPlanner::_isrIntervalFP = 0;

// ── eStop ─────────────────────────────────────────────────────
IRAM_ATTR void MotionPlanner::eStop() {
    noInterrupts();
    _head = _tail = 0;
    _executing = false;
    interrupts();
    _drv->disable();
    if (_servo) _servo->penUp();
    Serial.println(F("!! ESTOP"));
}

// ── Hardware timer ISR ────────────────────────────────────────
// All IRAM_ATTR functions that are called from the ISR (stepLeft,
// stepRight, penDown, penUp) must also be defined in .cpp files —
// see StepperDriver.cpp.  PenServo methods are NOT IRAM_ATTR because
// servo moves happen infrequently and involve UART/I2C delays anyway.
void IRAM_ATTR MotionPlanner::timerISR() {
    MotionPlanner* self = _instance;
    if (!self) return;

    // ── Load next segment if idle ──────────────────────────────
    if (!_isrActive) {
        if (self->_head == self->_tail) {
            timerAlarmWrite(self->_timer, 5000, true);
            self->_executing = false;
            return;
        }
        memcpy((void*)&_isrSeg,
               (const void*)&self->_queue[self->_tail],
               sizeof(QueueSeg));
        self->_tail = (self->_tail + 1) % QUEUE_SIZE;
        _isrDone   = 0;
        _isrActive = true;
        self->_executing = true;

        // Tool change: select new pen (raises current, does not lower yet)
        if ((_isrSeg.flags & PLG_TOOL_CHANGE) && self->_servo)
            self->_servo->selectPen(_isrSeg.penIndex);
        // Pen down at segment start
        if ((_isrSeg.flags & PLG_PEN_DOWN) && self->_servo)
            self->_servo->penDown();

        _isrErrL = (int32_t)(_isrSeg.dominant >> 1);
        _isrErrR = (int32_t)(_isrSeg.dominant >> 1);
        _isrIntervalFP = (uint32_t)_isrSeg.entryInterval << 16;
        timerAlarmWrite(self->_timer, _isrSeg.entryInterval, true);
        return;
    }

    // ── Segment complete? ──────────────────────────────────────
    if (_isrDone >= _isrSeg.dominant) {
        if ((_isrSeg.flags & PLG_PEN_UP) && self->_servo)
            self->_servo->penUp();
        _isrActive       = false;
        self->_executing = false;
        timerAlarmWrite(self->_timer, 5000, true);
        return;
    }

    // ── Velocity interpolation ─────────────────────────────────
    uint32_t interval;
    if (_isrDone < _isrSeg.accelSteps && _isrSeg.accelSteps > 0) {
        uint32_t delta = (_isrSeg.entryInterval > _isrSeg.cruiseInterval)
            ? _isrSeg.entryInterval - _isrSeg.cruiseInterval : 0;
        interval = _isrSeg.entryInterval
                   - (uint32_t)(((uint64_t)delta * _isrDone) / _isrSeg.accelSteps);
    } else {
        uint32_t decelStart = _isrSeg.dominant - _isrSeg.decelSteps;
        if (_isrSeg.decelSteps > 0 && _isrDone >= decelStart) {
            uint32_t d     = _isrDone - decelStart;
            uint32_t delta = (_isrSeg.exitInterval > _isrSeg.cruiseInterval)
                ? _isrSeg.exitInterval - _isrSeg.cruiseInterval : 0;
            interval = _isrSeg.cruiseInterval
                       + (uint32_t)(((uint64_t)delta * d) / _isrSeg.decelSteps);
        } else {
            interval = _isrSeg.cruiseInterval;
        }
    }
    if (interval < 20)    interval = 20;
    if (interval > 65535) interval = 65535;
    timerAlarmWrite(self->_timer, interval, true);

    // ── Bresenham step ─────────────────────────────────────────
    bool doL = false, doR = false;
    int32_t dom = (int32_t)_isrSeg.dominant;

    _isrErrL -= (_isrSeg.deltaL < 0 ? -_isrSeg.deltaL : _isrSeg.deltaL);
    if (_isrErrL < 0) { _isrErrL += dom; doL = true; }
    _isrErrR -= (_isrSeg.deltaR < 0 ? -_isrSeg.deltaR : _isrSeg.deltaR);
    if (_isrErrR < 0) { _isrErrR += dom; doR = true; }

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
