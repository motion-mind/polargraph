/**
 * Polargraph Firmware v3.0 — Fysetc E4 (ESP32 + TMC2209)
 * ════════════════════════════════════════════════════════
 * Two job execution paths:
 *   .plg  — binary pre-planned segments, read by BinJobExecutor (Core 0)
 *   .gcode — text G-code,               read by JobExecutor     (Core 0)
 *
 * File type detected from magic bytes / extension on upload.
 * Manual G-code commands always available via serial or /gcode endpoint.
 *
 * Shared job state globals (volatile, accessed from Core 0 + Core 1):
 *   gJobRunning, gJobPaused, gJobStop  — control flags
 *   gJobLinesDone, gJobLinesTotal      — progress
 *   gJobFile, gJobType                 — current job identity
 */

#include <Arduino.h>
#include "Config.h"
#include "Kinematics.h"
#include "StepperDriver.h"
#include "PenServo.h"
#include "MotionPlanner.h"
#include "GCodeParser.h"
#include "StallHoming.h"
#include "JobExecutor.h"
#include "BinJobExecutor.h"
#include "WiFiManager.h"

// ── Shared job state ──────────────────────────────────────────
volatile bool     gJobRunning    = false;
volatile bool     gJobPaused     = false;
volatile bool     gJobStop       = false;
volatile uint32_t gJobLinesDone  = 0;
volatile uint32_t gJobLinesTotal = 0;
volatile char     gJobFile[64]   = "";
volatile char     gJobType[8]    = "";   // "gcode" or "plg"

// ── Subsystems ────────────────────────────────────────────────
Kinematics     kinematics;
StepperDriver  steppers;
PenServo       penServo;
MotionPlanner  planner;
GCodeParser    gcodeParser;
StallHoming    stallHoming;
JobExecutor    jobExec;
BinJobExecutor binExec;
WiFiManager    wifiMgr;

// ── Serial line accumulator ───────────────────────────────────
static void processSerial() {
    static char    buf[128];
    static uint8_t pos = 0;
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (pos > 0) { buf[pos] = '\0'; gcodeParser.process(buf); pos = 0; }
        } else if (pos < sizeof(buf)-1) {
            buf[pos++] = c;
        }
    }
}

// ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(250000);
    delay(300);
    Serial.println(F("\n== Polargraph v3.0 — Fysetc E4 =="));

    Config::load();
    Config::print();

    penServo.begin(PIN_SERVO, SERVO_UP_ANGLE, SERVO_DOWN_ANGLE);
    steppers.begin();
    planner.begin(&kinematics, &steppers, &penServo);   // pen wired in now
    stallHoming.begin(steppers.driverL(), steppers.driverR(), &steppers, &kinematics);
    gcodeParser.begin(&planner, &penServo, &kinematics, &stallHoming);

    // Core 0 tasks
    jobExec.begin(&gcodeParser, &planner);
    binExec.begin(&planner);

    wifiMgr.begin();

    Serial.println(F("Ready. Serial G-code at 250000 baud. Type M100 for help."));
}

void loop() {
    processSerial();
    planner.service();
    wifiMgr.service();
}
