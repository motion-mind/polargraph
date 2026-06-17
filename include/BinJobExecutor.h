#pragma once
/**
 * BinJobExecutor.h — .plg binary job runner
 * ══════════════════════════════════════════
 * Runs on Core 0 alongside JobExecutor (G-code runner).
 * Only one executor runs at a time — they share the same
 * MotionPlanner queue and the same job state flag.
 *
 * Binary execution is fundamentally faster than G-code because:
 *
 *   G-code path per segment:
 *     read line → parse → kinematics (sqrt) → trapezoid plan
 *     (float) → push segment
 *     ≈ 80–200µs per segment on ESP32
 *
 *   Binary path per segment:
 *     fread(20 bytes) → validate flags → pushBinSeg (struct copy)
 *     ≈ 3–8µs per segment
 *
 * At 150 mm/s with 80 steps/mm the planner needs to supply segments
 * at up to ~1000/s for short moves.  The binary path has headroom;
 * the G-code path can lag on complex geometry.
 *
 * Throttle logic:
 *   Reads segments in bursts of BURST_SIZE when queue depth is below
 *   QUEUE_LOW_WATER, then yields to WiFi until it drains to LOW_WATER.
 *   This keeps the motion queue full without starving the web server.
 */

#include <Arduino.h>
#include <LittleFS.h>
#include "PlgFormat.h"
#include "MotionPlanner.h"

constexpr size_t BIN_BURST_SIZE   = 8;    // segments per burst read
constexpr size_t BIN_HIGH_WATER   = 48;   // stop feeding above this
constexpr size_t BIN_LOW_WATER    = 16;   // resume feeding below this
constexpr uint32_t BIN_STACK      = 6144;
constexpr UBaseType_t BIN_PRIORITY = 3;   // slightly above JobExecutor

// Shared job state (also used by JobExecutor & WiFiManager)
// Defined in main.cpp
extern volatile bool    gJobRunning;
extern volatile bool    gJobPaused;
extern volatile bool    gJobStop;
extern volatile uint32_t gJobLinesDone;   // segments done (reused field)
extern volatile uint32_t gJobLinesTotal;
extern volatile char    gJobFile[64];
extern volatile char    gJobType[8];      // "gcode" or "plg"

class BinJobExecutor {
public:
    void begin(MotionPlanner* planner) {
        _planner = planner;
        xTaskCreatePinnedToCore(taskEntry, "BinExec",
            BIN_STACK, this, BIN_PRIORITY, &_handle, 0);
        Serial.println(F("  BinJobExecutor: task started on Core 0"));
    }

    // Start a .plg job — returns false if already running or invalid
    bool start(const char* path) {
        if (gJobRunning) return false;

        File f = LittleFS.open(path, "r");
        if (!f) { Serial.printf("[BinJob] cannot open %s\n", path); return false; }

        PlgHeader hdr{};
        if (f.read((uint8_t*)&hdr, sizeof(hdr)) != sizeof(hdr)
            || !plgHeaderValid(hdr)) {
            f.close();
            Serial.println(F("[BinJob] invalid header"));
            return false;
        }
        f.close();

        // Validate geometry matches current config (warn, don't abort)
        if (fabsf(hdr.stepsPerMm - Config::cfg.stepsPerMm) > 0.5f) {
            Serial.printf("[BinJob] WARN: file stepsPerMm=%.2f, machine=%.2f\n",
                hdr.stepsPerMm, Config::cfg.stepsPerMm);
        }

        strncpy((char*)gJobFile, path, sizeof(gJobFile)-1);
        strncpy((char*)gJobType, "plg", 8);
        gJobLinesTotal = hdr.segCount;
        gJobLinesDone  = 0;
        gJobStop       = false;
        gJobPaused     = false;
        gJobRunning    = true;

        Serial.printf("[BinJob] starting '%s' (%lu segs, ~%lu s)\n",
            path, (unsigned long)hdr.segCount,
            (unsigned long)(hdr.estDurationMs / 1000));
        return true;
    }

private:
    MotionPlanner* _planner = nullptr;
    TaskHandle_t   _handle  = nullptr;

    static void taskEntry(void* p) { static_cast<BinJobExecutor*>(p)->taskLoop(); }

    void taskLoop() {
        for (;;) {
            // Wait for a .plg job to be started
            while (!gJobRunning || strcmp((char*)gJobType, "plg") != 0)
                vTaskDelay(pdMS_TO_TICKS(50));

            runJob();
        }
    }

    void runJob() {
        File f = LittleFS.open((const char*)gJobFile, "r");
        if (!f) { gJobRunning = false; return; }

        // Skip header (already validated on start())
        f.seek(sizeof(PlgHeader));

        PlgSeg buf[BIN_BURST_SIZE];
        uint32_t totalSegs = gJobLinesTotal;
        uint32_t done = 0;

        while (done < totalSegs && !gJobStop) {
            // Pause
            while (gJobPaused && !gJobStop) vTaskDelay(pdMS_TO_TICKS(50));
            if (gJobStop) break;

            // Throttle — wait if queue is full
            while (_planner->queueDepth() >= BIN_HIGH_WATER && !gJobStop)
                vTaskDelay(pdMS_TO_TICKS(2));
            if (gJobStop) break;

            // Read a burst
            uint32_t want = min((uint32_t)BIN_BURST_SIZE, totalSegs - done);
            size_t   got  = f.read((uint8_t*)buf, want * sizeof(PlgSeg)) / sizeof(PlgSeg);
            if (got == 0) break;  // unexpected EOF

            for (size_t i = 0; i < got && !gJobStop; i++) {
                _planner->pushBinSeg(buf[i]);
                done++;
            }
            gJobLinesDone = done;
        }

        f.close();

        if (!gJobStop) {
            // Drain remaining motion
            Serial.println(F("[BinJob] all segs queued — draining motion..."));
            _planner->waitIdle();
            Serial.printf("[BinJob] complete (%lu segs)\n", (unsigned long)done);
        }

        gJobRunning = false;
        gJobStop    = false;
    }
};
