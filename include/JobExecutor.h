#pragma once
/**
 * JobExecutor.h — G-code text job runner (Core 0)
 * ════════════════════════════════════════════════
 * Reads .gcode files from LittleFS line-by-line and feeds GCodeParser.
 * Uses shared gJob* globals so WiFiManager can report unified status
 * for both .gcode and .plg jobs.
 *
 * File type routing (called by WiFiManager on /job/start):
 *   .plg extension or PLG1 magic → BinJobExecutor::start()
 *   otherwise                    → JobExecutor::start()
 */

#include <Arduino.h>
#include <LittleFS.h>
#include "GCodeParser.h"
#include "MotionPlanner.h"

// Shared state — defined in main.cpp
extern volatile bool     gJobRunning;
extern volatile bool     gJobPaused;
extern volatile bool     gJobStop;
extern volatile uint32_t gJobLinesDone;
extern volatile uint32_t gJobLinesTotal;
extern volatile char     gJobFile[64];
extern volatile char     gJobType[8];

constexpr uint32_t GCODE_STACK    = 8192;
constexpr UBaseType_t GCODE_PRI   = 2;
constexpr size_t GC_HIGH_WATER    = 48;

class JobExecutor {
public:
    void begin(GCodeParser* parser, MotionPlanner* planner) {
        _parser  = parser;
        _planner = planner;
        xTaskCreatePinnedToCore(taskEntry, "GcodeExec",
            GCODE_STACK, this, GCODE_PRI, &_handle, 0);
        Serial.println(F("  JobExecutor: task started on Core 0"));
    }

    bool start(const char* path) {
        if (gJobRunning) return false;
        if (!LittleFS.exists(path)) return false;

        strncpy((char*)gJobFile, path, sizeof(gJobFile)-1);
        strncpy((char*)gJobType, "gcode", 8);
        gJobLinesTotal = countLines(path);
        gJobLinesDone  = 0;
        gJobStop       = false;
        gJobPaused     = false;
        gJobRunning    = true;

        Serial.printf("[GcodeJob] starting '%s' (%lu lines)\n",
            path, (unsigned long)gJobLinesTotal);
        return true;
    }

    // Convenience accessors matching old interface
    bool        isIdle()    const { return !gJobRunning; }
    const char* stateStr()  const {
        if (!gJobRunning)         return "idle";
        if (gJobStop)             return "stopping";
        if (gJobPaused)           return "paused";
        return "running";
    }
    const char* filePath()  const { return (const char*)gJobFile; }
    uint32_t    linesDone() const { return gJobLinesDone; }
    uint32_t    linesTotal()const { return gJobLinesTotal; }
    uint8_t progressPct()   const {
        if (!gJobLinesTotal) return 0;
        uint32_t p = (gJobLinesDone * 100UL) / gJobLinesTotal;
        return p > 100 ? 100 : (uint8_t)p;
    }
    uint32_t etaSeconds()   const {
        if (!gJobLinesDone || !gJobRunning) return 0;
        uint32_t elapsed = (millis() - _startMs) / 1000;
        if (!elapsed) return 0;
        uint32_t total = (elapsed * gJobLinesTotal) / gJobLinesDone;
        return total > elapsed ? total - elapsed : 0;
    }

    static bool fileExists(const char* p) { return LittleFS.exists(p); }
    static bool deleteJob (const char* p) { return LittleFS.remove(p); }

    static String listFiles() {
        String out = "[";
        File root = LittleFS.open("/");
        File f = root.openNextFile();
        bool first = true;
        while (f) {
            if (!f.isDirectory()) {
                if (!first) out += ",";
                out += "{\"name\":\""; out += f.name();
                out += "\",\"size\":"; out += f.size(); out += "}";
                first = false;
            }
            f = root.openNextFile();
        }
        out += "]";
        return out;
    }

private:
    GCodeParser*  _parser  = nullptr;
    MotionPlanner* _planner = nullptr;
    TaskHandle_t  _handle  = nullptr;
    uint32_t      _startMs = 0;

    static void taskEntry(void* p) { static_cast<JobExecutor*>(p)->taskLoop(); }

    void taskLoop() {
        for (;;) {
            while (!gJobRunning || strcmp((const char*)gJobType, "gcode") != 0)
                vTaskDelay(pdMS_TO_TICKS(50));
            _startMs = millis();
            runJob();
        }
    }

    void runJob() {
        File f = LittleFS.open((const char*)gJobFile, "r");
        if (!f) { gJobRunning = false; return; }

        char   buf[128];
        size_t pos = 0;
        uint32_t done = 0;

        while (f.available() && !gJobStop) {
            while (gJobPaused && !gJobStop) vTaskDelay(pdMS_TO_TICKS(100));
            if (gJobStop) break;

            while (_planner->queueDepth() >= GC_HIGH_WATER && !gJobStop)
                vTaskDelay(pdMS_TO_TICKS(5));
            if (gJobStop) break;

            char c = f.read();
            if (c == '\n' || c == '\r') {
                if (pos > 0) {
                    buf[pos] = '\0';
                    _parser->process(buf);
                    gJobLinesDone = ++done;
                    pos = 0;
                }
            } else if (pos < sizeof(buf)-1) {
                buf[pos++] = c;
            }
        }
        if (pos > 0 && !gJobStop) {
            buf[pos] = '\0'; _parser->process(buf);
            gJobLinesDone = ++done;
        }
        f.close();

        if (!gJobStop) {
            _planner->waitIdle();
            Serial.printf("[GcodeJob] done (%lu lines)\n", (unsigned long)done);
        }
        gJobRunning = false;
        gJobStop    = false;
    }

    static uint32_t countLines(const char* path) {
        File f = LittleFS.open(path, "r");
        if (!f) return 0;
        uint32_t n = 0; bool inLine = false, isCmt = false;
        while (f.available()) {
            char c = f.read();
            if (c == '\n' || c == '\r') { if (inLine && !isCmt) n++; inLine=false; isCmt=false; }
            else { if (!inLine && c==';') isCmt=true; if(c!=' '&&c!='\t') inLine=true; }
        }
        if (inLine && !isCmt) n++;
        f.close(); return n;
    }
};
