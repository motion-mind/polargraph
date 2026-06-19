#include "Config.h"
#include <Preferences.h>

namespace Config {

MachineConfig cfg;
static Preferences prefs;
static const char* NVS_NS = "polargraph";

void load() {
    prefs.begin(NVS_NS, true);
    cfg.machineWidth  = prefs.getFloat("mw",  Defaults::MACHINE_WIDTH);
    cfg.machineHeight = prefs.getFloat("mh",  Defaults::MACHINE_HEIGHT);
    cfg.stepsPerMm    = prefs.getFloat("spm", Defaults::STEPS_PER_MM);
    cfg.maxFeedrate   = prefs.getFloat("mfr", Defaults::MAX_FEEDRATE);
    cfg.acceleration  = prefs.getFloat("acc", Defaults::ACCELERATION);
    cfg.runCurrent    = prefs.getUShort("rc",  Defaults::RUN_CURRENT);
    cfg.holdCurrent   = prefs.getUShort("hc",  Defaults::HOLD_CURRENT);
    cfg.microsteps    = prefs.getUChar("us",   Defaults::MICROSTEPS);

    for (int i = 0; i < NUM_PENS; i++) {
        char key[12];
        snprintf(key, sizeof(key), "psu%d", i);
        cfg.pens[i].servoUp   = prefs.getInt(key, SERVO_UP_ANGLE);
        snprintf(key, sizeof(key), "psd%d", i);
        cfg.pens[i].servoDown = prefs.getInt(key, SERVO_DOWN_ANGLE);
        snprintf(key, sizeof(key), "pox%d", i);
        cfg.pens[i].offsetX   = prefs.getFloat(key, Defaults::TOOL_OFFSET_X[i]);
        snprintf(key, sizeof(key), "poy%d", i);
        cfg.pens[i].offsetY   = prefs.getFloat(key, Defaults::TOOL_OFFSET_Y[i]);
        snprintf(key, sizeof(key), "pen%d", i);
        cfg.pens[i].enabled   = prefs.getBool(key, true);
    }
    prefs.end();
}

void save() {
    prefs.begin(NVS_NS, false);
    prefs.putFloat("mw",  cfg.machineWidth);
    prefs.putFloat("mh",  cfg.machineHeight);
    prefs.putFloat("spm", cfg.stepsPerMm);
    prefs.putFloat("mfr", cfg.maxFeedrate);
    prefs.putFloat("acc", cfg.acceleration);
    prefs.putUShort("rc",  cfg.runCurrent);
    prefs.putUShort("hc",  cfg.holdCurrent);
    prefs.putUChar("us",   cfg.microsteps);

    for (int i = 0; i < NUM_PENS; i++) {
        char key[12];
        snprintf(key, sizeof(key), "psu%d", i); prefs.putInt(key, cfg.pens[i].servoUp);
        snprintf(key, sizeof(key), "psd%d", i); prefs.putInt(key, cfg.pens[i].servoDown);
        snprintf(key, sizeof(key), "pox%d", i); prefs.putFloat(key, cfg.pens[i].offsetX);
        snprintf(key, sizeof(key), "poy%d", i); prefs.putFloat(key, cfg.pens[i].offsetY);
        snprintf(key, sizeof(key), "pen%d", i); prefs.putBool(key, cfg.pens[i].enabled);
    }
    prefs.end();
    Serial.println(F("ok — settings saved"));
}

void reset() {
    cfg = MachineConfig{};
    for (int i = 0; i < NUM_PENS; i++) {
        cfg.pens[i].offsetX = Defaults::TOOL_OFFSET_X[i];
        cfg.pens[i].offsetY = Defaults::TOOL_OFFSET_Y[i];
    }
    save();
    Serial.println(F("ok — settings reset to defaults"));
}

void print() {
    Serial.printf("  Machine WxH  : %.1f x %.1f mm\n", cfg.machineWidth, cfg.machineHeight);
    Serial.printf("  Steps/mm     : %.2f\n", cfg.stepsPerMm);
    Serial.printf("  Max feedrate : %.1f mm/s\n", cfg.maxFeedrate);
    Serial.printf("  Acceleration : %.1f mm/s²\n", cfg.acceleration);
    Serial.printf("  Current      : %d / %d mA\n", cfg.runCurrent, cfg.holdCurrent);
    for (int i = 0; i < NUM_PENS; i++) {
        Serial.printf("  Pen %d  up=%d° down=%d°  offset=(%.1f,%.1f)mm  %s\n",
            i,
            cfg.pens[i].servoUp, cfg.pens[i].servoDown,
            cfg.pens[i].offsetX, cfg.pens[i].offsetY,
            cfg.pens[i].enabled ? "enabled" : "DISABLED");
    }
}

} // namespace Config
