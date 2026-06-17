#include "Config.h"
#include <Preferences.h>

namespace Config {

MachineConfig cfg;   // the live instance

static Preferences prefs;
static const char* NVS_NS = "polargraph";

void load() {
    prefs.begin(NVS_NS, /*readOnly=*/true);
    cfg.machineWidth  = prefs.getFloat("mw",  Defaults::MACHINE_WIDTH);
    cfg.machineHeight = prefs.getFloat("mh",  Defaults::MACHINE_HEIGHT);
    cfg.stepsPerMm    = prefs.getFloat("spm", Defaults::STEPS_PER_MM);
    cfg.maxFeedrate   = prefs.getFloat("mfr", Defaults::MAX_FEEDRATE);
    cfg.acceleration  = prefs.getFloat("acc", Defaults::ACCELERATION);
    cfg.runCurrent    = prefs.getUShort("rc",  Defaults::RUN_CURRENT);
    cfg.holdCurrent   = prefs.getUShort("hc",  Defaults::HOLD_CURRENT);
    cfg.microsteps    = prefs.getUChar("us",   Defaults::MICROSTEPS);
    cfg.servoUp       = prefs.getInt("su",     SERVO_UP_ANGLE);
    cfg.servoDown     = prefs.getInt("sd",     SERVO_DOWN_ANGLE);
    prefs.end();
}

void save() {
    prefs.begin(NVS_NS, /*readOnly=*/false);
    prefs.putFloat("mw",  cfg.machineWidth);
    prefs.putFloat("mh",  cfg.machineHeight);
    prefs.putFloat("spm", cfg.stepsPerMm);
    prefs.putFloat("mfr", cfg.maxFeedrate);
    prefs.putFloat("acc", cfg.acceleration);
    prefs.putUShort("rc",  cfg.runCurrent);
    prefs.putUShort("hc",  cfg.holdCurrent);
    prefs.putUChar("us",   cfg.microsteps);
    prefs.putInt("su",     cfg.servoUp);
    prefs.putInt("sd",     cfg.servoDown);
    prefs.end();
    Serial.println(F("ok — settings saved to NVS"));
}

void reset() {
    cfg = MachineConfig{};
    save();
    Serial.println(F("ok — settings reset to defaults"));
}

void print() {
    Serial.printf("  Machine WxH   : %.1f x %.1f mm\n", cfg.machineWidth, cfg.machineHeight);
    Serial.printf("  Steps/mm      : %.2f\n",  cfg.stepsPerMm);
    Serial.printf("  Max feedrate  : %.1f mm/s\n", cfg.maxFeedrate);
    Serial.printf("  Acceleration  : %.1f mm/s²\n", cfg.acceleration);
    Serial.printf("  Run/Hold mA   : %d / %d\n", cfg.runCurrent, cfg.holdCurrent);
    Serial.printf("  Microsteps    : %d\n",  cfg.microsteps);
    Serial.printf("  Servo up/down : %d° / %d°\n", cfg.servoUp, cfg.servoDown);
}

} // namespace Config
