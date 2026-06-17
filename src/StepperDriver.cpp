#include "StepperDriver.h"

/**
 * IRAM_ATTR functions must be defined in a .cpp file, not a header.
 * When defined in a header they get instantiated in every translation
 * unit that includes it, causing the Xtensa linker to fail with
 * "dangerous relocation: l32r: literal placed after use" because it
 * can't resolve the literal pool for multiple IRAM copies.
 *
 * Defining them here means there is exactly one copy in IRAM.
 */

IRAM_ATTR void StepperDriver::stepLeft(bool dir) {
    digitalWrite(PIN_X_DIR,  dir ? HIGH : LOW);
    digitalWrite(PIN_X_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_X_STEP, LOW);
}

IRAM_ATTR void StepperDriver::stepRight(bool dir) {
    digitalWrite(PIN_Y_DIR,  dir ? HIGH : LOW);
    digitalWrite(PIN_Y_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_Y_STEP, LOW);
}
