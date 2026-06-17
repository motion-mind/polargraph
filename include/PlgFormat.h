#pragma once
/**
 * PlgFormat.h — Polargraph Binary Job Format (.plg)
 * ══════════════════════════════════════════════════
 * Designed for direct streaming from LittleFS into the step ISR
 * with zero parsing, zero float math, and zero kinematics on-device.
 *
 * All velocity planning, kinematics, and lookahead happen in the
 * browser before upload.  The firmware is a pure executor.
 *
 * File layout:
 *   [PlgHeader]  — 32 bytes, magic + metadata
 *   [PlgSeg]*    — 20 bytes × segCount, the motion segments
 *
 * Velocity encoding:
 *   Intervals stored as uint16_t µs/step.
 *   Range: 20µs (50 kHz, ~625 mm/s at 80 steps/mm)
 *          to 65535µs (~0.2 mm/s) — covers all practical speeds.
 *   Three intervals per segment: entry, cruise, exit.
 *   The ISR interpolates linearly between them across accelSteps/decelSteps.
 *
 * Pen events:
 *   Encoded in flags — no separate servo segments needed.
 *   PEN_DOWN fires at the START of the segment.
 *   PEN_UP   fires at the END   of the segment (after last step).
 *   This lets the servo move while the gondola is already in position.
 *
 * Coordinate tracking:
 *   finalStepsL / finalStepsR let the firmware update its absolute
 *   step position after each segment without accumulating delta errors.
 */

#include <stdint.h>

// ── Magic & version ───────────────────────────────────────────
constexpr uint8_t PLG_MAGIC[4]  = { 'P','L','G','1' };
constexpr uint8_t PLG_VERSION   = 2;

// ── Flag bits ─────────────────────────────────────────────────
constexpr uint8_t PLG_PEN_DOWN  = 0x01;  // lower pen at segment start
constexpr uint8_t PLG_PEN_UP    = 0x02;  // raise pen at segment end
constexpr uint8_t PLG_LAST      = 0x04;  // final segment in job
constexpr uint8_t PLG_RAPID     = 0x08;  // travel move (pen already up)

// ── File header (32 bytes, pad to 8-byte alignment) ───────────
struct __attribute__((packed)) PlgHeader {
    uint8_t  magic[4];        // PLG_MAGIC
    uint8_t  version;         // PLG_VERSION
    uint8_t  _pad0[3];
    uint32_t segCount;        // total number of PlgSeg records
    float    machineWidth;    // mm — for validation
    float    machineHeight;   // mm — for validation
    float    stepsPerMm;      // steps/mm used during planning
    uint32_t estDurationMs;   // estimated job duration (ms), 0=unknown
    uint8_t  _pad1[4];        // reserved, must be zero
};
static_assert(sizeof(PlgHeader) == 32, "PlgHeader must be 32 bytes");

// ── Motion segment (20 bytes) ─────────────────────────────────
struct __attribute__((packed)) PlgSeg {
    int32_t  deltaL;          // left  cord step delta (signed)
    int32_t  deltaR;          // right cord step delta (signed)
    uint16_t entryInterval;   // µs/step at segment start
    uint16_t cruiseInterval;  // µs/step at cruise speed
    uint16_t exitInterval;    // µs/step at segment end
    uint16_t accelSteps;      // steps in acceleration ramp
    uint16_t decelSteps;      // steps in deceleration ramp
    uint8_t  flags;           // PLG_PEN_DOWN / PLG_PEN_UP / PLG_LAST
    uint8_t  _pad;            // reserved
};
static_assert(sizeof(PlgSeg) == 20, "PlgSeg must be 20 bytes");

// ── Computed dominant step count (inline, no storage needed) ──
inline uint32_t plgDominant(const PlgSeg& s) {
    int32_t aL = s.deltaL < 0 ? -s.deltaL : s.deltaL;
    int32_t aR = s.deltaR < 0 ? -s.deltaR : s.deltaR;
    return (uint32_t)(aL > aR ? aL : aR);
}

// ── Validation helper ─────────────────────────────────────────
inline bool plgHeaderValid(const PlgHeader& h) {
    return h.magic[0] == PLG_MAGIC[0]
        && h.magic[1] == PLG_MAGIC[1]
        && h.magic[2] == PLG_MAGIC[2]
        && h.magic[3] == PLG_MAGIC[3]
        && h.version  == PLG_VERSION
        && h.segCount  > 0
        && h.stepsPerMm > 0.0f;
}
