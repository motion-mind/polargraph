#pragma once
/**
 * PlgFormat.h — Polargraph Binary Job Format (.plg) version 3
 * Branch: feature/multi-pen
 *
 * Changes from v2:
 *   - PLG_VERSION bumped to 3
 *   - PlgHeader gains penCount field (was _pad1[0])
 *   - PlgSeg._pad repurposed as penIndex (0–3)
 *     penIndex = which physical pen this segment belongs to
 *   - New flag: PLG_TOOL_CHANGE — gondola is repositioning between pens;
 *     firmware raises current pen, selects new penIndex, then continues.
 *   - PlgHeader.layerCount: number of colour layers in this job
 *
 * Pen selection protocol:
 *   A PLG_TOOL_CHANGE segment carries the new penIndex in seg.penIndex.
 *   The firmware executes the move (travel to the tool-change safe position
 *   or directly to the first draw point) with all pens up, then calls
 *   selectPen(penIndex).  The next PLG_PEN_DOWN lowers the new pen.
 *
 * Tool offsets:
 *   Kinematics are pre-applied in the browser.  Each pen's offset
 *   (from Config::cfg.pens[i].offsetX/Y) is added to the Cartesian
 *   waypoints before cord-length conversion, so the firmware needs no
 *   offset logic at all — the deltas are already correct for each pen.
 */

#include <stdint.h>

// ── Magic & version ────────────────────────────────────────────
constexpr uint8_t PLG_MAGIC[4] = { 'P','L','G','1' };
constexpr uint8_t PLG_VERSION  = 3;   // bumped from 2 for multi-pen

// ── Flag bits ──────────────────────────────────────────────────
constexpr uint8_t PLG_PEN_DOWN    = 0x01;  // lower active pen at seg start
constexpr uint8_t PLG_PEN_UP      = 0x02;  // raise active pen at seg end
constexpr uint8_t PLG_LAST        = 0x04;  // final segment in job
constexpr uint8_t PLG_RAPID       = 0x08;  // travel move (pen already up)
constexpr uint8_t PLG_TOOL_CHANGE = 0x10;  // select pen[penIndex] after move

// ── File header (32 bytes) ─────────────────────────────────────
struct __attribute__((packed)) PlgHeader {
    uint8_t  magic[4];        // PLG_MAGIC
    uint8_t  version;         // PLG_VERSION (3)
    uint8_t  penCount;        // number of pens used (1–4)
    uint8_t  layerCount;      // number of colour layers
    uint8_t  _pad0;
    uint32_t segCount;        // total PlgSeg records
    float    machineWidth;    // mm
    float    machineHeight;   // mm
    float    stepsPerMm;      // steps/mm used during planning
    uint32_t estDurationMs;   // estimated duration, 0=unknown
    uint8_t  _pad1[4];
};
static_assert(sizeof(PlgHeader) == 32, "PlgHeader must be 32 bytes");

// ── Motion segment (20 bytes) ─────────────────────────────────
struct __attribute__((packed)) PlgSeg {
    int32_t  deltaL;          // left  cord step delta (signed)
    int32_t  deltaR;          // right cord step delta (signed)
    uint16_t entryInterval;   // µs/step at start
    uint16_t cruiseInterval;  // µs/step at cruise
    uint16_t exitInterval;    // µs/step at end
    uint16_t accelSteps;
    uint16_t decelSteps;
    uint8_t  flags;           // see PLG_* constants above
    uint8_t  penIndex;        // active pen (0–3); used when PLG_TOOL_CHANGE set
};
static_assert(sizeof(PlgSeg) == 20, "PlgSeg must be 20 bytes");

inline uint32_t plgDominant(const PlgSeg& s) {
    int32_t aL = s.deltaL < 0 ? -s.deltaL : s.deltaL;
    int32_t aR = s.deltaR < 0 ? -s.deltaR : s.deltaR;
    return (uint32_t)(aL > aR ? aL : aR);
}

inline bool plgHeaderValid(const PlgHeader& h) {
    return h.magic[0] == PLG_MAGIC[0]
        && h.magic[1] == PLG_MAGIC[1]
        && h.magic[2] == PLG_MAGIC[2]
        && h.magic[3] == PLG_MAGIC[3]
        && h.version  == PLG_VERSION
        && h.segCount  > 0
        && h.stepsPerMm > 0.0f;
}
