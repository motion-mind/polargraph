#pragma once
/**
 * Kinematics.h — Polargraph cord-length ↔ Cartesian conversion
 *
 * Geometry:
 *
 *   L●─────────────────────────●R
 *    \                         /
 *     \  left cord (lL)       / right cord (lR)
 *      \                     /
 *       \                   /
 *        ●  gondola (x, y)
 *
 *   Origin (0,0) at L anchor.
 *   X increases to the right.
 *   Y increases downward.
 *   machineWidth W = horizontal distance between anchors.
 *
 *   Forward (XY → cord lengths):
 *     lL = sqrt(x²  + y²)
 *     lR = sqrt((W-x)² + y²)
 *
 *   Inverse (cord lengths → XY):
 *     x = (W² + lL² − lR²) / (2W)
 *     y = sqrt(lL² − x²)
 */

#include <math.h>
#include "Config.h"

class Kinematics {
public:
    // ── Forward kinematics: (x,y) mm → cord steps ─────────────
    void cartesianToSteps(float x, float y,
                          int32_t& stepsL, int32_t& stepsR) const
    {
        float lL = sqrtf(x * x + y * y);
        float dx = Config::cfg.machineWidth - x;
        float lR = sqrtf(dx * dx + y * y);

        stepsL = static_cast<int32_t>(lL * Config::cfg.stepsPerMm + 0.5f);
        stepsR = static_cast<int32_t>(lR * Config::cfg.stepsPerMm + 0.5f);
    }

    // ── Inverse kinematics: cord steps → (x,y) mm ─────────────
    void stepsToCartesian(int32_t stepsL, int32_t stepsR,
                          float& x, float& y) const
    {
        float W  = Config::cfg.machineWidth;
        float lL = stepsL / Config::cfg.stepsPerMm;
        float lR = stepsR / Config::cfg.stepsPerMm;

        x = (W * W + lL * lL - lR * lR) / (2.0f * W);
        float y2 = lL * lL - x * x;
        y = (y2 > 0.0f) ? sqrtf(y2) : 0.0f;
    }

    // ── Cord-length distance between two Cartesian points ──────
    // Returns the longer of the two cord deltas (conservative segment length)
    float segmentLength(float x0, float y0, float x1, float y1) const {
        return sqrtf((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    }

    // ── Bounds check ──────────────────────────────────────────
    bool inBounds(float x, float y) const {
        if (x < 0 || y < 0)                         return false;
        if (x > Config::cfg.machineWidth)            return false;
        if (y > Config::cfg.machineHeight)           return false;
        return true;
    }

    // ── Home position (centre-top of canvas) ──────────────────
    void homePosition(float& x, float& y) const {
        x = Config::cfg.machineWidth  / 2.0f;
        y = Defaults::HOME_Y;
    }
};
