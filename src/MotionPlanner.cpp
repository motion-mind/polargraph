#include "MotionPlanner.h"

// Static ISR-state members — must live in IRAM
MotionPlanner* IRAM_ATTR MotionPlanner::_instance    = nullptr;
volatile QueueSeg IRAM_ATTR MotionPlanner::_isrSeg   = {};
volatile bool     IRAM_ATTR MotionPlanner::_isrActive = false;
volatile uint32_t IRAM_ATTR MotionPlanner::_isrDone   = 0;
volatile int32_t  IRAM_ATTR MotionPlanner::_isrErrL   = 0;
volatile int32_t  IRAM_ATTR MotionPlanner::_isrErrR   = 0;
volatile uint32_t IRAM_ATTR MotionPlanner::_isrIntervalFP = 0;
