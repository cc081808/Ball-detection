#ifndef BALL_COLLECTOR_V4_H
#define BALL_COLLECTOR_V4_H
#include "vex.h"
#include "vision_assist.h"

// One set of motor objects is shared by all modes. Only the main thread drives.
struct CollectorDevices {
    vex::brain& brain;
    vex::controller& controller;
    vex::inertial& imu;
    vex::motor& left;
    vex::motor& right;
    vex::motor& intake;
    vex::distance& distance;
    vex::bumper& bumper;
    vex::touchled& touch;
};
// Original distance-sensor mission. With a reader, CV must confirm each candidate
// before the original collection drive. CV never steers. Returns with motors stopped.
// Calibrates, waits for Touch LED start, and returns on Brain Check or optional F-Up/F-Down.
typedef void (*CollectorReporter)(void*, const char* state, const char* event);
const char* runCollectorV4(CollectorDevices& devices, VisionReader reader = nullptr,
                          void* context = nullptr, CollectorReporter reporter = nullptr);
#endif
