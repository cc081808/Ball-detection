#ifndef BALL_CONTROL_H
#define BALL_CONTROL_H
#include "config.h"

enum class Mode { IDLE, MANUAL, AUTO };

constexpr bool canStartCollector(bool running, bool connected, bool neutral,
                                 bool pressed, bool previous, bool otherMode, bool pause) {
    return ENABLE_MOTORS && running && (neutral || !connected) && pressed &&
           !previous && !otherMode && !pause;
}

// Pure decisions are shared by the firmware and compile-time regression tests.
constexpr double magnitude(double v) { return v < 0 ? -v : v; }
constexpr double deadband(double v) {
    return magnitude(v) < DRIVER_DEADBAND_PCT ? 0.0 : v;
}
constexpr double maxMagnitude(double a, double b) {
    return magnitude(a) > magnitude(b) ? magnitude(a) : magnitude(b);
}
constexpr double mixScale(double a, double b) {
    return maxMagnitude(a, b) > 100.0 ? 100.0 / maxMagnitude(a, b) : 1.0;
}
struct Wheels { double left; double right; };
enum class BumpPhase { CLEAR, BACKOFF, TURN, BLOCKED };
struct BumpState { BumpPhase phase; unsigned started_ms; };
constexpr BumpState bumperStep(BumpState old, bool pressed, bool active, unsigned now) {
    return !active ? BumpState{BumpPhase::CLEAR, 0} :
           old.phase == BumpPhase::CLEAR ? (pressed ? BumpState{BumpPhase::BACKOFF, now} : old) :
           old.phase == BumpPhase::BACKOFF ? (now - old.started_ms < BUMPER_BACKOFF_MS ? old :
               pressed ? BumpState{BumpPhase::BLOCKED, now} : BumpState{BumpPhase::TURN, now}) :
           old.phase == BumpPhase::TURN ? (pressed ? BumpState{BumpPhase::BLOCKED, now} :
               now - old.started_ms < BUMPER_TURN_MS ? old : BumpState{BumpPhase::CLEAR, 0}) : old;
}
constexpr Wheels bumperWheels(BumpPhase phase) {
    return phase == BumpPhase::BACKOFF ? Wheels{-BUMPER_BACKOFF_PCT, -BUMPER_BACKOFF_PCT} :
           phase == BumpPhase::TURN ? Wheels{TURN_PCT, -TURN_PCT} : Wheels{0, 0};
}
constexpr Wheels searchWheels(unsigned elapsed) {
    return (elapsed / SEARCH_SWEEP_MS) % 2 == 0 ?
        Wheels{-SEARCH_PCT, SEARCH_PCT} : Wheels{SEARCH_PCT, -SEARCH_PCT};
}
struct CameraState { bool forward_ready; bool finishing; unsigned started_ms; };
constexpr CameraState cameraStep(CameraState old, char command, bool active, unsigned now) {
    return !active || command == 'S' ? CameraState{false, false, 0} :
           command == 'F' ? CameraState{true, false, 0} :
           command == 'C' ? (old.finishing ? old : old.forward_ready ?
               CameraState{false, true, now} : CameraState{false, false, 0}) :
           CameraState{false, false, 0};
}
constexpr char cameraAction(CameraState state, char command, unsigned now) {
    return command == 'C' ? (state.finishing && now - state.started_ms < FINISH_INTAKE_MS ? 'F' : 'N') : command;
}
constexpr Wheels scaleWheels(double left, double right) {
    return {left * mixScale(left, right) * DRIVER_MAX_PCT / 100.0,
            right * mixScale(left, right) * DRIVER_MAX_PCT / 100.0};
}
// Same split-arcade mixer as pingpong-collector/src/geometry.cpp.
constexpr Wheels manualWheels(double forward, double turn) {
    return scaleWheels(deadband(forward) + deadband(turn) * DRIVER_TURN_SCALE,
                       deadband(forward) - deadband(turn) * DRIVER_TURN_SCALE);
}
constexpr Wheels cameraWheels(char command) {
    return command == 'L' ? Wheels{-TURN_PCT, TURN_PCT} :
           command == 'N' ? Wheels{-SEARCH_PCT, SEARCH_PCT} :
           command == 'R' ? Wheels{TURN_PCT, -TURN_PCT} :
           command == 'F' ? Wheels{FORWARD_PCT, FORWARD_PCT} : Wheels{0, 0};
}
constexpr Mode chooseMode(Mode previous, bool connected, bool stop,
                          bool autoPress, bool manualPress, bool neutral, bool fresh) {
    return !ENABLE_MOTORS || stop ? Mode::IDLE :
           autoPress && manualPress ? Mode::IDLE :
           manualPress ? (connected && neutral ? Mode::MANUAL : Mode::IDLE) :
           autoPress ? (fresh && (neutral || !connected) ? Mode::AUTO : Mode::IDLE) :
           previous == Mode::MANUAL && !connected ? Mode::IDLE :
           previous == Mode::AUTO && !fresh ? Mode::IDLE : previous;
}
#endif
