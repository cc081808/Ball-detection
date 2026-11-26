#ifndef BALL_SERIAL_CONFIG_H
#define BALL_SERIAL_CONFIG_H

// Slot 4 only. Starts idle. Touch LED starts/pauses camera; Brain Left selects V4.
// Autonomous modes do not require a controller. E-Down selects optional manual.
constexpr bool ENABLE_MOTORS = true;
constexpr unsigned COMMAND_TIMEOUT_MS = 300;
constexpr unsigned CONTROL_PERIOD_MS = 20;
constexpr unsigned SCREEN_PERIOD_MS = 100;
constexpr double TURN_PCT = 10.0;
constexpr double SEARCH_PCT = 10.0;
constexpr unsigned SEARCH_SWEEP_MS = 8000;
constexpr unsigned FINISH_INTAKE_MS = 600;
constexpr unsigned BUMPER_BACKOFF_MS = 600;
constexpr unsigned BUMPER_TURN_MS = 1000;
constexpr double BUMPER_BACKOFF_PCT = 15.0;
constexpr double FORWARD_PCT = 15.0;
constexpr double INTAKE_PCT = 100.0;
// Copied from the latest collector's DRIVER configuration.
constexpr double DRIVER_MAX_PCT = 100.0;
constexpr double DRIVER_DEADBAND_PCT = 5.0;
constexpr double DRIVER_TURN_SCALE = 0.6;
// Motor ports/directions match the existing project's confirmed configuration:
// left 12, normal; right 8, reversed; intake 11, reversed; bumper 5; Touch LED 4.
#endif
