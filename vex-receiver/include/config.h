#ifndef BALL_SERIAL_CONFIG_H
#define BALL_SERIAL_CONFIG_H

// Serial integration prototype; separate from the distance-sensor autonomous project.
// First confirm L/R/F/S on the Brain screen. Enabling motors requires rebuilding.
constexpr bool ENABLE_MOTORS = false;
constexpr unsigned COMMAND_TIMEOUT_MS = 300;
constexpr unsigned CONTROL_PERIOD_MS = 20;
constexpr unsigned SCREEN_PERIOD_MS = 100;
constexpr double TURN_PCT = 10.0;
constexpr double FORWARD_PCT = 15.0;
constexpr double INTAKE_PCT = 100.0;
// Motor ports/directions match the existing project's confirmed configuration:
// left 12, normal; right 8, reversed; intake 11, reversed; bumper 5; Touch LED 4.
#endif
