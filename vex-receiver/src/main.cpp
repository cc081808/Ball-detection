#include "vex.h"
#include "config.h"

struct Shared {
    vex::mutex lock;
    char command = 'S';
    uint32_t received_ms = 0;
    bool running = true;
};

int receiveCommands(void* argument) {
    Shared& shared = *static_cast<Shared*>(argument);
    bool running = true;
    while (running) {
        int value = getchar();
        shared.lock.lock();
        running = shared.running;
        shared.lock.unlock();
        if (running && value >= 0) {
            char command = static_cast<char>(value);
            bool valid = command == 'L' || command == 'R' || command == 'F' || command == 'S';
            if (valid) {
                shared.lock.lock();
                shared.command = command;
                shared.received_ms = vex::timer::system();
                shared.lock.unlock();
                printf("ACK %c\n", command);
                fflush(stdout);
            } else if (command != '\r' && command != '\n') {
                // Handshake and unexpected input both stop any previous motion.
                shared.lock.lock();
                shared.command = 'S';
                shared.lock.unlock();
                if (command == '?') {
                    printf("BALL_SERIAL_V1 %s\n", ENABLE_MOTORS ? "DRIVE" : "TEST");
                    fflush(stdout);
                }
            }
        } else if (value < 0) {
            vex::this_thread::sleep_for(10);
        }
    }
    return 0;
}

void stopAll(vex::motor& left, vex::motor& right, vex::motor& intake) {
    left.stop(vex::brakeType::brake);
    right.stop(vex::brakeType::brake);
    intake.stop(vex::brakeType::coast);
}

int main() {
    vex::brain brain;
    vex::controller controller;
    vex::motor left(vex::PORT12, false);
    vex::motor right(vex::PORT8, true);
    vex::motor intake(vex::PORT11, true);
    vex::bumper bumper(vex::PORT5);
    vex::touchled touch(vex::PORT4);
    Shared shared;
    stopAll(left, right, intake);
    vex::thread receiver(receiveCommands, &shared);
    uint32_t next_screen_ms = 0;
    bool running = true;
    while (running) {
        running = !bumper.pressing() && !touch.pressing() && !controller.ButtonFUp.pressing();
        shared.lock.lock();
        char command = shared.command;
        uint32_t received_ms = shared.received_ms;
        shared.lock.unlock();
        uint32_t now_ms = vex::timer::system();
        bool fresh = now_ms - received_ms <= COMMAND_TIMEOUT_MS;
        if (!fresh) command = 'S';
        bool allowed = ENABLE_MOTORS && running && controller.installed() && controller.ButtonEUp.pressing();

        if (!allowed || command == 'S') {
            stopAll(left, right, intake);
        } else if (command == 'L') {
            left.spin(vex::directionType::rev, TURN_PCT, vex::percentUnits::pct);
            right.spin(vex::directionType::fwd, TURN_PCT, vex::percentUnits::pct);
            intake.stop();
        } else if (command == 'R') {
            left.spin(vex::directionType::fwd, TURN_PCT, vex::percentUnits::pct);
            right.spin(vex::directionType::rev, TURN_PCT, vex::percentUnits::pct);
            intake.stop();
        } else if (command == 'F') {
            left.spin(vex::directionType::fwd, FORWARD_PCT, vex::percentUnits::pct);
            right.spin(vex::directionType::fwd, FORWARD_PCT, vex::percentUnits::pct);
            intake.spin(vex::directionType::fwd, INTAKE_PCT, vex::percentUnits::pct);
        }

        if (now_ms >= next_screen_ms) {
            brain.Screen.setCursor(1, 1);
            brain.Screen.print("%s Cmd:%c     ", ENABLE_MOTORS ? "DRIVE" : "TEST", command);
            brain.Screen.setCursor(2, 1);
            brain.Screen.print("%s        ", !fresh ? "No fresh data" : !ENABLE_MOTORS ? "Motors disabled" : allowed ? "E-Up held" : "Hold E-Up");
            next_screen_ms = now_ms + SCREEN_PERIOD_MS;
        }
        vex::this_thread::sleep_for(CONTROL_PERIOD_MS);
    }
    stopAll(left, right, intake);
    shared.lock.lock();
    shared.running = false;
    shared.lock.unlock();
    receiver.interrupt();
    brain.Screen.clearScreen();
    brain.Screen.setCursor(1, 1);
    brain.Screen.print("Stopped. Restart.");
    return 0;
}
