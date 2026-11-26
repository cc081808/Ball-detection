#include "vex.h"
#include "config.h"
#include "control.h"
#include "collector.h"

struct Shared {
    vex::mutex lock;
    char command = 'S';
    uint32_t received_ms = 0;
    bool has_command = false;
    bool stop_requested = false;
    bool running = true;
    unsigned observation_id = 0;
    unsigned frame_age_ms = 0;
    char state[32] = "IDLE";
    char last_check[40] = "No check yet";
};

void reportState(void* argument, const char* state, const char* event) {
    Shared& shared = *static_cast<Shared*>(argument);
    shared.lock.lock();
    snprintf(shared.state, sizeof(shared.state), "%s", state);
    if (event) snprintf(shared.last_check, sizeof(shared.last_check), "%s", event);
    shared.lock.unlock();
}

int receiveCommands(void* argument) {
    Shared& shared = *static_cast<Shared*>(argument);
    bool running = true;
    char packet[48];
    unsigned length = 0;
    bool reading_packet = false, overflow = false;
    while (running) {
        int value = getchar();
        shared.lock.lock();
        running = shared.running;
        shared.lock.unlock();
        if (running && value >= 0) {
            char command = static_cast<char>(value);
            // B <ball in confirmation region 0/1> <frame age milliseconds>\n
            // A packet is applied atomically, only after complete validation.
            if (command == 'X' || command == '?') { reading_packet = false; length = 0; }
            if (reading_packet) {
                if (command == '\n') {
                    packet[length] = 0;
                    char extra = 0;
                    int ball = 0;
                    unsigned age = 0;
                    bool valid = !overflow && sscanf(packet, " %d %u %c", &ball, &age, &extra) == 2 &&
                        (ball == 0 || ball == 1) && age <= 5000;
                    char state[32], last_check[40];
                    shared.lock.lock();
                    shared.command = valid ? (ball ? 'Y' : 'N') : 'S';
                    shared.frame_age_ms = valid ? age : 1000;
                    if (valid) shared.observation_id++;
                    shared.received_ms = vex::timer::system();
                    shared.has_command = valid;
                    snprintf(state, sizeof(state), "%s", shared.state);
                    snprintf(last_check, sizeof(last_check), "%s", shared.last_check);
                    shared.lock.unlock();
                    // Only this thread writes serial output: status and ACK stay together.
                    if (valid) printf("ACK B %s|%s\n", state, last_check);
                    else printf("ERR B\n");
                    fflush(stdout);
                    reading_packet = false;
                } else if (length < sizeof(packet) - 1) packet[length++] = command;
                else overflow = true;
                continue;
            }
            if (command == 'B') {
                reading_packet = true; overflow = false; length = 0;
                continue;
            }
            bool valid = command == 'S' || command == 'X';
            if (valid) {
                shared.lock.lock();
                shared.command = command;
                shared.received_ms = vex::timer::system();
                shared.has_command = true;
                shared.frame_age_ms = 0;
                if (command == 'X') {
                    shared.command = 'S';
                    shared.stop_requested = true;
                }
                shared.lock.unlock();
                printf("ACK %c\n", command);
                fflush(stdout);
            } else if (command != '\r' && command != '\n') {
                // Invalidate previous observations on handshake or malformed input.
                shared.lock.lock();
                shared.command = 'S';
                shared.has_command = false;
                shared.lock.unlock();
                if (command == '?') {
                    printf("BALL_SERIAL_V5 %s\n", ENABLE_MOTORS ? "DRIVE" : "TEST");
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

VisionInput readVision(void* argument) {
    Shared& shared = *static_cast<Shared*>(argument);
    shared.lock.lock();
    VisionInput input = {shared.command,
        shared.has_command ? vex::timer::system() - shared.received_ms + shared.frame_age_ms : 1000,
        shared.stop_requested, shared.observation_id};
    shared.lock.unlock();
    return input;
}

int main() {
    vex::brain brain;
    vex::controller controller;
    vex::motor left(vex::PORT12, false), right(vex::PORT8, true), intake(vex::PORT11, true);
    vex::bumper bumper(vex::PORT5);
    vex::touchled touch(vex::PORT4);
    vex::distance distance(vex::PORT7);
    vex::inertial imu;
    CollectorDevices collector = {brain, controller, imu, left, right, intake, distance, bumper, touch};
    Shared shared;
    stopAll(left, right, intake);
    vex::thread receiver(receiveCommands, &shared);
    uint32_t next_screen_ms = 0;
    bool running = true, manual = false;
    bool touch_prev = true, sensor_prev = true, manual_prev = true, filtered_prev = true;
    const char* collector_result = nullptr;
    while (running) {
        bool connected = controller.installed();
        running = !(connected && controller.ButtonFUp.pressing());
        shared.lock.lock();
        VisionInput input = {shared.command,
            shared.has_command ? vex::timer::system() - shared.received_ms + shared.frame_age_ms : 1000,
            shared.stop_requested, shared.observation_id};
        shared.stop_requested = false;
        shared.lock.unlock();
        uint32_t now_ms = vex::timer::system();
        bool touch_button = touch.pressing();
        bool touch_press = touch_button && !touch_prev;
        bool sensor_button = brain.buttonLeft.pressing() || (connected && controller.ButtonLUp.pressing());
        bool manual_button = connected && controller.ButtonEDown.pressing();
        bool filtered_button = connected && controller.ButtonEUp.pressing();
        bool pause = !running || input.stop_requested || brain.buttonCheck.pressing() ||
            (connected && controller.ButtonFDown.pressing()) || (manual && touch_press);
        bool bumper_pressed = bumper.pressing();
        double forward = connected ? controller.AxisA.position() : 0;
        double turn = connected ? controller.AxisC.position() : 0;
        bool neutral = !connected || (deadband(forward) == 0 && deadband(turn) == 0);
        bool sensor_start = canStartCollector(running, connected, neutral, sensor_button, sensor_prev,
            touch_button || filtered_button || manual_button, pause || bumper_pressed);
        bool filtered_start = ENABLE_MOTORS && !manual && running && neutral && !pause && !bumper_pressed &&
            !sensor_button && !manual_button && (touch_press || (filtered_button && !filtered_prev));
        if (sensor_start || filtered_start) {
            stopAll(left, right, intake);
            reportState(&shared, "CALIBRATING", "No check yet");
            // The original collector owns motion; CV can only approve candidates.
            collector_result = runCollectorV4(collector, filtered_start ? readVision : nullptr,
                                               &shared, reportState);
            manual = false;
            touch_prev = sensor_prev = manual_prev = filtered_prev = true;
            shared.lock.lock();
            shared.command = 'S'; shared.has_command = false; shared.stop_requested = false;
            shared.lock.unlock();
            reportState(&shared, "IDLE", collector_result);
            brain.Screen.clearScreen();
            continue;
        }
        if (pause || bumper_pressed || !connected) manual = false;
        else if (ENABLE_MOTORS && manual_button && !manual_prev && neutral) manual = true;
        touch_prev = touch_button; sensor_prev = sensor_button;
        manual_prev = manual_button; filtered_prev = filtered_button;
        reportState(&shared, manual ? "MANUAL" : "IDLE", nullptr);
        if (!manual) stopAll(left, right, intake);
        else {
            Wheels wheels = manualWheels(forward, turn);
            if (wheels.left == 0) left.stop(vex::brakeType::brake);
            else left.spin(vex::directionType::fwd, wheels.left, vex::percentUnits::pct);
            if (wheels.right == 0) right.stop(vex::brakeType::brake);
            else right.spin(vex::directionType::fwd, wheels.right, vex::percentUnits::pct);
            intake.spin(controller.ButtonRDown.pressing() ? vex::directionType::rev : vex::directionType::fwd,
                        INTAKE_PCT, vex::percentUnits::pct);
        }
        if (now_ms >= next_screen_ms) {
            brain.Screen.setCursor(1, 1);
            brain.Screen.print("%s CV:%c          ", manual ? "MANUAL" : "IDLE", input.command);
            brain.Screen.setCursor(2, 1);
            brain.Screen.print("%s        ", !manual && collector_result ? collector_result :
                visionLive(input) ? "Camera connected" : "No fresh camera");
            brain.Screen.setCursor(3, 1); brain.Screen.print("LED: SENSOR + CV    ");
            brain.Screen.setCursor(4, 1); brain.Screen.print("Left: sensor only   ");
            brain.Screen.setCursor(5, 1); brain.Screen.print("Check: Stop         ");
            next_screen_ms = now_ms + SCREEN_PERIOD_MS;
        }
        vex::this_thread::sleep_for(CONTROL_PERIOD_MS);
    }
    stopAll(left, right, intake);
    shared.lock.lock(); shared.running = false; shared.lock.unlock();
    receiver.interrupt();
    brain.Screen.clearScreen();
    brain.Screen.setCursor(1, 1); brain.Screen.print("F-Up stop. Restart.");
    return 0;
}
