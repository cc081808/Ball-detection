#include "vex.h"
#include "cv_protocol.h"

struct Shared {
    vex::mutex lock;
    cvbot::Frame frame;
    cvbot::Settings settings;
    unsigned config_version = 0;
    bool stop = false, active = false;
    int attempts = 0, seconds = 300;
    const char* state = "CALIBRATING";
    const char* reason = "KEEP_STILL";
};

int receiver(void* argument) {
    Shared& shared = *static_cast<Shared*>(argument);
    char packet[160]; unsigned length = 0;
    bool overflow = false;
    uint32_t sequence = 0;
    while (true) {
        int value = getchar();
        if (value < 0) { vex::this_thread::sleep_for(5); continue; }
        char ch = (char)value;
        if (ch == 'X' || ch == '?') {
            length = 0; overflow = false;
            shared.lock.lock();
            shared.stop = true; shared.frame.valid = false;
            shared.lock.unlock();
            if (ch == '?') { sequence = 0; printf("CV_COORD_V1 DRIVE\n"); }
            else printf("ACK X\n");
            fflush(stdout); continue;
        }
        if (ch != '\n') {
            if (length < sizeof(packet) - 1) packet[length++] = ch;
            else overflow = true;
            continue;
        }
        packet[length] = 0;
        bool valid = false;
        if (!overflow && length > 2 && packet[1] == ' ') {
            shared.lock.lock();
            if (packet[0] == 'C' && !shared.active) {
                cvbot::Settings settings;
                valid = cvbot::parseConfig(packet + 2, settings);
                if (valid) { shared.settings = settings; shared.config_version++; shared.frame.valid = false; }
            } else if (packet[0] == 'F') {
                cvbot::Frame frame;
                valid = cvbot::parseFrame(packet + 2, frame, sequence, vex::timer::system());
                if (valid) { shared.frame = frame; sequence = frame.sequence; }
            }
            if (!valid) { shared.frame.valid = false; shared.stop = true; }
            const char* reported_state = shared.state;
            const char* reported_reason = shared.reason;
            int reported_attempts = shared.attempts, reported_seconds = shared.seconds;
            shared.lock.unlock();
            // USB output may block when disconnected. Never hold the control
            // mutex while writing: the motor loop must still run its watchdog.
            if (valid && packet[0] == 'C') printf("ACK C\n");
            else if (valid) printf("ACK F %lu %s %d %d %s\n", (unsigned long)sequence,
                                   reported_state, reported_attempts, reported_seconds, reported_reason);
        }
        if (!valid) {
            shared.lock.lock(); shared.stop = true; shared.frame.valid = false; shared.lock.unlock();
            printf("ERR PACKET\n");
        }
        fflush(stdout); length = 0; overflow = false;
    }
    return 0;
}

void motorOutput(vex::motor& motor, double percent) {
    if (fabs(percent) < .01) motor.stop(vex::brakeType::brake);
    else motor.spin(vex::directionType::fwd, percent, vex::percentUnits::pct);
}

int main() {
    vex::brain brain;
    vex::motor left(vex::PORT12, false), right(vex::PORT8, true), intake(vex::PORT11, true);
    vex::touchled touch(vex::PORT4);
    vex::bumper bumper(vex::PORT5);
    vex::inertial imu;
    // No distance sensor and no controller are constructed in this program.
    motorOutput(left, 0); motorOutput(right, 0); intake.stop();
    Shared shared;
    vex::thread serial(receiver, &shared);
    brain.Screen.setCursor(1, 1); brain.Screen.print("CV SLOT 5 V1");
    brain.Screen.newLine(); brain.Screen.print("Calibrating: keep still");
    imu.calibrate();
    uint32_t calibration_start = vex::timer::system();
    vex::this_thread::sleep_for(200);
    while (imu.isCalibrating() && vex::timer::system() - calibration_start < 10000)
        vex::this_thread::sleep_for(20);
    bool calibration_ok = !imu.isCalibrating();
    cvbot::Controller control;
    unsigned applied_version = 0;
    uint32_t last_screen = 0;
    while (true) {
        cvbot::Input input;
        input.now = vex::timer::system();
        input.left = left.position(vex::rotationUnits::deg);
        input.right = right.position(vex::rotationUnits::deg);
        input.heading = imu.rotation(vex::rotationUnits::deg);
        input.touch = touch.pressing(); input.check = brain.buttonCheck.pressing();
        input.bumper = bumper.pressing();
        shared.lock.lock();
        input.frame = shared.frame; input.stop = shared.stop; shared.stop = false;
        if (shared.config_version != applied_version) {
            if (control.active()) control.stop("CONFIG_CHANGED");
            control.cfg = shared.settings; control.configured = calibration_ok;
            applied_version = shared.config_version;
        }
        shared.lock.unlock();
        // Stamp after taking the serial snapshot: a packet arriving during the
        // motor reads must not appear to be from the future (unsigned wrap).
        input.now = vex::timer::system();
        cvbot::Output out;
        if (calibration_ok) out = control.step(input);
        else control.stop("IMU_CALIBRATION_FAILED");
        motorOutput(left, out.left); motorOutput(right, out.right);
        if (out.intake) intake.spin(vex::directionType::fwd, out.intake, vex::percentUnits::pct);
        else intake.stop(vex::brakeType::coast);
        shared.lock.lock();
        shared.active = control.active(); shared.state = cvbot::stateName(control.state);
        shared.reason = control.reason; shared.attempts = control.attempts;
        shared.seconds = control.secondsLeft(input.now);
        shared.lock.unlock();
        touch.on(!control.active() ? vex::color::red :
                 (control.state == cvbot::TRACK || control.state == cvbot::FINISH) ? vex::color::blue : vex::color::green);
        if (input.now - last_screen >= 150) {
            brain.Screen.clearScreen();
            brain.Screen.setCursor(1, 1); brain.Screen.print("CV SLOT 5 V1");
            brain.Screen.newLine(); brain.Screen.print("%s", cvbot::stateName(control.state));
            brain.Screen.newLine(); brain.Screen.print("Tries %d/10 Time %d", control.attempts, control.secondsLeft(input.now));
            brain.Screen.newLine(); brain.Screen.print("%s", control.active() ? "LED/Check: STOP" : control.reason);
            brain.Screen.newLine(); brain.Screen.print("%s", cvbot::Controller::fresh(input) ? "LED press/release: start" : "Run PC slot 5 script");
            last_screen = input.now;
        }
        vex::this_thread::sleep_for(20);
    }
}
