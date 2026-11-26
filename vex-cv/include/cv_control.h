#ifndef CV_CONTROL_H
#define CV_CONTROL_H
#include <math.h>
#include <stdint.h>

namespace cvbot {
const double PI = 3.141592653589793;
inline double clamp(double v, double lo, double hi) { return v < lo ? lo : v > hi ? hi : v; }
inline double wrap(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

struct Settings {
    int aim = 500, near_y = 780, finish_mm = 200;
    int length = 2000, width = 2000, steer_sign = 1;
    int approach = 240, pickup = 120;
};
struct Frame {
    uint32_t sequence = 0, received = 0;
    int target = 0, visible = 0, x = 0, y = 0, w = 0, h = 0, confidence = 0, age = 5000;
    bool valid = false;
};
struct Input {
    uint32_t now = 0;
    double left = 0, right = 0, heading = 0;
    bool touch = false, check = false, bumper = false, stop = false;
    Frame frame;
};
enum State { IDLE, STOPPED, SCAN, TRAVEL, TRACK, REACQUIRE, FINISH, BACKOFF, AVOID_TURN };
inline const char* stateName(State s) {
    switch (s) {
    case IDLE: return "IDLE"; case STOPPED: return "STOPPED"; case SCAN: return "SEARCH_SCAN";
    case TRAVEL: return "SEARCH_MOVE"; case TRACK: return "TRACK"; case REACQUIRE: return "REACQUIRE";
    case FINISH: return "FINISH_PICKUP"; case BACKOFF: return "BACKOFF";
    case AVOID_TURN: return "AVOID_TURN";
    } return "UNKNOWN";
}
struct Output { double left = 0, right = 0, intake = 0; };

// This controller is independent of VEX I/O so the actual motion decisions can
// be simulated and tested on a computer. Only step() changes movement state.
class Controller {
public:
    Settings cfg;
    State state = IDLE;
    const char* reason = "START_WITH_LED";
    int attempts = 0;
    double x = 400, y = 400, yaw = 0;
    bool configured = false;
    bool active() const { return state != IDLE && state != STOPPED; }
    int secondsLeft(uint32_t now) const {
        return active() ? (int)clamp(300.0 - (now - started) / 1000.0, 0, 300) : stopped_remaining;
    }
    static bool fresh(const Input& in) {
        return in.frame.valid && (uint32_t)(in.now - in.frame.received) <= 700 &&
            in.frame.age >= 0 && in.frame.age + (uint32_t)(in.now - in.frame.received) <= 700;
    }
    void stop(const char* why) {
        if (active()) stopped_remaining = secondsLeft(last_now);
        state = STOPPED; reason = why; pending_start = false;
    }

    Output step(const Input& in) {
        Output out;
        last_now = in.now;
        if (in.left != in.left || in.right != in.right || in.heading != in.heading ||
            fabs(in.left) > 1e8 || fabs(in.right) > 1e8 || fabs(in.heading) > 1e8) {
            stop("FEEDBACK_INVALID"); return out;
        }
        const bool pressed = in.touch && !previous_touch;
        const bool released = !in.touch && previous_touch;
        previous_touch = in.touch;
        const double dl = (in.left - prev_left) * 1.111;
        const double dr = (in.right - prev_right) * 1.111;
        prev_left = in.left; prev_right = in.right;
        if (active()) {
            double new_yaw = -(in.heading - heading_zero);
            double middle_yaw = (yaw + new_yaw) * .5 * PI / 180;
            double d = (dl + dr) * .5;
            x += d * cos(middle_yaw); y += d * sin(middle_yaw);
            distance += d;
            wheel_motion += (fabs(dl) + fabs(dr)) * .5;
            yaw = new_yaw;
        }
        if (in.stop || in.check) { stop(in.stop ? "PC_STOP" : "CHECK_STOP"); return out; }
        if (active() && pressed) { stop("TOUCH_STOP"); return out; }
        if (!active()) {
            if (!configured || !fresh(in) || in.bumper) {
                pending_start = false;
                if (state == IDLE) reason = !configured ? "CALIBRATE_CAMERA" :
                    in.bumper ? "RELEASE_BUMPER" : "WAIT_CAMERA";
                return out;
            }
            if (pressed) pending_start = true;
            if (!(released && pending_start)) return out;
            pending_start = false;
            started = state_since = stall_since = in.now;
            stopped_remaining = 300;
            heading_zero = in.heading; yaw = 0;
            x = y = 400; distance = wheel_motion = stall_motion = 0;
            attempts = waypoint = collisions = 0;
            target_id = 0; near_hits = 0; last_sequence = 0;
            last_target_sequence = in.frame.sequence; // Arm only from subsequent images.
            scan_start = yaw;
            state = SCAN; reason = "RUNNING";
        }
        // Stops always outrank target memory, pickup completion and recovery.
        if (!fresh(in)) { stop("CAMERA_TIMEOUT"); return out; }
        if (in.now - started >= 300000) { stop("TIME_LIMIT"); return out; }
        if (attempts >= 10) { stop("PICKUP_LIMIT"); return out; }
        if (x < 200 || x > cfg.length - 200 || y < 200 || y > cfg.width - 200) {
            stop("AREA_EDGE"); return out;
        }
        if (in.bumper && state != BACKOFF) {
            if (++collisions > 3) { stop("REPEATED_BUMPER"); return out; }
            state = BACKOFF; state_since = in.now; move_start = distance;
            hold_yaw = yaw; target_id = 0; near_hits = 0;
            last_target_sequence = in.frame.sequence;
        }
        bool new_frame = in.frame.sequence != last_sequence;
        if (new_frame) last_sequence = in.frame.sequence;
        bool target = in.frame.visible && in.frame.target > 0 && in.frame.confidence >= 300 &&
            in.frame.sequence != last_target_sequence;
        if (state == SCAN || state == TRAVEL) {
            if (target) {
                state = TRACK; state_since = in.now;
                target_id = in.frame.target; near_hits = 0;
            }
        }
        double v = 0, omega = 0;
        if (state == TRACK) {
            if (target && in.frame.target == target_id) {
                double error = (in.frame.x - cfg.aim) / 500.0 * cfg.steer_sign;
                last_error = error;
                if (new_frame) {
                    last_seen = in.now; seen_yaw = yaw;
                    bool close = absValue(in.frame.x - cfg.aim) <= 60 &&
                        in.frame.y >= cfg.near_y && in.frame.confidence >= 500;
                    if (close) {
                        if (near_hits == 0) { near_start = distance; near_since = in.now; }
                        near_hits++;
                    } else near_hits = 0;
                    if (near_hits >= 2) {
                        // Latch once, starting from the first close observation.
                        // Later missed detections cannot reset or extend this move.
                        state = FINISH; state_since = in.now;
                        move_start = near_start; hold_yaw = yaw;
                        last_target_sequence = in.frame.sequence;
                    }
                }
                if (state == TRACK) {
                    omega = clamp(-error * 55, -45, 45);
                    if (fabs(error) < .45) {
                        double proximity = clamp((in.frame.y - 350.0) / (cfg.near_y - 350.0), 0, 1);
                        v = (cfg.approach * (1 - proximity) + cfg.pickup * proximity) *
                            clamp(1 - fabs(error) / .5, .2, 1);
                    }
                }
            } else {
                near_hits = 0;
                state = REACQUIRE; state_since = in.now;
                hold_yaw = seen_yaw;
            }
        }
        if (state == REACQUIRE) {
            if (target && in.frame.target == target_id) {
                state = TRACK; state_since = in.now;
                // Next control tick uses the new target; no full scan or route change.
            } else if (in.now - state_since >= 1600) {
                target_id = 0; beginScan(in.now);
                last_target_sequence = in.frame.sequence;
            } else {
                // Search locally around the last bearing. No blind forward charge.
                uint32_t elapsed = in.now - state_since;
                double side = last_error > 0 ? -1 : 1;
                double aim = hold_yaw + side * (elapsed < 600 ? 12 : -12);
                omega = clamp(wrap(aim - yaw) * 2, -20, 20);
            }
        }
        if (state == FINISH) {
            double moved = distance - move_start;
            if (moved >= cfg.finish_mm) {
                attempts++;
                target_id = 0; near_hits = 0;
                last_target_sequence = in.frame.sequence;
                if (attempts >= 10) { stop("PICKUP_LIMIT"); return out; }
                beginScan(in.now);
            } else if (in.now - state_since > (uint32_t)(2000 + cfg.finish_mm * 1000 / cfg.pickup)) {
                stop("PICKUP_TIMEOUT"); return out;
            } else {
                v = cfg.pickup;
                omega = clamp(wrap(hold_yaw - yaw) * 3, -20, 20);
            }
        }
        if (state == BACKOFF) {
            if (move_start - distance >= 150) {
                if (in.bumper) { stop("BUMPER_HELD"); return out; }
                state = AVOID_TURN; state_since = in.now; hold_yaw = yaw + 90;
            } else if (in.now - state_since > 2500) {
                stop("BACKOFF_TIMEOUT"); return out;
            } else {
                v = -120; omega = clamp(wrap(hold_yaw - yaw) * 3, -20, 20);
            }
        }
        if (state == AVOID_TURN) {
            double error = wrap(hold_yaw - yaw);
            if (fabs(error) < 4) beginScan(in.now);
            else if (in.now - state_since > 6000) { stop("TURN_TIMEOUT"); return out; }
            else omega = clamp(error * 2, -35, 35);
        }
        if (state == SCAN) {
            if (fabs(yaw - scan_start) >= 380) {
                state = TRAVEL; state_since = in.now;
            } else if (in.now - state_since > 18000) {
                stop("SCAN_TIMEOUT"); return out;
            } else omega = 35;
        }
        if (state == TRAVEL) {
            // Serpentine grid, inset from the configured area boundary.
            int cols = (cfg.width - 800 + 599) / 600 + 1;
            int rows = (cfg.length - 800 + 599) / 600 + 1;
            int n = waypoint + 1;
            if (n >= rows * cols) { stop("AREA_COVERED"); return out; }
            int col = n / rows, row = n % rows;
            if (col % 2) row = rows - 1 - row;
            double gx = 400 + row * (cfg.length - 800.0) / (rows - 1);
            double gy = 400 + col * (cfg.width - 800.0) / (cols - 1);
            double dx = gx - x, dy = gy - y;
            double remaining = sqrt(dx * dx + dy * dy);
            if (remaining < 70) { waypoint++; beginScan(in.now); omega = 35; }
            else if (in.now - state_since > 20000) { stop("ROUTE_TIMEOUT"); return out; }
            else {
                double error = wrap(atan2(dy, dx) * 180 / PI - yaw);
                omega = clamp(error * 2, -45, 45);
                if (fabs(error) < 25) v = clamp(remaining, 80, cfg.approach) * (1 - fabs(error) / 30);
            }
        }
        if (v > 0) {
            double ahead = 100 + v * .2;
            double nx = x + ahead * cos(yaw * PI / 180), ny = y + ahead * sin(yaw * PI / 180);
            if (nx < 250 || nx > cfg.length - 250 || ny < 250 || ny > cfg.width - 250) {
                stop("AREA_EDGE"); return out;
            }
        }
        double turn = omega * PI / 180 * 225 / 2;
        out.left = clamp((v - turn) / 8, -50, 50);
        out.right = clamp((v + turn) / 8, -50, 50);
        out.intake = 100;
        // Encoder movement detects a blocked drive, including during pickup.
        bool commanded = fabs(out.left) + fabs(out.right) > 5;
        if (!commanded) { stall_since = in.now; stall_motion = wheel_motion; }
        else if (in.now - stall_since >= 700) {
            if (wheel_motion - stall_motion < 8) { stop("DRIVE_STALLED"); return Output(); }
            stall_since = in.now; stall_motion = wheel_motion;
        }
        return out;
    }
private:
    bool previous_touch = true, pending_start = false;
    uint32_t started = 0, state_since = 0, last_seen = 0, last_sequence = 0;
    uint32_t last_target_sequence = 0, near_since = 0, stall_since = 0;
    uint32_t last_now = 0;
    int stopped_remaining = 300;
    double prev_left = 0, prev_right = 0, heading_zero = 0, distance = 0;
    double wheel_motion = 0, stall_motion = 0;
    double scan_start = 0, hold_yaw = 0, seen_yaw = 0, last_error = 0;
    double move_start = 0, near_start = 0;
    int target_id = 0, near_hits = 0, waypoint = 0, collisions = 0;
    static int absValue(int v) { return v < 0 ? -v : v; }
    void beginScan(uint32_t now) {
        state = SCAN; state_since = now; scan_start = yaw;
    }
};
} // namespace cvbot
#endif
