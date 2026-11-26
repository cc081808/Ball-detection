#ifndef CV_PROTOCOL_H
#define CV_PROTOCOL_H
#include "cv_control.h"
#include <stdlib.h>
#include <errno.h>

namespace cvbot {
inline bool numbers(const char* s, long* values, int count) {
    for (int i = 0; i < count; ++i) {
        char* end = 0;
        errno = 0;
        values[i] = strtol(s, &end, 10);
        if (s == end || errno == ERANGE) return false;
        s = end;
    }
    while (*s == ' ' || *s == '\r' || *s == '\t') ++s;
    return *s == 0;
}
inline bool parseConfig(const char* line, Settings& cfg) {
    long a[8];
    if (!numbers(line, a, 8)) return false;
    if (a[0] < 200 || a[0] > 800 || a[1] < 400 || a[1] > 950 || a[2] < 40 || a[2] > 450 ||
        a[3] < 1200 || a[3] > 6000 || a[4] < 1200 || a[4] > 5000 ||
        (a[5] != -1 && a[5] != 1) || a[6] < 80 || a[6] > 400 || a[7] < 60 || a[7] > 240) return false;
    cfg.aim = a[0]; cfg.near_y = a[1]; cfg.finish_mm = a[2];
    cfg.length = a[3]; cfg.width = a[4]; cfg.steer_sign = a[5];
    cfg.approach = a[6]; cfg.pickup = a[7];
    return true;
}
inline bool parseFrame(const char* line, Frame& frame, uint32_t previous, uint32_t now) {
    long a[9];
    if (!numbers(line, a, 9)) return false;
    if (a[0] <= 0 || a[0] > 2147483647 || (uint32_t)a[0] <= previous ||
        a[1] < 0 || a[1] > 2147483647 || (a[2] != 0 && a[2] != 1) || a[8] < 0 || a[8] > 5000) return false;
    for (int i = 3; i <= 7; ++i) if (a[i] < 0 || a[i] > 1000) return false;
    if (a[2] && (a[1] == 0 || a[5] < 1 || a[6] < 1 || a[7] < 1 ||
        2*a[3] < a[5]-2 || 2*a[3]+a[5] > 2002 || 2*a[4] < a[6]-2 || 2*a[4]+a[6] > 2002)) return false;
    if (!a[2] && (a[3] || a[4] || a[5] || a[6] || a[7])) return false;
    frame.sequence = a[0]; frame.target = a[1]; frame.visible = a[2];
    frame.x = a[3]; frame.y = a[4]; frame.w = a[5]; frame.h = a[6];
    frame.confidence = a[7]; frame.age = a[8]; frame.received = now; frame.valid = true;
    return true;
}
}
#endif
