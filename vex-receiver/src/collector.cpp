// Imported from Ping pong ball collector V4.iqcpp (2026-11-20 rework).
// Exact original is in reference/collector_v4_original.iqcpp; slot 1 is untouched.
// Adaptations: shared devices, re-entry reset, controller stop checks, bounded
// calibration, motor-stop guard, serial-safe logging and return to the mode menu.
#include "collector.h"
namespace collector_v4 {
using namespace vex;
static CollectorDevices* hw = nullptr;
static VisionReader vision_reader = nullptr;
static void* vision_context = nullptr;
static CollectorReporter state_reporter = nullptr;
VisionInput currentVision() {
    return vision_reader ? vision_reader(vision_context) : VisionInput{'N', 1000, false};
}
void collectorLog(const char*, ...) {} // Serial channel is reserved for ACKs.
bool abortRequested() {
    return currentVision().stop_requested || hw->brain.buttonCheck.pressing() ||
           (hw->controller.installed() && (hw->controller.ButtonFUp.pressing() ||
                                           hw->controller.ButtonFDown.pressing()));
}
// ============================================================ SETTINGS
const double TIME_LIMIT_S = 300;     // 5 min
const int    MAX_BALLS    = 10;

// Route
const double START_FORWARD_MM = 150; // away from the bottom wall before turning
// Search area (from the bottom-right corner) and pattern
const double AREA_UP_MM     = 6000;  // area limit along the start direction
const double AREA_LEFT_MM   = 5000;  // area limit to the left
const double LANE_FIRST_MM  = 700;   // first lane, from the right edge
const double LANE_GAP_MM    = 1350;  // between lanes
const double STOP_FIRST_MM  = 600;   // first stop, from the bottom edge
const double STOP_GAP_MM    = 1200;  // between stops along a lane
const double EDGE_KEEP_MM   = 600;   // last lane / last stop this far in from the edges
const int    MAX_SCANS_STOP = 8;     // 360s at one stop before moving on regardless
const double SCAN_DEG       = 380;   // a stop's spin: 20 deg overlap so a ball at the start is crossed fully
const double LEG_STOP_MM      = 450; // sensor reading straight ahead that ends a leg
// Where the walls are at the start (rear axle = origin, after nothing has moved):
const double START_BACK_GAP_MM = 150; // gap between the robot's back and the bottom wall
const double START_SIDE_GAP_MM = 30;  // gap between the robot's right side and the right wall
// Wall fixes: when the sensor points straight at a known wall, its reading corrects the position
const double WALL_FIX_TOL_DEG = 2.5;  // must point within this of straight at the wall
const double WALL_FIX_GATE_MM = 250;  // ignore readings that disagree by more (a ball, a chair)
const double WALL_FIX_GAIN    = 0.3;  // move this fraction of the way to the wall's answer
const bool   USE_WALL_FIX     = true; // false = never correct the position from walls

// Robot geometry (measured 2026-11-17)
const double MM_PER_DEG     = 1.111; // wheel mm per motor degree (2:1 gearing)
const double MM_S_PER_PCT   = 8.0;   // ~800 mm/s at 100 %
const double TRACK_MM       = 225;
const double SENSOR_X_MM    = 214;   // distance sensor window, from the rear axle
const double SENSOR_Y_MM    = 104;   // ...and to the left of the centre line
const double ROLLER_X_MM    = 173;
const double INTAKE_Y_MM    = 10;
// Aim correction: the ball was landing on the sensor (left) side of the roller, so the
// robot aims AIM_SHIFT_MM further left. Ball still on the sensor side -> raise it;
// ball now on the right side of the roller -> lower it.
const double AIM_SHIFT_MM   = 20;   // 0 = ball hit the sensor side, 40 = missed to the left
const double SENSOR_LATENCY_MS = 0;  // direction correction for sensor delay (off: tune AIM_SHIFT_MM first)
const double READING_FIX_MM = 7.5;   // reading -> ball centre
// The shared intake is reversed=true; forward matches V4 false+reverse.
const directionType ROLLER_IN = forward;

// Detection
const double MAX_SEEN_MM     = 2000; // readings beyond this = nothing there
const double BALL_RANGE_MM   = 900;  // balls are only trusted this close
const double EDGE_MM         = 45;   // a jump between readings bigger than this...
const double EDGE_FRAC       = 0.07; // ...and 7 % of the reading = sharp edge
const double BEAM_HALF_DEG   = 7;
const double WIDTH_EXTRA_DEG = 8;    // ball may look this much wider than expected
const double MIN_WIDTH_FRAC  = 0.35; // ...and no narrower than this fraction
const double BALL_MERGE_MM   = 100;  // same ball seen twice
const double SKIP_RADIUS_MM  = 150;
const double APPEAR_DROP_MM  = 150;  // straight driving: reading drops this much...
const double APPEAR_MAX_MM   = 800;  // ...to under this = something appeared
const double EXAMINE_DEG     = 30;
const double EXAMINED_MM     = 250;  // don't re-examine the same spot

// Motion
const double SPIN_DPS     = 35;      // scan turn rate (average of 45 and the 25 check speed)
const double TOP_MM_S     = 400;     // 50 %
const double PUSH_MM_S    = 240;     // 30 %
const double START_MM_S   = 80;
const double ACCEL_MM     = 150;
const double DECEL_MM     = 150;
const double MIN_MM_S     = 50;
const double TURN_KP      = 3.0;
const double TURN_MIN_DPS = 10;
const double TURN_MAX_DPS = 90;
const double TURN_TOL_DEG = 1.0;
const double HOLD_KP      = 3.0;

// Safety
const double WALL_STOP_MM = 200;     // sensor this close while driving forward = stop
const double STUCK_FRAC   = 0.25;    // moved under 25 % of the expected distance...
const int    STUCK_TICKS  = 12;      // ...over ~0.4 s = stuck
const double BACKUP_MM    = 150;

// Collecting
const double ROLL_OVER_MM    = 400;  // drive this far past where the ball meets the roller (carpet slip)
const int    MAX_TRIES_SPOT  = 2;    // attempts at the same spot before giving up on it
const double SLOW_BEFORE_MM  = 100;  // at 30 % this far before the ball
const int    DWELL_MS        = 300;
const int TICK_MS = 33;              // the distance sensor updates ~30 times a second

// ============================================================ STATE
const double PI  = 3.14159265358979;
const double D2R = PI / 180.0;

struct Pt { double x; double y; };

double px = 0, py = 0, th = 0;       // pose
double prevL = 0, prevR = 0;
double gyroSign = -1;                // VEX IQ counts clockwise as +; fixed (checkGyroSign only warns)
bool   gyroWarn = false;
double curR = 0;                     // latest distance reading (every tick)
double lastDthDeg = 0;               // heading change in the last tick
int    balls = 0;
bool   stopNow = false;
const char* stopReason = "";
const char* stateText = "IDLE";
timer  runTimer;
double lastDrawMs = -1000;

const int MAX_ITEMS = 40;
Pt ballList[MAX_ITEMS]; int nBall = 0;
Pt skipList[MAX_ITEMS]; int nSkip = 0;
Pt examined[MAX_ITEMS]; int nExam = 0;
Pt tried[MAX_ITEMS]; int triedN[MAX_ITEMS]; int nTried = 0;   // collect attempts per spot

const int R_OK = 0, R_STOP = 1, R_BUMP = 2, R_STALL = 3, R_TIMEOUT = 4, R_WALL = 5, R_SEEN = 6;

// ============================================================ HELPERS
double dist2(Pt a, Pt b) { return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); }
Pt here() { Pt p = {px, py}; return p; }
double wrapDeg(double a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }
double clampD(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

Pt toWorld(double rx, double ry, double heading) {
  Pt p = {px + rx * cos(heading) - ry * sin(heading), py + rx * sin(heading) + ry * cos(heading)};
  return p;
}

bool nearList(Pt p, Pt* list, int n, double r) {
  for (int i = 0; i < n; i++) if (dist2(p, list[i]) < r) return true;
  return false;
}
void addTo(Pt* list, int& n, Pt p) { if (n < MAX_ITEMS) { list[n] = p; n++; } }
void removeAt(Pt* list, int& n, int i) { for (int k = i; k < n - 1; k++) list[k] = list[k + 1]; n--; }

// ============================================================ MOTORS
void spinPct(motor& m, double pct) {
  if (stopNow) { m.stop(brake); return; }
  pct = clampD(pct, -100, 100);
  if (pct >= 0) m.spin(forward, pct, percent);
  else          m.spin(reverse, -pct, percent);
}

// v = forward mm/s, w = turn deg/s (counterclockwise +)
void setMotion(double v, double w) {
  double diff = w * D2R * TRACK_MM / 2.0;
  spinPct(hw->left,  (v - diff) / MM_S_PER_PCT);
  spinPct(hw->right, (v + diff) / MM_S_PER_PCT);
}

void stopDrive() { hw->left.stop(brake); hw->right.stop(brake); }

// ============================================================ SCREEN / LOG
void drawScreen() {
  hw->brain.Screen.clearScreen();
  hw->brain.Screen.setCursor(1, 1);
  hw->brain.Screen.print("Balls: %d / %d", balls, MAX_BALLS);
  hw->brain.Screen.newLine();
  hw->brain.Screen.print("Time: %d / %d s", (int)runTimer.time(seconds), (int)TIME_LIMIT_S);
  hw->brain.Screen.newLine();
  hw->brain.Screen.print("%s", stateText);
  hw->brain.Screen.newLine();
  if (vision_reader) {
    VisionInput v = currentVision();
    hw->brain.Screen.print("Cam %c %dms d%d", v.command, (int)v.age_ms, (int)curR);
  } else hw->brain.Screen.print("d%d h%d seen%d%s", (int)curR, (int)(th / D2R), nBall, gyroWarn ? " GYRO?" : "");
}

void setState(const char* s) {
  stateText = s;
  if (state_reporter) state_reporter(vision_context, s, nullptr);
  collectorLog("EVT,%d,%s,x=%d,y=%d,h=%d\n", (int)runTimer.time(msec), s, (int)px, (int)py, (int)(th / D2R));
}
void reportCheck(const char* event) {
  if (state_reporter) state_reporter(vision_context, stateText, event);
}

void checkStop() {
  if (stopNow) return;
  if (abortRequested()) { stopNow = true; stopReason = "button stop"; }
  else if (balls >= MAX_BALLS)                          { stopNow = true; stopReason = "10 balls"; }
  else if (runTimer.time(seconds) >= TIME_LIMIT_S) { stopNow = true; stopReason = "time up"; }
  else if (hw->touch.pressing())                   { stopNow = true; stopReason = "touch stop"; }
}

// ============================================================ WALLS / POSITION
// Known walls (frame: x up the right wall, y to the left). Bottom and right are known from
// the start position; top and left are learned when a leg ends at them.
double wallBottomX = -(52 + START_BACK_GAP_MM);
double wallRightY  = -(135 + START_SIDE_GAP_MM);
double wallTopX = 0, wallLeftY = 0;
bool   haveTop = false, haveLeft = false;
bool   wallFixOn = true;
bool   newBall = false;   // set when the detector adds a NEW ball (ends sweeps)

// Called every tick: if the sensor points straight at a known wall, use the reading
// to correct x or y (small steps, and only when it roughly agrees).
void blendFix(double& coord, double implied, const char* name) {
  double err = implied - coord;
  if (fabs(err) < WALL_FIX_GATE_MM && fabs(err) > 5) {
    coord += WALL_FIX_GAIN * err;
    if (fabs(err) > 60) collectorLog("FIX,%d,%s,err=%d\n", (int)runTimer.time(msec), name, (int)err);
  }
}

void wallFix() {
  if (!USE_WALL_FIX || !wallFixOn || curR < 20 || curR > 1800) return;
  double hd = wrapDeg(th / D2R);
  // sensor window in the world; the beam points along the heading
  Pt s = toWorld(SENSOR_X_MM, SENSOR_Y_MM, th);
  double offX = s.x - px, offY = s.y - py;
  if (fabs(wrapDeg(hd - 180)) < WALL_FIX_TOL_DEG) blendFix(px, wallBottomX + curR - offX, "bottom");
  if (fabs(wrapDeg(hd + 90)) < WALL_FIX_TOL_DEG)  blendFix(py, wallRightY + curR - offY, "right");
  if (haveTop && fabs(hd) < WALL_FIX_TOL_DEG)     blendFix(px, wallTopX - curR - offX, "top");
  if (haveLeft && fabs(wrapDeg(hd - 90)) < WALL_FIX_TOL_DEG) blendFix(py, wallLeftY - curR - offY, "left");
}

bool outsideWalls(Pt p) {
  return p.x < wallBottomX + 20 || p.y < wallRightY + 20 ||
         p.x > wallBottomX + AREA_UP_MM || p.y > wallRightY + AREA_LEFT_MM ||
         (haveTop && p.x > wallTopX - 20) || (haveLeft && p.y > wallLeftY - 20);
}

// ============================================================ BALL LIST
void addBall(Pt p) {
  if (nearList(p, skipList, nSkip, SKIP_RADIUS_MM)) return;
  if (outsideWalls(p)) { collectorLog("OUTSIDE_WALLS,%d,x=%d,y=%d\n", (int)runTimer.time(msec), (int)p.x, (int)p.y); return; }
  int found = -1;
  for (int k = 0; k < nBall; k++) if (dist2(p, ballList[k]) < BALL_MERGE_MM) found = k;
  if (found >= 0) {
    ballList[found].x = (ballList[found].x + p.x) / 2;
    ballList[found].y = (ballList[found].y + p.y) / 2;
  } else {
    addTo(ballList, nBall, p);
    newBall = true;
    collectorLog("BALL_ADDED,%d,x=%d,y=%d,total=%d\n", (int)runTimer.time(msec), (int)p.x, (int)p.y, nBall);
  }
}

// ============================================================ LIVE DETECTOR
// Runs inside tick() whenever detOn is true (all turns in place).
bool   detOn = false;
bool   segOpen = false, prevValid = false, detWarm = false;
double prevReading = 0, segStartTh = 0, segLastTh = 0;
const int SEG_MAX = 120;
double segBuf[SEG_MAX], sortBuf[SEG_MAX];
int    segN = 0;

double edgeLimit(double a, double b) {
  double lim = EDGE_FRAC * (a < b ? a : b);
  return lim < EDGE_MM ? EDGE_MM : lim;
}

void openSeg() {
  segOpen = true; segN = 0;
  segBuf[0] = curR; segN = 1;
  segStartTh = th; segLastTh = th;
}

void closeSeg(bool risingEnd) {
  segOpen = false;
  if (segN < 3) return;              // too few readings to trust
  for (int i = 0; i < segN; i++) sortBuf[i] = segBuf[i];
  for (int i = 1; i < segN; i++) { double v = sortBuf[i]; int j = i - 1; while (j >= 0 && sortBuf[j] > v) { sortBuf[j + 1] = sortBuf[j]; j--; } sortBuf[j + 1] = v; }
  double med = sortBuf[segN / 2];
  double width = fabs(segLastTh - segStartTh) / D2R + fabs(lastDthDeg);
  double ballW = 2 * BEAM_HALF_DEG + 2 * atan(20.0 / (med + 20.0)) / D2R;
  const char* verdict = "BALL";
  if (!risingEnd)                          verdict = "NO_FAR_EDGE";   // something nearer came in after it
  else if (med > BALL_RANGE_MM)            verdict = "TOO_FAR";
  else if (width > ballW + WIDTH_EXTRA_DEG) verdict = "TOO_WIDE";
  else if (width < MIN_WIDTH_FRAC * ballW) verdict = "TOO_SMALL";
  collectorLog("PIECE,%d,d=%d,w=%.1f,expect=%.1f,n=%d,%s\n", (int)runTimer.time(msec), (int)med, width, ballW, segN, verdict);
  if (verdict[0] == 'B') {
    // the reading lags the beam: while turning, the ball was really a bit behind
    double mid = (segStartTh + segLastTh) / 2.0 - lastDthDeg * (SENSOR_LATENCY_MS / TICK_MS) * D2R;
    addBall(toWorld(SENSOR_X_MM + med + READING_FIX_MM, SENSOR_Y_MM, mid));
  }
}

void detectStep() {
  bool valid = curR >= 20 && curR < MAX_SEEN_MM;
  if (detWarm) {
    // first reading of a sweep: only remember it. A ball must be seen coming INTO the
    // beam (its near edge) during the sweep; a ball already half in the beam at the
    // start would give a cut-off, shifted position.
    detWarm = false;
  } else if (segOpen) {
    if (valid && fabs(curR - prevReading) <= edgeLimit(curR, prevReading)) {
      if (segN < SEG_MAX) { segBuf[segN] = curR; segN++; }
      segLastTh = th;
    } else {
      bool risingEnd = !valid || curR > prevReading;
      closeSeg(risingEnd);
      if (valid && curR < prevReading) openSeg();      // dropped onto something nearer
    }
  } else if (valid && (!prevValid || prevReading - curR > edgeLimit(curR, prevReading))) {
    openSeg();                                          // sudden drop: something came into the beam
  }
  prevReading = curR;
  prevValid = valid;
}

void detStart() { detOn = true; segOpen = false; prevValid = false; detWarm = true; }
void detStop()  { detOn = false; segOpen = false; }

// ============================================================ TICK
// Every loop iteration everywhere: odometry, distance reading, detector, stop check, screen.
void tick() {
  double l = hw->left.position(degrees), r = hw->right.position(degrees);
  double d = ((l - prevL) + (r - prevR)) / 2.0 * MM_PER_DEG;
  prevL = l; prevR = r;
  double newTh = gyroSign * hw->imu.rotation(degrees) * D2R;
  lastDthDeg = (newTh - th) / D2R;
  th = newTh;
  px += d * cos(th);
  py += d * sin(th);

  curR = hw->distance.objectDistance(mm);
  if (detOn) detectStep();
  wallFix();

  checkStop();
  if (stopNow) { stopDrive(); hw->intake.stop(); }
  if (runTimer.time(msec) - lastDrawMs > 500) { drawScreen(); lastDrawMs = runTimer.time(msec); }
  wait(TICK_MS, msec);
}

void settle() { stopDrive(); for (int i = 0; i < 8; i++) tick(); }

// ============================================================ MOTION
double profile(double moved, double remaining, double endSpeed) {
  double up = START_MM_S + (TOP_MM_S - START_MM_S) * (moved > 0 ? moved : 0) / ACCEL_MM;
  double down = endSpeed + (TOP_MM_S - endSpeed) * (remaining > 0 ? remaining : 0) / DECEL_MM;
  double v = TOP_MM_S;
  if (up < v) v = up;
  if (down < v) v = down;
  return v;
}

bool stuckCheck(int& ticks, double& lastMoved, double moved, double v) {
  ticks++;
  bool stuck = false;
  if (ticks >= STUCK_TICKS) {
    double expect = fabs(v) * ticks * TICK_MS / 1000.0;
    stuck = expect > 30 && fabs(moved - lastMoved) < STUCK_FRAC * expect;
    ticks = 0;
    lastMoved = moved;
  }
  return stuck;
}

// Turn in place to a heading. The detector runs the whole time.
int turnTo(double targetRad) {
  timer t; t.clear();
  int settled = 0, res = R_OK;
  double limitMs = fabs(wrapDeg((targetRad - th) / D2R)) / TURN_MAX_DPS * 2000 + 2000;
  detStart();
  while (settled < 5 && res == R_OK) {
    tick();
    if (stopNow) { res = R_STOP; break; }
    double err = wrapDeg((targetRad - th) / D2R);
    double rate = 0;
    if (fabs(err) > TURN_TOL_DEG) {
      rate = clampD(TURN_KP * err, -TURN_MAX_DPS, TURN_MAX_DPS);
      if (fabs(rate) < TURN_MIN_DPS) rate = rate > 0 ? TURN_MIN_DPS : -TURN_MIN_DPS;
      settled = 0;
    } else {
      settled++;
    }
    setMotion(0, rate);
    if (stopNow) res = R_STOP;
    else if (t.time(msec) > limitMs) res = R_TIMEOUT;
  }
  detStop();
  settle();
  return res;
}

// Turn deg (counterclockwise +) at a steady dps: the scan. Detector on.
// stopOnBall: the sweep ends as soon as a new ball is spotted (newBall).
int sweep(double deg, double dps, bool stopOnBall) {
  double startTh = th, rate = deg >= 0 ? dps : -dps;
  timer t; t.clear();
  double limitMs = fabs(deg) / dps * 2000 + 2000;
  int res = R_OK;
  newBall = false;
  detStart();
  while (fabs(th - startTh) / D2R < fabs(deg) && res == R_OK && !(stopOnBall && newBall)) {
    setMotion(0, rate);
    tick();
    if (stopNow) res = R_STOP;
    else if (t.time(msec) > limitMs) res = R_TIMEOUT;
  }
  detStop();
  settle();
  return res;
}

// Drive straight mm (negative = back), holding the heading.
//   stopAtMm > 0: forward only, stop with R_WALL when the sensor reads under it (2 in a row)
//   appear:       stop with R_SEEN when something suddenly appears in front
// Forward drives also stop on the bumper and when stuck.
int driveDist(double mm, double stopAtMm, bool appear) {
  Pt start = here();
  double hold = th, dir = mm >= 0 ? 1 : -1, goal = fabs(mm);
  timer t; t.clear();
  double limitMs = goal / MIN_MM_S * 1000 + 2000;
  int res = R_OK, wallN = 0, appearN = 0, stuckT = 0;
  double moved = 0, lastMoved = 0, before = curR < 20 ? MAX_SEEN_MM : curR;
  while (moved < goal && res == R_OK) {
    tick();
    if (stopNow) { res = R_STOP; break; }
    moved = dir * ((px - start.x) * cos(hold) + (py - start.y) * sin(hold));
    double v = profile(moved, goal - moved, MIN_MM_S);
    double w = clampD(HOLD_KP * wrapDeg((hold - th) / D2R), -TURN_MAX_DPS, TURN_MAX_DPS);
    setMotion(dir * v, w);

    double r = (curR < 20 || curR > MAX_SEEN_MM) ? MAX_SEEN_MM : curR;
    if (dir > 0 && stopAtMm > 0 && r < stopAtMm) wallN++; else wallN = 0;
    bool appeared = dir > 0 && appear && r < APPEAR_MAX_MM && before - r > APPEAR_DROP_MM &&
                    !nearList(toWorld(SENSOR_X_MM + r, SENSOR_Y_MM, th), examined, nExam, EXAMINED_MM);
    if (appeared) appearN++; else appearN = 0;
    if (appearN == 0) before = r;          // keep the reading from before the drop
    bool stuck = stuckCheck(stuckT, lastMoved, moved, v) && t.time(msec) > 600;

    if (stopNow) res = R_STOP;
    else if (dir > 0 && hw->bumper.pressing()) res = R_BUMP;
    else if (wallN >= 2) res = R_WALL;
    else if (appearN >= 2) res = R_SEEN;
    else if (dir > 0 && stuck) res = R_STALL;
    else if (t.time(msec) > limitMs) res = R_TIMEOUT;
  }
  settle();
  return res;
}

void backOff() { driveDist(-BACKUP_MM, 0, false); }

int goToPoint(Pt target) {
  double dx = target.x - px, dy = target.y - py, L = sqrt(dx * dx + dy * dy);
  if (L < 40) return R_OK;
  int res = turnTo(atan2(dy, dx));
  if (res == R_OK) res = driveDist(L, WALL_STOP_MM, false);
  if (res == R_WALL || res == R_BUMP || res == R_STALL) backOff();
  return res;
}

// ============================================================ COLLECTING
// Heading that puts the ball on the intake's centre line (plus the aim correction).
double aimHeading(Pt b) {
  double dx = b.x - px, dy = b.y - py, L = sqrt(dx * dx + dy * dy);
  double lateral = INTAKE_Y_MM - AIM_SHIFT_MM;
  if (L < fabs(lateral) + 1) L = fabs(lateral) + 1;
  return atan2(dy, dx) - asin(lateral / L);
}

double aheadMm(Pt b) { return (b.x - px) * cos(th) + (b.y - py) * sin(th); }


// Through the ball in one move: ramps up, 30 % near the ball, coasts to rest past it.
// Safety: bumper, stuck, and the sensor (the ball is outside its beam when close).
int driveThrough(double mm) {
  Pt start = here();
  wallFixOn = false;
  double hold = th, base = 0, peak = 0, moved = 0, lastMoved = 0;
  int nBase = 0, res = R_OK, wallN = 0, stuckT = 0;
  timer t; t.clear();
  double limitMs = mm / MIN_MM_S * 1000 + 2000;
  while (moved < mm && res == R_OK) {
    tick();
    moved = (px - start.x) * cos(hold) + (py - start.y) * sin(hold);
    double toSlow = mm - moved - ROLL_OVER_MM - SLOW_BEFORE_MM;
    double v = toSlow > 0 ? profile(moved, toSlow, PUSH_MM_S) : PUSH_MM_S;
    double w = clampD(HOLD_KP * wrapDeg((hold - th) / D2R), -TURN_MAX_DPS, TURN_MAX_DPS);
    setMotion(v, w);
    double rc = hw->intake.current(amp);
    if (nBase < 10) { base += rc; nBase++; } else if (rc > peak) peak = rc;
    if (curR >= 20 && curR < WALL_STOP_MM) wallN++; else wallN = 0;
    bool stuck = stuckCheck(stuckT, lastMoved, moved, v) && t.time(msec) > 600;
    if (stopNow) res = R_STOP;
    else if (hw->bumper.pressing()) res = R_BUMP;
    else if (wallN >= 2) res = R_WALL;
    else if (stuck) res = R_STALL;
    else if (t.time(msec) > limitMs) res = R_TIMEOUT;
  }
  hw->left.stop();                  // coast: a hard brake bounces the ball out
  hw->right.stop();
  for (int i = 0; i < 8; i++) { tick(); double rc = hw->intake.current(amp); if (rc > peak) peak = rc; }
  wallFixOn = true;
  collectorLog("ROLLER,%d,base=%.2f,peak=%.2f\n", (int)runTimer.time(msec), nBase > 0 ? base / nBase : 0, peak);
  return res;
}

int nearestBall() {
  int best = -1; double bestD = 1e9;
  for (int i = 0; i < nBall; i++) { double d = dist2(here(), ballList[i]); if (d < bestD) { bestD = d; best = i; } }
  return best;
}

// The distance sensor has already chosen and aimed at this candidate.
// CV only gives permission for the original drive-through; it cannot steer.
bool confirmCandidate() {
  stopDrive();
  setState("CHECK BALL (CV)"); hw->touch.on(purple); drawScreen();
  unsigned started = (unsigned)runTimer.time(msec);
  BallCheck check = beginBallCheck(currentVision(), started);
  while ((unsigned)runTimer.time(msec) - started < 1500 && !stopNow) {
    tick();
    if (stopNow) return false;
    if (hw->bumper.pressing()) { backOff(); return false; }
    check = ballCheckStep(check, currentVision(), (unsigned)runTimer.time(msec));
    if (ballApproved(check)) { reportCheck("CV approved"); return true; }
  }
  if (!stopNow && !visionLive(currentVision())) {
    stopNow = true; stopReason = "CV unavailable";
    reportCheck("No fresh CV data");
    stopDrive(); hw->intake.stop();
  } else if (!stopNow) {
    setState("CV SKIP: NO MATCH"); reportCheck("CV rejected candidate"); drawScreen();
  }
  return false;
}

// Returns true if a ball was collected.
bool collectBall(int idx) {
  Pt b = ballList[idx];
  removeAt(ballList, nBall, idx);
  // how many times have we gone for this spot already?
  int t = -1;
  for (int i = 0; i < nTried; i++) if (dist2(tried[i], b) < SKIP_RADIUS_MM + 50) t = i;
  if (t < 0 && nTried < MAX_ITEMS) { tried[nTried] = b; triedN[nTried] = 0; t = nTried; nTried++; }
  if (t >= 0) {
    triedN[t]++;
    if (triedN[t] > MAX_TRIES_SPOT) {
      collectorLog("EVT,%d,GIVE_UP_SPOT,x=%d,y=%d\n", (int)runTimer.time(msec), (int)b.x, (int)b.y);
      addTo(skipList, nSkip, b);
      return false;
    }
  }
  setState(vision_reader ? "AIM CANDIDATE" : "GO_TO_BALL");
  hw->touch.on(vision_reader ? purple : blue);
  collectorLog("EVT,%d,TARGET,x=%d,y=%d\n", (int)runTimer.time(msec), (int)b.x, (int)b.y);

  int res = turnTo(aimHeading(b));
  if (res != R_OK) return false;
  // the aiming turn may have re-detected this same ball: drop those copies
  for (int i = nBall - 1; i >= 0; i--) if (dist2(ballList[i], b) < 2 * BALL_MERGE_MM) removeAt(ballList, nBall, i);
  if (vision_reader && !confirmCandidate()) {
    if (!stopNow) addTo(skipList, nSkip, b);
    return false;
  }
  if (stopNow || abortRequested()) {
    stopNow = true; stopReason = "button stop"; stopDrive(); return false;
  }
  setState("COLLECT BALL");
  hw->touch.on(blue);
  res = driveThrough(aheadMm(b) - ROLLER_X_MM + ROLL_OVER_MM);
  if (res == R_WALL || res == R_BUMP || res == R_STALL) {
    collectorLog("EVT,%d,NOT_A_BALL,res=%d\n", (int)runTimer.time(msec), res);
    addTo(skipList, nSkip, b);
    backOff();
    return false;
  }
  if (res != R_OK) return false;
  timer dwell; dwell.clear();
  while (dwell.time(msec) < DWELL_MS && !stopNow) tick();
  if (stopNow) return false;

  // No intake check: the drive-through finished without the bumper (or the wall /
  // stuck safety) stopping it, so the ball is in. Count it.
  balls++;
  hw->touch.on(green);
  collectorLog("EVT,%d,COLLECTED,balls=%d\n", (int)runTimer.time(msec), balls);
  drawScreen();
  return true;
}

// Something appeared while driving: sweep across it.
void examine() {
  setState("EXAMINE");
  hw->touch.on(purple);
  addTo(examined, nExam, toWorld(SENSOR_X_MM + curR, SENSOR_Y_MM, th));
  double h0 = th;
  if (turnTo(h0 - EXAMINE_DEG * D2R) == R_OK) sweep(2 * EXAMINE_DEG, SPIN_DPS, true);
  if (!newBall) turnTo(h0);
}

// Something close straight ahead ended a route drive. Before calling it a wall, sweep
// across it: if the detector finds a ball there, it was a ball. Returns true if so.
bool aheadIsBall() {
  setState("WALL_OR_BALL");
  double r = curR < 20 ? LEG_STOP_MM : curR;
  Pt spot = toWorld(SENSOR_X_MM + r, SENSOR_Y_MM, th);
  int before = nBall;
  double h0 = th;
  if (turnTo(h0 - EXAMINE_DEG * D2R) == R_OK) sweep(2 * EXAMINE_DEG, SPIN_DPS, true);
  if (!newBall) turnTo(h0);
  bool ball = false;
  for (int i = 0; i < nBall; i++) if (dist2(ballList[i], spot) < 250) ball = true;
  collectorLog("EVT,%d,WALL_CHECK,%s,new_balls=%d\n", (int)runTimer.time(msec), ball ? "BALL" : "WALL", nBall - before);
  return ball;
}

// Which way does the gyro count? A short LEFT turn, watched.
void checkGyroSign() {
  setState("GYRO_CHECK");
  double r0 = hw->imu.rotation(degrees);
  timer t; t.clear();
  while (t.time(msec) < 700) { spinPct(hw->left, -15); spinPct(hw->right, 15); tick(); }
  stopDrive();
  for (int i = 0; i < 10; i++) tick();
  double r1 = hw->imu.rotation(degrees);
  // VEX IQ counts clockwise as +, so a LEFT turn should make rotation go DOWN. Warn if not.
  gyroWarn = fabs(r1 - r0) > 3 && r1 > r0;
  collectorLog("GYRO,%d,before=%.1f,after=%.1f,warn=%d\n", (int)runTimer.time(msec), r0, r1, gyroWarn ? 1 : 0);
  tick();
  turnTo(0);
}

// ============================================================ SEARCH PATTERN
// Usable size: the area limit, or a wall the robot found, whichever is nearer.
double upLimit()   { double u = AREA_UP_MM;   if (haveTop  && wallTopX - wallBottomX < u) u = wallTopX - wallBottomX;  return u; }
double leftLimit() { double l = AREA_LEFT_MM; if (haveLeft && wallLeftY - wallRightY < l) l = wallLeftY - wallRightY;  return l; }

// Lane i's distance from the right edge, or -1 if there is no lane i.
double laneLat(int i) {
  double maxL = leftLimit() - EDGE_KEEP_MM;
  double lat = LANE_FIRST_MM + i * LANE_GAP_MM;
  double prev = LANE_FIRST_MM + (i - 1) * LANE_GAP_MM;
  if (lat <= maxL) return lat;
  if (i > 0 && prev < maxL - 400) return maxL;        // last lane pulled in to reach the edge
  return -1;
}

// Next stop along lane i after `last` (going up if `up`, else down). False = lane done.
bool nextStop(int i, double last, bool up, double& along) {
  double maxA = upLimit() - EDGE_KEEP_MM;
  double first = (i % 2 == 0) ? STOP_FIRST_MM : STOP_FIRST_MM + STOP_GAP_MM / 2;
  bool found = false;
  double best = up ? 1e9 : -1e9, lastGrid = first;
  double a = first;
  while (a <= maxA) {
    if (up ? (a > last + 1 && a < best) : (a < last - 1 && a > best)) { best = a; found = true; }
    lastGrid = a;
    a += STOP_GAP_MM;
  }
  if (maxA - lastGrid > 400 && (up ? (maxA > last + 1 && maxA < best) : (maxA < last - 1 && maxA > best))) {
    best = maxA; found = true;                        // extra stop so the far edge is covered
  }
  along = best;
  return found;
}

// Learn a wall straight ahead (only for the two unknown walls: top and left).
void learnWallAhead() {
  double hd = wrapDeg(th / D2R);
  double r = (curR >= 20 && curR < MAX_SEEN_MM) ? curR : LEG_STOP_MM;
  if (fabs(hd) < 20 && !haveTop)         { wallTopX = px + SENSOR_X_MM + r; haveTop = true; }
  if (fabs(hd - 90) < 20 && !haveLeft)   { wallLeftY = py + SENSOR_X_MM + r; haveLeft = true; }
  collectorLog("EVT,%d,WALL_AHEAD,h=%d,top=%d,left=%d\n", (int)runTimer.time(msec), (int)hd,
         haveTop ? (int)wallTopX : -1, haveLeft ? (int)wallLeftY : -1);
}

// ============================================================ MAIN
bool waitForMissionStart() {
  MissionStartGate gate = {true, false, false};
  int lastView = -1;
  while (!gate.ready) {
    if (abortRequested()) { stopReason = "Start cancelled"; return false; }
    bool cameraReady = !vision_reader || visionLive(currentVision());
    bool pressed = hw->touch.pressing();
    gate = missionStartStep(gate, cameraReady, pressed);
    int view = !cameraReady ? 0 : pressed ? (gate.press_armed ? 1 : 2) : 3;
    if (view != lastView) {
      setState(!cameraReady ? "WAIT FOR CAMERA" : pressed ? "RELEASE TOUCH LED" : "PRESS TOUCH LED");
      hw->brain.Screen.clearScreen(); hw->brain.Screen.setCursor(1, 1);
      hw->brain.Screen.print(vision_reader ? "SENSOR + CV V5.1" : "V4 SENSOR ONLY");
      hw->brain.Screen.newLine();
      hw->brain.Screen.print(!cameraReady ? "WAIT FOR CAMERA" : pressed ? "Release TouchLED" : "Press TouchLED");
      hw->brain.Screen.newLine();
      hw->brain.Screen.print(!cameraReady ? "Run PC script 3" : view == 2 ? "then tap to start" : "Check: back");
      hw->touch.on(cameraReady ? white : yellow);
      lastView = view;
    }
    wait(20, msec);
  }
  return !abortRequested();
}

void runMission() {
  // Reset all mission state so selecting V4 again starts a clean run.
  px = py = th = prevL = prevR = curR = lastDthDeg = 0;
  gyroSign = -1; gyroWarn = false;
  balls = nBall = nSkip = nExam = nTried = 0;
  stopNow = false; stopReason = ""; stateText = "IDLE";
  lastDrawMs = -1000;
  wallBottomX = -(52 + START_BACK_GAP_MM);
  wallRightY = -(135 + START_SIDE_GAP_MM);
  wallTopX = wallLeftY = 0;
  haveTop = haveLeft = newBall = false; wallFixOn = true;
  detOn = segOpen = prevValid = detWarm = false;
  prevReading = segStartTh = segLastTh = 0; segN = 0;
  runTimer.clear();
  stopDrive(); hw->intake.stop();
  if (!hw->imu.installed() || !hw->distance.installed() ||
      !hw->left.installed() || !hw->right.installed() || !hw->intake.installed()) {
    hw->brain.Screen.clearScreen();
    hw->brain.Screen.print("V4 missing device");
    stopReason = "V4 missing device";
    return;
  }

  hw->brain.Screen.clearScreen();
  hw->brain.Screen.setCursor(1, 1);
  setState("CALIBRATING");
  hw->brain.Screen.print("Calibrating...");
  hw->brain.Screen.newLine();
  hw->brain.Screen.print("keep still");
  hw->imu.calibrate();
  // Give the SDK time to enter calibration, with a bounded wait and stop checks.
  timer calibration; calibration.clear();
  do {
    if (abortRequested()) return;
    if (calibration.time(msec) > 10000) {
      stopReason = "Calibration timeout";
      hw->brain.Screen.clearScreen(); hw->brain.Screen.print("%s", stopReason); return;
    }
    wait(20, msec);
  } while (calibration.time(msec) < 1000 || hw->imu.isCalibrating());

  // Missing/slow camera data waits here instead of returning to IDLE and
  // recalibrating. Connecting the camera alone can never start movement.
  if (!waitForMissionStart()) return;

  hw->imu.setRotation(0, degrees);
  hw->left.resetPosition();
  hw->right.resetPosition();
  prevL = 0; prevR = 0;
  px = 0; py = 0; th = 0;
  runTimer.clear();
  hw->intake.spin(ROLLER_IN, 100, percent);   // runs the whole time: holds the balls in

  // Start: off the bottom wall, gyro check
  setState("START");
  driveDist(START_FORWARD_MM, WALL_STOP_MM, false);
  if (!stopNow) checkGyroSign();

  int lane = 0;
  bool up = true;                 // lane 0 goes up, lane 1 down, ...
  double lastAlong = -1e9;        // last stop done on this lane
  bool atStop = false;            // arrived at a stop: scan it
  int scansHere = 0, triesStop = 0;
  bool finished = false;

  while (!finished && !stopNow) {
    checkStop();
    if (stopNow) break;
    if (nBall > 0) {
      // ---- known balls first: nearest one
      collectBall(nearestBall());
    } else if (atStop) {
      // ---- scan here: a full 360 that ends early on a new ball
      setState("SCAN_360");
      hw->touch.on(green);
      sweep(SCAN_DEG, SPIN_DPS, true);
      scansHere++;
      if (!newBall || scansHere >= MAX_SCANS_STOP) atStop = false;   // nothing new: stop done
    } else {
      // ---- go to the next stop
      double along = 0;
      if (laneLat(lane) < 0) {
        finished = true;
      } else if (!nextStop(lane, lastAlong, up, along)) {
        lane++;                                        // lane done: next lane, other direction
        up = !up;
        lastAlong = up ? -1e9 : 1e9;
        collectorLog("EVT,%d,NEXT_LANE,%d\n", (int)runTimer.time(msec), lane);
      } else {
        Pt target = {wallBottomX + along, wallRightY + laneLat(lane)};
        setState("TO_STOP");
        collectorLog("EVT,%d,STOP,lane=%d,along=%d,x=%d,y=%d\n", (int)runTimer.time(msec), lane, (int)along, (int)target.x, (int)target.y);
        double dx = target.x - px, dy = target.y - py, L = sqrt(dx * dx + dy * dy);
        int res = R_OK;
        if (L > 60) {
          res = turnTo(atan2(dy, dx));
          if (res == R_OK) res = driveDist(L, LEG_STOP_MM, true);
        }
        if (res == R_OK) {
          atStop = true; scansHere = 0; triesStop = 0;
          lastAlong = along;
        } else if (res == R_SEEN) {
          examine();                                   // the loop collects anything it found
        } else if (res == R_WALL && aheadIsBall()) {
          // a ball, not a wall: collected next, then on to the same stop
        } else if (res == R_WALL || res == R_BUMP || res == R_STALL || res == R_TIMEOUT) {
          if (res == R_WALL) learnWallAhead();
          if (res != R_WALL) backOff();
          triesStop++;
          if (res == R_WALL || triesStop >= 2) {      // can't reach it: scan from here instead
            atStop = true; scansHere = 0; triesStop = 0;
            lastAlong = along;
          }
        }
      }
    }
  }
  if (!stopNow) stopReason = "area covered";

  stopDrive();
  hw->intake.stop();
  hw->touch.on(red);
  setState(stopReason);
  drawScreen();
  collectorLog("EVT,%d,DONE,%s,balls=%d\n", (int)runTimer.time(msec), stopReason, balls);
  return;
}

} // namespace collector_v4

const char* runCollectorV4(CollectorDevices& devices, VisionReader reader, void* context, CollectorReporter reporter) {
    collector_v4::hw = &devices;
    collector_v4::vision_reader = reader;
    collector_v4::vision_context = context;
    collector_v4::state_reporter = reporter;
    collector_v4::runMission();
    devices.left.stop(vex::brakeType::brake);
    devices.right.stop(vex::brakeType::brake);
    devices.intake.stop(vex::brakeType::coast);
    collector_v4::hw = nullptr;
    collector_v4::vision_reader = nullptr;
    collector_v4::vision_context = nullptr;
    collector_v4::state_reporter = nullptr;
    return collector_v4::stopReason[0] ? collector_v4::stopReason : "V4 cancelled";
}
