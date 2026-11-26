/*
 * Path_Reproduction - drive a designed path with straights, arcs, on-the-spot
 * rotations, speed changes and a reversal, using only the wheel encoders.
 * Group project 04 "Path Reproduction": the same path and the same controller
 * run in the Digital Twin (../DigitalTwin_PathReproduction, ../twin/twin.py),
 * so the real robot and the twin can be compared sample by sample.
 * ------------------------------------------------------------------
 * The path (start: axle midpoint at the origin, facing +x; units mm, deg):
 *   1 straight 200          max PWM 30  (slow)
 *   2 arc left  R80, 180    max PWM 35  (wide curve, wheels at different speeds)
 *   3 straight 100          max PWM 50  (fast)
 *   4 rotate  -90 (CW)      max PWM 35
 *   5 straight -100         max PWM 35  (reverse)
 *   6 rotate  +90 (CCW)     max PWM 35
 *   7 straight 100          max PWM 40
 *   8 arc left  R60, 90     max PWM 35  (tight curve: inner wheel near deadband)
 *   9 rotate  +90 (CCW)     max PWM 35
 *  10 straight 60           max PWM 30  -> back at the start, facing +x
 *
 * One controller for every segment type:
 *   Each segment becomes a left and a right wheel distance (sL, sR in mm):
 *     straight d:          sL = sR = d
 *     rotate a:            sL = -a*B/2,  sR = +a*B/2
 *     arc R, a (+ = left): sL = |a|*R - a*B/2,  sR = |a|*R + a*B/2
 *   Targets in counts: T = s / (2*pi*r) * 358.3 (r = 16.36, B = 83.48).
 *   Progress of each wheel p = counts / T; p = distance-weighted mean.
 *   Speed:    base = K_DIST * (1 - p) * Tmax, limited to [MIN_PWM, max PWM]
 *   Each wheel: magnitude = FF_DEADBAND + (base - FF_DEADBAND) * |T|/Tmax
 *             (the slower wheel of an arc gets proportionally less PWM, plus
 *              a deadband offset so it still turns)
 *   Sync:     corr = K_SYNC * (pL - pR) * Tmax   (wheel ahead slows down)
 *   PWM:      left = sign(TL)*(magL - corr),  right = sign(TR)*(magR + corr)
 *   Done when p >= 1: motors off, SETTLE_MS still, (CHECKPOINT_PAUSE_MS),
 *   next segment.  For straights this is exactly the Go_Straight controller,
 *   for rotations exactly the Rotate controller.
 *
 * Usage (no button): place the robot, power on, join Wi-Fi CSY, run
 * Path_Reproduction/wifi_logger.py. 10 s countdown, then the path runs.
 * Nothing is sent over Wi-Fi while moving; the record (20 ms rows) is sent
 * after the robot stops, and again whenever a laptop connects.
 * CSV: run,time_ms,segment,left_pwm,right_pwm,left_counts,right_counts,x_mm,y_mm,theta_rad
 * Set CHECKPOINT_PAUSE_MS to e.g. 3000 to get a beep and a still robot after
 * every segment, long enough to mark its position on the paper.
 */

#include <stdarg.h>
#include <math.h>
#include <esp_random.h>
#include <Preferences.h>
#include "RobotWifiAP.h"
#include "Robot.h"
#include "TaskTimer.h"

#define WIFI_SSID     "CSY"
#define WIFI_PASSWORD "csy12345"
RobotWifiAP_c server(WIFI_SSID, WIFI_PASSWORD, 80);

// ---------------------------- the path -----------------------------------
enum SegType { SEG_STRAIGHT, SEG_ROTATE, SEG_ARC };
struct Segment {
  SegType type;
  float a;          // straight: distance mm (signed); rotate/arc: angle deg (+ = CCW/left)
  float radius;     // arc radius mm (arcs only)
  float max_pwm;    // speed limit for this segment
};
const Segment PATH[] = {
  {SEG_STRAIGHT,  200.0f,  0.0f, 30.0f},
  {SEG_ARC,       180.0f, 80.0f, 35.0f},
  {SEG_STRAIGHT,  100.0f,  0.0f, 50.0f},
  {SEG_ROTATE,    -90.0f,  0.0f, 35.0f},
  {SEG_STRAIGHT, -100.0f,  0.0f, 35.0f},
  {SEG_ROTATE,     90.0f,  0.0f, 35.0f},
  {SEG_STRAIGHT,  100.0f,  0.0f, 40.0f},
  {SEG_ARC,        90.0f, 60.0f, 35.0f},
  {SEG_ROTATE,     90.0f,  0.0f, 35.0f},
  {SEG_STRAIGHT,   60.0f,  0.0f, 30.0f},
};
const int N_SEGMENTS = sizeof(PATH) / sizeof(PATH[0]);

// ---------------- calibrated geometry (Exercises 2 and 3) ----------------
const float WHEEL_RADIUS_MM     = 16.36f;
const float WHEEL_SEPARATION_MM = 83.48f;
const float ENCODER_CPR         = 358.3f;

// ------------------------- controller settings ---------------------------
// Keep identical to DigitalTwin_PathReproduction/PathPlan.pde and twin/twin.py
const float MIN_PWM        = 25.0f;
const float FF_DEADBAND    = 15.0f;
const float K_DIST         = 0.3f;
const float K_SYNC         = 0.5f;
const float MAX_CORRECTION = 10.0f;
const unsigned long RAMP_MS            = 200;
const unsigned long SETTLE_MS          = 400;
const unsigned long CHECKPOINT_PAUSE_MS = 0;     // e.g. 3000 to mark positions
const unsigned long SEGMENT_TIMEOUT_MS = 10000;
const unsigned long START_DELAY_MS     = 10000;
const unsigned long PRE_ROLL_MS        = 500;
const unsigned long POST_ROLL_MS       = 1000;
const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long LOG_INTERVAL_MS     = 20;
const int MAX_STORED_ROWS = 2500;              // 50 s of record
const int REPLAY_LINES_PER_LOOP = 20;
// -------------------------------------------------------------------------

Robot_c robot;
TaskTimer_c control_timer(CONTROL_INTERVAL_MS);
TaskTimer_c log_timer(LOG_INTERVAL_MS);
Preferences prefs;

enum State { WAITING, PRE_ROLL, MOVING, SETTLING, POST_ROLL, STOPPED };
State state = WAITING;

int seg_index = 0, seg_label = 0;
unsigned long boot_ms = 0, start_ms = 0, seg_start_ms = 0, settle_start_ms = 0, motors_off_ms = 0;
int last_countdown_s = -1, run_num = 0, trial_num = 0;
int32_t start_left = 0, start_right = 0, seg_left0 = 0, seg_right0 = 0, prev_left = 0, prev_right = 0;
float target_left = 0.0f, target_right = 0.0f;     // signed counts for this segment
float left_pwm = 0.0f, right_pwm = 0.0f;
float odom_x = 0.0f, odom_y = 0.0f, odom_theta = 0.0f;
const char* end_reason = "";

struct StoredRow {
  uint32_t time_ms;
  uint8_t segment;
  int16_t left_pwm_x10, right_pwm_x10;
  int32_t left_counts, right_counts;
  float x, y, theta;
};
StoredRow stored_rows[MAX_STORED_ROWS];
int stored_count = 0;
bool stored_overflow = false;
uint32_t boot_id = 0;
char start_line[260], motors_off_line[120], stop_line[220];
bool run_finished = false, replay_active = false, was_connected = false, checkpoint_beeped = false;
int replay_index = -2;

const char* CSV_HEADER =
  "run,time_ms,segment,left_pwm,right_pwm,left_counts,right_counts,x_mm,y_mm,theta_rad\n";

bool robotMoving() { return state == PRE_ROLL || state == MOVING || state == SETTLING || state == POST_ROLL; }

void logPrintf(const char* format, ...) {
  char buffer[260];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  if (!robotMoving()) server.send(buffer);
  Serial.print(buffer);
}

void showStateLED() {
  if (state == WAITING)        robot.setLED(0, 0, 255, 40);
  else if (state == PRE_ROLL)  robot.setLED(255, 200, 0, 40);
  else if (state == MOVING)    robot.setLED(0, 255, 0, 40);
  else if (state == SETTLING)  robot.setLED(0, 120, 120, 40);
  else                         robot.setLED(255, 0, 0, 40);
}

void nextTrialNumber() {
  prefs.begin("path", false);
  trial_num = prefs.getInt("trial", 0) + 1;
  prefs.putInt("trial", trial_num);
  prefs.end();
}

float countsPerMm() { return ENCODER_CPR / (2.0f * PI * WHEEL_RADIUS_MM); }

// Wheel distances (mm) of one segment.
void wheelDistances(const Segment &s, float &sl, float &sr) {
  if (s.type == SEG_STRAIGHT) {
    sl = sr = s.a;
  } else {
    float ang = s.a * PI / 180.0f;
    float arc = (s.type == SEG_ARC) ? fabsf(ang) * s.radius : 0.0f;
    sl = arc - ang * WHEEL_SEPARATION_MM / 2.0f;
    sr = arc + ang * WHEEL_SEPARATION_MM / 2.0f;
  }
}

void startSegment(unsigned long now, int32_t left_counts, int32_t right_counts) {
  float sl, sr;
  wheelDistances(PATH[seg_index], sl, sr);
  target_left = sl * countsPerMm();
  target_right = sr * countsPerMm();
  seg_left0 = left_counts;
  seg_right0 = right_counts;
  seg_start_ms = now;
  seg_label = seg_index + 1;
  state = MOVING;
  showStateLED();
}

void stopMotors() {
  robot.setMotorPWM(0, 0);
  left_pwm = right_pwm = 0.0f;
}

void endMotion(unsigned long now, const char* reason) {
  stopMotors();
  end_reason = reason;
  motors_off_ms = now;
  seg_label = N_SEGMENTS + 1;
  state = POST_ROLL;
  showStateLED();
  robot.playTone(880, 150);
  snprintf(motors_off_line, sizeof(motors_off_line),
           "# MOTORS_OFF boot=%08lx run=%d time_ms=%lu reason=%s segments_done=%d",
           (unsigned long)boot_id, run_num, now - start_ms, reason, seg_index);
  Serial.printf("%s\n", motors_off_line);
}

void updateOdometry(int32_t left_counts, int32_t right_counts) {
  float mm_per_count = 1.0f / countsPerMm();
  float sl = (left_counts - prev_left) * mm_per_count;
  float sr = (right_counts - prev_right) * mm_per_count;
  prev_left = left_counts;
  prev_right = right_counts;
  float ds = 0.5f * (sl + sr);
  odom_x += ds * cosf(odom_theta);
  odom_y += ds * sinf(odom_theta);
  odom_theta += (sr - sl) / WHEEL_SEPARATION_MM;
}

// One 10 ms control step of the current segment (same in the Digital Twin).
void driveSegment(unsigned long now, int32_t left_counts, int32_t right_counts) {
  const Segment &s = PATH[seg_index];
  float dl = (float)(left_counts - seg_left0);
  float dr = (float)(right_counts - seg_right0);
  float al = fabsf(target_left), ar = fabsf(target_right);
  float tmax = (al > ar) ? al : ar;
  float pl = (al > 0.5f) ? dl / target_left : 1.0f;
  float pr = (ar > 0.5f) ? dr / target_right : 1.0f;
  float p = (al * pl + ar * pr) / (al + ar);

  if (p >= 1.0f) {
    stopMotors();
    state = SETTLING;
    settle_start_ms = now;
    checkpoint_beeped = false;
    showStateLED();
    robot.playTone(660, 40);
    return;
  }
  if (now - seg_start_ms >= SEGMENT_TIMEOUT_MS) {
    endMotion(now, "timeout");
    return;
  }

  float base = K_DIST * (1.0f - p) * tmax;
  if (base > s.max_pwm) base = s.max_pwm;
  if (base < MIN_PWM) base = MIN_PWM;
  unsigned long t = now - seg_start_ms;
  if (t < RAMP_MS) base = MIN_PWM + (base - MIN_PWM) * (float)t / (float)RAMP_MS;

  float mag_l = FF_DEADBAND + (base - FF_DEADBAND) * al / tmax;
  float mag_r = FF_DEADBAND + (base - FF_DEADBAND) * ar / tmax;
  float corr = K_SYNC * (pl - pr) * tmax;
  if (corr > MAX_CORRECTION) corr = MAX_CORRECTION;
  if (corr < -MAX_CORRECTION) corr = -MAX_CORRECTION;

  left_pwm  = ((target_left  >= 0.0f) ? 1.0f : -1.0f) * (mag_l - corr);
  right_pwm = ((target_right >= 0.0f) ? 1.0f : -1.0f) * (mag_r + corr);
  robot.setMotorPWM(left_pwm, right_pwm);
}

void formatRow(const StoredRow &r, char* out, size_t len) {
  snprintf(out, len, "%d,%lu,%u,%.1f,%.1f,%ld,%ld,%.2f,%.2f,%.5f\n",
           run_num, (unsigned long)r.time_ms, (unsigned)r.segment,
           r.left_pwm_x10 / 10.0f, r.right_pwm_x10 / 10.0f,
           (long)r.left_counts, (long)r.right_counts, r.x, r.y, r.theta);
}

void storeAndPrintRow(unsigned long elapsed, int32_t left_counts, int32_t right_counts) {
  StoredRow r;
  r.time_ms = elapsed;
  r.segment = (uint8_t)seg_label;
  r.left_pwm_x10 = (int16_t)lroundf(left_pwm * 10.0f);
  r.right_pwm_x10 = (int16_t)lroundf(right_pwm * 10.0f);
  r.left_counts = left_counts;
  r.right_counts = right_counts;
  r.x = odom_x; r.y = odom_y; r.theta = odom_theta;
  if (stored_count < MAX_STORED_ROWS) stored_rows[stored_count++] = r;
  else stored_overflow = true;
  char line[140];
  formatRow(r, line, sizeof(line));
  Serial.print(line);
}

void startRun(unsigned long now) {
  robot.getEncoders();
  start_left = robot.getLeftEncoderCount();
  start_right = robot.getRightEncoderCount();
  start_ms = now;
  run_num++;
  prev_left = prev_right = 0;
  odom_x = odom_y = odom_theta = 0.0f;
  seg_index = 0;
  seg_label = 0;
  stored_count = 0;
  stored_overflow = false;
  run_finished = false;
  state = PRE_ROLL;
  showStateLED();
  snprintf(start_line, sizeof(start_line),
           "# START boot=%08lx run=%d test=path trial=%d segments=%d r_mm=%.2f B_mm=%.2f "
           "min_pwm=%.0f ff=%.0f k_dist=%.2f k_sync=%.2f settle_ms=%lu checkpoint_ms=%lu",
           (unsigned long)boot_id, run_num, trial_num, N_SEGMENTS, WHEEL_RADIUS_MM,
           WHEEL_SEPARATION_MM, MIN_PWM, FF_DEADBAND, K_DIST, K_SYNC, SETTLE_MS, CHECKPOINT_PAUSE_MS);
  Serial.printf("%s\n%s", start_line, CSV_HEADER);
}

void startReplay() {
  if (!run_finished || replay_active) return;
  replay_active = true;
  replay_index = -2;
}

void serviceReplay() {
  if (!replay_active) return;
  if (!server.clientConnected()) { replay_active = false; return; }
  char line[300];
  for (int n = 0; n < REPLAY_LINES_PER_LOOP; n++) {
    if (replay_index == -2) {
      snprintf(line, sizeof(line), "%s replay=1%s\n", start_line, stored_overflow ? " buffer_full=1" : "");
    } else if (replay_index == -1) {
      snprintf(line, sizeof(line), "%s", CSV_HEADER);
    } else if (replay_index < stored_count) {
      formatRow(stored_rows[replay_index], line, sizeof(line));
    } else {
      snprintf(line, sizeof(line), "%s replay=1\n%s replay=1\n", motors_off_line, stop_line);
      server.send(line);
      replay_active = false;
      return;
    }
    if (!server.send(line)) { replay_active = false; return; }
    replay_index++;
  }
}

void checkReplayTriggers() {
  bool connected = server.clientConnected();
  if (connected && !was_connected && run_finished) startReplay();
  was_connected = connected;
  while (server.available() > 0) {
    int c = server.read();
    if (c == 'R' && run_finished) startReplay();
  }
}

void setup() {
  Serial.begin(115200);
  if (!server.begin()) Serial.println("Warning: Wi-Fi access point failed to start");
  robot.initialise();
  showStateLED();
  boot_id = esp_random();
  nextTrialNumber();
  start_line[0] = motors_off_line[0] = stop_line[0] = '\0';
  Serial.println("Path_Reproduction ready - the path starts automatically");
  boot_ms = millis();
}

void loop() {
  unsigned long now = millis();

  if (!robotMoving()) {
    server.update();
    checkReplayTriggers();
    serviceReplay();
  }

  if (state == WAITING) {
    unsigned long waited = now - boot_ms;
    if (waited >= START_DELAY_MS) {
      startRun(now);
    } else {
      int remaining_s = (int)((START_DELAY_MS - waited + 999) / 1000);
      if (remaining_s != last_countdown_s) {
        last_countdown_s = remaining_s;
        logPrintf("# starting in %d s (path, %d segments, trial %d)\n", remaining_s, N_SEGMENTS, trial_num);
        if (remaining_s <= 3) {
          robot.setLED(255, 200, 0, 40);
          robot.playTone(440, 40);
        }
      }
    }
    return;
  }
  if (state == STOPPED) return;

  if (!control_timer.isReady(now)) return;
  control_timer.resetTimer(now);
  unsigned long elapsed = now - start_ms;

  robot.getEncoders();
  int32_t left_counts = robot.getLeftEncoderCount() - start_left;
  int32_t right_counts = robot.getRightEncoderCount() - start_right;
  updateOdometry(left_counts, right_counts);

  if (state == PRE_ROLL && elapsed >= PRE_ROLL_MS) {
    robot.playTone(660, 50);
    startSegment(now, left_counts, right_counts);
  } else if (state == MOVING) {
    driveSegment(now, left_counts, right_counts);
  } else if (state == SETTLING && now - settle_start_ms >= SETTLE_MS + CHECKPOINT_PAUSE_MS) {
    seg_index++;
    if (seg_index >= N_SEGMENTS) endMotion(now, "done");
    else startSegment(now, left_counts, right_counts);
  }
  if (state == SETTLING && CHECKPOINT_PAUSE_MS > 0 && !checkpoint_beeped &&
      now - settle_start_ms >= SETTLE_MS) {
    checkpoint_beeped = true;
    robot.playTone(1200, 60);              // "mark the position now"
  }

  bool finished = (state == POST_ROLL && now - motors_off_ms >= POST_ROLL_MS);
  if (log_timer.isReady(now) || finished) {
    log_timer.resetTimer(now);
    storeAndPrintRow(elapsed, left_counts, right_counts);
  }

  if (finished) {
    float wrapped = fmodf(odom_theta * 180.0f / PI, 360.0f);
    if (wrapped > 180.0f) wrapped -= 360.0f;
    if (wrapped <= -180.0f) wrapped += 360.0f;
    snprintf(stop_line, sizeof(stop_line),
             "# STOP boot=%08lx run=%d time_ms=%lu dir=path trial=%d reason=%s "
             "odom_x_mm=%.2f odom_y_mm=%.2f odom_theta_deg=%.2f odom_dtheta_wrapped_deg=%.2f",
             (unsigned long)boot_id, run_num, elapsed, trial_num, end_reason,
             odom_x, odom_y, odom_theta * 180.0f / PI, wrapped);
    Serial.printf("%s\n", stop_line);
    state = STOPPED;
    run_finished = true;
    showStateLED();
    robot.playTone(990, 80);
    if (server.clientConnected()) startReplay();
  }
}
