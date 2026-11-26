/*
 * System_error - drive one closed rectangle with the Pololu 3Pi+ using ONLY
 * the wheel encoders (no line following), then report where odometry thinks
 * the robot ended. Exercise 4 (bidirectional square / UMBmark-style test).
 * ------------------------------------------------------------------
 * EXPERIMENT SETTING (change, upload, test):
 *   LOOP_DIRECTION = +1  anticlockwise (CCW, 逆时针)
 *   LOOP_DIRECTION = -1  clockwise     (CW,  顺时针)
 *
 * Start pose: axle midpoint on the BOTTOM-LEFT corner of the rectangle.
 *   CCW: face along the bottom edge (to the right). Sides: LENGTH, WIDTH,
 *        LENGTH, WIDTH, with a +90 deg (left) turn after each side.
 *   CW:  face along the left edge (upwards). Sides: WIDTH, LENGTH, WIDTH,
 *        LENGTH, with a -90 deg (right) turn after each side.
 *   Either way the robot should finish on the start corner facing the start
 *   direction. Odometry starts at x = 0, y = 0, theta = 0 (x = start heading,
 *   y = to the robot's left).
 *
 * Motion (encoder closed loop, the same controllers as Go_Straight / Rotate):
 *   - Geometry from the calibrations: r = 16.36 mm (Ex2), B = 83.48 mm (Ex3).
 *   - Straight side: both wheels to  mm / (2*pi*r) * 358.3  counts,
 *     left/right counts kept equal (K_SYNC), slow-down near the end.
 *   - 90 deg turn on the spot: each wheel to (pi/2)*(B/2) / (2*pi*r) * 358.3
 *     counts in opposite directions, left + right kept at 0 (K_SYNC_TURN).
 *   - After every side and turn: motors off, SETTLE_MS still, then go on.
 *
 * Odometry: Euler update every 10 ms with the same r and B (as
 * OdometryModel.cpp): x += ds*cos(theta), y += ds*sin(theta),
 * theta += (sR - sL)/B. The final pose is printed in the '# STOP' line.
 *
 * Wi-Fi: NOTHING is sent over Wi-Fi while the robot moves (a slow Wi-Fi
 * send can stall the control loop - seen once in Exercise 3). The run is
 * stored in RAM and sent to the laptop after the robot has stopped, and
 * again whenever a laptop connects (or wifi_logger.py asks with 'R').
 *
 * Usage (no button): place the robot, power on, join Wi-Fi CSY and run
 * System_error/wifi_logger.py within the 10 s countdown (or any time
 * after the run - the data waits in RAM until the robot is switched off).
 * CSV columns (one row every 20 ms):
 *   run,time_ms,segment,left_pwm,right_pwm,left_counts,right_counts,x_mm,y_mm,theta_rad
 *   segment: 0 = before start, 1..8 = side/turn 1..8, 9 = after the last turn.
 * Trial number: kept in flash, counted separately for CW and CCW.
 */

#include <stdarg.h>
#include <math.h>
#include <esp_random.h>
#include <Preferences.h>
#include "RobotWifiAP.h"
#include "Robot.h"
#include "TaskTimer.h"

// ------------------------------ Wi-Fi ------------------------------------
#define WIFI_SSID     "CSY"
#define WIFI_PASSWORD "csy12345"
RobotWifiAP_c server(WIFI_SSID, WIFI_PASSWORD, 80);

// ======================= EXPERIMENT SETTING ==============================
const int   LOOP_DIRECTION = -1;      // +1 = 逆时针 CCW,  -1 = 顺时针 CW
const float RECT_LENGTH_MM = 321.0f;  // long side of the rectangle (32.1 cm)
const float RECT_WIDTH_MM  = 217.0f;  // short side (21.7 cm)
// =========================================================================

// ---------------- calibrated geometry (Exercises 2 and 3) ----------------
const float WHEEL_RADIUS_MM     = 16.36f;
const float WHEEL_SEPARATION_MM = 83.48f;
const float ENCODER_CPR         = 358.3f;

// ------------------------- settings you can tune -------------------------
const float MIN_PWM          = 25.0f;   // lowest |PWM| that still moves the robot
const float MAX_PWM_STRAIGHT = 40.0f;
const float MAX_PWM_TURN     = 35.0f;
const float K_DIST           = 0.3f;    // PWM per count still to go (slow-down)
const float K_SYNC           = 0.5f;    // straight: PWM per count of (L - R)
const float K_SYNC_TURN      = 0.5f;    // turn: PWM per count of (L + R)
const float MAX_CORRECTION   = 10.0f;
const unsigned long RAMP_MS        = 200;    // soft start of every segment
const unsigned long SETTLE_MS      = 400;    // still time after every segment
const unsigned long SEGMENT_TIMEOUT_MS = 8000;
const unsigned long START_DELAY_MS = 10000;
const unsigned long PRE_ROLL_MS    = 500;
const unsigned long POST_ROLL_MS   = 1000;
const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long LOG_INTERVAL_MS     = 20;
const int MAX_STORED_ROWS = 2000;            // 2000 x 20 ms = 40 s of record
const int REPLAY_LINES_PER_LOOP = 20;
const bool RESET_TRIAL_COUNTERS = false;     // true once to restart at trial 1
// -------------------------------------------------------------------------

Robot_c robot;
TaskTimer_c control_timer(CONTROL_INTERVAL_MS);
TaskTimer_c log_timer(LOG_INTERVAL_MS);
Preferences prefs;

enum State { WAITING, PRE_ROLL, MOVING, SETTLING, POST_ROLL, STOPPED };
State state = WAITING;

enum SegType { SEG_STRAIGHT, SEG_TURN };
struct Segment { SegType type; float value; };   // mm or degrees
Segment segments[8];
const int N_SEGMENTS = 8;
int seg_index = 0;                 // 0..7 while moving
int seg_label = 0;                 // value written to the CSV (0..9)

unsigned long boot_ms = 0, start_ms = 0, seg_start_ms = 0, settle_start_ms = 0;
unsigned long motors_off_ms = 0;
int last_countdown_s = -1;
int run_num = 0;
int trial_num = 0;
int32_t start_left = 0, start_right = 0;       // counts at run start
int32_t seg_left0 = 0, seg_right0 = 0;         // counts (since run start) at segment start
int32_t prev_left = 0, prev_right = 0;         // for the odometry update
float seg_target_counts = 0.0f;                // > 0, per wheel
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
char start_line[260];
char motors_off_line[120];
char stop_line[220];
bool run_finished = false;
bool replay_active = false;
int replay_index = -2;
bool was_connected = false;

const char* CSV_HEADER =
  "run,time_ms,segment,left_pwm,right_pwm,left_counts,right_counts,x_mm,y_mm,theta_rad\n";

// Serial always; Wi-Fi only while the robot is not moving.
bool robotMoving() { return state == PRE_ROLL || state == MOVING || state == SETTLING || state == POST_ROLL; }

void logLine(const char* line) {
  if (!robotMoving()) server.send(line);
  Serial.print(line);
}

void logPrintf(const char* format, ...) {
  char buffer[260];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logLine(buffer);
}

void showStateLED() {
  if (state == WAITING)        robot.setLED(0, 0, 255, 40);    // blue
  else if (state == PRE_ROLL)  robot.setLED(255, 200, 0, 40);  // yellow
  else if (state == MOVING)    robot.setLED(0, 255, 0, 40);    // green
  else if (state == SETTLING)  robot.setLED(0, 120, 120, 40);  // cyan
  else                         robot.setLED(255, 0, 0, 40);    // red
}

void buildSegments() {
  float first  = (LOOP_DIRECTION > 0) ? RECT_LENGTH_MM : RECT_WIDTH_MM;
  float second = (LOOP_DIRECTION > 0) ? RECT_WIDTH_MM : RECT_LENGTH_MM;
  float turn = (LOOP_DIRECTION > 0) ? 90.0f : -90.0f;
  for (int i = 0; i < 4; i++) {
    segments[2 * i].type = SEG_STRAIGHT;
    segments[2 * i].value = (i % 2 == 0) ? first : second;
    segments[2 * i + 1].type = SEG_TURN;
    segments[2 * i + 1].value = turn;
  }
}

void nextTrialNumber() {
  const char* key = (LOOP_DIRECTION > 0) ? "trial_ccw" : "trial_cw";
  prefs.begin("square", false);
  if (RESET_TRIAL_COUNTERS) prefs.clear();
  trial_num = prefs.getInt(key, 0) + 1;
  prefs.putInt(key, trial_num);
  prefs.end();
}

float countsPerMm() { return ENCODER_CPR / (2.0f * PI * WHEEL_RADIUS_MM); }

void startSegment(unsigned long now, int32_t left_counts, int32_t right_counts) {
  const Segment &s = segments[seg_index];
  seg_left0 = left_counts;
  seg_right0 = right_counts;
  seg_start_ms = now;
  seg_label = seg_index + 1;
  if (s.type == SEG_STRAIGHT) {
    seg_target_counts = s.value * countsPerMm();
  } else {
    float arc_mm = fabsf(s.value) * PI / 180.0f * (WHEEL_SEPARATION_MM / 2.0f);
    seg_target_counts = arc_mm * countsPerMm();
  }
  state = MOVING;
  showStateLED();
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
           "# START boot=%08lx run=%d test=square dir=%s trial=%d length_mm=%.1f width_mm=%.1f "
           "r_mm=%.2f B_mm=%.2f max_pwm=%.0f/%.0f k_sync=%.2f/%.2f settle_ms=%lu",
           (unsigned long)boot_id, run_num, LOOP_DIRECTION > 0 ? "ccw" : "cw", trial_num,
           RECT_LENGTH_MM, RECT_WIDTH_MM, WHEEL_RADIUS_MM, WHEEL_SEPARATION_MM,
           MAX_PWM_STRAIGHT, MAX_PWM_TURN, K_SYNC, K_SYNC_TURN, SETTLE_MS);
  Serial.printf("%s\n%s", start_line, CSV_HEADER);   // Wi-Fi gets it after the run
}

void stopMotors() {
  robot.setMotorPWM(0, 0);
  left_pwm = right_pwm = 0.0f;
}

void endMotion(unsigned long now, const char* reason) {
  stopMotors();
  end_reason = reason;
  motors_off_ms = now;
  seg_label = 9;
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

// One control step of the current segment.
void driveSegment(unsigned long now, int32_t left_counts, int32_t right_counts) {
  const Segment &s = segments[seg_index];
  int32_t dl = left_counts - seg_left0;
  int32_t dr = right_counts - seg_right0;
  float sign = (s.value >= 0.0f) ? 1.0f : -1.0f;   // straight: always +; turn: +CCW / -CW
  float progress, max_pwm, correction;

  if (s.type == SEG_STRAIGHT) {
    progress = 0.5f * (float)(dl + dr);
    max_pwm = MAX_PWM_STRAIGHT;
    correction = K_SYNC * (float)(dl - dr);          // left ahead -> slow left
  } else {
    progress = 0.5f * sign * (float)(dr - dl);
    max_pwm = MAX_PWM_TURN;
    correction = K_SYNC_TURN * (float)(dl + dr);     // centre drifting -> shift both
  }
  float remaining = seg_target_counts - progress;

  if (remaining <= 0.0f) {                           // segment done
    stopMotors();
    state = SETTLING;
    settle_start_ms = now;
    showStateLED();
    robot.playTone(660, 40);
    return;
  }
  if (now - seg_start_ms >= SEGMENT_TIMEOUT_MS) {
    endMotion(now, "timeout");
    return;
  }

  float magnitude = K_DIST * remaining;
  if (magnitude > max_pwm) magnitude = max_pwm;
  if (magnitude < MIN_PWM) magnitude = MIN_PWM;
  unsigned long t = now - seg_start_ms;
  if (t < RAMP_MS) magnitude = MIN_PWM + (magnitude - MIN_PWM) * (float)t / (float)RAMP_MS;
  if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
  if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;

  if (s.type == SEG_STRAIGHT) {
    left_pwm = magnitude - correction;
    right_pwm = magnitude + correction;
  } else {
    left_pwm = -sign * magnitude - correction;
    right_pwm = sign * magnitude - correction;
  }
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
    if (!server.send(line)) { replay_active = false; return; }   // laptop gone
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
  buildSegments();
  nextTrialNumber();
  start_line[0] = motors_off_line[0] = stop_line[0] = '\0';
  Serial.println("System_error ready - the rectangle starts automatically");
  boot_ms = millis();
}

void loop() {
  unsigned long now = millis();

  if (!robotMoving()) {               // Wi-Fi work only while standing still
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
        logPrintf("# starting in %d s (%s rectangle %.0f x %.0f mm, trial %d)\n", remaining_s,
                  LOOP_DIRECTION > 0 ? "CCW" : "CW", RECT_LENGTH_MM, RECT_WIDTH_MM, trial_num);
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
  } else if (state == SETTLING && now - settle_start_ms >= SETTLE_MS) {
    seg_index++;
    if (seg_index >= N_SEGMENTS) {
      endMotion(now, "done");
    } else {
      startSegment(now, left_counts, right_counts);
    }
  }

  bool finished = (state == POST_ROLL && now - motors_off_ms >= POST_ROLL_MS);
  if (log_timer.isReady(now) || finished) {
    log_timer.resetTimer(now);
    storeAndPrintRow(elapsed, left_counts, right_counts);
  }

  if (finished) {
    // Heading change wrapped to (-180, 180]: 0 means "facing the start direction".
    float wrapped = fmodf(odom_theta * 180.0f / PI, 360.0f);
    if (wrapped > 180.0f) wrapped -= 360.0f;
    if (wrapped <= -180.0f) wrapped += 360.0f;
    snprintf(stop_line, sizeof(stop_line),
             "# STOP boot=%08lx run=%d time_ms=%lu dir=%s trial=%d reason=%s "
             "odom_x_mm=%.2f odom_y_mm=%.2f odom_theta_deg=%.2f odom_dtheta_wrapped_deg=%.2f",
             (unsigned long)boot_id, run_num, elapsed, LOOP_DIRECTION > 0 ? "ccw" : "cw",
             trial_num, end_reason, odom_x, odom_y, odom_theta * 180.0f / PI, wrapped);
    Serial.printf("%s\n", stop_line);
    state = STOPPED;
    run_finished = true;
    showStateLED();
    robot.playTone(990, 80);
    if (server.clientConnected()) startReplay();   // send the whole run now
  }
}
