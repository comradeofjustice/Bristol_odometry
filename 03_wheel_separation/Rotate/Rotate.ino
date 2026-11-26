/*
 * Rotate - turn the Pololu 3Pi+ on the spot by a set angle (Exercise 3,
 * effective wheel separation B).
 * ------------------------------------------------------------------
 * EXPERIMENT SETTING: change TARGET_ANGLE_DEG below, upload, test.
 *   +90  = anticlockwise 90 deg (seen from above)   -90  = clockwise 90 deg
 *   +180 / -180     +270 / -270     +360 / -360  (one full turn, about 8 s)
 *   Sign convention = odometry: theta grows anticlockwise, because
 *   d_theta = (s_right - s_left) / B. So + means right wheel forward,
 *   left wheel backward.
 *
 * How the turn is made (closed loop on the wheel encoders):
 *   - Each wheel must roll the arc  angle * B_OLD/2  in opposite directions.
 *     Converted to counts with the radius calibrated in Exercise 2:
 *         wheel_counts = |angle| * (B_OLD/2) / (2*pi*r) * 358.3
 *     with r = 16.36 mm and B_OLD = 90 mm (the separation the odometry model
 *     uses during this test). The commanded angle is only the INPUT: the
 *     angle you measure on the printed sheet is the reference.
 *   - Speed: |PWM| = K_TURN * counts still to go, limited to
 *     [MIN_PWM, MAX_PWM], with a soft start. Motors off as soon as the
 *     average of the two wheels' turning counts reaches wheel_counts.
 *   - Staying on the spot: for a perfect in-place turn left_counts +
 *     right_counts = 0. If the sum drifts, both PWMs are shifted the same
 *     way (correction = K_SYNC * sum) so the centre point does not wander.
 *
 * Logger: Rotate/wifi_logger.py saves Rotate/logs/rot_<angle>deg_t<trial>_*.csv
 *
 * Usage (no button needed), exactly as Go_Straight:
 *   1. Place the robot on the centre of the radial marker, press On/Off.
 *   2. 10 s countdown (LED blue, yellow + beep for the last 3 s) - join the
 *      CSY Wi-Fi and run wifi_logger.py.
 *   3. 0.5 s still (yellow), turn (green), motors off (red, beep), 1 s more.
 *   4. CSV over Wi-Fi and USB Serial (same columns as Go_Straight):
 *      run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff
 *      The '# START' line also carries trial=, angle_deg=, angle_rad=.
 *      The '# STOP' line carries the odometry estimate theta_est_deg.
 *
 * Trial number: kept in the ESP32's flash (survives power-off). It counts
 * 1, 2, 3, ... for the current TARGET_ANGLE_DEG and restarts at 1 when you
 * upload a different angle. Write it in the sheet next to the CSV name.
 *
 * Safety net: identical to Go_Straight - the whole run is kept in RAM and
 * replayed when a laptop joins late (or wifi_logger.py sends 'R').
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
#define WIFI_PASSWORD "csy12345"   // at least 8 characters
RobotWifiAP_c server(WIFI_SSID, WIFI_PASSWORD, 80);   // TCP port 80

// ======================= EXPERIMENT SETTING ==============================
const float TARGET_ANGLE_DEG = 90.0f;   // + anticlockwise, - clockwise
// =========================================================================

// ------------------------- settings you can tune -------------------------
const float WHEEL_RADIUS_MM   = 16.36f;  // calibrated in Exercise 2
const float B_OLD_MM          = 90.0f;   // separation assumed during this test
const float ENCODER_CPR       = 358.3f;  // counts per wheel revolution (fixed)
const float MIN_PWM           = 25.0f;   // lowest |PWM| that still turns the robot
const float MAX_PWM           = 35.0f;   // cruising |PWM| (slower = less slip)
const float K_TURN            = 0.3f;    // PWM per count still to go (slow-down)
const bool  USE_CENTRE_SYNC   = true;    // keep left_counts + right_counts = 0
const float K_SYNC            = 0.5f;    // PWM per count of (left + right)
const float MAX_CORRECTION    = 8.0f;    // limit on that correction
const unsigned long RAMP_MS   = 200;     // soft start from MIN_PWM
const unsigned long START_DELAY_MS = 10000;   // wait after power-up
const unsigned long MOTION_TIMEOUT_MS = 15000; // give up if target not reached
const unsigned long PRE_ROLL_MS  = 500;  // stationary logging before turning
const unsigned long POST_ROLL_MS = 1000; // logging after motors off (coasting)
const unsigned long CONTROL_INTERVAL_MS = 10;
const unsigned long LOG_INTERVAL_MS     = 10;
const int MAX_STORED_ROWS = 1500;
const int REPLAY_LINES_PER_LOOP = 20;
// -------------------------------------------------------------------------

Robot_c robot;
TaskTimer_c control_timer(CONTROL_INTERVAL_MS);
TaskTimer_c log_timer(LOG_INTERVAL_MS);
Preferences prefs;

enum State { WAITING, PRE_ROLL, TURNING, POST_ROLL, STOPPED };
State state = WAITING;

unsigned long boot_ms = 0;
unsigned long start_ms = 0;
unsigned long turn_start_ms = 0;
unsigned long motors_off_ms = 0;
int last_countdown_s = -1;
int run_num = 0;
int trial_num = 0;                 // persistent, per commanded angle
int32_t start_left = 0;
int32_t start_right = 0;
float direction = 1.0f;            // +1 anticlockwise, -1 clockwise
float wheel_target_counts = 0.0f;  // counts each wheel should turn (> 0)
float left_pwm = 0.0f;
float right_pwm = 0.0f;
int32_t last_left = 0, last_right = 0;   // latest counts (for the STOP line)

struct StoredRow {
  uint32_t time_ms;
  int16_t left_pwm_x10;
  int16_t right_pwm_x10;
  int32_t left_counts;
  int32_t right_counts;
};
StoredRow stored_rows[MAX_STORED_ROWS];
int stored_count = 0;
bool stored_overflow = false;
uint32_t boot_id = 0;
char start_line[220];
char motors_off_line[96];
char stop_line[128];
bool replay_active = false;
int replay_index = -2;
bool was_connected = false;
unsigned long client_since_ms = 0;

void logLine(const char* line) {
  server.send(line);
  Serial.print(line);
}

void logPrintf(const char* format, ...) {
  char buffer[240];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logLine(buffer);
}

void showStateLED() {
  if (state == WAITING)       robot.setLED(0, 0, 255, 40);   // blue
  else if (state == PRE_ROLL) robot.setLED(255, 200, 0, 40); // yellow
  else if (state == TURNING)  robot.setLED(0, 255, 0, 40);   // green
  else                        robot.setLED(255, 0, 0, 40);   // red
}

// Trial counter in flash: +1 per power-up, restarts at 1 for a new angle.
void nextTrialNumber() {
  int32_t angle_x10 = (int32_t)lroundf(TARGET_ANGLE_DEG * 10.0f);
  prefs.begin("rotate", false);
  int32_t saved_angle = prefs.getInt("angle_x10", 0x7fffffff);
  int32_t saved_trial = prefs.getInt("trial", 0);
  if (saved_angle != angle_x10) saved_trial = 0;
  trial_num = saved_trial + 1;
  prefs.putInt("angle_x10", angle_x10);
  prefs.putInt("trial", trial_num);
  prefs.end();
}

// Odometry heading change from the counts, with B_OLD (same as the notebook).
float headingEstimateRad(int32_t left_counts, int32_t right_counts) {
  float mm_per_count = 2.0f * PI * WHEEL_RADIUS_MM / ENCODER_CPR;
  return (right_counts - left_counts) * mm_per_count / B_OLD_MM;
}

void startRun(unsigned long now) {
  robot.getEncoders();
  start_left = robot.getLeftEncoderCount();
  start_right = robot.getRightEncoderCount();
  start_ms = now;
  run_num = run_num + 1;
  float angle_rad = TARGET_ANGLE_DEG * PI / 180.0f;
  direction = (angle_rad >= 0.0f) ? 1.0f : -1.0f;
  wheel_target_counts = fabsf(angle_rad) * (B_OLD_MM / 2.0f)
                        / (2.0f * PI * WHEEL_RADIUS_MM) * ENCODER_CPR;
  state = PRE_ROLL;
  showStateLED();
  stored_count = 0;
  stored_overflow = false;
  snprintf(start_line, sizeof(start_line),
           "# START boot=%08lx run=%d test=rotation trial=%d angle_deg=%.1f angle_rad=%.4f "
           "wheel_counts=%.1f r_mm=%.2f B_old_mm=%.1f min_pwm=%.0f max_pwm=%.0f k_sync=%.2f",
           (unsigned long)boot_id, run_num, trial_num, TARGET_ANGLE_DEG, angle_rad,
           wheel_target_counts, WHEEL_RADIUS_MM, B_OLD_MM, MIN_PWM, MAX_PWM,
           USE_CENTRE_SYNC ? K_SYNC : 0.0f);
  logPrintf("%s\n", start_line);
  logLine("run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff\n");
}

void motorsOff(unsigned long now, const char* reason) {
  robot.setMotorPWM(0, 0);
  left_pwm = 0;
  right_pwm = 0;
  motors_off_ms = now;
  state = POST_ROLL;
  showStateLED();
  robot.playTone(880, 100);
  snprintf(motors_off_line, sizeof(motors_off_line), "# MOTORS_OFF boot=%08lx run=%d time_ms=%lu reason=%s",
           (unsigned long)boot_id, run_num, now - start_ms, reason);
  logPrintf("%s\n", motors_off_line);
}

void formatRow(const StoredRow &r, char* out, size_t len) {
  snprintf(out, len, "%d,%lu,%.1f,%.1f,%ld,%ld,%ld\n",
           run_num, (unsigned long)r.time_ms,
           r.left_pwm_x10 / 10.0f, r.right_pwm_x10 / 10.0f,
           (long)r.left_counts, (long)r.right_counts,
           (long)(r.left_counts - r.right_counts));
}

void startReplay() {
  if (state != STOPPED || replay_active) return;
  replay_active = true;
  replay_index = -2;
}

void serviceReplay() {
  if (!replay_active) return;
  if (!server.clientConnected()) {
    replay_active = false;
    return;
  }
  char line[260];
  for (int n = 0; n < REPLAY_LINES_PER_LOOP; n++) {
    if (replay_index == -2) {
      snprintf(line, sizeof(line), "%s replay=1%s\n", start_line,
               stored_overflow ? " buffer_full=1" : "");
    } else if (replay_index == -1) {
      snprintf(line, sizeof(line), "run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff\n");
    } else if (replay_index < stored_count) {
      formatRow(stored_rows[replay_index], line, sizeof(line));
    } else {
      snprintf(line, sizeof(line), "%s replay=1\n%s replay=1\n", motors_off_line, stop_line);
      server.send(line);
      replay_active = false;
      return;
    }
    server.send(line);
    replay_index++;
  }
}

void checkReplayTriggers(unsigned long now) {
  bool connected = server.clientConnected();
  if (connected && !was_connected) {
    client_since_ms = now;
    if (state == STOPPED) startReplay();
  }
  was_connected = connected;

  while (server.available() > 0) {
    int c = server.read();
    if (c == 'R' && state == STOPPED) startReplay();
  }
}

void setup() {
  Serial.begin(115200);
  if (!server.begin()) {
    Serial.println("Warning: Wi-Fi access point failed to start");
  }
  robot.initialise();
  showStateLED();
  boot_id = esp_random();
  nextTrialNumber();
  start_line[0] = motors_off_line[0] = stop_line[0] = '\0';
  Serial.println("Rotate ready - turning starts automatically");
  boot_ms = millis();
}

void loop() {
  unsigned long now = millis();

  server.update();
  checkReplayTriggers(now);
  serviceReplay();

  if (state == WAITING) {
    unsigned long waited = now - boot_ms;
    if (waited >= START_DELAY_MS) {
      startRun(now);
    } else {
      int remaining_s = (int)((START_DELAY_MS - waited + 999) / 1000);
      if (remaining_s != last_countdown_s) {
        last_countdown_s = remaining_s;
        logPrintf("# starting in %d s (trial %d, angle %.1f deg)\n",
                  remaining_s, trial_num, TARGET_ANGLE_DEG);
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
  int32_t diff = left_counts - right_counts;
  last_left = left_counts;
  last_right = right_counts;

  if (state == PRE_ROLL && elapsed >= PRE_ROLL_MS) {
    state = TURNING;
    turn_start_ms = now;
    showStateLED();
    robot.playTone(660, 50);
  }

  if (state == TURNING) {
    // Average turning progress of the two wheels, in counts (> 0 = right way).
    float progress = 0.5f * direction * (float)(right_counts - left_counts);
    float remaining = wheel_target_counts - progress;

    if (remaining <= 0.0f) {
      motorsOff(now, "target");
    } else if (now - turn_start_ms >= MOTION_TIMEOUT_MS) {
      motorsOff(now, "timeout");
    } else {
      float magnitude = K_TURN * remaining;
      if (magnitude > MAX_PWM) magnitude = MAX_PWM;
      if (magnitude < MIN_PWM) magnitude = MIN_PWM;
      unsigned long turning = now - turn_start_ms;
      if (turning < RAMP_MS) {
        magnitude = MIN_PWM + (magnitude - MIN_PWM) * (float)turning / (float)RAMP_MS;
      }

      // Keep the centre still: left + right should stay 0.
      float correction = 0.0f;
      if (USE_CENTRE_SYNC) {
        correction = K_SYNC * (float)(left_counts + right_counts);
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
      }

      left_pwm  = -direction * magnitude - correction;
      right_pwm =  direction * magnitude - correction;
      robot.setMotorPWM(left_pwm, right_pwm);
    }
  }

  bool finished = (state == POST_ROLL && now - motors_off_ms >= POST_ROLL_MS);

  if (log_timer.isReady(now) || finished) {
    log_timer.resetTimer(now);
    logPrintf("%d,%lu,%.1f,%.1f,%ld,%ld,%ld\n",
              run_num, elapsed, left_pwm, right_pwm,
              (long)left_counts, (long)right_counts, (long)diff);

    if (stored_count < MAX_STORED_ROWS) {
      StoredRow &r = stored_rows[stored_count++];
      r.time_ms = elapsed;
      r.left_pwm_x10 = (int16_t)lroundf(left_pwm * 10.0f);
      r.right_pwm_x10 = (int16_t)lroundf(right_pwm * 10.0f);
      r.left_counts = left_counts;
      r.right_counts = right_counts;
    } else {
      stored_overflow = true;
    }
  }

  if (finished) {
    float theta_est_deg = headingEstimateRad(last_left, last_right) * 180.0f / PI;
    snprintf(stop_line, sizeof(stop_line),
             "# STOP boot=%08lx run=%d time_ms=%lu trial=%d theta_est_deg=%.2f",
             (unsigned long)boot_id, run_num, elapsed, trial_num, theta_est_deg);
    logPrintf("%s\n", stop_line);
    state = STOPPED;
    if (server.clientConnected() && client_since_ms > start_ms) startReplay();
  }
}
