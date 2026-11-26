/*
 * Go_Straight - drive the Pololu 3Pi+ a set distance in a straight line.
 * ------------------------------------------------------------------
 * EXPERIMENT SETTING: change TARGET_DISTANCE_MM below, upload, test.
 *   +10   = forward 1 cm        +100 = forward 10 cm    +200 = forward 20 cm
 *   -100  = reverse 10 cm       -200 = reverse 20 cm
 *
 * How the distance is reached (closed loop on the wheel encoders):
 *   - The target is converted to encoder counts with the NOMINAL wheel
 *     radius (16.0 mm, the same r_old the Colab notebook uses):
 *         target_counts = target_mm / (2*pi*16.0) * 358.3
 *     So "10 mm" means "the counts a 16.0 mm wheel needs for 10 mm". If the
 *     real wheel is bigger or smaller, the robot travels a bit more or less -
 *     that difference is exactly what Exercise 2 measures with the ruler.
 *   - Speed: base PWM = K_DIST * remaining counts, limited to
 *     [MIN_PWM, MAX_PWM], so the robot slows down near the target and does
 *     not overshoot much. It stops as soon as the average of the two wheels
 *     reaches the target (it never drives backwards to correct).
 *   - Straightness: both wheels get the same base PWM, nudged in opposite
 *     directions so that the left and right counts stay equal:
 *         diff       = left_counts - right_counts
 *         correction = K_SYNC * diff  (limited to +/- MAX_CORRECTION)
 *         left_pwm   = base - correction,  right_pwm = base + correction
 *     (This works for reverse too: base is then negative.)
 *
 * Usage (no button needed):
 *   1. Put batteries in, place the robot on the start mark, press On/Off.
 *   2. The robot waits START_DELAY_MS (10 s) so you can join its Wi-Fi and
 *      run wifi_logger.py. LED blue = waiting; yellow + beep = last 3 s.
 *   3. Logging: 0.5 s standing still (LED yellow), drive to the target
 *      (LED green), motors off (LED red, beep), 1 s more logging so the
 *      coasting is recorded. One run per power-up.
 *   4. CSV over Wi-Fi AND USB Serial (115200):
 *      run,time_ms,left_pwm,right_pwm,left_counts,right_counts,diff
 *      '#' lines are status messages (START, MOTORS_OFF, STOP).
 *   If the robot cannot reach the target within MOTION_TIMEOUT_MS (e.g. the
 *   PWM is too low to overcome friction), it stops and reports reason=timeout.
 *
 * Safety net - data is never lost if the laptop joins late:
 *   Every CSV row of the run is also kept in RAM. After the run, the whole
 *   record is sent again ("replayed") whenever a laptop connects, when the
 *   laptop joined part-way through the run, or when wifi_logger.py asks for
 *   it (it sends 'R' on connect). Each power-up has a random boot id, so the
 *   logger can tell runs apart and does not save a run twice.
 */

#include <stdarg.h>
#include <esp_random.h>
#include "RobotWifiAP.h"
#include "Robot.h"
#include "TaskTimer.h"

// ------------------------------ Wi-Fi ------------------------------------
// Change the name/password if you like. Never reuse a real account password.
#define WIFI_SSID     "CSY"
#define WIFI_PASSWORD "csy12345"   // at least 8 characters
RobotWifiAP_c server(WIFI_SSID, WIFI_PASSWORD, 80);   // TCP port 80

// ======================= EXPERIMENT SETTING ==============================
const float TARGET_DISTANCE_MM = -180.0f;   // +10 = forward 1 cm (see table above)
// =========================================================================

// ------------------------- settings you can tune -------------------------
const float NOMINAL_WHEEL_RADIUS_MM = 16.0f;  // r_old used to convert mm -> counts
const float ENCODER_CPR       = 358.3f;  // counts per wheel revolution (fixed)
const float MIN_PWM           = 25.0f;   // lowest |PWM| that still moves the robot
const float MAX_PWM           = 40.0f;   // cruising |PWM| for longer distances
const float K_DIST            = 0.3f;    // PWM per count still to go (slow-down)
const bool  USE_ENCODER_SYNC  = true;    // false = equal PWM only (open loop)
const float K_SYNC            = 0.5f;    // PWM per encoder count of difference
const float MAX_CORRECTION    = 10.0f;   // limit on the steering correction
const unsigned long RAMP_MS   = 200;     // soft start from MIN_PWM, less slip
const unsigned long START_DELAY_MS = 10000;   // wait after power-up
const unsigned long MOTION_TIMEOUT_MS = 8000; // give up if target not reached
const unsigned long PRE_ROLL_MS  = 500;  // stationary logging before driving
const unsigned long POST_ROLL_MS = 1000; // logging after motors off (coasting)
const unsigned long CONTROL_INTERVAL_MS = 10;  // control loop period
const unsigned long LOG_INTERVAL_MS     = 10;  // CSV row period (Wi-Fi + Serial)
const int MAX_STORED_ROWS = 1500;              // RAM copy for replay (16 bytes/row)
const int REPLAY_LINES_PER_LOOP = 20;          // send the replay in small chunks
// -------------------------------------------------------------------------

Robot_c robot;
TaskTimer_c control_timer(CONTROL_INTERVAL_MS);
TaskTimer_c log_timer(LOG_INTERVAL_MS);

enum State { WAITING, PRE_ROLL, DRIVING, POST_ROLL, STOPPED };
State state = WAITING;

unsigned long boot_ms = 0;         // millis() at the end of setup()
unsigned long start_ms = 0;        // logging start (time_ms = 0)
unsigned long drive_start_ms = 0;  // motors on
unsigned long motors_off_ms = 0;   // motors off
int last_countdown_s = -1;         // last countdown second announced
int run_num = 0;                   // +1 every time a run starts
int32_t start_left = 0;            // encoder counts when the run started
int32_t start_right = 0;
float target_counts = 0.0f;        // signed target in encoder counts
float left_pwm = 0.0f;
float right_pwm = 0.0f;

// ---- RAM copy of the run, for replaying to a laptop that joined late ----
struct StoredRow {
  uint32_t time_ms;
  int16_t left_pwm_x10;    // PWM * 10 (one decimal place, as in the CSV)
  int16_t right_pwm_x10;
  int32_t left_counts;
  int32_t right_counts;
};
StoredRow stored_rows[MAX_STORED_ROWS];
int stored_count = 0;
bool stored_overflow = false;
uint32_t boot_id = 0;                 // random per power-up
char start_line[200];                 // "# START ..." line of this run
char motors_off_line[96];             // "# MOTORS_OFF ..." line
char stop_line[64];                   // "# STOP ..." line
bool replay_active = false;
int replay_index = -2;                // -2 = START line, -1 = header, 0.. = rows
bool was_connected = false;
unsigned long client_since_ms = 0;    // when the current laptop connection began

// Send one line (must end in '\n') to the Wi-Fi client and to USB Serial.
// server.send() simply returns false when no laptop is connected.
void logLine(const char* line) {
  server.send(line);
  Serial.print(line);
}

void logPrintf(const char* format, ...) {
  char buffer[192];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  logLine(buffer);
}

void showStateLED() {
  if (state == WAITING)       robot.setLED(0, 0, 255, 40);   // blue
  else if (state == PRE_ROLL) robot.setLED(255, 200, 0, 40); // yellow
  else if (state == DRIVING)  robot.setLED(0, 255, 0, 40);   // green
  else                        robot.setLED(255, 0, 0, 40);   // red
}

void startRun(unsigned long now) {
  // Remember where the encoders are now: the Pololu does not reset its
  // counts when the StampC3 restarts, so we always work with differences.
  robot.getEncoders();
  start_left = robot.getLeftEncoderCount();
  start_right = robot.getRightEncoderCount();
  start_ms = now;
  run_num = run_num + 1;
  target_counts = TARGET_DISTANCE_MM / (2.0f * PI * NOMINAL_WHEEL_RADIUS_MM) * ENCODER_CPR;
  state = PRE_ROLL;
  showStateLED();
  stored_count = 0;
  stored_overflow = false;
  snprintf(start_line, sizeof(start_line),
           "# START boot=%08lx run=%d target_mm=%.1f target_counts=%.1f nominal_r_mm=%.2f "
           "min_pwm=%.0f max_pwm=%.0f k_sync=%.2f sync=%d",
           (unsigned long)boot_id, run_num, TARGET_DISTANCE_MM, target_counts,
           NOMINAL_WHEEL_RADIUS_MM, MIN_PWM, MAX_PWM, K_SYNC, USE_ENCODER_SYNC ? 1 : 0);
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

// Format one stored row exactly like the live CSV row.
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

// Send a few replay lines per loop() so the robot stays responsive.
// Only the Wi-Fi client gets the replay (USB Serial already had the data).
void serviceReplay() {
  if (!replay_active) return;
  if (!server.clientConnected()) {      // laptop left: start over next time
    replay_active = false;
    return;
  }
  char line[220];
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

// Decide when to replay: a laptop connects after the run, a laptop that
// joined part-way through the run is still there at the end, or the
// logger asks with 'R'.
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
  start_line[0] = motors_off_line[0] = stop_line[0] = '\0';
  Serial.println("Go_Straight ready - driving starts automatically");
  boot_ms = millis();
}

void loop() {
  unsigned long now = millis();

  // Accept / keep the laptop's Wi-Fi connection (non-blocking), and
  // replay the finished run to a laptop that joined late.
  server.update();
  checkReplayTriggers(now);
  serviceReplay();

  // No button: count down after power-up, then start automatically.
  if (state == WAITING) {
    unsigned long waited = now - boot_ms;
    if (waited >= START_DELAY_MS) {
      startRun(now);
    } else {
      int remaining_s = (int)((START_DELAY_MS - waited + 999) / 1000);
      if (remaining_s != last_countdown_s) {
        last_countdown_s = remaining_s;
        logPrintf("# starting in %d s (target %.1f mm)\n", remaining_s, TARGET_DISTANCE_MM);
        if (remaining_s <= 3) {
          robot.setLED(255, 200, 0, 40);   // yellow: about to move
          robot.playTone(440, 40);
        }
      }
    }
    return;
  }

  if (state == STOPPED) return;   // finished: stay still until power-off

  // Run the controller every CONTROL_INTERVAL_MS (non-blocking).
  if (!control_timer.isReady(now)) return;
  control_timer.resetTimer(now);

  unsigned long elapsed = now - start_ms;   // time since logging started

  // Counts travelled by each wheel since the start of this run.
  // If an I2C read fails, the last good snapshot is kept.
  robot.getEncoders();
  int32_t left_counts = robot.getLeftEncoderCount() - start_left;
  int32_t right_counts = robot.getRightEncoderCount() - start_right;
  int32_t diff = left_counts - right_counts;
  float travelled = 0.5f * (float)(left_counts + right_counts);   // average, counts

  // Phase changes.
  if (state == PRE_ROLL && elapsed >= PRE_ROLL_MS) {
    state = DRIVING;
    drive_start_ms = now;
    showStateLED();
    robot.playTone(660, 50);
  }

  if (state == DRIVING) {
    float direction = (target_counts >= 0.0f) ? 1.0f : -1.0f;
    float remaining = (target_counts - travelled) * direction;   // > 0 = not there yet

    if (remaining <= 0.0f) {
      motorsOff(now, "target");
    } else if (now - drive_start_ms >= MOTION_TIMEOUT_MS) {
      motorsOff(now, "timeout");
    } else {
      // Slow down near the target: |base| = K_DIST * remaining, limited.
      float magnitude = K_DIST * remaining;
      if (magnitude > MAX_PWM) magnitude = MAX_PWM;
      if (magnitude < MIN_PWM) magnitude = MIN_PWM;

      // Soft start: rise from MIN_PWM to the computed value over RAMP_MS.
      unsigned long driving = now - drive_start_ms;
      if (driving < RAMP_MS) {
        magnitude = MIN_PWM + (magnitude - MIN_PWM) * (float)driving / (float)RAMP_MS;
      }
      float base = direction * magnitude;

      float correction = 0.0f;
      if (USE_ENCODER_SYNC) {
        correction = K_SYNC * (float)diff;
        if (correction > MAX_CORRECTION) correction = MAX_CORRECTION;
        if (correction < -MAX_CORRECTION) correction = -MAX_CORRECTION;
      }

      left_pwm = base - correction;
      right_pwm = base + correction;
      robot.setMotorPWM(left_pwm, right_pwm);
    }
  }

  bool finished = (state == POST_ROLL && now - motors_off_ms >= POST_ROLL_MS);

  // One CSV row (pre-roll, driving and post-roll alike); always log the
  // final row so the record ends with the robot at rest.
  if (log_timer.isReady(now) || finished) {
    log_timer.resetTimer(now);
    logPrintf("%d,%lu,%.1f,%.1f,%ld,%ld,%ld\n",
              run_num, elapsed, left_pwm, right_pwm,
              (long)left_counts, (long)right_counts, (long)diff);

    // Keep a RAM copy for a late laptop.
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
    snprintf(stop_line, sizeof(stop_line), "# STOP boot=%08lx run=%d time_ms=%lu",
             (unsigned long)boot_id, run_num, elapsed);
    logPrintf("%s\n", stop_line);
    state = STOPPED;
    // A laptop that joined after the run started missed the beginning:
    // send it the complete record now.
    if (server.clientConnected() && client_since_ms > start_ms) startReplay();
  }
}
