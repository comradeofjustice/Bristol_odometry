/**
 * @brief Path-reproduction controller: the same behaviour as the StampC3
 * firmware Path_Reproduction.ino, written against the simulated robot.
 * @details Every CONTROL_INTERVAL_MS it reads integer encoder counts, updates
 * the robot's own odometry estimate (r = 16.36 mm, B = 83.48 mm) and runs one
 * step of the segment controller. Every LOG_INTERVAL_MS it stores a row with
 * the same columns as the robot's CSV, plus the twin's true pose.
 * Note: the supplied Controller.pde referred to a type RobotSimulation_c that
 * does not exist in the sketch; this version uses Robot_c.
 */
class Controller_c {
  final int WAITING = 0, PRE_ROLL = 1, MOVING = 2, SETTLING = 3, POST_ROLL = 4, STOPPED = 5;
  int signal = 0, mode = WAITING;
  TaskTimer_c update_timer = new TaskTimer_c(CONTROL_INTERVAL_MS);

  int segIndex = 0, segLabel = 0;
  long startMs = 0, segStartMs = 0, settleStartMs = 0, motorsOffMs = 0, lastLogMs = -1000;
  int segLeft0 = 0, segRight0 = 0, prevLeft = 0, prevRight = 0, startLeft = 0, startRight = 0;
  float targetLeft = 0, targetRight = 0, leftPwm = 0, rightPwm = 0;
  float odomX = 0, odomY = 0, odomTheta = 0;
  ArrayList<String> rows = new ArrayList<String>();
  ArrayList<float[]> truePath = new ArrayList<float[]>();

  void reset() {
    signal = 0;
    mode = WAITING;
    rows.clear();
    truePath.clear();
  }
  int getSignal() { return signal; }

  /** @brief A non-zero value starts the path from the current pose. */
  void setSignal(int requested) {
    if (requested == 0) { reset(); return; }
    if (signal == 0) { signal = 1; mode = PRE_ROLL; }
  }

  boolean finished() { return mode == STOPPED; }

  float countsPerMm() { return CTRL_CPR / (TWO_PI * CTRL_WHEEL_RADIUS_MM); }

  void update(Robot_c robot) {
    long now = robot.getMillis();
    if (!update_timer.isReady(now)) return;
    update_timer.resetTimer(now);
    if (signal == 0 || mode == STOPPED) return;

    robot.getEncoders();
    if (mode == PRE_ROLL && rows.size() == 0) {         // start of the run
      startMs = now;
      startLeft = robot.getLeftEncoderCount();
      startRight = robot.getRightEncoderCount();
      prevLeft = prevRight = 0;
      odomX = odomY = odomTheta = 0;
      segIndex = 0;
      segLabel = 0;
      lastLogMs = -1000;
    }
    long elapsed = now - startMs;
    int leftCounts = robot.getLeftEncoderCount() - startLeft;
    int rightCounts = robot.getRightEncoderCount() - startRight;
    updateOdometry(leftCounts, rightCounts);

    if (mode == PRE_ROLL && elapsed >= PRE_ROLL_MS) {
      startSegment(now, leftCounts, rightCounts);
    } else if (mode == MOVING) {
      driveSegment(robot, now, leftCounts, rightCounts);
    } else if (mode == SETTLING && now - settleStartMs >= SETTLE_MS + CHECKPOINT_PAUSE_MS) {
      segIndex++;
      if (segIndex >= PATH.length) endMotion(robot, now);
      else startSegment(now, leftCounts, rightCounts);
    }

    boolean done = (mode == POST_ROLL && now - motorsOffMs >= POST_ROLL_MS);
    if (elapsed - lastLogMs >= LOG_INTERVAL_MS || done) {
      lastLogMs = elapsed;
      RobotPose_c p = robot.odometry.pose;
      rows.add(elapsed + "," + segLabel + "," + nf(leftPwm, 0, 1) + "," + nf(rightPwm, 0, 1) + ","
        + leftCounts + "," + rightCounts + "," + nf(odomX, 0, 3) + "," + nf(odomY, 0, 3) + ","
        + nf(odomTheta, 0, 5) + "," + nf(p.x, 0, 3) + "," + nf(p.y, 0, 3) + "," + nf(p.theta, 0, 5));
      truePath.add(new float[] {p.x, p.y});
    }
    if (done) mode = STOPPED;
  }

  void updateOdometry(int leftCounts, int rightCounts) {
    float mmPerCount = 1.0 / countsPerMm();
    float sl = (leftCounts - prevLeft) * mmPerCount;
    float sr = (rightCounts - prevRight) * mmPerCount;
    prevLeft = leftCounts;
    prevRight = rightCounts;
    float ds = 0.5 * (sl + sr);
    odomX += ds * cos(odomTheta);
    odomY += ds * sin(odomTheta);
    odomTheta += (sr - sl) / CTRL_WHEEL_SEPARATION_MM;
  }

  void startSegment(long now, int leftCounts, int rightCounts) {
    float[] s = wheelDistances(PATH[segIndex], CTRL_WHEEL_SEPARATION_MM);
    targetLeft = s[0] * countsPerMm();
    targetRight = s[1] * countsPerMm();
    segLeft0 = leftCounts;
    segRight0 = rightCounts;
    segStartMs = now;
    segLabel = segIndex + 1;
    mode = MOVING;
  }

  void stopMotors(Robot_c robot) {
    robot.setMotorPWM(0, 0);
    leftPwm = rightPwm = 0;
  }

  void endMotion(Robot_c robot, long now) {
    stopMotors(robot);
    motorsOffMs = now;
    segLabel = PATH.length + 1;
    mode = POST_ROLL;
  }

  void driveSegment(Robot_c robot, long now, int leftCounts, int rightCounts) {
    PathSegment_c s = PATH[segIndex];
    float dl = leftCounts - segLeft0;
    float dr = rightCounts - segRight0;
    float al = abs(targetLeft), ar = abs(targetRight);
    float tmax = max(al, ar);
    float pl = (al > 0.5) ? dl / targetLeft : 1.0;
    float pr = (ar > 0.5) ? dr / targetRight : 1.0;
    float p = (al * pl + ar * pr) / (al + ar);

    if (p >= 1.0) {
      stopMotors(robot);
      mode = SETTLING;
      settleStartMs = now;
      return;
    }
    if (now - segStartMs >= SEGMENT_TIMEOUT_MS) {
      endMotion(robot, now);
      return;
    }
    float base = constrain(K_DIST * (1.0 - p) * tmax, MIN_PWM, s.maxPwm);
    long t = now - segStartMs;
    if (t < RAMP_MS) base = MIN_PWM + (base - MIN_PWM) * t / (float)RAMP_MS;
    float magL = FF_DEADBAND + (base - FF_DEADBAND) * al / tmax;
    float magR = FF_DEADBAND + (base - FF_DEADBAND) * ar / tmax;
    float corr = constrain(K_SYNC * (pl - pr) * tmax, -MAX_CORRECTION, MAX_CORRECTION);
    leftPwm = (targetLeft >= 0 ? 1 : -1) * (magL - corr);
    rightPwm = (targetRight >= 0 ? 1 : -1) * (magR + corr);
    robot.setMotorPWM(leftPwm, rightPwm);
  }

  /** @brief Writes the stored rows; columns match the robot CSV plus true pose. */
  void writeCsv(String path) {
    PrintWriter w = createWriter(path);
    w.println("time_ms,segment,left_pwm,right_pwm,left_counts,right_counts,x_mm,y_mm,theta_rad,"
      + "x_true_mm,y_true_mm,theta_true_rad");
    for (String r : rows) w.println(r);
    w.flush();
    w.close();
  }
}

/**
 * @brief Runs a complete path with one model as fast as possible (no drawing)
 * and saves twin_<model>_processing.csv next to the sketch.
 * @return The final true pose {x, y, theta}.
 */
float[] runBatch(TwinModel_c model, String csvPath) {
  Robot_c r = new Robot_c(model, null, 0, 0);
  Controller_c c = new Controller_c();
  c.setSignal(1);
  int guard = 0;
  while (!c.finished() && guard < 20000) {
    r.advanceTime(CONTROL_INTERVAL_MS);
    c.update(r);
    r.updatePhysics(CONTROL_INTERVAL_MS);
    guard++;
  }
  c.writeCsv(csvPath);
  return new float[] {r.odometry.pose.x, r.odometry.pose.y, r.odometry.pose.theta};
}
