/**
 * @brief Top-level model of the robot, excluding the environment.
 * @details Same structure as the supplied Robot_c, with a configurable motor
 * and geometry model (TwinModel_c) and 1 ms physics sub-steps. odometry.pose
 * is the twin's TRUE pose; odometry.encoders are its (continuous) encoder
 * counts. The controller reads integer counts, as on the StampC3.
 */
class Robot_c {
  /** @brief Radius used only to draw the robot body, in mm. */
  final float bodyRadiusMm = 48.0;
  final float encoderCountsPerRevolution = 358.3;
  TwinModel_c model;

  long simulationMillis = 0;
  PImage surfaceImage;
  final float surfaceOffsetX;
  final float surfaceOffsetY;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors;
  OdometryModel_c odometry;
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  Robot_c(TwinModel_c modelIn, PImage surfaceIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    model = modelIn;
    surfaceImage = surfaceIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    motors = new MotorsModel_c(model);
    odometry = new OdometryModel_c(model.radiusMm, encoderCountsPerRevolution);
    sensors[0] = new SurfaceSensorModel_c("DN1", 45, 45);
    sensors[1] = new SurfaceSensorModel_c("DN2", 45, 14);
    sensors[2] = new SurfaceSensorModel_c("DN3", 45, 0);
    sensors[3] = new SurfaceSensorModel_c("DN4", 45, -14);
    sensors[4] = new SurfaceSensorModel_c("DN5", 45, -45);
    odometry.reset(0.0, 0.0, 0.0, 0.0, 0.0);
  }

  long getMillis() { return simulationMillis; }
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  void getSurfaceSensors() {
    if (surfaceImage == null) return;
    RobotPose_c pose = odometry.pose;
    for (int i = 0; i < sensors.length; i++) {
      surface.reading[i] = sensors[i].sample(surfaceImage, surfaceOffsetX,
        surfaceOffsetY, pose.x, pose.y, pose.theta);
    }
  }

  void getEncoders() { }
  void getPose() { }
  /** @return Integer encoder counts, as the StampC3 receives them. */
  int getLeftEncoderCount() { return floor(odometry.encoders.left); }
  int getRightEncoderCount() { return floor(odometry.encoders.right); }

  void setMotorPWM(float leftPwm, float rightPwm) { motors.setPWM(leftPwm, rightPwm); }

  void resetForRun(float x, float y, float theta, float leftEncoder, float rightEncoder) {
    odometry.reset(x, y, theta, leftEncoder, rightEncoder);
    motors.reset();
    simulationMillis = 0;
  }

  /** @brief Advances motors and differential-drive motion in 1 ms steps. */
  void updatePhysics(int dtMs) {
    for (int i = 0; i < dtMs; i++) {
      motors.update1ms();
      float leftDistance = motors.left.speedMmPerS / 1000.0;
      float rightDistance = motors.right.speedMmPerS / 1000.0;
      odometry.update(leftDistance, rightDistance, model.separationMm);
    }
  }

  float[] sensorReadings() { return surface.reading; }

  void draw() {
    RobotPose_c pose = odometry.pose;
    pushMatrix();
    translate(pose.x, pose.y);
    rotate(pose.theta);
    fill(190, 205, 220);
    stroke(40);
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .72, 0);
    fill(40, 100, 220);
    noStroke();
    triangle(bodyRadiusMm * .92, 0, bodyRadiusMm * .62, bodyRadiusMm * .14,
      bodyRadiusMm * .62, -bodyRadiusMm * .14);
    popMatrix();
  }
}
