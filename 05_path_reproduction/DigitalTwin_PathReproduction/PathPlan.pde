/**
 * @brief The designed path, the controller constants and the twin models.
 * @details Keep PATH and the controller constants identical to
 * ../Path_Reproduction/Path_Reproduction.ino and ../twin/twin.py so that the
 * real robot and both twins run exactly the same behaviour.
 */
final int SEG_STRAIGHT = 0, SEG_ROTATE = 1, SEG_ARC = 2;

/** One path segment: straight a = distance mm (signed); rotate/arc a = angle
 * deg (+ = CCW / left); arcs also have a radius in mm. */
class PathSegment_c {
  int type;
  float a;
  float radius;
  float maxPwm;
  String label;
  PathSegment_c(int typeIn, float aIn, float radiusIn, float maxPwmIn, String labelIn) {
    type = typeIn; a = aIn; radius = radiusIn; maxPwm = maxPwmIn; label = labelIn;
  }
}

PathSegment_c[] PATH = {
  new PathSegment_c(SEG_STRAIGHT,  200.0,  0.0, 30.0, "straight 200 slow"),
  new PathSegment_c(SEG_ARC,       180.0, 80.0, 35.0, "arc L R80 180"),
  new PathSegment_c(SEG_STRAIGHT,  100.0,  0.0, 50.0, "straight 100 fast"),
  new PathSegment_c(SEG_ROTATE,    -90.0,  0.0, 35.0, "rotate CW 90"),
  new PathSegment_c(SEG_STRAIGHT, -100.0,  0.0, 35.0, "reverse 100"),
  new PathSegment_c(SEG_ROTATE,     90.0,  0.0, 35.0, "rotate CCW 90"),
  new PathSegment_c(SEG_STRAIGHT,  100.0,  0.0, 40.0, "straight 100"),
  new PathSegment_c(SEG_ARC,        90.0, 60.0, 35.0, "arc L R60 90 (tight)"),
  new PathSegment_c(SEG_ROTATE,     90.0,  0.0, 35.0, "rotate CCW 90"),
  new PathSegment_c(SEG_STRAIGHT,   60.0,  0.0, 30.0, "straight 60 slow")
};

// ---- robot controller constants (the robot's own, calibrated geometry) ----
final float CTRL_WHEEL_RADIUS_MM = 16.36;
final float CTRL_WHEEL_SEPARATION_MM = 83.48;
final float CTRL_CPR = 358.3;
final float MIN_PWM = 25.0;
final float FF_DEADBAND = 15.0;
final float K_DIST = 0.3;
final float K_SYNC = 0.5;
final float MAX_CORRECTION = 10.0;
final int RAMP_MS = 200;
final int SETTLE_MS = 400;
final int CHECKPOINT_PAUSE_MS = 0;
final int SEGMENT_TIMEOUT_MS = 10000;
final int PRE_ROLL_MS = 500;
final int POST_ROLL_MS = 1000;
final int CONTROL_INTERVAL_MS = 10;
final int LOG_INTERVAL_MS = 20;

/** Left and right wheel distances (mm) of one segment. */
float[] wheelDistances(PathSegment_c s, float separationMm) {
  if (s.type == SEG_STRAIGHT) return new float[] {s.a, s.a};
  float ang = radians(s.a);
  float arc = (s.type == SEG_ARC) ? abs(ang) * s.radius : 0.0;
  return new float[] {arc - ang * separationMm / 2.0, arc + ang * separationMm / 2.0};
}

/**
 * @brief Physical assumptions of one twin.
 * "course": the supplied Digital Twin (linear PWM->speed, both wheels equal,
 * no deadband or lag, B = 90 mm, r = 16 mm).
 * "calibrated": fitted to this robot's Ex2-Ex4 logs by replaying the logged
 * PWM through the model (per-wheel gain above a deadband, lag, right delay).
 */
class TwinModel_c {
  String name;
  float kLeft, kRight;        // mm/s per PWM above the deadband
  float deadband;             // |PWM| below which a wheel does not turn
  float tauMs;                // first-order motor lag
  int rightDelayMs;           // right-wheel command delay
  float separationMm;         // physical wheel separation
  float radiusMm;             // physical wheel radius (encoder mm per count)
  TwinModel_c(String nameIn, float kL, float kR, float dz, float tau, int delayR,
    float sep, float rad) {
    name = nameIn; kLeft = kL; kRight = kR; deadband = dz; tauMs = tau;
    rightDelayMs = delayR; separationMm = sep; radiusMm = rad;
  }
}

TwinModel_c COURSE_MODEL = new TwinModel_c("course", 1.25, 1.25, 0.0, 1.0, 0, 90.0, 16.0);
TwinModel_c CALIBRATED_MODEL = new TwinModel_c("calibrated", 3.992, 3.117, 14.9, 5.0, 20, 83.48, 16.36);
