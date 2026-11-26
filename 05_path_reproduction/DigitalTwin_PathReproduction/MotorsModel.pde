/**
 * @brief Paired motor-response model, configurable between the supplied ideal
 * assumption and the calibrated model of this robot.
 * @details Each wheel: the requested PWM is truncated to an integer (as the
 * robot's Robot.cpp sends int16 values), optionally delayed (right wheel),
 * mapped to a target speed k * (|PWM| - deadband) above the deadband (zero
 * below it), and approached with a first-order lag. Updated every 1 ms.
 * With the course model (k = 1.25 mm/s per PWM, no deadband, lag 1 ms, no
 * delay) this reduces to the supplied linear map: PWM 200 -> 0.25 mm/ms.
 */
class MotorsModel_c {
  TwinModel_c model;
  MotorModel_c left = new MotorModel_c();
  MotorModel_c right = new MotorModel_c();
  float[] rightQueue = new float[64];
  int queueIndex = 0;

  MotorsModel_c(TwinModel_c modelIn) { model = modelIn; }

  /** @brief Stores requested left and right PWM values after limiting them. */
  void setPWM(float leftPwm, float rightPwm) {
    left.setPWM(leftPwm);
    right.setPWM(rightPwm);
  }

  /** @brief Advances both wheel speeds by 1 ms. */
  void update1ms() {
    float uLeft = (float)((int)left.pwm);                 // int16 truncation
    rightQueue[queueIndex] = (float)((int)right.pwm);
    int delayed = (queueIndex - model.rightDelayMs + rightQueue.length) % rightQueue.length;
    float uRight = rightQueue[delayed];
    queueIndex = (queueIndex + 1) % rightQueue.length;
    left.update(uLeft, model.kLeft, model.deadband, model.tauMs);
    right.update(uRight, model.kRight, model.deadband, model.tauMs);
  }

  void reset() {
    left.speedMmPerS = 0;
    right.speedMmPerS = 0;
    for (int i = 0; i < rightQueue.length; i++) rightQueue[i] = 0;
    setPWM(0, 0);
  }
}

/** @brief Response model for one motor. */
class MotorModel_c {
  float pwm = 0;
  float speedMmPerS = 0;

  /** @brief Limits and stores a requested PWM value in [-200, +200]. */
  void setPWM(float value) { pwm = constrain(value, -200, 200); }

  /** @brief Moves the wheel speed one millisecond towards its target. */
  void update(float u, float gain, float deadband, float tauMs) {
    float target = 0;
    if (abs(u) > deadband) target = (u > 0 ? 1 : -1) * gain * (abs(u) - deadband);
    speedMmPerS += (target - speedMmPerS) / tauMs;
  }
}
