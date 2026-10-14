#pragma once
#include <math.h>
#include "Magnetometer.h"
#include "MotorController.h"

class HeadingLock {
 public:
  struct Params {
    float   Kp;
    float   dead;   // deg
    int16_t min;    // minimum command to overcome stiction
    int16_t cap;    // max |command|
    Params() : Kp(14.0f), dead(2.0f), min(80), cap(400) {}
  };

  HeadingLock() : p_() {}                 // uses default Params
  explicit HeadingLock(const Params& p) : p_(p) {}

  bool  active() const { return active_; }
  float target() const { return target_deg_; }

  void  setTarget(float deg) { target_deg_ = norm360(deg); active_ = true; }

  bool  armAtCurrent(Magnetometer& mag, MotorController& motor) {
    if (!sensorsReady(mag, motor)) return false;
    float rot = mag.getRotatorHeading(motor.getAngleDegCumulative());
    if (rot < 0.0f) return false;
    setTarget(rot);
    return true;
  }

  void  drop(MotorController& motor) { active_ = false; }

  bool  sensorsReady(Magnetometer& mag, MotorController& motor) const {
    return mag.getRotatorHeading(motor.getAngleDegCumulative()) >= 0.0f;
  }

  void  tick(Magnetometer& mag, MotorController& motor) {
    if (!sensorsReady(mag, motor)) { motor.drive(0); return; }
    if (!active_) { motor.drive(0); return; }    
    float rot = mag.getRotatorHeading(motor.getAngleDegCumulative());
    if (rot < 0.0f) { motor.drive(0); return; }

    float err = wrap180(target_deg_ - rot);
    int16_t cmd = 0;
    if (fabsf(err) > p_.dead) {
      cmd = (int16_t)(p_.Kp * err);
      if (abs(cmd) < p_.min) cmd = (cmd > 0) ? p_.min : -p_.min;
      cmd = constrain(cmd, -p_.cap, p_.cap);
    }
    motor.drive(cmd);
  }

 private:
  Params p_;
  bool   active_     = false;
  float  target_deg_ = 0.0f;

  static inline float wrap180(float x) {
    x = fmodf(x + 180.0f, 360.0f); if (x < 0) x += 360.0f; return x - 180.0f;
  }
  static inline float norm360(float x) {
    x = fmodf(x, 360.0f); if (x < 0) x += 360.0f; return x;
  }
};