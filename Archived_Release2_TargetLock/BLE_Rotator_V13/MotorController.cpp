#include "MotorController.h"

MotorController* MotorController::instance_ = nullptr;

MotorController::MotorController(const Pins& pins, float counts_per_rev)
: pins_(pins), counts_per_rev_(counts_per_rev) {}

bool MotorController::begin(uint8_t dac_addr) {
  dac_addr_ = dac_addr;
  instance_ = this;

  pinMode(pins_.in1, OUTPUT);
  pinMode(pins_.in2, OUTPUT);
  pinMode(pins_.slp, OUTPUT);
  pinMode(pins_.flt, INPUT);
  pinMode(pins_.ipropi, INPUT);
  pinMode(pins_.enc_a, INPUT);
  pinMode(pins_.enc_b, INPUT);

  digitalWrite(pins_.in1, LOW);
  digitalWrite(pins_.in2, LOW);
  digitalWrite(pins_.slp, LOW);

  // Attach quadrature on both edges; single handler reads A & B
  attachInterrupt(digitalPinToInterrupt(pins_.enc_a), isrTrampoline, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins_.enc_b), isrTrampoline, CHANGE);

  // DAC (Wire.begin() should already be called in your sketch)
  if (!dac_.begin(dac_addr_)) {
    return false;
  }
  setOvercurrentTrip(overcurrent_trip_mA_);
  return true;
}

void MotorController::drive(int16_t cmd) {
  // Angle limit guard (same logic as your sketch)
  float angle = getAngleDegCumulative();
  if ((angle >= angle_limit_deg_ - angle_hyst_deg_ && cmd > 0) ||
      (angle <= -angle_limit_deg_ + angle_hyst_deg_ && cmd < 0)) {
#ifdef PRINT_MOTOR_COMMAND
    Serial.print("Blocked @ angle ");
    Serial.println(angle, 2);
#endif
    cmd = 0;
  }

  last_cmd_ = cmd;

  if (cmd == 0) {
    digitalWrite(pins_.slp, LOW);         // sleep
    analogWrite(pins_.in1, 0);
#ifdef PRINT_MOTOR_COMMAND
    Serial.println("Cmnd:0 | Direction: STOP");
#endif
    return;
  }

  digitalWrite(pins_.slp, HIGH);          // wake

  // PHASE: IN2 = HIGH when cmd > 0 (matches your chosen mapping)
  digitalWrite(pins_.in2, (cmd > 0) ? HIGH : LOW);

  // PWM magnitude: map [-512..512] -> [0..1023] with floor
  uint16_t mag = (cmd < 0) ? -cmd : cmd;
  if (mag > 512) mag = 512;
  uint16_t pwm = (uint16_t)( (uint32_t)mag * 1023u / 512u );
  if (pwm < min_pwm_) pwm = min_pwm_;
  analogWrite(pins_.in1, pwm);

#ifdef PRINT_MOTOR_COMMAND
  Serial.print("Cmnd:");
  Serial.print((int)mag);
  Serial.print(" | Direction: ");
  Serial.print((last_cmd_>0) ? "CW" : "CCW");
  Serial.print(" | A:");
  Serial.print(motor_current_);
  Serial.print(" | Angle: ");
  Serial.println(angle, 2);
#endif
}

void MotorController::stop() {
  drive(0);
}

void MotorController::updateCurrent() {
  int adc = analogRead(pins_.ipropi);
  float v  = (adc * ADC_VREF) / ADC_MAX;
  float iA = v / (RIPROPI_OHMS * IPROPI_GAIN); // I = V/(R*gain)

  samples_[samp_idx_] = iA;
  samp_idx_ = (uint8_t)((samp_idx_ + 1) % CUR_WIN);

  float sum = 0;
  for (uint8_t i = 0; i < CUR_WIN; ++i) sum += samples_[i];
  motor_current_ = sum / CUR_WIN;
}

float MotorController::getAngleDeg() const {
  noInterrupts();
  int32_t c = encoder_count_;
  interrupts();
  float deg = (c * 360.0f) / counts_per_rev_;
  deg = fmodf(deg + 360.0f, 360.0f);
  return deg; // [0,360)
}

float MotorController::getAngleDegCumulative() const {
  noInterrupts();
  int32_t c = encoder_count_;
  interrupts();
  return (c * 360.0f) / counts_per_rev_; // unbounded
}

void MotorController::resetEncoder() {
  noInterrupts();
  encoder_count_ = 0;
  interrupts();
}

void MotorController::setOvercurrentTrip(uint16_t mA) {
  overcurrent_trip_mA_ = mA;
  float tripA = mA / 1000.0f;
  float vref  = tripA * RIPROPI_OHMS * IPROPI_GAIN;     // V = I*R*gain
  uint16_t dacVal = (uint16_t)((vref / DAC_VDD) * 4095.0f);
  dac_.setVoltage(dacVal, false);
#ifdef DEBUG_PRINTS
  Serial.print("Trip current: "); Serial.print(overcurrent_trip_mA_);
  Serial.print(" mA | VREF: ");   Serial.print(vref, 3);
  Serial.print(" V | DAC = ");    Serial.println(dacVal);
#endif
}

void MotorController::setAngleLimits(float limit_deg, float hysteresis_deg) {
  angle_limit_deg_ = limit_deg;
  angle_hyst_deg_  = hysteresis_deg;
}

void MotorController::setMinPwm(uint16_t min_pwm) {
  min_pwm_ = min_pwm;
}

void MotorController::isrTrampoline() {
  if (instance_) instance_->encoderISR();
}

void MotorController::encoderISR() {
  static uint8_t lastAB = 0;
  uint8_t a = digitalRead(pins_.enc_a);
  uint8_t b = digitalRead(pins_.enc_b);
  uint8_t ab = (a << 1) | b;

  // Gray-code delta table (same as your sketch)
  static const int8_t dirTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
  };
  encoder_count_ += dirTable[(lastAB << 2) | ab];
  lastAB = ab;
}