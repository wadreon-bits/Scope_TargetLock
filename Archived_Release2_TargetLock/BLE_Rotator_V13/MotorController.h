#pragma once
#include <Arduino.h>
#include <Adafruit_MCP4725.h>

//#define PRINT_MOTOR_COMMAND

class MotorController {
public:
  struct Pins {
    uint8_t in1;     // PWM pin
    uint8_t in2;     // PHASE pin
    uint8_t slp;     // SLEEP pin (active LOW sleep)
    uint8_t flt;     // DRV fault input (optional)
    uint8_t ipropi;  // current sense (ADC)
    uint8_t enc_a;   // encoder A
    uint8_t enc_b;   // encoder B
  };

  // counts_per_rev = 16940.0 for your 11 PPR * 4 * 110:1 * 3.5:1
  MotorController(const Pins& pins, float counts_per_rev);

  // Initializes pins, attaches encoder ISRs, starts DAC (addr default 0x60)
  bool begin(uint8_t dac_addr = 0x60);

  // Command drive: signed [-512..+512] like your existing code
  void drive(int16_t cmd);
  void stop();                       // hard stop (sleep + PWM=0)

  // Current measurement: call periodically when running
  void updateCurrent();
  float currentA() const { return motor_current_; }

  // Encoder angle helpers
  float getAngleDeg() const;             // [0,360)
  float getAngleDegCumulative() const;   // unwrapped
  void  resetEncoder();

  // Overcurrent (DRV8874 IPROPI DAC Vref) in mA
  void     setOvercurrentTrip(uint16_t mA);
  uint16_t getOvercurrentTrip_mA() const { return overcurrent_trip_mA_; }

  // Limits and PWM floor
  void setAngleLimits(float limit_deg, float hysteresis_deg);
  void setMinPwm(uint16_t min_pwm);

  // Convenience
  int16_t state() const { return last_cmd_; }     // last commanded signed magnitude
  bool    isRunning() const { return last_cmd_ != 0; }

private:
  // Encoder ISR trampoline
  static void isrTrampoline();
  void       encoderISR();

  // Members
  Pins   pins_;
  float  counts_per_rev_;
  Adafruit_MCP4725 dac_;
  uint8_t dac_addr_{0x60};

  // Encoder
  volatile int32_t encoder_count_{0};
  static MotorController* instance_;

  // Current filter
  static constexpr float ADC_VREF      = 3.3f;
  static constexpr int   ADC_MAX       = 4095;
  static constexpr uint8_t CUR_WIN     = 10;
  static constexpr float IPROPI_GAIN   = 0.000455f; // A at IPROPI per A motor
  static constexpr float RIPROPI_OHMS  = 2300.0f;   // your resistor
  static constexpr float DAC_VDD       = 3.3f;

  float   samples_[CUR_WIN]{};
  uint8_t samp_idx_{0};
  float   motor_current_{0.0f};

  // Control/limits
  uint16_t min_pwm_{50};    // your MIN_PWM
  int16_t  last_cmd_{0};
  float    angle_limit_deg_{270.0f};
  float    angle_hyst_deg_{1.0f};
  uint16_t overcurrent_trip_mA_{1300};
};