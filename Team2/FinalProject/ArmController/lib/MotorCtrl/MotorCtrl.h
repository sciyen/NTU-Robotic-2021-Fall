#pragma once
#include <ESP32Encoder.h>
#include <analogWrite.h>

#define LIMIT(C, L) ((C > L ? L : (C < -L ? -L : C)))

struct CtrlParam {
    double v_kp;
    double v_ki;
    double s_kp;
};

class MotorCtrl
{
public:
    enum ENC_TYPE { SINGLE, FULL_QUAD };

private:
    volatile int8_t dir;
    const ENC_TYPE enc_type;
    const double sig_per_sec;
    const double update_freq;
    const double gear_ratio;

    const uint8_t pin_motor_A;
    const uint8_t pin_motor_B;
    const uint8_t pin_motor_E;

    const double vel_limit;

    // For motor control
    volatile double accumulate;
    volatile double real_cmd;
    volatile double current_cmd;
    volatile int32_t pre_count;
    int32_t counter;
    int8_t ctrl_counter;

    volatile int32_t position;
    volatile int32_t position_offset;

public:
    ESP32Encoder *enc;

    volatile double v_cmd;
    volatile double p_cmd;

    volatile double velocity;

    CtrlParam param;

    MotorCtrl(const ENC_TYPE _enc_type,
              const int32_t _sig_per_sec,
              const int32_t _update_freq,
              const double _gear_ratio,
              const uint8_t _motor_pin_A,
              const uint8_t _motor_pin_B,
              const uint8_t _motor_pin_E,
              const double _vel_limit,
              ESP32Encoder *const _enc);
    volatile void encoder_update();
    volatile void drive(double cmd);

    void set_param(CtrlParam p) { param = p; }

    void set_position(double p) { p_cmd = p * gear_ratio; }

    void set_position_offset(double p) { position_offset = p * gear_ratio; }

    double get_position() { return (position + position_offset) / gear_ratio; }

    int32_t get_counter() { return counter; }

    volatile void control();
};