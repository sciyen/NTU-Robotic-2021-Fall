#include <MotorCtrl.h>

MotorCtrl::MotorCtrl(const ENC_TYPE _enc_type,
                     const int32_t _sig_per_sec,
                     const int32_t _update_freq,
                     const double _gear_ratio,
                     const uint8_t _motor_pin_A,
                     const uint8_t _motor_pin_B,
                     const uint8_t _motor_pin_E,
                     const double _vel_limit,
                     ESP32Encoder *const _enc)
    : enc_type(_enc_type),
      sig_per_sec(_sig_per_sec),
      update_freq(_update_freq),
      gear_ratio(_gear_ratio),
      pin_motor_A(_motor_pin_A),
      pin_motor_B(_motor_pin_B),
      pin_motor_E(_motor_pin_E),
      vel_limit(_vel_limit),
      enc(_enc)
{
    dir = 1;
    enc = _enc;
    pre_count = 0;
    ctrl_counter = 0;

    pinMode(pin_motor_A, OUTPUT);
    pinMode(pin_motor_B, OUTPUT);
    digitalWrite(pin_motor_A, LOW);
    digitalWrite(pin_motor_B, LOW);
    analogWrite(pin_motor_E, 0);
}

volatile void MotorCtrl::encoder_update()
{
    counter = (int32_t) enc->getCount();
    if (enc_type == ENC_TYPE::SINGLE) {
        int32_t dist = (dir > 0 ? -1 : 1) * (counter - pre_count);
        velocity = dist * update_freq / sig_per_sec;
        position += dist;
    } else {
        velocity = (counter - pre_count) * update_freq / sig_per_sec;
        position = counter;
    }
    pre_count = counter;
}

volatile void MotorCtrl::drive(double cmd)
{
    int16_t pwm = (int16_t) (256 * cmd);
    pwm = (pwm > 255 ? 255 : (pwm < -255 ? -255 : pwm));
    dir = cmd > 0;

    digitalWrite(pin_motor_A, dir ? HIGH : LOW);
    digitalWrite(pin_motor_B, dir ? LOW : HIGH);
    analogWrite(pin_motor_E, abs(pwm));
    real_cmd = pwm / 256.0;
}

volatile void MotorCtrl::control()
{
    if (ctrl_counter >= 5) {
        // position control
        v_cmd = param.s_kp * (p_cmd - position);
        v_cmd = LIMIT(v_cmd, vel_limit);
        ctrl_counter = 0;
    }

    // velocity control
    double v_err = v_cmd - velocity;
    if (param.v_ki != 0) {
        accumulate += (v_err - (current_cmd - real_cmd) / param.v_ki);
        current_cmd = param.v_kp * v_err + param.v_ki * accumulate;
    } else {
        accumulate += (v_err);
        current_cmd = param.v_kp * v_err;
    }
    // Serial.println(current_cmd);
    drive(current_cmd);
    // deal with accumulator saturation
    ctrl_counter++;
}