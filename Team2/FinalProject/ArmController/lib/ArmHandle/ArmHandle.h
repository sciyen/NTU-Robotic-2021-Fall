#pragma once
#include <../../include/configs.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <MotorCtrl.h>

class ArmHandle
{
private:
    const int servoPin;
    const int vr_pin;
    const int btnUpPin;
    const int btnDownPin;
    MotorCtrl motor_yaw;
    MotorCtrl motor_height;

    Servo servo;
    double pitch;
    double p_pitch;
    double height_offset;
    double yaw_offset;

public:
    struct Command {
        double yaw;
        double pitch;
        int height;
    };

    ArmHandle(const int _servoPin,
              const int _vr_pin,
              const int _btnUpPin,
              const int _btnDownPin,
              ESP32Encoder *const _enc_yaw,
              ESP32Encoder *const _enc_height);

    void init();

    void encoder_update();

    void control_update();

    void handle_adjustment();

    void set(Command cmd);

    bool validCommandCheck(Command cmd);

    void processInput();

    void printState();
};