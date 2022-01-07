#pragma once
#include <../../include/configs.h>
#include <ESP32Encoder.h>
#include <ESP32Servo.h>
#include <MotorCtrl.h>

class ArmHandle
{
private:
    const int servoPin;
    MotorCtrl motor_yaw;
    MotorCtrl motor_height;

    Servo servo;

public:
    struct Command {
        double yaw;
        double pitch;
        int height;
    };

    ArmHandle(const int _servoPin,
              ESP32Encoder *const _enc_yaw,
              ESP32Encoder *const _enc_height);

    void init();

    void encoder_update();

    void control_update();

    void set(Command cmd);

    bool validCommandCheck(Command cmd);

    void processInput();

    void printState();
};