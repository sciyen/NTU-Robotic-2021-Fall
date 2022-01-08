#include <Arduino.h>
#include <ESP32Encoder.h>

#include <ArmHandle.h>
#include <MotorCtrl.h>
#include <configs.h>

ESP32Encoder enc_yaw;
ESP32Encoder enc_hei;

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timer_control = NULL;
portMUX_TYPE timerMux_control = portMUX_INITIALIZER_UNLOCKED;

ArmHandle arm(PIN_SERVO,
              PIN_VR_YAW,
              PIN_BTN_UP,
              PIN_BTN_DOWN,
              &enc_yaw,
              &enc_hei);

void IRAM_ATTR on_encoder_update()
{
    portENTER_CRITICAL_ISR(&timerMux);
    arm.encoder_update();
    portEXIT_CRITICAL_ISR(&timerMux);
}

void IRAM_ATTR on_motor_control()
{
    portENTER_CRITICAL_ISR(&timerMux_control);
    arm.control_update();
    portEXIT_CRITICAL_ISR(&timerMux_control);
}

void setup()
{
    arm.init();

    // Setup timer
    timer = timerBegin(0, 80, true);  // The clock freq. of the ESP is 80MHz
    timerAttachInterrupt(timer, &on_encoder_update, true);
    timerAlarmWrite(timer, 1000000 / CONFIG_ENC_UPDATE_FREQ, true);
    timerAlarmEnable(timer);

    // Setup timer
    timer_control =
        timerBegin(1, 80, true);  // The clock freq. of the ESP is 80MHz
    timerAttachInterrupt(timer_control, &on_motor_control, true);
    timerAlarmWrite(timer_control, 1000000 / CONFIG_MOTOR_CONTROL_FREQ, true);
    timerAlarmEnable(timer_control);

    // Initial state
    // arm.set(0, 0, 90);
}

void loop()
{
    arm.processInput();
    arm.printState();
    arm.handle_adjustment();
    delay(100);
}