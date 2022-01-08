#pragma once

// pin settings for encoder
#define PIN_ENC_YAW_1 21
#define PIN_ENC_YAW_2 22
#define PIN_ENC_YAW_3 18
#define PIN_ENC_YAW_4 19

#define PIN_MOTOR1_A 25
#define PIN_MOTOR1_B 26
#define PIN_MOTOR1_E 33

#define PIN_MOTOR2_A 13
#define PIN_MOTOR2_B 14
#define PIN_MOTOR2_E 12

#define PIN_SERVO 15

#define PIN_VR_YAW 34
#define PIN_BTN_UP 35
#define PIN_BTN_DOWN 36

// encoder update freqency
#define CONFIG_ENC_UPDATE_FREQ 20

#define CONFIG_MOTOR_CONTROL_FREQ 20

// The number of signals for a rotation
// number of slots on encoder: 24
// full quad rotary encoder: 4
// gear ratio of gearbox: 50
// gear ratio of outter gears: 24/11
#define GEAR_RATIO_YAW ((double) 24 * 4 * 50 * 24 / 11 / PI)

// The number of signals for 1 millimeter
// number of slot for 1 rotation: 180
// pitch of the screw: 1.2 mm
//
#define GEAR_RATIO_HEIGHT (180 / 1.2)

#define SERVO_MIN_US 600
#define SERVO_MAX_US 2600