#include <ArmHandle.h>

ArmHandle::ArmHandle(const int _servoPin,
                     ESP32Encoder *const _enc_yaw,
                     ESP32Encoder *const _enc_height)
    : servoPin(_servoPin),
      motor_yaw(MotorCtrl::ENC_TYPE::FULL_QUAD,
                360,
                CONFIG_ENC_UPDATE_FREQ,
                GEAR_RATIO_YAW,
                PIN_MOTOR1_A,
                PIN_MOTOR1_B,
                PIN_MOTOR1_E,
                20,
                _enc_yaw),
      motor_height(MotorCtrl::ENC_TYPE::SINGLE,
                   360,
                   CONFIG_ENC_UPDATE_FREQ,
                   GEAR_RATIO_HEIGHT,
                   PIN_MOTOR2_A,
                   PIN_MOTOR2_B,
                   PIN_MOTOR2_E,
                   0.6,
                   _enc_height)
{
}

void ArmHandle::init()
{
    Serial.begin(115200);
    Serial.println("Arm Control start to initialization!");

    CtrlParam yaw_param = {.v_kp = 0.035, .v_ki = 0.03, .s_kp = 0.003};
    CtrlParam height_param = {.v_kp = 2, .v_ki = 0.1, .s_kp = 0.01};

    motor_yaw.set_param(yaw_param);
    motor_height.set_param(height_param);

    // Attach encoders
    ESP32Encoder::useInternalWeakPullResistors = UP;
    motor_yaw.enc->attachFullQuad(PIN_ENC_YAW_1, PIN_ENC_YAW_2);
    motor_yaw.enc->clearCount();

    motor_height.enc->attachSingleEdge(PIN_ENC_YAW_3, PIN_ENC_YAW_4);
    motor_height.enc->clearCount();
    Serial.println("Encoders attached");

    // Attach servo
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo.setPeriodHertz(50);  // Standard 50hz servo
    servo.attach(servoPin, SERVO_MIN_US, SERVO_MAX_US);
    Serial.println("Servo attached");

    Serial.println("Arm Control initialization finished!");
}

void ArmHandle::encoder_update()
{
    motor_yaw.encoder_update();
    motor_height.encoder_update();
}

void ArmHandle::control_update()
{
    motor_yaw.control();
    motor_height.control();
}

void ArmHandle::set(Command cmd)
{
    // To rotate counter-clockwise for 1 rotation
    motor_yaw.set_position(cmd.yaw);

    // To move up for 1 centimeter
    motor_height.set_position(cmd.height);

    servo.write((int) (RAD_TO_DEG * cmd.pitch));
}

bool ArmHandle::validCommandCheck(Command cmd)
{
    if (cmd.yaw < -PI || cmd.yaw > PI)
        return false;
    if (cmd.pitch < 0 || cmd.pitch > PI)
        return false;
    if (cmd.height < 0 || cmd.height > 35)
        return false;
    return true;
}

void ArmHandle::processInput()
{
    if (Serial.available()) {
        double yaw, pitch, height;
        int i = 0, j;
        String str = Serial.readStringUntil('\n');

        // Processing yaw
        j = str.indexOf(',');
        yaw = str.substring(i, j).toDouble();

        // Processing pitch
        i = j + 1;
        j = str.indexOf(',', i);
        pitch = str.substring(i, j).toDouble();

        // Processing height
        i = j + 1;
        j = str.length() - 1;
        height = str.substring(i, j).toInt();

        // Serial.println("Get: " + String(yaw) + ", " + String(pitch) + ", " +
        //               String(height));

        Command cmd = {.yaw = yaw, .pitch = pitch, .height = height};
        if (validCommandCheck(cmd))
            set(cmd);
        else {
            char buf[40];
            sprintf(buf, "{%.2lf, %.2lf, %d}", cmd.yaw, cmd.pitch, cmd.height);
            Serial.println("Warning: bad commands. " + String(buf));
        }
    }
}

void ArmHandle::printState()
{
    char buf[40];
    sprintf(buf, "%.2lf, %.2lf", motor_yaw.get_position(),
            motor_height.get_position());
    Serial.println(buf);
}