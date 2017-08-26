#include "BalanceMotors.h"

BalanceMotors::BalanceMotors():
	AFMS(Adafruit_MotorShield()),
	motor_1(AFMS.getMotor(1)),
	motor_2(AFMS.getMotor(2)),
    motor_3(AFMS.getMotor(3)),
    motor_4(AFMS.getMotor(4)),
	motor_1_control(K_P_1, K_I_1, K_D_1, MAX_SPEED, MIN_SPEED),
    motor_2_control(K_P_2, K_I_2, K_D_2, MAX_SPEED, MIN_SPEED)
{
}

BalanceMotors::~BalanceMotors()
{
}

void BalanceMotors::run_motors(float motor_1_speed_ref, float motor_2_speed_ref, float motor_1_measure, float motor_2_measure)
{
	float motor_1_command = motor_1_control.getCmd(motor_1_speed_ref, motor_1_measure);
	float motor_2_command = motor_2_control.getCmd(motor_2_speed_ref, motor_2_measure);
    // float motor_1_command = motor_1_speed_ref;
    // float motor_2_command = motor_2_speed_ref;

    if (motor_1_command < 0) {
        digitalWrite(PIN::motor1DIR,LOW);
        // motor_1->run(BACKWARD);
        // motor_3->run(BACKWARD);
    }
    else {
        digitalWrite(PIN::motor1DIR,HIGH);
        // motor_1->run(FORWARD);
        // motor_3->run(FORWARD);
    }
    if (motor_2_command < 0) {
        digitalWrite(PIN::motor2DIR,LOW);
        // motor_2->run(BACKWARD);
        // motor_4->run(BACKWARD);

    }
    else {
        digitalWrite(PIN::motor2DIR,HIGH);
        // motor_2->run(FORWARD);
        // motor_4->run(FORWARD);
    }
    motor_1_command = abs(motor_1_command);
    motor_2_command = abs(motor_2_command);
    if (motor_1_command < 20 && !(motor_1_command == 0)) {
        motor_1_command += 20;
    }
    if (motor_2_command < 20 && !(motor_2_command == 0)) {
        motor_2_command += 20;
    }
    // Serial.println(motor_1_command);
    analogWrite(PIN::motor1PWM, abs(motor_1_command));
    analogWrite(PIN::motor2PWM, abs(motor_2_command));
    // motor_1->setSpeed(abs(motor_1_command));
    // motor_2->setSpeed(abs(motor_2_command));
    // motor_3->setSpeed(abs(motor_1_command));
    // motor_4->setSpeed(abs(motor_2_command));
}