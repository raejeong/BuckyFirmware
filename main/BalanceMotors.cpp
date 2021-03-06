#include "BalanceMotors.h"

BalanceMotors::BalanceMotors():
	AFMS(Adafruit_MotorShield()),
	motor_1(AFMS.getMotor(1)),
	motor_2(AFMS.getMotor(2)),
	motor_1_control(K_P_1, K_I_1, K_D_1, MAX_SPEED, MIN_SPEED),
    motor_2_control(K_P_2, K_I_2, K_D_2, MAX_SPEED, MIN_SPEED)
{
}

BalanceMotors::~BalanceMotors()
{
}

void BalanceMotors::run_motors(float motor_1_speed_ref, float motor_2_speed_ref, float motor_1_speed, float motor_2_speed)
{
	float motor_1_command = motor_1_control.getCmd(motor_1_speed_ref, motor_1_speed);
	float motor_2_command = motor_2_control.getCmd(motor_2_speed_ref, motor_2_speed);

    if (motor_1_command < 0) {
        motor_1->run(BACKWARD);
    }
    else {
        motor_1->run(FORWARD);
    }
    if (motor_2_command < 0) {
        motor_2->run(BACKWARD);
    }
    else {
        motor_2->run(FORWARD);
    }
    motor_1->setSpeed(abs(motor_1_command));
    motor_2->setSpeed(abs(motor_2_command));
}