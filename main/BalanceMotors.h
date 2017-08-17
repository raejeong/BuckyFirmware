#ifndef BALANCE_MOTORS_H
#define BALANCE_MOTORS_H

#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"
#include "digitalWriteFast.h"
#include "PID.h"

// 
// Conversion factors from ticks to linear velocity.
#define TICKS_PER_REVOLUTION (12.0f * 47.0f)
#define TO_OMEGA(ticks) ((ticks) / TICKS_PER_REVOLUTION * 2.0f * M_PI)

#define K_P_1 10.0f
#define K_I_1 20.0f
#define K_D_1 0.3f

#define K_P_2 10.0f
#define K_I_2 20.0f
#define K_D_2 0.3f

#define MAX_SPEED 255
#define MIN_SPEED -255

class BalanceMotors
{
private:
	PID motor_1_control;
	PID motor_2_control;

public:
	Adafruit_MotorShield AFMS;
	Adafruit_DCMotor *motor_1, *motor_2, *motor_3, *motor_4;
	BalanceMotors();
	~BalanceMotors();
	void run_motors(float motor_1_speed_ref, float motor_2_speed_ref, float motor_1_speed, float motor_2_speed);
};

#endif