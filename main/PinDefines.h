#ifndef __PIN_DEFINITIONS_H__
#define __PIN_DEFINITIONS_H__

#include <Arduino.h>

enum PIN: byte
{
	// 
	// Quadrature encoders.
	leftEncoderInterruptPin = 19, // A side of encoder
	leftEncoderPinB = 17,
	rightEncoderInterruptPin =  18, // A side of encoder
	rightEncoderPinB =  16,
	// Jumping solenoid pin
	jumpPin = 7,
	motor1PWM = 3,
	motor2PWM = 4,
	motor1DIR = 5,
	motor2DIR = 6
};

#endif