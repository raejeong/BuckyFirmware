#include "digitalWriteFast.h"
#include "PinDefines.h"
#include "Robot_ISR.h"
#include "IMU.h"
#include "BalanceMotors.h"

#define STEPS_PER_REVOLUTION 200
#define RPM_TO_RAD_PER_SECONDS 0.10471975512
#define DATA_FLAG 999

float robot_data[9];
IMU robot_imu;
BalanceMotors robot_motors;
float commands[] = {0, 0, 0};
int32_t last_time = millis();
int32_t last_time_1 = millis();
int32_t last_motor_1 = 0;
int32_t last_motor_2 = 0;
float last_motor_1_velocity = 0.0;
float last_motor_2_velocity = 0.0;
float motor_1_velocity = 0.0;
float motor_2_velocity = 0.0;

void get_velocity(bool encoder)
{
	float calcR, calcL = 0.0f;
	if(encoder)
	{
	    int32_t curTime = millis();
	    float deltaT = (curTime - last_time) / 1000.0;
	    // 
	    // Calculate the current speed.
	    calcL = TO_OMEGA(gLeftEncoderTicks - last_motor_1) / (deltaT);
	    calcR = TO_OMEGA(gRightEncoderTicks - last_motor_2) / (deltaT);
	    // 
	    // Update state variables.
	    last_motor_1 = gLeftEncoderTicks;
	    last_motor_2 = gRightEncoderTicks;
	    last_time = curTime;
	}
    robot_data[7] = calcL;
    robot_data[8] = calcR;
}

void get_accleration(bool encoder)
{
	float calcR, calcL = 0.0f;
	if(encoder)
	{
		get_velocity(true);
	    int32_t curTime = millis();
	    float deltaT = (curTime - last_time_1) / 1000.0;
	    // 
	    // Calculate the current speed.
	    calcL = (motor_1_velocity - last_motor_1_velocity) / (deltaT);
	    calcR = (motor_2_velocity - last_motor_2_velocity) / (deltaT);
	    // 
	    // Update state variables.
	    last_motor_1_velocity = motor_1_velocity;
	    last_motor_2_velocity = motor_2_velocity;
	    last_time_1 = curTime;
	}
    // robot_data[7] = calcL;
    // robot_data[8] = calcR;
}

void send_info(String info)
{
	Serial.print(0);
	Serial.print(", ");
	Serial.println(info);
}

void send_data(float data[])
{
	Serial.print(1);
	Serial.print(", ");
	int8_t data_len = 9;
	for(int8_t i=0; i<data_len-1; i++)
	{
		Serial.print(data[i]);
		Serial.print(", ");
	}
	Serial.println(data[data_len-1]);
}

void read_data()
{
	if(Serial.available() > 0)
	{
		String data_string = Serial.readString();
		int commaIndex = data_string.indexOf(',');
		int secondCommaIndex = data_string.indexOf(',', commaIndex + 1);
		int thirdCommaIndex = data_string.indexOf(',', secondCommaIndex + 1);
		int fourthCommaIndex = data_string.indexOf(',', thirdCommaIndex + 1);
		String firstValue = data_string.substring(0, commaIndex);
		String secondValue = data_string.substring(commaIndex + 1, secondCommaIndex);
		String thirdValue = data_string.substring(secondCommaIndex + 1, thirdCommaIndex);
		String fourthValue = data_string.substring(thirdCommaIndex + 1, fourthCommaIndex);
		int data_flag = firstValue.toInt();
		int data_0 = secondValue.toInt();
		float data_1 = thirdValue.toFloat();
		float data_2 = fourthValue.toFloat();

		if(data_flag==DATA_FLAG)
		{
			commands[0] = data_0;
			commands[1] = data_1;
			commands[2] = data_2;
			String data_confirm = "INFO: Command data received.";
			data_confirm += commands[0];
			data_confirm += ",";
			data_confirm += commands[1];
			data_confirm += ",";
			data_confirm += commands[2];
			// send_info(data_confirm);
		}
		else
		{
			send_info("INFO: DATA_FLAG wrong!");
			while(Serial.available()>0)
			{
				Serial.read();
			}
		}
	}
}

void jump(int jump_effort)
{
	analogWrite(PIN::jumpPin, jump_effort);
}

void setup()
{
	pinMode(PIN::jumpPin, OUTPUT);
	pinMode(PIN::motor1PWM,OUTPUT);		
  	pinMode(PIN::motor2PWM,OUTPUT);	
  	digitalWrite(PIN::motor1PWM,LOW);	
  	digitalWrite(PIN::motor2PWM,LOW);	
  	digitalWrite(PIN::motor1DIR,LOW);	
  	digitalWrite(PIN::motor2DIR,LOW);	
  	pinMode(PIN::motor1DIR,OUTPUT);	
  	pinMode(PIN::motor2DIR,OUTPUT);
	bool success;
	robot_motors.AFMS.begin();
	Serial.begin(115200);
	send_info("INFO: Serial started");
	success = robot_imu.initialize_imu();
	while(!success)
	{
		success = robot_imu.initialize_imu();
	}
	send_info("INFO: IMU initialized!");
	delay(1000);
	// int calibration_info[] = {0, 0, 0, 0};
	// String calibration_string;
	// success = false;
	// do
	// {
	// 	success = robot_imu.calibrate_imu(calibration_info);
	// 	calibration_string = "ERROR: IMU needs to be calibrated. system: ";
	// 	calibration_string += calibration_info[0];
	// 	calibration_string += ", gyro: ";
	// 	calibration_string += calibration_info[1];
	// 	calibration_string += ", accel: ";
	// 	calibration_string += calibration_info[2];
	// 	calibration_string += ", mag: ";
	// 	calibration_string += calibration_info[3];
	// 	send_info(calibration_string);
	// 	delay(1000);
	// 	// success = true;
	// }while(!success);
	// calibration_string = "ERROR: IMU calibrated. system: ";
	// calibration_string += calibration_info[0];
	// calibration_string += ", gyro: ";
	// calibration_string += calibration_info[1];
	// calibration_string += ", accel: ";
	// calibration_string += calibration_info[2];
	// calibration_string += ", mag: ";
	// calibration_string += calibration_info[3];
	// send_info(calibration_string);
    setupPins();
    Serial.setTimeout(10);
}

void loop()
{
	read_data();
	robot_imu.get_imu_data(robot_data);
	get_velocity(true);
	jump(commands[0]);
	robot_motors.run_motors(commands[1], commands[2], robot_data[7], robot_data[8]);
	send_data(robot_data);
}
