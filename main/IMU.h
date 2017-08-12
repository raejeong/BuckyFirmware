#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BNO055.h"
#include "utility/imumaths.h"

class IMU
{
private:
	Adafruit_BNO055 bno;

public:
	IMU();
	~IMU();
	bool initialize_imu();
	bool calibrate_imu(int (&calibration_info)[4]);
	void get_imu_data(float (&robot_data)[9]);
};

#endif