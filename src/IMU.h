/*
  IMU.h - Library to use the IMU 
  Created by Romain Goussault <romain.goussault@gmail.com>
  
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  afloat with this program. If not, see <http://www.gnu.org/licenses/>. 
*/
#ifndef IMU_h
#define IMU_h

#include <cmath>
#include <unistd.h>
#include <cstdio>
#include <pigpio.h>
#include "MPU6050/MPU6050.h"




#define rac22 0.7071

#define ROLL_MAX_IMU  30
#define PITCH_MAX_IMU 30

#define ROLL_OFFSET -0.16
#define PITCH_OFFSET 2.10
#define YAW_OFFSET 0

#define	PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define MPU_CALIBRATION_ITERATIONS 200

enum {
	ROLL = 0,
	PITCH,
	YAW
};

class IMU
{

public:
	IMU();
	void init();
	bool getAngles(float angles[]);

private:
	/* IMU Data */
	bool dmpReady;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

	MPU6050 accelgyro;
};

#endif
