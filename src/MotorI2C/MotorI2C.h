#ifndef MOTORI2C_HPP_
#define MOTORI2C_HPP_

#include <cstdio>
#include <cmath>
#include <stdint.h>
#include <unistd.h>
#include "../PCA9685/PCA9685.h"

#define MOTOR_STOP			(uint16_t)2000
#define MOTOR_MAX			(uint16_t)3000

enum {
	CHANNEL0 = 0,
	CHANNEL1,
	CHANNEL2,
	CHANNEL3
};

class MotorI2C
{

public:
	MotorI2C(uint32_t channel, PCA9685 *controller);
	void setSpeedPercent(uint16_t speed);
	void setSpeed(uint16_t speed);
	uint16_t getPercentage();
	uint16_t getSpeed();
	//Set Motor to 0%
	void stop();
	//Shutdown Motor PWM Timer
	void shutdown();

private:
	uint16_t channel;
	uint16_t speed;
	PCA9685 *controller;
};

#endif /* MOTORI2C_HPP_ */
