#include "MotorI2C.h"

MotorI2C::MotorI2C(uint32_t channel, PCA9685 *controller)
{
	this->controller = controller;
	this->channel = channel;
	speed = 0;
}

void MotorI2C::setSpeedPercent(uint16_t speed)
{
#ifdef TRACE
	printf("MotorI2C::setSpeed: speed [%i]\n", speed);
#endif

//	if(speed > 100)
//		speed = 100;
//
//	uint32_t stepEnd = 0;
//
//	if(speed > 0)
//		stepEnd = (uint32_t)round(PWM_STEPS * ((float)speed/100));

	uint16_t stepEnd = (speed * 10) + 2000;

	controller->setSpeed(channel, stepEnd);
	speed = stepEnd;
}

void MotorI2C::setSpeed(uint16_t speed)
{
	uint16_t stepEnd = speed + 1000;
	//printf("Setting speed [%i]\n", stepEnd);
	controller->setSpeed(channel, stepEnd);
	this->speed = stepEnd;
}

uint16_t MotorI2C::getSpeed()
{
	return speed;
}

uint16_t MotorI2C::getPercentage()
{
	uint16_t percentage = round((speed - 2000) / 10);
	return percentage;
}

void MotorI2C::stop()
{
	controller->setSpeed(channel,MOTOR_STOP);
}

void MotorI2C::shutdown()
{
	controller->shutdown(channel);
}


