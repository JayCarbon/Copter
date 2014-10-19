#include <cmath>
#include <pigpio.h>
#include <stdio.h>
#include "RcChannel.h"

RcChannel::RcChannel(int channel, int gpio,
		int rangeLow, int rangeHigh, CallBack_t callback)
{
	this->channel = channel;
	this->gpio = gpio;
	this->callback = callback;
	this->rangeLow = rangeLow;
	this->rangeHigh = rangeHigh;

	bound = false;
	polling = false;
	level = 0;
	percentage = 0;

	angleLower = -180;
	angleUpper = 180;

	//Timing
	startPWM = 0;
	endPWM = 0;
	deltaPWM = 0;

	bind();
	poll();
}

void RcChannel::_pulse(int currentGpio, int currentLevel, uint32_t tick)
{
	level = currentLevel;

	if(level == 1)
	{
		startPWM = tick;
	}
	else
	{
		endPWM = tick;
		deltaPWM = endPWM - startPWM;

		percentage = (int)(((float)(deltaPWM - rangeLow)/(float)(rangeHigh - rangeLow))*100);

		if(callback != NULL)
			(callback)(channel, percentage);
	}
}

void RcChannel::_pulseEx(int gpio, int level, uint32_t tick, void *user)
{
	RcChannel *self = (RcChannel *) user;

	self->_pulse(gpio, level, tick);
}

bool RcChannel::bind()
{
	if(bound)
		return bound;

	if(gpioSetMode(gpio, PI_INPUT) == 0)
	{
		printf("Setmode PI_INPUT on gpio[%i]\n", gpio);
		return bound = true;
	}

	printf("Failed to bind for gpio [%i]\n", gpio);

	return false;
}

bool RcChannel::poll()
{
	if(polling)
		return polling;

	if(gpioSetAlertFuncEx(gpio, _pulseEx, this) == 0)
	{
		printf("Polling on gpio[%i]\n", gpio);
		return polling = true;
	}

	printf("Failed to start polling for gpio [%i]", gpio);

	return false;
}

void RcChannel::cancel()
{
	gpioSetAlertFuncEx(gpio, 0, this);
}

void RcChannel::setAngleLimits(int16_t lower, int16_t upper)
{
	angleLower = lower;
	angleUpper = upper;
}

int8_t RcChannel::getPercentage()
{
	if(bind() && poll())
		return percentage;

	return 0;
}

int32_t RcChannel::getAngle()
{
	//left -180
	//right 180
	//360 * .% - 180

	int16_t angle = round(3.6 * percentage) - 180;

	if(angle < angleLower)
		angle = angleLower;

	if(angle > angleUpper)
		angle = angleUpper;

	return angle;
}

uint32_t RcChannel::getStartPWM()
{
	return startPWM;
}

uint32_t RcChannel::getEndPWM()
{
	return endPWM;
}

uint32_t RcChannel::getDeltaPWM()
{
	return deltaPWM;
}



