#include "../I2Cdev/I2Cdev.h"
#include "PCA9685.h"



PCA9685::PCA9685()
{
	devAddr = DEFAULT_ADDRESS;
}

void PCA9685::init()
{
}

void PCA9685::reset()
{
	printf("Resetting PCA9685 (Adafruit PWM 16)\n");
	//std::bitset<8> bits(oldMode1);
	//if(!bits.test(7))

	//disable all PWM
	shutdownAll();

	//Set's allcall, sleep, restart to true
	uint8_t mode1 = MODE1_ALLCALL | MODE1_SLEEP;
	I2Cdev::writeByte(devAddr, REG_MODE1, mode1 | MODE1_RESTART);
	I2Cdev::writeByte(devAddr, REG_MODE1, mode1 ^ MODE1_SLEEP);
	usleep(50000);
}

void PCA9685::setFrequency(uint16_t frequency)
{
	printf("Setting PWM frequency to [%zu]\n", frequency);

	//25MHZ clock /(4096 steps * frequency)) - 1
	uint32_t prescale =  (25000000/ (PWM_STEPS * frequency)) -1;
#ifdef DEBUG
	printf("PCA9685::setFrequency: prescale [%zu]\n", prescale);
#endif

	uint8_t oldMode;
	I2Cdev::readByte(devAddr, REG_MODE1, &oldMode);
	I2Cdev::writeByte(devAddr, REG_MODE1, (oldMode & 0x7F) | 0x10);
	I2Cdev::writeByte(devAddr, REG_PRE_SCALE, prescale);
	I2Cdev::writeByte(devAddr, REG_MODE1, oldMode);

	//sleep for 5 milliseconds to let clock stabalize
	//Specsheet says only 500us, but let's give it extra
	usleep(50000);

	I2Cdev::writeByte(devAddr, REG_MODE1, oldMode | 0x80);
}

void PCA9685::setSpeed(uint8_t channel, uint16_t stepEnd)
{
#ifdef TRACE
	printf("PCA9685::setSpeed: stepEnd [%zu]\n", stepEnd);
#endif

	I2Cdev::writeByte(devAddr, PWM0_ON_L + (4 * channel), PWM_ZERO);
	I2Cdev::writeByte(devAddr, PWM0_ON_H + (4 * channel), PWM_ZERO);
	I2Cdev::writeByte(devAddr, PWM0_OFF_L + (4 * channel), stepEnd & 0xFF);
	I2Cdev::writeByte(devAddr, PWM0_OFF_H + (4 * channel), stepEnd >> 8);
}

void PCA9685::setSpeedAll(uint16_t stepEnd)
{
#ifdef TRACE
	printf("PCA9685::setSpeedAll: stepEnd [%zu]\n", stepEnd);
#endif

	I2Cdev::writeByte(devAddr, ALL_ON_L, PWM_ZERO);
	I2Cdev::writeByte(devAddr, ALL_ON_H, PWM_ZERO);
	I2Cdev::writeByte(devAddr, ALL_OFF_L, stepEnd & 0xFF);
	I2Cdev::writeByte(devAddr, ALL_OFF_H, stepEnd >> 8);
}

void PCA9685::shutdown(uint8_t channel)
{
	printf("Shutting down PWM timer for channel [%zu]\n", channel);
	I2Cdev::writeByte(devAddr, PWM0_OFF_H + (4 * channel), PWM_OFF_H_DISABLE);
}

void PCA9685::shutdownAll()
{
	printf("Shutting down all PWM timers\n");
	I2Cdev::writeByte(devAddr, ALL_OFF_H, PWM_OFF_H_DISABLE);
}

