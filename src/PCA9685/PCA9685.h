#ifndef PCA9685_H_
#define PCA9685_H_

#include <bitset>
#include <cstdio>
#include <unistd.h>

#define DEFAULT_ADDRESS			0x40

//Registers
#define REG_MODE1				0x00
#define REG_MODE2				0x01
#define REG_SUBADR1				0x02
#define REG_SUBADR2				0x03
#define REG_SUBADR3				0x04
#define REG_ALLCALL				0x05


//Channel 0
#define PWM0_ON_L				0x06
#define PWM0_ON_H				0x07
#define PWM0_OFF_L				0x08
#define PWM0_OFF_H				0x09

#define ALL_ON_L				0xFA
#define ALL_ON_H				0xFB
#define ALL_OFF_L				0xFC
#define ALL_OFF_H				0xFD

#define REG_PRE_SCALE			0xFE

#define MODE1_ALLCALL			0x01
#define MODE1_SLEEP				0x10
#define MODE1_RESTART			0x80

#define PWM_OFF_H_DISABLE		0x08

#define PWM_DEFAULT_FREQUENCY	(uint16_t)400
#define PWM_ZERO				(uint8_t)0
#define PWM_STEPS				(uint16_t)4096


class PCA9685
{
public:
	PCA9685();
	void reset();
	void init();
	void setFrequency(uint16_t frequency);
	void setSpeed(uint8_t channel, uint16_t pwmLength);
	void setSpeedAll(uint16_t pwmLength);
	void shutdown(uint8_t channel);
	void shutdownAll();

private:
	uint8_t devAddr;
};


#endif /* PCA9685_H_ */
