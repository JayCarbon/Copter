#ifndef RXCHANNEL_HPP_
#define RXCHANNEL_HPP_

#include <stdint.h>

typedef void (*CallBack_t)(int channel, int percentage);

class RcChannel
{
private:
	int 		channel,
				gpio,
				level,
				rangeLow,
				rangeHigh,
				percentage;

	uint16_t 	angleLower,
				angleUpper;

	bool 		bound,
		 	 	polling;

	uint32_t 	startPWM,
				endPWM,
				deltaPWM;
	CallBack_t callback;

	void _pulse(int gpio, int level, uint32_t tick);
	static void _pulseEx(int gpio, int level, uint32_t tick, void *orangeRx);

public:
	RcChannel(int channel, int gpio, int rangeLow, int rangeHigh, CallBack_t callback);
	bool bind();
	bool poll();
	void cancel(void);
	void setAngleLimits(int16_t lower, int16_t upper);
	int8_t getPercentage();
	int32_t getAngle();
	uint32_t getStartPWM();
	uint32_t getEndPWM();
	uint32_t getDeltaPWM();
};




#endif /* RXCHANNEL_HPP_ */
