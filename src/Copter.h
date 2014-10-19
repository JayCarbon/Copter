#ifndef COPTER_HPP_
#define COPTER_HPP_

#include <stdlib.h>
#include <pigpio.h>


#define MAX_CHANNELS 	4	// Throttle Aileron Elevator Rudder
#define MAX_MOTORS 		4 	// x config
#define MAX_PIDS		6

#define TASK_200HZ		1
#define TASK_100HZ 		2
#define TASK_50HZ 		4
#define TASK_10HZ 		20
#define TASK_5HZ		40
#define TASK_2HZ		100
#define TASK_1HZ 		200

#define PID_ROLL_RATE 0
#define PID_PITCH_RATE 1
#define PID_YAW_RATE 2

#define PID_ROLL_STAB 3
#define PID_PITCH_STAB 4
#define PID_YAW_STAB 5

//function declares
void process200HzTasks(void);
void process100HzTasks(void);
void process50HzTasks(void);
void process10HzTasks(void);
void process5HzTasks(void);
void process2HzTasks(void);
void process1HzTasks(void);
void updateRollPid(void);
void StartupMotors(void);
void processPilotCommands(void);
void processFlightControl(void);
void loop(void);

void signal_callback_handler(int signum);

//Channel alias
enum {
	RC_THROTTLE = 0,
	RC_ROLL, 		// x axis
    RC_PITCH,		// y axis
    RC_YAW,			// z axis
    RC_GEAR,		// unused
    RC_AUX1			// unused
};

enum {
	RC_XAXIS = 1,	// Roll
	RC_YAXIS,		// Pitch
	RC_ZAXIS		// Yaw
};

enum {
	MOTOR_FL = 0,
	MOTOR_FR,
	MOTOR_BL,
	MOTOR_BR
};

typedef struct master_t {
	// motor/esc/servo related stuff
	uint16_t ESCFrequency;				// ESC PWM frequency
	uint16_t MinESCPWM;					// Set the minimum throttle command sent to the ESC (Electronic Speed Controller). This is the minimum value that allow motors to run at a idle speed.
	uint16_t MaxESCPWM;					// This is the maximum value for the ESCs at full power this value can be increased up to 2000
	uint16_t MotorPWMRate;				// The update rate of motor outputs (50-498Hz)

	// Radio/ESC-related configuration
	uint16_t MinRCPWM;					// minimum rc end
	uint16_t MaxRCPWM;					// maximum rc end

	uint16_t MinThrottle;
	uint16_t MaxThrottle;
	uint16_t RCThrottleArmed;
	uint16_t RCThrottleDisarm;
	uint16_t MinRCThrottle;
	uint16_t MaxRCThrottle;

	uint16_t MinRoll;
	uint16_t MaxRoll;
	uint16_t MinPitch;
	uint16_t MaxPitch;
	uint16_t MinYaw;
	uint16_t MaxYaw;

} master_t;

#endif /* COPTER_HPP_ */
