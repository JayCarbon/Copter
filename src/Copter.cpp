#include "MotorI2C/MotorI2C.h"
#include "PCA9685/PCA9685.h"
#include "RcChannel/RcChannel.h"
#include "IMU.h"
#include "PID.h"
#include "Copter.h"

master_t masterCFG;
uint32_t times[300];

bool run = true;
int gpios[MAX_CHANNELS] = {17, 27, 22, 25};
const int lRange = 1070;
const int hRange = 1920;
IMU *imu;
PCA9685 *motorController;

uint32_t previousTime = 0;
uint32_t currentTime = 0;
uint32_t deltaTime = 0;
uint32_t frameCount = 0;

bool MotorsStartup = false;
bool MotorsArmed = false;
bool MotorsArming = false;

bool InFlight = false;

bool calibrating = true;
bool IMU_problem = false;
float max_X ;
float max_Y ;
float angles[3];
float rates[3];
float targetAngles[3];
float old_a;
float mdiff;


int count = 0;
#define PID_INCREMENT 0.01

RcChannel *rcChannels[MAX_CHANNELS];
MotorI2C *motors[MAX_MOTORS];
PID *pids[MAX_PIDS];

double	pid_inputs[MAX_PIDS];
double	pid_outputs[MAX_PIDS];
double	pid_setpoints[MAX_PIDS];

int main(void)
{
	printf("Initializing PIGPIO\n");
	if (gpioInitialise() < 0)
	{
		printf("FAILED\n");
		return EXIT_FAILURE;
	}

	printf("Setting master configuration\n");
	masterCFG.ESCFrequency = 400;
	masterCFG.MinESCPWM = 1000;
	masterCFG.MaxESCPWM = 2000;

	masterCFG.MinRCPWM = 1000;
	masterCFG.MaxRCPWM = 2000;

	masterCFG.RCThrottleArmed = 1250;
	masterCFG.RCThrottleDisarm = 1100;

	masterCFG.MinRCThrottle = 1100;
	masterCFG.MaxRCThrottle = 1950;

	masterCFG.MinThrottle = 1100;
	masterCFG.MaxThrottle = 1950;

	masterCFG.MinRoll = 1250;
	masterCFG.MaxRoll = 1900;

	masterCFG.MinPitch = 1250;
	masterCFG.MaxPitch = 1900;

	masterCFG.MinYaw = 1250;
	masterCFG.MaxYaw = 1900;

	// PID Configuration

	//PID(double* Input, double* Output, double* Setpoint,
    //double Kp, double Ki, double Kd, int ControllerDirection)

	pids[PID_ROLL_RATE] = new PID(
			&pid_inputs[PID_ROLL_RATE],
			&pid_outputs[PID_ROLL_RATE],
			&pid_setpoints[PID_ROLL_RATE],
			0.15, 0, 0, REVERSE);
	pids[PID_ROLL_RATE]->SetOutputLimits(-90, 90);
	pids[PID_ROLL_RATE]->SetMode(AUTOMATIC);

//	pids[PID_PITCH_RATE] = new PID(
//			&pid_inputs[PID_PITCH_RATE],
//			&pid_outputs[PID_PITCH_RATE],
//			&pid_setpoints[PID_PITCH_RATE],
//			0.1, 0, 0, REVERSE);
//	pids[PID_PITCH_RATE]->SetOutputLimits(-45, 45);
//	//pids[PID_PITCH_RATE]->SetMode(AUTOMATIC);
//
//	pids[PID_YAW_RATE] = new PID(
//			&pid_inputs[PID_YAW_RATE],
//			&pid_outputs[PID_YAW_RATE],
//			&pid_setpoints[PID_YAW_RATE],
//			0.1, 0, 0, REVERSE);
//	pids[PID_YAW_RATE]->SetOutputLimits(-45, 45);
//	//pids[PID_YAW_RATE]->SetMode(AUTOMATIC);
//
//	pids[PID_PITCH_STAB] = new PID(
//			&pid_inputs[PID_PITCH_STAB],
//			&pid_outputs[PID_PITCH_STAB],
//			&pid_setpoints[PID_PITCH_STAB],
//			4.5, 0, 0, REVERSE);
//	pids[PID_ROLL_STAB] = new PID(
//			&pid_inputs[PID_ROLL_STAB],
//			&pid_outputs[PID_ROLL_STAB],
//			&pid_setpoints[PID_ROLL_STAB],
//			4.5, 0, 0, REVERSE);
//	pids[PID_YAW_STAB] = new PID(
//			&pid_inputs[PID_YAW_STAB],
//			&pid_outputs[PID_YAW_STAB],
//			&pid_setpoints[PID_YAW_STAB],
//			10, 1, 0, REVERSE);

	printf("Initializing RC channels\n");
	for(int rc = 0; rc < MAX_CHANNELS; rc++)
		rcChannels[rc] = new RcChannel(rc, gpios[rc], lRange, hRange, NULL);

	printf("Initializing MotorController\n");
	motorController = new PCA9685();
	motorController->reset();
	motorController->setFrequency(masterCFG.ESCFrequency);

	printf("Initializing Motors\n");
	for(int mo = 0; mo < MAX_MOTORS; mo++)
		motors[mo] = new MotorI2C(mo, motorController);

	if(!MotorsStartup)
	{
		StartupMotors();
		MotorsStartup = true;
	}

	printf("Initializing IMU\n");
	imu = new IMU();
	imu->init();

	printf("Starting loop\n");
	while(run)
		loop();

	gpioTerminate();

	return EXIT_SUCCESS;
}

void loop()
{
	currentTime = gpioTick();
	//First tick will be huge since previous will be zero
	deltaTime = currentTime - previousTime;

	//200Hz task loop
	if(deltaTime >= 5000)
	{
		frameCount++;

		process200HzTasks();

		//100Hz task loop
		if(frameCount % TASK_100HZ == 0)
			process100HzTasks();

		//50Hz task loop
		if (frameCount % TASK_50HZ == 0)
		  process50HzTasks();

		//10Hz task loop
		if(frameCount % TASK_10HZ == 0)
			process10HzTasks();

		//5Hz task loop
		if(frameCount % TASK_5HZ == 0)
			process5HzTasks();

		//2Hz task loop
		if(frameCount % TASK_2HZ == 0)
			process2HzTasks();

		//1Hz task loop
		if(frameCount % TASK_1HZ == 0)
			process1HzTasks();


		previousTime = currentTime;
	}

	if(frameCount >= 200)
		frameCount = 0;
}

void process200HzTasks()
{
	if(frameCount >= 1)
		times[frameCount - 1] = deltaTime;

	if(imu->getAngles(angles))
	{
		pid_inputs[PID_ROLL_RATE] = angles[ROLL];
		pid_inputs[PID_PITCH_RATE] = angles[PITCH];
		pid_inputs[PID_YAW_RATE] = angles[YAW];
	}

	for(int pid = 0; pid < MAX_PIDS; pid++)
		if(pids[pid] != 0)
			pids[pid]->Compute(currentTime);

	processFlightControl();

}

void process100HzTasks()
{

}

void process50HzTasks()
{
	processPilotCommands();
}

void process10HzTasks()
{
	//printf("10hz\n");
}

void process5HzTasks()
{

}

void process2HzTasks()
{
#ifdef DEBUG

	printf("Flags: ");
	printf("MotorsArmed [%s] ", MotorsArmed ? "True":"False");
	printf("\n");
	//RC
	printf("RC*** thr[%i][%i] aile[%i][%i] elev[%i][%i] rudd[%i][%i]\n",
			rcChannels[RC_THROTTLE]->getDeltaPWM(),
			rcChannels[RC_THROTTLE]->getPercentage(),
			rcChannels[RC_ROLL]->getDeltaPWM(),
			rcChannels[RC_ROLL]->getPercentage(),
			rcChannels[RC_PITCH]->getDeltaPWM(),
			rcChannels[RC_PITCH]->getPercentage(),
			rcChannels[RC_YAW]->getDeltaPWM(),
			rcChannels[RC_YAW]->getPercentage());

	//MPU6050
	printf("MPU** angle ROLL [%f] PITCH [%f] YAW [%f]  rate[%f][%f][%f]\n", angles[0], angles[1], angles[2], rates[0], rates[1], rates[2]);

	printf("Pid setpoints: ");
	printf("PID_ROLL_RATE [%f] ", pid_setpoints[PID_ROLL_RATE]);
	printf("PID_PITCH_RATE [%f] ", pid_setpoints[PID_PITCH_RATE]);
	printf("PID_YAW_RATE [%f] ", pid_setpoints[PID_YAW_RATE]);
	printf("\n");

	printf("Pid inputs: ");
	printf("PID_ROLL_RATE [%f] ", pid_inputs[PID_ROLL_RATE]);
	printf("PID_PITCH_RATE [%f] ", pid_inputs[PID_PITCH_RATE]);
	printf("PID_YAW_RATE [%f] ", pid_inputs[PID_YAW_RATE]);
	printf("\n");

	printf("Pid outputs: ");
	printf("PID_ROLL_RATE [%f] ", pid_outputs[PID_ROLL_RATE]);
	printf("PID_PITCH_RATE [%f] ", pid_outputs[PID_PITCH_RATE]);
	printf("PID_YAW_RATE [%f] ", pid_outputs[PID_YAW_RATE]);
	printf("\n");

	printf("Motor 0[%i][%i] 1[%i][%i] 2[%i][%i] 3[%i][%i]\n",
			motors[0]->getSpeed(),
			motors[0]->getPercentage(),
			motors[1]->getSpeed(),
			motors[1]->getPercentage(),
			motors[2]->getSpeed(),
			motors[2]->getPercentage(),
			motors[3]->getSpeed(),
			motors[3]->getPercentage());

	printf("\nROLL PID [%g]\n", pids[PID_ROLL_RATE]->GetKp());

	printf("\n\n");

#endif

	updateRollPid();
}

void process1HzTasks()
{
//	printf("200Hz times: \n");
//	for(int i = 0; i < 200; i++)
//	{
//		printf("tick [%3i] time [%6zu] ", i, times[i]);
//
//		if(i % 5 == 0)
//		{
//			printf("\n");
//		}
//	}
//
//	printf("\n");
//
//	printf("PID samples time\n");
//
//	for(int i = 0; i < 200; i++)
//	{
//		printf("sample [%3i] calledTime [%6zu] now [%6zu] delta [%6zu] ",
//				i,
//				pids[PID_ROLL_RATE]->sampletimes[i][0],
//				pids[PID_ROLL_RATE]->sampletimes[i][1],
//				pids[PID_ROLL_RATE]->sampletimes[i][2]);
//
//		if(i % 2 == 0)
//		{
//			printf("\n");
//		}
//	}

}

void updateRollPid()
{
	if(rcChannels[RC_ROLL]->getPercentage() < 30)
		pids[PID_ROLL_RATE]->SetKp(pids[PID_ROLL_RATE]->GetKp()+PID_INCREMENT);

	if(rcChannels[RC_ROLL]->getPercentage() > 70)
		pids[PID_ROLL_RATE]->SetKp(pids[PID_ROLL_RATE]->GetKp()-PID_INCREMENT);
}

void StartupMotors()
{
	printf("Arming motors\n");

	motorController->reset();
	motorController->setFrequency(PWM_DEFAULT_FREQUENCY);

	printf("Setting throttle to zero\n");
	motorController->setSpeedAll(PWM_ZERO);

	//sleep for 5 second
	printf("Sleeping for 2 seconds\n");
	usleep(2000000);

	printf("Setting throttle to max\n");
	motorController->setSpeedAll(MOTOR_MAX);

	//sleep for 1 second
	printf("Sleeping for 1 second\n");
	usleep(1000000);

	printf("Setting throttle back to zero\n");
	motorController->setSpeedAll(MOTOR_STOP);

	printf("Waiting for 1 second.\n");
	usleep(1000000);
}
//50 15 90 -45 45
//35 * 90 / 75 + -45
long map(uint16_t x, uint16_t in_min, uint16_t in_max, int16_t out_min, int16_t out_max)
{
	long mapped = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	//printf("map: x[%d] in_min[%d] in_max[%d] out_min[%d] out_max[%d] result[%li]\n", x, in_min, in_max, out_min, out_max, mapped);

	return mapped;
}

void processPilotCommands()
{
	uint16_t rcThrottle = rcChannels[RC_THROTTLE]->getDeltaPWM();

	if(rcThrottle <= masterCFG.RCThrottleDisarm)
	{
		if(MotorsArmed)
		{
			for(int mo = 0; mo < MAX_MOTORS; mo++)
				motors[mo]->stop();

			//printf("DisArmed\n");
			MotorsArmed = false;
		}
	}

	if(rcThrottle >= masterCFG.RCThrottleArmed && !MotorsArmed){
		//printf("Armed\n");
		MotorsArmed = true;
	}

	InFlight = (MotorsArmed == true && rcThrottle > masterCFG.MinRCThrottle);

	if(InFlight)
	{
		//printf("InFlight\n");
		pid_setpoints[PID_ROLL_RATE] = 0;
		//pid_setpoints[PID_ROLL_RATE] = map(rcChannels[RC_ROLL]->getDeltaPWM(), masterCFG.MinRoll, masterCFG.MaxRoll, -45, 45);
		//pid_setpoints[PID_PITCH_RATE] = map(rcChannels[RC_PITCH]->getDeltaPWM(), masterCFG.MinPitch, masterCFG.MaxPitch, -45, 45);
		//pid_setpoints[PID_YAW_RATE] = map(rcChannels[RC_YAW]->getDeltaPWM(), masterCFG.MinYaw, masterCFG.MaxYaw, -45, 45);
	}
}

uint16_t constrain(uint16_t value, uint16_t min, uint16_t max)
{
	if(value < min)
		value = min;

	if(value > max)
		value = max;

	return value;
}

void processFlightControl()
{
	uint16_t throttle = map(rcChannels[RC_THROTTLE]->getDeltaPWM(), lRange, hRange, 1000, 2000);

	//if(throttle > masterCFG.MaxRCThrottle)
	//	throttle = masterCFG.MaxRCThrottle;

	//if(throttle < masterCFG.MinRCThrottle)
	//	throttle = masterCFG.MinRCThrottle;



	if(throttle >= masterCFG.MinRCThrottle && MotorsArmed)
	{
		//printf("Throttle is something\n");

		uint16_t fl = throttle - pid_outputs[PID_ROLL_RATE];// - pid_outputs[PID_PITCH_RATE];// - pid_outputs[PID_YAW_RATE];
		//uint16_t bl = throttle - pid_outputs[PID_ROLL_RATE];// + pid_outputs[PID_PITCH_RATE];// + pid_outputs[PID_YAW_RATE];
		uint16_t fr = throttle + pid_outputs[PID_ROLL_RATE];// - pid_outputs[PID_PITCH_RATE];// + pid_outputs[PID_YAW_RATE];
		//uint16_t br = throttle + pid_outputs[PID_ROLL_RATE];// + pid_outputs[PID_PITCH_RATE];// - pid_outputs[PID_YAW_RATE];

		//printf("bl [%i] br [%i]\n", bl, br);

		//motors[MOTOR_FL]->setSpeed(map(fl, lRange, hRange, 1000, 2000));
		//motors[MOTOR_FR]->setSpeed(map(fr, lRange, hRange, 1000, 2000));

		//if(throttle < fl)
		//	fl = throttle;
		if(fl < 0)
			fl = 0;

		//if(throttle < fr)
		//	fr = throttle;
		if(fr < 0)
			fr = 0;

		motors[MOTOR_BL]->setSpeed(fl);
		motors[MOTOR_BR]->setSpeed(fr);



		//motors[MOTOR_FL]->setSpeed(constrain(fl, masterCFG.MinThrottle, masterCFG.MaxThrottle));
		//motors[MOTOR_BL]->setSpeed(constrain(bl, masterCFG.MinThrottle, masterCFG.MaxThrottle));
		//motors[MOTOR_FR]->setSpeed(constrain(fr, masterCFG.MinThrottle, masterCFG.MaxThrottle));
		//motors[MOTOR_BR]->setSpeed(constrain(br, masterCFG.MinThrottle, masterCFG.MaxThrottle));
	}
}
