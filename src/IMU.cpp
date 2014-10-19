#include "MPU6050/MPU6050_6Axis_MotionApps20.h"
#include "IMU.h"

IMU::IMU()
{  
}

void IMU::init()
{
	//reset
	printf("Reseting MPU. . .\n");
	accelgyro.reset();
	usleep(1000000); //sleep for 1 seconds

	// Initialize device
	printf("Initializing MPU devices . . .\n");
	accelgyro.initialize();

	// Check connection
	printf("Testing MPU connection...\n");
	printf(accelgyro.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

	usleep(1000000); // Wait for sensor to stabilize

	printf("Initializing DMP...\n");

	dmpReady = false;

	// make sure it worked (returns 0 if so)
	if (accelgyro.dmpInitialize() == 0) {

		//Set offset to zero Z axis
		accelgyro.setZGyroOffset(50);
		//Set offset to zero X axis
		accelgyro.setXGyroOffset(50); //currently pitch since x and y are reversed
		//Set offset to zero Y axis
		accelgyro.setYGyroOffset(50); //currently roll since x and y are reversed

		// turn on the DMP, now that it's ready
		printf("Enabling DMP...\n");
		accelgyro.setDMPEnabled(true);

		// enable Arduino interrupt detection
		//Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		//attachInterrupt(0, dmpDataReady, RISING);
		mpuIntStatus = accelgyro.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		printf("DMP ready!\n");
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = accelgyro.dmpGetFIFOPacketSize();
	} else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		printf("DMP Initialization failed (code %d)\n", devStatus);
	}

	printf("Testing DMP enabled\n");
	printf(accelgyro.getDMPEnabled() ? "DMP Enabled\n" : "DMP Disabled\n");

}

bool IMU::getAngles(float angles[])
{
	if (!dmpReady) return false;

	if(accelgyro.getIntDataReadyStatus())
	{
		// get current FIFO count
		fifoCount = accelgyro.getFIFOCount();

		if (fifoCount == 1024)
		{
		        // reset so we can continue cleanly
			accelgyro.resetFIFO();
			printf("FIFO overflow!\n");

			return false;
		}
		else if (fifoCount >= 42)
		{
			// orientation/motion vars
			Quaternion q;           // [w, x, y, z]         quaternion container
			VectorInt16 aa;         // [x, y, z]            accel sensor measurements
			VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
			VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
			VectorFloat gravity;    // [x, y, z]            gravity vector
			//float euler[3];         // [psi, theta, phi]    Euler angle container
			float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

			accelgyro.getFIFOBytes(fifoBuffer, packetSize);

			// display Euler angles in degrees
			accelgyro.dmpGetQuaternion(&q, fifoBuffer);
			accelgyro.dmpGetGravity(&gravity, &q);
			accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
			//accelgyro.dmpGetEuler(euler, &q);

			//angles[] = rpy
			//Due to the way we mounted it, pitch and roll are reversed
			angles[YAW] = ypr[0] * 180/M_PI;
			angles[ROLL] = ypr[1] * 180/M_PI;
			angles[PITCH] = ypr[2] * 180/M_PI;

			//printf("angles  %7.2f %7.2f %7.2f    \n", angles[0], angles[1], angles[2]);
		}
	}

	return true;
}

