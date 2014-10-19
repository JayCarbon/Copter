CC = g++
CXXFLAGS = -c -Wall
CXXLIBS =-lpigpio -lpthread -lrt
CXXDEFS = -DDMP_FIFO_RATE=0 -DDEBUG #-DTRACE
OBJECTS = src/Copter.o src/Filter.o src/IMU.o src/PID.o \
src/I2Cdev/I2Cdev.o \
src/MotorI2C/MotorI2C.o \
src/MPU6050/MPU6050.o \
src/PCA9685/PCA9685.o \
src/RcChannel/RcChannel.o 
TARGET = copter

$(TARGET) : $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(CXXLIBS)

%.o: %.cpp
	$(CC) $(CXXDEFS) $(CXXFLAGS) $< -o $@

clean:
	rm -rf $(OBJECTS) $(TARGET)
