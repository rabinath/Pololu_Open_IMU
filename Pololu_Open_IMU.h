#ifndef IMU_h
#define IMU_h

//#define DEBUG_IMU

/*
Basically just a class made out of the Arduino Sketch Pololu_Open_IMU by mikeshub
 See: https://github.com/mikeshub/Pololu_Open_IMU/blob/master/Pololu_Open_IMU/Pololu_Open_IMU.ino

Based on the Madgwick algorithm found at:
 See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
 
 This code inherits all relevant liscenses and may be freely modified and redistributed.
 The MinIMU v1 has a roughly +/- 10degree accuracy
 The MinIMU v2 has a roughly +/- 1 degree accuracy
 */
#include <LSM303.h>
#include <L3G.h>

#define ToRad(x) ((x) * 0.01745329252)  // *pi/180
#define ToDeg(x) ((x) * 57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f
#define GYRO_SCALE 0.07f
#define betaDef		0.08f

//To find the calibration values us the sketch included with the LSM303 driver from pololu
/*Change line 11 from
 
 compass.enableDefault();
 
 to 
 
 compass.writeMagReg(LSM303_CRA_REG_M, 0x1C);
 compass.writeMagReg(LSM303_CRB_REG_M, 0x60);
 compass.writeMagReg(LSM303_MR_REG_M, 0x00);  
 
 Then put the calibration values below
 
 */
#define compassXMax 169.0f
#define compassXMin -489.0f
#define compassYMax 321.0f
#define compassYMin -324.0f
#define compassZMax 781.0f
#define compassZMin -321.0f

#define inverseXRange (float)(2.0 / (compassXMax - compassXMin))
#define inverseYRange (float)(2.0 / (compassYMax - compassYMin))
#define inverseZRange (float)(2.0 / (compassZMax - compassZMin))

class IMU {
public:
	float pitch,roll,yaw;
	
	IMU(L3G *gyro, LSM303 *compass) : gyro(gyro), compass(compass) {};
	virtual ~IMU(){};
	
	void setup();
	void loop();
	int getHeading();
	void getGyroscope(float *x, float *y, float *z);
	void getAcceleration(float *x, float *y, float *z);

private:
	void IMUinit();
	void IMUupdate(float *dt);
	void AHRSupdate(float *dt);
	void GetEuler(void);
	float fastAtan2( float y, float x);
	float invSqrt(float number);
	void Smoothing(float *raw, float *smooth);
	
	L3G *gyro;
	LSM303 *compass;

	long timer, printTimer;
	float G_Dt;
	int loopCount;

	float q0;
	float q1;
	float q2;
	float q3;
	float beta;

	float magnitude;

	float gyroSumX,gyroSumY,gyroSumZ;
	float offSetX,offSetY,offSetZ;

	float floatMagX,floatMagY,floatMagZ;
	float smoothAccX,smoothAccY,smoothAccZ;
	float accToFilterX,accToFilterY,accToFilterZ;

	int i;
};

#endif
