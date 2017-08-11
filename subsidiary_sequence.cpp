#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdarg.h>
#include "subsidiary_sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"

Escaping gEscapingState;
EscapingRandom gEscapingRandomState;
EscapingByStabi gEscapingByStabiState;
SensorLogging gSensorLoggingState;


bool Escaping::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWARD;
	gMotorDrive.drive(-100);
	mEscapingTriedCount = 0;
	return true;
}
void Escaping::onClean()
{
	;
}
void Escaping::onUpdate(const struct timespec& time)
{
  const static unsigned int ESCAPING_MAX_CAMERA_ESCAPING_COUNT = 20;
  const static unsigned int ESCAPING_MAX_RANDOM_ESCAPING_COUNT = 20;
  switch (mCurStep)
    {
    case STEP_BACKWARD:
      //�o�b�N���s��
      if (Time::dt(time, mLastUpdateTime) >= 2)
	{
	  Debug::print(LOG_SUMMARY, "Escaping: Backward finished!\r\n");
	  mCurStep = STEP_AFTER_BACKWARD;
	  mLastUpdateTime = time;
	  gMotorDrive.drive(0);
	}
      break;
    case STEP_AFTER_BACKWARD:
      //�ċN���h�~�̂��ߑҋ@
      if (Time::dt(time, mLastUpdateTime) >= 3)
	{
	  if (mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
	    {
	      //�����_���ڍs
	      Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
	      mEscapingTriedCount = 0;
	      mCurStep = STEP_RANDOM;
	      mCurRandomStep = RANDOM_STEP_FORWARD;
	      break;
	    }
	  mLastUpdateTime = time;
	}
  break;
 case STEP_PRE_CAMERA:
   //�摜�B�e�p�ɋN���オ�蓮�����s���A���b�ҋ@����
   if (Time::dt(time, mLastUpdateTime) > 2)//�N���オ�芮�����A���莞�Ԃ��o�߂��Ă�����
     {
       Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
       //�摜�B�e�������s��
       mCurStep = STEP_CAMERA;
       mLastUpdateTime = time;
       
       
       gMotorDrive.drive(0);
     }
   break;
   
 case STEP_CAMERA_TURN:
		break;
 case STEP_CAMERA_FORWARD:
		//�摜�����̌��ʁA���i�����K�v���������ꍇ
   if (Time::dt(time, mLastUpdateTime) >= 10)
     {
			gMotorDrive.drive(-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		break;
	}
}
void Escaping::stuckMoveRandom()
{
	switch (mCurRandomStep)
	{
	case RANDOM_STEP_BACKWARD:
		//�o�b�N���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100, -100);
		break;
	case RANDOM_STEP_TURN:
		//���̏����]���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100);
		break;
	case RANDOM_STEP_FORWARD:
		//�O�i���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): forward\r\n");
		mCurRandomStep = RANDOM_STEP_BACKWARD;
		gMotorDrive.drive(-100);
		break;
	}
}
Escaping::Escaping()
{
	setName("escaping");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Escaping::~Escaping()
{
}

bool EscapingByStabi::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Escaping By Stabi Start! ");
	Time::showNowTime();

	mLastUpdateTime = time;
	gMultiServo.setRunMode(true);
	mFlag = false;
	mTryCount = 0;
	return true;
}
void EscapingByStabi::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < 1) return;

	mLastUpdateTime = time;

	if (!mFlag)
	{
		gMotorDrive.drive(30);
		gMultiServo.start(0);
	}
	else
	{
		gMotorDrive.drive(100);
		gMultiServo.start(1);
		mTryCount++;

		if (mTryCount % 5 == 0)
		{
			Debug::print(LOG_SUMMARY, "Escaping TryCount: %d\r\n", mTryCount);
		}
		if (mTryCount > Escaping_Chance_limit)
		{
			gEscapingByStabiState.setRunMode(false);
			gEscapingRandomState.setRunMode(true);
		}
	}
	mFlag = !mFlag;
}
bool EscapingByStabi::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("start") == 0)
		{
			gEscapingByStabiState.setRunMode(true);
			return true;
		}
		if (args[1].compare("s") == 0)
		{
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "Escaping By Stabi Finished!\r\n");
			gEscapingByStabiState.setRunMode(false);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "esc start       : start Escaping by stabi mode\r\n\esc s        : stop  Escaping by stabi mode\r\n");
	return false;
}
unsigned int EscapingByStabi::getTryCount()
{
	return mTryCount;
}
EscapingByStabi::EscapingByStabi()
{
	setName("esc1");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
EscapingByStabi::~EscapingByStabi()
{
}

bool EscapingRandom::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Start Escaping Random... ");
	Time::showNowTime();
	gMultiServo.setRunMode(true);
	mLastUpdateTime = time;
	mCurStep = STEP_TURN;
	gMotorDrive.drive(100, -100);
	return true;
}
void EscapingRandom::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) >= 5){
		mLastUpdateTime = time;

		//出力を選ぶ
		int stabiswitch = rand() % 2;//スタビ
		if (RandomCount < 2){
			//0,1回目は左右一斉運動
			int motordirection = pow(-1, rand() % 2);
			int motorforce0 = rand() % 100 * motordirection;
			int motorforce1 = motorforce0;
		}
		else if (RandomCount < 5) {
			//2,3,4回目はタイヤ左右別々方向出力
			int motordirection0 = pow(-1, rand() % 2);
			int motordirection1 = motordirection0 * (-1);
			int motorforce0 = rand() % 100 * motordirection0;
			int motorforce1 = rand() % 100 * motordirection1;
		}

		//出力を挙げる
		gMultiServo.start(stabiswitch);
		gMotorDrive.drive(motorforce0, motorforce1);

		Debug::print(LOG_SUMMARY, "Escaping Random turn %d, choiced by stabi %d, motor0 %d, motor1 %d \r\n", 
			RandomCount, stabiswitch, motorforce0, motorforce1);
		RandomCount++;//カウント	
	}

	if (RandomCount > 4) {
		gEscapingRandomState.setRunMode(false);
		gEscapingByStabiState.setRunMode(true);
	}

	//switch (mCurStep)
	//{
	//case STEP_TURN:
	//	//���̏����]���s��
	//	if (Time::dt(time, mLastUpdateTime) >= 5)
	//	{
	//		mCurStep = STEP_FORWARD;
	//		mLastUpdateTime = time;
	//		gMotorDrive.drive(100);
	//	}
	//	break;
	//case STEP_FORWARD:
	//	//�O�i���s��
	//	if (Time::dt(time, mLastUpdateTime) >= 3)
	//	{
	//		mCurStep = STEP_TURN;
	//		mLastUpdateTime = time;
	//		gMotorDrive.drive(100, -100);
	//	}
	//	break;
	//}
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
EscapingRandom::~EscapingRandom()
{
}





bool SensorLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameGPS, "Log started\r\n");
	write(mFilenamePressure, "Log started\r\n");
	write(mFilenameNineAxis, "Log started\r\n");
	write(mFilenameNineAxis, "time/sec,Ax/G,Ay/G,Az/G,Rvx/deg/sec,Rvy/deg/sec,Rvz/deg/sec,Rx/deg,Ry/deg,Rz/deg,Mx/uT,My/uT,Mz/uT");

	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
    gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;
	return true;
}
void SensorLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) >= 0.1)
	{
		mLastUpdateTime = time;

		//���O���ۑ�
		VECTOR3 vec;
		gGPSSensor.get(vec);
		if (gGPSSensor.isActive())write(mFilenameGPS, "%ld.%ld,%f,%f,%f\r\n", time.tv_sec,time.tv_nsec,vec.x, vec.y, vec.z);
		else write(mFilenameGPS, "unavailable\r\n");

    if (gPressureSensor.isActive())write(mFilenamePressure, "%ld.%ld,%f\r\n", time.tv_sec,time.tv_nsec,gPressureSensor.get());
		else write(mFilenamePressure, "unavailable\r\n");


		if (gNineAxisSensor.isActive())write(mFilenameNineAxis, "%ld.%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",time.tv_sec,time.tv_nsec, gNineAxisSensor.getAx(), gNineAxisSensor.getAy(), gNineAxisSensor.getAz(),gNineAxisSensor.getRvx(),gNineAxisSensor.getRvy(),gNineAxisSensor.getRvz(),gNineAxisSensor.getRx(),gNineAxisSensor.getRy(),gNineAxisSensor.getRz(),gNineAxisSensor.getMx(),gNineAxisSensor.getMy(),gNineAxisSensor.getMz());
		else write(mFilenameNineAxis, "unavailable\r\n");
	}
}
void SensorLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
SensorLogging::SensorLogging() : mLastUpdateTime()
{
	setName("sensorlogging");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	Filename("log_gps", ".txt").get(mFilenameGPS);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameGPS.c_str());
	Filename("log_pressure", ".txt").get(mFilenamePressure);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenamePressure.c_str());
	Filename("log_nineaxis", ".txt").get(mFilenameNineAxis);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameNineAxis.c_str());
}
SensorLogging::~SensorLogging()
{
}
