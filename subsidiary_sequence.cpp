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
#include <time.h>
#include "constants.cpp"
#include "pose_detector.h"

EscapingRandom gEscapingRandomState;
EscapingByStabi gEscapingByStabiState;
SensorLogging gSensorLoggingState;
//Waking gWakingState;
WakingFromLie gWakingFromLie;

bool EscapingByStabi::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Escaping By Stabi Start! ");
	Time::showNowTime();

	mLastUpdateTime = time;
	gMultiServo.setRunMode(true);
	mFlag = false;
	mTryCount = 0;
	Escaping_Chance_limit = 10;
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
	setName("esc");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
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
	//gMotorDrive.drive(100, -100);
	RandomCount = 0;
	return true;
}
void EscapingRandom::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) >= 5){
		mLastUpdateTime = time;

		//出力を選ぶ
		int stabiswitch = rand() % 2;//スタビ
		if (RandomCount < 5){
			//0-4回目は左右一斉運動
			motorforce0 = ((rand() % 50)+50) * pow(-1, rand() % 2);
			motorforce1 = motorforce0;
		}
		else{
			//5-9回目はタイヤ左右別々方向出力
			motorforce0 = rand() % 100 * pow(-1, rand() % 2);
			motorforce1 = rand() % 100 * pow(-1, rand() % 2);
		}

		//出力を上げる
		gMultiServo.start(stabiswitch);
		gMotorDrive.drive(motorforce0, motorforce1);

		Debug::print(LOG_SUMMARY, "Escaping Random turn %d, choiced by stabi %d, motor0 %d, motor1 %d \r\n", 
			RandomCount, stabiswitch, motorforce0, motorforce1);
		RandomCount++;//カウント	
	}

	if (RandomCount > 10) {
		gEscapingRandomState.setRunMode(false);
		gEscapingByStabiState.setRunMode(true);
	}


}
bool EscapingRandom::onCommand(const std::vector<std::string>& args)
{
  if (args.size() == 2)
  {
    if (args[1].compare("start") == 0)
    {
      gEscapingRandomState.setRunMode(true);
      return true;
    }
    if (args[1].compare("s") == 0)
    {
      gMotorDrive.drive(0);
      Debug::print(LOG_SUMMARY, "Escaping Random Finished!\r\n");
      gEscapingRandomState.setRunMode(false);
      return true;
    }
  }
  Debug::print(LOG_SUMMARY, "random start       : start Escaping by stabi mode\r\n\random s        : stop  Escaping by stabi mode\r\n");
  return false;
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
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
	setPriority(UINT_MAX, Constants::TASK_INTERVAL_SEQUENCE);

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
/*
bool Waking::onInit(const struct timespec& time)
{
	mCurStep = STEP_CHECK_LIE;

	gMotorDrive.setRunMode(true);
	gMultiServo.setRunMode(true);
	gMultiServo.start(BACK_STABI_RUN_ANGLE);
	gNineAxisSensor.setRunMode(true);
	mWakeRetryCount = 0;

	return true;
}
void Waking::onClean()
{
	gMotorDrive.drive(0);
}
void Waking::onUpdate(const struct timespec& time)
{
	double power;
	const static double WAKING_THRESHOLD = 200;

	switch (mCurStep)
	{
	case STEP_CHECK_LIE:
		Debug::print(LOG_SUMMARY, "in STEP CHECK LIE!\r\n");
		if (gPoseDetecting.isLie())
		{
		Debug::print(LOG_SUMMARY, "LieAngle= %f, LieThreshold= %f!\r\n",gPoseDetecting.getLieAngle(),gPoseDetecting.getLieThreshold());
		Debug::print(LOG_SUMMARY, "accel = (%f, %f, %f)\r\n", gNineAxisSensor.getAx(), gNineAxisSensor.getAy(),gNineAxisSensor.getAz());
		gWakingFromLie.setRunMode(true);
		mCurStep = STEP_WAIT_LIE;
		return;
		}
		
		//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);//角度調節
		gMultiServo.start(BACK_STABI_RUN_ANGLE);
		
		//gArmServo.start(ARM_FOLD_ANGLE);
		//gNeckServo.start(NECK_FOLD_ANGLE);
		// Do following case without breaking
	case STEP_WAIT_LIE:
		//if (gWakingFromLieState.isActive())return;
		//begin waking
		gMotorDrive.drive(mStartPower);		
		mAngleOnBegin = gNineAxisSensor.getRz(); //gGyroSensor.getRz();
		Debug::print(LOG_SUMMARY, "in STEP WAIT LIE\r\n");
		Debug::print(LOG_SUMMARY, "accel = (%f, %f, %f)\r\n", gNineAxisSensor.getAx(), gNineAxisSensor.getAy(), gNineAxisSensor.getAz());
		mLastUpdateTime = time;
		mCurStep = STEP_START;
		break;
	case STEP_STOP:
		if (Time::dt(time, mLastUpdateTime) > 2)
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
			gMotorDrive.drive(0);
		}
		//if (gAccelerationSensor.getPhi() < mAngleThreshold)	
		if (gNineAxisSensor.getPhi() < mAngleThreshold)	
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);

		//出力を選ぶ
		int stabiswitch = rand() % 2;//スタビ
		if (RandomCount < 5){
			//0-4回目は左右一斉運動
			motorforce0 = ((rand() % 50)+50) * pow(-1, rand() % 2);
			motorforce1 = motorforce0;
		}

		//power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
		//gMotorDrive.drive(power);
		break;

		double dt;
	case STEP_START:
		if (Time::dt(time, mLastUpdateTime) > 0.5)
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		//if (abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)
		if (abs(gNineAxisSensor.getRvx()) > WAKING_THRESHOLD)
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_DEACCELERATE;
		}

	case STEP_DEACCELERATE:	
		dt = Time::dt(time, mLastUpdateTime);
		if (dt > mDeaccelerateDuration)
		{
			Debug::print(LOG_SUMMARY, "Waking Deaccelerate finished!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		else
		{
			int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (mStartPower / 2)), 0);
			gMotorDrive.drive(tmp_power);
		}
		break;

	case STEP_VERIFY:
		//2.5秒置き　姿勢をチェックする
		if (Time::dt(time, mLastUpdateTime) <= 2.5)	return;
		
		Debug::print(LOG_SUMMARY, "in STEP VERIFY !\r\n");
		
		if (!gPoseDetecting.isFlip())
		{
		Debug::print(LOG_SUMMARY, "FlipAngle= %f , FlipThreshold= %f !\r\n", abs(gPoseDetecting.getFlipAngle()), gPoseDetecting.getmFlipThreshold());
		Debug::print(LOG_SUMMARY, "accel = (%f, %f, %f)\r\n", gNineAxisSensor.getAx(), gNineAxisSensor.getAy(), gNineAxisSensor.getAz());
		Debug::print(LOG_SUMMARY, "Waking Successed!\r\n");
		gBuzzer.start(30, 20, 4);
		setRunMode(false);
		gMultiServo.start(BACK_STABI_RUN_ANGLE); // 角度調節

		}
		else
		{
		mLastUpdateTime = time;
		mCurStep = STEP_START;
		mAngleOnBegin = gNineAxisSensor.getRvx();//gGyroSensor.getRvx();
		power = std::min((unsigned int)100, mStartPower + ((mWakeRetryCount + 1) * 5));	
		gMotorDrive.drive(power);

		if (++mWakeRetryCount > WAKING_RETRY_COUNT)
		{
		Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
		setRunMode(false);
		return;
		}
		gMultiServo.start(BACK_STABI_RUN_ANGLE); // 角度調節
		Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d) by power %f\r\n", mWakeRetryCount, WAKING_RETRY_COUNT, power);
		}
		break;
	}
}
bool Waking::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 4)
	{
		if (args[1].compare("set") == 0)
		{
			if (args[2].compare("power") == 0)//mStartPower
			{
				setPower(atoi(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if (args[2].compare("angle") == 0)//mAngleThreshold
			{
				setAngle(atof(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if (args[2].compare("d_time") == 0)//mDeaccelerateDuration
			{
				mDeaccelerateDuration = atof(args[3].c_str());
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
		}
	}
	else if (args.size() == 2)
	{
		if (args[1].compare("show") == 0)
		{
			Debug::print(LOG_SUMMARY, "mStartPower: %d\r\n", mStartPower);
			Debug::print(LOG_SUMMARY, "mAngleThreshold: %f\r\n", mAngleThreshold);
			Debug::print(LOG_SUMMARY, "mDeaccelerateDuration: %f\r\n", mDeaccelerateDuration);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "waking set power [1-100]: set start motor power\r\n\
							  waking set angle [0-180]: set AngleThreshold\r\n\
							waking set d_time [time]: set mDeaccelerateDuration\r\n\
						waking show             : show parameters\r\n");
	return true;
}
void Waking::setPower(int p)
{
	if (p >= MOTOR_MAX_POWER)
	{
		mStartPower = MOTOR_MAX_POWER;
		return;
	}
	else if (p < 1)
	{
		mStartPower = 1;
		return;
	}
	mStartPower = p;
}
void Waking::setAngle(double a)
{
	if (a >= 180)
	{
		mAngleThreshold = 180;
	}
	else if (a < 0)
	{
		mAngleThreshold = 0;
	}
	mAngleThreshold = a;
}
Waking::Waking() : mWakeRetryCount(0), mStartPower(45), mAngleThreshold(70), mDeaccelerateDuration(0.5)
{
	setName("waking");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
{
}
*/

bool WakingFromLie::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mLastDtTime = time;
	mCurStep = STEP_FORWARD;
	mCurrentPower = 0;
	mNotLieCount = 0;
	gMotorDrive.setRunMode(true);
	gPoseDetecting.setRunMode(true);
	mWakeRetryCount = 0;

	return true;
}
void WakingFromLie::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastDtTime);
	mLastDtTime = time;

	switch (mCurStep)
	{
	case STEP_FORWARD:
		


		if (!gPoseDetecting.isLie())mNotLieCount++;
		else mNotLieCount = 0;

		gMotorDrive.drive(mCurrentPower, mCurrentPower);

		if (mNotLieCount > 100 || mCurrentPower >= Constants::MOTOR_MAX_POWER)
		{
			gMotorDrive.drive(0, 0);
			mCurStep = STEP_VERIFY;
			mLastUpdateTime = time;
		}
		//段々モータの速さを早くする
		mCurrentPower += Constants::MOTOR_MAX_POWER * dt * (Constants::WAKING_RETRY_COUNT + 1 - mWakeRetryCount) / (Constants::WAKING_RETRY_COUNT + 1) / mShortestSpeedUpPeriod;

		break;
	case STEP_VERIFY:
		if (Time::dt(time, mLastUpdateTime) > 3)
		{
			if (!gPoseDetecting.isLie())
			{
				Debug::print(LOG_SUMMARY, "WakingFromLie: Successed waking!!\r\n");
				gBuzzer.start(30, 20, 4);
				setRunMode(false);
				return;
			}

			if (mWakeRetryCount++ >= Constants::WAKING_RETRY_COUNT)
			{
				setRunMode(false);
				return;
			}
			Debug::print(LOG_SUMMARY, "WakingFromLie: Retrying (%d/%d)\r\n", mWakeRetryCount, Constants::WAKING_RETRY_COUNT);
			gMotorDrive.drive(Constants::MOTOR_MAX_POWER, Constants::MOTOR_MAX_POWER);
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			mCurrentPower = 0;
		}
		break;
	}
}
bool WakingFromLie::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		mShortestSpeedUpPeriod = atof(args[1].c_str());
		Debug::print(LOG_SUMMARY, "Command executed!\r\n");
		return true;
	}
	Debug::print(LOG_SUMMARY, "waking [period]: set speed up period\r\n");
	return true;
}
void WakingFromLie::onClean()
{
	gMotorDrive.drive(0);
}
WakingFromLie::WakingFromLie() : mShortestSpeedUpPeriod(10)
{
	setName("wakinglie");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
}
WakingFromLie::~WakingFromLie()

{
}
