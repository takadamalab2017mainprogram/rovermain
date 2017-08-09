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
Waking gWakingState;
//WakingFromLie gWakingFromLieState;
//Turning gTurningState;
//Avoiding gAvoidingState;
//WadachiPredicting gPredictingState;
//PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
//MovementLogging gMovementLoggingState;
//EncoderMonitoring gEncoderMonitoringState;
//CameraSave_Sequence gCameraSave_Sequence;


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
	gWakingState.setRunMode(false);
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
   if (gWakingState.isActive())mLastUpdateTime = time;//�N���オ�蓮�쒆�͑ҋ@����
   if (Time::dt(time, mLastUpdateTime) > 2)//�N���オ�芮�����A���莞�Ԃ��o�߂��Ă�����
     {
       Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
       //�摜�B�e�������s��
       mCurStep = STEP_CAMERA;
       mLastUpdateTime = time;
       
       
       gMotorDrive.drive(0);
       //gCameraCapture.startWarming();
     }
   break;
   
 case STEP_CAMERA_TURN:
   //�摜�����̌��ʁA���]�����K�v���������ꍇ
	 /*
		if (Time::dt(time, mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		  {
		    //gCameraCapture.startWarming();
		    mCurStep = STEP_CAMERA_FORWARD;
		    gMotorDrive.drivePIDGyro(0, 100, true);
		    mLastUpdateTime = time;
		  }
		  */
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
		//�摜�����̌��ʁA���̏����]�����K�v���������ꍇ
		/*
		if (Time::dt(time, mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
		  //gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//�����_������
		if (Time::dt(time, mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if (mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//�����_���ڍs
				mEscapingTriedCount = 0;
				mCurStep = STEP_BACKWARD;
				break;
			}
			stuckMoveRandom();
			mLastUpdateTime = time;

		}
		*/
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
			//gSServo.start(0, 0);
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
	switch (mCurStep)
	{
		//case STEP_BACKWARD:
		//	//�o�b�N���s��
		//	if(Time::dt(time,mLastUpdateTime) >= 3)
		//	{
		//		mCurStep = STEP_TURN;
		//		mLastUpdateTime = time;
		//		gMotorDrive.drive(100,-100);
		//		gMultiServo.start(0);					//�X�^�r������
		//	}
		//	break;
	case STEP_TURN:
		//���̏����]���s��
		if (Time::dt(time, mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100);
			//gMultiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	case STEP_FORWARD:
		//�O�i���s��
		if (Time::dt(time, mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100, -100);
			//gMultiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	}
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
EscapingRandom::~EscapingRandom()
{
}

bool Waking::onInit(const struct timespec& time)
{
	mCurStep = STEP_CHECK_LIE;

	gMotorDrive.setRunMode(true);
	//gGyroSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	gMultiServo.start(BACK_STABI_RUN_ANGLE);
	gNineAxisSensor.setRunMode(true);
	//gSServo.setRunMode(true);
	//gSServo.moveRun();
	//gPoseDetecting.setRunMode(true);
	mWakeRetryCount = 0;
	mLastUpdateTime = time;
	gMotorDrive.drive(-100);

	return true;
}
void Waking::onClean()
{
	gMotorDrive.drive(0);
}
void Waking::onUpdate(const struct timespec& time)
{

	if (Time::dt(time, mLastUpdateTime) >= 2) {
		gMotorDrive.drive(0);
		gWakingState.setRunMode(false)
		return;
	}
	else {
		return;		
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
/*
bool WakingFromLie::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mLastDtTime = time;
	mCurStep = STEP_FORWARD;
	mCurrentPower = 0;
	mNotLieCount = 0;

	gMotorDrive.setRunMode(true);
	//gSServo.setRunMode(true);
	//gSServo.start(0, 0);
	//gPoseDetecting.setRunMode(true);
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

		if (mNotLieCount > 100 || mCurrentPower >= MOTOR_MAX_POWER)
		{
			gMotorDrive.drive(0, 0);
			mCurStep = STEP_VERIFY;
			mLastUpdateTime = time;
		}
		mCurrentPower += MOTOR_MAX_POWER * dt * (WAKING_RETRY_COUNT + 1 - mWakeRetryCount) / (WAKING_RETRY_COUNT + 1) / mShortestSpeedUpPeriod;

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

			if (mWakeRetryCount++ >= WAKING_RETRY_COUNT)
			{
				setRunMode(false);
				return;
			}
			Debug::print(LOG_SUMMARY, "WakingFromLie: Retrying (%d/%d)\r\n", mWakeRetryCount, WAKING_RETRY_COUNT);
			gMotorDrive.drive(MOTOR_MAX_POWER, MOTOR_MAX_POWER);
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
	//gSServo.moveRun();
}
WakingFromLie::WakingFromLie() : mShortestSpeedUpPeriod(10)
{
	setName("wakinglie");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
WakingFromLie::~WakingFromLie()
{
}
*/

/*
bool Turning::onInit(const struct timespec& time)
{
	mTurnPower = 0;
	//gGyroSensor.setRunMode(true);
	mAngle=gNineAxisSensor.getRz();//gGyroSensor.getRz();
	mLastUpdateTime = time;
	return true;
}
void Turning::onUpdate(const struct timespec& time)
{
	//double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	if (Time::dt(time, mLastUpdateTime) >= 5 || turnedAngle > 15)
	{
		Debug::print(LOG_SUMMARY, "Turning: Detected turning\r\n");
		gMotorDrive.drive(0);
		setRunMode(false);
	}
	else
	{
		if (mIsTurningLeft)gMotorDrive.drive(-mTurnPower, mTurnPower);
		else gMotorDrive.drive(mTurnPower, -mTurnPower);
		if (turnedAngle < 5)mTurnPower += 0.1;
	}
}
void Turning::setDirection(bool left)
{
	mIsTurningLeft = left;
}
Turning::Turning()
{
	setName("turning");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Turning::~Turning()
{
}
*/

/*
bool Avoiding::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	if (!gEscapingState.isActive())gMotorDrive.drive(0, 50);
	mAngle = gGyroSensor.getRz();
	mCurStep = STEP_TURN;
	return true;
}
void Avoiding::onUpdate(const struct timespec& time)
{
	if (gEscapingState.isActive())
	{
		Debug::print(LOG_SUMMARY, "Avoiding: Escaping is already running. Avoiding Canceled!\r\n");
		setRunMode(false);
	}
	switch (mCurStep)
	{
	case STEP_TURN:
		if (Time::dt(time, mLastUpdateTime) > 5 || abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle)) > 45)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: forwarding\r\n");
			mLastUpdateTime = time;
			gMotorDrive.drivePIDGyro(10, MOTOR_MAX_POWER, true);
			mCurStep = STEP_FORWARD;
		}
		break;
	case STEP_FORWARD:
		if (Time::dt(time, mLastUpdateTime) > 4)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: finished\r\n");
			setRunMode(false);
		}
		break;
	}
}
Avoiding::Avoiding()
{
	setName("avoiding");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Avoiding::~Avoiding()
{
}
*/

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
