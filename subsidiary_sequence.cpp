#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <stdarg.h>
#include "subsidiary_sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "pose_detector.h"
#include "actuator.h"
#include "motor.h"
#include "image_proc.h"
#include "constants.h"
//Escaping gEscapingState;
//EscapingRandom gEscapingRandomState;
//EscapingByStabi gEscapingByStabiState;
Waking gWakingState;
//Turning gTurningState;
//Avoiding gAvoidingState;
//WadachiPredicting gPredictingState;
//PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
MovementLogging gMovementLoggingState;
EncoderMonitoring gEncoderMonitoringState;
StatusSending gStatusSending;

//bool WadachiPredicting::onInit(const struct timespec& time)
//{
//	mLastUpdateTime = time;
//	gCameraCapture.startWarming();
//
//	return true;
//}
//void WadachiPredicting::onUpdate(const struct timespec& time)
//{
//	if(gAvoidingState.isActive())return;
//	if(!mIsAvoidingEnable)
//	{
//		if(Time::dt(time,mLastUpdateTime) >= 2.5)
//		{
//			mLastUpdateTime = time;
//			IplImage* pImage = gCameraCapture.getFrame();
//			gCameraCapture.save(NULL,pImage);
//			if(!gImageProc.isWadachiExist(pImage))return;
//			//?Q???O???m????
//			gCameraCapture.startWarming();
//		}
//		return;
//	}
//
//	switch(mCurStep)
//	{
//	case STEP_RUNNING:
//		if(Time::dt(time,mLastUpdateTime) > 60)
//		{
//			Debug::print(LOG_SUMMARY, "Predicting: Stoping started\r\n");
//			mCurStep = STEP_STOPPING;
//			mLastUpdateTime = time;
//			gMotorDrive.drive(0);
//		}
//		break;
//	case STEP_STOPPING:
//		if(Time::dt(time,mLastUpdateTime) > 3)
//		{
//			Debug::print(LOG_SUMMARY, "Predicting: Waking started\r\n");
//			mCurStep = STEP_WAKING;
//			mLastUpdateTime = time;
//			gWakingState.setRunMode(true);
//		}
//		break;
//	case STEP_WAKING:
//		if(!gWakingState.isActive())
//		{
//			Debug::print(LOG_SUMMARY, "Predicting: Checking started\r\n");
//			mCurStep = STEP_CHECKING;
//			mLastUpdateTime = time;
//			gCameraCapture.startWarming();
//		}
//		break;
//	case STEP_CHECKING:
//		if(Time::dt(time,mLastUpdateTime) > 3)
//		{
//			Debug::print(LOG_SUMMARY, "Predicting: Avoiding started\r\n");
//			mLastUpdateTime = time;
//			IplImage* pImage = gCameraCapture.getFrame();
//			gCameraCapture.save(NULL,pImage);
//			if(gImageProc.isWadachiExist(pImage))
//			{
//				//?Q???O???m????
//				gAvoidingState.setRunMode(true);
//				mCurStep = STEP_AVOIDING;
//			}else
//			{
//				mCurStep = STEP_RUNNING;
//				gMotorDrive.startPID(0,MOTOR_MAX_POWER);
//			}
//		}
//		break;
//	case STEP_AVOIDING:
//		if(!gAvoidingState.isActive())
//		{
//			Debug::print(LOG_SUMMARY, "Predicting: Avoiding finished\r\n");
//			mCurStep = STEP_RUNNING;
//			mLastUpdateTime = time;
//		}
//		break;
//	}
//}
//bool WadachiPredicting::onCommand(const std::vector<std::string>& args)
//{
//	if(args.size() == 2)
//	{
//		if(args[1].compare("enable") == 0)
//		{
//			mIsAvoidingEnable = true;
//			return true;
//		}
//		if(args[1].compare("disable") == 0)
//		{
//			mIsAvoidingEnable = false;
//			return true;
//		}
//	}
//	Debug::print(LOG_SUMMARY, "predicting [enable/disable]  : switch avoiding mode\r\n");
//	return false;
//}
//bool WadachiPredicting::isWorking(const struct timespec& time)
//{
//	return mIsAvoidingEnable && (mCurStep != STEP_RUNNING || (mCurStep == STEP_RUNNING && Time::dt(time,mLastUpdateTime) < 6));
//}
//WadachiPredicting::WadachiPredicting() : mIsAvoidingEnable(false),mCurStep(STEP_RUNNING)
//{
//	setName("predicting");
//	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
//}
//WadachiPredicting::~WadachiPredicting()
//{
//}
/*
bool Escaping::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWARD;
	gMotorDrive.drive(-100);
	gCameraCapture.setRunMode(true);
	gGyroSensor.setRunMode(true);
	mEscapingTriedCount = 0;
	return true;
}
void Escaping::onClean()
{
	gWakingState.setRunMode(false);
	gTurningState.setRunMode(false);
}
void Escaping::onUpdate(const struct timespec& time)
{
	const static unsigned int ESCAPING_MAX_CAMERA_ESCAPING_COUNT = 20;
	const static unsigned int ESCAPING_MAX_RANDOM_ESCAPING_COUNT = 20;
	switch(mCurStep)
	{
	case STEP_BACKWARD:
		//?o?b?N??s??
		if(Time::dt(time,mLastUpdateTime) >= 2)
		{
			Debug::print(LOG_SUMMARY, "Escaping: Backward finished!\r\n");
			mCurStep = STEP_AFTER_BACKWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_AFTER_BACKWARD:
		//??N???h?~??????@
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			if(mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
			{
				//?????_????s
				Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
				mEscapingTriedCount = 0;
				mCurStep = STEP_RANDOM;
				mCurRandomStep = RANDOM_STEP_FORWARD;
				break;
			}
			mCurStep = STEP_PRE_CAMERA;
			mLastUpdateTime = time;
			//?N???オ?蓮???s??
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isSky(pImage))gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_CAMERA:
		//???B?e?p??N???オ?蓮???s???A???b??@????
		if(gWakingState.isActive())mLastUpdateTime = time;//?N???オ?蓮?????@????
		if(Time::dt(time,mLastUpdateTime) > 2)//?N???オ??????A??莞????o??????????
		{
			Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
			//???B?e?????s??
			mCurStep = STEP_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CAMERA:
		//????????s???A?????s????????
		if(Time::dt(time,mLastUpdateTime) >= 2)
		{
			Debug::print(LOG_SUMMARY, "Escaping: taking picture!\r\n");
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			stuckMoveCamera(pImage);
			gCameraCapture.save(NULL,pImage);
			mAngle = gGyroSensor.getRz();
			++mEscapingTriedCount;
		}
		break;
	case STEP_CAMERA_TURN:
		//???????????A??]????K?v??????????
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_CAMERA_FORWARD;
			gMotorDrive.startPID(0,100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_FORWARD:
		//???????????A???i????K?v??????????
		if(Time::dt(time,mLastUpdateTime) >= 10)
		{
			gMotorDrive.drive(-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		//???????????A??????]????K?v??????????
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//?????_??????
		if(Time::dt(time,mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if(mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//?????_????s
				mEscapingTriedCount = 0;
				mCurStep = STEP_BACKWARD;
				break;
			}
			stuckMoveRandom();
			mLastUpdateTime = time;

		}
		break;
	}
}
void Escaping::stuckMoveRandom()
{
	switch(mCurRandomStep)
	{
	case RANDOM_STEP_BACKWARD:
		//?o?b?N??s??
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100,-100);
		break;
	case RANDOM_STEP_TURN:
		//??????]??s??
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100);
		break;
	case RANDOM_STEP_FORWARD:
		//?O?i??s??
		Debug::print(LOG_SUMMARY, "Escaping(random): forward\r\n");
		mCurRandomStep = RANDOM_STEP_BACKWARD;
		gMotorDrive.drive(-100);
		break;
	}
}
void Escaping::stuckMoveCamera(IplImage* pImage)
{
	switch(gImageProc.wadachiExiting(pImage)){
		case -1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Left\r\n");
			gMotorDrive.drive(-100,100);
			mCurStep = STEP_CAMERA_TURN;
			break;
		case 1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Right\r\n");
			gMotorDrive.drive(100,-100);
			mCurStep = STEP_CAMERA_TURN;
			break;
		case 0:
            Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn here\r\n");
			gTurningState.setRunMode(true);
			gTurningState.setDirection(true);
			mCurStep = STEP_CAMERA_TURN_HERE;
			break;
		default://?J?????g?????????
			mCurStep = STEP_RANDOM;
			mCurRandomStep = RANDOM_STEP_FORWARD;
			break;
		
	}
}
Escaping::Escaping()
{
	setName("escaping");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Escaping::~Escaping()
{
}

bool EscapingByStabi::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Escaping By Stabi Start! ");
	Time::showNowTime();
	
	mLastUpdateTime = time;
	gStabiServo.setRunMode(true);
	//gMotorDrive.drive(20);
	mFlag = false;
	mTryCount = 0;
	return true;
}
void EscapingByStabi::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) < 1) return;

	mLastUpdateTime = time;

	if(!mFlag)
	{
		gMotorDrive.drive(0);
		gStabiServo.start(mAngle);
	}
	else
	{
		gMotorDrive.drive(100);
		gStabiServo.start(STABI_BASE_ANGLE);
		mTryCount++;
		
		if(mTryCount % 5 == 0)
		{
			Debug::print(LOG_SUMMARY, "Escaping TryCount: %d\r\n", mTryCount);
		}
	}
	mFlag = !mFlag;
}
bool EscapingByStabi::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("start") == 0)
		{
			gEscapingByStabiState.setRunMode(true);
			return true;
		}
		if(args[1].compare("stop") == 0)
		{
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "Escaping By Stabi Finished!\r\n");
			gEscapingByStabiState.setRunMode(false);
			return true;
		}
	}
	else if(args.size() == 3)
	{
		if(args[1].compare("angle") == 0)
		{
			mAngle = atof(args[2].c_str());
			Debug::print(LOG_SUMMARY, "angle: %f\r\n", mAngle);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "escapingbystabi start       : start Escaping by stabi mode\r\n\
escapingbystabi stop        : stop  Escaping by stabi mode\r\n\
escapingbystabi angle [0-1] : set stabi angle\r\n");
	return false;
}
unsigned int EscapingByStabi::getTryCount()
{
	return mTryCount;
}
EscapingByStabi::EscapingByStabi()
{
	setName("escapingbystabi");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
EscapingByStabi::~EscapingByStabi()
{
}

bool EscapingRandom::onInit(const struct timespec& time)
{
		Debug::print(LOG_SUMMARY, "Start Escaping Random... ");
	Time::showNowTime();
	gStabiServo.setRunMode(true);
	mLastUpdateTime = time;
	mCurStep = STEP_TURN;
	gMotorDrive.drive(100,-100);
	return true;
}
void EscapingRandom::onUpdate(const struct timespec& time)
{
	switch(mCurStep)
	{
	//case STEP_BACKWARD:
	//	//?o?b?N??s??
	//	if(Time::dt(time,mLastUpdateTime) >= 3)
	//	{
	//		mCurStep = STEP_TURN;
	//		mLastUpdateTime = time;
	//		gMotorDrive.drive(100,-100);
	//		gStabiServo.start(0);					//?X?^?r??????
	//	}
	//	break;
	case STEP_TURN:
		//??????]??s??
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100);
			gStabiServo.start(STABI_BASE_ANGLE);	//?X?^?r?L???
		}
		break;
	case STEP_FORWARD:
		//?O?i??s??
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,-100);
			gStabiServo.start(STABI_BASE_ANGLE);	//?X?^?r?L???
		}
		break;
	}
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
EscapingRandom::~EscapingRandom()
{
}
*/
bool Waking::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_START;

	gMotorDrive.setRunMode(true);
	gMotorDrive.drive(-mStartPower);	//8-9 chou マイナスにし??//???[?^?o??
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gStabiServo.setRunMode(true);
	mAngleOnBegin = gGyroSensor.getRz();
	mWakeRetryCount = 0;
	gBackStabiServo.setRunMode(true);
	gSoftCameraServo.setRunMode(true);
	//backstabi ??? backstabi????

	gBackStabiServo.moveHold();  //

	//softcameraservo ??? ????????@???????????
	gSoftCameraServo.moveHold();

	//?Ostabi ????????@stop
	//8-9 gStabiServo.stop();
	
	//8-9 comment out gStabiServo.start(STABI_FOLD_ANGLE);
	gStabiServo.start(0.2);//8-9
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
	switch(mCurStep)//?N???オ??J?n?????m??????
	{
		case STEP_STOP:
			if(Time::dt(time,mLastUpdateTime) > 2)//2?b????????n?????m?????????????????
			{
				Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
				setRunMode(false);
				gMotorDrive.drive(0);
			}
			if(gAccelerationSensor.getPhi() < mAngleThreshold)	//?p?x????????????????n?????(?????x?Z???T???p)
			{
				Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
				gBuzzer.start(30,20,2);
				mLastUpdateTime = time;
				mCurStep = STEP_VERIFY;
				gMotorDrive.drive(0);

			}

			//??]?????p?x?????????[?^??o???ω???????
			//power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
			//gMotorDrive.drive(power);
			break;

		double dt;
		case STEP_START:
			if(Time::dt(time,mLastUpdateTime) > 0.5)//??莞???]?????m????????????]?s??\????f
			{
				Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
				mLastUpdateTime = time;
				mCurStep = STEP_VERIFY;
				gMotorDrive.drive(0);
			}
			if(abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)//??]?????m?????????N???オ??J?n????????f(?W???C?????p) waking_threshold =200
			{
				Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
				gBuzzer.start(30,20,2);
				mLastUpdateTime = time;
				//gStabiServo.start(0.2);
				mCurStep = STEP_DEACCELERATE;
			}
			break;

		case STEP_DEACCELERATE:	//??????茸??????
			dt = Time::dt(time, mLastUpdateTime);


			//gStabiServo.start(0.2);//8-9
			//gBackStabiServo.moveRelease();
		if(dt > mDeaccelerateDuration)
			{
				Debug::print(LOG_SUMMARY, "Waking Deaccelerate finished!\r\n");
				gBuzzer.start(30,20,2);
				mLastUpdateTime = time;
				mCurStep = STEP_VERIFY;
				gMotorDrive.drive(0);
			}
			else
			{
				int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (mStartPower / 2/*2?????*/)), 0);

				tmp_power=-tmp_power;
				gMotorDrive.drive(tmp_power);
			}
			break;

		case STEP_VERIFY:	
			//?N???オ?肪???????????????????x?Z???T?????
			if(Time::dt(time,mLastUpdateTime) <= 2.5)	//???[?o??p?????????????莞????
			{
				return;
			}

			if(gAccelerationSensor.getAz() > 0.0)
			{
				Debug::print(LOG_SUMMARY,"Waking Successed!\r\n");
				gBuzzer.start(30,20,4);

				gBackStabiServo.moveRelease();

				//?N???オ??????A?Ostabi ????
				//gStabiServo.start(STABI_WAKING_ANGLE);
				gStabiServo.start(STABI_BASE_ANGLE); // ?N???オ?萬????????X?^?r??x?[?X??p?x????

				gSoftCameraServo.moveRelease();
				mLastUpdateTime = time;
				mCurStep = STEP_LAST;
			}
			else
			{
				gBackStabiServo.moveHold();
				//8-9 gStabiServo.start(STABI_FOLD_ANGLE);
				mLastUpdateTime = time;
				mCurStep = STEP_START;
				mAngleOnBegin = gGyroSensor.getRvx();
				power = std::min((unsigned int)100, mStartPower + ((mWakeRetryCount + 1) * 5));	//???s?????????[?^?o???グ??
				power=-power;
				gMotorDrive.drive(power);

				if(++mWakeRetryCount > WAKING_RETRY_COUNT)
				{
					Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
					setRunMode(false);
					return;
				}
				Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d) by power %f\r\n",mWakeRetryCount,WAKING_RETRY_COUNT,power);
			}
			break;

		case STEP_LAST:
			if(Time::dt(time,mLastUpdateTime) >0.5)
			{
				gSoftCameraServo.start(15);
			}
			if(Time::dt(time,mLastUpdateTime) >1.0)
			{
				gSoftCameraServo.stop();
				setRunMode(false);
			}
	}
}
bool Waking::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 4)
	{
		if(args[1].compare("set") == 0)
		{
			if(args[2].compare("power") == 0)//mStartPower
			{
				setPower(atoi(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if(args[2].compare("angle") == 0)//mAngleThreshold
			{
				setAngle(atof(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if(args[2].compare("d_time") == 0)//mDeaccelerateDuration
			{
				mDeaccelerateDuration = atof(args[3].c_str());
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
		}
	}
	else if(args.size() == 2)
	{
		if(args[1].compare("show") == 0)
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
	return false;
}
void Waking::setPower(int p)
{
	if(p >= MOTOR_MAX_POWER)
	{
		mStartPower = MOTOR_MAX_POWER;
		return;
	}
	else if(p < 1)
	{
		mStartPower = 1;
		return;
	}
	mStartPower = p;
}
void Waking::setAngle(double a)
{
	if(a >= 180)
	{
		mAngleThreshold = 180;
	}
	else if(a < 0)
	{
		mAngleThreshold = 0;
	}
	mAngleThreshold = a;
}
Waking::Waking() : mWakeRetryCount(0),mStartPower(100),mAngleThreshold(70),mDeaccelerateDuration(1.0)
{
	setName("waking");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
{
}

//bool Turning::onInit(const struct timespec& time)
//{
//	mTurnPower = 0;
//	gGyroSensor.setRunMode(true);
//	mAngle = gGyroSensor.getRz();
//	mLastUpdateTime = time;
//	return true;
//}
//void Turning::onUpdate(const struct timespec& time)
//{
//	double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
//	if(Time::dt(time,mLastUpdateTime) >= 5 || turnedAngle > 15)
//	{
//		Debug::print(LOG_SUMMARY, "Turning: Detected turning\r\n");
//		gMotorDrive.drive(0);
//		setRunMode(false);
//	}else
//	{
//		if(mIsTurningLeft)gMotorDrive.drive(-mTurnPower,mTurnPower);
//		else gMotorDrive.drive(mTurnPower,-mTurnPower);
//		if(turnedAngle < 5)mTurnPower += 0.1;
//	}
//}
//void Turning::setDirection(bool left)
//{
//	mIsTurningLeft = left;
//}
//Turning::Turning()
//{
//	setName("turning");
//	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
//}
//Turning::~Turning()
//{
//}
//
//bool Avoiding::onInit(const struct timespec& time)
//{
//	mLastUpdateTime = time;
//	if(!gEscapingState.isActive())gMotorDrive.drive(0,50);
//	mAngle = gGyroSensor.getRz();
//	mCurStep = STEP_TURN;
//	return true;
//}
//void Avoiding::onUpdate(const struct timespec& time)
//{
//	if(gEscapingState.isActive())
//	{
//		Debug::print(LOG_SUMMARY, "Avoiding: Escaping is already running. Avoiding Canceled!\r\n");
//		setRunMode(false);
//	}
//	switch(mCurStep)
//	{
//	case STEP_TURN:
//		if(Time::dt(time,mLastUpdateTime) > 5 || abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle)) > 45)
//		{
//			Debug::print(LOG_SUMMARY, "Avoiding: forwarding\r\n");
//			mLastUpdateTime = time;
//			gMotorDrive.startPID(10,MOTOR_MAX_POWER);
//			mCurStep = STEP_FORWARD;
//		}
//		break;
//	case STEP_FORWARD:
//		if(Time::dt(time,mLastUpdateTime) > 4)
//		{
//			Debug::print(LOG_SUMMARY, "Avoiding: finished\r\n");
//			setRunMode(false);
//		}
//		break;
//	}
//}
//Avoiding::Avoiding()
//{
//	setName("avoiding");
//	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
//}
//Avoiding::~Avoiding()
//{
//}

//bool PictureTaking::onInit(const struct timespec& time)
//{
//	mLastUpdateTime = time;
//	gCameraCapture.setRunMode(true);
//	gBuzzer.setRunMode(true);
//	gWakingState.setRunMode(true);
//	mStepCount = 0;
//	return true;
//}
//void PictureTaking::onUpdate(const struct timespec& time)
//{
//	if(gWakingState.isActive())return;
//	if(Time::dt(time,mLastUpdateTime) > 1)
//	{
//		mLastUpdateTime = time;
//		++mStepCount;
//		gBuzzer.start(mStepCount > 25 ? 30 : 10);
//
//		if(mStepCount == 25)
//		{
//			gCameraCapture.startWarming();
//		}
//		if(mStepCount >= 30)
//		{
//			Debug::print(LOG_SUMMARY, "Say cheese!\r\n");
//			setRunMode(false);
//			gBuzzer.start(300);
//			gCameraCapture.save();
//		}
//	}
//}
//PictureTaking::PictureTaking() : mStepCount(0)
//{
//	setName("kinen");
//	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
//}
//PictureTaking::~PictureTaking()
//{
//}

bool SensorLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameGPS,"Log started\r\n");
	write(mFilenameGyro,"Log started\r\n");
	write(mFilenamePressure,"Log started\r\n");
	//write(mFilenameEncoder,"Log started\r\n");

	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	//mLastEncL = gMotorDrive.getL();
	//mLastEncR = gMotorDrive.getR();
	return true;
}
void SensorLogging::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) >= 1)
	{
		mLastUpdateTime = time;

		//???O????
		VECTOR3 vec;
		gGPSSensor.get(vec);
		if(gGPSSensor.isActive())write(mFilenameGPS,"%f,%f,%f\r\n",vec.x,vec.y,vec.z);
		else write(mFilenameGPS,"unavailable\r\n");

		if(gGyroSensor.isActive())write(mFilenameGyro,"%f,%f,%f,%f,%f,%f\r\n",gGyroSensor.getRvx(),gGyroSensor.getRvy(),gGyroSensor.getRvz(),gGyroSensor.getRx(),gGyroSensor.getRy(),gGyroSensor.getRz());
		else write(mFilenameGyro,"unavailable\r\n");

		if(gPressureSensor.isActive())write(mFilenamePressure,"%d\r\n",gPressureSensor.get());
		else write(mFilenamePressure,"unavailable\r\n");

		//if(gMotorDrive.isActive())
		//{
		//	write(mFilenameEncoder,"%llu,%llu\r\n",gMotorDrive.getL() - mLastEncL,gMotorDrive.getR() - mLastEncR);
		//	mLastEncL = gMotorDrive.getL();
		//	mLastEncR = gMotorDrive.getR();
		//}else write(mFilenameEncoder,"unavailable\r\n");
	}
}
void SensorLogging::write(const std::string& filename, const char* fmt, ... )
{
	std::ofstream of(filename.c_str(),std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
SensorLogging::SensorLogging() : mLastUpdateTime()
{
	setName("sensorlogging");
	setPriority(UINT_MAX,TASK_INTERVAL_SEQUENCE);

	Filename("log_gps",".txt").get(mFilenameGPS);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameGPS.c_str());
	Filename("log_gyro",".txt").get(mFilenameGyro);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameGyro.c_str());
	Filename("log_pressure",".txt").get(mFilenamePressure);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenamePressure.c_str());
	//Filename("log_encoder_by_sensorlogging",".txt").get(mFilenameEncoder);
	//Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameEncoder.c_str());
}
SensorLogging::~SensorLogging()
{
}

bool MovementLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameEncoder,"Log started\r\n");
	write(mFilenameAcceleration,"Log started\r\n");

	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	gMotorDrive.drive(100);
	return true;
}
void MovementLogging::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) < 1)
	{
		return;
	}
	mLastUpdateTime = time;

	//?????x????O????
	if(gAccelerationSensor.isActive())
	{
		write(mFilenameAcceleration,"%f,%f,%f\r\n",gAccelerationSensor.getAx(),gAccelerationSensor.getAy(),gAccelerationSensor.getAz());
	}
	else
	{
		write(mFilenameAcceleration,"unavailable\r\n");
	}

	if(!gMotorDrive.isActive())
	{
		write(mFilenameEncoder,"unavailable\r\n");
		return;
	}

	//?G???R?[?_????O????
	//???V?I????X??????log????f????
	if(gMotorDrive.getPowerL() != mPrevPowerL || gMotorDrive.getPowerR() != mPrevPowerR)
	{
		mPrevPowerL = gMotorDrive.getPowerL();
		mPrevPowerR = gMotorDrive.getPowerR();
		write(mFilenameEncoder,		"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
		Debug::print(LOG_SUMMARY,	"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
	}

	//?G???R?[?_?p???X??????l??擾
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();	

	//??]??????Z
	unsigned long long rotationsL = MotorEncoder::convertRotation(deltaPulseL);
	unsigned long long rotationsR = MotorEncoder::convertRotation(deltaPulseR);

	if(mPrintFlag)
	{
		Debug::print(LOG_SUMMARY,"Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);
	}
	write(mFilenameEncoder,	 	 "Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);

	//?X?^?b?N?????e?X?g
	if(mPrevDeltaPulseL >= STUCK_ENCODER_PULSE_THRESHOLD && mPrevDeltaPulseR >= STUCK_ENCODER_PULSE_THRESHOLD)	//?O???p???X?????l???
	{
		if(deltaPulseL < STUCK_ENCODER_PULSE_THRESHOLD && deltaPulseR < STUCK_ENCODER_PULSE_THRESHOLD)			//?????p???X?????l???
		{
			write(mFilenameEncoder,		"Stuck detected!");
			if(mBuzzerFlag)
			{
				gBuzzer.start(200, 50 ,3);		//?X?^?b?N????(?????????)
			}
		}
	}
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
}
bool MovementLogging::onCommand(const std::vector<std::string>& args)
{
	if(!gMovementLoggingState.isActive())
	{
		Debug::print(LOG_PRINT,"Movement Logging is not active\r\n");
		return true;
	}

	if(args.size() == 2)
	{
		if(args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			gMotorDrive.drive(0);
			gMovementLoggingState.setRunMode(false);
			return true;
		}
		else if(args[1].compare("buzzer") == 0)
		{
			mBuzzerFlag = !mBuzzerFlag;	//flag??????
			if(mBuzzerFlag)
			{
				Debug::print(LOG_PRINT,"Command Executed! Buzzer(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT,"Command Executed! Buzzer(OFF)\r\n");
			}
			return true;
		}
		else if(args[1].compare("print") == 0)
		{
			mPrintFlag = !mPrintFlag;	//flag??????
			if(mPrintFlag)
			{
				Debug::print(LOG_PRINT,"Command Executed! Print(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT,"Command Executed! Print(OFF)\r\n");
			}
			return true;
		}
	}
	else if(args.size() == 3)
	{
		if(args[1].compare("comment") == 0)
		{
			std::string str = args[2];
			Debug::print(LOG_PRINT,"Command Executed! comment: %s\r\n", str.c_str());
			write(mFilenameAcceleration,"comment: %s\r\n", str.c_str());
			write(mFilenameEncoder,		"comment: %s\r\n", str.c_str());
			return true;
		}
	}
	
	Debug::print(LOG_PRINT,"movementlogging stop            : stop MovementLogging\r\n\
movementlogging buzzer          : switch buzzer\r\n\
movementlogging print           : switch pulse print\r\n\
movementlogging comment [string]: comment into log\r\n");
	return true;
}
void MovementLogging::write(const std::string& filename, const char* fmt, ... )
{
	std::ofstream of(filename.c_str(),std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
	//Debug::print(LOG_SUMMARY, "%s\r\n",buf);
}
MovementLogging::MovementLogging() : mLastUpdateTime(), mPrevPowerL(0), mPrevPowerR(0), mPrevDeltaPulseL(0), mPrevDeltaPulseR(0), mBuzzerFlag(true), mPrintFlag(false)
{
	setName("movementlogging");
	setPriority(UINT_MAX,TASK_INTERVAL_SEQUENCE);

	Filename("log_encoder",".txt").get(mFilenameEncoder);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameEncoder.c_str());
	Filename("log_acceleration",".txt").get(mFilenameAcceleration);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameAcceleration.c_str());
}
MovementLogging::~MovementLogging()
{
}

bool EncoderMonitoring::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "EncoderMonitoring: Start!\r\n");
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);
	
	mLastSamplingTime = time;
	mLastUpdateTime = time;
	mCurrentMaxPulse = 0;
	mPrevDeltaPulseL = 0;
	mPrevDeltaPulseR = 0;
	gMotorDrive.getDeltaPulseL();//?p???X?????????擾??????Z?b?g???????
	gMotorDrive.getDeltaPulseR();
	return true;
}
void EncoderMonitoring::onUpdate(const struct timespec& time)
{
	//??????o???????????????????
	if(Time::dt(time,mLastSamplingTime) < 1) return;

	mLastSamplingTime = time;
	
	//?X?^?b?N???????return
/*	if(gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}*/

	//?G???R?[?_?p???X??????l??擾
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();

	if(mIsPrint) Debug::print(LOG_SUMMARY, "EncoderMonitoring: current pulse count(%llu %llu)\r\n",deltaPulseL,deltaPulseR);
	
	//?O??l?????????
	if(removeError(deltaPulseL,deltaPulseR))
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}
	
	//?l??v?Z
	unsigned long long pulse_threshold = std::min(mStoredPulse - mThresholdPulse, mUpperThreshold);
	
	//?X?^?b?N?`?F?b?N?D?O???l????C?????l??????X?^?b?N??????
	if(mPrevDeltaPulseL >= pulse_threshold && mPrevDeltaPulseR >= pulse_threshold)	//?O???p???X?????l???
	{
		if(deltaPulseL < pulse_threshold || deltaPulseR < pulse_threshold)			//?????p???X?????l???
		{
			//?X?^?b?N????
			gBuzzer.start(80, 10 ,6);
			Debug::print(LOG_SUMMARY, "EncoderMonitoring: STUCK detected by pulse count(%llu %llu). Threshold:%llu\r\n",deltaPulseL,deltaPulseR,pulse_threshold);
			//gEscapingByStabiState.setRunMode(true);
			setRunMode(false);
			return;
		}
	}
	
	//?O???p???X??X?V
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
	
	//mCurrentMaxPulse?????????mCurrentMaxPulse??X?V????
	if(std::max(deltaPulseL,deltaPulseR) > mCurrentMaxPulse)
	{
		mCurrentMaxPulse = std::max(deltaPulseL,deltaPulseR);
	}
	
	//?l??X?V
	if(Time::dt(time,mLastUpdateTime) >= mUpdateTimer)
	{
		updateThreshold();
		mCurrentMaxPulse = 0;
		mLastUpdateTime = time;
	}
}
bool EncoderMonitoring::onCommand(const std::vector<std::string>& args)
{
	//if(!gEncoderMonitoringState.isActive())
	//{
	//	Debug::print(LOG_PRINT,"EncoderMonitoring is not active\r\n");
	//	return true;
	//}

	if(args.size() == 2)
	{
		if(args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			gMotorDrive.drive(0);
			gEncoderMonitoringState.setRunMode(false);
			return true;
		}
		else if(args[1].compare("print") == 0)
		{
			mIsPrint = !mIsPrint;
			if(mIsPrint) Debug::print(LOG_PRINT,"Print ON!\r\n");
			else Debug::print(LOG_PRINT,"Print OFF!\r\n");
			return true;
		}
		else if(args[1].compare("show") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n\n");
			
			Debug::print(LOG_PRINT,"UpdateTimer           : %d\r\n\
StoredPulse           : %llu\r\n\
ThresholdPulse        : %llu\r\n\
IgnoredDeltaUpperPulse: %llu\r\n\
IgnoredDeltaLowerPulse: %llu\r\n\
UpperThreshold        : %llu\r\n\
LowerThreshold        : %llu\r\n",mUpdateTimer,mStoredPulse,mThresholdPulse,mIgnoredDeltaUpperPulse,mIgnoredDeltaLowerPulse,mUpperThreshold,mLowerThreshold);
			return true;
		}
	}
	else if(args.size() == 4)
	{
 		if(args[1].compare("set") == 0)
		{
			if(args[2].compare("timer") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mUpdateTimer = atoi(args[3].c_str());
				return true;
			}
			else if(args[2].compare("stored") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mStoredPulse = atol(args[3].c_str());
				return true;
			}
			else if(args[2].compare("threpulse") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mThresholdPulse = atol(args[3].c_str());
				return true;
			}
			else if(args[2].compare("deltaupper") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mIgnoredDeltaUpperPulse = atol(args[3].c_str());
				return true;
			}
			else if(args[2].compare("deltalower") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mIgnoredDeltaLowerPulse = atol(args[3].c_str());
				return true;
			}
			else if(args[2].compare("upper") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mUpperThreshold = atol(args[3].c_str());
				return true;
			}
			else if(args[2].compare("lower") == 0)
			{
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
				mLowerThreshold = atol(args[3].c_str());
				return true;
			}
		}
	}
	
	Debug::print(LOG_PRINT,"monitoring stop                   : stop EncoderMonitoring\r\n\
monitoring set timer [second]     : set update timer[s]\r\n\
monitoring set stored [pulse]     : set StoredPulse\r\n\
monitoring set threpulse [pulse]  : set Threshold Pulse\r\n\
monitoring set deltaupper [pulse] : set IgnoredDeltaUpperPulse\r\n\
monitoring set deltalower [pulse] : set IgnoredDeltaLowerPulse\r\n\
monitoring set upper [pulse]      : set UpperThreshold\r\n\
monitoring set lower [pulse]      : set LowerThreshold\r\n\
monitoring print                  : switch print\r\n\
monitoring show                   : show each value\r\n");
	return true;
}
void EncoderMonitoring::updateThreshold()
{
	//?p???X????????????????(or????)???????????
	if(mCurrentMaxPulse <= mLowerThreshold)
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: threshold update is ignored. mCurrentMaxPulse(%llu) <= mLowerThreshold(%llu)\r\n",mCurrentMaxPulse,mLowerThreshold);
		return;
	}
	else if(mCurrentMaxPulse >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: threshold update is ignored. mCurrentMaxPulse(%llu) <= mUpperThreshold(%llu)\r\n",mCurrentMaxPulse,mUpperThreshold);
		return;
	}
	
	//???????????l???X?V???????????????
	if((mStoredPulse >= mCurrentMaxPulse) && (mStoredPulse - mCurrentMaxPulse) >= mIgnoredDeltaLowerPulse) 
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: threshold update is ignored. %llu >= mIgnoredDeltaLowerPulse(%llu))\r\n",(mStoredPulse - mCurrentMaxPulse),mIgnoredDeltaLowerPulse);
		return;
	}
	else if((mStoredPulse < mCurrentMaxPulse) && (mCurrentMaxPulse - mStoredPulse) >= mIgnoredDeltaUpperPulse) 
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: threshold update is ignored. %llu >= mIgnoredDeltaUpperPulse(%llu))\r\n",(mCurrentMaxPulse - mStoredPulse),mIgnoredDeltaUpperPulse);
		return;
	}
	
	mStoredPulse = mCurrentMaxPulse;
	Debug::print(LOG_SUMMARY,"EncoderMonitoring: StoredPulse is updated! -> %llu\r\n",mStoredPulse);
}
bool EncoderMonitoring::removeError(unsigned long long pulseL,unsigned long long pulseR)
{
	bool ret = false;
	//L??`?F?b?N
	if(pulseL == 0) 
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: [ERROR] Left pulse is zero...\r\n");
		ret = true;
	}
	else if(pulseL >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: Left pulse is error... pulseL: %llu, UpperThreshold: %llu\r\n",pulseL ,mUpperThreshold);
		ret = true;
	}
	
	//R??`?F?b?N
	if(pulseR == 0)
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: [ERROR] Right pulse is zero...\r\n");
		ret = true;
	}
	else if(pulseR >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY,"EncoderMonitoring: Right pulse is error... pulseR: %llu, UpperThreshold: %llu\r\n",pulseR ,mUpperThreshold);
		ret = true;
	}
	return ret;
}
EncoderMonitoring::EncoderMonitoring() : mLastSamplingTime(),mLastUpdateTime(),mStoredPulse(2500),mUpdateTimer(30),mThresholdPulse(1000),mIgnoredDeltaUpperPulse(1500),mIgnoredDeltaLowerPulse(800),mUpperThreshold(3000),mLowerThreshold(1500),mIsPrint(false)
{
	setName("monitoring");
	setPriority(UINT_MAX,TASK_INTERVAL_SEQUENCE);
}
EncoderMonitoring::~EncoderMonitoring()
{
}

bool StatusSending::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "StatusSending: start!\r\n");
	mLastSendedTime = time;
	gGPSSensor.setRunMode(true);
	gPoseDetecting.setRunMode(true);
	return true;
}
void StatusSending::onUpdate(const struct timespec& time)
{
	if(Time::dt(time, mLastSendedTime) > mSendPeriod) //double mSendPeriod周期でステータス送信
	{
		mLastSendedTime = time;
		sendStatus();
	}
}
bool StatusSending::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 2)
	{
		setPeriod(atoi(args[1].c_str()));
		Debug::print(LOG_SUMMARY, "mSendPeriod: %d\r\n", mSendPeriod);
		return true;
	}
	Debug::print(LOG_SUMMARY, "mSendPeriod: %d\r\n", mSendPeriod);
	Debug::print(LOG_SUMMARY, "sending setperiod [period]: set period of sending data\r\n");
	return false;
}
//UIに情報を送る
void StatusSending::sendStatus()
{
	char send_string[256];
	int mSatelites = gGPSSensor.getSatelites();
	VECTOR3 vec;
	gGPSSensor.get(vec);

	//衛星数 X Y Z direction isFlip isLie
	sprintf(send_string, "python /home/pi/high-ball-server/websocket_upload/websocket_sendstatus.py rover_status %d %f %f %f %f %d %d", gGPSSensor.getSatelites(), vec.x, vec.y, vec.z, gPoseDetecting.getYaw(), gPoseDetecting.isFlip(), gPoseDetecting.isLie());

	system(send_string);
}
void StatusSending::setPeriod(double period)
{
	mSendPeriod = period;
}
StatusSending::StatusSending() : mSendPeriod(1.0)
{
	setName("sending");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
StatusSending::~StatusSending()
{
}
