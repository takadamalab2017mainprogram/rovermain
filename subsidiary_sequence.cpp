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
#include "actuator.h"
#include "motor.h"
#include "image_proc.h"

Escaping gEscapingState;
EscapingRandom gEscapingRandomState;
EscapingByStabi gEscapingByStabiState;
Waking gWakingState;
Turning gTurningState;
Avoiding gAvoidingState;
WadachiPredicting gPredictingState;
PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
MovementLogging gMovementLoggingState;
EncoderMonitoring gEncoderMonitoringState;

bool WadachiPredicting::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	gCameraCapture.startWarming();

	return true;
}
void WadachiPredicting::onUpdate(const struct timespec& time)
{
	if(gAvoidingState.isActive())return;
	if(!mIsAvoidingEnable)
	{
		if(Time::dt(time,mLastUpdateTime) >= 2.5)
		{
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(!gImageProc.isWadachiExist(pImage))return;
			//轍を事前検知した
			gCameraCapture.startWarming();
		}
		return;
	}

	switch(mCurStep)
	{
	case STEP_RUNNING:
		if(Time::dt(time,mLastUpdateTime) > 60)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Stoping started\r\n");
			mCurStep = STEP_STOPPING;
			mLastUpdateTime = time;
			gMotorDrive.drive(0);
		}
		break;
	case STEP_STOPPING:
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Waking started\r\n");
			mCurStep = STEP_WAKING;
			mLastUpdateTime = time;
			gWakingState.setRunMode(true);
		}
		break;
	case STEP_WAKING:
		if(!gWakingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Predicting: Checking started\r\n");
			mCurStep = STEP_CHECKING;
			mLastUpdateTime = time;
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CHECKING:
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Avoiding started\r\n");
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isWadachiExist(pImage))
			{
				//轍を事前検知した
				gAvoidingState.setRunMode(true);
				mCurStep = STEP_AVOIDING;
			}else
			{
				mCurStep = STEP_RUNNING;
				gMotorDrive.startPID(0,MOTOR_MAX_POWER);
			}
		}
		break;
	case STEP_AVOIDING:
		if(!gAvoidingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Predicting: Avoiding finished\r\n");
			mCurStep = STEP_RUNNING;
			mLastUpdateTime = time;
		}
		break;
	}
}
bool WadachiPredicting::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("enable") == 0)
		{
			mIsAvoidingEnable = true;
			return true;
		}
		if(args[1].compare("disable") == 0)
		{
			mIsAvoidingEnable = false;
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "predicting [enable/disable]  : switch avoiding mode\r\n");
	return false;
}
bool WadachiPredicting::isWorking(const struct timespec& time)
{
	return mIsAvoidingEnable && (mCurStep != STEP_RUNNING || (mCurStep == STEP_RUNNING && Time::dt(time,mLastUpdateTime) < 6));
}
WadachiPredicting::WadachiPredicting() : mIsAvoidingEnable(false),mCurStep(STEP_RUNNING)
{
	setName("predicting");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
WadachiPredicting::~WadachiPredicting()
{
}

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
		//バックを行う
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
		//再起動防止のため待機
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			if(mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
			{
				//ランダム移行
				Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
				mEscapingTriedCount = 0;
				mCurStep = STEP_RANDOM;
				mCurRandomStep = RANDOM_STEP_FORWARD;
				break;
			}
			mCurStep = STEP_PRE_CAMERA;
			mLastUpdateTime = time;
			//起き上がり動作を行う
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isSky(pImage))gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_CAMERA:
		//画像撮影用に起き上がり動作を行い、数秒待機する
		if(gWakingState.isActive())mLastUpdateTime = time;//起き上がり動作中は待機する
		if(Time::dt(time,mLastUpdateTime) > 2)//起き上がり完了後、一定時間が経過していたら
		{
			Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
			//画像撮影動作を行う
			mCurStep = STEP_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CAMERA:
		//画像処理を行い、今後の行動を決定する
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
		//画像処理の結果、回転する必要があった場合
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_CAMERA_FORWARD;
			gMotorDrive.startPID(0,100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_FORWARD:
		//画像処理の結果、直進する必要があった場合
		if(Time::dt(time,mLastUpdateTime) >= 10)
		{
			gMotorDrive.drive(-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		//画像処理の結果、その場回転する必要があった場合
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//ランダム動作
		if(Time::dt(time,mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if(mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//ランダム移行
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
		//バックを行う
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100,-100);
		break;
	case RANDOM_STEP_TURN:
		//その場回転を行う
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100);
		break;
	case RANDOM_STEP_FORWARD:
		//前進を行う
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
		default://カメラ使えなかった
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
	//	//バックを行う
	//	if(Time::dt(time,mLastUpdateTime) >= 3)
	//	{
	//		mCurStep = STEP_TURN;
	//		mLastUpdateTime = time;
	//		gMotorDrive.drive(100,-100);
	//		gStabiServo.start(0);					//スタビたたむ
	//	}
	//	break;
	case STEP_TURN:
		//その場回転を行う
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100);
			gStabiServo.start(STABI_BASE_ANGLE);	//スタビ伸ばす
		}
		break;
	case STEP_FORWARD:
		//前進を行う
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,-100);
			gStabiServo.start(STABI_BASE_ANGLE);	//スタビ伸ばす
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

bool Waking::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_START;

	gMotorDrive.setRunMode(true);
	gMotorDrive.drive(mStartPower);		//モータ出力
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gStabiServo.setRunMode(true);
	mAngleOnBegin = gGyroSensor.getRz();
	mWakeRetryCount = 0;

	gStabiServo.start(STABI_WAKING_ANGLE);
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
	switch(mCurStep)//起き上がり開始が検知された場合
	{
	case STEP_STOP:
		if(Time::dt(time,mLastUpdateTime) > 2)//2秒まわしても着地が検知されない場合はあきらめる
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
			gMotorDrive.drive(0);
		}
		if(gAccelerationSensor.getPhi() < mAngleThreshold)	//角度が一定以下になったら着地と判定(加速度センサを採用)
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			gBuzzer.start(30,20,2);
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);

		}

		//回転した角度に応じてモータの出力を変化させる
		//power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
		//gMotorDrive.drive(power);
		break;

	double dt;
	case STEP_START:
		if(Time::dt(time,mLastUpdateTime) > 0.5)//一定時間回転が検知されない場合→回転不可能と判断
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		if(abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)//回転が検知された場合→起き上がり開始したと判断(ジャイロを採用)
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			gBuzzer.start(30,20,2);
			mLastUpdateTime = time;
			mCurStep = STEP_DEACCELERATE;
		}
		break;

	case STEP_DEACCELERATE:	//ゆっくり減速する
		dt = Time::dt(time, mLastUpdateTime);
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
			int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (mStartPower / 2/*2で割る*/)), 0);
			gMotorDrive.drive(tmp_power);
		}
		break;

	case STEP_VERIFY:
		//起き上がりが成功したか否かを加速度センサで検証
		if(Time::dt(time,mLastUpdateTime) <= 2.5)	//ローバの姿勢が安定するまで一定時間待つ
		{
			return;
		}

		if(gAccelerationSensor.getAz() > 0.0)
		{
			Debug::print(LOG_SUMMARY,"Waking Successed!\r\n");
			gBuzzer.start(30,20,4);
			setRunMode(false);
			gStabiServo.start(STABI_BASE_ANGLE); // 起き上がり成功したらスタビをベースの角度に戻す
		}
		else
		{
			mLastUpdateTime = time;
			mCurStep = STEP_START;
			mAngleOnBegin = gGyroSensor.getRvx();
			power = std::min((unsigned int)100, mStartPower + ((mWakeRetryCount + 1) * 5));	//試行回数ごとにモータ出力を上げる
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
Waking::Waking() : mWakeRetryCount(0),mStartPower(45),mAngleThreshold(70),mDeaccelerateDuration(0.5)
{
	setName("waking");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
{
}

bool Turning::onInit(const struct timespec& time)
{
	mTurnPower = 0;
	gGyroSensor.setRunMode(true);
	mAngle = gGyroSensor.getRz();
	mLastUpdateTime = time;
	return true;
}
void Turning::onUpdate(const struct timespec& time)
{
	double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	if(Time::dt(time,mLastUpdateTime) >= 5 || turnedAngle > 15)
	{
		Debug::print(LOG_SUMMARY, "Turning: Detected turning\r\n");
		gMotorDrive.drive(0);
		setRunMode(false);
	}else
	{
		if(mIsTurningLeft)gMotorDrive.drive(-mTurnPower,mTurnPower);
		else gMotorDrive.drive(mTurnPower,-mTurnPower);
		if(turnedAngle < 5)mTurnPower += 0.1;
	}
}
void Turning::setDirection(bool left)
{
	mIsTurningLeft = left;
}
Turning::Turning()
{
	setName("turning");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Turning::~Turning()
{
}

bool Avoiding::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	if(!gEscapingState.isActive())gMotorDrive.drive(0,50);
	mAngle = gGyroSensor.getRz();
	mCurStep = STEP_TURN;
	return true;
}
void Avoiding::onUpdate(const struct timespec& time)
{
	if(gEscapingState.isActive())
	{
		Debug::print(LOG_SUMMARY, "Avoiding: Escaping is already running. Avoiding Canceled!\r\n");
		setRunMode(false);
	}
	switch(mCurStep)
	{
	case STEP_TURN:
		if(Time::dt(time,mLastUpdateTime) > 5 || abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle)) > 45)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: forwarding\r\n");
			mLastUpdateTime = time;
			gMotorDrive.startPID(10,MOTOR_MAX_POWER);
			mCurStep = STEP_FORWARD;
		}
		break;
	case STEP_FORWARD:
		if(Time::dt(time,mLastUpdateTime) > 4)
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
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Avoiding::~Avoiding()
{
}

bool PictureTaking::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	gCameraCapture.setRunMode(true);
	gBuzzer.setRunMode(true);
	gWakingState.setRunMode(true);
	mStepCount = 0;
	return true;
}
void PictureTaking::onUpdate(const struct timespec& time)
{
	if(gWakingState.isActive())return;
	if(Time::dt(time,mLastUpdateTime) > 1)
	{
		mLastUpdateTime = time;
		++mStepCount;
		gBuzzer.start(mStepCount > 25 ? 30 : 10);

		if(mStepCount == 25)
		{
			gCameraCapture.startWarming();
		}
		if(mStepCount >= 30)
		{
			Debug::print(LOG_SUMMARY, "Say cheese!\r\n");
			setRunMode(false);
			gBuzzer.start(300);
			gCameraCapture.save();
		}
	}
}
PictureTaking::PictureTaking() : mStepCount(0)
{
	setName("kinen");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
PictureTaking::~PictureTaking()
{
}

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

		//ログを保存
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

	//加速度のログを保存
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

	//エンコーダのログを保存
	//レシオ比が変更されたらlogに反映する
	if(gMotorDrive.getPowerL() != mPrevPowerL || gMotorDrive.getPowerR() != mPrevPowerR)
	{
		mPrevPowerL = gMotorDrive.getPowerL();
		mPrevPowerR = gMotorDrive.getPowerR();
		write(mFilenameEncoder,		"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
		Debug::print(LOG_SUMMARY,	"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
	}

	//エンコーダパルスの差分値の取得
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();	

	//回転数に換算
	unsigned long long rotationsL = MotorEncoder::convertRotation(deltaPulseL);
	unsigned long long rotationsR = MotorEncoder::convertRotation(deltaPulseR);

	if(mPrintFlag)
	{
		Debug::print(LOG_SUMMARY,"Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);
	}
	write(mFilenameEncoder,	 	 "Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);

	//スタック判定のテスト
	if(mPrevDeltaPulseL >= STUCK_ENCODER_PULSE_THRESHOLD && mPrevDeltaPulseR >= STUCK_ENCODER_PULSE_THRESHOLD)	//前回のパルス数が閾値以上
	{
		if(deltaPulseL < STUCK_ENCODER_PULSE_THRESHOLD && deltaPulseR < STUCK_ENCODER_PULSE_THRESHOLD)			//今回のパルス数が閾値以下
		{
			write(mFilenameEncoder,		"Stuck detected!");
			if(mBuzzerFlag)
			{
				gBuzzer.start(200, 50 ,3);		//スタック判定(音を鳴らすのみ)
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
			mBuzzerFlag = !mBuzzerFlag;	//flagの切り替え
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
			mPrintFlag = !mPrintFlag;	//flagの切り替え
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
	gMotorDrive.getDeltaPulseL();//パルスは差分なので取得してリセットしておく
	gMotorDrive.getDeltaPulseR();
	return true;
}
void EncoderMonitoring::onUpdate(const struct timespec& time)
{
	//時間が経過していなければ処理を返す
	if(Time::dt(time,mLastSamplingTime) < 1) return;

	mLastSamplingTime = time;
	
	//スタック判定中ならreturn
	if(gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}

	//エンコーダパルスの差分値の取得
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();

	if(mIsPrint) Debug::print(LOG_SUMMARY, "EncoderMonitoring: current pulse count(%llu %llu)\r\n",deltaPulseL,deltaPulseR);
	
	//外れ値は無視する
	if(removeError(deltaPulseL,deltaPulseR))
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}
	
	//閾値の計算
	unsigned long long pulse_threshold = std::min(mStoredPulse - mThresholdPulse, mUpperThreshold);
	
	//スタックチェック．前回が閾値以上で，今回が閾値以下ならスタック判定する
	if(mPrevDeltaPulseL >= pulse_threshold && mPrevDeltaPulseR >= pulse_threshold)	//前回のパルス数が閾値以上
	{
		if(deltaPulseL < pulse_threshold || deltaPulseR < pulse_threshold)			//今回のパルス数が閾値以下
		{
			//スタック判定
			gBuzzer.start(80, 10 ,6);
			Debug::print(LOG_SUMMARY, "EncoderMonitoring: STUCK detected by pulse count(%llu %llu). Threshold:%llu\r\n",deltaPulseL,deltaPulseR,pulse_threshold);
			gEscapingByStabiState.setRunMode(true);
			setRunMode(false);
			return;
		}
	}
	
	//前回のパルスの更新
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
	
	//mCurrentMaxPulseより大きければmCurrentMaxPulseを更新する
	if(std::max(deltaPulseL,deltaPulseR) > mCurrentMaxPulse)
	{
		mCurrentMaxPulse = std::max(deltaPulseL,deltaPulseR);
	}
	
	//閾値の更新
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
	//パルス数があまりにも小さい(or大きい)場合は無視する
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
	
	//あまりにも大きく閾値が更新される場合は無視する
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
	//Lをチェック
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
	
	//Rをチェック
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