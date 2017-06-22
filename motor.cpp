#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include <math.h>
#include "utils.h"
#include "motor.h"
#include "sensor.h"

MotorDrive gMotorDrive;

bool Motor::init(int powPin, int revPin)
{
	//ピン番号を確認
	VERIFY(powPin < 0 || revPin < 0);

	//ピンを初期化
	ForwardPin = powPin;
	ReversePin = revPin;
Debug::print(LOG_SUMMARY, "これがForwardPin番号%d\r\n",ForwardPin);
Debug::print(LOG_SUMMARY, "これReversePin番号%d\r\n",ReversePin);
	//前ピンを出力モードに
	pinMode(ForwardPin, OUTPUT);
	if (softPwmCreate(ForwardPin, 0, 100) != 0)
	{
		Debug::print(LOG_SUMMARY, "Failed to initialize soft-PWM\r\n");
		return false;
	}
	//バックピンを出力モード

	pinMode(ReversePin, OUTPUT);
	if (softPwmCreate(ReversePin, 0, 100) != 0)
	{
		Debug::print(LOG_SUMMARY, "Failed to initialize soft-PWM\r\n");
		return false;
	}

	//LOW状態にして開放
	softPwmWrite(ForwardPin, 0);
	softPwmWrite(ReversePin, 0);


	//現在の出力を保持
	mCurPower = 0;
	return true;
}
void Motor::update(double elapsedSeconds)
{
	if (fabs(mCurPower - mTargetPower) != 0)//目標出力と現在出力に差がある場合
	{
		//なめらかにモータ出力を変化させる
		double curFrameTarget = mTargetPower;//この呼び出しで設定するモーター出力
		double maxMotorPowerChange = MOTOR_MAX_POWER_CHANGE * mCoeff;

		//モータ出力変化量を制限
		//最大モーター出力変化量が目標出力-現在の出力より大きければ
		if (fabs(mTargetPower - mCurPower) > maxMotorPowerChange)
		{
			//モーターの目標出力を現在の出力に変更
			curFrameTarget = mCurPower;
			//現在の出力に最大出力変化量5を足すか引くかする
			curFrameTarget += ((mTargetPower > mCurPower) ? maxMotorPowerChange : -maxMotorPowerChange);
      Debug::print(LOG_SUMMARY,"MOTOR power Limitation %f %f(%d) \r\n",mCurPower,curFrameTarget,mTargetPower);
		}

		//新しいpowerをもとにpinの状態を設定する
		    softPwmWrite(ForwardPin, curFrameTarget > 0 ? fabs(curFrameTarget) : 0);
		    softPwmWrite(ReversePin, curFrameTarget > 0 ? 0 : fabs(curFrameTarget) ) ;
        mCurPower = curFrameTarget;
	}
}
void Motor::clean()
{
	if (ForwardPin >= 0)softPwmWrite(ForwardPin, 0);
	if (ReversePin >= 0)softPwmWrite(ReversePin, 0);
	mCurPower = 0;
}
void Motor::set(int power)
{
	//値の範囲をチェックし、正しい範囲に丸める
	if (power > MOTOR_MAX_POWER)power = MOTOR_MAX_POWER;
	else if (power < -MOTOR_MAX_POWER)power = -MOTOR_MAX_POWER;

	//目標出力を設定
	mTargetPower = power;
}
int Motor::getPower()
{
	return mCurPower;
}
void Motor::setCoeff(double coeff)
{
	mCoeff = coeff;
}
Motor::Motor() : ForwardPin(-1), ReversePin(-1), mCurPower(0), mTargetPower(0), mCoeff(1)
{
}
Motor::~Motor()
{
}

MotorEncoder* MotorEncoder::getInstance()
{
	static MotorEncoder singleton;
return &singleton;
}

void MotorEncoder::pulseLCallback()
{
	MotorEncoder::getInstance()->mPulseCountL++;
	digitalRead(MotorEncoder::getInstance()->mEncoderPin2L) == 1 ? MotorEncoder::getInstance()->mPulseCountL-- : MotorEncoder::getInstance()->mPulseCountL++;
}
void MotorEncoder::pulseRCallback()
{
	MotorEncoder::getInstance()->mPulseCountR++;
  	digitalRead(MotorEncoder::getInstance()->mEncoderPin2R)==1 ? MotorEncoder::getInstance()->mPulseCountR++ : MotorEncoder::getInstance()->mPulseCountR--;
}		
bool MotorEncoder::init()
{
mPulseCountL = mPulseCountR = 0;

	//ピンのパルスを監視する
	if (wiringPiISR(mEncoderPinL, INT_EDGE_RISING, pulseLCallback) == -1 || wiringPiISR(mEncoderPinR, INT_EDGE_RISING, pulseRCallback) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to onInitialize Motor encoder\r\n");
		return false;
	}
	return true;
}
void MotorEncoder::clean()
{
	//両方のピンの割り込みを無効にする
	char command[64];
	sprintf(command, "/usr/local/bin/gpio edge %d none", mEncoderPinL);
	system(command);
	sprintf(command, "/usr/local/bin/gpio edge %d none", mEncoderPinR);
	system(command);

	//スレッドが複数残ることを防止するためsleep
	delay(100);
}
long long MotorEncoder::getL()
{
	//エンコーダーで左側のパルス数を取得
	return mPulseCountL;
}
long long MotorEncoder::getR()
{
	//エンコーダーで右側のパルス数を取得
	return mPulseCountR;
}
long long MotorEncoder::getDeltaPulseL()
{
	long long ret = mPulseCountL;
	mPulseCountL = 0; //リセット
	return ret;
}
long long MotorEncoder::getDeltaPulseR()
{
	long long ret = mPulseCountR;
	mPulseCountR = 0; //リセット
	return ret;
}
double MotorEncoder::convertRotation(long long pulse)
{
	//分解能とギア比で割る
	return pulse / (double)(RESOLVING_POWER * GEAR_RATIO);
}
void MotorEncoder::reset()
{
	mPulseCountL = 0;
	mPulseCountR = 0;
	Debug::print(LOG_SUMMARY, "Motor Pulse Count Reset\r\n");
}

MotorEncoder::MotorEncoder() : mEncoderPinL(PIN_PULSE_B), mEncoderPinR(PIN_PULSE_A), mPulseCountL(0), mPulseCountR(0)
{
	pinMode(mEncoderPin2L, INPUT);
	pinMode(mEncoderPin2R, INPUT);
}
MotorEncoder::~MotorEncoder()
{
}

bool MotorDrive::onInit(const struct timespec& time)
{
	//ジャイロを使うように設定
	gGyroSensor.setRunMode(true);

	//初期化
	if (!mMotorR.init(PIN_PWM_A1, PIN_PWM_A2) || !mMotorL.init(PIN_PWM_B1, PIN_PWM_B2))
	{
		Debug::print(LOG_SUMMARY, "Failed to initialize Motors\r\n");
		return false;
	}
	if (!mpMotorEncoder->init())
	{
		Debug::print(LOG_SUMMARY, "Failed to initialize Motor Encoders\r\n");
		return false;
	}
	
	if (clock_gettime(CLOCK_MONOTONIC_RAW, &mLastUpdateTime) != 0)
	{
		Debug::print(LOG_SUMMARY, "Unable to get time!\r\n");
	}

	mLastUpdateTime = time;
	mAngle = 0;
	return true;
}

void MotorDrive::onClean()
{
  	mpMotorEncoder->clean();
	mMotorL.clean();
	mMotorR.clean();
}

void MotorDrive::updatePIDState(const VECTOR3& pid, double dangle, double maxControlRatio)
{
	//ずれ情報を更新
	double angle = gGyroSensor.normalize(dangle);
	mDiff3 = mDiff2; mDiff2 = mDiff1; mDiff1 = angle;

	//ずれ情報を元に新しいモーター出力を設定(PID)
	double powerDiff = pid.x * (mDiff1 - mDiff2) + pid.y * mDiff1 + pid.z * ((mDiff1 - mDiff2) - (mDiff2 - mDiff3));
	mControlPower += powerDiff;
	
	//モータ速度係数を用意
	double drivePowerRatio = (double)mDrivePower / MOTOR_MAX_POWER;//モータ出力の割合

	//モータの逆回転をせずに方向転換する
	double controlRatio = 1 - std::min(fabs(mControlPower), maxControlRatio);
	if (controlRatio <= 0)controlRatio = 0;

	//モータ出力を適用
	if (mControlPower > 0)
	{
		//Turn Right
		mMotorL.set(mRatioL * drivePowerRatio);
		mMotorR.set(-mRatioR * controlRatio * drivePowerRatio);
	}
	else
	{
		//Turn Left
		mMotorL.set(mRatioL * controlRatio * drivePowerRatio);
		mMotorR.set(-mRatioR * drivePowerRatio);
	}
}
void MotorDrive::updatePIDGyroMove()
{
	updatePIDState(mPIDGyro, gGyroSensor.getRz() - mAngle, mMaxPIDControlRatioGyro);
}

void MotorDrive::onUpdate(const struct timespec& time)
{
	//最後の出力更新からの経過時間を取得
	double dt = Time::dt(time, mLastUpdateTime);
	mLastUpdateTime = time;

	switch (mDriveMode)
	{
	case DRIVE_PID:
		updatePIDGyroMove();
		break;
	default:
		break;
	}

	//モータ出力を更新
	mMotorL.update(dt);
	mMotorR.update(dt);
}
void MotorDrive::setRatio(int ratioL, int ratioR)
{
	mMotorL.setCoeff((double)(mRatioL = std::max(std::min(ratioL, MOTOR_MAX_POWER), 0)) / MOTOR_MAX_POWER);
	mMotorR.setCoeff((double)(mRatioR = std::max(std::min(ratioR, MOTOR_MAX_POWER), 0)) / MOTOR_MAX_POWER);
}

double MotorDrive::getPowerL()
{
	return mMotorL.getPower();
}
double MotorDrive::getPowerR()
{
	return -mMotorR.getPower();
}

void MotorDrive::drive(int powerL, int powerR)
{
	mDriveMode = DRIVE_RATIO;
	mMotorL.set(mRatioL * powerL / MOTOR_MAX_POWER);
	mMotorR.set(-mRatioR * powerR / MOTOR_MAX_POWER);

	mAngle = 0;
}
void MotorDrive::drive(int power)
{
	drive(power, power);
}

void MotorDrive::setPIDGyro(double p, double i, double d)
{
	Debug::print(LOG_SUMMARY, "PID params: %f %f %f\r\n", p, i, d);
	mPIDGyro.x = p;
	mPIDGyro.y = i;
	mPIDGyro.z = d;
}
void MotorDrive::setPIDPose(double p, double i, double d)
{
	Debug::print(LOG_SUMMARY, "PID params: %f %f %f\r\n", p, i, d);
	mPIDPose.x = p;
	mPIDPose.y = i;
	mPIDPose.z = d;
}
void MotorDrive::drivePIDGyro(double angle, int power, bool reset)
{
	if(reset) mAngle = gGyroSensor.getRz();
	else mAngle = GyroSensor::normalize(angle + mAngle);

	mDrivePower = std::max(std::min(power, MOTOR_MAX_POWER), 0);
	Debug::print(LOG_SUMMARY, "PID(Gyro) is Started (%f, %d)\r\n", mAngle, mDrivePower);
	mDriveMode = DRIVE_PID;

	if(reset)
	{
		mDiff1 = mDiff2 = mDiff3 = 0;
		mControlPower = 0;
	}
}
bool MotorDrive::onCommand(const std::vector<std::string>& args)
{
	int size = args.size();
	if (size == 1)
	{
	  Debug::print(LOG_SUMMARY, "Current Motor Ratio : %d %d\r\n", mMotorL.getPower(), -mMotorR.getPower());
	  Debug::print(LOG_SUMMARY, "Current Motor Pulse : %lld %lld\r\n", mpMotorEncoder->getL(), mpMotorEncoder->getR());
	}
	else if (size >= 2)
	{
		if (args[1].compare("w") == 0)
		{
			//前進
			drive(MOTOR_MAX_POWER, MOTOR_MAX_POWER);
			return true;
		}
		else if (args[1].compare("s") == 0)
		{
			//後退
			drive(-MOTOR_MAX_POWER, -MOTOR_MAX_POWER);
			return true;
		}
		else if (args[1].compare("a") == 0)
		{
			//左折
			drive(0, MOTOR_MAX_POWER * 0.7);
			return true;
		}
		else if (args[1].compare("d") == 0)
		{
			//右折
			drive(MOTOR_MAX_POWER * 0.7, 0);
			return true;
		}
		else if (args[1].compare("h") == 0)
		{
			//停止
			drive(0, 0);
			return true;
		}
		else if (args[1].compare("p") == 0)
		{
			//PID制御関連
			if (size == 2)
			{
				//PID制御開始(現在の向き)
				drivePIDGyro(0, MOTOR_MAX_POWER, true);
				return true;
			}
			else if (size == 3)
			{
				//PID(相対角度指定)
				drivePIDGyro(atoi(args[2].c_str()), MOTOR_MAX_POWER, true);
				return true;
			}
			else if (size == 5)
			{
				//PIDパラメータ設定
				setPIDGyro(atof(args[2].c_str()), atof(args[3].c_str()), atof(args[4].c_str()));
				return true;
			}
		}
		else if (args[1].compare("r") == 0)
		{
			if (size == 4)
			{
				//レシオ設定
				setRatio(atoi(args[2].c_str()), atoi(args[3].c_str()));
				return true;
			}
		}
		else if(args[1].compare("cpose") == 0 && size == 3)
		{
			mMaxPIDControlRatioPose = atof(args[2].c_str());
			return true;
		}
		else if(args[1].compare("cgyro") == 0 && size == 3)
		{
			mMaxPIDControlRatioGyro = atof(args[2].c_str());
			return true;
		}
		else
		{
			if (size == 3)
			{
				drive(atoi(args[1].c_str()), atoi(args[2].c_str()));//出力直接指定
				return true;
			}
		}
	}
	Debug::print(LOG_PRINT, "motor              : show motor state\r\n\
motor [w/s/a/d/h]  : move\r\n\
motor p            : pid start\r\n\
motor p [angle]    : pid start with angle to move\r\n\
motor p [P] [I] [D]: set pid params\r\n\
motor r [l] [r]    : set motor ratio\r\n\
motor [l] [r]      : drive motor by specified ratio\r\n\
motor [cpose/cgyro] [param]   : set max control ratio\r\n");
	return true;
}

long long MotorDrive::getL()
{
	return mpMotorEncoder->getL();
}
long long MotorDrive::getR()
{
	return mpMotorEncoder->getR();
}
long long MotorDrive::getDeltaPulseL()
{
	return mpMotorEncoder->getDeltaPulseL();
}
long long MotorDrive::getDeltaPulseR()
{
	return mpMotorEncoder->getDeltaPulseR();
}
MotorDrive::MotorDrive() : mMotorL(), mMotorR(), mDriveMode(DRIVE_RATIO), mRatioL(100), mRatioR(100), mPIDGyro(0.003, 0, 0), mPIDPose(0.006, 0, 0), mMaxPIDControlRatioGyro(1), mMaxPIDControlRatioPose(0.5), mDiff1(0), mDiff2(0), mDiff3(0), mAngle(0), mControlPower(0), mDrivePower(0)
{
	setName("motor");
	setPriority(TASK_PRIORITY_MOTOR, TASK_INTERVAL_MOTOR);

	mpMotorEncoder = MotorEncoder::getInstance();
}
MotorDrive::~MotorDrive(){}
