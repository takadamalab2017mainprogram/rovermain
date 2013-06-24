#include <wiringPi.h>
#include <softPwm.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <pthread.h>
#include "debug.h"
#include "motor.h"
#include "sensor.h"

MotorDrive gMotorDrive;

bool Motor::init(int powPin, int revPin)
{
	//ピン番号を確認
    VERIFY(powPin < 0 || revPin < 0);

	//ピンを初期化
    mPowerPin = powPin;
    mReversePin = revPin;
    pinMode(mPowerPin, OUTPUT);
    if(softPwmCreate(mPowerPin ,0,100) != 0)
	{
		Debug::print(LOG_SUMMARY,"Failed to initialize soft-PWM\r\n");
		return false;
	}
    pinMode(mReversePin, OUTPUT);
    digitalWrite(mReversePin, LOW);

	//現在の出力を保持
    mCurPower = 0;
    return true;
}
void Motor::update(double elapsedSeconds)
{
	if(fabs(mCurPower - mTargetPower) > 0.5)//目標出力と現在出力に差がある場合
	{
		//なめらかにモータ出力を変化させる
		double curFrameTarget = mTargetPower;//この呼び出しで設定するモーター出力

		double maxMotorPowerChange = MOTOR_MAX_POWER_CHANGE * mCoeff * elapsedSeconds;

		//モータ出力変化量を制限
		if(fabs(mTargetPower - mCurPower) > maxMotorPowerChange)
		{
			curFrameTarget = mCurPower;
			curFrameTarget += ((mTargetPower > mCurPower) ? maxMotorPowerChange : -maxMotorPowerChange);
			Debug::print(LOG_DETAIL,"MOTOR power Limitation %f %f(%d) \r\n",mCurPower,curFrameTarget,mTargetPower);
		}

		//新しいpowerをもとにpinの状態を設定する
		if(curFrameTarget < 0 && mCurPower >= 0)digitalWrite(mReversePin, HIGH);
		else if(curFrameTarget > 0 && mCurPower <= 0)digitalWrite(mReversePin, LOW);
		mCurPower = curFrameTarget;
		softPwmWrite(mPowerPin, fabs(mCurPower));
	}
}
void Motor::set(int power)
{
	//値の範囲をチェックし、正しい範囲に丸める
	if(power > MOTOR_MAX_POWER)power = MOTOR_MAX_POWER;
	else if(power < -MOTOR_MAX_POWER)power = -MOTOR_MAX_POWER;
	
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
Motor::Motor() : mPowerPin(-1), mReversePin(-1), mCurPower(0), mTargetPower(0), mCoeff(1)
{
}
Motor::~Motor()
{
    if(mPowerPin >= 0)digitalWrite(mPowerPin, LOW);
}
MotorEncoder* MotorEncoder::getInstance()
{
	static MotorEncoder singleton;
	return &singleton;
}
void MotorEncoder::pulseLCallback()
{
	MotorEncoder::getInstance()->mPulseCountL++;
}
void MotorEncoder::pulseRCallback()
{
	MotorEncoder::getInstance()->mPulseCountR++;
}
bool MotorEncoder::init()
{
	mEncoderPinL = PIN_PULSE_B;
	mEncoderPinR = PIN_PULSE_A;

	mPulseCountL = mPulseCountR = 0;

	//ピンのパルスを監視する
	if(wiringPiISR(mEncoderPinL, INT_EDGE_RISING, pulseLCallback) == -1 || wiringPiISR(mEncoderPinR, INT_EDGE_RISING, pulseRCallback) == -1)
	{
		Debug::print(LOG_SUMMARY,"Failed to onInitialize Motor encoder\r\n");
		return false;
	}
	return true;
}
void MotorEncoder::clean()
{
	//両方のピンの割り込みを無効にする
	char command [64];
	sprintf (command, "/usr/local/bin/gpio edge %d none", mEncoderPinL) ;
    system (command) ;
	sprintf (command, "/usr/local/bin/gpio edge %d none", mEncoderPinR) ;
    system (command) ;

	//スレッドが複数残ることを防止するためsleep
	delay(100);
}
long long MotorEncoder::getL()
{
	return mPulseCountL;
}
long long MotorEncoder::getR()
{
	return mPulseCountR;
}


MotorEncoder::MotorEncoder() : mEncoderPinL(-1),mEncoderPinR(-1),mPulseCountL(-1),mPulseCountR(-1)
{
}
MotorEncoder::~MotorEncoder()
{
	clean();
}

bool MotorDrive::onInit()
{
    if(!mMotorR.init(PIN_PWM_A,PIN_INVERT_MOTOR_A) || !mMotorL.init(PIN_PWM_B,PIN_INVERT_MOTOR_B))
	{
		Debug::print(LOG_SUMMARY,"Failed to initialize Motors\r\n");
		return false;
	}
	if(!mpMotorEncoder->init())
	{
		Debug::print(LOG_SUMMARY,"Failed to initialize Motor Encoders\r\n");
		return false;
	}
	if(clock_gettime(CLOCK_MONOTONIC_RAW,&mLastUpdateTime) != 0)
	{
		Debug::print(LOG_SUMMARY,"Unable to get time!\r\n");
	}
	Debug::print(LOG_DETAIL,"MotorDrive is Ready!\r\n");
    return true;
}

void MotorDrive::onClean()
{
	mpMotorEncoder->clean();
}

void MotorDrive::onUpdate()
{
	//最後の出力更新からの経過時間を取得
	double dt = 0;
	struct timespec newTime;
	if(clock_gettime(CLOCK_MONOTONIC_RAW,&newTime) == 0)
	{
		dt = ((double)(newTime.tv_sec - mLastUpdateTime.tv_sec) * 1000000000 + newTime.tv_nsec - mLastUpdateTime.tv_nsec) / 1000000000.0;
		mLastUpdateTime = newTime;
	}

	if(mDriveMode == DRIVE_PID)
	{
		//ずれ情報を更新
		mDiff3 = mDiff2;mDiff2 = mDiff1;mDiff1 = gGyroSensor.normalize(gGyroSensor.getRz() - mAngle);

		//ずれ情報を元に新しいモーター出力を設定(PID)
		double powerDiff = mP * (mDiff1 - mDiff2) + mI * mDiff1 + mD * ((mDiff1 - mDiff2) - (mDiff2 - mDiff3));
		mControlPower += powerDiff;

		//モータ速度係数を用意
		double drivePowerRatio = (double)mDrivePower / MOTOR_MAX_POWER;//モータ出力の割合

		//モータの逆回転をせずに方向転換する
		double controlRatio = 1 - fabs(mControlPower);
		if(controlRatio <= 0)controlRatio = 0;

		//モータ出力を適用
		if(mControlPower > 0)
		{
			//左に曲がる
			mMotorL.set(mRatioL * drivePowerRatio);
			mMotorR.set(-mRatioR * controlRatio * drivePowerRatio);
		}else
		{
			//右に曲がる
			mMotorL.set(mRatioL * controlRatio * drivePowerRatio);
			mMotorR.set(-mRatioR * drivePowerRatio);
		}
	}
	
	//モータ出力を更新
	mMotorL.update(dt);
	mMotorR.update(dt);
}
void MotorDrive::setRatio(int ratioL,int ratioR)
{
	if(ratioL <= MOTOR_MAX_POWER && ratioL > 0)mMotorL.setCoeff((double)(mRatioL = ratioL) / MOTOR_MAX_POWER);
	if(ratioR <= MOTOR_MAX_POWER && ratioR > 0)mMotorR.setCoeff((double)(mRatioR = ratioR) / MOTOR_MAX_POWER);
}

void MotorDrive::drive(int powerL, int powerR)
{
	mDriveMode = DRIVE_RATIO;
	Debug::print(LOG_DETAIL,"Motor ratio: %d %d\r\n",powerL,powerR);
    mMotorL.set(mRatioL * powerL / MOTOR_MAX_POWER);
    mMotorR.set(-mRatioR * powerR / MOTOR_MAX_POWER);
}

void MotorDrive::set(double p,double i,double d)
{
	Debug::print(LOG_SUMMARY, "PID params: %f %f %f\r\n",p,i,d);
	mP = p;
	mI = i;
	mD = d;
}

void MotorDrive::startPID(int angle,int power)
{
	Debug::print(LOG_SUMMARY, "PID is Started!!\r\n");
	mDiff1 = mDiff2 = mDiff3 = 0;
	mAngle = angle;
	mControlPower = 0;
	mDrivePower = power;
	mDriveMode = DRIVE_PID;
	gGyroSensor.setZero();
}

bool MotorDrive::onCommand(const std::vector<std::string> args)
{
	int size = args.size();
	if(size == 1)
	{
		Debug::print(LOG_PRINT, "Current Motor Ratio: %d %d\r\n",mMotorL.getPower(),-mMotorR.getPower());
		Debug::print(LOG_PRINT, "Current Motor Pulse: %lld %lld\r\n",mpMotorEncoder->getL(),mpMotorEncoder->getR());
	}else if(size >= 2)
	{
		if(args[1].compare("w") == 0)
		{
			//前進
			drive(MOTOR_MAX_POWER,MOTOR_MAX_POWER);
			return true;
		}else if(args[1].compare("s") == 0)
		{
			//後退
			drive(-MOTOR_MAX_POWER,-MOTOR_MAX_POWER);
			return true;
		}else if(args[1].compare("a") == 0)
		{
			//左折
			drive(0,MOTOR_MAX_POWER * 0.7);
			return true;
		}else if(args[1].compare("d") == 0)
		{
			//右折
			drive(MOTOR_MAX_POWER * 0.7,0);
			return true;
		}else if(args[1].compare("h") == 0)
		{
			//停止
			drive(0,0);
			return true;
		}else if(args[1].compare("p") == 0)
		{
			//PID制御関連
			if(size == 2)
			{
				//PID制御開始(現在の向き)
				startPID(0,MOTOR_MAX_POWER);
				return true;
			}else if(size == 3)
			{
				//PID(相対角度指定)
				startPID(atoi(args[2].c_str()),MOTOR_MAX_POWER);
				return true;
			}else if(size == 5)
			{
				//PIDパラメータ設定
				set(atof(args[2].c_str()),atof(args[3].c_str()),atof(args[4].c_str()));
				return true;
			}
		}else if(args[1].compare("r") == 0)
		{
			//レシオ設定
			setRatio(atoi(args[2].c_str()),atoi(args[3].c_str()));
			return true;
		}else
		{
			if(size == 3)
			{
				drive(atoi(args[1].c_str()),atoi(args[2].c_str()));//出力直接指定
				return true;
			}
		}
	}
	Debug::print(LOG_PRINT, "motor [w/s/a/d/h]  : move\r\n\
motor p            : pid start\r\n\
motor p [angle]    : pid start with angle to move\r\n\
motor p [P] [I] [D]: set pid params\r\n\
motor r [l] [r]    : set motor ratio\r\n\
motor [l] [r]      : drive motor by specified ratio\r\n");
	return true;
}
MotorDrive::MotorDrive() : mDriveMode(DRIVE_RATIO),mRatioL(100),mRatioR(100),mP(0),mI(0),mD(0),mDiff1(0),mDiff2(0),mDiff3(0),mAngle(0),mControlPower(0),mDrivePower(0)
{
	setName("motor");
	setPriority(TASK_PRIORITY_MOTOR,TASK_INTERVAL_MOTOR);

	mpMotorEncoder = MotorEncoder::getInstance();
}
MotorDrive::~MotorDrive(){}
