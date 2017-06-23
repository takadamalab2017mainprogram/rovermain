#include <wiringPi.h>
#include <softPwm.h>
#include <stdlib.h>
#include "actuator.h"
#include "constants.h"
#include "utils.h"

//////////////////////////////////////////////
// Buzzer
//////////////////////////////////////////////

bool Buzzer::onInit(const struct timespec& time)
{
	pinMode(mPin, OUTPUT);
	digitalWrite(mPin, LOW);
	return true;
}
void Buzzer::onClean()
{
	digitalWrite(mPin, LOW);
}
bool Buzzer::onCommand(const std::vector<std::string>& args)
{
	int period, on_period, off_period, count;
	switch (args.size())
	{
	case 2:
		if (args[1].compare("stop") == 0)		//buzzer stop
		{
			if (mOnPeriod == 0 && mOffPeriod == 0 && mCount == 0)
			{
				Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT, "Stop Command Executed!\r\n");
				mOffPeriod = 0;
				mCount = 1;
				stop();
			}
			return true;
		}
		else									//buzzer [period]
		{
			Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
			period = atoi(args[1].c_str());
			start(period);
			return true;
		}
		break;

	case 3:										//buzzer [period] [count]
		Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
		period = atoi(args[1].c_str());
		count = atoi(args[2].c_str());
		start(period, count);
		return true;
		break;

	case 4:									//buzzer [on oeriod] [off period] [count]
		Debug::print(LOG_PRINT, "Start Command Executed!\r\n");
		on_period = atoi(args[1].c_str());
		off_period = atoi(args[2].c_str());
		count = atoi(args[3].c_str());
		start(on_period, off_period, count);
		return true;
		break;
	default:
		break;
	}

	Debug::print(LOG_PRINT, "buzzer [period]                         : wake buzzer while period\r\n\
													   buzzer [period] [count]                 : wake buzzer several times (COUNT)\r\n\
													   						   buzzer [on period] [off period] [count] : wake buzzer several times (COUNT)\r\n\
																			   						   buzzer stop                             : stop buzzer\r\n");
	return true;
}
void Buzzer::onUpdate(const struct timespec& time)
{
	if (mOffPeriod == 1)		//鳴らさない時間の終了
	{
		restart();
	}
	else if (mOffPeriod > 0)	//鳴らさない時間
	{
		--mOffPeriod;
		return;
	}

	if (mOnPeriod == 1)		//鳴らす時間の終了
	{
		stop();
	}
	else if (mOnPeriod > 0)	//鳴らす時間
	{
		--mOnPeriod;
	}
}
void Buzzer::start(int period)
{
	start(period, 1, 1);
}
void Buzzer::start(int on_period, int count)
{
	start(on_period, DEFAULT_OFF_PERIOD, count);
}
void Buzzer::start(int on_period, int off_period, int count)
{
	if (mOnPeriod == 0 && on_period >= 1 && off_period >= 1 && count >= 1)
	{
		mOnPeriodMemory = on_period;
		mOnPeriod = on_period;
		mOffPeriodMemory = off_period;
		mOffPeriod = 0;
		mCount = count;
		digitalWrite(mPin, HIGH);
	}
}
void Buzzer::restart()
{
	digitalWrite(mPin, HIGH);
	mOnPeriod = mOnPeriodMemory;
	mOffPeriod = 0;
}
void Buzzer::stop()
{
	mOnPeriod = 0;
	digitalWrite(mPin, LOW);

	if (mCount == 1)
	{
		mCount = 0;
	}
	else if (mCount > 0)
	{
		mOffPeriod = mOffPeriodMemory;
		--mCount;
	}
}
Buzzer::Buzzer() : mPin(PIN_BUZZER), mOnPeriodMemory(0), mOnPeriod(0), mOffPeriodMemory(0), mOffPeriod(0), mCount(0)
{
	setName("buzzer");
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_ACTUATOR);
}
Buzzer::~Buzzer()
{
}

//////////////////////////////////////////////
// ParaServo(Hardware PWM)
//////////////////////////////////////////////
bool ParaServo::onInit(const struct timespec& time)
{
	pinMode(mPin, PWM_OUTPUT);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(9000);
	pwmSetClock(32);

	return true;
}
void ParaServo::onClean()
{
	stop();
}
bool ParaServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		if (args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
		else
		{
			//角度指定
			float period = 0;
			if (args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "paraservo [0-1]          : set servo angle\r\n\
paraservo stop           : stop servo\r\n");
	}
	return true;
}
void ParaServo::start(double angle)
{
	if (angle > 1)
		angle = 1;
	else if (angle < -1.5)
		angle = -1.5;
	mAngle = angle;

	angle *= -1;
	double tmp = 0.8*(angle + 1.0) / 2.0;
	pwmWrite(mPin, SERVO_BASE_VALUE + tmp * SERVO_MOVABLE_RANGE);
}
void ParaServo::stop()
{
	pwmWrite(mPin, 0);
}
double ParaServo::get()
{
	return mAngle;
}
void ParaServo::moveRelease()
{
	start(0.0);
}
void ParaServo::moveHold()
{
	start(1.0);
}
ParaServo::ParaServo() : mPin(PIN_PARA_SERVO)
{
	setName("paraservo");
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
ParaServo::~ParaServo()
{
}

////////////////////////////////////////////////
//// ParaServo(Software PWM) 
////////////////////////////////////////////////
//
//bool ParaServo::onInit(const struct timespec& time)
//{
//	//if (wiringPiSetup() == -1)//Software PWMを使う前にwiringPiSetupを呼ぶ必要があるらしい
//	//{
//	//	Debug::print(LOG_PRINT,"ParaServoError: wiringPi setup failed...\n");
//	//}
//	pinMode(mPin, PWM_OUTPUT);
//	if (softPwmCreate(mPin, 0, SERVO_RANGE) != 0)
//	{
//		Debug::print(LOG_PRINT, "Failed to initialize soft-PWM\r\n");
//		return false;
//	}
//	//softPwmCreate(mPin, 0, SERVO_RANGE);	//int softPwmCreate (int pin, int initialValue, int pwmRange);
//	return true;
//}
//void ParaServo::onClean()
//{
//	stop();
//}
//bool ParaServo::onCommand(const std::vector<std::string>& args)
//{
//	if (args.size() >= 2)
//	{
//		if (args[1].compare("stop") == 0)
//		{
//			stop();
//			Debug::print(LOG_PRINT, "Command Executed!\r\n");
//			return true;
//		}
//		else
//		{
//			//角度指定
//			int period = 0;
//			if (args.size() == 2)
//			{
//				period = atof(args[1].c_str());
//			}
//			start(period);
//			Debug::print(LOG_PRINT, "Command Executed!\r\n");
//			return true;
//		}
//	}
//	else
//	{
//		Debug::print(LOG_PRINT, "paraservo [0 or %d-%d]\t: set servo position\r\nparaservo stop\t\t: stop servo\r\n", SERVO_MIN_RANGE, SERVO_MAX_RANGE);
//	}
//	return true;
//}
//void ParaServo::start(int angle)
//{
//	//範囲のチェック
//	if (angle >= SERVO_MAX_RANGE)
//	{
//		//angle = SERVO_MAX_RANGE;
//	}
//	else if (angle <= 0)
//	{
//		//angle = 0;
//	}
//	else if (angle < SERVO_MIN_RANGE)
//	{
//		//angle = SERVO_MIN_RANGE;
//	}
//
//	softPwmWrite(mPin, angle);			//void softPwmWrite (int pin, int value);
//	Debug::print(LOG_PRINT, "ParaServo Start (%d)!\r\n", angle);
//}
//void ParaServo::start(POSITION p)
//{
//	start((int)p);
//}
//void ParaServo::stop()
//{
//	softPwmWrite(mPin, 0);
//}
//void ParaServo::moveRelease()
//{
//	start(POSITION_RELEASE);
//}
//void ParaServo::moveHold()
//{
//	start(POSITION_HOLD);
//}
//ParaServo::ParaServo() : mPin(PIN_PARA_SERVO)
//{
//	setName("paraservo");
//	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
//}
//ParaServo::~ParaServo()
//{
//}

////////////////////////////////////////////////
//// SoftwarePWMServo 
////////////////////////////////////////////////
//
//bool SoftwarePWMServo::onInit(const struct timespec& time)
//{
//	if (wiringPiSetup() == -1)//Software PWMを使う前にwiringPiSetupを呼ぶ必要があるらしい
//	{
//		Debug::print(LOG_PRINT, "SoftCameraServoError: wiringPi setup failed...\n");
//	}
//	pinMode(mPin, PWM_OUTPUT);
//	if (softPwmCreate(mPin, 0, SERVO_RANGE) != 0)
//	{
//		Debug::print(LOG_PRINT, "Failed to initialize soft-PWM\r\n");
//		return false;
//	}
//	//softPwmCreate(mPin, 0, SERVO_RANGE);	//int softPwmCreate (int pin, int initialValue, int pwmRange);
//	return true;
//}
//void SoftwarePWMServo::onClean()
//{
//	stop();
//}
//bool SoftwarePWMServo::onCommand(const std::vector<std::string>& args)
//{
//	if (args.size() >= 2)
//	{
//		if (args[1].compare("stop") == 0)
//		{
//			stop();
//			Debug::print(LOG_PRINT, "Command Executed!\r\n");
//			return true;
//		}
//		else
//		{
//			//角度指定
//			int period = 0;
//			if (args.size() == 2)
//			{
//				period = atof(args[1].c_str());
//			}
//			start(period);
//			Debug::print(LOG_PRINT, "Command Executed!\r\n");
//			return true;
//		}
//	}
//	else
//	{
//		Debug::print(LOG_PRINT, "SoftwarePWMServo [0 or %d-%d]\t: set servo position\r\nSoftwarePWMServo stop\t\t: stop servo\r\n", SERVO_MIN_RANGE, SERVO_MAX_RANGE);
//	}
//	return true;
//}
//void SoftwarePWMServo::start(int angle)
//{
//	//範囲のチェック
//	if (angle >= SERVO_MAX_RANGE)
//	{
//		angle = SERVO_MAX_RANGE;
//	}
//	else if (angle <= 0)
//	{
//		angle = 0;
//	}
//	else if (angle < SERVO_MIN_RANGE)
//	{
//		angle = SERVO_MIN_RANGE;
//	}
//	mAngle = angle;
//	softPwmWrite(mPin, angle);			//void softPwmWrite (int pin, int value);
//	Debug::print(LOG_PRINT, "SoftwarePWMServo Start (%d)!\r\n", angle);
//}
//int SoftwarePWMServo::get()
//{
//	return mAngle;
//}
//void SoftwarePWMServo::stop()
//{
//	softPwmWrite(mPin, 0);
//}
//SoftwarePWMServo::SoftwarePWMServo(const char* name, unsigned int pin) : mPin(pin), mAngle(0)
//{
//	setName(name);
//	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
//}
//SoftwarePWMServo::~SoftwarePWMServo()
//{
//}

//////////////////////////////////////////////
// SoftwarePWMServo 
//////////////////////////////////////////////

bool FrontStabiServo::onInit(const struct timespec& time)
{
	//if (wiringPiSetup() == -1)//Software PWMを使う前にwiringPiSetupを呼ぶ必要があるらしい
	//{
	//	Debug::print(LOG_PRINT, "SoftCameraServoError: wiringPi setup failed...\n");
	//}
	pinMode(mPin, PWM_OUTPUT);
	if (softPwmCreate(mPin, 0, SERVO_RANGE) != 0)
	{
		Debug::print(LOG_PRINT, "Failed to initialize soft-PWM\r\n");
		return false;
	}
	//softPwmCreate(mPin, 0, SERVO_RANGE);	//int softPwmCreate (int pin, int initialValue, int pwmRange);
	return true;
}
void FrontStabiServo::onClean()
{
	stop();
}
bool FrontStabiServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		if (args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
		else
		{
			//角度指定
			int period = 0;
			if (args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "FrontStabiServo [0 or %d-%d]\t: set servo position\r\nFrontStabiServo stop\t\t: stop servo\r\n", SERVO_MIN_RANGE, SERVO_MAX_RANGE);
	}
	return true;
}
void FrontStabiServo::start(int angle)
{
	//範囲のチェック
	if (angle >= SERVO_MAX_RANGE)
	{
		angle = SERVO_MAX_RANGE;
	}
	else if (angle <= 0)
	{
		angle = 0;
	}
	else if (angle < SERVO_MIN_RANGE)
	{
		angle = SERVO_MIN_RANGE;
	}
	mAngle = angle;
	softPwmWrite(mPin, angle);			//void softPwmWrite (int pin, int value);
	//Debug::print(LOG_PRINT, "FrontStabiServo Start (%d)!\r\n", angle);
}
int FrontStabiServo::get()
{
	return mAngle;
}
void FrontStabiServo::stop()
{
	softPwmWrite(mPin, 0);
}
FrontStabiServo::FrontStabiServo() : mPin(PIN_JOHN_SERVO), mAngle(0)
{
	setName("fservo");
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
FrontStabiServo::~FrontStabiServo()
{
}

//////////////////////////////////////////////
// SoftwarePWMServo 
//////////////////////////////////////////////

bool BackStabiServo::onInit(const struct timespec& time)
{
	//if (wiringPiSetup() == -1)//Software PWMを使う前にwiringPiSetupを呼ぶ必要があるらしい
	//{
	//	Debug::print(LOG_PRINT, "SoftCameraServoError: wiringPi setup failed...\n");
	//}
	pinMode(mPin, PWM_OUTPUT);
	if (softPwmCreate(mPin, 0, SERVO_RANGE) != 0)
	{
		Debug::print(LOG_PRINT, "Failed to initialize soft-PWM\r\n");
		return false;
	}
	//softPwmCreate(mPin, 0, SERVO_RANGE);	//int softPwmCreate (int pin, int initialValue, int pwmRange);
	return true;
}
void BackStabiServo::onClean()
{
	stop();
}
bool BackStabiServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		if (args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
		else
		{
			//角度指定
			int period = 0;
			if (args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "BackStabiServo [0 or %d-%d]\t: set servo position\r\nBackStabiServo stop\t\t: stop servo\r\n", SERVO_MIN_RANGE, SERVO_MAX_RANGE);
	}
	return true;
}
void BackStabiServo::start(int angle)
{
	//範囲のチェック
	if (angle >= SERVO_MAX_RANGE)
	{
		angle = SERVO_MAX_RANGE;
	}
	else if (angle <= 0)
	{
		angle = 0;
	}
	else if (angle < SERVO_MIN_RANGE)
	{
		angle = SERVO_MIN_RANGE;
	}
	mAngle = angle;
	softPwmWrite(mPin, angle);			//void softPwmWrite (int pin, int value);
	//Debug::print(LOG_PRINT, "BackStabiServo Start (%d)!\r\n", angle);
}
int BackStabiServo::get()
{
	return mAngle;
}
void BackStabiServo::stop()
{
	softPwmWrite(mPin, 0);
}
BackStabiServo::BackStabiServo() : mPin(PIN_MIKE_SERVO), mAngle(0)
{
	setName("bservo");
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
BackStabiServo::~BackStabiServo()
{
}

//////////////////////////////////////////////
// SoftwarePWMServo 
//////////////////////////////////////////////
bool ArmServo::onInit(const struct timespec& time)
{
	//if (wiringPiSetup() == -1)//Software PWMを使う前にwiringPiSetupを呼ぶ必要があるらしい
	//{
	//	Debug::print(LOG_PRINT,"ArmServoError: wiringPi setup failed...\n");
	//}
	pinMode(mPin, PWM_OUTPUT);
	if (softPwmCreate(mPin, 0, SERVO_RANGE) != 0)
	{
		Debug::print(LOG_PRINT, "Failed to initialize soft-PWM\r\n");
		return false;
	}
	//softPwmCreate(mPin, 0, SERVO_RANGE);	//int softPwmCreate (int pin, int initialValue, int pwmRange);
	return true;
}
void ArmServo::onClean()
{
	stop();
}
bool ArmServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		if (args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
		else
		{
			//角度指定
			int period = 0;
			if (args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "Armservo [0 or %d-%d]\t: set servo position\r\nArmservo stop\t\t: stop servo\r\n", SERVO_MIN_RANGE, SERVO_MAX_RANGE);
	}
	return true;
}
void ArmServo::start(int angle)
{
	//範囲のチェック
	if (angle >= SERVO_MAX_RANGE)
	{
		//angle = SERVO_MAX_RANGE;
	}
	else if (angle <= 0)
	{
		//angle = 0;
	}
	else if (angle < SERVO_MIN_RANGE)
	{
		//angle = SERVO_MIN_RANGE;
	}
	mAngle = angle;
	softPwmWrite(mPin, angle);			//void softPwmWrite (int pin, int value);
	Debug::print(LOG_PRINT, "ArmServo Start (%d)!\r\n", angle);
}
void ArmServo::start(POSITION p)
{
	start((int)p);
}
void ArmServo::stop()
{
	softPwmWrite(mPin, 0);
}
int ArmServo::get()
{
	return mAngle;
}
void ArmServo::moveRelease()
{
	start(POSITION_RELEASE);
}
void ArmServo::moveHold()
{
	start(POSITION_HOLD);
}
ArmServo::ArmServo() : mPin(PIN_ARM_SERVO)
{
	setName("armservo");
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
ArmServo::~ArmServo()
{
}

//////////////////////////////////////////////
// HardwarePWMServo(Hardware PWM)
//////////////////////////////////////////////

bool NeckServo::onInit(const struct timespec& time)
{
	pinMode(mPin, PWM_OUTPUT);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(9000);
	pwmSetClock(32);

	return true;
}
void NeckServo::onClean()
{
	stop();
}
bool NeckServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		if (args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
		else
		{
			//角度指定
			float period = 0;
			if (args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "jservo [0-1]          : set servo angle\r\n\
jservo stop           : stop servo\r\n");
	}
	return true;
}
void NeckServo::start(double angle)
{
	if (angle > 1)
		angle = 1;
	else if (angle < -1.5)
		angle = -1.5;
	mAngle = angle;

	angle *= -1;
	double tmp = 0.8*(angle + 1.0) / 2.0;
	pwmWrite(mPin, SERVO_BASE_VALUE + tmp * SERVO_MOVABLE_RANGE);
}
void NeckServo::stop()
{
	pwmWrite(mPin, 0);
}
double NeckServo::get()
{
	return mAngle;
}
NeckServo::NeckServo(const char* name, unsigned int pin) : mPin(pin), mAngle(0)
{
	setName(name);
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
NeckServo::~NeckServo()
{
}


bool SServo::onInit(const struct timespec& time)
{
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	return true;
}
void SServo::onClean()
{
}
void SServo::startJohn(int j)
{
	gJohnServo.start(j);
}
void SServo::startMike(int m)
{
	gMikeServo.start(m);
}
void SServo::startArm(int a)
{
	gArmServo.start(a);
}
void SServo::startNeck(double n)
{
	gNeckServo.start(n);
}
void SServo::start(int j, int m, int a, double n)
{
	startJohn(j);
	startMike(m);
	startArm(a);
	startNeck(n);
}
void SServo::stop()
{
	gJohnServo.stop();
	gMikeServo.stop();
	gArmServo.stop();
	gNeckServo.stop();
}
void SServo::moveRun()
{
	start(std::get<0>(mRunAngle), std::get<1>(mRunAngle), std::get<2>(mRunAngle), std::get<3>(mRunAngle));
}

void SServo::moveFold()
{
	start(std::get<0>(mFoldAngle), std::get<1>(mFoldAngle), std::get<2>(mFoldAngle), std::get<3>(mFoldAngle));
}

bool SServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if(args[1].compare("stop") == 0)
		{
			stop();
		}else if(args[1].compare("run") == 0)
		{
			moveRun();
		}else if(args[1].compare("fold") == 0)
		{
			moveFold();
		}
		
		return true;
	}
	else if (args.size() == 3)
	{
		if (args[1].compare("j") == 0 || args[1].compare("J") == 0)
			start(atoi(args[2].c_str()), gMikeServo.get(), gArmServo.get(), gNeckServo.get());
		else if (args[1].compare("m") == 0 || args[1].compare("M") == 0)
			start(gJohnServo.get(), atoi(args[2].c_str()), gArmServo.get(), gNeckServo.get());
		else if (args[1].compare("a") == 0 || args[1].compare("A") == 0)
			start(gJohnServo.get(), gMikeServo.get(), atoi(args[2].c_str()), gNeckServo.get());
		else if (args[1].compare("n") == 0 || args[1].compare("N") == 0)
			start(gJohnServo.get(), gMikeServo.get(), gArmServo.get(), atof(args[2].c_str()));
		return true;
	}
	else if (args.size() == 6)
	{
		if(args[1].compare("run") == 0)
		{
			int j = atoi(args[2].c_str()), m = atoi(args[3].c_str()), a = atoi(args[4].c_str());
			double n = atof(args[5].c_str());
			std::get<0>(mRunAngle) = j;
			std::get<1>(mRunAngle) = m;
			std::get<2>(mRunAngle) = a;
			std::get<3>(mRunAngle) = n;
			Debug::print(LOG_SUMMARY, " %s angle is updated to %d, %d, %d, %f\r\n", args[1].c_str(), j, m);
			return true;
		}else if(args[1].compare("fold") == 0)
		{
			int j = atoi(args[2].c_str()), m = atoi(args[3].c_str()), a = atoi(args[4].c_str());
			double n = atof(args[5].c_str());
			std::get<0>(mFoldAngle) = j;
			std::get<1>(mFoldAngle) = m;
			std::get<2>(mFoldAngle) = a;
			std::get<3>(mFoldAngle) = n;
			Debug::print(LOG_SUMMARY, " %s angle is updated to %d, %d, %d, %f\r\n", args[1].c_str(), j, m);

			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "jmans          : set servo angle\r\n\
jmans [0:100] [0:100] [0:100] [-1:1]    : set JMservo angle\r\n\
jmans m [0:100]           : set Mike servo angle\r\n\
jmans j [0:100]           : set John servo angle\r\n\
jmans a [0:100]           : set Arm servo angle\r\n\
jmans n [-1:1]           : set Neck servo angle\r\n\
jmans {stop/run/fold} : set LRservo angle as in specified state\r\n\
jmans {stop/run/fold} [0:100] [0:100] [0:100] [-1:1] : set angle of specified state\r\n");
	}
	return false;
}

SServo::SServo() : mRunAngle(15, 6, 5, -0.1), mFoldAngle(1, 20, 15, 1)
{
	setName("jmans");
	setPriority(TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
SServo::~SServo()
{
}

Buzzer gBuzzer;
ParaServo gParaServo;
ArmServo gArmServo;
FrontStabiServo gJohnServo;
BackStabiServo gMikeServo;
NeckServo gNeckServo("neckservo", PIN_NECK_SERVO);
SServo gSServo;
