#include <wiringPi.h>
#include <softPwm.h>
#include <stdlib.h>
#include "actuator.h"
#include "constants.cpp"
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
Buzzer::Buzzer() : mPin(Constants::PIN_BUZZER), mOnPeriodMemory(0), mOnPeriod(0), mOffPeriodMemory(0), mOffPeriod(0), mCount(0)
{
	setName("buzzer");
	setPriority(Constants::TASK_PRIORITY_ACTUATOR, Constants::TASK_INTERVAL_ACTUATOR);
}
Buzzer::~Buzzer()
{
}

//////////////////////////////////////////////
// MultiServo(Hardware PWM)
//////////////////////////////////////////////
bool MultiServo::onInit(const struct timespec& time)
{
	//Pin番号をセットして出力状態に
	pinMode(mPin, PWM_OUTPUT);
	//サーボの制御にはmark:spacemode(MS)を使う
	pwmSetMode(PWM_MODE_MS);
	//範囲レジスタを設定、デフォルトは1024
	pwmSetRange(9000);
	pwmSetClock(32);

	return true;
}
void MultiServo::onClean()
{
	stop();
}
bool MultiServo::onCommand(const std::vector<std::string>& args)
{
	if (args.size() >= 2)
	{
		//stopと入力したらサーボに入れる力が0になる
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
				//入力した角度をdouble型に変換
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "multiservo [0-1]          : set servo angle\r\n\
multiservo stop           : stop servo\r\n");
	}
	return true;
}
void MultiServo::start(double angle)
{
	if (angle > 1)
		angle = 1;
	else if (angle < -1.5)
		angle = -1.5;
	mAngle = angle;

	angle *= -1;
	double tmp = 0.8*(angle + 1.0) / 2.0;
	pwmWrite(mPin, Constants::SERVO_BASE_VALUE + tmp * Constants::SERVO_MOVABLE_RANGE);
}
void MultiServo::stop()
{
	pwmWrite(mPin, 0);
}
double MultiServo::get()
{
	return mAngle;
}
//パラ切り離し時使用
void MultiServo::moveRelease()
{
	start(Constants::STABI_RELEASE_ANGLE);
}
//パラ切り離し時使用
void MultiServo::moveHold()
{
	start(Constants::STABI_HOLD_ANGLE);
}

void MultiServo::Running()
{
	start(Constants::STABI_RUNNING_ANGLE);
}

void MultiServo::fold() 
{
	start(Constants::STABI_FOLD_ANGLE);
}
MultiServo::MultiServo() : mPin(Constants::PIN_MULTI_SERVO)
{
	setName("multiservo");
	setPriority(Constants::TASK_PRIORITY_ACTUATOR, UINT_MAX);
}
MultiServo::~MultiServo()
{
}




Buzzer gBuzzer;
MultiServo gMultiServo;

