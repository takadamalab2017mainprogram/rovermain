#include <wiringPi.h>
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
bool Buzzer::onCommand(const std::vector<std::string> args)
{
	if(args.size() < 2)
	{
			Debug::print(LOG_PRINT,"buzzer [period]                         : wake buzzer while period\r\n\
buzzer [period] [count]                 : wake buzzer several times (COUNT)\r\n\
buzzer [on period] [off period] [count] : wake buzzer several times (COUNT)\r\n\
buzzer stop                             : stop buzzer\r\n");
		return true;
	}

	int period, on_period, off_period, count;
	switch(args.size())
	{
		case 2:
			if(args[1].compare("stop") == 0)	//buzzer stop
			{
				stop();
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
			}
			else								//buzzer [period]
			{
				period = atoi(args[1].c_str());
				start(period);
				Debug::print(LOG_PRINT,"Command Executed!\r\n");
			}
			break;
			
		case 3:									//buzzer [period] [count]
			period = atoi(args[1].c_str());
			count  = atoi(args[2].c_str());
			start(period, count);
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			break;
			
		case 4:									//buzzer [on oeriod] [off period] [count]
			on_period  = atoi(args[1].c_str());
			off_period = atoi(args[2].c_str());
			count  	   = atoi(args[3].c_str());
			start(on_period, off_period, count);
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			break;
		default:
			break;
	}
	return true;
}
void Buzzer::onUpdate(const struct timespec& time)
{
	if(mOffPeriod == 1)		//鳴らさない時間の終了
	{
		restart();
	}
	else if(mOffPeriod > 0)	//鳴らさない時間
	{
		--mOffPeriod;
		return;
	}

	if(mOnPeriod == 1)		//鳴らす時間の終了
	{
		stop();
	}
	else if(mOnPeriod > 0)	//鳴らす時間
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
	if(mOnPeriod == 0 && on_period > 1 && off_period > 1 && count >= 1)
	{
		Debug::print(LOG_DETAIL,"Buzzer Start!\r\n");
		mOnPeriodMemory  = on_period;
		mOnPeriod		 = on_period;
		mOffPeriodMemory = off_period;
		mOffPeriod		 = 0;
		mCount			 = count;
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

	if(mCount == 1)
	{
		Debug::print(LOG_DETAIL,"Buzzer Stop!\r\n");
		mCount = 0;
	}
	else if(mCount > 0)
	{
		mOffPeriod = mOffPeriodMemory;
		--mCount;
	}
}
Buzzer::Buzzer() : mPin(PIN_BUZZER),mOnPeriodMemory(0),mOnPeriod(0),mOffPeriodMemory(0),mOffPeriod(0),mCount(0)
{
	setName("buzzer");
	setPriority(TASK_PRIORITY_ACTUATOR,TASK_INTERVAL_ACTUATOR);
}
Buzzer::~Buzzer()
{
}

//////////////////////////////////////////////
// ParaServo
//////////////////////////////////////////////

bool ParaServo::onInit(const struct timespec& time)
{
	softPwmCreate(mPin, 0, 100);
	softpwmWrite(mPin,0);
	return true;
}
void ParaServo::onClean()
{
	stop();
}
bool ParaServo::onCommand(const std::vector<std::string> args)
{
	if(args.size() >= 2)
	{
		if(args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}else
		{
			//角度指定
			float period = 0;
			if(args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}
	}else
	{
		Debug::print(LOG_PRINT,"servo [0-1]          : set servo angle\r\n\
servo stop           : stop servo\r\n");
	}
	return true;
}
void ParaServo::start(double angle)
{
	if(angle > 1)angle = 1;
	else if(angle < 0)angle = 0;

	softpwmWrite (mPin, angle * 100);
	Debug::print(LOG_DETAIL,"ParaServo Start (%f)!\r\n",angle);
}
void ParaServo::stop()
{
	pwmWrite (mPin, 0);
	Debug::print(LOG_DETAIL,"ParaServo Stop!\r\n");
}
Servo::Servo() : mPin(PIN_PARA_SERVO)
{
	setName("paraservo");
	setPriority(TASK_PRIORITY_ACTUATOR,UINT_MAX);
}
Servo::~Servo()
{
}

//////////////////////////////////////////////
// StabiServo
//////////////////////////////////////////////

bool StabiServo::onInit(const struct timespec& time)
{
	pinMode(mPin, PWM_OUTPUT);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(9000);
	pwmSetClock(32);

	pwmWrite (mPin, 0);
	return true;
}
void StabiServo::onClean()
{
	stop();
}
bool StabiServo::onCommand(const std::vector<std::string> args)
{
	if(args.size() >= 2)
	{
		if(args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}else if(args[1].compare("close") == 0)
		{
			close();
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}else
		{
			//角度指定
			float period = 0;
			if(args.size() == 2)
			{
				period = atof(args[1].c_str());
			}
			start(period);
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}
	}else
	{
		Debug::print(LOG_PRINT,"servo [0-1]          : set servo angle\r\n\
servo stop           : stop servo\r\n");
	}
	return true;
}
void StabiServo::start(double angle)
{
	if(angle > 1)angle = 1;
	else if(angle < 0)angle = 0;

	pwmWrite (mPin, SERVO_BASE_VALUE + angle * SERVO_MOVABLE_RANGE);
	Debug::print(LOG_DETAIL,"StabiServo Start (%f)!\r\n",angle);
}
void StabiServo::stop()
{
	pwmWrite (mPin, 0);
	Debug::print(LOG_DETAIL,"StabiServo Stop!\r\n");
}
void StabiServo::close()
{
	pwmWrite(mPin, SERVO_BASE_VALUE);
	Debug::print(LOG_DETAIL,"StabiServo Close!\r\n");
}
StabiServo::StabiServo() : mPin(PIN_STABI_SERVO)
{
	setName("stabiservo");
	setPriority(TASK_PRIORITY_ACTUATOR,UINT_MAX);
}
StabiServo::~StabiServo()
{
}

//////////////////////////////////////////////
// XBee Sleep
//////////////////////////////////////////////

bool XBeeSleep::onInit(const struct timespec& time)
{
	mPin = PIN_XBEE_SLEEP;
	pinMode(mPin, OUTPUT);
	digitalWrite(mPin, LOW);
	return true;
}
void XBeeSleep::onClean()
{
	digitalWrite(mPin, LOW);
}
bool XBeeSleep::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("sleep") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			setState(true);
			return true;
		}else if(args[1].compare("wake") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			setState(false);
			return true;
		}
	}else
	{
		Debug::print(LOG_PRINT,"xbee [sleep/wake] : set sleep mode\r\n");
	}
	return true;
}
void XBeeSleep::setState(bool sleep)
{
	digitalWrite(mPin, sleep ? HIGH : LOW);
}
XBeeSleep::XBeeSleep() : mPin(PIN_XBEE_SLEEP)
{
	setName("xbee");
	setPriority(TASK_PRIORITY_ACTUATOR,UINT_MAX);
}
XBeeSleep::~XBeeSleep()
{
}

Buzzer gBuzzer;
ParaServo gParaServo;
StabiServo gStabiServo;
XBeeSleep gXbeeSleep;
