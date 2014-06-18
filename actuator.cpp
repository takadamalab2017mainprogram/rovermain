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
	if(args.size() >= 2)
	{
		if(args[1].compare("stop") == 0)
		{
			stop();
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			return true;
		}else if(args.size() == 2)
		{
			int period = atoi(args[1].c_str());
			start(period);
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
		}
	}else{
			Debug::print(LOG_PRINT,"buzzer [period]       : wake buzzer while period\r\n\
buzzer stop           : stop buzzer\r\n");
	}
	return true;
}
void Buzzer::onUpdate(const struct timespec& time)
{
	if(mPeriod == 1)stop();
	if(mPeriod > 0)--mPeriod;
}
void Buzzer::start(int period)
{
	if(mPeriod == 0 && period > 1)Debug::print(LOG_DETAIL,"Buzzer Start!\r\n");
	mPeriod = period;
	digitalWrite(mPin, HIGH);
}
void Buzzer::stop()
{
	mPeriod = 0;
	digitalWrite(mPin, LOW);
	Debug::print(LOG_DETAIL,"Buzzer Stop!\r\n");
}
Buzzer::Buzzer() : mPin(PIN_BUZZER),mPeriod(0)
{
	setName("buzzer");
	setPriority(TASK_PRIORITY_ACTUATOR,TASK_INTERVAL_ACTUATOR);
}
Buzzer::~Buzzer()
{
}

//////////////////////////////////////////////
// Servo
//////////////////////////////////////////////

bool Servo::onInit(const struct timespec& time)
{
	pinMode(mPin, PWM_OUTPUT);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(9000);
	pwmSetClock(32);

	pwmWrite (mPin, 0);
	return true;
}
void Servo::onClean()
{
	stop();
}
bool Servo::onCommand(const std::vector<std::string> args)
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
void Servo::start(double angle)
{
	if(angle > 1)angle = 1;
	else if(angle < 0)angle = 0;

	pwmWrite (mPin, SERVO_BASE_VALUE + angle * SERVO_MOVABLE_RANGE);
	Debug::print(LOG_DETAIL,"Servo Start (%f)!\r\n",angle);
}
void Servo::stop()
{
	pwmWrite (mPin, 0);
	Debug::print(LOG_DETAIL,"Servo Stop!\r\n");
}
Servo::Servo() : mPin(PIN_SERVO)
{
	setName("servo");
	setPriority(TASK_PRIORITY_ACTUATOR,UINT_MAX);
}
Servo::~Servo()
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
Servo gServo;
XBeeSleep gXbeeSleep;
