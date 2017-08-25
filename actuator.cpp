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


//////////////////////////////
// フルカラーLED
//////////////////////////////
bool LED::onInit(const struct timespec& time) {
	pinMode(PIN_LED_R, PWM_OUTPUT);
	pinMode(PIN_LED_G, PWM_OUTPUT);
	pinMode(PIN_LED_B, PWM_OUTPUT);
	softPwmCreate(PIN_LED_R, 0, 256);
	softPwmCreate(PIN_LED_G, 0, 256);
	softPwmCreate(PIN_LED_B, 0, 256);
	digitalWrite(PIN_LED_R, LOW);
	digitalWrite(PIN_LED_G, LOW);
	digitalWrite(PIN_LED_B, LOW);
	mLastUpdateTime1 = mLastUpdateTime2 = time;
	r = g = b = t = s = u = v = p = d = 0;
	rbw = bnk = hf = false;
	return true;
}

void LED::onClean() {
	digitalWrite(PIN_LED_R, LOW);
	digitalWrite(PIN_LED_G, LOW);
	digitalWrite(PIN_LED_B, LOW);
	rbw = false;
}

bool LED::onCommand(const std::vector<std::string>& args) {
	switch (args.size()) {
	case 1:
		Debug::print(LOG_PRINT, "(R,G,B)=(%d,%d,%d)\r\n", r, g, b);
		if (bnk)
			Debug::print(LOG_PRINT, "Brink=true (%f,%f)\r\n", u, v);
		else
			Debug::print(LOG_PRINT, "Brink=false\r\n");
		if (rbw)
			Debug::print(LOG_PRINT, "Rainbow=true (%f)\r\n", s);
		else
			Debug::print(LOG_PRINT, "Rainbow=false\r\n");
		if (hf)
			Debug::print(LOG_PRINT, "HSV=true (%f)\r\n", p);
		else
			Debug::print(LOG_PRINT, "HSV=false\r\n");

		Debug::print(LOG_PRINT, "led : Show led status\r\nled s : Stop lightning LED\r\nled test : = led 255 255 255\r\nled rainbow [seconds] : Rainbow w/ [seconds]\r\nled rainbow s : Stop rainbow\r\nled brink s : Stop brinking\r\nled brink [seconds] : Brink w/ [seconds]\r\nled brink [sec1] [sec2] : Brink w/ sec1 on / sec2 off\r\nled hsv [sec] : hsv w/ [sec]\r\nled hsv s : stop hsv\r\nled [R] [G] [B] : Light LED w/ RGB in 256 steps (0-255)\r\n");
		return true;
		break;
	case 2:
		if (args[1].compare("s") == 0) {
			clearLED();
			return true;
		}
		else if (args[1].compare("test") == 0) {
			setColor(255, 255, 255);
			return true;
		}
		break;
	case 3:
		if (args[1].compare("rainbow") == 0) {
			if (args[2].compare("s") == 0) {
				stopRainbow();
				Debug::print(LOG_PRINT, "stop rainbow\r\n");
				return true;
			}
			else {
				rainbow(atof(args[2].c_str()));
				Debug::print(LOG_PRINT, "rainbow(%f)\r\n", s);
				return true;
			}
		}
		else if (args[1].compare("brink") == 0) {
			if (args[2].compare("s") == 0) {
				stopBrink();
				Debug::print(LOG_PRINT, "stop brink\r\n");
				return true;
			}
			else {
				brink(atof(args[2].c_str()));
				Debug::print(LOG_PRINT, "brink(%f)\r\n", u);
				return true;
			}
		}
		else if (args[1].compare("hsv") == 0) {
			if (args[2].compare("s") == 0) {
				stopHSV();
				Debug::print(LOG_PRINT, "stop hsv\r\n");
				return true;
			}
			else {
				startHSV(atof(args[2].c_str()));
				Debug::print(LOG_PRINT, "hsv(%f)\r\n", atof(args[2].c_str()));
				return true;
			}
		}

		break;
	case 4:
		if (args[1].compare("brink") == 0) {
			brink(atof(args[2].c_str()), atof(args[3].c_str()));
			Debug::print(LOG_PRINT, "brink(%f,%f)\r\n", u, v);
			return true;
		}
		else {
			r = (int)atof(args[1].c_str());
			if (r>255)
				r = 255;
			g = (int)atof(args[2].c_str());
			if (g>255)
				g = 255;
			b = (int)atof(args[3].c_str());
			if (b>255)
				b = 255;
			reflect();
			return true;
		}
		break;
	}

	Debug::print(LOG_PRINT, "led : Show led status\r\nled s : Stop lightning LED\r\nled test : = led 255 255 255\r\nled rainbow [seconds] : Rainbow w/ [seconds]\r\nled rainbow s : Stop rainbow\r\nled brink s : Stop brinking\r\nled brink [seconds] : Brink w/ [seconds]\r\nled brink [sec1] [sec2] : Brink w/ sec1 on / sec2 off\r\nled hsv [sec] : hsv w/ [sec]\r\nled hsv s : stop hsv\r\nled [R] [G] [B] : Light LED w/ RGB in 256 steps (0-255)\r\n");

	return false;
}

void LED::reflect() {
	softPwmWrite(PIN_LED_R, r);
	softPwmWrite(PIN_LED_G, g);
	softPwmWrite(PIN_LED_B, b);
}

void LED::setColor(int x) {
	r = (x >> 16) & 255;
	g = (x >> 8) & 255;
	b = x & 255;
	reflect();
}

void LED::setColor(int c1, int c2, int c3) {
	r = (c1>255 ? 255 : c1);
	g = (c2>255 ? 255 : c2);
	b = (c3>255 ? 255 : c3);
	reflect();
}


void LED::turnOff() {
	softPwmWrite(PIN_LED_R, 0);
	softPwmWrite(PIN_LED_G, 0);
	softPwmWrite(PIN_LED_B, 0);
}


void LED::onUpdate(const struct timespec& time) {
	if (hf) {
		if (Time::dt(time, mLastUpdateTime1)<p)
			;
		else {
			hsv((float)d);
			d += 0.01;
			if (d>1.0)
				d = 0;
			mLastUpdateTime1 = time;
		}
	}
	else if (rbw) {
		if (Time::dt(time, mLastUpdateTime1)<s)
			;
		else {
			setColor(((t >> 2) & 1) * 255, ((t >> 1) & 1) * 255, (t & 1) * 255);
			t++;
			if (t == 8)
				t = 1;
			mLastUpdateTime1 = time;
		}
	}
	if (bnk) {
		if (Time::dt(time, mLastUpdateTime2)<u)
			reflect();
		else if (Time::dt(time, mLastUpdateTime2)<u + v)
			turnOff();
		else
			mLastUpdateTime2 = time;
	}
}

void LED::rainbow(double x) {
	s = x;
	rbw = true;
	hf = false;
}

void LED::stopRainbow() {
	rbw = false;
}

void LED::startHSV(double x) {
	p = x;
	d = 0;
	hf = true;
	rbw = false;
}

void LED::stopHSV() {
	hf = false;
}

void LED::brink(double x) {
	u = v = x;
	bnk = true;
}

void LED::brink(double x, double y) {
	u = x;
	v = y;
	bnk = true;
}

void LED::stopBrink() {
	bnk = false;
}

void LED::hsv(float h) {
	float s_ = 1.0f, v_ = 1.0f, r_, g_, b_;
	r_ = g_ = b_ = v_;
	h *= 6.0f;
	int i = (int)h;
	float f = h - (float)i;
	switch (i) {
	default:
	case 0:
		g_ *= 1 - s_*(1 - f);
		b_ *= 1 - s_;
		break;
	case 1:
		r_ *= 1 - s_*f;
		b_ *= 1 - s_;
		break;
	case 2:
		r_ *= 1 - s_;
		b_ *= 1 - s_*(1 - f);
		break;
	case 3:
		r_ *= 1 - s_;
		g_ *= 1 - s_*f;
		break;
	case 4:
		r_ *= 1 - s_*(1 - f);
		g_ *= 1 - s_;
		break;
	case 5:
		g_ *= 1 - s_;
		b_ *= 1 - s_*f;
		break;
	}
	setColor((int)(r_ * 256), (int)(g_ * 256), (int)(b_ * 255));

}

void LED::clearLED() {
	stopBrink();
	stopRainbow();
	stopHSV();
	turnOff();
}



LED::LED() {
	setName("led");
	setPriority(TASK_PRIORITY_ACTUATOR, TASK_INTERVAL_ACTUATOR);
}

LED::~LED() {
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

