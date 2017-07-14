/*
	アクチュエータ制御プログラム

	モータ以外の実世界に働きかけるモジュールを操作します
	task.hも参照
	*/

#pragma once
#include "task.h"
#include <tuple>

// ブザー制御クラス
class Buzzer : public TaskBase
{
private:
	int mPin;
	int mOnPeriodMemory;	//鳴らす時間を保持
	int mOnPeriod;			//0以上なら鳴らす、負なら鳴らさない
	int mOffPeriodMemory;	//鳴らさない時間を保持
	int mOffPeriod;			//0以上なら鳴らさない
	int mCount;				//ブザーを鳴らす回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onUpdate(const struct timespec& time);
	virtual void restart();

public:
	//特に指定しない場合のブザーの間隔
	const static int DEFAULT_OFF_PERIOD = 500;

	//ブザーをperiod[ms]だけ鳴らす(長さは厳密ではありません！)
	void start(int period);

	//(鳴らす時間[ms], 鳴らす回数) ブザーを複数回数鳴らしたい場合に使用
	void start(int on_period, int count);

	//(鳴らす時間[ms], 鳴らさない時間[ms], 鳴らす回数)
	void start(int on_period, int off_period, int count);
	//ブザーを止める
	void stop();

	Buzzer();
	~Buzzer();
};

//// サーボ制御クラス(ソフトウェアPWM)
//class MultiServo : public TaskBase
//{
//private:
//	const static int SERVO_MIN_RANGE = 6;	//そのうちconstants.hに移す
//	const static int SERVO_MAX_RANGE = 25;	//そのうちconstants.hに移す
//	const static int SERVO_RANGE = 100;		//そのうちconstants.hに移す
//
//	//POSITION_RELEASE: ピンが抜ける位置, POSITION_HOLD: ピンが刺さった状態の位置
//	enum POSITION { POSITION_RELEASE = 25, POSITION_HOLD = 6 };
//
//	int mPin;
//protected:
//	virtual bool onInit(const struct timespec& time);
//	virtual void onClean();
//	virtual bool onCommand(const std::vector<std::string>& args);
//
//	//サーボを指定されたangle[0-SERVO_MAX_RANGE]になるように制御を開始する
//	//(※2014verはSoftware PWM使用のため細かい角度の調整は難しい)
//	virtual void start(int angle);
//	virtual void start(POSITION p);
//public:
//	//サーボの制御を終了する
//	void stop();
//
//	void moveRelease();//パラシュート切り離し
//	void moveHold();//ピンが刺さった状態の位置に移動
//
//	MultiServo();
//	~MultiServo();
//};

// パラサーボ制御クラス(ハードウェアPWM)
//20170626_パラシュートサーボとバックサーボを１つに統合(MultiServo)
class MultiServo : public TaskBase
{
private:
	int mPin;
	double mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//サーボを指定されたangle[0-1]になるように制御を開始する
	void start(double angle);
	//サーボの制御を終了する
	void stop();
	void moveRelease();//パラシュート切り離し
	void moveHold();//ピンが刺さった状態の位置に移動
	void Running();//走っているときの角度
	void fold();//たたんでいるときの角度
	double get();

	MultiServo();
	~MultiServo();
};

//// サーボ制御クラス(ソフトウェアPWM)
//class SoftwarePWMServo : public TaskBase
//{
//private:
//	const static int SERVO_MIN_RANGE = 1;	//そのうちconstants.hに移す
//	const static int SERVO_MAX_RANGE = 100;	//そのうちconstants.hに移す
//	int mPin;
//	int mAngle;
//protected:
//	virtual bool onInit(const struct timespec& time);
//	virtual void onClean();
//	virtual bool onCommand(const std::vector<std::string>& args);
//	//サーボを指定されたangle[0-SERVO_MAX_RANGE]になるように制御を開始する
//	//(※2014verはSoftware PWM使用のため細かい角度の調整は難しい)
//	virtual void start(int angle);
//public:
//	//サーボの制御を終了する
//	void stop();
//	int get();
//
//	SoftwarePWMServo(const char* name, unsigned int pin);
//	~SoftwarePWMServo();
//};

// サーボ制御クラス(ソフトウェアPWM)
//20170623_サーボを１つにするため削除
/*
class FrontStabiServo : public TaskBase
{
private:
	const static int SERVO_MIN_RANGE = 1;	//そのうちconstants.hに移す
	const static int SERVO_MAX_RANGE = 100;	//そのうちconstants.hに移す
	const static int SERVO_RANGE = 100;		//そのうちconstants.hに移す
	int mPin;
	int mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//サーボを指定されたangle[0-SERVO_MAX_RANGE]になるように制御を開始する
	//(※2014verはSoftware PWM使用のため細かい角度の調整は難しい)
	virtual void start(int angle);
	//サーボの制御を終了する
	void stop();
	int get();

	FrontStabiServo();
	~FrontStabiServo();
};
*/

// サーボ制御クラス(ソフトウェアPWM)
//20170623_このサーボとパラサーボを統合します
//
/*
class BackStabiServo : public TaskBase
{
private:
	const static int SERVO_MIN_RANGE = 1;	//そのうちconstants.hに移す
	const static int SERVO_MAX_RANGE = 100;	//そのうちconstants.hに移す
	const static int SERVO_RANGE = 100;		//そのうちconstants.hに移す
	int mPin;
	int mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//サーボを指定されたangle[0-SERVO_MAX_RANGE]になるように制御を開始する
	//(※2014verはSoftware PWM使用のため細かい角度の調整は難しい)
	virtual void start(int angle);
	//サーボの制御を終了する
	void stop();
	int get();

	BackStabiServo();
	~BackStabiServo();
};
*/
// サーボ制御クラス(ソフトウェアPWM)
/*
class ArmServo : public TaskBase
{
private:
	const static int SERVO_MIN_RANGE = 0;	//そのうちconstants.hに移す
	const static int SERVO_MAX_RANGE = 100;	//そのうちconstants.hに移す
	const static int SERVO_RANGE = 100;		//そのうちconstants.hに移す

	//POSITION_RELEASE: ピンが抜ける位置, POSITION_HOLD: ピンが刺さった状態の位置
	enum POSITION { POSITION_RELEASE = 25, POSITION_HOLD = 6 };

	int mAngle;
	int mPin;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//サーボを指定されたangle[0-SERVO_MAX_RANGE]になるように制御を開始する
	//(※2014verはSoftware PWM使用のため細かい角度の調整は難しい)
	virtual void start(int angle);
	virtual void start(POSITION p);
	//サーボの制御を終了する
	void stop();
	int get();

	void moveRelease();//パラシュート切り離し
	void moveHold();//ピンが刺さった状態の位置に移動

	ArmServo();
	~ArmServo();
};
*/

// サーボ制御クラス(ハードウェアPWM)
//20170623_サーボを１つにするため削除
/*
class NeckServo : public TaskBase
{
private:
	int mPin;
	double mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//サーボを指定されたangle[0-1]になるように制御を開始する
	void start(double angle);
	//サーボの制御を終了する
	void stop();

	double get();

	NeckServo(const char* name, unsigned int pin);
	~NeckServo();
};
*/

// 4つのサーボモーターをまとめるクラス
/*class SServo : public TaskBase
{
private:
	//std::tuple<int, int, int, double> mOffsetAngle; //set zero point for both of servo.
	std::tuple<int, int, int, double> mRunAngle, mFoldAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	void start(int j, int m, int a, double n);
	//void startJohn(int j);
	void startMulti(int m);
	//void startArm(int a);
	//void startNeck(double n);
	void stop();
	void moveFold();
	void moveRun();
	SServo();
	~SServo();
};
*/
//// 二つのスタビサーボをまとめるクラス
//class StabiServo : public TaskBase
//{
//private:
//	int angle;
//	int Rangle;
//	int Langle;
//protected:
//	virtual bool onInit(const struct timespec& time);
//	virtual void onClean();
//	virtual bool onCommand(const std::vector<std::string>& args);
//public:
//	//サーボを指定されたangle[0-1]になるように制御を開始する
//	void startall(double angle);
//	void startR();
//	void startL();
//	//サーボの制御を終了する
//	void stop();
//	//サーボをしまう
//	void close();
//
//	StabiServo();
//	~StabiServo();
//};

extern Buzzer gBuzzer;
extern MultiServo gMultiServo;
//extern SServo gSServo;
//extern ArmServo gArmServo;
//extern FrontStabiServo gJohnServo;
//extern MultiServo gMultiServo;
//extern NeckServo gNeckServo;
