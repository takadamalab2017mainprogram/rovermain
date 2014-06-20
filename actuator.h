/*
	アクチュエータ制御プログラム

	モータ以外の実世界に働きかけるモジュールを操作します
	task.hも参照
*/

#pragma once
#include "task.h"

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
	virtual bool onCommand(const std::vector<std::string> args);
	virtual void onUpdate(const struct timespec& time);
	virtual void restart();

public:
	//ブザーをperiod[ms]だけ鳴らす(長さは厳密ではありません！)
	void start(int period);

	//(鳴らす時間[ms], 鳴らす回数) ブザーを複数回数鳴らしたい場合に使用
	void start(int on_period, int count);	

	//(鳴らす時間[ms], 鳴らさない時間[ms], 鳴らす回数)
	void start(int on_period, off_period, count);	
	//ブザーを止める
	void stop();

	Buzzer();
	~Buzzer();
};

// サーボ制御クラス(ハードウェアPWMを使う)
class Servo : public TaskBase
{
private:
	int mPin;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string> args);
public:
	//サーボを指定されたangle[0-1]になるように制御を開始する
	void start(double angle);
	//サーボの制御を終了する
	void stop();

	Servo();
	~Servo();
};

// XBeeスリープ制御クラス
class XBeeSleep : public TaskBase
{
private:
	int mPin;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string> args);

public:
	void setState(bool sleep);

	XBeeSleep();
	~XBeeSleep();
};

extern Buzzer gBuzzer;
extern Servo gServo;
extern XBeeSleep gXbeeSleep;

