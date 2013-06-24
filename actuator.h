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
	int mPeriod;//0以上なら鳴らす、負なら鳴らさない
protected:
	virtual bool onInit();
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string> args);
	virtual void onUpdate();

public:
	//ブザーをperiod[ms]だけ鳴らす(長さは厳密ではありません！)
	void start(int period);
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
	virtual bool onInit();
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
	virtual bool onInit();
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

