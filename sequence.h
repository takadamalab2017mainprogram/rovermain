#pragma once
#include <time.h>
#include "task.h"
#include "utils.h"

//テスト用状態
class Testing : public TaskBase
{
protected:
	virtual bool onInit(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);

	//次の状態に移行
	void nextState();

public:
	Testing();
	~Testing();
};

//筒の中に入っている状態
class Waiting : public TaskBase
{
	struct timespec mStartTime;//状態開始時刻
	unsigned int mContinuousLightCount;//光っていることが検知された回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//次の状態に移行
	void nextState();

public:
	Waiting();
	~Waiting();
};

//落下している状態
class Falling : public TaskBase
{
private:
	struct timespec mStartTime;//状態開始時刻
	struct timespec mLastCheckTime;//前回のチェック時刻
	int mLastPressure;//前回の気圧
	unsigned int mContinuousPressureCount;//気圧が閾値以下の状態が続いた回数
	unsigned int mCoutinuousGyroCount;//角速度が閾値以下の状態が続いた回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//次の状態に移行
	void nextState();
public:
	Falling();
	~Falling();
};

//パラ分離状態(サーボを動かしてパラを切り離す)
class Separating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回サーボの向きを更新した時間
	bool mCurServoState;			//現在のサーボの向き(true = 1,false = 0)
	unsigned int mServoCount;		//サーボの向きを変更した回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//次の状態に移行
	void nextState();
public:
	Separating();
	~Separating();
};

//ゴールへの移動中
class Navigating : public TaskBase
{
private:
	struct timespec mLastCheckTime;//前回のチェック時刻

	//ゴール、現在位置、前回の位置(zの値が0の場合位置が有効であるとする)
	VECTOR3 mGoalPos,mCurrentPos,mLastPos;
	bool mIsGoalPos,mIsCurrentPos,mIsLastPos;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);

	//次の状態に移行
	void nextState();
public:
	void setGoal(const VECTOR3& pos);

	Navigating();
	~Navigating();
};

extern Testing gTestingState;
extern Waiting gWaitingState;
extern Falling gFallingState;
extern Separating gSeparatingState;
extern Navigating gNavigatingState;
