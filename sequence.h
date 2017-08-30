#pragma once
#include <time.h>
#include <list>
#include <iostream>
#include "task.h"
#include "utils.h"

//テスト用状態
class Testing : public TaskBase
{
protected:
	virtual bool onInit(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

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
	struct timespec mLastAngleCheckTime;

	int mLastPressure;//前回の気圧
	long long mLastMotorPulseL, mLastMotorPulseR;//前回チェック時のモーター回転数
	float mLastAngleTheta, mLastAnglePhi, mLastAnglePsi;
	unsigned int mContinuousPressureCount;//気圧が閾値以下の状態が続いた回数
	unsigned int mCoutinuousGyroCount;//角速度が閾値以下の状態が続いた回数
	unsigned int mContinuousMotorPulseCount;//モータ回転数が閾値以上の状態が続いた回数
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	//次の状態に移行
	void nextState();
public:
	Falling();
	~Falling();
};

/*
class Waking : public TaskBase
{
	struct timespec mLastUpdateTime;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
	void nextState();
public:
	Waking();
	~Waking();
};
*/

//パラ分離状態(サーボを動かしてパラを切り離す)
class Separating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回サーボの向きを更新した時間
	bool mCurServoState;			//現在のサーボの向き(true = 1,false = 0)
	unsigned int mServoCount;		//サーボの向きを変更した回数
	enum STEP{STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE,STEP_GO_FORWARD};
	enum STEP mCurStep;

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
	struct timespec mLastNaviMoveCheckTime;	 //前回のGPSによるスタック判定とナビゲーション処理のチェック時刻
	struct timespec mEscapingRandomStartTime;//EscapingRandomの開始時刻
	struct timespec mLastArmServoMoveTime;
	struct timespec mLastArmServoStopTime;
	struct timespec mLastUpdateTime;
	//ゴール位置
	VECTOR3 mGoalPos;//現在のゴール
  VECTOR3 currentPos;

					 //goallist を保存する
	VECTOR3 goal;
	//通過したゴールを保存するリスト
	std::list<VECTOR3> PassedGoal;

	bool mIsGoalPos;
	bool mArmMoveFlag;
	bool mArmStopFlag;
	double distance_from_goal_to_start;
  bool firstTime; // 初めて位置が取れたとき

  bool mStuckFlag;

	std::list<VECTOR3> mLastPos;
  
//方向推定手法切り替え
  int mMethod;
  unsigned int mGpsCountMax;

protected:
	
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	void navigationMove(double distance) const; //通常時の移動処理
	bool isStuckByGPS() ;//スタック判定(GPS)
	bool removeError();//異常値の除去

	//次の状態に移行
	void nextState();

	//ファイルから　GPSの座標を読んで、リストに保存する
	void getGoal(VECTOR3& goal);
	//ファイルに　通過したゴールの座標を書き込み
	//void writePassedGoal(std::list<VECTOR3>& PassedGoal, VECTOR3& mGoalPos);

	//GoalList.txtの中身　GoalListから通過したゴールを削除する,GoalList を書き込む
	//void deleteGoalList(std::list<VECTOR3>& GoalList);
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
//extern Waking gWakingState;

