#pragma once
#include <time.h>
#include <list>
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

//���[�o�[�̎p������
//�p�����䂪���������ƃ^�X�N���I�����܂�
class Waking : public TaskBase
{
	struct timespec mLastUpdateTime;
	enum STEP { STEP_CHECK_LIE, STEP_WAIT_LIE, STEP_START, STEP_STOP, STEP_DEACCELERATE, STEP_VERIFY };
	enum STEP mCurStep;
	double mAngleOnBegin;
	unsigned int mWakeRetryCount;
	int mStartPower;				//�N���オ���J�n���̃��[�^�o�͗�
	double mAngleThreshold;			//�N���オ�芮���Ƃ����p�x(ZX)
	double mDeaccelerateDuration;	//�����ɗv���鎞��

	void setPower(int p);
	void setAngle(double a);
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


//パラ分離状態(サーボを動かしてパラを切り離す)
class Separating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回サーボの向きを更新した時間
	bool mCurServoState;			//現在のサーボの向き(true = 1,false = 0)
	unsigned int mServoCount;		//サーボの向きを変更した回数
	enum STEP{STEP_STABI_OPEN = 0, STEP_WAIT_STABI_OPEN, STEP_SEPARATE, STEP_PRE_PARA_JUDGE,STEP_PARA_JUDGE,STEP_PARA_DODGE,STEP_GO_FORWARD};
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

	//ゴール位置
	VECTOR3 mGoalPos;
	bool mIsGoalPos;
	bool mArmMoveFlag;
	bool mArmStopFlag;
  double distance_from_goal_to_start;

	//GPS座標から計算された過去数回分の位置
	std::list<VECTOR3> mLastPos;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	void navigationMove(double distance) const; //通常時の移動処理
	bool isStuckByGPS() const;//スタック判定(GPS)
	bool removeError();//異常値の除去

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
extern Waking gWaitingState;

