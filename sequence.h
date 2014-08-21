#pragma once
#include <time.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
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
	unsigned long long mLastMotorPulseL,mLastMotorPulseR;//前回チェック時のモーター回転数
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

//パラ分離状態(サーボを動かしてパラを切り離す)
class Separating : public TaskBase
{
private:
	struct timespec mLastUpdateTime;//前回サーボの向きを更新した時間
	bool mCurServoState;			//現在のサーボの向き(true = 1,false = 0)
	unsigned int mServoCount;		//サーボの向きを変更した回数
	enum STEP{STEP_SEPARATE = 0, STEP_PRE_PARA_JUDGE,STEP_PARA_JUDGE,STEP_PARA_DODGE};
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
	struct timespec mLastNaviMoveCheckTime;	//前回のGPSによるスタック判定とナビゲーション処理のチェック時刻
	struct timespec mLastEncoderCheckTime;	//前回のエンコーダチェック時刻

	//ゴール位置
	VECTOR3 mGoalPos;
	bool mIsGoalPos;

	//GPS座標から計算された過去数回分の位置
	std::list<VECTOR3> mLastPos;

	//前回のパルス数
	unsigned long long mPrevDeltaPulseL, mPrevDeltaPulseR;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);

	void navigationMove(double distance) const; //通常時の移動処理
	bool isStuck() const;//スタック判定
	bool removeError();//異常値の除去

	//次の状態に移行
	void nextState();
public:
	void setGoal(const VECTOR3& pos);

	Navigating();
	~Navigating();
};

//轍事前検知動作
class WadachiPredicting : public TaskBase
{
	struct timespec mLastUpdateTime;//前回のチェック時刻
	bool mIsAvoidingEnable;
	enum STEP{STEP_RUNNING, STEP_STOPPING, STEP_WAKING, STEP_CHECKING, STEP_AVOIDING};
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);
public:
	bool isWorking(const struct timespec& time);//事前検知動作中か否か
	WadachiPredicting();
	~WadachiPredicting();
};

//轍脱出動作
//このタスクが有効の間はナビゲーションしません
class Escaping : public TaskBase
{
	struct timespec mLastUpdateTime;//前回の行動からの変化時間

	enum STEP{STEP_BACKWARD = 0, STEP_AFTER_BACKWARD, STEP_PRE_CAMERA, STEP_CAMERA, STEP_CAMERA_TURN, STEP_CAMERA_FORWARD, STEP_CAMERA_TURN_HERE, STEP_RANDOM};
	enum STEP mCurStep;
	enum RANDOM_STEP{RANDOM_STEP_BACKWARD = 0, RANDOM_STEP_TURN, RANDOM_STEP_FORWARD};
	enum RANDOM_STEP mCurRandomStep;
	unsigned int mEscapingTriedCount;//カメラ脱出を試行した回数
	double mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);

	void stuckMoveRandom();//スタック時の移動処理
	void stuckMoveCamera(IplImage* pImage);//カメラを用いたスタック時の移動処理
public:
	Escaping();
	~Escaping();
};

//轍脱出脱出（スタビ使用ver）
class EscapingByStabi : public TaskBase
{	
	struct timespec mLastUpdateTime;//前回の行動からの変化時間
	bool mFlag;
	int mTryCount;
	double mAngle;// 芋虫走行中のスタビ角度 
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);
public:
	EscapingByStabi();
	~EscapingByStabi();
};
////轍脱出脱出（旧ランダム）
//class EscapingRandom : public TaskBase
//{
//	struct timespec mLastUpdateTime;//前回の行動からの変化時間
//
//	enum STEP{STEP_BACKWARD = 0, STEP_TURN, STEP_FORWARD};
//	enum STEP mCurStep;
//protected:
//	virtual bool onInit(const struct timespec& time);
//	virtual void onUpdate(const struct timespec& time);
//public:
//	EscapingRandom();
//	~EscapingRandom();
//};
// ジャンプ機構（仮）
class Jumping : public TaskBase
{
	struct timespec mLastUpdateTime; // 前回の行動からの変化時間
	bool flag;
	int stopcount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	Jumping();
	~Jumping();
};

//ローバーの姿勢制御
//姿勢制御が完了するとタスクが終了します
class Waking : public TaskBase
{
	struct timespec mLastUpdateTime;//行動開始時刻
	enum STEP{STEP_START,STEP_STOP,STEP_VERIFY};
	enum STEP mCurStep;
	double mAngleOnBegin;
	unsigned int mWakeRetryCount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual void onClean();
public:
	Waking();
	~Waking();
};

//ローバーのその場回転
//完了するとタスクが終了します
class Turning : public TaskBase
{
	bool mIsTurningLeft;
	double mTurnPower;
	double mAngle;
	struct timespec mLastUpdateTime;//行動開始時刻
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	void setDirection(bool left);

	Turning();
	~Turning();
};

//轍事前検知時の回避動作
//完了するとタスクが終了します
class Avoiding : public TaskBase
{
	struct timespec mLastUpdateTime;//行動開始時刻
	double mAngle;
	enum STEP {STEP_TURN = 0, STEP_FORWARD};
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:

	Avoiding();
	~Avoiding();
};

/* ここから　2014年6月オープンラボ前に実装 */
class ColorAccessing : public TaskBase
{
	struct timespec mLastUpdateTime;//前回のチェック時刻
	bool mIsAvoidingEnable;
	enum STEP{STEP_STARTING, STEP_TURNING, STEP_STOPPING_FAST, STEP_STOPPING_LONG, STEP_CHECKING};
	enum STEP mCurStep;
    double mAngleOnBegin;
    bool mIsLastActionStraight;
    int mTryCount;
	int mMotorPower;
	int mCurrentMotorPower;
	int actCount;
	unsigned long long gDeltaPulseL;
	unsigned long long gDeltaPulseR;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);

	void setMotorPower();
	void adjMotorPower();
	//次の状態に移行
	void nextState();
public:
	ColorAccessing();
	~ColorAccessing();
};
/* ここまで　2014年6月オープンラボ前に実装 */


//記念撮影
class PictureTaking : public TaskBase
{
	struct timespec mLastUpdateTime;
	unsigned int mStepCount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	PictureTaking();
	~PictureTaking();
};

//センサーログ
class SensorLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string mFilenameGPS,mFilenameGyro,mFilenamePressure,mFilenameEncoder;
	unsigned long long mLastEncL,mLastEncR;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	void write(const std::string& filename,const char* fmt, ... );
public:
	SensorLogging();
	~SensorLogging();
};

// 前進しつつ1秒ごとにタイヤ回転数、加速度の値を取得
class MovementLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string mFilenameEncoder,mFilenameAcceleration;

	double mPrevPowerL,mPrevPowerR;

	//前回のパルス数
	unsigned long long mPrevDeltaPulseL, mPrevDeltaPulseR;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string> args);

	void write(const std::string& filename,const char* fmt, ... );
public:
	MovementLogging();
	~MovementLogging();
};
extern Testing gTestingState;
extern Waiting gWaitingState;
extern Falling gFallingState;
extern Separating gSeparatingState;
extern Navigating gNavigatingState;
extern Escaping gEscapingState;
extern Waking gWakingState;
extern Turning gTurningState;
extern Avoiding gAvoidingState;
extern WadachiPredicting gPredictingState;
//extern EscapingRandom gEscapingRandomState;
extern EscapingByStabi gEscapingByStabiState;
extern Jumping gJumpingState;
extern PictureTaking gPictureTakingState;
extern SensorLogging gSensorLoggingState;
extern ColorAccessing gColorAccessing;
extern MovementLogging gMovementLoggingState;
