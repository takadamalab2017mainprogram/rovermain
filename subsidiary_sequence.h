#pragma once
#include <time.h>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include "task.h"
#include "utils.h"

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
	virtual bool onCommand(const std::vector<std::string>& args);
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
	unsigned int mTryCount;
	double mAngle;// 芋虫走行中のスタビ角度 
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	unsigned int getTryCount();
	EscapingByStabi();
	~EscapingByStabi();
};
//轍脱出脱出（旧ランダム）
class EscapingRandom : public TaskBase
{
	struct timespec mLastUpdateTime;//前回の行動からの変化時間

	enum STEP{STEP_BACKWARD = 0, STEP_TURN, STEP_FORWARD};
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	EscapingRandom();
	~EscapingRandom();
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
	
	bool mBuzzerFlag;	//ブザーのON,OFFを管理 				true:ON, false:OFF
	bool mPrintFlag;	//TeraTerm上の表示のON,OFFを管理	true:ON, false:OFF
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	void write(const std::string& filename,const char* fmt, ... );
public:
	MovementLogging();
	~MovementLogging();
};

extern Escaping gEscapingState;
extern Waking gWakingState;
extern Turning gTurningState;
extern Avoiding gAvoidingState;
extern WadachiPredicting gPredictingState;
extern EscapingRandom gEscapingRandomState;
extern EscapingByStabi gEscapingByStabiState;
extern PictureTaking gPictureTakingState;
extern SensorLogging gSensorLoggingState;
extern MovementLogging gMovementLoggingState;