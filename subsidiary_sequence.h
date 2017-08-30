#pragma once
#include <time.h>
#include <list>
#include "task.h"
#include "utils.h"

class WadachiPredicting : public TaskBase
{
	struct timespec mLastUpdateTime;//�O���̃`�F�b�N����
	bool mIsAvoidingEnable;
	enum STEP{ STEP_RUNNING, STEP_STOPPING, STEP_WAKING, STEP_CHECKING, STEP_AVOIDING };
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	bool isWorking(const struct timespec& time);//���O���m���쒆���ۂ�
	WadachiPredicting();
	~WadachiPredicting();
};

class Escaping : public TaskBase
{
	struct timespec mLastUpdateTime;//�O���̍s�������̕ω�����

	enum STEP{ STEP_BACKWARD = 0, STEP_AFTER_BACKWARD, STEP_PRE_CAMERA, STEP_CAMERA, STEP_CAMERA_TURN, STEP_CAMERA_FORWARD, STEP_CAMERA_TURN_HERE, STEP_RANDOM };
	enum STEP mCurStep;
	enum RANDOM_STEP{ RANDOM_STEP_BACKWARD = 0, RANDOM_STEP_TURN, RANDOM_STEP_FORWARD };
	enum RANDOM_STEP mCurRandomStep;
	unsigned int mEscapingTriedCount;//�J�����E�o�����s��������
	double mAngle;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);

	void stuckMoveRandom();//�X�^�b�N���̈ړ�����
	//	void stuckMoveCamera(IplImage* pImage);//�J�������p�����X�^�b�N���̈ړ�����
public:
	Escaping();
	~Escaping();
};


class EscapingByStabi : public TaskBase
{
	struct timespec mLastUpdateTime;//�O���̍s�������̕ω�����
	bool mFlag;
	unsigned int mTryCount;
	unsigned int Escaping_Chance_limit;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	unsigned int getTryCount();
	EscapingByStabi();
	~EscapingByStabi();
};


class EscapingRandom : public TaskBase
{
	struct timespec mLastUpdateTime;

	enum STEP { STEP_TURN = 0, STEP_FORWARD };
	enum STEP mCurStep;
	unsigned int RandomCount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	int motorforce0;
	int motorforce1;
public:
	EscapingRandom();
	~EscapingRandom();
};

class Waking : public TaskBase
{
	struct timespec mLastUpdateTime;
	enum STEP{ STEP_CHECK_LIE, STEP_WAIT_LIE, STEP_START, STEP_STOP, STEP_DEACCELERATE, STEP_VERIFY };
	enum STEP mCurStep;
	double mAngleOnBegin;
	unsigned int mWakeRetryCount;
	int mStartPower;				
	double mAngleThreshold;			
	double mDeaccelerateDuration;	

	void setPower(int p);
	void setAngle(double a);
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
public:
	Waking();
	~Waking();
};


//横転復帰
class WakingFromLie : public TaskBase
{
	struct timespec mLastUpdateTime;
	struct timespec mLastDtTime;
	enum STEP{STEP_FORWARD, STEP_VERIFY};
	enum STEP mCurStep;
	unsigned int mWakeRetryCount;
	double mShortestSpeedUpPeriod;
	double mCurrentPower;
	unsigned int mNotLieCount;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
	virtual void onClean();
public:
	WakingFromLie();
	~WakingFromLie();
};



/*
class Turning : public TaskBase
{
	bool mIsTurningLeft;
	double mTurnPower;
	double mAngle;
	struct timespec mLastUpdateTime;//�s���J�n����
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	void setDirection(bool left);

	Turning();
	~Turning();
};
*/


/*
class Avoiding : public TaskBase
{
	struct timespec mLastUpdateTime;//�s���J�n����
	double mAngle;
	enum STEP { STEP_TURN = 0, STEP_FORWARD };
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:

	Avoiding();
	~Avoiding();
};
*/

/*

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
*/
class SensorLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string mFilenameGPS, mFilenameGyro, mFilenamePressure, mFilenameEncoder, mFilenameAccel,mFilenameNineAxis;
	unsigned long long mLastEncL, mLastEncR;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);

	void write(const std::string& filename, const char* fmt, ...);
public:
	SensorLogging();
	~SensorLogging();
};

/*
class MovementLogging : public TaskBase
{
	struct timespec mLastUpdateTime;
	std::string mFilenameEncoder, mFilenameAcceleration;

	double mPrevPowerL, mPrevPowerR;

	
	unsigned long long mPrevDeltaPulseL, mPrevDeltaPulseR;

	bool mBuzzerFlag;	//		true:ON, false:OFF
	bool mPrintFlag;	//TeraTermtrue:ON, false:OFF
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	void write(const std::string& filename, const char* fmt, ...);
public:
	MovementLogging();
	~MovementLogging();
};
*/

class EncoderMonitoring : public TaskBase
{
	struct timespec mLastSamplingTime;			
	struct timespec mLastUpdateTime;			

	long long mStoredPulse;			
	long long mCurrentMaxPulse;		
	long long mPrevDeltaPulseL, mPrevDeltaPulseR;

	int mUpdateTimer;					
	long long mThresholdPulse;			
	long long mIgnoredDeltaUpperPulse;	
	long long mIgnoredDeltaLowerPulse;	
	long long mUpperThreshold;			
	long long mLowerThreshold;			
	bool mIsPrint;								
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	//臒l���X�V����
	virtual void updateThreshold();
	virtual bool removeError(long long pulseL, long long pulseR);					//�p���X���ُ̈��l������ true: �ُ��l�����o
public:
	EncoderMonitoring();
	~EncoderMonitoring();
};


extern Escaping gEscapingState;
extern Waking gWakingState;
//extern WakingFromLie gWakingFromLieState;
//extern Turning gTurningState;
//extern Avoiding gAvoidingState;
//extern WadachiPredicting gPredictingState;
extern EscapingRandom gEscapingRandomState;
extern EscapingByStabi gEscapingByStabiState;
extern SensorLogging gSensorLoggingState;