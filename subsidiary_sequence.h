#pragma once
#include <time.h>
#include <list>
#include "task.h"
#include "utils.h"

//�Q���O���m����
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

//�Q�E�o����
//���̃^�X�N���L���̊Ԃ̓i�r�Q�[�V�������܂���
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

//�Q�E�o�E�o�i�X�^�r�g�pver�j
class EscapingByStabi : public TaskBase
{
	struct timespec mLastUpdateTime;//�O���̍s�������̕ω�����
	bool mFlag;
	unsigned int mTryCount;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	unsigned int getTryCount();
	EscapingByStabi();
	~EscapingByStabi();
};

//�Q�E�o�E�o�i�������_���j
class EscapingRandom : public TaskBase
{
	struct timespec mLastUpdateTime;//�O���̍s�������̕ω�����

	enum STEP{ STEP_TURN = 0, STEP_FORWARD };
	enum STEP mCurStep;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
public:
	EscapingRandom();
	~EscapingRandom();
};


//���[�o�[�̎p������
//�p�����䂪���������ƃ^�X�N���I�����܂�
/*
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
*/

//���[�o�[�̂��̏����]
//���������ƃ^�X�N���I�����܂�
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

//�Q���O���m���̉��𓮍�
//���������ƃ^�X�N���I�����܂�
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
//�L�O�B�e
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
//�Z���T�[���O
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


class EncoderMonitoring : public TaskBase
{
	struct timespec mLastSamplingTime;			//�p���X�����T���v�����O�����������ۑ�
	struct timespec mLastUpdateTime;			//臒l���X�V�����������ۑ�

	long long mStoredPulse;			//���莞�ԓ��̃p���X�ő��l(������臒l�Ɏg�p)
	long long mCurrentMaxPulse;		//���݂̊��ԓ��̃p���X�ő��l
	long long mPrevDeltaPulseL, mPrevDeltaPulseR;//�O���̃p���X��

	int mUpdateTimer;					//臒l���X�V�����Ԋu(�b)
	long long mThresholdPulse;			//mStoredPulse���炱�̒l���������l��臒l�ɂȂ�
	long long mIgnoredDeltaUpperPulse;	//���̒l���ȏ��p���X���������ꍇ��臒l���X�V���Ȃ�
	long long mIgnoredDeltaLowerPulse;	//���̒l���ȏ��p���X���������ꍇ��臒l���X�V���Ȃ�
	long long mUpperThreshold;			//臒l�̏���
	long long mLowerThreshold;			//臒l�̉���
	bool mIsPrint;								//true�Ȃ�1�b���ƂɃp���X�����\������
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
extern EscapingRandom gEscapingRandomState;
extern EscapingByStabi gEscapingByStabiState;
extern SensorLogging gSensorLoggingState;
extern Waking gWakingState;