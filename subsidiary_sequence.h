#pragma once
#include <time.h>
#include <list>
#include "task.h"
#include "utils.h"


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

extern EscapingRandom gEscapingRandomState;
extern EscapingByStabi gEscapingByStabiState;
extern SensorLogging gSensorLoggingState;