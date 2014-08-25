#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <stdarg.h>
#include "subsidiary_sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"
#include "image_proc.h"

Escaping gEscapingState;
EscapingRandom gEscapingRandomState;
EscapingByStabi gEscapingByStabiState;
Waking gWakingState;
Turning gTurningState;
Avoiding gAvoidingState;
WadachiPredicting gPredictingState;
PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
MovementLogging gMovementLoggingState;

bool WadachiPredicting::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	gCameraCapture.startWarming();

	return true;
}
void WadachiPredicting::onUpdate(const struct timespec& time)
{
	if(gAvoidingState.isActive())return;
	if(!mIsAvoidingEnable)
	{
		if(Time::dt(time,mLastUpdateTime) >= 2.5)
		{
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(!gImageProc.isWadachiExist(pImage))return;
			//�Q�����O���m����
			gCameraCapture.startWarming();
		}
		return;
	}

	switch(mCurStep)
	{
	case STEP_RUNNING:
		if(Time::dt(time,mLastUpdateTime) > 60)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Stoping started\r\n");
			mCurStep = STEP_STOPPING;
			mLastUpdateTime = time;
			gMotorDrive.drive(0,0);
		}
		break;
	case STEP_STOPPING:
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Waking started\r\n");
			mCurStep = STEP_WAKING;
			mLastUpdateTime = time;
			gWakingState.setRunMode(true);
		}
		break;
	case STEP_WAKING:
		if(!gWakingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Predicting: Checking started\r\n");
			mCurStep = STEP_CHECKING;
			mLastUpdateTime = time;
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CHECKING:
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			Debug::print(LOG_SUMMARY, "Predicting: Avoiding started\r\n");
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isWadachiExist(pImage))
			{
				//�Q�����O���m����
				gAvoidingState.setRunMode(true);
				mCurStep = STEP_AVOIDING;
			}else
			{
				mCurStep = STEP_RUNNING;
				gMotorDrive.startPID(0,MOTOR_MAX_POWER);
			}
		}
		break;
	case STEP_AVOIDING:
		if(!gAvoidingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Predicting: Avoiding finished\r\n");
			mCurStep = STEP_RUNNING;
			mLastUpdateTime = time;
		}
		break;
	}
}
bool WadachiPredicting::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("enable") == 0)
		{
			mIsAvoidingEnable = true;
			return true;
		}
		if(args[1].compare("disable") == 0)
		{
			mIsAvoidingEnable = false;
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "predicting [enable/disable]  : switch avoiding mode\r\n");
	return false;
}
bool WadachiPredicting::isWorking(const struct timespec& time)
{
	return mIsAvoidingEnable && (mCurStep != STEP_RUNNING || (mCurStep == STEP_RUNNING && Time::dt(time,mLastUpdateTime) < 6));
}
WadachiPredicting::WadachiPredicting() : mIsAvoidingEnable(false),mCurStep(STEP_RUNNING)
{
	setName("predicting");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
WadachiPredicting::~WadachiPredicting()
{
}

bool Escaping::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWARD;
	gMotorDrive.drive(-100,-100);
	gCameraCapture.setRunMode(true);
	gGyroSensor.setRunMode(true);
	mEscapingTriedCount = 0;
	return true;
}
void Escaping::onClean()
{
	gWakingState.setRunMode(false);
	gTurningState.setRunMode(false);
}
void Escaping::onUpdate(const struct timespec& time)
{
	const static unsigned int ESCAPING_MAX_CAMERA_ESCAPING_COUNT = 20;
	const static unsigned int ESCAPING_MAX_RANDOM_ESCAPING_COUNT = 20;
	switch(mCurStep)
	{
	case STEP_BACKWARD:
		//�o�b�N���s��
		if(Time::dt(time,mLastUpdateTime) >= 2)
		{
			Debug::print(LOG_SUMMARY, "Escaping: Backward finished!\r\n");
			mCurStep = STEP_AFTER_BACKWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_AFTER_BACKWARD:
		//�ċN���h�~�̂��ߑҋ@
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			if(mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
			{
				//�����_���ڍs
				Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
				mEscapingTriedCount = 0;
				mCurStep = STEP_RANDOM;
				mCurRandomStep = RANDOM_STEP_FORWARD;
				break;
			}
			mCurStep = STEP_PRE_CAMERA;
			mLastUpdateTime = time;
			//�N���オ�蓮����s��
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isSky(pImage))gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_CAMERA:
		//�摜�B�e�p�ɋN���オ�蓮����s���A���b�ҋ@����
		if(gWakingState.isActive())mLastUpdateTime = time;//�N���オ�蓮�쒆�͑ҋ@����
		if(Time::dt(time,mLastUpdateTime) > 2)//�N���オ�芮����A��莞�Ԃ��o�߂��Ă�����
		{
			Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
			//�摜�B�e������s��
			mCurStep = STEP_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CAMERA:
		//�摜�������s���A����̍s�������肷��
		if(Time::dt(time,mLastUpdateTime) >= 2)
		{
			Debug::print(LOG_SUMMARY, "Escaping: taking picture!\r\n");
			mLastUpdateTime = time;
			IplImage* pImage = gCameraCapture.getFrame();
			stuckMoveCamera(pImage);
			gCameraCapture.save(NULL,pImage);
			mAngle = gGyroSensor.getRz();
			++mEscapingTriedCount;
		}
		break;
	case STEP_CAMERA_TURN:
		//�摜�����̌��ʁA��]����K�v���������ꍇ
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_CAMERA_FORWARD;
			gMotorDrive.startPID(0,100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_FORWARD:
		//�摜�����̌��ʁA���i����K�v���������ꍇ
		if(Time::dt(time,mLastUpdateTime) >= 10)
		{
			gMotorDrive.drive(-100,-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		//�摜�����̌��ʁA���̏��]����K�v���������ꍇ
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100,-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//�����_������
		if(Time::dt(time,mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if(mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//�����_���ڍs
				mEscapingTriedCount = 0;
				mCurStep = STEP_BACKWARD;
				break;
			}
			stuckMoveRandom();
			mLastUpdateTime = time;

		}
		break;
	}
}
void Escaping::stuckMoveRandom()
{
	switch(mCurRandomStep)
	{
	case RANDOM_STEP_BACKWARD:
		//�o�b�N���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100,-100);
		break;
	case RANDOM_STEP_TURN:
		//���̏��]���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100,100);
		break;
	case RANDOM_STEP_FORWARD:
		//�O�i���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): forward\r\n");
		mCurRandomStep = RANDOM_STEP_BACKWARD;
		gMotorDrive.drive(-100,-100);
		break;
	}
}
void Escaping::stuckMoveCamera(IplImage* pImage)
{
	switch(gImageProc.wadachiExiting(pImage)){
		case -1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Left\r\n");
			gMotorDrive.drive(-100,100);
			mCurStep = STEP_CAMERA_TURN;
			break;
		case 1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Right\r\n");
			gMotorDrive.drive(100,-100);
			mCurStep = STEP_CAMERA_TURN;
			break;
		case 0:
            Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn here\r\n");
			gTurningState.setRunMode(true);
			gTurningState.setDirection(true);
			mCurStep = STEP_CAMERA_TURN_HERE;
			break;
		default://�J�����g���Ȃ�����
			mCurStep = STEP_RANDOM;
			mCurRandomStep = RANDOM_STEP_FORWARD;
			break;
		
	}
}
Escaping::Escaping()
{
	setName("escaping");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Escaping::~Escaping()
{
}

bool EscapingByStabi::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Escaping By Stabi Start! ");
	Time::showNowTime();
	
	mLastUpdateTime = time;
	gStabiServo.setRunMode(true);
	//gMotorDrive.drive(20,20);
	mFlag = false;
	mTryCount = 0;
	return true;
}
void EscapingByStabi::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) < 1) return;

	mLastUpdateTime = time;

	if(!mFlag)
	{
		gMotorDrive.drive(0,0);
		gStabiServo.start(mAngle);
	}
	else
	{
		gMotorDrive.drive(100,100);
		gStabiServo.start(STABI_BASE_ANGLE);
		mTryCount++;

		if(mTryCount % 5 == 0)
		{
			Debug::print(LOG_SUMMARY, "Escaping TryCount: %d\r\n", mTryCount);
		}
	}
	mFlag = !mFlag;
}
bool EscapingByStabi::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("start") == 0)
		{
			gEscapingByStabiState.setRunMode(true);
			return true;
		}
		if(args[1].compare("stop") == 0)
		{
			gMotorDrive.drive(0,0);
			Debug::print(LOG_SUMMARY, "Escaping By Stabi Finished!\r\n");
			gEscapingByStabiState.setRunMode(false);
			return true;
		}
	}
	else if(args.size() == 3)
	{
		if(args[1].compare("angle") == 0)
		{
			mAngle = atof(args[2].c_str());
			Debug::print(LOG_SUMMARY, "angle: %f\r\n", mAngle);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "escapingbystabi start       : start Escaping by stabi mode\r\n\
escapingbystabi stop        : stop  Escaping by stabi mode\r\n\
escapingbystabi angle [0-1] : set stabi angle\r\n");
	return false;
}
unsigned int EscapingByStabi::getTryCount()
{
	return mTryCount;
}
EscapingByStabi::EscapingByStabi()
{
	setName("escapingbystabi");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
EscapingByStabi::~EscapingByStabi()
{
}

bool EscapingRandom::onInit(const struct timespec& time)
{
		Debug::print(LOG_SUMMARY, "Start Escaping Random... ");
	Time::showNowTime();
	gStabiServo.setRunMode(true);
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWARD;
	gMotorDrive.drive(-100,-100);
	return true;
}
void EscapingRandom::onUpdate(const struct timespec& time)
{
	switch(mCurStep)
	{
	case STEP_BACKWARD:
		//�o�b�N���s��
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,-100);
			gStabiServo.start(0);					//�X�^�r������
		}
		break;
	case STEP_TURN:
		//���̏��]���s��
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,100);
			gStabiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	case STEP_FORWARD:
		//�O�i���s��
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(-100,-100);
			gStabiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	}
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
EscapingRandom::~EscapingRandom()
{
}

bool Waking::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_START;
	gMotorDrive.setRunMode(true);
	gMotorDrive.drive(18,18);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gStabiServo.setRunMode(true);
	mAngleOnBegin = gGyroSensor.getRz();
	mWakeRetryCount = 0;
	return true;
}
void Waking::onClean()
{
	gMotorDrive.drive(0,0);
}
void Waking::onUpdate(const struct timespec& time)
{
	double power;
	const static double WAKING_THRESHOLD = 200;
	switch(mCurStep)//�N���オ��J�n�����m���ꂽ�ꍇ
	{
	case STEP_STOP:
		if(Time::dt(time,mLastUpdateTime) > 2)//2�b�܂킵�Ă����n�����m����Ȃ��ꍇ�͂�����߂�
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
		}
		if(abs(gGyroSensor.getRvx()) < WAKING_THRESHOLD)//�p���x�����ȉ��ɂȂ����璅�n�Ɣ���
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}

		//��]�����p�x�ɉ����ă��[�^�̏o�͂�ω�������
		power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
		gMotorDrive.drive(power,power);
		break;

	case STEP_START:
		if(Time::dt(time,mLastUpdateTime) > 0.5)//��莞�ԉ�]�����m����Ȃ��ꍇ����]�s�\�Ɣ��f
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}
		if(abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)//��]�����m���ꂽ�ꍇ���N���オ��J�n�����Ɣ��f
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_STOP;
		}
		break;

	case STEP_VERIFY:
		//�N���オ�肪�����������ۂ����J�����摜�Ō���
		//�����x�g���D
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			// IplImage* pCaptureFrame = gCameraCapture.getFrame();
			// gCameraCapture.save(NULL,pCaptureFrame);
			// if(gImageProc.isSky(pCaptureFrame))
			// {
			// 	mLastUpdateTime = time;
			// 	mCurStep = STEP_START;
			// 	mAngleOnBegin = gGyroSensor.getRvx();
			// 	gMotorDrive.drive(50,50);

			// 	if(++mWakeRetryCount > WAKING_RETRY_COUNT)
			// 	{
			// 		Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
			// 		setRunMode(false);
			// 		return;
			// 	}
			// 	Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d)\r\n",mWakeRetryCount,WAKING_RETRY_COUNT);
			// }else
			// {
			// 	Debug::print(LOG_SUMMARY, "Waking Successed!\r\n");
			// 	setRunMode(false);
			// }
			if(gAccelerationSensor.getAz()>0.0)
			{
				Debug::print(LOG_SUMMARY,"Waking Successed!\r\n");
				setRunMode(false);
			}
			else
			{
				mLastUpdateTime = time;
				mCurStep = STEP_START;
				mAngleOnBegin = gGyroSensor.getRvx();
				gMotorDrive.drive(50,50);

				if(++mWakeRetryCount > WAKING_RETRY_COUNT)
				{
					Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
					setRunMode(false);
					return;
				}
				Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d)\r\n",mWakeRetryCount,WAKING_RETRY_COUNT);
			}

		}
		break;
	}
}
Waking::Waking() : mWakeRetryCount(0)
{
	setName("waking");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
{
}

bool Turning::onInit(const struct timespec& time)
{
	mTurnPower = 0;
	gGyroSensor.setRunMode(true);
	mAngle = gGyroSensor.getRz();
	mLastUpdateTime = time;
	return true;
}
void Turning::onUpdate(const struct timespec& time)
{
	double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	if(Time::dt(time,mLastUpdateTime) >= 5 || turnedAngle > 15)
	{
		Debug::print(LOG_SUMMARY, "Turning: Detected turning\r\n");
		gMotorDrive.drive(0,0);
		setRunMode(false);
	}else
	{
		if(mIsTurningLeft)gMotorDrive.drive(-mTurnPower,mTurnPower);
		else gMotorDrive.drive(mTurnPower,-mTurnPower);
		if(turnedAngle < 5)mTurnPower += 0.1;
	}
}
void Turning::setDirection(bool left)
{
	mIsTurningLeft = left;
}
Turning::Turning()
{
	setName("turning");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Turning::~Turning()
{
}

bool Avoiding::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	if(!gEscapingState.isActive())gMotorDrive.drive(0,50);
	mAngle = gGyroSensor.getRz();
	mCurStep = STEP_TURN;
	return true;
}
void Avoiding::onUpdate(const struct timespec& time)
{
	if(gEscapingState.isActive())
	{
		Debug::print(LOG_SUMMARY, "Avoiding: Escaping is already running. Avoiding Canceled!\r\n");
		setRunMode(false);
	}
	switch(mCurStep)
	{
	case STEP_TURN:
		if(Time::dt(time,mLastUpdateTime) > 5 || abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle)) > 45)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: forwarding\r\n");
			mLastUpdateTime = time;
			gMotorDrive.startPID(10,MOTOR_MAX_POWER);
			mCurStep = STEP_FORWARD;
		}
		break;
	case STEP_FORWARD:
		if(Time::dt(time,mLastUpdateTime) > 4)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: finished\r\n");
			setRunMode(false);
		}
		break;
	}
}
Avoiding::Avoiding()
{
	setName("avoiding");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Avoiding::~Avoiding()
{
}

bool PictureTaking::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	gCameraCapture.setRunMode(true);
	gBuzzer.setRunMode(true);
	gWakingState.setRunMode(true);
	mStepCount = 0;
	return true;
}
void PictureTaking::onUpdate(const struct timespec& time)
{
	if(gWakingState.isActive())return;
	if(Time::dt(time,mLastUpdateTime) > 1)
	{
		mLastUpdateTime = time;
		++mStepCount;
		gBuzzer.start(mStepCount > 25 ? 30 : 10);

		if(mStepCount == 25)
		{
			gCameraCapture.startWarming();
		}
		if(mStepCount >= 30)
		{
			Debug::print(LOG_SUMMARY, "Say cheese!\r\n");
			setRunMode(false);
			gBuzzer.start(300);
			gCameraCapture.save();
		}
	}
}
PictureTaking::PictureTaking() : mStepCount(0)
{
	setName("kinen");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
PictureTaking::~PictureTaking()
{
}

bool SensorLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameGPS,"Log started\r\n");
	write(mFilenameGyro,"Log started\r\n");
	write(mFilenamePressure,"Log started\r\n");
	write(mFilenameEncoder,"Log started\r\n");

	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	mLastEncL = gMotorDrive.getL();
	mLastEncR = gMotorDrive.getR();
	return true;
}
void SensorLogging::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) >= 1)
	{
		mLastUpdateTime = time;

		//���O��ۑ�
		VECTOR3 vec;
		gGPSSensor.get(vec);
		if(gGPSSensor.isActive())write(mFilenameGPS,"%f,%f,%f\r\n",vec.x,vec.y,vec.z);
		else write(mFilenameGPS,"unavailable\r\n");

		if(gGyroSensor.isActive())write(mFilenameGyro,"%f,%f,%f,%f,%f,%f\r\n",gGyroSensor.getRvx(),gGyroSensor.getRvy(),gGyroSensor.getRvz(),gGyroSensor.getRx(),gGyroSensor.getRy(),gGyroSensor.getRz());
		else write(mFilenameGyro,"unavailable\r\n");

		if(gPressureSensor.isActive())write(mFilenamePressure,"%d\r\n",gPressureSensor.get());
		else write(mFilenamePressure,"unavailable\r\n");

		if(gMotorDrive.isActive())
		{
			write(mFilenameEncoder,"%llu,%llu\r\n",gMotorDrive.getL() - mLastEncL,gMotorDrive.getR() - mLastEncR);
			mLastEncL = gMotorDrive.getL();
			mLastEncR = gMotorDrive.getR();
		}else write(mFilenameEncoder,"unavailable\r\n");
	}
}
void SensorLogging::write(const std::string& filename, const char* fmt, ... )
{
	std::ofstream of(filename.c_str(),std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
SensorLogging::SensorLogging() : mLastUpdateTime()
{
	setName("sensorlogging");
	setPriority(UINT_MAX,TASK_INTERVAL_SEQUENCE);

	Filename("log_gps",".txt").get(mFilenameGPS);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameGPS.c_str());
	Filename("log_gyro",".txt").get(mFilenameGyro);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameGyro.c_str());
	Filename("log_pressure",".txt").get(mFilenamePressure);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenamePressure.c_str());
	Filename("log_encoder",".txt").get(mFilenameEncoder);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameEncoder.c_str());
}
SensorLogging::~SensorLogging()
{
}

bool MovementLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameEncoder,"Log started\r\n");
	write(mFilenameAcceleration,"Log started\r\n");

	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	gMotorDrive.drive(100,100);
	return true;
}
void MovementLogging::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) < 1)
	{
		return;
	}
	mLastUpdateTime = time;

	//�����x�̃��O��ۑ�
	if(gAccelerationSensor.isActive())
	{
		write(mFilenameAcceleration,"%f,%f,%f\r\n",gAccelerationSensor.getAx(),gAccelerationSensor.getAy(),gAccelerationSensor.getAz());
	}
	else
	{
		write(mFilenameAcceleration,"unavailable\r\n");
	}

	if(!gMotorDrive.isActive())
	{
		write(mFilenameEncoder,"unavailable\r\n");
		return;
	}

	//�G���R�[�_�̃��O��ۑ�
	//���V�I�䂪�ύX���ꂽ��log�ɔ��f����
	if(gMotorDrive.getPowerL() != mPrevPowerL || gMotorDrive.getPowerR() != mPrevPowerR)
	{
		mPrevPowerL = gMotorDrive.getPowerL();
		mPrevPowerR = gMotorDrive.getPowerR();
		write(mFilenameEncoder,		"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
		Debug::print(LOG_SUMMARY,	"Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
	}

	//�G���R�[�_�p���X�̍����l�̎擾
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();	

	//��]���Ɋ��Z
	unsigned long long rotationsL = MotorEncoder::convertRotation(deltaPulseL);
	unsigned long long rotationsR = MotorEncoder::convertRotation(deltaPulseR);

	if(mPrintFlag)
	{
		Debug::print(LOG_SUMMARY,"Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);
	}
	write(mFilenameEncoder,	 	 "Pulse: %llu,%llu, Rotation: %llu,%llu\r\n",deltaPulseL,deltaPulseR,rotationsL,rotationsR);

	//�X�^�b�N����̃e�X�g
	if(mPrevDeltaPulseL >= STUCK_ENCODER_PULSE_THRESHOLD && mPrevDeltaPulseR >= STUCK_ENCODER_PULSE_THRESHOLD)	//�O��̃p���X����臒l�ȏ�
	{
		if(deltaPulseL < STUCK_ENCODER_PULSE_THRESHOLD && deltaPulseR < STUCK_ENCODER_PULSE_THRESHOLD)			//����̃p���X����臒l�ȉ�
		{
			write(mFilenameEncoder,		"Stuck detected!");
			if(mBuzzerFlag)
			{
				gBuzzer.start(200, 50 ,3);		//�X�^�b�N����(����炷�̂�)
			}
		}
	}
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
}
bool MovementLogging::onCommand(const std::vector<std::string> args)
{
	if(!gMovementLoggingState.isActive())
	{
		Debug::print(LOG_PRINT,"Movement Logging is not active\r\n");
		return true;
	}

	if(args.size() == 2)
	{
		if(args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT,"Command Executed!\r\n");
			gMotorDrive.drive(0,0);
			gMovementLoggingState.setRunMode(false);
			return true;
		}
		else if(args[1].compare("buzzer") == 0)
		{
			mBuzzerFlag = !mBuzzerFlag;	//flag�̐؂�ւ�
			if(mBuzzerFlag)
			{
				Debug::print(LOG_PRINT,"Command Executed! Buzzer(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT,"Command Executed! Buzzer(OFF)\r\n");
			}
			return true;
		}
		else if(args[1].compare("print") == 0)
		{
			mPrintFlag = !mPrintFlag;	//flag�̐؂�ւ�
			if(mPrintFlag)
			{
				Debug::print(LOG_PRINT,"Command Executed! Print(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT,"Command Executed! Print(OFF)\r\n");
			}
			return true;
		}
	}
	else if(args.size() == 3)
	{
		if(args[1].compare("comment") == 0)
		{
			std::string str = args[2];
			Debug::print(LOG_PRINT,"Command Executed! comment: %s\r\n", str.c_str());
			write(mFilenameAcceleration,"comment: %s\r\n", str.c_str());
			write(mFilenameEncoder,		"comment: %s\r\n", str.c_str());
			return true;
		}
	}
	
	Debug::print(LOG_PRINT,"movementlogging stop            : stop MovementLogging\r\n\
movementlogging buzzer          : switch buzzer\r\n\
movementlogging print           : switch pulse print\r\n\
movementlogging comment [string]: comment into log\r\n");
	return true;
}
void MovementLogging::write(const std::string& filename, const char* fmt, ... )
{
	std::ofstream of(filename.c_str(),std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
	//Debug::print(LOG_SUMMARY, "%s\r\n",buf);
}
MovementLogging::MovementLogging() : mLastUpdateTime(), mPrevPowerL(0), mPrevPowerR(0), mPrevDeltaPulseL(0), mPrevDeltaPulseR(0), mBuzzerFlag(true), mPrintFlag(false)
{
	setName("movementlogging");
	setPriority(UINT_MAX,TASK_INTERVAL_SEQUENCE);

	Filename("log_encoder",".txt").get(mFilenameEncoder);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameEncoder.c_str());
	Filename("log_acceleration",".txt").get(mFilenameAcceleration);
	Debug::print(LOG_SUMMARY, "%s\r\n",mFilenameAcceleration.c_str());
}
MovementLogging::~MovementLogging()
{
}