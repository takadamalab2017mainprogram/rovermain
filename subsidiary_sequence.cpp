#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <stdarg.h>
#include "subsidiary_sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"
#include "pose_detector.h"

Escaping gEscapingState;
EscapingRandom gEscapingRandomState;
EscapingByStabi gEscapingByStabiState;
Waking gWakingState;
//WakingFromLie gWakingFromLieState;
//Turning gTurningState;
//Avoiding gAvoidingState;
//WadachiPredicting gPredictingState;
//PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
//MovementLogging gMovementLoggingState;
//EncoderMonitoring gEncoderMonitoringState;
//CameraSave_Sequence gCameraSave_Sequence;


bool Escaping::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWARD;
	gMotorDrive.drive(-100);
	//gCameraCapture.setRunMode(true);
	//gGyroSensor.setRunMode(true);
	mEscapingTriedCount = 0;
	return true;
}
void Escaping::onClean()
{
	gWakingState.setRunMode(false);
	//gTurningState.setRunMode(false);
}
void Escaping::onUpdate(const struct timespec& time)
{
  const static unsigned int ESCAPING_MAX_CAMERA_ESCAPING_COUNT = 20;
  const static unsigned int ESCAPING_MAX_RANDOM_ESCAPING_COUNT = 20;
  switch (mCurStep)
    {
    case STEP_BACKWARD:
      //�o�b�N���s��
      if (Time::dt(time, mLastUpdateTime) >= 2)
	{
	  Debug::print(LOG_SUMMARY, "Escaping: Backward finished!\r\n");
	  mCurStep = STEP_AFTER_BACKWARD;
	  mLastUpdateTime = time;
	  gMotorDrive.drive(0);
	  //gCameraCapture.startWarming();
	}
      break;
    case STEP_AFTER_BACKWARD:
      //�ċN���h�~�̂��ߑҋ@
      if (Time::dt(time, mLastUpdateTime) >= 3)
	{
	  if (mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
	    {
	      //�����_���ڍs
	      Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
	      mEscapingTriedCount = 0;
	      mCurStep = STEP_RANDOM;
	      mCurRandomStep = RANDOM_STEP_FORWARD;
	      break;
	    }
	  //mCurStep = STEP_PRE_CAMERA;
	  mLastUpdateTime = time;
	  //�N���オ�蓮�����s��
	  //IplImage* pImage = gCameraCapture.getFrame();
	}
      //gCameraCapture.save(NULL, pImage);
      //if (gImageProc.isSky(pImage))gWakingState.setRunMode(true);
  break;
 case STEP_PRE_CAMERA:
   //�摜�B�e�p�ɋN���オ�蓮�����s���A���b�ҋ@����
   if (gWakingState.isActive())mLastUpdateTime = time;//�N���オ�蓮�쒆�͑ҋ@����
   if (Time::dt(time, mLastUpdateTime) > 2)//�N���オ�芮�����A���莞�Ԃ��o�߂��Ă�����
     {
       Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
       //�摜�B�e�������s��
       mCurStep = STEP_CAMERA;
       mLastUpdateTime = time;
       
       
       gMotorDrive.drive(0);
       //gCameraCapture.startWarming();
     }
   break;
   /*
 case STEP_CAMERA:
   //�摜�������s���A�����̍s�������肷��
   if (Time::dt(time, mLastUpdateTime) >= 2)
     {
       Debug::print(LOG_SUMMARY, "Escaping: taking picture!\r\n");
       mLastUpdateTime = time;
       IplImage* pImage = gCameraCapture.getFrame();
       stuckMoveCamera(pImage);
       //gCameraCapture.save(NULL, pImage);
       mAngle = gGyroSensor.getRz();
       ++mEscapingTriedCount;
     }
   break;
   */
 case STEP_CAMERA_TURN:
   //�摜�����̌��ʁA���]�����K�v���������ꍇ
	 /*
		if (Time::dt(time, mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		  {
		    //gCameraCapture.startWarming();
		    mCurStep = STEP_CAMERA_FORWARD;
		    gMotorDrive.drivePIDGyro(0, 100, true);
		    mLastUpdateTime = time;
		  }
		  */
		break;
 case STEP_CAMERA_FORWARD:
		//�摜�����̌��ʁA���i�����K�v���������ꍇ
   if (Time::dt(time, mLastUpdateTime) >= 10)
     {
			gMotorDrive.drive(-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		//�摜�����̌��ʁA���̏����]�����K�v���������ꍇ
		/*
		if (Time::dt(time, mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
		  //gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//�����_������
		if (Time::dt(time, mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if (mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//�����_���ڍs
				mEscapingTriedCount = 0;
				mCurStep = STEP_BACKWARD;
				break;
			}
			stuckMoveRandom();
			mLastUpdateTime = time;

		}
		*/
		break;
	}
}
void Escaping::stuckMoveRandom()
{
	switch (mCurRandomStep)
	{
	case RANDOM_STEP_BACKWARD:
		//�o�b�N���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100, -100);
		break;
	case RANDOM_STEP_TURN:
		//���̏����]���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100);
		break;
	case RANDOM_STEP_FORWARD:
		//�O�i���s��
		Debug::print(LOG_SUMMARY, "Escaping(random): forward\r\n");
		mCurRandomStep = RANDOM_STEP_BACKWARD;
		gMotorDrive.drive(-100);
		break;
	}
}
Escaping::Escaping()
{
	setName("escaping");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Escaping::~Escaping()
{
}

bool EscapingByStabi::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Escaping By Stabi Start! ");
	Time::showNowTime();

	mLastUpdateTime = time;
	gMultiServo.setRunMode(true);
	//gJohnServo.setRunMode(true);
	//gMotorDrive.drive(20);
	mFlag = false;
	mTryCount = 0;
	return true;
}
void EscapingByStabi::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < 1) return;

	mLastUpdateTime = time;

	if (!mFlag)
	{
		gMotorDrive.drive(30);
		gMultiServo.start(0);
	}
	else
	{
		gMotorDrive.drive(100);
		gMultiServo.start(1);
		mTryCount++;

		if (mTryCount % 5 == 0)
		{
			Debug::print(LOG_SUMMARY, "Escaping TryCount: %d\r\n", mTryCount);
		}
	}
	mFlag = !mFlag;
}
bool EscapingByStabi::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("start") == 0)
		{
			gEscapingByStabiState.setRunMode(true);
			//gSServo.start(0, 0);
			return true;
		}
		if (args[1].compare("s") == 0)
		{
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "Escaping By Stabi Finished!\r\n");
			gEscapingByStabiState.setRunMode(false);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "esc start       : start Escaping by stabi mode\r\n\esc s        : stop  Escaping by stabi mode\r\n");
	return false;
}
unsigned int EscapingByStabi::getTryCount()
{
	return mTryCount;
}
EscapingByStabi::EscapingByStabi()
{
	setName("esc1");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
EscapingByStabi::~EscapingByStabi()
{
}

bool EscapingRandom::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Start Escaping Random... ");
	Time::showNowTime();
	gMultiServo.setRunMode(true);
	mLastUpdateTime = time;
	mCurStep = STEP_TURN;
	gMotorDrive.drive(100, -100);
	return true;
}
void EscapingRandom::onUpdate(const struct timespec& time)
{
	switch (mCurStep)
	{
		//case STEP_BACKWARD:
		//	//�o�b�N���s��
		//	if(Time::dt(time,mLastUpdateTime) >= 3)
		//	{
		//		mCurStep = STEP_TURN;
		//		mLastUpdateTime = time;
		//		gMotorDrive.drive(100,-100);
		//		gMultiServo.start(0);					//�X�^�r������
		//	}
		//	break;
	case STEP_TURN:
		//���̏����]���s��
		if (Time::dt(time, mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100);
			//gMultiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	case STEP_FORWARD:
		//�O�i���s��
		if (Time::dt(time, mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100, -100);
			//gMultiServo.start(STABI_BASE_ANGLE);	//�X�^�r�L�΂�
		}
		break;
	}
}
EscapingRandom::EscapingRandom()
{
	setName("random");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
EscapingRandom::~EscapingRandom()
{
}

bool Waking::onInit(const struct timespec& time)
{
	mCurStep = STEP_CHECK_LIE;

	gMotorDrive.setRunMode(true);
	//gGyroSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	gMultiServo.start(BACK_STABI_RUN_ANGLE);
	gNineAxisSensor.setRunMode(true);
	//gSServo.setRunMode(true);
	//gSServo.moveRun();
	//gPoseDetecting.setRunMode(true);
	mWakeRetryCount = 0;

	return true;
}
void Waking::onClean()
{
	gMotorDrive.drive(0);
}
void Waking::onUpdate(const struct timespec& time)
{
	double power;
	const static double WAKING_THRESHOLD = 200;

	switch (mCurStep)
	{
	case STEP_CHECK_LIE:
		/*
		if (gPoseDetecting.isLie())
		{
			gWakingFromLieState.setRunMode(true);
			mCurStep = STEP_WAIT_LIE;
			return;
		}
		*/
		//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);//角度調節
		gMultiServo.start(BACK_STABI_RUN_ANGLE);
		//gArmServo.start(ARM_FOLD_ANGLE);
		//gNeckServo.start(NECK_FOLD_ANGLE);
		// Do following case without breaking
	case STEP_WAIT_LIE:
		//if (gWakingFromLieState.isActive())return;
		//begin waking
		gMotorDrive.drive(mStartPower);		//���[�^�o��
		mAngleOnBegin=gNineAxisSensor.getRz(); //gGyroSensor.getRz();

		mLastUpdateTime = time;
		mCurStep = STEP_START;
		break;
	case STEP_STOP:
		if (Time::dt(time, mLastUpdateTime) > 2)//2�b�܂킵�Ă����n�����m�����Ȃ��ꍇ�͂������߂�
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
			gMotorDrive.drive(0);
		}
		//if (gAccelerationSensor.getPhi() < mAngleThreshold)	//�p�x�������ȉ��ɂȂ����璅�n�Ɣ���(�����x�Z���T���̗p)
		if (gNineAxisSensor.getPhi() < mAngleThreshold)	//�p�x�������ȉ��ɂȂ����璅�n�Ɣ���(�����x�Z���T���̗p)
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);

		}

		//���]�����p�x�ɉ����ă��[�^�̏o�͂��ω�������
		//power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
		//gMotorDrive.drive(power);
		break;

		double dt;
	case STEP_START:
		if (Time::dt(time, mLastUpdateTime) > 0.5)//���莞�ԉ��]�����m�����Ȃ��ꍇ�����]�s�\�Ɣ��f
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		//if (abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)//���]�����m���ꂽ�ꍇ���N���オ���J�n�����Ɣ��f(�W���C�����̗p)
		if (abs(gNineAxisSensor.getRvx()) > WAKING_THRESHOLD)//���]�����m���ꂽ�ꍇ���N���オ���J�n�����Ɣ��f(�W���C�����̗p)
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_DEACCELERATE;
		}
		break;

	case STEP_DEACCELERATE:	//�������茸������
		dt = Time::dt(time, mLastUpdateTime);
		if (dt > mDeaccelerateDuration)
		{
			Debug::print(LOG_SUMMARY, "Waking Deaccelerate finished!\r\n");
			gBuzzer.start(30, 20, 2);
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0);
		}
		else
		{
			int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (mStartPower / 2/*2�Ŋ���*/)), 0);
			gMotorDrive.drive(tmp_power);
		}
		break;

	case STEP_VERIFY:
		//�N���オ�肪�����������ۂ�������
		if (Time::dt(time, mLastUpdateTime) <= 2.5)	//���[�o�̎p�������肷���܂ň��莞�ԑ҂�
		{
			return;
		}
		/*
		if (!gPoseDetecting.isFlip())
		{
			Debug::print(LOG_SUMMARY, "Waking Successed!\r\n");
			gBuzzer.start(30, 20, 4);
			setRunMode(false);
			//gJohnServo.start(FRONT_STABI_RUN_ANGLE);
			gMultiServo.start(BACK_STABI_RUN_ANGLE); // 角度調節
			//gArmServo.start(ARM_RUN_ANGLE);
			//gSServo.moveRun(); // �N���オ�萬���������X�^�r���x�[�X�̊p�x�ɖ߂�
		}
		else
		{
			mLastUpdateTime = time;
			mCurStep = STEP_START;
			mAngleOnBegin = gNineAxisSensor.getRvx();//gGyroSensor.getRvx();
			power = std::min((unsigned int)100, mStartPower + ((mWakeRetryCount + 1) * 5));	//���s�񐔂��ƂɃ��[�^�o�͂��グ��
			gMotorDrive.drive(power);

			if (++mWakeRetryCount > WAKING_RETRY_COUNT)
			{
				Debug::print(LOG_SUMMARY, "Waking Failed!\r\n");
				setRunMode(false);
				return;
			}
			//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
			gMultiServo.start(BACK_STABI_RUN_ANGLE); // 角度調節
			//gArmServo.start(ARM_FOLD_ANGLE);
			//gNeckServo.start(NECK_FOLD_ANGLE);
			Debug::print(LOG_SUMMARY, "Waking will be retried (%d / %d) by power %f\r\n", mWakeRetryCount, WAKING_RETRY_COUNT, power);
		}*/
		break;
	}
}
bool Waking::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 4)
	{
		if (args[1].compare("set") == 0)
		{
			if (args[2].compare("power") == 0)//mStartPower
			{
				setPower(atoi(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if (args[2].compare("angle") == 0)//mAngleThreshold
			{
				setAngle(atof(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if (args[2].compare("d_time") == 0)//mDeaccelerateDuration
			{
				mDeaccelerateDuration = atof(args[3].c_str());
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
		}
	}
	else if (args.size() == 2)
	{
		if (args[1].compare("show") == 0)
		{
			Debug::print(LOG_SUMMARY, "mStartPower: %d\r\n", mStartPower);
			Debug::print(LOG_SUMMARY, "mAngleThreshold: %f\r\n", mAngleThreshold);
			Debug::print(LOG_SUMMARY, "mDeaccelerateDuration: %f\r\n", mDeaccelerateDuration);
			return true;
		}
	}
	Debug::print(LOG_SUMMARY, "waking set power [1-100]: set start motor power\r\n\
							  							  waking set angle [0-180]: set AngleThreshold\r\n\
														  							  waking set d_time [time]: set mDeaccelerateDuration\r\n\
																					  							  waking show             : show parameters\r\n");
	return true;
}
void Waking::setPower(int p)
{
	if (p >= MOTOR_MAX_POWER)
	{
		mStartPower = MOTOR_MAX_POWER;
		return;
	}
	else if (p < 1)
	{
		mStartPower = 1;
		return;
	}
	mStartPower = p;
}
void Waking::setAngle(double a)
{
	if (a >= 180)
	{
		mAngleThreshold = 180;
	}
	else if (a < 0)
	{
		mAngleThreshold = 0;
	}
	mAngleThreshold = a;
}
Waking::Waking() : mWakeRetryCount(0), mStartPower(45), mAngleThreshold(70), mDeaccelerateDuration(0.5)
{
	setName("waking");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
{
}
/*
bool WakingFromLie::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mLastDtTime = time;
	mCurStep = STEP_FORWARD;
	mCurrentPower = 0;
	mNotLieCount = 0;

	gMotorDrive.setRunMode(true);
	//gSServo.setRunMode(true);
	//gSServo.start(0, 0);
	//gPoseDetecting.setRunMode(true);
	mWakeRetryCount = 0;

	return true;
}
void WakingFromLie::onUpdate(const struct timespec& time)
{
	double dt = Time::dt(time, mLastDtTime);
	mLastDtTime = time;

	switch (mCurStep)
	{
	case STEP_FORWARD:
		
		if (!gPoseDetecting.isLie())mNotLieCount++;
		else mNotLieCount = 0;
	
		gMotorDrive.drive(mCurrentPower, mCurrentPower);

		if (mNotLieCount > 100 || mCurrentPower >= MOTOR_MAX_POWER)
		{
			gMotorDrive.drive(0, 0);
			mCurStep = STEP_VERIFY;
			mLastUpdateTime = time;
		}
		mCurrentPower += MOTOR_MAX_POWER * dt * (WAKING_RETRY_COUNT + 1 - mWakeRetryCount) / (WAKING_RETRY_COUNT + 1) / mShortestSpeedUpPeriod;

		break;
	case STEP_VERIFY:
		if (Time::dt(time, mLastUpdateTime) > 3)
		{
			if (!gPoseDetecting.isLie())
			{
				Debug::print(LOG_SUMMARY, "WakingFromLie: Successed waking!!\r\n");
				gBuzzer.start(30, 20, 4);
				setRunMode(false);
				return;
			}

			if (mWakeRetryCount++ >= WAKING_RETRY_COUNT)
			{
				setRunMode(false);
				return;
			}
			Debug::print(LOG_SUMMARY, "WakingFromLie: Retrying (%d/%d)\r\n", mWakeRetryCount, WAKING_RETRY_COUNT);
			gMotorDrive.drive(MOTOR_MAX_POWER, MOTOR_MAX_POWER);
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			mCurrentPower = 0;
		}
		break;
	}
}
bool WakingFromLie::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		mShortestSpeedUpPeriod = atof(args[1].c_str());
		Debug::print(LOG_SUMMARY, "Command executed!\r\n");
		return true;
	}
	Debug::print(LOG_SUMMARY, "waking [period]: set speed up period\r\n");
	return true;
}
void WakingFromLie::onClean()
{
	gMotorDrive.drive(0);
	//gSServo.moveRun();
}
WakingFromLie::WakingFromLie() : mShortestSpeedUpPeriod(10)
{
	setName("wakinglie");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
WakingFromLie::~WakingFromLie()
{
}
*/

/*
bool Turning::onInit(const struct timespec& time)
{
	mTurnPower = 0;
	//gGyroSensor.setRunMode(true);
	mAngle=gNineAxisSensor.getRz();//gGyroSensor.getRz();
	mLastUpdateTime = time;
	return true;
}
void Turning::onUpdate(const struct timespec& time)
{
	//double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	double turnedAngle = abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle));
	if (Time::dt(time, mLastUpdateTime) >= 5 || turnedAngle > 15)
	{
		Debug::print(LOG_SUMMARY, "Turning: Detected turning\r\n");
		gMotorDrive.drive(0);
		setRunMode(false);
	}
	else
	{
		if (mIsTurningLeft)gMotorDrive.drive(-mTurnPower, mTurnPower);
		else gMotorDrive.drive(mTurnPower, -mTurnPower);
		if (turnedAngle < 5)mTurnPower += 0.1;
	}
}
void Turning::setDirection(bool left)
{
	mIsTurningLeft = left;
}
Turning::Turning()
{
	setName("turning");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Turning::~Turning()
{
}
*/

/*
bool Avoiding::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	if (!gEscapingState.isActive())gMotorDrive.drive(0, 50);
	mAngle = gGyroSensor.getRz();
	mCurStep = STEP_TURN;
	return true;
}
void Avoiding::onUpdate(const struct timespec& time)
{
	if (gEscapingState.isActive())
	{
		Debug::print(LOG_SUMMARY, "Avoiding: Escaping is already running. Avoiding Canceled!\r\n");
		setRunMode(false);
	}
	switch (mCurStep)
	{
	case STEP_TURN:
		if (Time::dt(time, mLastUpdateTime) > 5 || abs(GyroSensor::normalize(gGyroSensor.getRz() - mAngle)) > 45)
		{
			Debug::print(LOG_SUMMARY, "Avoiding: forwarding\r\n");
			mLastUpdateTime = time;
			gMotorDrive.drivePIDGyro(10, MOTOR_MAX_POWER, true);
			mCurStep = STEP_FORWARD;
		}
		break;
	case STEP_FORWARD:
		if (Time::dt(time, mLastUpdateTime) > 4)
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
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Avoiding::~Avoiding()
{
}
*/

bool SensorLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameGPS, "Log started\r\n");
	write(mFilenamePressure, "Log started\r\n");
	write(mFilenameNineAxis, "Log started\r\n");
	write(mFilenameNineAxis, "time/sec,Ax/G,Ay/G,Az/G,Rvx/deg/sec,Rvy/deg/sec,Rvz/deg/sec,Rx/deg,Ry/deg,Rz/deg,Mx/uT,My/uT,Mz/uT");

	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
    gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;
	return true;
}
void SensorLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) >= 0.1)
	{
		mLastUpdateTime = time;

		//���O���ۑ�
		VECTOR3 vec;
		gGPSSensor.get(vec);
		if (gGPSSensor.isActive())write(mFilenameGPS, "%ld.%ld,%f,%f,%f\r\n", time.tv_sec,time.tv_nsec,vec.x, vec.y, vec.z);
		else write(mFilenameGPS, "unavailable\r\n");

    if (gPressureSensor.isActive())write(mFilenamePressure, "%ld.%ld,%f\r\n", time.tv_sec,time.tv_nsec,gPressureSensor.get());
		else write(mFilenamePressure, "unavailable\r\n");


		if (gNineAxisSensor.isActive())write(mFilenameNineAxis, "%ld.%ld,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",time.tv_sec,time.tv_nsec, gNineAxisSensor.getAx(), gNineAxisSensor.getAy(), gNineAxisSensor.getAz(),gNineAxisSensor.getRvx(),gNineAxisSensor.getRvy(),gNineAxisSensor.getRvz(),gNineAxisSensor.getRx(),gNineAxisSensor.getRy(),gNineAxisSensor.getRz(),gNineAxisSensor.getMx(),gNineAxisSensor.getMy(),gNineAxisSensor.getMz());
		else write(mFilenameNineAxis, "unavailable\r\n");
	}
}
void SensorLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
SensorLogging::SensorLogging() : mLastUpdateTime()
{
	setName("sensorlogging");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	Filename("log_gps", ".txt").get(mFilenameGPS);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameGPS.c_str());
	Filename("log_pressure", ".txt").get(mFilenamePressure);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenamePressure.c_str());
	Filename("log_nineaxis", ".txt").get(mFilenameNineAxis);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameNineAxis.c_str());
}
SensorLogging::~SensorLogging()
{
}

/*
bool MovementLogging::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Log: Enabled\r\n");

	write(mFilenameEncoder, "Log started\r\n");
	write(mFilenameAcceleration, "Log started\r\n");

	//gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);
	mLastUpdateTime = time;
	gMotorDrive.drive(100);
	return true;
}
void MovementLogging::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateTime) < 1)
	{
		return;
	}
	mLastUpdateTime = time;

	//�����x�̃��O���ۑ�
	if (gAccelerationSensor.isActive())
	{
		write(mFilenameAcceleration, "%f,%f,%f\r\n", gAccelerationSensor.getAx(), gAccelerationSensor.getAy(), gAccelerationSensor.getAz());
	}
	else
	{
		write(mFilenameAcceleration, "unavailable\r\n");
	}

	if (!gMotorDrive.isActive())
	{
		write(mFilenameEncoder, "unavailable\r\n");
		return;
	}

	//�G���R�[�_�̃��O���ۑ�
	//���V�I�䂪�ύX���ꂽ��log�ɔ��f����
	if (gMotorDrive.getPowerL() != mPrevPowerL || gMotorDrive.getPowerR() != mPrevPowerR)
	{
		mPrevPowerL = gMotorDrive.getPowerL();
		mPrevPowerR = gMotorDrive.getPowerR();
		write(mFilenameEncoder, "Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
		Debug::print(LOG_SUMMARY, "Ratio Power has been changed!(%f, %f)\r\n", mPrevPowerL, mPrevPowerR);
	}

	//�G���R�[�_�p���X�̍����l�̎擾
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();

	//���]���Ɋ��Z
	unsigned long long rotationsL = MotorEncoder::convertRotation(deltaPulseL);
	unsigned long long rotationsR = MotorEncoder::convertRotation(deltaPulseR);

	if (mPrintFlag)
	{
		Debug::print(LOG_SUMMARY, "Pulse: %llu,%llu, Rotation: %llu,%llu\r\n", deltaPulseL, deltaPulseR, rotationsL, rotationsR);
	}
	write(mFilenameEncoder, "Pulse: %llu,%llu, Rotation: %llu,%llu\r\n", deltaPulseL, deltaPulseR, rotationsL, rotationsR);

	//�X�^�b�N�����̃e�X�g
	if (mPrevDeltaPulseL >= STUCK_ENCODER_PULSE_THRESHOLD && mPrevDeltaPulseR >= STUCK_ENCODER_PULSE_THRESHOLD)	//�O���̃p���X����臒l�ȏ�
	{
		if (deltaPulseL < STUCK_ENCODER_PULSE_THRESHOLD && deltaPulseR < STUCK_ENCODER_PULSE_THRESHOLD)			//�����̃p���X����臒l�ȉ�
		{
			write(mFilenameEncoder, "Stuck detected!");
			if (mBuzzerFlag)
			{
				gBuzzer.start(200, 50, 3);		//�X�^�b�N����(�����炷�̂�)
			}
		}
	}
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
}
bool MovementLogging::onCommand(const std::vector<std::string>& args)
{
	if (!gMovementLoggingState.isActive())
	{
		Debug::print(LOG_PRINT, "Movement Logging is not active\r\n");
		return true;
	}

	if (args.size() == 2)
	{
		if (args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			gMotorDrive.drive(0);
			gMovementLoggingState.setRunMode(false);
			return true;
		}
		else if (args[1].compare("buzzer") == 0)
		{
			mBuzzerFlag = !mBuzzerFlag;	//flag�̐؂��ւ�
			if (mBuzzerFlag)
			{
				Debug::print(LOG_PRINT, "Command Executed! Buzzer(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT, "Command Executed! Buzzer(OFF)\r\n");
			}
			return true;
		}
		else if (args[1].compare("print") == 0)
		{
			mPrintFlag = !mPrintFlag;	//flag�̐؂��ւ�
			if (mPrintFlag)
			{
				Debug::print(LOG_PRINT, "Command Executed! Print(ON)\r\n");
			}
			else
			{
				Debug::print(LOG_PRINT, "Command Executed! Print(OFF)\r\n");
			}
			return true;
		}
	}
	else if (args.size() == 3)
	{
		if (args[1].compare("comment") == 0)
		{
			std::string str = args[2];
			Debug::print(LOG_PRINT, "Command Executed! comment: %s\r\n", str.c_str());
			write(mFilenameAcceleration, "comment: %s\r\n", str.c_str());
			write(mFilenameEncoder, "comment: %s\r\n", str.c_str());
			return true;
		}
	}

	Debug::print(LOG_PRINT, "movementlogging stop            : stop MovementLogging\r\n\
														movementlogging buzzer          : switch buzzer\r\n\
																					movementlogging print           : switch pulse print\r\n\
																												movementlogging comment [string]: comment into log\r\n");
	return true;
}
void MovementLogging::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

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
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);

	Filename("log_encoder", ".txt").get(mFilenameEncoder);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameEncoder.c_str());
	Filename("log_acceleration", ".txt").get(mFilenameAcceleration);
	Debug::print(LOG_SUMMARY, "%s\r\n", mFilenameAcceleration.c_str());
}
MovementLogging::~MovementLogging()
{
}

/*

bool EncoderMonitoring::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "EncoderMonitoring: Start!\r\n");
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);

	mLastSamplingTime = time;
	mLastUpdateTime = time;
	mCurrentMaxPulse = 0;
	mPrevDeltaPulseL = 0;
	mPrevDeltaPulseR = 0;
	gMotorDrive.getDeltaPulseL();//�p���X�͍����Ȃ̂Ŏ擾���ă��Z�b�g���Ă���
	gMotorDrive.getDeltaPulseR();
	return true;
}
void EncoderMonitoring::onUpdate(const struct timespec& time)
{
	//���Ԃ��o�߂��Ă��Ȃ����Ώ������Ԃ�
	if (Time::dt(time, mLastSamplingTime) < 1) return;

	mLastSamplingTime = time;

	//�X�^�b�N���蒆�Ȃ�return
	if (gEscapingByStabiState.isActive()||gEscapingRandomState.isActive()
	    )
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}

	//�G���R�[�_�p���X�̍����l�̎擾
	long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	long long deltaPulseR = gMotorDrive.getDeltaPulseR();

	if (mIsPrint) Debug::print(LOG_SUMMARY, "EncoderMonitoring: current pulse count(%llu %llu)\r\n", deltaPulseL, deltaPulseR);

	//�O���l�͖�������
	if (removeError(deltaPulseL, deltaPulseR))
	{
		mPrevDeltaPulseL = 0;
		mPrevDeltaPulseR = 0;
		return;
	}

	//臒l�̌v�Z
	//	long long pulse_threshold = std::min(abs(mStoredPulse - mThresholdPulse), //ab//s(mUpperThreshold));
	//
	////�X�^�b�N�`�F�b�N�D�O����臒l�ȏ��ŁC������臒l�ȉ��Ȃ��X�^�b�N���肷��
	//if (mPrevDeltaPulseL >= pulse_threshold && mPrevDeltaPulseR >= pulse_threshold)	//�O���̃p���X����臒l�ȏ�
	//{
	//	//if (deltaPulseL < pulse_threshold || deltaPulseR < pulse_threshold)			//�����̃p���X����臒l�ȉ�
	//	if ((deltaPulseL + deltaPulseR) / 2 < pulse_threshold)			//�����̃p���X����臒l�ȉ�
	//	{
	//		//�X�^�b�N����
	//		gBuzzer.start(80, 10, 6);
	//		Debug::print(LOG_SUMMARY, "EncoderMonitoring: STUCK detected by pulse count(%llu %llu). Threshold:%llu\r\n", deltaPulseL, deltaPulseR, pulse_threshold);
	//		gEscapingByStabiState.setRunMode(true);
	//		setRunMode(false);
	//		return;
	//	}
	//}

	//�O���̃p���X�̍X�V
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;

	//mCurrentMaxPulse�����傫������mCurrentMaxPulse���X�V����
	if (std::max(deltaPulseL, deltaPulseR) > mCurrentMaxPulse)
	{
		mCurrentMaxPulse = std::max(deltaPulseL, deltaPulseR);
	}

	//臒l�̍X�V
	if (Time::dt(time, mLastUpdateTime) >= mUpdateTimer)
	{
		updateThreshold();
		mCurrentMaxPulse = 0;
		mLastUpdateTime = time;
	}
}
bool EncoderMonitoring::onCommand(const std::vector<std::string>& args)
{
	//if(!gEncoderMonitoringState.isActive())
	//{
	//	Debug::print(LOG_PRINT,"EncoderMonitoring is not active\r\n");
	//	return true;
	//}

	if (args.size() == 2)
	{
		if (args[1].compare("stop") == 0)
		{
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			gMotorDrive.drive(0);
			//gEncoderMonitoringState.setRunMode(false);
			return true;
		}
		else if (args[1].compare("print") == 0)
		{
			mIsPrint = !mIsPrint;
			if (mIsPrint) Debug::print(LOG_PRINT, "Print ON!\r\n");
			else Debug::print(LOG_PRINT, "Print OFF!\r\n");
			return true;
		}
		else if (args[1].compare("show") == 0)
		{
			Debug::print(LOG_PRINT, "Command Executed!\r\n\n");

			Debug::print(LOG_PRINT, "UpdateTimer           : %d\r\n\
																		StoredPulse           : %llu\r\n\
																											ThresholdPulse        : %llu\r\n\
																																				IgnoredDeltaUpperPulse: %llu\r\n\
																																													IgnoredDeltaLowerPulse: %llu\r\n\
																																																						UpperThreshold        : %llu\r\n\
																																																															LowerThreshold        : %llu\r\n", mUpdateTimer, mStoredPulse, mThresholdPulse, mIgnoredDeltaUpperPulse, mIgnoredDeltaLowerPulse, mUpperThreshold, mLowerThreshold);
			return true;
		}
	}
	else if (args.size() == 4)
	{
		if (args[1].compare("set") == 0)
		{
			if (args[2].compare("timer") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mUpdateTimer = atoi(args[3].c_str());
				return true;
			}
			else if (args[2].compare("stored") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mStoredPulse = atol(args[3].c_str());
				return true;
			}
			else if (args[2].compare("threpulse") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mThresholdPulse = atol(args[3].c_str());
				return true;
			}
			else if (args[2].compare("deltaupper") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mIgnoredDeltaUpperPulse = atol(args[3].c_str());
				return true;
			}
			else if (args[2].compare("deltalower") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mIgnoredDeltaLowerPulse = atol(args[3].c_str());
				return true;
			}
			else if (args[2].compare("upper") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mUpperThreshold = atol(args[3].c_str());
				return true;
			}
			else if (args[2].compare("lower") == 0)
			{
				Debug::print(LOG_PRINT, "Command Executed!\r\n");
				mLowerThreshold = atol(args[3].c_str());
				return true;
			}
		}
	}

	Debug::print(LOG_PRINT, "monitoring stop                   : stop EncoderMonitoring\r\n\
														monitoring set timer [second]     : set update timer[s]\r\n\
																					monitoring set stored [pulse]     : set StoredPulse\r\n\
																												monitoring set threpulse [pulse]  : set Threshold Pulse\r\n\
																																			monitoring set deltaupper [pulse] : set IgnoredDeltaUpperPulse\r\n\
																																										monitoring set deltalower [pulse] : set IgnoredDeltaLowerPulse\r\n\
																																																	monitoring set upper [pulse]      : set UpperThreshold\r\n\
																																																								monitoring set lower [pulse]      : set LowerThreshold\r\n\
																																																															monitoring print                  : switch print\r\n\
																																																																						monitoring show                   : show each value\r\n");
	return true;
}
void EncoderMonitoring::updateThreshold()
{
	//�p���X�������܂��ɂ�������(or�傫��)�ꍇ�͖�������
	if (mCurrentMaxPulse <= mLowerThreshold)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: threshold update is ignored. mCurrentMaxPulse(%llu) <= mLowerThreshold(%llu)\r\n", mCurrentMaxPulse, mLowerThreshold);
		return;
	}
	else if (mCurrentMaxPulse >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: threshold update is ignored. mCurrentMaxPulse(%llu) <= mUpperThreshold(%llu)\r\n", mCurrentMaxPulse, mUpperThreshold);
		return;
	}

	//���܂��ɂ��傫��臒l���X�V�������ꍇ�͖�������
	if ((mStoredPulse >= mCurrentMaxPulse) && (mStoredPulse - mCurrentMaxPulse) >= mIgnoredDeltaLowerPulse)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: threshold update is ignored. %llu >= mIgnoredDeltaLowerPulse(%llu))\r\n", (mStoredPulse - mCurrentMaxPulse), mIgnoredDeltaLowerPulse);
		return;
	}
	else if ((mStoredPulse < mCurrentMaxPulse) && (mCurrentMaxPulse - mStoredPulse) >= mIgnoredDeltaUpperPulse)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: threshold update is ignored. %llu >= mIgnoredDeltaUpperPulse(%llu))\r\n", (mCurrentMaxPulse - mStoredPulse), mIgnoredDeltaUpperPulse);
		return;
	}

	mStoredPulse = mCurrentMaxPulse;
	Debug::print(LOG_SUMMARY, "EncoderMonitoring: StoredPulse is updated! -> %llu\r\n", mStoredPulse);
}
bool EncoderMonitoring::removeError(long long pulseL, long long pulseR)
{
	bool ret = false;
	//L���`�F�b�N
	if (pulseL == 0)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: [ERROR] Left pulse is zero...\r\n");
		ret = true;
	}
	else if (pulseL >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: Left pulse is error... pulseL: %llu, UpperThreshold: %llu\r\n", pulseL, mUpperThreshold);
		ret = true;
	}

	//R���`�F�b�N
	if (pulseR == 0)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: [ERROR] Right pulse is zero...\r\n");
		ret = true;
	}
	else if (pulseR >= mUpperThreshold)
	{
		Debug::print(LOG_SUMMARY, "EncoderMonitoring: Right pulse is error... pulseR: %llu, UpperThreshold: %llu\r\n", pulseR, mUpperThreshold);
		ret = true;
	}
	return ret;
}
// mThresholdPulse:1000->0
EncoderMonitoring::EncoderMonitoring() : mLastSamplingTime(), mLastUpdateTime(), mStoredPulse(2500), mUpdateTimer(30), mThresholdPulse(1000), mIgnoredDeltaUpperPulse(1500), mIgnoredDeltaLowerPulse(800), mUpperThreshold(3000), mLowerThreshold(1500), mIsPrint(false)
{
	setName("monitoring");
	setPriority(UINT_MAX, TASK_INTERVAL_SEQUENCE);
}
EncoderMonitoring::~EncoderMonitoring()
{
}
*/
/*
bool CameraSave_Sequence::onInit(const struct timespec& time) {
	Debug::print(LOG_SUMMARY, "Start Camera Save Sequence...");
	Time::showNowTime();
	gCameraCapture.setRunMode(true);
	mLastUpdateTime = time;
	timing = 1.0;
	mIsUpdateCamera = false;
	mIsUpdateWadati = false;
	return true;
}
void CameraSave_Sequence::onUpdate(const struct timespec& time) {
	if (mIsUpdateCamera) {
		if (Time::dt(time, mLastUpdateTime) > timing) {
			Time::showNowTime();
			mLastUpdateTime = time;
			gCameraCapture.startWarming();
			if (mIsUpdateWadati) {
				gCameraCapture.wadatisave();
			}
			else {
				gCameraCapture.save();
			}
		}
	}
}
bool CameraSave_Sequence::onCommand(const std::vector<std::string>& args) {
	if (args.size() == 2) {
		if (args[1].compare("stop") == 0) {
			setRunMode(false);
			return true;
		}
		else if (args[1].compare("start") == 0) {
			mIsUpdateCamera = true;
			return true;
		}
	}
	else if (args.size() == 3) {
		if (args[1].compare("timing") == 0) {
			timing = atof(args[2].c_str());
			Debug::print(LOG_SUMMARY, "Camera Save timing is %f\n", timing);
			return true;
		}
		else if (args[1].compare("wadati") == 0) {
			if (args[2].compare("start") == 0) {
				mIsUpdateWadati = true;
				return true;
			}
			else if (args[2].compare("stop") == 0) {
				mIsUpdateWadati = false;
				return true;
			}
		}
	}
	Debug::print(LOG_SUMMARY, "camera_s\n"
		"start : camera start\n"
		"stop : camera stop\n"
		"timing [number] : timing setting\n"
		"wadati [start/stop] : wadati detection [start/stop]\n");
	return false;
}
CameraSave_Sequence::CameraSave_Sequence() {
	setName("camera_s");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
CameraSave_Sequence::~CameraSave_Sequence() {
}
*/
