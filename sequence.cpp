#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <functional>
//#include <opencv2/opencv.hpp>
//#include <opencv/cvaux.h>
//#include <opencv/highgui.h>
#include <stdarg.h>
#include <wiringPi.h>
#include "sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"
//#include "image_proc.h"
#include "subsidiary_sequence.h"
#include "delayed_execution.h"
#include "constants.h"

Testing gTestingState;
Waiting gWaitingState;
Falling gFallingState;
Separating gSeparatingState;
Navigating gNavigatingState;
//Modeling gModelingState;
//ColorAccessing gColorAccessingState;
//extern Filename gCaptureFilename;

//////////////////////////////////////////////
// Testing
//////////////////////////////////////////////

bool Testing::onInit(const struct timespec& time)
{
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gParaServo.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	gDelayedExecutor.setRunMode(true);

	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	//gWebCamera.setRunMode(true);
	//gDistanceSensor.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	//gCameraSave_Sequence.setRunMode(true);

	gMotorDrive.setRunMode(true);

	gSerialCommand.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	std::function<void()> f = [&]()
	{
		gBuzzer.start(200);
	};
	auto pExecutable = new DelayedExecutableFunction(f, 1000);
	gDelayedExecutor.add(std::shared_ptr<DelayedExecutable>(pExecutable));

	return true;
}
bool Testing::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("sensor") == 0)
		{
			Debug::print(LOG_SUMMARY, "*** Sensor states ***\r\n");

			VECTOR3 vec;
			gGPSSensor.get(vec);
			if (gGPSSensor.isActive())Debug::print(LOG_SUMMARY, " GPS      (%f %f %f)\r\n", vec.x, vec.y, vec.z);
			else Debug::print(LOG_SUMMARY, " GPS is NOT working\r\n");

			if (gPressureSensor.isActive())Debug::print(LOG_SUMMARY, " Pressure (%f) hPa\r\n", gPressureSensor.get());
			else Debug::print(LOG_SUMMARY, " Pressure is NOT working\r\n");

			if (gGyroSensor.isActive())
			{
				gGyroSensor.getRPos(vec);
				Debug::print(LOG_SUMMARY, " Gyro pos (%f %f %f) d\r\n", vec.x, vec.y, vec.z);
				gGyroSensor.getRVel(vec);
				Debug::print(LOG_SUMMARY, " Gyro vel (%f %f %f) dps\r\n", vec.x, vec.y, vec.z);
			}
			else Debug::print(LOG_SUMMARY, " Gyro is NOT working\r\n");

			if (gAccelerationSensor.isActive())
			{
				gAccelerationSensor.getAccel(vec);
				Debug::print(LOG_SUMMARY, " Accel val (%f %f %f) d\r\n", vec.x, vec.y, vec.z);
			}
			else Debug::print(LOG_SUMMARY, " Accel is NOT working\r\n");

			if (gLightSensor.isActive())Debug::print(LOG_SUMMARY, " Light    (%s)\r\n", gLightSensor.get() ? "High" : "Low");
			else Debug::print(LOG_SUMMARY, " Light is NOT working\r\n");

			return true;
		}
		else if (args[1].compare("waking") == 0)
		{
			gWakingState.setRunMode(true);
		}
		else if (args[1].compare("time") == 0)
		{
			Time::showNowTime();
			return true;
		}
		else if (args[1].compare("version") == 0)
		{
			Debug::print(LOG_SUMMARY, "Version: %d\r\n", VERSION);
			return true;
		}
	}
	if (args.size() == 3)
	{
		if (args[1].compare("start") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if (pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Start %s\r\n", args[2].c_str());
				pTask->setRunMode(true);
				return true;
			}
			else Debug::print(LOG_SUMMARY, "%s Not Found\r\n", args[2].c_str());
			return false;
		}
		else if (args[1].compare("stop") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if (pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Stop %s\r\n", args[2].c_str());
				pTask->setRunMode(false);
				return true;
			}
			else Debug::print(LOG_SUMMARY, "%s Not Found\r\n", args[2].c_str());
			return false;
		}
	}
	if (args.size() == 4) {
		if (args[1].compare("pin") == 0)
		{
			if ((int)atof(args[3].c_str()) == 0 || (int)atof(args[3].c_str()) == 1)
			{
				pinMode((int)atof(args[2].c_str()), OUTPUT);
				digitalWrite((int)atof(args[2].c_str()), (int)atof(args[3].c_str()));
				Debug::print(LOG_SUMMARY, "digitalWrite(%d,%d)\r\n", (int)atof(args[2].c_str()), (int)atof(args[3].c_str()));
				return true;
			}
		}
	}
	Debug::print(LOG_PRINT, "testing [start/stop] [task name]  : enable/disable task\r\n\
							testing time                      : show current time\r\n\
							testing sensor                    : check sensor values\r\n\
							testing pin [PinNum] [output(0|1)]: digitalWrite(PinNum,output)\r\n\
							testing version                   : show program version\r\n");

	return true;
}
Testing::Testing()
{
	setName("testing");
	setPriority(UINT_MAX, UINT_MAX);
}
Testing::~Testing()
{
}

//////////////////////////////////////////////
// Waiting
//////////////////////////////////////////////

bool Waiting::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Waiting... ");
	Time::showNowTime();

	mContinuousLightCount = 0;

	//現在の時刻を保存
	mStartTime = time;

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gLightSensor.setRunMode(true);
	//gXbeeSleep.setRunMode(true);//Xbeeをスリープモードにするならコメントアウトを解除すること
	//Debug::print(LOG_SUMMARY, "Disable Communication\r\ncya!\r\n");
	gSerialCommand.setRunMode(true);//Xbeeをスリープモードにするならコメントアウトすること
	gBuzzer.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
	gMikeServo.start(BACK_STABI_FOLD_ANGLE);
	return true;
}
void Waiting::nextState()
{
	gBuzzer.start(100);

	//スリープを解除
	//gXbeeSleep.setState(false);//Xbeeをスリープモードにするならコメントアウトを解除すること

	//次の状態を設定
	gFallingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Waiting Finished!\r\n");
}
void Waiting::onUpdate(const struct timespec& time)
{
	//XBeeをスリープモードに設定(ロケット内電波規制)
	//gXbeeSleep.setState(true);//Xbeeをスリープモードにするならコメントアウトを解除すること

	//明るい場合カウント
	if (gLightSensor.get())
	{
		++mContinuousLightCount;
		gBuzzer.start(10);
	}
	else mContinuousLightCount = 0;

	if (mContinuousLightCount >= WAITING_LIGHT_COUNT)//明るい場合放出判定
	{
		nextState();
		return;
	}

	if (Time::dt(time, mStartTime) > WAITING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
		nextState();
		return;
	}
}
Waiting::Waiting()
{
	setName("waiting");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Waiting::~Waiting() {}

//////////////////////////////////////////////
// Falling
//////////////////////////////////////////////

bool Falling::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Falling... ");
	Time::showNowTime();

	mStartTime = mLastCheckTime = time;
	mLastPressure = 0;
	mContinuousPressureCount = 0;
	mCoutinuousGyroCount = 0;

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gParaServo.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
	gMikeServo.start(BACK_STABI_FOLD_ANGLE);
	//gSServo.setRunMode(true);

	return true;
}
void Falling::onUpdate(const struct timespec& time)
{
	struct timespec curTime;
	Time::get(curTime);

	//初回処理
	if (mLastPressure == 0)
	{
		mLastPressure = gPressureSensor.get();
		gParaServo.moveHold();
		//gSServo.moveFold();//スタビを格納状態で固定
		gJohnServo.start(FRONT_STABI_FOLD_ANGLE); // 角度調節
		gMikeServo.start(BACK_STABI_FOLD_ANGLE);
		//gNeckServo.start(0.5);
	}

	//閾値以下ならカウント
	if (abs(gGyroSensor.getRvx()) < FALLING_GYRO_THRESHOLD && abs(gGyroSensor.getRvy()) < FALLING_GYRO_THRESHOLD && abs(gGyroSensor.getRvz()) < FALLING_GYRO_THRESHOLD)
	{
		if (mCoutinuousGyroCount < FALLING_GYRO_COUNT)++mCoutinuousGyroCount;
	}
	else mCoutinuousGyroCount = 0;

	//1秒ごとに以下の処理を行う
	if (Time::dt(time, mLastCheckTime) < 1)return;
	mLastCheckTime = time;

	//気圧の差が一定以下ならカウント
	int newPressure = gPressureSensor.get();
	if (abs((int)(newPressure - mLastPressure)) < FALLING_DELTA_PRESSURE_THRESHOLD)
	{
		if (mContinuousPressureCount < FALLING_PRESSURE_COUNT)++mContinuousPressureCount;
	}
	else mContinuousPressureCount = 0;
	mLastPressure = newPressure;

	//エンコーダの値の差が一定以上ならカウント
	long long newMotorPulseL = gMotorDrive.getL(), newMotorPulseR = gMotorDrive.getR();
	if (abs(newMotorPulseL - mLastMotorPulseL) > FALLING_MOTOR_PULSE_THRESHOLD && abs(newMotorPulseR - mLastMotorPulseR) > FALLING_MOTOR_PULSE_THRESHOLD)
	{
		if (mContinuousMotorPulseCount < FALLING_MOTOR_PULSE_COUNT)++mContinuousMotorPulseCount;
	}
	else mContinuousMotorPulseCount = 0;

	//判定状態を表示
	Debug::print(LOG_SUMMARY, "Pressure Count   %d / %d (%d hPa)\r\n", mContinuousPressureCount, FALLING_PRESSURE_COUNT, newPressure);
	Debug::print(LOG_SUMMARY, "Gyro Count       %d / %d\r\n", mCoutinuousGyroCount, FALLING_GYRO_COUNT);
	Debug::print(LOG_SUMMARY, "MotorPulse Count %d / %d (%lld,%lld)\r\n", mContinuousMotorPulseCount, FALLING_MOTOR_PULSE_COUNT, newMotorPulseL - mLastMotorPulseL, newMotorPulseR - mLastMotorPulseR);

	mLastMotorPulseL = newMotorPulseL;
	mLastMotorPulseR = newMotorPulseR;

	//GPS情報ログ
	VECTOR3 pos;
	if (gGPSSensor.get(pos))Debug::print(LOG_SUMMARY, "GPS Position     (%f %f %f)\r\n", pos.x, pos.y, pos.z);
	else Debug::print(LOG_SUMMARY, "GPS Position     Unable to get\r\n");

	//カウント回数が一定以上なら次の状態に移行
	if (mContinuousPressureCount >= FALLING_PRESSURE_COUNT && (mCoutinuousGyroCount >= FALLING_GYRO_COUNT || mContinuousMotorPulseCount >= FALLING_MOTOR_PULSE_COUNT))
	{
		nextState();
		return;
	}

	if (Time::dt(time, mStartTime) > FALLING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
	{
		Debug::print(LOG_SUMMARY, "Falling Timeout\r\n");
		nextState();
		return;
	}
}
void Falling::nextState()
{
	gBuzzer.start(100);

	//次の状態を設定
	gSeparatingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Falling Finished!\r\n");
}
Falling::Falling() : mLastPressure(0), mLastMotorPulseL(0), mLastMotorPulseR(0), mContinuousPressureCount(0), mCoutinuousGyroCount(0), mContinuousMotorPulseCount(0)
{
	setName("falling");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Falling::~Falling()
{
}

//////////////////////////////////////////////
// Separating
//////////////////////////////////////////////

bool Separating::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Separating... ");
	Time::showNowTime();

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gBuzzer.setRunMode(true);
	//gSServo.setRunMode(true);
	gParaServo.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	mLastUpdateTime = time;
	mCurServoState = false;
	mServoCount = 0;
	mCurStep = STEP_STABI_OPEN;

	return true;
}
void Separating::onUpdate(const struct timespec& time)
{
	switch (mCurStep)
	{
	case STEP_STABI_OPEN:
		gParaServo.moveHold();
		//gJohnServo.start(20); // 角度調節
		//gMikeServo.start(20);
		//gSServo.moveRun();//スタビを走行時の位置に移動

		mCurStep = STEP_WAIT_STABI_OPEN;
		mLastUpdateTime = time;
		break;
	case STEP_WAIT_STABI_OPEN:
		if (Time::dt(time, mLastUpdateTime) > 0.5)//スタビ展開動作後待機する
		{
			//次状態に遷移
			mLastUpdateTime = time;
			mCurStep = STEP_SEPARATE;
		}
		break;
	case STEP_SEPARATE:
		//パラシュートを切り離す
		if (Time::dt(time, mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
		mLastUpdateTime = time;

		mCurServoState = !mCurServoState;

		if (mCurServoState)
		{
			gParaServo.moveRelease();
		}
		else
		{
			gParaServo.moveHold();
		}

		++mServoCount;
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if (mServoCount >= SEPARATING_SERVO_COUNT)//サーボを規定回数動かした
		{
			//次状態に遷移
			gParaServo.stop();
			mLastUpdateTime = time;
			mCurStep = STEP_PRE_PARA_JUDGE;
			gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_PARA_JUDGE:
		//起き上がり動作を実行し、画像処理を行う前に1秒待機して画像のブレを防止する
		if (gWakingState.isActive())mLastUpdateTime = time;//起き上がり動作中は待機する
		if (Time::dt(time, mLastUpdateTime) > 1)//起き上がり動作後1秒待機する
		{
			//次状態に遷移
			mLastUpdateTime = time;
			mCurStep = STEP_PARA_JUDGE;
			//gCameraCapture.startWarming();
		}
		break;
	case STEP_PARA_JUDGE:
		//ローバーを起こし終わったら，パラシュート検知を行い，存在する場合は回避行動に遷移する
		if (Time::dt(time, mLastUpdateTime) > 2)
		{
			//パラシュートの存在チェックを行う
			//IplImage* pImage = gCameraCapture.getFrame();
			//if (gImageProc.isParaExist(pImage))
			//{
				//回避動作に遷移
			//	gBuzzer.start(20, 20, 5);
			//	mCurStep = STEP_PARA_DODGE;
			//	mLastUpdateTime = time;
			//	gTurningState.setRunMode(true);
			//	Debug::print(LOG_SUMMARY, "Para check: Found!!\r\n");
			//}
			//else
			//{
				//次状態(ナビ)に遷移
		  //	Debug::print(LOG_SUMMARY, "Para check: Not Found!!\r\n");
		  //		nextState();
		  //	}
			//パラ検知に用いた画像を保存する
			//gCameraCapture.save(NULL, pImage);
		}
		break;
	case STEP_PARA_DODGE:
		if (!gTurningState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Para check: Turn Finished!\r\n");
			gMotorDrive.drive(100);
			mLastUpdateTime = time;
			mCurStep = STEP_GO_FORWARD;
		}
		break;
	case STEP_GO_FORWARD:	//パラ検知後，しばらく直進する
		if (Time::dt(time, mLastUpdateTime) > 3)
		{
			gMotorDrive.drive(0);
			nextState();
		}
		break;
	};
}

void Separating::nextState()
{
	//ブザー鳴らしとく
	gBuzzer.start(100);

	//次の状態を設定
	gNavigatingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Separating Finished!\r\n");
}
Separating::Separating() : mCurServoState(false), mServoCount(0)
{
	setName("separating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Separating::~Separating()
{
}

//////////////////////////////////////////////
// Navigating
//////////////////////////////////////////////

//ゴールへの移動中
bool Navigating::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Navigating...\r\n");

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gEncoderMonitoringState.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	gJohnServo.start(FRONT_STABI_RUN_ANGLE); // 角度調節
	gMikeServo.start(BACK_STABI_RUN_ANGLE);
	gArmServo.start(ARM_RUN_ANGLE);
	//gNeckServo.start(1);
	//gSServo.setRunMode(true);
	//gSServo.moveRun();		//スタビを走行時の位置に移動

	mLastNaviMoveCheckTime = time;
	mLastArmServoMoveTime = time;
	mLastArmServoStopTime  = time;
	mArmStopFlag = true;

	mLastPos.clear();
	return true;
}
void Navigating::onUpdate(const struct timespec& time)
{
	//gArmServo.start(ARM_RUN_ANGLE);
	gNeckServo.start(NECK_RUN_ANGLE);
	VECTOR3 currentPos;

	//ゴールが設定されているか確認
	if (!mIsGoalPos)
	{
		//ゴールが設定されていないため移動できない
		Debug::print(LOG_SUMMARY, "NAVIGATING : Please set goal!\r\n");
		gMotorDrive.drive(0);
		nextState();
		return;
	}

	if (Time::dt(time, mLastArmServoStopTime) > 10.0 && mArmMoveFlag == true){
                mLastArmServoMoveTime = time;
                gArmServo.start(ARM_RUN_ANGLE);
                mArmMoveFlag = false;
                mArmStopFlag = true;
        }
        if(Time::dt(time, mLastArmServoMoveTime) > 0.5 && mArmStopFlag == true){
                gArmServo.stop();
                mLastArmServoStopTime = time;
                mArmStopFlag = false;
                mArmMoveFlag = true;
        }

	bool isNewData = gGPSSensor.isNewPos();
	//新しい位置を取得できなければ処理を返す
	if (!gGPSSensor.get(currentPos, false))return;


	//新しい座標であればバッファに追加
	if (isNewData && finite(currentPos.x) && finite(currentPos.y) && finite(currentPos.z))
	{
		//最初の座標を取得したら移動を開始する
		if (mLastPos.empty())
		{
			Debug::print(LOG_SUMMARY, "Starting navigation...");
			Time::showNowTime();//制御開始時刻をログに出力
			Debug::print(LOG_SUMMARY, "Control Start Point:(%f %f)\r\n", currentPos.x, currentPos.y);
			gMotorDrive.drivePIDGyro(0, 30/*MOTOR_MAX_POWER*/, true);
			//gPredictingState.setRunMode(true);
			distance_from_goal_to_start = VECTOR3::calcDistanceXY(currentPos, mGoalPos);
			mLastNaviMoveCheckTime = time;
			mLastArmServoMoveTime = time;
			mLastArmServoStopTime = time;
			mArmStopFlag = true;
			gArmServo.stop();
		}
		mLastPos.push_back(currentPos);
	}


	//ゴールとの距離を確認
	double distance = VECTOR3::calcDistanceXY(currentPos, mGoalPos);
	//double p = distance/distance_from_goal_to_start;
	if (distance < NAVIGATING_GOAL_DISTANCE_THRESHOLD)
	{
		//ゴール判定
		gMotorDrive.drive(0);
		Debug::print(LOG_SUMMARY, "Navigating Finished!\r\n");
		Debug::print(LOG_SUMMARY, "Navigating Finish Point:(%f %f)\r\n", currentPos.x, currentPos.y);
		nextState();
		return;
	}
	//数秒たっていなければ処理を返す
	//if (Time::dt(time, mLastNaviMoveCheckTime) < NAVIGATING_DIRECTION_UPDATE_INTERVAL)return;
	//mLastNaviMoveCheckTime = time;

       	//gCameraCapture.wadatisave();	

	//異常値排除
	if (removeError())
	{
		Debug::print(LOG_SUMMARY, "NAVIGATING: GPS Error value detected\r\n");
	}
	
	if (gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())//脱出モードが完了した時
	{
		//if (gEscapingByStabiState.isActive() && gEscapingByStabiState.getTryCount() < ESCAPING_BY_STABI_MIN_COUNT)//少なくとも何回かは芋虫動作をする
		{
			//スタック脱出処理続行
		//}
		//else
		//{
			//ローバーがひっくり返っている可能性があるため、しばらく前進する
			gMotorDrive.drivePIDGyro(0, MOTOR_MAX_POWER, true);
			gEscapingByStabiState.setRunMode(false);
			gEscapingRandomState.setRunMode(false);
			//gSServo.moveRun();  //スタビを通常の状態に戻す
			Debug::print(LOG_SUMMARY, "NAVIGATING: Navigating restart! \r\n");
			gEncoderMonitoringState.setRunMode(true);	//EncoderMoniteringを再開する
			gBuzzer.start(20, 10, 3);
		}//
	}
	else
	{
		//通常のナビゲーション
		if (mLastPos.size() < 2)return;//過去の座標が1つ以上(現在の座標をあわせて2つ以上)なければ処理を返す(進行方向決定不可能)
		navigationMove(distance);//過去の座標から進行方向を変更する
	}
	//}

	//座標データをひとつ残して削除
	currentPos = mLastPos.back();
	mLastPos.clear();
	mLastPos.push_back(currentPos);
}
bool Navigating::removeError()
{
	if (mLastPos.size() <= 2)return false;//最低2点は残す
	std::list<VECTOR3>::iterator it = mLastPos.begin();
	VECTOR3 average, sigma;
	while (it != mLastPos.end())
	{
		average += *it;
		++it;
	}
	average /= mLastPos.size();

	const static double THRESHOLD = 100 / DEGREE_2_METER;
	it = mLastPos.begin();
	while (it != mLastPos.end())
	{
		if (VECTOR3::calcDistanceXY(average, *it) > THRESHOLD)
		{
			mLastPos.erase(it);
			removeError();
			return true;
		}
		++it;
	}
	return false;
}
bool Navigating::isStuckByGPS() const
{
	//スタック判定
	VECTOR3 averagePos1, averagePos2;
	unsigned int i, border;
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	for (i = 0; i < mLastPos.size() / 2; ++i)
	{
		averagePos1 += *it;
		it++;
	}
	averagePos1 /= border = i;

	for (; i < mLastPos.size(); ++i)
	{
		averagePos2 += *it;
		it++;
	}
	averagePos2 /= i - border;

	return VECTOR3::calcDistanceXY(averagePos1, averagePos2) < NAVIGATING_STUCK_JUDGEMENT_THRESHOLD;//移動量が閾値以下ならスタックと判定
}
void Navigating::navigationMove(double distance) const
{
	//過去の座標の平均値を計算する
	VECTOR3 averagePos;
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	while (it != mLastPos.end())
	{
		averagePos += *it;
		++it;
	}
	averagePos -= mLastPos.back();
	averagePos /= mLastPos.size() - 1;

	//新しい角度を計算
	VECTOR3 currentPos = mLastPos.back();
	double currentDirection = -VECTOR3::calcAngleXY(averagePos, currentPos);
	double newDirection = -VECTOR3::calcAngleXY(currentPos, mGoalPos);
	double deltaDirection = GyroSensor::normalize(newDirection - currentDirection);
	deltaDirection = std::max(std::min(deltaDirection, NAVIGATING_MAX_DELTA_DIRECTION), -1 * NAVIGATING_MAX_DELTA_DIRECTION);

	//新しい速度を計算
	double speed = MOTOR_MAX_POWER;
	if (distance < NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD)
	{
		speed *= NAVIGATING_GOAL_APPROACH_POWER_RATE;	//接近したら速度を落とす
		gEncoderMonitoringState.setRunMode(false);		//エンコーダによるスタック判定をOFF
	}

	Debug::print(LOG_SUMMARY, "NAVIGATING: Last %d samples (%f %f) Current(%f %f)\r\n", mLastPos.size(), averagePos.x, averagePos.y, currentPos.x, currentPos.y);
	Debug::print(LOG_SUMMARY, "distance = %f (m)  delta angle = %f(%s)\r\n", distance * DEGREE_2_METER, deltaDirection, deltaDirection > 0 ? "LEFT" : "RIGHT");

	//方向と速度を変更
	gMotorDrive.drivePIDGyro(deltaDirection, speed, true);
}
bool Navigating::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 1)
	{
		if (mIsGoalPos)Debug::print(LOG_SUMMARY, "Current Goal (%f %f)\r\n", mGoalPos.x, mGoalPos.y);
		else Debug::print(LOG_SUMMARY, "NO Goal\r\n");
	}
	if (args.size() == 2)
	{
		if (args[1].compare("here") == 0)
		{
			VECTOR3 pos;
			if (!gGPSSensor.get(pos))
			{
				Debug::print(LOG_SUMMARY, "Unable to get current position!\r\n");
				return true;
			}

			setGoal(pos);
			return true;
		}
		else if (args[1].compare("goal") == 0)
		{
			nextState();
			return true;
		}
	}
	if (args.size() == 3)
	{
		VECTOR3 pos;
		pos.x = atof(args[1].c_str());
		pos.y = atof(args[2].c_str());

		setGoal(pos);
		return true;
	}
	Debug::print(LOG_PRINT, "navigating                 : get goal\r\n\
							navigating [pos x] [pos y] : set goal at specified position\r\n\
							navigating here            : set goal at current position\r\n\
							navigating goal            : call nextState\r\n");
	return true;
}

//次の状態に移行
void Navigating::nextState()
{
	gBuzzer.start(1000);

	//gColorAccessingState.setRunMode(true);
	//gModelingState.setRunMode(true);

	gMotorDrive.drive(0);//念のため2回
	gMotorDrive.drive(0);

	Time::showNowTime();
	Debug::print(LOG_SUMMARY, "Goal!\r\n");
}
void Navigating::setGoal(const VECTOR3& pos)
{
	mIsGoalPos = true;
	mGoalPos = pos;
	Debug::print(LOG_SUMMARY, "Set Goal ( %f %f )\r\n", mGoalPos.x, mGoalPos.y);
}
Navigating::Navigating() : mGoalPos(), mIsGoalPos(false), mLastPos()
{
	setName("navigating");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Navigating::~Navigating()
{
}

//////////////////////////////////////////////
// ColorAccessing
//////////////////////////////////////////////
/* ここから　2014年実装 */
/* ここまで　2014年実装 */

bool Modeling::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Start Modeling... ");
	Time::showNowTime();

	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);

	gDelayedExecutor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMikeServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	//gSServo.setRunMode(true);

	//gSServo.moveDetect();		//スタビをゴール検知の位置に移動
	return true;
}
*/
//void Modeling::onUpdate(const struct timespec& time)
//{
  //if (Time::dt(time, mLastUpdateTime) < 1) return;
  //mLastUpdateTime = time;
  //	IplImage* pImage = gCameraCapture.getFrame();
  //	gCameraCapture.save(NULL, pImage);

//	if (pImage == NULL)//カメラが死んでるのでその場でゴール判定する
	  //{
	  //		Debug::print(LOG_SUMMARY, "Detecting: Camera is not working...\r\n");
//}
//	nextState();
//	return;
//}
/*
bool Modeling::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("stop") == 0)
		{
			setRunMode(false);
			return true;
		}
		else if (args[1].compare("goal") == 0)
		{
			nextState();
			return true;
		}
	}
	Debug::print(LOG_PRINT, "modeling                 : following command\r\n\
							modeling stop  : stop modeling\r\n");
	return true;
}
//次の状態に移行
void Modeling::nextState()
{
	gBuzzer.start(1000);

	//次の状態を設定
	gTestingState.setRunMode(true);
	setRunMode(false);

	gMotorDrive.drive(0);//念のため2回
	gMotorDrive.drive(0);

	Debug::print(LOG_SUMMARY, "Modeling Finish! ");
	Time::showNowTime();
}
//前の状態に移行
void Modeling::prevState()
{
	gBuzzer.start(10, 5, 8);

	//前の状態に戻る
	gModelingState.setRunMode(false);
	gNavigatingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Navigating Restart!\r\n");
}
Modeling::Modeling()
{
	setName("modeling");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
}
Modeling::~Modeling()
{
}
*/
