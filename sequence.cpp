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
	gMultiServo.setRunMode(true);
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	//PgDelayedExecutor.setRunMode(true);

	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	//gWebCamera.setRunMode(true);
	//gDistanceSensor.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	//gCameraSave_Sequence.setRunMode(true);
  	gNineAxisSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);

	gSerialCommand.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
//  gLED.setRunMode(true);
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
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
	gMultiServo.start(BACK_STABI_FOLD_ANGLE);
	Debug::print(LOG_SUMMARY, "Disconnecting Wi-Fi...");
	system("sudo ruby /home/pi/network/disconnect.rb &");
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
	gMultiServo.setRunMode(true);
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
	//gMultiServo.start(BACK_STABI_FOLD_ANGLE);
	//gSServo.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Turning ON Wi-Fi...");
	system("sudo ruby -d /home/pi/network/build_network.rb &");

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
		gMultiServo.moveHold();
		//gSServo.moveFold();//スタビを格納状態で固定
		//gJohnServo.start(FRONT_STABI_FOLD_ANGLE); // 角度調節
		gMultiServo.start(BACK_STABI_FOLD_ANGLE);
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
	gMultiServo.setRunMode(true);
	//gJohnServo.setRunMode(true);
	//gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
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
		gMultiServo.moveHold();
		//gJohnServo.start(20); // 角度調節
		//gMultiServo.start(20);
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
			gMultiServo.moveRelease();
		}
		else
		{
			gMultiServo.moveHold();
		}

		++mServoCount;
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if (mServoCount >= SEPARATING_SERVO_COUNT)//サーボを規定回数動かした
		{
			//次状態に遷移
			gMultiServo.stop();
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
	//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	//gJohnServo.start(FRONT_STABI_RUN_ANGLE); // 角度調節
	gMultiServo.start(BACK_STABI_RUN_ANGLE);
	//gArmServo.start(ARM_RUN_ANGLE);
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
	//gNeckServo.start(NECK_RUN_ANGLE);
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
                //gArmServo.start(ARM_RUN_ANGLE);
                mArmMoveFlag = false;
                mArmStopFlag = true;
        }
        if(Time::dt(time, mLastArmServoMoveTime) > 0.5 && mArmStopFlag == true){
                //gArmServo.stop();
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
			//gArmServo.stop();
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
	//if (gPredictingState.isWorking(time))
	//{
	//	//轍回避中
	//}
	//else if (isStuckByGPS())//GPSスタック判定
	//{
	//	if ((!gEscapingRandomState.isActive()) && (!gLearningEscapeState.isActive()))
	//	{
	//		gEscapingByStabiState.setRunMode(true);
	//		//gEsc4State.setRunMode(true);
	//	}
	//	Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK detected by GPS at (%f %f)\r\n", currentPos.x, currentPos.y);
	//	gBuzzer.start(20, 10, 8);

	//	if (gEscapingByStabiState.isActive())		//EscapingByStabi中
	//	{
	//		if (gEscapingByStabiState.getTryCount() >= ESCAPING_BY_STABI_MAX_COUNT) {
	//			gEscapingByStabiState.setRunMode(false);
	//			Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping w/ GA Start! \r\n");
	//			gLearningEscapeState.setRunMode(true);
	//		}
	//	}
	//	else if (gLearningEscapeState.isActive() && gLearningEscapeState.getTryCount() >= LEARNING_ESCAPING_LIMIT) {
	//		//EscapingRandomに移行
	//		gLearningEscapeState.setRunMode(false);
	//		//gSServo.moveRun();  //スタビを通常の状態に戻す
	//		Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping Random Start! \r\n");
	//		gEscapingRandomState.setRunMode(true);
	//		mEscapingRandomStartTime = time;
	//	}
	//	else if (gEscapingRandomState.isActive())//EscapingRandom中
	//	{
	//		if (Time::dt(time, mEscapingRandomStartTime) > ESCAPING_RANDOM_TIME_THRESHOLD)
	//		{
	//			//EscapingByStabiに移行
	//			gEscapingRandomState.setRunMode(false);
	//			Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping ByStabi Start! \r\n");
	//			gEscapingByStabiState.setRunMode(true);
	//		}
	//	}
	//}
	//else
	//{
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
/*
bool ColorAccessing::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Start Goal Detecting... ");
	Time::showNowTime();

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gBuzzer.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	gArmServo.setRunMode(true);
	gNeckServo.setRunMode(true);
	//gSServo.setRunMode(true);
	gJohnServo.start(FRONT_STABI_RUN_ANGLE); // 角度調節
	gMultiServo.start(BACK_STABI_RUN_ANGLE);
	//gSServo.moveDetect();		//スタビをゴール検知の位置に移動

	mCurStep = STEP_STARTING;
	mStartTime = time;		//開始時刻を保存
	mLastUpdateTime = time;
	gCameraCapture.startWarming();
	mIsLastActionStraight = false;
	mTryCount = 0;
	mMotorPower = 40;
	mCurrentMotorPower = 40;
	actCount = 0;
	gThresholdHigh = 900;
	gThresholdLow = 200;
	gPastDeltaPulseL = 0;
	gPastDeltaPulseR = 0;
	gMotorDrive.getDeltaPulseL();
	gMotorDrive.getDeltaPulseR();
	gDeltaPulseL = 0;
	gDeltaPulseR = 0;
	mIsGPS = false;
	mCalcedStabiAngle = STABI_BASE_ANGLE;

	return true;
}
void ColorAccessing::onUpdate(const struct timespec& time)
{
	//gSServo.moveDetect();		//スタビをゴール検知の位置に移動

	if (gAvoidingState.isActive())return;

	// Debug::print(LOG_SUMMARY, "accel = %f\r\n",gAccelerationSensor.getAz());

	if (gAccelerationSensor.getAz() < -0.3 && !gWakingState.isActive() && mCurStep != STEP_GO_BACK)
	{
		Debug::print(LOG_SUMMARY, "accel = %f\r\n", gAccelerationSensor.getAz());
		mLastUpdateTime = time;
		// mCurStep = STEP_PRE_PARA_JUDGE;
		gPastDeltaPulseL = gDeltaPulseL;
		gPastDeltaPulseR = gDeltaPulseR;
		gWakingState.setRunMode(true);
	}

	// if(gWakingState.isActive())
	// {
	// 	mLastUpdateTime = time;//起き上がり動作中は待機する
	// 	return;
	// }
	double dt;

	switch (mCurStep)
	{
	case STEP_STARTING:
		if (!gWakingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Detecting: Checking started\r\n");
			//gMultiServo.start(STABI_BASE_ANGLE);		//スタビを走行時の位置に移動
			//setHorizontalStabiAngle();
			mCurStep = STEP_CHECKING;
			mLastUpdateTime = time;
			gCameraCapture.startWarming();
			mAngleOnBegin = gGyroSensor.getRvx();
		}
		break;
	case STEP_CHECKING:
		// 最後の行動が直進だったら，Gyroの補正を考慮してちょっと待ってから処理を開始する．
		if ((Time::dt(time, mLastUpdateTime) > mProcessFrequency && !mIsLastActionStraight) || (Time::dt(time, mLastUpdateTime) > mProcessFrequencyForGyro && mIsLastActionStraight))
		{
			Debug::print(LOG_SUMMARY, "Detecting: Approaching started\r\n");

			//新しい位置を取得できていれば座標を表示する
			if (gGPSSensor.get(mCurrentPos, false))
			{
				mIsGPS = true;	//一度でもGPS座標取得に成功したらtrueに
				Debug::print(LOG_SUMMARY, "Detecting: Current Position:(%f %f)\r\n", mCurrentPos.x, mCurrentPos.y);
			}

			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL, pImage);

			if (pImage == NULL)//カメラが死んでるのでその場でゴール判定する
			{
				Debug::print(LOG_SUMMARY, "Detecting: Camera is not working...\r\n");
				nextState();
				return;
			}

			double count = 0; //画像上の赤色の割合
			int x_pos = gImageProc.howColorGap(pImage, &count);

			if (x_pos != INT_MAX)	//色検知したら
			{
				mLastUpdateTime = time;
				if (x_pos == INT_MIN)	//ゴール判定時
				{
					//新しい位置を取得できていれば座標を表示する
					if (mIsGPS) Debug::print(LOG_SUMMARY, "Detecting: Control Finish at (%f %f)\r\n", mCurrentPos.x, mCurrentPos.y);//制御終了位置の座標を表示
					gMotorDrive.drive(0);
					nextState();
					return;
				}
				else if (count < mColorCount) // もし，コーンから遠かったら 
				{
					Debug::print(LOG_SUMMARY, "Detecting: FAR count:%f%%\r\n", count);
					if (x_pos < -mColorWidth)
					{
						Debug::print(LOG_SUMMARY, "Detecting: turn LEFT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(0, mMotorPower);
						mIsLastActionStraight = false;
					}
					else if (mColorWidth < x_pos)
					{
						Debug::print(LOG_SUMMARY, "Detecting: turn RIGHT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(mMotorPower, 0);
						mIsLastActionStraight = false;
					}
					else if (-mColorWidth <= x_pos && x_pos <= mColorWidth)
					{
						Debug::print(LOG_SUMMARY, "Detecting: go STRAIGHT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_VERYLONG;
						gMotorDrive.drivePIDGyro(0, mMotorPower, true);
						mIsLastActionStraight = true;
						mIsStraightTimeLong = true;
						mAngleOnBegin = gGyroSensor.getRz();
						actCount = 0;
					}
				}
				else if (count > mColorCount) // もし，コーンに近かったら 
				{
					Debug::print(LOG_SUMMARY, "Detecting: NEAR count:%f\r\n", count);
					if (x_pos < -mColorWidth + 20)
					{
						Debug::print(LOG_SUMMARY, "Detecting: turn LEFT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(0, mMotorPower);
						mIsLastActionStraight = false;
					}
					else if (mColorWidth - 20 < x_pos)
					{
						Debug::print(LOG_SUMMARY, "Detecting: turn RIGHT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(mMotorPower, 0);
						mIsLastActionStraight = false;
					}
					else if (-mColorWidth + 20 <= x_pos && x_pos <= mColorWidth - 20)
					{
						Debug::print(LOG_SUMMARY, "Detecting: go STRAIGHT <- pos= %d\r\n", x_pos);
						mCurStep = STEP_STOPPING_LONG;
						gMotorDrive.drivePIDGyro(0, mMotorPower, true);
						mIsLastActionStraight = true;
						mAngleOnBegin = gGyroSensor.getRz();
						actCount = 0;
						mIsStraightTimeLong = false;
					}
				}
				mTryCount = 0;
			}
			else//色検知しなかったら
			{
				Debug::print(LOG_SUMMARY, "MotorPower(false) = %f\r\n", mMotorPower);
				if (mIsLastActionStraight)	//前回の行動が直進なら．
				{
					double diff = GyroSensor::normalize(gGyroSensor.getRz() - mAngleOnBegin);

					Debug::print(LOG_SUMMARY, "Detecting: Gyro diff = %f\r\n", diff);

					if (diff < 0)
					{
						//右に向いた時の行動
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(0, mMotorPower);
					}
					else
					{
						//左に向いた時の行動
						mCurStep = STEP_STOPPING_FAST;
						gMotorDrive.drive(mMotorPower, 0);
					}
				}
				else //前回の行動が直進以外なら
				{
					double diff = GyroSensor::normalize(gGyroSensor.getRz() - mAngleOnBegin);

					mCurStep = STEP_TURNING;

					if (mTryCount > 2)
					{
						//とりあえず右に曲がるか．
						mCurStep = STEP_TURNING;
						gMotorDrive.drive(mMotorPower - 10, -mMotorPower + 10);
					}
					else if (diff < 0)
					{
						//右を向いていた時の処理．
						mCurStep = STEP_TURNING;
						gMotorDrive.drive(-mMotorPower + 10, mMotorPower - 10);
						mTryCount++;
					}
					else if (diff >= 0)
					{
						//左を向いていた時の処理．
						mCurStep = STEP_TURNING;
						gMotorDrive.drive(mMotorPower - 10, -mMotorPower + 10);
						mTryCount++;
					}
				}

				mIsLastActionStraight = false;
			}
			mLastUpdateTime = time;
			actCount++;
			//gCameraCapture.startWarming();
		}
		break;
	case STEP_TURNING:
		if (Time::dt(time, mLastUpdateTime) > 0.5)
		{
			gMotorDrive.drive(0);
			mCurStep = STEP_STARTING;
			setMotorPower(100);
		}
		break;
	case STEP_STOPPING_FAST:
		if (Time::dt(time, mLastUpdateTime) > 0.5)
		{
			gMotorDrive.drive(0);
			mCurStep = STEP_STARTING;
			setMotorPower(-100);
		}
		break;
	case STEP_STOPPING_LONG:
		//gMultiServo.start(STABI_WAKING_ANGLE);		//スタビを上げる
		if (Time::dt(time, mLastUpdateTime) > mStraightTime)
		{
			mCurStep = STEP_DEACCELERATE;
		}
		break;
	case STEP_STOPPING_VERYLONG:
		//gMultiServo.start(STABI_WAKING_ANGLE);		//スタビを上げる
		if (Time::dt(time, mLastUpdateTime) > mStraightTimeFromFar)
		{
			mCurStep = STEP_DEACCELERATE;
		}
		break;
	case STEP_DEACCELERATE:	//ゆっくり減速する
		dt = Time::dt(time, mLastUpdateTime);
		if (dt > mDeaccelerateDuration)
		{
			mLastUpdateTime = time;
			mCurStep = STEP_WAIT_FIRST;

			gMotorDrive.drive(0);
			setMotorPower(0);
		}
		else
		{
			int tmp_power = std::max((int)((1 - dt / mDeaccelerateDuration) * (20 / 2)), 0);	//ToDo: 20を変数に置き換える
			gMotorDrive.drive(tmp_power);
		}
		break;
	case STEP_WAIT_FIRST:
		if (Time::dt(time, mLastUpdateTime) > (mWaitTime / 2))
		{
			//gMultiServo.start((STABI_BASE_ANGLE + STABI_WAKING_ANGLE) / 2);	//中間の角度
			mCurStep = STEP_WAIT_SECOND;
		}
		break;
	case STEP_WAIT_SECOND:
		if (Time::dt(time, mLastUpdateTime) > mWaitTime)
		{
			mLastUpdateTime = time;
			mCurStep = STEP_STARTING;
		}
		break;
	case STEP_GO_BACK:	//バックする
		if (Time::dt(time, mLastUpdateTime) > 3)
		{
			gBuzzer.start(100);
			Debug::print(LOG_SUMMARY, "Detecting: CHANGE_OF_DIRECTION start!\r\n");
			mCurStep = STEP_CHANGE_OF_DIRECTION;
			gMotorDrive.drive(0);
			gTurningState.setRunMode(true);
		}
		break;
	case STEP_CHANGE_OF_DIRECTION:	//方向転換する
		if (!gTurningState.isActive() && !gWakingState.isActive())
		{
			gBuzzer.start(100);
			Debug::print(LOG_SUMMARY, "Detecting: TURNING Finished!\r\n");
			gMotorDrive.drive(100);
			mLastUpdateTime = time;
			mCurStep = STEP_LEAVING;
		}
		break;
	case STEP_LEAVING:	//しばらく直進し、ゴールから一時的に離れる
		if (Time::dt(time, mLastUpdateTime) > 10)//しばらく直進する
		{
			prevState();
		}
		break;
	}

	//ColorAccessingを開始してからの経過時間を確認
	if (mCurStep != STEP_GO_BACK && mCurStep != STEP_CHANGE_OF_DIRECTION && mCurStep != STEP_LEAVING)

	{
		bool retry_flag = timeCheck(time);
		if (!retry_flag)
		{
			Debug::print(LOG_SUMMARY, "Detecting failed ... try count: %d\r\n", mDetectingRetryCount - 1);
			nextState();//一定回数以上ナビ復帰を繰り返したのでその場でゴール判定
			return;
		}
	}
}

//void ColorAccessing::setHorizontalStabiAngle()
//{
//	gMultiServo.start(mCalcedStabiAngle);
//
//	double az = gAccelerationSensor.getAz();
//	double ay = gAccelerationSensor.getAy();
//
//	//while ( az < 0.9 )
//	{
//		if (az < 0)
//		{
//			// 何もしない
//		}
//		else if (((az < 0.88 || 0.94 < az) && ay < 0) || ay < -0.5)
//		{
//			mCalcedStabiAngle += 0.02;
//
//			if (mCalcedStabiAngle > 1)
//				mCalcedStabiAngle = 1.0;
//		}
//		else if ((az < 0.88 || 0.98 < az) && ay >= 0)
//		{
//			mCalcedStabiAngle -= 0.02;
//			if (mCalcedStabiAngle < 0)
//				mCalcedStabiAngle = 0.0;
//		}
//		gMultiServo.start(mCalcedStabiAngle);
//	}
//
//}

//モータの出力設定

void ColorAccessing::setMotorPower(int mode)
{
	gDeltaPulseL = abs(gMotorDrive.getDeltaPulseL());
	gDeltaPulseR = abs(gMotorDrive.getDeltaPulseR());
		//if(gPastDeltaPulseL > 0)
		//{
		//	gDeltaPulseL = gPastDeltaPulseL - 800;
		//	gPastDeltaPulseL = 0;
		//	Debug::print(LOG_SUMMARY, "use past delta pulse left!\r\n");
		//}
		//if (gPastDeltaPulseR > 0)
		//{
		//	gDeltaPulseR = gPastDeltaPulseR - 800;
		//	gPastDeltaPulseR = 0;
		//	Debug::print(LOG_SUMMARY, "use past delta pulse right!\r\n");
		//}
	Debug::print(LOG_SUMMARY, "deltapulseL: %llu,  deltapulseR: %llu\r\n", gDeltaPulseL, gDeltaPulseR);

	if (mode == 0)
	{
		if (mIsStraightTimeLong)
		{
			gThresholdHigh = gStraightThresholdHigh*mStraightTimeFromFar / 0.8;
			gThresholdLow = gStraightThresholdLow*mStraightTimeFromFar / 0.8;
		}
		else
		{
			gThresholdHigh = gStraightThresholdHigh;
			gThresholdLow = gStraightThresholdLow;
		}
	}
	else
	{
		if (mode > 50)
		{
			gThresholdHigh = gRotationThresholdHigh;
			gThresholdLow = gRotationThresholdLow;
		}
		else if (mode < 50)
		{
			gThresholdHigh = gCurveThresholdHigh;
			gThresholdLow = gCurveThresholdLow;
			if (gDeltaPulseL < gDeltaPulseR) gDeltaPulseL = gDeltaPulseR;
			else gDeltaPulseR = gDeltaPulseL;
		}
	}
	Debug::print(LOG_SUMMARY, "mode is %d\r\n", mode);
	Debug::print(LOG_SUMMARY, "gThresholdHigh : %llu,  gThresholdLow : %llu\r\n", gThresholdHigh, gThresholdLow);

	if (gDeltaPulseL > gThresholdHigh || gDeltaPulseR > gThresholdHigh) mMotorPower -= 5;
	else if (gDeltaPulseL < gThresholdLow || gDeltaPulseR < gThresholdLow) mMotorPower += 5;
	if (mMotorPower < 0) mMotorPower = 0;
	if (mMotorPower > 100) mMotorPower = 100;
	mCurrentMotorPower = mMotorPower;
	Debug::print(LOG_SUMMARY, "motorpower : %d\r\n", mMotorPower);
}
bool ColorAccessing::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("setmode") == 0)
		{
			if (mIsDetectingExecute) Debug::print(LOG_SUMMARY, "Detecting mode: ON\r\n");
			else Debug::print(LOG_SUMMARY, "Detecting mode: OFF\r\n");
			return true;
		}
		else if (args[1].compare("reset") == 0)
		{
			Debug::print(LOG_PRINT, "Command Executed!\r\n");
			mDetectingRetryCount = 0;
			return true;
		}
	}
	else if (args.size() == 3)
	{
		if (args[1].compare("setmode") == 0)
		{
			if (args[2].compare("ON") == 0)
			{
				setIsDetectingExecute(true);
				return true;
			}
			else if (args[2].compare("OFF") == 0)
			{
				setIsDetectingExecute(false);
				return true;
			}
		}
		else if (args[1].compare("straighttime") == 0)
		{
			mStraightTime = atof(args[2].c_str());
			return true;
		}
		else if (args[1].compare("straighttimef") == 0)
		{
			mStraightTimeFromFar = atof(args[2].c_str());
			return true;
		}
		else if (args[1].compare("colorwidth") == 0)
		{
			mColorWidth = atoi(args[2].c_str());
			return true;
		}
		else if (args[1].compare("pf") == 0)
		{
			mProcessFrequency = atof(args[2].c_str());
			return true;
		}
		else if (args[1].compare("pfg") == 0)
		{
			mProcessFrequencyForGyro = atof(args[2].c_str());
			return true;
		}
	}
	else if (args.size() == 4)
	{
		if (args[1].compare("set") == 0)
		{
			if (args[2].compare("d_time") == 0)//mDeaccelerateDuration
			{
				mDeaccelerateDuration = atof(args[3].c_str());
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
			else if (args[2].compare("waittime") == 0)//mWaitTime
			{
				mWaitTime = atof(args[3].c_str());
				Debug::print(LOG_SUMMARY, "Command executed!\r\n");
				return true;
			}
		}
	}
	else if (args.size() == 5)
	{
		if (args[1].compare("threshold") == 0)
		{
			if (args[2].compare("straight") == 0)
			{
				if (args[3].compare("high") == 0)
				{
					gStraightThresholdHigh = atof(args[4].c_str());
				}
				if (args[3].compare("low") == 0)
				{
					gStraightThresholdLow = atof(args[4].c_str());
				}
			}
			if (args[2].compare("rotation") == 0)
			{
				if (args[3].compare("high") == 0)
				{
					gRotationThresholdHigh = atof(args[4].c_str());
				}
				if (args[3].compare("low") == 0)
				{
					gRotationThresholdLow = atof(args[4].c_str());
				}
			}
			if (args[2].compare("curve") == 0)
			{
				if (args[3].compare("high") == 0)
				{
					gCurveThresholdHigh = atof(args[4].c_str());
				}
				if (args[3].compare("low") == 0)
				{
					gCurveThresholdLow = atof(args[4].c_str());
				}
			}
		}
		return true;
	}
	Debug::print(LOG_SUMMARY, "predicting [enable/disable]  : switch avoiding mode\r\n");
	Debug::print(LOG_SUMMARY, "pf/pfg [value]  : process frequency / process frequency for gyro\r\n");
	Debug::print(LOG_SUMMARY, "threshold straight : %llu %llu\r\n", gStraightThresholdLow, gStraightThresholdHigh);
	Debug::print(LOG_SUMMARY, "threshold rotation : %llu %llu\r\n", gRotationThresholdLow, gRotationThresholdHigh);
	Debug::print(LOG_SUMMARY, "threshold curve    : %llu %llu\r\n", gCurveThresholdLow, gCurveThresholdHigh);
	Debug::print(LOG_SUMMARY, "color width is %d\r\n", mColorWidth);
	Debug::print(LOG_SUMMARY, "process frequency is %f\r\n", mProcessFrequency);
	Debug::print(LOG_SUMMARY, "process frequency for gyro is %f\r\n", mProcessFrequencyForGyro);

	Debug::print(LOG_PRINT, "detecting set d_time [time]   : set mDeaccelerateDuration\r\n\
							detecting set waittime [time] : set waittime\r\n\
							detecting reset               : reset detecting retry count\r\n\
							detecting setmode [ON/OFF]    : set detecting mode\r\n\
							detecting setmode             : show detecting mode state\r\n");
	return true;
}
//次の状態に移行
void ColorAccessing::nextState()
{
	gBuzzer.start(1000);

	//次の状態を設定
	gTestingState.setRunMode(true);
	setRunMode(false);

	gMotorDrive.drive(0);//念のため2回
	gMotorDrive.drive(0);

	Debug::print(LOG_SUMMARY, "Detecting Finish! ");
	Time::showNowTime();
	Debug::print(LOG_SUMMARY, "Goal!\r\n");
}
//前の状態に移行
void ColorAccessing::prevState()
{
	gBuzzer.start(10, 5, 8);

	//前の状態に戻る
	gColorAccessingState.setRunMode(false);
	gNavigatingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Navigating Restart!\r\n");
}
bool ColorAccessing::timeCheck(const struct timespec& time)
{
	if (Time::dt(time, mStartTime) > COLOR_ACCESSING_ABORT_TIME)//一定時間が経過したらNavigatingからやり直し
	{
		mDetectingRetryCount++;
		Debug::print(LOG_SUMMARY, "ColorAccessing Timeout! try count: %d\r\n", mDetectingRetryCount);

		if (mDetectingRetryCount >= COLOR_ACCESSING_MAX_RETRY_COUNT)//一定回数以上ナビ復帰を繰り返した場合はfalse
		{
			return false;
		}

		Debug::print(LOG_SUMMARY, "Detecting: GO_BACK start!\r\n");
		mCurStep = STEP_GO_BACK;
		gMotorDrive.drive(-100);
		mLastUpdateTime = time;
		gBuzzer.start(10, 5, 8);
	}
	return true;
}
void ColorAccessing::setIsDetectingExecute(bool flag)
{
	if (flag)
	{
		if (mIsDetectingExecute)
		{
			Debug::print(LOG_SUMMARY, "Detecting has already set \"ON\"\r\n");
			return;
		}
		mIsDetectingExecute = flag;
		Debug::print(LOG_SUMMARY, "Detecting has set \"ON\"\r\n");
	}
	else
	{
		if (!mIsDetectingExecute)
		{
			Debug::print(LOG_SUMMARY, "Detecting has already set \"OFF\"\r\n");
			return;
		}
		mIsDetectingExecute = flag;
		Debug::print(LOG_SUMMARY, "Detecting has set \"OFF\"\r\n");
		Debug::print(LOG_SUMMARY, "\n****** WARNING ******\r\n\
								  This mode does \"not\" execute Detecting!\r\n\
								  ****** WARNING ******\r\n\n");
	}
}
bool ColorAccessing::getIsDetectingExecute()
{
	return mIsDetectingExecute;
}
ColorAccessing::ColorAccessing() :mIsDetectingExecute(true), mDetectingRetryCount(0), mDeaccelerateDuration(0.5), mWaitTime(0.5)
{
	setName("detecting");
	setPriority(TASK_PRIORITY_SEQUENCE, TASK_INTERVAL_SEQUENCE);
	gStraightThresholdHigh = 2000;
	gStraightThresholdLow = 1000;
	gRotationThresholdHigh = 500;
	gRotationThresholdLow = 200;
	gCurveThresholdHigh = 300;
	gCurveThresholdLow = 200;
	mStraightTime = 0.8;
	mStraightTimeFromFar = 2.3;
	mColorWidth = 100;
	mColorCount = 0.04;
	mProcessFrequency = 1.0;
	mProcessFrequencyForGyro = 2.0;
	mTryCount = 3;
	mIsStraightTimeLong = false;
}
ColorAccessing::~ColorAccessing()
{
}
 ここまで　2014年実装 */
/*
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
	gMultiServo.setRunMode(true);
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
