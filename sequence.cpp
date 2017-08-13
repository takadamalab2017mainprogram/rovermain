#include <stdlib.h>
#include <math.h>
#include <fstream>
#include <functional>
//#include <opencv2/opencv.hpp>
//#include <opencv/cvaux.h>
//#include <opencv/highgui.h>
#include <stdarg.h>
#include <wiringPi.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include "sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"
//#include "chat.h"
//#include "image_proc.h"
#include "subsidiary_sequence.h"
#include "delayed_execution.h"
#include "constants.h"

#include <string.h>
#include <unistd.h>
#include <stdio.h>
#include <iterator>
#include <iostream>

Testing gTestingState;
Waiting gWaitingState;
Falling gFallingState;
Separating gSeparatingState;
Navigating gNavigatingState;
//Modeling gModelingState;
//ColorAccessing gColorAccessingState;
//extern Filename gCaptureFilename

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
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	//マルチーズ追加
	//gServer.setRunMode(true);
	//gClient.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	//gGyroSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	//gWebCamera.setRunMode(true);
	//gDistanceSensor.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	//gCameraSave_Sequence.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
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

			if (gNineAxisSensor.isActive())
			{
				gNineAxisSensor.getRPos(vec);
				Debug::print(LOG_SUMMARY, " Gyro pos (%f %f %f) d\r\n", vec.x, vec.y, vec.z);
				gNineAxisSensor.getRVel(vec);
				Debug::print(LOG_SUMMARY, " Gyro vel (%f %f %f) dps\r\n", vec.x, vec.y, vec.z);
			}
			else Debug::print(LOG_SUMMARY, " Gyro is NOT working\r\n");

			if (gNineAxisSensor.isActive())
			{
				gNineAxisSensor.getAccel(vec);
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
	gMultiServo.setRunMode(true);
	gMultiServo.fold();//スタビたたんでいる状態
	gNineAxisSensor.setRunMode(true);
	//  gNineAxisSensor.isMonitoring = true;
	Debug::print(LOG_SUMMARY, "Disconnecting Wi-Fi...\r\n");
	system("sudo ruby /home/pi/network/disconnect.rb &");
	return true;
}
void Waiting::nextState()
{
	gBuzzer.start(100);
	Debug::print(LOG_SUMMARY, "Turning ON Wi-Fi...\r\n");
	system("sudo ruby -d /home/pi/network/build_network.rb &");
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
	//gGyroSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gJohnServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	//gJohnServo.start(FRONT_STABI_FOLD_ANGLE);
	//gMultiServo.start(BACK_STABI_FOLD_ANGLE);
	//gSServo.setRunMode(true);

	gNineAxisSensor.setRunMode(true);
	gNineAxisSensor.isMonitoring = false;
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
		gMultiServo.fold();//たたむ
		//gNeckServo.start(0.5);
	}

	//閾値以下ならカウント
	//Debug::print(LOG_PRINT,"vx:%f vy:%f vz%f",gNineAxisSensor.getRvx(),gNineAxisSensor.getRvy(),gNineAxisSensor.getRvz());
	if (abs(gNineAxisSensor.getRvx()) < FALLING_GYRO_THRESHOLD && abs(gNineAxisSensor.getRvy()) < FALLING_GYRO_THRESHOLD && abs(gNineAxisSensor.getRvz()) < FALLING_GYRO_THRESHOLD)
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
	/*
	long long newMotorPulseL = gMotorDrive.getL(), newMotorPulseR = gMotorDrive.getR();
	if (abs(newMotorPulseL - mLastMotorPulseL) > FALLING_MOTOR_PULSE_THRESHOLD && abs(newMotorPulseR - mLastMotorPulseR) > FALLING_MOTOR_PULSE_THRESHOLD)
	{
		if (mContinuousMotorPulseCount < FALLING_MOTOR_PULSE_COUNT)++mContinuousMotorPulseCount;
	}
	else mContinuousMotorPulseCount = 0;
	*/

	//判定状態を表示
	Debug::print(LOG_SUMMARY, "Pressure Count   %d / %d (%d hPa)\r\n", mContinuousPressureCount, FALLING_PRESSURE_COUNT, newPressure);
	Debug::print(LOG_SUMMARY, "Gyro Count       %d / %d\r\n", mCoutinuousGyroCount, FALLING_GYRO_COUNT);
	//Debug::print(LOG_SUMMARY, "MotorPulse Count %d / %d (%lld,%lld)\r\n", mContinuousMotorPulseCount, FALLING_MOTOR_PULSE_COUNT, newMotorPulseL - mLastMotorPulseL, newMotorPulseR - mLastMotorPulseR);

	//mLastMotorPulseL = newMotorPulseL;
	//mLastMotorPulseR = newMotorPulseR;

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
	//gGyroSensor.setRunMode(true);
	//gAccelerationSensor.setRunMode(true);
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
			mCurStep = STEP_GO_FORWARD;
			gWakingState.setRunMode(true);
		}
		break;
	case STEP_GO_FORWARD:	//パラ検知後，しばらく直進する
		if (Time::dt(time, mLastUpdateTime) > 3)
		{
			gMotorDrive.drive(0);
			nextState();
		}
		else
		{
			gMotorDrive.drive(100);

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
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	//gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	//	gEncoderMonitoringState.setRunMode(true);
		//gJohnServo.setRunMode(true);
	gMultiServo.setRunMode(true);
	//gArmServo.setRunMode(true);
	//gNeckServo.setRunMode(true);
	//gJohnServo.start(FRONT_STABI_RUN_ANGLE); // 角度調節
	gMultiServo.Running();//走っているときの角度に設定
	//gArmServo.start(ARM_RUN_ANGLE);
	//gNeckServo.start(1);
	//gSServo.setRunMode(true);
	//gSServo.moveRun();		//スタビを走行時の位置に移動
	gNineAxisSensor.setRunMode(true);
	mLastNaviMoveCheckTime = time;
	mLastArmServoMoveTime = time;
	mLastArmServoStopTime = time;
	mLastUpdateTime = time;
	mArmStopFlag = true;
	mGoalFlag = false;
	mLastPos.clear();
	getGoalList(GoalList);

	//GoalListの最初にイテレータを置く
	//std::list<VECTOR3>::iterator itr;
	itr = GoalList.begin();
	//最初の座標をゴールにする
	mGoalPos = *itr;
	Debug::print(LOG_SUMMARY, "init goal is setted at ( %lf,%lf ) \r\n", mGoalPos.x, mGoalPos.y);
	mIsGoalPos = true;

	return true;
}
void Navigating::onUpdate(const struct timespec& time)
{
	VECTOR3 currentPos;

	//５秒置きに、GoalList を読み込む
	if (Time::dt(time, mLastUpdateTime) > 5.0) {
		//ファイルから　GoalList を読み込む、GoalList に保存する
		getGoalList(GoalList);
		//GoalListの最初にイテレータを置く
		//std::list<VECTOR3>::iterator itr;
		itr = GoalList.begin();
		//最初の座標をゴールにする
		VECTOR3 firstGoal = *itr;

		if (firstGoal.z == -1) {
			//見つかりません、とりあえず停止
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "(%f)can not find the object, stop rover\r\n",firstGoal.z);
		}
		else if (firstGoal.z == -2) {

			//Goal 判定した、終わり
			nextState();
			Debug::print(LOG_SUMMARY, "(%f)Find the object, mission finished\r\n",firstGoal.z);
			return;
		}
		else if (firstGoal.z == -3) {

			//ルート計算中、待機する
			gMotorDrive.drive(0);
			Debug::print(LOG_SUMMARY, "(%f)Calculating the route, waiting... \r\n", firstGoal.z);
			
		}
		else {
			Debug::print(LOG_SUMMARY, "(%f) goal is setted at ( %f,%f ) \r\n", firstGoal.z,firstGoal.x, firstGoal.y);
			mGoalPos = *itr;
			mIsGoalPos = true;
		}
		

		if (mGoalFlag)//途中のゴールに到達した
		{
			//ゴール判定
			gMotorDrive.drive(0);

			Debug::print(LOG_SUMMARY, "Navigating Finish Point:(%f %f)\r\n", currentPos.x, currentPos.y);
			//なんかの処理で、途中のゴールに到達したを通知する
			
			mGoalFlag=false;
			//ファイナルゴールなら終わり
			//if (GoalList.size() == 0) //リスト最後に辿り付いた、ファイナルゴールに到達、終了する
			//{
			//	nextState();
			//	Debug::print(LOG_SUMMARY, "Final Goal Navigating Finished!\r\n");
			//	return;
			//}
			//else
			//{	//GoalListのイテレータを増やす、つぎのゴースを設定
			//	++itr;

			//	//passedGoal のリストに保存する、passedGoal.txtファイルの中身に追加
			//	writePassedGoal(PassedGoal, mGoalPos);

			//	//探索済みのものを消す、ファイルの中身も消す
			//	deleteGoalList(GoalList);

			//	mGoalFlag = false;

			//}
		}

		mLastUpdateTime = time;
	}

	//ゴールが設定されているか確認
	if (!mIsGoalPos)
	{
		//ゴールが設定されていないため移動できない
		Debug::print(LOG_SUMMARY, "NAVIGATING : Please set goal!\r\n");
		gMotorDrive.drive(0);
		nextState();
		return;
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
		}
		mLastPos.push_back(currentPos);
	}


	//ゴールとの距離を確認
	double distance = VECTOR3::calcDistanceXY(currentPos, mGoalPos);
	//double p = distance/distance_from_goal_to_start;

	//途中のゴールに到達しているかのフラグ
	if (distance < NAVIGATING_GOAL_DISTANCE_THRESHOLD) {
		mGoalFlag = true;
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
			//			gEncoderMonitoringState.setRunMode(true);	//EncoderMoniteringを再開する
			gBuzzer.start(20, 10, 3);
		}//
	}
	else
	{
		//通常のナビゲーション
		if (mLastPos.size() < 10)return;//過去の座標が1つ以上(現在の座標をあわせて2つ以上)なければ処理を返す(進行方向決定不可能)
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
	double deltaDirection = NineAxisSensor::normalize(newDirection - currentDirection);
	deltaDirection = std::max(std::min(deltaDirection, NAVIGATING_MAX_DELTA_DIRECTION), -1 * NAVIGATING_MAX_DELTA_DIRECTION);

	//新しい速度を計算
	double speed = MOTOR_MAX_POWER;
	if (distance < NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD)
	{
		speed *= NAVIGATING_GOAL_APPROACH_POWER_RATE;	//接近したら速度を落とす
//		gEncoderMonitoringState.setRunMode(false);		//エンコーダによるスタック判定をOFF
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
	Debug::print(LOG_SUMMARY, "in nextState finished\r\n");
	gTestingState.setRunMode(true);
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
void Navigating::getGoalList(std::list<VECTOR3>& GoalList) {

	//goallist 読み込む

	//ファイルの読み込み
	std::ifstream ifs("GoalList.txt");
	if (!ifs) {
		Debug::print(LOG_SUMMARY, "can not find GoalList.txt \r\n");
	}

	//txtファイルを1行ずつ読み込む
	std::string str;


	while (getline(ifs, str)) {
		//一行ずつ読み込む、VECTOR３に代入、GoalListに保存する
		VECTOR3 temp = VECTOR3();
		//Debug::print(LOG_SUMMARY, "%s\r\n",str.c_str());
		double a, b, c;
		sscanf(str.c_str(), "%lf, %lf, %lf", &a, &b, &c);

		temp.x = a;
		temp.y = b;
		temp.z = c;

		GoalList.push_back(temp);
		Debug::print(LOG_SUMMARY, "Get Goal from Goalist ( %.8lf %.8lf )\r\n", temp.x, temp.y);

	}

}
//PassedGoal.txt に通過したゴールを書き込む
//void Navigating::writePassedGoal(std::list<VECTOR3>& PassedGoal, VECTOR3& mGoalPos) {
//	PassedGoal.push_back(mGoalPos);
//
//	//一旦PassedGoal.txt を削除し
//	if (remove("PassedGoal.txt") == 0) {
//		//削除成功した
//	}
//	else {
//		std::cout << "failled delete PassedGoal.txt" << std::endl;
//	}
//
//	std::string filename = "PassedGoal.txt";
//	std::ofstream write_file;
//	write_file.open(filename, std::ios::out);
//
//	for (auto itr = PassedGoal.begin(); itr != PassedGoal.end(); ++itr) {
//		write_file << itr->x << "," << itr->y << "," << itr->z << std::endl;
//		Debug::print(LOG_SUMMARY, "PassedGoal ( %f %f )\r\n", itr->x, itr->y);
//	}
//}

//GoalList（削除済み）をGoalList.txtに書き込む
//void Navigating::deleteGoalList(std::list<VECTOR3>& GoalList) {
//	//GoalList から通過したゴールを削除
//	GoalList.pop_front();
//
//	//一旦GoalList.txt を削除し
//	if (remove("GoalList.txt") == 0) {
//		//削除成功した
//	}
//	else {
//		std::cout << "failled delete GoalList.txt" << std::endl;
//	}
//
//	std::string filename = "GoalList.txt";
//	std::ofstream write_file;
//	write_file.open(filename, std::ios::out);
//
//	for (auto itr = GoalList.begin(); itr != GoalList.end(); ++itr) {
//		write_file << itr->x << "," << itr->y << "," << itr->z << std::endl;
//	}
//}
Navigating::~Navigating()
{
}

