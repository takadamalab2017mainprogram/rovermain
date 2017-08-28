#include <stdlib.h>
#include <math.h>
#include <list>
#include <fstream>
#include <functional>
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
#include "subsidiary_sequence.h"
#include "delayed_execution.h"
#include "constants.cpp"
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
Waking gWakingState;

//////////////////////////////////////////////
// Testing
//////////////////////////////////////////////

bool Testing::onInit(const struct timespec& time)
{
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gMultiServo.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gLED.setRunMode(true);
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
			Debug::print(LOG_SUMMARY, "Version: %d\r\n", Constants::VERSION);
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
	gSerialCommand.setRunMode(true);//Xbeeをスリープモードにするならコメントアウトすること
	gBuzzer.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gDelayedExecutor.setRunMode(true);
	gMultiServo.setRunMode(true);
	gMultiServo.fold();//スタビたたんでいる状態
	gLED.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gLED.hsv(0.03);
  Debug::print(LOG_SUMMARY, "Disconnecting Wi-Fi...\r\n");
  //system("sudo ruby /home/pi/network/disconnect.rb &");
	return true;
}
void Waiting::nextState()
{
	gBuzzer.start(100);
  Debug::print(LOG_SUMMARY, "Turning ON Wi-Fi...\r\n");
	//スリープを解除
  //system("sudo ruby -d /home/pi/network/build_network.rb &");

	//次の状態を設定
	gFallingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Waiting Finished!\r\n");
}
void Waiting::onUpdate(const struct timespec& time)
{

	//明るい場合カウント
	if (gLightSensor.get())
	{
		++mContinuousLightCount;
		gBuzzer.start(10);
	}
	else mContinuousLightCount = 0;

	if (mContinuousLightCount >= Constants::WAITING_LIGHT_COUNT)//明るい場合放出判定
	{
		nextState();
		return;
	}

	if (Time::dt(time, mStartTime) > Constants::WAITING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
		nextState();
		return;
	}
}
Waiting::Waiting()
{
	setName("waiting");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
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
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gMultiServo.setRunMode(true);
	gLED.setRunMode(true);
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
		gMultiServo.fold();//たたむ
	}

	//閾値以下ならカウント
	if (abs(gNineAxisSensor.getRvx()) < Constants::FALLING_GYRO_THRESHOLD && abs(gNineAxisSensor.getRvy()) < Constants::FALLING_GYRO_THRESHOLD && abs(gNineAxisSensor.getRvz()) < Constants::FALLING_GYRO_THRESHOLD)
	{
		if (mCoutinuousGyroCount < Constants::FALLING_GYRO_COUNT)++mCoutinuousGyroCount;
	}
	else mCoutinuousGyroCount = 0;

	//1秒ごとに以下の処理を行う
	if (Time::dt(time, mLastCheckTime) < 1)return;
	mLastCheckTime = time;

	//気圧の差が一定以下ならカウント
	int newPressure = gPressureSensor.get();
	if (abs((int)(newPressure - mLastPressure)) < Constants::FALLING_DELTA_PRESSURE_THRESHOLD)
	{
		if (mContinuousPressureCount < Constants::FALLING_PRESSURE_COUNT)++mContinuousPressureCount;
	}
	else mContinuousPressureCount = 0;
	mLastPressure = newPressure;


	//判定状態を表示
	Debug::print(LOG_SUMMARY, "Pressure Count   %d / %d (%d hPa)\r\n", mContinuousPressureCount, Constants::FALLING_PRESSURE_COUNT, newPressure);
	Debug::print(LOG_SUMMARY, "Gyro Count       %d / %d\r\n", mCoutinuousGyroCount, Constants::FALLING_GYRO_COUNT);

	//GPS情報ログ
	VECTOR3 pos;
	if (gGPSSensor.get(pos))Debug::print(LOG_SUMMARY, "GPS Position     (%f %f %f)\r\n", pos.x, pos.y, pos.z);
	else Debug::print(LOG_SUMMARY, "GPS Position     Unable to get\r\n");

	//カウント回数が一定以上なら次の状態に移行
	if (mContinuousPressureCount >= Constants::FALLING_PRESSURE_COUNT && (mCoutinuousGyroCount >= Constants::FALLING_GYRO_COUNT || mContinuousMotorPulseCount >= Constants::FALLING_MOTOR_PULSE_COUNT))
	{
		nextState();
		return;
	}

	if (Time::dt(time, mStartTime) > Constants::FALLING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
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
	gWakingState.setRunMode(true);
	
	

	Debug::print(LOG_SUMMARY, "Falling Finished!\r\n");
	
}
Falling::Falling() : mLastPressure(0), mLastMotorPulseL(0), mLastMotorPulseR(0), mContinuousPressureCount(0), mCoutinuousGyroCount(0), mContinuousMotorPulseCount(0)
{
	setName("falling");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
}
Falling::~Falling()
{
}

//////Waking///////////////////
/////////////////////////////
bool Waking::onInit(const struct timespec& time)
{
	gMotorDrive.setRunMode(true);
	gMultiServo.setRunMode(true);
	gLED.setRunMode(true);
	gMultiServo.start(Constants::BACK_STABI_RUN_ANGLE);
	gNineAxisSensor.setRunMode(true);
	mLastUpdateTime = time;
	gMotorDrive.drive(100);

	return true;
}
void Waking::onClean()
{
	gMotorDrive.drive(0);
}
void Waking::onUpdate(const struct timespec& time)
{

	if (Time::dt(time, mLastUpdateTime) >= 2) {
		gMotorDrive.drive(0);
		nextState();
		return;
	}
	else {
		return;
	}

}

void Waking::nextState()
{
	//次の状態を設定
	gWakingState.setRunMode(false);
	gSeparatingState.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Waking Finished!\r\n");

}

bool Waking::onCommand(const std::vector<std::string>& args)
{
	return true;
}

Waking::Waking()
{
	setName("waking");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
}
Waking::~Waking()
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
	gMultiServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gNineAxisSensor.setRunMode(true);
	gLED.setRunMode(true);
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
		if(gNineAxisSensor.getAz() < 0)
		{
		  gMotorDrive.drive(100);
		}
		else
		{
		  gMotorDrive.drive(0);
		}

		if (Time::dt(time, mLastUpdateTime) < Constants::SEPARATING_SERVO_INTERVAL)return;
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
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, Constants::SEPARATING_SERVO_COUNT);

		if (mServoCount >= Constants::SEPARATING_SERVO_COUNT)//サーボを規定回数動かした
		{
			//次状態に遷移
			gMultiServo.stop();
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
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
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
	gSensorLoggingState.setRunMode(true);
	gLED.setRunMode(true);
	gMultiServo.setRunMode(true);
	gMultiServo.Running();//走っているときの角度に設定
	gNineAxisSensor.setRunMode(true);
	mLastNaviMoveCheckTime = time;
	mLastArmServoMoveTime = time;
	mLastArmServoStopTime = time;
	mLastUpdateTime = time;
	mArmStopFlag = true;
	mStuckFlag = false;
	mLastPos.clear();
  firstTime=true;
	Debug::print(LOG_SUMMARY, "initializing goal list \r\n", mGoalPos.x, mGoalPos.y);
  system("sudo ruby /home/pi/network/reset_goal.rb");
	getGoal(goal);

	//最初の座標をゴールにする
	//Debug::print(LOG_SUMMARY, "init goal is setted at ( %lf,%lf ) \r\n", mGoalPos.x, mGoalPos.y);
	mIsGoalPos = true;

	return true;
}
void Navigating::onUpdate(const struct timespec& time)
{

	//５秒置きに、GoalList を読み込む
	if (Time::dt(time, mLastUpdateTime) > 5.0) {
		//ファイルから　GoalList を読み込む、GoalList に保存する
		getGoal(goal);
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
	if (isNewData && isfinite(currentPos.x) && isfinite(currentPos.y) && isfinite(currentPos.z))
	{
    if(firstTime){
      Debug::print(LOG_SUMMARY, "Calculating route...\r\n",(int)goal.z);
      char s[100];
      sprintf(s,"ruby /home/pi/network/main.rb %f %f &",currentPos.y,currentPos.x);
      system(s);
      firstTime=false;
    }
		//最初の座標を取得したら移動を開始する
		if (mLastPos.empty())
		{
			Debug::print(LOG_SUMMARY, "Starting navigation...");
			Time::showNowTime();//制御開始時刻をログに出力
			//Debug::print(LOG_SUMMARY, "Control Start Point:(%f %f)\r\n", currentPos.x, currentPos.y);
			Debug::print(LOG_SUMMARY, " NAV START @%f,%f\r\n", currentPos.x, currentPos.y);
			gMotorDrive.drivePIDGyro(0, Constants::MOTOR_MAX_POWER, true);
			gMultiServo.Running();
			distance_from_goal_to_start = VECTOR3::calcDistanceXY(currentPos, mGoalPos);
			mLastNaviMoveCheckTime = time;
		}
		mLastPos.push_back(currentPos);
		//Time::showNowTime();
		//Debug::print(LOG_SUMMARY, "mLastPos.Size() = %d mLastpos.push(currentPos)= (%f,%f)\r\n",mLastPos.size(),currentPos.x,currentPos.y);
	}

  // ゴールもしくは探索終了もしくはルート計算中は停止
  if(goal.z<0){
    gMotorDrive.drive(0);
    return;
  }

	//ゴールとの距離を確認
	double distance = VECTOR3::calcDistanceXY(currentPos, mGoalPos);

	//途中のゴールに到達しているかのフラグ
	if (distance < Constants::NAVIGATING_GOAL_DISTANCE_THRESHOLD) {
		char s[60];
		sprintf(s,"ruby /home/pi/network/inform.rb %d",(int)goal.z);
		system(s);
    Debug::print(LOG_SUMMARY, "Block %d completed!\r\n",(int)goal.z);
    getGoal(goal);
	}

	//Navigating の更新頻度、何秒置き以下の処理をする
	if(Time::dt(time,mLastNaviMoveCheckTime) < Constants::NAVIGATING_DIRECTION_UPDATE_INTERVAL)return;
	mLastNaviMoveCheckTime = time;

	//異常値排除,2個以下なら、	removeError()=false
	if (removeError())
	{
		Time::showNowTime();
		Debug::print(LOG_SUMMARY, "NAVIGATING: GPS Error value detected\r\n");
		//return;
	}
    //スタックしたときの処理
  else if (isStuckByGPS()) {
		if (!gEscapingRandomState.isActive())
		{
			gEscapingByStabiState.setRunMode(true);
		}
		Time::showNowTime();
		//Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK =true, GPS=(%f %f)\r\n", currentPos.x, currentPos.y);
		Debug::print(LOG_SUMMARY, " NAV STUCK @%f,%f\r\n", currentPos.x, currentPos.y);
		gBuzzer.start(20, 10, 8);

	//esc by stabi と　esc by random の２つに繰り返す
		if (gEscapingByStabiState.isActive())		//EscapingByStabi中
		{
			if (gEscapingByStabiState.getTryCount() >= Constants::ESCAPING_BY_STABI_MAX_COUNT)
			{
				//EscapingRandomに移行
				gEscapingByStabiState.setRunMode(false);
				Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping Random Start! \r\n");
				gEscapingRandomState.setRunMode(true);
				mEscapingRandomStartTime = time;
			}
		}
		else if (gEscapingRandomState.isActive())	//EscapingRandom中
		{
			if (Time::dt(time, mEscapingRandomStartTime) > Constants::ESCAPING_RANDOM_TIME_THRESHOLD)
			{
				//EscapingByStabiに移行
				gEscapingRandomState.setRunMode(false);
				Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping ByStabi Start! \r\n");
				gEscapingByStabiState.setRunMode(true);
			}
		}
		return;//chou
	}

	// スタックしないときの処理
	else
	{
		if (gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())
		{
      gMultiServo.Running();
			gMotorDrive.drivePIDGyro(0, Constants::MOTOR_MAX_POWER, true);
			gEscapingByStabiState.setRunMode(false);
      gEscapingRandomState.setRunMode(false);
				Time::showNowTime();
			Debug::print(LOG_SUMMARY, "NAVIGATING: ESCAPING FINISHED,Navigating restart! \r\n");
			gBuzzer.start(20, 10, 3);
		}
		else
		{
			//スタックしない、escaping 終了したとき、通常のナビゲーション
//			if (mLastPos.size() < 10)
//			{
//				Time::showNowTime();
//				Debug::print(LOG_SUMMARY, "NAVIGATING: mLastPos.size=%d <10 NORMAL navigating \r\n",mLastPos.size());
//				return;
//			}		
		if (mLastPos.size() < mGpsCountMax)return;//過去の座標が1つ以上(現在の座標をあわせて2つ以上)なければ処理を返す(進行方向決定不可能)
		navigationMove(distance);//過去の座標から進行方向を変更する
    }
	}
	
	//方向変更したら、座標データをひとつ残して、mlastposのリストを削除
	currentPos = mLastPos.back();
	mLastPos.clear();
	mLastPos.push_back(currentPos);
}

bool Navigating::removeError()
{
	if (mLastPos.size() <= 2)
		Debug::print(LOG_SUMMARY, "in removeError,mLastpos <= 2, size()= %d",mLastPos.size());
		return false;//最低2点は残す
	std::list<VECTOR3>::iterator it = mLastPos.begin();
	VECTOR3 average, sigma;
	while (it != mLastPos.end())
	{
		average += *it;
		++it;
	}
	average /= mLastPos.size();

	const static double THRESHOLD = 100 / Constants::DEGREE_2_METER;
	it = mLastPos.begin();
	while (it != mLastPos.end())
	{
		if (VECTOR3::calcDistanceXY(average, *it) > THRESHOLD)
		{
			mLastPos.erase(it);
			Debug::print(LOG_SUMMARY, "In removeError GPS, mLastPos.erase(it)\r\n");
			removeError();
			return true;
		}
		++it;
	}
	return false;
}
bool Navigating::isStuckByGPS() 
{
	//スタック判定
	VECTOR3 averagePos1, averagePos2;
	unsigned int i, border;
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	
	//過去の位置が２個以上なら、2つに分けて、その平均の差を計算する
	//if (mLastPos.size()>8) {
		for (i = 0; i < mLastPos.size() / 2; ++i)
		{
			averagePos1 += *it;
			it++;
		}
		border = i;
		averagePos1 /= border;

		for (; i < mLastPos.size(); ++i)
		{
			averagePos2 += *it;
			it++;
		}
		averagePos2 /= i - border;
		double dist = VECTOR3::calcDistanceXY(averagePos1, averagePos2);
		//Debug::print(LOG_SUMMARY, "ave1(%f ,%f), ave2(%f ,%f) \r\n", averagePos1.x, averagePos1.y, averagePos2.x, averagePos2.y);
		//Debug::print(LOG_SUMMARY, "posSize = %d ,distance =%f\r\n", mLastPos.size()
			//, dist);
		 
		if (isfinite(dist) && dist<Constants::NAVIGATING_STUCK_JUDGEMENT_THRESHOLD) {
			//Debug::print(LOG_SUMMARY, "mLastPos.size()=%d, mStuckFlag = true\r\n",mLastPos.size());
			mStuckFlag = true;//移動量が閾値以下ならスタックと判定
		}
		else {
			//Debug::print(LOG_SUMMARY, "mLastPos.size()=%d, mStuckFlag = false\r\n",mLastPos.size());
			mStuckFlag = false;
		}
			
		return mStuckFlag;
	//}
	//else {
		//判断の位置数がある数以下なら、前回の結果を返す、判断しない
	//	Debug::print(LOG_SUMMARY, "mLastPos.size() <<8, mStuckFlag is unknow = %d \r\n",mStuckFlag);
	//	return mStuckFlag;
	//}

}
void Navigating::navigationMove(double distance) const
{
	//過去の座標の平均値を計算する(currentPos を除く)
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
	double currentDirection=0;
	double newDirection = -VECTOR3::calcAngleXY(currentPos, mGoalPos);//ゴールの方向
switch (mMethod) {
	case 1://従来手法 サンプルをとって方向推定
		currentDirection = -VECTOR3::calcAngleXY(averagePos, currentPos);//ローバーの方向
		break;
	case 2://gpsdライブラリのcourseを使った方向推定
		currentDirection =  -gGPSSensor.getCourse();
		break;
	case 3://上の2手法の平均をとってる
		currentDirection = (( gGPSSensor.getCourse()) + (-VECTOR3::calcAngleXY(averagePos, currentPos))) / 2;
		break;
	case 4://use magnet
		currentDirection = -gNineAxisSensor.getMagnetPhi();
		break;
	default:
		break;
	}
double deltaDirection = NineAxisSensor::normalize(newDirection - currentDirection);
	deltaDirection = std::max(std::min(deltaDirection, Constants::NAVIGATING_MAX_DELTA_DIRECTION), -1 * Constants::NAVIGATING_MAX_DELTA_DIRECTION);

	//新しい速度を計算
	double speed = Constants::MOTOR_MAX_POWER;
	if (distance < Constants::NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD)
	{
		speed *= Constants::NAVIGATING_GOAL_APPROACH_POWER_RATE;	//接近したら速度を落とす
	}

	//Debug::print(LOG_SUMMARY, "NAVIGATING: Last %d samples (%f %f) Current(%f %f)\r\n", mLastPos.size(), averagePos.x, averagePos.y, currentPos.x, currentPos.y);
  //Debug::print(LOG_SUMMARY, "current angle = %f goal angle = %f",currentDirection, newDirection);
	Debug::print(LOG_SUMMARY, "distance = %f (m)\r\n", distance * Constants::DEGREE_2_METER);
	if(deltaDirection>0)
	  Debug::print(LOG_SUMMARY, " NAV LEFT %f @%f,%f\r\n", deltaDirection, currentPos.x, currentPos.y);
	else
	  Debug::print(LOG_SUMMARY, " NAV RIGHT %f @%f,%f\r\n", -deltaDirection, currentPos.x, currentPos.y);

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
    else if(args[1].compare("method") == 0)
    {
      Debug::print(LOG_SUMMARY,"now method is %d\r\n",mMethod);
      return true;
    }
    else if(args[1].compare("count") == 0)
    {
      Debug::print(LOG_SUMMARY,"gps count max is %d\r\n",mGpsCountMax);
      return true;
    }
	}
	if (args.size() == 3)
	{
    if(args[1].compare("method") == 0)
    {
      mMethod = atof(args[2].c_str());
      return true;
    }
    else if (args[1].compare("count") == 0)
    {
      mGpsCountMax = atof(args[2].c_str());
      return true;
    }
    else{ 
		  VECTOR3 pos;
		  pos.x = atof(args[1].c_str());
		  pos.y = atof(args[2].c_str());

	  	setGoal(pos);
		return true;
    }
	}
	Debug::print(LOG_PRINT, "navigating                 : get goal\r\n\
							navigating [pos x] [pos y] : set goal at specified position\r\n\
							navigating here            : set goal at current position\r\n\
							navigating goal            : call nextState\r\n\
              navigating method [n]      : set navigation method\r\n\
              navigating count [n]       : set GPS count max\r\n");
	return true;
}

//次の状態に移行
void Navigating::nextState()
{
	gBuzzer.start(1000);

	gMotorDrive.drive(0);//念のため2回
	gMotorDrive.drive(0);

	Time::showNowTime();
	Debug::print(LOG_SUMMARY, "navigating finished\r\n");
	system("sudo ruby /home/pi/network/exit_navigation.rb");
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
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
  mMethod = 1;
  mGpsCountMax = 5;
}
void Navigating::getGoal(VECTOR3& goal) {

	//goallist 読み込む

	//ファイルの読み込み
	std::ifstream ifs("../goal_list");
	if (!ifs) {
		Debug::print(LOG_SUMMARY, "can not find goal_list \r\n");
	}

	//txtファイルを1行ずつ読み込む
	std::string str;
  double tmp_z=goal.z;

  int count=0;
	while (getline(ifs, str)) {
    count++;
		//一行ずつ読み込む、VECTOR３に代入、GoalListに保存する
		//Debug::print(LOG_SUMMARY, "%s\r\n",str.c_str());
		double a, b, c;
		sscanf(str.c_str(), "%lf, %lf, %lf", &a, &b, &c);
    if(c==tmp_z) // 更新なければここで終了
      return;
		goal.x = a;
		goal.y = b;
		goal.z = c;
    Debug::print(LOG_SUMMARY,"Get Goal from Goalist ( %.8lf %.8lf ); ID=%d\r\n",goal.x,goal.y,(int)goal.z);
    if(count>0)
      break;
	}

  if (goal.z == -1) {
    //見つかりません、とりあえず停止
    gMotorDrive.drive(0);
    Debug::print(LOG_SUMMARY, "(%d)can not find the object, stop rover\r\n",(int)goal.z);
    Debug::print(LOG_SUMMARY, " NAV STOP @%f,%f\r\n",currentPos.x,currentPos.y);
    nextState();
    return;
  }
  else if (goal.z == -2) {

    //Goal 判定した、終わり
    Debug::print(LOG_SUMMARY, "(%d)Find the object, mission finished\r\n",(int)goal.z);
    Debug::print(LOG_SUMMARY, " NAV TARGET @%f,%f\r\n",currentPos.x,currentPos.y);
    nextState();
    return;
  }
  else if(goal.z==-3) {

    //ルート計算中、待機する
    gMotorDrive.drive(0);
    Debug::print(LOG_SUMMARY,"(%d)Calculating the route, waiting... \r\n",(int)goal.z);
  } else{
    Debug::print(LOG_SUMMARY, "(%d) goal is setted at ( %f,%f ) \r\n", (int)goal.z,goal.x, goal.y);
    Debug::print(LOG_SUMMARY, " NAV SET %d @%f,%f\r\n",(int)goal.z,goal.x,goal.y);
    mGoalPos=goal;
    mIsGoalPos = true;
  }
}
Navigating::~Navigating()
{
};

bool Blinding::onInit(const struct timespec& time) {
	Debug::print(LOG_SUMMARY, "Blinding...\r\n");
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);
	gLED.setRunMode(true);
	gMultiServo.setRunMode(true);
	gMultiServo.Running();//走っているときの角度に設定
	gNineAxisSensor.setRunMode(true);
	mStuckFlag = false;

	mLastListWriteTime = time;
	mLastCheckTime = time;

	return true;
};

void Blinding::set_goal(double dis, double angle) {
	double pos = polar_to_xy(dis, angle);
	goal = pos;
};

void Blinding::move() {
	//今の座標と目標座標からモーターの角度を変更
	motorangle = atan2f(goal, currentpos);
	gMotorDrive.drivePIDGyro(motorangle, myspeed, true);
};

void Blinding::polar_to_xy(double dis, double angle) {
	//極座標からｘｙ座標に変換
	double pos[2];
	pos[0] = cos(angle) * dis;
	pos[1] = sin(angle) * dis;
	return pos;
};

void Blinding::onUpdate(const struct timespec& time) {
	//if (isStuckByGPS()) {
	//	//poslistの最後の5秒の記録を全部5秒前の位置に変更

	//	if (!gEscapingRandomState.isActive()) {
	//		gEscapingByStabiState.setRunMode(true);
	//	}
	//	Time::showNowTime();
	//	Debug::print(LOG_SUMMARY, " Blind STUCK @%f,%f\r\n", currentPos.x, currentPos.y);
	//	gBuzzer.start(20, 10, 8);

	//	//esc by stabi と　esc by random の２つに繰り返す
	//	if (gEscapingByStabiState.isActive()) {
	//		//EscapingByStabi中
	//		if (gEscapingByStabiState.getTryCount() >= Constants::ESCAPING_BY_STABI_MAX_COUNT) {
	//			//EscapingRandomに移行
	//			gEscapingByStabiState.setRunMode(false);
	//			Debug::print(LOG_SUMMARY, "Blinding: Escaping Random Start! \r\n");
	//			gEscapingRandomState.setRunMode(true);
	//			mEscapingRandomStartTime = time;
	//		}
	//	}
	//	else if (gEscapingRandomState.isActive()) {
	//		//EscapingRandom中		
	//		if (Time::dt(time, mEscapingRandomStartTime) > Constants::ESCAPING_RANDOM_TIME_THRESHOLD) {
	//			//EscapingByStabiに移行
	//			gEscapingRandomState.setRunMode(false);
	//			Debug::print(LOG_SUMMARY, "Blinding: Escaping ByStabi Start! \r\n");
	//			gEscapingByStabiState.setRunMode(true);
	//		}
	//	}
	//	return;
	//}
	//else {
		//今の自分の座標を更新
		periodtime = time - mLastCheckTime;
		mLastCheckTime = time;
		double acc;
		acc = pow(pow(NineAxisSensor.getAx() - averageAx, 2) +
			pow(NineAxisSensor.getAy() - averageAy, 2) +
			pow(NineAxisSensor.getAz() - averageAz, 2), 0.5);
		periodspeed = acc * ConstantNineAxisPeriod;
		dx = periodspeed * periodtime;
		currentangle = NineAxisSensor.getMagnetPhi();
		currentPos[0] += cos(currentangle) * dx;
		currentPos[1] += sin(currentangle) * dx;

		if (Time::dt(time, mLastListWriteTime) > 1.0) {
			//1秒ごときposリストを更新
			mLastListWriteTime = time;
			VECTOR3 i;
			i.x = time;
			i.y = currtpos[0];
			i.z = currtpos[1];
			mylist.push_back(i);
		}

		//目標に向かう
		Blinding.move(currentPos);
	//}
};
bool Blinding::onCommand(const std::vector<std::string>& args){
	if (args.size() == 1){
		Debug::print(LOG_SUMMARY, "happy cafe");		
	}
	if (args.size() == 3) {
		double dis = atof(args[1].c_str());
		double angle = atof(args[2].c_str());//単位が度
		angle = angle / 360 * M_PI;
		set_goal(dis, angle);
		Debug::print("seted");
		return true;
	}
};

Blinding::Blinding(){
	setName("blind");
	setPriority(Constants::TASK_PRIORITY_SEQUENCE, Constants::TASK_INTERVAL_SEQUENCE);
};
void Blinding::nextState(){
	gBuzzer.start(1000);

	gMotorDrive.drive(0);//念のため2回
	gMotorDrive.drive(0);

	Time::showNowTime();
	Debug::print(LOG_SUMMARY, "Blinding finished\r\n");	

};