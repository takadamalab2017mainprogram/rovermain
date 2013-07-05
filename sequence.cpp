#include<stdlib.h>
#include <fstream>
#include "sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"

Testing gTestingState;
Waiting gWaitingState;
Falling gFallingState;
Separating gSeparatingState;
Navigating gNavigatingState;

bool Testing::onInit(const struct timespec& time)
{
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gServo.setRunMode(true);
	gXbeeSleep.setRunMode(true);

	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	gWebCamera.setRunMode(true);

	gMotorDrive.setRunMode(true);

	gSerialCommand.setRunMode(true);

	return true;
}
bool Testing::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 3)
	{
		if(args[1].compare("start") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if(pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Start %s\r\n",args[2].c_str());
				pTask->setRunMode(true);
				return true;
			}else Debug::print(LOG_SUMMARY, "%s Not Found\r\n",args[2].c_str());
			return false;
		}else if(args[1].compare("stop") == 0)
		{
			TaskBase* pTask = TaskManager::getInstance()->get(args[2]);
			if(pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Stop %s\r\n",args[2].c_str());
				pTask->setRunMode(true);
				return true;
			}else Debug::print(LOG_SUMMARY, "%s Not Found\r\n",args[2].c_str());
			return false;
		}
	}
	Debug::print(LOG_PRINT, "testing [start/stop] [task name]  : enable/disable task\r\n");

	return true;
}
Testing::Testing()
{
	setName("testing");
	setPriority(UINT_MAX,UINT_MAX);
}
Testing::~Testing()
{
}


bool Waiting::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Waiting...\r\n");

	mContinuousLightCount = 0;

	//現在の時刻を保存
	mStartTime = time;

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gLightSensor.setRunMode(true);
	gXbeeSleep.setRunMode(true);
	gBuzzer.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Disable Communication\r\ncya!\r\n");

	return true;
}
void Waiting::nextState()
{
	gBuzzer.start(100);

	//スリープを解除
	gXbeeSleep.setState(false);
	//次の状態を設定
	gFallingState.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Waiting Finished!\r\n");
}
void Waiting::onUpdate(const struct timespec& time)
{
	//XBeeをスリープモードに設定(ロケット内電波規制)
	gXbeeSleep.setState(true);

	//明るい場合カウント
	if(gLightSensor.get())
	{
		++mContinuousLightCount;
		gBuzzer.start(2);
	}else mContinuousLightCount = 0;

	if(mContinuousLightCount >= WAITING_LIGHT_COUNT)//明るい場合放出判定
	{
		nextState();
		return;
	}

	if(Time::dt(time,mStartTime) > WAITING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
		nextState();
		return;
	}
}
Waiting::Waiting()
{
	setName("waiting");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Waiting::~Waiting(){}

bool Falling::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Falling...\r\n");

	mStartTime = mLastCheckTime = time;
	mLastPressure = 0;
	mContinuousPressureCount = 0;
	mCoutinuousGyroCount = 0;

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gPressureSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);

	return true;
}
void Falling::onUpdate(const struct timespec& time)
{
	if(mLastPressure == 0)mLastPressure = gPressureSensor.get();

	//角速度が閾値以下ならカウント
	if(gGyroSensor.getRvx() < FALLING_GYRO_THRESHOLD && gGyroSensor.getRvy() < FALLING_GYRO_THRESHOLD && gGyroSensor.getRvz() < FALLING_GYRO_THRESHOLD)
	{
		if(mCoutinuousGyroCount < FALLING_GYRO_COUNT)++mCoutinuousGyroCount;
	}else mCoutinuousGyroCount = 0;

	//1秒ごとに以下の処理を行う
	if(Time::dt(time,mLastCheckTime) < 1)return;
	mLastCheckTime = time;

	//気圧の差が一定以下ならカウント
	int newPressure = gPressureSensor.get();
	if(abs((int)(newPressure - mLastPressure)) < FALLING_DELTA_PRESSURE_THRESHOLD)
	{
		if(mContinuousPressureCount < FALLING_PRESSURE_COUNT)++mContinuousPressureCount;
		Debug::print(LOG_SUMMARY, "Pressure Count %d / %d\r\n",mContinuousPressureCount,FALLING_PRESSURE_COUNT);
	}else mContinuousPressureCount = 0;

	//ジャイロの判定状態を表示
	Debug::print(LOG_SUMMARY, "Gyro Count     %d / %d\r\n",mCoutinuousGyroCount,FALLING_GYRO_COUNT);

	//カウント回数が一定以上なら次の状態に移行
	if(mContinuousPressureCount >= FALLING_PRESSURE_COUNT && mCoutinuousGyroCount >= FALLING_GYRO_COUNT)
	{
		nextState();
		return;
	}

	if(Time::dt(time,mStartTime) > FALLING_ABORT_TIME)//一定時間が経過したら次の状態に強制変更
	{
		Debug::print(LOG_SUMMARY, "Waiting Timeout\r\n");
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
Falling::Falling() : mLastPressure(0),mContinuousPressureCount(0),mCoutinuousGyroCount(0)
{
	setName("falling");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Falling::~Falling()
{
}

bool Separating::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Separating...\r\n");

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gServo.setRunMode(true);
	gSerialCommand.setRunMode(true);

	mLastUpdateTime = time;
	gServo.start(0);
	mCurServoState = false;
	mServoCount = 0;

	return true;
}
void Separating::onUpdate(const struct timespec& time)
{
	if(Time::dt(time,mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
	mLastUpdateTime = time;

	mCurServoState = !mCurServoState;
	gServo.start(mCurServoState);
	++mServoCount;

	if(mServoCount >= SEPARATING_SERVO_COUNT)
	{
		nextState();
	}
}

void Separating::nextState()
{
	gBuzzer.start(100);

	//次の状態を設定
	gNavigatingState.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Separating Finished!\r\n");
}

Separating::Separating() : mCurServoState(false),mServoCount(0)
{
	setName("separating");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Separating::~Separating()
{
}
//ゴールへの移動中
bool Navigating::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Navigating...\r\n");

	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);

	mLastCheckTime = time;
	mLastPos.clear();

	return true;
}
void Navigating::onUpdate(const struct timespec& time)
{
	VECTOR3 currentPos;

	bool isNewData = gGPSSensor.isNewPos();
	//新しい位置を取得
	if(!gGPSSensor.get(currentPos))return;

	//最初の座標を取得したら移動を開始する
	if(mLastPos.empty())
	{
		Debug::print(LOG_SUMMARY, "Starting navigation...\r\n");
		gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
	}

	//新しい座標であればキューに追加
	if(isNewData)
	{
		Debug::print(LOG_DETAIL, "NAVIGATING: Got Position (%f %f %f)\r\n",currentPos.x,currentPos.y,currentPos.z);
		mLastPos.push_back(currentPos);
	}

	//古いデータを削除
	//while(mLastPos.size() > NAVIGATING_NUMBER_OF_POSITION_HISTORIES)mLastPos.pop_front();

	//ゴールが設定されているか確認
	if(!mIsGoalPos)
	{
		//ゴールが設定されていないため移動できない
		return;
	}

	//ゴールとの距離を確認
	double distance = VECTOR3::calcDistanceXY(currentPos,mGoalPos);
	if(distance < NAVIGATING_GOAL_DISTANCE_THRESHOLD)
	{
		//ゴール判定
		gMotorDrive.drive(0,0);
		nextState();
		return;
	}

	//数秒たっていなければ処理を返す
	if(Time::dt(time,mLastCheckTime) < NAVIGATING_DIRECTION_UPDATE_INTERVAL)return;
	mLastCheckTime = time;

	//過去の座標の平均値を計算する
	VECTOR3 averagePos;
	mLastPos.pop_back();//現在の位置を一時的に取り除く
	std::list<VECTOR3>::iterator it = mLastPos.begin();
	while(it != mLastPos.end())
	{
		averagePos += *it;
		++it;
	}
	averagePos /= mLastPos.size();

	mLastPos.push_back(currentPos);//現在の位置を再び追加する

	//新しい角度を計算
	double currentDirection = -VECTOR3::calcAngleXY(averagePos,currentPos);
	double newDirection = -VECTOR3::calcAngleXY(currentPos,mGoalPos);
	double deltaDirection = GyroSensor::normalize(newDirection - currentDirection);

	//新しい速度を計算
	double speed = MOTOR_MAX_POWER;
	if(distance < NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD)speed *= NAVIGATING_GOAL_APPROACH_POWER_RATE;//接近したら速度を落とす

	Debug::print(LOG_SUMMARY, "NAVIGATING: Last %d samples (%f %f) Current(%f %f)\r\n",mLastPos.size(),averagePos.x,averagePos.y,currentPos.x,currentPos.y);
	Debug::print(LOG_SUMMARY, "distance = %f (m)  angles(delta: %f old: %f new:%f)\r\n",distance * DEGREE_2_METER,deltaDirection,currentDirection,newDirection);

	//方向と速度を変更
	gMotorDrive.drivePID(deltaDirection ,speed);

	//座標データをひとつ残して削除
	while(mLastPos.size() > 1)mLastPos.pop_front();
}
bool Navigating::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 1)
	{
		VECTOR3 pos;
		if(!gGPSSensor.get(pos))
		{
			Debug::print(LOG_SUMMARY, "Unable to get current position!\r\n");
			return true;
		}

		setGoal(pos);
		return true;
	}
	if(args.size() == 3)
	{
		VECTOR3 pos;
		pos.x = atof(args[1].c_str());
		pos.y = atof(args[2].c_str());

		setGoal(pos);
		return true;
	}
	Debug::print(LOG_PRINT, "navigating [pos x] [pos y]\r\n");
	return true;
}
//次の状態に移行
void Navigating::nextState()
{
	gBuzzer.start(1000);

	//次の状態を設定
	gTestingState.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Goal!\r\n");
}
void Navigating::setGoal(const VECTOR3& pos)
{
	mIsGoalPos = true;
	mGoalPos = pos;
	Debug::print(LOG_SUMMARY, "Set Goal ( %f ,%f )\r\n",mGoalPos.x,mGoalPos.y);
}
Navigating::Navigating() : mGoalPos(),  mIsGoalPos(false), mLastPos()
{
	setName("navigating");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Navigating::~Navigating()
{
}
