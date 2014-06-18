#include<stdlib.h>
#include <math.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <stdarg.h>
#include "sequence.h"
#include "utils.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"
#include "motor.h"
#include "image_proc.h"

Testing gTestingState;
Waiting gWaitingState;
Falling gFallingState;
Separating gSeparatingState;
Navigating gNavigatingState;
Escaping gEscapingState;
EscapingRandom gEscapingRandomState;
Waking gWakingState;
Turning gTurningState;
Avoiding gAvoidingState;
WadachiPredicting gPredictingState;
PictureTaking gPictureTakingState;
SensorLogging gSensorLoggingState;
ColorAccessing ColorAccessing;

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
	//gAccelerationSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	gWebCamera.setRunMode(true);
	//gDistanceSensor.setRunMode(true);
	gCameraCapture.setRunMode(true);

	gMotorDrive.setRunMode(true);

	gSerialCommand.setRunMode(true);

	return true;
}
bool Testing::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("sensor") == 0)
		{
			Debug::print(LOG_SUMMARY, "*** Sensor states ***\r\n");
			
			VECTOR3 vec;
			gGPSSensor.get(vec);
			if(gGPSSensor.isActive())Debug::print(LOG_SUMMARY, " GPS      (%f %f %f)\r\n",vec.x,vec.y,vec.z);

			if(gPressureSensor.isActive())Debug::print(LOG_SUMMARY, " Pressure (%d) hPa\r\n",gPressureSensor.get());

			gGyroSensor.getRPos(vec);
			if(gGyroSensor.isActive())Debug::print(LOG_SUMMARY, " Gyro pos (%f %f %f) d\r\n",vec.x,vec.y,vec.z);
			gGyroSensor.getRVel(vec);
			if(gGyroSensor.isActive())Debug::print(LOG_SUMMARY, " Gyro vel (%f %f %f) dps\r\n",vec.x,vec.y,vec.z);

			if(gLightSensor.isActive())Debug::print(LOG_SUMMARY, " Light    (%s)\r\n",gLightSensor.get() ? "High" : "Low");
			return true;
		}else if(args[1].compare("waking") == 0)
		{
			gWakingState.setRunMode(true);
		}
	}
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
				pTask->setRunMode(false);
				return true;
			}else Debug::print(LOG_SUMMARY, "%s Not Found\r\n",args[2].c_str());
			return false;
		}
	}
	Debug::print(LOG_PRINT, "testing [start/stop] [task name]  : enable/disable task\r\n\
testing sensor                    : check sensor values\r\n");

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
	gSensorLoggingState.setRunMode(true);

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
	gGPSSensor.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	return true;
}
void Falling::onUpdate(const struct timespec& time)
{
	//初回のみ気圧を取得
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
	}else mContinuousPressureCount = 0;
	mLastPressure = newPressure;

	//エンコーダの値の差が一定以上ならカウント
	unsigned long long newMotorPulseL = gMotorDrive.getL(), newMotorPulseR = gMotorDrive.getR();
	if(newMotorPulseL - mLastMotorPulseL > FALLING_MOTOR_PULSE_THRESHOLD || newMotorPulseR - mLastMotorPulseR > FALLING_MOTOR_PULSE_THRESHOLD)
	{
		if(mContinuousMotorPulseCount < FALLING_MOTOR_PULSE_COUNT)++mContinuousMotorPulseCount;
	}else mContinuousMotorPulseCount = 0;

	//判定状態を表示
	Debug::print(LOG_SUMMARY, "Pressure Count   %d / %d (%d hPa)\r\n",mContinuousPressureCount,FALLING_PRESSURE_COUNT,newPressure);
	Debug::print(LOG_SUMMARY, "Gyro Count       %d / %d\r\n",mCoutinuousGyroCount,FALLING_GYRO_COUNT);
	Debug::print(LOG_SUMMARY, "MotorPulse Count %d / %d (%llu,%llu)\r\n",mContinuousMotorPulseCount,FALLING_MOTOR_PULSE_COUNT,newMotorPulseL - mLastMotorPulseL,newMotorPulseR - mLastMotorPulseR);

	mLastMotorPulseL = newMotorPulseL;
	mLastMotorPulseR = newMotorPulseR;

	//GPS情報ログ
	VECTOR3 pos;
	if(gGPSSensor.get(pos))Debug::print(LOG_SUMMARY, "GPS Position     (%f %f %f)\r\n",pos.x,pos.y,pos.z);
	else Debug::print(LOG_SUMMARY, "GPS Position     Unable to get\r\n");

	//カウント回数が一定以上なら次の状態に移行
	if(mContinuousPressureCount >= FALLING_PRESSURE_COUNT && (mCoutinuousGyroCount >= FALLING_GYRO_COUNT || mContinuousMotorPulseCount >= FALLING_MOTOR_PULSE_COUNT))
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
Falling::Falling() : mLastPressure(0),mLastMotorPulseL(0),mLastMotorPulseR(0),mContinuousPressureCount(0),mCoutinuousGyroCount(0),mContinuousMotorPulseCount(0)
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
	gMotorDrive.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	mLastUpdateTime = time;
	gServo.start(0);
	mCurServoState = false;
	mServoCount = 0;
	mCurStep = STEP_SEPARATE;

	return true;
}
void Separating::onUpdate(const struct timespec& time)
{
	switch(mCurStep)
	{
	case STEP_SEPARATE:
		//パラシュートを切り離す
		if(Time::dt(time,mLastUpdateTime) < SEPARATING_SERVO_INTERVAL)return;
		mLastUpdateTime = time;

		mCurServoState = !mCurServoState;
		gServo.start(mCurServoState);
		++mServoCount;
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if(mServoCount >= SEPARATING_SERVO_COUNT)//サーボを規定回数動かした
		{
			//次状態に遷移
			mLastUpdateTime = time;
			mCurStep = STEP_PRE_PARA_JUDGE;
			gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_PARA_JUDGE:
		//起き上がり動作を実行し、画像処理を行う前に1秒待機して画像のブレを防止する
		if(gWakingState.isActive())mLastUpdateTime = time;//起き上がり動作中は待機する
		if(Time::dt(time,mLastUpdateTime) > 1)//起き上がり動作後1秒待機する
		{
			//次状態に遷移
			mLastUpdateTime = time;
			mCurStep = STEP_PARA_JUDGE;
			gCameraCapture.startWarming();
		}
		break;
	case STEP_PARA_JUDGE:
		//ローバーを起こし終わったら，パラシュート検知を行い，存在する場合は回避行動に遷移する
		if(Time::dt(time,mLastUpdateTime) > 2)
		{
			//パラシュートの存在チェックを行う
			IplImage* pImage = gCameraCapture.getFrame();
			if(gImageProc.isParaExist(pImage))
			{
				//回避動作に遷移
				mCurStep = STEP_PARA_DODGE;
				mLastUpdateTime = time;
				gTurningState.setRunMode(true);
				Debug::print(LOG_SUMMARY, "Para check: Found!!\r\n");
			}else
			{
				//次状態(ナビ)に遷移
				Debug::print(LOG_SUMMARY, "Para check: Not Found!!\r\n");
				nextState();
			}
			//パラ検知に用いた画像を保存する
			gCameraCapture.save(NULL, pImage);
		}
		break;
	case STEP_PARA_DODGE:
		if(!gTurningState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Para check: Turn Finished!\r\n");
			nextState();
		}
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
	gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	mLastCheckTime = time;
	mLastPos.clear();

	return true;
}
void Navigating::onUpdate(const struct timespec& time)
{
	VECTOR3 currentPos;

	//ゴールが設定されているか確認
	if(!mIsGoalPos)
	{
		//ゴールが設定されていないため移動できない
		Debug::print(LOG_SUMMARY, "NAVIGATING : Please set goal!\r\n");
		gMotorDrive.drive(0,0);
		nextState();
		return;
	}

	bool isNewData = gGPSSensor.isNewPos();
	//新しい位置を取得できなければ処理を返す
	if(!gGPSSensor.get(currentPos,false))return;


	//新しい座標であればバッファに追加
	if(isNewData && finite(currentPos.x) && finite(currentPos.y) && finite(currentPos.z))
	{
		//最初の座標を取得したら移動を開始する
		if(mLastPos.empty())
		{
			Debug::print(LOG_SUMMARY, "Starting navigation...\r\n");
			gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
			gPredictingState.setRunMode(true);
			mLastCheckTime = time;
		}
		mLastPos.push_back(currentPos);
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

	//異常値排除
	if(removeError())
	{
		Debug::print(LOG_SUMMARY, "NAVIGATING: GPS Error value detected\r\n");
	}

	if(gPredictingState.isWorking(time))
	{
		//轍回避中
	}else if(isStuck())//スタック判定
	{
		Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK detected at (%f %f)\r\n",currentPos.x,currentPos.y);
		gEscapingState.setRunMode(true);
	}else
	{
		if(gEscapingState.isActive())//脱出モードが完了した時
		{
			//ローバーがひっくり返っている可能性があるため、しばらく前進する
			gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
			gEscapingState.setRunMode(false);
		}else
		{
			//通常のナビゲーション
			if(mLastPos.size() < 2)return;//過去の座標が1つ以上(現在の座標をあわせて2つ以上)なければ処理を返す(進行方向決定不可能)
			navigationMove(distance);//過去の座標から進行方向を変更する
		}
	}

	//座標データをひとつ残して削除
	currentPos = mLastPos.back();
	mLastPos.clear();
	mLastPos.push_back(currentPos);
}
bool Navigating::removeError()
{
	if(mLastPos.size() <= 2)return false;//最低2点は残す
	std::list<VECTOR3>::iterator it = mLastPos.begin();
	VECTOR3 average,sigma;
	while(it != mLastPos.end())
	{
		average += *it;
		++it;
	}
	average /= mLastPos.size();
	
	const static double THRESHOLD = 100 / DEGREE_2_METER;
	it = mLastPos.begin();
	while(it != mLastPos.end())
	{
		if(VECTOR3::calcDistanceXY(average,*it) > THRESHOLD)
		{
			mLastPos.erase(it);
			removeError();
			return true;
		}
		++it;
	}
	return false;
}
bool Navigating::isStuck() const
{
	//スタック判定
	VECTOR3 averagePos1,averagePos2;
	unsigned int i,border;
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	for(i = 0;i < mLastPos.size() / 2;++i)
	{
		averagePos1 += *it;
		it++;
	}
	averagePos1 /= border = i;

	for(;i < mLastPos.size();++i)
	{
		averagePos2 += *it;
		it++;
	}
	averagePos2 /= i - border;

	return VECTOR3::calcDistanceXY(averagePos1,averagePos2) < NAVIGATING_STUCK_JUDGEMENT_THRESHOLD;//移動量が閾値以下ならスタックと判定
}
void Navigating::navigationMove(double distance) const
{
	//過去の座標の平均値を計算する
	VECTOR3 averagePos;
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	while(it != mLastPos.end())
	{
		averagePos += *it;
		++it;
	}
	averagePos -= mLastPos.back();
	averagePos /= mLastPos.size() - 1;

	//新しい角度を計算
	VECTOR3 currentPos = mLastPos.back();
	double currentDirection = -VECTOR3::calcAngleXY(averagePos,currentPos);
	double newDirection = -VECTOR3::calcAngleXY(currentPos,mGoalPos);
	double deltaDirection = GyroSensor::normalize(newDirection - currentDirection);
	deltaDirection = std::max(std::min(deltaDirection,NAVIGATING_MAX_DELTA_DIRECTION),-1 * NAVIGATING_MAX_DELTA_DIRECTION);

	//新しい速度を計算
	double speed = MOTOR_MAX_POWER;
	if(distance < NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD)speed *= NAVIGATING_GOAL_APPROACH_POWER_RATE;//接近したら速度を落とす

	Debug::print(LOG_SUMMARY, "NAVIGATING: Last %d samples (%f %f) Current(%f %f)\r\n",mLastPos.size(),averagePos.x,averagePos.y,currentPos.x,currentPos.y);
	Debug::print(LOG_SUMMARY, "distance = %f (m)  delta angle = %f(%s)\r\n",distance * DEGREE_2_METER,deltaDirection,deltaDirection > 0 ? "LEFT" : "RIGHT");

	//方向と速度を変更
	gMotorDrive.drivePID(deltaDirection ,speed);
}
bool Navigating::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 1)
	{
		if(mIsGoalPos)Debug::print(LOG_SUMMARY ,"Current Goal (%f %f)\r\n",mGoalPos.x,mGoalPos.y);
		else Debug::print(LOG_SUMMARY ,"NO Goal\r\n");
	}
	if(args.size() == 2)
	{
		if(args[1].compare("here") == 0)
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
	}
	if(args.size() == 3)
	{
		VECTOR3 pos;
		pos.x = atof(args[1].c_str());
		pos.y = atof(args[2].c_str());

		setGoal(pos);
		return true;
	}
	Debug::print(LOG_PRINT, "navigating                 : get goal\r\n\
navigating [pos x] [pos y] : set goal at specified position\r\n\
navigating here            : set goal at current position\r\n");
	return true;
}
//次の状態に移行
void Navigating::nextState()
{
	gBuzzer.start(1000);

	//次の状態を設定
	gTestingState.setRunMode(true);
	gPictureTakingState.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Goal!\r\n");
}
void Navigating::setGoal(const VECTOR3& pos)
{
	mIsGoalPos = true;
	mGoalPos = pos;
	Debug::print(LOG_SUMMARY, "Set Goal ( %f %f )\r\n",mGoalPos.x,mGoalPos.y);
}
Navigating::Navigating() : mGoalPos(),  mIsGoalPos(false), mLastPos()
{
	setName("navigating");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Navigating::~Navigating()
{
}

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
			//轍を事前検知した
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
				//轍を事前検知した
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

/* ここから　2014年6月オープンラボ前に実装 */
bool ColorAccessing::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	gCameraCapture.startWarming();
	gMotorDrive.setRunMode(true);
    mIsLastActionStraight = false;
	return true;
}
void ColorAccessing::onUpdate(const struct timespec& time)
{
	if(gAvoidingState.isActive())return;

	switch(mCurStep)
	{
	case STEP_STARTING:
		if(!gWakingState.isActive())
		{
			Debug::print(LOG_SUMMARY, "Detecting: Checking started\r\n");
			mCurStep = STEP_CHECKING;
			mLastUpdateTime = time;
			gCameraCapture.startWarming();
            mAngleOnBegin = gGyroSensor.getRvx();
		}
		break;
	case STEP_CHECKING:
		if(Time::dt(time,mLastUpdateTime) > 1.0)
		{
			Debug::print(LOG_SUMMARY, "Detecting: Approaching started\r\n");
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			int x_pos = gImageProc.howColorGap(pImage);
			
            //色検知したよ
            if(x_pos != INT_MAX)
			{
				mLastUpdateTime = time;
				if(x_pos < -80){
					mCurStep = STEP_STOPPING_FAST;
					gMotorDrive.drive(0,40);
                    mIsLastActionStraight = false;
				}
				else if(80 < x_pos){
					mCurStep = STEP_STOPPING_FAST;
					gMotorDrive.drive(40,0);
                    mIsLastActionStraight = false;
				}
				else{
					mCurStep = STEP_STOPPING_LONG;
					gMotorDrive.drive(40,40);
                    mIsLastActionStraight = true;
                    mAngleOnBegin = gGyroSensor.getRz();
				}
			}
			else//色検知しなかったら
			{
                // もし前回の行動が直進なら．
                if (mIsLastActionStraight)//前回の行動が直進なら．
                {
                	double diff = GyroSensor::normalize(gGyroSensor.getRz() - mAngleOnBegin);

                	Debug::print(LOG_SUMMARY, "diff=%f\r\n", diff);

                    if (diff < 0)
                    {
                        //右に向いた時の行動
                        mCurStep = STEP_STOPPING_FAST;
                        gMotorDrive.drive(0,40);
                    }
                    else
                    {
                        //左に向いた時の行動
                        mCurStep = STEP_STOPPING_FAST;
                        gMotorDrive.drive(40,0);
                    }
                }
                //前回の行動が直進以外なら
				else
                {
                	double diff = GyroSensor::normalize(gGyroSensor.getRz() - mAngleOnBegin);

                	mCurStep = STEP_TURNING;

                	if ( diff < 0 )
                	{
                		//右を向いていた時の処理．
                		mCurStep = STEP_TURNING;
	                    gMotorDrive.drive(-30,30);

                	}
                	else
                	{
                		//左を向いていた時の処理．
                		mCurStep = STEP_TURNING;
	                    gMotorDrive.drive(30,-30);
                	}
                }
                
                mIsLastActionStraight = false;
			}
            
			//2014/06/13移動
			mLastUpdateTime = time;
		}
		break;
	case STEP_TURNING:
		if(Time::dt(time,mLastUpdateTime) > 0.5){//0.5
            gMotorDrive.drive(0,0);
			mCurStep = STEP_STARTING;
		}
		break;
	case STEP_STOPPING_FAST:
		if(Time::dt(time,mLastUpdateTime) > 0.5){//0.5
			gMotorDrive.drive(0,0);
			mCurStep = STEP_STARTING;
		}
		break;
	case STEP_STOPPING_LONG:
		if(Time::dt(time,mLastUpdateTime) > 0.8){//1.5
			gMotorDrive.drive(0,0);
			mCurStep = STEP_STARTING;
		}
		break;
	}
}
bool ColorAccessing::onCommand(const std::vector<std::string> args)
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
ColorAccessing::ColorAccessing() : mIsAvoidingEnable(false),mCurStep(STEP_STARTING)
{
	setName("detecting");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
ColorAccessing::~ColorAccessing()
{
}
/* ここまで　2014年6月オープンラボ前に実装 */

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
		//バックを行う
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
		//再起動防止のため待機
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			if(mEscapingTriedCount > ESCAPING_MAX_CAMERA_ESCAPING_COUNT)
			{
				//ランダム移行
				Debug::print(LOG_SUMMARY, "Escaping: aborting camera escape!\r\n");
				mEscapingTriedCount = 0;
				mCurStep = STEP_RANDOM;
				mCurRandomStep = RANDOM_STEP_FORWARD;
				break;
			}
			mCurStep = STEP_PRE_CAMERA;
			mLastUpdateTime = time;
			//起き上がり動作を行う
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			if(gImageProc.isSky(pImage))gWakingState.setRunMode(true);
		}
		break;
	case STEP_PRE_CAMERA:
		//画像撮影用に起き上がり動作を行い、数秒待機する
		if(gWakingState.isActive())mLastUpdateTime = time;//起き上がり動作中は待機する
		if(Time::dt(time,mLastUpdateTime) > 2)//起き上がり完了後、一定時間が経過していたら
		{
			Debug::print(LOG_SUMMARY, "Escaping: camera warming...\r\n");
			//画像撮影動作を行う
			mCurStep = STEP_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}
		break;
	case STEP_CAMERA:
		//画像処理を行い、今後の行動を決定する
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
		//画像処理の結果、回転する必要があった場合
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_CAMERA_FORWARD;
			gMotorDrive.startPID(0,100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_FORWARD:
		//画像処理の結果、直進する必要があった場合
		if(Time::dt(time,mLastUpdateTime) >= 10)
		{
			gMotorDrive.drive(-100,-100);
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
		}
		break;
	case STEP_CAMERA_TURN_HERE:
		//画像処理の結果、その場回転する必要があった場合
		if(Time::dt(time,mLastUpdateTime) > 0.4 || abs(gGyroSensor.getRz() - mAngle) > 70)
		{
			gCameraCapture.startWarming();
			mCurStep = STEP_BACKWARD;
			gMotorDrive.drive(-100,-100);
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		//ランダム動作
		if(Time::dt(time,mLastUpdateTime) >= 5)
		{
			++mEscapingTriedCount;
			if(mEscapingTriedCount > ESCAPING_MAX_RANDOM_ESCAPING_COUNT)
			{
				//ランダム移行
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
		//バックを行う
		Debug::print(LOG_SUMMARY, "Escaping(random): backward\r\n");
		mCurRandomStep = RANDOM_STEP_TURN;
		gMotorDrive.drive(100,-100);
		break;
	case RANDOM_STEP_TURN:
		//その場回転を行う
		Debug::print(LOG_SUMMARY, "Escaping(random): turning\r\n");
		mCurRandomStep = RANDOM_STEP_FORWARD;
		gMotorDrive.drive(100,100);
		break;
	case RANDOM_STEP_FORWARD:
		//前進を行う
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
		default://カメラ使えなかった
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
bool EscapingRandom::onInit(const struct timespec& time)
{
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
		//バックを行う
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_TURN;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,-100);
		}
		break;
	case STEP_TURN:
		//その場回転を行う
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_FORWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(100,100);
		}
		break;
	case STEP_FORWARD:
		//前進を行う
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_BACKWARD;
			mLastUpdateTime = time;
			gMotorDrive.drive(-100,-100);
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
	switch(mCurStep)//起き上がり開始が検知された場合
	{
	case STEP_STOP:
		if(Time::dt(time,mLastUpdateTime) > 2)//2秒まわしても着地が検知されない場合はあきらめる
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to land\r\n");
			setRunMode(false);
		}
		if(abs(gGyroSensor.getRvx()) < WAKING_THRESHOLD)//角速度が一定以下になったら着地と判定
		{
			Debug::print(LOG_SUMMARY, "Waking Landed!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}

		//回転した角度に応じてモータの出力を変化させる
		power = std::min(0,std::max(100,MOTOR_MAX_POWER - abs(gGyroSensor.getRvx() - mAngleOnBegin) / 130 + 50));
		gMotorDrive.drive(power,power);
		break;

	case STEP_START:
		if(Time::dt(time,mLastUpdateTime) > 0.5)//一定時間回転が検知されない場合→回転不可能と判断
		{
			Debug::print(LOG_SUMMARY, "Waking Timeout : unable to spin\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_VERIFY;
			gMotorDrive.drive(0,0);
			gCameraCapture.startWarming();
		}
		if(abs(gGyroSensor.getRvx()) > WAKING_THRESHOLD)//回転が検知された場合→起き上がり開始したと判断
		{
			Debug::print(LOG_SUMMARY, "Waking Detected Rotation!\r\n");
			mLastUpdateTime = time;
			mCurStep = STEP_STOP;
		}
		break;

	case STEP_VERIFY:
		//起き上がりが成功したか否かをカメラ画像で検証
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			IplImage* pCaptureFrame = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pCaptureFrame);
			if(gImageProc.isSky(pCaptureFrame))
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
			}else
			{
				Debug::print(LOG_SUMMARY, "Waking Successed!\r\n");
				setRunMode(false);
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

		//ログを保存
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
	setName("logging");
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
