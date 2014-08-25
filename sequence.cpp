#include <stdlib.h>
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
#include "subsidiary_sequence.h"

Testing gTestingState;
Waiting gWaitingState;
Falling gFallingState;
Separating gSeparatingState;
Navigating gNavigatingState;
ColorAccessing gColorAccessingState;

bool Testing::onInit(const struct timespec& time)
{
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gParaServo.setRunMode(true);
	gStabiServo.setRunMode(true);
	gXbeeSleep.setRunMode(true);

	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gAccelerationSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	gWebCamera.setRunMode(true);
	//gDistanceSensor.setRunMode(true);
	gCameraCapture.setRunMode(true);

	gMotorDrive.setRunMode(true);

	gSerialCommand.setRunMode(true);

	gStabiServo.stop();

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
		}
		else if(args[1].compare("waking") == 0)
		{
			gWakingState.setRunMode(true);
		}
		else if(args[1].compare("time") == 0)
		{
			Time::showNowTime();
			return true;
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
testing time                      : show current time\r\n\
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
	if(gLightSensor.get())
	{
		++mContinuousLightCount;
		gBuzzer.start(10);
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
	Debug::print(LOG_SUMMARY, "Falling... ");
	Time::showNowTime();
	
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
	gParaServo.setRunMode(true);
	gStabiServo.setRunMode(true);
	gParaServo.moveHold();
	gStabiServo.start(STABI_FOLD_ANGLE);		//スタビを格納状態で固定

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
	Debug::print(LOG_SUMMARY, "Separating... ");
	Time::showNowTime();
	
	//必要なタスクを使用できるようにする
	TaskManager::getInstance()->setRunMode(false);
	setRunMode(true);
	gBuzzer.setRunMode(true);
	gParaServo.setRunMode(true);
	gStabiServo.setRunMode(true);
	gSerialCommand.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gCameraCapture.setRunMode(true);
	gSensorLoggingState.setRunMode(true);

	mLastUpdateTime = time;
	gParaServo.moveHold();
	gStabiServo.start(STABI_BASE_ANGLE);		//スタビを走行時の位置に移動
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

		if(mCurServoState)
		{
			gParaServo.moveRelease();
		}
		else
		{
			gParaServo.moveHold();
		}
		
		++mServoCount;
		Debug::print(LOG_SUMMARY, "Separating...(%d/%d)\r\n", mServoCount, SEPARATING_SERVO_COUNT);

		if(mServoCount >= SEPARATING_SERVO_COUNT)//サーボを規定回数動かした
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
				gBuzzer.start(20, 20, 5);
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
			gMotorDrive.drive(100,100);
			mLastUpdateTime = time;
			mCurStep = STEP_GO_FORWARD;
		}
		break;
	case STEP_GO_FORWARD:	//パラ検知後，しばらく直進する
		if(Time::dt(time,mLastUpdateTime) > 3)
		{
			gMotorDrive.drive(0,0);
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
	gStabiServo.setRunMode(true);
	
	gStabiServo.start(STABI_BASE_ANGLE);		//スタビを走行時の位置に移動

	mLastNaviMoveCheckTime = time;
	mLastPos.clear();
	mPrevDeltaPulseL = 0;
	mPrevDeltaPulseR = 0;
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
			Debug::print(LOG_SUMMARY, "Starting navigation...");
			Time::showNowTime();//制御開始時刻をログに出力
			Debug::print(LOG_SUMMARY, "Control Start Point:(%f %f)\r\n",currentPos.x,currentPos.y);
			gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
			gPredictingState.setRunMode(true);
			mLastNaviMoveCheckTime = time;
		}
		mLastPos.push_back(currentPos);
	}	

	//ゴールとの距離を確認
	double distance = VECTOR3::calcDistanceXY(currentPos,mGoalPos);
	if(distance < NAVIGATING_GOAL_DISTANCE_THRESHOLD)
	{
		//ゴール判定
		gMotorDrive.drive(0,0);
		Debug::print(LOG_SUMMARY, "Navigating Finished!\r\n");
		Debug::print(LOG_SUMMARY, "Navigating Finish Point:(%f %f)\r\n",currentPos.x,currentPos.y);
		nextState();
		return;
	}

	//エンコーダの値によるスタック判定処理
	if(Time::dt(time,mLastEncoderCheckTime) > 1 && (distance > NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD))
	{
		chechStuckByEncoder(time, currentPos);
	}

	//数秒たっていなければ処理を返す
	if(Time::dt(time,mLastNaviMoveCheckTime) < NAVIGATING_DIRECTION_UPDATE_INTERVAL)return;
	mLastNaviMoveCheckTime = time;

	//異常値排除
	if(removeError())
	{
		Debug::print(LOG_SUMMARY, "NAVIGATING: GPS Error value detected\r\n");
	}

	if(gPredictingState.isWorking(time))
	{
		//轍回避中
	}
	else if(isStuckByGPS())//スタック判定
	{
		if(!gEscapingRandomState.isActive())
		{
			gEscapingByStabiState.setRunMode(true);
		}
		Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK detected by GPS at (%f %f)\r\n",currentPos.x,currentPos.y);
		gBuzzer.start(20, 20, 8);

		if(gEscapingByStabiState.isActive())		//EscapingByStabi中
		{
			if(gEscapingByStabiState.getTryCount() >= ESCAPING_BY_STABI_COUNT_THRESHOLD)
			{
				//EscapingRandomに移行
				gEscapingByStabiState.setRunMode(false);
				gStabiServo.start(STABI_BASE_ANGLE);		//スタビを通常の状態に戻す
				Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping Random Start! \r\n");
				gEscapingRandomState.setRunMode(true);
				mEscapingRandomStartTime = time;
			}
		}
		else if(gEscapingRandomState.isActive())	//EscapingRandom中
		{
			if(Time::dt(time,mEscapingRandomStartTime) > ESCAPING_RANDOM_TIME_THRESHOLD)
			{
				//EscapingByStabiに移行
				gEscapingRandomState.setRunMode(false);
				Debug::print(LOG_SUMMARY, "NAVIGATING: Escaping ByStabi Start! \r\n");
				gEscapingByStabiState.setRunMode(true);
			}
		}
	}
	else
	{
		if(gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())//脱出モードが完了した時
		{
			//ローバーがひっくり返っている可能性があるため、しばらく前進する
			gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
			gEscapingByStabiState.setRunMode(false);
			gEscapingRandomState.setRunMode(false);
			gStabiServo.start(STABI_BASE_ANGLE);		//スタビを通常の状態に戻す
			Debug::print(LOG_SUMMARY, "NAVIGATING: Navigating restart! \r\n");
			gBuzzer.start(80, 20, 3);
		}
		else
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
bool Navigating::isStuckByGPS() const
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
void Navigating::chechStuckByEncoder(const struct timespec& time, VECTOR3 currentPos)
{
	mLastEncoderCheckTime = time;

	//エンコーダパルスの差分値の取得
	unsigned long long deltaPulseL = gMotorDrive.getDeltaPulseL();
	unsigned long long deltaPulseR = gMotorDrive.getDeltaPulseR();

	if(gEscapingByStabiState.isActive() || gEscapingRandomState.isActive())//既にスタック判定中である
	{
		mPrevDeltaPulseL = 0;//リセット
		mPrevDeltaPulseR = 0;
		return;
	}

	//Debug::print(LOG_SUMMARY, "NAVIGATING: Encoder Pulse(LEFT RIGHT)= (%llu %llu)\r\n", deltaPulseL, deltaPulseR);//パルスの出力

	//前回が閾値以上で、今回が閾値以下ならスタック判定する
	if(mPrevDeltaPulseL >= STUCK_ENCODER_PULSE_THRESHOLD && mPrevDeltaPulseR >= STUCK_ENCODER_PULSE_THRESHOLD)	//前回のパルス数が閾値以上
	{
		if(deltaPulseL < STUCK_ENCODER_PULSE_THRESHOLD && deltaPulseR < STUCK_ENCODER_PULSE_THRESHOLD)			//今回のパルス数が閾値以下
		{
			//スタック判定
			gBuzzer.start(200, 50 ,3);
			Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK detected by pulse count(%llu %llu) at (%f %f)\r\n",deltaPulseL,deltaPulseR,currentPos.x,currentPos.y);
			gEscapingByStabiState.setRunMode(true);
		}
	}
	mPrevDeltaPulseL = deltaPulseL;
	mPrevDeltaPulseR = deltaPulseR;
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
		else if(args[1].compare("goal") == 0)
		{
			nextState();
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
navigating here            : set goal at current position\r\n\
navigating goal            : call nextState\r\n");
	return true;
}

//次の状態に移行
void Navigating::nextState()
{
	if(gColorAccessingState.getIsDetectingExecute())
	{
		gBuzzer.start(100, 50, 3);

		//次の状態を設定
		gColorAccessingState.setRunMode(true);
	}
	else
	{
		gBuzzer.start(1000);

		//次の状態を設定
		gTestingState.setRunMode(true);
		gPictureTakingState.setRunMode(true);
	
		gMotorDrive.drive(0,0);//念のため2回
		gMotorDrive.drive(0,0);
	
		Time::showNowTime();
		Debug::print(LOG_SUMMARY, "Goal!\r\n");
	}
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

/* ここから　2014年実装 */
bool ColorAccessing::onInit(const struct timespec& time)
{
	Debug::print(LOG_SUMMARY, "Start Goal Detecting... ");
	Time::showNowTime();
	
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
	gAccelerationSensor.setRunMode(true);
	gStabiServo.setRunMode(true);

	gStabiServo.start(STABI_BASE_ANGLE);		//スタビを走行時の位置に移動
	mCurStep = STEP_STARTING;
	mStartTime = time;		//開始時刻を保存
	mLastUpdateTime = time;
	gCameraCapture.startWarming();
    mIsLastActionStraight = false;
    mTryCount = 0;
	mIsGPS = false;
	return true;
}
void ColorAccessing::onUpdate(const struct timespec& time)
{
	if(gAvoidingState.isActive())return;

	// Debug::print(LOG_SUMMARY, "accel = %f\r\n",gAccelerationSensor.getAz());

	if ( gAccelerationSensor.getAz() < -0.3 && !gWakingState.isActive() )
	{
		Debug::print(LOG_SUMMARY, "accel = %f\r\n",gAccelerationSensor.getAz());
		mLastUpdateTime = time;
		// mCurStep = STEP_PRE_PARA_JUDGE;
		gWakingState.setRunMode(true);
	}

	// if(gWakingState.isActive())
	// {
	// 	mLastUpdateTime = time;//起き上がり動作中は待機する
	// 	return;
	// }
	double dt;
	
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
			
			//新しい位置を取得できていれば座標を表示する
			if(gGPSSensor.get(mCurrentPos,false))
			{
				mIsGPS = true;	//一度でもGPS座標取得に成功したらtrueに
				Debug::print(LOG_SUMMARY, "Current Position:(%f %f)\r\n",mCurrentPos.x,mCurrentPos.y);
			}
			
			IplImage* pImage = gCameraCapture.getFrame();
			gCameraCapture.save(NULL,pImage);
			
			if(pImage == NULL)//カメラが死んでるのでその場でゴール判定する
			{
				Debug::print(LOG_SUMMARY, "Detecting: Camera is not working...\r\n");
				nextState();
				return;
			}
			
			int x_pos = gImageProc.howColorGap(pImage);
			
            if( x_pos != INT_MAX )	//色検知したら
			{
				mLastUpdateTime = time;
				if ( x_pos == INT_MIN )	//ゴール判定時
				{
					//新しい位置を取得できていれば座標を表示する
					if(mIsGPS) Debug::print(LOG_SUMMARY, "Control Finish Point:(%f %f)\r\n",mCurrentPos.x,mCurrentPos.y);//制御終了位置の座標を表示
					nextState();
				}
				else if ( x_pos < -40 )
				{
					mCurStep = STEP_STOPPING_FAST;
					gMotorDrive.drive(0,20);
                    mIsLastActionStraight = false;
				}
				else if ( 40 < x_pos )
				{
					mCurStep = STEP_STOPPING_FAST;
					gMotorDrive.drive(20,0);
                    mIsLastActionStraight = false;
				}
				else if ( -40 <= x_pos && x_pos <= 40 )
				{
					mCurStep = STEP_STOPPING_LONG;
					gMotorDrive.drive(20,20);
                    mIsLastActionStraight = true;
                    mAngleOnBegin = gGyroSensor.getRz();
				}
				mTryCount = 0;
			}
			else//色検知しなかったら
			{
                if (mIsLastActionStraight)	//前回の行動が直進なら．
                {
                	double diff = GyroSensor::normalize(gGyroSensor.getRz() - mAngleOnBegin);

                	Debug::print(LOG_SUMMARY, "diff = %f\r\n", diff);

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

                	if ( mTryCount > 2 )
                	{
                		//とりあえず右に曲がるか．
                		mCurStep = STEP_TURNING;
	                    gMotorDrive.drive(30,-30);
                	}
                	else if ( diff < 0 )
                	{
                		//右を向いていた時の処理．
                		mCurStep = STEP_TURNING;
	                    gMotorDrive.drive(-30,30);
	                    mTryCount++;
                	}
                	else if ( diff >= 0 )
                	{
                		//左を向いていた時の処理．
                		mCurStep = STEP_TURNING;
	                    gMotorDrive.drive(30,-30);
	                    mTryCount++;
                	}
                }
                
                mIsLastActionStraight = false;
			}
			mLastUpdateTime = time;
		}
		break;
	case STEP_TURNING:
		if(Time::dt(time,mLastUpdateTime) > 0.5){//0.5
			gMotorDrive.drive(0, 0);
			mCurStep = STEP_STARTING;
		}
		break;
	case STEP_STOPPING_FAST:
		if(Time::dt(time,mLastUpdateTime) > 0.5){//0.5
			gMotorDrive.drive(0, 0);
			mCurStep = STEP_STARTING;
		}
		break;
	case STEP_STOPPING_LONG:
		if(Time::dt(time,mLastUpdateTime) > 0.8){//1.5
			mCurStep = STEP_DEACCELERATE;
			mLastUpdateTime = time;
		}
		break;
	case STEP_DEACCELERATE:	//ゆっくり減速する
		dt = Time::dt(time, mLastUpdateTime);
        if(dt > DEACCELERATE_DURATION)
        {
            mLastUpdateTime = time;
            mCurStep = STEP_STARTING;
                
            gMotorDrive.drive(0, 0);
        }
		else
		{
			int tmp_power = std::max((int)((1 - dt / DEACCELERATE_DURATION) * (20 / 2/*2で割る*/)), 0);	//ToDo: 20を変数に置き換える
			gMotorDrive.drive(tmp_power, tmp_power);
		}
		break;
	case STEP_RESTART:
		if(Time::dt(time,mLastUpdateTime) > 10)//しばらく直進する
		{
			prevState();
		}
		break;
	}

	//ColorAccessingを開始してからの経過時間を確認
	if(mCurStep != STEP_RESTART)
	{
		timeCheck(time);	
	}
}
bool ColorAccessing::onCommand(const std::vector<std::string> args)
{
	if(args.size() == 2)
	{
		if(args[1].compare("setmode") == 0)
		{
			if(mIsDetectingExecute) Debug::print(LOG_SUMMARY, "Detecting mode: ON\r\n");
			else Debug::print(LOG_SUMMARY, "Detecting mode: OFF\r\n");
			return true;
		}
	}
	else if(args.size() == 3)
	{
		if(args[1].compare("setmode") == 0)
		{
			if(args[2].compare("ON") == 0)
			{
				setIsDetectingExecute(true);
				return true;
			}
			else if(args[2].compare("OFF") == 0)
			{
				setIsDetectingExecute(false);
				return true;
			}
		}
	}
	Debug::print(LOG_PRINT, "detecting setmode [ON/OFF]: set detecting mode\r\n\
detecting setmode         : show detecting mode state\r\n");
	return true;
}
//次の状態に移行
void ColorAccessing::nextState()
{
	gBuzzer.start(1000);

	//次の状態を設定
	gTestingState.setRunMode(true);
	gPictureTakingState.setRunMode(true);
	
	gMotorDrive.drive(0,0);//念のため2回
	gMotorDrive.drive(0,0);
	
	Debug::print(LOG_SUMMARY, "Detecting Finish! ");
	Time::showNowTime();
	Debug::print(LOG_SUMMARY, "Goal!\r\n");
}
//前の状態に移行
void ColorAccessing::prevState()
{
	gBuzzer.start(30,10,8);

	//前の状態に戻る
	gColorAccessingState.setRunMode(false);
	gNavigatingState.setRunMode(true);
	
	Debug::print(LOG_SUMMARY, "Navigating Restart!\r\n");
}
void ColorAccessing::timeCheck(const struct timespec& time)
{
	if(Time::dt(time,mStartTime) > COLOR_ACCESSING_ABORT_TIME)//一定時間が経過したらNavigatingからやり直し
	{
		Debug::print(LOG_SUMMARY, "ColorAccessing Timeout!\r\n");
		mCurStep = STEP_RESTART;
		gMotorDrive.drive(100,100);
		mLastUpdateTime = time;
		gBuzzer.start(30,10,8);
	}
}
void ColorAccessing::setIsDetectingExecute(bool flag)
{
	if(flag)
	{
		if(mIsDetectingExecute)
		{
			Debug::print(LOG_SUMMARY, "Detecting has already set \"ON\"\r\n");
			return;
		}
		mIsDetectingExecute = flag;
		Debug::print(LOG_SUMMARY, "Detecting has set \"ON\"\r\n");
	}
	else
	{
		if(!mIsDetectingExecute)
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
ColorAccessing::ColorAccessing():mIsDetectingExecute(true)
{
	setName("detecting");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
ColorAccessing::~ColorAccessing()
{
}
/* ここまで　2014年実装 */
