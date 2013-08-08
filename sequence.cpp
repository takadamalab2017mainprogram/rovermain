#include<stdlib.h>
#include <math.h>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
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
Escaping gEscapingState;

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
	gDistanceSensor.setRunMode(true);
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
				pTask->setRunMode(true);
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
	gCameraCapture.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGyroSensor.setRunMode(true);

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

		if(mServoCount >= SEPARATING_SERVO_COUNT)
		{
			mCurStep = STEP_PRE_PARA_JUDGE;
			gMotorDrive.drive(50,50);
		}
		break;
	case STEP_PRE_PARA_JUDGE:
		if(Time::dt(time,mLastUpdateTime) < ROVER_WAKING_DRIVE_TIME)return;
		gMotorDrive.drive(0,0);
		mCurStep = STEP_PARA_JUDGE;
		break;
	case STEP_PARA_JUDGE:
		//ローバーを起こし終わったら，パラシュート検知を行い，存在する場合は回避行動に遷移する
		if(Time::dt(time,mLastUpdateTime) < 2)return;
		{
			IplImage* pImage = gCameraCapture.getFrame();
			if(isParaExist(pImage))
			{
				mCurStep = STEP_PARA_DODGE;
				gGyroSensor.setZero();
				gMotorDrive.drive(50,0);
				Debug::print(LOG_SUMMARY, "Para check: Found!!\r\n");
			}else
			{
				Debug::print(LOG_SUMMARY, "Para check: Not Found!!\r\n");
				nextState();
			}
			gCameraCapture.save(NULL, pImage);
		}
		break;
	case STEP_PARA_DODGE:
		if(abs(gGyroSensor.getRz()) >= 90)
		{
			Debug::print(LOG_SUMMARY, "Para check: Turn Finished!\r\n");
			gMotorDrive.drive(0,0);
			nextState();
		}
	};
}
bool Separating::isParaExist(IplImage* src)
{
	if(src == NULL)
	{
		Debug::print(LOG_SUMMARY, "Para detection: Unable to get Image\r\n");
		return true;
	}
	unsigned long pixelCount = 0;
	int x = 0, y = 0;
	uchar H, S, V;
	uchar minH, minS, minV, maxH, maxS, maxV;
    
	CvPixelPosition8u pos_src;
	uchar* p_src;
	IplImage* tmp = cvCreateImage(cvGetSize(src), IPL_DEPTH_8U, 3);
    
	//HSVに変換
	cvCvtColor(src, tmp, CV_BGR2HSV);
    
	CV_INIT_PIXEL_POS(pos_src, (unsigned char*) tmp->imageData,
                      tmp->widthStep,cvGetSize(tmp), x, y, tmp->origin);
    
	minH = 0;	maxH = 25;
	minS = 128;	maxS = 255;
	minV = 128;	maxV = 255;
	for(y = 0; y < tmp->height; y++) {
		for(x = 0; x < tmp->width; x++) {
			p_src = CV_MOVE_TO(pos_src, x, y, 3);
            
			H = p_src[0];
			S = p_src[1];
			V = p_src[2];
            
			if( minH <= H && H <= maxH &&
               minS <= S && S <= maxS &&
               minV <= V && V <= maxV
               ) {
				++pixelCount;
			}
		}
	}
	double ratio = (double)pixelCount / tmp->height / tmp->width;
	Debug::print(LOG_SUMMARY, "Para ratio: %f\r\n",ratio);
	return ratio > SEPARATING_PARA_DETECT_THRESHOLD;
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
	gCameraCapture.setRunMode(true);

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
	if(!gGPSSensor.get(currentPos))return;

	//最初の座標を取得したら移動を開始する
	if(mLastPos.empty())
	{
		Debug::print(LOG_SUMMARY, "Starting navigation...\r\n");
		gMotorDrive.startPID(0 ,MOTOR_MAX_POWER);
		mLastCheckTime = time;
	}

	//新しい座標であればバッファに追加
	if(isNewData && finite(currentPos.x) && finite(currentPos.y) && finite(currentPos.z))
	{
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

	IplImage* pImage = gCameraCapture.getFrame();
	gCameraCapture.save(NULL,pImage);

	//スタック判定
	if(isStuck())
	{
		Debug::print(LOG_SUMMARY, "NAVIGATING: STUCK detected at (%f %f)\r\n",currentPos.x,currentPos.y);
		gBuzzer.start(10);

		gEscapingState.setRunMode(true);
	}else
	{
		if(gEscapingState.isActive())
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
bool Navigating::isStuck() const
{
	double averageVel = 0;
	VECTOR3 lastPos = mLastPos.front();
	std::list<VECTOR3>::const_iterator it = mLastPos.begin();
	while(it != mLastPos.end())
	{
		averageVel += VECTOR3::calcDistanceXY(*it,lastPos);
		lastPos = *it;
		++it;
	}

	return averageVel < NAVIGATING_STUCK_JUDGEMENT_THRESHOLD;
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

bool Escaping::onInit(const struct timespec& time)
{
	mLastUpdateTime = time;
	mCurStep = STEP_BACKWORD;
	return true;
}
void Escaping::onUpdate(const struct timespec& time)
{
	switch(mCurStep)
	{
	case STEP_BACKWORD:
		gMotorDrive.drive(-100,-100);
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			mCurStep = STEP_PRE_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(50,50);
		}
		break;
	case STEP_PRE_CAMERA:
		if(Time::dt(time,mLastUpdateTime) >= ROVER_WAKING_DRIVE_TIME)
		{
			mCurStep = STEP_CAMERA;
			mLastUpdateTime = time;
			gMotorDrive.drive(0,0);
		}
		break;
	case STEP_CAMERA:
		if(Time::dt(time,mLastUpdateTime) >= 1)
		{
			mCurStep = STEP_CAMERA_WAIT;
			mLastUpdateTime = time;
			mWaitTime = stuckMoveCamera(gCameraCapture.getFrame());
		}
		break;
	case STEP_CAMERA_WAIT:
		if(Time::dt(time,mLastUpdateTime) >= mWaitTime)
		{
			mCurStep = STEP_RANDOM;
			mLastUpdateTime = time;
		}
		break;
	case STEP_RANDOM:
		if(Time::dt(time,mLastUpdateTime) >= 3)
		{
			stuckMoveRandom();
			mLastUpdateTime = time;
		}
		break;
	}
}
void Escaping::stuckMoveRandom()
{
	//進行方向をランダムで変更
	gMotorDrive.drive(100 * (rand() % 2 ? 1 : -1), 100 * (rand() % 2 ? 1 : -1));
}
double Escaping::stuckMoveCamera(IplImage* pImage)
{
	IplImage* src_img = pImage;
	const static int DIV_NUM = 3;
	IplImage *gray_img, *dst_img1, *tmp_img;
	double risk[DIV_NUM];

	if(src_img == NULL)
	{
		Debug::print(LOG_SUMMARY, "Escaping: Unable to get Image for Camera Escaping!\r\n");
		mCurStep = STEP_RANDOM;
		return 0;
	}
	CvSize size = cvSize(src_img->width,src_img->height);

	gray_img = cvCreateImage(size, IPL_DEPTH_8U, 1);
	cvCvtColor(src_img, gray_img, CV_BGR2GRAY);
	cvRectangle(gray_img, cvPoint(0, 0),cvPoint(src_img->width, src_img->height / 3),cvScalar(0), CV_FILLED, CV_AA);

	// Medianフィルタ
	cvSmooth (gray_img, gray_img, CV_MEDIAN, 5, 0, 0, 0);
		
	tmp_img = cvCreateImage(size, IPL_DEPTH_16S, 1);
	dst_img1 = cvCreateImage(size, IPL_DEPTH_8U, 1);
		
	// SobelフィルタX方向
	cvSobel(gray_img, tmp_img, 1, 0, 3);
	cvConvertScaleAbs (tmp_img, dst_img1);
	cvThreshold (dst_img1, dst_img1, 50, 255, CV_THRESH_BINARY);

	//Sum
	int width = src_img->width / DIV_NUM;
	double risksum = 0;
	int i;
	for(i = 0;i < DIV_NUM;++i)
	{
		cvSetImageROI(dst_img1, cvRect(width * i,0,width,src_img->height));//Set image part
		risksum += risk[i] = sum(cv::cvarrToMat(dst_img1))[0];
		cvResetImageROI(dst_img1);//Reset image part (normal)
	}

	//Draw graph
	for(i = 0;i < DIV_NUM;++i){
		cvRectangle(dst_img1, cvPoint(width * i,src_img->height - risk[i] / risksum * src_img->height),cvPoint(width * (i + 1),src_img->height),cvScalar(255), 2, CV_AA);
	}

	int min_id = 0;
	for(int i=1; i<3; ++i){
		if(risk[min_id] > risk[i]){
			min_id = i;
		}
	}

	double waitTime = 0;
	switch(min_id){
		case 0:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Left\r\n");
			gMotorDrive.drive(60, 100);
			waitTime = 1;
			break;
		case 1:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Go Straight\r\n");
			gMotorDrive.drive(100, 100);
			waitTime = 5;
			break;
		case 2:
			Debug::print(LOG_SUMMARY, "Wadachi kaihi:Turn Right\r\n");
			gMotorDrive.drive(100, 60);
			waitTime = 1;
			break;
		default:
			break;
	}
	cvReleaseImage (&dst_img1);
	cvReleaseImage (&tmp_img);

	return waitTime;
}
Escaping::Escaping()
{
	setName("escaping");
	setPriority(TASK_PRIORITY_SEQUENCE,TASK_INTERVAL_SEQUENCE);
}
Escaping::~Escaping()
{
}
