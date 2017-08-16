#pragma once
#include <time.h>
#include <tuple>
#include "task.h"
#include "utils.h"

//calc pose of rover
//X: to Front
//Y: to Left
//Z: to Top
class PoseDetecting : public TaskBase
{
private:
	QUATERNION mEstimatedAngle, mEstimatedAngleWithLPF;
	std::tuple<KalmanFilter, KalmanFilter, KalmanFilter> mKalmanGyro;
	double mEstimatedRelativeGpsCourse, mEstimatedVelocity;
	long long mLastEncL, mLastEncR;
	bool mIsInitializedAngle;
	double mAccelCoeff, mEncCoeff, mGpsCoeff;
	double mAngleLPFCoeff;
	double mAccelUsableRange;
	double mFlipThreshold, mLieThreshold;
	//2017 
	double mFlipAngle;
	double mLieAngle;

	VECTOR3 mLastGpsPos;
	int mLastGpsSampleTime;

	struct timespec mLastUpdatedTime;
	int mRoverid;

protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);

	//慣性制御情報を更新
	void updateUsingIMU(double dt, double gx, double gy, double gz, double ax, double ay, double az);
public:
	//following functions returns degrees
	//オイラー角を計算
	VECTOR3 getEulerZYX() const;
	VECTOR3 getEulerZYXLPF() const;
	VECTOR3 getEulerXYZ() const;
	VECTOR3 getEulerXYZLPF() const;

	//Roll(横回転方向)を取得
	double getRoll() const;
	//Pitch(前転後転)を取得 前方向が正
	double getPitch() const;
	//Yaw(方角)を取得 反時計回り
	//absoluteをtrueにすると今向いている方位(GPS基準)を返す
	double getYaw(bool absolute = false, bool flipfix = true) const;
	double getYawLPF(bool absolute = true/*8-24 chou trueにした*/, bool flipfix = true) const;

	//Velocity using encoder
	double getVelocity() const;

	//ひっくり返ったことを検知
	bool isFlip() ;
	double getFlipAngle()const;
	double getmFlipThreshold()const;


	bool isFlipCoord() const;
	//横転を検知
	bool isLie();
	double getLieAngle()const;
	double getLieThreshold()const;


	bool isIllegalAccel(const VECTOR3& accel) const;

	//エンコーダの値から方向転換量を取得
	static double calcEncAngle(long long left, long long right);

	PoseDetecting();
	~PoseDetecting();
};

extern PoseDetecting gPoseDetecting;
