#include "pose_detector.h"
#include "sensor.h"
#include "motor.h"

PoseDetecting gPoseDetecting;

bool PoseDetecting::onInit(const struct timespec& time)
{
	gAccelerationSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gMotorDrive.setRunMode(true);
	gGPSSensor.setRunMode(true);

	Time::get(mLastUpdatedTime);
	mLastEncL = gMotorDrive.getL();
	mLastEncR = gMotorDrive.getR();
	mLastGpsSampleTime = 0;

	return true;
}
void PoseDetecting::onUpdate(const struct timespec& time)
{
	//calc dt
	struct timespec newTime;
	Time::get(newTime);
	double dt = Time::dt(newTime, mLastUpdatedTime);
	mLastUpdatedTime = newTime;

	//get gyro and accel using kalman-filter
	VECTOR3 accelRaw;
	gAccelerationSensor.getAccel(accelRaw);
	VECTOR3 accel(-accel.x, -accel.y, accel.z);
	VECTOR3 accelAngle(
			atan2f(accel.y, sqrt(accel.x*accel.x + accel.z*accel.z)),
			atan2f(accel.x, sqrt(accel.y*accel.y + accel.z*accel.z)),
			atan2f(sqrt(accel.x*accel.x + accel.y*accel.y), accel.z)
			);

	//VECTOR3 gyroRaw(-gGyroSensor.getRvy() / 180 * M_PI, gGyroSensor.getRvx() / 180 * M_PI, gGyroSensor.getRvz() / 180 * M_PI);
	/*VECTOR3 gyro(
			std::get<0>(mKalmanGyro).update(accelAngle.x, gyroRaw.x, dt),
			std::get<1>(mKalmanGyro).update(accelAngle.y, gyroRaw.y, dt),
			std::get<2>(mKalmanGyro).update(accelAngle.z, gyroRaw.z, dt)
			);*/
	VECTOR3 gyro(-gGyroSensor.getRvy() / 180 * M_PI, gGyroSensor.getRvx() / 180 * M_PI, gGyroSensor.getRvz() / 180 * M_PI);

	//update IMU
	updateUsingIMU(dt, gyro.x, gyro.y, gyro.z, accel.x, accel.y, accel.z);
	mEstimatedAngleWithLPF = mEstimatedAngleWithLPF * (1 - mAngleLPFCoeff) + mEstimatedAngle * mAngleLPFCoeff;

	//ジャイロ値取得
	if(gMotorDrive.isActive())
	{
		if(dt > 0)
		{
			//update course by encoder
			long long newEncL = gMotorDrive.getL(), newEncR = gMotorDrive.getR();
			long long deltaEncL = newEncL - mLastEncL, deltaEncR = newEncR - mLastEncR;
			mLastEncL = newEncL;
			mLastEncR = newEncR;

			//update velocity by encoder
			double distance = 0.5 * (deltaEncL + deltaEncR) / static_cast<double>(RESOLVING_POWER) / GEAR_RATIO * DISTANCE_PER_ROTATION;
			double velocity = distance / dt;
			mEstimatedVelocity = mEstimatedVelocity * (1 - mEncCoeff) + velocity * mEncCoeff;
		}
	}else
	{
		mEstimatedVelocity = 0;
	}

	//GPS方角と内部方角の差分を更新
	VECTOR3 gpsPos;
	if(gGPSSensor.isActive() && gGPSSensor.get(gpsPos, true) && gGPSSensor.getSpeed() > 0.1 && gGPSSensor.getTime() != mLastGpsSampleTime && !isFlip())
	{
		if(mLastGpsSampleTime != 0)
		{
			double gpsCourse = -VECTOR3::calcAngleXY(mLastGpsPos, gpsPos);
			double inertialCourse = getYawLPF(true);
			double relativeCourse = GyroSensor::normalize(gpsCourse - inertialCourse);

			mEstimatedRelativeGpsCourse += relativeCourse * mGpsCoeff;
			mEstimatedRelativeGpsCourse = GyroSensor::normalize(mEstimatedRelativeGpsCourse);

			mEstimatedVelocity = mEstimatedVelocity * (1 - mGpsCoeff) + gGPSSensor.getSpeed() * mGpsCoeff;
		}
		mLastGpsSampleTime = gGPSSensor.getTime();
		mLastGpsPos = gpsPos;
	}
}
bool PoseDetecting::onCommand(const std::vector<std::string>& args)
{
	VECTOR3 angle = getEulerZYX();
	Debug::print(LOG_PRINT, "EulerZYX: (%.3f, %.3f, %.3f)\r\n", angle.x, angle.y, angle.z);
	Debug::print(LOG_PRINT, "(Flip, Lie): (%s, %s) Velocity: %f GpsAngle: %.3f\r\n", isFlip() ? "y" : "n", isLie() ? "y" : "n", getVelocity(), mEstimatedRelativeGpsCourse);
	Debug::print(LOG_PRINT, "YawAngle: %.3f(%.3f)\r\n", getYaw(true, true), getYaw(true, false));

	if(args.size() == 3)
	{
		if(args[1].compare("accel") == 0)
		{
			mAccelCoeff = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Accel coeff: %f\r\n", mAccelCoeff);
			return true;
		}
		if(args[1].compare("enc") == 0)
		{
			mEncCoeff = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Enc coeff: %f\r\n", mEncCoeff);
			return true;
		}
		if(args[1].compare("flip") == 0)
		{
			mFlipThreshold = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Flip Threshold: %f\r\n", mFlipThreshold);
			return true;
		}
		if(args[1].compare("lie") == 0)
		{
			mLieThreshold = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Lie Threshold: %f\r\n", mLieThreshold);
			return true;
		}
		if(args[1].compare("gps") == 0)
		{
			mGpsCoeff = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "GPS coeff: %f\r\n", mGpsCoeff);
			return true;
		}
		if(args[1].compare("angle") == 0)
		{
			mGpsCoeff = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Angle LPF coeff: %f\r\n", mGpsCoeff);
			return true;
		}
	}
	if(args.size() == 1)
	{
		Debug::print(LOG_PRINT, "Usage:\r\n");
		Debug::print(LOG_PRINT, " %s {accel, enc, gps, angle} [coeff] : set coeff [0-1]\r\n", args[0].c_str());
		Debug::print(LOG_PRINT, " %s {flip, lie} [coeff] : set threshold [0-90]\r\n", args[0].c_str());
		return true;
	}
	return false;
}
VECTOR3 PoseDetecting::getEulerZYX() const
{
	VECTOR3 ret;
	mEstimatedAngle.toEulerZYX(ret);
	ret *= 180 / M_PI;
	return ret;
}
VECTOR3 PoseDetecting::getEulerZYXLPF() const
{
	VECTOR3 ret;
	mEstimatedAngleWithLPF.toEulerZYX(ret);
	ret *= 180 / M_PI;
	return ret;
}
VECTOR3 PoseDetecting::getEulerXYZ() const
{
	VECTOR3 ret;
	mEstimatedAngle.toEulerXYZ(ret);
	ret *= 180 / M_PI;
	return ret;
}
VECTOR3 PoseDetecting::getEulerXYZLPF() const
{
	VECTOR3 ret;
	mEstimatedAngleWithLPF.toEulerXYZ(ret);
	ret *= 180 / M_PI;
	return ret;
}
double PoseDetecting::getRoll() const
{
	VECTOR3 ypr = getEulerZYX();
	return GyroSensor::normalize(ypr.x);
}
double PoseDetecting::getPitch() const
{
	VECTOR3 ypr = getEulerZYX();
	return GyroSensor::normalize(ypr.y);
}
double PoseDetecting::getYaw(bool absolute, bool flipfix) const /////<<<<getYaw から方向の角度を得る
{
	VECTOR3 ypr = getEulerZYX();
	double z = ypr.z + (absolute ? mEstimatedRelativeGpsCourse : 0);
	//if(flipfix && isFlipCoord())z += 180;
	return GyroSensor::normalize(z);
}
double PoseDetecting::getYawLPF(bool absolute, bool flipfix) const
{
	VECTOR3 ypr = getEulerZYXLPF();
	double z = ypr.z + (absolute ? mEstimatedRelativeGpsCourse : 0);
	//if(flipfix && isFlipCoord())z += 180;
	return GyroSensor::normalize(z);
}
double PoseDetecting::getVelocity() const
{
	return mEstimatedVelocity;
}
bool PoseDetecting::isFlip() const
{
	QUATERNION qTop(0,0,1,0);
	qTop = mEstimatedAngle.inverse() * qTop * mEstimatedAngle;
	double flipAngle = GyroSensor::normalize(acos(qTop.z) * 180 / M_PI);
	return abs(flipAngle) > mFlipThreshold;
}
bool PoseDetecting::isFlipCoord() const
{
	QUATERNION qTop(0,0,1,0);
	qTop = mEstimatedAngle.inverse() * qTop * mEstimatedAngle;
	return qTop.z < 0;
}
bool PoseDetecting::isLie() const
{
	QUATERNION qLeft(0,1,0,0);
	qLeft = mEstimatedAngle.inverse() * qLeft * mEstimatedAngle;
	double lieAngle = GyroSensor::normalize(asin(qLeft.z) * 180 / M_PI);
	return abs(lieAngle) > mLieThreshold && abs(lieAngle) < 180 - mLieThreshold;
}
// Ref: http://www.olliw.eu/2013/imu-data-fusing/
void PoseDetecting::updateUsingIMU(double dt, double gx, double gy, double gz, double ax, double ay, double az)
{
	double q0 = mEstimatedAngle.w, q1 = mEstimatedAngle.x, q2 = mEstimatedAngle.y, q3 = mEstimatedAngle.z;
	QUATERNION sQuat;
	QUATERNION dotQuat;
	double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	dotQuat.w = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	dotQuat.x = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	dotQuat.y = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	dotQuat.z = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!(ax == 0 && ay == 0 && ay == 0)) {
		VECTOR3 accel;
		gAccelerationSensor.getAccel(accel);

		// Normalise accelerometer measurement
		accel = accel.normalize();

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		sQuat.w = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		sQuat.x = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		sQuat.y = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		sQuat.z = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		sQuat = sQuat.normalize();

		// Apply feedback step
		dotQuat -= sQuat * mAccelCoeff;
	}

	mEstimatedAngle = QUATERNION(q1, q2, q3, q0);
	// Integrate rate of change of quaternion to yield quaternion
	mEstimatedAngle += dotQuat * dt;
	
	// Normalise quaternion
	mEstimatedAngle = mEstimatedAngle.normalize();
}

double PoseDetecting::calcEncAngle(long long left, long long right)
{
	double distance = (right - left) / static_cast<double>(RESOLVING_POWER) / GEAR_RATIO * DISTANCE_PER_ROTATION;
	double rotation = distance / (2 * DISTANCE_BETWEEN_TIRES * M_PI) * 2 * 180;
	return rotation;
}

PoseDetecting::PoseDetecting() : mEstimatedRelativeGpsCourse(0), mEstimatedVelocity(0), mLastEncL(0), mLastEncR(0), mAccelCoeff(0.2), mEncCoeff(0.05), mGpsCoeff(0.1), mAngleLPFCoeff(0.1), mFlipThreshold(60), mLieThreshold(30)
{
	setName("pose");
	setPriority(TASK_PRIORITY_SENSOR + 1,TASK_INTERVAL_SENSOR);
}

PoseDetecting::~PoseDetecting()
{
}
