//ttb
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include <time.h>
#include <string.h>
#include <fstream>
#include <sstream>
#include <unistd.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "sensor.h"
#include "utils.h"
#include <stdarg.h>
#include "MotionSensor.h"
PressureSensor gPressureSensor;
GPSSensor gGPSSensor;
GyroSensor gGyroSensor;
LightSensor gLightSensor;
//WebCamera gWebCamera;
DistanceSensor gDistanceSensor;
//CameraCapture gCameraCapture;
AccelerationSensor gAccelerationSensor;
Filename gCaptureFilename = Filename("capture", ".png");

//using namespace cv;

//
//// I2C definitions
//
//#define I2C_SLAVE       0x0703
//#define I2C_SMBUS       0x0720  /* SMBus-level access */
//
//#define I2C_SMBUS_READ  1
//#define I2C_SMBUS_WRITE 0
//
//// SMBus transaction types
//
//#define I2C_SMBUS_QUICK             0
//#define I2C_SMBUS_BYTE              1
//#define I2C_SMBUS_BYTE_DATA         2
//#define I2C_SMBUS_WORD_DATA         3
//#define I2C_SMBUS_PROC_CALL         4
//#define I2C_SMBUS_BLOCK_DATA        5
//#define I2C_SMBUS_I2C_BLOCK_BROKEN  6
//#define I2C_SMBUS_BLOCK_PROC_CALL   7           /* SMBus 2.0 */
//#define I2C_SMBUS_I2C_BLOCK_DATA    8
//
//#define I2C_SMBUS_BLOCK_MAX     32      /* As specified in SMBus standard */
//#define I2C_SMBUS_I2C_BLOCK_MAX 32      /* Not specified but we use same structure */
//
//union i2c_smbus_data
//{
//  uint8_t  byte ;
//  uint16_t word ;
//  uint8_t  block [I2C_SMBUS_BLOCK_MAX + 2] ;    // block [0] is used for length + one more for PEC
//};
//
//struct i2c_smbus_ioctl_data
//{
//  char read_write ;
//  uint8_t command ;
//  int size ;
//  union i2c_smbus_data *data ;
//};
//
//static inline int i2c_smbus_access (int fd, char rw, uint8_t command, int size,
//union i2c_smbus_data *data)
//{
//  struct i2c_smbus_ioctl_data args ;
//
//  args.read_write = rw ;
//  args.command    = command ;
//  args.size       = size ;
//  args.data       = data ;
//  return ioctl (fd, I2C_SMBUS, &args) ;
//}

unsigned int wiringPiI2CReadReg32LE(int fd, int address)
{
	return (unsigned int)((unsigned long)wiringPiI2CReadReg8(fd, address + 3) << 24 | (unsigned int)wiringPiI2CReadReg8(fd, address + 2) << 16 | (unsigned int)wiringPiI2CReadReg8(fd, address + 1) << 8 | (unsigned int)wiringPiI2CReadReg8(fd, address));
}
unsigned short wiringPiI2CReadReg16BE(int fd, int address)
{
	return (unsigned short)((unsigned short)wiringPiI2CReadReg8(fd, address) << 8 | (unsigned short)wiringPiI2CReadReg8(fd, address + 1));
}
unsigned short wiringPiI2CReadReg16LE(int fd, int address)
{
	return (unsigned short)((unsigned short)wiringPiI2CReadReg8(fd, address) | (unsigned short)wiringPiI2CReadReg8(fd, address + 1) << 8);
}



//////////////////////////////////////////////
// Pressure Sensor
//////////////////////////////////////////////
float PressureSensor::val2float(unsigned int val, int total_bits, int fractional_bits, int zero_pad)
{

	//気圧センサの係数を読み込んで正しい値を返す
	return static_cast<float>((short int)val) / ((unsigned int)1 << (16 - total_bits + fractional_bits + zero_pad));
}

bool PressureSensor::onInit(const struct timespec& time)
{
	if ((mFileHandle = wiringPiI2CSetup(0b01011100)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup Pressure Sensor\r\n");
		return false;
	}

	//気圧センサーの動作を確認
	if (wiringPiI2CReadReg8(mFileHandle, 0x0F) != 0xBD)
	{
		close(mFileHandle);
		Debug::print(LOG_SUMMARY, "Failed to verify Pressure Sensor\r\n");
		return false;
	}
	//気圧取得開始
	wiringPiI2CWriteReg8(mFileHandle, 0x20, 0x90);


	mLastUpdateRequest = time;

	Debug::print(LOG_SUMMARY, "Pressure Sensor is Ready!: (%f %f %f %f)\r\n", mA0, mB1, mB2, mC12);
	return true;
}

void PressureSensor::onClean()
{
	close(mFileHandle);
}
void PressureSensor::onUpdate(const struct timespec& time)
{
	if (Time::dt(time, mLastUpdateRequest) > 1.000)//前回のデータ要請から1000ms以上経過している場合値を読み取って更新する
	{
		//気圧値計算
		int pressure =wiringPiI2CReadReg8(mFileHandle,0x2A) << 16 | wiringPiI2CReadReg8(mFileHandle,0x29) << 8 |  wiringPiI2CReadReg8(mFileHandle,0x28);
		mPressure = pressure / 4096.0;
short temperature = wiringPiI2CReadReg8(mFileHandle, 0x2c) << 8 | wiringPiI2CReadReg8(mFileHandle, 0x2b) ;
		mTemperature = (temperature /480.0) + 42.5 ;
		//気圧更新要請時刻を記録
		mLastUpdateRequest = time;
	}
}

bool PressureSensor::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;
	Debug::print(LOG_SUMMARY, "Pressure: %f\r\nTemperature: %f\r\n", mPressure, mTemperature);
	return true;
}

float PressureSensor::get() const
{
	return mPressure;
}
float PressureSensor::getTemperature() const
{
	return mTemperature;
}
PressureSensor::PressureSensor() : mA0(0), mB1(0), mB2(0), mC12(0), mPressure(0), mTemperature(0), mFileHandle(-1)
{
	setName("pressure");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
PressureSensor::~PressureSensor()
{
}

//////////////////////////////////////////////
// GPS Sensor
//////////////////////////////////////////////
bool GPSSensor::onInit(const struct timespec& time)
{
	mLastCheckTime = time;
	if ((mFileHandle = wiringPiI2CSetup(0x20)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup GPS Sensor\r\n");
		return false;
	}

	//座標を更新するように設定(一応2回書き込み)
	wiringPiI2CWriteReg8(mFileHandle, 0x01, 0x05);
	wiringPiI2CWriteReg8(mFileHandle, 0x01, 0x05);

	//バージョン情報を表示
	Debug::print(LOG_SUMMARY, "GPS Firmware Version:%d\r\n", wiringPiI2CReadReg8(mFileHandle, 0x03));

	mPos.x = mPos.y = mPos.z = 0;
	mIsNewData = false;
	mIsLogger = false;
	return true;
}
void GPSSensor::onClean()
{
	//動作を停止するコマンドを発行
	wiringPiI2CWriteReg8(mFileHandle, 0x01, 0x06);

	close(mFileHandle);
}
void GPSSensor::onUpdate(const struct timespec& time)
{
	unsigned char status = wiringPiI2CReadReg8(mFileHandle, 0x00);
	if (status & 0x06)// Found Position
	{
		//座標を更新(読み取り時のデータ乱れ防止用に2回読み取って等しい値が取れた場合のみ採用する)

		//経度
		int read = (int)wiringPiI2CReadReg32LE(mFileHandle, 0x07);
		if (read == (int)wiringPiI2CReadReg32LE(mFileHandle, 0x07))mPos.x = read / 10000000.0;

		//緯度
		read = (int)wiringPiI2CReadReg32LE(mFileHandle, 0x0B);
		if (read == (int)wiringPiI2CReadReg32LE(mFileHandle, 0x0B))mPos.y = read / 10000000.0;

		//高度
		read = (short)wiringPiI2CReadReg16LE(mFileHandle, 0x21);
		if (read == (short)wiringPiI2CReadReg16LE(mFileHandle, 0x21))mPos.z = read;

		//Ground course
		read = (short)wiringPiI2CReadReg16LE(mFileHandle, 35);
		if (read == (short)wiringPiI2CReadReg16LE(mFileHandle, 35))mGpsCourse = read / 10.0f;

		//Ground speed
		read = (short)wiringPiI2CReadReg16LE(mFileHandle, 31);
		if (read == (short)wiringPiI2CReadReg16LE(mFileHandle, 31))mGpsSpeed = read / 100.0f;

		//新しいデータが届いたことを記録する
		if (status & 0x01)mIsNewData = true;
	}
	//衛星個数を更新(読み取り時のデータ乱れ防止用に2回読み取って等しい値が取れた場合のみ採用する)
	if (wiringPiI2CReadReg8(mFileHandle, 0x00) == status)mSatelites = (unsigned char)status >> 4;

	if(mSatelites > 0)
	{
		//Time
		int read = (int)wiringPiI2CReadReg32LE(mFileHandle, 39);
		if (read == (int)wiringPiI2CReadReg32LE(mFileHandle, 39))mGpsTime = read;
	}


	if (mIsLogger)
	{
		//1秒ごとにGPS座標を表示する
		if (Time::dt(time, mLastCheckTime) > 1)
		{
			mLastCheckTime = time;
			showState();
		}
	}
}
bool GPSSensor::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;

	if (args.size() == 1)
	{
		showState();
		return true;
	}
	else if (args.size() == 2)
	{
		if (args[1].compare("start") == 0)
		{
			mIsLogger = true;
			Debug::print(LOG_SUMMARY, "GPS logger start!\r\n");
			return true;
		}
		else if (args[1].compare("stop") == 0)
		{
			mIsLogger = false;
			Debug::print(LOG_SUMMARY, "GPS logger stop!\r\n");
			return true;
		}
	}
	Debug::print(LOG_PRINT, "gps      : show GPS state\r\n\
gps start: GPS logger start\r\n\
gps stop : GPS logger stop\r\n");
	return true;
}
bool GPSSensor::get(VECTOR3& pos, bool disableNewFlag)
{
	if (mSatelites >= 4 && !(mPos.x == 0 && mPos.y == 0 && mPos.z == 0))//3D fix
	{
		if (!disableNewFlag)mIsNewData = false;//データを取得したことを記録
		pos = mPos;//引数のposに代入
		return true;
	}
	return false;//Invalid Position
}
bool GPSSensor::isNewPos() const
{
	return mIsNewData;
}
int GPSSensor::getTime() const
{
	return mGpsTime;
}
float GPSSensor::getCourse() const
{
	return GyroSensor::normalize(mGpsCourse);
}
float GPSSensor::getSpeed() const
{
	return mGpsSpeed;
}
void GPSSensor::showState() const
{
	if (mSatelites < 4) Debug::print(LOG_SUMMARY, "Unknown Position\r\nSatelites: %d\r\n", mSatelites);
	else Debug::print(LOG_SUMMARY, "Satelites: %d \r\nPosition: %f %f %f,\r\nTime: %d\r\nCourse: %f\r\nSpeed: %f\r\n", mSatelites, mPos.x, mPos.y, mPos.z, mGpsTime, mGpsCourse, mGpsSpeed);
}
GPSSensor::GPSSensor() : mFileHandle(-1), mPos(), mSatelites(0), mIsNewData(false)
{
	setName("gps");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
GPSSensor::~GPSSensor()
{
}

//////////////////////////////////////////////
// Gyro Sensor
//////////////////////////////////////////////

bool GyroSensor::onInit(const struct timespec& time)
{
	mRVel.x = mRVel.y = mRVel.z = 0;
	mRAngle.x = mRAngle.y = mRAngle.z = 0;
	memset(&mLastSampleTime, 0, sizeof(mLastSampleTime));

	if ((mFileHandle = wiringPiI2CSetup(0x6b)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup Gyro Sensor\r\n");
		return false;
	}

	//ジャイロセンサーが正常動作中か確認
	if (wiringPiI2CReadReg8(mFileHandle, 0x0F) != 0xD4)
	{
		close(mFileHandle);
		Debug::print(LOG_SUMMARY, "Failed to verify Gyro Sensor\r\n");
		return false;
	}
	//データサンプリング無効化
	wiringPiI2CWriteReg8(mFileHandle, 0x20, 0x00);

	//ビッグエンディアンでのデータ出力に設定&スケールを2000dpsに変更
	wiringPiI2CWriteReg8(mFileHandle, 0x23, 0x40 | 0x20);

	//FIFO有効化(ストリームモード)
	wiringPiI2CWriteReg8(mFileHandle, 0x24, 0x40);
	wiringPiI2CWriteReg8(mFileHandle, 0x2E, 0x40);

	//データサンプリング有効化
	wiringPiI2CWriteReg8(mFileHandle, 0x20, 0x0f);

	return true;
}

void GyroSensor::onClean()
{
	//データサンプリング無効化
	wiringPiI2CWriteReg8(mFileHandle, 0x20, 0x00);

	close(mFileHandle);
}

void GyroSensor::onUpdate(const struct timespec& time)
{
	int status_reg;
	int data_samples = 0;
	VECTOR3 newRv;

	//蓄えられたサンプルの平均値を現時点での速度とする
	while ((status_reg = wiringPiI2CReadReg8(mFileHandle, 0x27)) & 0x08)
	{
		if (status_reg == -1)
		{
			Debug::print(LOG_DETAIL, "Gyro reading error!\r\n");
			return;
		}
		//if(status_reg & 0x70)Debug::print(LOG_DETAIL,"Gyro Data Overrun!\r\n");

		//ジャイロのFIFO内のデータをすべて読み込み、和を取る
		VECTOR3 sample;
		sample.x = (short)wiringPiI2CReadReg16BE(mFileHandle, 0x28) * 0.070;
		sample.y = (short)wiringPiI2CReadReg16BE(mFileHandle, 0x2A) * 0.070;
		sample.z = (short)wiringPiI2CReadReg16BE(mFileHandle, 0x2C) * 0.070;
		newRv += sample;

		//ドリフト誤差計算中であれば配列にデータを突っ込む
		if (mIsCalculatingOffset)
		{
			mRVelHistory.push_back(sample);
			if (mRVelHistory.size() >= GYRO_SAMPLE_COUNT_FOR_CALCULATE_OFFSET)//必要なサンプル数がそろった
			{
				//平均値を取ってみる
				std::list<VECTOR3>::iterator it = mRVelHistory.begin();
				while (it != mRVelHistory.end())
				{
					mRVelOffset += *it;
					++it;
				}
				mRVelOffset /= mRVelHistory.size();//ドリフト誤差補正量を適用
				mRVelHistory.clear();
				mIsCalculatingOffset = false;
				Debug::print(LOG_SUMMARY, "Gyro: offset is (%f %f %f)\r\n", mRVelOffset.x, mRVelOffset.y, mRVelOffset.z);
			}
		}

		//ドリフト誤差を補正
		newRv -= mRVelOffset;

		++data_samples;
	}

	//データが来ていたら現在の角速度と角度を更新
	if (data_samples != 0)
	{
		//平均
		newRv /= data_samples;

		newRv.x = abs(newRv.x) < mCutOffThreshold ? 0 : newRv.x;
		newRv.y = abs(newRv.y) < mCutOffThreshold ? 0 : newRv.y;
		newRv.z = abs(newRv.z) < mCutOffThreshold ? 0 : newRv.z;

		//積分
		if (mLastSampleTime.tv_sec != 0 || mLastSampleTime.tv_nsec != 0)
		{
			double dt = Time::dt(time, mLastSampleTime);
			mRAngle += (newRv + mRVel) / 2 * dt;
			normalize(mRAngle);
		}
		mRVel = newRv;
		mLastSampleTime = time;
	}
}
bool GyroSensor::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("reset") == 0)
		{
			setZero();
			return true;
		}
		else if (args[1].compare("calib") == 0)
		{
			if (!isActive())return false;
			calibrate();
			return true;
		}
		return false;
	}
	else if (args.size() == 3)
	{
		if (args[1].compare("cutoff") == 0)
		{
			mCutOffThreshold = atof(args[2].c_str());
			Debug::print(LOG_SUMMARY, "Gyro: cutoff threshold is %f\r\n", mCutOffThreshold);
			return true;
		}
		return false;
	}
	else if (args.size() == 5)
	{
		if (args[1].compare("calib") == 0)
		{
			mRVelOffset.x = atof(args[2].c_str());
			mRVelOffset.y = atof(args[3].c_str());
			mRVelOffset.z = atof(args[4].c_str());
			Debug::print(LOG_SUMMARY, "Gyro: offset is (%f %f %f)\r\n", mRVelOffset.x, mRVelOffset.y, mRVelOffset.z);
			return true;
		}
		return false;
	}
	Debug::print(LOG_SUMMARY, "Angle: %f %f %f\r\nAngle Velocity: %f %f %f\r\n\
 gyro reset  : set angle to zero point\r\n\
 gyro cutoff : set cutoff threshold\r\n\
 gyro calib  : calibrate gyro *do NOT move*\r\n\
 gyro calib [x_offset] [y_offset] [z_offset] : calibrate gyro by specified params\r\n", getRx(), getRy(), getRz(), getRvx(), getRvy(), getRvz());
	return true;
}
bool GyroSensor::getRVel(VECTOR3& vel) const
{
	if (isActive())
	{
		vel = mRVel;
		return true;
	}
	return false;
}
double GyroSensor::getRvx() const
{
	return mRVel.x;
}
double GyroSensor::getRvy() const
{
	return mRVel.y;
}
double GyroSensor::getRvz() const
{
	return mRVel.z;
}
void GyroSensor::setZero()
{
	mRAngle.x = mRAngle.y = mRAngle.z = 0;
}
bool GyroSensor::getRPos(VECTOR3& pos) const
{
	if (isActive())
	{
		pos = mRAngle;
		return true;
	}
	return false;
}
double GyroSensor::getRx() const
{
	return mRAngle.x;
}
double GyroSensor::getRy() const
{
	return mRAngle.y;
}
double GyroSensor::getRz() const
{
	return mRAngle.z;
}
void GyroSensor::calibrate()
{
	mIsCalculatingOffset = true;
}
double GyroSensor::normalize(double pos)
{
	while (pos >= 180 || pos < -180)pos += (pos > 0) ? -360 : 360;
	return pos;
}
void GyroSensor::normalize(VECTOR3& pos)
{
	pos.x = normalize(pos.x);
	pos.y = normalize(pos.y);
	pos.z = normalize(pos.z);
}
GyroSensor::GyroSensor() : mFileHandle(-1), mRVel(), mRAngle(), mRVelHistory(), mRVelOffset(), mCutOffThreshold(0.1), mIsCalculatingOffset(false)
{
	setName("gyro");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_GYRO);
}
GyroSensor::~GyroSensor()
{
}

////////////////////////////////////////////////
//// Accel Sensor
////////////////////////////////////////////////
//
bool AccelerationSensor::onInit(const struct timespec& time)
{
	mAccel.x = mAccel.y = mAccel.z = 0;

	if ((mFileHandle = wiringPiI2CSetup(0x1d)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup Acceleration Sensor\r\n");
		return false;
	}

	//データサンプリング有効化
	wiringPiI2CWriteReg8(mFileHandle, 0x16, 0x09); // 64 LSB/g
	wiringPiI2CWriteReg8(mFileHandle, 0x18, 0x38);
	return true;
}

void AccelerationSensor::onClean()
{
	//データサンプリング無効化
	wiringPiI2CWriteReg8(mFileHandle, 0x16, 0x00);

	close(mFileHandle);
}

short ushortTo10BitShort(unsigned short val)
{
	if (val & 0x200)val |= 0xfc00;
	return (short)val;
}
void AccelerationSensor::onUpdate(const struct timespec& time)
{
	//union i2c_smbus_data data;
	//i2c_smbus_access(mFileHandle, I2C_SMBUS_READ, 0x01, I2C_SMBUS_I2C_BLOCK_DATA, &data);
	//mAccel.x = ((signed char)data.block[1]);
	//mAccel.y = ((signed char)data.block[2]);
	//mAccel.z = ((signed char)data.block[3]);
	short x = ushortTo10BitShort(wiringPiI2CReadReg16LE(mFileHandle, 0x00));
	short y = ushortTo10BitShort(wiringPiI2CReadReg16LE(mFileHandle, 0x02));
	short z = ushortTo10BitShort(wiringPiI2CReadReg16LE(mFileHandle, 0x04));

	mAccel.x = x / 64.0f;
	mAccel.y = y / 64.0f;
	mAccel.z = z / 64.0f;

  mAccelAve = mAccel * mAccelAlpha + mAccelAve * (1 - mAccelAlpha);
}
bool AccelerationSensor::onCommand(const std::vector<std::string>& args)
{
	if(args.size() == 3)
	{
		if(args[1].compare("alpha") == 0)
		{
			mAccelAlpha = atof(args[2].c_str());
			Debug::print(LOG_PRINT, "Accel: Alpha is %f\r\n", mAccelAlpha);
			return true;
		}
	}
	if(!isActive())
	{
		Debug::print(LOG_PRINT, "Start accel before sampling\r\n");
		return false;
	}
	Debug::print(LOG_SUMMARY, "Acceleration: %f %f %f\r\n", getAx(), getAy(), getAz());
	Debug::print(LOG_SUMMARY, "Accel. angle: %f %f %f\r\n", getTheta() / M_PI * 180, getPsi() / M_PI * 180, getPhi() / M_PI * 180);
	Debug::print(LOG_SUMMARY, "Usage:\r\n %s alpha [val] : set accel alpha coefficient\r\n",args[0].c_str());
	return true;
}
bool AccelerationSensor::getAccel(VECTOR3& acc) const
{
	if (isActive())
	{
		acc = mAccelAve;
		return true;
	}
	return false;
}
double AccelerationSensor::getAx() const
{
	return mAccelAve.x;
}
double AccelerationSensor::getAy() const
{
	return mAccelAve.y;
}
double AccelerationSensor::getAz() const
{
	return mAccelAve.z;
}
double AccelerationSensor::getTheta() const
{
    return atan2f(mAccelAve.x, sqrt(pow(mAccelAve.y, 2) + pow(mAccelAve.z, 2)));
}
double AccelerationSensor::getPsi() const
{
    return atan2f(mAccelAve.y, sqrt(pow(mAccelAve.x, 2) + pow(mAccelAve.z, 2)));
}
double AccelerationSensor::getPhi() const
{
    return atan2f(sqrt(pow(mAccelAve.x, 2) + pow(mAccelAve.y, 2)), mAccelAve.z);
}
bool AccelerationSensor::getRawAccel(VECTOR3& acc) const
{
	if(isActive())
	{
		acc = mAccel;
		return true;
	}
	return false;
}
AccelerationSensor::AccelerationSensor() : mFileHandle(-1),mAccel(), mAccelAve(), mAccelAlpha(0.5)
{
	setName("accel");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
AccelerationSensor::~AccelerationSensor()
{
}

///////////////////////////////////////////////
// CdS Sensor
///////////////////////////////////////////////
bool LightSensor::onInit(const struct timespec& time)
{
	pinMode(mPin, INPUT);
	return true;
}
void LightSensor::onClean()
{
}
bool LightSensor::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;
	if (get())Debug::print(LOG_SUMMARY, "light is high\r\n");
	else Debug::print(LOG_SUMMARY, "light is low\r\n");
	return true;
}
bool LightSensor::get() const
{
	return digitalRead(mPin) == 0;
}
LightSensor::LightSensor() : mPin(PIN_LIGHT_SENSOR)
{
	setName("light");
	setPriority(TASK_PRIORITY_SENSOR, UINT_MAX);
}
LightSensor::~LightSensor()
{
}

///////////////////////////////////////////////
// Webカメラ
///////////////////////////////////////////////

bool WebCamera::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;
	if (args.size() >= 2)
	{
		if (args[1].compare("start") == 0)
		{
			stop();
			Debug::print(LOG_SUMMARY, "Start capturing!\r\n");
			if (args.size() == 3)start(args[2].c_str());
			else start();

			return true;
		}
		else if (args[1].compare("stop") == 0)
		{
			stop();
			return true;
		}
	}
	else
	{
		Debug::print(LOG_PRINT, "capture start [filename] : save movie to filename\r\n\
apture stop             : stop capturing movie\r\n");
		return true;
	}
	return false;
}
void WebCamera::onClean()
{
	stop();
}
void WebCamera::start(const char* filename)
{
	time_t timer = time(NULL);
	std::stringstream ss;
	std::string name;
	if (filename == NULL)
	{
		name = ctime(&timer) + std::string(".avi");
	}
	else
	{
		name = filename;
	}

	ss << "mencoder tv:// -tv width=320:height=240:device=/dev/video0 -nosound -ovc lavc -o \"" << name << "\" 1> /dev/null 2>&1 &";
	system(ss.str().c_str());
}
void WebCamera::stop()
{
	//Todo: 末尾が5秒くらい保存されない問題
	system("killall -15 mencoder 1> /dev/null 2>&1 ");
	Debug::print(LOG_DETAIL, "Send kill signal to mencoder\r\n");
}

WebCamera::WebCamera()
{
	setName("capture");
	setPriority(UINT_MAX, UINT_MAX);
}
WebCamera::~WebCamera()
{
}

void* DistanceSensor::waitingThread(void* arg)
{
	DistanceSensor& parent = *reinterpret_cast<DistanceSensor*>(arg);
	struct timespec newTime;

	while (1)
	{
		while (!parent.mIsCalculating)usleep(1000);

		//Send Ping
		pinMode(PIN_DISTANCE, OUTPUT);
		digitalWrite(PIN_DISTANCE, HIGH);
		clock_gettime(CLOCK_MONOTONIC_RAW, &parent.mLastSampleTime);
		do
		{
			clock_gettime(CLOCK_MONOTONIC_RAW, &newTime);
		} while (Time::dt(newTime, parent.mLastSampleTime) < 0.000001);
		digitalWrite(PIN_DISTANCE, LOW);

		//Wait For Result
		pinMode(PIN_DISTANCE, INPUT);
		do
		{
			clock_gettime(CLOCK_MONOTONIC_RAW, &newTime);
			if (Time::dt(newTime, parent.mLastSampleTime) > 0.001)
			{
				//Timeout
				parent.mIsCalculating = false;
				parent.mLastDistance = -1;
				break;
			}
		} while (digitalRead(PIN_DISTANCE) == LOW);
		parent.mLastSampleTime = newTime;
		do
		{
			clock_gettime(CLOCK_MONOTONIC_RAW, &newTime);
			if (Time::dt(newTime, parent.mLastSampleTime) > 0.02)
			{
				//Timeout
				parent.mIsCalculating = false;
				parent.mLastDistance = -1;
				break;
			}
		} while (digitalRead(PIN_DISTANCE) == HIGH);
		clock_gettime(CLOCK_MONOTONIC_RAW, &newTime);

		double delay = Time::dt(newTime, parent.mLastSampleTime);
		parent.mLastDistance = delay * 100 * 3 / 2;
		if (delay > 0.019)parent.mLastDistance = -1;
		parent.mIsNewData = true;
		parent.mIsCalculating = false;
	}
	return NULL;
}
bool DistanceSensor::onInit(const struct timespec& time)
{
	mLastDistance = -1;
	if (pthread_create(&mPthread, NULL, waitingThread, this) != 0)
	{
		Debug::print(LOG_SUMMARY, "DistanceSensor: Unable to create thread!\r\n");
		return false;
	}
	return true;
}
void DistanceSensor::onClean()
{
	if (mIsCalculating)pthread_cancel(mPthread);
	mLastDistance = -1;
	mIsCalculating = false;
	mIsNewData = false;
}

void DistanceSensor::onUpdate(const struct timespec& time)
{

}
bool DistanceSensor::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;
	if (args.size() == 1)
	{
		Debug::print(LOG_SUMMARY, "Last Distance: %f m\r\n", mLastDistance);
		if (ping())Debug::print(LOG_SUMMARY, "Calculating New Distance!\n", mLastDistance);
		return true;
	}
	return false;
}

bool DistanceSensor::ping()
{
	if (mIsCalculating)return false;//すでに計測を開始している
	mIsCalculating = true;
	return true;
}
bool DistanceSensor::getDistance(double& distance)
{
	bool ret = mIsNewData;
	mIsNewData = false;
	distance = mLastDistance;
	return ret;
}

DistanceSensor::DistanceSensor() : mLastDistance(-1), mIsCalculating(false), mIsNewData(false)
{
	setName("distance");
	setPriority(TASK_PRIORITY_SENSOR, TASK_INTERVAL_SENSOR);
}
DistanceSensor::~DistanceSensor()
{
}

//////////////////////////////////////////////
// Web Camera
//////////////////////////////////////////////
/*
bool CameraCapture::onInit(const struct timespec& time)
{
	mpCapture = cvCreateCameraCapture(-1);
	if (mpCapture == NULL)
	{
		Debug::print(LOG_SUMMARY, "Unable to initialize camera\r\n");
		return false;
	}
	cvSetCaptureProperty(mpCapture, CV_CAP_PROP_FRAME_WIDTH, WIDTH); //撮影サイズを指定
	cvSetCaptureProperty(mpCapture, CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

	mIsWarming = false;
	verifyCamera(false);

	gGPSSensor.setRunMode(true);


	count = 0;

	return true;
}
void CameraCapture::onClean()
{
	cvReleaseCapture(&mpCapture);
	mpCapture = NULL;
}
bool CameraCapture::onCommand(const std::vector<std::string>& args)
{
	if (!isActive())return false;
	if (args.size() == 2)
	{
		if (args[1].compare("save") == 0)
		{
			save();
			startWarming();
		}
		else if (args[1].compare("warm") == 0)
		{
			startWarming();
		}
		return true;
	}
	else if (args.size() == 3)
	{
		if (args[1].compare("save") == 0)
		{
			save(&args[2]);
			startWarming();
		}
		return true;
	}
	Debug::print(LOG_SUMMARY, "camera warm  : stand by for capturing\r\n\
 camera save  : take picture\r\n\
 camera save [name] : take picture as name\r\n");
	return false;
}
void CameraCapture::onUpdate(const struct timespec& time)
{
	if (mIsWarming)
	{
		getFrame(0, 0);
		mIsWarming = true;
	}
}
void CameraCapture::verifyCamera(bool reinitialize)
{
	unsigned int deviceId = 0;
	bool exist = false;
	struct stat st;
	do
	{
		std::stringstream filename;
		filename << "/dev/video" << deviceId++;
		exist = stat(filename.str().c_str(), &st) == 0;
		if (deviceId > 32)return;//失敗
	} while (!exist);
	--deviceId;

	if (deviceId != mCurVideoDeviceID && reinitialize)
	{
		Debug::print(LOG_SUMMARY, "Camera: not available, trying to reinitialize\r\n");
		setRunMode(false);
		//cvReleaseCapture(&mpCapture);
		//mpCapture = cvCreateCameraCapture(-1);
	}
	mCurVideoDeviceID = deviceId;
}
void CameraCapture::startWarming()
{
	mIsWarming = true;
}
void CameraCapture::save(const std::string* name, IplImage* pImage, bool nolog)
{
	if (!isActive())return;
	std::string filename;
	if (name != NULL)filename.assign(*name);
	else gCaptureFilename.get(filename);
	if (pImage == NULL)pImage = getFrame(0, 0);
	cvSaveImage(filename.c_str(), pImage);
	if (!nolog)Debug::print(LOG_SUMMARY, "Captured image was saved as %s\r\n", filename.c_str());
}
void CameraCapture::wadatisave(const std::string* name, IplImage* pImage, bool nolog)
{

	gGPSSensor.get(vec);
	if (gGPSSensor.isActive())write(gpsfilename, "%f,%f,%f,%d\r\n", vec.x, vec.y, vec.z, count);
	else write(gpsfilename, "unavailable\r\n");

	if (!isActive())return;
	std::string filename, filename_c;
	std::string picname = "pic_gps_" + std::to_string(count);
	std::string picname_c = "pic_gps_c_" + std::to_string(count++);
	Filename(picname.c_str(), ".jpg").getNoIndex(filename);
	Filename(picname_c.c_str(), ".jpg").getNoIndex(filename_c);
	//if (name != NULL)filename.assign(*name);
	//else gCaptureFilename.get(filename);
	if (pImage == NULL)pImage = getFrame(0, 0);
	cvSaveImage(filename.c_str(), pImage);
	if (!nolog)Debug::print(LOG_SUMMARY, "Captured image was saved as %s\r\n", filename.c_str());
	//filename.c_str()
    // src(source): to save source image(grayscale)
    Mat src = imread(filename, 0);

	// blr(blur): to save blurred source image
	// cny(canny): save the canny processed image
	Mat blr, cny;
	blur(src, blr, Size(3, 3));
	Canny(blr, cny, 50, 200);

	// block_h, block_w: decide the size of image segmentation
	const int block_h = 8, block_w = 9;
	int canny_result[block_h][block_w] = {0};
	int src_y = src.rows, src_x = src.cols, x_inc = src_x / block_w, y_inc =src_y / block_h;

	for (int h = 0, y = 0; h < block_h; h++, y += y_inc) {
		for (int w = 0, x = 0; w < block_w; w++, x += x_inc) {
			Mat cut(cny, Rect(x, y, x_inc, y_inc));
			MatIterator_<uchar> itr = cut.begin<uchar>();
			for (; itr != cut.end<uchar>(); ++itr) {
				if (*itr == 255) {
					canny_result[h][w]++;
				}
			}
		}
	}

	//Draw lines and result
	Mat rut;
	cvtColor(src, rut, CV_GRAY2RGB);
	for (int h = 0, y = y_inc; h < block_h - 1; h++, y += y_inc) {
		Point p0 = Point(0, y);
		Point p1 = Point(src_x, y);
		line(rut, p0, p1, Scalar(0, 0, 0), 1, 4);
	}
	for (int w = 0, x = x_inc; w < block_w - 1; w++, x += x_inc) {
		Point p0 = Point(x, 0);
		Point p1 = Point(x, src_y);
		line(rut, p0, p1, Scalar(0, 0, 0), 1, 4);
	}
	std::stringstream rs;
	for (int h = 0, y = 0; h < block_h; h++, y += y_inc) {
		for (int w = 0, x = 0; w < block_w; w++, x += x_inc) {
			rs.str("");
			Point p = Point(x + x_inc * 0.1, y + y_inc * 0.64);
			rs << canny_result[h][w];
			if (h > block_h / 2) {
				if (canny_result[h][w] < 60) {
					putText(rut, rs.str(), p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 255, 0), 1, CV_AA);
				} else if (canny_result[h][w] < 200) {
					putText(rut, rs.str(), p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(255, 0, 0), 1, CV_AA);
				} else {
					putText(rut, rs.str(), p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 255), 1, CV_AA);
				}
			}else {
				putText(rut, rs.str(), p, FONT_HERSHEY_TRIPLEX, 0.5, Scalar(0, 0, 0), 1, CV_AA);
			}
		}
	}

	if (name != NULL)filename.assign(*name);
	else gCaptureFilename.get(filename);
	IplImage test = rut;
	cvSaveImage(filename_c.c_str(), &test);

}
*/
/*
void CameraCapture::write(const std::string& filename, const char* fmt, ...)
{
	std::ofstream of(filename.c_str(), std::ios::out | std::ios::app);

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);

	of << buf;
}
IplImage* CameraCapture::getFrame(int width, int height)
{
	if (!isActive())return NULL;
	mIsWarming = false;

	verifyCamera();
	IplImage* pImage = cvQueryFrame(mpCapture);
	if (pImage == NULL)
	{
		//エラー返してくれない
	}
	if(width <= 0 || height <= 0)return pImage;

	if(mpResizedImage == NULL || mpResizedImage->width != width || mpResizedImage->height != height)
	{
		if(mpResizedImage != NULL)cvReleaseImage(&mpResizedImage);
		mpResizedImage = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 3);
	}
	cvResize(pImage, mpResizedImage);

	return mpResizedImage;
}
CameraCapture::CameraCapture() : mpCapture(NULL), mpResizedImage(NULL), mIsWarming(false)
{
	setName("camera");
	setPriority(UINT_MAX, 5);
	Filename("pic_gps", ".txt").get(gpsfilename);
}
CameraCapture::~CameraCapture()
{
	if(mpResizedImage != NULL)cvReleaseImage(&mpResizedImage);
}
*/
bool NineAxisSensor::onInit(const struct timespec& time)
{
	if(ms_open() == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup NineAxisSensor\r\n");
		return false;
	}
}
