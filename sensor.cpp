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
#include <wiringSerial.h>
#include <iostream>
#include <iomanip>
#include <ctime>
#include <libgpsmm.h>
#include "actuator.h"
PressureSensor gPressureSensor;
GPSSensor gGPSSensor;
//GyroSensor gGyroSensor;
LightSensor gLightSensor;
//WebCamera gWebCamera;
//DistanceSensor gDistanceSensor;
//CameraCapture gCameraCapture;
//AccelerationSensor gAccelerationSensor;
//Filename gCaptureFilename = Filename("capture", ".png");
NineAxisSensor gNineAxisSensor;
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


	if (gps_rec.stream(WATCH_ENABLE | WATCH_JSON) == NULL) {
		Debug::print(LOG_SUMMARY, "Failed to setup GPS Sensor\r\n");
	}

	mPos.x = mPos.y = mPos.z = 0;
	mIsNewData = false;
	mIsLogger = false;
	return true;
}
void GPSSensor::onClean()
{
	Debug::print(LOG_SUMMARY, "onclean\r\n");
}
void GPSSensor::onUpdate(const struct timespec& time)
{

	if ((newdata = gps_rec.read()) == NULL) {
		return;
	}
	else 
  {
    {
  		mPos.x = newdata->fix.latitude;
    	mPos.y = newdata->fix.longitude;
  		mPos.z = newdata->fix.altitude;
  		mSatelites = newdata->satellites_visible;
  		mGpsSpeed = newdata->fix.speed;
  		mGpsCourse = newdata->fix.track;
  		mIsNewData = true;
    }
  }

	if (mSatelites > 0) {
		mGpsTime = newdata->fix.time;
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
	return mGpsCourse;
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
GPSSensor::GPSSensor() : mFileHandle(-1), mPos(), mSatelites(0), mIsNewData(false), gps_rec("localhost", DEFAULT_GPSD_PORT)
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
/*
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
*/

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


bool NineAxisSensor::onInit(const struct timespec& time)
{
		if ((mFileHandle = wiringPiI2CSetup(0b01101000)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup NineAxis Sensor\r\n");
		return false;
	}

    wiringPiI2CWriteReg8(mFileHandle, 0x6B,0x80);
    delay(200);
 	if (wiringPiI2CReadReg8(mFileHandle, 0x75) != 0x71)
	{
		close(mFileHandle);
		Debug::print(LOG_SUMMARY, "Failed to reset NineAxis Sensor\r\n");
		return false;
	}

 wiringPiI2CWriteReg8(mFileHandle,0x6B,0x00);
  wiringPiI2CWriteReg8(mFileHandle,0x6A, 0x24); // Enable Master I2C, disable primary I2C I/F, and reset FIFO.
  wiringPiI2CWriteReg8(mFileHandle,0x19, 19); // SMPLRT_DIV = 9, 100Hz sampling;
  wiringPiI2CWriteReg8(mFileHandle,0x1A, (1 << 6) | (1 << 0)); // FIFO_mode = 1 (accept overflow), Use LPF, Bandwidth_gyro = 184 Hz, Bandwidth_temperature = 188 Hz,
  wiringPiI2CWriteReg8(mFileHandle,0x1B, (3 << 3)); // FS_SEL = 3 (2000dps)
  wiringPiI2CWriteReg8(mFileHandle,0x1C, (3 << 3)); // AFS_SEL = 3 (16G)
  wiringPiI2CWriteReg8(mFileHandle,0x23, 0x78); // FIFO enabled for , gyro(2 * 3), accelerometer(2 * 3),  Total 12 bytes.
  wiringPiI2CWriteReg8(mFileHandle,0x6A, 0x40); // Enable FIFO 

  wiringPiI2CWriteReg8(mFileHandle,0x37, 0x02);
 if ((mFileHandleCompass = wiringPiI2CSetup(0b0001100)) == -1)
	{
		Debug::print(LOG_SUMMARY, "Failed to setup Compass\r\n");
		return false;
	}
 wiringPiI2CWriteReg8(mFileHandleCompass,0x0A,0x16);
	if (wiringPiI2CReadReg8(mFileHandleCompass, 0x00) != 0x48)
	{
		close(mFileHandleCompass);
		Debug::print(LOG_SUMMARY, "Failed to setup Compass Sensor\r\n");
		return false;
	}



	if (wiringPiI2CReadReg8(mFileHandle, 0x75) != 0x71)
	{
		close(mFileHandle);
		Debug::print(LOG_SUMMARY, "Failed to setup NineAxis Sensor\r\n");
		return false;
	}

	Debug::print(LOG_SUMMARY, "NineAxis Sensor is Ready!\r\n");
	return true;
}
void NineAxisSensor::onClean()
{
}
void NineAxisSensor::onUpdate(const struct timespec& time)
{
	
	//更新を１秒置きにする
	if (Time::dt(time, mLastSampleTime) <2.0) return;

  if(!isFIFOEnable)
  {
  unsigned short fifo_count = (wiringPiI2CReadReg8(mFileHandle, 0x72) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x73);
  wiringPiI2CWriteReg8(mFileHandle,0x72,0);
  wiringPiI2CWriteReg8(mFileHandle,0x73,0);
//  for(int i = 0; i < fifo_count ; i++)
//  {
#define AXEL_RANGE 0.000488
  short mX = (wiringPiI2CReadReg8(mFileHandle, 0x3B) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x3C);
  short mY = (wiringPiI2CReadReg8(mFileHandle, 0x3D) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x3E);
  short mZ = (wiringPiI2CReadReg8(mFileHandle, 0x3F) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x40);
	mAccel.x = AXEL_RANGE*mX;
	mAccel.y = AXEL_RANGE*mY;
	mAccel.z = AXEL_RANGE*mZ;
  mAccelAlpha = 1;
  mAccelAve = mAccel * mAccelAlpha + mAccelAve * (1 - mAccelAlpha);
#define GYRO_RANGE 0.06097
	mX = (wiringPiI2CReadReg8(mFileHandle, 0x43) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x44);
  mY = (wiringPiI2CReadReg8(mFileHandle, 0x45) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x46);
  mZ = (wiringPiI2CReadReg8(mFileHandle, 0x47) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x48);
	VECTOR3 newRv;
	newRv.x =  GYRO_RANGE * mX;
	newRv.y =  GYRO_RANGE * mY;
	newRv.z =  GYRO_RANGE * mZ;
  if(wiringPiI2CReadReg8(mFileHandleCompass, 0x02)| 0x01)
  {
	mX = (wiringPiI2CReadReg8(mFileHandleCompass, 0x04) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x03);
	mY = (wiringPiI2CReadReg8(mFileHandleCompass, 0x06) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x05);
	mX = (wiringPiI2CReadReg8(mFileHandleCompass, 0x08) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x07);
#define COMPASS_RANGE 0.15
	mMagnet.x = COMPASS_RANGE * mX;
	mMagnet.y = COMPASS_RANGE * mY;
	mMagnet.z = COMPASS_RANGE * mZ;
  wiringPiI2CReadReg8(mFileHandleCompass, 0x09);
  }
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
  else 
  {
    getFIFO(time);
  }
  if(wiringPiI2CReadReg8(mFileHandleCompass, 0x02)| 0x01)
  {
	short mX = (wiringPiI2CReadReg8(mFileHandleCompass, 0x04) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x03);
	short mY = (wiringPiI2CReadReg8(mFileHandleCompass, 0x06) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x05);
	short mZ = (wiringPiI2CReadReg8(mFileHandleCompass, 0x08) << 8) | wiringPiI2CReadReg8(mFileHandleCompass, 0x07);
#define COMPASS_RANGE 0.15
	mMagnet.x = COMPASS_RANGE * mX;
	mMagnet.y = COMPASS_RANGE * mY;
	mMagnet.z = COMPASS_RANGE * mZ;
  wiringPiI2CReadReg8(mFileHandleCompass, 0x09);
	calcMagnetOffset(mMagnet);

  }

  if(isMonitoring){
  Debug::print(LOG_SUMMARY, "\
  AccelNorm %f \
  Accel %f %f %f\
	Angle %f %f %f\
	Compass %f %f %f \r\n",
  mAccel.norm(),
  getAx() ,getAy(), getAz(),
	 getRx(), getRy(), getRz(),
	 getMagnetNorm(), getMagnetTheta(), getMagnetPhi());
  }
 
}
void NineAxisSensor::getFIFO(const struct timespec& time)
{
	 unsigned short fifo_count = (wiringPiI2CReadReg8(mFileHandle, 0x72) << 8) | wiringPiI2CReadReg8(mFileHandle, 0x73);
//   Debug::print(LOG_SUMMARY,"FIFO count %d\n",fifo_count); 
  if(fifo_count >=512)
  {
    onInit(time);
    return;
  }
	 VECTOR3 newRv;
	 VECTOR3 newA;
	 int data_samples = 0;
	 while(fifo_count >= 12)
	 {
		 short tmp[6];
		 VECTOR3 sampleRv;
		 for(int i = 0; i < 6 ;i++)
		 {
			 unsigned short high = (wiringPiI2CReadReg8(mFileHandle, 0x74) << 8) ;
			 tmp[i] = high | wiringPiI2CReadReg8(mFileHandle, 0x74);

		 }
		newA.x += AXEL_RANGE * tmp[0];
		newA.y += AXEL_RANGE * tmp[1];
		newA.z += AXEL_RANGE * tmp[2];
		sampleRv.x +=  GYRO_RANGE * tmp[3];
		sampleRv.y +=  GYRO_RANGE * tmp[4];
		sampleRv.z +=  GYRO_RANGE * tmp[5];
		newRv += sampleRv;

		//ドリフト誤差計算中であれば配列にデータを突っ込む
		if (mIsCalculatingOffset)
		{
			mRVelHistory.push_back(sampleRv);
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
        gBuzzer.start(10,2,10);
			}
		}

		//ドリフト誤差を補正
		newRv -= mRVelOffset;

		++data_samples;		

    fifo_count -= 12;
	 }
	if (data_samples != 0)
	{
		newA /= data_samples;
		mAccel = newA;
		mAccelAve = mAccel * mAccelAlpha + mAccelAve * (1 - mAccelAlpha);
		
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

bool NineAxisSensor::getAccel(VECTOR3& acc) const
{
	if (isActive())
	{
		acc = mAccelAve;
		return true;
	}
	return false;
}
double NineAxisSensor::getAx() const
{
	return mAccelAve.x;
}
double NineAxisSensor::getAy() const
{
	return mAccelAve.y;
}
double NineAxisSensor::getAz() const
{
	return mAccelAve.z;
}
double NineAxisSensor::getTheta() const
{
    return atan2f(mAccelAve.x, sqrt(pow(mAccelAve.y, 2) + pow(mAccelAve.z, 2)));
}
double NineAxisSensor::getPsi() const
{
    return atan2f(mAccelAve.y, sqrt(pow(mAccelAve.x, 2) + pow(mAccelAve.z, 2)));
}
double NineAxisSensor::getPhi() const
{
    return atan2f(sqrt(pow(mAccelAve.x, 2) + pow(mAccelAve.y, 2)), mAccelAve.z);
}
void NineAxisSensor::calcMagnetOffset(VECTOR3& newMagnet)
{
	//Calc Offset
	mMagnetMax.x = mMagnetMax.x > newMagnet.x ? mMagnetMax.x : newMagnet.x;
	mMagnetMax.y = mMagnetMax.y > newMagnet.y ? mMagnetMax.y : newMagnet.y;
	mMagnetMax.z = mMagnetMax.z > newMagnet.z ? mMagnetMax.z : newMagnet.z;

	mMagnetMin.x = mMagnetMin.x < newMagnet.x ? mMagnetMin.x : newMagnet.x;
	mMagnetMin.y = mMagnetMin.y < newMagnet.y ? mMagnetMin.y : newMagnet.y;
	mMagnetMin.z = mMagnetMin.z < newMagnet.z ? mMagnetMin.z : newMagnet.z;
}
double NineAxisSensor::getMagnetTheta()//z軸からの角度
{
	VECTOR3 magnet = mMagnet - ((mMagnetMax + mMagnetMin) / 2);
	return acos(magnet.z / magnet.norm())/M_PI*180;
}
double NineAxisSensor::getMagnetPhi()//ｘｙ平面に投影したときのｘ（北方向）軸からの角度、基本これを使う
{
	VECTOR3 magnet = mMagnet - ((mMagnetMax + mMagnetMin) / 2);
	return atan2(magnet.x,magnet.y)/M_PI *180;
}
double NineAxisSensor::getMagnetNorm()//地磁気の大きさ
{
	VECTOR3 magnet = mMagnet - ((mMagnetMax + mMagnetMin) / 2);
	return magnet.norm();	
}

bool NineAxisSensor::getRawAccel(VECTOR3& acc) const
{
	if(isActive())
	{
		acc = mAccel;
		return true;
	}
	return false;
}

bool NineAxisSensor::getRVel(VECTOR3& vel) const
{
	if (isActive())
	{
		vel = mRVel;
		return true;
	}
	return false;
}
double NineAxisSensor::getRvx() const
{
	return mRVel.x;
}
double NineAxisSensor::getRvy() const
{
	return mRVel.y;
}
double NineAxisSensor::getRvz() const
{
	return mRVel.z;
}
float NineAxisSensor::getRoll() const
{
	return mRoll;
}
float NineAxisSensor::getPitch() const
{
	return mPitch;
}
float NineAxisSensor::getYaw() const
{
	return mYaw;
}
bool NineAxisSensor::getMagnet(VECTOR3& mag) const
{
	if(isActive())
	{
		mag = mMagnet;
		return true;
	}
	return false;
}
double NineAxisSensor::getMx() const
{
	return mMagnet.x;
}
double NineAxisSensor::getMy() const
{
	return mMagnet.y;
}
double NineAxisSensor::getMz() const
{
	return mMagnet.z;
}
void NineAxisSensor::setMonitoring(bool val)
{
	isMonitoring = val;
	return;
}
double NineAxisSensor::normalize(double pos)
{
	while (pos >= 180 || pos < -180)pos += (pos > 0) ? -360 : 360;
	return pos;
}
void NineAxisSensor::normalize(VECTOR3& pos)
{
	pos.x = normalize(pos.x);
	pos.y = normalize(pos.y);
	pos.z = normalize(pos.z);
}
bool NineAxisSensor::getRPos(VECTOR3& pos) const
{
	if (isActive())
	{
		pos = mRAngle;
		return true;
	}
	return false;
}
double NineAxisSensor::getRx() const
{
	return mRAngle.x;
}
double NineAxisSensor::getRy() const
{
	return mRAngle.y;
}
double NineAxisSensor::getRz() const
{
	return mRAngle.z;
}
bool NineAxisSensor::onCommand(const std::vector<std::string>& args)
{
  if (args.size() == 3)
	{
		if (args[1].compare("monitor") == 0)
		{
			if(args[2].compare("true") == 0)
			{
				isMonitoring =true;
			}
			else if(args[2].compare("false") == 0)
			{
				isMonitoring = false;
			}
			return true;
		}
    if(args[1].compare("FIFO") ==0)
    {
      if(args[2].compare("true") == 0)
      {
        setFIFOmode(true);
      }
      if(args[2].compare("false") == 0)
      {
        setFIFOmode(false);
    
      }
      return true;
    }
		if (args[1].compare("cutoff") == 0)
		{
			mCutOffThreshold = atof(args[2].c_str());
			Debug::print(LOG_SUMMARY, "Gyro: cutoff threshold is %f\r\n", mCutOffThreshold);
			return true;
		}
		return false;
  }
  else if(args.size() == 2)
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
  else{
	Debug::print(LOG_SUMMARY, "Accel %f %f %f\r\n\
	Angle Velocity %f %f %f\r\n\
	Angle %f %f %f\r\n\
	Compass %f %f %f \r\n",getAx() ,getAy(), getAz(),
	 getRvx(), getRvy(), getRvz(),
	 getRx(), getRy(), getRz(),
	 getMagnetNorm(), getMagnetTheta(), getMagnetPhi());
	Debug::print(LOG_SUMMARY, "Usage:\r\n %s monitor [true/false] : enable/disable monitoring mode\r\n\
	nineaxis reset  : set angle to zero point\r\n\
  	nineaxis cutoff : set cutoff threshold\r\n\
  	nineaxis calib  : calibrate gyro *do NOT move*\r\n\
	nineaxis calib [x_offset] [y_offset] [z_offset] : calibrate gyro by specified params\r\n",args[0].c_str());
  }
  return true;
}
void NineAxisSensor::calibrate()
{
	mIsCalculatingOffset = true;	
}
void NineAxisSensor::setZero()
{
	mRAngle.x = mRAngle.y = mRAngle.z = 0;
}
void NineAxisSensor::setFIFOmode(bool val)
{
  isFIFOEnable = val;
}
NineAxisSensor::NineAxisSensor() : mFileHandle(-1),mAccel(), mAccelAve(), mAccelAlpha(0.5), mRVel(), mRAngle(), mMagnet(), mRVelHistory(), mRVelOffset(), mYaw(0), mPitch(0), mRoll(0)
{
  isMonitoring = false;
  isFIFOEnable = true;
  setName("nineaxis");
  setPriority(TASK_PRIORITY_SENSOR ,TASK_INTERVAL_SENSOR);
  mAccelAlpha = 1;
}
NineAxisSensor::~NineAxisSensor()
{
}
