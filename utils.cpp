#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <iterator>
#include <sstream>
#include <iostream>
#include <math.h>
#include <float.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include "utils.h"

void Debug::print(LOG_LEVEL level, const char* fmt, ...)
{
	static std::string mFilename;
	if (mFilename.length() == 0)
	{
		Filename filename("log", ".txt");
		filename.get(mFilename);
	}
#ifndef _LOG_DETAIL
	if (level == LOG_DETAIL)return; //デバッグモードでなければログ出力しない
#endif

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);
  timespec nowtime;
  clock_gettime(CLOCK_REALTIME,&nowtime);
  char timebuf[MAX_STRING_LENGTH];
  sprintf(timebuf,"%ld.%ld:",nowtime.tv_sec,nowtime.tv_nsec);

	//画面に出力
	printf(buf);

	//ログファイルに出力
	if (level != LOG_PRINT)
	{
		std::ofstream of(mFilename.c_str(), std::ios::out | std::ios::app);
    of << timebuf;
		of << buf;
	}
}
void Filename::get(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << ++mIndex << mSuffix;
	name.assign(filename.str());
}
void Filename::getNow(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << mIndex << mSuffix;
	name.assign(filename.str());
}
void Filename::getNoIndex(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << mSuffix;
	name.assign(filename.str());
}
Filename::Filename(const std::string& prefix, const std::string& suffix) : mPrefix(prefix), mSuffix(suffix), mIndex(0)
{
	//撮影インデックスを既存のファイルに上書きしないように変更
	std::string filename;
	struct stat st;
	do
	{
		get(filename);
	} while (stat(filename.c_str(), &st) == 0);
	--mIndex;
}
double Time::dt(const struct timespec& now, const struct timespec& last)
{
	return ((double)(now.tv_sec - last.tv_sec) * 1000000000 + now.tv_nsec - last.tv_nsec) / 1000000000.0;
}
void Time::get(struct timespec& time)
{
	if(clock_gettime(CLOCK_MONOTONIC_RAW,&time) != 0)
	{
		Debug::print(LOG_DETAIL, "FAILED to get time!\r\n");
	}
}
void Time::showNowTime()
{
	time_t now;
	struct tm *ts;
	now = time(NULL);
	ts = localtime(&now);

	Debug::print(LOG_SUMMARY, "Time --> %d:%d:%d\r\n", ts->tm_hour, ts->tm_min, ts->tm_sec);
}
void String::split(const std::string& input, std::vector<std::string>& outputs)
{
	//文字列を空白文字で分割してvectorに格納
	outputs.clear();
	std::istringstream iss(input);
	std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(outputs));
}
void ConstantManager::add(unsigned int index, const char* name, double value)
{
	if (mData.count(index) != 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d  is already exist!\r\n", index);
		return;
	}
	struct CONSTANT constant = { name, value };
	mData[index] = constant;
}
double& ConstantManager::operator[](int index)
{
	static double error = DBL_MIN;
	if (mData.count(index) == 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d not found!\r\n", index);
		return error;
	}
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.find(index);
	return it->second.value;
}
double& ConstantManager::operator[](const char* name)
{
	static double error = DBL_MIN;
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.begin();
	while (it != mData.end())
	{
		if (it->second.name.compare(name) == 0)
		{
			return it->second.value;
		}
		++it;
	}
	Debug::print(LOG_SUMMARY, "Constant %s not found!\r\n", name);
	return error;
}

void ConstantManager::save(const char* filename)
{
	if (filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ofstream of(filename, std::ios::out);
	std::map<unsigned int, struct CONSTANT>::iterator it = mData.begin();
	while (it != mData.end())
	{
		of << it->first << " " << it->second.name << " " << it->second.value << std::endl;
		++it;
	}
}
void ConstantManager::load(const char* filename)
{
	if (filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ifstream ifs(filename, std::ios::in);
	std::string str;
	if (ifs.good())
	{
		Debug::print(LOG_SUMMARY, "Reading %s\r\n", filename);

		//行ごとに読み込んで設定を読み込む
		while (!ifs.eof() && !ifs.fail() && !ifs.bad())
		{
			std::getline(ifs, str);
			std::vector<std::string> str_constant;
			String::split(str, str_constant);

			if (str_constant.size() != 3)
			{
				Debug::print(LOG_SUMMARY, "Constant: parse error\r\n");
				continue;
			}

			add(atoi(str_constant[0].c_str()), str_constant[1].c_str(), atof(str_constant[2].c_str()));
		}
	}
}
ConstantManager& ConstantManager::get()
{
	static ConstantManager instance;
	return instance;
}
ConstantManager::ConstantManager() : mData()
{
}
ConstantManager::~ConstantManager()
{
}
float KalmanFilter::update(float newAngle, float newRate, float dt)
{
	mXAngle += dt * (newRate - mXBias);
	mP[0][0] += dt * (mQAngle - mP[1][0] - mP[0][1]);
	mP[0][1] += -dt * mP[1][1];
	mP[1][0] += -dt * mP[1][1];
	mP[1][1] += mQBias * dt;

	float y = newAngle - mXBias;
	float s = mP[0][0] + mRAngle;
	float K[] = {mP[0][0] / s, mP[1][0] / s};

	mXAngle += K[0] * y;
	mXBias += K[1] * y;

	mP[0][0] -= K[0] * mP[0][0];
	mP[0][1] -= K[0] * mP[0][1];
	mP[1][0] -= K[1] * mP[0][0];
	mP[1][1] -= K[1] * mP[0][1];

	return mXAngle;
}
KalmanFilter::KalmanFilter() : mQAngle(0.001f), mQBias(0.003f), mRAngle(0.03f), mXAngle(0), mXBias(0), mP{{0}}
{}
KalmanFilter::~KalmanFilter()
{}

double VECTOR3::calcAngleXY(const VECTOR3& current, const VECTOR3& target)
{
	return atan2(target.y - current.y, target.x - current.x) / M_PI * 180;
}
double VECTOR3::calcDistanceXY(const VECTOR3& current, const VECTOR3& target)
{
	VECTOR3 dif = current - target;
	return sqrt(pow(dif.x, 2) + pow(dif.y, 2));
}

VECTOR3 VECTOR3::operator+() const
{
	return *this;
}
VECTOR3 VECTOR3::operator-() const
{
	return VECTOR3(-x, -y, -z);
}
VECTOR3& VECTOR3::operator+=(const VECTOR3& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}
VECTOR3& VECTOR3::operator-=(const VECTOR3& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}
VECTOR3 VECTOR3::operator+(const VECTOR3& v) const
{
	return VECTOR3(x + v.x, y + v.y, z + v.z);
}
VECTOR3 VECTOR3::operator-(const VECTOR3& v) const
{
	return VECTOR3(x - v.x, y - v.y, z - v.z);
}
VECTOR3 VECTOR3::operator+(const double v) const
{
	return VECTOR3(x + v, y + v, z + v);
}
VECTOR3 VECTOR3::operator-(const double v) const
{
	return VECTOR3(x - v, y - v, z - v);
}
VECTOR3& VECTOR3::operator*=(const double v)
{
	x *= v;
	y *= v;
	z *= v;
	return *this;
}
VECTOR3& VECTOR3::operator/=(const double v)
{
	x /= v;
	y /= v;
	z /= v;
	return *this;
}
VECTOR3 VECTOR3::operator*(const double v) const
{
	return VECTOR3(x * v, y * v, z * v);
}
VECTOR3 VECTOR3::operator/(const double v) const
{
	return VECTOR3(x / v, y / v, z / v);
}
bool VECTOR3::operator==(const VECTOR3& v) const
{
	return (x == v.x) && (y == v.y) && (z == v.z);
}
bool VECTOR3::operator!=(const VECTOR3& v) const
{
	return (x != v.x) || (y != v.y) || (z != v.z);
}
double VECTOR3::norm()
{
  return sqrt(x*x + y*y + z*z);
}
VECTOR3 VECTOR3::normalize() const
{
	double length = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
	if(length == 0)
	{
		return VECTOR3(0,0,1);
	}else
	{
		return *this / length;
	}
}

VECTOR3::VECTOR3() : x(0),y(0),z(0){}
VECTOR3::VECTOR3(double tx, double ty, double tz) : x(tx),y(ty),z(tz){}

QUATERNION::QUATERNION() : x(0), y(0), z(0), w(1)
{
}

QUATERNION::QUATERNION(double tx, double ty, double tz, double tw) : x(tx), y(ty), z(tz), w(tw)
{
}

QUATERNION::QUATERNION(double tx, double ty, double tz)
{
	double angle;

	angle = tx * 0.5;
	const double sr = sin(angle);
	const double cr = cos(angle);

	angle = ty * 0.5;
	const double sp = sin(angle);
	const double cp = cos(angle);

	angle = tz * 0.5;
	const double sy = sin(angle);
	const double cy = cos(angle);

	const double cpcy = cp * cy;
	const double spcy = sp * cy;
	const double cpsy = cp * sy;
	const double spsy = sp * sy;

	x = sr * cpcy - cr * spsy;
	y = cr * spcy + sr * cpsy;
	z = cr * cpsy - sr * spcy;
	w = cr * cpcy + sr * spsy;

	normalize();
}

QUATERNION::QUATERNION(const VECTOR3 v)
{
	double angle;

	angle = x * 0.5;
	const double sr = sin(angle);
	const double cr = cos(angle);

	angle = y * 0.5;
	const double sp = sin(angle);
	const double cp = cos(angle);

	angle = z * 0.5;
	const double sy = sin(angle);
	const double cy = cos(angle);

	const double cpcy = cp * cy;
	const double spcy = sp * cy;
	const double cpsy = cp * sy;
	const double spsy = sp * sy;

	x = (double)(sr * cpcy - cr * spsy);
	y = (double)(cr * spcy + sr * cpsy);
	z = (double)(cr * cpsy - sr * spcy);
	w = (double)(cr * cpcy + sr * spsy);

	normalize();
}
QUATERNION QUATERNION::operator+() const
{
	return QUATERNION(x, y, z, w);
}
QUATERNION QUATERNION::operator-() const
{
	return QUATERNION(-x, -y, -z, -w);
}
QUATERNION& QUATERNION::operator+=(const QUATERNION& q)
{
	x += q.x;
	y += q.y;
	z += q.z;
	w += q.w;
	return *this;
}
QUATERNION& QUATERNION::operator-=(const QUATERNION& q)
{
	x -= q.x;
	y -= q.y;
	z -= q.z;
	w -= q.w;
	return *this;
}
QUATERNION QUATERNION::operator+(const QUATERNION& q) const
{
	return QUATERNION(x + q.x, y + q.y, z + q.z, w + q.w);
}
QUATERNION QUATERNION::operator-(const QUATERNION& q) const
{
	return QUATERNION(x - q.x, y - q.y, z - q.z, w - q.w);
}
QUATERNION QUATERNION::operator*(const QUATERNION& q) const
{
	QUATERNION tmp;

	tmp.w = (q.w * w) - (q.x * x) - (q.y * y) - (q.z * z);
	tmp.x = (q.w * x) + (q.x * w) + (q.y * z) - (q.z * y);
	tmp.y = (q.w * y) + (q.y * w) + (q.z * x) - (q.x * z);
	tmp.z = (q.w * z) + (q.z * w) + (q.x * y) - (q.y * x);

	return tmp;
}
QUATERNION& QUATERNION::operator *=(const double q)
{
	return (*this = (*this) * q);
}
QUATERNION QUATERNION::operator*(const double q) const
{
	QUATERNION tmp(x * q, y * q, z * q, w * q);
	return tmp;
}
bool QUATERNION::operator==(const QUATERNION& q)const
{
	return (x == q.x && y == q.y && z == q.z && w == q.w);
}
bool QUATERNION::operator!=(const QUATERNION& q)const
{
	return !(*this == q);
}
QUATERNION& QUATERNION::fromAngleAxis(double angle, const VECTOR3& axis)
{
	const double halfAngle = 0.5f*angle;
	const double sinv = sinf(halfAngle);
	w = cosf(halfAngle);
	x = sinv*axis.x;
	y = sinv*axis.y;
	z = sinv*axis.z;
	return *this;
}
double QUATERNION::toAngleAxis(VECTOR3& v) const
{
	const double scale = sqrtf(pow(x,2) + pow(y,2) + pow(z,2));
	double angle;

	if (scale == 0 || w > 1.0f || w < -1.0f)
	{
		angle = 0.0f;
		v.x = 0.0f;
		v.y = 1.0f;
		v.z = 0.0f;
	}
	else
	{
		const double invscale = 1 / scale;
		angle = 2.0f * acosf(w);
		v.x = x * invscale;
		v.y = y * invscale;
		v.z = z * invscale;
	}
	return angle;
}
void QUATERNION::toEulerZYX(VECTOR3& rpy) const
{
	rpy.x = atan2f(2.0*(w*x + y*z), 1 - 2 * (x*x - y*y));
	rpy.y = asin(2.0*(w*y - z*x));
	rpy.z = atan2f(2.0*(w*z + x*y), 1 - 2 * (y*y + z*z));
}
void QUATERNION::toEulerXYZ(VECTOR3& rpy) const
{
	rpy.x = atan2f(2.0*(x*w - y*z), 1 - 2 * (x*x - y*y));
	rpy.y = asin(2.0*(x*z + w*y));
	rpy.z = atan2f(2.0*(z*w - x*y), 1 - 2 * (y*y + z*z));
}
double QUATERNION::getRoll() const
{
	QUATERNION tmp = *this;
	tmp.y = tmp.z = 0;
	tmp = tmp.normalize();
	return 2 * acos(tmp.w);
}
double QUATERNION::getPitch() const
{
	QUATERNION tmp = *this;
	tmp.z = tmp.x = 0;
	tmp = tmp.normalize();
	return 2 * acos(tmp.w);
}
double QUATERNION::getYaw() const
{
	QUATERNION tmp = *this;
	tmp.x = tmp.y = 0;
	tmp = tmp.normalize();
	return 2 * acos(tmp.w);
}
QUATERNION QUATERNION::normalize() const
{
	const double n = pow(x,2) + pow(y,2) + pow(z,2) + pow(w,2);

	if (n == 1)return *this;

	double rootn = sqrt(n);
	if(rootn == 0)
	{
		return QUATERNION();
	}
	QUATERNION ret = *this;
	return ret * (1/rootn);
}

QUATERNION QUATERNION::inverse() const
{
	QUATERNION tmp(-x, -y, -z, w);
	return tmp;
}
