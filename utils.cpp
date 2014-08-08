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
#include "utils.h"

void Debug::print(LOG_LEVEL level, const char* fmt, ... )
{
	static std::string mFilename;
	if(mFilename.length() == 0)
	{
		Filename filename("log",".txt");
		filename.get(mFilename);
	}
#ifndef _LOG_DETAIL
	if(level == LOG_DETAIL)return; //デバッグモードでなければログ出力しない
#endif

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);
	
	//画面に出力
	printf(buf);
	//ログファイルに出力
	if(level != LOG_PRINT)
	{
		std::ofstream of(mFilename.c_str(),std::ios::out | std::ios::app);
		of << buf;
	}
}
void Filename::get(std::string& name)
{
	std::stringstream filename;
	filename << mPrefix << ++mIndex << mSuffix;
	name.assign(filename.str());
}
Filename::Filename(const std::string& prefix,const std::string& suffix) : mPrefix(prefix),mSuffix(suffix),mIndex(0)
{
	//撮影インデックスを既存のファイルに上書きしないように変更
	std::string filename;
	struct stat st;
	do
	{
		get(filename);
	}while(stat(filename.c_str(), &st) == 0);
	--mIndex;
}
double Time::dt(const struct timespec& now,const struct timespec& last)
{
	return ((double)(now.tv_sec - last.tv_sec) * 1000000000 + now.tv_nsec - last.tv_nsec) / 1000000000.0;
}
void Time::showNowTime()
{
	time_t now;
	struct tm *ts;
	now = time(NULL);
	ts = localtime(&now);

	//日本時間とラズパイ時刻の差を調整
	int min = ts->tm_hour * 60 + ts->tm_min + 1440;	//ラズパイ時刻を秒に変換
	min -= 410;										//日本時間との差を引く
	if(min >= 1440) min -= 1440;
	
	int h = min / 60, m = min % 60;

	Debug::print(LOG_SUMMARY,"Time: %d:%d:%d\r\n",h,m,ts->tm_sec);
}
void String::split(const std::string& input,std::vector<std::string>& outputs)
{
	//文字列を空白文字で分割してvectorに格納
	outputs.clear();
	std::istringstream iss(input);
	std::copy(std::istream_iterator<std::string>(iss),  std::istream_iterator<std::string>(), std::back_inserter(outputs));
}
void ConstantManager::add(unsigned int index,const char* name,double value)
{
	if(mData.count(index) != 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d  is already exist!\r\n",index);
		return;
	}
	struct CONSTANT constant = {name,value};
	mData[index] = constant;
}
double& ConstantManager::operator[](int index)
{
	static double error = DBL_MIN;
	if(mData.count(index) == 0)
	{
		Debug::print(LOG_SUMMARY, "Constant %d not found!\r\n",index);
		return error;
	}
	std::map<unsigned int,struct CONSTANT>::iterator it = mData.find(index);
	return it->second.value;
}
double& ConstantManager::operator[](const char* name)
{
	static double error = DBL_MIN;
	std::map<unsigned int,struct CONSTANT>::iterator it = mData.begin();
	while(it != mData.end())
	{
		if(it->second.name.compare(name) == 0)
		{
			return it->second.value;
		}
		++it;
	}
	Debug::print(LOG_SUMMARY, "Constant %s not found!\r\n",name);
	return error;
}

void ConstantManager::save(const char* filename)
{
	if(filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ofstream of(filename,std::ios::out);
	std::map<unsigned int,struct CONSTANT>::iterator it = mData.begin();
	while(it != mData.end())
	{
		of << it->first << " " << it->second.name << " " << it->second.value << std::endl;
		++it;
	}
}
void ConstantManager::load(const char* filename)
{
	if(filename == NULL)
	{
		Debug::print(LOG_SUMMARY, "Constant: null po\r\n");
		return;
	}
	std::ifstream ifs(filename,std::ios::in);
	std::string str;
	if(ifs.good())
	{
		Debug::print(LOG_SUMMARY, "Reading %s\r\n",filename);
		
		//行ごとに読み込んで設定を読み込む
		while(!ifs.eof() && !ifs.fail() && !ifs.bad())
		{
			std::getline(ifs,str);
			std::vector<std::string> str_constant;
			String::split(str,str_constant);

			if(str_constant.size() != 3)
			{
				Debug::print(LOG_SUMMARY, "Constant: parse error\r\n");
				continue;
			}

			add(atoi(str_constant[0].c_str()),str_constant[1].c_str(), atof(str_constant[2].c_str()));
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

double VECTOR3::calcAngleXY(const VECTOR3& current,const VECTOR3& target)
{
	return atan2(target.y - current.y,target.x - current.x) / M_PI * 180;
}
double VECTOR3::calcDistanceXY(const VECTOR3& current,const VECTOR3& target)
{
	VECTOR3 dif = current - target;
	return sqrt(pow(dif.x,2) + pow(dif.y,2));
}

VECTOR3 VECTOR3::operator+() const
{
	return *this;
}
VECTOR3 VECTOR3::operator-() const
{
	return VECTOR3(-x,-y,-z);
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
	return VECTOR3(x + v.x,y + v.y,z + v.z);
}
VECTOR3 VECTOR3::operator-(const VECTOR3& v) const
{
	return VECTOR3(x - v.x,y - v.y,z - v.z);
}
VECTOR3 VECTOR3::operator+(const double v) const
{
	return VECTOR3(x + v,y + v,z + v);
}
VECTOR3 VECTOR3::operator-(const double v) const
{
	return VECTOR3(x - v,y - v,z - v);
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
	return VECTOR3(x * v,y * v,z * v);
}
VECTOR3 VECTOR3::operator/(const double v) const
{
	return VECTOR3(x / v,y / v,z / v);
}
bool VECTOR3::operator==(const VECTOR3& v) const
{
	return (x == v.x) && (y == v.y) && (z == v.z);
}
bool VECTOR3::operator!=(const VECTOR3& v) const
{
	return (x != v.x) || (y != v.y) || (z != v.z);
}

VECTOR3::VECTOR3() : x(0),y(0),z(0){}
VECTOR3::VECTOR3(double tx, double ty, double tz) : x(tx),y(ty),z(tz){}
