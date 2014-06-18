/*
	その他関数など

	デバッグ用マクロやprint関数を用意してあります
	・print関数は画面とファイル両方に出力します
	・ログレベルは重要ではないログで画面が埋め尽くされないように設定します
	・staticクラスのため単純にDebug::print()のように呼び出してください
*/

#pragma once
#include <vector>
#include <string>
#include <map>
#include "constants.h"

#ifdef _DEBUG
	#include <assert.h>
	//xが0ならabort
	#define ASSERT(x) assert(x);
	//xが非0ならabort
	#define VERIFY(x) assert(!(x));
#else
	#define ASSERT(x)
	#define VERIFY(x)
#endif

typedef enum
{
	LOG_DETAIL = 0,	//デバッグログ(バグが出たときの状況確認用)
	LOG_SUMMARY,	//ファイル保存、画面表示共にするもの
	LOG_PRINT		//画面にのみ表示するもの
}LOG_LEVEL;			//ログレベル(Apacheとかと似た感じで)

const static unsigned int MAX_STRING_LENGTH = 1024;//Print用のバッファサイズ

class Debug
{
public:
	static void print(LOG_LEVEL level, const char* fmt, ... );//ストリーム面倒だからprintfタイプでいいよね
	Debug();
};

class Time
{
public:
	//時間の変化量を計算(秒)
	static double dt(const struct timespec& now,const struct timespec& last);
};

class String
{
public:
	//文字列を空白で分割
	static void split(const std::string& input,std::vector<std::string>& outputs);
};

class Filename
{
	std::string mPrefix,mSuffix;
	unsigned int mIndex;
public:
	void get(std::string& name);
	Filename(const std::string& prefix,const std::string& suffix);
};

//定数マネージャ
class ConstantManager
{
	ConstantManager();
	struct CONSTANT {std::string name; double value;};
	std::map<unsigned int,struct CONSTANT> mData;
public:
	static ConstantManager& get();

	void add(unsigned int index, const char* name, double value = 0);

	double& operator[](int index);
	double& operator[](const char* name);

	void save(const char* filename);
	void load(const char* filename);

	~ConstantManager();
};

class VECTOR3
{
public:
	double x,y,z;

	VECTOR3 operator+() const;
	VECTOR3 operator-() const;
	VECTOR3& operator+=(const VECTOR3& v);
	VECTOR3& operator-=(const VECTOR3& v);
	VECTOR3 operator+(const VECTOR3& u) const;
	VECTOR3 operator-(const VECTOR3& u) const;
	VECTOR3 operator+(const double v) const;
	VECTOR3 operator-(const double v) const;
	VECTOR3& operator*=(const double v);
	VECTOR3& operator/=(const double v);
	VECTOR3 operator*(const double v) const;
	VECTOR3 operator/(const double v) const;
	bool operator==(const VECTOR3& v)const;
	bool operator!=(const VECTOR3& v)const;

	VECTOR3();
	VECTOR3(double tx, double ty, double tz);

	//XY平面状の2点を結ぶ直線の角度(北が0度で+180度(東)〜-180度(西))。ジャイロの角度とは正負が逆です
	static double calcAngleXY(const VECTOR3& current,const VECTOR3& target);
	//2点間の距離を計算
	static double calcDistanceXY(const VECTOR3& current,const VECTOR3& target);
};
