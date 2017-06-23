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
#include <time.h>
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
	static void print(LOG_LEVEL level, const char* fmt, ...);//ストリーム面倒だからprintfタイプでいいよね
	Debug();
};

class String
{
public:
	//文字列を空白で分割
	static void split(const std::string& input, std::vector<std::string>& outputs);
};

class Filename
{
	std::string mPrefix, mSuffix;
	unsigned int mIndex;
public:
	void get(std::string& name);
	void getNow(std::string& name);
	void getNoIndex(std::string& name);
	Filename(const std::string& prefix, const std::string& suffix);
};

//定数マネージャ
class ConstantManager
{
	ConstantManager();
	struct CONSTANT { std::string name; double value; };
	std::map<unsigned int, struct CONSTANT> mData;
public:
	static ConstantManager& get();

	void add(unsigned int index, const char* name, double value = 0);

	double& operator[](int index);
	double& operator[](const char* name);

	void save(const char* filename);
	void load(const char* filename);

	~ConstantManager();
};

/*
 * Timespec
 */
#ifndef __timespec_defined
#define __timespec_defined
struct timespec {
  time_t  tv_sec;   // Seconds
  long    tv_nsec;  // Nanoseconds
};
#endif

class Time
{
public:
    //時間の変化量を計算(秒)
    static double dt(const struct timespec& now,const struct timespec& last);
	//現在時刻を取得
    static void get(struct timespec& time);
	//現在時刻をログに出力する
	static void showNowTime();
};

class KalmanFilter
{
private:
	float mQAngle, mQBias, mRAngle;
	float mXAngle, mXBias;
	float mP[2][2];
public:
	float update(float newAngle, float newRate, float dt);

	KalmanFilter();
	virtual ~KalmanFilter();
};

class VECTOR3
{
public:
	double x, y, z;

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
	static double calcAngleXY(const VECTOR3& current, const VECTOR3& target);
	//2点間の距離を計算
	static double calcDistanceXY(const VECTOR3& current, const VECTOR3& target);
	//normalize length to 1
	VECTOR3 normalize() const;
};

//3Dの角度を扱うときに便利なクラス
// 参考: https://svn.code.sf.net/p/irrlicht/code/trunk/include/quaternion.h
class QUATERNION
{
public:
	double x, y, z, w;

	//単位クォータニオンを生成
	QUATERNION();
	
	//各要素の値で初期化
	QUATERNION(double x, double y, double z, double w);

	//オイラー角で初期化
	QUATERNION(double x, double y, double z);
	QUATERNION(VECTOR3 v);

	QUATERNION operator+() const;
	QUATERNION operator-() const;
	QUATERNION& operator+=(const QUATERNION& q);
	QUATERNION& operator-=(const QUATERNION& q);
	QUATERNION operator+(const QUATERNION& q) const;
	QUATERNION operator-(const QUATERNION& q) const;
	QUATERNION& operator*=(const QUATERNION& q);
	QUATERNION operator*(const QUATERNION& q) const;
	QUATERNION& operator*=(const double q);
	QUATERNION operator*(const double q) const;

	bool operator==(const QUATERNION& q)const;
	bool operator!=(const QUATERNION& q)const;

	//ある軸周りに回転させたクォータニオンを取得
	QUATERNION& fromAngleAxis(double angle, const VECTOR3& axis);
	//ある軸周りの回転角度を導出
	double toAngleAxis(VECTOR3& v) const;
	void toEulerXYZ(VECTOR3& euler) const;
	void toEulerZYX(VECTOR3& euler) const;
	double getRoll() const;
	double getPitch() const;
	double getYaw() const;
	QUATERNION normalize() const;
	QUATERNION inverse() const;
};
