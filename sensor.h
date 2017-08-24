/*
	センサ制御プログラム

	モータ以外の実世界から情報を取得するモジュールを操作します
	task.hも参照
	*/
#pragma once
#include "task.h"
#include "utils.h"
#include <pthread.h>
#include <list>
#include <libgpsmm.h>

//MPL115A2からデータを取得するクラス
//気圧の値はhPa単位で+-10hPaの誤差が存在
class PressureSensor : public TaskBase
{
private:
	float mA0, mB1, mB2, mC12;//気圧計算用の係数
	float mPressure, mTemperature;//最後に取得した気圧
	int mFileHandle;//winringPi i2c　のファイルハンドラ

	struct timespec mLastUpdateRequest;//最後に気圧の更新をMPL115A2に指示した時刻

	float val2float(unsigned int val, int total_bits, int fractional_bits, int zero_pad);
	void requestSample();
protected:
	//気圧センサを初期化
	virtual bool onInit(const struct timespec& time);
	//センサの使用を終了する
	virtual void onClean();

	//一定間隔ごとに気圧をアップデートする
	virtual void onUpdate(const struct timespec& time);

	//コマンドを処理する
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	//最後にアップデートされた気圧を返す
	float get() const;
	float getTemperature() const;

	PressureSensor();
	~PressureSensor();
};

//Navigatron v2からデータを取得するクラス
class GPSSensor : public TaskBase
{
private:
	struct timespec mLastCheckTime;//前回のチェック時刻
	int mFileHandle;//winringPi i2c　のファイルハンドラ
	VECTOR3 mPos;//座標(経度、緯度、高度)
	int mSatelites;//補足した衛星の数
	int mGpsTime;
	float mGpsSpeed;
	float mGpsCourse;
	bool mIsNewData;//新しい座標データがあれば真
	bool mIsLogger;//真なら1秒ごとにgpsコマンドを実行
	gpsmm gps_rec;
	struct gps_data_t *newdata;

	void showState() const;//補足した衛星数と座標を表示
protected:
	//GPSを初期化
	virtual bool onInit(const struct timespec& time);
	//センサの使用を終了する
	virtual void onClean();
	//現在の座標をアップデートする
	virtual void onUpdate(const struct timespec& time);
	//コマンドを処理する
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	//現在の座標を取得する(falseを返した場合は場所が不明)
	//disableNewFlagをfalseにすると座標が新しいという情報を削除
	bool get(VECTOR3& pos, bool disableNewFlag = false);

	//前回の座標取得以降にデータが更新された場合は真
	bool isNewPos() const;
	int getTime() const;
	float getCourse() const;
	float getSpeed() const;

	GPSSensor();
	~GPSSensor();
};

//Cdsからデータを取得するクラス
class LightSensor : public TaskBase
{
private:
	int mPin;
protected:
	//初期化
	virtual bool onInit(const struct timespec& time);
	//センサの使用を終了する
	virtual void onClean();
	//コマンドを処理する
	virtual bool onCommand(const std::vector<std::string>& args);

public:
	//現在の明るさを取得する
	bool get() const;

	LightSensor();
	~LightSensor();
};

class NineAxisSensor : public TaskBase
{
private:
	int mFileHandle,mFileHandleCompass;
	VECTOR3 mAccel, mAccelAve;
	double mAccelAlpha;
	VECTOR3 mRVel, mRAngle;
	VECTOR3 mMagnet;
	struct timespec mLastSampleTime;
	//ドリフト誤差補正用
	std::list<VECTOR3> mRVelHistory;//過去の角速度
	VECTOR3 mRVelOffset;//サンプルあたりのドリフト誤差の推定値
	double mCutOffThreshold;
	bool mIsCalculatingOffset;//ドリフト誤差計算中フラグ
	float mYaw;
	float mPitch;
	float mRoll;
	bool isFIFOEnable;
	VECTOR3 mMagnetMax;
	VECTOR3 mMagnetMin;
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual void onUpdate(const struct timespec& time);
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	bool getAccel(VECTOR3& acc) const;
	double getAx() const;
	double getAy() const;
	double getAz() const;

	double getTheta() const; //XY
	double getPsi() const; //YZ
	double getPhi() const; //XZ

	//現在の角度を返す(-180〜+180)
	bool getRPos(VECTOR3& pos) const;
	double getRx() const;
	double getRy() const;
	double getRz() const;

	bool getRVel(VECTOR3& vel) const;
	double getRvx() const;
	double getRvy() const;
	double getRvz() const;
	bool getRawAccel(VECTOR3& acc)const;
	//現在の角度を基準とする
	void setZero();

	//引数のベクトルを(-180〜+180)の範囲に修正
	static void normalize(VECTOR3& pos);
	static double normalize(double pos);

	bool getMagnet(VECTOR3& mag) const;
	double getMx() const;
	double getMy() const;
	double getMz() const;

	float getRoll() const;
	float getPitch() const;
	float getYaw() const;
	bool isMonitoring;
	void getFIFO(const struct timespec& time);
  void setMonitoring(bool val);
  void calibrate();
  void setFIFOmode(bool val);
	void calcMagnetOffset(VECTOR3& newMagnet);
	double getMagnetTheta();
	double getMagnetPhi();
	double getMagnetNorm();
	NineAxisSensor();
	~NineAxisSensor();
};

extern GPSSensor gGPSSensor;
extern PressureSensor gPressureSensor;
extern LightSensor gLightSensor;
extern NineAxisSensor gNineAxisSensor;
