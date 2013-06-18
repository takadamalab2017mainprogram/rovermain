/*
	センサ制御プログラム

	モータ以外の実世界から情報を取得するモジュールを操作します
	task.hも参照
*/
#pragma once
#include "task.h"

typedef struct
{
	double x,y,z;
}VECTOR3;

//MPL115A2からデータを取得するクラス
//気圧の値はhPa単位で+-10hPaの誤差あり
class PressureSensor : public TaskBase
{
private:
	float mA0,mB1,mB2,mC12;//気圧計算用の係数
	int mPressure;//最後に取得した気圧
	int mFileHandle;//winringPi i2c　のファイルハンドラ

	struct timespec mLastUpdateRequest;//最後に気圧の更新をMPL115A2に指示した時刻

	float val2float(unsigned int val, int total_bits, int fractional_bits, int zero_pad);
	void requestSample();
protected:
	//気圧センサを初期化
	virtual bool init();
	//センサの使用を終了する
	virtual void clean();

	//一定間隔ごとに気圧をアップデートする
	virtual void update();

	//コマンドを処理する
	virtual bool command(const std::vector<std::string> args);
public:
	//最後にアップデートされた気圧を返す
	int get();

	PressureSensor();
	~PressureSensor();
};

//Navigatron v2からデータを取得するクラス
class GPSSensor : public TaskBase
{
private:
	int mFileHandle;//winringPi i2c　のファイルハンドラ
	VECTOR3 mPos;//座標(経度、緯度、高度)
	int mSatelites;//補足した衛星の数
protected:
	//GPSを初期化
	virtual bool init();
	//センサの使用を終了する
	virtual void clean();
	//現在の座標をアップデートする
	virtual void update();
	//コマンドを処理する
	virtual bool command(const std::vector<std::string> args);

public:
	//現在の座標を取得する(falseを返した場合は場所が不明)
	bool get(VECTOR3& pos);

	GPSSensor();
	~GPSSensor();
};

//L3GD20からデータを取得するクラス
class GyroSensor : public TaskBase
{
private:
	int mFileHandle;//winringPi i2c　のファイルハンドラ
	VECTOR3 mRVel;//角速度
	VECTOR3 mRAngle;//角度
	struct timespec mLastSampleTime;
protected:
	//ジャイロセンサを初期化
	virtual bool init();
	//センサの使用を終了する
	virtual void clean();

	//一定間隔ごとにデータをアップデートする
	virtual void update();

	//コマンドを処理する
	virtual bool command(const std::vector<std::string> args);
public:
	//最後にアップデートされたデータを返す
	void getRVel(VECTOR3& vel);
	double getRvx();
	double getRvy();
	double getRvz();

	//////////////////////////////////////////////////
	//角速度から計算された角度を処理する関数

	//現在の角度を基準とする
	void setZero();

	//現在の角度を返す(-180～+180)
	void getRPos(VECTOR3& pos);
	double getRx();
	double getRy();
	double getRz();

	//引数のベクトルを(-180～+180)の範囲に修正
	static void normalize(VECTOR3& pos);
	static double normalize(double pos);

	GyroSensor();
	~GyroSensor();
};

//Cdsからデータを取得するクラス
class LightSensor : public TaskBase
{
private:
	int mPin;
protected:
	//初期化
	virtual bool init();
	//センサの使用を終了する
	virtual void clean();
	//コマンドを処理する
	virtual bool command(const std::vector<std::string> args);

public:
	//現在の明るさを取得する
	bool get();

	LightSensor();
	~LightSensor();
};
extern GyroSensor gGyroSensor;
extern GPSSensor gGPSSensor;
extern PressureSensor gPressureSensor;
extern LightSensor gLightSensor;

