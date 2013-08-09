/*
	モータをコントロールするためのクラス
*/

#pragma once
#include "task.h"


//モータ1個をコントロールするクラス
class Motor
{
private:
        // Pin Numbers
        int mPowerPin;
        int mReversePin;

        // Current Power
        double mCurPower;

		//出力変化用
		// 目標出力
		int mTargetPower;
		//出力変化量係数
		double mCoeff;
public:
        // Initialize Motor by 2 pins
        bool init(int powPin,int revPin);

		void clean();

		// モータ出力を更新(ratioはほかのモータとの個体差吸収用)
		void update(double elapsedSeconds);

        // Controle motor power. Negative value means reverse
		// range: [-MOTOR_MAX_POWER MOTOR_MAX_POWER]
        void set(int pow);

		//モータの出力を変化させる量(MOTOR_MAX_POWER_CHANGE)に掛けられる係数を設定
		//これによって左右のモータ間の個体差を吸収して、まっすぐに加速/減速ができる
		void setCoeff(double coeff);

        // Get Current power
        int getPower();

        Motor();
        ~Motor();
};

//エンコーダ2個を管理するクラス(シングルトン)
//wiringPiの実装の都合上、複数のインスタンスを作成すると正しく動作しません。

/////監視スレッドの管理が微妙だったり、コールバック関数に引数を取れないしでWiringPiの実装が微妙
class MotorEncoder
{
private:
	int mEncoderPinL, mEncoderPinR;//エンコーダのピン番号
	long long mPulseCountL, mPulseCountR;//パルス数

	MotorEncoder();

	static void pulseLCallback();//エンコーダーパルスが到着したときに呼ばれる割り込み関数
	static void pulseRCallback();
public:
	static MotorEncoder* getInstance();
	//パルスチェック開始
	bool init();
	//パルスチェック終了
	void clean();

	//モータのパルス数を返す
	long long getL();
	long long getR();
	
	~MotorEncoder();
};

//モータ2個とセンサーを組み合わせて走行するクラス
class MotorDrive : public TaskBase
{
private:
        Motor mMotorL,mMotorR;
		MotorEncoder* mpMotorEncoder;

		typedef enum{//制御モード
			DRIVE_RATIO, //レシオ指定で制御する
			DRIVE_PID,//PID制御による角度指定で制御する
			DRIVE_PID_TURN,
		}DRIVE_MODE;
		DRIVE_MODE mDriveMode;
		int mRatioL,mRatioR;//レシオ比

		//PID用パラメータ
		double mP,mI,mD;//PIDパラメタ
		double mDiff1,mDiff2,mDiff3;//PID用過去のずれ
		double mAngle;//目標角度
		double mControlPower;//前回の操作量
		int mDrivePower;//走行速度

		//最後にモータ出力を更新した時刻
		struct timespec mLastUpdateTime;
protected:
		//初期化
        virtual bool onInit(const struct timespec& time);
		virtual void onClean();
		//毎フレームの処理
		virtual void onUpdate(const struct timespec& time);

		//コマンド受付
		virtual bool onCommand(const std::vector<std::string> args);

		void updatePIDState();
		void updatePIDMove();
		void updatePIDTurn();
public:
		//モータの左右比を設定
		void setRatio(int ratioL,int ratioR);

		//指定された出力でモータを回転させる((100,100)の場合、実際のpwm出力はsetRatioに設定した値になる)
        void drive(int powerL,int powerR);

		//PID制御用パラメータを設定
		void set(double p,double i,double d);

		//PID制御を開始する(ジャイロのZ回転軸の角度が一定になるように制御する)
		//制御目標角度は、現在の向き+angle
		void startPID(double angle,int power);

		//PID制御の制御目標角度を変更する
		//制御目標角度は、現在の制御目標角度+angle
		void drivePID(double angle,int power);

		//PID制御によって指定角度にその場回転する
		void turnPID(double angle,int power);

        MotorDrive();
        ~MotorDrive();
};

extern MotorDrive gMotorDrive;
