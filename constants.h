/*
	各種定数
*/
#pragma once

//本番はコメントアウトすること！！（Ctrl-Cによるプログラム終了が無効になります）
#define _DEBUG 1

//ピン番号(WiringPiのピン番号、GPIOとは違います)
const static int PIN_PWM_A = 4;		//モータPWM Right
const static int PIN_PWM_B = 5;
const static int PIN_PULSE_A = 0;	//モータエンコーダ Right
const static int PIN_PULSE_B = 7;
const static int PIN_INVERT_MOTOR_A = 3;	//モータ反転ピン Right
const static int PIN_INVERT_MOTOR_B = 2;
const static int PIN_BUZZER = 12;			//ブザー
const static int PIN_XBEE_SLEEP = 13;		//XBeeスリープピン
const static int PIN_LIGHT_SENSOR = 14;		//Cdsセンサピン
const static int PIN_SERVO = 1;				//サーボピン

//モータ設定
const static int MOTOR_MAX_POWER = 100;
const static double MOTOR_MAX_POWER_CHANGE = MOTOR_MAX_POWER;//モータ出力の1秒あたりの最大変化量

//サーボ設定
const static int SERVO_RANGE = 9000;//パルス間隔
const static int SERVO_MOVABLE_RANGE = 1200;//パルス幅変更範囲
const static int SERVO_BASE_VALUE = 910 - SERVO_MOVABLE_RANGE / 2;//最小パルス幅

//////////////////////////////////////////////
// シーケンス系設定
//////////////////////////////////////////////
const static unsigned int CONTINUOUS_LIGHT_COUNT = 100;//何回連続で光っていると判定されたときに放出判定とするか
const static unsigned int WAITING_ABORT_TIME = 3600;//強制的に放出判定とする時間（秒）

const static unsigned int FALLING_DELTA_PRESSURE_THRESHOLD = 2;//前回との気圧の差がこれ以内なら停止中とカウント
const static unsigned int FALLING_PRESSURE_COUNT = 5;//気圧変化量が閾値以下の状態がこれだけ続いたら着地と判定
const static unsigned int FALLING_ABORT_TIME = 1800;//落下状態を強制終了する時間

const static double SEPARATING_SERVO_INTERVAL = 1;//サーボの向きを変える間隔(秒)
const static unsigned int SEPARATING_SERVO_COUNT = 5;//サーボの向きを変える回数

//////////////////////////////////////////////
//タスク系設定
//////////////////////////////////////////////
//タスク優先順位(低いほど先に実行される)
const static unsigned int TASK_PRIORITY_SENSOR = 10;
const static unsigned int TASK_PRIORITY_MOTOR = 100;
const static unsigned int TASK_PRIORITY_COMMUNICATION = 0;
const static unsigned int TASK_PRIORITY_ACTUATOR = 10000;
const static unsigned int TASK_PRIORITY_SEQUENCE = 1000;
//タスク実行間隔(低いほど多く実行される)
const static unsigned int TASK_INTERVAL_GYRO = 0;
const static unsigned int TASK_INTERVAL_SENSOR = 10;
const static unsigned int TASK_INTERVAL_MOTOR = 0;
const static unsigned int TASK_INTERVAL_COMMUNICATION = 1;
const static unsigned int TASK_INTERVAL_ACTUATOR = 0;
const static unsigned int TASK_INTERVAL_SEQUENCE = 0;
