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
const static double MOTOR_MAX_POWER_CHANGE = 0.5;//モータ出力の最大変化量
const static int MOTOR_MAX_POWER = 100;

//サーボ設定
const static int SERVO_RANGE = 9000;
const static int SERVO_MOVABLE_RANGE = 1200;
const static int SERVO_BASE_VALUE = 900 - SERVO_MOVABLE_RANGE / 2;


//////////////////////////////////////////////
//タスク系設定
//////////////////////////////////////////////
//タスク優先順位(低いほど先に実行される)
const static unsigned int TASK_PRIORITY_SENSOR = 10;
const static unsigned int TASK_PRIORITY_MOTOR = 100;
const static unsigned int TASK_PRIORITY_COMMUNICATION = 0;
const static unsigned int TASK_PRIORITY_ACTUATOR = 10000;
//タスク実行間隔(低いほど多く実行される)
const static unsigned int TASK_INTERVAL_GYRO = 0;
const static unsigned int TASK_INTERVAL_SENSOR = 10;
const static unsigned int TASK_INTERVAL_MOTOR = 0;
const static unsigned int TASK_INTERVAL_COMMUNICATION = 1;
const static unsigned int TASK_INTERVAL_ACTUATOR = 0;
