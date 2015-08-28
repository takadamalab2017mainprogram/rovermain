/*
	各種定数
*/
#pragma once

//本番はコメントアウトすること！！（Ctrl-Cによるプログラム終了が無効になります）
#define _DEBUG 1
#define _USE_MATH_DEFINES
#include <math.h>
//詳細なログ表示が必要な場合はつかってください
//#define _LOG_DETAIL 1



//////////////////////////////////////////////
// ハードウェア系設定
//////////////////////////////////////////////
//ピン番号(WiringPiのピン番号、GPIOとは違います)
const static int PIN_PWM_A = 5;				//モータPWM Right
const static int PIN_PWM_B = 4;
const static int PIN_PULSE_A = 7;			//モータエンコーダ Right
const static int PIN_PULSE_B = 0;
const static int PIN_INVERT_MOTOR_A = 3;	//モータ反転ピン Right
const static int PIN_INVERT_MOTOR_B = 2;
const static int PIN_BUZZER = 12;			//ブザー
const static int PIN_XBEE_SLEEP = 13;		//XBeeスリープピン
const static int PIN_LIGHT_SENSOR = 14;		//Cdsセンサピン
const static int PIN_PARA_SERVO = 6;		//パラシュートサーボピン
const static int PIN_STABI_SERVO = 1;		//スタビサーボピン(前）（ハードPWM)
const static int PIN_CAMERA_SERVO = 24;		//カメラサーボピン(hard pwm)
const static int PIN_CAMERA_SERVO_SOFT = 25; //カメラサーボピン(soft pwm) 審査会対策　番号未定
const static int PIN_FRONT_STABI_SERVO = 28; //前スタビピン（ソフトPWM
const static int PIN_DISTANCE = 8;			//距離センサー(ピン番号は適当)

//モータ設定
const static int MOTOR_MAX_POWER = 100;
const static double MOTOR_MAX_POWER_CHANGE =(double)5;//モータ出力の最大変化量

//エンコーダ関連 ※モータを変えたらここも変えてください。
const static int RESOLVING_POWER = 32;		//モータの分解能
const static int GEAR_RATIO = 29;			//モータのギア比
const static double DISTANCE_PER_ROTATION = 0.14 * M_PI; //m8-24 chou pose_detector 関連
const static double DISTANCE_BETWEEN_TIRES = 0.20; //m



//サーボ設定
const static int SERVO_RANGE = 9000;//パルス間隔
const static int SERVO_MOVABLE_RANGE = 1200;//パルス幅変更範囲
const static int SERVO_BASE_VALUE = 910 - SERVO_MOVABLE_RANGE / 2;//最小パルス幅

//スタビサーボ設定
const static double STABI_BASE_ANGLE = 0.7;	//通常時のスタビ角度
const static double STABI_FOLD_ANGLE = 0.0;	//収納時のスタビ角度
const static double STABI_WAKING_ANGLE = 0.4; //起き上がり用のスたビの角度
 
//ジャイロ設定
const static unsigned int GYRO_SAMPLE_COUNT_FOR_CALCULATE_OFFSET = 100;//ドリフト誤差補正時に用いるサンプル数

//////////////////////////////////////////////
// シーケンス系設定
//////////////////////////////////////////////
const static unsigned int WAITING_LIGHT_COUNT = 3000;//何回連続で光っていると判定されたときに放出判定とするか
const static unsigned int WAITING_ABORT_TIME =7200;//強制的に放出判定とする時間（秒）

const static unsigned int FALLING_DELTA_PRESSURE_THRESHOLD = 2;//前回との気圧の差がこれ以内なら停止中とカウント(1秒間隔でサンプリング)
const static unsigned int FALLING_PRESSURE_COUNT = 5;//気圧変化量が閾値以下の状態がこれだけ続いたら着地と判定
const static double FALLING_GYRO_THRESHOLD = 10;//角速度がこの値以下なら停止中とカウント
const static unsigned int FALLING_GYRO_COUNT = 1000;//角速度の値が閾値以下のサンプルがこれだけ連続したら着地と判定
const static unsigned int FALLING_ABORT_TIME = 1800;//落下状態を強制終了する時間
const static unsigned int FALLING_MOTOR_PULSE_THRESHOLD = 1000;//１秒辺りのパルス変化量がこれ以上ならタイヤ回転中とカウント
const static unsigned int FALLING_MOTOR_PULSE_COUNT = 5;//モータパルスの値が閾値以下のサンプルがこれだけ連続したら着地と判定

const static double SEPARATING_SERVO_INTERVAL = 0.8;//サーボの向きを変える間隔(秒)
const static unsigned int SEPARATING_SERVO_COUNT = 16;//サーボの向きを変える回数
const static double SEPARATING_PARA_DETECT_THRESHOLD = 0.005;//この割合以上パラシュート色が検出されたらパラが存在するものとする

const static double NAVIGATING_GOAL_DISTANCE_THRESHOLD = 3 / 111111.1;//ゴール判定とするゴールからの距離(度)
const static double NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD = 10 / 111111.1;//移動速度を減速するゴールからの距離(近づいた場合、行き過ぎ防止のため減速する)
const static double NAVIGATING_GOAL_APPROACH_POWER_RATE = 0.5;//ゴール接近時の速度(最大比)
const static double NAVIGATING_DIRECTION_UPDATE_INTERVAL = 5;//進行方向を変更する間隔(秒)
const static double NAVIGATING_MAX_DELTA_DIRECTION = 90;//一回の操作で方向転換する最大の角度
const static double NAVIGATING_STUCK_JUDGEMENT_THRESHOLD = 0.7 / 111111.1; //NAVIGATING_DIRECTION_UPDATE_INTERVALの間に移動した距離がこの閾値以下ならスタック判定とする
const static unsigned long long STUCK_ENCODER_PULSE_THRESHOLD = 3000;//前回のエンコーダパルス数がこの値以上で、現在のパルス数がこの値以下ならスタック判定とする
const static unsigned int ESCAPING_BY_STABI_MIN_COUNT = 5; //最低でもこの回数以上EscapingByStabiの動作をする
const static unsigned int ESCAPING_BY_STABI_MAX_COUNT = 35;//この回数以上EscapingByStabiの動作を繰り返してもスタック脱出できない場合，EscapingRandomに遷移する
const static unsigned int ESCAPING_RANDOM_TIME_THRESHOLD = 60;//この秒数以上EscapingRandom動作をしてもスタック脱出できない場合，EscapingByStabiに遷移する
const static unsigned int COLOR_ACCESSING_ABORT_TIME = 300;//0mゴール検知状態を強制終了しNavigatingに復帰する時間
const static unsigned int COLOR_ACCESSING_MAX_RETRY_COUNT = 5;//この回数以上DetectingからNavigating復帰を繰り返したらその場でゴール判定して停止する

const static double WAKING_THRESHOLD = 200;
const static unsigned int WAKING_RETRY_COUNT = 10;

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

//////////////////////////////////////////////
//その他
//////////////////////////////////////////////
const static double DEGREE_2_METER = 111111.111111;//これを度に掛けるとメートルに変換できる
const static char INITIALIZE_SCRIPT_FILENAME[] = "initialize.txt";

