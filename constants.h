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

const static int VERSION = 01;

//////////////////////////////////////////////
// ハードウェア系設定
//////////////////////////////////////////////
//ピン番号(WiringPiのピン番号、GPIOとは違います)
const static int PIN_PWM_A1 = 26;//モータPWM 
const static int PIN_PWM_A2 = 27;
const static int PIN_PULSE_A = 0;//モータエンコーダ Right             
const static int PIN_PULSE_B = 7;//                                    
const static int PIN_PWM_B1 = 28;//モータPWM					   
const static int PIN_PWM_B2 = 29;                              //Clear!
const static int PIN_BUZZER = 12;//ブザー                               Clear!
const static int PIN_XBEE_SLEEP = 13;//XBeeスリープピン
const static int PIN_LIGHT_SENSOR = 14;//Cdsセンサピン                  Clear!
const static int PIN_PARA_SERVO = 23;// パラシュートサーボピン          Clear!
const static int PIN_JOHN_SERVO = 25;//                                 Clear!
const static int PIN_MIKE_SERVO = 21;       //スタビサーボピン          Clear!
const static int PIN_NECK_SERVO = 1;//                                  Clear!
const static int PIN_ARM_SERVO = 22;//                                  Clear!
const static int PIN_DISTANCE = 8;//距離センサー(ピン番号は適当)

//モータ設定
const static int MOTOR_MAX_POWER = 100;
const static double MOTOR_MAX_POWER_CHANGE = (double)5;//モータ出力の最大変化量
const static double MOTOR_PID_MAX_ANGLE_DIFF = 90;// d_angle in pid is limited to this range

//エンコーダ関連 ※モータを変えたらここも変えてください。
const static int RESOLVING_POWER = 32;		//モータの分解能
const static int GEAR_RATIO = 29;			//モータのギア比
const static double DISTANCE_PER_ROTATION = 0.14 * M_PI; //m
const static double DISTANCE_BETWEEN_TIRES = 0.20; //m

//サーボ設定
const static int SERVO_RANGE = 9000;//パルス間隔
const static int SERVO_MOVABLE_RANGE = 1200;//パルス幅変更範囲
const static int SERVO_BASE_VALUE = 910 - SERVO_MOVABLE_RANGE / 2;//最小パルス幅

//スタビサーボ設定
const static double STABI_BASE_ANGLE = 0.5;	//通常時のスタビ角度

//スタビ学習用の設定
const static int POPULATION_NUM = 6; // 個体数
const static int ACTION_NUM = 6; // 行動回数
const static int CIRCLE = 1000; // 試行回数
const static double ENCODER_THRESHOLD = 4500; // エンコーダー関数のしきい値
const static double ENCODER_THRESHOLD_PERCENT = 0.90; // エンコーダー関数の中央
const static double ENCODER_THRESHOLD_DISCOUNT = 0.25; // エンコーダー関数の割引率
 
//ジャイロ設定
const static unsigned int GYRO_SAMPLE_COUNT_FOR_CALCULATE_OFFSET = 100;//ドリフト誤差補正時に用いるサンプル数

const static double STABI_ZERO_ANGLE = -17.0; // スタビ正常時のゼロ点角度
const static double STABI_J_ZERO_ANGLE = -26.0; // スタビ正常時のゼロ点角度

// サーボの初期設定
// 前スタビ
const static int FRONT_STABI_FOLD_ANGLE = 1;
const static int FRONT_STABI_RUN_ANGLE = 15;
// 後スタビ
const static int BACK_STABI_FOLD_ANGLE = 20;
const static int BACK_STABI_RUN_ANGLE = 6;
// アームサーボ
const static int ARM_FOLD_ANGLE = 16;
const static int ARM_RUN_ANGLE = 3;
// ネックサーボ
const static double NECK_FOLD_ANGLE = 1.0;
const static double NECK_RUN_ANGLE = -0.2;

//////////////////////////////////////////////
// シーケンス系設定
//////////////////////////////////////////////
const static unsigned int WAITING_LIGHT_COUNT = 5000;//何回連続で光っていると判定されたときに放出判定とするか
const static unsigned int WAITING_ABORT_TIME = 7200;//強制的に放出判定とする時間（秒）

const static unsigned int FALLING_DELTA_PRESSURE_THRESHOLD = 2;//前回との気圧の差がこれ以内なら停止中とカウント(1秒間隔でサンプリング)
const static unsigned int FALLING_PRESSURE_COUNT = 5;//気圧変化量が閾値以下の状態がこれだけ続いたら着地と判定
const static double FALLING_GYRO_THRESHOLD = 15;//角速度がこの値以下なら停止中とカウント
const static unsigned int FALLING_GYRO_COUNT = 1000;//角速度の値が閾値以下のサンプルがこれだけ連続したら着地と判定
const static unsigned int FALLING_ABORT_TIME = 1800;//落下状態を強制終了する時間
const static unsigned int FALLING_MOTOR_PULSE_THRESHOLD = 1000;//１秒辺りのパルス変化量がこれ以上ならタイヤ回転中とカウント
const static unsigned int FALLING_MOTOR_PULSE_COUNT = 10;//モータパルスの値が閾値以下のサンプルがこれだけ連続したら着地と判定

const static double SEPARATING_SERVO_INTERVAL = 0.8;//サーボの向きを変える間隔(秒)
const static unsigned int SEPARATING_SERVO_COUNT = 16;//サーボの向きを変える回数
const static double SEPARATING_PARA_DETECT_THRESHOLD = 0.005;//この割合以上パラシュート色が検出されたらパラが存在するものとする

const static double NAVIGATING_GOAL_DISTANCE_THRESHOLD = 7 / 111111.1;//ゴール判定とするゴールからの距離(度) 2016/08/31 3->7
const static double NAVIGATING_GOAL_APPROACH_DISTANCE_THRESHOLD = 10 / 111111.1;//移動速度を減速するゴールからの距離(近づいた場合、行き過ぎ防止のため減速する)
const static double NAVIGATING_GOAL_APPROACH_POWER_RATE = 0.8;//ゴール接近時の速度(最大比)
const static double NAVIGATING_DIRECTION_UPDATE_INTERVAL = 1;//進行方向を変更する間隔(秒) 2016/08/31 5->1
const static double NAVIGATING_MAX_DELTA_DIRECTION = 90;//一回の操作で方向転換する最大の角度
const static double NAVIGATING_STUCK_JUDGEMENT_THRESHOLD = 0.7 / 111111.1; // NAVIGATING_DIRECTION_UPDATE_INTERVALの間に移動した距離がこの閾値以下ならスタック判定とする
const static unsigned long long STUCK_ENCODER_PULSE_THRESHOLD = 3000; // 前回のエンコーダパルス数がこの値以上で、現在のパルス数がこの値以下ならスタック判定とする
const static unsigned int ESCAPING_BY_STABI_MIN_COUNT = 5; //最低でもこの回数以上EscapingByStabiの動作をする
const static unsigned int ESCAPING_BY_STABI_MAX_COUNT = 78;//この回数以上EscapingByStabiの動作を繰り返してもスタック脱出できない場合，EscapingRandomに遷移する
const static unsigned int LEARNING_ESCAPING_LIMIT = 59;
const static unsigned int ESCAPING_RANDOM_TIME_THRESHOLD = 30;//この秒数以上EscapingRandom動作をしてもスタック脱出できない場合，EscapingByStabiに遷移する
const static unsigned int COLOR_ACCESSING_ABORT_TIME = 300;//0mゴール検知状態を強制終了しNavigatingに復帰する時間
const static unsigned int COLOR_ACCESSING_MAX_RETRY_COUNT = 3;//この回数以上DetectingからNavigating復帰を繰り返したらその場でゴール判定して停止する
const static unsigned int CASCADE_ACCESSING_ABORT_TIME = 200;//0mゴール検知状態を強制終了しNavigatingに復帰する時間
const static unsigned int CASCADE_ACCESSING_MAX_RETRY_COUNT = 3;//この回数以上DetectingからNavigating復帰を繰り返したらその場でゴール判定して停止する

const static double WAKING_THRESHOLD = 200;
const static unsigned int WAKING_RETRY_COUNT = 5;

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

