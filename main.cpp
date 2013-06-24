#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>
#include "constants.h"
#include "debug.h"
#include "motor.h"
#include "serial_command.h"
#include "sensor.h"
#include "actuator.h"

void sigHandler(int p_signame);
bool setSighandle(int p_signame);

//実行中フラグ(falseで終了)
static bool gIsRunning = true;

int main(int argc, char** argv)
{
	time_t timer;
	timer = time(NULL);
	Debug::print(LOG_SUMMARY,"%s\r\n2013 Takadama-lab ARLISS\r\n* Rivai Team *\r\n",ctime(&timer));

	//キーボードによる終了を阻止
	if(!(setSighandle(SIGINT) && setSighandle(SIGQUIT)))
	{
		Debug::print(LOG_SUMMARY,"Failed to set signal!\r\n");
	}

	//wiring pi初期化
    if(wiringPiSetup() != 0)
	{
		Debug::print(LOG_SUMMARY,"Failed to setup wiringPi!\r\n");
		return -1;
	}

	//タスク設定
	TaskManager* pTaskMan = TaskManager::getInstance();

	///////////////////////////////////////////
	//タスクを使用するように設定

	//コマンド受付設定
	gSerialCommand.setRunMode(true);

	//モータ駆動設定
	gMotorDrive.setRunMode(true);
	
	//センサー設定
	gPressureSensor.setRunMode(true);
	gGPSSensor.setRunMode(true);
	gGyroSensor.setRunMode(true);
	gLightSensor.setRunMode(true);
	gWebCamera.setRunMode(true);

	//アクチュエータ設定
	gBuzzer.setRunMode(true);
	gServo.setRunMode(true);
	gXbeeSleep.setRunMode(true);

	Debug::print(LOG_SUMMARY, "Ready.\r\n");

	
	////////////////////////////////////////////
	//メインループ(ブロックなどせずに短時間で処理を返すこと)
	while(gIsRunning)
	{
		//タスク処理(この関数ひとつでタスクが実行される)
		pTaskMan->update();
		
		//CPU処理を占有しないようにWaitをはさむ
		delay(1);
	}

	pTaskMan->clean();
	return 0;
}

//シグナル処理ハンドラ設定
bool setSighandle(int p_signame)
{
	return signal(p_signame, sigHandler) != SIG_ERR;
}

//シグナル処理(Ctrl-Cに対する安全性確保)
void sigHandler(int p_signame)
{
	
	switch(p_signame)
	{
	case 2:	//Ctrl-Cを無視
		{
			static time_t last_pressed = 0;
			time_t cur_time;
			time(&cur_time);
			if(last_pressed + 3 > cur_time)
			{

#ifdef _DEBUG
				Debug::print(LOG_SUMMARY,"Shutting down...\r\n");
				gIsRunning = false;
				return;
#endif

			}else Debug::print(LOG_SUMMARY,"Ctrl-C is disabled\r\n");

#ifdef _DEBUG
			Debug::print(LOG_SUMMARY,"To quit, Press Ctrl-C again!\r\n");
#endif

			last_pressed = cur_time;
			break;
		}
	case 3: //キーボードによる中止を無視
		Debug::print(LOG_SUMMARY,"Quit by keyboard is disabled\r\n");
		break;
	}
}

