#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include <signal.h>
#include <time.h>
#include <stdio.h>
#include <fstream>
#include <string>
#include <iostream>
#include "sequence.h"
#include "utils.h"

void sigHandler(int p_signame);
bool setSighandle(int p_signame);

//実行中フラグ(falseで終了)
static bool gIsRunning = true;

//initialize.txtを読み込んで初期設定を行う関数
// 1行目　　：使用するタスクをスペースで区切って列挙
// 2行目以降：実行するコマンドを行ごとに列挙
bool parseInitializer()
{
	TaskManager* pTaskMan = TaskManager::getInstance();
	Debug::print(LOG_SUMMARY, "Reading initialize.txt...");
	std::ifstream ifs( INITIALIZE_SCRIPT_FILENAME );
	std::string str;
	if(ifs.good())
	{
		//initialize.txtが存在する場合
		
		//先頭行に列挙されたタスクを使用するように設定する
		std::getline(ifs,str);
		Debug::print(LOG_SUMMARY, "OK!\r\n");

		std::vector<std::string> tasks;
		String::split(str,tasks);

		std::vector<std::string>::iterator it = tasks.begin();
		while(it != tasks.end())
		{
			TaskBase* pTask = pTaskMan->get(*it);
			if(pTask != NULL)
			{
				Debug::print(LOG_SUMMARY, "Loading %s...\r\n",it->c_str());
				pTask->setRunMode(true);
			}else Debug::print(LOG_SUMMARY, "%s is not Available!\r\n",it->c_str());
			++it;
		}

		//使用するタスクを初期化する
		pTaskMan->update();

		Debug::print(LOG_SUMMARY, "Executing Initializing Commands...\r\n");
		//2行目以降のコマンドをすべて実行する
		while(!ifs.eof() && !ifs.fail() && !ifs.bad())
		{
			std::getline(ifs,str);
			if(str[0] == '/' && str[1] == '/') continue;//initialize.txtのうち"//"で始まる行は無視

			pTaskMan->command(str);
		}
		return true;
	}
	return false;
}

int main(int argc, char** argv)
{
//#define TEST_CONSTANT1 1
//#define TEST_CONSTANT0 0
//
//	ConstantManager::get().add(TEST_CONSTANT1,"TEST_CONSTANT1",0);
//	ConstantManager::get().add(TEST_CONSTANT0,"TEST_CONSTANT0",0);
//	ConstantManager::get()[TEST_CONSTANT1] = 10;
//	Debug::print(LOG_SUMMARY, "constant test: %f",ConstantManager::get()[TEST_CONSTANT1_STR]);
//	ConstantManager::get()[TEST_CONSTANT0] = 20;
//	Debug::print(LOG_SUMMARY, "constant test: %f",ConstantManager::get()[TEST_CONSTANT0_STR]);
//
	Time::showNowTime();
	Debug::print(LOG_SUMMARY,"2015 Takadama-lab ARLISS\r\n** HIGHBALL Team ***\r\n");

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
	// タスクを使用するように設定
	if(!parseInitializer())
	{
		//initialize.txtが読み込めなかったため、標準状態で起動する
		Debug::print(LOG_SUMMARY, "Not Found.\r\nLoading Default Task...\r\n");
		gTestingState.setRunMode(true);
	}
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

	//書き込まれていないファイルを強制的にSDに書き込み
	system("sync");
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
	case 2:	//SIGINT
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
	case 3: //SIGQUIT
		Debug::print(LOG_SUMMARY,"Quit by keyboard is disabled\r\n");
		break;
	}
}

