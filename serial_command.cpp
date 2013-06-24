#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <iterator>
#include <sstream>
#include <iostream>
#include <string>
#include "utils.h"
#include "serial_command.h"
#include "motor.h"

SerialCommand gSerialCommand;

void SerialCommand::onUpdate(const struct timespec& time)
{
	int c;
	while((c = getchar()) != EOF)
	{
		mCommandBuffer.push_back(c);
		bool need2update = true;
		if(!mHistory.empty() && mCommandBuffer.size() >= 3)
		{
			if(mCommandBuffer[mCommandBuffer.size() - 3] == '\033')
			{
				//コマンド履歴処理
				if(mCommandBuffer[mCommandBuffer.size() - 2] == '[' && mCommandBuffer[mCommandBuffer.size() - 1] == 'A')
				{
					//矢印上キー
					mCommandBuffer = *mHistoryIterator;

					std::list<std::string>::iterator lastIterator = mHistoryIterator++;
					if(mHistoryIterator == mHistory.end())mHistoryIterator = lastIterator;

					Debug::print(LOG_PRINT, "\r\033[2K%s",mCommandBuffer.c_str());
					need2update = false;
				}else if(mCommandBuffer[mCommandBuffer.size() - 2] == '[' && mCommandBuffer[mCommandBuffer.size() - 1] == 'B')
				{
					//矢印下キー
					if(mHistoryIterator != mHistory.begin())
					{
						--mHistoryIterator;
						mCommandBuffer = *mHistoryIterator;
					}else mCommandBuffer.clear();
				
					Debug::print(LOG_PRINT, "\r\033[2K%s",mCommandBuffer.c_str());
					need2update = false;
				}
			}
		}
		if(need2update)Debug::print(LOG_PRINT, "%c", c);//文字を表示
		
		if(c == '\n')
		{
			mCommandBuffer.erase(mCommandBuffer.size()-1);//改行文字を削除

			//コマンドを実行
			TaskManager::getInstance()->command(mCommandBuffer);

			if(mCommandBuffer.length() != 0)mHistory.push_front(mCommandBuffer);
			mHistoryIterator = mHistory.begin();
			mCommandBuffer.clear();
			break;
		}
	}
}
SerialCommand::SerialCommand()
{
	//現在のターミナル設定を保存し、変更する
	tcgetattr( STDIN_FILENO, &mOldTermios );
	mNewTermios = mOldTermios;
	mNewTermios.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &mNewTermios );
	fcntl(STDIN_FILENO ,F_SETFL,O_NONBLOCK);

	//タスク設定
	setPriority(TASK_PRIORITY_COMMUNICATION,TASK_INTERVAL_COMMUNICATION);
}
SerialCommand::~SerialCommand()
{
	//ターミナル設定を元に戻す
	Debug::print(LOG_DETAIL,"Restoreing Terminal Settings\r\n");
	tcsetattr( STDIN_FILENO, TCSANOW, &mOldTermios );
}

