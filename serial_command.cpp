#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <iterator>
#include <sstream>
#include <iostream>
#include <string>
#include "debug.h"
#include "serial_command.h"
#include "motor.h"

SerialCommand gSerialCommand;

void SerialCommand::split(const std::string& input,std::vector<std::string>& outputs)
{
	//文字列を空白文字で分割してvectorに格納
	outputs.clear();
	std::istringstream iss(input);
	std::copy(std::istream_iterator<std::string>(iss),  std::istream_iterator<std::string>(), std::back_inserter(outputs));
}
void SerialCommand::update()
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

					Debug::print(LOG_MINIMUM, "\r\033[2K%s",mCommandBuffer.c_str());
					need2update = false;
				}else if(mCommandBuffer[mCommandBuffer.size() - 2] == '[' && mCommandBuffer[mCommandBuffer.size() - 1] == 'B')
				{
					//矢印下キー
					if(mHistoryIterator != mHistory.begin())
					{
						--mHistoryIterator;
						mCommandBuffer = *mHistoryIterator;
					}else mCommandBuffer.clear();
				
					Debug::print(LOG_MINIMUM, "\r\033[2K%s",mCommandBuffer.c_str());
					need2update = false;
				}
			}
		}
		if(need2update)Debug::print(LOG_MINIMUM, "%c", c);//文字を表示
		
		if(c == '\n')
		{
			mCommandBuffer.erase(mCommandBuffer.size()-1);//改行文字を削除

			//コマンドを実行
			std::vector<std::string> args;
			split(mCommandBuffer,args);
			TaskManager::getInstance()->command(args);

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

