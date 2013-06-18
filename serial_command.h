/*
	シリアルから入力されたコマンドを処理するクラス

	このクラスがTaskManagerのcommandメソッドを呼び出します
	task.hも参照
*/
#pragma once
#include <string>
#include <vector>
#include <termios.h>
#include <list>
#include "task.h"

class SerialCommand : public TaskBase
{
private:
	std::string mCommandBuffer;
	std::list<std::string> mHistory;
	std::list<std::string>::iterator mHistoryIterator;
	struct termios mOldTermios,mNewTermios;
	static void split(const std::string& input,std::vector<std::string> &outputs);//スペースで文字列を分割
public:
	virtual void update();//シリアルポートに到着したコマンドを確認する

	SerialCommand();
	~SerialCommand();
};
extern SerialCommand gSerialCommand;
