#include <stdarg.h>
#include <stdio.h>
#include <fstream>
#include "debug.h"

const static unsigned int MAX_STRING_LENGTH = 1024;//Print用のバッファサイズ

void Debug::print(LOG_LEVEL level, const char* fmt, ... )
{
#ifndef _DEBUG
	if(level == LOG_DETAIL)return; //デバッグモードでなければログ出力しない
#endif

	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);
	
	//画面に出力
	printf(buf);
	//ログファイルに出力
	if(level != LOG_PRINT)
	{
		std::ofstream of("log.txt",std::ios::out | std::ios::app);
		of << buf;
	}
}
