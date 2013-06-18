#include <stdarg.h>
#include <stdio.h>
#include <fstream>
#include "debug.h"

const static unsigned int MAX_STRING_LENGTH = 1024;//Print用のバッファサイズ
LOG_LEVEL Debug::mLogLevel = LOG_SUMMARY;				//デフォルトのログ出力レベル

void Debug::print(LOG_LEVEL level, const char* fmt, ... )
{
	if(level < mLogLevel)return;//ログ出力しない
	char buf[MAX_STRING_LENGTH];

	va_list argp;
	va_start(argp, fmt);
	vsprintf(buf, fmt, argp);
	
	//画面に出力
	printf(buf);
	//ログファイルに出力
	std::ofstream of("log.txt",std::ios::out | std::ios::app);
	of << buf;
}
