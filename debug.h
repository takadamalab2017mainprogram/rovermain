/*
	デバッグプログラム

	デバッグ用マクロやprint関数を用意してあります
	・print関数は画面とファイル両方に出力します
	・ログレベルは重要ではないログで画面が埋め尽くされないように設定します
	　実行に大した影響がないログはLOG_DETAIL、それなりに影響があるログはLOG_SUMMARY、常に表示するログはLOG_MINIMUMを設定してください
	・staticクラスのため単純にDebug::print()のように呼び出してください
*/

#pragma once
#include "constants.h"

#ifdef _DEBUG
	#include <assert.h>
	//xが0ならabort
	#define ASSERT(x) assert(x);
	//xが非0ならabort
	#define VERIFY(x) assert(!(x));
#else
	#define ASSERT(x)
	#define VERIFY(x)
#endif

typedef enum
{
	LOG_DETAIL = 0,	//完全なデバッグログ
	LOG_SUMMARY,	//実験時に表示するもの
	LOG_MINIMUM		//本番に表示するもの
}LOG_LEVEL;			//ログレベル(Apacheとかと似た感じで)

class Debug
{
public:
	static LOG_LEVEL mLogLevel;

	static void print(LOG_LEVEL level, const char* fmt, ... );
	Debug();
};

//ストリーム面倒だからprintfタイプでいいよね
