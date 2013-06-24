/*
	その他関数など

	デバッグ用マクロやprint関数を用意してあります
	・print関数は画面とファイル両方に出力します
	・ログレベルは重要ではないログで画面が埋め尽くされないように設定します
	・staticクラスのため単純にDebug::print()のように呼び出してください
*/

#pragma once
#include <vector>
#include <string>
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
	LOG_DETAIL = 0,	//デバッグログ(バグが出たときの状況確認用)
	LOG_SUMMARY,	//ファイル保存、画面表示共にするもの
	LOG_PRINT		//画面にのみ表示するもの
}LOG_LEVEL;			//ログレベル(Apacheとかと似た感じで)

class Debug
{
public:
	static void print(LOG_LEVEL level, const char* fmt, ... );//ストリーム面倒だからprintfタイプでいいよね
	Debug();
};

class Time
{
public:
	//時間の変化量を計算(秒)
	static double dt(const struct timespec& now,const struct timespec& last);
};

class String
{
public:
	//文字列を空白で分割
	static void split(const std::string& input,std::vector<std::string>& outputs);
};
