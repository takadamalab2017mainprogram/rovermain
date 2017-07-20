
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
#include "chat.h"
#include "utils.h"
using namespace std;

//20170630マルチーズ追加チャットプログラム
//文字列委を受けとるserverのセットアップ

bool Send::onInit(const struct timespec& time)
{
	/*
	//ソケットの作成
	//引数はアドレスファミリ、ソケットタイプ、プロトコル
	sock0 = socket(AF_INET, SOCK_STREAM, 0);

	//sockが-1を返したら失敗
	if (sock < 0)
	{
		//エラーを表示する処理
		perror("socketのエラーが出ました");
		printf("%d\n", errno);
		//return 1;
	}
	//ソケットの設定
	addr.sin_family = AF_INET;
	addr.sin_port = htons(12345);
	addr.sin_addr.s_addr = INADDR_ANY;
	bind(sock0, (struct sockaddr *)&addr, sizeof(addr));

	//TCPクライアントからの接続要求を待てる状態にする
	listen(sock0, 5);
	return true;
	*/
}

//何度も接続要求受付を試みる
void Send::onUpdate(const struct timespec& time)
{
	/*
	//TCPクライアントからの接続要求を受け付ける
	len = sizeof(client);
	sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
	*/
}
//sock操作を一端終了（電力消費軽減らしい？）
void Send::onClean()
{
	/*
	//listenするsocketの終了
	close(sock0);
	mes = NULL;
	*/
}
bool Send::onCommand(const vector<string>& args)
{
  switch (args.size())
	{
	case 2:
		if (args[1].compare("sen"))
		{
			/* ソケットの作成 */
			sock0 = socket(AF_INET, SOCK_STREAM, 0);

			/* ソケットの設定 */
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			//5回ほど相手にメッセージを送ったら終了する
			/* TCPクライアントからの接続要求を待てる状態にする */
			listen(sock0, 5);
			//while(k < 5){
			/* TCPクライアントからの接続要求を受け付ける */
			len = sizeof(client);
			sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
			/* 5文字送信 */
			n = write(sock, "HELLO", 5);
			if (n < 1)
			{
				perror("write");
				break;
			}
			/* TCPセッションの終了 */
		   // k++;
			close(sock);
			/* listen するsocketの終了 */
			close(sock0);
		}
//		}
		else
		{
			Debug::print(LOG_PRINT, "chat_s              : show chat state\r\n\
chat_s sen: send messeage to client\r\n\"");
			}
return true;
}
/*
void Send::send()
{
	//clientに5文字HELLOを送る
	write(sock, "KOUKI", 5);
	close(sock);
}
*/
}

Send::Send():sock(0),sock0(0)
{
	setName("chat_s");
	setPriority(TASK_PRIORITY_CHAT, TASK_INTERVAL_CHAT);
}

Send::~Send()
{
}

//引数としてサーバーのIPアドレスが必要
bool Rec::onInit(const struct timespec& time)
{
	//ソケットの作成
	//引数はアドレスファミリ、ソケットタイプ、プロトコル
	//sock = socket(AF_INET, SOCK_STREAM, 0);

	//ソケットの設定
	//server.sin_family = AF_INET;
	//server.sin_port = htons(12345);
	//server.sin_addr.s_addr = inet_addr("10.0.0.5");

	return true;
}

void Rec::onUpdate(const struct timespec& time)
{
  //ソケットの作成
  //引数はアドレスファミリ、ソケットタイプ、プロトコル
	//sockがサーバーの
  
	/* サーバに接続 */
	//connect(sock1, (struct sockaddr *)&server, sizeof(server));
}

void Rec::onClean()
{
}

bool Rec::onCommand(const std::vector<std::string>& args)
{
	switch (args.size())
	{
	case 2:
		if (args[1].compare("rec"))
		{
			/* ソケットの作成 */
			sock1 = socket(AF_INET, SOCK_STREAM, 0);

			/* 接続先指定用構造体の準備 */
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("10.0.0.12");

			/* サーバに接続 */
			connect(sock1, (struct sockaddr *)&server, sizeof(server));

			/* サーバからデータを受信 */
			memset(buf, 0, sizeof(buf));
			n = read(sock1, buf, sizeof(buf));
			if (n < 0) {
				perror("read");
				printf("相手のプログラムから何も送られてきてないよ");
				return 1;
			}

			printf("%d, %s\n", n, buf);

			/* socketの終了 */
			close(sock1);
			
		}
		else
		{
			Debug::print(LOG_PRINT, "chat_r              : show chat state\r\n\
chat_r rec: recieve message from server\r\n\"");
		}
	}
	return true;
}
//レシーブ関数
/*
void Rec::receive() 
{
	sock1 = socket(AF_INET, SOCK_STREAM, 0);

	//ソケットの設定
	server.sin_family = AF_INET;
	server.sin_port = htons(12345);
	server.sin_addr.s_addr = inet_addr("10.0.0.10");

	connect(sock1, (struct sockaddr *)&server, sizeof(server));
	memset(buf, 0, sizeof(buf));
	n = read(sock1, buf, sizeof(buf));

	printf("%d, %s\n", n, buf);
	close(sock1);
}
*/
Rec::Rec():sock1(0),buf(),n(0)
{
	setName("chat_r");
	setPriority(TASK_PRIORITY_CHAT, TASK_INTERVAL_CHAT);
}

Rec::~Rec()
{
}

/*
//サーバーとクライアントをまとめたクラス
bool Chat::onInit(const struct timespec& time)
{
	gSend.setRunMode(true);
	//gRec.setRunMode(true);
	return true;
}
void Chat::onClean()
{
}

bool Chat::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		//サーバー
		if (args[1].compare("sen") == 0)
		{
			gSend.send();
		}
		//クライアント
		else if (args[1].compare("rec") == 0)
		{
			//gRec.receive();
		}
		return true;
	}
	else
	{
		Debug::print(LOG_PRINT, "chat              : show chat state\r\n\
chat sen: send messeage to client\r\n\
chat rec: recieve message from server\r\n\"");
	}
	return false;
}
//初期化するものはちゃんと決める
Chat::Chat()
{
	setName("chat");
	setPriority(TASK_PRIORITY_CHAT, TASK_INTERVAL_CHAT);
}
Chat::~Chat()
{
}
*/

Send gSend;
//繧ｯ繝ｩ繧､繧｢繝ｳ繝医・繧､繝ｳ繧ｹ繧ｿ繝ｳ繧ｹ繧剃ｽ懊ｋ縺ｨ繝励Ο繧ｰ繝ｩ繝�縺檎ｵゆｺ・☆繧・
Rec gRec;
//Chat gChat;
