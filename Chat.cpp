#include <stdio.h>
#include <iostream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <arpa/inet.h>
#include "chat.h"
#include "utils.h"
using namespace std;

//20170630マルチーズ追加チャットプログラム
//文字列委を受けとるserverのセットアップ
bool Server::init()
{
	//ソケットの作成
	//引数はアドレスファミリ、ソケットタイプ、プロトコル
	sock0 = socket(AF_INET, SOCK_STREAM, 0);

	//sockが-1を返したら失敗
	if (sock < 0)
		{
			//エラーを表示する処理
			perror("socketのエラーが出ました");
			printf("%d\n", errno);
			return 1;
		}
		//ソケットの設定
		addr.sin_family = AF_INET;
		addr.sin_port = htons(12345);
		addr.sin_addr.s_addr = INADDR_ANY;
		bind(sock0, (struct sockaddr *)&addr, sizeof(addr));

		//TCPクライアントからの接続要求を待てる状態にする
		listen(sock0, 5);

		return true;
}
//何度も接続要求受付を試みる
void Server::update(double elapsedSeconds)
{
	//TCPクライアントからの接続要求を受け付ける
	len = sizeof(client);
	sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
}
//sock操作を一端終了（電力消費軽減らしい？）
void Server::clean()
{
	//listenするsocketの終了
	close(sock0);
	mes = NULL;
}
void Server::onCommand(const std::vector<std::string>& args)
{
	switch(args.size())
	{
		case 2:
		if(args[1].compare("send"))
		{
			//clientに5文字HELLOを送る
			write(sock, "HELLO", 5);
			close(sock);
		}
	}
}

Server::Server()
{
	setName("chat_s");
	setPriority(TASK_);
}

Server::~Server()
{
}

//引数としてサーバーのIPアドレスが必要
bool Client::init(int sv_ip)
{
		//ソケットの作成
		//引数はアドレスファミリ、ソケットタイプ、プロトコル
		sock = socket(AF_INET, SOCK_STREAM, 0);

			//ソケットの設定
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("192.168.0.249");

			return true;
}

void Client::update()
{
	/* サーバに接続 */
  connect(sock, (struct sockaddr *)&server, sizeof(server));
}

void Client::clean()
{
}

void Client::onCommand(const std::vector<std::string>& args)
{
	switch(args.size())
	{
		case 2:
		if(args[1].compare("receive"))
		{
			memset(buf, 0, sizeof(buf));
			n = read(sock, buf, sizeof(buf));

			printf("%d, %s\n", n, buf);
			close(sock);
		}
	}
}
