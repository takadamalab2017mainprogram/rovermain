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

//マルチーズ追加チャットプログラム
//chatを受けとるserverのセットアップ
bool Server::init(int cl_ip)
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

	//クライアントからデータ(文字列)を受信
	memset(buf_client, 0, sizeof(buf_client));

	//nは文字列
	int n = recv(sock, buf_client, sizeof(buf_client), 0);

	//クライアントから受信した文字列を表示
	Debug::print(LOG_SUMMARY, "次の文字列を受信しました→,%s\n",buf_client)
	//送るメッセージを入力
	cin >> mes;
	//入力した5文字を送信
	write(sock, mes, 5);
	Debug::print(LOG_SUMMARY,"次の文字列を送信しました→%s\n",mes)
}
