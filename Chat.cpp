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

bool Server::init()
{
	//ソケットの作成
	//引数はアドレスファミリ、ソケットタイプ、プロトコル
	sock0 = socket(AF_INET, SOCK_STREAM, 0);


}

void error_check(int sock) 
{
	//sockが-1を返したら失敗
	if (sock < 0)
	{
		//エラーを表示する処理
		perror("socketのエラーが出ました");
		printf("%d\n", errno);
		return 1;
	}
}