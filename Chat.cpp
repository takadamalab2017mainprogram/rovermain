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