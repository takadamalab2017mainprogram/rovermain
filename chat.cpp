#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>
#include <arpa/inet.h>
#include "chat.h"
#include "utils.h"
using namespace std;

//Sendクラスは相手にメッセージが送られるまで実行される。
//送られたらsockを閉じて終了、現在は送られないとプログラムの終了ができなくなる。
bool Send::onInit(const struct timespec& time)
{
	return true;
}

void Send::onUpdate(const struct timespec& time)
{
}

void Send::onClean()
{
}
bool Send::onCommand(const vector<string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("sen") == 0)
		{	
			send();
			return true;
		}
		return false;
	}
	else {
		Debug::print(LOG_PRINT, "chat_s              : show chat state\r\n\
chat_s sen: send messeage to client\r\n");
		return true;
	}
}

void Send::send()
{
	//soket作成時のエラーを表示
	if ((sock1 = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket");
		exit(1);
	}
	memset((char*)&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(12345);
	addr.sin_addr.s_addr = INADDR_ANY;

	setsockopt(sock1, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
	//bind時のエラー
	if (bind(sock1, (struct sockaddr *)&addr, sizeof(addr)) < 0)
	{
		perror("bind");    exit(1);
	}
	//listenのエラーを表示
	if (listen(sock1, 5) < 0)
	{
		perror("listen"); exit(1);
	}

	len = sizeof(client);
	if (sock2 = accept(sock1, (struct sockaddr *)&client, (socklen_t *)&len) < 0)
	{
		perror("accept"); exit(1);
	}
	else
	{
		//相手のIPアドレスとポート番号を表示
		Debug::print(LOG_PRINT, "accepted connection from %s, port=%d\n",
			inet_ntoa(client.sin_addr), ntohs(client.sin_port));
	}
	close(sock1);
	strcpy(buf, "I'm a server.\n");
	write(sock2, buf, sizeof(buf));
	close(sock2);
}

Send::Send():buf()
{
	setName("chat_s");
	setPriority(TASK_PRIORITY_SEND, TASK_INTERVAL_SEND);
}

Send::~Send()
{
}

bool Rec::onInit(const struct timespec& time)
{
	return true;
}

void Rec::onUpdate(const struct timespec& time)
{
}

void Rec::onClean()
{
}

bool Rec::onCommand(const std::vector<std::string>& args)
{
	if (args.size() == 2)
	{
		if (args[1].compare("rec") == 0)
		{
			receive();
			return true;
		}
		//Debug::print(LOG_PRINT, "FIREF");
		return false;
	}
	else {
		Debug::print(LOG_PRINT, "chat_r              : show chat state\r\n\
chat_r rec: recieve message from server\r\n");
		return true;
	}
}

void Rec::receive()
{

	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket"); exit(1);
	}
	memset((char*)&server, 0, sizeof(server));


	server.sin_family = AF_INET;
	server.sin_port = htons(12345);
	server.sin_addr.s_addr = inet_addr("10.0.0.12");
	if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0)
	{
		perror("connect"); exit(1);
	}
	n = read(sock, buf, sizeof(buf));
	if(n > 0)
	{
	printf("ソケットからデータを読み取った")
	} 
	else if (n < 0) 
	{
	perror("read");
	printf("何も送られてないです。sendプログラムを実行してください");
	}
	else if (n == 0)
	{
	Debug::print(LOG_PRINT, "%d,%s\n", n, buf);
	close(sock);
	}
}

Rec::Rec() :buf(), n(0)
{
	setName("chat_r");
	setPriority(TASK_PRIORITY_REC, TASK_INTERVAL_REC);
}

Rec::~Rec()
{
}

/*
//繧オ繝シ繝舌・縺ィ繧ッ繝ゥ繧、繧「繝ウ繝医ｒ縺セ縺ィ繧√◆繧ッ繝ゥ繧ケ
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
//繧オ繝シ繝舌・
if (args[1].compare("sen") == 0)
{
gSend.send();
}
//繧ッ繝ゥ繧、繧「繝ウ繝・
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
//蛻晄悄蛹悶☆繧九ｂ縺ョ縺ッ縺。繧・ｓ縺ィ豎コ繧√ｋ
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
Rec gRec;
//Chat gChat;
