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
		if (args[1].compare("send") == 0)
		{		
			sock0 = socket(AF_INET, SOCK_STREAM, 0);
			//ここまでは動いている
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			listen(sock0, 5);
			len = sizeof(client);
			sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
			Debug::print(LOG_PRINT, "accepted connection from %s, port=%d\n",
				inet_ntoa(client.sin_addr), ntohs(client.sin_port));
			nn = write(sock, "HELLO", 5);
			close(sock);
			close(sock0);
			return true;
		}
		return false;
	}
	else {
		Debug::print(LOG_PRINT, "chat: show chat state\r\n\
		chat send : send messeage to client\r\n\
		rec : recieve message from server\r\n");
		return true;
	}
}

Send::Send()
{
	setName("chat");
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
	//Debug::print(LOG_PRINT, "FIRE_soto");
	if (args.size() == 2)
	{
		if (args[1].compare("rec") == 0)
		{
			sock = socket(AF_INET, SOCK_STREAM, 0);
			//ここまでは動いている
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("10.0.0.12");

			connect(sock, (struct sockaddr *)&server, sizeof(server));

			memset(buf, 0, sizeof(buf));
			//ここまで動いている
			n = read(sock, buf, sizeof(buf));
			if (n < 0) {
				perror("read");
				printf("何も送られてないです。sendプログラムを実行してください");
				return 1;
			}
			Debug::print(LOG_PRINT,"%d,%s\n",n,buf);
			close(sock);
			return true;
		}
		//Debug::print(LOG_PRINT, "FIREF");
		return false;
	}
	else {
		//Debug::print(LOG_PRINT, "chat_r              : show chat state\r\n\
//chat_r rec: recieve message from server\r\n\"");
		return true;
	}
}

Rec::Rec() :buf(), n(0)
{
	//setName("chat_r");
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
