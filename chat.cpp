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

//送信クラス
void Send::onUpdate(const struct timespec& time)
{
}
void Send::onClean()
{
}
bool Send::onCommand(const vector<string>& args)
{
  if(args.size()==2)
	{
		if (args[1].compare("sen")==0)
		{
			
			sock0 = socket(AF_INET, SOCK_STREAM, 0);
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			Debug::print(LOG_PRINT, "FIRE");
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
//		}
	}
  else {
	  Debug::print(LOG_PRINT, "chat              : show chat state\r\n\
chat sen: send messeage to client\r\n\
chat rec: recieve message from server\r\n\"");
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
	if(args.size() == 2)
	{
		if (args[1].compare("rec") == 0)
		{
			sock = socket(AF_INET, SOCK_STREAM, 0);
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("10.0.0.12");
			connect(sock, (struct sockaddr *)&server, sizeof(server));
			memset(buf, 0, sizeof(buf));
			n = read(sock, buf, sizeof(buf));
			if (n < 0) {
				perror("read");
				printf("相手が送信プログラムを起動してないよ");
				return 1;
			}
			Debug::print(LOG_PRINT,"%d, %s\n", n, buf);
			close(sock);
			return true;
		}
		return false;
	}
	/*
	else {
		Debug::print(LOG_PRINT, "chat              : show chat state\r\n\
chat rec: recieve message from server\r\n\"");
		return true;
	}
	*/
}

Rec::Rec():buf(),n(0)
{
	close(sock);
	//setName("chat");
	//setPriority(TASK_PRIORITY_REC, TASK_INTERVAL_REC);
}

Rec::~Rec()
{
	close(sock);
}

/*
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
		//繧ｵ繝ｼ繝舌・
		if (args[1].compare("sen") == 0)
		{
			gSend.send();
		}
		//繧ｯ繝ｩ繧､繧｢繝ｳ繝・
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
//蛻晄悄蛹悶☆繧九ｂ縺ｮ縺ｯ縺｡繧・ｓ縺ｨ豎ｺ繧√ｋ
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
