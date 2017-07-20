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

//SendƒNƒ‰ƒX‚Í‘Šè‚ÉƒƒbƒZ[ƒW‚ª‘—‚ç‚ê‚é‚Ü‚ÅÀs‚³‚ê‚éB
//‘—‚ç‚ê‚½‚çsock‚ğ•Â‚¶‚ÄI—¹AŒ»İ‚Í‘—‚ç‚ê‚È‚¢‚ÆƒvƒƒOƒ‰ƒ€‚ÌI—¹‚ª‚Å‚«‚È‚­‚È‚éB
bool Send::onInit(const struct timespec& time)
{
	return true;
}

//‘—MƒNƒ‰ƒX
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
			/* ã‚½ã‚±ãƒEï¿½ï¿½ã®ä½œï¿½E */
			sock0 = socket(AF_INET, SOCK_STREAM, 0);
			//ï¿½ï¿½ï¿½ï¿½ï¿½Ü‚Å‚Í“ï¿½ï¿½ï¿½ï¿½Ä‚ï¿½ï¿½ï¿½
			Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			/* ã‚½ã‚±ãƒEï¿½ï¿½ã®è¨­å®E*/
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			Debug::print(LOG_PRINT, "FIRE");
			//5å›ã»ã©ç›¸æ‰‹ã«ãƒ¡ãƒEï¿½ï¿½ãƒ¼ã‚¸ã‚’é€ã£ãŸã‚‰çµ‚äºEï¿½ï¿½ã‚E
			/* TCPã‚¯ãƒ©ã‚¤ã‚¢ãƒ³ãƒˆã‹ã‚‰ï¿½Eæ¥ç¶šè¦æ±‚ã‚’å¾Eï¿½ï¿½ã‚‹çŠ¶æ…‹ã«ã™ã‚‹ */
			listen(sock0, 5);
			//while(k < 5){
			/* TCPã‚¯ãƒ©ã‚¤ã‚¢ãƒ³ãƒˆã‹ã‚‰ï¿½Eæ¥ç¶šè¦æ±‚ã‚’å—ã‘ä»˜ã‘ã‚E*/
			len = sizeof(client);
			sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
			/* 5æ–Eï¿½ï¿½é€ä¿¡ */
			Debug::print(LOG_PRINT, "accepted connection from %s, port=%d\n",
				inet_ntoa(client.sin_addr), ntohs(client.sin_port));
			nn = write(sock, "HELLO", 5);
			/* TCPã‚»ãƒEï¿½ï¿½ãƒ§ãƒ³ã®çµ‚äºE*/
			close(sock);
			/* listen ã™ã‚‹socketã®çµ‚äºE*/
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

//å¼•æ•°ã¨ã—ã¦ã‚µãƒ¼ãƒï¿½Eã®IPã‚¢ãƒ‰ãƒ¬ã‚¹ãŒå¿Eï¿½ï¿½E
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
				printf("‘Šè‚ª‘—MƒvƒƒOƒ‰ƒ€‚ğ‹N“®‚µ‚Ä‚È‚¢‚æ");
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
		//ã‚µãƒ¼ãƒãE
		if (args[1].compare("sen") == 0)
		{
			gSend.send();
		}
		//ã‚¯ãƒ©ã‚¤ã‚¢ãƒ³ãƒE
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
//åˆæœŸåŒ–ã™ã‚‹ã‚‚ã®ã¯ã¡ã‚E‚“ã¨æ±ºã‚ã‚‹
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
