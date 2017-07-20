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

//20170630ãã«ããEãºè¿½å ãã£ãEãã­ã°ã©ã 
//æE­åEå§ãåãã¨ãserverã®ã»ãEã¢ãEE

bool Send::onInit(const struct timespec& time)
{
	/*
	//ã½ã±ãEã®ä½æE
	//å¼æ°ã¯ã¢ãã¬ã¹ãã¡ããªãã½ã±ãEã¿ã¤ãããEã­ãã³ã«
	sock0 = socket(AF_INET, SOCK_STREAM, 0);

	//sockãE1ãè¿ãããå¤±æE
	if (sock < 0)
	{
		//ã¨ã©ã¼ãè¡¨ç¤ºããå¦çE
		perror("socketã®ã¨ã©ã¼ãåEã¾ãã");
		printf("%d\n", errno);
		//return 1;
	}
	//ã½ã±ãEã®è¨­å®E
	addr.sin_family = AF_INET;
	addr.sin_port = htons(12345);
	addr.sin_addr.s_addr = INADDR_ANY;
	bind(sock0, (struct sockaddr *)&addr, sizeof(addr));

	//TCPã¯ã©ã¤ã¢ã³ããããEæ¥ç¶è¦æ±ãå¾E¦ãç¶æã«ãã
	listen(sock0, 5);
	return true;
	*/
}

//ä½åº¦ãæ¥ç¶è¦æ±åä»ãè©¦ã¿ãE
void Send::onUpdate(const struct timespec& time)
{
	/*
	//TCPã¯ã©ã¤ã¢ã³ããããEæ¥ç¶è¦æ±ãåãä»ããE
	len = sizeof(client);
	sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
	*/
}
//sockæä½ãä¸ç«¯çµäºE¼é»åæ¶è²»è»½æ¸ãããEï¼E
void Send::onClean()
{
	/*
	//listenããsocketã®çµäºE
	close(sock0);
	mes = NULL;
	*/
}
bool Send::onCommand(const vector<string>& args)
{
  if(args.size()==2)
	{
		if (args[1].compare("sen"))
		{
			/* ã½ã±ãEã®ä½æE */
			sock0 = socket(AF_INET, SOCK_STREAM, 0);

			/* ã½ã±ãEã®è¨­å®E*/
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			//5åã»ã©ç¸æã«ã¡ãE»ã¼ã¸ãéã£ããçµäºEãE
			/* TCPã¯ã©ã¤ã¢ã³ããããEæ¥ç¶è¦æ±ãå¾E¦ãç¶æã«ãã */
			listen(sock0, 5);
			while(k < 5){
			/* TCPã¯ã©ã¤ã¢ã³ããããEæ¥ç¶è¦æ±ãåãä»ããE*/
			len = sizeof(client);
			sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
			/* 5æE­éä¿¡ */
			nn = write(sock, "HELLO", 5);
			if (nn < 1)
			{
				perror("write");
				break;
			}
			/* TCPã»ãE·ã§ã³ã®çµäºE*/
		   	 k++;
			close(sock);
			}
			/* listen ããsocketã®çµäºE*/
			close(sock0);
		}
//		}
	}
  else {
	  Debug::print(LOG_PRINT, "chat_s              : show chat state\r\n\
chat_s sen: send messeage to client\r\n\"");
	  return true;
  }
/*
void Send::send()
{
	//clientã«5æE­HELLOãéã
	write(sock, "KOUKI", 5);
	close(sock);
}
*/
}

Send::Send():sock(0),sock0(0)
{
	setName("chat_s");
	setPriority(TASK_PRIORITY_SEND, TASK_INTERVAL_SEND);
}

Send::~Send()
{
}

//å¼æ°ã¨ãã¦ãµã¼ããEã®IPã¢ãã¬ã¹ãå¿E¦E
bool Rec::onInit(const struct timespec& time)
{
	//ã½ã±ãEã®ä½æE
	//å¼æ°ã¯ã¢ãã¬ã¹ãã¡ããªãã½ã±ãEã¿ã¤ãããEã­ãã³ã«
	//sock = socket(AF_INET, SOCK_STREAM, 0);

	//ã½ã±ãEã®è¨­å®E
	//server.sin_family = AF_INET;
	//server.sin_port = htons(12345);
	//server.sin_addr.s_addr = inet_addr("10.0.0.5");

	return true;
}

void Rec::onUpdate(const struct timespec& time)
{
  //ã½ã±ãEã®ä½æE
  //å¼æ°ã¯ã¢ãã¬ã¹ãã¡ããªãã½ã±ãEã¿ã¤ãããEã­ãã³ã«
	//sockããµã¼ããEã®
  
	/* ãµã¼ãã«æ¥ç¶E*/
	//connect(sock1, (struct sockaddr *)&server, sizeof(server));
}

void Rec::onClean()
{
}

bool Rec::onCommand(const std::vector<std::string>& args)
{
	if(args.size()==2)
	{
		if (args[1].compare("rec"))
		{
			/* ã½ã±ãEã®ä½æE */
			sock1 = socket(AF_INET, SOCK_STREAM, 0);

			/* æ¥ç¶åEæE®ç¨æ§é ä½ãEæºå */
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("10.0.0.12");

			/* ãµã¼ãã«æ¥ç¶E*/
			connect(sock1, (struct sockaddr *)&server, sizeof(server));

			/* ãµã¼ããããã¼ã¿ãåä¿¡ */
			memset(buf, 0, sizeof(buf));
			n = read(sock1, buf, sizeof(buf));
			if (n < 0) {
				perror("read");
				printf("ç¸æãEãã­ã°ã©ã ããä½ãéããã¦ãã¦ãªãE");
				return 1;
			}

			printf("%d, %s\n", n, buf);

			/* socketã®çµäºE*/
			close(sock1);
			
		}
	}
	else {
		Debug::print(LOG_PRINT, "chat_r              : show chat state\r\n\
chat_r rec: recieve message from server\r\n\"");
	}
	return true;
}
//ã¬ã·ã¼ãé¢æ°
/*
void Rec::receive() 
{
	sock1 = socket(AF_INET, SOCK_STREAM, 0);

	//ã½ã±ãEã®è¨­å®E
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
	setPriority(TASK_PRIORITY_SEND, TASK_INTERVAL_SEND);
}

Rec::~Rec()
{
}

/*
//ãµã¼ããEã¨ã¯ã©ã¤ã¢ã³ããã¾ã¨ããã¯ã©ã¹
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
		//ãµã¼ããE
		if (args[1].compare("sen") == 0)
		{
			gSend.send();
		}
		//ã¯ã©ã¤ã¢ã³ãE
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
//åæåãããã®ã¯ã¡ãEã¨æ±ºãã
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
//ç¹§E¯ç¹ï½©ç¹§E¤ç¹§E¢ç¹ï½³ç¹å»ã»ç¹§E¤ç¹ï½³ç¹§E¹ç¹§E¿ç¹ï½³ç¹§E¹ç¹§åE½½æï½ç¸ºE¨ç¹å±Îç¹§E°ç¹ï½©ç¹ ç¸ºæªï½µãE½ºã»âE¹§ã»
Rec gRec;
//Chat gChat;
