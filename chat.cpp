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

//20170630繝槭Ν繝√・繧ｺ霑ｽ蜉繝√Ε繝・ヨ繝励Ο繧ｰ繝ｩ繝
//譁・ｭ怜・蟋斐ｒ蜿励￠縺ｨ繧虐erver縺ｮ繧ｻ繝・ヨ繧｢繝・・
//Sendクラスは相手にメッセージが送られるまで実行される。
//送られたらsockを閉じて終了、現在は送られないとプログラムの終了ができなくなる。
bool Send::onInit(const struct timespec& time)
{
	return true;
}

//菴募ｺｦ繧よ磁邯夊ｦ∵ｱょ女莉倥ｒ隧ｦ縺ｿ繧・
void Send::onUpdate(const struct timespec& time)
{
}
//sock謫堺ｽ懊ｒ荳遶ｯ邨ゆｺ・ｼ磯崕蜉帶ｶ郁ｲｻ霆ｽ貂帙ｉ縺励＞・滂ｼ・
void Send::onClean()
{
}
bool Send::onCommand(const vector<string>& args)
{
  if(args.size()==2)
	{
		if (args[1].compare("sen")==0)
		{
			/* 繧ｽ繧ｱ繝・ヨ縺ｮ菴懈・ */
			sock0 = socket(AF_INET, SOCK_STREAM, 0);
			//ここまでは動いている
			Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			/* 繧ｽ繧ｱ繝・ヨ縺ｮ險ｭ螳・*/
			addr.sin_family = AF_INET;
			addr.sin_port = htons(12345);
			addr.sin_addr.s_addr = INADDR_ANY;
			bind(sock0, (struct sockaddr *)&addr, sizeof(addr));
			Debug::print(LOG_PRINT, "FIRE");
			//5蝗槭⊇縺ｩ逶ｸ謇九↓繝｡繝・そ繝ｼ繧ｸ繧帝√▲縺溘ｉ邨ゆｺ・☆繧・
			/* TCP繧ｯ繝ｩ繧､繧｢繝ｳ繝医°繧峨・謗･邯夊ｦ∵ｱゅｒ蠕・※繧狗憾諷九↓縺吶ｋ */
			listen(sock0, 5);
			//while(k < 5){
			/* TCP繧ｯ繝ｩ繧､繧｢繝ｳ繝医°繧峨・謗･邯夊ｦ∵ｱゅｒ蜿励￠莉倥￠繧・*/
			len = sizeof(client);
			sock = accept(sock0, (struct sockaddr *)&client, (socklen_t *)&len);
			/* 5譁・ｭ鈴∽ｿ｡ */
			Debug::print(LOG_PRINT, "accepted connection from %s, port=%d\n",
				inet_ntoa(client.sin_addr), ntohs(client.sin_port));
			send(sock, "HELLO", 5);
			/* TCP繧ｻ繝・す繝ｧ繝ｳ縺ｮ邨ゆｺ・*/
			close(sock);
			/* listen 縺吶ｋsocket縺ｮ邨ゆｺ・*/
			close(sock0);
			return true;
		}
		return false;
//		}
	}
  else {
	  Debug::print(LOG_PRINT, "chat_s              : show chat state\r\n\
chat_s sen: send messeage to client\r\n\"");
	  return true;
  }
}

Send::Send()
{
	setName("chat_s");
	setPriority(TASK_PRIORITY_SEND, TASK_INTERVAL_SEND);
}

Send::~Send()
{
}

//蠑墓焚縺ｨ縺励※繧ｵ繝ｼ繝舌・縺ｮIP繧｢繝峨Ξ繧ｹ縺悟ｿ・ｦ・
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
			/* 繧ｽ繧ｱ繝・ヨ縺ｮ菴懈・ */
			sock = socket(AF_INET, SOCK_STREAM, 0);
			//ここまでは動いている
			Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			/* 謗･邯壼・謖・ｮ夂畑讒矩菴薙・貅門ｙ */
			server.sin_family = AF_INET;
			server.sin_port = htons(12345);
			server.sin_addr.s_addr = inet_addr("10.0.0.12");

			/* 繧ｵ繝ｼ繝舌↓謗･邯・*/
			connect(sock, (struct sockaddr *)&server, sizeof(server));

			/* 繧ｵ繝ｼ繝舌°繧峨ョ繝ｼ繧ｿ繧貞女菫｡ */
			memset(buf, 0, sizeof(buf));
			//ここまで動いている
			Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			n = read(sock, buf, sizeof(buf));
			if (n < 0) {
				perror("read");
				printf("逶ｸ謇九・繝励Ο繧ｰ繝ｩ繝縺九ｉ菴輔ｂ騾√ｉ繧後※縺阪※縺ｪ縺・ｈ");
				return 1;
			}
			Debug::print(LOG_PRINT, "Buzzer is already stopping\r\n");
			Debug::print(LOG_PRINT,"%d, %s\n", n, buf);
			/* socket縺ｮ邨ゆｺ・*/
			close(sock);
			return true;
		}
		//Debug::print(LOG_PRINT, "FIREF");
		return false;
	}
	else {
		Debug::print(LOG_PRINT, "chat_r              : show chat state\r\n\
chat_r rec: recieve message from server\r\n\"");
		return true;
	}
}
//繝ｬ繧ｷ繝ｼ繝夜未謨ｰ

Rec::Rec():buf(),n(0)
{
	setName("chat_r");
	setPriority(TASK_PRIORITY_REC, TASK_INTERVAL_REC);
}

Rec::~Rec()
{
}

/*
//繧ｵ繝ｼ繝舌・縺ｨ繧ｯ繝ｩ繧､繧｢繝ｳ繝医ｒ縺ｾ縺ｨ繧√◆繧ｯ繝ｩ繧ｹ
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
//郢ｧ・ｯ郢晢ｽｩ郢ｧ・､郢ｧ・｢郢晢ｽｳ郢晏現繝ｻ郢ｧ・､郢晢ｽｳ郢ｧ・ｹ郢ｧ・ｿ郢晢ｽｳ郢ｧ・ｹ郢ｧ蜑・ｽｽ諛奇ｽ狗ｸｺ・ｨ郢晏干ﾎ溽ｹｧ・ｰ郢晢ｽｩ郢擒邵ｺ讙趣ｽｵ繧・ｽｺ繝ｻ笘・ｹｧ繝ｻ
Rec gRec;
//Chat gChat;
