//一度だけプログラムを呼び出す宣言
#pragma once
#include "task.h"
#include <sys/socket.h>
//マルチーズ追加raspberrypi同士でチャット（お互いの情報を送る）をするプログラム
/*
これはLinux上で動くC++を用いたTCP通信のサーバープログラムです。
参考サイト(Geekなページ)　http://www.geekpage.jp/programming/winsock/tcp.php
(Game engineering MAGAZINE上記だけだとエラーが出る) http://gmagazine00.blog115.fc2.com/blog-category-2.html
*/

//サーバークラス
class Server : public TaskBase
{
private:
	//Linuxでソケットはintで表現されるファイルディスクリプタ
	int sock0;

	//構造体で簡単に
	struct sockaddr_in addr;
	struct sockaddr_in client;

	//送信するメッセージ
	char* mes;

	//よく分かってない
	int len;

	//sock0と何が違うんだろう
	int sock;

	//クライアントから取得する文字列32
	char buf_client[32];

protected:
	//引数は何を入れるんだろう
	virtual bool onInit();

	virtual void onClean();

	virtual void onUpdate(double x);

	virtual bool onCommand(const std::vector<std::string>& args);

	//void error_check(sock);
public:
	void send();
	Server();
	~Server();

};
//クライアントクラス
class Client : public TaskBase
{
private:
	//構造体サーバーに関する
	struct sockaddr_in server;
	int sock1;
	//送信する文字
	char buf[32];
	//文字数
	int n;
protected:
	//int yは適当な値なのでのちのち修正
	virtual bool onInit(const struct timespec& time);

	virtual void onClean();

	virtual void onUpdate();

	virtual bool onCommand(const std::vector<std::string>& args);
public:
	void receive();
	Client();
	~Client();
};

class Chat : public TaskBase
{
private:
protected:
	virtual bool onInit(const struct timespec& time);
	virtual void onClean();
	virtual bool onCommand(const std::vector<std::string>& args);
public:
	Chat();
	~Chat();
};

extern Server gServer;
extern Client gClient;
extern Chat gChat;
