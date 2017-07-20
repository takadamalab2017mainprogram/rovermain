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
class Send : public TaskBase
{
private:
	//Linuxでソケットはintで表現されるファイルディスクリプタ
	int sock0;
	//構造体で簡単に
	struct sockaddr_in addr;
	struct sockaddr_in client;
	//
	int len;
	//
	int sock;
	//エラー処理で用いるnn
	int nn;
	//while文で用いる数（文字を送る回数)
	int k;
protected:
	//引数は何を入れるんだろう
	virtual bool onInit(const struct timespec& time);

	virtual void onClean();

	virtual void onUpdate(const struct timespec& time);

	virtual bool onCommand(const std::vector<std::string>& args);

	//void error_check(sock);
public:
	void send();
	Send();
	~Send();

};
//クライアントクラス
class Rec : public TaskBase
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

	virtual void onUpdate(const struct timespec& time);

	virtual bool onCommand(const std::vector<std::string>& args);
public:
	void receive();
	Rec();
	~Rec();
};
/*
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
*/
extern Send gSend;
extern Rec gRec;
//extern Chat gChat;
