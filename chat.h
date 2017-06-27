//一度だけプログラムを呼び出す宣言
#pragma once
#include "task.h"

//マルチーズ追加raspberrypi同士でチャット（お互いの情報を送る）をするプログラム
/*
これはLinux上で動くC++を用いたTCP通信のサーバープログラムです。
参考サイト(Geekなページ)　http://www.geekpage.jp/programming/winsock/tcp.php
(Game engineering MAGAZINE上記だけだとエラーが出る) http://gmagazine00.blog115.fc2.com/blog-category-2.html
*/

//サーバークラス
class Server
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

public:
	//引数は何を入れるんだろう
	bool init();

	void clean();

	void update();
	//エラーチェック用の関数
	void error_check(sock);

	Server();
	~Server();

};
//クライアントクラス
class Client
{
private:
	//構造体サーバーに関する
	struct sock addr_in_server;
	int sock;
	//送信する文字
	char buf[32];
    //文字数
	int n;
	
public:

	bool init();

	void clean();

	void update();

	Client();
	~Client();
};