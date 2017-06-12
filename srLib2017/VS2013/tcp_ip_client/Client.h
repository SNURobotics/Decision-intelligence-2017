#pragma once
#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>

#ifndef BUFSIZE
#define BUFSIZE 30000
#endif // !BUFSIZE

#ifndef BUFFER_SIZE
#define BUFFER_SIZE 30000
#endif // !BUFFER_SIZE

static char recv_str[BUFSIZE + 1];

class Client
{
	bool isEnd;
	SOCKET sock;
	HANDLE hServerThread;
public:
	Client();
	~Client();
	void MakeSendCommandLine();
	void SendMessageToServer(char *buf);

	char *RecevData();
};

