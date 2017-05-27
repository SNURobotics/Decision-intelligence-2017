#pragma once

#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include <thread>

#define BUFSIZE 4096
#define BUFFER_SIZE 4096

static int sendValue;
static char sendBuf[BUFSIZE];

static char test_str[10][BUFFER_SIZE + 1];
static int servCount = 0;

class Server
{
	SOCKET listen_sock;
	HANDLE waitClientThread;
	//SOCKET listen_sock;
	public:
		Server();
		~Server();
		//void ControlTower();
		void WaitServer();
		void SendMessageToClient(char *buf);
		void MakeSendCommandLine();
		
		char **RecevData();
};

