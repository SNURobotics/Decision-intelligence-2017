#include "stdafx.h"
#include "Client.h"

// 사용자 정의 데이터 수신 함수
int recvn(SOCKET s, char *buf, int len, int flags)
{
	int received;
	char *ptr = buf;
	int left = len;

	while (left > 0)
	{
		received = recv(s, ptr, left, flags);
		if (received == SOCKET_ERROR)
			return SOCKET_ERROR;
		else if (received == 0)
			break;
		left -= received;
		ptr += received;
	}

	return (len - left);
}

/**
	메세지 전송을 테스트 하기 위한 terminal 형태의 메세지 전송 commandline
	쓸 필요는 없다.
**/

DWORD WINAPI SendMessageCommandLine(LPVOID arg)
{
	SOCKET sock = (SOCKET)arg;
	char buf[BUFSIZE + 1];
	int len;
	while (1) {
		// 데이터 입력
		ZeroMemory(buf, sizeof(buf));
		if (fgets(buf, BUFSIZE + 1, stdin) == NULL)
			break;

		printf("> ");

		// '\n' 문자 제거
		len = strlen(buf);
		if (buf[len - 1] == '\n')
			buf[len - 1] = '\0';
		if (strlen(buf) == 0)
			break;

		// 데이터 보내기
		int retval = send(sock, buf, strlen(buf), 0);
		if (retval == SOCKET_ERROR)
		{
			printf("송신() 에러");
			break;
		}
	}
	return 0;
}

/**

	서버로부터 데이터를 받아오는 함수
	데이터 처리는 받아온 변수 buf 로부터 처리된다.

**/

DWORD WINAPI RecieveMessage(LPVOID arg)
{
	SOCKET sock = (SOCKET)arg;
	char buf[BUFSIZE + 1];
	//SOCKADDR_IN clientaddr;
	//int addrlen;
	int retval;
	//addrlen = sizeof(clientaddr);
	//getpeername(client_sock, (SOCKADDR*)&clientaddr, &addrlen);

	while (1) {
		// 데이터 초기화
		ZeroMemory(buf, sizeof(buf));
		// 데이터 받기
		retval = recv(sock, buf, BUFSIZE, 0);
		if (retval == SOCKET_ERROR)
		{
			printf("수신() 에러");
			break;
		}
		else if (retval == 0)
		{
			recv_str[retval] = '\0';
			break;
		}
			


		/*

			받아온 데이터를 이쪽에서 처리해주면 된다.
			buf 변수에 받은 데이터가 저장되어 있다.

		*/
		strcpy(recv_str, buf);

		// 받은 데이터 출력
		buf[retval] = '\0';
		//printf("TCP 클라이언트 %d 바이트를 받았습니다.", retval);
		printf("%s\n> ", buf);
		printf("     ...receivedData\n");
	}

	return 0;
}

Client::Client()
{
	isEnd = false;
	int retval;

	// 윈속초기화
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) 
	{
		printf("Error!! WSA Startup error");
		return;
	}

	// socket()
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET) printf("소켓() 에러염");

	// connect()
	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(9000);
	serveraddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	retval = connect(sock, (SOCKADDR*)&serveraddr, sizeof(serveraddr));
	if (retval == SOCKET_ERROR) printf("연결() 에러염");

	// 데이터 통신에 사용할 변수
	char buf[BUFSIZE + 1];
	int len;


	DWORD ServerThreadID;              // 스레드 아이디

	hServerThread = CreateThread(NULL, 0, RecieveMessage, (LPVOID)sock, 0, &ServerThreadID);

	printf("> ");

	/*
	// 서버와 데이터 통신
	while (1)
	{
		if (isEnd)
			break;
		//printf("TCP 클라이언트 %d 바이트를 보냈습니다.", retval);

	}
	*/
}


Client::~Client()
{
	// closesocket()
	closesocket(sock);

	TerminateThread(hServerThread, 0);
	CloseHandle(hServerThread);

	// 윈속 종료
	WSACleanup();
}

/**
	메세지 전송을 테스트 하기 위한 terminal 형태의 메세지 전송 commandline
	을 기동하는 함수
**/

void Client::MakeSendCommandLine()
{
	HANDLE hSendThread;
	DWORD SendThreadID;
	hSendThread = CreateThread(NULL, 0, SendMessageCommandLine, (LPVOID)sock, 0, &SendThreadID);
}

/**
	메세지 전송을 수행하는 함수
	수정할 필요는 없을 것 같다.
**/

void Client::SendMessageToServer(char *buf)
{
	//char sendBuf[4096] = "";
	// '\n' 문자 제거
	int len = strlen(buf);
	if (buf[len - 1] == '\n')
		buf[len - 1] = '\0';
	//buf[len] = '\0';
	//strcat(sendBuf, buf);


	// 데이터 보내기
	int retval = send(sock, buf, 4096, 0);
	if (retval == SOCKET_ERROR)
	{
		printf("송신() 에러");
		return;
	}
}

char * Client::RecevData()
{
	char* name = recv_str;
	return name;
}
