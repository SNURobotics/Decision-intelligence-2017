#include "stdafx.h"
#include "Server.h"

#define SEOUL "192.168.137.102"
#define HANYANG "192.168.137.103"
#define SUNGGEUN "192.168.137.104"
#define ROBOT01 "192.168.137.100"
#define ROBOT02 "192.168.137.101"
/**
	메세지 전송을 테스트 하기 위한 terminal 형태의 메세지 전송 commandline
	쓸 필요는 없다.
**/

DWORD WINAPI ControlTower(LPVOID arg)
{
	char buf[BUFSIZE + 1];
	// 클라이언트와 데이터 통신

	while (1)
	{

		ZeroMemory(buf, sizeof(buf));
		fgets(buf, BUFSIZE + 1, stdin);
		strcpy(sendBuf, buf);
		sendValue++;
		if (sendValue > 2048)
			sendValue = 0;

		printf("> ");

	}

	return 0;
}


int SendMessageToClient(char *buffer, void *value) {
	SOCKET *client_sock = (SOCKET *)value;
	int len = strlen(buffer);
	if (buffer[len - 1] == '\n')
		buffer[len - 1] = '\0';

	// 데이터 보내기
	int retval = send(*client_sock, buffer, strlen(buffer), 0);

	if (retval == SOCKET_ERROR)
	{
		printf("송신() 에러\n> ");
		return -1;
	}
	return 1;
}


/**

	Thread로 돌아가는 함수로 
	클라이언트의 개수만큼 실행된다.
	각각의 클라이언트와 붙어서 메세지를 보내는 역할을 하며
	Control Tower로부터 나온 서버의 메세지를 받아서 
	담당하고 있는 하나의 클라이언트에게 메세지를 보낸다.
	혹 클라이언트를 특정해서 보내고 싶다면 여기에 코드를 수정하면 될 것이다.

**/

DWORD WINAPI SendClient(LPVOID arg)
{
	SOCKET client_sock = (SOCKET)arg;
	int sendClient = sendValue;
	SOCKADDR_IN clientaddr;
	int addrlen;
	addrlen = sizeof(clientaddr);
	getpeername(client_sock, (SOCKADDR*)&clientaddr, &addrlen);

	// 클라이언트와 데이터 통신
	while (1)
	{
		if (sendClient != sendValue) {
			sendClient = sendValue;

			/*

				보낼 데이터는 sendBuf에 저장되어 있다.
				sendBuf의 형식을 읽어들여 client에게 보낼지 말지를 
				여기에 코드를 수정해서 결정할 수도 있다.
			
			*/
			if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT01) == 0 && (sendBuf[0] == 'I' || sendBuf[0] == 'V' || sendBuf[0] == 'R' || sendBuf[1] == '2'))
				continue;
			if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT02) == 0 && (sendBuf[0] == 'I' || sendBuf[0] == 'V' || sendBuf[0] == 'R' || sendBuf[1] == '1'))
				continue;
			else if (strcmp(inet_ntoa(clientaddr.sin_addr), HANYANG) == 0 && (sendBuf[0] == 'G' || sendBuf[0] == 'I' || sendBuf[0] == 'P' || sendBuf[0] == 'S' || sendBuf[0] == 'J'))
				continue;
			else if (strcmp(inet_ntoa(clientaddr.sin_addr), SUNGGEUN) == 0 && (sendBuf[0] == 'V' || sendBuf[0] == 'G' || sendBuf[0] == 'R' || sendBuf[0] == 'S' || sendBuf[0] == 'P' || sendBuf[0] == 'J'))
				continue;
			else
				SendMessageToClient(sendBuf, &client_sock);

		}

	}
	closesocket(client_sock);
	return 0;
}

/**

	클라이언트로부터 데이터를 받아오는 함수
	데이터 처리는 받아온 변수 buf 로부터 처리된다.

**/

DWORD WINAPI ProcessClient(LPVOID arg)
{
	SOCKET client_sock = (SOCKET)arg;
	char buf[BUFSIZE + 1];
	int addrlen;
	int retval;
	SOCKADDR_IN clientaddr;
	addrlen = sizeof(clientaddr);
	getpeername(client_sock, (SOCKADDR*)&clientaddr, &addrlen);
	int testIndex = 0;
	for (int i = 0; i < 10; i++) {
		testIndex = i;
		if (strcmp(test_str[i], "\n\n\0") == 0)
			break;
	}



	HANDLE hServerThread;              // 스레드 핸들
	DWORD ServerThreadID;              // 스레드 아이디

	hServerThread = CreateThread(NULL, 0, SendClient, (LPVOID)client_sock, 0, &ServerThreadID);

	// 클라이언트와 데이터 통신
	while (1)
	{
		retval = recv(client_sock, buf, BUFSIZE, 0);
		if (retval == SOCKET_ERROR)
		{
			printf("수신() 에러\n> ");
			break;
		}
		else if (retval == 0) {
			test_str[testIndex][retval] =  '\0';
			break;
		}

		// 받은 데이터 출력
		buf[retval] = '\0';

		/*
			받아온 데이터를 이쪽에서 처리해주면 된다.
			buf 변수에 받은 데이터가 저장되어 있다.
		*/
		char newBuf[BUFSIZE + 1] = "";
		if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT02) == 0 && buf[0] == 'R') {
			int len = strlen(buf);
			strcat(newBuf, "R1");
			strncpy(buf, buf + 1, len - 1);
			strcat(newBuf, buf);
			strcpy(test_str[testIndex], newBuf);
		}
		else if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT02) == 0 && buf[0] == 'R') {
			int len = strlen(buf);
			strcat(newBuf, "R2");
			strncpy(buf, buf + 1, len - 1);
			strcat(newBuf, buf);
			strcpy(test_str[testIndex], newBuf);
		}
		else if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT02) == 0 && buf[0] == 'P') {
			int len = strlen(buf);
			strcat(newBuf, "P1");
			strncpy(buf, buf + 1, len - 1);
			strcat(newBuf, buf);
			strcpy(test_str[testIndex], newBuf);
		}
		else if (strcmp(inet_ntoa(clientaddr.sin_addr), ROBOT02) == 0 && buf[0] == 'P') {
			int len = strlen(buf);
			strcat(newBuf, "P2");
			strncpy(buf, buf + 1, len - 1);
			strcat(newBuf, buf);
			strcpy(test_str[testIndex], newBuf);
		}
		//
		else
			strcpy(test_str[testIndex], buf);

				
		printf("%d[%s] \n> ", retval, inet_ntoa(clientaddr.sin_addr));
		//printf("[TCP /%s:%d] %s\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port), recvBuf);
	}
	closesocket(client_sock);

	TerminateThread(hServerThread, 0);
	CloseHandle(hServerThread);
	strcpy(test_str[testIndex], "\n\n\0");

	printf("TCP 서버, 클라이언트 종료 : IP 주소 = %s, 포트번호 = %d\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));
	return 0;
}

// 클라이언트의 접속을 기다리고 접속때마다 새로운 thread를 생성해서 연결해준다
DWORD WINAPI WaitClientLoop(LPVOID arg)
{
	SOCKET listen_sock = (SOCKET)arg;
	// 데이터 통신에 사용할 변수
	SOCKET client_sock;
	SOCKADDR_IN clientaddr;
	int addrlen;
	HANDLE hThread;              // 스레드 핸들
	DWORD ThreadID;              // 스레드 아이디

	while (1)
	{
		// accept()
		addrlen = sizeof(clientaddr);

		client_sock = accept(listen_sock, (SOCKADDR*)&clientaddr, &addrlen);
		if (client_sock == INVALID_SOCKET) {
			printf("accept() 에러\n");
			continue;
		}

		printf("TCP 서버, 클라이언트 접속 : IP 주소 = %s, 포트번호 = %d\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));
		hThread = CreateThread(NULL, 0, ProcessClient, (LPVOID)client_sock, 0, &ThreadID);
		if (hThread == NULL)
		{
			printf("스레드 생성 실패\n");
		}
		else
		{
			CloseHandle(hThread);
		}

	}
	return 0;
}


Server::Server()
{
	sendValue = 0;

	// 윈속초기화
	WSADATA wsa;

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Error!! WSA Startup error");
		return;
	}

	// 리턴 값 저장시킬 변수
	int return_val;

	// socket() 
	listen_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_sock == INVALID_SOCKET) printf("소켓() 에러\n");

	long port;
	//port = atol(argv[1]);
	port = 9000;

	// bind()
	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(port);
	/*
	XP 이전에는 INADDR_ANY의 값에 빈 스트링을 넘겨줄 경우 INADDR_ANY로 세팅되었지만,
	2003 이후로는 INADDR_NONE의 에러값을 넘겨준다, INADDR_NONE은 IP대역이 A.B.C.D 중 하나라도 255를 초과 할 경우 세팅
	INADDR_ANY는 어느 주소로 접속하던 접속을 받아들인다.
	*/

	serveraddr.sin_addr.s_addr = htons(INADDR_ANY);
	return_val = bind(listen_sock, (SOCKADDR*)&serveraddr, sizeof(serveraddr)); // connect가 아니라 bind

	if (return_val == SOCKET_ERROR) printf("바인딩() 에러\n");

	// listen()
	return_val = listen(listen_sock, SOMAXCONN);
	if (return_val == SOCKET_ERROR) printf("리슨() 에러염\n");

	printf("> ");

	for (int i = 0; i < 10; i++) {
		strcpy(test_str[i], "\n\n\0");
	}
	//std::thread control(&Server::ControlTower, this);

	
	DWORD WaitClientThreadID;
	waitClientThread = CreateThread(NULL, 0, WaitClientLoop, (LPVOID)listen_sock, 0, &WaitClientThreadID);
	
}

void Server::WaitServer()
{
	WaitForSingleObject(waitClientThread, INFINITE);

}

/**
	메세지 전송을 수행하는 함수
	전체 메세지 전송에 해당하는 역할을 한다.
	수정할 필요는 없을 것 같다.
**/

void Server::SendMessageToClient(char *buf)
{
	strcpy(sendBuf, buf);
	sendValue++;
	if (sendValue > 2048)
		sendValue = 0;

}

void Server::MakeSendCommandLine()
{
	HANDLE hServerThread;              // 스레드 핸들
	DWORD ServerThreadID;              // 스레드 아이디
	hServerThread = CreateThread(NULL, 0, ControlTower, NULL, 0, &ServerThreadID);
}

char *Server::RecevData()
{
	for (int i = 0; i < 10; i++) {
		int index = (receiveIndex + 1 + i) % 10;
		if (strcmp(test_str[index], "") == 0 || strcmp(test_str[index], "\0") == 0 || strcmp(test_str[index], "\n\n\0") == 0)
			continue;
		else {
			char *name = (char *)malloc(sizeof(char)*BUFFER_SIZE);
			printf(test_str[index]);
			printf("             asfda\n");
			strcpy(name, test_str[index]);
			int len = strlen(name);
			if (name[len - 1] == '\n')
				name[len - 1] = '\0';
			receiveIndex = index;
			strcpy(test_str[index], "\0");
			return name;
		}

	}
	return "\0";
}

Server::~Server()
{
	closesocket(listen_sock);
	// 윈속 종료
	WSACleanup();
}
