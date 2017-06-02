#include "stdafx.h"
#include "Server.h"

#define SEOUL "192.168.137.102"
#define HANYANG "192.168.137.103"
#define SUNGGEUN "192.168.137.104"
#define ROBOT01 "192.168.137.100"
#define ROBOT02 "192.168.137.101"
/**
	�޼��� ������ �׽�Ʈ �ϱ� ���� terminal ������ �޼��� ���� commandline
	�� �ʿ�� ����.
**/

DWORD WINAPI ControlTower(LPVOID arg)
{
	char buf[BUFSIZE + 1];
	// Ŭ���̾�Ʈ�� ������ ���

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

	// ������ ������
	int retval = send(*client_sock, buffer, strlen(buffer), 0);

	if (retval == SOCKET_ERROR)
	{
		printf("�۽�() ����\n> ");
		return -1;
	}
	return 1;
}


/**

	Thread�� ���ư��� �Լ��� 
	Ŭ���̾�Ʈ�� ������ŭ ����ȴ�.
	������ Ŭ���̾�Ʈ�� �پ �޼����� ������ ������ �ϸ�
	Control Tower�κ��� ���� ������ �޼����� �޾Ƽ� 
	����ϰ� �ִ� �ϳ��� Ŭ���̾�Ʈ���� �޼����� ������.
	Ȥ Ŭ���̾�Ʈ�� Ư���ؼ� ������ �ʹٸ� ���⿡ �ڵ带 �����ϸ� �� ���̴�.

**/

DWORD WINAPI SendClient(LPVOID arg)
{
	SOCKET client_sock = (SOCKET)arg;
	int sendClient = sendValue;
	SOCKADDR_IN clientaddr;
	int addrlen;
	addrlen = sizeof(clientaddr);
	getpeername(client_sock, (SOCKADDR*)&clientaddr, &addrlen);

	// Ŭ���̾�Ʈ�� ������ ���
	while (1)
	{
		if (sendClient != sendValue) {
			sendClient = sendValue;

			/*

				���� �����ʹ� sendBuf�� ����Ǿ� �ִ�.
				sendBuf�� ������ �о�鿩 client���� ������ ������ 
				���⿡ �ڵ带 �����ؼ� ������ ���� �ִ�.
			
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

	Ŭ���̾�Ʈ�κ��� �����͸� �޾ƿ��� �Լ�
	������ ó���� �޾ƿ� ���� buf �κ��� ó���ȴ�.

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



	HANDLE hServerThread;              // ������ �ڵ�
	DWORD ServerThreadID;              // ������ ���̵�

	hServerThread = CreateThread(NULL, 0, SendClient, (LPVOID)client_sock, 0, &ServerThreadID);

	// Ŭ���̾�Ʈ�� ������ ���
	while (1)
	{
		retval = recv(client_sock, buf, BUFSIZE, 0);
		if (retval == SOCKET_ERROR)
		{
			printf("����() ����\n> ");
			break;
		}
		else if (retval == 0) {
			test_str[testIndex][retval] =  '\0';
			break;
		}

		// ���� ������ ���
		buf[retval] = '\0';

		/*
			�޾ƿ� �����͸� ���ʿ��� ó�����ָ� �ȴ�.
			buf ������ ���� �����Ͱ� ����Ǿ� �ִ�.
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

	printf("TCP ����, Ŭ���̾�Ʈ ���� : IP �ּ� = %s, ��Ʈ��ȣ = %d\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));
	return 0;
}

// Ŭ���̾�Ʈ�� ������ ��ٸ��� ���Ӷ����� ���ο� thread�� �����ؼ� �������ش�
DWORD WINAPI WaitClientLoop(LPVOID arg)
{
	SOCKET listen_sock = (SOCKET)arg;
	// ������ ��ſ� ����� ����
	SOCKET client_sock;
	SOCKADDR_IN clientaddr;
	int addrlen;
	HANDLE hThread;              // ������ �ڵ�
	DWORD ThreadID;              // ������ ���̵�

	while (1)
	{
		// accept()
		addrlen = sizeof(clientaddr);

		client_sock = accept(listen_sock, (SOCKADDR*)&clientaddr, &addrlen);
		if (client_sock == INVALID_SOCKET) {
			printf("accept() ����\n");
			continue;
		}

		printf("TCP ����, Ŭ���̾�Ʈ ���� : IP �ּ� = %s, ��Ʈ��ȣ = %d\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port));
		hThread = CreateThread(NULL, 0, ProcessClient, (LPVOID)client_sock, 0, &ThreadID);
		if (hThread == NULL)
		{
			printf("������ ���� ����\n");
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

	// �����ʱ�ȭ
	WSADATA wsa;

	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
	{
		printf("Error!! WSA Startup error");
		return;
	}

	// ���� �� �����ų ����
	int return_val;

	// socket() 
	listen_sock = socket(AF_INET, SOCK_STREAM, 0);
	if (listen_sock == INVALID_SOCKET) printf("����() ����\n");

	long port;
	//port = atol(argv[1]);
	port = 9000;

	// bind()
	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(port);
	/*
	XP �������� INADDR_ANY�� ���� �� ��Ʈ���� �Ѱ��� ��� INADDR_ANY�� ���õǾ�����,
	2003 ���ķδ� INADDR_NONE�� �������� �Ѱ��ش�, INADDR_NONE�� IP�뿪�� A.B.C.D �� �ϳ��� 255�� �ʰ� �� ��� ����
	INADDR_ANY�� ��� �ּҷ� �����ϴ� ������ �޾Ƶ��δ�.
	*/

	serveraddr.sin_addr.s_addr = htons(INADDR_ANY);
	return_val = bind(listen_sock, (SOCKADDR*)&serveraddr, sizeof(serveraddr)); // connect�� �ƴ϶� bind

	if (return_val == SOCKET_ERROR) printf("���ε�() ����\n");

	// listen()
	return_val = listen(listen_sock, SOMAXCONN);
	if (return_val == SOCKET_ERROR) printf("����() ������\n");

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
	�޼��� ������ �����ϴ� �Լ�
	��ü �޼��� ���ۿ� �ش��ϴ� ������ �Ѵ�.
	������ �ʿ�� ���� �� ����.
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
	HANDLE hServerThread;              // ������ �ڵ�
	DWORD ServerThreadID;              // ������ ���̵�
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
	// ���� ����
	WSACleanup();
}
