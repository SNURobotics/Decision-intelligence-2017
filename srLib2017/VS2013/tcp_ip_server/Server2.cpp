#include "stdafx.h"
#include "Server.h"


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
	SOCKADDR_IN clientaddr;
	int addrlen;
	int retval;
	int servValue = servCount - 1;
	addrlen = sizeof(clientaddr);
	getpeername(client_sock, (SOCKADDR*)&clientaddr, &addrlen);



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
			test_str[servValue][retval] =  '\0';
			break;
		}

		// ���� ������ ���
		buf[retval] = '\0';

		/*
			�޾ƿ� �����͸� ���ʿ��� ó�����ָ� �ȴ�.
			buf ������ ���� �����Ͱ� ����Ǿ� �ִ�.
		*/
		
		//
		strcpy(test_str[servValue], buf);
				
		printf("%d \n> ", retval);
		//printf("[TCP /%s:%d] %s\n> ", inet_ntoa(clientaddr.sin_addr), ntohs(clientaddr.sin_port), recvBuf);
	}
	closesocket(client_sock);

	TerminateThread(hServerThread, 0);
	CloseHandle(hServerThread);

	servCount--;
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

		servCount++;
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

char **Server::RecevData()
{
	char **name;
	name = (char **)malloc(sizeof(char*) * 10);
	for (int i = 0; i < 10; i++) {
		name[i] = (char *)malloc(sizeof(char)*(BUFFER_SIZE + 1));
		strcpy(name[i], test_str[i]);
		strcpy(test_str[i], "");
	}

	return name;
}

Server::~Server()
{
	closesocket(listen_sock);
	// ���� ����
	WSACleanup();
}
