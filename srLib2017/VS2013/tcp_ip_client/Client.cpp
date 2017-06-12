#include "stdafx.h"
#include "Client.h"

// ����� ���� ������ ���� �Լ�
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
	�޼��� ������ �׽�Ʈ �ϱ� ���� terminal ������ �޼��� ���� commandline
	�� �ʿ�� ����.
**/

DWORD WINAPI SendMessageCommandLine(LPVOID arg)
{
	SOCKET sock = (SOCKET)arg;
	char buf[BUFSIZE + 1];
	int len;
	while (1) {
		// ������ �Է�
		ZeroMemory(buf, sizeof(buf));
		if (fgets(buf, BUFSIZE + 1, stdin) == NULL)
			break;

		printf("> ");

		// '\n' ���� ����
		len = strlen(buf);
		if (buf[len - 1] == '\n')
			buf[len - 1] = '\0';
		if (strlen(buf) == 0)
			break;

		// ������ ������
		int retval = send(sock, buf, strlen(buf), 0);
		if (retval == SOCKET_ERROR)
		{
			printf("�۽�() ����");
			break;
		}
	}
	return 0;
}

/**

	�����κ��� �����͸� �޾ƿ��� �Լ�
	������ ó���� �޾ƿ� ���� buf �κ��� ó���ȴ�.

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
		// ������ �ʱ�ȭ
		ZeroMemory(buf, sizeof(buf));
		// ������ �ޱ�
		retval = recv(sock, buf, BUFSIZE, 0);
		if (retval == SOCKET_ERROR)
		{
			printf("����() ����");
			break;
		}
		else if (retval == 0)
		{
			recv_str[retval] = '\0';
			break;
		}
			


		/*

			�޾ƿ� �����͸� ���ʿ��� ó�����ָ� �ȴ�.
			buf ������ ���� �����Ͱ� ����Ǿ� �ִ�.

		*/
		strcpy(recv_str, buf);

		// ���� ������ ���
		buf[retval] = '\0';
		//printf("TCP Ŭ���̾�Ʈ %d ����Ʈ�� �޾ҽ��ϴ�.", retval);
		printf("%s\n> ", buf);
		printf("     ...receivedData\n");
	}

	return 0;
}

Client::Client()
{
	isEnd = false;
	int retval;

	// �����ʱ�ȭ
	WSADATA wsa;
	if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) 
	{
		printf("Error!! WSA Startup error");
		return;
	}

	// socket()
	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET) printf("����() ������");

	// connect()
	SOCKADDR_IN serveraddr;
	ZeroMemory(&serveraddr, sizeof(serveraddr));
	serveraddr.sin_family = AF_INET;
	serveraddr.sin_port = htons(9000);
	serveraddr.sin_addr.s_addr = inet_addr("127.0.0.1");
	retval = connect(sock, (SOCKADDR*)&serveraddr, sizeof(serveraddr));
	if (retval == SOCKET_ERROR) printf("����() ������");

	// ������ ��ſ� ����� ����
	char buf[BUFSIZE + 1];
	int len;


	DWORD ServerThreadID;              // ������ ���̵�

	hServerThread = CreateThread(NULL, 0, RecieveMessage, (LPVOID)sock, 0, &ServerThreadID);

	printf("> ");

	/*
	// ������ ������ ���
	while (1)
	{
		if (isEnd)
			break;
		//printf("TCP Ŭ���̾�Ʈ %d ����Ʈ�� ���½��ϴ�.", retval);

	}
	*/
}


Client::~Client()
{
	// closesocket()
	closesocket(sock);

	TerminateThread(hServerThread, 0);
	CloseHandle(hServerThread);

	// ���� ����
	WSACleanup();
}

/**
	�޼��� ������ �׽�Ʈ �ϱ� ���� terminal ������ �޼��� ���� commandline
	�� �⵿�ϴ� �Լ�
**/

void Client::MakeSendCommandLine()
{
	HANDLE hSendThread;
	DWORD SendThreadID;
	hSendThread = CreateThread(NULL, 0, SendMessageCommandLine, (LPVOID)sock, 0, &SendThreadID);
}

/**
	�޼��� ������ �����ϴ� �Լ�
	������ �ʿ�� ���� �� ����.
**/

void Client::SendMessageToServer(char *buf)
{
	//char sendBuf[4096] = "";
	// '\n' ���� ����
	int len = strlen(buf);
	if (buf[len - 1] == '\n')
		buf[len - 1] = '\0';
	//buf[len] = '\0';
	//strcat(sendBuf, buf);


	// ������ ������
	int retval = send(sock, buf, 4096, 0);
	if (retval == SOCKET_ERROR)
	{
		printf("�۽�() ����");
		return;
	}
}

char * Client::RecevData()
{
	char* name = recv_str;
	return name;
}
