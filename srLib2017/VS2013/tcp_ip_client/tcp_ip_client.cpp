// tcp_ip_client.cpp : �ܼ� ���� ���α׷��� ���� �������� �����մϴ�.
//

#include "stdafx.h"
#include "Client.h"
#include <string>

using namespace std;

int main()
{
	// Ŭ���̾�Ʈ �ʱ�ȭ
	Client client = Client::Client();


	string tmp_data1 = "S" + to_string(1) + "d" + to_string(2);
	for (int i = 0; i < 2 * 19; i++)
		tmp_data1 = tmp_data1 + to_string(0) + "d";

	string tmp_data2 = "P" + to_string(1) + "d";
	for (int i = 0; i < 19; i++)
	{
		tmp_data2 = tmp_data2 + to_string(0) + "d";
	}

	char* send_data1 = (char*)malloc(sizeof(char)*strlen(tmp_data1.c_str()) + 1);
	char* send_data2 = (char*)malloc(sizeof(char)*strlen(tmp_data2.c_str()) + 1);

	send_data1 = strcpy(send_data1, tmp_data1.c_str());
	send_data2 = strcpy(send_data2, tmp_data2.c_str());


	while (TRUE) {
		Sleep(3000);
		client.SendMessageToServer(send_data1);
		Sleep(3000);
		client.SendMessageToServer(send_data2);
		// �������� �޼����� ������ �Լ�
		printf("send << bb \n> ");
	}

	// Ŭ���̾�Ʈ�� ���� ��Ŵ
	client.~Client();
	return 0;
}

