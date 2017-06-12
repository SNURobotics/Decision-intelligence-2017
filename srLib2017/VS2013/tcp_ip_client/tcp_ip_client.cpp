// tcp_ip_client.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include "Client.h"
#include <string>

using namespace std;

int main()
{
	// 클라이언트 초기화
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
		// 서버에게 메세지를 보내는 함수
		printf("send << bb \n> ");
	}

	// 클라이언트를 종료 시킴
	client.~Client();
	return 0;
}

