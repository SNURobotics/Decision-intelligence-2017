// tcp_ip_server.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "stdafx.h"
#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include "Server.h"

#define BUFSIZE 512
#define BUFFER_SIZE 4096
//static int sendValue;
//static char sendBuf[BUFSIZE];

#include <stdlib.h>
#include <vector>
using namespace std;

struct dataset
{
	vector<double> robot_pos;
	vector<double> robot_rot;
	vector<double> robot_gripper;
	vector<double> robot_ft;

	vector<double> obj1_pos;
	vector<double> obj1_rot;
};

struct desired_dataset
{
	vector<double> robot_pos;
	vector<double> robot_rot;
	vector<double> robot_gripper;
	vector<double> robot_ft;
};

void Eliminate(char *str, char ch)
{
	for (; *str != '\0'; str++)//종료 문자를 만날 때까지 반복
	{
		if (*str == ch)//ch와 같은 문자일 때
		{
			strcpy(str, str + 1);
			str--;
		}
	}
}

int main(int argc, char* argv[])
{
	// 서버 초기화
	Server serv = Server::Server();

	dataset hyu_dataset;
	desired_dataset hyu_desired_dataset;

	char *hyu_data;
	char *pbuffer;

	char hyu_data_flag;
	char tmp_buffer[255];
	char div = 'd';

	int digit_num = 5;
	int nway = 0;

	while (TRUE) {
		//Receiving data from HYU client
		hyu_data = serv.RecevData();
		hyu_data_flag = hyu_data[0];

		// 데이터 전송
		if (hyu_data_flag == 'G') {
			string tmp_data;
			
			//Robot EE Position (3x1)
			hyu_dataset.robot_pos.resize(3);
			for (int i = 0; i < 3; i++) {
				hyu_dataset.robot_pos[i] = 0.512325;
				pbuffer = gcvt(hyu_dataset.robot_pos[i], digit_num, tmp_buffer);

				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}

			//Robot EE Rotation (3x3)
			hyu_dataset.robot_rot.resize(9);
			for (int i = 0; i < 9; i++){
				hyu_dataset.robot_rot[i] = 0.5123;
				pbuffer = gcvt(hyu_dataset.robot_rot[i], digit_num, tmp_buffer);

				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}
			
			//Robot Gripper (1x1)
			hyu_dataset.robot_gripper.resize(1);
			hyu_dataset.robot_gripper[0] = 1;
			pbuffer = gcvt(hyu_dataset.robot_gripper[0], digit_num, tmp_buffer);

			tmp_data = tmp_data + pbuffer;
			tmp_data = tmp_data + div;

			//Robot Gripper (6x1)
			hyu_dataset.robot_ft.resize(6);
			for (int i = 0; i < 6; i++) {
				hyu_dataset.robot_ft[i] = 0.31225;
				pbuffer = gcvt(hyu_dataset.robot_ft[i], digit_num, tmp_buffer);

				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}

			//Object1 Position (3x1)
			hyu_dataset.obj1_pos.resize(3);
			for (int i = 0; i < 3; i++) {
				hyu_dataset.obj1_pos[i] = 0.225;
				pbuffer = gcvt(hyu_dataset.obj1_pos[i], digit_num, tmp_buffer);

				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}

			//Object1 Rotation (3x3)
			hyu_dataset.obj1_rot.resize(9);
			for (int i = 0; i < 9; i++) {
				hyu_dataset.obj1_rot[i] = 0.3225;
				pbuffer = gcvt(hyu_dataset.obj1_rot[i], digit_num, tmp_buffer);

				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}

			char *send_data = new char[tmp_data.length() + 1];
			strcpy(send_data, tmp_data.c_str());
			serv.SendMessageToClient(send_data);

		}
		else if (hyu_data_flag == 'S') {
			Eliminate(hyu_data, 'S');
			
			nway = int(hyu_data[0]) - 48;
			hyu_data[0] = ' ';

			Eliminate(hyu_data, hyu_data[0]);

			char *recv_data = strtok(hyu_data, "d");
			int recv_cnt = 0;
			int nway_cnt = 0;

			hyu_desired_dataset.robot_pos.resize(3 * nway);
			hyu_desired_dataset.robot_rot.resize(9 * nway);
			hyu_desired_dataset.robot_gripper.resize(1 * nway);
			hyu_desired_dataset.robot_ft.resize(6 * nway);

			while (recv_data != NULL)
			{
				if (recv_cnt < 3){
					hyu_desired_dataset.robot_pos[recv_cnt + (nway_cnt*3)] = atof(recv_data);
				}
				else if (3<=recv_cnt && recv_cnt<12){
					hyu_desired_dataset.robot_rot[(recv_cnt-3) + (nway_cnt * 9)] = atof(recv_data);
				}
				else if (recv_cnt == 12){
					hyu_desired_dataset.robot_gripper[(recv_cnt-12) + (nway_cnt * 1)] = atof(recv_data);
				}
				else if (recv_cnt >= 13){
					hyu_desired_dataset.robot_ft[(recv_cnt-13) + (nway_cnt * 6)] = atof(recv_data);
				}

				recv_data = strtok(NULL, "d");
				recv_cnt += 1;

				if (recv_cnt == 19){
					recv_cnt = 0;
					nway_cnt += 1;
				}
			}
		}
		hyu_data[0] = '\0';
		hyu_data_flag = ' ';
		Sleep(100);
	}

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 
	
	// 서버를 종료 시킴
	serv.~Server();
	return 0;

}

