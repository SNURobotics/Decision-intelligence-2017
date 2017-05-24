// tcp_ip_server.cpp : 콘솔 응용 프로그램에 대한 진입점을 정의합니다.
//

#include "../VS2013/tcp_ip_server/stdafx.h"
#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include "../VS2013/tcp_ip_server/Server.h"
#include "tcp_ip_communication.h"
#include "Eigen/Dense"
#define BUFSIZE 512
#define BUFFER_SIZE 4096
//static int sendValue;
//static char sendBuf[BUFSIZE];

#include <stdlib.h>
#include <vector>
using namespace std;


int main(int argc, char* argv[])
{
	// 서버 초기화
	Server serv = Server::Server();

	dataset hyu_dataset;
	desired_dataset hyu_desired_dataset;
	vision_data skku_dataset;

	char *hyu_data;
	char *copy;
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
		if (hyu_data_flag == 'V')
		{
			// vision data
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);

			readSKKUvision(hyu_data, skku_dataset);
		}
		else if (hyu_data_flag == 'G') {
			//string tmp_data;
			//
			////Robot EE Position (3x1)
			//hyu_dataset.robot_pos.resize(3);
			//for (int i = 0; i < 3; i++) {
			//	hyu_dataset.robot_pos[i] = 0.512325;
			//	pbuffer = _gcvt(hyu_dataset.robot_pos[i], digit_num, tmp_buffer);

			//	tmp_data = tmp_data + pbuffer;
			//	tmp_data = tmp_data + div;
			//}

			////Robot EE Rotation (3x3)
			//hyu_dataset.robot_rot.resize(9);
			//for (int i = 0; i < 9; i++){
			//	hyu_dataset.robot_rot[i] = 0.5123;
			//	pbuffer = _gcvt(hyu_dataset.robot_rot[i], digit_num, tmp_buffer);

			//	tmp_data = tmp_data + pbuffer;
			//	tmp_data = tmp_data + div;
			//}
			//
			////Robot Gripper (1x1)
			//hyu_dataset.robot_gripper.resize(1);
			//hyu_dataset.robot_gripper[0] = 1;
			//pbuffer = _gcvt(hyu_dataset.robot_gripper[0], digit_num, tmp_buffer);

			//tmp_data = tmp_data + pbuffer;
			//tmp_data = tmp_data + div;

			////Robot FTsensor (6x1)
			//hyu_dataset.robot_ft.resize(6);
			//for (int i = 0; i < 6; i++) {
			//	hyu_dataset.robot_ft[i] = 0.31225;
			//	pbuffer = _gcvt(hyu_dataset.robot_ft[i], digit_num, tmp_buffer);

			//	tmp_data = tmp_data + pbuffer;
			//	tmp_data = tmp_data + div;
			//}

			////Object1 Position (3x1)
			//hyu_dataset.obj1_pos.resize(3);
			//for (int i = 0; i < 3; i++) {
			//	hyu_dataset.obj1_pos[i] = 0.225;
			//	pbuffer = _gcvt(hyu_dataset.obj1_pos[i], digit_num, tmp_buffer);

			//	tmp_data = tmp_data + pbuffer;
			//	tmp_data = tmp_data + div;
			//}

			////Object1 Rotation (3x3)
			//hyu_dataset.obj1_rot.resize(9);
			//for (int i = 0; i < 9; i++) {
			//	hyu_dataset.obj1_rot[i] = 0.3225;
			//	pbuffer = _gcvt(hyu_dataset.obj1_rot[i], digit_num, tmp_buffer);

			//	tmp_data = tmp_data + pbuffer;
			//	tmp_data = tmp_data + div;
			//}

			//char *send_data = new char[tmp_data.length() + 1];
			//strcpy(send_data, tmp_data.c_str());
			//serv.SendMessageToClient(send_data);
			serv.SendMessageToClient(hyu_data);
		}
		else if (hyu_data_flag == 'R')
		{
			serv.SendMessageToClient(hyu_data);
		}
		else if (hyu_data_flag == 'S') {
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];

			double normFT = readRobotCommand(hyu_data, hyu_desired_dataset);
			if (normFT > 0.0)
			{
				// send to robot
				serv.SendMessageToClient(copy);
			}
			else
			{
				// do planning... assume output is vector<vector<Eigen::VectorXd>>
				vector<vector<Eigen::VectorXd>> jointTraj;
				jointTraj.resize(2);
				for (unsigned int i = 0; i < jointTraj.size(); i++)
				{
					jointTraj[i].resize(2 + i);
					for (unsigned int j = 0; j < jointTraj[i].size(); j++)
					{
						jointTraj[i][j] = Eigen::VectorXd::Ones(6);
						for (int k = 0; k < 6; k++)
						{
							jointTraj[i][j][k] = 100 * i + j + 0.01*k;
						}
					}
				}

				char* send_data = makeJointCommand(jointTraj, hyu_desired_dataset);
				serv.SendMessageToClient(send_data);
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

