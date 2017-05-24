#include "../../VS2013/tcp_ip_server/stdafx.h"
#include "../../VS2013/tcp_ip_server/Server.h"

#include "Eigen/Dense"
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

struct vision_data
{
	vector<vector<double>> objPos;		//id, (x, y, z)
	vector<vector<double>> objOri;		//id, (R_x, R_y, R_z)
	vector<vector<double>> obsInfo;		//id, (center, size)
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
void readSKKUvision(char* hyu_data, vision_data& skku_dataset);
double readRobotCommand(char* hyu_data, desired_dataset& hyu_desired_dataset);
char* makeJointCommand(vector<vector<Eigen::VectorXd>>& jointTraj, desired_dataset& hyu_desired_dataset);


void readSKKUvision(char* hyu_data, vision_data& skku_dataset)
{
	// read vision data
	Eliminate(hyu_data, 'V');

	char *recv_data = strtok(hyu_data, "d");
	int recv_cnt = 0;
	int nway_cnt = 0;

	skku_dataset.objPos.resize(100);
	skku_dataset.objOri.resize(100);
	skku_dataset.obsInfo.resize(0);
	int objID;
	int obsID = 0;
	bool obsData = false;
	while (recv_data != NULL)
	{
		if (!obsData && recv_cnt < 1)
		{
			objID = atoi(recv_data);
			if (objID == 100)
				obsData = true;
			else
			{
				skku_dataset.objPos[objID].resize(3);
				skku_dataset.objOri[objID].resize(9);
			}
		}

		if (!obsData)
		{
			if (1 <= recv_cnt && recv_cnt < 10)
				skku_dataset.objOri[objID][recv_cnt - 1] = atof(recv_data);
			else if (10 <= recv_cnt && recv_cnt<13)
				skku_dataset.objPos[objID][recv_cnt - 10] = atof(recv_data);
		}
		else
		{
			if (recv_cnt % 6 == 1)
			{
				vector<double> temp(0);
				skku_dataset.obsInfo.push_back(temp);
				skku_dataset.obsInfo[skku_dataset.obsInfo.size() - 1].push_back(atof(recv_data));
			}
			else if (recv_cnt > 0)
				skku_dataset.obsInfo[skku_dataset.obsInfo.size() - 1].push_back(atof(recv_data));
		}

		recv_data = strtok(NULL, "d");
		recv_cnt += 1;

		if (!obsData && recv_cnt == 13)
			recv_cnt = 0;
	}
};

double readRobotCommand(char* hyu_data, desired_dataset& hyu_desired_dataset)
{
	int nway = 0;
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
		if (recv_cnt < 3) {
			hyu_desired_dataset.robot_pos[recv_cnt + (nway_cnt * 3)] = atof(recv_data);
		}
		else if (3 <= recv_cnt && recv_cnt<12) {
			hyu_desired_dataset.robot_rot[(recv_cnt - 3) + (nway_cnt * 9)] = atof(recv_data);
		}
		else if (recv_cnt == 12) {
			hyu_desired_dataset.robot_gripper[(recv_cnt - 12) + (nway_cnt * 1)] = atof(recv_data);
		}
		else if (recv_cnt >= 13) {
			hyu_desired_dataset.robot_ft[(recv_cnt - 13) + (nway_cnt * 6)] = atof(recv_data);
		}

		recv_data = strtok(NULL, "d");
		recv_cnt += 1;

		if (recv_cnt == 19) {
			recv_cnt = 0;
			nway_cnt += 1;
		}
	}
	double normFT = 0.0;
	for (unsigned int i = 0; i < hyu_desired_dataset.robot_ft.size(); i++)
		normFT += hyu_desired_dataset.robot_ft[i] * hyu_desired_dataset.robot_ft[i];
	return normFT;
};

char* makeJointCommand(vector<vector<Eigen::VectorXd>>& jointTraj, desired_dataset& hyu_desired_dataset)
{
	char *pbuffer;

	char hyu_data_flag;
	char tmp_buffer[255];
	char div = 'd';

	int digit_num = 5;
	string tmp_data = "J";

	//Robot joint trajectory
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		for (unsigned int j = 0; j < jointTraj[i].size(); j++)
		{
			for (int k = 0; k < jointTraj[i][j].size(); k++)
			{
				pbuffer = _gcvt(jointTraj[i][j][k], digit_num, tmp_buffer);
				tmp_data = tmp_data + pbuffer;
				tmp_data = tmp_data + div;
			}
			pbuffer = _gcvt(hyu_desired_dataset.robot_gripper[i], digit_num, tmp_buffer);
			tmp_data = tmp_data + pbuffer;
			tmp_data = tmp_data + div;
		}
	}
	char *send_data = new char[tmp_data.length() + 1];
	strcpy(send_data, tmp_data.c_str());
	return send_data;
};