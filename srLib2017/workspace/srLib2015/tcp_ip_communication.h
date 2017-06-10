#include "../../VS2013/tcp_ip_server/stdafx.h"
#include "../../VS2013/tcp_ip_server/Server.h"

#include "Eigen/Dense"

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
	vector<int>	objID;
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
	int command_flag;
};

struct robot_current_data
{
	Eigen::VectorXd robot_joint;
	int robot_isOnWaypoint;
	int robot_isDead;
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
int readRobotCurState(char *hyu_data, robot_current_data& robot_state);
pair<int,vector<int>> readRobotCommand(char* hyu_data, vector<desired_dataset>& hyu_desired_dataset);
char* makeJointCommand(vector<vector<Eigen::VectorXd>>& jointTraj, desired_dataset& hyu_desired_dataset);


void readSKKUvision(char* hyu_data, vision_data& skku_dataset)
{
	// read vision data
	Eliminate(hyu_data, 'V');

	char *recv_data = strtok(hyu_data, "d");
	int recv_cnt = 0;
	int nway_cnt = 0;
	skku_dataset.objID.resize(0);
	skku_dataset.objPos.resize(0);
	skku_dataset.objOri.resize(0);
	skku_dataset.obsInfo.resize(0);
	vector<double> zeropos(3, 0.0);
	vector<double> zeroori(9, 0.0);
	int objIdx = 0;
	int objID;
	int obsID = 0;
	bool obsData = false;
	while (recv_data != NULL)
	{
		if (!obsData && recv_cnt < 1)
		{
			objID = atoi(recv_data);
			if (objID == -1)
				obsData = true;
			else
			{
				skku_dataset.objID.push_back(objID);
				skku_dataset.objPos.push_back(zeropos);
				skku_dataset.objOri.push_back(zeroori);
			}
		}
		if (obsData && recv_cnt < 1)
		{
			objID = atoi(recv_data);
			if (objID == 0)
				break;
			else
			{
				vector<double> temp(0);
				skku_dataset.obsInfo.push_back(temp);
			}
		}

		if (!obsData)
		{
			if (1 <= recv_cnt && recv_cnt < 10)
			{
				int temp = recv_cnt - 1;
				int idx = (temp % 3) * 3 + temp / 3;
				skku_dataset.objOri[objIdx][idx] = atof(recv_data);
			}
			else if (10 <= recv_cnt && recv_cnt<13)
				skku_dataset.objPos[objIdx][recv_cnt - 10] = atof(recv_data);
		}
		else if (recv_cnt > 0)
		{
			skku_dataset.obsInfo[skku_dataset.obsInfo.size() - 1].push_back(atof(recv_data));
		}

		recv_data = strtok(NULL, "d");
		recv_cnt += 1;

		if (!obsData && recv_cnt == 13)
		{
			recv_cnt = 0;
			objIdx++;
		}
		if (obsData && recv_cnt == 7)
			recv_cnt = 0;
	}
}
int readRobotCurState(char * hyu_data, robot_current_data & robot_state)
{
	Eliminate(hyu_data, hyu_data[0]);
	char *recv_data = strtok(hyu_data, "d");
	char tempFlag[2];
	strcpy(tempFlag, "");
	sprintf(tempFlag, "%c", recv_data[0]);
	int robotFlag = 0;
	robotFlag = atoi(tempFlag);
	recv_data = strtok(NULL, "d");


	int recv_cnt = 0;
	int nrobot_cnt = 0;


	if (robotFlag == 1 || robotFlag == 2)
	{
		robot_state.robot_joint.resize(6);
		robot_state.robot_pos.resize(3);
		robot_state.robot_rot.resize(9);
		robot_state.robot_ft.resize(6);
		robot_state.robot_gripper.resize(1);
		while (recv_data != NULL)
		{
			if (recv_cnt < 3)
				robot_state.robot_pos[recv_cnt] = atof(recv_data);
			else if (recv_cnt < 3 + 9)
				robot_state.robot_rot[recv_cnt - 3] = atof(recv_data);
			else if (recv_cnt < 3 + 9 + 1)
				robot_state.robot_gripper[recv_cnt - 3 - 9] = atof(recv_data);
			else if (recv_cnt < 3 + 9 + 1 + 6)
				robot_state.robot_ft[recv_cnt - 3 - 9 - 1] = atof(recv_data);
			else if (recv_cnt < 3 + 9 + 1 + 6 + 1)
				robot_state.robot_isOnWaypoint = atoi(recv_data);
			else if (recv_cnt < 3 + 9 + 1 + 6 + 1 + 1)
				robot_state.robot_isDead = atoi(recv_data);
			else
				robot_state.robot_joint[recv_cnt - 3 - 9 - 1 - 6 - 1 - 1] = atof(recv_data);

			recv_data = strtok(NULL, "d");
			recv_cnt += 1;
		}
	}
	/*else if (robotFlag == 3)
	{
		robot_state.robot_joint.resize(6 * 2);
		robot_state.robot_pos.resize(3 * 2);
		robot_state.robot_rot.resize(9 * 2);
		robot_state.robot_ft.resize(6 * 2);
		robot_state.robot_gripper.resize(1 * 2);
		while (recv_data != NULL)
		{
			if (recv_cnt < 3)
				robot_state.robot_pos[recv_cnt + (nrobot_cnt * 3)] = atof(recv_data);
			else if (recv_cnt < 3 + 9)
				robot_state.robot_rot[recv_cnt - 3 + (nrobot_cnt * 9)] = atof(recv_data);
			else if (recv_cnt < 3 + 9 + 1)
				robot_state.robot_gripper[recv_cnt - 3 - 9 + (nrobot_cnt * 1)] = atof(recv_data);
			else if (recv_cnt < 3 + 9 + 1 + 6)
				robot_state.robot_ft[recv_cnt - 3 - 9 - 1 + (nrobot_cnt * 6)] = atof(recv_data);
			else
				robot_state.robot_joint[recv_cnt - 3 - 9 - 1 - 6 + (nrobot_cnt * 6)] = atof(recv_data);

			recv_data = strtok(NULL, "d");
			recv_cnt += 1;

			if (recv_cnt == 19) {
				recv_cnt = 0;
				nrobot_cnt += 1;
			}
		}
	}*/
	else
		printf("Wrong robotFlag is given!!!!!!! (readRobotCurState())");
	

	return robotFlag;
}




pair<int,vector<int>> readRobotCommand(char* hyu_data, vector<desired_dataset>& hyu_desired_dataset)
{
	int nway1 = 0;
	int nway2 = 0;
	vector<int> nway_vec(2);
	Eliminate(hyu_data, 'S');
	//printf(hyu_data);
	char *recv_data = strtok(hyu_data, "d");
	//nway = int(hyu_data[0]) - 48;
	//hyu_data[0] = ' ';

	//Eliminate(hyu_data, hyu_data[0]);

	int robotFlag = 0;

	robotFlag = atoi(recv_data);
	recv_data = strtok(NULL, "d");

	int recv_cnt = 0;
	int nway_cnt = 0;
	//vector<double> normFT;
	vector<int> commandFlag;

	hyu_desired_dataset.resize(2);

	if (robotFlag == 1 || robotFlag == 2)
	{
		nway1 = atoi(recv_data);
		recv_data = strtok(NULL, "d");
		hyu_desired_dataset[robotFlag-1].robot_pos.resize(3 * nway1);
		hyu_desired_dataset[robotFlag-1].robot_rot.resize(9 * nway1);
		hyu_desired_dataset[robotFlag-1].robot_gripper.resize(1 * nway1);
		hyu_desired_dataset[robotFlag-1].robot_ft.resize(6 * nway1);

		while (recv_data != NULL)
		{
			if (recv_cnt < 3) {
				hyu_desired_dataset[robotFlag - 1].robot_pos[recv_cnt + (nway_cnt * 3)] = atof(recv_data);
			}
			else if (3 <= recv_cnt && recv_cnt < 12) {
				hyu_desired_dataset[robotFlag - 1].robot_rot[(recv_cnt - 3) + (nway_cnt * 9)] = atof(recv_data);
			}
			else if (recv_cnt == 12) {
				hyu_desired_dataset[robotFlag - 1].robot_gripper[(recv_cnt - 12) + (nway_cnt * 1)] = atof(recv_data);
			}
			else if (recv_cnt >= 13) {
				hyu_desired_dataset[robotFlag - 1].robot_ft[(recv_cnt - 13) + (nway_cnt * 6)] = atof(recv_data);
			}

			recv_data = strtok(NULL, "d");
			recv_cnt += 1;

			if (recv_cnt == 19) {
				recv_cnt = 0;
				nway_cnt += 1;
				if (nway_cnt == nway1)
				{
					hyu_desired_dataset[robotFlag - 1].command_flag = atoi(recv_data);
				}
			}
		}
		commandFlag.resize(1);
		commandFlag[0] = hyu_desired_dataset[robotFlag - 1].command_flag;
		//normFT.resize(1);
		//normFT[0] = 0.0;
		//for (unsigned int i = 0; i < hyu_desired_dataset[robotFlag - 1].robot_ft.size(); i++)
		//	normFT[0] += hyu_desired_dataset[robotFlag - 1].robot_ft[i] * hyu_desired_dataset[robotFlag - 1].robot_ft[i];
	}
	else if (robotFlag == 3)
	{
		nway1 = atoi(recv_data);
		recv_data = strtok(NULL, "d");
		nway2 = atoi(recv_data);
		nway_vec[0] = nway1;
		nway_vec[1] = nway2;

		recv_data = strtok(NULL, "d");
		hyu_desired_dataset[0].robot_pos.resize(3 * nway1);
		hyu_desired_dataset[0].robot_rot.resize(9 * nway1);
		hyu_desired_dataset[0].robot_gripper.resize(1 * nway1);
		hyu_desired_dataset[0].robot_ft.resize(6 * nway1);
		hyu_desired_dataset[1].robot_pos.resize(3 * nway2);
		hyu_desired_dataset[1].robot_rot.resize(9 * nway2);
		hyu_desired_dataset[1].robot_gripper.resize(1 * nway2);
		hyu_desired_dataset[1].robot_ft.resize(6 * nway2);
		int robot_cnt = 0;
		while (recv_data != NULL)
		{
			if (recv_cnt < 3) {
				hyu_desired_dataset[robot_cnt].robot_pos[recv_cnt + (nway_cnt * 3)] = atof(recv_data);
			}
			else if (3 <= recv_cnt && recv_cnt < 12) {
				hyu_desired_dataset[robot_cnt].robot_rot[(recv_cnt - 3) + (nway_cnt * 9)] = atof(recv_data);
			}
			else if (recv_cnt == 12) {
				hyu_desired_dataset[robot_cnt].robot_gripper[(recv_cnt - 12) + (nway_cnt * 1)] = atof(recv_data);
			}
			else if (recv_cnt >= 13) {
				hyu_desired_dataset[robot_cnt].robot_ft[(recv_cnt - 13) + (nway_cnt * 6)] = atof(recv_data);
			}

			recv_data = strtok(NULL, "d");
			recv_cnt += 1;

			if (recv_cnt == 19) 
			{
				recv_cnt = 0;
				nway_cnt += 1;
				if (nway_cnt == nway_vec[robot_cnt])
				{
					hyu_desired_dataset[robot_cnt].command_flag = atoi(recv_data);
					recv_data = strtok(NULL, "d");
					nway_cnt = 0;
					robot_cnt += 1;
				}
					
			}
		}
		commandFlag.resize(2);
		for (unsigned int i = 0; i < hyu_desired_dataset.size(); i++)
		{
			commandFlag[i] = hyu_desired_dataset[i].command_flag;
		}
		
		//normFT.resize(2);
		//normFT[0] = 0.0;
		//normFT[1] = 0.0;
		//for (unsigned int i = 0; i < hyu_desired_dataset.size(); i++)
		//{
		//	for (unsigned int j = 0; j < hyu_desired_dataset[i].robot_ft.size(); j++)
		//	{
		//		 normFT[i] += hyu_desired_dataset[i].robot_ft[j] * hyu_desired_dataset[i].robot_ft[j];
		//	}
		//}




	}
	return std::make_pair(robotFlag, commandFlag);
};

char* makeJointCommand(vector<vector<Eigen::VectorXd>>& jointTraj, desired_dataset& hyu_desired_dataset)
{
	char *pbuffer;

	char tmp_buffer[255];
	char div = 'd';

	int digit_num = 5;
	unsigned int totalNum = 0;
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		totalNum += jointTraj[i].size();
	}
	string tmp_data = "J" + to_string(totalNum) + "d";

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


char* makeJointCommand_SingleRobot(vector<vector<Eigen::VectorXd>>& jointTraj, desired_dataset& hyu_desired_dataset, int robotFlag)
{
	char pbuffer[100];

	//char hyu_data_flag;
	char tmp_buffer[255];
	char div = 'd';

	int digit_num = 5;
	unsigned int totalNum = 0;
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		totalNum += jointTraj[i].size();
	}


	//char tmp_data[30000] = "J";
	char* tmp_data = (char*)malloc(sizeof(char) * 30000);
	memset(tmp_data, NULL, sizeof(char) * 30000);
	strcat(tmp_data, "J");
	char plus[3];
	sprintf(plus, "%dd", robotFlag);
	strcat(tmp_data, plus);
	sprintf(plus, "%dd", totalNum);
	strcat(tmp_data, plus);
	//string tmp_data = "J" + to_string(robotFlag)+"d"+to_string(totalNum) + "d";

	//Robot joint trajectory
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		for (unsigned int j = 0; j < jointTraj[i].size(); j++)
		{
			for (int k = 0; k < jointTraj[i][j].size(); k++)
			{
				strcpy(pbuffer, "");
				strcat(pbuffer,_gcvt(jointTraj[i][j][k], digit_num, tmp_buffer));
				//string add = string(pbuffer);
				//tmp_data = tmp_data + add;
				strcat(tmp_data, pbuffer);
				sprintf(plus, "%c", div);
				strcat(tmp_data, plus);

				//tmp_data = tmp_data + div;
			}
			strcpy(pbuffer, "");
			strcat(pbuffer, _gcvt(hyu_desired_dataset.robot_gripper[i], digit_num, tmp_buffer));
			//pbuffer = _gcvt(hyu_desired_dataset.robot_gripper[i], digit_num, tmp_buffer);
			strcat(tmp_data, pbuffer);
			sprintf(plus, "%c", div);
			strcat(tmp_data, plus);
			//tmp_data = tmp_data + string(pbuffer);
			//tmp_data = tmp_data + div;
		}
	}
	char *send_data = new char[strlen(tmp_data) + 1];
	strcpy(send_data, tmp_data);
	free(tmp_data);
	return send_data;
};

char* makeForceResult(vector<vector<Eigen::VectorXd>>& forceTraj, int robotFlag)
{
	char pbuffer[100];

	//char hyu_data_flag;
	char tmp_buffer[255];
	char div = 'd';

	int digit_num = 5;
	unsigned int totalNum = 0;
	for (unsigned int i = 0; i < forceTraj.size(); i++)
	{
		totalNum += forceTraj[i].size();
	}


	//char tmp_data[30000] = "M";
	char* tmp_data = (char*)malloc(sizeof(char) * 30000);
	memset(tmp_data, NULL, sizeof(char) * 30000);
	strcat(tmp_data, "M");
	char plus[256];
	sprintf(plus, "%dd", robotFlag);
	strcat(tmp_data, plus);
	sprintf(plus, "%dd", totalNum);
	strcat(tmp_data, plus);
	//string tmp_data = "J" + to_string(robotFlag)+"d"+to_string(totalNum) + "d";

	//Robot joint trajectory
	for (unsigned int i = 0; i < forceTraj.size(); i++)
	{
		for (unsigned int j = 0; j < forceTraj[i].size(); j++)
		{
			for (int k = 0; k < forceTraj[i][j].size(); k++)
			{
				strcpy(pbuffer, "");
				strcat(pbuffer, _gcvt(forceTraj[i][j][k], digit_num, tmp_buffer));
				//string add = string(pbuffer);
				//tmp_data = tmp_data + add;
				strcat(tmp_data, pbuffer);
				sprintf(plus, "%c", div);
				strcat(tmp_data, plus);
				//tmp_data = tmp_data + div;
			}
		}
	}
	char *send_data = new char[strlen(tmp_data) + 1];
	strcpy(send_data, tmp_data);
	return send_data;
};

//
//vector<char*> makeJointCommand_MultiRobot(vector<vector<vector<Eigen::VectorXd>>>& jointTraj, vector<desired_dataset>& hyu_desired_dataset, int robotFlag)
//{
//	char *pbuffer;
//
//	char hyu_data_flag;
//	char tmp_buffer[255];
//	char div = 'd';
//	int digit_num = 5;
//	vector<unsigned int> totalNum(2);
//	totalNum[0] = 0;
//	totalNum[1] = 0;
//	for (int robotnum = 0; robotnum < 2; robotnum++)
//	{
//		for (unsigned int i = 0; i < jointTraj[robotnum].size(); i++)
//		{
//			totalNum[robotnum] += jointTraj[robotnum][i].size();
//		}
//	}
//
//	vector<char*> send_data(2);
//
//	string tmp_data;
//	
//	//tmp_data= "J" + to_string(robotFlag) + "d" + to_string(totalNum) + "d";
//
//	//Robot joint trajectory
//	for (int robotnum = 0; robotnum < 2; robotnum++)
//	{
//		tmp_data = "J" + to_string(robotnum+1) + "d" + to_string(totalNum[robotnum]) + "d";
//		for (unsigned int i = 0; i < jointTraj[robotnum].size(); i++)
//		{
//			for (unsigned int j = 0; j < jointTraj[robotnum][i].size(); j++)
//			{
//				for (int k = 0; k < jointTraj[robotnum][i][j].size(); k++)
//				{
//					pbuffer = _gcvt(jointTraj[robotnum][i][j][k], digit_num, tmp_buffer);
//					tmp_data = tmp_data + pbuffer;
//					tmp_data = tmp_data + div;
//				}
//				pbuffer = _gcvt(hyu_desired_dataset[robotnum].robot_gripper[i], digit_num, tmp_buffer);
//				tmp_data = tmp_data + pbuffer;
//				tmp_data = tmp_data + div;
//			}
//		}
//		send_data[i] = new char[tmp_data.length() + 1];
//		strcpy(send_data[robotnum], tmp_data.c_str());
//	}
//	return send_data;
//	
//	//char *send_data = new char[tmp_data.length() + 1];
//	strcpy(send_data, tmp_data.c_str());
//	return send_data;
//};