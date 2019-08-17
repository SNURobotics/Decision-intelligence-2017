#pragma once
#include "../../VS2013/tcp_ip_server/stdafx.h"
#include "../../VS2013/tcp_ip_server/Server.h"

#include "Eigen/Dense"

#include <stdlib.h>
#include <vector>
using namespace std;

struct vision_data
{
	vector<int>	objID;
	vector<vector<double>> objPos;					//id, (x, y, z)
	vector<vector<double>> objOri;					//id, (R_x, R_y, R_z)
	vector<int> objCand;							//id, (number of candidate)
	vector<vector<vector<double>>> objCandPos;		//id, cand_id, (x, y, z)
	vector<vector<double>> obsInfo;					//id, (center, size)
};



static void Eliminate(char *str, char ch)
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

static void readSKKUvision(char* hyu_data, vision_data& skku_dataset)
{
	// read vision data
	Eliminate(hyu_data, 'V');

	char *recv_data = strtok(hyu_data, "d");
	int recv_cnt = 0;
	int nway_cnt = 0;
	skku_dataset.objID.resize(0);
	skku_dataset.objPos.resize(0);
	skku_dataset.objOri.resize(0);
	skku_dataset.objCand.resize(0);
	skku_dataset.obsInfo.resize(0);
	vector<double> zeropos(3, 0.0);
	vector<double> zeroori(9, 0.0);
	int objIdx = 0;
	int objID;
	int obsID = 0;
	int numCand = 0;
	int max_recv_cnt = 0;
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
			if (1 <= recv_cnt && recv_cnt < 4)
				skku_dataset.objPos[objIdx][recv_cnt] = atof(recv_data);
			else if (4 <= recv_cnt && recv_cnt < 13)
			{
				int temp = recv_cnt - 4;
				int idx = (temp % 3) * 3 + temp / 3;  //need fix
				skku_dataset.objOri[objIdx][idx] = atof(recv_data);
			}
			else if (recv_cnt == 13)
			{
				numCand = atoi(recv_data);
				skku_dataset.objCand.push_back(numCand);
				max_recv_cnt = 14 + 3 * numCand;
				skku_dataset.objCandPos[objIdx].resize(numCand);
			}
			else if (14 <= recv_cnt && recv_cnt < 17 && numCand > 0)
			{
				skku_dataset.objCandPos[objIdx].push_back(zeropos);
				skku_dataset.objCandPos[objIdx][0][recv_cnt - 14] = atof(recv_data);
			}
			else if (17 <= recv_cnt && recv_cnt < 20 && numCand > 1)
			{
				skku_dataset.objCandPos[objIdx].push_back(zeropos);
				skku_dataset.objCandPos[objIdx][1][recv_cnt - 17] = atof(recv_data);
			}
			else if (20 <= recv_cnt && recv_cnt < 23 && numCand > 2)
			{
				skku_dataset.objCandPos[objIdx].push_back(zeropos);
				skku_dataset.objCandPos[objIdx][2][recv_cnt - 20] = atof(recv_data);
			}
				
		}
		else if (recv_cnt > 0)
		{
			skku_dataset.obsInfo[skku_dataset.obsInfo.size() - 1].push_back(atof(recv_data));
		}

		recv_data = strtok(NULL, "d");
		recv_cnt += 1;

		if (!obsData && recv_cnt == max_recv_cnt)
		{
			recv_cnt = 0;
			objIdx++;
		}
		if (obsData && recv_cnt == 7)
			recv_cnt = 0;
	}
}
