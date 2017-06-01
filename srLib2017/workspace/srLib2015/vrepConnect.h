#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "srDyn/srDYN.h"
extern "C" {
#include "extApi.h"
}
std::pair<Vec3, Vec3> SE3toVec3Pair(SE3 T)
{
	std::pair<Vec3, Vec3> temp;
	// first: pos, second: ori
	temp.first = T.GetPosition();
	temp.second[0] = atan2(-T.GetOrientation()[7], T.GetOrientation()[8]);
	temp.second[1] = asin(T.GetOrientation()[6]);
	temp.second[2] = atan2(-T.GetOrientation()[3], T.GetOrientation()[1]);
	return temp;
}

void Vec3ToSimxFloat(Vec3 vec, simxFloat* temp)
{
	for (int i = 0; i < 3; i++)
		temp[i] = vec[i];
}

int getObjectHandle(int clientID, vector<simxChar*> name, vector<simxInt>& objhandles)
{
	// use same code for joint handles
	int returnCode = 0;
	for (unsigned int i = 0; i < name.size(); i++)
		returnCode = simxGetObjectHandle(clientID, name[i], &objhandles[i], simx_opmode_blocking);
	return returnCode;
}

int setVrepBaseLinksFromSrlib(int clientID, vector<srSystem*> systems, vector<simxInt> objhandles)
{
	if (systems.size() != objhandles.size())
		printf("check size of setVrepFromSrlib inputs!!!\n");
	simxFloat temp[3];
	vector<std::pair<Vec3, Vec3>> Tobjects;
	Tobjects.resize(systems.size());
	int check = 0;
	for (unsigned int i = 0; i < systems.size(); i++)
	{
		Tobjects[i] = SE3toVec3Pair(systems[i]->GetBaseLink()->GetFrame());
		Vec3ToSimxFloat(Tobjects[i].first, temp);
		check = simxSetObjectPosition(clientID, objhandles[i], -1, temp, simx_opmode_oneshot);
		Vec3ToSimxFloat(Tobjects[i].second, temp);
		check = simxSetObjectOrientation(clientID, objhandles[i], -1, temp, simx_opmode_oneshot);
	}
	return check;
}

int setVrepJointValuesFromSrlib(int clientID, vector<srStateJoint*> joints, vector<simxInt> jointhandles)
{
	int check = 0;
	for (unsigned int i = 0; i < joints.size(); i++)
	{
		check = simxSetJointPosition(clientID, jointhandles[i], joints[i]->m_State.m_rValue[0], simx_opmode_oneshot);	//simx_opmode_oneshot, simx_opmode_streaming
	}
	
	return check;
}