// Copyright 2006-2016 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.3.2 on August 29th 2016

// Make sure to have the server side running in V-REP: 
// in a child script of a V-REP scene, add following command
// to be executed just once, at simulation start:
//
// simExtRemoteApiStart(19999)
//
// then start simulation, and run this program.
//
// IMPORTANT: for each successful call to simxStart, there
// should be a corresponding call to simxFinish at the end!

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include "myRenderer.h"
#include "srDyn/srDYN.h"
extern "C" {
    #include "extApi.h"
}

void rendering(int argc, char **argv);
void updateFunc();
void initDynamics();
srSystem* createBox(SE3 initPos, Vec3 dim, Vec3 color = Vec3(0.3, 0.3, 0.3), srSystem::BASELINKTYPE type = srSystem::BASELINKTYPE::DYNAMIC);

std::pair<Vec3, Vec3> SE3toVec3Pair(SE3 T);
void Vec3ToSimxFloat(Vec3 vec, simxFloat* temp);
void setVrepFromSrlib();

srSpace gSpace;
myRenderer* renderer;

srSystem* box1;
srSystem* box2;
srSystem* box3;
srSystem* box0;

// variables for v-rep
int clientID;
std::pair<Vec3, Vec3> T1;
std::pair<Vec3, Vec3> T2;
std::pair<Vec3, Vec3> T3;
simxInt obj1handle;
simxInt obj2handle;
simxInt obj3handle;
simxFloat temp[3];

int main(int argc,char* argv[])
{

	box1 = createBox(EulerZYX(Vec3(0.3, 0.1, 0.2), Vec3(0.0, 0.0, 2.5)), Vec3(0.1, 0.1, 0.1));
	box2 = createBox(EulerZYX(Vec3(0.0, -0.1, 0.2), Vec3(0.05, 0.05, 1.7)), Vec3(0.1, 0.1, 0.1));
	box3 = createBox(EulerZYX(Vec3(0.1, 0.1, -0.2), Vec3(0.05, -0.05, 2.1)), Vec3(0.1, 0.1, 0.1));
	box0 = createBox(SE3(Vec3(0.0, 0.0, -0.005)), Vec3(100.0, 100.0, 0.01), Vec3(0.1, 0.1, 0.1), srSystem::BASELINKTYPE::FIXED);
   

	gSpace.AddSystem(box1);
	gSpace.AddSystem(box2);
	gSpace.AddSystem(box3);
	gSpace.AddSystem(box0);

	initDynamics();

	clientID = simxStart((simxChar*)"127.0.0.1", 19999, true, true, 2000, 5);
	simxChar* name1 = "Cuboid1";
	simxChar* name2 = "Cuboid2";
	simxChar* name3 = "Cuboid3";
	
	
	int returnCode = simxGetObjectHandle(clientID, name1, &obj1handle, simx_opmode_blocking);
	returnCode = simxGetObjectHandle(clientID, name2, &obj2handle, simx_opmode_blocking);
	returnCode = simxGetObjectHandle(clientID, name3, &obj3handle, simx_opmode_blocking);
	
	setVrepFromSrlib();


	if (clientID != -1)
	{


		printf("Connected to remote API server\n");
		setVrepFromSrlib();



		// Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
		int pingTime;
		simxGetPingTime(clientID, &pingTime);

		//// Now close the connection to V-REP:   
		//simxFinish(clientID);
	}

	rendering(argc, argv);

	
    return(0);
}



void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);
	renderer->setUpdateFunc(updateFunc);

	renderer->RunRendering();
}


void initDynamics()
{
	gSpace.SetTimestep(0.001);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(10);
	gSpace.DYN_MODE_PRESTEP();
}

srSystem * createBox(SE3 initPos, Vec3 dim, Vec3 color, srSystem::BASELINKTYPE type)
{
	srSystem* temp = new srSystem;
	srLink* box = new srLink;
	box->m_Restitution = 0.8;
	box->GetGeomInfo().SetDimension(dim);
	box->GetGeomInfo().SetColor(color[0], color[1], color[2]);
	temp->SetBaseLink(box);
	temp->SetBaseLinkType(type);
	srCollision* colli = new srCollision;
	colli->GetGeomInfo().SetDimension(dim);
	box->AddCollision(colli);
	box->SetFrame(initPos);
	return temp;
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

void setVrepFromSrlib()
{
	T1 = SE3toVec3Pair(box1->GetBaseLink()->GetFrame());
	T2 = SE3toVec3Pair(box2->GetBaseLink()->GetFrame());
	T3 = SE3toVec3Pair(box3->GetBaseLink()->GetFrame());

	//simxFloat temp[3];
	//temp = Vec3ToSimxFloat(T2.first);

	//cout << *temp;
	//cout << *(temp + sizeof(simxFloat));
	//cout << *(temp + sizeof(simxFloat) + sizeof(simxFloat));
	Vec3ToSimxFloat(T1.first, temp);
	int check = simxSetObjectPosition(clientID, obj1handle, -1, temp, simx_opmode_blocking);
	Vec3ToSimxFloat(T2.first, temp);
	check = simxSetObjectPosition(clientID, obj2handle, -1, temp, simx_opmode_blocking);
	Vec3ToSimxFloat(T3.first, temp);
	check = simxSetObjectPosition(clientID, obj3handle, -1, temp, simx_opmode_blocking);
	Vec3ToSimxFloat(T1.second, temp);
	check = simxSetObjectOrientation(clientID, obj1handle, -1, temp, simx_opmode_blocking);
	Vec3ToSimxFloat(T2.second, temp);
	check = simxSetObjectOrientation(clientID, obj2handle, -1, temp, simx_opmode_blocking);
	Vec3ToSimxFloat(T3.second, temp);
	check = simxSetObjectOrientation(clientID, obj3handle, -1, temp, simx_opmode_blocking);
	//simxFloat temp2[3];
	//temp2[0] = 1.0;
	//temp2[1] = 1.0;
	//temp2[2] = 1.0;
	
	//int check = simxSetObjectPosition(clientID, obj1handle, -1, temp2, simx_opmode_blocking);
	//check = simxSetObjectPosition(clientID, obj2handle, -1, temp2, simx_opmode_blocking);
	//check = simxSetObjectPosition(clientID, obj3handle, -1, temp2, simx_opmode_blocking);
	//check = simxSetObjectOrientation(clientID, obj1handle, -1, temp2, simx_opmode_blocking);
	//check = simxSetObjectOrientation(clientID, obj2handle, -1, temp2, simx_opmode_blocking);
	//check = simxSetObjectOrientation(clientID, obj3handle, -1, temp2, simx_opmode_blocking);
}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	if (clientID != -1)
	{


		printf("Connected to remote API server\n");
		setVrepFromSrlib();



		// Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
		//int pingTime;
		//simxGetPingTime(clientID, &pingTime);

		//// Now close the connection to V-REP:   
		//simxFinish(clientID);
	}

}