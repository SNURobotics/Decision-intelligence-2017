#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "common\dataIO.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager/UR5Robot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager.h"
#include <fstream>
#include <iostream>

using namespace std;

// Environment
Base_HYU* busbarBase = new Base_HYU;
JigAssem_QB_bar* jigAssem = new JigAssem_QB_bar(false);
vector<Jig_HYU*> jig(4);
vector<SE3> jigSE3(4);
vector<SE3> jig2busbar(2);
//vector<BusBar_HYU*> busbar(8);
//vector<SE3>	initSE3(8);
//vector<SE3>	goalSE3(8);
//vector<SE3> allSE3_busbar(2 * busbar.size());
int busbarNum = 4;
vector<BusBar_HYU*> busbar(busbarNum);
vector<Insert*> ctCase(1);
vector<SE3>	initSE3(2);
vector<SE3>	goalSE3(2);
vector<SE3> allSE3_busbar(2 * initSE3.size());

// Workspace
int workcell_mode = 2;
WorkCell* workCell = new WorkCell(workcell_mode);
Eigen::VectorXd stageVal(3);

// Robot
UR5Robot* robot1 = new UR5Robot(false);
UR5Robot* robot2 = new UR5Robot(false);
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);
srSpace gSpace;
myRenderer* renderer;
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
//SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.0, 0.04));    // for UR5
SE3 TctCase2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.01));
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));

// Planning
vector<vector<Eigen::VectorXd>> traj(0);
vector<vector<SE3>>	Ttraj(0);


vector<bool> attachObject;

vector<vector<int>> idxTraj(0);
vector<vector<int>> totalFlag(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
vector<SE3> wayPoints(0);

Eigen::VectorXd homePos = Eigen::VectorXd::Zero(6);

SE3 initBusbar;

// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> TtrajFromJoint(2);

vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);

UR5RobotManager* rManager1;
UR5RobotManager* rManager2;
robotRRTManager* RRTManager1 = new robotRRTManager;
robotRRTManager* RRTManager2 = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
hybridPFCtrlManager_6dof* hctrl = new hybridPFCtrlManager_6dof;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
//void updateFuncInput();
//void updateFuncPlanning();
//void updateFuncTestSensor();
//void updateFuncTestSensorToRobot();
//void updateFuncLoadJointValAttachStatus();
//void updateFuncLoadSuccessData();
void loadSuccessData(int nWay, bool connect);
void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation);
void environmentSetting_HYU2(bool connect);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting_HYU();
void RRT_problemSetting();
void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<int> busbarIdx);
//void RRT_problemSetting_robot2(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject);
void RRTSolve();
void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize, vector<int> busbarIdx);
//void RRTSolve_HYU_robot2(vector<bool> attachObject, vector<double> stepsize);
void rrtSetting();

Vec2 goalLocation; // busbar insertion location

vector<int> flags(0);

bool loadSuccess = false;
double planning = 0;
vector<Eigen::VectorXd> loadJointVal(0);
vector<Eigen::VectorXd> loadAttachStatus(0);
vector<Eigen::VectorXd>	loadBusbar(0);
SE3 Twaypoint;
SE3 Trobotbase1;
SE3 Trobotbase2;
Eigen::VectorXd testWaypoint;
vector<Eigen::VectorXd> testJointVal(0);
vector<Eigen::VectorXd> testJointVal2(0);
vector<vector<Eigen::VectorXd>> testJointValVec(2);

srWeldJoint* wJointJig = new srWeldJoint;
Eigen::VectorXd testjointvalue(6);
vector<SE3> initBusbars(busbarNum);
vector<Eigen::VectorXd> jointTrajAll;
vector<vector<SE3>>	busbarTrajAll(busbarNum);
int main(int argc, char **argv)
{
	srand(time(NULL));
	// Robot home position
	//homePos[1] = -SR_PI_HALF; homePos[3] = SR_PI_HALF; homePos[4] = -0.5 * SR_PI;
	homePos = robot1->homePos;
	//homePos[4] = -0.5 * SR_PI;		// match to robot workcell
	// environment
	//workspaceSetting();
	int startLocation =0;
	goalLocation = Vec2(1, 1);
	bool connectJig2workcell = false;
	environmentSetting_HYU2(connectJig2workcell);




	//jigAssem->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.5)));
	////////////////////////////////////////////// for testing workspace
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.099 + 0.009));	// when only stage4 is used
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	vector<SE3> busbarSE3set(0);


	robotSetting();
	Trobotbase1 = robot1->GetBaseLink()->GetFrame();
	Trobotbase2 = robot2->GetBaseLink()->GetFrame();

	//if (loadSuccess)
	//{
	//	int nWaypoint = 4;
	//	loadSuccessData(nWaypoint, connectJig2workcell);			// should be called after robotSetting
	//	planning = 0.0;
	//}


	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();
	rManager1->setJointVal(robot1->homePos);
	//rManager2->setJointVal(robot2->homePos);

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.005;
	gripInput[1] = 0.005;
	rManager1->setGripperPosition(gripInput);
	//rManager2->setGripperPosition(gripInput);

	// rrt
	rrtSetting();

	
	/////////////////////////////////////////////////////////////////////// 180724 success data
	int nWay = busbarNum * 4;
	vector<bool> includeOri(nWay, true);
	wayPoints.resize(nWay);
	vector<int> holeNum(busbarNum);
	holeNum[0] = 5;
	holeNum[1] = 1;
	holeNum[2] = 7;
	holeNum[3] = 3;
	//double tran_x = -(double)rand() / RAND_MAX * 0.15;
	//double tran_y = -(double)rand() / RAND_MAX * 0.1;
	//double tran_z = (double)rand() / RAND_MAX * 0.05;
	//double z_angle = -(double)rand() / RAND_MAX * 0.15;
	//double y_angle = (double)rand() / RAND_MAX * 0.15;
	//double x_angle = -(double)rand() / RAND_MAX * 0.15;

	//initBusbar = EulerZYX(Vec3(z_angle, y_angle, x_angle), Vec3(0.0 + tran_x, -0.4 + tran_y, -0.06 + tran_z))* jigAssem->GetBaseLink()->GetFrame();
	initBusbars[0] = SE3(Vec3(0.3, 0.4, -0.184))* jigAssem->GetBaseLink()->GetFrame();
	initBusbars[0].SetOrientation(RotZ(SR_PI_HALF));
	initBusbars[1] = SE3(Vec3(0.3, 0.5, -0.184))* jigAssem->GetBaseLink()->GetFrame();
	initBusbars[1].SetOrientation(RotZ(SR_PI_HALF));
	initBusbars[2] = SE3(Vec3(0.3, 0.6, -0.184))* jigAssem->GetBaseLink()->GetFrame();
	initBusbars[2].SetOrientation(RotZ(SR_PI_HALF));
	initBusbars[3] = SE3(Vec3(0.3, 0.7, -0.184))* jigAssem->GetBaseLink()->GetFrame();
	initBusbars[3].SetOrientation(RotZ(SR_PI_HALF));
	for (int i = 0; i < busbarNum; i++)
		busbar[i]->setBaseLinkFrame(initBusbars[i]);

	//initBusbar = SE3(Vec3(0.0 + tran_x, -0.4 + tran_y,  -0.06+tran_z)) * jigAssem->GetBaseLink()->GetFrame();
	//initSE3[0] = initBusbar;
	//busbar[0]->GetBaseLink()->SetFrame(initBusbar);
	attachObject.resize(nWay);
	vector<double> stepsize(nWay, 0.05);
	vector<int> busbarIdx(nWay);
	for (int i = 0; i < busbarNum; i++)
	{
		wayPoints[0 + 4 * i] = SE3(Vec3(0.0, 0.0, 0.04))*initBusbars[i];
		wayPoints[1 + 4 * i] = initBusbars[i];
		wayPoints[2 + 4 * i] = SE3(Vec3(0.0, 0.0, 0.02)) * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[holeNum[i]] * Thole2busbar;
		wayPoints[3 + 4 * i] = SE3(Vec3(0.0, 0.0, -0.02)) * wayPoints[2 + 4 * i];

		attachObject[0 + 4 * i] = false;
		attachObject[1 + 4 * i] = false;
		attachObject[2 + 4 * i] = true;
		attachObject[3 + 4 * i] = true;

		stepsize[1 + 4 * i] = 0.01;
		stepsize[3 + 4 * i] = 0.01;
		for (int j = 0; j < 4; j++)
			busbarIdx[j + 4 * i] = i;
	}
	
	RRT_problemSetting(homePos, wayPoints, includeOri, attachObject, busbarIdx);
	
	//busbar[0]->setBaseLinkFrame(initBusbar);

	if (!loadSuccess)
	{
		printf("do planning?: ");
		cin >> planning;
		if (planning)
			RRTSolve_HYU(attachObject, stepsize, busbarIdx);
	}
	FTtraj.resize(initPos.size());
	TtrajVec.resize(initPos.size());
	busbarTraj.resize(initPos.size());
	for (unsigned int i = 0; i < traj.size(); i++)
	{
		printf("%d-th traj length: %d\n", i, traj[i].size());
	}

	/////////////////////////////////////////////////////////////////////
	////////////////////// gather trajectories //////////////////////////
	/////////////////////////////////////////////////////////////////////
	for (int k = 0; k < busbarNum; k++)
		busbarTrajAll[k].push_back(initBusbars[k]);
	for (unsigned int i = 0; i < traj.size(); i++)
	{
		for (unsigned int j = 0; j < traj[i].size(); j++)
		{
			jointTrajAll.push_back(traj[i][j]);

			for (int k = 0; k < busbarNum; k++)
			{
				if (attachObject[i] && busbarIdx[i] == k)
					busbarTrajAll[k].push_back(rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)));
				else
					busbarTrajAll[k].push_back(busbarTrajAll[k].back());
			}
		}

	}
	for (int k = 0; k < busbarNum; k++)
		busbarTrajAll[k].erase(busbarTrajAll[k].begin() + 0);

	bool saveTrj = true;
	if (saveTrj)
	{
		string folder = "../../../data/busbar_retargetting/";
		vector<vector<Eigen::VectorXd>> busbarTrajAllVec(busbarNum);
		for (int k = 0; k < busbarNum; k++)
		{
			for (unsigned int i = 0; i < busbarTrajAll[k].size(); i++)
			{
				busbarTrajAllVec[k].push_back(SE3toVector(busbarTrajAll[k][i]));
			}
			saveDataToText(busbarTrajAllVec[k], folder + "busbarTraj" + to_string(k + 1) + ".txt");
		}
		saveDataToText(jointTrajAll, folder + "jointTraj.txt");
	}

	//rManager1->setJointVal(Eigen::VectorXd::Zero(6));
	//busbar[0]->setBaseLinkFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_new));
	//cout << robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() << endl;
	rendering(argc, argv);

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);

	//renderer->setUpdateFunc(updateFuncTestSensorToRobot);
	renderer->setUpdateFunc(updateFunc);
	//if (loadSuccess)
	//	renderer->setUpdateFunc(updateFuncLoadSuccessData);
	//else
	//{
	//	if (planning)
	//		renderer->setUpdateFunc(updateFuncPlanning);
	//	else
	//		renderer->setUpdateFunc(updateFuncInput);
	//}
	
	//renderer->setUpdateFunc(updateFuncPlanning);
	//renderer->setUpdateFunc(updateFuncLoadJointValAttachStatus);

	//renderer->setUpdateFunc(updateFunc);
	renderer->RunRendering();
}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, 0.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	
	//busbar[0]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -10.0)));
	static int cnt = 0;
	static int temp_cnt = 0;
	//rManager1->setJointVal(testJointValVec[0][cnt % (testJointValVec[0].size() - 1)]);
	//rManager2->setJointVal(testJointValVec[1][cnt % (testJointValVec[1].size() - 1)]);
	//Eigen::VectorXd testpos = homePos;
	//testpos[5] += 1.0;
	//Eigen::VectorXd testpos1 = Eigen::VectorXd::Zero(6);
	//testpos1[1] = DEG2RAD(30); testpos1[2] = DEG2RAD(-90); testpos1[3] = DEG2RAD(90); testpos1[4] = DEG2RAD(-100);
	//static double q3 = DEG2RAD(-235);

	//Eigen::VectorXd testpos2 = robot2->homePos;
	//static double q2 = robot2->homePos[1];
	//q2 -= 0.01;
	//testpos2[1] = q2;
	//q3 += 0.01;
	//testpos1[2] = q3;
	//rManager1->setJointVal(robot1->qInvKinInit);
	//cout << jointVal << endl;
	//rManager1->setJointVal(testJointVal[temp_cnt % testJointVal.size()]);
	//rManager1->setJointVal(testJointVal[testJointVal.size() - 1]);
	//rManager1->setJointVal(robot1->homePos);
	//rManager1->setJointVal(jointVal);
	//rManager2->setJointVal(robot2->homePos);
	//cout << Trobotbase1 % robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << Trobotbase2 % robot2->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() << endl;
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

	//if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	//{
	//	cout << q2 << endl;
	//	int stop = 1;
	//}
		

	//cout << testJointVal[testJointVal.size() - 1].transpose() << endl;
	//rManager1->setJointVal(testjointvalue);
	//rManager1->setJointVal(testWaypoint);
	//rManager1->setJointVal(traj[0][cnt%(traj[0].size()-1)]);
	cnt++;
	if (cnt % 5 == 0)
		temp_cnt++;
	
	
	///////////////////////////////////////////////////////////////
	//////////////////// plot init and goal pos ///////////////////
	///////////////////////////////////////////////////////////////
	//if (temp_cnt % 2 == 0)
	//{
	//	int idx = (temp_cnt / 2) % busbarNum;
	//	rManager1->setJointVal(initPos[idx * 4 + 1]);
	//	printf("init pos %d\n", idx);
	//	cout << initPos[idx * 4 + 1].transpose() << endl;
	//}
	//else
	//{
	//	int idx = (temp_cnt / 2) % busbarNum;
	//	rManager1->setJointVal(initPos[idx * 4 + 3]);
	//	printf("goal pos %d\n", idx);
	//	cout << initPos[idx * 4 + 3].transpose() << endl;
	//}
	//int idx = 10;
	//rManager1->setJointVal(initPos[idx * 4 + 1]);
	///////////////////////////////////////////////////////////////
	/////////////////// plot planned trajectory ///////////////////
	///////////////////////////////////////////////////////////////
	if (jointTrajAll.size() > 0)
	{
		rManager1->setJointVal(jointTrajAll[temp_cnt % jointTrajAll.size()]);
		for (int k = 0; k < busbarNum; k++)
			busbar[k]->setBaseLinkFrame(busbarTrajAll[k][temp_cnt % jointTrajAll.size()]);
	}
	else
		rManager1->setJointVal(homePos);

	//workCell->m_ObjWeldJoint[3].SetParentLinkFrame(SE3(Vec3(0.0, 0.0, (double)cnt * 0.01)));
	//workCell->KIN_UpdateFrame_All_The_Entity();
	//gSpace.KIN_MODE_PRESTEP();
	//if (!gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	//{
	//	cout << RAD2DEG(q3) << endl;
	//	int stop = 1;
	//}

	//if (loadJointVal.size() > 0)
	//	rManager1->setJointVal(loadJointVal[cnt % loadJointVal.size()]);
	//else
	//	rManager1->setJointVal(homePos);
	//cnt++;



	//static int taskNum = 0;
	//rManager1->setJointVal(goalPos[taskNum % goalPos.size()]);
	//if (cnt % 1000 == 0)
	//{
	//	taskNum++;
	//	cout << taskNum % goalPos.size() << endl;
	//}
		


	//static double alpha = 0.0;
	//static int cnt = 0;
	//alpha += 0.01;

	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -0.009;
	//gripInput[1] = 0.009;
	//rManager1->setGripperPosition(gripInput);
	//rManager2->setGripperPosition(gripInput);


	//// control acceleration
	////rManager1->controlJointAcc(acc);

	//// busbar movement
	////busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper));

	//// read sensor value
	////dse3 Ftsensor = rManager1->readSensorValue();
	////Eigen::VectorXd ftsensor = dse3toVector(Ftsensor);

	//// move stage
	////((srStateJoint*)workCell->pJoint[0])->m_State.m_rValue[0] = 0.05*sin(alpha);
	////((srStateJoint*)workCell->pJoint[1])->m_State.m_rValue[0] = 0.05*sin(2.0*alpha);
	////((srStateJoint*)workCell->rJoint[0])->m_State.m_rValue[0] = alpha;

	////test->GetBaseLink()->SetFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame());
	////test2->GetBaseLink()->SetFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame()*SE3(Vec3(0.0,0.0,0.03)));

	////if (cnt == 0)
	////	cout << robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() << endl;
	////cnt++;

	////static int temp = 0;
	////if (cnt % 30 == 0)
	////	temp++;
	////int idx = temp % initPos.size();
	////rManager1->setJointVal(initPos[idx]);
	//int idx = 1;
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);

	////cout << "taskIdx: " << idx << endl;
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

}

void updateFuncPlanning()
{
	
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);
	rManager2->setJointVal(robot2->homePos);

	static int cnt = 0;
	static int writeCnt = 0;
	static int writeTaskCnt = 0;

	static int trjIdx = 0;
	static int taskIdx = 0;

	int idx = taskIdx % traj.size();
	if (idx == 0 && cnt > 0)
		cnt = 0;
	if (cnt == 0)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
		cnt++;
	}
	
	// Calculate acc
	if (trjIdx == traj[idx].size() - 1)
		jointAcc = (traj[idx][trjIdx ] - 2.0*traj[idx][trjIdx - 1] + traj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == traj[idx].size() - 2)
		jointAcc = (traj[idx][trjIdx+1] - 2.0*traj[idx][trjIdx] + traj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (traj[idx][trjIdx + 2] - 2.0*traj[idx][trjIdx + 1] + traj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	
	SE3 renderEndeff = rManager1->forwardKin(traj[idx][trjIdx], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP]);


	// Calculate init vel
	if (trjIdx < traj[idx].size() - 1)
	{
		jointVel = (traj[idx][trjIdx + 1] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
		rManager1->setJointValVel(traj[idx][trjIdx], jointVel);
	}
	else
		jointVel = (traj[idx][trjIdx] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
	
	// control acceleration
	//rManager1->controlJointAcc(jointAcc);

	

	Eigen::VectorXd tau = rManager1->inverseDyn(traj[idx][trjIdx], jointVel, jointAcc);
	//rManager1->setJointVal(traj[idx][trjIdx]);
	rManager1->controlJointTorque(tau);

	//busbar movement
	if (attachObject[taskIdx % traj.size()])
	{
		busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper_new));
		busbar[0]->setBaseLinkFrame(renderEndeff * Inv(Tbusbar2gripper_new));
	}
	else
		busbar[0]->setBaseLinkFrame(initBusbar);

	// read sensor value
	Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3( rManager1->getBodyJacobian(traj[idx][trjIdx], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	Vdot = Vectortose3(rManager1->getBodyJacobian(traj[idx][trjIdx], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointAcc +
		rManager1->getBodyJacobianDot(traj[idx][trjIdx], jointVel, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V) - busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	
	if (attachObject[taskIdx % traj.size()])
		ftsensor = dse3toVector(Ftsensor + InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper_new), Fr));
	else
		ftsensor = dse3toVector(Ftsensor);


	//cout << "traj: " << traj[idx][trjIdx].transpose() << endl;
	//if (trjIdx < traj[idx].size()-1)
	//	cout << "trav: " << (traj[idx][trjIdx+1]-traj[idx][trjIdx]).transpose() << endl;
	//else
	//	cout << "trav: " << (traj[idx][trjIdx] - traj[idx][trjIdx]).transpose() << endl;
	//cout << "pos:  " << rManager1->getJointVal().transpose() << endl;
	//cout << "vel:  " << rManager1->getJointVel().transpose() << endl;
	//cout << "acc:  " << rManager1->getJointAcc().transpose() << endl;
	//cout << "ctrl: " << jointAcc.transpose() << endl << endl;
	//cout << "FT sensor:  " << ftsensor.transpose() << endl;
	//cout << "Ttraj:  " << Ttraj[idx][trjIdx] << endl << endl;




	

	//rManager1->setJointVal(traj[idx][trjIdx]);
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
	/*

	if (idxTraj[idx][trjIdx] != 100)
	busbar[objNum]->setBaseLinkFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);*/






	if (writeTaskCnt < traj.size())
	{
		if (writeCnt < traj[idx].size())
		{
			
			// make vector<> format
     		FTtraj[idx].push_back(ftsensor);
			// See from the robot base like frame
			TtrajVec[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*Ttraj[idx][trjIdx]));
			//TtrajVec[idx].push_back(SE3toVectorXd(Ttraj[idx][trjIdx]));
     		busbarTraj[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*(busbar[0]->GetBaseLink()->GetFrame())));
			goalJigLocation[0] = SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*jigAssem->GetBaseLink()->GetFrame()*Inv(jigAssem->m_visionOffset));
			
			writeCnt++;
			
		}
		if (writeCnt == traj[idx].size())
		{
			string dir_folder = "../../../data/test_traj";
			// save
			string dir_temp = dir_folder;
			saveDataToText(traj[idx], dir_temp.append("/jointValTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(FTtraj[idx], dir_temp.append("/sensorValTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(TtrajVec[idx], dir_temp.append("/robotEndTraj_robotbase").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(busbarTraj[idx], dir_temp.append("/busbarTraj_robotbase").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(goalJigLocation, dir_temp.append("/setting_robotbase").append(".txt"));

			writeCnt = 0;
			writeTaskCnt++;

			printf("%d-th task write done!! \n", idx);
		}
		
	}
		

	trjIdx++;

	
	if (trjIdx == traj[idx].size())
	{
		trjIdx = 0;

		taskIdx++;
		cout << "taskIdx: " << taskIdx % traj.size() << endl;
	}
	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	{
		/*cout << taskIdx << endl;
		cout << traj[idx][trjIdx].transpose() << endl;*/
	}
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}

void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation) // startLocation: x location according to jig number / goalLocation[0]: jig number, goalLocation[1]: front or back
{
	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i] = new Jig_HYU;
		jig[i]->SetBaseLinkType(srSystem::FIXED);
	}
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
		
	SE3 Tbase = EulerZYX(Vec3(SR_PI / 6, 0.0, 0.0), Vec3(0.025, 1.095, 1.176));
		
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));

	jig2busbar[0] = SE3(Vec3(0.00006, -0.0639, 0.0));
	jig2busbar[1] = SE3(Vec3(0.0, 0.0484, 0.01));
	jigSE3[0] = Tbase*SE3(Vec3(-0.1426, -0.0329, 0.01));
	jigSE3[1] = Tbase*SE3(Vec3(-0.0475, -0.0329, 0.01));
	jigSE3[2] = Tbase*SE3(Vec3(0.0475, -0.0329, 0.01));
	jigSE3[3] = Tbase*SE3(Vec3(0.1426, -0.0329, 0.01));

	busbarBase->setBaseLinkFrame(Tbase);

	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i]->setBaseLinkFrame(jigSE3[i]);
		if (!connect)
			gSpace.AddSystem(jig[i]);
		//for (unsigned int j = 0; j < 2; j++)
		//{
		//	busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
		//	gSpace.AddSystem(busbar[2 * i + j]);
		//}
	}
	if (!connect)
		gSpace.AddSystem((srSystem*)busbarBase);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(busbarBase->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase);
		wJoint->SetChildLinkFrame(SE3());
		vector<srWeldJoint*> wJoints(jig.size());
		for (unsigned int i = 0; i < jig.size(); i++)
		{
			wJoints[i] = new srWeldJoint;
			wJoints[i]->SetParentLink(workCell->getStagePlate());
			wJoints[i]->SetChildLink(jig[i]->GetBaseLink());
			wJoints[i]->SetParentLinkFrame(jigSE3[i]);
			wJoints[i]->SetChildLinkFrame(SE3());
		}
	}


	SE3 initSE3_x = jigSE3[startLocation] * jig2busbar[0];
	initSE3[0] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(initSE3_x.GetPosition()[0], 0.654, 1.1111 + 0.0001));

	goalSE3[0] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]] * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.10));

	initSE3[1] = goalSE3[0];
	goalSE3[1] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]];

	busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);
	gSpace.AddSystem(busbar[0]);

}


void updateFuncInput()
{
	gSpace.m_Gravity = Vec3(0.0, 0.0, 0.0);
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	rManager2->setJointVal(robot2->homePos);
	static int cnt = 0;
	int taskNum;
	if (cnt % 300 == 0)
	{
		printf("enter number: ");
		cin >> taskNum;
		if (taskNum < 0)
		{
			rManager1->setJointVal(initPos[0]);
			taskNum = 0;
		}
		else
			rManager1->setJointVal(goalPos[taskNum % goalPos.size()]);
		if (attachObject[taskNum % goalPos.size()])
			busbar[0]->setBaseLinkFrame(robot1->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper_new);
		else
			busbar[0]->setBaseLinkFrame(initBusbar);
		cout << "colli: " << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	}
	cnt++;
}

void updateFuncTestSensor()
{
	static Eigen::VectorXd testjointVel = Eigen::VectorXd::Ones(6);
	static int cnt = 0;
	if (cnt == 0)
	{
		rManager1->setJointValVel(homePos, testjointVel);
		jointVal = homePos;
		jointVel = testjointVel;
	}
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	jointVal = rManager1->getJointVal();
	jointVel = rManager1->getJointVel();
	
		
	cnt++;
	Eigen::VectorXd jointAcc = Eigen::VectorXd::Ones(6);
	Eigen::VectorXd tau = rManager1->inverseDyn(jointVal, jointVel, jointAcc);
	rManager1->controlJointTorque(tau);
	SE3 renderEndeff = rManager1->forwardKin(jointVal, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP]);
	busbar[0]->setBaseLinkFrame(renderEndeff*Inv(Tbusbar2gripper));
	// read sensor value
	dse3 Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	if (cnt == 0)
		jointAcc = rManager1->getJointAcc();
	Vdot = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointAcc +
		rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V);
	dse3 Fr_g = - (busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g));
	se3 g_bus = InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	SE3 Tbus = busbar[0]->GetBaseLink()->GetFrame();
	Fr += Fr_g;
	dse3 Fr_busbar = InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper), Fr);
	ftsensor = dse3toVector(Ftsensor + Fr_busbar);
	

	cout << "q: " << jointVal.transpose() << endl;
	cout << "a: " << rManager1->getJointAcc().transpose() << endl;
	cout << "f: " << ftsensor.transpose() << endl;
	cout << "busbar: " << endl << busbar[0]->GetBaseLink()->GetFrame() << endl;
	cout << "V: " << V << endl;
	cout << "A: " << Vdot << endl;
}

void environmentSetting_HYU2(bool connect)
{
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.099 + 0.009));	// when only stage4 is used
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	double z_angle = (double)rand() / RAND_MAX * 0.0;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = -(double)rand() / RAND_MAX * 0.1;
	SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	//SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
		busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, 10.0*(i+1))));
		gSpace.AddSystem(busbar[i]);
	}
	for (unsigned int i = 0; i < ctCase.size(); i++)
	{
		ctCase[i] = new Insert;
		ctCase[i]->SetBaseLinkType(srSystem::FIXED);
		ctCase[i]->setBaseLinkFrame(SE3(Vec3(0.0, 10.0, (double) 10.0 * (i + 1))));
		gSpace.AddSystem(ctCase[i]);
	}
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		wJointJig->SetParentLink(workCell->GetBaseLink()); // removed stage
		//wJointJig->SetParentLink(workCell->getStagePlate());
		wJointJig->SetChildLink(jigAssem->GetBaseLink());
		wJointJig->SetParentLinkFrame(Tbase*Tbase2jigbase);
		wJointJig->SetChildLinkFrame(SE3());
	}
}


void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	
	robot1->GetBaseLink()->SetFrame(SE3(Vec3(-0.5, 1.35, 1.0)));
	
	robot1->SetActType(srJoint::ACTTYPE::TORQUE);
	
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	
	//gSpace.AddSystem((srSystem*)robot2);
	//robot2->GetBaseLink()->SetFrame(SE3(Vec3(-0.7, 0.55, 0.95)));
	//robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	//robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new UR5RobotManager(robot1, &gSpace);

	// robot 2
	//rManager2 = new UR5RobotManager(robot2, &gSpace);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}

void rrtSetting()
{
	vector<srStateJoint*> planningJoints(6);
	vector<srStateJoint*> planningJoints2(6);
	for (unsigned int i = 0; i < planningJoints.size(); i++)
	{
		planningJoints[i] = (srStateJoint*)robot1->gJoint[i];
		planningJoints2[i] = (srStateJoint*)robot2->gJoint[i];
	}
		
	RRTManager1->setSystem(planningJoints);
	RRTManager1->setSpace(&gSpace);
	RRTManager1->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());

	//RRTManager2->setSystem(planningJoints2);
	//RRTManager2->setSpace(&gSpace);
	//RRTManager2->setStateBound(robot2->getLowerJointLimit(), robot2->getUpperJointLimit());
}


void RRT_problemSetting()
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	qInit[1] = 0.65*SR_PI;
	qInit[2] = -1.1*SR_PI;

	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);
	for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	{
		qInit = initPos[i - 1];
		goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit));
		cout << goalPos[i - 1].transpose() << endl;
		initPos.push_back(goalPos[i - 1]);
		if (i % 2 == 0)
			printf("%d-th init inv kin flag: %d\n", i / 2, flag);
		else
			printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	}
}

void RRT_problemSetting_HYU()
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	// elbow down
	qInit[2] = 0.6*SR_PI_HALF;
	qInit[4] = 0.6*SR_PI_HALF;
	qInit[3] = SR_PI_HALF;
	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit,1500));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);

	qInit = initPos[0];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[1] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[0].transpose() << endl;
	initPos.push_back(goalPos[0]);
	printf("%d-th init inv kin flag: %d\n", 1, flag);

	qInit = initPos[1];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[3] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[1].transpose() << endl;

	//for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	//{
	//	qInit = initPos[i - 1];
	//	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	//	cout << goalPos[i - 1].transpose() << endl;
	//	initPos.push_back(goalPos[i - 1]);
	//	if (i % 2 == 0)
	//		printf("%d-th init inv kin flag: %d\n", i / 2, flag);
	//	else
	//		printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	//}


}
void RRTSolve()
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = wayPoints.size();		// <= 15
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		objNum = i / 2;
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);

		for (unsigned int j = 0; j < goalSE3.size(); j++)
		{
			if (j < objNum)
				busbar[j]->setBaseLinkFrame(goalSE3[j]);
			else
				busbar[j]->setBaseLinkFrame(initSE3[j]);
		}
		if (i % 2 == 0)
		{
			isAttached = true;
			RRTManager1->attachObject(busbar[objNum], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			RRTManager1->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager1->execute(0.05);
		tempTraj = RRTManager1->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager1->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);

		tempIdxTraj.resize(tempTraj.size());
		for (unsigned int j = 0; j < tempIdxTraj.size(); j++)
		{
			if (isAttached)
				tempIdxTraj[j] = objNum;
			else
				tempIdxTraj[j] = 100;
		}

		idxTraj.push_back(tempIdxTraj);

		tempTtraj.resize(tempTraj.size());
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);
		}
			


		
	}
}


void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<int> busbarIdx)
{
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//// elbow up
	//qInit[1] = -0.65*SR_PI;
	//qInit[2] = 0.3*SR_PI;
	//qInit[3] = 0.5*SR_PI_HALF;
	//// elbow down
	////qInit[2] = 0.6*SR_PI_HALF;
	////qInit[4] = 0.6*SR_PI_HALF;
	////qInit[3] = SR_PI_HALF;
	//
	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;

	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	vector<bool> feas(2);
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		//qInit = initPos[i];
		goalPos.push_back(rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, robot1->qInvKinInit));
		//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag);
		if (flag != 0)
			goalPos[i] = rManager1->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[i]);
		printf("%d-th init inv kin flag: %d\n", i, flag);
		cout << goalPos[i].transpose() << endl;
		if (i < wayPoints.size() - 1)
			initPos.push_back(goalPos[i]);
		if (attachObject[i])
			RRTManager1->attachObject(busbar[busbarIdx[i]], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManager1->detachObject();


		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
	}
}

void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize, vector<int> busbarIdx)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = stepsize.size(); 		// <= 15
	vector<bool> feas(2);

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	
	clock_t begin_time, end_time;


	for (int i = start; i < end; i++)
	{
		begin_time = clock();
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);
		
		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;
		
		if (attachObject[i])
			RRTManager1->attachObject(busbar[busbarIdx[i]], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManager1->detachObject();


		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager1->execute(stepsize[i]);
		tempTraj = RRTManager1->extractPath();
		end_time = clock();
		
		cout << "way point RRT time: " << end_time - begin_time << endl;

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager1->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);


		tempTtraj.resize(tempTraj.size());

	
		for (unsigned int j = 0; j < traj[i].size(); j++)
			tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP]);
		Ttraj.push_back(tempTtraj);
	}
}

//void RRT_problemSetting_robot2(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject)
//{
//	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
//	// elbow up
//	qInit[1] = -0.65*SR_PI;
//	qInit[2] = 0.3*SR_PI;
//	qInit[3] = 0.5*SR_PI_HALF;
//	// elbow down
//	//qInit[2] = 0.6*SR_PI_HALF;
//	//qInit[4] = 0.6*SR_PI_HALF;
//	//qInit[3] = SR_PI_HALF;
//
//	Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
//	qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
//
//	int flag;
//	initPos.resize(0);
//	goalPos.resize(0);
//	initPos.push_back(init);
//	vector<bool> feas(2);
//	for (unsigned int i = 0; i < wayPoints.size(); i++)
//	{
//		//qInit = initPos[i];
//		goalPos.push_back(rManager2->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot2->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit2));
//		//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
//		if (flag != 0)
//			goalPos[i] = rManager2->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot2->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
//		if (flag != 0)
//			goalPos[i] = rManager2->inverseKin(wayPoints[i] * Tbusbar2gripper_new, &robot2->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[i]);
//		printf("%d-th init inv kin flag: %d\n", i, flag);
//		cout << goalPos[i].transpose() << endl;
//		if (i < wayPoints.size() - 1)
//			initPos.push_back(goalPos[i]);
//		if (attachObject[i])
//			RRTManager2->attachObject(busbar[0], &robot2->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
//		else
//			RRTManager2->detachObject();
//
//
//		feas = RRTManager2->checkFeasibility(initPos[i], goalPos[i]);
//		cout << feas[0] << feas[1] << endl;
//	}
//}

//void RRTSolve_HYU_robot2(vector<bool> attachObject, vector<double> stepsize)
//{
//	int nDim = 6;
//	vector<Eigen::VectorXd> tempTraj;
//	vector<int> tempIdxTraj(0);
//	vector<SE3> tempTtraj(0);
//	int start = 0;		//  >= 0
//	int end = stepsize.size(); 		// <= 15
//	vector<bool> feas(2);
//
//	traj.resize(0);
//	Ttraj.resize(0);
//	idxTraj.resize(0);
//
//	clock_t begin_time, end_time;
//
//
//	for (int i = start; i < end; i++)
//	{
//		begin_time = clock();
//		RRTManager2->setStartandGoal(initPos[i], goalPos[i]);
//
//		cout << "initpos:  " << initPos[i].transpose() << endl;
//		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;
//
//		if (attachObject[i])
//			RRTManager2->attachObject(busbar[0], &robot2->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
//		else
//			RRTManager2->detachObject();
//
//
//		feas = RRTManager2->checkFeasibility(initPos[i], goalPos[i]);
//		cout << feas[0] << feas[1] << endl;
//		RRTManager2->execute(stepsize[i]);
//		tempTraj = RRTManager2->extractPath();
//		end_time = clock();
//
//		cout << "way point RRT time: " << end_time - begin_time << endl;
//
//		// check collision
//		for (unsigned int j = 0; j < tempTraj.size(); j++)
//			if (RRTManager2->setState(tempTraj[j]))
//				cout << "collide at " << j << "th point!!!" << endl;
//
//		traj.push_back(tempTraj);
//
//
//		tempTtraj.resize(tempTraj.size());
//
//
//		for (unsigned int j = 0; j < traj[i].size(); j++)
//			tempTtraj[j] = rManager2->forwardKin(traj[i][j], &robot2->gMarkerLink[UR5_Index::MLINK_GRIP]);
//		Ttraj.push_back(tempTtraj);
//	}
//}


//void updateFuncTestSensorToRobot()
//{
//	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	hctrl->hybridPFControl();
//
//	cout << "q: " << rManager1->getJointVal().transpose() << endl;
//	cout << "F: " << rManager1->readSensorValue() << endl;
//	//cout << rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() << endl;
//	cout << "Trob: " << endl << Trobotbase1 % rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() << endl;
//	SE3 Tdes = Trobotbase1 % hctrl->T_des_trj[0];
//	cout << "Tdes: " << endl << Trobotbase1 % hctrl->T_des_trj[0] << endl;
//
//
//	SE3 Toffset = rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() % hctrl->T_des_trj[0];
//	cout << "off: " <<  Toffset[11] << endl;
//	cout << "des: " << Tdes[11] << endl;
//	int stop = 1;
//}
//
//void updateFuncLoadJointValAttachStatus()
//{
//	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	static int cnt = 0;
//	///////////////////////////// read from text
//	if (cnt == 0)
//	{
//		string dir_folder = "../../../data/communication_test";
//		loadJointVal = loadDataFromText(dir_folder + "/testJointValTraj_robot2.txt", 6);
//	}
//	
//	rManager1->setJointVal(jointVal);
//	rManager2->setJointVal(loadJointVal[cnt % loadJointVal.size()]);
//	cnt++;
//	if (loadAttachStatus.size() > 0)
//	{
//		if (abs(loadAttachStatus[cnt % loadAttachStatus.size()][0]) < DBL_EPSILON)
//		{
//		}
//		else
//		{
//			busbar[0]->setBaseLinkFrame(rManager1->forwardKin(loadJointVal[cnt % loadJointVal.size()],
//				&robot1->gMarkerLink[UR5_Index::MLINK_GRIP])*Inv(Tbusbar2gripper_new));
//		}
//	}
//
//}
//
//void updateFuncLoadSuccessData()
//{
//	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	static unsigned int len = loadJointVal.size();
//	static unsigned int cnt = 0;
//
//	rManager1->setJointVal(loadJointVal[cnt % len]);
//	rManager2->setJointVal(robot2->homePos);
//
//	busbar[0]->setBaseLinkFrame(Trobotbase1 * VectorXdtoSE3(loadBusbar[cnt % len]) * busbar[0]->m_visionOffset);
//	busbar[0]->KIN_UpdateFrame_All_The_Entity();
//
//	cnt++;
//}
//
//void loadSuccessData(int nWay, bool connect)
//{
//	string dir_folder = "../../../data/test_traj";
//	vector<Eigen::VectorXd> tempVecTraj;
//	vector<Eigen::VectorXd> tempSE3Traj;
//	loadJointVal.resize(0);
//	loadBusbar.resize(0);
//	for (int i = 0; i < nWay; i++)
//	{
//		tempVecTraj = loadDataFromText(dir_folder + "/jointValTraj" + to_string(i) + ".txt", 6);
//		tempSE3Traj = loadDataFromText(dir_folder + "/busbarTraj_robotbase" + to_string(i) + ".txt", 12);
//
//		for (unsigned int j = 0; j < tempVecTraj.size(); j++)
//		{
//			loadJointVal.push_back(tempVecTraj[j]);
//			loadBusbar.push_back(tempSE3Traj[j]);
//		}
//	}
//	SE3 Tjig = Trobotbase1 * VectorXdtoSE3(loadDataFromText(dir_folder + "/setting_robotbase" + ".txt", 12)[0]) * jigAssem->m_visionOffset;
//	if (connect)
//		wJointJig->SetParentLinkFrame(Tjig);
//	else
//		jigAssem->GetBaseLink()->SetFrame(Tjig);
//}
