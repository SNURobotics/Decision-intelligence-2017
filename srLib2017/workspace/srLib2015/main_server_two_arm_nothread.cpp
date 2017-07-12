#include <cstdio>

//#include "myRenderer.h"
#include "serverRenderer.h"

#include "common\dataIO.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\robotRRTManager.h"
#include <fstream>
#include <iostream>
//#include <thread>

#include <mutex>

#include "../VS2013/tcp_ip_server/stdafx.h"
#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include "../VS2013/tcp_ip_server/Server.h"
#include "tcp_ip_communication.h"
#include "Eigen/Dense"

#include <stdlib.h>
#include <vector>

// memory leakaage check
#include <crtdbg.h>

static mutex m;

//srLib
srSpace gSpace;
//serverRenderer* renderer;

// Environment
JigAssem_QB_bar* jigAssem = new JigAssem_QB_bar(false);
vector<BusBar_HYU*> busbar(8);
vector<Insert*> ctCase(4);
vector<Object*> objects(busbar.size() + ctCase.size());
vector<SE3> TobjectsInitSimul(objects.size());
bool isJigConnectedToWorkCell = true;
SE3 initBusbar = SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.5)));

// Workspace
int workcell_mode = 0;
WorkCell* workCell = new WorkCell(workcell_mode);
Eigen::VectorXd stageVal(3);
bool useNoVisionTestSetting = true;
bool useNoVisionTestSettingJig = true;

// Robot
IndyRobot* robot1 = new IndyRobot(false);
IndyRobot* robot2 = new IndyRobot(false);
vector<IndyRobot*> robotVector(2);
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Tbusbar2gripper_tight = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
SE3 TctCase2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.01));
vector<SE3> Tobject2gripper(objects.size());
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
SE3 Trobotbase1;
SE3 Trobotbase2;
vector<SE3> TrobotbaseVector(2);

indyRobotManager* rManager1;
indyRobotManager* rManager2;
vector<indyRobotManager*> rManagerVector(2);
robotRRTManager* RRTManager1 = new robotRRTManager;
robotRRTManager* RRTManager2 = new robotRRTManager;
vector<robotRRTManager*> RRTManagerVector(2);

// Planning
vector<vector<vector<Eigen::VectorXd>>> renderTraj_multi(2);
vector<vector<SE3>>	Ttraj(0);
vector<vector<Eigen::VectorXd>> qWaypoint(2);
vector<vector<bool>> attachObjectWaypoint(2);
vector<vector<int>> idxTraj(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
bool checkTorque = false;
Eigen::VectorXd qval1;
Eigen::VectorXd qval2;

// Rendering flags
static bool isVision = false;
static bool isHYUPlanning = false;
static bool isRobotState = true;
static bool isWaypoint = false;

// save last gripper state
int gripState = 0;
vector<int> gripState_multi(2);
vector<int> gripObjectIdx(2, -1);			// save which busbar is moving with each robot during planning
vector<int> gripObjectIdxRender(2, -1);		// save which busbar is moving with each robot during rendering
vector<Eigen::VectorXd> lastJointVal_multi(2);// save last joint value

// save initial and final busbar SE(3)
vector<SE3> TinitObjects_multi(2);
vector<SE3> TinitObjects_multiRender(2);
vector<bool> initialObjectSaved(2, false);			// save if initial busbar location is saved (saved when busbar is moving)
vector<bool> initialObjectSavedRender(2, false);
vector<SE3>	TlastObjects_multi(2);
vector<bool> initialPlanning(2, true);
vector<bool> startPlanningFromCurRobotState(2, false);
vector<bool> attachObjRender(0);
vector<vector<bool>> attachObjRender_multi(2);
vector<Eigen::VectorXd> homePosRobotVector(2);

// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);
bool saveTraj = true;




// modelling functions
void workspaceSetting();
void robotSetting();
void environmentSetting_HYU2(bool connect);
void objectSetting();
void connectJigToWorkCell();
void initDynamics();
void robotManagerSetting();

// rendering functions
//void rendering(int argc, char **argv);
//void updateFunc();
//void updateFuncVision();
//void updateFuncPlanning_multi();
//void updateFuncWaypoint();
//void updateFuncRobotState();
//void updateFuncTotal();

// communication function
void setEnviromentFromVision(const vision_data& skku_dataset, int& bNum, int& cNum);
void setRobotFromRealRobot(const robot_current_data& robot_state);
char* getSimulationState(vector<srSystem*> objects);

// RRT multi-robot
void rrtSetting();
void RRT_problemSettingFromSingleRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag, int robotFlag);
void RRT_problemSetting_SingleRobot(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag, int robotFlag);
void RRTSolve_HYU_SingleRobot(vector<bool> attachObject, vector<double> stepsize, int robotFlag);
int getObjectIdx(int robotIdx);
void planning_demo(int robotFlag, int objNum);
vector<Eigen::VectorXd> calculateJointTorque(vector<vector<Eigen::VectorXd>>& traj, int robotFlag);
bool saveTrajectories = true;
void savePlannedResultToText(unsigned int robotFlag, vector<vector<Eigen::VectorXd>>& traj, vector<vector<bool>>& attachObjectVec);

void communicationFunc(int argc, char **argv);		// run on the other thread



// 서버 초기화
Server serv = Server::Server();
dataset hyu_dataset;
vector<desired_dataset> hyu_desired_dataset;
vision_data skku_dataset;
robot_current_data robot_state;

//char hyu_data[30000];
char hyu_data_flag;
bool useSleep = false;
bool isSystemAssembled = false; 

vector<srLink*> obstacle(0);
vector<srWeldJoint*> wJoint(0);		// weld joint for connecting workcell and obstacle

SE3 Tjigbase;

int main(int argc, char **argv)
{
	//Eigen::initParallel();
	bool useVision = false;
	if (useVision)
		useNoVisionTestSetting = false;
	srand(time(NULL));
	// Robot home position
	robotSetting();
	homePosRobotVector[0] = robot1->homePos;
	homePosRobotVector[1] = robot2->homePos;
	// environment
	workspaceSetting();
	objectSetting();
	////////////////////////////////////////////// setting environment (replacable from vision data)
	if (!useVision)
	{
		environmentSetting_HYU2(true);			// temporary environment setting
		initDynamics();								// initialize srLib				
		isSystemAssembled = true;
		robotManagerSetting();						// robot manager setting

		// workcell robot initial config
		rManager1->setJointVal(robot1->homePos);
		rManager2->setJointVal(robot2->homePos);
		Eigen::VectorXd gripInput(2);
		gripInput[0] = -0.005;
		gripInput[1] = 0.005;
		rManager1->setGripperPosition(gripInput);
		rManager2->setGripperPosition(gripInput);

		//cout << Trobotbase2 % robot2->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;


		// rrt
		rrtSetting();

		serv.SendMessageToClient("Ld0d");
		if (useSleep)
			Sleep(50);
	}


	communicationFunc(argc, argv);

	//thread commuThread(communicationFunc, argc, argv);
	//commuThread.detach();

	//if (commuThread.joinable())
	//	commuThread.join();


	//if (useVision)
	//{
	//	renderer = new serverRenderer();
	//	SceneGraphRenderer::NUM_WINDOWS windows;
	//	windows = SceneGraphRenderer::SINGLE_WINDOWS;
	//	renderer->InitializeRenderer(argc, argv, windows, false);
	//	renderer->InitializeNode_1st(&gSpace);
	//	while (1)
	//	{
	//		if (isVision)
	//			break;
	//	}
	//	// generate renderer
	//	renderer->InitializeNode_2nd();
	//	renderer->setUpdateFunc(updateFuncTotal);
	//	//renderer->setUpdateFunc(updateFunc);
	//	renderer->RunRendering();
	//}
	//else
	//{
	//	rendering(argc, argv);
	//}

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

//void rendering(int argc, char **argv)
//{
//	renderer = new serverRenderer();
//
//	SceneGraphRenderer::NUM_WINDOWS windows;
//
//	windows = SceneGraphRenderer::SINGLE_WINDOWS;
//
//	renderer->InitializeRenderer(argc, argv, windows, false);
//	renderer->InitializeNode_1st(&gSpace);
//	renderer->InitializeNode_2nd();
//	renderer->setUpdateFunc(updateFuncTotal);
//	//renderer->setUpdateFunc(updateFunc);
//	renderer->RunRendering();
//}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

//void updateFunc()
//{
//	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	qval1 = robot1->homePos;
//	//qval2 = robot2->homePos;
//	rManager2->setJointVal(qval2);
//	rManager1->setJointVal(qval1);
//	//static double th = 0.0;
//	//busbar[0]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, 2.0 * sin(th))));
//	//th += 0.1;
//	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
//	//int stop = 1;
//}
//
//void updateFuncVision()
//{
//	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	// robot to homePos
//	rManager2->setJointVal(robot2->homePos);
//	rManager1->setJointVal(robot1->homePos);
//	//printf("Vision Update func called\n");
//	//cout << Trobotbase1 % jigAssem->GetBaseLink()->GetFrame() << endl;
//	//cout << Trobotbase1 % busbar[0]->GetBaseLink()->GetFrame() << endl;
//	int stop = 0;
//}
//
//void updateFuncRobotState()
//{
//	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	// robot to homePos
//	//rManager2->setJointVal(homePosRobot2);
//	//rManager1->setJointVal(homePosRobot1);
//	static int updateFuncCall = 0;
//	updateFuncCall++;
//	//int flag;
//	//Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
//	//qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;
//
//	//Eigen::VectorXd jointVal = rManager2->inverseKin(ctCase[0]->GetBaseLink()->GetFrame() * TctCase2gripper, &robot2->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit2);
//	//rManager2->setJointVal(jointVal);
//	//printf("updateFunc called: %d\n", updateFuncCall);
//}


void environmentSetting_HYU2(bool connect)
{
	// should be called later than workcell setting
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
	{
		Vec3 stage4Trans = workCell->m_ObjWeldJoint[3].GetParentLinkFrame().GetPosition();
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.105 + 0.009) + stage4Trans);	// when only stage4 is used
	}
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	double z_angle = (double)rand() / RAND_MAX * 1.0;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = (double)rand() / RAND_MAX * 0.1;
	//SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	//for (unsigned int i = 0; i < busbar.size(); i++)
	//{
	//	busbar[i] = new BusBar_HYU;
	//	busbar[i]->SetBaseLinkType(srSystem::FIXED);
	//	gSpace.AddSystem(busbar[i]);
	//}
	//Vec3 testJigPosFromRobot1(-0.702151, -0.014057, 0.750026);		// 17.06.09 using robot1
	//Vec3 testJigPosFromRobot1(-0.8254, 0.0338, 0.7483);		// 17.06.10 using robot2
	Vec3 testJigPosFromRobot1(-0.8277, - 0.0536,    0.8620);		// 17.06.10 using robot2
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	if (!useNoVisionTestSettingJig)
		jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
	else
	{
		SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
		jigAssem->setBaseLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
	}
		
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->GetBaseLink()); // removed stage
		//wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(jigAssem->GetBaseLink());
		if (!useNoVisionTestSettingJig)
			wJoint->SetParentLinkFrame(Tbase*Tbase2jigbase);
		else
		{
			SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
			SE3 Tjig = Trobotbase1 % SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset;
			//cout << "Tjig" << endl;
			//cout << Tjig << endl;
			wJoint->SetParentLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
		}
		wJoint->SetChildLinkFrame(SE3());
	}

	// ctCase test
	//ctCase[0]->setBaseLinkFrame(Tbase*Tbase2jigbase*SE3(Vec3(0.0, 0.0, 0.03)));
}


void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005 - 0.12, 1.972)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005 + 0.12, 1.972)));
	robot1->SetActType(srJoint::ACTTYPE::TORQUE);
	robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	robotVector[0] = robot1;
	robotVector[1] = robot2;
	Trobotbase1 = robot1->GetBaseLink()->GetFrame();
	Trobotbase2 = robot2->GetBaseLink()->GetFrame();
	TrobotbaseVector[0] = Trobotbase1;
	TrobotbaseVector[1] = Trobotbase2;
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new indyRobotManager(robot1, &gSpace);

	// robot 2
	rManager2 = new indyRobotManager(robot2, &gSpace);


	rManagerVector[0] = rManager1;
	rManagerVector[1] = rManager2;
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
	// change stage4 location
	if (workcell_mode == 2)
	{
		Vec2 stage4xyTrans(0.0, 0.0);
		workCell->m_ObjWeldJoint[3].SetParentLinkFrame(SE3(Vec3(stage4xyTrans[0], stage4xyTrans[1], 0.0)));
	}

	
}

void rrtSetting()
{
	vector<srStateJoint*> planningJoints1(6);
	vector<srStateJoint*> planningJoints2(6);
	for (unsigned int i = 0; i < planningJoints1.size(); i++)
	{
		planningJoints1[i] = (srStateJoint*)robot1->gJoint[i];
		planningJoints2[i] = (srStateJoint*)robot2->gJoint[i];
	}
		

	RRTManager1->setSystem(planningJoints1);
	RRTManager1->setSpace(&gSpace);
	RRTManager1->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
	RRTManager2->setSystem(planningJoints2);
	RRTManager2->setSpace(&gSpace);
	RRTManager2->setStateBound(robot2->getLowerJointLimit(), robot2->getUpperJointLimit());

	RRTManagerVector[0] = RRTManager1;
	RRTManagerVector[1] = RRTManager2;
}




void RRT_problemSetting_SingleRobot(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag, int robotFlag)
{
	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	if (attachObject[0] && gripObjectIdx[robotFlag - 1] != -1)
		RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
	else
		RRTManagerVector[robotFlag - 1]->detachObject();
	bool feas = RRTManagerVector[robotFlag-1]->checkFeasibility(init);
	if (feas != 0)
		printf("initial point not feasible!!!\n");
	Eigen::VectorXd qtemp;
	waypointFlag.resize(wayPoints.size());
	qWaypoint[robotFlag - 1].resize(0);
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, robotVector[robotFlag - 1]->qInvKinInit);
		//if (flag != 0)
		//	qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
		if (flag != 0)
			qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[initPos.size() - 1]);
		printf("%d-th init inv kin flag: %d\n", i, flag);

		if (attachObject[i] && gripObjectIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();

		//cout << TrobotbaseVector[0] * wayPoints[i] << endl;
		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(qtemp);
		qWaypoint[robotFlag - 1].push_back(qtemp);
		if (feas == 0 && flag == 0)
		{
			waypointFlag[i] = true;
			goalPos.push_back(qtemp);
			if (i < wayPoints.size() - 1)
				initPos.push_back(goalPos[goalPos.size() - 1]);
		}
		else
		{
			waypointFlag[i] = false;
			if (i == wayPoints.size() - 1)
				printf("final waypoint not feasible!!!\n");
			else
				printf("%d-th waypoint not feasible!!!\n", i + 1);
			if (i > 0 && attachObject[i] != attachObject[i - 1])
				printf("grasp point is not feasible!!!\n");
		}
	}
}


void RRTSolve_HYU_SingleRobot(vector<bool> attachObject, vector<double> stepsize, int robotFlag)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	//vector<int> tempIdxTraj(0);
	//vector<SE3> tempTtraj(0);
	int start = 0;
	int end = goalPos.size();
	vector<bool> feas(2);
	vector<vector<Eigen::VectorXd>> traj(0);
	//Ttraj.resize(0);
	//idxTraj.resize(0);
	printf("waypoints: \n");
	for (unsigned int i = 0; i < initPos.size(); i++)
		cout << initPos[i].transpose() << endl;
	if (goalPos.size() > 0)
		cout << goalPos[goalPos.size() - 1].transpose() << endl;
	initialObjectSaved[robotFlag - 1] = false;
	for (int i = start; i < end; i++)
	{
		RRTManagerVector[robotFlag-1]->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		if (attachObject[i] && gripObjectIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag-1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();


		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManagerVector[robotFlag - 1]->execute(stepsize[i]);
		tempTraj = RRTManagerVector[robotFlag - 1]->extractPath(40);

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManagerVector[robotFlag - 1]->setState(tempTraj[j]))
				printf("collide at %d-th trj, %d-th point!!!\n", i, j);

		traj.push_back(tempTraj);


		if (gripObjectIdx[robotFlag - 1] != -1)
		{
			// set busbar final location
			// if busbar is attached, busbar will be located at last location. 
			// otherwise, busbar will not move, initial and final location will be the same.
			RRTManagerVector[robotFlag - 1]->setState(tempTraj[tempTraj.size() - 1]);
			TlastObjects_multi[robotFlag - 1] = objects[gripObjectIdx[robotFlag - 1]]->GetBaseLink()->GetFrame();
			cout << "TlastBusbar" << endl;
			cout << Trobotbase1 % TlastObjects_multi[robotFlag - 1] << endl;
			if (attachObject[i])
			{
				// set busbar initial location
				if (!initialObjectSaved[robotFlag - 1])
				{
					initialObjectSaved[robotFlag - 1] = true;
					RRTManagerVector[robotFlag - 1]->setState(tempTraj[0]);
					TinitObjects_multi[robotFlag - 1] = objects[gripObjectIdx[robotFlag - 1]]->GetBaseLink()->GetFrame();
					cout << "TinitBusbar" << endl;
					cout << Trobotbase1 % TinitObjects_multi[robotFlag - 1] << endl;
				}
			}
		}
			
		//tempTtraj.resize(tempTraj.size());
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
	renderTraj_multi[robotFlag-1] = traj;
	// save last joint val
	if (goalPos.size() > 0)
		lastJointVal_multi[robotFlag - 1] = traj[traj.size() - 1][traj[traj.size() - 1].size() - 1];
	else
		lastJointVal_multi[robotFlag - 1] = initPos[0];

	// save planned result to text file
	if (saveTrajectories)
	{
		vector<vector<bool>> attachObjectVec(0);
		attachObjectVec.push_back(attachObject);
		savePlannedResultToText(robotFlag, traj, attachObjectVec);
	}
}




void objectSetting()
{
	vector<SE3> testInit(8);
	// 17.06.09 (using robot1)
	testInit[0] = Trobotbase1 * SE3(-0.51328, 0.85822, 1.1137e-06, 0.85822, 0.51328, 5.4682e-06, 3.5287e-06, 3.3661e-06, -1, -0.28018, -0.10959, 0.85844);
	testInit[1] = Trobotbase1 * SE3(-0.9354,0.35358,5.9942e-06,0.35358,0.9354,-6.9362e-06,-8.4326e-06,-4.4553e-06,-1,-0.26685,-0.020595,0.85866);
	testInit[2] = Trobotbase1 * SE3(-0.66831,-0.74389,-1.1055e-05,-0.74389,0.66831,-4.5087e-06,9.9899e-06,5.0997e-06,-1,-0.3734,0.011193,0.85824);
	testInit[3] = Trobotbase1 * SE3(-0.20649,0.97845,-2.1991e-05,0.97845,0.20649,-1.3097e-06,2.7943e-06,-2.1829e-05,-1,-0.36767,0.11754,0.85743);
	// 17.06.10 (using robot2)
	SE3 Trans = SE3(Vec3(-0.35, -0.1, 0.0));
	testInit[4] = Trans * Trobotbase1 * SE3(0.49715, -0.86767, -2.3813e-06, -0.86767, -0.49714, -1.4671e-06, 4.218e-07, 2.6782e-06, -1, -1.2308, 0.17082, 1.0477);
	testInit[5] = Trans * Trobotbase1 * SE3(-0.11762, -0.99306, 1.039e-06, -0.99306, 0.11762, -5.0989e-06, 5.0868e-06, -8.7669e-07, -1, -1.204, 0.028933, 1.0467);
	testInit[6] = Trans * Trobotbase1 * SE3(-0.84321, -0.53758, 6.7572e-06, -0.53758, 0.84321, 4.3601e-06, -8.1082e-06, 3.8774e-07, -1, -1.2815, -0.060334, 1.0468);
	testInit[7] = Trans * Trobotbase1 * SE3(-0.014703, -0.99989, -8.5711e-07, -0.99989, 0.014703, -3.6814e-06, 3.9231e-06, 7.9429e-08, -1, -1.2911, 0.095115, 1.0473);
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		if (useNoVisionTestSetting && i < testInit.size())
			busbar[i]->setBaseLinkFrame(testInit[i]);
		else
			busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		gSpace.AddSystem(busbar[i]);
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
	for (unsigned int i = 0; i < ctCase.size(); i++)
	{
		ctCase[i] = new Insert;
		ctCase[i]->SetBaseLinkType(srSystem::FIXED);
		ctCase[i]->setBaseLinkFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
		gSpace.AddSystem(ctCase[i]);
	}
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		if (i < busbar.size())
		{
			objects[i] = busbar[i];
			Tobject2gripper[i] = Tbusbar2gripper_new;
		}
		else if (i < busbar.size() + ctCase.size())
		{
			objects[i] = ctCase[i - busbar.size()];
			Tobject2gripper[i] = TctCase2gripper;
		}
		TobjectsInitSimul[i] = objects[i]->GetBaseLink()->GetFrame();
	}
	
}

void setEnviromentFromVision(const vision_data & skku_dataset, int& bNum, int& cNum)
{
	// set object (id:1 - busbar, id:2 - CTcase (insert), id:3 - jig) 
	int bIdx = 0;
	int cIdx = 0;
	for (unsigned int i = 0; i < skku_dataset.objID.size(); i++)
	{
		if (skku_dataset.objID[i] == 1)
		{
			printf("objID: %d, index: %d\n", skku_dataset.objID[i], bIdx);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			busbar[bIdx]->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * busbar[bIdx]->m_visionOffset);
			busbar[bIdx]->KIN_UpdateFrame_All_The_Entity();
			bIdx++;
		}
		else if (skku_dataset.objID[i] == 2)
		{
			printf("objID: %d, index: %d\n", skku_dataset.objID[i], cIdx);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			ctCase[cIdx]->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * ctCase[cIdx]->m_visionOffset);
			ctCase[cIdx]->KIN_UpdateFrame_All_The_Entity();
			cIdx++;
		}
		else if (skku_dataset.objID[i] == 3)
		{
			// only set once when jig is connected to workcell by weld joint
			printf("objID: %d\n", skku_dataset.objID[i]);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			jigAssem->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * jigAssem->m_visionOffset);
			jigAssem->KIN_UpdateFrame_All_The_Entity();
		}
		else if (skku_dataset.objID[i] != 0)
			printf("object ID is outside range!!!\n");
	}
	bNum = bIdx;
	cNum = cIdx;
	// set other objects to far away location
	for (unsigned int i = bIdx; i < busbar.size(); i++)
	{
		busbar[bIdx]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		busbar[bIdx]->KIN_UpdateFrame_All_The_Entity();
	}
	for (unsigned int i = cIdx; i < ctCase.size(); i++)
	{
		ctCase[cIdx]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
		ctCase[cIdx]->KIN_UpdateFrame_All_The_Entity();
	}
	// set obstacle (only set by the first vision input)
	if (!isSystemAssembled)
	{
		obstacle.resize(skku_dataset.obsInfo.size());
		wJoint.resize(skku_dataset.obsInfo.size());
		for (unsigned int i = 0; i < skku_dataset.obsInfo.size(); i++)
		{
			obstacle[i] = new srLink();
			wJoint[i] = new srWeldJoint;
			obstacle[i]->GetGeomInfo().SetDimension(skku_dataset.obsInfo[i][3], skku_dataset.obsInfo[i][4], skku_dataset.obsInfo[i][5]);
			wJoint[i]->SetParentLink(workCell->GetBaseLink());
			wJoint[i]->SetParentLinkFrame(robot1->GetBaseLink()->GetFrame()*SE3(Vec3(skku_dataset.obsInfo[i][0], skku_dataset.obsInfo[i][1], skku_dataset.obsInfo[i][2])));
			wJoint[i]->SetChildLink(obstacle[i]);
			wJoint[i]->SetChildLinkFrame(SE3());
			obstacle[i]->GetGeomInfo().SetColor(0.2, 0.2, 0.2);
		}
	}
}

void setRobotFromRealRobot(const robot_current_data & robot_state)
{
	rManager1->setJointVal(robot_state.robot_joint);
}

char * getSimulationState(vector<srSystem*> objects)
{
	string tmp_data;
	char *pbuffer;

	char tmp_buffer[255];
	char divChar = 'd';
	int digit_num = 5;

	//Robot EE Position (3x1)
	SE3 T = Trobotbase1 % robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame();
	//cout << T << endl;
	for (int i = 0; i < 3; i++) {
		pbuffer = _gcvt(T[9 + i], digit_num, tmp_buffer);

		tmp_data = tmp_data + pbuffer;
		tmp_data = tmp_data + divChar;
	}

	//Robot EE Rotation (3x3)
	for (int i = 0; i < 9; i++) {
		pbuffer = _gcvt(T[i], digit_num, tmp_buffer);

		tmp_data = tmp_data + pbuffer;
		tmp_data = tmp_data + divChar;
	}

	//Robot Gripper (1x1)
	pbuffer = _gcvt(gripState, digit_num, tmp_buffer);

	tmp_data = tmp_data + pbuffer;
	tmp_data = tmp_data + divChar;

	//Robot FTsensor (6x1)
	dse3 ftsensor = rManager1->readSensorValue();
	for (int i = 0; i < 6; i++) {
		pbuffer = _gcvt(ftsensor[i], digit_num, tmp_buffer);

		tmp_data = tmp_data + pbuffer;
		tmp_data = tmp_data + divChar;
	}
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		SE3 Ttemp = Trobotbase1 % objects[i]->GetBaseLink()->GetFrame();
		//cout << Ttemp << endl;
		//Object1 Position (3x1)
		for (int i = 0; i < 3; i++) {
			pbuffer = _gcvt(Ttemp[9 + i], digit_num, tmp_buffer);

			tmp_data = tmp_data + pbuffer;
			tmp_data = tmp_data + divChar;
		}

		//Object1 Rotation (3x3)
		for (int i = 0; i < 9; i++) {
			pbuffer = _gcvt(Ttemp[i], digit_num, tmp_buffer);

			tmp_data = tmp_data + pbuffer;
			tmp_data = tmp_data + divChar;
		}
	}


	char *send_data = new char[tmp_data.length() + 1];
	strcpy(send_data, tmp_data.c_str());
	return send_data;
}

void RRT_problemSettingFromSingleRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag, int robotFlag)
{
	unsigned int nWay = hyu_desired_dataset.robot_pos.size() / 3;
	attachObject.resize(nWay);
	vector<bool> includeOri(nWay, true);
	vector<SE3> wayPoints(nWay);
	vector<double> pos(3);
	vector<double> ori(9);
	for (unsigned int i = 0; i < nWay; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			pos[j] = hyu_desired_dataset.robot_pos[i * 3 + j];
			for (int k = 0; k < 3; k++)
				ori[3 * j + k] = hyu_desired_dataset.robot_rot[3 * j + k + 9 * i];
		}
		wayPoints[i] = SKKUtoSE3(ori, pos);
		if (abs(hyu_desired_dataset.robot_gripper[i] - 1.0) < DBL_EPSILON)
			attachObject[i] = true;
		else
			attachObject[i] = false;
	}

	RRT_problemSetting_SingleRobot(init, wayPoints, includeOri, attachObject, waypointFlag, robotFlag);
}



void communicationFunc(int argc, char **argv)
{
	while (TRUE) {

		//Receiving data from HYU client
		char* hyu_data = serv.RecevData();
		//strcpy(hyu_data, "");
		//strcat(hyu_data, serv.RecevData());

		hyu_data_flag = hyu_data[0];

		//printf(&hyu_data_flag);
		//cout << endl;
		//serv.SendMessageToClient("G");

		// 데이터 전송
		if (hyu_data_flag == 'I')
		{
			serv.SendMessageToClient("I");
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'D')
		{
			serv.SendMessageToClient("D");
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'F')
		{
			serv.SendMessageToClient("F");
			if (useSleep)
				Sleep(50);
		}

		else if (hyu_data_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			Sleep(50);
			printf("%s\n", hyu_data);
			readSKKUvision(hyu_data, skku_dataset);

			int bNum = 0;
			int cNum = 0;
			setEnviromentFromVision(skku_dataset, bNum, cNum);		// should be called later than robotSetting
			

			if (!isSystemAssembled)
			{
				if (isJigConnectedToWorkCell)
					connectJigToWorkCell();
				/////////////////////////////////////// after setting environment
				initDynamics();								// initialize srLib

				isSystemAssembled = true;
				robotManagerSetting();						// robot manager setting

															// workcell robot initial config
				rManager2->setJointVal(robot2->homePos);
				rManager1->setJointVal(robot1->homePos);
				Eigen::VectorXd gripInput(2);
				gripInput[0] = -0.005;
				gripInput[1] = 0.005;
				rManager1->setGripperPosition(gripInput);
				rManager2->setGripperPosition(gripInput);

				// rrt
				rrtSetting();
			}
			
			// lift objects if collision occur
			bool liftObjects = false;
			if (liftObjects)
			{
				Vec3 delta_z = Vec3(0.0, 0.0, 0.0001);
				int liftIter = 0;
				while (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
				{
					for (int i = 0; i < bNum; i++)
					{
						busbar[i]->GetBaseLink()->SetPosition(busbar[i]->GetBaseLink()->GetPosition() + delta_z);
						busbar[i]->KIN_UpdateFrame_All_The_Entity();
					}
					for (int i = 0; i < cNum; i++)
					{
						ctCase[i]->GetBaseLink()->SetPosition(ctCase[i]->GetBaseLink()->GetPosition() + delta_z);
						ctCase[i]->KIN_UpdateFrame_All_The_Entity();
					}
					liftIter++;
				}
				char liftBuf[20];
				sprintf(liftBuf, "Ld%fd", -(double)liftIter * delta_z[2]);
				serv.SendMessageToClient(liftBuf);
				if (useSleep)
					Sleep(50);
				printf(liftBuf);
				printf("\n");
			}
			else
			{
				serv.SendMessageToClient("Ld0d");
				if (useSleep)
					Sleep(50);
			}
			//////////////////////////////////////////////////////////////////////
			//m.lock();
			isVision = true;
			isHYUPlanning = false;
			isRobotState = false;
			isWaypoint = false;
			//m.unlock();
			free(copy);
		}
		else if (hyu_data_flag == 'G') {

			//char* send_data;
			//send_data = getSimulationState(objects);
			//serv.SendMessageToClient(send_data);	
			printf(hyu_data);
			serv.SendMessageToClient(hyu_data);
			if (useSleep)
				Sleep(50);
		}
		else if (hyu_data_flag == 'R')
		{
			// Robot cur data
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			if (useSleep)
				Sleep(50);
			printf("%s\n", hyu_data);
			int robotFlag = 0;
			robotFlag = readRobotCurState(hyu_data, robot_state);

			if (robotFlag == 1)
			{
				//rManager1->setJointVal(robot_state.robot_joint);

				serv.SendMessageToClient("T1");
				if (useSleep)
					Sleep(50);
			}
			else if (robotFlag == 2)
			{
				//rManager2->setJointVal(robot_state.robot_joint);
				//char pbuffer[100];
				//char tmp_buffer[255];
				//int digit_num = 5;
				//SE3 robot2StateFromRobot1;
				//for (int i = 0; i < 9; i++)
				//	robot2StateFromRobot1[i] = robot_state.robot_rot[i];
				//for (int i = 0; i < 3; i++)
				//	robot2StateFromRobot1[9 + i] = robot_state.robot_pos[i];
				//robot2StateFromRobot1 = Trobotbase1 % Trobotbase2 * robot2StateFromRobot1;

				//char *temp_data = strtok(copy, "d");

				//char* copy2 = (char*)malloc(sizeof(char) * 30000);
				//memset(copy2, NULL, sizeof(char) * 30000);
				//strcat(copy2, temp_data);
				//strcat(copy2, "d");
				//for (int i = 0; i < 12; i++)
				//{
				//	temp_data = strtok(NULL, "d");
				//	strcpy(pbuffer, "");
				//	strcat(pbuffer, _gcvt(robot2StateFromRobot1[i], digit_num, tmp_buffer));
				//	strcat(copy2, pbuffer);
				//	strcat(copy2, "d");
				//}
				//while (temp_data != NULL)
				//{
				//	temp_data = strtok(NULL, "d");
				//	strcat(copy2, temp_data);
				//	strcat(copy2, "d");
				//}
				//serv.SendMessageToClient(copy2);
				//if (useSleep)
				//	Sleep(50);


				serv.SendMessageToClient("T2");
				if (useSleep)
					Sleep(50);
			}
			else
				printf("Wrong robot flag is given (Flag = 'R')!!!!!!\n");

			//cout << robot_state.robot_joint.transpose() << endl;

			//m.lock();
			//isVision = false;
			//isHYUPlanning = false;
			//isRobotState = true;
			//m.unlock();
			free(copy);
		}
		else if (hyu_data_flag == 'S') {
			char* copy = (char*)malloc(sizeof(char)*(strlen(hyu_data) + 1));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];

			pair<int,vector<int>> hyu_data_output = readRobotCommand(hyu_data, hyu_desired_dataset);
			if (hyu_data_output.first == 1 || hyu_data_output.first == 2)
			{
				if (hyu_data_output.second[0] == 1 || hyu_data_output.second[0] == 2)
				{
					// send to robot
					serv.SendMessageToClient(copy);
					if (useSleep)
						Sleep(50);
					printf(copy);
					printf("\n");
					if (hyu_data_output.second[0] == 1) // when gripper input comes
						gripObjectIdx[hyu_data_output.first - 1] = getObjectIdx(hyu_data_output.first);
				}
				else
				{
					char temp_char[3];
					sprintf(temp_char, "P%d", hyu_data_output.first);
					serv.SendMessageToClient(temp_char);
					if (useSleep)
						Sleep(50);
				}
			}
			else if (hyu_data_output.first == 3)
			{
				int nway1;
				int nway;
				int iter = 0;
				int n_inside_way = 19;
				char plus[10];
				char nway_char[10];
				int disregardNum = 3;
				if (hyu_data_output.second[0] == 0)
				{
					serv.SendMessageToClient("P1");
					Sleep(50);
				}
				else
				{
					char* tmp_Data1 = (char*)malloc(sizeof(char)*30000);
					memset(tmp_Data1, NULL, sizeof(char)*30000);
					strcat(tmp_Data1, "S");
					sprintf(plus, "%dd", 1);
					strcat(tmp_Data1, plus);
					strcpy(plus, "");
					strcpy(nway_char, "");

					//char tmp_Data1[30000] = "S";
					//sprintf(plus, "%dd", 1);
					//strcat(tmp_Data1, plus);
					//strcpy(plus, "");
					//strcpy(nway_char, "");


					for (unsigned int p = 0; p <= strlen(copy); p++)
					{
						if (iter != 0 && iter != 2)
						{
							sprintf(plus, "%c", copy[p]);
							strcat(tmp_Data1, plus);
							if (iter == 1 && copy[p] != 'd')
								strcat(nway_char, plus);
							if (iter == 1 && copy[p] == 'd')
								nway = atoi(nway_char);
						}
						if (copy[p] == 'd')
							iter++;
						if (iter == nway*n_inside_way + 1 + disregardNum)
							break;
					}
					char* send_data1 = (char*)malloc(sizeof(char)*(strlen(tmp_Data1) + 1));
					strcpy(send_data1, tmp_Data1);

					serv.SendMessageToClient(send_data1);
					Sleep(50);
					printf(send_data1);
					printf("\n");
					free(send_data1);
					free(tmp_Data1);
					if (hyu_data_output.second[0] == 1) // when gripper input comes
						gripObjectIdx[0] = getObjectIdx(1);
				}

				if (hyu_data_output.second[1] == 0)
				{
					serv.SendMessageToClient("P2");
					Sleep(50);
				}
					
				else
				{

					char* tmp_Data2 = (char*)malloc(sizeof(char)*30000);
					memset(tmp_Data2, NULL, sizeof(char)*30000);
					strcat(tmp_Data2, "S");
					sprintf(plus, "%dd", 2);
					strcat(tmp_Data2, plus);
					strcpy(plus, "");
					strcpy(nway_char, "");

					//char tmp_Data2[30000] = "S";
					//sprintf(plus, "%dd", 2);
					//strcat(tmp_Data2, plus);
					//strcpy(plus, "");
					//strcpy(nway_char, "");
					iter = 0;
					for (unsigned int p = 0; p <= strlen(copy); p++)
					{
						if (iter == 1 || iter == 2)
							sprintf(plus, "%c", copy[p]);
						if (iter == 1 && copy[p] != 'd')
							strcat(nway_char, plus);
						if (iter == 1 && copy[p] == 'd')
						{
							nway1 = atoi(nway_char);
							strcpy(nway_char, "");
						}
						if (iter == 2 && copy[p] != 'd')
							strcat(nway_char, plus);
						if (iter == 2 && copy[p] == 'd')
							nway = atoi(nway_char);

						if (iter == 2 || iter >= nway1*n_inside_way + 1 + disregardNum)
						{
							sprintf(plus, "%c", copy[p]);
							strcat(tmp_Data2, plus);
						}
						if (copy[p] == 'd')
							iter++;
						if (iter == (nway+nway1)*n_inside_way + 2 + disregardNum || copy[p] == '\0')
							break;
					}
					char* send_data2 = (char*)malloc(sizeof(char)*(strlen(tmp_Data2)+1));
					strcpy(send_data2, tmp_Data2);

					serv.SendMessageToClient(send_data2);
					Sleep(50);
					printf(send_data2);
					printf("\n");
					free(send_data2);
					free(tmp_Data2);
					if (hyu_data_output.second[1] == 1) // when gripper input comes
						gripObjectIdx[1] = getObjectIdx(2);
				}
			}
			free(copy);
		}
		else if (hyu_data_flag == 'A') 
		{
			isHYUPlanning = false;
			isVision = true;
			isRobotState = false;
			isWaypoint = false;
			char temp_char[2];
			sprintf(temp_char, "%c", hyu_data[1]);
			int robotFlag = atoi(temp_char);
			sprintf(temp_char, "%c", hyu_data[2]);
			int objectMaintainFlag = atoi(temp_char);
			if (objectMaintainFlag == 0)
			{
				for (unsigned int i = 0; i < objects.size(); i++)
				{
					objects[i]->setBaseLinkFrame(TobjectsInitSimul[i]);
					objects[i]->KIN_UpdateFrame_All_The_Entity();
				}
				for (unsigned int i = 0; i < gripObjectIdx.size(); i++)
				{
					if (gripObjectIdx[i] != -1)
						TlastObjects_multi[i] = TobjectsInitSimul[gripObjectIdx[i]];
					gripObjectIdx[i] = -1;
				}
					
				initialPlanning[robotFlag - 1] = true;

			}
			else
				startPlanningFromCurRobotState[robotFlag - 1] = true;

			// to see values
			gripObjectIdx;
			TlastObjects_multi;
			TinitObjects_multi;
			int stop = 1;
		}

		else if (hyu_data_flag == 'P') {
			isHYUPlanning = false;		// to turn off rendering
			vector<bool> attachObject(0);		// grip status from received data
			vector<bool> waypointFlag(0);
			vector<double> stepsize(0);
			vector<bool> attachobject(0);		// grip status with disregarding unavailable points
			int robotFlag = 0;
			robotFlag = readRobotCurState(hyu_data, robot_state);

			if (robotFlag == 1 || robotFlag == 2)
			{
				// send confirming message to robot
				char temp_char[3];
				sprintf(temp_char, "T%d", robotFlag);
				serv.SendMessageToClient(temp_char);
				if (useSleep)
					Sleep(50);

				// RRT problem setting
				// decide initial point (read from robot for initial planning, use last joint val otherwise)
				Eigen::VectorXd planningInit;
				if (initialPlanning[robotFlag - 1])
					planningInit = robot_state.robot_joint;
				else
				{
					if (startPlanningFromCurRobotState[robotFlag - 1])
						planningInit = robot_state.robot_joint;
					else
						planningInit = lastJointVal_multi[robotFlag - 1];
					startPlanningFromCurRobotState[robotFlag - 1] = false;
					if (gripObjectIdx[robotFlag - 1] != -1 && distSE3(TinitObjects_multi[robotFlag - 1], TlastObjects_multi[robotFlag - 1]) > 1.0e-5)		// move busbar to its last location when releasing
					{
						objects[gripObjectIdx[robotFlag - 1]]->setBaseLinkFrame(TlastObjects_multi[robotFlag - 1]);
						objects[gripObjectIdx[robotFlag - 1]]->KIN_UpdateFrame_All_The_Entity();
					}
				}
					
				RRT_problemSettingFromSingleRobotCommand(hyu_desired_dataset[robotFlag-1], attachObject, planningInit, waypointFlag, robotFlag);
				for (unsigned int i = 0; i < waypointFlag.size(); i++)
				{
					if (waypointFlag[i])
					{
						stepsize.push_back(0.1);
						attachobject.push_back(attachObject[i]);
					}
				}
				attachObjectWaypoint[robotFlag - 1] = attachObject;

				// Solve RRT
				//m.lock();
				if (goalPos.size() > 0)
				{
					RRTSolve_HYU_SingleRobot(attachobject, stepsize, robotFlag);
					attachObjRender_multi[robotFlag - 1] = attachobject;
					gripObjectIdxRender[robotFlag - 1] = gripObjectIdx[robotFlag - 1];
					TinitObjects_multiRender[robotFlag - 1] = TinitObjects_multi[robotFlag - 1];
					initialObjectSavedRender[robotFlag - 1] = initialObjectSaved[robotFlag - 1];
					isHYUPlanning = true;
					isVision = false;
					isRobotState = false;
					isWaypoint = false;

					char* send_data = (char*)malloc(sizeof(char)*30000);
					memset(send_data, NULL, sizeof(char) * 30000);
					char *add = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
					strcat(send_data, add);
					delete(add);

					//char send_data[30000];
					//strcpy(send_data, "");
					//strcat(send_data, makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag));

					//char* send_data = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
					serv.SendMessageToClient(send_data);
					if (useSleep)
						Sleep(50);
					printf("%s\n", send_data);
					free(send_data);
					if (checkTorque)
					{
						vector<Eigen::VectorXd> tauTrj = calculateJointTorque(renderTraj_multi[robotFlag - 1], robotFlag);
						Eigen::VectorXd maxTau = Eigen::VectorXd::Zero(6);
						for (unsigned int i = 0; i < tauTrj.size(); i++)
						{
							maxTau = maxTau.cwiseMax(tauTrj[i]);
							printf("maximum torque: \n");
							cout << maxTau.transpose() << endl;
						}
					}
					if (attachobject.size() > 0 && attachobject[attachobject.size() - 1])
						gripState_multi[robotFlag - 1] = 1;
					else
						gripState_multi[robotFlag - 1] = 0;

					initialPlanning[robotFlag - 1] = false;
				}
				else
				{
					// send not feasible flag
					serv.SendMessageToClient("W");
					isHYUPlanning = false;
					isVision = false;
					isRobotState = false;
					isWaypoint = true;
				}
				//m.unlock();
			}
			else
				printf("Wrong robot Flag is given (Flag = 'P')!!!\n");
		}

		// _CrtDumpMemoryLeaks();
		/*hyu_data[0] = '\0';*/
		if (hyu_data[0] != '\0')
			free(hyu_data);


		hyu_data_flag = ' ';
		if (useSleep)
			Sleep(50);
	}
}

//void renderFunc()
//{
//	//renderer->setUpdateFunc(updateFunc);
//	renderer->RunRendering();
//}
//
//void updateFuncTotal()
//{
//	//updateFuncRobotState();
//	if (isVision)
//		updateFuncVision();
//	else if (isHYUPlanning)
//		updateFuncPlanning_multi();
//	else if (isRobotState)
//		updateFuncRobotState();
//	else
//		updateFuncWaypoint();
//	//static int updateFuncCnt = 0;
//	//printf("update func cnt: %d\n", updateFuncCnt++);
//}
//
void updateFuncPlanning_multi()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	unsigned int num_robot = renderTraj_multi.size();
	static vector<int> trjIdx(num_robot, 0);
	static vector<int> taskIdx(num_robot, 0);

	// variable for restart rendering when planning result is updated
	vector<bool> replanned(num_robot, false);
	vector<unsigned int> init(0);
	static vector<vector<unsigned int>> len_bf(num_robot, init);
	static vector<vector<unsigned int>> len_cur(num_robot, init);
	for (unsigned int j = 0; j < renderTraj_multi.size(); j++)
	{
		len_cur[j].resize(renderTraj_multi[j].size());
		if (len_cur[j].size() != len_bf[j].size())
			replanned[j] = true;
		else
		{
			for (unsigned int i = 0; i < len_cur[j].size(); i++)
			{
				len_cur[j][i] = renderTraj_multi[j][i].size();
				if (len_cur[j][i] != len_bf[j][i])
				{
					replanned[j] = true;
					break;
				}
			}
		}
		len_bf[j] = len_cur[j];
	}
			

	vector<int> idx(num_robot);
	for (unsigned int j = 0; j < num_robot; j++)
	{
		if (replanned[j])
		{
			idx[j] = 0;
			trjIdx[j] = 0;
		}
		else if (renderTraj_multi[j].size()==0)
			idx[j] = 0;
		else 
			idx[j] = taskIdx[j] % renderTraj_multi[j].size();

		if (gripObjectIdxRender[j] != -1 && idx[j] == 0 && trjIdx[j] == 0 && initialObjectSavedRender[j])
		{
			objects[gripObjectIdxRender[j]]->setBaseLinkFrame(TinitObjects_multiRender[j]);
			objects[gripObjectIdxRender[j]]->KIN_UpdateFrame_All_The_Entity();
		}
		//cout << Trobotbase1 % TinitObjects_multiRender[1] << endl;
		//cout << Trobotbase1 % TlastObjects_multi[1] << endl;
	}
	
	
	for (unsigned int i = 0; i < num_robot; i++)
	{		
		if (renderTraj_multi[i].size() == 0)
		{
			// render homepos when planning is not done
			rManagerVector[i]->setJointVal(homePosRobotVector[i]);
		}
		else if (renderTraj_multi[i].size() > 0)
		{
			// set joint val
			rManagerVector[i]->setJointVal(renderTraj_multi[i][idx[i]][trjIdx[i]]);
			//busbar movement
			if (attachObjRender_multi[i][idx[i]] && gripObjectIdxRender[i] != -1)
			{
				// to be fixed (change object type later)			gripObjectIdxRender do not consider task Idx ---- not perfect???
				objects[gripObjectIdxRender[i]]->setBaseLinkFrame(rManagerVector[i]->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tobject2gripper[gripObjectIdxRender[i]]));
				objects[gripObjectIdxRender[i]]->KIN_UpdateFrame_All_The_Entity();
			}
		}
	}

	for (unsigned int i = 0; i < num_robot; i++)
	{
		if (renderTraj_multi[i].size() > 0)
		{
			trjIdx[i]++;
			if (trjIdx[i] == renderTraj_multi[i][idx[i]].size())
			{
				trjIdx[i] = 0;

				taskIdx[i]++;
				//printf("taskIdx of robot %d: %d       ", i, idx[i]);
			}
		}		
	}
}

void connectJigToWorkCell()
{
	srWeldJoint* wJoint = new srWeldJoint;
	wJoint->SetParentLink(workCell->GetBaseLink());
	wJoint->SetChildLink(jigAssem->GetBaseLink());
	wJoint->SetParentLinkFrame(jigAssem->GetBaseLink()->GetFrame());
	wJoint->SetChildLinkFrame(SE3());
}

int getObjectIdx(int robotIdx)
{
	// robotIdx = 1: robot1, robotIdx = 2: robot2
	if (robotIdx != 1 && robotIdx != 2)
		return -1;
	double mindist = 100.0;
	unsigned int minIdx = 100;
	double tempdist = 100.0;
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		tempdist = Norm(robotVector[robotIdx - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame().GetPosition() - (objects[i]->GetBaseLink()->GetFrame()*Tobject2gripper[i]).GetPosition());
		if (tempdist < mindist)
		{
			mindist = tempdist;
			minIdx = i;
		}
	}
	if (minIdx == -1 || mindist > 0.01)
		printf("check if robot can grip object\n");
	return minIdx;
}

//void updateFuncWaypoint()
//{
//	static int cntWay = 0;
//	static int cntPlot = 0;
//	cntWay++;
//	if (cntWay % 100 == 0)
//		cntPlot++;
//	for (unsigned int i = 0; i < robotVector.size(); i++)
//	{
//		if (qWaypoint[i].size() > 0)
//		{
//			rManagerVector[i]->setJointVal(qWaypoint[i][cntPlot % qWaypoint[i].size()]);
//			if (gripObjectIdx[i] != -1 && attachObjectWaypoint[i][cntPlot % qWaypoint[i].size()])
//			{
//				objects[gripObjectIdx[i]]->GetBaseLink()->SetFrame(robotVector[i]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tobject2gripper[gripObjectIdx[i]]));
//				objects[gripObjectIdx[i]]->KIN_UpdateFrame_All_The_Entity();
//			}
//		}
//			
//	}
//}

vector<Eigen::VectorXd> calculateJointTorque(vector<vector<Eigen::VectorXd>>& traj, int robotFlag)
{
	vector<Eigen::VectorXd> qTrj(0);
	for (unsigned int i = 0; i < traj.size(); i++)
	{
		for (unsigned int j = 0; j < traj[i].size(); j++)
		{
			if (qTrj.size() > 0 && (traj[i][j] - qTrj[qTrj.size() - 1]).norm() > 1.0e-5)
				qTrj.push_back(traj[i][j]);
		}
	}
	vector<double> dt(qTrj.size());
	vector<Eigen::VectorXd> vTrj(qTrj.size());
	vector<Eigen::VectorXd> aTrj(qTrj.size());
	for (unsigned int i = 0; i < qTrj.size() - 1; i++)
	{
		dt[i] = (qTrj[i + 1] - qTrj[i]).cwiseAbs().sum() * 10.0;
		vTrj[i] = (qTrj[i + 1] - qTrj[i]) / dt[i];
	}
	for (unsigned int i = 0; i < qTrj.size() - 1; i++)
		aTrj[i] = (vTrj[i + 1] - vTrj[i]) / dt[i];
	vector<Eigen::VectorXd> tauTrj(qTrj.size() - 1);
	for (unsigned int i = 0; i < qTrj.size() - 1; i++)
		tauTrj[i] = rManagerVector[robotFlag - 1]->inverseDyn(qTrj[i], vTrj[i], aTrj[i]);
	return tauTrj;
}

void planning_demo(int robotFlag, int objNum)
{
	isHYUPlanning = false;
	if (robotFlag < 1)
		robotFlag = 1;
	else if (robotFlag > 2)
		robotFlag = 2;
	//cout << jigAssem->getBaseLinkFrame() << endl;
	int busbarNum;
	int holeNum;
	if (robotFlag == 1)
	{
		busbarNum = objNum;
		holeNum = objNum + 4;
	}
	else
	{
		busbarNum = objNum + 4;
		holeNum = objNum;
	}
	Eigen::VectorXd qInit;
	vector<SE3> wayPoints(0);
	vector<bool> attachObject(0);
	int flag;
	if (objNum == 0)
	{
		qInit = robotVector[robotFlag - 1]->homePos;
	}
	else
	{
		wayPoints.push_back(TrobotbaseVector[0] % jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[holeNum - 1] * Thole2busbar * Tbusbar2gripper_new);
		attachObject.push_back(false);
		SE3 Tprevgoal = SE3(Vec3(0.0, 0.0, -0.025)) * jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[holeNum - 1] * Thole2busbar;
		//cout << "prev goal" << endl << Tprevgoal << endl;
		qInit = rManagerVector[robotFlag - 1]->inverseKin(Tprevgoal * Tbusbar2gripper_new, &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, robotVector[robotFlag - 1]->qInvKinInit);
		int holenum_i;
		for (int i = (robotFlag - 1) * 4; i < (robotFlag - 1) * 4 + objNum; i++)
		{
			if (robotFlag == 1)
				holenum_i = i + 4;
			else
				holenum_i = i - 4;
			busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.025)) * jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[holenum_i] * Thole2busbar);
		}
	}
	wayPoints.push_back(TrobotbaseVector[0] % SE3(Vec3(0.0, 0.0, 0.025)) * objects[busbarNum]->getBaseLinkFrame() * Tbusbar2gripper_new);
	attachObject.push_back(false);
	wayPoints.push_back(TrobotbaseVector[0] % objects[busbarNum]->getBaseLinkFrame() * Tbusbar2gripper_new);
	attachObject.push_back(false);
	wayPoints.push_back(TrobotbaseVector[0] % jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar * Tbusbar2gripper_new);
	attachObject.push_back(true);
	wayPoints.push_back(TrobotbaseVector[0] % SE3(Vec3(0.0, 0.0, -0.025)) * jigAssem->getBaseLinkFrame() * jigAssem->holeCenter[holeNum] * Thole2busbar * Tbusbar2gripper_new);
	attachObject.push_back(true);
	//cout << jigAssem->getBaseLinkFrame() << endl;

	//cout << wayPoints[1] << endl;
	//cout << wayPoints[0] << endl;
	//cout << wayPoints[2] << endl;
	//cout << wayPoints[3] << endl;
	
	vector<bool> includeOri(wayPoints.size(), true);
	attachObject[0] = false; attachObject[1] = false;
	vector<bool> waypointFlag(wayPoints.size());
	gripObjectIdx[robotFlag - 1] = busbarNum;
	RRT_problemSetting_SingleRobot(qInit, wayPoints, includeOri, attachObject, waypointFlag, robotFlag);
	vector<double> stepsize(0);
	vector<bool> attachobject(0);
	for (unsigned int i = 0; i < waypointFlag.size(); i++)
	{
		if (waypointFlag[i])
		{
			stepsize.push_back(0.1);
			attachobject.push_back(attachObject[i]);
		}
	}
	attachObjectWaypoint[robotFlag - 1] = attachObject;

	/////////////////////////////////////////////
	//qval2 = goalPos[goalPos.size() - 1];
	//rManagerVector[robotFlag - 1]->setJointVal(qval2);
	//busbar[busbarNum]->setBaseLinkFrame(robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_new));
	/////////////////////////////////////////////
	// Solve RRT
	if (goalPos.size() > 0)
	{
		RRTSolve_HYU_SingleRobot(attachobject, stepsize, robotFlag);
		attachObjRender_multi[robotFlag - 1] = attachobject;
		gripObjectIdxRender[robotFlag - 1] = gripObjectIdx[robotFlag - 1];
		TinitObjects_multiRender[robotFlag - 1] = TinitObjects_multi[robotFlag - 1];
		initialObjectSavedRender[robotFlag - 1] = initialObjectSaved[robotFlag - 1];
		isHYUPlanning = true;
		isVision = false;
		isRobotState = false;
		isWaypoint = false;

		if (attachobject.size() > 0 && attachobject[attachobject.size() - 1])
			gripState_multi[robotFlag - 1] = 1;
		else
			gripState_multi[robotFlag - 1] = 0;

		initialPlanning[robotFlag - 1] = false;
	 }
}

void savePlannedResultToText(unsigned int robotFlag, vector<vector<Eigen::VectorXd>>& traj, vector<vector<bool>>& attachObjectVec)
{
	// attachObject: first vector - robotnum, second vector - tasknum
	// save joint trajectory and object trajectories to text
	vector<Eigen::VectorXd> saveTraj(0);
	string loc = "../../../data/render_traj/";
	if (robotFlag == 1 || robotFlag == 2)
	{
		// single robot case
		saveTraj.resize(0);
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
			{
				saveTraj.push_back(traj[i][j]);
			}
		}
		saveDataToText(saveTraj, loc + "jointVal" + to_string(robotFlag) + ".txt");
		for (unsigned int k = 0; k < objects.size(); k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					if (attachObjectVec[0][i] && k == gripObjectIdx[robotFlag - 1])
					{
						rManagerVector[robotFlag - 1]->setJointVal(traj[i][j]);
						saveTraj.push_back(SE3toVector(robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tobject2gripper[k])));
					}
					else
						saveTraj.push_back(SE3toVector(objects[k]->GetBaseLink()->GetFrame()));
				}
			}
			saveDataToText(saveTraj, loc + "object" + to_string(k) + ".txt");
		}
	}
	else
	{
		// multi robot case
		for (int k = 0; k < 1; k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					saveTraj.push_back(traj[i][j].segment(6*k, 6));
				}
			}
			saveDataToText(saveTraj, loc + "jointVal" + to_string(k + 1) + ".txt");
		}
		
		for (unsigned int k = 0; k < objects.size(); k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				int robotnum = -1;
				for (int r = 1; r < 3; r++)
				{
					if (attachObjectVec[r - 1][i] && k == gripObjectIdx[r - 1])
					{
						robotnum = r;
						break;
					}
				}
				
				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					if (robotnum != -1)
					{
						rManagerVector[robotnum - 1]->setJointVal(traj[i][j]);
						saveTraj.push_back(SE3toVector(robotVector[robotnum - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tobject2gripper[k])));
					}
					else
						saveTraj.push_back(SE3toVector(objects[k]->GetBaseLink()->GetFrame()));
				}
			}
			saveDataToText(saveTraj, loc + "object" + to_string(k) + ".txt");
		}
	}
}
