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
#include <thread>

#include <mutex>

#include "../VS2013/tcp_ip_server/stdafx.h"
#include <Winsock2.h>
#include <stdlib.h>
#include <stdio.h>
#include "../VS2013/tcp_ip_server/Server.h"
#include "tcp_ip_communication.h"
#include "Eigen/Dense"
//static int sendValue;
//static char sendBuf[BUFSIZE];

#include <stdlib.h>
#include <vector>

static mutex m;

// Environment
JigAssem_QB_bar* jigAssem = new JigAssem_QB_bar(false);
vector<BusBar_HYU*> busbar(8);
bool isJigConnectedToWorkCell = true;


// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
vector<IndyRobot*> robotVector(2);


srSpace gSpace;
serverRenderer* renderer;
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
SE3 Trobotbase1;
SE3 Trobotbase2;
vector<SE3> TrobotbaseVector(2);
// Planning
vector<vector<vector<Eigen::VectorXd>>> renderTraj_multi(2);
vector<vector<SE3>>	Ttraj(0);

vector<vector<int>> idxTraj(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);



vector<SE3> wayPoints(0);

Eigen::VectorXd homePosRobot1 = Eigen::VectorXd::Zero(6);
Eigen::VectorXd homePosRobot2 = Eigen::VectorXd::Zero(6);
Eigen::VectorXd lastRobot1Joint;
Eigen::VectorXd lastRobot2Joint;

SE3 initBusbar = SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.5)));




indyRobotManager* rManager1;
indyRobotManager* rManager2;
vector<indyRobotManager*> rManagerVector(2);
robotRRTManager* RRTManager1 = new robotRRTManager;
robotRRTManager* RRTManager2 = new robotRRTManager;
vector<robotRRTManager*> RRTManagerVector(2);

srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
void initDynamics();
// rendering
void rendering(int argc, char **argv);
void updateFunc();

void updateFuncVision();
void updateFuncPlanning_multi();
void updateFuncRobotState();

void updateFuncTotal();

void environmentSetting_HYU2(bool connect);

// communication function
void setEnviromentFromVision(const vision_data& skku_dataset);
void connectJigToWorkCell();
void setRobotFromRealRobot(const robot_current_data& robot_state);
char* getSimulationState(vector<srSystem*> objects);

// RRT multi-robot
void RRT_problemSettingFromSingleRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag, int robotFlag);
void RRT_problemSetting_SingleRobot(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag, int robotFlag);
void RRTSolve_HYU_SingleRobot(vector<bool> attachObject, vector<double> stepsize, int robotFlag);

void workspaceSetting();
void robotSetting();
void robotManagerSetting();

void rrtSetting();

int getBusbarIdx(int robotIdx);

void communicationFunc(int argc, char **argv);

void renderFunc();



bool renderNow;

// Rendering flags
static bool isVision = false;
static bool isHYUPlanning = false;
static bool isRobotState = true;


vector<int> flags(0);

double planning = 0;
// save last gripper state
int gripState = 0;
vector<int> gripState_multi(2);
vector<int> gripBusbarIdx(2, -1);			// save which busbar is moving with each robot during planning
vector<int> gripBusbarIdxRender(2, -1);		// save which busbar is moving with each robot during rendering
											// save last joint value
vector<Eigen::VectorXd> lastJointVal_multi(2);
vector<bool> initialPlanning(2, true);
vector<bool> attachObjRender(0);
vector<vector<bool>> attachObjRender_multi(2);
vector<Eigen::VectorXd> homePosRobotVector(2);
// 서버 초기화
Server serv = Server::Server();
dataset hyu_dataset;
vector<desired_dataset> hyu_desired_dataset;
vision_data skku_dataset;
robot_current_data robot_state;


char *hyu_data;
char *copy;
char *pbuffer;



char hyu_data_flag;
char tmp_buffer[255];
char divChar = 'd';

int digit_num = 5;
int nway = 0;

vector<srLink*> obstacle(0);
vector<srWeldJoint*> wJoint(0);

// result visualization 관련
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);
// save data
vector<vector<vector<Eigen::VectorXd>>> FTtraj(2);
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);
bool saveTraj = true;

int main(int argc, char **argv)
{
	bool useVision = false;
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		gSpace.AddSystem(busbar[i]);
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
	//busbar[4]->GetBaseLink()->GetGeomInfo().SetColor(0.0, 0.0, 1.0);

	srand(time(NULL));
	// Robot home position
	homePosRobot1[1] = -SR_PI_HALF; homePosRobot1[3] = SR_PI_HALF; homePosRobot1[4] = -0.5 * SR_PI;
	homePosRobot2[1] = -SR_PI_HALF; homePosRobot2[3] = SR_PI_HALF; homePosRobot2[4] = -0.5 * SR_PI;
	homePosRobotVector[0] = homePosRobot1;
	homePosRobotVector[1] = homePosRobot2;
	// environment
	workspaceSetting();

	robotSetting();

	//initBusbar = SE3(Vec3(0.0, -0.4, 0.05)) * jigAssem->GetBaseLink()->GetFrame();
	//busbar[4]->GetBaseLink()->SetFrame(Trobotbase1 * SE3(1.0000, - 0.0074,         0, - 0.0074, - 1.0000,         0,         0,         0, - 1.0000, 0,		0,		0.5740));


	////////////////////////////////////////////// setting environment (replacable from vision data)
	if (!useVision)
	{
		environmentSetting_HYU2(true);			// temporary environment setting
		initDynamics();								// initialize srLib


		robotManagerSetting();						// robot manager setting

													// workcell robot initial config
													//homePosRobot2.setZero();
													//homePosRobot2[0] = 0.0; homePosRobot2[1] = -SR_PI_HALF; homePosRobot2[2] = 80.0 / 90.0*SR_PI_HALF; homePosRobot2[3] = SR_PI_HALF;
		rManager1->setJointVal(homePosRobot1);
		rManager2->setJointVal(homePosRobot2);
		Eigen::VectorXd gripInput(2);
		gripInput[0] = -0.009;
		gripInput[1] = 0.009;
		rManager1->setGripperPosition(gripInput);
		rManager2->setGripperPosition(gripInput);

		// rrt
		rrtSetting();

		cout << Trobotbase1 % jigAssem->GetBaseLink()->GetFrame() << endl;
		cout << Trobotbase1 % busbar[0]->GetBaseLink()->GetFrame() << endl;
		cout << busbar[4]->GetBaseLink()->GetFrame() << endl;
	}





	thread commuThread(communicationFunc, argc, argv);
	commuThread.detach();

	////m.lock();

	////thread rendThread(rendering);

	////thread rendThread(renderFunc);
	////rendThread.detach();
	if (commuThread.joinable())
		commuThread.join();
	//if (rendThread.joinable())
	//	rendThread.join();


	if (useVision)
	{
		renderer = new serverRenderer();
		SceneGraphRenderer::NUM_WINDOWS windows;
		windows = SceneGraphRenderer::SINGLE_WINDOWS;
		renderer->InitializeRenderer(argc, argv, windows, false);
		renderer->InitializeNode_1st(&gSpace);
		while (1)
		{
			if (isVision)
				break;
		}
		// generate renderer
		renderer->InitializeNode_2nd();
		renderer->setUpdateFunc(updateFuncTotal);
		static int renderCall = 0;
		renderCall++;
		printf("rendering called: %d\n", renderCall);
		renderer->RunRendering();
	}
	else
	{
		rendering(argc, argv);
	}

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new serverRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode_1st(&gSpace);
	renderer->InitializeNode_2nd();
	renderer->setUpdateFunc(updateFuncTotal);
	//renderer->setUpdateFunc(updateFuncVision);
	static int renderCall = 0;
	renderCall++;
	printf("rendering called: %d\n", renderCall);
	renderer->RunRendering();
}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	rManager2->setJointVal(homePosRobot2);
	rManager1->setJointVal(homePosRobot1);

}

void updateFuncVision()
{
	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	// robot to homePos
	rManager2->setJointVal(homePosRobot2);
	rManager1->setJointVal(homePosRobot1);
	//printf("Vision Update func called\n");
	//cout << Trobotbase1 % jigAssem->GetBaseLink()->GetFrame() << endl;
	//cout << Trobotbase1 % busbar[0]->GetBaseLink()->GetFrame() << endl;
	int stop = 0;
}

void updateFuncRobotState()
{
	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	// robot to homePos
	//rManager2->setJointVal(homePosRobot1);
	//rManager1->setJointVal(homePosRobot1);
	static int updateFuncCall = 0;
	updateFuncCall++;
	//printf("updateFunc called: %d\n", updateFuncCall);
}
void environmentSetting_HYU2(bool connect)
{
	//SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	SE3 Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
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
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->GetBaseLink()); // removed stage
														//wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(jigAssem->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase*Tbase2jigbase);
		wJoint->SetChildLinkFrame(SE3());
	}
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
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	// elbow up
	qInit[1] = -0.65*SR_PI;
	qInit[2] = 0.3*SR_PI;
	qInit[3] = 0.5*SR_PI_HALF;
	// elbow down
	//qInit[2] = 0.6*SR_PI_HALF;
	//qInit[4] = 0.6*SR_PI_HALF;
	//qInit[3] = SR_PI_HALF;

	Eigen::VectorXd qInit2 = Eigen::VectorXd::Zero(6);
	qInit2[0] = -0.224778; qInit2[1] = -1.91949; qInit2[2] = -0.384219; qInit2[3] = 1.5708; qInit2[4] = -0.73291; qInit2[5] = 1.79557;

	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	bool feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(init);
	if (feas != 0)
		printf("initial point not feasible!!!\n");
	Eigen::VectorXd qtemp;
	waypointFlag.resize(wayPoints.size());
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit2);
		if (flag != 0)
			qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
		if (flag != 0)
			qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[initPos.size() - 1]);
		printf("%d-th init inv kin flag: %d\n", i, flag);

		if (attachObject[i] && gripBusbarIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(busbar[gripBusbarIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();


		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(qtemp);

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
	vector<vector<Eigen::VectorXd>> traj (0);
	//Ttraj.resize(0);
	//idxTraj.resize(0);
	printf("waypoints: \n");
	for (unsigned int i = 0; i < initPos.size(); i++)
		cout << initPos[i].transpose() << endl;
	cout << goalPos[goalPos.size() - 1].transpose() << endl;
	for (int i = start; i < end; i++)
	{
		RRTManagerVector[robotFlag - 1]->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		if (attachObject[i] && gripBusbarIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(busbar[gripBusbarIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();


		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManagerVector[robotFlag - 1]->execute(stepsize[i]);
		tempTraj = RRTManagerVector[robotFlag - 1]->extractPath();


		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManagerVector[robotFlag - 1]->setState(tempTraj[j]))
				printf("collide at %d-th trj, %d-th point!!!\n", i, j);

		traj.push_back(tempTraj);


		//tempTtraj.resize(tempTraj.size());
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
	renderTraj_multi[robotFlag - 1] = traj;
	// save last joint val
	lastJointVal_multi[robotFlag - 1] = traj[traj.size() - 1][traj[traj.size() - 1].size() - 1];
}




void setEnviromentFromVision(const vision_data & skku_dataset)
{
	// set object (id:1 - busbar, id:2 - jig),   objects start from 0, objPos start from 1 
	int bIdx = 0;
	for (unsigned int i = 0; i < skku_dataset.objID.size(); i++)
	{
		if (skku_dataset.objID[i] == 1)
		{
			busbar[bIdx]->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * busbar[bIdx]->m_visionOffset);
			busbar[bIdx]->SetBaseLinkType(srSystem::FIXED);
			bIdx++;
		}
		else if (skku_dataset.objID[i] == 2)
		{
			jigAssem->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * jigAssem->m_visionOffset);
			jigAssem->SetBaseLinkType(srSystem::FIXED);
		}
		else
			printf("object ID is outside range!!!\n");
	}

	// set obstacle
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
		//recv_data = serv.RecevData();
		hyu_data = serv.RecevData();


		hyu_data_flag = hyu_data[0];

		printf(&hyu_data_flag);
		//cout << endl;
		//serv.SendMessageToClient("G");

		// 데이터 전송
		if (hyu_data_flag == 'I')
		{
			serv.SendMessageToClient("I");
		}
		else if (hyu_data_flag == 'D')
			serv.SendMessageToClient("D");
		else if (hyu_data_flag == 'F')
			serv.SendMessageToClient("F");

		else if (hyu_data_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			printf("%s\n", hyu_data);
			readSKKUvision(hyu_data, skku_dataset);

			/*	cout << skku_dataset.obsInfo[0][0] << endl;
			cout << skku_dataset.objOri[0][0] << endl;
			cout << skku_dataset.objPos[0][0] << endl;
			*/


			setEnviromentFromVision(skku_dataset);		// should be called later than robotSetting


			if (isJigConnectedToWorkCell)
				connectJigToWorkCell();

			/////////////////////////////////////// after setting environment
			initDynamics();								// initialize srLib
			robotManagerSetting();						// robot manager setting

														// workcell robot initial config
			rManager2->setJointVal(homePosRobot2);
			rManager1->setJointVal(homePosRobot1);
			Eigen::VectorXd gripInput(2);
			gripInput[0] = -0.009;
			gripInput[1] = 0.009;
			rManager1->setGripperPosition(gripInput);
			rManager2->setGripperPosition(gripInput);

			// rrt
			rrtSetting();
			//////////////////////////////////////////////////////////////////////
			//rendering(argc, argv);
			m.lock();
			isVision = true;
			isHYUPlanning = false;
			isRobotState = false;
			m.unlock();
		}
		else if (hyu_data_flag == 'G') {

			//char* send_data;
			//send_data = getSimulationState(objects);
			//serv.SendMessageToClient(send_data);	
			serv.SendMessageToClient(hyu_data);
		}
		else if (hyu_data_flag == 'R')
		{
			// Robot cur data
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			Sleep(100);
			printf("%s\n", hyu_data);
			int robotFlag = 0;
			robotFlag = readRobotCurState(hyu_data, robot_state);

			if (robotFlag == 1)
			{
				rManager1->setJointVal(robot_state.robot_joint);
				serv.SendMessageToClient("T1");
			}
			else if (robotFlag == 2)
			{
				rManager2->setJointVal(robot_state.robot_joint);
				serv.SendMessageToClient("T2");
			}


			else
				printf("Wrong robot flag is given (Flag = 'R')!!!!!!\n");
			//else if (robotFlag == 3)
			//{
			//	Eigen::VectorXd robot1Joint(6), robot2Joint(6);
			//	for (int i = 0; i < 6; i++)
			//	{
			//		robot1Joint[i] = robot_state.robot_joint[i];
			//		robot2Joint[i] = robot_state.robot_joint[i + 6];
			//	}
			//	rManager1->setJointVal(robot1Joint);
			//	rManager2->setJointVal(robot2Joint);
			//}

			//cout << robot_state.robot_joint.transpose() << endl;

			//rendering(argc, argv);
			m.lock();
			isVision = false;
			isHYUPlanning = false;
			isRobotState = true;
			m.unlock();
		}
		else if (hyu_data_flag == 'S') {
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (unsigned int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];

			pair<int, vector<int>> hyu_data_output = readRobotCommand(hyu_data, hyu_desired_dataset);
			if (hyu_data_output.first == 1 || hyu_data_output.first == 2)
			{
				if (hyu_data_output.second[0] == 1 || hyu_data_output.second[0] == 2)
				{
					// send to robot
					serv.SendMessageToClient(copy);

					if (hyu_data_output.second[0] == 1) // when gripper input comes
						gripBusbarIdx[hyu_data_output.first - 1] = getBusbarIdx(hyu_data_output.first);
				}
				else
				{
					string tempP = "P" + to_string(hyu_data_output.first);
					char* tempPchar = (char*)malloc(sizeof(char) * 2);
					tempPchar = strcpy(tempPchar, tempP.c_str());
					serv.SendMessageToClient(tempPchar);
				}
			}
			else if (hyu_data_output.first == 3)
			{
				char* copy2 = (char*)malloc(sizeof(char)*strlen(copy));
				for (unsigned int p = 0; p <= strlen(copy); p++)
					copy2[p] = copy[p];
				char div = 'd';
				char* div_data;
				int nway1;
				int nway;
				int iter = 0;
				int n_inside_way = 19;
				if (hyu_data_output.second[0] == 0)
				{
					serv.SendMessageToClient("P1");
					Sleep(50);
				}
				else
				{
					string tmp_data1 = "S" + to_string(1) + "d";
					div_data = strtok(copy, "d");		// disregard
					nway = atoi(strtok(NULL, "d"));
					tmp_data1 = tmp_data1 + to_string(nway) + "d";
					div_data = strtok(NULL, "d");		// disregard
					iter = 0;
					while (iter < nway * n_inside_way)
					{
						div_data = strtok(NULL, "d");
						tmp_data1 = tmp_data1 + div_data + "d";
						iter++;
						if (iter == nway*n_inside_way)
						{
							div_data = strtok(NULL, "d");
							tmp_data1 = tmp_data1 + div_data + "d";
						}
					}
					//printf("\n\n\n");
					//printf(tmp_data1.c_str());
					//char* send_data1 = (char*)malloc(sizeof(char)*strlen(hyu_data));
					char* send_data1 = (char*)malloc(sizeof(char)*strlen(tmp_data1.c_str()));
					strcpy(send_data1, tmp_data1.c_str());
					serv.SendMessageToClient(send_data1);
					if (hyu_data_output.second[0] == 1) // when gripper input comes
						gripBusbarIdx[0] = getBusbarIdx(1);
				}

				if (hyu_data_output.second[1] == 0)
				{
					serv.SendMessageToClient("P2");
					Sleep(50);
				}

				else
				{
					string tmp_data2 = "S" + to_string(2) + "d";

					div_data = strtok(copy2, "d");		// disregard
					nway1 = atoi(strtok(NULL, "d"));		// disregard
					nway = atoi(strtok(NULL, "d"));
					tmp_data2 = tmp_data2 + to_string(nway) + "d";
					iter = 0;
					while (iter < (nway + nway1) * n_inside_way + 2)
					{
						div_data = strtok(NULL, "d");
						if (iter > nway1 * n_inside_way)
							tmp_data2 = tmp_data2 + div_data + "d";
						iter++;
						//printf("\n\n\n");
						//printf(tmp_data2.c_str());
						//if (iter == (nway + nway1) * n_inside_way+1)
						//{
						//	div_data = strtok(NULL, "d");
						//	tmp_data2 = tmp_data2 + div_data + "d";
						//}
						//printf("\n\n\n");
						//printf(tmp_data2.c_str());
					}
					//printf("\n\n\n");
					//printf(tmp_data2.c_str());
					//char* send_data2 = (char*)malloc(sizeof(char)*strlen(hyu_data));
					char* send_data2 = (char*)malloc(sizeof(char)*strlen(tmp_data2.c_str()));
					strcpy(send_data2, tmp_data2.c_str());
					serv.SendMessageToClient(send_data2);
					if (hyu_data_output.second[1] == 1) // when gripper input comes
						gripBusbarIdx[1] = getBusbarIdx(2);
				}
			}
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
				string temp = "T";
				temp = temp + to_string(robotFlag);
				char* tempchar = (char *)malloc(sizeof(char) * 3);
				tempchar = strcpy(tempchar, temp.c_str());
				serv.SendMessageToClient(tempchar);
				Sleep(50);

				// RRT problem setting
				// decide initial point (read from robot for initial planning, use last joint val otherwise)
				Eigen::VectorXd planningInit;
				if (initialPlanning[robotFlag - 1])
					planningInit = robot_state.robot_joint;
				else
					planningInit = lastJointVal_multi[robotFlag - 1];
				RRT_problemSettingFromSingleRobotCommand(hyu_desired_dataset[robotFlag - 1], attachObject, planningInit, waypointFlag, robotFlag);
				for (unsigned int i = 0; i < waypointFlag.size(); i++)
				{
					if (waypointFlag[i])
					{
						stepsize.push_back(0.1);
						attachobject.push_back(attachObject[i]);
					}
				}
				//busbar[0]->setBaseLinkFrame(initBusbar);

				// Solve RRT
				m.lock();
				RRTSolve_HYU_SingleRobot(attachobject, stepsize, robotFlag);

				attachObjRender_multi[robotFlag - 1] = attachobject;
				gripBusbarIdxRender[robotFlag - 1] = gripBusbarIdx[robotFlag - 1];
				isHYUPlanning = true;
				isVision = false;
				isRobotState = false;
				m.unlock();
				char* send_data = makeJointCommand_SingleRobot(renderTraj_multi[robotFlag - 1], hyu_desired_dataset[robotFlag - 1], robotFlag);
				serv.SendMessageToClient(send_data);
				printf("%s\n", send_data);
				if (attachobject[attachobject.size() - 1])
					gripState_multi[robotFlag - 1] = 1;
				else
					gripState_multi[robotFlag - 1] = 0;

				initialPlanning[robotFlag - 1] = false;

				// for save
				for (int i = 0; i<2; i++)
				{
					FTtraj[i].resize(renderTraj_multi[i].size());
				}


			}
			else
				printf("Wrong robot Flag is given (Flag = 'P')!!!\n");
		}
		/*hyu_data[0] = '\0';*/
		hyu_data_flag = ' ';
		Sleep(100);
	}
}

void renderFunc()
{
	//renderer->setUpdateFunc(updateFunc);
	renderer->RunRendering();
}

void updateFuncTotal()
{
	//updateFuncRobotState();
	if (isVision)
		updateFuncVision();
	else if (isHYUPlanning)
		updateFuncPlanning_multi();
	else
		updateFuncRobotState();
	//static int updateFuncCnt = 0;
	//printf("update func cnt: %d\n", updateFuncCnt++);
}

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

	static vector<int> writeCnt(2);
	static vector<int> writetaskCnt(2);

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
			writeCnt[j] = 0;
			writetaskCnt[j] = 0;
		}
		else if (renderTraj_multi[j].size() == 0)
			idx[j] = 0;
		else
			idx[j] = taskIdx[j] % renderTraj_multi[j].size();
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
			if (renderTraj_multi[i][idx[i]].size() == 2) // if traj size is equal to 2, only joint control.
			{
				//// set joint val
				rManagerVector[i]->setJointVal(renderTraj_multi[i][idx[i]][trjIdx[i]]);
				if (attachObjRender_multi[i][idx[i]] && gripBusbarIdxRender[i] != -1)
				{
					// to be fixed (change object type later)			gripBusbarIdxRender do not consider task Idx ---- not perfect???
					busbar[gripBusbarIdxRender[i]]->setBaseLinkFrame(rManagerVector[i]->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper_new));
				}
				writeCnt[i]++;
				if (writeCnt[i] == renderTraj_multi[i][idx[i]].size())
				{
					writeCnt[i] = 0;
					writetaskCnt[i]++;
				}
			}
			else
			{
				// Calculate acc
				if (trjIdx[i] == renderTraj_multi[i][idx[i]].size() - 1)
					jointAcc = (renderTraj_multi[i][idx[i]][trjIdx[i]] - 2.0*renderTraj_multi[i][idx[i]][trjIdx[i] - 1] + renderTraj_multi[i][idx[i]][trjIdx[i] - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
				else if (trjIdx[i] == renderTraj_multi[i][idx[i]].size() - 2)
					jointAcc = (renderTraj_multi[i][idx[i]][trjIdx[i] + 1] - 2.0*renderTraj_multi[i][idx[i]][trjIdx[i]] + renderTraj_multi[i][idx[i]][trjIdx[i] - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
				else
					jointAcc = (renderTraj_multi[i][idx[i]][trjIdx[i] + 2] - 2.0*renderTraj_multi[i][idx[i]][trjIdx[i] + 1] + renderTraj_multi[i][idx[i]][trjIdx[i]]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);

				SE3 renderEndeff = rManagerVector[i]->forwardKin(renderTraj_multi[i][idx[i]][trjIdx[i]], &robotVector[i]->gMarkerLink[Indy_Index::MLINK_GRIP]);

				// Calculate init vel
				if (trjIdx[i] < renderTraj_multi[i][idx[i]].size() - 1)
				{
					jointVel = (renderTraj_multi[i][idx[i]][trjIdx[i] + 1] - renderTraj_multi[i][idx[i]][trjIdx[i]]) / gSpace.m_Timestep_dyn_fixed;
					rManagerVector[i]->setJointValVel(renderTraj_multi[i][idx[i]][trjIdx[i]], jointVel);
				}
				else
					jointVel = (renderTraj_multi[i][idx[i]][trjIdx[i]] - renderTraj_multi[i][idx[i]][trjIdx[i]]) / gSpace.m_Timestep_dyn_fixed;

				Eigen::VectorXd tau = rManagerVector[i]->inverseDyn(renderTraj_multi[i][idx[i]][trjIdx[i]], jointVel, jointAcc);
				rManagerVector[i]->controlJointTorque(tau);

				Ftsensor = rManagerVector[i]->readSensorValue();
				dse3 Fr(0.0);
				se3 g(0.0);
				se3 Vdot(0.0);
				se3 V(0.0);
				V = Vectortose3(rManagerVector[i]->getBodyJacobian(renderTraj_multi[i][idx[i]][trjIdx[i]], &robotVector[i]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
				Vdot = Vectortose3(rManagerVector[i]->getBodyJacobian(renderTraj_multi[i][idx[i]][trjIdx[i]], &robotVector[i]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointAcc +
					rManagerVector[i]->getBodyJacobianDot(renderTraj_multi[i][idx[i]][trjIdx[i]], jointVel, &robotVector[i]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper_new)) * jointVel);
				for (int i = 0; i < 3; i++)
					g[i + 3] = gSpace.m_Gravity[i];
				//busbar movement
				if (attachObjRender_multi[i][idx[i]] && gripBusbarIdxRender[i] != -1)
				{
					// to be fixed (change object type later)			gripBusbarIdxRender do not consider task Idx ---- not perfect???
					busbar[gripBusbarIdxRender[i]]->setBaseLinkFrame(rManagerVector[i]->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper_new));

					Fr = busbar[gripBusbarIdxRender[i]]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[gripBusbarIdxRender[i]]->GetBaseLink()->m_Inertia * V) - busbar[gripBusbarIdxRender[i]]->GetBaseLink()->m_Inertia*InvAd(busbar[gripBusbarIdxRender[i]]->GetBaseLink()->GetFrame(), g);
					ftsensor = dse3toVector(Ftsensor + InvdAd((rManagerVector[i]->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManagerVector[i]->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper_new), Fr));
				}
				else
					ftsensor = dse3toVector(Ftsensor);

				// save for only one loop
				if (writetaskCnt[i] < renderTraj_multi[i].size())
				{
					if (writeCnt[i] < renderTraj_multi[i][idx[i]].size())
					{
						FTtraj[i][idx[i]].push_back(ftsensor);
						writeCnt[i]++;

					}
					if (writeCnt[i] == renderTraj_multi[i][idx[i]].size())
					{
						string dir_folder = "../../../data/visualization";
						// save
						string dir_temp = dir_folder;
						saveDataToText(FTtraj[i][idx[i]], dir_temp.append("/sensorValTraj").append(to_string(idx[i] + 1)).append("_robot").append(to_string(i + 1)).append(".txt"));
						writeCnt[i] = 0;
						writetaskCnt[i]++;

						printf("robot %d's %d-th task write done!! \n", i + 1, idx[i] + 1);
					}
				}
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
				printf("taskIdx of robot %d: %d \n", i + 1, idx[i] + 1);
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

int getBusbarIdx(int robotIdx)
{
	// robotIdx = 1: robot1, robotIdx = 2: robot2
	if (robotIdx != 1 && robotIdx != 2)
		return -1;
	double mindist = 100.0;
	unsigned int minIdx = 100;
	double tempdist = 100.0;
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		tempdist = Norm(robotVector[robotIdx - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame().GetPosition() - (busbar[i]->GetBaseLink()->GetFrame()*Tbusbar2gripper_new).GetPosition());
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

