#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

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
JigAssem_QB* jigAssem = new JigAssem_QB;
vector<BusBar_HYU*> busbar(1);

vector<SE3>	initSE3(2);
vector<SE3>	goalSE3(2);

// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd homePosRobot2(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);
srSpace gSpace;
myRenderer* renderer;
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
SE3 Trobotbase;

// Planning
vector<vector<Eigen::VectorXd>> renderTraj(0);
vector<vector<SE3>>	Ttraj(0);

vector<vector<int>> idxTraj(0);
vector<vector<int>> totalFlag(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
vector<SE3> wayPoints(0);

Eigen::VectorXd homePosRobot1 = Eigen::VectorXd::Zero(6);
Eigen::VectorXd lastRobot1Joint;
Eigen::VectorXd lastRobot2Joint;

SE3 initBusbar = SE3(Vec3(0.0, 0.0, -0.5));

// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);
bool saveTraj = true;

indyRobotManager* rManager1;
indyRobotManager* rManager2;
robotRRTManager* RRTManager = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
void initDynamics();
// rendering
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncInput();
void updateFuncPlanning();
void updateFuncTestSensor();

void updateFuncVision();
void updateFuncHYUPlanning();
void updateFuncRobotState();

void updateFuncTotal();

void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation);
void environmentSetting_HYU2(bool connect);

// communication function
void setEnviromentFromVision(const vision_data& skku_dataset);
void setRobotFromRealRobot(const robot_current_data& robot_state);
char* getSimulationState(vector<srSystem*> objects);
void RRT_problemSettingFromRobotCommand(const desired_dataset& hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag);

void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting_HYU();
void RRT_problemSetting();
void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject);
void RRTSolve();
void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize);
void rrtSetting();

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
vector<bool> attachObjRender(0);

// 서버 초기화
Server serv = Server::Server();
dataset hyu_dataset;
desired_dataset hyu_desired_dataset;
vision_data skku_dataset;
robot_current_data robot_state;


char *hyu_data;
char *copy;
char *pbuffer;



char hyu_data_flag;
char tmp_buffer[255];
char divChar= 'd';

int digit_num = 5;
int nway = 0;

vector<srSystem*> objects(2);


vector<srLink*> obstacle(10);
vector<srWeldJoint*> wJoint(10);


int main(int argc, char **argv)
{

	busbar[0] = new BusBar_HYU;

	for (int i = 0; i < obstacle.size(); i++)
	{
		obstacle[i] = new srLink();
		wJoint[i] = new srWeldJoint;
	}
	



	srand(time(NULL));
	// Robot home position
	homePosRobot1[1] = -SR_PI_HALF; homePosRobot1[3] = SR_PI_HALF; homePosRobot1[4] = -0.5 * SR_PI;

	// environment
	workspaceSetting();
	environmentSetting_HYU2(false);			// temporary environment setting
	robotSetting();
	Trobotbase = robot1->GetBaseLink()->GetFrame();

	
	//vector<srSystem*> objects(2);
	objects[0] = busbar[0];
	objects[1] = jigAssem;
	//initBusbar = SE3(Vec3(0.0, -0.4, 0.05)) * jigAssem->GetBaseLink()->GetFrame();
	busbar[0]->setBaseLinkFrame(initBusbar);

	////////////////////////////////////////////// setting srLib
	initDynamics();								// initialize srLib

												// robot manager setting
	robotManagerSetting();

	// workcell robot initial config
	homePosRobot2.setZero();
	homePosRobot2[0] = 0.0; homePosRobot2[1] = -SR_PI_HALF; homePosRobot2[2] = 80.0 / 90.0*SR_PI_HALF; homePosRobot2[3] = SR_PI_HALF;
	rManager2->setJointVal(homePosRobot2);
	rManager1->setJointVal(homePosRobot1);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	// rrt
	rrtSetting();

	///////////////////////////////////////////////////////////


	///////////////////////////////////////////////////////////
	//cout << Trobotbase % initBusbar << endl;
	//cout << jigAssem->GetBaseLink()->GetFrame() << endl;
	//char* send_data;
	//send_data = getSimulationState(objects);
	///////////////////////////////////////////////////////////


	thread commuThread(communicationFunc, argc, argv);
	commuThread.detach();
	
	//m.lock();
	
	//thread rendThread(rendering);
	
	//thread rendThread(renderFunc);
	//rendThread.detach();
	if (commuThread.joinable())
		commuThread.join();
	//if (rendThread.joinable())
	//	rendThread.join();

	rendering(argc, argv);

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);
	renderer->setUpdateFunc(updateFuncTotal);
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

void updateFuncPlanning()
{
	
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);
	rManager2->setJointVal(homePosRobot2);

	static int cnt = 0;
	static int writeCnt = 0;
	static int writeTaskCnt = 0;

	static int trjIdx = 0;
	static int taskIdx = 0;

	int idx = taskIdx % renderTraj.size();
	if (idx == 0 && cnt > 0)
		cnt = 0;
	if (cnt == 0)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
		cnt++;
	}
	
	// Calculate acc
	if (trjIdx == renderTraj[idx].size() - 1)
		jointAcc = (renderTraj[idx][trjIdx ] - 2.0*renderTraj[idx][trjIdx - 1] + renderTraj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == renderTraj[idx].size() - 2)
		jointAcc = (renderTraj[idx][trjIdx+1] - 2.0*renderTraj[idx][trjIdx] + renderTraj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (renderTraj[idx][trjIdx + 2] - 2.0*renderTraj[idx][trjIdx + 1] + renderTraj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	
	SE3 renderEndeff = rManager1->forwardKin(renderTraj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);


	// Calculate init vel
	if (trjIdx < renderTraj[idx].size() - 1)
	{
		jointVel = (renderTraj[idx][trjIdx + 1] - renderTraj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
		rManager1->setJointValVel(renderTraj[idx][trjIdx], jointVel);
	}
	else
		jointVel = (renderTraj[idx][trjIdx] - renderTraj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
	
	// control acceleration
	//rManager1->controlJointAcc(jointAcc);

	

	Eigen::VectorXd tau = rManager1->inverseDyn(renderTraj[idx][trjIdx], jointVel, jointAcc);
	rManager1->controlJointTorque(tau);

	//busbar movement
	if (attachObjRender[taskIdx % renderTraj.size()])
	{
		busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper));
		busbar[0]->setBaseLinkFrame(renderEndeff * Inv(Tbusbar2gripper));
	}
	else
		busbar[0]->setBaseLinkFrame(initBusbar);

	// read sensor value
	Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3( rManager1->getBodyJacobian(renderTraj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	Vdot = Vectortose3(rManager1->getBodyJacobian(renderTraj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointAcc +
		rManager1->getBodyJacobianDot(renderTraj[idx][trjIdx], jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V) - busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	
	if (attachObjRender[taskIdx % renderTraj.size()])
		ftsensor = dse3toVector(Ftsensor + InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper), Fr));
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
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
	/*

	if (idxTraj[idx][trjIdx] != 100)
	busbar[objNum]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);*/






	if (writeTaskCnt < renderTraj.size())
	{
		if (writeCnt < renderTraj[idx].size())
		{
			
			// make vector<> format
     		FTtraj[idx].push_back(ftsensor);
			// See from the robot base like frame
			TtrajVec[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*Ttraj[idx][trjIdx]));
     		busbarTraj[idx].push_back(SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*(busbar[0]->GetBaseLink()->GetFrame())));
			goalJigLocation[0] = SE3toVectorXd(Inv(robot1->GetBaseLink()->GetFrame())*jigAssem->GetBaseLink()->GetFrame());
			
			writeCnt++;
			
		}
		if (writeCnt == renderTraj[idx].size())
		{
			string dir_folder = "../../../data/HYU_data2/success_data";
			// save
			string dir_temp = dir_folder;
			saveDataToText(renderTraj[idx], dir_temp.append("/jointValTraj").append(to_string(idx)).append(".txt"));
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

	
	if (trjIdx == renderTraj[idx].size())
	{
		trjIdx = 0;

		taskIdx++;
		cout << "taskIdx: " << taskIdx % renderTraj.size() << endl;
	}
	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	{
		/*cout << taskIdx << endl;
		cout << traj[idx][trjIdx].transpose() << endl;*/
	}
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}

void updateFuncInput()
{
	gSpace.m_Gravity = Vec3(0.0, 0.0, 0.0);
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
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
		if (attachObjRender[taskNum % goalPos.size()])
			busbar[0]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);
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
		rManager1->setJointValVel(homePosRobot1, testjointVel);
		homePosRobot2 = homePosRobot1;
		jointVel = testjointVel;
	}
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	homePosRobot2 = rManager1->getJointVal();
	jointVel = rManager1->getJointVel();
	
		
	cnt++;
	Eigen::VectorXd jointAcc = Eigen::VectorXd::Ones(6);
	Eigen::VectorXd tau = rManager1->inverseDyn(homePosRobot2, jointVel, jointAcc);
	rManager1->controlJointTorque(tau);
	SE3 renderEndeff = rManager1->forwardKin(homePosRobot2, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	busbar[0]->setBaseLinkFrame(renderEndeff*Inv(Tbusbar2gripper));
	// read sensor value
	dse3 Ftsensor = rManager1->readSensorValue();
	dse3 Fr(0.0);
	se3 g(0.0);
	se3 Vdot(0.0);
	se3 V(0.0);
	V = Vectortose3(rManager1->getBodyJacobian(homePosRobot2, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	if (cnt == 0)
		jointAcc = rManager1->getJointAcc();
	Vdot = Vectortose3(rManager1->getBodyJacobian(homePosRobot2, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointAcc +
		rManager1->getBodyJacobianDot(homePosRobot2, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	for (int i = 0; i < 3; i++)
		g[i + 3] = gSpace.m_Gravity[i];
	Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V);
	dse3 Fr_g = - (busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g));
	se3 g_bus = InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);
	SE3 Tbus = busbar[0]->GetBaseLink()->GetFrame();
	Fr += Fr_g;
	dse3 Fr_busbar = InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper), Fr);
	ftsensor = dse3toVector(Ftsensor + Fr_busbar);
	

	cout << "q: " << homePosRobot2.transpose() << endl;
	cout << "a: " << rManager1->getJointAcc().transpose() << endl;
	cout << "f: " << ftsensor.transpose() << endl;
	cout << "busbar: " << endl << busbar[0]->GetBaseLink()->GetFrame() << endl;
	cout << "V: " << V << endl;
	cout << "A: " << Vdot << endl;
}

void updateFuncVision()
{
	//gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	// robot to homePos
	rManager2->setJointVal(homePosRobot1);
	rManager1->setJointVal(homePosRobot1);
	//cout << "Vision Update func called" << endl;
}

void updateFuncHYUPlanning()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trjIdx = 0;
	static int taskIdx = 0;

	// variable for restart rendering when planning result is updated
	bool replanned = false;
	static vector<unsigned int> len_bf(0);
	static vector<unsigned int> len_cur(0);
	len_cur.resize(renderTraj.size());
	if (len_cur.size() != len_bf.size())
		replanned = true;
	else
	{
		for (unsigned int i = 0; i < len_cur.size(); i++)
		{
			len_cur[i] = renderTraj[i].size();
			if (len_cur[i] != len_bf[i])
			{
				replanned = true;
				break;
			}
		}
	}
	
	int idx = taskIdx % renderTraj.size();
	if (idx == 0 && cnt > 0 || replanned)
		cnt = 0;
	len_bf = len_cur;
	// Calculate acc
	if (trjIdx == renderTraj[idx].size() - 1)
		jointAcc = (renderTraj[idx][trjIdx] - 2.0*renderTraj[idx][trjIdx - 1] + renderTraj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == renderTraj[idx].size() - 2)
		jointAcc = (renderTraj[idx][trjIdx + 1] - 2.0*renderTraj[idx][trjIdx] + renderTraj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (renderTraj[idx][trjIdx + 2] - 2.0*renderTraj[idx][trjIdx + 1] + renderTraj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);

	SE3 renderEndeff = rManager1->forwardKin(renderTraj[idx][trjIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);

	if (trjIdx < renderTraj[idx].size() - 1)
	{
		jointVel = (renderTraj[idx][trjIdx + 1] - renderTraj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
		rManager1->setJointValVel(renderTraj[idx][trjIdx], jointVel);
	}
	else
		jointVel = (renderTraj[idx][trjIdx] - renderTraj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;

	Eigen::VectorXd tau = rManager1->inverseDyn(renderTraj[idx][trjIdx], jointVel, jointAcc);
	rManager1->controlJointTorque(tau);

	//busbar movement
	if (attachObjRender[taskIdx % renderTraj.size()])
	{
		busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper_new));
		busbar[0]->setBaseLinkFrame(renderEndeff * Inv(Tbusbar2gripper_new));
	}

	trjIdx++;


	if (trjIdx == renderTraj[idx].size())
	{
		trjIdx = 0;

		taskIdx++;
		cout << "taskIdx: " << taskIdx % renderTraj.size() << endl;
	}
	rManager2->setJointVal(homePosRobot1);
	//if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	//{
	//	/*cout << taskIdx << endl;
	//	cout << traj[idx][trjIdx].transpose() << endl;*/
	//}
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

	//printf("updateFuncHYUplanning Called!!!\n");
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
	double z_angle = (double)rand() / RAND_MAX * 0.1;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = (double)rand() / RAND_MAX * 0.1;
	//SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
		gSpace.AddSystem(busbar[i]);
	}
	jigAssem->SetBaseLinkType(srSystem::FIXED);
	jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->getStagePlate());
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
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new indyRobotManager(robot1, &gSpace);

	// robot 2
	rManager2 = new indyRobotManager(robot2, &gSpace);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}

void rrtSetting()
{
	vector<srStateJoint*> planningJoints(6);
	for (unsigned int i = 0; i < planningJoints.size(); i++)
		planningJoints[i] = (srStateJoint*)robot1->gJoint[i];
	RRTManager->setSystem(planningJoints);
	RRTManager->setSpace(&gSpace);
	RRTManager->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
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

	renderTraj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		objNum = i / 2;
		RRTManager->setStartandGoal(initPos[i], goalPos[i]);

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
			RRTManager->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			RRTManager->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = RRTManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager->execute(0.05);
		tempTraj = RRTManager->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		renderTraj.push_back(tempTraj);

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
		for (unsigned int i = 0; i < renderTraj.size(); i++)
		{
			for (unsigned int j = 0; j < renderTraj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(renderTraj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);
		}
			


		
	}
}


void RRT_problemSetting(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag)
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
	bool feas = RRTManager->checkFeasibility(init);
	if (feas != 0)
		printf("initial point not feasible!!!\n");
	Eigen::VectorXd qtemp;
	waypointFlag.resize(wayPoints.size());
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		qtemp = rManager1->inverseKin(Trobotbase * wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit2);
		if (flag != 0)
			qtemp = rManager1->inverseKin(Trobotbase * wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
		if (flag != 0)
			qtemp = rManager1->inverseKin(Trobotbase * wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[initPos.size() - 1]);
		printf("%d-th init inv kin flag: %d\n", i, flag);
		
		if (attachObject[i])
			RRTManager->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
		else
			RRTManager->detachObject();


		feas = RRTManager->checkFeasibility(qtemp);

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

void RRTSolve_HYU(vector<bool> attachObject, vector<double> stepsize)
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
	cout << goalPos[goalPos.size() - 1].transpose() << endl;
	for (int i = start; i < end; i++)
	{
		RRTManager->setStartandGoal(initPos[i], goalPos[i]);
		
		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;
		
		if (attachObject[i])
			RRTManager->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
		else
			RRTManager->detachObject();


		feas = RRTManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager->execute(stepsize[i]);
		tempTraj = RRTManager->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager->setState(tempTraj[j]))
				printf("collide at %d-th trj, %d-th point!!!\n", i, j);

		traj.push_back(tempTraj);


		//tempTtraj.resize(tempTraj.size());
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
	renderTraj = traj;
}

void setEnviromentFromVision(const vision_data & skku_dataset)
{
	// set object (id:1 - busbar, id:2 - jig) 
	if (skku_dataset.objPos[1].size() > 0)
	{
		busbar[0]->GetBaseLink()->SetFrame(Trobotbase * SKKUtoSE3(skku_dataset.objOri[1], skku_dataset.objPos[1]));
		busbar[0]->SetBaseLinkType(srSystem::FIXED);
	}
	if (skku_dataset.objPos[2].size() > 0)
	{
		jigAssem->GetBaseLink()->SetFrame(Trobotbase * SKKUtoSE3(skku_dataset.objOri[2], skku_dataset.objPos[2]));
		jigAssem->SetBaseLinkType(srSystem::FIXED);
	}


	// set obstacle
	for (unsigned int i = 0; i < skku_dataset.obsInfo.size(); i++)
	{

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
	SE3 T = Trobotbase % robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame();
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
		SE3 Ttemp = Trobotbase % objects[i]->GetBaseLink()->GetFrame();
		//cout << Ttemp << endl;
		//Object1 Position (3x1)
		for (int i = 0; i < 3; i++) {
			pbuffer = _gcvt(Ttemp[9+i], digit_num, tmp_buffer);

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

void RRT_problemSettingFromRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag)
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
		
	RRT_problemSetting(init, wayPoints, includeOri, attachObject, waypointFlag);
}

void communicationFunc(int argc, char **argv)
{
	while (TRUE) {

		//Receiving data from HYU client
		//recv_data = serv.RecevData();
		hyu_data = serv.RecevData();
		hyu_data_flag = hyu_data[0];
		//serv.SendMessageToClient("G");

		// 데이터 전송
		if (hyu_data_flag == 'I')
		{
			serv.SendMessageToClient("I");
		}
		else if (hyu_data_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			printf("%s\n", hyu_data);
			readSKKUvision(hyu_data, skku_dataset);

			/*	cout << skku_dataset.obsInfo[0][0] << endl;
			cout << skku_dataset.objOri[0][0] << endl;
			cout << skku_dataset.objPos[0][0] << endl;
			*/


			setEnviromentFromVision(skku_dataset);		// should be called later than robotSetting

														/////////////////////////////////////// after setting environment
			initDynamics();								// initialize srLib

														// robot manager setting
			robotManagerSetting();

			// workcell robot initial config
			homePosRobot2.setZero();
			homePosRobot2[0] = 0.0; homePosRobot2[1] = -SR_PI_HALF; homePosRobot2[2] = 80.0 / 90.0*SR_PI_HALF; homePosRobot2[3] = SR_PI_HALF;
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

			char* send_data;
			//send_data = getSimulationState(objects);
			//serv.SendMessageToClient(send_data);	
			serv.SendMessageToClient(hyu_data);
		}
		else if (hyu_data_flag == 'R')
		{
			// Robot cur data
			char* copy = (char*)malloc(sizeof(char)*strlen(hyu_data));
			for (int p = 0; p <= strlen(hyu_data); p++)
				copy[p] = hyu_data[p];
			serv.SendMessageToClient(copy);
			printf("%s\n", hyu_data);
			readRobotCurState(hyu_data, robot_state);

			cout << robot_state.robot_joint.transpose() << endl;
			rManager1->setJointVal(robot_state.robot_joint);

			//rendering(argc, argv);
			m.lock();
			isVision = false;
			isHYUPlanning = false;
			isRobotState = true;
			m.unlock();
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
				serv.SendMessageToClient("P");
			}

		}
		else if (hyu_data_flag == 'P') {
			isHYUPlanning = false;		// to turn off rendering
			vector<bool> attachObject(0);

			vector<bool> waypointFlag(0);

			readRobotCurState(hyu_data, robot_state);
			cout << robot_state.robot_joint.transpose() << endl;
			RRT_problemSettingFromRobotCommand(hyu_desired_dataset, attachObject, robot_state.robot_joint, waypointFlag);		// change homepos later to read current joint values of robot
			vector<double> stepsize(0);
			vector<double> attachobject(0);
			for (unsigned int i = 0; i < waypointFlag.size(); i++)
			{
				if (waypointFlag[i])
				{
					stepsize.push_back(0.1);
					attachobject.push_back(attachObject[i]);
				}
			}
			busbar[0]->setBaseLinkFrame(initBusbar);								// change initial busbar SE3 later 
			
			// update global variable
			m.lock();
			RRTSolve_HYU(attachObject, stepsize);
			attachObjRender = attachObject;
			isHYUPlanning = true;
			isVision = false;
			isRobotState = false;
			m.unlock();
			//////////////////////////
			char* send_data = makeJointCommand(renderTraj, hyu_desired_dataset);
			serv.SendMessageToClient(send_data);
			printf("%s\n", send_data);
			if (attachObject[attachObject.size() - 1])
				gripState = 1;
			else
				gripState = 0;

			if (saveTraj)
			{
				vector<Eigen::VectorXd> saveTrj(0);
				vector<Eigen::VectorXd> saveAttach(0);
				Eigen::VectorXd truevec(1);
				truevec[0] = 1;
				Eigen::VectorXd falsevec(1);
				falsevec[0] = 0;
				for (unsigned int i = 0; i < renderTraj.size(); i++)
				{
					for (unsigned int j = 0; j < renderTraj[i].size(); j++)
					{
						saveTrj.push_back(renderTraj[i][j]);
						if (attachObject[i])
							saveAttach.push_back(truevec);
						else
							saveAttach.push_back(falsevec);
					}
				}
				string dir_folder = "../../../data/communication_test";
				// save
				string dir_temp = dir_folder;
				saveDataToText(saveTrj, dir_temp.append("/jointValTraj").append(".txt"));
				dir_temp = dir_folder;
				saveDataToText(saveAttach, dir_temp.append("/attachTraj").append(".txt"));

				printf("planned trajectory saved!!!\n");
			}
			
			//rendering();
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
	if (isVision)
		updateFuncVision();
	else if (isHYUPlanning)
		updateFuncHYUPlanning();
	else
		updateFuncRobotState();
	static int updateFuncCnt = 0;
	//printf("update func cnt: %d\n", updateFuncCnt++);
}
