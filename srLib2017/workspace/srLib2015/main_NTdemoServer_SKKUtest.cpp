#include <cstdio>
#include "myRenderer.h"
#include "NTdemoEnvSetting_4th.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager\MH12Robot.h"
#include <time.h>


// for communication
#include <iostream>
#include <Windows.h>
#include <windowsx.h>

//#define USE_COMMUNICATION // when using communication

// srSpace and renderer
srSpace gSpace;
myRenderer* renderer;

// Robot
MH12Robot* MHRobot = new MH12Robot;
MH12RobotManager* rManager1;

Eigen::VectorXd qval;

// demo environment
demoEnvironment* demoEnv;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
demoTaskManager* demoTask;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void envSetting();
void MHRobotSetting();
void MHRobotManagerSetting();
void URrrtSetting();
void loadDataFromFile(string loc, char* visiondata);
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);

// server
Server serv = Server::Server(); 
char communication_flag;
void communicationFunc(int argc, char **argv);



struct CUR_POS
{

	double X;
	double Y;
	double Z;
	double Rx;
	double Ry;
	double Rz;
};
LRESULT CALLBACK getMessageFromRobot(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (uMsg == WM_COPYDATA)
	{
		CUR_POS cur_pos;
		COPYDATASTRUCT* pcds = (COPYDATASTRUCT*)lParam;
		memcpy_s(&cur_pos, sizeof(cur_pos), pcds->lpData, pcds->cbData);
		vector<double> curPos(0);
		// convert deg -> rad, mm -> m
		curPos.push_back(cur_pos.Rx*(SR_PI/180.0));
		curPos.push_back(cur_pos.Ry*(SR_PI/180.0));
		curPos.push_back(cur_pos.Rz*(SR_PI/180.0));
		curPos.push_back(cur_pos.X*0.001);
		curPos.push_back(cur_pos.Y*0.001);
		curPos.push_back(cur_pos.Z*0.001);
		demoTask->setCurPos(curPos);

		//std::cout << "==================" << std::endl;
		//std::cout << "getMessageFromRobot function" << std::endl;
		//for(int i = 0; i < 6; i++)
		//{
		//	std::cout << "curPos: " << curPos[i] << std::endl;
		//}
	}

	return DefWindowProc(hWnd, uMsg, wParam, lParam);
}


int main(int argc, char **argv)
{
	////////////////////////////////////////////////////////////////
	////////////////////// initialize //////////////////////////////
	////////////////////////////////////////////////////////////////
	// set the number of objects in demoEnvirionment function
	demoEnv = new demoEnvironment(5);
	// add robot to system
    MHRobotSetting();
	// add bin and objects to system
	envSetting();
	initDynamics();
	MHRobotManagerSetting();
	demoTask = new demoTaskManager(demoEnv, rManager1);
	demoTask->setRobotRRTManager();

	qval.setZero(6);
	qval[0] = DEG2RAD(0.0);
	qval[1] = DEG2RAD(0.0);
	qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
	qval[3] = DEG2RAD(0.0);
	qval[4] = DEG2RAD(-90.0);
	qval[5] = DEG2RAD(0.0);
	rManager1->setJointVal(qval);

	int flag;
	
	qval = rManager1->inverseKin(demoTask->homeSE3, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qval);
	rManager1->setJointVal(qval);
	
	////////////////////////////////// when using communication //////////////////////////////////////
#ifdef USE_COMMUNICATION
	communicationFunc(argc, argv);
#else
	char visiondataSKKU[1000];
	// if load file from txt
	//loadDataFromFile("../../../workspace/robot/poseData/TextExample.txt", visiondataSKKU);
	//printf("%s\n", visiondataSKKU);
	/////////////////////////////////////////
	// input string directly
	strcpy(visiondataSKKU, "V1.d-0.8615d6.395e-003d0.1991d0.9403d0.2304d0.2503d-0.3366d0.7365d0.5867d-4.916e-002d-0.636d0.7702d-1.d-1.d-1.d-1.d1.d-0.9382d-4.005e-002d0.2501d-0.8364d-0.5466d-4.17e-002d0.5161d-0.8108d0.2762d-0.1848d0.2095d0.9602d-1.d-1.d-1.d-1.d1.d-0.8756d-0.1131d0.2517d0.4978d0.858d0.1266d-0.8672d0.495d5.443e-002d-1.597e-002d-0.1369d0.9905d-1.d-1.d-1.d-1.d1.d-0.9752d-0.1559d0.2589d0.9197d-0.3067d0.2451d0.3136d0.9495d1.112e-002d-0.2361d6.665e-002d0.9694d-1.d-1.d-1.d-1.d1.d-1.083d-5.901e-003d0.2216d0.543d-0.8376d-6.02e-002d0.6758d0.3933d0.6234d-0.4985d-0.3792d0.7796d-1.d-1.d-1.d-1.d");
	//strcpy(visiondataSKKU, "V9.d0.8925d0.0667d0.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d-1.d1.d0.77d6.853e-002d4.464e-002d-0.9875d-8.888e-002d-0.1303d0.1011d-0.9907d-9.07e-002d-0.1211d-0.1027d0.9873d-2.e-002d-2.e-002d0.d0.d");
	/////////////////////////////////////////
	demoTask->updateEnv(visiondataSKKU);
	demoTask->setObjectNum();
#endif
	////////////////////////////////// when not using communication //////////////////////////////////

	rendering(argc, argv);

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

void communicationFunc(int argc, char **argv)
{
	static int goalNum = 0;
	static bool sentInit = false;
	while (TRUE) 
	{

		//Receiving data from HYU client
		char* received_data = serv.RecevData();
		communication_flag = received_data[0];

		if (!sentInit)
		{
			printf("push the botton to get vision data\n");
			getchar();
			serv.SendMessageToClient("I");
			sentInit = true;
		}
		// 데이터 전송

		if (communication_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			printf("%s\n", received_data);
			demoTask->updateEnv(received_data);
			demoTask->setObjectNum();
			//////////////////////////////////////////////////////////////////////
			sentInit = false;

			break;
		}
	}
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
	gSpace.SetGravity(0.0, 0.0, -0.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{

	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static double JointVal = 0;
	//((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	((srStateJoint*)MHRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	JointVal += 0.01;
	//qval[5] = JointVal;
	rManager1->setJointVal(qval);
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;
	//demoTask->rManager->setJointVal(demoTask->robotrrtManager->getStart());
	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);

	
	//cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << MHRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);

	int stop = 1;
}


void MHRobotSetting()
{
	gSpace.AddSystem((srSystem*)MHRobot);
	MHRobot->GetBaseLink()->SetFrame(demoEnv->Trobotbase * demoEnv->Trobotbase2link1);
	MHRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	MHRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	//Trobotbase1 = MHRobot->GetBaseLink()->GetFrame() * MHRobot->TsrLinkbase2robotbase;
	//robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	//robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	//vector<int> gpIdx(2);
	//gpIdx[0] = 0;
	//gpIdx[1] = 1;
	//robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//gpIdx[0] = 2;
	//gpIdx[1] = 3;
	//robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void MHRobotManagerSetting()
{
	rManager1 = new MH12RobotManager(MHRobot, &gSpace);
}

void envSetting()
{
	demoEnv->setEnvironmentInSrSpace(&gSpace);
}

void loadDataFromFile(string loc, char* visiondata)
{
	std::ifstream fin;
	fin.open(loc);

	int i = 0;

	if (fin.is_open())
	{
		while (!fin.eof())
		{
			fin >> visiondata[i];
			i++;
		}
	}
	fin.close();
}