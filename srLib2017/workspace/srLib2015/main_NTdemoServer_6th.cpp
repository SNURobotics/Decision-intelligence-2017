#include <cstdio>
#include "myRenderer.h"
#include "NTdemoEnvSetting_6th.h"
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

#define USE_TASK_MANAGER_FUNC

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
int activeJointIdx = 0;
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

struct CUR_JOINT
{
	double q1;
	double q2;
	double q3;
	double q4;
	double q5;
	double q6;
};

LRESULT CALLBACK getMessageFromRobot(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	if (uMsg == WM_COPYDATA)
	{
		COPYDATASTRUCT* pcds = (COPYDATASTRUCT*)lParam;
		if (pcds->dwData == 222)
		{
			CUR_POS cur_pos;
			memcpy_s(&cur_pos, sizeof(cur_pos), pcds->lpData, pcds->cbData);
			vector<double> curPos(0);
			// convert deg -> rad, mm -> m
			curPos.push_back(cur_pos.Rx*(SR_PI / 180.0));
			curPos.push_back(cur_pos.Ry*(SR_PI / 180.0));
			curPos.push_back(cur_pos.Rz*(SR_PI / 180.0));
			curPos.push_back(cur_pos.X*0.001);
			curPos.push_back(cur_pos.Y*0.001);
			curPos.push_back(cur_pos.Z*0.001);

			double eps = 1e-5;
			if (abs(curPos[0]) < eps && abs(curPos[1]) < eps && abs(curPos[2]) < eps && abs(curPos[3]) < eps && abs(curPos[4]) < eps && abs(curPos[5]) < eps)
			{
				cout << "RPOSC error: retry get currunt position" << endl;
				demoTask->getCurPosSignal();
				return 0;
			}
			else
				demoTask->setCurPos(curPos);
		}
		else if (pcds->dwData == 223)
		{
			/////////////////////////////////////////////////////// check later ///////////////////////////////////////////////////////
			CUR_JOINT cur_joint;
			memcpy_s(&cur_joint, sizeof(cur_joint), pcds->lpData, pcds->cbData);
			vector<double> curJoint(0);
			// convert deg -> rad, mm -> m
			curJoint.push_back(cur_joint.q1*(SR_PI / 180.0));
			curJoint.push_back(cur_joint.q2*(SR_PI / 180.0));
			curJoint.push_back(cur_joint.q3*(SR_PI / 180.0));
			curJoint.push_back(cur_joint.q4*(SR_PI / 180.0));
			curJoint.push_back(cur_joint.q5*(SR_PI / 180.0));
			curJoint.push_back(cur_joint.q6*(SR_PI / 180.0));
			demoTask->setCurJoint(curJoint);
		}
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
	demoTask = new demoTaskManager(demoEnv, rManager1, 0);
	demoTask->setRobotRRTManager();

	// for communication (dummy window dialog)
	WNDCLASS windowClass = {};
	windowClass.lpfnWndProc = getMessageFromRobot;
	LPCWSTR windowClassName = L"srLibServer";
	windowClass.lpszClassName = windowClassName;
	if (!RegisterClass(&windowClass)) {
		std::cout << "Failed to register window class" << std::endl;
		return 1;
	}
	HWND messageWindow = CreateWindow(windowClassName, 0, 0, 0, 0, 0, 0, HWND_MESSAGE, 0, 0, 0);
	if (!messageWindow) {
		std::cout << "Failed to create message-only window" << std::endl;
		return 1;
	}

	// test code (19.03.21)
	demoTask->startConnection();
	demoTask->endConnection();
	demoTask->startConnection();
	for (int i = 0; i < 2; i++)
	{
		demoTask->getCurPosSignal();
		Sleep(1000);
	}
	demoTask->gripperOnSignal();
	Sleep(100);
	demoTask->gripperOffSignal();
	/*Eigen::VectorXd curPos(6);
	curPos[0] = DEG2RAD(180.0);
	curPos[1] = DEG2RAD(2);
	curPos[2] = DEG2RAD(-132);
	curPos[3] = 0.417;
	curPos[4] = 0.088;
	curPos[5] = 0.509;
	SE3 testPos1 = demoTask->YKpos2SE3(curPos);
	SE3 testPos2 = SE3(Vec3(0.0, 0.0, 0.1)) * testPos1;
	demoTask->printImovCommand(demoTask->getTcurRobot(), testPos1);
	for (int i = 0; i < 3; i++)
	{
		cout << "######################    " << i+1 << "    ########################" << endl;
		demoTask->goToWaypoint(testPos1);
		cout << "pos2 to pos1 (down) finished !!!!!!!!!!!!!" << endl;
		Sleep(1000);
		demoTask->goToWaypoint(testPos2);
		cout << "pos1 to pos2 (up) finished !!!!!!!!!!!!!" << endl;
		Sleep(1000);
	}*/
	//demoTask->endConnection();

	demoTask->goToWaypoint(demoTask->homeSE3);

	//qval.setZero(6);
	//qval[2] = 1.0;
	//rManager1->setJointVal(qval);
	//qval[0] = DEG2RAD(0.0);
	//qval[1] = DEG2RAD(0.0);
	//qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
	//qval[3] = DEG2RAD(0.0);
	//qval[4] = DEG2RAD(-90.0);
	//qval[5] = DEG2RAD(0.0);
	//rManager1->setJointVal(qval);

	int flag;

	//qval = rManager1->inverseKin(demoTask->goalSE3[0] * demoTask->goalOffset * curGraspOffset, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qval);
	//rManager1->setJointVal(qval);
	//cout << flag << endl;

	// read given text data (for Test)
	//std::ifstream in("../../../data/SKKU_data_6th/TestData.txt");
	//std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	//char* received_data = (char*)contents.c_str();

	//communication_flag = received_data[0];

	//if (communication_flag == 'V')
	//{
	//	// vision data
	//	std::printf("%s\n", received_data);
	//	demoTask->updateEnv(received_data);
	//	demoTask->setObjectNum();
	//}

	cout << "EE SE3: " << endl;
	cout << rManager1->forwardKin(rManager1->getJointVal(), &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], SE3()) << endl;

	// 둘 중 하나만 골라서 실행
	//rendering(argc, argv);
	communicationFunc(argc, argv);

	//만약 while 루프를 돌리지 않을 경우 무한정 서버를 기다리는 함수, 실제 사용하지는 않는다.
	//serv.WaitServer(); 

	// 서버를 종료 시킴
	serv.~Server();

	return 0;
}

void communicationFunc(int argc, char **argv)
{
	static bool sentInit = false;
	while (TRUE)
	{
		//std::ifstream in("../../../data/SKKU_data_6th/TestData.txt");
		//std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		//char* received_data = (char*)contents.c_str();

		//Receiving data from HYU client
		char* received_data = serv.RecevData();
		communication_flag = received_data[0];
		if (!sentInit)
		{
			std::printf("push the button to get vision data\n");
			getchar();
			serv.SendMessageToClient("I");
			sentInit = true;
		}
		// 데이터 전송

		if (communication_flag == 'V')
		{
			// vision data
			std::printf("%s\n", received_data);
			demoTask->updateEnv(received_data);
			demoTask->setObjectNum();

#ifdef USE_TASK_MANAGER_FUNC

			// move robot to deliver workingobject
			std::printf("push the botton to do move job\n");
			getchar();
			demoTask->startConnection();
			demoTask->endConnection();
			demoTask->startConnection();
			bool isJobFinished = demoTask->moveJob();

			// return to home pos
			std::printf("push the botton to do return job\n");
			getchar();
			demoTask->startConnection();
			demoTask->endConnection();
			demoTask->startConnection();
			bool isReturned = demoTask->returnJob();
			if (isReturned) sentInit = 0;
#else

			int curObjID = demoTask->curObjID;
			int curGoalID = demoTask->getGoalNum();
			SE3 curGraspOffset = demoTask->curGraspOffset;
			SE3 reachOffset = demoTask->reachOffset;

			// reachObject
			printf("push the botton to get reachObject imov command\n");
			getchar();
			demoTask->getCurPosSignal();	// to set demoTask->TcurRobot
			SE3 T1 = demoTask->curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset;
			demoTask->printImovCommand(demoTask->TcurRobot, T1);

			// goToGraspPos
			printf("push the botton to get goToGraspPos imov command\n");
			getchar();
			SE3 T2 = demoTask->curObjectData.objectSE3[curObjID] * curGraspOffset;
			demoTask->printImovCommand(T1, T2);

			// graspObject
			printf("push the botton to call graspObject command\n");
			getchar();
			demoTask->graspObject();

			// moveWorkspaceDisplacement(Vec3(0.0, 0.0, 0.05))
			printf("push the botton to get moveWorkspaceDisplacement imov command\n");
			getchar();
			SE3 T3 = SE3(Vec3(0.0, 0.0, 0.05)) * T2;
			demoTask->printImovCommand(T2, T3);

			// moveObject
			printf("push the botton to get moveObject imov command\n");
			getchar();
			SE3 T4 = demoTask->goalSE3[curGoalID] * demoTask->goalOffset * curGraspOffset;
			demoTask->printImovCommand(T3, T4);

			// releaseObject
			printf("push the botton to call releaseObject command\n");
			getchar();
			demoTask->releaseObject();

			// goHomepos
			printf("push the botton to get goHomepose imov command\n");
			getchar();
			demoTask->printImovCommand(T4, demoTask->homeSE3);
#endif
			sentInit = false;
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
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP();

	//static double JointVal = 0;
	////((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)MHRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	////qval[5] = JointVal;
	//rManager1->setJointVal(qval);
	//static int cnt = 0;
	//static int trajcnt = 0;
	//cnt++;
	//demoTask->rManager->setJointVal(demoTask->robotrrtManager->getStart());
	//if (cnt % 10 == 0)
	//	trajcnt++;
	//if (traj.size() > 0)
	//	rManager1->setJointVal(traj[trajcnt % traj.size()]);


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
