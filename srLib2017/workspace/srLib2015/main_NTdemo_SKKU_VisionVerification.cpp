#include <cstdio>
#include "myRenderer.h"
#include "NTdemoEnvSetting_6th.h"

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

//
vector<SE3> Twaypoints1;
vector<SE3> Twaypoints2;
vector<SE3> Twaypoints3;
vector<SE3> Twaypoints4;
int cnt;
int iter;
vector<Eigen::VectorXd> renderTraj;

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
			//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		}

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
	demoTask = new demoTaskManager(demoEnv, rManager1, 1);
	demoTask->setRobotRRTManager();
	demoTask->setWhichTask(1);

	// for communication (dummy window dialog)
	WNDCLASS windowClass = {};
	//windowClass.lpfnWndProc = demoTask->WindowProcedure;
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

	// read given text data (for Test)
	std::ifstream in("../../../data/SKKU_data_6th/ResultsChkeck.txt");
	std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	char* received_data = (char*)contents.c_str();
	communication_flag = received_data[0];
	cout << received_data << endl;

	if (communication_flag == 'V')
	{
		// vision data
		printf("%s\n", received_data);
		demoTask->updateEnv(received_data);
		demoTask->setObjectNum();
	}

	cnt = 0;
	iter = 0;

	//Twaypoints1 = demoTask->planBetweenWaypoints(demoTask->homeSE3, demoTask->curObjectData.objectSE3[demoTask->curObjID] * demoTask->curGraspOffset * demoTask->reachOffset, 9);
	//int flag;
	//renderTraj.push_back(rManager1->inverseKin(demoTask->homeSE3, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//for (int i = 0; i < Twaypoints1.size(); i++)
	//{
	//	// Reaching
	//	renderTraj.push_back(rManager1->inverseKin(Twaypoints1[i], &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//	rManager1->setJointVal(renderTraj.back());
	//}
	//// Go to grasp pose
	//renderTraj.push_back(rManager1->inverseKin(demoTask->curObjectData.objectSE3[demoTask->curObjID] * demoTask->curGraspOffset * SE3(Vec3(0.0, 0.0, 0.005)), &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//rManager1->setJointVal(renderTraj.back());
	//renderTraj.push_back(rManager1->inverseKin(SE3(Vec3(0.0, 0.0, 0.15)) * demoTask->curObjectData.objectSE3[demoTask->curObjID] * demoTask->curGraspOffset * SE3(Vec3(0.0, 0.0, 0.005)), &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//rManager1->setJointVal(renderTraj.back());

	//Twaypoints2 = demoTask->planBetweenWaypoints(SE3(Vec3(0.0, 0.0, 0.15)) * demoTask->curObjectData.objectSE3[demoTask->curObjID] * demoTask->curGraspOffset * SE3(Vec3(0.0, 0.0, 0.005)), demoTask->goalSE3[demoTask->curGoalID] * demoTask->goalOffset * demoTask->curGraspOffset, 9);
	//for (int i = 0; i < Twaypoints2.size(); i++)
	//{
	//	// Reaching
	//	renderTraj.push_back(rManager1->inverseKin(Twaypoints2[i], &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//	rManager1->setJointVal(renderTraj.back());
	//}

	//Twaypoints3 = demoTask->planBetweenWaypoints(demoTask->goalSE3[demoTask->curGoalID] * demoTask->goalOffset * demoTask->curGraspOffset, demoTask->homeSE3, 9);
	//for (int i = 0; i < Twaypoints3.size(); i++)
	//{
	//	// Home position
	//	renderTraj.push_back(rManager1->inverseKin(Twaypoints3[i], &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, rManager1->getJointVal()));
	//	rManager1->setJointVal(renderTraj.back());
	//}

	// 둘 중 하나만 골라서 실행
	rendering(argc, argv);
	//communicationFunc(argc, argv);

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
		//Receiving data from HYU client

		// For code test
		char* received_data = serv.RecevData();
		/*std::ifstream in("../../../data/SKKU_data_6th/ResultsChkeck03.txt");
		std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		char* received_data = (char*)contents.c_str();*/

		communication_flag = received_data[0];
		if (!sentInit)
		{
			printf("push the button to get vision data\n");
			getchar();
			serv.SendMessageToClient("I");
			sentInit = true;
		}
		// 데이터 전송

		if (communication_flag == 'V')
		{
			// vision data
			printf("%s\n", received_data);
			demoTask->updateEnv(received_data);
			demoTask->setObjectNum();
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
	if (cnt % 10 == 0)
	{
		int flag;
		iter++;
		//rManager1->setJointVal(renderTraj[iter-1]);
		if (iter == renderTraj.size())
		{
			cnt = 0; iter = 0;
		}
	}
	cnt++;
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
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
}

void MHRobotManagerSetting()
{
	rManager1 = new MH12RobotManager(MHRobot, &gSpace);
}

void envSetting()
{
	demoEnv->setEnvironmentInSrSpace(&gSpace);
}
