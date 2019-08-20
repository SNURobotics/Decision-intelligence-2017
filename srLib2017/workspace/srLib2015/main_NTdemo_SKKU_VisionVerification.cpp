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
			//double eps = 1e-5;
			//if (abs(curJoint[0]) < eps && abs(curJoint[1]) < eps && abs(curJoint[2]) < eps && abs(curJoint[3]) < eps && abs(curJoint[4]) < eps && abs(curJoint[5]) < eps)
			//{
			//	cout << "RPOSJ error: retry get currunt position" << endl;
			//	demoTask->getCurJointSignal();
			//	return 0;
			//}
			//else
			//	demoTask->setCurJoint(curJoint);
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
	demoTask = new demoTaskManager(demoEnv, rManager1);
	demoTask->setRobotRRTManager();

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
	std::ifstream in("../../../data/SKKU_data_6th/ResultsChkeck000.txt");
	std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
	char* received_data = (char*)contents.c_str();
	communication_flag = received_data[0];

	if (communication_flag == 'V')
	{
		// vision data
		printf("%s\n", received_data);
		demoTask->updateEnv(received_data);
		demoTask->setObjectNum();
	}

	// �� �� �ϳ��� ��� ����
	rendering(argc, argv);
	//communicationFunc(argc, argv);

	//���� while ������ ������ ���� ��� ������ ������ ��ٸ��� �Լ�, ���� ��������� �ʴ´�.
	//serv.WaitServer(); 

	// ������ ���� ��Ŵ
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
		//char* received_data = serv.RecevData();
		std::ifstream in("../../../data/SKKU_data_6th/ResultsChkeck000.txt");
		std::string contents((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
		char* received_data = (char*)contents.c_str();

		communication_flag = received_data[0];
		if (!sentInit)
		{
			printf("push the button to get vision data\n");
			getchar();
			serv.SendMessageToClient("I");
			sentInit = true;
		}
		// ������ ����

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
