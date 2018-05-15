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
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);

// server
Server serv = Server::Server(); 
char communication_flag;
void communicationFunc(int argc, char **argv);



struct CUR_POS
{
	char Rx[256];
	char Ry[256];
	char Rz[256];
	char X[256];
	char Y[256];
	char Z[256];
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
		curPos.push_back(atof(cur_pos.Rx)*(SR_PI/180.0));
		curPos.push_back(atof(cur_pos.Ry)*(SR_PI/180.0));
		curPos.push_back(atof(cur_pos.Rz)*(SR_PI/180.0));
		curPos.push_back(atof(cur_pos.X)*0.001);
		curPos.push_back(atof(cur_pos.Y)*0.001);
		curPos.push_back(atof(cur_pos.Z)*0.001);
		demoTask->setCurPos(curPos);

		std::cout << "==================" << std::endl;
		std::cout << "getMessageFromRobot function" << std::endl;
		for(int i = 0; i < 6; i++)
		{
			std::cout << "curPos: " << curPos[i] << std::endl;
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
	demoTask = new demoTaskManager(demoEnv, rManager1);

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

	qval.setZero(6);
	qval[0] = DEG2RAD(0.0);
	qval[1] = DEG2RAD(0.0);
	qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg �϶��� ���� 0deg�϶��� ���
	qval[3] = DEG2RAD(0.0);
	qval[4] = DEG2RAD(-93.472);
	qval[5] = DEG2RAD(0.08);
	rManager1->setJointVal(qval);
	cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	int flag;
	cout << EulerXYZ(Vec3(DEG2RAD(179), DEG2RAD(-2), DEG2RAD(9)), Vec3(0.0, 0.0, 0.0)) << endl;
	// set object SE(3) from text
	//demoEnv->setObjectFromRobot2ObjectText("C:/Users/snurobotics/Documents/�Ǵ�����/4���⵵/PoseData180504/data00/Pose.txt", false);

	//rendering(argc, argv);
	
	communicationFunc(argc, argv);

	//���� while ������ ������ ���� ��� ������ ������ ��ٸ��� �Լ�, ���� ��������� �ʴ´�.
	//serv.WaitServer(); 

	// ������ ���� ��Ŵ
	serv.~Server();

	return 0;
}

void communicationFunc(int argc, char **argv)
{
	static int goalNum = 0;
	while (TRUE) {

		//Receiving data from HYU client
		char* received_data = serv.RecevData();
		//strcpy(received_data, "");
		//strcat(received_data, serv.RecevData());

		communication_flag = received_data[0];

		//printf(&communication_flag);
		//cout << endl;
		//serv.SendMessageToClient("G");

		// ������ ����

		if (communication_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			char* copy = (char*)malloc(sizeof(char)*(strlen(received_data) + 1));
			for (unsigned int p = 0; p <= strlen(received_data); p++)
				copy[p] = received_data[p];
			serv.SendMessageToClient(copy);
			Sleep(50);
			printf("%s\n", received_data);
			//demoTask->curObjectData.setObjectDataFromString(received_data);
			
			//////////////////////////////////////////////////////////////////////
			//m.lock();
			//m.unlock();
			free(copy);


			bool isJobFinished = demoTask->moveJob(goalNum);
			if (isJobFinished)
				while (1)
				{
					// when dual arm robot task is finished
					break;
					/////////////////////////////////////////////
				}
			bool isReturned = demoTask->returnJob();
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
	gSpace.AddSystem(demoEnv->bin);
	gSpace.AddSystem(demoEnv->table);
	for (unsigned int i = 0; i < demoEnv->objectNum; i++)
		gSpace.AddSystem(demoEnv->objects[i]);
}
