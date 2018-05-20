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
	demoEnv = new demoEnvironment(1);
	// add robot to system
    MHRobotSetting();
	// add bin and objects to system
	envSetting();
	initDynamics();
	MHRobotManagerSetting();
	demoTask = new demoTaskManager(demoEnv, rManager1);
	demoTask->setRobotRRTManager();
	//cout << demoEnv->Trobotbase2camera * demoEnv->Tcamera2robotbase;

	SE3 temp = EulerZYX(Vec3(DEG2RAD(129.3924), DEG2RAD(0.1922), DEG2RAD(176.9429)), Vec3());
	SE3 result = EulerZYX(Vec3(DEG2RAD(129.3926), DEG2RAD(-0.5806), DEG2RAD(176.3080)), Vec3());

	SE3 disp = Inv(temp) * result ;
	cout << temp << endl;
	cout << result << endl;
	cout << Log(disp.GetOrientation()) / Log(disp.GetOrientation()).Normalize() << endl;
	cout << RAD2DEG(Log(disp.GetOrientation()).Normalize()) << endl;


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
	demoTask->getCurPosSignal();
	//demoTask->gripperOnSignal();
	//demoTask->gripperOffSignal();

	//SE3 Ttemp = EulerZYX(Vec3(0.0, 0.0, DEG2RAD(-5.0)), Vec3(0.0, 0.0, 0.02)) * demoTask->TcurRobot;
	//demoTask->goToWaypoint(Ttemp);
	SE3 curGraspOffset = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.0036));
	//demoTask->goToWaypoint(demoTask->homeSE3);
	SE3 waypoiny1 = EulerZYX(Vec3(DEG2RAD(0.0), DEG2RAD(2.0460), DEG2RAD(179.0799)), Vec3(0.416704, 0.093653, 0.509468));
	//demoTask->goToWaypoint(waypoiny1);
	demoTask->goToWaypoint(demoTask->goalSE3[0] * curGraspOffset);

	//vector<SE3> Twaypoints = demoTask->planBetweenWaypoints(demoTask->TcurRobot, demoTask->goalSE3[0] * curGraspOffset);
	//traj = demoTask->tempTraj;
	
	//demoTask->goThroughWaypoints(Twaypoints);

	//vector<SE3> Ttemps(2);
	//Ttemps[0] = SE3(Vec3(0.0, 0.0, 0.05)) * demoTask->TcurRobot;
	//Ttemps[1] = SE3(Vec3(-0.05, 0.0, 0.0)) * Ttemps[0];
	//demoTask->goThroughWaypoints(Ttemps);



	qval.setZero(6);
	qval[0] = DEG2RAD(0.0);
	qval[1] = DEG2RAD(0.0);
	qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
	qval[3] = DEG2RAD(0.0);
	qval[4] = DEG2RAD(-90.0);
	qval[5] = DEG2RAD(0.0);
	rManager1->setJointVal(qval);

	//SE3 Ttemp = SE3(Vec3(0.0, 0.0, -0.2))*MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame();
	//cout << Ttemp << endl;
	int flag;
	//cout << EulerZYX(Vec3(DEG2RAD(179), DEG2RAD(-2), DEG2RAD(9)), Vec3(0.0, 0.0, 0.0)) << endl;
	
	//qval = rManager1->inverseKin(demoTask->goalSE3[0] * demoTask->goalOffset * curGraspOffset, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qval);
	//rManager1->setJointVal(qval);
	//cout << flag << endl;
	
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
	static int goalNum = 0;
	static bool sentInit = false;
	while (TRUE) 
	{
		//Receiving data from HYU client
		char* received_data = serv.RecevData();
		//strcpy(received_data, "");
		//strcat(received_data, serv.RecevData());
		communication_flag = received_data[0];

		//printf(&communication_flag);
		//cout << endl;
		//serv.SendMessageToClient("G");
		if (!sentInit)
		{
			printf("push the botton");
			getchar();
			serv.SendMessageToClient("I");
			sentInit = true;
		}
		// 데이터 전송

		if (communication_flag == 'V')
		{
			//cout << "Vision communicate called" << endl;

			// vision data
			//char* copy = (char*)malloc(sizeof(char)*(strlen(received_data) + 1));
			//for (unsigned int p = 0; p <= strlen(received_data); p++)
			//	copy[p] = received_data[p];
			//serv.SendMessageToClient(copy);
			//Sleep(50);
			printf("%s\n", received_data);
			demoTask->updateEnv(received_data);
			//demoTask->updateEnv(dummy);
			demoTask->setObjectNum();
			//////////////////////////////////////////////////////////////////////
			//m.lock();
			//m.unlock();
			//free(copy);


			bool isJobFinished = demoTask->moveJob(goalNum);
			if (isJobFinished)
				while (1)
				{
					// when dual arm robot task is finished
					break;
					/////////////////////////////////////////////
				}
			bool isReturned = demoTask->returnJob();
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

	static double JointVal = 0;
	//((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	((srStateJoint*)MHRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	JointVal += 0.01;
	//qval[5] = JointVal;
	rManager1->setJointVal(qval);
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;
	demoTask->rManager->setJointVal(demoTask->robotrrtManager->getStart());
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
