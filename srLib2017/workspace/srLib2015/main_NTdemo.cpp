#include <cstdio>

#include "myRenderer.h"
#include "NTdemoEnvSetting_4th.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager\MH12Robot.h"
#include <time.h>

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

int main(int argc, char **argv)
{
	////////////////////////////////////////////////////////////////
	////////////////////// initialize //////////////////////////////
	////////////////////////////////////////////////////////////////
	// set the number of objects in demoEnvirionment function (Since object data is now using dummy data, set the number as 1)
	demoEnv = new demoEnvironment(1);
	// add robot to system
    MHRobotSetting();
	// add bin and objects to system
	envSetting();
	initDynamics();
	MHRobotManagerSetting();
	demoTask = new demoTaskManager(demoEnv, rManager1);

	qval.setZero(6);
	qval[0] = DEG2RAD(0.0);
	qval[1] = DEG2RAD(0.0);
	qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
	qval[3] = DEG2RAD(0.0);
	qval[4] = DEG2RAD(-93.472);
	qval[5] = DEG2RAD(0.08);
	//rManager1->setJointVal(qval);
	cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	int flag;
	cout << EulerXYZ(Vec3(DEG2RAD(179), DEG2RAD(-2), DEG2RAD(9)), Vec3(0.0, 0.0, 0.0)) << endl;
	// set object SE(3) from text
	//demoEnv->setObjectFromRobot2ObjectText("C:/Users/snurobotics/Documents/판단지능/4차년도/PoseData180504/data00/Pose.txt", false);

	// dummy for front side
	char dummy_oneobj[] = "d0.0d0.0d1.0d1.0d0.0d0.0d0.0d-1.0d0.0d0.0d0.0d-1.0d1d-0.02d0.0d0.0036d";

	// dummy for back side
	//char dummy_oneobj[] = "d0.0d0.0d1.0d1.0d0.0d0.0d0.0d1.0d0.0d0.0d0.0d1.0d1d-0.02d0.0d-0.0004d";

	char dummy[1000];
	strcpy(dummy, "Vd0"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "1"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "2"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "3"); strcat(dummy, dummy_oneobj);
	strcat(dummy, "4"); strcat(dummy, dummy_oneobj);
	
	demoTask->updateEnv(dummy);
	demoTask->setObjectNum();

	bool collision = rManager1->checkCollision();

	if (~collision)
		cout << "collision free!" << endl;
	else
		cout << "collision OCCURED!" << endl;


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
	//((srStateJoint*)MHRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	//qval[5] = JointVal;
	//rManager1->setJointVal(qval);
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
