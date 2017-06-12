#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager/IndyRobot.h"
#include "robotManager\UR3Robot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\robotRRTManager.h"


// Robot
UR3Robot* URRobot = new UR3Robot;
UR3RobotManager* rManager1;

BusBar_HYU* busbar = new BusBar_HYU;
Insert* ctCase = new Insert;
SE3 Tbusbar2gripper_ur = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.025));
SE3 Tbusbar2gripper_tight = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
SE3 TctCase2gripper_ur = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.015));

Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;


robotManager* rManager2;
robotRRTManager* RRTManager = new robotRRTManager;
srLink* ee = new srLink;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);

int main(int argc, char **argv)
{

    URrobotSetting();

	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	ee->GetGeomInfo().SetDimension(0.01);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs);
	gSpace.AddSystem(busbar);
	gSpace.AddSystem(ctCase);
	initDynamics();

	
	URrobotManagerSetting();

	rManager1->setGripperDistance(0.0);
	//busbar->setBaseLinkFrame(URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_ur));
	//ctCase->setBaseLinkFrame(URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Inv(TctCase2gripper_ur));
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	rManager1->setJointVal(qval);
	obs->GetBaseLink()->SetFrame(URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame());
	busbar->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -10.0)));
	ctCase->setBaseLinkFrame(EulerZYX(Vec3(0.0, SR_PI_HALF, 0.0), Vec3(0.0, 0.3, 0.35)));
	int flag;
	Eigen::VectorXd goal = rManager1->inverseKin(ctCase->getBaseLinkFrame() * TctCase2gripper_ur, &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag);
	cout << flag << endl;
	rManager1->setJointVal(goal);

	URrrtSetting();
	
	RRTManager->setStartandGoal(qval, goal);

	cout << RRTManager->checkFeasibility(qval) << RRTManager->checkFeasibility(goal);
	//if (!RRTManager->checkFeasibility(goal) && !RRTManager->checkFeasibility(qval))
	//{
	//	RRTManager->execute(0.1);
	//	traj = RRTManager->extractPath();

	//}

	rManager1->setJointVal(qval);
	
	//for (int i = 0; i < 6; i++)
	//{
	//	((srStateJoint*)URRobot->m_KIN_Joints[i])->m_State.m_rValue[0] = qval[i];
	//}
	SE3 Tend = Trobotbase1 % URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame();
	cout << Tend << endl;
	cout << Tend[9] << endl << Tend[10] << endl << Tend[11] << endl;
	SE3 T = SE3();
	T.SetOrientation(Exp(Vec3(0.0, SR_PI / sqrt(2), -SR_PI / sqrt(2))));
	cout << Log( Tend.GetOrientation()) << endl;


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
	//((srStateJoint*)URRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	JointVal += 0.01;

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);

	
	//cout << URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << URRobot->gLink[UR3_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);

	// check inv dyn with gripper
	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -1.5;
	//gripInput[1] = 1.5;
	//rManager1->setGripperInput(gripInput);
	//rManager2->setGripperInput(gripInput);
	////cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	//Eigen::VectorXd acc(6);
	//acc.setRandom();
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager1->controlJointAcc(acc);
	//rManager2->controlJointTorque(tau);

	//cout << "robot2 acc:  " << rManager2->getJointAcc().transpose() << endl << endl;
	////cout << "robot1 tau:  " << rManager1->getJointTorque().transpose() << endl;
	//cout << "desired acc: " << acc.transpose() << endl;
	////cout << "desired tau: " << tau.transpose() << endl;


	// check sensor val
	//Eigen::VectorXd acc(6);
	//acc.setZero();
	//rManager1->controlJointAcc(acc);
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager2->controlJointTorque(tau);
	//cout << "Fsensor1: " << rManager1->readSensorValue() << endl;
	//cout << "Fext1:    " << rManager1->m_ftSensorInfo[0]->getExtForce() << endl;
	//cout << "Fsensor2: " << rManager2->readSensorValue() << endl;
	//cout << "Fext2:    " << rManager2->m_ftSensorInfo[0]->getExtForce() << endl;
	int stop = 1;
}


void URrobotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	URRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	URRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = URRobot->GetBaseLink()->GetFrame() * URRobot->TsrLinkbase2robotbase;
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

void URrobotManagerSetting()
{

	//rManager2 = new robotManager();
	//rManager2->setRobot((srSystem*)robot2);
	//rManager2->setSpace(&gSpace);
	//rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);

	//// gripper setting
	//vector<srJoint*> gripperJoint(2);
	//gripperJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L];
	//gripperJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U];
	//vector<srJoint*> gripperDummyJoint(2);
	//gripperDummyJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	//gripperDummyJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	//rManager2->setGripper(gripperJoint, gripperDummyJoint);

	//// sensor setting
	//rManager2->setFTSensor(robot2->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);

	rManager1 = new UR3RobotManager(URRobot, &gSpace);

}

void URrrtSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(6);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)URRobot->gJoint[i];
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(URRobot->getLowerJointLimit(), URRobot->getUpperJointLimit());
}