#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include "robotManager\environment_4th.h"
#include <time.h>
#include "common\dataIO.h"

// Robot
UR5Robot* URRobot = new UR5Robot;
UR5RobotManager* rManager1;

// objects
//Bin* bin = new Bin(0.01);
//BlueMaleConnector* blueMaleConnector = new BlueMaleConnector();
//BlueFemaleConnector* blueFemaleConnector = new BlueFemaleConnector();
//RedFemaleConnector* redFemaleConnector = new RedFemaleConnector();
//RedMaleConnector* redMaleConnector = new RedMaleConnector();
//TableRetarget* tableRetarget = new TableRetarget();

//workingObject* workingObj = new workingObject();


Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

//srLink* ee = new srLink;
//srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URRobotSetting();
void URRobotManagerSetting();
void URrrtSetting();
int activeJointIdx = 0;
vector<Eigen::VectorXd> traj(0);

int main(int argc, char **argv)
{



	
	URRobotSetting();

	//ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	//ee->GetGeomInfo().SetDimension(0.03);
	//ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	//obs->SetBaseLink(ee);
	//obs->SetBaseLinkType(srSystem::FIXED);
	//gSpace.AddSystem(obs);
	//gSpace.AddSystem(bin);
	//gSpace.AddSystem(redFemaleConnector);
	//gSpace.AddSystem(tableRetarget);
	initDynamics();


	//URRobotManagerSetting();

	//busbar->setBaseLinkFrame(MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_ur));
	//ctCase->setBaseLinkFrame(MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() * Inv(TctCase2gripper_ur));
	//qval.setZero(6);
	//qval[0] = SR_PI_HALF/3;
	//qval[1] = SR_PI_HALF/4;
	//qval[2] = SR_PI_HALF/7;
	//qval[3] = SR_PI_HALF/2;
	//qval[4] = SR_PI_HALF/6;
	//qval[5] = SR_PI_HALF/5;
	//rManager1->setJointVal(qval);
	//obs->GetBaseLink()->SetFrame(URRobot->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame());
	//bin->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));
	//tableRetarget->setBaseLinkFrame(SE3(SO3(), Vec3(0, 0, -1)));

	int flag;

	//rManager1->setJointVal(qval);

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

	//URRobot->gJoint[UR5_Index::JOINT_6]->m_State.m_rValue[0] += 0.01;
	//URRobot->gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->m_State.m_rValue[0] += 0.01;
	//URRobot->gGripJoint[UR5_Index::GRIPJOINT_Pal_1]->m_State.m_rValue[0] += 0.01;
	//URRobot->gGripJoint[UR5_Index::GRIPJOINT_B_1]->m_State.m_rValue[0] += 0.01;
	//URRobot->gGripJoint[UR5_Index::GRIPJOINT_P_1]->m_State.m_rValue[0] += 0.01;
	//URRobot->gGripJoint[UR5_Index::GRIPJOINT_M_1]->m_State.m_rValue[0] += 0.01;

	//static double JointVal = 0;
	//((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)URRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	//obs->GetBaseLink()->SetFrame(URRobot->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame());
	//static int cnt = 0;
	//static int trajcnt = 0;
	//cnt++;

	//if (cnt % 10 == 0)
	//	trajcnt++;
	//if (traj.size() > 0)
		//rManager1->setJointVal(traj[trajcnt % traj.size()]);


	//cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << MHRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
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


void URRobotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
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

void URRobotManagerSetting()
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

	rManager1 = new UR5RobotManager(URRobot, &gSpace);

}
