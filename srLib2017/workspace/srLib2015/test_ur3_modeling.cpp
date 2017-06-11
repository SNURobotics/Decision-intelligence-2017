#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager/IndyRobot.h"
#include "robotManager\UR3Robot.h"
#include <time.h>
#include "robotManager\environment_QBtech.h"
#include "RRTmanager\rrtManager.h"


// Robot
UR3Robot* URRobot = new UR3Robot;
UR3RobotManager* rManager1;



Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;


robotManager* rManager2;
rrtManager* RRTManager = new rrtManager;
srLink* ee;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void robotSetting();
void robotManagerSetting();

int activeJointIdx =5;

int main(int argc, char **argv)
{

    robotSetting();

	initDynamics();

	rManager1 = new UR3RobotManager(URRobot, &gSpace);
	//robotManagerSetting();

	qval.setZero(6);

	for (int i = 0; i < 6; i++)
	{
		((srStateJoint*)URRobot->m_KIN_Joints[i])->m_State.m_rValue[0] = qval[i];
	}


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

	static double dist = 0.05;

	//dist += 0.001;

	rManager1->setGripperDistance(dist);


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


void robotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	URRobot->SetActType(srJoint::ACTTYPE::HYBRID);



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

void robotManagerSetting()
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