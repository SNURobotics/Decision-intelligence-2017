#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environment_QBtech.h"
#include "RRTmanager\rrtManager.h"

// Environment
Base* busbarBase = new Base;
Jig_QB* jig_QB = new Jig_QB;
JigAssem_QB* jigAssem = new JigAssem_QB;
Insert* insert = new Insert;
LowerFrame* lowerFrame = new LowerFrame;
UpperFrame* upperFrame = new UpperFrame;

vector<Jig*> jig(4);
vector<BusBar*> busbar(8);
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

indyRobotManager* rManager1;
robotManager* rManager2;
rrtManager* RRTManager = new rrtManager;
srLink* ee;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void environmentSetting();
void robotSetting();
void robotManagerSetting();

int main(int argc, char **argv)
{
	// environment

	//SE3 Tbase = SE3(Vec3(0.0, 0.0, 0.0));
	//upperFrame->setBaseLinkFrame(Tbase);

	//
	//gSpace.AddSystem((srSystem*)lowerFrame);
	gSpace.AddSystem((srSystem*)jigAssem);
	busbar[0] = new BusBar;
	gSpace.AddSystem((srSystem*)busbar[0]);
	jigAssem->GetBaseLink()->SetFrame(SE3(Vec3(-0.5, 0.0, 0.0)));
	
	//environmentSetting();
	robotSetting();
	
	//srSystem* test = new srSystem;
	//srLink* link = new srLink;
	//test->SetBaseLink(link);
	//link->SetFrame(Vec3(0.0, 0.0, 0.5));
	//gSpace.AddSystem(test);
	
	initDynamics();

	robotManagerSetting();

	SE3 Tgoal = SE3(Vec3(0.0, 0.0, -0.025)) * jigAssem->GetBaseLink()->GetFrame() * jigAssem->holeCenter[0];
	busbar[0]->GetBaseLink()->SetFrame(Tgoal*EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));

	cout << Tgoal << endl;
	cout << busbar[0]->GetBaseLink()->GetFrame() << endl;
	//Inertia G1(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 2.0);
	//Inertia G2(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 2.0);
	//Inertia G3 = G1.Transform(SE3(Vec3(0.7, 0.8, 0.9)));

	Eigen::VectorXd q(6);
	q.setZero();
	rManager1->setJointVal(q);
	rManager2->setJointVal(q);
	//cout << robot2->gLink[Indy_Index::ENDEFFECTOR].GetFrame() << endl;
	//cout << robot2->gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->GetFrame() << endl;
	//cout << robot2->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->GetFrame() << endl;

	Eigen::VectorXd gripPos(2);
	gripPos[0] = -0.008;
	gripPos[1] = 0.008;
	rManager1->setGripperPosition(gripPos);
	rManager2->setGripperPosition(gripPos);

	//cout << robot1->gLink[Indy_Index::SENSOR].GetFrame() << endl;
	int flag;
	qval = rManager1->inverseKin(busbar[0]->GetBaseLink()->GetFrame()*Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag);

	rManager1->setJointVal(qval);
	//jigAssem->GetBaseLink()->SetFrame(SE3());
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

void environmentSetting()
{
	for (unsigned int i = 0; i < jig.size(); i++)
		jig[i] = new Jig;
	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i] = new BusBar;
	SE3 Tbase = SE3(Vec3(0.0, 0.0, 0.0));
	vector<SE3> jigSE3(4);
	vector<SE3> jig2busbar(2);
	jig2busbar[0] = SE3(Vec3(0.00006, -0.0639, 0.0));
	jig2busbar[1] = SE3(Vec3(0.0, 0.0484, 0.01));
	jigSE3[0] = Tbase*SE3(Vec3(-0.1426, -0.0329, 0.01));
	jigSE3[1] = Tbase*SE3(Vec3(-0.0475, -0.0329, 0.01));
	jigSE3[2] = Tbase*SE3(Vec3(0.0475, -0.0329, 0.01));
	jigSE3[3] = Tbase*SE3(Vec3(0.1426, -0.0329, 0.01));
	
	busbarBase->setBaseLinkFrame(Tbase);
	

	for (unsigned int i = 0; i < jig.size(); i++)
	{
		//jig[i]->setBaseLinkFrame(jigSE3[i]);
		//gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
			gSpace.AddSystem(busbar[2 * i + j]);
		}
	}


	//gSpace.AddSystem((srSystem*)busbarBase);
}

void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF,0.0,0.0),Vec3(-0.5, 0.7, 0.0)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.5, 0.7, 0.0)));
	robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
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

	rManager2 = new indyRobotManager(robot2, &gSpace);
	rManager1 = new indyRobotManager(robot1, &gSpace);
}