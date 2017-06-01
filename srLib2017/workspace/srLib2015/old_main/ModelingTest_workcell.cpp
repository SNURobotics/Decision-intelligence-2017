#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_workcell.h"
#include "RRTmanager\rrtManager.h"

// Environment
Base* busbarBase = new Base;
vector<Jig*> jig(4);
vector<BusBar*> busbar(8);

// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;

srSpace gSpace;
myRenderer* renderer;

robotManager* rManager = new robotManager;
rrtManager* RRTManager = new rrtManager;
srLink* ee;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void environmentSetting();
void workspaceSetting();
void robotSetting();
void robotManagerSetting();

int main(int argc, char **argv)
{
	// environment
	//environmentSetting();
	workspaceSetting();
	robotSetting();
	//srSystem* test = new srSystem;
	//srLink* link = new srLink;
	//test->SetBaseLink(link);
	//link->SetFrame(Vec3(0.0, 0.0, 0.5));
	//gSpace.AddSystem(test);
	
	initDynamics();


	//robotManagerSetting();

	//Inertia G1(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.0, 0.0, 0.0, 2.0);
	//Inertia G2(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 2.0);
	//Inertia G3 = G1.Transform(SE3(Vec3(0.7, 0.8, 0.9)));

	//Eigen::VectorXd q(6);
	//q.setRandom();
	//rManager->setJointVal(q);
	//cout << robot2->gLink[Indy_Index::ENDEFFECTOR].GetFrame() << endl;
	//cout << robot2->gWeldJoint[Indy_Index::WELDJOINT_SENSOR]->GetFrame() << endl;
	//cout << robot2->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]->GetFrame() << endl;

	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = 0.015;
	//gripInput[1] = -0.015;
	//rManager->setGripperPosition(gripInput);

	stageVal.setZero();
	workCell->setStageVal(stageVal);

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
	static double alpha = 0.0;
	alpha += 0.01;
	((srStateJoint*)workCell->pJoint[0])->m_State.m_rValue[0] = 0.05*sin(alpha);
	((srStateJoint*)workCell->pJoint[1])->m_State.m_rValue[0] = 0.05*sin(2.0*alpha);
	((srStateJoint*)workCell->rJoint[0])->m_State.m_rValue[0] = alpha;
	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -0.015;
	//gripInput[1] = 0.015;
	//rManager->setGripperInput(gripInput);
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

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
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005, 1.972)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005, 1.972)));
	robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void robotManagerSetting()
{
	rManager->setRobot((srSystem*)robot1);
	rManager->setSpace(&gSpace);
	rManager->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	vector<srJoint*> gripperDummyJoint(2);
	gripperDummyJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	gripperDummyJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	rManager->setGripper(gripperJoint, gripperDummyJoint);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}
