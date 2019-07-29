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
#include "robotManager\robotRRTManager.h"

// Environment
Base* busbarBase = new Base;
vector<Jig*> jig(4);
Jig_QB* jigQB = new Jig_QB;
UpperFrame* uFrame = new UpperFrame;
LowerFrame* lFrame = new LowerFrame;
vector<Insert*> insert(4);
vector<BusBar*> busbar(8);
vector<SE3>	initSE3(8);
vector<SE3>	goalSE3(8);
vector<SE3> allSE3_busbar(2*busbar.size());

// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd jointVal(6);
srSpace gSpace;
myRenderer* renderer;
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));

// Planning
vector<vector<Eigen::VectorXd>> traj(0);
vector<vector<SE3>>	Ttraj(0);
vector<vector<int>> idxTraj(0);
vector<vector<int>> totalFlag(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);

robotManager* rManager1 = new robotManager;
robotManager* rManager2 = new robotManager;
robotRRTManager* RRTManager = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncPlanning();
void environmentSetting(bool connect = false);
void environmentSettingQB(bool connect = false);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting();
void RRTSolve();
void rrtSetting();


int main(int argc, char **argv)
{
	// environment
	workspaceSetting();
	environmentSettingQB(true);
	robotSetting();

	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();

	// rrt
	jointVal.setZero();
	jointVal[0] = SR_PI_HALF;
	jointVal[2] = 0.6*SR_PI_HALF;
	jointVal[4] = -0.4*SR_PI_HALF;
	jointVal[3] = SR_PI_HALF;
	rManager2->setJointVal(jointVal);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);
	rrtSetting();
	RRT_problemSetting();
	RRTSolve();
	
	//Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	//qInit[2] = 0.6*SR_PI_HALF;
	//qInit[4] = 0.6*SR_PI_HALF;
	//qInit[3] = SR_PI_HALF;

	//int flag;
	//for (int i = 0; i < 8; i++)
	//{
	//	if (i > 0)
	//		qInit = initPos[i - 1];
	//	initPos.push_back(rManager1->inverseKin(initSE3[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	//	cout << flag << endl;
	//	cout << initPos[i].transpose() << endl;
	//}

	// idx = 1
	//jointVal << 0.469173, 0.723925, 0.239127, 1.53372, 2.36015, -0.427431;
	//jointVal << 0.975131, -0.335552, 0.761938, 1.5708, 1.99718, -0.975131;
	// idx = 3
	//jointVal << 0.707913, -0.742161, 0.906805, 1.46511, 1.31366, -0.749334;	// actual colli
	//jointVal << 0.80376, -0.755397, 0.927902, 1.49777, 1.43491, -0.817374;
	//jointVal << 0.995454, -0.781868, 0.970097, 1.56308, 1.6774, -0.953454;	// actual colli
	//jointVal << 1.05788, -0.732252, 0.958045, 1.5708, 1.79659, -1.05788;
	rManager1->setJointVal(jointVal);
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
	renderer->setUpdateFunc(updateFuncPlanning);

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
	static int cnt = 0;
	alpha += 0.01;
	
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	// move stage
	//((srStateJoint*)workCell->pJoint[0])->m_State.m_rValue[0] = 0.05*sin(alpha);
	//((srStateJoint*)workCell->pJoint[1])->m_State.m_rValue[0] = 0.05*sin(2.0*alpha);
	//((srStateJoint*)workCell->rJoint[0])->m_State.m_rValue[0] = alpha;

	//test->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame());
	//test2->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame()*SE3(Vec3(0.0,0.0,0.03)));
	
	//if (cnt == 0)
	//	cout << robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;
	//cnt++;

	//static int temp = 0;
	//if (cnt % 30 == 0)
	//	temp++;
	//int idx = temp % initPos.size();
	//rManager1->setJointVal(initPos[idx]);
	int idx = 1;
	int objNum = idx / 2;

	for (unsigned int j = 0; j < goalSE3.size(); j++)
	{
		if (j < objNum)
			busbar[j]->setBaseLinkFrame(goalSE3[j]);
		else
			busbar[j]->setBaseLinkFrame(initSE3[j]);
		if (j == objNum && idx % 2 == 1)
			busbar[j]->setBaseLinkFrame(goalSE3[j]);
	}
	if (idx % 2 == 0)
		busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);
	//cout << "taskIdx: " << idx << endl;
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

}

void updateFuncPlanning()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	static int cnt = 0;
	

	static int trjIdx = 0;
	static int taskIdx = 0;

	int idx = taskIdx % traj.size();
	if (idx == 0 && cnt > 0)
		cnt = 0;
	if (cnt == 0)
	{
		for (unsigned int i = 0; i < busbar.size(); i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
		cnt++;
	}
	rManager1->setJointVal(traj[idx][trjIdx]);
	int objNum = idx / 2;

	for (unsigned int j = 0; j < goalSE3.size(); j++)
	{
		if (j < objNum)
			busbar[j]->setBaseLinkFrame(goalSE3[j]);
		else
			busbar[j]->setBaseLinkFrame(initSE3[j]);
		if (j == objNum && idx % 2 == 1)
			busbar[j]->setBaseLinkFrame(goalSE3[j]);
	}
	if (idx % 2 == 0)
		busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
/*

	if (idxTraj[idx][trjIdx] != 100)
		busbar[objNum]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);*/
	
	trjIdx++;
	if (trjIdx == traj[idx].size())
	{
		trjIdx = 0;
		taskIdx++;
		cout << "taskIdx: " << taskIdx % traj.size() << endl;
	}
	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	{
		cout << taskIdx << endl;
		cout << traj[idx][trjIdx].transpose() << endl;
	}
	//cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}

void environmentSetting(bool connect)
{
	for (unsigned int i = 0; i < jig.size(); i++)
		jig[i] = new Jig;
	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i] = new BusBar;
	SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));
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
		jig[i]->setBaseLinkFrame(jigSE3[i]);
		if (!connect)
			gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
			gSpace.AddSystem(busbar[2 * i + j]);
		}
	}
	if (!connect)
		gSpace.AddSystem((srSystem*)busbarBase);
	else
	{
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(busbarBase->GetBaseLink());
		wJoint->SetParentLinkFrame(Tbase);
		wJoint->SetChildLinkFrame(SE3());
		vector<srWeldJoint*> wJoints(jig.size());
		for (unsigned int i = 0; i < jig.size(); i++)
		{
			wJoints[i] = new srWeldJoint;
			wJoints[i]->SetParentLink(workCell->getStagePlate());
			wJoints[i]->SetChildLink(jig[i]->GetBaseLink());
			wJoints[i]->SetParentLinkFrame(jigSE3[i]);
			wJoints[i]->SetChildLinkFrame(SE3());
		}
	}

}

void environmentSettingQB(bool connect)
{
	SE3 Tbase = SE3(Vec3(0.025, 1.095, 1.176));
	SE3 uFrameSE3 = Tbase * SE3(Vec3(0.0, 0.0, 0.2001));
	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i] = new BusBar;
	SE3 lFrameSE3 = uFrameSE3 * EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, -0.0));
	vector<SE3> lower2insert(4);
	double z = 0.05;
	lower2insert[0] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0927, 0.14125, 0.0105));
	lower2insert[1] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0927, 0.04525, 0.0105));
	lower2insert[2] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0927, -0.04675, 0.0105));
	lower2insert[3] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0927, -0.14125, 0.0105));
	for (unsigned int i = 0; i < insert.size(); i++)
	{
		insert[i] = new Insert();
		insert[i]->GetBaseLink()->SetFrame(lFrameSE3*lower2insert[i]);
		gSpace.AddSystem(insert[i]);
	}
	gSpace.AddSystem(lFrame);
	lFrame->GetBaseLink()->SetFrame(lFrameSE3);
	gSpace.AddSystem(uFrame);
	uFrame->GetBaseLink()->SetFrame(uFrameSE3);
	//gSpace.AddSystem(jigQB);
	//jigQB->GetBaseLink()->SetFrame(uFrameSE3 * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.03635, 0.05)));
	SE3 insert2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.006, 0.0, 0.0245 + 0.0001));
	SE3 insert2busbar2 = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(-0.11529, 0.0, 0.0245 + 0.0001));
	for (int i = 0; i < 4; i++)
	{
		goalSE3[2 * i] = lFrameSE3*lower2insert[i] * insert2busbar;
		goalSE3[2 * i + 1] = lFrameSE3*lower2insert[i] * insert2busbar2;
		initSE3[2 * i] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(goalSE3[2 * i].GetPosition()[0], 0.604, 1.1111 + 0.0001));
		initSE3[2 * i + 1] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(goalSE3[2 * i + 1].GetPosition()[0], 0.684, 1.1111 + 0.0001));
		busbar[2 * i]->GetBaseLink()->SetFrame(initSE3[2 * i]);
		busbar[2 * i + 1]->GetBaseLink()->SetFrame(initSE3[2 * i + 1]);
		gSpace.AddSystem(busbar[2 * i]);
		gSpace.AddSystem(busbar[2 * i + 1]);
	}
}


void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005, 1.972)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005, 1.972)));
	robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	robot2->SetActType(srJoint::ACTTYPE::HYBRID);
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
	// robot 1
	rManager1->setRobot((srSystem*)robot1);
	rManager1->setSpace(&gSpace);
	rManager1->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	vector<srJoint*> gripperDummyJoint(2);
	gripperDummyJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	gripperDummyJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	rManager1->setGripper(gripperJoint, gripperDummyJoint);

	// robot 2
	rManager2->setRobot((srSystem*)robot2);
	rManager2->setSpace(&gSpace);
	rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);
	gripperJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U];
	gripperDummyJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	gripperDummyJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	rManager2->setGripper(gripperJoint, gripperDummyJoint);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}

void rrtSetting()
{
	vector<srStateJoint*> planningJoints(6);
	for (unsigned int i = 0; i < planningJoints.size(); i++)
		planningJoints[i] = (srStateJoint*) robot1->gJoint[i];
	RRTManager->setSystem(planningJoints);
	RRTManager->setSpace(&gSpace);
	RRTManager->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
}


void RRT_problemSetting()
{
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	qInit[2] = 0.6*SR_PI_HALF;
	qInit[4] = 0.6*SR_PI_HALF;
	qInit[3] = SR_PI_HALF;
	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);
	for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	{
		qInit = initPos[i - 1];
		goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
		cout << goalPos[i - 1].transpose() << endl;
		initPos.push_back(goalPos[i - 1]);
		if (i % 2 == 0)
			printf("%d-th init inv kin flag: %d\n", i / 2, flag);
		else
			printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	}
}

void RRTSolve()
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = 15;		// <= 15
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		objNum = i / 2;
		RRTManager->setStartandGoal(initPos[i], goalPos[i]);

		for (unsigned int j = 0; j < goalSE3.size(); j++)
		{
			if (j < objNum)
				busbar[j]->setBaseLinkFrame(goalSE3[j]);
			else
				busbar[j]->setBaseLinkFrame(initSE3[j]);
		}
		if (i % 2 == 0)
		{
			isAttached = true;
			RRTManager->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			RRTManager->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = RRTManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager->execute(0.1);
		tempTraj = RRTManager->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);

		tempIdxTraj.resize(tempTraj.size());
		for (unsigned int j = 0; j < tempIdxTraj.size(); j++)
		{
			if (isAttached)
				tempIdxTraj[j] = objNum;
			else
				tempIdxTraj[j] = 100;
		}

		idxTraj.push_back(tempIdxTraj);

		tempTtraj.resize(tempTraj.size());
		for (unsigned int i = 0; i < traj.size(); i++)
			tempTtraj[i] = rManager1->forwardKin(tempTraj[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		Ttraj.push_back(tempTtraj);
	}
}
