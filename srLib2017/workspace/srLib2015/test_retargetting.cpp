#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\robotTaskManager.h"
#include "Math\Spline.h"

srSpace gSpace;
myRenderer* renderer;

// Environment
TableBusbar* table;
Base* busbarBase = new Base;
vector<Jig*> jig(4);
vector<BusBar*> busbar(8);
vector<SE3>	initSE3(8);
vector<SE3>	goalSE3(8);
vector<SE3> allSE3_busbar(initSE3.size() + goalSE3.size());
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0, 0.0, 0.04));

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
robotManager* rManager1 = new robotManager;
robotManager* rManager2 = new robotManager;
robotTaskManager* rTaskManager;


// Planning
vector<vector<Eigen::VectorXd>> traj(0);
vector<vector<SE3>>	Ttraj(0);
vector<vector<int>> idxTraj(0);
vector<vector<int>> totalFlag(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
// Retargetting
vector<vector<Eigen::VectorXd>> reTraj(0);
// test
srSystem* test = new srSystem;
srSystem* test2 = new srSystem;
srRevoluteJoint* joint1;
vector<Eigen::VectorXd> qTraj0;
vector<Eigen::VectorXd> qTraj1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void environmentSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting();
void RRTSolve();
void collisionTestSetting();
void retargetting();

int main(int argc, char **argv)
{
	double a = 1.9;
	cout << std::round(a) << endl;
	//srand(time(NULL));
	// environment
	robotSetting();
	environmentSetting();
	//collisionTestSetting();
	initDynamics();

	robotManagerSetting();
	RRT_problemSetting();
	RRTSolve();

	retargetting();
	busbar[0]->setBaseLinkFrame(initSE3[0]);
	//rManager1->setJointVal(initPos[0]);
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

	for (int i = 0; i < 8; i++)
		busbar[i]->setBaseLinkFrame(initSE3[i]);
	rManager1->setJointVal(initPos[0]);

	static unsigned int step = 0;
	static int count = 0;
	count++;
	if (count % 10 == 0)
		step++;
	if (step >= qTraj1.size())
		step = 0;

	////test2->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.05, 0.8 + (double) count*0.001)));
	rManager1->setJointVal(qTraj1[step]);
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

}

void environmentSetting()
{
	SE3 tableCenter = SE3(Vec3(0.6, 0.0, 0.25));
	Vec3 tableDim(0.6, 1.2, 0.1);
	table = new TableBusbar(tableCenter, tableDim);
	gSpace.AddSystem((srSystem*)table);

	for (unsigned int i = 0; i < jig.size(); i++)
		jig[i] = new Jig;
	for (unsigned int i = 0; i < busbar.size(); i++)
		busbar[i] = new BusBar;
	SE3 Tbase = tableCenter*SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.00001)));
	vector<SE3> jigSE3(4);
	vector<SE3> jig2busbar(2);
	jig2busbar[0] = SE3(Vec3(0.00006, -0.0639, 0.0 + 0.0005));
	jig2busbar[1] = SE3(Vec3(0.0, 0.0484, 0.01 + 0.0005));
	jigSE3[0] = Tbase*SE3(Vec3(-0.1426, -0.0329, 0.01));
	jigSE3[1] = Tbase*SE3(Vec3(-0.0475, -0.0329, 0.01));
	jigSE3[2] = Tbase*SE3(Vec3(0.0475, -0.0329, 0.01));
	jigSE3[3] = Tbase*SE3(Vec3(0.1426, -0.0329, 0.01));

	busbarBase->setBaseLinkFrame(Tbase);

	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i]->setBaseLinkFrame(jigSE3[i]);
		gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			initSE3[2 * i + j] = Tbase*EulerZYX(Vec3(SR_PI*0.1*i, 0.0, 0.0), Vec3(-0.2 + 0.1*i, 0.3 + 0.1*j, 0.0));
			goalSE3[2 * i + j] = (jigSE3[i] * jig2busbar[j]);
			gSpace.AddSystem(busbar[2 * i + j]);
			busbar[2 * i + j]->setBaseLinkFrame(initSE3[2 * i + j]);
		}
	}
	gSpace.AddSystem((srSystem*)busbarBase);
}

void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	//gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.0, 0.0, 0.0)));
	//robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, 0.0), Vec3(0.5, 0.7, 0.0)));
}

void robotManagerSetting()
{
	rManager1->setRobot((srSystem*)robot1);
	rManager1->setSpace(&gSpace);
	rManager1->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);

	rTaskManager = new robotTaskManager(rManager1);

	rManager2->setRobot((srSystem*)robot2);
	rManager2->setSpace(&gSpace);
	rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);
}

void RRT_problemSetting()
{
	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}
		
	int flag;	
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, Eigen::VectorXd::Zero(6), 500, robotManager::invKinAlg::QP/*, robotManager::invKinMet::LOG*/));
	cout << initPos[0].transpose() << endl;
	printf("%d-th init inv kin flag: %d\n", 0, flag);
	for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	{
		goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, initPos[i - 1], 1000));
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
		rTaskManager->m_rrtManager->setStartandGoal(initPos[i], goalPos[i]);

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
			rTaskManager->m_rrtManager->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			rTaskManager->m_rrtManager->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = rTaskManager->m_rrtManager->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		rTaskManager->m_rrtManager->execute(0.1);
		tempTraj = rTaskManager->m_rrtManager->extractPath();
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

void collisionTestSetting()
{
	
	srLink* link0 = new srLink;
	srLink* link0_2 = new srLink;
	srLink* link = new srLink;
	srLink* link2 = new srLink;
	srCollision* coli = new srCollision;
	srCollision* coli2 = new srCollision;
	link->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link0->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	Vec3 boxdim(0.03, 0.03, 0.03);
	link->GetGeomInfo().SetDimension(boxdim);
	link2->GetGeomInfo().SetDimension(boxdim);
	coli->GetGeomInfo().SetDimension(boxdim);
	coli2->GetGeomInfo().SetDimension(boxdim);
	link0->GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	link0_2->GetGeomInfo().SetDimension(Vec3(0.0, 0.0, 0.0));
	link->AddCollision(coli);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link0_2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link2->AddCollision(coli2);
	joint1 = new srRevoluteJoint;
	srWeldJoint* joint0 = new srWeldJoint;
	srWeldJoint* joint0_2 = new srWeldJoint;
	joint0->SetParentLink(link0);
	joint0->SetParentLinkFrame(SE3());
	joint0->SetChildLink(link);
	joint0->SetChildLinkFrame(SE3());

	joint0_2->SetParentLink(link0_2);
	joint0_2->SetParentLinkFrame(SE3());
	joint0_2->SetChildLink(link2);
	joint0_2->SetChildLinkFrame(SE3());

	//((srStateJoint*)joint1)->m_State.m_rValue[0] = 0.5;
	//srLink* link3 = 
	test->SetBaseLink(link0);
	test->SetBaseLinkType(srSystem::FIXED);
	link0->SetFrame(Vec3(0.0, 0.1, 0.5));
	gSpace.AddSystem(test);
	test->SetSelfCollision(true);

	test2->SetBaseLink(link0_2);
	test2->SetBaseLinkType(srSystem::FIXED);
	link0_2->SetFrame(Vec3(0.0, 0.0, 0.8));
	gSpace.AddSystem(test2);
	test2->SetSelfCollision(true);
}

void retargetting()
{
	reTraj.resize(Ttraj.size());
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;
	for (unsigned int i = 0; i < Ttraj.size(); i++)
	{
		objNum = i / 2;
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
			rTaskManager->m_rrtManager->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			rTaskManager->m_rrtManager->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}
		vector<int> flag(0);
		rTaskManager->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
		reTraj.push_back(rTaskManager->retargetting(Ttraj[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag));
		totalFlag.push_back(flag);
	}
	
}
