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
#include "common\dataIO.h"

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
vector<Eigen::VectorXd> reTraj2(0);
vector<int> reIdxTraj(0);
vector<Eigen::VectorXd> reTraj3(0);
vector<int> flags(0);
int objIdx;
srSystem* sph = new srSystem;
vector<vector<SE3>> busbarTrajSet(0);
vector<vector<Eigen::VectorXd>> reTrajSet(0);
vector<vector<Eigen::VectorXd>> tempTrajSet(0);

// test
srSystem* test = new srSystem;
srSystem* test2 = new srSystem;
srRevoluteJoint* joint1;
vector<Eigen::VectorXd> qTraj0;
vector<Eigen::VectorXd> qTraj1;

cubicSpline* cSpline = new cubicSpline;
vector<SE3> busbarTraj;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFunc2();
void environmentSetting();
void robotSetting();
void robotManagerSetting();
vector<Eigen::VectorXd> invkintraj(vector<SE3> Ttraj);
void RRT_problemSetting();
void RRTSolve();
void collisionTestSetting();
vector<SE3> generateBusbarTraj(int idx);

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


	// single retargetting
	//objIdx = 2;			// 0~2
	//busbarTraj = generateBusbarTraj(objIdx);
	//rTaskManager->attachObject(busbar[objIdx], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
	//reTraj2 = rTaskManager->retargetting(busbarTraj, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, Inv(Tbusbar2gripper), flags, Eigen::VectorXd::Zero(6));
	//
	
	// multi retargetting
	int numObj = 2;
	// generate obj trj
	for (int i = 0; i < numObj; i++)
	{
		busbarTraj = generateBusbarTraj(i);
		busbarTrajSet.push_back(busbarTraj);
		cout << i << endl;
		cout << busbarTraj[0] << endl;
		cout << busbarTraj[busbarTraj.size() - 1] << endl;
	}
	// generate retarget trj
	vector<Eigen::VectorXd>  tempTraj(0);
	for (int i = 0; i < numObj; i++)
	{
		rTaskManager->attachObject(busbar[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
		tempTraj = rTaskManager->retargetting(busbarTrajSet[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, Inv(Tbusbar2gripper), flags, Eigen::VectorXd::Zero(6));
		reTrajSet.push_back(tempTraj);
		cout << i << endl;
		cout << busbarTrajSet[i][0] << endl;
		cout << busbarTrajSet[i][busbarTrajSet[i].size() - 1] << endl;
	}
	// generate intermediate planning trj
	rTaskManager->detachObject();
	vector<bool> feas(2);
	for (int i = 0; i < numObj - 1; i++)
	{
		reTraj2.insert(reTraj2.end(), reTrajSet[i].begin(), reTrajSet[i].end());
		for (unsigned int j = 0; j < reTrajSet[i].size(); j++)
			reIdxTraj.push_back(i);
		for (int j = 0; j < numObj; j++)
		{
			if (j < i + 1)
				busbar[j]->setBaseLinkFrame(busbarTrajSet[j][busbarTrajSet[j].size() - 1]);
			else
				busbar[j]->setBaseLinkFrame(busbarTrajSet[j][0]);
		}
		feas = rTaskManager->m_rrtManager->checkFeasibility(reTrajSet[i][reTrajSet[i].size() - 1], reTrajSet[i + 1][0]);
		cout << feas[0] << feas[1] << endl;
		rManager1->setJointVal(reTrajSet[i][reTrajSet[i].size() - 1]);
		//rTaskManager->m_rrtManager->setStartandGoal(reTrajSet[i][reTrajSet[i].size() - 1], reTrajSet[i + 1][0]);
		//rTaskManager->m_rrtManager->execute(0.1);
		//tempTraj = rTaskManager->m_rrtManager->extractPath();
		//reTraj2.insert(reTraj2.end(), tempTraj.begin(), tempTraj.end());
		//for (unsigned int j = 0; j < reTrajSet[i].size(); j++)
		//	reIdxTraj.push_back(100);
	}
	//reTraj2.insert(reTraj2.end(), reTrajSet[numObj - 1].begin(), reTrajSet[numObj - 1].end());
	//for (unsigned int j = 0; j < reTrajSet[numObj - 1].size(); j++)
	//	reIdxTraj.push_back(numObj - 1);
	
	//reTraj3 = invkintraj(busbarTraj);

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
	renderer->setUpdateFunc(updateFunc2);

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
	static unsigned int step = 0;

	static int count = 0;
	static int taskCount = 0;
	static int taskTrajNum = 0;

	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	if (step == 0)
	{
		for (int i = 0; i < 8; i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
	}

	Eigen::VectorXd gripperPos(2);
	gripperPos[0] = 0.009;
	gripperPos[1] = -0.009;
	rManager1->setGripperPosition(gripperPos);

	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
		sph->GetBaseLink()->SetFrame(SE3(Vec3(1.0, 0.0, 0.7)));
	else
		sph->GetBaseLink()->SetFrame(SE3(Vec3(1.0, 0.0, 5.7)));
	count++;
	if (count % 4 == 0)
	{
		step++;
	}
	if (step >= busbarTraj.size())
		step = 0;

	busbar[objIdx]->setBaseLinkFrame(busbarTraj[step]);
	
	//rManager1->setJointVal(reTraj2[step]);
	
	
	
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
	SE3 Tbase = tableCenter*SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.001)));
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
			busbar[2 * i + j]->SetBaseLinkType(srSystem::DYNAMIC);
		}
	}
	gSpace.AddSystem((srSystem*)busbarBase);
	busbar[0]->m_ObjLink[0].GetGeomInfo().SetColor(0.3f, 0.0f, 0.0f, 1.0f);
	busbar[1]->m_ObjLink[0].GetGeomInfo().SetColor(0.0f, 0.3f, 0.0f, 1.0f);
	busbar[2]->m_ObjLink[0].GetGeomInfo().SetColor(0.0f, 0.0f, 0.3f, 1.0f);

	// sphere
	srLink* sphlink = new srLink();
	sphlink->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	sphlink->GetGeomInfo().SetColor(0.3f, 0.0f, 0.0f, 1.0f);
	sphlink->GetGeomInfo().SetDimension(0.05);
	sph->SetBaseLink(sphlink);
	sphlink->SetFrame(SE3(Vec3(1.0, 0.0, 0.7)));
	gSpace.AddSystem(sph);

}

void robotSetting()
{

	robot1->SetActType(srJoint::HYBRID);
	robot1->SetGripperActType(srJoint::TORQUE);
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
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	rManager1->setGripper(gripperJoint);

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

vector<SE3> generateBusbarTraj(int idx)
{
	vector<double> knot(0);
	vector<Eigen::VectorXd> controlPoint(0);
	int nKnot = 4;
	double t0 = 0.0;
	double tf = 1.0;
	int nStep = 50;

	Vec3 initEuler;
	Vec3 finalEuler(0.0, 0.0, 0.0);
	Vec3 initPos;
	Vec3 finalPos = goalSE3[idx].GetPosition();
	vector<double> time(nStep);
	Eigen::VectorXd tempvec;
	knot.resize(nKnot);
	controlPoint.resize(nKnot);
	for (int i = 0; i < nKnot; i++)
	{
		knot[i] = (double)(tf - t0) * i / (nKnot - 1) + t0;
		controlPoint[i] = Eigen::VectorXd::Zero(6);
	}
		
	if (idx == 0)
	{
		initEuler = Vec3(0.0, -0.5*SR_PI_HALF, 0.0);
		initPos = Vec3(0.5, -0.3, 0.45);
	}
	else if (idx == 1)
	{
		initEuler = Vec3(SR_PI_HALF, -0.5*SR_PI_HALF, 0.0);
		initPos = Vec3(0.5, 0.3, 0.45);
	}
	else
	{
		initEuler = Vec3(SR_PI*0.1, 0.0, 0.0);
		initPos = initSE3[idx].GetPosition();
	}
	SE3 Tinit = EulerZYX(initEuler, initPos);
	int flag;
	//Eigen::VectorXd q = rManager1->inverseKin(Tinit, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, Inv(Tbusbar2gripper), flag, Eigen::VectorXd::Zero(6), 500, robotManager::invKinAlg::QP);
	//rManager1->setJointVal(q);
	//cout << q.transpose() << endl;
	for (int i = 0; i < nKnot; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			controlPoint[i][j] = initEuler[j] * (double) (nKnot - 1 - i) / (nKnot - 1) + finalEuler[j] * (double)i / (nKnot - 1);
			controlPoint[i][j+3] = initPos[j] * (double)(nKnot - 1 - i) / (nKnot - 1) + finalPos[j] * (double)i / (nKnot - 1);
		}
	}

	if (idx == 0)
	{
		controlPoint[1].tail(3) << 0.3, -0.15, 0.45;
		controlPoint[2].tail(3) << 0.5, 0.15, 0.45;
	}
	else if (idx == 1)
	{
		controlPoint[1].tail(3) << 0.3, -0.15, 0.45;
		controlPoint[2].tail(3) << 0.5, 0.15, 0.45;
	}
	else
	{
		controlPoint[1].tail(3) << 0.3, 0.15, 0.65;
		controlPoint[2].tail(3) << 0.5, -0.15, 0.45;
	}
	cSpline->interpolation(knot, controlPoint);
	vector<SE3> _busbarTraj;
	_busbarTraj.resize(0);
	for (unsigned int i = 0; i < nStep; i++)
	{
		time[i] = (double)(tf - t0) * i / (nStep - 1) + t0;
		tempvec = cSpline->getPosition(time[i]);
		//cout << tempvec.transpose() << endl;
		_busbarTraj.push_back(EulerZYX(Vec3(tempvec[0], tempvec[1], tempvec[2]), Vec3(tempvec[3], tempvec[4], tempvec[5])));
	}
	printf("busbar trajectory generated...\n");
	return _busbarTraj;
}

vector<Eigen::VectorXd> invkintraj(vector<SE3> Ttraj)
{
	int flag;
	vector<Eigen::VectorXd> traj(0);
	vector<int> flagss(0);
	for (unsigned int i = 0; i < Ttraj.size(); i++)
	{
		traj.push_back(rManager1->inverseKin(Ttraj[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, Inv(Tbusbar2gripper), flag, Eigen::VectorXd::Zero(6), 500, robotManager::invKinAlg::QP));
		flagss.push_back(flag);
	}
		
	
	return traj;
}

void updateFunc2()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	//static unsigned int step = 0;

	//static int count = 0;
	//static int taskCount = 0;
	//static int taskTrajNum = 0;


	//Eigen::VectorXd gripperPos(2);
	//gripperPos[0] = 0.009;
	//gripperPos[1] = -0.009;
	//rManager1->setGripperPosition(gripperPos);

	//if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	//	sph->GetBaseLink()->SetFrame(SE3(Vec3(1.0, 0.0, 0.7)));
	//else
	//	sph->GetBaseLink()->SetFrame(SE3(Vec3(1.0, 0.0, 5.7)));
	//count++;
	//if (count % 4 == 0)
	//{
	//	step++;
	//}


	//if (step >= reTraj2.size())
	//	step = 0;
	//if (step == 0)
	//{
	//	for (int i = 0; i < 3; i++)
	//		busbar[i]->setBaseLinkFrame(busbarTrajSet[i][0]);
	//	for (int i = 3; i < 8; i++)
	//		busbar[i]->setBaseLinkFrame(initSE3[i]);
	//}

	//if (reIdxTraj[step] != 100)
	//	busbar[reIdxTraj[step]]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
	//rManager1->setJointVal(reTraj2[step]);

	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}
