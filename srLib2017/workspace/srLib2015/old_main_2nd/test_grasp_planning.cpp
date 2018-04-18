#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\robotRRTManager.h"


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
SE3 Tbusbar2gripper = EulerZYX(Vec3(0.0, SR_PI, 0.0), Vec3(0.0, 0.0, 0.02));

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
robotManager* rManager1 = new robotManager;
robotManager* rManager2 = new robotManager;

// Planning
robotRRTManager* RRTManager = new robotRRTManager;
vector<vector<Eigen::VectorXd>> traj;
vector<vector<Eigen::VectorXd>> trajAcc;
vector<vector<int>> idxTraj(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);

// Grasping
Eigen::VectorXd gripForce(2);

// test
srRevoluteJoint* joint1;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFunc2();
void environmentSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting();
void RRTSolve();
void collisionTestSetting();
vector<Eigen::VectorXd> releaseGripTraj(int nStep);
vector<Eigen::VectorXd> joint2Acc(vector<Eigen::VectorXd> trajJoint);



int main(int argc, char **argv)
{
	gripForce[0] = 100.0;
	gripForce[1] = -100.0;

	// environment
	environmentSetting();
	robotSetting();

	initDynamics();

	robotManagerSetting();
	RRT_problemSetting();

	//rManager1->setJointVal(Eigen::VectorXd::Zero(6));
	//bool isInit = true;
	////isInit = true;
	////int objNum = 2;
	//for (int objNum = 0; objNum < 8; objNum++)
	//{
	//	for (int j = 0; j < 8; j++)
	//	{
	//		if (isInit)
	//			busbar[j]->setBaseLinkFrame(initSE3[j]);
	//		else
	//			busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	}


	//	if (isInit)
	//		rManager1->setJointVal(initPos[objNum]);
	//	else
	//		rManager1->setJointVal(goalPos[objNum]);


	//	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	//}

	RRTSolve();

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

	//rManager1->setJointVal(goalPos[0]);
	/*for (int i = 0; i < 8; i++)
	busbar[i]->setBaseLinkFrame(goalSE3[i]);*/
	static vector<Eigen::VectorXd> releaseTraj;
	static int count = 0;
	static int graspCount = 0;
	static int graspCountNum = 10;
	static int releaseCount = 0;
	static int releaseCountNum = 200;
	static int taskCount = 0;

	static bool graspFinished = false;
	static bool releaseFinished = false;
	static bool moveFinished = false;
	static bool initialized = false;

	static unsigned int trajStep = 0;

	if (taskCount >= traj.size())
	{
		taskCount = 0;
		count = 0;
		initialized = false;
	}
		

	if (count == 0)
	{
		rManager1->setJointVal(traj[0][0]);
		for (int i = 0; i < 8; i++)
			busbar[i]->setBaseLinkFrame(initSE3[i]);
	}
	count++;

	if (idxTraj[taskCount][0] != 100) // move the object with grasping
	{
		// If moving object, grasp first
		// check if grasp is finished
		if (!graspFinished && graspCount >= graspCountNum)
		{
			graspFinished = true;
			moveFinished = false;
			trajStep = 0;
		}
		if (!graspFinished)
		{
			rManager1->setGripperInput(gripForce);
			graspCount++;	
		}
		// grasp finished, move object
		// check if move is finished
		if (!moveFinished && trajStep >= trajAcc[taskCount].size())
		{
			moveFinished = true;
			releaseFinished = false;
			releaseCount = 0;
			taskCount++;
		}
		if (graspFinished && !moveFinished)
		{
			// move the object
			if (!initialized)
			{
				//rManager1->setJointValVel(traj[0][0], (traj[0][1] - traj[0][0]) / gSpace.m_Timestep_dyn_fixed);
				//rManager1->setJointVal(traj[0][0]);
				initialized = true;
			}
			rManager1->setGripperInput(gripForce);
			//rManager1->setJointVal(traj[taskCount][trajStep++]);
			//Eigen::VectorXd tempAcc(6);
			//tempAcc.setZero();
			//tempAcc[2] = -0.10;
			//rManager1->controlJointAcc(tempAcc);
			rManager1->controlJointAcc(trajAcc[taskCount][trajStep++]);
		}
	}
	else // move robot to another obejct without grasping
	{
		// release object and move to next object
		// check if release is finished
		if (!releaseFinished && releaseCount >= releaseCountNum)
		{
			releaseFinished = true;
			moveFinished = false;
			trajStep = 0;
		}
		if (!releaseFinished)
		{
			// release first
			if (releaseCount == 0)
				releaseTraj = releaseGripTraj(releaseCountNum);
			rManager1->setGripperPosition(releaseTraj[releaseCount++]);
		}
		// release finshed, move to next object
		// check if move is finished
		if (!moveFinished && trajStep >= traj[taskCount].size())
		{
			moveFinished = true;
			graspFinished = false;
			graspCount = 0;
			taskCount++;
		}
		if (releaseFinished && !moveFinished)
		{
			rManager1->setJointVal(traj[taskCount][trajStep++]);
		}
	}
	
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
			busbar[2 * i + j]->SetBaseLinkType(srSystem::DYNAMIC);
		}
	}
	gSpace.AddSystem((srSystem*)busbarBase);
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
	rManager1->setEndeffector(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP], actType);
	vector<srJoint*> gripperJoint(2);
	gripperJoint[0] = robot1->gPjoint[Indy_Index::GRIPJOINT_L];
	gripperJoint[1] = robot1->gPjoint[Indy_Index::GRIPJOINT_U];
	rManager1->setGripper(gripperJoint);

	rManager2->setRobot((srSystem*)robot2);
	rManager2->setSpace(&gSpace);
	rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP], actType);
}

void RRT_problemSetting()
{
	for (unsigned int i = 0; i < initSE3.size(); i++)
	{
		allSE3_busbar[2 * i] = initSE3[i];
		allSE3_busbar[2 * i + 1] = goalSE3[i];
	}

	int flag;
	initPos.push_back(rManager1->inverseKin(allSE3_busbar[0] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, Eigen::VectorXd::Zero(6), 500, robotManager::invKinAlg::QP));
	cout << initPos[0].transpose() << endl;
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
	vector<srStateJoint*> stateJoints(6);
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	int start = 0;		//  >= 0
	int end = 1;		// <= 15
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;
	for (int i = 0; i < 6; i++)
	{
		stateJoints[i] = rManager1->m_activeArmInfo->getActiveStateJoints()[i];
		//stateJoints[6 + i] = rManager2->m_activeArmInfo->getActiveStateJoints()[i];
	}
	RRTManager->setSpace(&gSpace);
	RRTManager->setSystem(stateJoints);
	RRTManager->setStateBound(-SR_TWO_PI*Eigen::VectorXd::Ones(nDim), SR_TWO_PI*Eigen::VectorXd::Ones(nDim));
	traj.resize(0);
	trajAcc.resize(0);
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
		RRTManager->execute(0.01);
		tempTraj = RRTManager->extractPath();

		traj.push_back(tempTraj);
		trajAcc.push_back(joint2Acc(tempTraj));

		tempIdxTraj.resize(tempTraj.size());
		for (unsigned int j = 0; j < tempIdxTraj.size(); j++)
		{
			if (isAttached)
				tempIdxTraj[j] = objNum;
			else
				tempIdxTraj[j] = 100;
		}

		idxTraj.push_back(tempIdxTraj);
	}
	//////////// test
	//RRTManager->detachObject();
	
	cout << "position------------------" << endl;
	for (int i = 0; i < trajAcc[0].size(); i++)
		cout << traj[0][i].transpose() << endl;


	cout << "acc------------------" << endl;
	for (int i = 0; i < trajAcc[0].size(); i++)
		cout << trajAcc[0][i].transpose() << endl;

}

void collisionTestSetting()
{
	srSystem* test = new srSystem;
	srLink* link0 = new srLink;
	srLink* link = new srLink;
	srLink* link2 = new srLink;
	srCollision* coli = new srCollision;
	srCollision* coli2 = new srCollision;
	link->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link2->GetGeomInfo().SetColor(0.3, 0.3, 0.3);
	link->AddCollision(coli);
	link2->AddCollision(coli2);
	joint1 = new srRevoluteJoint;
	srWeldJoint* joint0 = new srWeldJoint;

	joint0->SetParentLink(link0);
	joint0->SetParentLinkFrame(SE3());
	joint0->SetChildLink(link);
	joint0->SetChildLinkFrame(SE3());

	joint1->SetParentLink(link);
	joint1->SetParentLinkFrame(SE3());
	joint1->SetChildLink(link2);
	joint1->SetChildLinkFrame(SE3(Vec3(0.11, 0.0, 0.0)));

	((srStateJoint*)joint1)->m_State.m_rValue[0] = 0.5;
	//srLink* link3 = 
	test->SetBaseLink(link0);
	link->SetFrame(Vec3(0.0, 0.0, 0.0));
	gSpace.AddSystem(test);
	test->SetSelfCollision(true);
}



vector<Eigen::VectorXd> releaseGripTraj(int nStep)
{
	Eigen::VectorXd initialGripJoint = rManager1->getGripperPosition();
	Eigen::VectorXd finalGripJoint = Eigen::VectorXd::Zero(2);
	vector<Eigen::VectorXd> gripJointTraj(nStep);
	for (int i = 0; i < nStep; i++)
	{
		gripJointTraj[i] = initialGripJoint * (double)(nStep - 1 - i) / (nStep - 1) + finalGripJoint * (double)i / (nStep - 1);
	}
	return gripJointTraj;
}

vector<Eigen::VectorXd> joint2Acc(vector<Eigen::VectorXd> trajJoint)
{
	vector<Eigen::VectorXd> jointAcc(trajJoint.size());
	double dt_inv = 1.0 / gSpace.m_Timestep_dyn_fixed;
	for (unsigned int i = 1; i < trajJoint.size() - 1; i++)
	{
		jointAcc[i] = Eigen::VectorXd::Zero(6);
		for (int j = 0; j < 6; j++)
		{
			jointAcc[i][j] = (trajJoint[i + 1][j] - 2.0*trajJoint[i][j] + trajJoint[i - 1][j]) *dt_inv*dt_inv;
		}
	}
		

	jointAcc[0] = jointAcc[1];
	jointAcc[jointAcc.size() - 1] = jointAcc[jointAcc.size() - 2];
	return jointAcc;
}

void updateFunc2()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int taskCount = 0;
	static int trajStep = 0;

	if (trajStep == 0)
		rManager1->setJointValVel(traj[0][0], (traj[0][1] - traj[0][0]) / gSpace.m_Timestep_dyn_fixed);

	//rManager1->setJointVal(traj[taskCount][trajStep++]);

	rManager1->controlJointAcc(trajAcc[taskCount][trajStep]);
	cout << "simul: " << rManager1->getJointVal().transpose() << endl;
	cout << "plann: " << traj[taskCount][trajStep].transpose() << endl;
	trajStep++;
	if (trajStep >= traj[taskCount].size())
		trajStep = 0;

	
}
