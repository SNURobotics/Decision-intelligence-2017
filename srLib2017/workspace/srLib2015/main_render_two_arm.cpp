#include <cstdio>

//#include "myRenderer.h"
#include "serverRenderer.h"
#include "simulationEnvSetting.h"
//#include "common\dataIO.h"
//#include "srDyn/srDYN.h"
//#include "srGamasot\srURDF.h"
//#include "robotManager\indyRobotManager.h"
//#include "robotManager/IndyRobot.h"
//#include <time.h>
//#include "robotManager\environmentBusbar.h"
//#include "robotManager\environment_QBtech.h"
//#include "robotManager\environment_workcell.h"
//#include "robotManager\robotRRTManager.h"
#include <fstream>
#include <iostream>

#include <stdlib.h>
#include <stdio.h>

#include "Eigen/Dense"

#include <stdlib.h>
#include <vector>

// memory leakaage check
#include <crtdbg.h>

//srLib
//srSpace gSpace;
serverRenderer* renderer;

// Environment
//JigAssem_QB_bar* jigAssem = new JigAssem_QB_bar(false);
//vector<BusBar_HYU*> busbar(8);
//vector<Insert*> ctCase(4);
//vector<Object*> objects(busbar.size() + ctCase.size());
//vector<SE3> TobjectsInitSimul(objects.size());
//bool isJigConnectedToWorkCell = true;
//SE3 initBusbar = SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.5)));

// Workspace
//int workcell_mode = 0;
//WorkCell* workCell = new WorkCell(workcell_mode);
//Eigen::VectorXd stageVal(3);
//bool useNoVisionTestSetting = true;
//bool useNoVisionTestSettingJig = true;

// Robot
//IndyRobot* robot1 = new IndyRobot(false);
//IndyRobot* robot2 = new IndyRobot(false);
//vector<IndyRobot*> robotVector(2);
//SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
//SE3 Tbusbar2gripper_tight = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
//SE3 TctCase2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.01));
//vector<SE3> Tobject2gripper(objects.size());
//SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
//SE3 Trobotbase1;
//SE3 Trobotbase2;
//vector<SE3> TrobotbaseVector(2);

//indyRobotManager* rManager1;
//indyRobotManager* rManager2;
//vector<indyRobotManager*> rManagerVector(2);
//robotRRTManager* RRTManager1 = new robotRRTManager;
//robotRRTManager* RRTManager2 = new robotRRTManager;
//vector<robotRRTManager*> RRTManagerVector(2);


// save last gripper state
int gripState = 0;
vector<int> gripState_multi(2);
vector<int> gripObjectIdx(2, -1);			// save which busbar is moving with each robot during planning
vector<int> gripObjectIdxRender(2, -1);		// save which busbar is moving with each robot during rendering
vector<Eigen::VectorXd> lastJointVal_multi(2);// save last joint value

// save initial and final busbar SE(3)
vector<SE3> TinitObjects_multi(2);
vector<SE3> TinitObjects_multiRender(2);
vector<bool> initialObjectSaved(2, false);			// save if initial busbar location is saved (saved when busbar is moving)
vector<bool> initialObjectSavedRender(2, false);
vector<SE3>	TlastObjects_multi(2);
vector<bool> initialPlanning(2, true);
vector<bool> startPlanningFromCurRobotState(2, false);
vector<bool> attachObjRender(0);
vector<vector<bool>> attachObjRender_multi(2);
vector<Eigen::VectorXd> homePosRobotVector(2);

// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj;
vector<vector<Eigen::VectorXd>> TtrajVec;
vector<vector<Eigen::VectorXd>> busbarTraj;
vector<Eigen::VectorXd> goalJigLocation(1);
bool saveTraj = true;




// modelling functions
//void workspaceSetting();
//void robotSetting();
//void environmentSetting_HYU2(bool connect);
//void objectSetting();
//void connectJigToWorkCell();
//void initDynamics();
//void robotManagerSetting();

// rendering functions
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncLoadData();

bool isSystemAssembled = false; 

vector<srLink*> obstacle(0);
vector<srWeldJoint*> wJoint(0);		// weld joint for connecting workcell and obstacle

SE3 Tjigbase;

int main(int argc, char **argv)
{
	Eigen::initParallel();
	bool useVision = false;
	if (useVision)
		useNoVisionTestSetting = false;
	srand(time(NULL));
	// Robot home position
	robotSetting();
	homePosRobotVector[0] = robot1->homePos;
	homePosRobotVector[1] = robot2->homePos;
	// environment
	workspaceSetting();
	objectSetting();
	////////////////////////////////////////////// setting environment (replacable from vision data)

	environmentSetting_HYU2(true);			// temporary environment setting
	initDynamics();								// initialize srLib				
	isSystemAssembled = true;
	robotManagerSetting();						// robot manager setting

	// workcell robot initial config
	rManager1->setJointVal(robot1->homePos);
	rManager2->setJointVal(robot2->homePos);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.005;
	gripInput[1] = 0.005;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);
	lastJointVal_multi[0] = robot1->homePos;
	lastJointVal_multi[1] = robot2->homePos;
	//cout << Trobotbase2 % robot2->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() << endl;

	
	rendering(argc, argv);


	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new serverRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode_1st(&gSpace);
	renderer->InitializeNode_2nd();
	renderer->setUpdateFunc(updateFuncLoadData);
	//renderer->setUpdateFunc(updateFunc);
	renderer->RunRendering();
}

//void initDynamics()
//{
//	gSpace.SetTimestep(0.01);
//	gSpace.SetGravity(0.0, 0.0, -10.0);
//	gSpace.SetNumberofSubstepForRendering(1);
//	gSpace.DYN_MODE_PRESTEP();
//}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

//void environmentSetting_HYU2(bool connect)
//{
//	// should be called later than workcell setting
//	SE3 Tbase;
//	if (workcell_mode == 1)
//		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
//	else if (workcell_mode == 2)
//	{
//		Vec3 stage4Trans = workCell->m_ObjWeldJoint[3].GetParentLinkFrame().GetPosition();
//		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.105 + 0.009) + stage4Trans);	// when only stage4 is used
//	}
//	else
//		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
//	double z_angle = (double)rand() / RAND_MAX * 1.0;
//	double x_trans = -(double)rand() / RAND_MAX * 0.1;
//	double y_trans = (double)rand() / RAND_MAX * 0.1;
//	//SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
//	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
//	//for (unsigned int i = 0; i < busbar.size(); i++)
//	//{
//	//	busbar[i] = new BusBar_HYU;
//	//	busbar[i]->SetBaseLinkType(srSystem::FIXED);
//	//	gSpace.AddSystem(busbar[i]);
//	//}
//	//Vec3 testJigPosFromRobot1(-0.702151, -0.014057, 0.750026);		// 17.06.09 using robot1
//	//Vec3 testJigPosFromRobot1(-0.8254, 0.0338, 0.7483);		// 17.06.10 using robot2
//	Vec3 testJigPosFromRobot1(-0.8277, - 0.0536,    0.8620);		// 17.06.10 using robot2
//	jigAssem->SetBaseLinkType(srSystem::FIXED);
//	if (!useNoVisionTestSettingJig)
//		jigAssem->setBaseLinkFrame(Tbase*Tbase2jigbase);
//	else
//	{
//		SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
//		jigAssem->setBaseLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
//	}
//		
//	if (!connect)
//		gSpace.AddSystem((srSystem*)jigAssem);
//	else
//	{
//		srWeldJoint* wJoint = new srWeldJoint;
//		wJoint->SetParentLink(workCell->GetBaseLink()); // removed stage
//		//wJoint->SetParentLink(workCell->getStagePlate());
//		wJoint->SetChildLink(jigAssem->GetBaseLink());
//		if (!useNoVisionTestSettingJig)
//			wJoint->SetParentLinkFrame(Tbase*Tbase2jigbase);
//		else
//		{
//			SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
//			SE3 Tjig = Trobotbase1 % SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset;
//			cout << "Tjig" << endl;
//			cout << Tjig << endl;
//			wJoint->SetParentLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
//		}
//		wJoint->SetChildLinkFrame(SE3());
//	}
//
//	// ctCase test
//	//ctCase[0]->setBaseLinkFrame(Tbase*Tbase2jigbase*SE3(Vec3(0.0, 0.0, 0.03)));
//}
//
//
//void robotSetting()
//{
//	gSpace.AddSystem((srSystem*)robot1);
//	gSpace.AddSystem((srSystem*)robot2);
//	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005 - 0.12, 1.972)));
//	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005 + 0.12, 1.972)));
//	robot1->SetActType(srJoint::ACTTYPE::TORQUE);
//	robot2->SetActType(srJoint::ACTTYPE::TORQUE);
//	vector<int> gpIdx(2);
//	gpIdx[0] = 0;
//	gpIdx[1] = 1;
//	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
//	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
//	gpIdx[0] = 2;
//	gpIdx[1] = 3;
//	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
//	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
//
//	robotVector[0] = robot1;
//	robotVector[1] = robot2;
//	Trobotbase1 = robot1->GetBaseLink()->GetFrame();
//	Trobotbase2 = robot2->GetBaseLink()->GetFrame();
//	TrobotbaseVector[0] = Trobotbase1;
//	TrobotbaseVector[1] = Trobotbase2;
//}
//
//void robotManagerSetting()
//{
//	// robot 1
//	rManager1 = new indyRobotManager(robot1, &gSpace);
//
//	// robot 2
//	rManager2 = new indyRobotManager(robot2, &gSpace);
//
//
//	rManagerVector[0] = rManager1;
//	rManagerVector[1] = rManager2;
//}
//
//void workspaceSetting()
//{
//	gSpace.AddSystem(workCell);
//	// change stage4 location
//	if (workcell_mode == 2)
//	{
//		Vec2 stage4xyTrans(0.0, 0.0);
//		workCell->m_ObjWeldJoint[3].SetParentLinkFrame(SE3(Vec3(stage4xyTrans[0], stage4xyTrans[1], 0.0)));
//	}
//}
//
//void objectSetting()
//{
//	vector<SE3> testInit(8);
//	// 17.06.09 (using robot1)
//	testInit[0] = Trobotbase1 * SE3(-0.51328, 0.85822, 1.1137e-06, 0.85822, 0.51328, 5.4682e-06, 3.5287e-06, 3.3661e-06, -1, -0.28018, -0.10959, 0.85844);
//	testInit[1] = Trobotbase1 * SE3(-0.9354,0.35358,5.9942e-06,0.35358,0.9354,-6.9362e-06,-8.4326e-06,-4.4553e-06,-1,-0.26685,-0.020595,0.85866);
//	testInit[2] = Trobotbase1 * SE3(-0.66831,-0.74389,-1.1055e-05,-0.74389,0.66831,-4.5087e-06,9.9899e-06,5.0997e-06,-1,-0.3734,0.011193,0.85824);
//	testInit[3] = Trobotbase1 * SE3(-0.20649,0.97845,-2.1991e-05,0.97845,0.20649,-1.3097e-06,2.7943e-06,-2.1829e-05,-1,-0.36767,0.11754,0.85743);
//	// 17.06.10 (using robot2)
//	SE3 Trans = SE3(Vec3(-0.35, -0.1, 0.0));
//	testInit[4] = Trans * Trobotbase1 * SE3(0.49715, -0.86767, -2.3813e-06, -0.86767, -0.49714, -1.4671e-06, 4.218e-07, 2.6782e-06, -1, -1.2308, 0.17082, 1.0477);
//	testInit[5] = Trans * Trobotbase1 * SE3(-0.11762, -0.99306, 1.039e-06, -0.99306, 0.11762, -5.0989e-06, 5.0868e-06, -8.7669e-07, -1, -1.204, 0.028933, 1.0467);
//	testInit[6] = Trans * Trobotbase1 * SE3(-0.84321, -0.53758, 6.7572e-06, -0.53758, 0.84321, 4.3601e-06, -8.1082e-06, 3.8774e-07, -1, -1.2815, -0.060334, 1.0468);
//	testInit[7] = Trans * Trobotbase1 * SE3(-0.014703, -0.99989, -8.5711e-07, -0.99989, 0.014703, -3.6814e-06, 3.9231e-06, 7.9429e-08, -1, -1.2911, 0.095115, 1.0473);
//	for (unsigned int i = 0; i < busbar.size(); i++)
//	{
//		busbar[i] = new BusBar_HYU;
//		if (useNoVisionTestSetting && i < testInit.size())
//			busbar[i]->setBaseLinkFrame(testInit[i]);
//		else
//			busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
//		gSpace.AddSystem(busbar[i]);
//		busbar[i]->SetBaseLinkType(srSystem::FIXED);
//	}
//	for (unsigned int i = 0; i < ctCase.size(); i++)
//	{
//		ctCase[i] = new Insert;
//		ctCase[i]->SetBaseLinkType(srSystem::FIXED);
//		ctCase[i]->setBaseLinkFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
//		gSpace.AddSystem(ctCase[i]);
//	}
//	for (unsigned int i = 0; i < objects.size(); i++)
//	{
//		if (i < busbar.size())
//		{
//			objects[i] = busbar[i];
//			Tobject2gripper[i] = Tbusbar2gripper_new;
//		}
//		else if (i < busbar.size() + ctCase.size())
//		{
//			objects[i] = ctCase[i - busbar.size()];
//			Tobject2gripper[i] = TctCase2gripper;
//		}
//		TobjectsInitSimul[i] = objects[i]->GetBaseLink()->GetFrame();
//	}
//	
//}
//
//void connectJigToWorkCell()
//{
//	srWeldJoint* wJoint = new srWeldJoint;
//	wJoint->SetParentLink(workCell->GetBaseLink());
//	wJoint->SetChildLink(jigAssem->GetBaseLink());
//	wJoint->SetParentLinkFrame(jigAssem->GetBaseLink()->GetFrame());
//	wJoint->SetChildLinkFrame(SE3());
//}

void updateFuncLoadData()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	string loc = "../../../data/render_traj/";
	//string loc = "../../../../../Decision-intelligence-2017/srLib2017/data/render_traj/";
	static bool renderingStarted = false;
	static bool renderingFinished = true;
	static bool updateRobot1 = false;
	static bool updateRobot2 = false;
	static int replayIdx = 0;
	static int replayNum = 10;
	static int trjIdx = 0;
	static vector<vector<Eigen::VectorXd>> objTraj(objects.size());
	static vector<Eigen::VectorXd> robot1Traj(0);
	static vector<Eigen::VectorXd> robot2Traj(0);
	bool loadData = false;
	if (!renderingStarted || renderingFinished)
	{
		printf("select to load data: ");
		cin >> loadData;
		printf("\n");
	}
		
	if (!renderingStarted || loadData)
	{
		renderingStarted = true;
		renderingFinished = false;
		robot1Traj = loadDataFromText(loc + "/jointVal1.txt", 6);
		robot2Traj = loadDataFromText(loc + "/jointVal2.txt", 6);
		for (unsigned int k = 0; k < objects.size(); k++)
			objTraj[k] = loadDataFromText(loc + "/object" + to_string(k) + ".txt", 6);
		if (objTraj[0].size() == robot1Traj.size())
		{
			updateRobot1 = true;
			lastJointVal_multi[0] = robot1Traj[robot1Traj.size() - 1];
		}
		if (objTraj[0].size() == robot2Traj.size())
		{
			updateRobot2 = true;
			lastJointVal_multi[1] = robot2Traj[robot2Traj.size() - 1];
		}
		trjIdx = 0;
	}
	if (renderingStarted && !renderingFinished)
	{
		if (updateRobot1)
			rManager1->setJointVal(robot1Traj[trjIdx]);
		else
			rManager1->setJointVal(lastJointVal_multi[0]);
		if (updateRobot2)
			rManager2->setJointVal(robot2Traj[trjIdx]);
		else
			rManager2->setJointVal(lastJointVal_multi[1]);
		for (unsigned int i = 0; i < objects.size(); i++)
		{
			objects[i]->setBaseLinkFrame(VectortoSE3(objTraj[i][trjIdx]));
			objects[i]->KIN_UpdateFrame_All_The_Entity();
		}
		trjIdx++;
		if (trjIdx == objTraj[0].size())
		{
			trjIdx = 0;
			replayIdx++;
			if (replayIdx == replayNum)
			{
				replayIdx = 0;
				renderingFinished = true;
			}
		}
	}
}
