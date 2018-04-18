#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "common\dataIO.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\robotRRTManager.h"

#include <fstream>
#include <iostream>


// Environment
Base_HYU* busbarBase = new Base_HYU;
vector<Jig_HYU*> jig(4);
//vector<BusBar_HYU*> busbar(8);
//vector<SE3>	initSE3(8);
//vector<SE3>	goalSE3(8);
//vector<SE3> allSE3_busbar(2 * busbar.size());

vector<BusBar_HYU*> busbar(1);
vector<SE3>	initSE3(2);
vector<SE3>	goalSE3(2);
vector<SE3> allSE3_busbar(2 * initSE3.size());

// Workspace
WorkCell* workCell = new WorkCell;
Eigen::VectorXd stageVal(3);

// Robot
IndyRobot* robot1 = new IndyRobot;
IndyRobot* robot2 = new IndyRobot;
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);
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




// Measure F/T sensor
dse3 Ftsensor;
Eigen::VectorXd ftsensor(6);

// save data
vector<vector<Eigen::VectorXd>> FTtraj(initSE3.size());
vector<vector<Eigen::VectorXd>> TtrajVec(initSE3.size());
vector<vector<Eigen::VectorXd>> busbarTraj(initSE3.size());


indyRobotManager* rManager1;
indyRobotManager* rManager2;
robotRRTManager* RRTManager1 = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncPlanning();
void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting_HYU();
void RRT_problemSetting();
void RRTSolve();
void RRTSolve_HYU();
void rrtSetting();


vector<Eigen::VectorXd> loadJointVal = loadDataFromText("../../../data/workcell_test_data3/jointValTraj2.txt", 6);
//vector<Eigen::VectorXd> loadBusbarVec = loadDataFromText("../../../data/HYU_data/task1/busbarTraj0.txt", 12);




int main(int argc, char **argv)
{
	// environment
	workspaceSetting();
	int startLocation = 1;
	Vec2 goalLocation(0, 0);
	environmentSetting_HYU(true, startLocation, goalLocation);
	robotSetting();

	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();

	// workcell robot initial config
	jointVal.setZero();
	jointVal[0] = 0.0; jointVal[1] = -SR_PI_HALF; jointVal[2] = 80.0 / 90.0*SR_PI_HALF;  jointVal[3] = SR_PI_HALF;
	rManager2->setJointVal(jointVal);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	// rrt
	/*rrtSetting();
	RRT_problemSetting_HYU();
	RRTSolve_HYU();
*/

	//cout << loadJointVal.size() << endl;
	//cout << loadJointVal[0] << endl;

	//SE3 test;
	//test = EulerZYZ(Vec3(0.2, 0.3, 0.4), Vec3(1.2, 3.4, 3.2));
	//cout << "Original SE3:  "<< test << endl;
	//Eigen::VectorXd test_vec = SE3toVectorXd(test);
	//cout << "SE3 to VectorXd:  "<<test_vec << endl;
	//SE3 test_SE3 = VectorXdtoSE3(test_vec);
	//cout << "VectorXd to SE3:  " << test_SE3 << endl;

	unsigned int len = loadJointVal.size();
	for (unsigned int i = 0; i < len - 1; i++)
	{
		cout << (loadJointVal[i + 1] - loadJointVal[i]).norm() << endl;
	}

	
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
	renderer->setUpdateFunc(updateFunc);

	renderer->RunRendering();
}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
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

	
	//SE3 busbarSE3 = VectorXdtoSE3(loadBusbarVec[cnt]);

	rManager1->setJointVal(loadJointVal[cnt]);
	//busbar[0]->setBaseLinkFrame(busbarSE3);
		


	//int idx = 1;
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);

	//cout << "taskIdx: " << idx << endl;

	cnt++;

	if (cnt == loadJointVal.size())
	{
		cnt = 0;
	}
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;

}

void updateFuncPlanning()
{

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	static int cnt = 0;
	static int writeCnt = 0;
	static int writeTaskCnt = 0;

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

	// Calculate acc
	if (trjIdx == traj[idx].size() - 1)
		jointAcc = (traj[idx][trjIdx] - 2.0*traj[idx][trjIdx - 1] + traj[idx][trjIdx - 2]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else if (trjIdx == traj[idx].size() - 2)
		jointAcc = (traj[idx][trjIdx + 1] - 2.0*traj[idx][trjIdx] + traj[idx][trjIdx - 1]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);
	else
		jointAcc = (traj[idx][trjIdx + 2] - 2.0*traj[idx][trjIdx + 1] + traj[idx][trjIdx]) / (gSpace.m_Timestep_dyn_fixed*gSpace.m_Timestep_dyn_fixed);

	// Calculate init vel
	if (trjIdx < traj[idx].size() - 1)
	{
		jointVel = (traj[idx][trjIdx + 1] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;
		rManager1->setJointValVel(traj[idx][trjIdx], jointVel);
	}
	else
		jointVel = (traj[idx][trjIdx] - traj[idx][trjIdx]) / gSpace.m_Timestep_dyn_fixed;

	// control acceleration
	//rManager1->controlJointAcc(jointAcc);

	Eigen::VectorXd tau = rManager1->inverseDyn(traj[idx][trjIdx], jointVel, jointAcc);
	rManager1->controlJointTorque(tau);

	//busbar movement
	busbar[0]->setBaseLinkFrame(rManager1->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(Tbusbar2gripper));

	// read sensor value
	Ftsensor = rManager1->readSensorValue();
	ftsensor = dse3toVector(Ftsensor);

	//cout << "traj: " << traj[idx][trjIdx].transpose() << endl;
	//if (trjIdx < traj[idx].size()-1)
	//	cout << "trav: " << (traj[idx][trjIdx+1]-traj[idx][trjIdx]).transpose() << endl;
	//else
	//	cout << "trav: " << (traj[idx][trjIdx] - traj[idx][trjIdx]).transpose() << endl;
	//cout << "pos:  " << rManager1->getJointVal().transpose() << endl;
	//cout << "vel:  " << rManager1->getJointVel().transpose() << endl;
	//cout << "acc:  " << rManager1->getJointAcc().transpose() << endl;
	//cout << "ctrl: " << jointAcc.transpose() << endl << endl;
	//cout << "FT sensor:  " << ftsensor.transpose() << endl;
	//cout << "Ttraj:  " << Ttraj[idx][trjIdx] << endl << endl;




	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	//rManager1->setJointVal(traj[idx][trjIdx]);
	//int objNum = idx / 2;

	//for (unsigned int j = 0; j < goalSE3.size(); j++)
	//{
	//	if (j < objNum)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//	else
	//		busbar[j]->setBaseLinkFrame(initSE3[j]);
	//	if (j == objNum && idx % 2 == 1)
	//		busbar[j]->setBaseLinkFrame(goalSE3[j]);
	//}
	//if (idx % 2 == 0)
	//	busbar[objNum]->GetBaseLink()->SetFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper));
	/*

	if (idxTraj[idx][trjIdx] != 100)
	busbar[objNum]->setBaseLinkFrame(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() / Tbusbar2gripper);*/






	if (writeTaskCnt < traj.size())
	{
		if (writeCnt < traj[idx].size())
		{

			// make vector<> format
			FTtraj[idx].push_back(ftsensor);
			TtrajVec[idx].push_back(SE3toVectorXd(Ttraj[idx][trjIdx]));
			busbarTraj[idx].push_back(SE3toVectorXd(busbar[0]->GetBaseLink()->GetFrame()));

			writeCnt++;

		}
		if (writeCnt == traj[idx].size())
		{
			string dir_folder = "../../../data/HYU_data/task1";
			// save
			string dir_temp = dir_folder;
			saveDataToText(traj[idx], dir_temp.append("/jointTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(FTtraj[idx], dir_temp.append("/FTtraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(TtrajVec[idx], dir_temp.append("/endEffectTraj").append(to_string(idx)).append(".txt"));
			dir_temp = dir_folder;
			saveDataToText(busbarTraj[idx], dir_temp.append("/busbarTraj").append(to_string(idx)).append(".txt"));


			writeCnt = 0;
			writeTaskCnt++;

			printf("%d-th task write done!! \n", idx);
		}

	}


	trjIdx++;


	if (trjIdx == traj[idx].size())
	{
		trjIdx = 0;

		taskIdx++;
		cout << "taskIdx: " << taskIdx % traj.size() << endl;
	}
	if (gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
	{
		/*cout << taskIdx << endl;
		cout << traj[idx][trjIdx].transpose() << endl;*/
	}
	cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
}

void environmentSetting_HYU(bool connect, int startLocation, Vec2 goalLocation) // startLocation: x location according to jig number / goalLocation[0]: jig number, goalLocation[1]: front or back
{
	for (unsigned int i = 0; i < jig.size(); i++)
	{
		jig[i] = new Jig_HYU;
		jig[i]->SetBaseLinkType(srSystem::FIXED);
	}
	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}

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
		//for (unsigned int j = 0; j < 2; j++)
		//{
		//	busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
		//	gSpace.AddSystem(busbar[2 * i + j]);
		//}
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


	SE3 initSE3_x = jigSE3[startLocation] * jig2busbar[0];
	initSE3[0] = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(initSE3_x.GetPosition()[0], 0.604, 1.1111 + 0.0001));

	goalSE3[0] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]] * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.02, 0.02, 0.15));

	initSE3[1] = goalSE3[0];
	goalSE3[1] = jigSE3[goalLocation[0]] * jig2busbar[goalLocation[1]];

	busbar[0]->GetBaseLink()->SetFrame(initSE3[0]);
	gSpace.AddSystem(busbar[0]);

}


void robotSetting()
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005 - 0.12, 1.972)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005 + 0.12, 1.972)));
	robot1->SetActType(srJoint::ACTTYPE::TORQUE);
	robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	gpIdx[0] = 2;
	gpIdx[1] = 3;
	robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new indyRobotManager(robot1, &gSpace);

	// robot 2
	rManager2 = new indyRobotManager(robot2, &gSpace);
}

void workspaceSetting()
{
	gSpace.AddSystem(workCell);
}

void rrtSetting()
{
	vector<srStateJoint*> planningJoints(6);
	for (unsigned int i = 0; i < planningJoints.size(); i++)
		planningJoints[i] = (srStateJoint*)robot1->gJoint[i];
	RRTManager1->setSystem(planningJoints);
	RRTManager1->setSpace(&gSpace);
	RRTManager1->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
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

void RRT_problemSetting_HYU()
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

	qInit = initPos[0];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[1] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[0].transpose() << endl;
	initPos.push_back(goalPos[0]);
	printf("%d-th init inv kin flag: %d\n", 1, flag);

	qInit = initPos[1];
	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[3] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	cout << goalPos[1].transpose() << endl;

	//for (unsigned int i = 1; i < allSE3_busbar.size(); i++)
	//{
	//	qInit = initPos[i - 1];
	//	goalPos.push_back(rManager1->inverseKin(allSE3_busbar[i] * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit));
	//	cout << goalPos[i - 1].transpose() << endl;
	//	initPos.push_back(goalPos[i - 1]);
	//	if (i % 2 == 0)
	//		printf("%d-th init inv kin flag: %d\n", i / 2, flag);
	//	else
	//		printf("%d-th goal inv kin flag: %d\n", i / 2, flag);
	//}


}
void RRTSolve()
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = 2;		// <= 15
	vector<bool> feas(2);
	unsigned int objNum;
	bool isAttached = false;

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		objNum = i / 2;
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);

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
			RRTManager1->attachObject(busbar[objNum], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));
			printf("%d-th busbar moving...\n", objNum);
		}
		else
		{
			isAttached = false;
			RRTManager1->detachObject();
			busbar[objNum]->setBaseLinkFrame(goalSE3[objNum]);
			printf("%d-th busbar moved, reaching to next one...\n", objNum);
		}

		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager1->execute(0.1);
		tempTraj = RRTManager1->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager1->setState(tempTraj[j]))
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
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);
		}




	}
}


void RRTSolve_HYU()
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	vector<int> tempIdxTraj(0);
	vector<SE3> tempTtraj(0);
	int start = 0;		//  >= 0
	int end = 2;		// <= 15
	vector<bool> feas(2);

	traj.resize(0);
	Ttraj.resize(0);
	idxTraj.resize(0);
	for (int i = start; i < end; i++)
	{
		RRTManager1->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		RRTManager1->attachObject(busbar[0], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper));



		feas = RRTManager1->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		RRTManager1->execute(0.1);
		tempTraj = RRTManager1->extractPath();

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager1->setState(tempTraj[j]))
				cout << "collide at " << j << "th point!!!" << endl;

		traj.push_back(tempTraj);


		tempTtraj.resize(tempTraj.size());
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
				tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
			Ttraj.push_back(tempTtraj);
		}




	}
}
