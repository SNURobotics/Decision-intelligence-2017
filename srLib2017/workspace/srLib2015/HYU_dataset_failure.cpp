#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_workcell.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\robotRRTManager.h"
#include "ForceCtrlManager\impedanceCtrlManager.h"
#include "common\dataIO.h"

// Environment
Base_HYU* busbarBase = new Base_HYU;
vector<Jig_HYU*> jig(4);
vector<BusBar_HYU*> busbar(8);
vector<SE3>	initSE3(8);
vector<SE3>	goalSE3(8);
vector<SE3> allSE3_busbar(2 * busbar.size());
srLink* busbarlink = new srLink;

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

indyRobotManager* rManager1;
indyRobotManager* rManager2;
robotRRTManager* RRTManager1 = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::HYBRID;

vector<SE3> r2o(1);
impedanceCtrlManager* iManager = new impedanceCtrlManager;

// save variables
vector<Eigen::VectorXd> contactFTrj(0);
vector<Eigen::VectorXd> sensorFTrj(0);
vector<Eigen::VectorXd> robotEndSE3Trj(0);
vector<Eigen::VectorXd> busbarSE3Trj(0);
vector<Eigen::VectorXd> robotEndSE3Trj_robotbase(0);
vector<Eigen::VectorXd> busbarSE3Trj_robotbase(0);
vector<Eigen::VectorXd> jointTrj(0);
int dataSaving_nStep = 10;
SE3 goalJigSE3;
SE3 holeSE3;
SE3 initOffsetSE3fromHole;
Eigen::VectorXd homePos;
Eigen::VectorXd jointVel;
Eigen::VectorXd jointAcc;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncDefault();
void updateFuncPlanning();
void updateFuncTestSensor();
void updateFuncData();
void environmentSetting_HYU(srSystem* object, bool connectStageBusbarBase = false);
void workspaceSetting();
void robotSetting();
void robotManagerSetting();
void RRT_problemSetting();
void RRTSolve();
void rrtSetting();
void setImpede(SE3 TrobotInit, SE3 Tgoal, SE3 Trobot2obj, srSystem* object1, bool weld = false);
void convertSE3Data(string dir_folder, string fileName, string addtext, SE3 pre, SE3 post);

int main(int argc, char **argv)
{
	// Robot home position
	homePos = Eigen::VectorXd::Zero(6);
	homePos[1] = -SR_PI_HALF; homePos[3] = SR_PI_HALF; homePos[4] = SR_PI;

	// environment
	workspaceSetting();
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
	environmentSetting_HYU(busbar[1], true);
	robotSetting();

	//BusBar_HYU* busbar_test = new BusBar_HYU;
	//busbar_test->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -0.5)));
	//gSpace.AddSystem(busbar_test);

	// initialize srLib
	initDynamics();

	// robot manager setting
	robotManagerSetting();

	// workcell robot initial config
	jointVal.setZero();
	jointVal[0] = 0.0; jointVal[1] = -SR_PI_HALF; jointVal[2] = 80.0 / 90.0*SR_PI_HALF; jointVal[3] = SR_PI_HALF;
	rManager2->setJointVal(jointVal);
	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	//// rrt
	////rrtSetting();
	////RRT_problemSetting();
	////RRTSolve();


	// impedance control
	Eigen::VectorXd qInit = Eigen::VectorXd::Zero(6);
	qInit[1] = -0.65*SR_PI;
	qInit[2] = 0.3*SR_PI;
	qInit[3] = 0.5*SR_PI_HALF;
	int holeNum = 2;		// from 0 ~ 7
	goalJigSE3 = jig[holeNum/2]->GetBaseLink()->GetFrame();
	SE3 TgoalPos = goalSE3[holeNum] * SE3(Vec3(0.01, 0.01, 0.0));
	SE3 TinitPos = SE3(Vec3(0.0, 0.0, 0.02)) * TgoalPos * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	holeSE3 = goalSE3[holeNum];
	initOffsetSE3fromHole = holeSE3 % TinitPos;
	
	int flag;
	qInit = rManager1->inverseKin(TinitPos * Tbusbar2gripper, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], true, SE3(), flag, qInit);
	printf("invkin flag: %d\n", flag);
	rManager1->setJointVal(qInit);
	SE3 TrobotInit = robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame();
	setImpede(TrobotInit, TgoalPos, Inv(Tbusbar2gripper), busbar[1], true);


	////busbar[1]->setBaseLinkFrame(TgoalPos * SE3(Vec3(0.0, 0.0, 0.01)));

	////rManager1->setJointVal(jointVal);

	//// load data
	//string dir_folder = "../../../data/HYU_data/failure_data";
	//int taskIdx = 3;
	//dir_folder.append(to_string(taskIdx));
	//jointTrj = loadDataFromText(dir_folder.append("/jointValTraj").append(".txt"), 6);


	//cout << "robot end-effector" << endl;
	//cout << TrobotInit << endl;
	//cout << "robot sensor" << endl;
	//cout << rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() << endl;

	//// convert SE3 data
	//string dir_folder = "../../../data/HYU_data/failure_data";
	//for (int taskIdx = 1; taskIdx < 4; taskIdx++)
	//{
	//	string temp = dir_folder;
	//	temp.append(to_string(taskIdx));
	//	convertSE3Data(temp, "busbarTraj", "_robotbase", Inv(robot1->GetBaseLink()->GetFrame()), SE3());
	//	convertSE3Data(temp, "robotEndTraj", "_robotbase", Inv(robot1->GetBaseLink()->GetFrame()), SE3());
	//	convertSE3Data(temp, "setting", "_robotbase", Inv(robot1->GetBaseLink()->GetFrame()), SE3());
	//}
	//
	//// check
	//vector<Eigen::VectorXd> check1 = loadDataFromText("../../../data/HYU_data/failure_data3/busbarTraj.txt", 12);
	//vector<Eigen::VectorXd> check2 = loadDataFromText("../../../data/HYU_data/failure_data3/busbarTraj_robotbase.txt", 12);
	//for (unsigned int i = 0; i < check1.size(); i++)
	//{
	//	SE3 T1 = VectorXdtoSE3(check1[i]);
	//	SE3 T2 = VectorXdtoSE3(check2[i]);
	//	cout << T1 / T2;
	//	cout << robot1->GetBaseLink()->GetFrame();
	//	cout << "-------------" << endl;
	//	int stop = 1;
	//}


	cout << "jig" << endl;
	cout << goalJigSE3 << endl;
	cout << robot1->GetBaseLink()->GetFrame() << endl;

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
	renderer->setUpdateFunc(updateFuncTestSensor);

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
	static bool isDataSaved = false;
	alpha += 0.01;

	Eigen::VectorXd gripInput(2);
	gripInput[0] = -0.009;
	gripInput[1] = 0.009;
	rManager1->setGripperPosition(gripInput);
	rManager2->setGripperPosition(gripInput);

	if (cnt == 0)
		cout << rManager1->getBodyJacobian(rManager1->getJointVal(), &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], SE3()) << endl;
	cnt++;
	int stop = 1;
	iManager->impedanceControlWeld();
	//iManager->objectImpedanceControl();


	// gather data /////////////////////////////
	if (cnt % dataSaving_nStep == 1)
	{
		sensorFTrj.push_back(dse3toVector(rManager1->readSensorValue()));
		contactFTrj.push_back(dse3toVector(busbarlink->m_ConstraintImpulse * (1.0 / gSpace.m_Timestep_dyn_fixed)));
		robotEndSE3Trj.push_back(SE3toVectorXd(robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame()));
		busbarSE3Trj.push_back(SE3toVectorXd(busbarlink->GetFrame()));
		robotEndSE3Trj_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % robot1->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame()));
		busbarSE3Trj_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % busbarlink->GetFrame()));
		jointTrj.push_back(rManager1->getJointVal());
	}
	
	////////////////////////////////////////////

	// change desired points
	int changeIdx = 1000;
	bool useUserInput = true;
	if (useUserInput)
	{
		int changeType = 0;
		static double pos_x;
		static double pos_y;
		static double pos_z;
		static double theta_x;
		static double theta_y;
		static double theta_z;
		if (cnt % changeIdx == 0)
		{
			printf("change type?: (0: no change, 1: pos, 2: ori, 3: absori, 4: pos & ori, 5: pos & absori, 6: save data)");
			cin >> changeType;
			switch (changeType)
			{
			case 0:
				break;
			case 1:
			{
				printf("\ndisplacement: (dx,dy,dz): ");
				cin >> pos_x;
				cin >> pos_y;
				cin >> pos_z;
				iManager->Tdes_trj[0].SetPosition(iManager->Tdes_trj[0].GetPosition() + Vec3(pos_x, pos_y, pos_z));
				printf("\nnew Tdes: \n");
				cout << iManager->Tdes_trj[0] << endl;
				printf("\nhole SE3: \n");
				cout << holeSE3 << endl;
				break;
			}
			case 2:
			{
				printf("\ndisplacement: (dtheta_x,dtheta_y,dtheta_z): ");
				cin >> theta_x; cin >> theta_y; cin >> theta_z;
				iManager->Tdes_trj[0].SetOrientation(iManager->Tdes_trj[0].GetOrientation() * Exp(Vec3(theta_x, theta_y, theta_z)));
				printf("\nnew Tdes: \n");
				cout << iManager->Tdes_trj[0] << endl;
				printf("\nhole SE3: \n");
				cout << holeSE3 << endl;
				break;
			}
			case 3:
			{
				printf("\ndisplacement: (theta_x,theta_y,theta_z): ");
				cin >> theta_x; cin >> theta_y; cin >> theta_z;
				iManager->Tdes_trj[0].SetOrientation(Exp(Vec3(theta_x, theta_y, theta_z)));
				printf("\nnew Tdes: \n");
				cout << iManager->Tdes_trj[0] << endl;
				printf("\nhole SE3: \n");
				cout << holeSE3 << endl;
				break;
			}
			case 4:
			{
				printf("\ndisplacement: (dx,dy,dz,dtheta_x,dtheta_y,dtheta_z): ");
				cin >> pos_x;
				cin >> pos_y;
				cin >> pos_z;
				cin >> theta_x; cin >> theta_y; cin >> theta_z;
				iManager->Tdes_trj[0].SetPosition(iManager->Tdes_trj[0].GetPosition() + Vec3(pos_x, pos_y, pos_z));
				iManager->Tdes_trj[0].SetOrientation(iManager->Tdes_trj[0].GetOrientation() * Exp(Vec3(theta_x, theta_y, theta_z)));
				printf("\nnew Tdes: \n");
				cout << iManager->Tdes_trj[0] << endl;
				printf("\nhole SE3: \n");
				cout << holeSE3 << endl;
				break;
			}
			case 5:
			{
				printf("\ndisplacement: (dx,dy,dz,theta_x,theta_y,theta_z): ");
				cin >> pos_x;
				cin >> pos_y;
				cin >> pos_z;
				cin >> theta_x; cin >> theta_y; cin >> theta_z;
				iManager->Tdes_trj[0].SetPosition(iManager->Tdes_trj[0].GetPosition() + Vec3(pos_x, pos_y, pos_z));
				iManager->Tdes_trj[0].SetOrientation(Exp(Vec3(theta_x, theta_y, theta_z)));
				printf("\nnew Tdes: \n");
				cout << iManager->Tdes_trj[0] << endl;
				printf("\nhole SE3: \n");
				cout << holeSE3 << endl;
				break;
			}
			case 6:
			{
				if (!isDataSaved)
				{
					isDataSaved = true;
					vector<Eigen::VectorXd> setting(0);
					setting.push_back(SE3toVectorXd(goalJigSE3));
					setting.push_back(SE3toVectorXd(holeSE3));
					setting.push_back(SE3toVectorXd(initOffsetSE3fromHole));
					vector<Eigen::VectorXd> setting_robotbase(0);
					setting_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % goalJigSE3));
					setting_robotbase.push_back(SE3toVectorXd(robot1->GetBaseLink()->GetFrame() % holeSE3));
					setting_robotbase.push_back(SE3toVectorXd(initOffsetSE3fromHole));
					string dir_folder = "../../../data/HYU_data/failure_data";
					string dir_temp = dir_folder;
					saveDataToText(setting, dir_temp.append("/setting").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(setting_robotbase, dir_temp.append("/setting_robotbase").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(sensorFTrj, dir_temp.append("/sensorValTraj").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(contactFTrj, dir_temp.append("/contactFValTraj").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(jointTrj, dir_temp.append("/jointValTraj").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(robotEndSE3Trj, dir_temp.append("/robotEndTraj").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(busbarSE3Trj, dir_temp.append("/busbarTraj").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(robotEndSE3Trj_robotbase, dir_temp.append("/robotEndTraj_robotbase").append(".txt"));
					dir_temp = dir_folder;
					saveDataToText(busbarSE3Trj_robotbase, dir_temp.append("/busbarTraj_robotbase").append(".txt"));
				}
			}
			default:
				break;
			}
		}
	}

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

void updateFuncDefault()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
}

void updateFuncData()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();
	static unsigned int len = jointTrj.size();
	static unsigned int cnt = 0;
	rManager1->setJointVal(jointTrj[cnt % len]);
	cnt++;
}

void environmentSetting_HYU(srSystem* object, bool connectStageBase)
{
	
		
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
		if (!connectStageBase)
			gSpace.AddSystem(jig[i]);
		for (unsigned int j = 0; j < 2; j++)
		{
			busbar[2 * i + j]->setBaseLinkFrame(jigSE3[i] * jig2busbar[j]);
			goalSE3[2 * i + j] = jigSE3[i] * jig2busbar[j];
			if (object == NULL)
				gSpace.AddSystem(busbar[2 * i + j]);
		}
	}
	if (object != NULL)
	{
		srWeldJoint* wobjJoint = new srWeldJoint;
		wobjJoint->SetParentLink(&robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		wobjJoint->SetChildLink(object->GetBaseLink());
		wobjJoint->SetParentLinkFrame(SE3());
		wobjJoint->SetChildLinkFrame(Tbusbar2gripper);
		busbarlink = object->GetBaseLink();
	}
	if (!connectStageBase)
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
	qInit[1] = 0.65*SR_PI;
	qInit[2] = -1.1*SR_PI;
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
			tempTtraj[i] = rManager1->forwardKin(tempTraj[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		Ttraj.push_back(tempTtraj);
	}
}

void setImpede(SE3 TrobotInit, SE3 Tgoalpos, SE3 Trobot2obj, srSystem* object1, bool weld)
{
	vector<srLink*> ees(1);
	vector<SE3> ofs(1, SE3());

	Eigen::MatrixXd mass = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd res = 100.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd cap = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
	//Eigen::MatrixXd integralGain = 0.0 * Eigen::MatrixXd::Identity(6, 6);
	//integralGain(3, 3) = 1000.0;
	//integralGain(4, 4) = 1000.0;
	for (int i = 0; i < 3; i++)
		cap(i, i) = 100.0;
	cap(5, 5) = 100.0;
	for (int i = 3; i < 4; i++)
		res(i, i) = 2.0*sqrt(mass(i, i)*cap(i, i));

	se3 zero_se3 = se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	ees[0] = rManager1->m_activeArmInfo->m_endeffector[0];
	r2o[0] = Trobot2obj;
	if (!weld)
		object1->GetBaseLink()->SetFrame(TrobotInit*r2o[0]);
	gSpace._KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	iManager->setSystem(rManager1, ees, ofs, r2o);
	if (!weld)
		iManager->setObject(object1);
	else
	{
		vector<srLink*> links(1);
		links[0] = object1->GetBaseLink();
		iManager->setObjectLinks(object1->GetBaseLink(), links);
	}
	
	iManager->setInertial(mass);
	iManager->setCapacitive(cap);
	iManager->setResistive(res);
	//iManager->setIntegralGain(integralGain);
	iManager->setGravity(gSpace.m_Gravity);
	iManager->setRobotPostureAmendThreshold(0.01);
	iManager->setTimeStep(gSpace.m_Timestep_dyn_fixed);
	iManager->setDesiredTrajectory(SE3(object1->GetBaseLink()->GetOrientation(), Tgoalpos.GetPosition()), zero_se3, zero_se3);
}

void convertSE3Data(string dir_folder, string fileName, string addtext, SE3 pre, SE3 post)
{
	// load data
	string dir_file = dir_folder;
	vector<Eigen::VectorXd> trj = loadDataFromText(dir_file.append("/").append(fileName).append(".txt"), 12);
	// convert data
	vector<Eigen::VectorXd> convTrj(0);
	SE3 temp;
	for (unsigned int i = 0; i < trj.size(); i++)
	{
		temp = pre * VectorXdtoSE3(trj[i]) * post;
		convTrj.push_back(SE3toVectorXd(temp));
	}
	// save data
	string dir_newfile = dir_folder;
	saveDataToText(convTrj, dir_newfile.append("/").append(fileName).append(addtext).append(".txt"));
}


void updateFuncTestSensor()
{
	static int cnt = 0;
	static Eigen::VectorXd testjointVel = Eigen::VectorXd::Ones(6);
	if (cnt == 0)
	{
		rManager1->setJointValVel(homePos, testjointVel);
		jointVal = homePos;
		jointVel = testjointVel;
	}
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	
	

	jointVal = rManager1->getJointVal();
	jointVel = rManager1->getJointVel();
	
	cnt++;
	Eigen::VectorXd jointAcc = Eigen::VectorXd::Ones(6);
	Eigen::VectorXd tau = rManager1->inverseDyn(jointVal, jointVel, jointAcc);
	rManager1->controlJointTorque(tau);
	//SE3 renderEndeff = rManager1->forwardKin(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//busbar[0]->setBaseLinkFrame(renderEndeff*Inv(Tbusbar2gripper));
	// read sensor value
	dse3 Ftsensor = rManager1->readSensorValue();
	//dse3 Fr(0.0);
	//se3 g(0.0);
	//se3 Vdot(0.0);
	//se3 V(0.0);
	//V = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	//Vdot = Vectortose3(rManager1->getBodyJacobian(jointVal, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointAcc +
	//	rManager1->getBodyJacobianDot(jointVal, jointVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tbusbar2gripper)) * jointVel);
	//for (int i = 0; i < 3; i++)
	//	g[i] = gSpace.m_Gravity[i];
	//Fr = busbar[0]->GetBaseLink()->m_Inertia * Vdot - dad(V, busbar[0]->GetBaseLink()->m_Inertia * V) - busbar[0]->GetBaseLink()->m_Inertia*InvAd(busbar[0]->GetBaseLink()->GetFrame(), g);

	//ftsensor = dse3toVector(Ftsensor + InvdAd((rManager1->m_ftSensorInfo[0]->m_sensorLocJoint->GetFrame() * rManager1->m_ftSensorInfo[0]->m_offset) % renderEndeff * Inv(Tbusbar2gripper), Fr));

	Eigen::VectorXd ftsensor = dse3toVector(Ftsensor);
	cout << "q: " << jointVal.transpose() << endl;
	cout << "a: " << rManager1->getJointAcc().transpose() << endl;
	cout << "f: " << ftsensor.transpose() << endl;
	cout << "busbar: " << endl << busbar[1]->GetBaseLink()->GetFrame() << endl;
	cout << "V: " << busbar[1]->GetBaseLink()->m_Vel << endl;
	cout << "A: " << busbar[1]->GetBaseLink()->m_Acc << endl;
}
