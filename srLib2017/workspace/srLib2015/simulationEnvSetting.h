#pragma once

#include "common\dataIO.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\indyRobotManager.h"
#include "robotManager/IndyRobot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_workcell.h"
#include "tcp_ip_communication.h"

//srLib
srSpace gSpace;

// Environment
JigAssem_QB_bar* jigAssem = new JigAssem_QB_bar(false);
vector<BusBar_HYU*> busbar(8);
vector<Insert*> ctCase(0);
vector<Object*> objects(busbar.size() + ctCase.size());
vector<SE3> TobjectsInitSimul(objects.size());
bool isJigConnectedToWorkCell = true;
SE3 initBusbar = SE3(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, -0.5)));

// Workspace
int workcell_mode = 0;
WorkCell* workCell;
Eigen::VectorXd stageVal(3);
bool useNoVisionTestSetting = true;
bool useNoVisionTestSettingJig = true;

// Robot
IndyRobot* robot1 = new IndyRobot(false);
IndyRobot* robot2 = new IndyRobot(false);
vector<IndyRobot*> robotVector(2);
SE3 Trobotbase1;
SE3 Trobotbase2;
vector<SE3> TrobotbaseVector(2);
indyRobotManager* rManager1;
indyRobotManager* rManager2;
vector<indyRobotManager*> rManagerVector(2);
vector<Eigen::VectorXd> homePosRobotVector(2);

//SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.04));
SE3 Tbusbar2gripper_new = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.02));
SE3 Tbusbar2gripper_tight = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
SE3 TctCase2gripper = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.01));
vector<SE3> Tobject2gripper(objects.size());
SE3 Thole2busbar = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));


vector<srLink*> obstacle(0);
vector<srWeldJoint*> wJoint(0);		// weld joint for connecting workcell and obstacle
bool isSystemAssembled = false;

// modelling functions
void workspaceSetting(double height = 0.0);
void robotSetting(double height = 0.0);
void environmentSetting_HYU2(bool connect, bool fixJigPos = false);
void objectSetting();
void connectJigToWorkCell();
void initDynamics();
void robotManagerSetting();
// communication function
void setEnviromentFromVision(const vision_data& skku_dataset, int& bNum, int& cNum);
void loadVisionResultFromText(string loc);

bool saveVisionResult = true;

void workspaceSetting(double height)
{
	workCell = new WorkCell(workcell_mode, height);
	gSpace.AddSystem(workCell);
	// change stage4 location
	if (workcell_mode == 2)
	{
		Vec2 stage4xyTrans(0.0, 0.0);
		workCell->m_ObjWeldJoint[3].SetParentLinkFrame(SE3(Vec3(stage4xyTrans[0], stage4xyTrans[1], 0.0)));
	}
}


void robotSetting(double height)
{
	gSpace.AddSystem((srSystem*)robot1);
	gSpace.AddSystem((srSystem*)robot2);
	//robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005 - 0.12, 1.972 + height)));
	robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005 + 0.12, 1.972 + height)));
	robot1->GetBaseLink()->SetFrame(EulerZYX(Vec3(-SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 0.4005, 1.972 + height)));
	//robot2->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.0205, 1.6005, 1.972 + height)));
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

	robotVector[0] = robot1;
	robotVector[1] = robot2;
	Trobotbase1 = robot1->GetBaseLink()->GetFrame();
	Trobotbase2 = robot2->GetBaseLink()->GetFrame();
	TrobotbaseVector[0] = Trobotbase1;
	TrobotbaseVector[1] = Trobotbase2;

	homePosRobotVector[0] = robot1->homePos;
	homePosRobotVector[1] = robot2->homePos;
}

void environmentSetting_HYU2(bool connect, bool fixJigPos)
{
	// should be called later than workcell setting
	SE3 Tbase;
	if (workcell_mode == 1)
		Tbase = SE3(Vec3(0.025, 1.095, 1.176));		// when stage attached
	else if (workcell_mode == 2)
	{
		Vec3 stage4Trans = workCell->m_ObjWeldJoint[3].GetParentLinkFrame().GetPosition();
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.105 + 0.009) + stage4Trans);	// when only stage4 is used
	}
	else
		Tbase = SE3(Vec3(0.025, 1.095, 0.910 + 0.009));		// when stage removed
	double z_angle = (double)rand() / RAND_MAX * 1.0;
	double x_trans = -(double)rand() / RAND_MAX * 0.1;
	double y_trans = (double)rand() / RAND_MAX * 0.1;
	//SE3 Tbase2jigbase = EulerZYX(Vec3(z_angle, 0.0, 0.0), Vec3(x_trans, y_trans, 0.184));
	SE3 Tbase2jigbase = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.184));
	SE3 Ttempjigbase = Tbase * Tbase2jigbase;
	//Ttempjigbase[10] = 0.5*(robot1->GetBaseLink()->GetFrame()[10] + robot2->GetBaseLink()->GetFrame()[10]);
	//for (unsigned int i = 0; i < busbar.size(); i++)
	//{
	//	busbar[i] = new BusBar_HYU;
	//	busbar[i]->SetBaseLinkType(srSystem::FIXED);
	//	gSpace.AddSystem(busbar[i]);
	//}
	//Vec3 testJigPosFromRobot1(-0.702151, -0.014057, 0.750026);		// 17.06.09 using robot1
	//Vec3 testJigPosFromRobot1(-0.8254, 0.0338, 0.7483);		// 17.06.10 using robot2
	//Vec3 testJigPosFromRobot1(-0.8277, -0.0536, 0.8620);		// 17.06.10 using robot2
	//Vec3 testJigPosFromRobot2(-0.5276, -0.032456, 0.869522);		// 17.07.13 using robot2
	//Vec3 testJigPosFromRobot2(-0.551584, -0.073662, 0.865189);		// 17.07.21 using robot2
	Vec3 testJigPosFromRobot2(-0.8562, 0.0450, 0.8206);		// 17.07.21 using robot2
	//Vec3 testJigPosFromRobot2(-0.55, 0.550, 0.5506);		// 17.07.21 using robot2

	jigAssem->SetBaseLinkType(srSystem::FIXED);
	if (!useNoVisionTestSettingJig || fixJigPos)
		jigAssem->setBaseLinkFrame(Ttempjigbase);
	else
	{
		//SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
		SE3 tempSE3 = Trobotbase2 * SE3(testJigPosFromRobot2);
		jigAssem->setBaseLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
	}

	if (!connect)
		gSpace.AddSystem((srSystem*)jigAssem);
	else
	{
		// jig is connected to workcell via weld joint
		srWeldJoint* wJoint = new srWeldJoint;
		wJoint->SetParentLink(workCell->GetBaseLink()); // removed stage
														//wJoint->SetParentLink(workCell->getStagePlate());
		wJoint->SetChildLink(jigAssem->GetBaseLink());
		if (!useNoVisionTestSettingJig || fixJigPos)
			wJoint->SetParentLinkFrame(Ttempjigbase);
		else
		{
			//SE3 tempSE3 = Trobotbase1 * SE3(testJigPosFromRobot1);
			SE3 tempSE3 = Trobotbase2 * SE3(testJigPosFromRobot2);
			SE3 Tjig = Trobotbase1 % SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset;
			//cout << "Tjig" << endl;
			//cout << Tjig << endl;
			wJoint->SetParentLinkFrame(SE3(tempSE3.GetPosition()) * jigAssem->m_visionOffset);
		}
		wJoint->SetChildLinkFrame(SE3());
	}

	// ctCase test
	//ctCase[0]->setBaseLinkFrame(Tbase*Tbase2jigbase*SE3(Vec3(0.0, 0.0, 0.03)));
}

void objectSetting()
{
	vector<SE3> testInit(8);
	//// 17.06.09 (using robot1)
	//testInit[0] = Trobotbase1 * SE3(-0.51328, 0.85822, 1.1137e-06, 0.85822, 0.51328, 5.4682e-06, 3.5287e-06, 3.3661e-06, -1, -0.28018, -0.10959, 0.85844);
	//testInit[1] = Trobotbase1 * SE3(-0.9354, 0.35358, 5.9942e-06, 0.35358, 0.9354, -6.9362e-06, -8.4326e-06, -4.4553e-06, -1, -0.26685, -0.020595, 0.85866);
	//testInit[2] = Trobotbase1 * SE3(-0.66831, -0.74389, -1.1055e-05, -0.74389, 0.66831, -4.5087e-06, 9.9899e-06, 5.0997e-06, -1, -0.3734, 0.011193, 0.85824);
	//testInit[3] = Trobotbase1 * SE3(-0.20649, 0.97845, -2.1991e-05, 0.97845, 0.20649, -1.3097e-06, 2.7943e-06, -2.1829e-05, -1, -0.36767, 0.11754, 0.85743);
	//// 17.06.10 (using robot2)
	//SE3 Trans = SE3(Vec3(-0.35, -0.1, 0.0));
	//testInit[4] = Trans * Trobotbase1 * SE3(0.49715, -0.86767, -2.3813e-06, -0.86767, -0.49714, -1.4671e-06, 4.218e-07, 2.6782e-06, -1, -1.2308, 0.17082, 1.0477);
	//testInit[5] = Trans * Trobotbase1 * SE3(-0.11762, -0.99306, 1.039e-06, -0.99306, 0.11762, -5.0989e-06, 5.0868e-06, -8.7669e-07, -1, -1.204, 0.028933, 1.0467);
	//testInit[6] = Trans * Trobotbase1 * SE3(-0.84321, -0.53758, 6.7572e-06, -0.53758, 0.84321, 4.3601e-06, -8.1082e-06, 3.8774e-07, -1, -1.2815, -0.060334, 1.0468);
	//testInit[7] = Trans * Trobotbase1 * SE3(-0.014703, -0.99989, -8.5711e-07, -0.99989, 0.014703, -3.6814e-06, 3.9231e-06, 7.9429e-08, -1, -1.2911, 0.095115, 1.0473);
	// 17.07.21 (using robot1 and 2)
	Vec3 zoffset2(0.0, 0.0, 0.02);
	Vec3 zoffset1(0.0, 0.0, 0.005);
	testInit[0] = Trobotbase1 * SE3(Vec3(-0.3714, 0.0473, 0.849) + zoffset1);
	testInit[0].SetOrientation(SO3());
	testInit[1] = Trobotbase2 * SE3(Vec3(-0.556345, -0.459122, 0.827823) + zoffset2);
	testInit[1].SetOrientation(SO3());
	for (int i = 0; i < 6; i++)
		testInit[i + 2] = SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar;
	// 17.08.18 
	for (int i = 0; i < 8; i++)
		testInit[i] = SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar;

	for (unsigned int i = 0; i < busbar.size(); i++)
	{
		busbar[i] = new BusBar_HYU;
		if (useNoVisionTestSetting && i < testInit.size())
			busbar[i]->setBaseLinkFrame(testInit[i]);
		else
			busbar[i]->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		gSpace.AddSystem(busbar[i]);
		busbar[i]->SetBaseLinkType(srSystem::FIXED);
	}
	for (unsigned int i = 0; i < ctCase.size(); i++)
	{
		ctCase[i] = new Insert;
		ctCase[i]->SetBaseLinkType(srSystem::FIXED);
		ctCase[i]->setBaseLinkFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
		gSpace.AddSystem(ctCase[i]);
	}
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		if (i < busbar.size())
		{
			objects[i] = busbar[i];
			Tobject2gripper[i] = Tbusbar2gripper_new;
		}
		else if (i < busbar.size() + ctCase.size())
		{
			objects[i] = ctCase[i - busbar.size()];
			Tobject2gripper[i] = TctCase2gripper;
		}
		TobjectsInitSimul[i] = objects[i]->GetBaseLink()->GetFrame();
	}

}

void connectJigToWorkCell()
{
	srWeldJoint* wJoint = new srWeldJoint;
	wJoint->SetParentLink(workCell->GetBaseLink());
	wJoint->SetChildLink(jigAssem->GetBaseLink());
	wJoint->SetParentLinkFrame(jigAssem->GetBaseLink()->GetFrame());
	wJoint->SetChildLinkFrame(SE3());
}

void initDynamics()
{
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, -10.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void robotManagerSetting()
{
	// robot 1
	rManager1 = new indyRobotManager(robot1, &gSpace);

	// robot 2
	rManager2 = new indyRobotManager(robot2, &gSpace);


	rManagerVector[0] = rManager1;
	rManagerVector[1] = rManager2;
}

void setEnviromentFromVision(const vision_data & skku_dataset, int& bNum, int& cNum)
{
	// set object (id:1 - busbar, id:2 - CTcase (insert), id:3 - jig) 
	int bIdx = 0;
	int cIdx = 0;
	for (unsigned int i = 0; i < skku_dataset.objID.size(); i++)
	{
		if (skku_dataset.objID[i] == 1)
		{
			printf("objID: %d, index: %d\n", skku_dataset.objID[i], bIdx);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			busbar[bIdx]->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * busbar[bIdx]->m_visionOffset);
			busbar[bIdx]->KIN_UpdateFrame_All_The_Entity();
			bIdx++;
		}
		else if (skku_dataset.objID[i] == 2)
		{
			printf("objID: %d, index: %d\n", skku_dataset.objID[i], cIdx);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			ctCase[cIdx]->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * ctCase[cIdx]->m_visionOffset);
			ctCase[cIdx]->KIN_UpdateFrame_All_The_Entity();
			cIdx++;
		}
		else if (skku_dataset.objID[i] == 3)
		{
			// only set once when jig is connected to workcell by weld joint
			printf("objID: %d\n", skku_dataset.objID[i]);
			cout << SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) << endl;
			jigAssem->GetBaseLink()->SetFrame(Trobotbase1 * SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]) * jigAssem->m_visionOffset);
			jigAssem->KIN_UpdateFrame_All_The_Entity();
		}
		else if (skku_dataset.objID[i] != 0)
			printf("object ID is outside range!!!\n");
	}
	bNum = bIdx;
	cNum = cIdx;
	// set other objects to far away location
	for (unsigned int i = bIdx; i < busbar.size(); i++)
	{
		busbar[i]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		busbar[i]->KIN_UpdateFrame_All_The_Entity();
	}
	for (unsigned int i = cIdx; i < ctCase.size(); i++)
	{
		ctCase[i]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
		ctCase[i]->KIN_UpdateFrame_All_The_Entity();
	}
	// set obstacle (only set by the first vision input)
	if (!isSystemAssembled)
	{
		obstacle.resize(skku_dataset.obsInfo.size());
		wJoint.resize(skku_dataset.obsInfo.size());
		for (unsigned int i = 0; i < skku_dataset.obsInfo.size(); i++)
		{
			obstacle[i] = new srLink();
			wJoint[i] = new srWeldJoint;
			obstacle[i]->GetGeomInfo().SetDimension(skku_dataset.obsInfo[i][3], skku_dataset.obsInfo[i][4], skku_dataset.obsInfo[i][5]);
			wJoint[i]->SetParentLink(workCell->GetBaseLink());
			wJoint[i]->SetParentLinkFrame(robot1->GetBaseLink()->GetFrame()*SE3(Vec3(skku_dataset.obsInfo[i][0], skku_dataset.obsInfo[i][1], skku_dataset.obsInfo[i][2])));
			wJoint[i]->SetChildLink(obstacle[i]);
			wJoint[i]->SetChildLinkFrame(SE3());
			obstacle[i]->GetGeomInfo().SetColor(0.2, 0.2, 0.2);
		}
	}

	if (saveVisionResult)
	{
		string loc = "../../../data/render_traj/";
		// save object info
		vector<Eigen::VectorXd> objData(0);
		Eigen::VectorXd tempObjData(13);
		for (unsigned int i = 0; i < skku_dataset.objID.size(); i++)
		{
			tempObjData[0] = skku_dataset.objID[i];
			tempObjData.segment(1, 12) = SE3toVectorXd(SKKUtoSE3(skku_dataset.objOri[i], skku_dataset.objPos[i]));
			objData.push_back(tempObjData);
		}
		saveDataToText(objData, loc + "objInit.txt");

		// save obstacle info
		vector<Eigen::VectorXd> obsData(0);
		Eigen::VectorXd tempObsData(6);
		for (unsigned int i = 0; i < skku_dataset.obsInfo.size(); i++)
		{
			for (int j = 0; j < 6; j++)
				tempObsData[j] = skku_dataset.obsInfo[i][j];
			obsData.push_back(tempObsData);
		}
		saveDataToText(obsData, loc + "obstacle.txt");
	}

}


void loadVisionResultFromText(string loc)
{
	vector<Eigen::VectorXd> objData = loadDataFromText(loc + "objInit.txt", 13);
	vector<Eigen::VectorXd> obsData = loadDataFromText(loc + "obstacle.txt", 6);
	int bIdx = 0;
	int cIdx = 0;
	for (unsigned int i = 0; i < objData.size(); i++)
	{
		if (objData[i][0] == 1)
		{
			busbar[bIdx]->GetBaseLink()->SetFrame(Trobotbase1 * VectorXdtoSE3(objData[i].segment(1, 12)) * busbar[bIdx]->m_visionOffset);
			busbar[bIdx]->KIN_UpdateFrame_All_The_Entity();
			bIdx++;
		}
		else if (objData[i][0] == 2)
		{
			ctCase[cIdx]->GetBaseLink()->SetFrame(Trobotbase1 * VectorXdtoSE3(objData[i].segment(1, 12)) * ctCase[cIdx]->m_visionOffset);
			ctCase[cIdx]->KIN_UpdateFrame_All_The_Entity();
			cIdx++;
		}
		else if (objData[i][0] == 3)
		{
			jigAssem->GetBaseLink()->SetFrame(Trobotbase1 * VectorXdtoSE3(objData[i].segment(1, 12)) * jigAssem->m_visionOffset);
			jigAssem->KIN_UpdateFrame_All_The_Entity();
			if (isJigConnectedToWorkCell)
				connectJigToWorkCell();
			else
				gSpace.AddSystem((srSystem*)jigAssem);
		}
		else if (objData[i][0] != 0)
			printf("object ID is outside range!!!\n");
	}

	// set other objects to far away location
	for (unsigned int i = bIdx; i < busbar.size(); i++)
	{
		busbar[i]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -(double)0.1*i)) * initBusbar);
		busbar[i]->KIN_UpdateFrame_All_The_Entity();
	}
	for (unsigned int i = cIdx; i < ctCase.size(); i++)
	{
		ctCase[i]->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 10.0, -(double)0.1*i)) * initBusbar);
		ctCase[i]->KIN_UpdateFrame_All_The_Entity();
	}
	// set obstacle (only set by the first vision input)
	if (!isSystemAssembled)
	{
		obstacle.resize(obsData.size());
		wJoint.resize(obsData.size());
		for (unsigned int i = 0; i < obsData.size(); i++)
		{
			obstacle[i] = new srLink();
			wJoint[i] = new srWeldJoint;
			obstacle[i]->GetGeomInfo().SetDimension(obsData[i][3], obsData[i][4], obsData[i][5]);
			wJoint[i]->SetParentLink(workCell->GetBaseLink());
			wJoint[i]->SetParentLinkFrame(robot1->GetBaseLink()->GetFrame()*SE3(Vec3(obsData[i][0], obsData[i][1], obsData[i][2])));
			wJoint[i]->SetChildLink(obstacle[i]);
			wJoint[i]->SetChildLinkFrame(SE3());
			obstacle[i]->GetGeomInfo().SetColor(0.2, 0.2, 0.2);
		}
	}
}
