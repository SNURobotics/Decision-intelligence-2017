#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\SDA20DRobotManager.h"
#include "robotManager\SDA20DRobot.h"
#include "robotManager\environment_4th.h"
#include <time.h>
#include <ctime>

// Robot
SDA20D* sdaRobot = new SDA20D;
SDA20DManager* rManager1;
SDA20DManager* rManager2;
Bin* bin = new Bin(0.01);

Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

rrtManager* RRTManager = new rrtManager;

srLink* ee = new srLink;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void sdaRobotSetting();
void sdaRobotManagerSetting(int robotMode, int excludeNum = 0);
void sdarrtSetting();
int activeJointIdx = 0;
vector<Eigen::VectorXd> traj(0);
Eigen::VectorXd qTemp;
Eigen::VectorXd q;
Eigen::VectorXd zeroPoint(15);

vector<Eigen::VectorXd> angleData(10);


int main(int argc, char **argv)
{
	srand(time(NULL));
	sdaRobotSetting();

	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	ee->GetGeomInfo().SetDimension(0.03);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	//gSpace.AddSystem(obs);
	//gSpace.AddSystem(bin);

	initDynamics();
	int excludeNum = 0;
	sdaRobotManagerSetting(SDA20DManager::MoveWholeBody);

	sdarrtSetting();

	//busbar->setBaseLinkFrame(sdaRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_ur));
	//ctCase->setBaseLinkFrame(sdaRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() * Inv(TctCase2gripper_ur));
	//qval.setZero(6);
	//qval[0] = SR_PI_HALF/3;
	//qval[1] = SR_PI_HALF/4;
	//qval[2] = SR_PI_HALF/7;
	//qval[3] = SR_PI_HALF/2;
	//qval[4] = SR_PI_HALF/6;
	//qval[5] = SR_PI_HALF/5;

	qTemp = Eigen::VectorXd::Random(rManager1->m_activeArmInfo->m_numJoint);
	Eigen::VectorXd qTemp2 = Eigen::VectorXd::Random(rManager1->m_activeArmInfo->m_numJoint);

	SE3 Ttemp = rManager1->forwardKin(qTemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);
	int flag;
	int flag2;
	int flag3;
	std::clock_t start = std::clock();
	//Ttemp = Ttemp * SE3(Vec3(100.0, 0.0, 0.0));
	rManager1->setJointVal(Eigen::VectorXd::Zero(rManager1->m_activeArmInfo->m_numJoint));
	cout << rManager2->qInvKinInitActiveJoint.transpose() << endl;
	q = rManager2->inverseKin(Ttemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], false, SE3(), flag, rManager2->qInvKinInitActiveJoint, 500, robotManager::QP, robotManager::DG);
	//q = rManager2->inverseKin(Ttemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, q, 500, robotManager::NR, robotManager::DG);
	Eigen::VectorXd q2 = rManager2->inverseKin(Ttemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], false, SE3(), flag2, Eigen::VectorXd::Zero(6), 500, robotManager::QP, robotManager::DG);
	cout << "elapsed time for invKin: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;

	start = std::clock();
	//double mainp = rManager1->manipulability(qTemp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], robotManager::VOL);
	cout << "elapsed time for manip: " << (std::clock() - start) / (double)CLOCKS_PER_SEC << endl;

	cout << qTemp.transpose() << endl;
	cout << Ttemp << endl;
	cout << q.transpose() << endl;
	SE3 T = rManager2->forwardKin(q, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T]);
	cout << T << endl;

	//RRTManager->setStartandGoal(qTemp, qTemp2);
	//cout << RRTManager->setState(qTemp) << RRTManager->setState(qTemp2);
	//RRTManager->execute(0.1);
	////rManager1->setJointVal(qval);
	//obs->GetBaseLink()->SetFrame(sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame());
	//bin->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));
	//if (RRTManager->isExecuted())
	//{
	//	traj = RRTManager->extractPath();
	//}

	//rManager2->setJointVal(q);

	//SE3 temp = SE3(Vec3(0.0, -0.1, 0.3));
	//Eigen::VectorXd qq = rManager1->inverseKin(temp, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], false, SE3(), flag, rManager1->qInvKinInitActiveJoint, 500, robotManager::QP, robotManager::DG);

	//vector<Eigen::VectorXd> angleData(10);

	vector<Eigen::VectorXd> rawData(10);
	ifstream in("../../../data/NT_Data/Data.txt");
	string str;
	for (int i = 0; i < 10; i++)
	{
		string a;
		string b;
		rawData[i] = Eigen::VectorXd(15);
		for (int j = 0; j < 15; j++)
		{
			getline(in, str, ',');
			rawData[i][j] = atof((char*)str.c_str());
		}
	}
	for (int i = 0; i < 10; i++)
	{
		angleData[i] = Eigen::VectorXd(15);
		angleData[i][0] = rawData[i][0];
		for (int j = 1; j < 3; j++)
		{
			angleData[i][j] = rawData[i][j];
			angleData[i][j + 7] = rawData[i][j + 7];
		}
		angleData[i][3] = rawData[i][7];
		angleData[i][3 + 7] = rawData[i][7 + 7];
		for (int j = 4; j < 8; j++)
		{
			angleData[i][j] = rawData[i][j - 1];
			angleData[i][j + 7] = rawData[i][j + 7 - 1];
		}
	}
	cout << angleData[0] << endl<< endl;
	cout << angleData[1] << endl<< endl;
	cout << angleData[2] << endl<< endl;
	cout << angleData[3] << endl<< endl;
	cout << angleData[4] << endl<< endl;
	zeroPoint << 1993, -62195, 91219, -96329, 84423, -4091, 77212, -50749,
					   -60931, 93994, -93520, 80367, 1291, 82073, -50613;
	//zeroPoint << 1993, -62195, 91219, -96329, 84423, -4091, 77212, -50749,
	//	-184320, 93994, -93520, 80368, 1291, 82072, -50614;

	Eigen::VectorXd weight(15);
	weight << 180 / 324576.0,
			180 / 184320.0, 110 / 112640.0, -170 / 174080.0, -130 / 133120.0, -180 / 206848.0, 110 / 126408.0, 180 / 104448.0,
			180 / 184320.0, 110 / 112640.0, -170 / 174080.0, -130 / 133120.0, -180 / 206848.0, 110 / 126408.0, 180 / 104448.0;
	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			angleData[i][j] = DEG2RAD(angleData[i][j]) * weight[j];
		}
	}
	for (int i = 0; i < 15; i++) {
		zeroPoint[i] = DEG2RAD(zeroPoint[i]) * weight[i];
	}

	Eigen::VectorXd qq(15);
	qq.setZero();
	qq(0) = DEG2RAD(30);
	qq(1) = DEG2RAD(0);
	qq(2) = DEG2RAD(0);
	qq(3) = DEG2RAD(0);
	qq(4) = DEG2RAD(0);
	qq(5) = DEG2RAD(0);
	qq(6) = DEG2RAD(0);
	qq(7) = DEG2RAD(0);
	qq(8) = DEG2RAD(0);
	qq(9) = DEG2RAD(0);
	qq(10) = DEG2RAD(0);
	qq(11) = DEG2RAD(0);
	qq(12) = DEG2RAD(0);
	qq(13) = DEG2RAD(0);
	qq(14) = DEG2RAD(0);
	qq = angleData[0];
	cout << angleData[0] << endl;
	rManager1->setJointVal(qq);
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

	static double JointVal = 0;
	//((srStateJoint*)sdaRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)sdaRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	obs->GetBaseLink()->SetFrame(sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame());
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 100 == 0)
		trajcnt++;

	rManager1->setJointVal(angleData[trajcnt % angleData.size()]);
	//rManager1->setJointVal(zeroPoint);

	//if (cnt % 100 == 0)
	//	trajcnt++;
	//if (trajcnt % 2 == 0)
	//	rManager1->setJointVal(qTemp);
	//else
	//{
	//	rManager1->setJointVal(Eigen::VectorXd::Zero(7));
	//	rManager2->setJointVal(q);
	//}

	//cout << sdaRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << sdaRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);


	int stop = 1;
}


void sdaRobotSetting()
{
	gSpace.AddSystem((srSystem*)sdaRobot);
	sdaRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	sdaRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	sdaRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = sdaRobot->GetBaseLink()->GetFrame() * sdaRobot->TsrLinkbase2robotbase;
	//robot1->SetActType(srJoint::ACTTYPE::HYBRID);
	//robot2->SetActType(srJoint::ACTTYPE::TORQUE);
	//vector<int> gpIdx(2);
	//gpIdx[0] = 0;
	//gpIdx[1] = 1;
	//robot1->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::TORQUE, gpIdx);
	//gpIdx[0] = 2;
	//gpIdx[1] = 3;
	//robot1->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	//robot2->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
}

void sdaRobotManagerSetting(int robotMode, int excludeNum /*= 0*/)
{
	rManager1 = new SDA20DManager(sdaRobot, &gSpace, robotMode);
	if (excludeNum == 0)
		rManager2 = new SDA20DManager(sdaRobot, &gSpace, robotMode);
	else
	{
		// set non-operating joint
		vector<srJoint*> excludeJoints(0);
		excludeJoints.push_back(sdaRobot->gJoint[excludeNum]);
		rManager2 = new SDA20DManager(sdaRobot, &gSpace, robotMode, excludeJoints);
	}
}

void sdarrtSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(0);
	for (int i = 0; i < rManager1->m_activeArmInfo->m_numJoint; i++)
		planningJoint.push_back((srStateJoint*)rManager1->m_activeArmInfo->m_activeJoint[i]);
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(VecToVector(rManager1->m_lowerJointLimit), VecToVector(rManager1->m_upperJointLimit));
}


