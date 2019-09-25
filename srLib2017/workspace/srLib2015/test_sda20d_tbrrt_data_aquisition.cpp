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
#include "common/dataIO.h"
#include "robotManager/environment_5th.h"

// Robot
SDA20D* sdaRobot = new SDA20D;
SDA20DManager* rManager1;
SDA20DManager* rManager2;
Bin* bin = new Bin(0.01);

Eigen::VectorXd qval;
Eigen::VectorXd qval_bfproj;

srSpace gSpace;
myRenderer* renderer;


srLink* ee = new srLink;
srSystem* obs = new srSystem;
srCollision* colli = new srCollision;
srLink* ee2 = new srLink;
srSystem* obs2 = new srSystem;
srCollision* colli2 = new srCollision;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
MovingContact* obj1 = new MovingContact;
FixedContact* obj2 = new FixedContact;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void sdaRobotSetting();
void sdaRobotManager1Setting(int robotMode, vector<int> excludeNum = vector<int>(0));
void sdaRobotManager2Setting(int robotMode, vector<int> excludeNum = vector<int>(0));
void sdarrtSetting();
void setObstacle();
void setObstacle2();
void setObjects();
int activeJointIdx = 0;
vector<Eigen::VectorXd> traj(0);
vector<Eigen::VectorXd> traj1(0);
vector<Eigen::VectorXd> traj2(0);
Eigen::VectorXd qTemp;
Eigen::VectorXd q;

int useWaist = 1;
Eigen::VectorXd tempJointVal = Eigen::VectorXd::Zero(14 + useWaist);
Eigen::VectorXd tempJointVal2;
bool doPlanning = true;

SDA20DDualArmClosedLoopConstraint* armConstraint;
TBrrtManager* RRTManager = new TBrrtManager(armConstraint);

int main(int argc, char **argv)
{
	srand(0);
	sdaRobotSetting();

	initDynamics();

	obj1->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));
	obj2->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));

	if (useWaist == 0) {
		sdaRobotManager1Setting(SDA20DManager::MoveBothArmOnly);
	}
	else {
		sdaRobotManager1Setting(SDA20DManager::MoveWholeBody);
	}
	vector<int> jointForRight = { 1, 2, 3, 4, 5, 6, 7 };
	sdaRobotManager2Setting(SDA20DManager::MoveBothArmOnly, jointForRight);

	tempJointVal[0 + useWaist] = 1 * SR_PI_HALF;
	tempJointVal[7 + useWaist] = 1 * SR_PI_HALF;
	tempJointVal[1 + useWaist] = 1 * SR_PI_HALF;
	tempJointVal[8 + useWaist] = 1 * SR_PI_HALF;
	tempJointVal[2 + useWaist] = -SR_PI_HALF;
	tempJointVal[9 + useWaist] = -SR_PI_HALF;
	tempJointVal[3 + useWaist] = -SR_PI_HALF;
	tempJointVal[10 + useWaist] = -SR_PI_HALF;
	tempJointVal[5 + useWaist] = SR_PI_HALF;
	tempJointVal[12 + useWaist] = -SR_PI_HALF;
	tempJointVal[4 + useWaist] = SR_PI_HALF;
	tempJointVal[11 + useWaist] = -SR_PI_HALF;
	rManager1->setJointVal(tempJointVal);

	SE3 Tcenter = EulerZYX(Vec3(0, 0, 0), Vec3(0.8+ double(rand() % 1000) / 5000, double(rand() % 1000) / 5000, double(rand() % 1000) / 5000));
	SE3 TtoRight = EulerXYZ(Vec3(-SR_PI_HALF, 0.0, SR_PI_HALF*(1)), Vec3(0.0, -0.2, 0.0));
	SE3 TtoLeft = EulerXYZ(Vec3(SR_PI_HALF, 0.0, SR_PI_HALF*(0)), Vec3(-0.0, 0.2, 0.0));
	SE3 Trotate = EulerZYX(Vec3(0, SR_PI, 0), Vec3(0.0, 0.0, 0.0));

	SE3 Tright;
	SE3 Tleft;

	srand((unsigned int)time(NULL));

	vector<Eigen::VectorXd> outData(0);
	for (int iter = 0; iter < 100; iter++) {
		int feasiblePoints = 0;
		while (!feasiblePoints) {
			Eigen::VectorXd temp = tempJointVal;
			Eigen::VectorXd upper = VecToVector(rManager1->m_upperJointLimit);
			Eigen::VectorXd lower = VecToVector(rManager1->m_lowerJointLimit);
			for (int i = 0; i < temp.size(); i++) {
				int tmp = int((upper[i] - lower[i]) * 1000);
				temp[i] = double(rand() % tmp) / 1000 + lower[i];
			}

			int flag = 0;
			Eigen::VectorXd qRight;
			Eigen::VectorXd qLeft;
			qRight = rManager1->inverseKin(Tcenter*TtoRight, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, temp, 500, robotManager::NR, robotManager::DG);
			//cout << "Right arm IK flag: " << flag << endl;
			rManager1->setJointVal(qRight);
			qLeft = rManager2->inverseKin(Tcenter*TtoLeft, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_LEFT_T], true, SE3(), flag, temp, 500, robotManager::NR, robotManager::DG);
			//cout << "Left arm IK flag: " << flag << endl;
			rManager2->setJointVal(qLeft);
			tempJointVal = rManager1->getJointVal();
			bool isCol_Start = rManager1->checkCollision();

			Tright = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame();
			Tleft = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_LEFT_T].GetFrame();
			//cout << "Const" << endl << Tright % Tleft << endl;

			qRight = rManager1->inverseKin(Tcenter*Trotate*TtoRight, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T], true, SE3(), flag, temp, 500, robotManager::NR, robotManager::DG);
			//cout << "Right arm IK flag: " << flag << endl;
			rManager1->setJointVal(qRight);
			qLeft = rManager2->inverseKin(Tcenter*Trotate*TtoLeft, &sdaRobot->gMarkerLink[SDA20D_Index::MLINK_LEFT_T], true, SE3(), flag, temp, 500, robotManager::NR, robotManager::DG);
			//cout << "Left arm IK flag: " << flag << endl;
			rManager2->setJointVal(qLeft);
			tempJointVal2 = rManager1->getJointVal();
			bool isCol_Goal = rManager1->checkCollision();

			if (!isCol_Start && !isCol_Goal) feasiblePoints = 1;
		}

		// set constraint
		Tright = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame();
		Tleft = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_LEFT_T].GetFrame();
		armConstraint = new SDA20DDualArmClosedLoopConstraint(rManager1, Tright % Tleft);
		printf("constraint vector:\n");
		cout << armConstraint->getConstraintVector(tempJointVal).transpose() << endl;

		cout << "Const" << endl << Tright % Tleft << endl;

		outData.push_back(rManager1->getJointVal());
	}

	saveDataToText(outData, "../../../data/tbrrt_traj/constrained_sample.txt");

	
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
	//obs->GetBaseLink()->SetFrame(sdaRobot->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame());
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;


	if (doPlanning) {
		if (traj.size() > 0)
			rManager1->setJointVal(traj[trajcnt % traj.size()]);
		if (traj2.size() > 0)
			rManager1->setJointVal(traj2[trajcnt % traj2.size()]);
		if (cnt % 10 == 0)
			trajcnt++;
	}
	else
	{
		if (cnt % 100 < 50) {
			//rManager1->setJointVal(qval);
		}
		else
			rManager1->setJointVal(qval_bfproj);
	}

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

	//if (rManager1->checkCollision())
		//cout << "Collision!" << endl;

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

void sdaRobotManager1Setting(int robotMode, vector<int> excludeNum /*= vector<int>(0)*/)
{
	if (excludeNum.size() == 0)
		rManager1 = new SDA20DManager(sdaRobot, &gSpace, robotMode);
	else
	{
		// set non-operating joint
		vector<srJoint*> excludeJoints(0);
		for (int i = 0; i < excludeNum.size(); i++)
			excludeJoints.push_back(sdaRobot->gJoint[excludeNum[i]]);
		rManager1 = new SDA20DManager(sdaRobot, &gSpace, robotMode, excludeJoints);
	}
}

void sdaRobotManager2Setting(int robotMode, vector<int> excludeNum /*= vector<int>(0)*/)
{
	if (excludeNum.size() == 0)
		rManager2 = new SDA20DManager(sdaRobot, &gSpace, robotMode);
	else
	{
		// set non-operating joint
		vector<srJoint*> excludeJoints(0);
		for (int i = 0; i < excludeNum.size(); i++)
			excludeJoints.push_back(sdaRobot->gJoint[excludeNum[i]]);
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

void setObstacle()
{
	ee->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(0.05, 5.0, 0.05);
	Vec3 obs_col_size = Vec3(0.09, 5.0, 0.09);
	ee->GetGeomInfo().SetDimension(obs_size);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	colli->GetGeomInfo().SetDimension(obs_col_size);
	ee->AddCollision(colli);
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs);
	obs->GetBaseLink()->SetFrame(SE3(Vec3(0.9, 0.0, 0.2)));
}
void setObstacle2()
{
	ee2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(2.5, 2.5, 0.03);
	Vec3 obs_col_size = Vec3(2.5, 2.5, 0.09);
	ee2->GetGeomInfo().SetDimension(obs_size);
	ee2->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	colli2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	colli2->GetGeomInfo().SetDimension(obs_col_size);
	ee2->AddCollision(colli2);
	obs2->SetBaseLink(ee2);
	obs2->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs2);
	obs2->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, 0.7)));
}

void setObjects()
{
	gSpace.AddSystem((srSystem*)obj1);
	//gSpace.AddSystem((srSystem*)obj2);
}
