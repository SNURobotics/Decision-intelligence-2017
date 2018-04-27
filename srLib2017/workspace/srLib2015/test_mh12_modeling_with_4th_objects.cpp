#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager/IndyRobot.h"
#include "robotManager\MH12Robot.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\environment_4th.h"
#include "robotManager\robotRRTManager.h"
#include "sort_external_lib.h"

#include <random>

// Robot
MH12Robot* NTRobot = new MH12Robot;
MH12RobotManager* rManager1;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
Eigen::VectorXd homePos;

// Planning
robotRRTManager* RRTManager = new robotRRTManager;
vector<vector<Eigen::VectorXd>> traj(0);
vector<bool> attachObject;
vector<SE3> wayPoints(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);



BusBar_HYU* busbar = new BusBar_HYU;
Insert* ctCase = new Insert;
SE3 Tbusbar2gripper_ur = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.025));
SE3 Tbusbar2gripper_tight = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.015));
SE3 TctCase2gripper_ur = EulerZYX(Vec3(SR_PI_HALF, 0.0, SR_PI), Vec3(0.006, 0.031625, 0.015));
SE3 Tbin2gripper_ur = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.05));

// object
Bin* bin = new Bin(0.01);
int nWorkingObj = 3;
vector<workingObject*> workingObj(nWorkingObj);
vector<SE3> initSE3(nWorkingObj);
SE3 TworkingObj2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0., 0.04));
vector<size_t> positionZ_ordered_index; // z축 position 높은 순으로 정렬된 index -> positionZ_ordered_index[i]가 꺼내야하는 object 순서



Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

srLink* ee = new srLink;
srSystem* obs = new srSystem;


void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
int activeJointIdx = 0;

int main(int argc, char **argv)
{

	URrobotSetting();



	gSpace.AddSystem(bin);
	for (int i = 0; i < workingObj.size(); i++)
	{
		workingObj[i] = new workingObject;
		gSpace.AddSystem(workingObj[i]);
	}

	initDynamics();


	URrobotManagerSetting();

	rManager1->setGripperDistance(0.01);
	//busbar->setBaseLinkFrame(URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Inv(Tbusbar2gripper_ur));
	//ctCase->setBaseLinkFrame(URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Inv(TctCase2gripper_ur));
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	rManager1->setJointVal(qval);

	// bin location
	bin->setBaseLinkFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0,0.5)));


	// randomly generate working object location
	vector<pair<double, double>> randNumRangeSet;
	randNumRangeSet.push_back(make_pair(-0.135 + 0.07, 0.135 - 0.07));// x range
	randNumRangeSet.push_back(make_pair(-0.205 + 0.07, 0.205 - 0.07));// y range
	randNumRangeSet.push_back(make_pair(0.003, 0.003 + 0.114));// z range
	randNumRangeSet.push_back(make_pair(-10.0*SR_PI / 180.0, 10.0*SR_PI / 180.0));// theta x range
	randNumRangeSet.push_back(make_pair(-10.0*SR_PI / 180.0, 10.0*SR_PI / 180.0));// theta y range
	randNumRangeSet.push_back(make_pair(0.0, 2 * SR_PI));// theta z range 
	double randNumSet[6];
	while (1)
	{
		for (int k = 0; k < workingObj.size(); k++)
		{
			std::random_device rd;
			std::mt19937 eng(rd());
			for (int i = 0; i < 6; i++)
			{
				std::uniform_real_distribution<> distr(randNumRangeSet[i].first, randNumRangeSet[i].second);
				randNumSet[i] = distr(eng);
			}
			SE3 Tbin2obj = EulerZYX(Vec3(randNumSet[5], randNumSet[4], randNumSet[3]), Vec3(randNumSet[0], randNumSet[1], randNumSet[2]));
			workingObj[k]->setBaseLinkFrame(bin->getBaseLinkFrame()*Tbin2obj);
		}
		cout << "Random object location collision check (if 0, pass): " << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
		if (!gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP())
			break;
	}

	// initial Position setting
	for (int i = 0; i < workingObj.size(); i++)
		initSE3[i] = workingObj[i]->getBaseLinkFrame();


	// ------------------------- Planning setting ------------------------------
	// Object order setting (top object first)
	vector<double> workingObjs_positionZ;
	for (int i = 0; i < workingObj.size(); i++)
	{
		workingObjs_positionZ.push_back(workingObj[i]->getBaseLinkFrame().GetPosition()[2]);
	}
	vector<double> workingObjs_positionZ_sorted;

	sort_external_reverse(workingObjs_positionZ, workingObjs_positionZ_sorted, positionZ_ordered_index);


	// way point setting 

	int nWay = 2 * nWorkingObj;
	vector<bool> includeOri(nWay, true);
	wayPoints.resize(nWay);
	attachObject.resize(nWay);
	for (int i = 0; i < workingObj.size(); i++)
	{
		wayPoints[2 * i] = workingObj[positionZ_ordered_index[i]]->getBaseLinkFrame() * TworkingObj2robotEE;
		attachObject[2 * i] = false;
		SE3 Tbin2objPlacement = EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-0.4, -0.12, 0.10));
		//wayPoints[2 * i + 1] = bin->getBaseLinkFrame()*Tbin2objPlacement*TworkingObj2robotEE;
		wayPoints[2 * i + 1] = Tbin2objPlacement*TworkingObj2robotEE;
		attachObject[2 * i + 1] = true;
	}

	int flag;

	//Eigen::VectorXd goal = rManager1->inverseKin(workingObj[0]->getBaseLinkFrame() * TworkingObj2robotEE, &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag);
	Eigen::VectorXd goal = rManager1->inverseKin(wayPoints[0], &NTRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag);
	cout << "goal position inverse kinematics flag: " << flag << endl;
	rManager1->setJointVal(goal);
	cout << "collision check at the goal position: "<<gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;


	//URrrtSetting();

	//RRTManager->setStartandGoal(qval, goal);

	//cout << RRTManager->checkFeasibility(qval) << RRTManager->checkFeasibility(goal);
	//if (!RRTManager->checkFeasibility(goal) && !RRTManager->checkFeasibility(qval))
	//{
	//	RRTManager->execute(0.1);
	//	traj = RRTManager->extractPath();

	//}

	//rManager1->setJointVal(qval);

	//for (int i = 0; i < 6; i++)
	//{
	//	((srStateJoint*)URRobot->m_KIN_Joints[i])->m_State.m_rValue[0] = qval[i];
	//}
	// SE3 Tend = Trobotbase1 % URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame();
	// cout << Tend << endl;
	// cout << Tend[9] << endl << Tend[10] << endl << Tend[11] << endl;
	// SE3 T = SE3();
	// T.SetOrientation(Exp(Vec3(0.0, SR_PI / sqrt(2), -SR_PI / sqrt(2))));
	// cout << Log( Tend.GetOrientation()) << endl;


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
	//((srStateJoint*)URRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)URRobot->m_KIN_Joints[0])->m_State.m_rValue[0] = JointVal;
	JointVal += 0.01;

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	//if (traj.size() > 0)
	//	rManager1->setJointVal(traj[trajcnt % traj.size()]);


	//cout << URRobot->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << URRobot->gLink[UR3_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);

	// check inv dyn with gripper
	//Eigen::VectorXd gripInput(2);
	//gripInput[0] = -1.5;
	//gripInput[1] = 1.5;
	//rManager1->setGripperInput(gripInput);
	//rManager2->setGripperInput(gripInput);
	////cout << gSpace._KIN_COLLISION_RUNTIME_SIMULATION_LOOP() << endl;
	//Eigen::VectorXd acc(6);
	//acc.setRandom();
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager1->controlJointAcc(acc);
	//rManager2->controlJointTorque(tau);

	//cout << "robot2 acc:  " << rManager2->getJointAcc().transpose() << endl << endl;
	////cout << "robot1 tau:  " << rManager1->getJointTorque().transpose() << endl;
	//cout << "desired acc: " << acc.transpose() << endl;
	////cout << "desired tau: " << tau.transpose() << endl;


	// check sensor val
	//Eigen::VectorXd acc(6);
	//acc.setZero();
	//rManager1->controlJointAcc(acc);
	//Eigen::VectorXd tau = rManager1->inverseDyn(rManager1->getJointVal(), rManager1->getJointVel(), acc);
	//rManager2->controlJointTorque(tau);
	//cout << "Fsensor1: " << rManager1->readSensorValue() << endl;
	//cout << "Fext1:    " << rManager1->m_ftSensorInfo[0]->getExtForce() << endl;
	//cout << "Fsensor2: " << rManager2->readSensorValue() << endl;
	//cout << "Fext2:    " << rManager2->m_ftSensorInfo[0]->getExtForce() << endl;
	int stop = 1;
}


void URrobotSetting()
{
	gSpace.AddSystem((srSystem*)NTRobot);
	NTRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	NTRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	NTRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = NTRobot->GetBaseLink()->GetFrame() * NTRobot->TsrLinkbase2robotbase;
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

void URrobotManagerSetting()
{

	//rManager2 = new robotManager();
	//rManager2->setRobot((srSystem*)robot2);
	//rManager2->setSpace(&gSpace);
	//rManager2->setEndeffector(&robot2->gMarkerLink[Indy_Index::MLINK_GRIP]);

	//// gripper setting
	//vector<srJoint*> gripperJoint(2);
	//gripperJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L];
	//gripperJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U];
	//vector<srJoint*> gripperDummyJoint(2);
	//gripperDummyJoint[0] = robot2->gPjoint[Indy_Index::GRIPJOINT_L_DUMMY];
	//gripperDummyJoint[1] = robot2->gPjoint[Indy_Index::GRIPJOINT_U_DUMMY];
	//rManager2->setGripper(gripperJoint, gripperDummyJoint);

	//// sensor setting
	//rManager2->setFTSensor(robot2->gWeldJoint[Indy_Index::WELDJOINT_GRIPPER]);

	rManager1 = new MH12RobotManager(NTRobot, &gSpace);

}

void URrrtSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(6);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)NTRobot->gJoint[i];
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(NTRobot->getLowerJointLimit(), NTRobot->getUpperJointLimit());
}
