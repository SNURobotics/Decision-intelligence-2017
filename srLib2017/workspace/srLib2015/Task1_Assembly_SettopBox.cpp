#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager\UR3Robot.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include "robotManager\environment_5th.h"
#include <time.h>
#include "robotManager\robotRRTManager.h"
#include "common/dataIO.h"

srSpace gSpace;
myRenderer* renderer;
// Robot
UR3Robot* ur3 = new UR3Robot;
UR3RobotManager* ur3Manager;
UR5Robot* ur5 = new UR5Robot;
UR5RobotManager* ur5Manager;
robotRRTManager* ur3RRTManager = new robotRRTManager;
robotRRTManager* ur5RRTManager = new robotRRTManager;

srLink* floor_link = new srLink;
srSystem* Floor = new srSystem;
srCollision* floor_colli = new srCollision;
void setObstacle();

HDMI* hdmi = new HDMI();
Power* power = new Power();
Settop* settop = new Settop();
Soldering* soldering = new Soldering();
PCB* pcb = new PCB();
PCBJig* pcbjig = new PCBJig();
Tape* tape = new Tape();
BoxForTape* boxfortape = new BoxForTape();


srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 T_ur3base;
SE3 T_ur5base;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
//void tempObjectSetting();
Eigen::VectorXd qval;

Eigen::VectorXd UR5point0;
Eigen::VectorXd UR5point1;
Eigen::VectorXd UR5point2;
Eigen::VectorXd UR3point0;
Eigen::VectorXd UR3point1;
Eigen::VectorXd UR3point2;
Eigen::VectorXd UR3point3;
Eigen::VectorXd UR3point4;
Eigen::VectorXd UR3point5;
Eigen::VectorXd UR3point6;
Eigen::VectorXd UR5init;
vector<Eigen::VectorXd> ur5traj1(0);
vector<Eigen::VectorXd> ur5traj2(0);
vector<Eigen::VectorXd> ur3traj1(0);
vector<Eigen::VectorXd> ur3traj2(0);
vector<Eigen::VectorXd> ur3traj3(0);
vector<Eigen::VectorXd> ur3traj4(0);
vector<Eigen::VectorXd> ur3traj5(0);
vector<Eigen::VectorXd> ur3traj6(0);
vector<SE3> SettupTraj(0);
vector<SE3> HdmiTraj(0);
vector<SE3> HdmiTraj2(0);
vector<SE3> PowerTraj(0);
vector<SE3> PowerTraj2(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
// srSystem* obs = new srSystem;
SE3 Tobs2robot = SE3();
SE3 Tsettop = EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(-0.25, -0.2, 0));
SE3 Tur52settop = EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.05, 0, 0));
SE3 Tsettop2hdmi_init = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.047 + 0.0275, -0.02, 0.0));
SE3 Tsettop2power_init = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.047 + 0.017, 0.017, 0.0));
Eigen::VectorXd ur5_invkinInit = Eigen::VectorXd::Zero(6);
Eigen::VectorXd ur3_invkinInit = Eigen::VectorXd::Zero(6);


vector<Eigen::VectorXd> GripTraj(0);
vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos);

Eigen::VectorXd UR3robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original);
Eigen::VectorXd UR5robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original);

float Settup_x_left = -0.7;
float Settup_x_right = -0.3;
float Settup_y_left = -0.6;
float Settup_y_right = -0.1;


float Hdmi_x_left = 0.2;
float Hdmi_x_right = 0.3;
float Hdmi_y_left = -0.6;
float Hdmi_y_right = -0.4;

float Power_x_left = 0.3;
float Power_x_right = 0.4;
float Power_y_left = -0.6;
float Power_y_right = -0.4;


SE3 hdmi_init;
SE3 power_init;
SE3 settup_init;

int main(int argc, char **argv)
{

	///////////////////////// test file read /////////////////////////
	string str = "../../../data/environment_setting/test.txt";
	vector<int> lineNums(3);
	lineNums[0] = 10;
	lineNums[1] = 13;
	lineNums[2] = 16;
	vector<vector<double>> read = loadDataFromTextSpecifiedLines(str, lineNums);
	//////////////////////////////////////////////////////////////////

	//float SETTUP_x = read[0][0];
	//float SETTUP_y = read[0][1];
	//float HDMI_x = read[1][0];
	//float HDMI_y = read[1][1];
	//float POWER_x = read[2][0];
	//float POWER_y = read[2][1];


	srand((unsigned int)time(NULL));

	// rnadom setting
	int temp1 = int((Settup_x_right - Settup_x_left) * 1000);
	float Settup_rand_x = (float)(rand() % temp1) / (float)1000 + Settup_x_left;
	int temp2 = int((Settup_y_right - Settup_y_left) * 1000);
	float Settup_rand_y = (float)(rand() % temp2) / (float)1000 + Settup_y_left;

	int temp3 = int((Hdmi_x_right - Hdmi_x_left) * 1000);
	float Hdmi_rand_x = (float)((rand() % temp3)) / (float)1000 + Hdmi_x_left;
	int temp4 = int((Hdmi_y_right - Hdmi_y_left) * 1000);
	float Hdmi_rand_y = (float)((rand() % temp4)) / (float)1000 + Hdmi_y_left;

	int temp5 = int((Power_x_right - Power_x_left) * 1000);
	float Power_rand_x = (float)((rand() % temp5)) / (float)1000 + Power_x_left;
	int temp6 = int((Power_y_right - Power_y_left) * 1000);
	float Power_rand_y = (float)((rand() % temp6)) / (float)1000 + Power_y_left;

	float HDMI_x = Hdmi_rand_x;
	float HDMI_y = Hdmi_rand_y;
	float POWER_x = Power_rand_x;
	float POWER_y = Power_rand_y;
	float SETTUP_x = Settup_rand_x;
	float SETTUP_y = Settup_rand_y;
	//

	hdmi_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(HDMI_x, HDMI_y, 0));
	power_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(POWER_x, POWER_y, 0));
	settup_init = SE3(Vec3(SETTUP_x, SETTUP_y, 0));

	float distance_hdmi_power = pow(pow((HDMI_x - POWER_x), 2) + pow((HDMI_y - POWER_y), 2), 0.5);

	if (distance_hdmi_power < 0.02)
	{
		cout << "HDMI and power cable are too close, nominal values are chosen!" << endl;
		hdmi_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(0.25, -0.55, 0));
		power_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(0.35, -0.45, 0));
	}
	if (SETTUP_x < Settup_x_left || Settup_x_right < SETTUP_x || SETTUP_y < Settup_y_left || Settup_y_right < SETTUP_y)
	{
		cout << "Given set-top box position is out of the range, nominal values is chosen" << endl;
		settup_init = SE3(Vec3(-0.5, -0.3, 0));
	}
	if (HDMI_x < Hdmi_x_left || Hdmi_x_right < HDMI_x || HDMI_y < Hdmi_y_left || Hdmi_y_left < HDMI_y)
	{
		cout << "Given HDMI position is out of the range, nominal values is chosen" << endl;
		hdmi_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(0.25, -0.55, 0));
	}
	if (POWER_x < Power_x_left || Power_x_right < POWER_x || POWER_y < Power_y_left || Power_y_right < POWER_y)
	{
		cout << "Given power cable position is out of the range, nominal values is chosen" << endl;
		power_init = EulerXYZ(Vec3(0, 0, -SR_PI_HALF / 2), Vec3(0.35, -0.45, 0));
	}


	// robot, object, environment settings should come before initDynamics()
	URrobotSetting();
	// tempObjectSetting();
	gSpace.AddSystem(hdmi);
	gSpace.AddSystem(power);
	gSpace.AddSystem(settop);

	setObstacle();

	//gSpace.AddSystem(soldering);
	//gSpace.AddSystem(pcb);
	//gSpace.AddSystem(pcbjig);
	//gSpace.AddSystem(tape);
	//gSpace.AddSystem(boxfortape);
	initDynamics();

	// robotManager setting should come after initDynamics()
	URrobotManagerSetting();

	ur3Manager->setGripperDistance(0.01);
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	ur3Manager->setJointVal(qval);


	// rrtSetting should come after robotManager setting
	URrrtSetting();

	// UR5 initial pose
	//cout << ur5Manager->forwardKin(qval, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP]) << endl;

	// Placing env
	hdmi->setBaseLinkFrame(hdmi_init);
	power->setBaseLinkFrame(power_init);
	settop->setBaseLinkFrame(settup_init);
	//soldering->setBaseLinkFrame(ur5Manager->forwardKin(qval, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP]));
	//pcb->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI / 2, 0), Vec3(-0.2, 0.5, 0)));
	//pcbjig->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(-2, -0.5, 0.31)));
	//tape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.5, 0.5, 0)));
	//boxfortape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.4, -0.5, 0)));

	///////////// RRT planning to reach object (1. approaching the settup box & 2. grasping the settup box) ///////////////
	cout << endl << "1. approaching the settup box & 2. grasping the settup box" << endl;
	clock_t start = clock();
	UR5point0 = Eigen::VectorXd::Zero(6);
	int flag = 0;
	UR5point1 = UR5robustInverseKinematics(settop->GetBaseLink()->GetFrame() * Tobs2robot * Inv(SE3(EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.05, 0, 0)))), UR5point0);
	/*cout << ur5Manager->checkCollision() << endl;
	ur5Manager->setJointVal(UR5point1);
	cout << ur5Manager->checkCollision() << endl;*/
	/*
	cout << "inverse kinematics flag: " <<  flag << endl;
	cout << soldering->GetBaseLink()->GetFrame() * Tobs2robot << endl;
	cout << ur5->gMarkerLink[UR5_Index::MLINK_GRIP].GetFrame() << endl;
	*/
	ur5RRTManager->setStartandGoal(UR5point0, UR5point1);
	ur5RRTManager->execute(0.1);
	ur5traj1 = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur5traj1.size(); i++)
	{
		ur5RRTManager->setState(ur5traj1[i]);
		SettupTraj.push_back(settop->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for UR5 with object attached (3. placing the settup box) ///////////////
	cout << endl << "3. placing the settup box" << endl;
	/*Eigen::VectorXd point2(6);
	point2(0) = 0;
	point2(1) = -0.3*SR_PI_HALF;
	point2(2) = 1.5*SR_PI_HALF;
	point2(3) = 0;
	point2(4) = 0;
	point2(5) = 0;*/
	start = clock();
	ur5_invkinInit[0] = -1.754548; ur5_invkinInit[1] = 2.409134; ur5_invkinInit[2] = -1.813758;
	ur5_invkinInit[3] = -2.546216; ur5_invkinInit[4] = -1.754548; ur5_invkinInit[5] = 3.141593;
	UR5point2 = ur5Manager->inverseKin(Tsettop / Tur52settop, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, ur5_invkinInit);
	//point2 = ur5Manager->inverseKin(SE3(EulerXYZ(Vec3(0, 0, 0), Vec3(-0.4, -0.2, 0))), &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag);

	ur5RRTManager->attachObject(settop, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3(EulerXYZ(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0.05, 0, 0))));		// attaching object occurs here
	ur5RRTManager->setStartandGoal(UR5point1, UR5point2);
	ur5RRTManager->execute(0.1);
	ur5traj2 = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur5traj2.size(); i++)
	{
		ur5RRTManager->setState(ur5traj2[i]);
		SettupTraj.push_back(settop->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	///////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR3 with object detached (4. Approaching the hdmi connector & 5. grasping the hdmi connector) ///////////////
	cout << endl << "4. Approaching the hdmi connector & 5. grasping the hdmi connector" << endl;
	start = clock();
	UR3point0 = qval;
	ur3_invkinInit[0] = -2.796488; ur3_invkinInit[1] = -SR_PI_HALF; ur3_invkinInit[2] = 1.787395;
	ur3_invkinInit[3] = -1.75; ur3_invkinInit[4] = -1.570796; ur3_invkinInit[5] = 1.915901;
	UR3point1 = UR3robustInverseKinematics(hdmi->GetBaseLink()->GetFrame() * Inv(SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.)))), UR3point0);


	//cout << "inverse kinematics flag: " <<  flag << endl;

	ur3RRTManager->setStartandGoal(UR3point0, UR3point1);
	ur3RRTManager->execute(0.1);
	ur3traj1 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj1.size(); i++)
	{
		ur3RRTManager->setState(ur3traj1[i]);
	}

	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	///////////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR3 with object detached (6. Bringing the hdmi to the settup box) ///////////////
	cout << endl << "6. Bringing the hdmi to the settup box" << endl;
	start = clock();

	UR3point2 = UR3robustInverseKinematics(Tsettop * Tsettop2hdmi_init *Inv(SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.)))), UR3point1);

	//cout << "inverse kinematics flag: " << flag << endl;

	ur3RRTManager->attachObject(hdmi, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.))));		// attaching object occurs here
	ur3RRTManager->setStartandGoal(UR3point1, UR3point2);
	ur3RRTManager->execute(0.1);
	ur3traj2 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj2.size(); i++)
	{
		ur3RRTManager->setState(ur3traj2[i]);
		HdmiTraj.push_back(hdmi->GetBaseLink()->GetFrame());
	}

	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	///////////////////////////////////////////////////////////////////////////////


	////////////////// RRT planning for UR3 with object detached (7. Pusing the hdmi into the settup box) ///////////////
	cout << endl << "7. Pusing the hdmi into the settup box" << endl;
	start = clock();

	//cout << "inverse kinematics flag: " << flag << endl;
	UR3point3 = UR3point2;
	for (unsigned int i = 0; i < 10; i++)
	{
		UR3point3 = ur3Manager->inverseKin(SE3(Vec3(0.0, 0.0, -0.001*i))*Tsettop * Tsettop2hdmi_init, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.))), flag, UR3point3);	ur3traj3.push_back(UR3point3);
		HdmiTraj2.push_back(SE3(Vec3(0.0, 0.0, -0.001*i)) *Tsettop * Tsettop2hdmi_init * SE3(EulerXYZ(Vec3(-0, 0, -0), Vec3(0, 0, 0.))));
	}

	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	///////////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR3 with object detached (8. Approaching the power cable & 9. grasping the power cable) ///////////////
	cout << endl << "8. Approaching the power cable & 9. grasping the power cable" << endl;
	start = clock();
	ur3RRTManager->detachObject();

	UR3point4 = UR3robustInverseKinematics(power->GetBaseLink()->GetFrame() * Inv(SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0)))), UR3point3);
	//cout << "inverse kinematics flag: " << flag << endl;

	ur3RRTManager->setStartandGoal(UR3point3, UR3point4);
	ur3RRTManager->execute(0.1);
	ur3traj4 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj4.size(); i++)
	{
		ur3RRTManager->setState(ur3traj4[i]);
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	/////////////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR3 with object detached (10. Bringing the cable to the settup box) ///////////////
	cout << endl << "10. Bringing the cable to the settup box" << endl;
	start = clock();
	ur3RRTManager->attachObject(power, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.))));		// attaching object occurs here

	UR3point5 = UR3robustInverseKinematics(Tsettop * Tsettop2power_init * Inv(SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0)))), UR3point4);
	//cout << "inverse kinematics flag: " << flag << endl;

	ur3RRTManager->setStartandGoal(UR3point4, UR3point5);
	ur3RRTManager->execute(0.1);
	ur3traj5 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj5.size(); i++)
	{
		ur3RRTManager->setState(ur3traj5[i]);
		PowerTraj.push_back(power->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	/////////////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR3 with object detached (11. Pushing the cable into the settup box) ///////////////
	cout << endl << "11. Pushing the cable into the settup box" << endl;
	start = clock();

	//cout << "inverse kinematics flag: " << flag << endl;
	UR3point6 = UR3point5;
	for (unsigned int i = 0; i < 10; i++)
	{
		UR3point6 = ur3Manager->inverseKin(SE3(Vec3(0.0, 0.0, -0.001*i))*Tsettop * Tsettop2power_init, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(EulerXYZ(Vec3(-SR_PI_HALF, 0, -SR_PI_HALF), Vec3(0, 0, 0.))), flag, UR3point6);
		ur3traj6.push_back(UR3point6);
		PowerTraj2.push_back(SE3(Vec3(0.0, 0.0, -0.001*i)) * Tsettop * Tsettop2power_init * SE3(EulerXYZ(Vec3(-0, 0, -0), Vec3(0, 0, 0.))));
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << " error:0, success" << endl;
	/////////////////////////////////////////////////////////////////////////////////

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

	ur3Manager->setJointVal(qval);
	/*UR5init = UR5robustInverseKinematics(SE3(Vec3(0.25, 0.2, -0.4)) * ur5Manager->forwardKin(UR5point0, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP]), UR5point0);
	ur5Manager->setJointVal(UR5init);
	*/
	hdmi->setBaseLinkFrame(hdmi_init);
	power->setBaseLinkFrame(power_init);
	settop->setBaseLinkFrame(settup_init);
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
	//GripTraj = makeGriptraj(30, tempTraj.back());
	//ur5traj.insert(ur5traj.end(), GripTraj.begin(), GripTraj.end());
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;

	// plot planned trajectory

	double graspAngle = 0.6;
	if (trajcnt < ur5traj1.size())
	{
		ur5Manager->setJointVal(ur5traj1[trajcnt % ur5traj1.size()]);

		settop->GetBaseLink()->SetFrame(SettupTraj[trajcnt % ur5traj1.size()]);
		if (trajcnt == ur5traj1.size() - 1) {
			Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(3);
			tempPos(0) = -graspAngle;
			tempPos(1) = -graspAngle;
			tempPos(2) = graspAngle;
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
		}
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size())
	{
		ur5Manager->setJointVal(ur5traj2[(trajcnt - ur5traj1.size()) % ur5traj2.size()]);
		settop->GetBaseLink()->SetFrame(SettupTraj[trajcnt % (ur5traj1.size() + ur5traj2.size())]);
		if (trajcnt == ur5traj1.size() + ur5traj2.size() - 1) {
			Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(3);
			tempPos(0) = -0.629;
			tempPos(1) = -0.629;
			tempPos(2) = 0.629;
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
			((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
		}
	}

	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size())
	{
		ur3Manager->setJointVal(ur3traj1[(trajcnt - ur5traj1.size() - ur5traj2.size()) % ur3traj1.size()]);
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size())
	{
		ur3Manager->setJointVal(ur3traj2[(trajcnt - ur5traj1.size() - ur5traj2.size() - ur3traj1.size()) % ur3traj2.size()]);
		hdmi->GetBaseLink()->SetFrame(HdmiTraj[trajcnt % (ur5traj1.size() + ur5traj2.size() + ur3traj1.size())]);
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size())
	{
		ur3Manager->setJointVal(ur3traj3[(trajcnt - ur5traj1.size() - ur5traj2.size() - ur3traj1.size() - ur3traj2.size()) % ur3traj3.size()]);
		hdmi->GetBaseLink()->SetFrame(HdmiTraj2[trajcnt % (ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size())]);
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size() + ur3traj4.size())
	{
		ur3Manager->setJointVal(ur3traj4[(trajcnt - ur5traj1.size() - ur5traj2.size() - ur3traj1.size() - ur3traj2.size() - ur3traj3.size()) % ur3traj4.size()]);
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size() + ur3traj4.size() + ur3traj5.size())
	{
		ur3Manager->setJointVal(ur3traj5[(trajcnt - ur5traj1.size() - ur5traj2.size() - ur3traj1.size() - ur3traj2.size() - ur3traj3.size() - ur3traj4.size()) % ur3traj5.size()]);
		power->GetBaseLink()->SetFrame(PowerTraj[trajcnt % (ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size() + ur3traj4.size())]);
	}
	else if (trajcnt < ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size() + ur3traj4.size() + ur3traj5.size() + ur3traj6.size())
	{
		ur3Manager->setJointVal(ur3traj6[(trajcnt - ur5traj1.size() - ur5traj2.size() - ur3traj1.size() - ur3traj2.size() - ur3traj3.size() - ur3traj4.size() - ur3traj5.size()) % ur3traj6.size()]);
		power->GetBaseLink()->SetFrame(PowerTraj2[trajcnt % (ur5traj1.size() + ur5traj2.size() + ur3traj1.size() + ur3traj2.size() + ur3traj3.size() + ur3traj4.size() + ur3traj5.size())]);
	}
}


void URrobotSetting()
{

	// ur3 setting
	gSpace.AddSystem((srSystem*)ur3);
	ur3->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	ur3->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	ur3->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	T_ur3base = ur3->GetBaseLink()->GetFrame() * ur3->TsrLinkbase2robotbase;

	// ur5 setting
	gSpace.AddSystem((srSystem*)ur5);
	ur5->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0)));
	ur5->SetActType(srJoint::ACTTYPE::HYBRID);
	// add gripper setting for ur5 later
	//vector<int> gpIdx(2);
	//gpIdx[0] = 0;
	//gpIdx[1] = 1;
	//ur5->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);
	T_ur5base = ur5->GetBaseLink()->GetFrame() * ur5->TsrLinkbase2robotbase;


}

void URrobotManagerSetting()
{
	ur3Manager = new UR3RobotManager(ur3, &gSpace);
	ur5Manager = new UR5RobotManager(ur5, &gSpace);
}

void URrrtSetting()
{
	ur3RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> ur3planningJoint(6);
	for (int i = 0; i < 6; i++)
		ur3planningJoint[i] = (srStateJoint*)ur3->gJoint[i];
	ur3RRTManager->setSystem(ur3planningJoint);
	ur3RRTManager->setStateBound(ur3->getLowerJointLimit(), ur3->getUpperJointLimit());

	ur5RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> ur5planningJoint(6);
	for (int i = 0; i < 6; i++)
		ur5planningJoint[i] = (srStateJoint*)ur5->gJoint[i];
	ur5RRTManager->setSystem(ur5planningJoint);
	ur5RRTManager->setStateBound(ur5->getLowerJointLimit(), ur5->getUpperJointLimit());

	ur3RRTManager->printIter = false;
	ur3RRTManager->printFinish = false;
	ur3RRTManager->printDist = false;
	ur5RRTManager->printIter = false;
	ur5RRTManager->printFinish = false;
	ur5RRTManager->printDist = false;
}

void setObstacle()
{
	floor_link->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(5.0, 5.0, 0.05);
	Vec3 obs_col_size = Vec3(5.0, 5.0, 0.05);
	floor_link->GetGeomInfo().SetDimension(obs_size);
	floor_link->GetGeomInfo().SetColor(1, 1, 1);
	floor_colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	floor_colli->GetGeomInfo().SetDimension(obs_col_size);
	floor_link->AddCollision(floor_colli);
	Floor->SetBaseLink(floor_link);
	Floor->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(Floor);
	Floor->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -0.20)));
}

//void tempObjectSetting()
//{
//	double dim = 0.05;
//	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
//	ee->GetGeomInfo().SetDimension(dim);
//	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
//	srCollision* tempCol = new srCollision;
//	tempCol->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
//	tempCol->GetGeomInfo().SetDimension(dim);
//	ee->AddCollision(tempCol);
//	settop->SetBaseLink(ee);
//	settop->SetBaseLinkType(srSystem::FIXED);
//	gSpace.AddSystem(settop);
//}

Eigen::VectorXd UR3robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur3->getLowerJointLimit();
	Eigen::VectorXd upper = ur3->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur3Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < 24; i++) {
		Eigen::VectorXd tempsol;
		for (int j = 0; j < lower.size(); j++) {
			int temp = int((upper[j] - lower[j]) * 1000);
			initial[j] = (rand() % temp) / 1000 + lower[j];
		}
		tempsol = ur3Manager->inverseKin(finalpos, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag, initial);
		//cout << "inverse kinematics flag: " << flag << endl;

		ur3Manager->setJointVal(tempsol);
		if (!ur3Manager->checkCollision() && flag == 0) {
			solList.push_back(tempsol);
		}
	}
	if (solList.size() == 0) {
		cout << "Cannot find solution..." << endl;
		ur3Manager->setJointVal(currentpos);
		return Eigen::VectorXd::Zero(lower.size());
	}
	else {
		double temp = 10000;
		double newsize = 0;
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(lower.size());
		for (int i = 0; i < solList.size(); i++) {
			newsize = (solList[i] - original).squaredNorm();
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}

Eigen::VectorXd UR5robustInverseKinematics(SE3 finalpos, Eigen::VectorXd original)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur5->getLowerJointLimit();
	Eigen::VectorXd upper = ur5->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur5Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < 24; i++) {
		Eigen::VectorXd tempsol;
		for (int j = 0; j < lower.size(); j++) {
			int temp = int((upper[j] - lower[j]) * 1000);
			initial[j] = (rand() % temp) / 1000 + lower[j];
		}
		tempsol = ur5Manager->inverseKin(finalpos, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag, initial);
		//cout << "inverse kinematics flag: " << flag << endl;

		ur5Manager->setJointVal(tempsol);
		if (!ur5Manager->checkCollision() && flag == 0) {
			solList.push_back(tempsol);
			//ur5Manager->setJointVal(currentpos);
			//return tempsol;
		}
	}
	if (solList.size() == 0) {
		cout << "Cannot find solution..." << endl;
		ur5Manager->setJointVal(currentpos);
		return Eigen::VectorXd::Zero(lower.size());
	}
	else {
		double temp = 10000;
		double newsize = 0;
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(lower.size());
		for (int i = 0; i < solList.size(); i++) {
			newsize = (solList[i] - original).squaredNorm();
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}

vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos)
{
	vector<Eigen::VectorXd> gripTraj(0);
	for (int i = 0; i < 10; i++)
	{
		Eigen::VectorXd tempPos = currentPos;
		cout << tempPos << endl;
		tempPos[6] += gripangle / 180 * SR_PI / 10;
		tempPos[10] += gripangle / 180 * SR_PI / 10;
		tempPos[14] += gripangle / 180 * SR_PI / 10;
		gripTraj.push_back(tempPos);
	}
	return gripTraj;
}

