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
void setFloor();

//HDMI* hdmi = new HDMI();
//Power* power = new Power();
//Settop* settop = new Settop();
Soldering* soldering = new Soldering();
PCB* pcb = new PCB();
PCBJig* pcbjig = new PCBJig();
//Tape* tape = new Tape();
//BoxForTape* boxfortape = new BoxForTape();


srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 T_ur3base;
SE3 T_ur5base;

void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
void tempObjectSetting();
Eigen::VectorXd qval;

Eigen::VectorXd point0;
Eigen::VectorXd point1;
Eigen::VectorXd point2;
Eigen::VectorXd point3;
vector<Eigen::VectorXd> ur5traj1(0);
vector<Eigen::VectorXd> ur5traj2(0);
vector<Eigen::VectorXd> ur5traj3(0);
vector<Eigen::VectorXd> ur3traj1(0);
vector<Eigen::VectorXd> ur3traj2(0);
vector<Eigen::VectorXd> ur3traj3(0);
vector<SE3> objTraj(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
srSystem* obs = new srSystem;
SE3 Tobs2robot = SE3();

vector<Eigen::VectorXd> GripTraj(0);
vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos);

int main(int argc, char **argv)
{
	///////////////////////// test file read /////////////////////////
	string str = "../../../data/environment_setting/soldering.txt";
	vector<int> lineNums(2);
	lineNums[0] = 1;
	lineNums[1] = 3;
	vector<vector<double>> poss = loadDataFromTextSpecifiedLines(str, lineNums);
	//////////////////////////////////////////////////////////////////

	srand(NULL);
	// robot, object, environment settings should come before initDynamics()
	URrobotSetting();
	tempObjectSetting();
	//gSpace.AddSystem(hdmi);
	//gSpace.AddSystem(power);
	//gSpace.AddSystem(settop);
	//gSpace.AddSystem(soldering);
	gSpace.AddSystem(pcb);
	gSpace.AddSystem(pcbjig);
	//gSpace.AddSystem(tape);
	//gSpace.AddSystem(boxfortape);

	setFloor();

	initDynamics();

	// robotManager setting should come after initDynamics()
	URrobotManagerSetting();

	ur3Manager->setGripperDistance(0.0);
	qval.setZero(6);
	qval[1] = -SR_PI_HALF;
	qval[3] = -SR_PI_HALF;
	qval[4] = SR_PI_HALF;
	ur3Manager->setJointVal(qval);

	// rrtSetting should come after robotManager setting
	URrrtSetting();

	// place object in space
	obs->GetBaseLink()->SetFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.5, -0.8, 0.12)));
	cout << ur3Manager->forwardKin(qval, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP]) << endl;


	//hdmi->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, SR_PI_HALF), Vec3(-0.2, -0.5, 0)));
	//power->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, SR_PI_HALF), Vec3(-0.3, -0.5, 0)));
	//settop->setBaseLinkFrame(SE3(Vec3(-0.5, -0.3, 0)));
	//soldering->setBaseLinkFrame(ur3Manager->forwardKin(qval, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP]));
	pcb->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.0, 0.35, 0.12)));
	pcbjig->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(poss[0][0], poss[0][1], 0.31)));
	//tape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.5, 0.5, 0)));
	//boxfortape->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, -SR_PI_HALF), Vec3(-0.4, -0.5, 0)));

	//ur5Manager->setJointVal(Eigen::VectorXd::Zero(6));
	//ur5RRTManager->attachObject(soldering, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0, -0, 0)));		// attaching object occurs here
	//ur5RRTManager->setState(Eigen::VectorXd::Zero(6));

	Tobs2robot = EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.08, 0, 0));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	clock_t start = clock();
	point0 = qval;
	int flag = 0;
	point1 = ur3Manager->inverseKin(pcb->GetBaseLink()->GetFrame() * Tobs2robot, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag);
	cout << "inverse kinematics flag: " << flag << endl;
	cout << pcb->GetBaseLink()->GetFrame() * Tobs2robot << endl;
	cout << ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() << endl;
	ur3RRTManager->setStartandGoal(point0, point1);
	ur3RRTManager->execute(0.1);
	ur3traj1 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj1.size(); i++)
	{
		ur3RRTManager->setState(ur3traj1[i]);
		objTraj.push_back(pcb->GetBaseLink()->GetFrame());
	}
	double time1 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error1 = (point1 - ur3traj1[ur3traj1.size() - 1]).norm() / point1.norm();
	cout << "time for planning: " << time1 << endl;
	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for ur3 with object attached (point1 -> point2) ///////////////
	start = clock();
	SE3 Tmid = EulerXYZ(Vec3(0, -SR_PI_HALF, 0), Vec3(-0.6, -0.6, 0.7));
	point2 = ur3Manager->inverseKin(Tmid, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag, ur3traj1[ur3traj1.size()-1], 1500);
	cout << "inverse kinematics flag: " << flag << endl;
	ur3RRTManager->attachObject(pcb, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], Inv(Tobs2robot));		// attaching object occurs here
	ur3RRTManager->setStartandGoal(point1, point2);
	ur3RRTManager->execute(0.1);
	ur3traj2 = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur3traj2.size(); i++)
	{
		ur3RRTManager->setState(ur3traj2[i]);
		objTraj.push_back(pcb->GetBaseLink()->GetFrame());
	}
	double time2 = (clock() - start) / (double)CLOCKS_PER_SEC;
	double error2 = (point2 - ur3traj2[ur3traj2.size() - 1]).norm() / point2.norm();
	cout << "time for planning: " << time2 << endl;

	///////////////////////////////////////////////////////////////////////////////
	
	string out_line;
	ofstream out("../../../data/environment_setting/soldering_No1_output.txt");
	for (int i = 0; i < ur3Manager->m_lowerJointLimit.size(); i++) {
		out << ur3Manager->getJointVal()[i] << endl;
	}

	cout << fixed;
	cout.precision(2);
	cout << endl;
	cout << "1. Approch to PCB plate" << endl;
	cout << "time for planning: " << time1 << " error:" << error1 << ", ";
	if (error1 * 100 <= 5 && time1 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;
	cout << "2. Grab PCB plate & 3. move it to zig & 4. hold it" << endl;
	cout << "time for planning: " << time2 << " error:" << error2 << ", ";
	if (error2 * 100 <= 5 && time2 <= 0.6) cout << "success";
	else cout << "fail";
	cout << endl << endl;

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
	//GripTraj = makeGriptraj(30, tempTraj.back());
	//ur3traj.insert(ur3traj.end(), GripTraj.begin(), GripTraj.end());
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;

	// plot planned trajectory

	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] << endl;
	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] << endl;
	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] << endl;
	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[3]))->m_State.m_rValue[0] << endl;
	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[4]))->m_State.m_rValue[0] << endl;
	//cout << ((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[5]))->m_State.m_rValue[0] << endl;
	Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(6);
	double graspAngle = 0.2;
	double idleAngle = -0.825148;
	if (trajcnt == 1)
	{
		tempPos(0) = -graspAngle;
		tempPos(1) = -graspAngle;
		tempPos(2) = graspAngle;
		tempPos(3) = graspAngle;
		tempPos(4) = graspAngle;
		tempPos(5) = -graspAngle;
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[3]))->m_State.m_rValue[0] = tempPos(3);
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[4]))->m_State.m_rValue[0] = tempPos(4);
		((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[5]))->m_State.m_rValue[0] = tempPos(5);
	}
	if (trajcnt < ur3traj1.size())
	{
		ur3Manager->setJointVal(ur3traj1[trajcnt % ur3traj1.size()]);
		pcb->GetBaseLink()->SetFrame(objTraj[trajcnt % ur3traj1.size()]);
		if (trajcnt == ur3traj1.size() - 1) {
			tempPos(0) = idleAngle;
			tempPos(1) = idleAngle;
			tempPos(2) = -idleAngle;
			tempPos(3) = -idleAngle;
			tempPos(4) = -idleAngle;
			tempPos(5) = idleAngle;
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[3]))->m_State.m_rValue[0] = tempPos(3);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[4]))->m_State.m_rValue[0] = tempPos(4);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[5]))->m_State.m_rValue[0] = tempPos(5);
		}
	}
	else if (trajcnt < ur3traj1.size() + ur3traj2.size())
	{
		ur3Manager->setJointVal(ur3traj2[(trajcnt - ur3traj1.size()) % ur3traj2.size()]);
		pcb->GetBaseLink()->SetFrame(objTraj[trajcnt % (ur3traj1.size() + ur3traj2.size())]);
		if (trajcnt == ur3traj1.size() + ur3traj2.size() - 1) {
			tempPos(0) = idleAngle;
			tempPos(1) = idleAngle;
			tempPos(2) = -idleAngle;
			tempPos(3) = -idleAngle;
			tempPos(4) = -idleAngle;
			tempPos(5) = idleAngle;
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[3]))->m_State.m_rValue[0] = tempPos(3);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[4]))->m_State.m_rValue[0] = tempPos(4);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[5]))->m_State.m_rValue[0] = tempPos(5);
		}
	}
	else if (trajcnt < ur3traj1.size() + ur3traj2.size() + ur3traj3.size())
	{
		ur3Manager->setJointVal(ur3traj3[(trajcnt - ur3traj1.size() - ur3traj2.size()) % ur3traj3.size()]);
		pcb->GetBaseLink()->SetFrame(objTraj[trajcnt % (ur3traj1.size() + ur3traj2.size() + ur3traj3.size())]);
		if (trajcnt == ur3traj1.size() + ur3traj2.size() + ur3traj3.size() - 1) {
			tempPos(0) = idleAngle;
			tempPos(1) = idleAngle;
			tempPos(2) = -idleAngle;
			tempPos(3) = -idleAngle;
			tempPos(4) = -idleAngle;
			tempPos(5) = idleAngle;
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[3]))->m_State.m_rValue[0] = tempPos(3);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[4]))->m_State.m_rValue[0] = tempPos(4);
			((srStateJoint*)(ur3Manager->m_gripperInfo->m_gripJoint[5]))->m_State.m_rValue[0] = tempPos(5);

			trajcnt = 0;
		}
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
}

void tempObjectSetting()
{
	double dim = 0.05;
	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	ee->GetGeomInfo().SetDimension(dim);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	srCollision* tempCol = new srCollision;
	tempCol->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	tempCol->GetGeomInfo().SetDimension(dim);
	ee->AddCollision(tempCol);
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs);
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

void setFloor()
{
	floor_link->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(5.0, 5.0, 0.05);
	Vec3 obs_col_size = Vec3(5.0, 5.0, 0.05);
	floor_link->GetGeomInfo().SetDimension(obs_size);
	floor_link->GetGeomInfo().SetColor(1.0, 1.0, 1.0);
	floor_colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	floor_colli->GetGeomInfo().SetDimension(obs_col_size);
	floor_link->AddCollision(floor_colli);
	Floor->SetBaseLink(floor_link);
	Floor->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(Floor);
	Floor->GetBaseLink()->SetFrame(SE3(Vec3(0.0, 0.0, -0.05)));
}