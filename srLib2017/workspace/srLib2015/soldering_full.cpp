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
vector<Eigen::VectorXd> ur5traj(0);
vector<Eigen::VectorXd> ur3traj(0);
vector<SE3> objTraj1(0);
vector<SE3> objTraj2(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
srSystem* obs = new srSystem;
SE3 Tobs2robot = SE3();

vector<Eigen::VectorXd> GripTraj(0);
vector<Eigen::VectorXd> makeGriptraj(double gripangle, Eigen::VectorXd currentPos);
vector<int> gripTiming1;
vector<int> gripTiming2;

void gripperFunction_UR3(bool gripperON);
void gripperFunction_UR5(bool gripperON);
Eigen::VectorXd robustInverseKinematics_UR3(SE3 finalpos, Eigen::VectorXd original, int maxiter);
Eigen::VectorXd robustInverseKinematics_UR5(SE3 finalpos, Eigen::VectorXd original, int maxiter);

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
	
	gSpace.AddSystem(soldering);
	gSpace.AddSystem(pcb);
	gSpace.AddSystem(pcbjig);

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

	soldering->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.5, 0.45, 0.12)));
	pcb->setBaseLinkFrame(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.0, 0.35, 0.12)));
	pcbjig->setBaseLinkFrame(EulerXYZ(Vec3(0, 0, 0), Vec3(poss[0][0], poss[0][1], 0.31)));

	Tobs2robot = EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.08, 0, 0));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	point0 = qval;
	int flag = 0;
	point1 = ur3Manager->inverseKin(pcb->GetBaseLink()->GetFrame() * Tobs2robot, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag);
	//point1 = robustInverseKinematics_UR3(pcb->GetBaseLink()->GetFrame() * Tobs2robot, point0, 30);

	ur3RRTManager->setStartandGoal(point0, point1);
	ur3RRTManager->execute(0.1);
	tempTraj = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur3RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(tempTraj[i]);
		ur5traj.push_back(ur5Manager->getJointVal());
	}
	gripTiming1.push_back(ur3traj.size());
	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for ur3 with object attached (point1 -> point2) ///////////////
	SE3 Tmid = EulerXYZ(Vec3(0, -SR_PI_HALF, 0), Vec3(-0.6, -0.6, 0.7));
	point2 = ur3Manager->inverseKin(Tmid, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag, ur3traj[ur3traj.size() - 1], 1500);
	//point2 = robustInverseKinematics_UR3(Tmid, ur3traj[ur3traj.size() - 1], 80);
	ur3RRTManager->attachObject(pcb, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], Inv(Tobs2robot));		// attaching object occurs here
	ur3RRTManager->setStartandGoal(point1, point2);
	ur3RRTManager->execute(0.1);
	tempTraj = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur3RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(tempTraj[i]);
		ur5traj.push_back(ur5Manager->getJointVal());
	}
	/////////////////////////////////////////////////////////////////////////////// UR3 end

	SE3 pcbpos = ur3->gMarkerLink[UR3_Index::MLINK_GRIP].GetFrame() * Inv(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.08, 0, 0)));
	pcb->setBaseLinkFrame(pcbpos);
	Tobs2robot = EulerXYZ(Vec3(0, -SR_PI_HALF, 0), Vec3(-0, -0, -0.05));
	SE3 Tpcb2robot = EulerXYZ(Vec3(-0, SR_PI, 0), Vec3(poss[1][0], poss[1][1], -0.13));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	point0 = Eigen::VectorXd::Zero(6);
	point1 = robustInverseKinematics_UR5(soldering->GetBaseLink()->GetFrame() * Tobs2robot, point0, 20);

	ur5RRTManager->setStartandGoal(point0, point1);
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(ur3Manager->getJointVal());
		ur5traj.push_back(tempTraj[i]);
	}
	gripTiming2.push_back(ur5traj.size());
	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for UR5 with object attached (point1 -> point2) ///////////////
	Tmid = pcbpos * Tpcb2robot;
	point2 = robustInverseKinematics_UR5(Tmid, point1, 20);

	ur5RRTManager->attachObject(soldering, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(Tobs2robot));		// attaching object occurs here
	ur5RRTManager->setStartandGoal(point1, point2);
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(ur3Manager->getJointVal());
		ur5traj.push_back(tempTraj[i]);
	}
	///////////////////////////////////////////////////////////////////////////

	////////////////// RRT planning for UR5 with object detached (point2 -> point3) ///////////////

	ur5RRTManager->setStartandGoal(point2, point1);
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(ur3Manager->getJointVal());
		ur5traj.push_back(tempTraj[i]);
	}
	gripTiming2.push_back(ur5traj.size());

	////////////////// RRT planning for UR5 with object detached (point2 -> point3) ///////////////

	ur5RRTManager->detachObject();
	ur5RRTManager->setStartandGoal(point1, Eigen::VectorXd::Zero(6));
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(ur3Manager->getJointVal());
		ur5traj.push_back(tempTraj[i]);
	}
	/////////////////////////////////////////////////////////////////////////////// UR5 end

	Tobs2robot = EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.08, 0, 0));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	point0 = ur3Manager->getJointVal();
	point1 = ur3Manager->inverseKin(EulerXYZ(Vec3(0, SR_PI_HALF, 0), Vec3(-0.0, 0.35, 0.12)) * Tobs2robot, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], true, SE3(), flag);

	ur3RRTManager->attachObject(pcb, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], Inv(Tobs2robot));		// attaching object occurs here
	ur3RRTManager->setStartandGoal(point0, point1);
	ur3RRTManager->execute(0.1);
	tempTraj = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur3RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(tempTraj[i]);
		ur5traj.push_back(ur5Manager->getJointVal());
	}

	//////////////////////////////////////////////////////////////

	/////////////////// RRT planning for ur3 with object attached (point1 -> point2) ///////////////
	Tmid = EulerXYZ(Vec3(0, -SR_PI_HALF, 0), Vec3(-0.6, -0.6, 0.7));
	point2 = Eigen::VectorXd::Zero(6);
	point2[1] = -SR_PI_HALF;
	point2[3] = -SR_PI_HALF;
	point2[4] = SR_PI_HALF;
	
	ur3RRTManager->detachObject();		// detaching object from robot occurs here
	ur3RRTManager->setStartandGoal(point1, point2);
	ur3RRTManager->execute(0.1);
	tempTraj = ur3RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur3RRTManager->setState(tempTraj[i]);
		objTraj1.push_back(pcb->GetBaseLink()->GetFrame());
		objTraj2.push_back(soldering->GetBaseLink()->GetFrame());
		ur3traj.push_back(tempTraj[i]);
		ur5traj.push_back(ur5Manager->getJointVal());
	}

	///////////////////////////////////////////////////////////////////////////




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

	if (cnt % 5 == 0)
		trajcnt++;

	// plot planned trajectory

	if (trajcnt == 1)
		gripperFunction_UR3(1);

	if (trajcnt < ur3traj.size())
	{
		ur3Manager->setJointVal(ur3traj[trajcnt % ur3traj.size()]);
		ur5Manager->setJointVal(ur5traj[trajcnt % ur5traj.size()]);
		pcb->GetBaseLink()->SetFrame(objTraj1[trajcnt % ur3traj.size()]);
		soldering->GetBaseLink()->SetFrame(objTraj2[trajcnt % ur5traj.size()]);
		if (trajcnt == gripTiming1[0] - 1) {
			gripperFunction_UR3(0);
		}
		if (trajcnt == gripTiming2[0] - 1) {
			gripperFunction_UR5(1);
		}
		if (trajcnt == gripTiming2[1] - 1) {
			gripperFunction_UR5(0);
		}

		if (trajcnt == ur3traj.size() - 1)
			trajcnt = 0;
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

void gripperFunction_UR3(bool gripperON)
{
	Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(6);
	double graspAngle = 0.2;
	double idleAngle = -0.825148;
	if (gripperON)
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
		return;
	}
	else {
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
		return;
	}
}

void gripperFunction_UR5(bool gripperON)
{
	Eigen::VectorXd tempPos = Eigen::VectorXd::Zero(3);
	double graspAngle = 0.6;
	double idleAngle = 0.0;
	if (gripperON)
	{
		tempPos(0) = -graspAngle;
		tempPos(1) = -graspAngle;
		tempPos(2) = graspAngle;
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
		return;
	}
	else {
		tempPos(0) = -idleAngle;
		tempPos(1) = -idleAngle;
		tempPos(2) = idleAngle;
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[0]))->m_State.m_rValue[0] = tempPos(0);
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[1]))->m_State.m_rValue[0] = tempPos(1);
		((srStateJoint*)(ur5Manager->m_gripperInfo->m_gripJoint[2]))->m_State.m_rValue[0] = tempPos(2);
		return;
	}
}

Eigen::VectorXd robustInverseKinematics_UR3(SE3 finalpos, Eigen::VectorXd original, int maxiter)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur3->getLowerJointLimit();
	Eigen::VectorXd upper = ur3->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur3Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < maxiter; i++) {
		Eigen::VectorXd tempsol;
		if (i == 0) initial = original;
		else {
			for (int j = 0; j < lower.size(); j++) {
				int temp = int((upper[j] - lower[j]) * 1000);
				initial[j] = double(rand() % temp) / 1000 + lower[j];
			}
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
		Eigen::VectorXd weight = ur3Manager->getBodyJacobian(currentpos, &ur3->gMarkerLink[UR3_Index::MLINK_GRIP], SE3()).colwise().squaredNorm();
		for (int i = 0; i < solList.size(); i++) {
			//newsize = (solList[i] - original).squaredNorm();
			newsize = (solList[i] - original).transpose().cwiseAbs() * weight;
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}

Eigen::VectorXd robustInverseKinematics_UR5(SE3 finalpos, Eigen::VectorXd original, int maxiter)
{
	srand((unsigned int)time(NULL));

	int flag = 0;
	Eigen::VectorXd lower = ur5->getLowerJointLimit();
	Eigen::VectorXd upper = ur5->getUpperJointLimit();
	Eigen::VectorXd initial = Eigen::VectorXd::Zero(lower.size());
	Eigen::VectorXd currentpos = ur5Manager->getJointVal();

	vector<Eigen::VectorXd> solList(0);

	for (int i = 0; i < maxiter; i++) {
		Eigen::VectorXd tempsol;
		if (i == 0) initial = original;
		else {
			for (int j = 0; j < lower.size(); j++) {
				int temp = int((upper[j] - lower[j]) * 1000);
				initial[j] = double(rand() % temp) / 1000 + lower[j];
			}
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
	ur5Manager->setJointVal(currentpos);
	if (solList.size() == 0) {
		cout << "Cannot find solution..." << endl;
		return Eigen::VectorXd::Zero(lower.size());
	}
	else {
		double temp = 10000;
		double newsize = 0;
		Eigen::VectorXd sol = Eigen::VectorXd::Zero(lower.size());
		Eigen::VectorXd weight = ur5Manager->getBodyJacobian(currentpos, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3()).colwise().squaredNorm();
		for (int i = 0; i < solList.size(); i++) {
			//newsize = (solList[i] - original).squaredNorm();
			newsize = (solList[i] - original).transpose().cwiseAbs() * weight;
			if (temp > newsize) {
				temp = newsize;
				sol = solList[i];
			}
		}
		return sol;
	}
}
