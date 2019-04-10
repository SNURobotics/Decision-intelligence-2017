#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR3RobotManager.h"
#include "robotManager\UR3Robot.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include <time.h>
#include "robotManager\robotRRTManager.h"
#include "robotManager\environment_5th.h"


srSpace gSpace;
myRenderer* renderer;
// Robot
UR3Robot* ur3 = new UR3Robot;
UR3RobotManager* ur3Manager;
UR5Robot* ur5 = new UR5Robot;
UR5RobotManager* ur5Manager;
robotRRTManager* ur3RRTManager = new robotRRTManager;
robotRRTManager* ur5RRTManager = new robotRRTManager;

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

BoxForTape* boxfortape = new BoxForTape(0.01);

Eigen::VectorXd qval;

Eigen::VectorXd point0;
Eigen::VectorXd point1;
Eigen::VectorXd point2;
Eigen::VectorXd point3;
vector<Eigen::VectorXd> ur5traj(0);
vector<SE3> objTraj(0);
vector<SE3> boxfortapeTraj(0);
vector<Eigen::VectorXd> tempTraj(0);
srLink* ee = new srLink;
srSystem* obs = new srSystem;
SE3 Tobs2robot = SE3();

int main(int argc, char **argv)
{
	srand(NULL);
	// robot, object, environment settings should come before initDynamics()
    URrobotSetting();
	tempObjectSetting();

	gSpace.AddSystem(boxfortape);

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
	obs->GetBaseLink()->SetFrame(SE3(Vec3(-0.5, 0.5, 0.5)));
	boxfortape->setBaseLinkFrame(SE3(Vec3(1.0, 0.0, 0.0)));

	/////////////// RRT planning to reach object (point0 -> point1) ///////////////
	clock_t start = clock();
	point0 = Eigen::VectorXd::Zero(6);
	int flag = 0;
	point1 = ur5Manager->inverseKin(obs->GetBaseLink()->GetFrame() * Tobs2robot, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag);
	cout << "inverse kinematics flag: " <<  flag << endl;
	ur5RRTManager->setStartandGoal(point0, point1);
	ur5RRTManager->execute(0.1);
	ur5traj = ur5RRTManager->extractPath(20);
	// set object trajectory
	for (unsigned int i = 0; i < ur5traj.size(); i++)
	{
		ur5RRTManager->setState(ur5traj[i]);
		objTraj.push_back(obs->GetBaseLink()->GetFrame());
		boxfortapeTraj.push_back(boxfortape->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
	////////////////////////////////////////////////////////////

	///////////////// RRT planning for UR5 with object attached (point1 -> point2) ///////////////
	start = clock();
	point2 = Eigen::VectorXd::Ones(6);
	ur5RRTManager->attachObject(static_cast<srSystem*>(boxfortape), &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3());
	ur5RRTManager->attachObject(obs, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], SE3());		// attaching object occurs here
	ur5RRTManager->setStartandGoal(point1, point2);
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	ur5traj.insert(ur5traj.end(), tempTraj.begin(), tempTraj.end());
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj.push_back(obs->GetBaseLink()->GetFrame());
		boxfortapeTraj.push_back(boxfortape->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
	///////////////////////////////////////////////////////////////////////////

	//////////////// RRT planning for UR5 with object detached (point2 -> point3) ///////////////
	start = clock();
	SE3 Tgoal = SE3(Vec3(-0.5, 0.5, 0.5));
	point3 = ur5Manager->inverseKin(Tgoal, &ur5->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), flag);
	cout << "inverse kinematics flag: " << flag << endl;
	ur5RRTManager->detachObject();		// detaching object from robot occurs here
	ur5RRTManager->setStartandGoal(point2, point3);
	ur5RRTManager->execute(0.1);
	tempTraj = ur5RRTManager->extractPath(20);
	ur5traj.insert(ur5traj.end(), tempTraj.begin(), tempTraj.end());
	// set object trajectory
	for (unsigned int i = 0; i < tempTraj.size(); i++)
	{
		ur5RRTManager->setState(tempTraj[i]);
		objTraj.push_back(obs->GetBaseLink()->GetFrame());
		boxfortapeTraj.push_back(boxfortape->GetBaseLink()->GetFrame());
	}
	cout << "time for planning: " << (clock() - start) / (double)CLOCKS_PER_SEC << endl;
	/////////////////////////////////////////////////////////////////////////////

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

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	
	// plot planned trajectory
	ur5Manager->setJointVal(ur5traj[trajcnt % ur5traj.size()]);
	obs->GetBaseLink()->SetFrame(objTraj[trajcnt % ur5traj.size()]);
	
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
