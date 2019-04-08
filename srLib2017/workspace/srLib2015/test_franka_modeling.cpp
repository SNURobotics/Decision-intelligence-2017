#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\Franka.h"
#include <time.h>
#include "robotManager\environmentBusbar.h"
#include "robotManager\environment_QBtech.h"
#include "robotManager\robotRRTManager.h"
#include "robotManager/robotManager.h"

// Robot
Franka* frankaRobot = new Franka;
robotManager* rManager1;
Eigen::VectorXd qval;
Eigen::VectorXd qgoal;
srSpace gSpace;
myRenderer* renderer;

robotRRTManager* RRTManager = new robotRRTManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void frankaRobotSetting();
void frankaRobotManagerSetting();
void frankaRRTSetting();
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);

int main(int argc, char **argv)
{

    frankaRobotSetting();

	initDynamics();
	
	frankaRobotManagerSetting();

	//qval.setZero(7);
	//qval[1] = -SR_PI_HALF;
	//qval[3] = -SR_PI_HALF;
	//qval[4] = SR_PI_HALF;
	//rManager1->setJointVal(qval);
	
	////////////// INVERSE KINEMATICS /////////////
	//SE3 Tgoal;
	//int flag = 0;
	//Eigen::VectorXd qval = rManager1->inverseKin(Tgoal, &frankaRobot->gMarkerLink[Franka_Index::MLINK_GRIP], true, SE3(), flag);
	//rManager1->setJointVal(qval);
	///////////////////////////////////////////////

	///////////////	FOR TESTING RRT ////////////////
	//frankaRRTSetting();
	//qgoal = Eigen::VectorXd::Zero(7);
	//RRTManager->setStartandGoal(qval, goal);
	//cout << RRTManager->checkFeasibility(qval) << RRTManager->checkFeasibility(goal);
	//
	//RRTManager->execute(0.1);
	//traj = RRTManager->extractPath(200);
	//
	////////////////////////////////////////////////

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
	int jointIdx = 3;
	((srStateJoint*)frankaRobot->m_KIN_Joints[jointIdx])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;

	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);
}


void frankaRobotSetting()
{
	gSpace.AddSystem((srSystem*)frankaRobot);
	frankaRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	frankaRobot->SetActType(srJoint::ACTTYPE::HYBRID);
	Trobotbase1 = frankaRobot->GetBaseLink()->GetFrame() * frankaRobot->TsrLinkbase2robotbase;
}

void frankaRobotManagerSetting()
{
	rManager1 = new robotManager;
	rManager1->setRobot((srSystem*)frankaRobot);
	rManager1->setSpace(&gSpace);
	rManager1->setEndeffector(&frankaRobot->gMarkerLink[Franka_Index::MLINK_GRIP]);
}

void frankaRRTSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(7);
	for (int i = 0; i < 7; i++)
		planningJoint[i] = (srStateJoint*)frankaRobot->gJoint[i];
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(frankaRobot->getLowerJointLimit(), frankaRobot->getUpperJointLimit());
}