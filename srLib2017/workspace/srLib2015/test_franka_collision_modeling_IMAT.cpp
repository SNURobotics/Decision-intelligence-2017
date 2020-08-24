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
vector<srSystem*> markers(0);
vector<Eigen::VectorXd> checkPos(0);
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void frankaRobotSetting();
void frankaRobotManagerSetting();
void frankaRRTSetting();
void markerSetting(double x_min, double x_max, double y_max, double z);
int activeJointIdx = 0;
vector<Eigen::VectorXd> traj(0);

int main(int argc, char **argv)
{

	frankaRobotSetting();

	initDynamics();

	frankaRobotManagerSetting();

	Eigen::VectorXd pos = Eigen::VectorXd::Zero(7);
	pos[1] = -SR_PI / 3;
	pos[3] = -SR_PI / 3 * 2;
	pos[5] = SR_PI / 3;

	rManager1->setJointVal(pos);

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
