#include "myRenderer.h"
#include "robotManager/rrtVectorFields.h"
#include "myRenderer.h"
#include "robotManager\UR3Robot.h"
#include "robotManager\UR3RobotManager.h"
#include <time.h>
#include "RRTmanager\vfrrtManager.h"

srSpace gSpace;
myRenderer* renderer;

vector<Eigen::VectorXd> traj1(0);
vector<Eigen::VectorXd> traj2(0);
// Robot
UR3Robot* URRobot = new UR3Robot;
UR3RobotManager* rManager1;
SE3 Trobotbase1;
// Robot trajectory
vector<Eigen::VectorXd> traj(0);
// RRT setting
vfrrtManager* RRTManager = new vfrrtManager;
objectClearanceVectorField* objClearVF;
bool addVF = true;
// Obstacle
srSystem* obs = new srSystem;
//srSystem* obs1 = new srSystem;
void initDynamics();
// rendering functions
void rendering(int argc, char **argv);
void updateFunc();
void URrobotSetting();
void URrobotManagerSetting();
void URrrtSetting();
void obsSetting(srSystem* obs);

int main(int argc, char **argv)
{
	
	srand(time(NULL));
	URrobotSetting();
	obsSetting(obs);
	//obsSetting(obs1);
	initDynamics();
	URrobotManagerSetting();


	URrrtSetting();
	Eigen::VectorXd test = Eigen::VectorXd::Zero(6);
	test[0] = 2.0;
	cout << test.squaredNorm() << endl;

	Eigen::VectorXd start = Eigen::VectorXd::Zero(6);
	start(2) = -SR_PI_HALF;
	Eigen::VectorXd goal = Eigen::VectorXd::Zero(6);
	goal(0) = SR_PI_HALF;
	goal(2) = -SR_PI_HALF;
	RRTManager->setStartandGoal(start, goal);
	

	SE3 Tstart = rManager1->forwardKin(start, &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP]);
	SE3 Tgoal = rManager1->forwardKin(goal, &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP]);

	SE3 Tobs = SE3(0.5*(Tstart.GetPosition() + Tgoal.GetPosition()));
	obs->GetBaseLink()->SetFrame(Tobs);
	cout << Tobs << endl;
	if (addVF)
	{
		Vec3 objectLoc = Tobs.GetPosition();
		objClearVF->setObjectLocation(objectLoc);
	}
	
	rManager1->setJointVal(goal);

	cout << RRTManager->setState(start) << RRTManager->setState(goal);
	RRTManager->execute(0.1);
	traj = RRTManager->extractPath();

	//cout << pinv( rManager1->getAnalyticJacobian(start, objClearVF->_links[0], false, objClearVF->_offsets[0])) << endl;
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
	//static Eigen::VectorXd test = Eigen::VectorXd::Zero(6);
	//test[4] += 0.001;
	//rManager1->setJointVal(test);
	//obs1->GetBaseLink()->SetFrame(URRobot->gLink[UR3_Index::LINK_4].GetFrame() *SE3(Vec3(0.0, 0.0, 0.60855)));

	//cout << obs1->GetBaseLink()->GetFrame().GetPosition() << endl;
	static int cnt = 0;
	static int trajcnt = 0;
	cnt++;

	if (cnt % 10 == 0)
		trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);
}

void URrobotSetting()
{
	gSpace.AddSystem((srSystem*)URRobot);
	URRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0)));
	URRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	URRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = URRobot->GetBaseLink()->GetFrame() * URRobot->TsrLinkbase2robotbase;
}

void URrobotManagerSetting()
{
	rManager1 = new UR3RobotManager(URRobot, &gSpace);

}


void URrrtSetting()
{
	RRTManager->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(6);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)URRobot->gJoint[i];
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(URRobot->getLowerJointLimit(), URRobot->getUpperJointLimit());

	if (addVF)
	{
		// vector field setting
		objClearVF = new objectClearanceVectorField(rManager1, &URRobot->gMarkerLink[UR3_Index::MLINK_GRIP]);
		vector<SE3> offsets(1, SE3(Vec3(0.0, 0.0, 0.60855)));
		vector<srLink*> links(1, &URRobot->gLink[UR3_Index::LINK_4]);
		vector<double> weights(1, 0.5);
		objClearVF->setOffsets(offsets);
		objClearVF->setLinks(links);
		objClearVF->setWeights(weights);
		RRTManager->addVectorField(objClearVF);
		RRTManager->setLambda(0.001);
		RRTManager->setAlgorithmMode(vfrrtManager::MODE::GENUINE);
	}	
}

void obsSetting(srSystem* obs)
{
	srLink* obs_base = new srLink;
	srLink* obs_child = new srLink;
	srWeldJoint* wJoint = new srWeldJoint;
	srCollision* colli = new srCollision;
	obs_base->GetGeomInfo().SetDimension(0.0);
	obs_child->GetGeomInfo().SetDimension(0.05);
	obs_child->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	obs_child->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	colli->GetGeomInfo().SetDimension(0.05);
	colli->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	obs_child->AddCollision(colli);
	wJoint->SetParentLink(obs_base);
	wJoint->SetChildLink(obs_child);
	obs->SetBaseLink(obs_base);
	//obs->SetBaseLinkType(srSystem::KINEMATIC);
	obs->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs);
}
