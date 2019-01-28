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

// Robot
SDA20D* sdaRobot = new SDA20D;
SDA20DManager* rManager1;
SDA20DManager* rManager2;
Bin* bin = new Bin(0.01);

Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;


srLink* ee = new srLink;
srSystem* obs = new srSystem;
srCollision* colli = new srCollision;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void sdaRobotSetting();
void sdaRobotManagerSetting(int robotMode, int excludeNum = 0);
void sdarrtSetting();
void setObstacle();
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);
vector<Eigen::VectorXd> traj2(0);
Eigen::VectorXd qTemp;
Eigen::VectorXd q;

int useWaist = 0;
Eigen::VectorXd tempJointVal = Eigen::VectorXd::Zero(14 + useWaist);
Eigen::VectorXd tempJointVal2;
bool doPlanning = false;

SDA20DDualArmClosedLoopConstraint* armConstraint;
TBrrtManager* RRTManager = new TBrrtManager(armConstraint);

int main(int argc, char **argv)
{
	srand(time(NULL));
    sdaRobotSetting();
	setObstacle();

	initDynamics();
	if (useWaist == 0)
		sdaRobotManagerSetting(SDA20DManager::MoveBothArmOnly);
	else
		sdaRobotManagerSetting(SDA20DManager::MoveWholeBody);

	// set joint value for test
	tempJointVal[0 + useWaist] = 0.5*SR_PI_HALF;
	tempJointVal[7 + useWaist] = 0.5*SR_PI_HALF;
	tempJointVal[1 + useWaist] = 0.8*SR_PI_HALF;
	tempJointVal[8 + useWaist] = 0.8*SR_PI_HALF;
	tempJointVal[2 + useWaist] = -SR_PI_HALF;
	tempJointVal[9 + useWaist] = -SR_PI_HALF;
	tempJointVal[3 + useWaist] = -SR_PI_HALF;
	tempJointVal[10 + useWaist] = -SR_PI_HALF;
	tempJointVal[5 + useWaist] = 0.0;
	tempJointVal[12 + useWaist] = 0.0;
	rManager1->setJointVal(tempJointVal);

	// set constraint
	SE3 Tright = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_RIGHT_T].GetFrame();
	SE3 Tleft = ((SDA20D*)rManager1->m_robot)->gMarkerLink[SDA20D_Index::MLINK_LEFT_T].GetFrame();
	armConstraint = new SDA20DDualArmClosedLoopConstraint(rManager1, Tright % Tleft);
	printf("constraint vector:\n");
	cout << armConstraint->getConstraintVector(tempJointVal).transpose() << endl;
	//printf("constraint jacobian:\n");
	//cout << armConstraint->getConstraintJacobian(tempJointVal) << endl;


	// check projection
	tempJointVal2 = tempJointVal + 0.1*Eigen::VectorXd::Random(tempJointVal.size());
	tempJointVal2[0 + useWaist] += 0.5*SR_PI_HALF;
	tempJointVal2[7 + useWaist] += 0.5*SR_PI_HALF;
	printf("jointval before projection:\n");
	cout << tempJointVal2.transpose() << endl;
	printf("constraint vector before projection:\n");
	cout << armConstraint->getConstraintVector(tempJointVal2).transpose() << endl;
	armConstraint->project2ConstraintManifold(tempJointVal2);
	printf("constraint vector after projection:\n");
	cout << armConstraint->getConstraintVector(tempJointVal2).transpose() << endl;
	printf("jointval after projection:\n");
	cout << tempJointVal2.transpose() << endl;
	rManager1->setJointVal(tempJointVal2);

	sdarrtSetting();

	RRTManager->setConstraint(armConstraint);

	tangentSpace* TS = new tangentSpace(tempJointVal, armConstraint);
	Eigen::VectorXd q = Eigen::VectorXd::Random(14 + useWaist);
	TS->projectOntoTangentSpace(q);
	
	printf("tangent space basis\n");
	cout << TS->tangentBasis << endl;
	printf("projection matrix\n");
	cout << TS->ProjectionMatrix << endl;
	printf("check projection to tangent space\n");
	cout << (q - tempJointVal).transpose() << endl;
	cout << (armConstraint->getConstraintJacobian(tempJointVal) * (q - tempJointVal)).transpose() << endl;
	printf("joint value in tangent space\n");
	cout << q.transpose() << endl;
	printf("project to constraint manifold:\n");
	armConstraint->project2ConstraintManifold(q);
	cout << q.transpose() << endl;
	cout << armConstraint->getConstraintVector(q) << endl;

	///////////////////////////// solve RRT ////////////////////////////
	doPlanning = true;
	if (doPlanning)
	{
		for (int k = 0; k < 10; k++)
		{
			vector<unsigned int> colliIdx(0);
			// define planning problem
			RRTManager->setStartandGoal(tempJointVal, tempJointVal2);
			RRTManager->setThreshold(0.05);
			RRTManager->setSmoothingErrorThreshold(0.25);
			// run RRT
			RRTManager->execute(0.2);
			//traj = RRTManager->extractPath(0);
			//saveDataToText(traj, "../../../data/tbrrt_traj/tbrrt_traj_test.txt");
			//vector<Eigen::VectorXd> constraintVec(0);
			//for (unsigned int i = 0; i < traj.size(); i++)
			//	constraintVec.push_back(armConstraint->getConstraintVector(traj[i]));
			//saveDataToText(constraintVec, "../../../data/tbrrt_traj/tbrrt_traj_constraintVec.txt");
			traj2 = RRTManager->extractPath(200);
			saveDataToText(traj2, "../../../data/tbrrt_traj/tbrrt_traj_test2.txt");
			vector<Eigen::VectorXd> constraintVec2(0);
			for (unsigned int i = 0; i < traj2.size(); i++)
				constraintVec2.push_back(armConstraint->getConstraintVector(traj2[i]));
			saveDataToText(constraintVec2, "../../../data/tbrrt_traj/tbrrt_traj_constraintVec2.txt");


			// feasibility check for output trajectory
			bool pathFeasible = true;
			for (unsigned int i = 0; i < traj2.size(); i++)
			{
				rManager1->setJointVal(traj2[i]);
				pathFeasible = pathFeasible && (!rManager1->checkCollision());
				if (rManager1->checkCollision())
					colliIdx.push_back(i);
			}
			printf("output path feasible?\n");
			cout << pathFeasible << endl;
			printf("collision occuring path index:\n");
			for (unsigned int i = 0; i < colliIdx.size(); i++)
				cout << colliIdx[i] << ", ";
			cout << endl;
		}
	}
	
	///////////////////////////////////////////////////////////////////

	//rManager1->setJointVal(tempJointVal);
	//cout << rManager1->checkCollision() << endl;
	//rManager1->setJointVal(tempJointVal2);
	//cout << rManager1->checkCollision() << endl;
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

	//if (cnt % 10 == 0)
	//	trajcnt++;
	if (traj.size() > 0)
		rManager1->setJointVal(traj[trajcnt % traj.size()]);
	if (traj2.size() > 0)
		rManager1->setJointVal(traj2[trajcnt % traj2.size()]);
	if (cnt % 10 == 0)
		trajcnt++;
	if (!doPlanning)
	{
		if (cnt % 100 < 50)
			rManager1->setJointVal(tempJointVal);
		else
			rManager1->setJointVal(tempJointVal2);
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
		planningJoint.push_back( (srStateJoint*) rManager1->m_activeArmInfo->m_activeJoint[i]);
	RRTManager->setSystem(planningJoint);
	RRTManager->setStateBound(VecToVector(rManager1->m_lowerJointLimit), VecToVector(rManager1->m_upperJointLimit));
}

void setObstacle()
{
	ee->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	Vec3 obs_size = Vec3(0.05, 5.0, 0.05);
	ee->GetGeomInfo().SetDimension(obs_size);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
	colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	colli->GetGeomInfo().SetDimension(obs_size);
	ee->AddCollision(colli);
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	gSpace.AddSystem(obs);
	obs->GetBaseLink()->SetFrame(SE3(Vec3(0.9, 0.0, 0.2)));
}
