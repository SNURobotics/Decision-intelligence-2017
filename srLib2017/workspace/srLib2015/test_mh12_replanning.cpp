#include <cstdio>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager\MH12Robot.h"
#include "robotManager\environment_4th.h"
#include "robotManager/robotRRTManager.h"
#include <time.h>
#include <ctime>
#include "common/dataIO.h"

#define PROBLEM_DEFINITION
//#define REPLANNING

// Robot
MH12Robot* MHRobot = new MH12Robot;
MH12RobotManager* rManager1;
robotRRTManager* robotrrtManager;

Eigen::VectorXd qval;

srSpace gSpace;
myRenderer* renderer;

srLink* ee = new srLink;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void MHRobotSetting();
void MHRobotManagerSetting();
void MHrrtSetting();
void obstacleSetting(double dim = 0.1, int N = 1);
bool pathFeasibilityCheck(int curTimeStep, vector<SE3> recentObsTraj, int& collisionTimeStep, SE3& colSE3);
vector<Eigen::VectorXd> getReplanningTraj(Eigen::VectorXd replanInit);
void updateCurTraj(int initIdx, vector<Eigen::VectorXd> newTraj);
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);
vector<SE3> eeTraj(0);

// objects
Bin* bin = new Bin(0.01);
workingObject* objects = new workingObject;
Table4th* table = new Table4th(0.01);

// variables for replanning scenario
vector<SE3>		obsTraj(0);
vector<Eigen::VectorXd> jointTraj_initPlan(0);
vector<Eigen::VectorXd> jointTraj_curPlan(0);
vector<Eigen::VectorXd> jointTraj_newPlan(0);
double rrtStepSize = 0.1;
double robotOperationSpeed = 0.25;
double maxPlanningTime = 2.0;
int plannigHorizon = ceil(maxPlanningTime * robotOperationSpeed / rrtStepSize);
int predictionHorizon = 2 * plannigHorizon;
Eigen::VectorXd qInit;
Eigen::VectorXd qGoal;
int main(int argc, char **argv)
{

    MHRobotSetting();
	obstacleSetting(0.2, 0);

	gSpace.AddSystem(obs);
	gSpace.AddSystem(bin);
	gSpace.AddSystem(table);
	gSpace.AddSystem(objects);
	initDynamics();

	
	MHRobotManagerSetting();
	MHrrtSetting();

	// set objects
	SE3 Trobotbase2link1 = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, -0.450));		// change to exact value later

	Vec3 Plink12bin = Vec3(0.89, 0.14, 0.45);
	bin->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Plink12bin));	// change to exact value later

	Vec3 Plink12table = Vec3(0.89, 0.0, 0.41);
	table->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(0.0, 0.0, 0.0), Plink12table));	// change to exact value later

	SE3 Tinit = SE3(Vec3(0.0, 0.0, 0.1))*SE3(bin->getBaseLinkFrame());
	objects->setBaseLinkFrame(Tinit);
	double temp_r = 0.04 - 0.017*0.5 / sqrt(3.0);
	SE3 temp_robot2objhead = EulerZYX(Vec3(-SR_PI / 6.0, 0.0, SR_PI), Vec3(temp_r * cos(SR_PI / 6.0), -temp_r * sin(SR_PI / 6.0), 0.0));
	SE3 Tgoal = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.226048, 0.871385, 0.466791)) * temp_robot2objhead; // head case
	objects->setBaseLinkFrame(Tinit);
	SE3 Tobject2robot = EulerZYX(Vec3(0.33*SR_PI, 0.0, SR_PI), Vec3(-0.02, 0.0, 0.01)); 
	SE3 Trobot2object = Inv(Tobject2robot);
	int flag, flag2;
	Eigen::VectorXd qInvKinInit = Eigen::VectorXd::Zero(6);
	qInvKinInit[4] = -SR_PI_HALF;
	qInit = rManager1->inverseKin(Tinit * Tobject2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInvKinInit);
	qGoal = rManager1->inverseKin(Tgoal * Tobject2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag2, qInvKinInit);
	cout << qInit.transpose() << endl;
	cout << qGoal.transpose() << endl;

	robotrrtManager->attachObject(objects, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], Trobot2object);


#ifdef PROBLEM_DEFINITION
	/////////////////////////////////////////////////////////////////////
	///////////////////// generate initial traj /////////////////////////
	/////////////////////////////////////////////////////////////////////

	
	clock_t start1 = clock();
	robotrrtManager->setStartandGoal(qInit, qGoal);

	robotrrtManager->execute(0.1);
	if (robotrrtManager->isExecuted())
	{
		jointTraj_initPlan = robotrrtManager->extractPath();
		printf("planning time: %f\n", (clock() - start1) / (double)CLOCKS_PER_SEC);
		traj = jointTraj_initPlan;
		eeTraj.resize(traj.size());
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			eeTraj[i] = rManager1->forwardKin(traj[i], &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], SE3());
		}
	}
	Vec3 meanPos = eeTraj[traj.size() / 2].GetPosition();
	cout << meanPos << endl;

	

	/////////////////////////////////////////////////////////////////////
	/////////////////////// set obstacle traj ///////////////////////////
	/////////////////////////////////////////////////////////////////////

	Vec3 movingDir(-0.5, 0.0, 0.0);
	//Vec3 movingDir(0.2, 0.0, 0.0);
	
	obsTraj.resize(traj.size());
	vector<Eigen::VectorXd> obsTraj_vec(traj.size());
	int N = traj.size();
	for (int i = 0; i < obsTraj.size(); i++)
	{
		obsTraj[i] = SE3(meanPos + ((double)(i - N / 2) / N)*movingDir);
		//obsTraj[i] = SE3(meanPos +  sin((double)i/N * SR_TWO_PI)*movingDir);
		obsTraj_vec[i] = SE3toVector(obsTraj[i]);
	}


	// save trajectory
	saveDataToText(jointTraj_initPlan, "../../../workspace/robot/replanning/jointInitTraj.txt");
	saveDataToText(obsTraj_vec, "../../../workspace/robot/replanning/obsTraj.txt");
#endif


	/////////////////////////////////////////////////////////////////////
	////////////////////// replanning algorithm /////////////////////////
	/////////////////////////////////////////////////////////////////////
#ifdef REPLANNING
	// load trajectory
	jointTraj_initPlan = loadDataFromText("../../../workspace/robot/replanning/jointInitTraj.txt", 6);
	vector<Eigen::VectorXd> obsTraj_vec = loadDataFromText("../../../workspace/robot/replanning/obsTraj.txt", 6);
	obsTraj.resize(obsTraj_vec.size());
	for (unsigned int i = 0; i < obsTraj_vec.size(); i++)
		obsTraj[i] = VectortoSE3(obsTraj_vec[i]);

	// replanning
	int iter = 0;
	jointTraj_curPlan = jointTraj_initPlan;
	while (iter + plannigHorizon != jointTraj_curPlan.size())
	{
		vector<SE3> obsTrajTemp(0);
		if (iter > predictionHorizon - 1)
		{
			for (int i = 0; i < predictionHorizon; i++)
				obsTrajTemp.push_back(obsTraj[i + 1 + iter - predictionHorizon]);
		}
		else
		{
			for (int i = 0; i < iter + 1; i++)
				obsTrajTemp.push_back(obsTraj[i]);
		}
		//for (int i = 0; i < predictionHorizon; i++)
		//{
		//	if (iter + i < obsTraj.size())
		//		obsTrajTemp[i] = obsTraj[iter + i];
		//	else
		//		obsTrajTemp[i] = obsTraj.back();
		//}
		int collisionTimeStep;
		SE3 colSE3;
		bool doReplanning = !pathFeasibilityCheck(iter, obsTrajTemp, collisionTimeStep, colSE3);
		if (collisionTimeStep - iter < plannigHorizon)
		{
			printf("replanning impossible!!! plan task again!!!\n");
			printf("stop iter: %d\n", iter);
			break;
		}			
		if (doReplanning)
		{
			obs->GetBaseLink()->SetFrame(colSE3);
			std::clock_t start = clock();
			vector<Eigen::VectorXd> tempTraj = getReplanningTraj(jointTraj_curPlan[iter + plannigHorizon]);
			printf("replanning time: %f\n", (clock() - start) / (double)CLOCKS_PER_SEC);
			if (tempTraj.size() > 0)
				updateCurTraj(iter + plannigHorizon, tempTraj);
			else
			{
				printf("replanning impossible!!! plan task again!!!\n");
				printf("stop iter: %d\n", iter);
				break;
			}

		}
		iter++;
	}
	traj = jointTraj_curPlan;

	if (jointTraj_curPlan.size() > obsTraj.size())
	{
		vector<SE3> obsTempTraj = obsTraj;
		obsTraj.resize(jointTraj_curPlan.size());
		for (unsigned int i = 0; i < obsTraj.size(); i++)
		{
			if (i < obsTempTraj.size())
				obsTraj[i] = obsTempTraj[i];
			else
				obsTraj[i] = obsTempTraj.back();
		}
	}

#endif
	
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
	static bool printed = false;
	if (cnt % 10 == 0)
	{
		trajcnt++;
		printed = false;
	}
		
	if (traj.size() > 0)
	{
		//obs->GetBaseLink()->SetFrame(obsTraj[trajcnt % traj.size()]);
		obs->GetBaseLink()->SetFrame(obsTraj[0]);
		robotrrtManager->setState(traj[trajcnt % traj.size()]);
		gSpace._KIN_UpdateFrame_All_The_Entity_All_The_Systems();
		if (!printed)
			printf("traj iter: %d, collision?: %d\n", trajcnt % traj.size(), (int)rManager1->checkCollision());
		printed = true;
	}
		
		//rManager1->setJointVal(traj[trajcnt % traj.size()]);

	int stop = 1;
}


void MHRobotSetting()
{
	gSpace.AddSystem((srSystem*)MHRobot);
	MHRobot->GetBaseLink()->SetFrame(EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, -0.450)));
	MHRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	MHRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	Trobotbase1 = MHRobot->GetBaseLink()->GetFrame() * MHRobot->TsrLinkbase2robotbase;
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

void MHRobotManagerSetting()
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

	rManager1 = new MH12RobotManager(MHRobot, &gSpace);

}

void MHrrtSetting()
{
	robotrrtManager = new robotRRTManager;
	vector<srStateJoint*> planningJoints(DEGREE_OF_FREEDOM_MH12_JOINT);
	robotrrtManager->setSystem(rManager1->m_robot);
	robotrrtManager->setSpace(rManager1->m_space);
	robotrrtManager->setStateBound(((MH12Robot*)rManager1->m_robot)->getLowerJointLimit(), ((MH12Robot*)rManager1->m_robot)->getUpperJointLimit());
}

void obstacleSetting(double dim, int N)
{
	ee->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	ee->GetGeomInfo().SetDimension(dim);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);

	srCollision* tempColli = new srCollision;
	tempColli->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	tempColli->GetGeomInfo().SetDimension(dim);
	ee->AddCollision(tempColli);

	if (N > 1)
	{
		vector<srLink*> links(N - 1);
		vector<srPrismaticJoint*> joints(N - 1);
		vector<srCollision*> collis(N - 1);
		for (int i = 0; i < N - 1; i++)
		{
			links[i] = new srLink;
			links[i]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
			links[i]->GetGeomInfo().SetDimension(dim);
			links[i]->GetGeomInfo().SetColor(1.0, 0.0, 0.0);
			collis[i] = new srCollision;
			collis[i]->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
			collis[i]->GetGeomInfo().SetDimension(dim);
			links[i]->AddCollision(collis[i]);
			joints[i] = new srPrismaticJoint;
			if (i == 0)
			{
				joints[i]->SetParentLink(ee);
			}
			else
				joints[i]->SetParentLink(links[i - 1]);
			joints[i]->SetChildLink(links[i]);
		}
	}

	
	obs->SetBaseLink(ee);
	obs->SetBaseLinkType(srSystem::FIXED);
	obs->GetBaseLink()->SetFrame(Vec3(0.0, 0.0, -10.0));
}

bool pathFeasibilityCheck(int curTimeStep, vector<SE3> recentObsTraj, int& collisionTimeStep, SE3& colSE3)
{
	// predict obstacle position
	se3 meanVel = Log(Inv(recentObsTraj[0]) * recentObsTraj.back()) * (double) (1.0 / recentObsTraj.size());
	SE3 predictSE3 = recentObsTraj.back() * Exp(meanVel * (double)predictionHorizon);
	SE3 predictSE3_plan = recentObsTraj.back() * Exp(meanVel * (double)plannigHorizon);
	// check collision for current object loc
	int collisionTimeStep1 = jointTraj_curPlan.size() + 1;
	for (unsigned int i = curTimeStep; i < jointTraj_curPlan.size(); i++)
	{
		robotrrtManager->setState(jointTraj_curPlan[i]);
		obs->GetBaseLink()->SetFrame(recentObsTraj.back());
		if (gSpace.m_srDYN.RUNTIME_MARKKH())
		{
			collisionTimeStep1 = i;
			//return false;
		}
	}

	// check collision for future object loc (planning horizon)
	int collisionTimeStep2 = jointTraj_curPlan.size() + 1;
	for (unsigned int i = curTimeStep; i < jointTraj_curPlan.size(); i++)
	{
		robotrrtManager->setState(jointTraj_curPlan[i]);
		obs->GetBaseLink()->SetFrame(predictSE3_plan);
		if (gSpace.m_srDYN.RUNTIME_MARKKH())
		{
			collisionTimeStep2 = i;
		}
	}
	// check collision for future object loc (predict)
	int collisionTimeStep3 = jointTraj_curPlan.size() + 1;
	for (unsigned int i = curTimeStep; i < jointTraj_curPlan.size(); i++)
	{
		robotrrtManager->setState(jointTraj_curPlan[i]);
		obs->GetBaseLink()->SetFrame(predictSE3);
		if (gSpace.m_srDYN.RUNTIME_MARKKH())
		{
			collisionTimeStep3 = i;
		}
	}
	collisionTimeStep = min(collisionTimeStep1, collisionTimeStep2);
	collisionTimeStep = min(collisionTimeStep, collisionTimeStep3);
	if (collisionTimeStep == collisionTimeStep3)
		colSE3 = predictSE3;
	else if (collisionTimeStep == collisionTimeStep2)
		colSE3 = predictSE3_plan;
	else
		colSE3 = recentObsTraj.back();
	if (collisionTimeStep < jointTraj_curPlan.size() + 1)
		return false;
	return true;
}

vector<Eigen::VectorXd> getReplanningTraj(Eigen::VectorXd replanInit)
{
	robotrrtManager->setStartandGoal(replanInit, qGoal);
	robotrrtManager->execute(0.1);
	if (robotrrtManager->isExecuted())
	{
		return robotrrtManager->extractPath();
	}
	else
		return vector<Eigen::VectorXd>();
}

void updateCurTraj(int initIdx, vector<Eigen::VectorXd> newTraj)
{
	vector<Eigen::VectorXd> tempTraj = jointTraj_curPlan;
	jointTraj_curPlan.resize(initIdx + newTraj.size());
	for (unsigned int i = 0; i < initIdx; i++)
		jointTraj_curPlan[i] = tempTraj[i];
	for (unsigned int i = 0; i < newTraj.size(); i++)
		jointTraj_curPlan[initIdx + i] = newTraj[i];
}
