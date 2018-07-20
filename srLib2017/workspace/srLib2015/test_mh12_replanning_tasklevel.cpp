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
//#define ADD_TASK

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
void obstacleSetting(Vec3 dim);
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
	obstacleSetting(Vec3(0.2, 0.2, 0.05));

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
	SE3 Tobject2robot = EulerZYX(Vec3(0.33*SR_PI, 0.0, SR_PI), Vec3(-0.02, 0.0, 0.01)); 
	SE3 Trobot2object = Inv(Tobject2robot);
	int flag, flag2;
	Eigen::VectorXd qInvKinInit = Eigen::VectorXd::Zero(6);
	qInvKinInit[4] = -SR_PI_HALF;
	
	qInit = qInvKinInit;
	qGoal = rManager1->inverseKin(Tinit * Tobject2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInvKinInit);
	
	cout << qInit.transpose() << endl;
	cout << qGoal.transpose() << endl;

	
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
	

	

	/////////////////////////////////////////////////////////////////////
	/////////////////////// set obstacle traj ///////////////////////////
	/////////////////////////////////////////////////////////////////////
	Vec3 meanPos = Vec3(0.2, 0.0, 0.15) + bin->getBaseLinkFrame().GetPosition();
	Vec3 movingDir(-0.2, 0.0, 0.0);
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
	saveDataToText(jointTraj_initPlan, "../../../workspace/robot/replanning/jointInitTraj_tasklevel.txt");
	saveDataToText(obsTraj_vec, "../../../workspace/robot/replanning/obsTraj_tasklevel.txt");
#endif


	/////////////////////////////////////////////////////////////////////
	////////////////////// replanning algorithm /////////////////////////
	/////////////////////////////////////////////////////////////////////
#ifdef REPLANNING
	// load trajectory
	jointTraj_initPlan = loadDataFromText("../../../workspace/robot/replanning/jointInitTraj_tasklevel.txt", 6);
	vector<Eigen::VectorXd> obsTraj_vec = loadDataFromText("../../../workspace/robot/replanning/obsTraj_tasklevel.txt", 6);
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
	
#ifdef ADD_TASK
	// load trajectory
	jointTraj_initPlan = loadDataFromText("../../../workspace/robot/replanning/jointInitTraj_tasklevel.txt", 6);
	vector<Eigen::VectorXd> obsTraj_vec = loadDataFromText("../../../workspace/robot/replanning/obsTraj_tasklevel.txt", 6);
	obsTraj.resize(obsTraj_vec.size());
	for (unsigned int i = 0; i < obsTraj_vec.size(); i++)
		obsTraj[i] = VectortoSE3(obsTraj_vec[i]);

	SE3 Tobs2robot = EulerZYX(Vec3(0.0, 0.0, SR_PI), Vec3(0.0, 0.0, 0.035));
	SE3 Trobot2obs = Inv(Tobs2robot);
	SE3 TobsInit = obsTraj.back();
	SE3 TobsGoal = SE3(Vec3(0.0, 0.25, 0.025) + bin->getBaseLinkFrame().GetPosition());

	//rManager1->setJointVal(qInit);

	//obs->GetBaseLink()->SetFrame(MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() * Trobot2obs);


	//////////////////////////////////////////////
	////////////// task 1 go to obs //////////////
	//////////////////////////////////////////////
	vector<Eigen::VectorXd> jointTraj1;
	vector<SE3>	obsTraj1;
	qGoal = rManager1->inverseKin(TobsInit * Tobs2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInvKinInit);

	clock_t start1 = clock();
	robotrrtManager->setStartandGoal(qInit, qGoal);

	robotrrtManager->execute(0.1);
	if (robotrrtManager->isExecuted())
	{
		jointTraj1 = robotrrtManager->extractPath();
		printf("planning time: %f\n", (clock() - start1) / (double)CLOCKS_PER_SEC);
		obsTraj1.resize(jointTraj1.size());
		for (unsigned int i = 0; i < jointTraj1.size(); i++)
		{
			obsTraj1[i] = TobsInit;
		}
	}

	//////////////////////////////////////////////
	////////////// task 2 move obs //////////////
	//////////////////////////////////////////////
	vector<Eigen::VectorXd> jointTraj2;
	vector<SE3>	obsTraj2;
	qInit = qGoal;
	qGoal = rManager1->inverseKin(TobsGoal * Tobs2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInvKinInit);

	clock_t start2 = clock();
	robotrrtManager->setStartandGoal(qInit, qGoal);
	robotrrtManager->attachObject(obs, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], Trobot2obs);
	robotrrtManager->execute(0.1);
	if (robotrrtManager->isExecuted())
	{
		jointTraj2 = robotrrtManager->extractPath();
		printf("planning time: %f\n", (clock() - start2) / (double)CLOCKS_PER_SEC);
		obsTraj2.resize(jointTraj2.size());
		for (unsigned int i = 0; i < jointTraj2.size(); i++)
		{
			obsTraj2[i] = rManager1->forwardKin(jointTraj2[i], &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], Trobot2obs);
		}
	}



	//////////////////////////////////////////////
	////////////// task 3 go to obj //////////////
	//////////////////////////////////////////////
	vector<Eigen::VectorXd> jointTraj3;
	vector<SE3>	obsTraj3;
	qInit = qGoal;
	qGoal = rManager1->inverseKin(Tinit * Tobject2robot, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInvKinInit);
	robotrrtManager->detachObject();
	obs->GetBaseLink()->SetFrame(TobsGoal);
	clock_t start3 = clock();
	robotrrtManager->setStartandGoal(qInit, qGoal);
	robotrrtManager->execute(0.1);
	if (robotrrtManager->isExecuted())
	{
		jointTraj3 = robotrrtManager->extractPath();
		printf("planning time: %f\n", (clock() - start3) / (double)CLOCKS_PER_SEC);
		obsTraj3.resize(jointTraj3.size());
		for (unsigned int i = 0; i < jointTraj3.size(); i++)
		{
			obsTraj3[i] = TobsGoal;
		}
	}

	// gather traj
	traj.resize(0);
	obsTraj.resize(0);
	for (unsigned int i = 0; i < jointTraj1.size(); i++)
	{
		traj.push_back(jointTraj1[i]);
		obsTraj.push_back(obsTraj1[i]);
	}
	for (unsigned int i = 0; i < jointTraj2.size(); i++)
	{
		traj.push_back(jointTraj2[i]);
		obsTraj.push_back(obsTraj2[i]);
	}
	for (unsigned int i = 0; i < jointTraj3.size(); i++)
	{
		traj.push_back(jointTraj3[i]);
		obsTraj.push_back(obsTraj3[i]);
	}
	//obs->GetBaseLink()->SetFrame(TobsGoal);
	//rManager1->setJointVal(qInit);
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
		obs->GetBaseLink()->SetFrame(obsTraj[trajcnt % traj.size()]);
		//obs->GetBaseLink()->SetFrame(obsTraj[0]);
		//robotrrtManager->setState(traj[trajcnt % traj.size()]);
		rManager1->setJointVal(traj[trajcnt % traj.size()]);
		if (!printed)
			printf("traj iter: %d, collision?: %d\n", trajcnt % traj.size(), (int)rManager1->checkCollision());
		printed = true;
	}
	else
		cout << rManager1->checkCollision() << endl;
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

void obstacleSetting(Vec3 dim)
{
	srLink* temp = new srLink;
	srWeldJoint* tempWeld = new srWeldJoint;
	tempWeld->SetChildLink(ee);
	tempWeld->SetParentLink(temp);
	temp->GetGeomInfo().SetDimension(Vec3(0.001, 0.001, 0.001));
	ee->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	ee->GetGeomInfo().SetDimension(dim);
	//ee->GetGeomInfo().SetDimension(0.2);
	ee->GetGeomInfo().SetColor(1.0, 0.0, 0.0);

	srCollision* tempColli = new srCollision;
	tempColli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	tempColli->GetGeomInfo().SetDimension(dim);
	//tempColli->GetGeomInfo().SetDimension(0.2);
	ee->AddCollision(tempColli);
	obs->SetBaseLink(temp);
	obs->SetBaseLinkType(srSystem::FIXED);
	obs->GetBaseLink()->SetFrame(Vec3(0.0, 0.0, -10.0));
}

bool pathFeasibilityCheck(int curTimeStep, vector<SE3> recentObsTraj, int& collisionTimeStep, SE3& colSE3)
{
	// predict obstacle position
	se3 meanVel = Log(Inv(recentObsTraj[0]) * recentObsTraj.back()) * (double) (1.0 / recentObsTraj.size());
	SE3 predictSE3 = recentObsTraj.back() * Exp(meanVel * (double)predictionHorizon);
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
	// check collision for future object loc
	int collisionTimeStep2 = jointTraj_curPlan.size() + 1;
	for (unsigned int i = curTimeStep; i < jointTraj_curPlan.size(); i++)
	{
		robotrrtManager->setState(jointTraj_curPlan[i]);
		obs->GetBaseLink()->SetFrame(predictSE3);
		if (gSpace.m_srDYN.RUNTIME_MARKKH())
		{
			collisionTimeStep2 = i;
		}
	}
	collisionTimeStep = min(collisionTimeStep1, collisionTimeStep2);
	if (collisionTimeStep == collisionTimeStep2)
		colSE3 = predictSE3;
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
