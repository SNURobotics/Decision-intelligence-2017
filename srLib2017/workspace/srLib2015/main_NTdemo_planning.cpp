#include <cstdio>

#include "myRenderer.h"
#include "NTdemoEnvSetting_4th.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager\MH12Robot.h"
#include <time.h>


// srSpace and renderer
srSpace gSpace;
myRenderer* renderer;

// Robot
MH12Robot* MHRobot = new MH12Robot;
MH12RobotManager* rManager1;

Eigen::VectorXd qval;

// demo environment
demoEnvironment* demoEnv;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
demoTaskManager* demoTask;

void initDynamics();
void loadDataFromFile(string loc, char* visiondata);
void rendering(int argc, char **argv);
void updateFunc();
void envSetting();
void MHRobotSetting();
void MHRobotManagerSetting();
void URrrtSetting();
int activeJointIdx =0;
vector<Eigen::VectorXd> traj(0);


int main(int argc, char **argv)
{
	clock_t rrt1_begin, rrt1_end, rrt2_begin, rrt2_end, rrt3_begin, rrt3_end;

	////////////////////////////////////////////////////////////////
	////////////////////// initialize //////////////////////////////
	////////////////////////////////////////////////////////////////
	// set the number of objects in demoEnvirionment function (Since object data is now using dummy data, set the number as 1)
	demoEnv = new demoEnvironment(1);
	// add robot to system
    MHRobotSetting();
	// add bin and objects to system
	envSetting();
	initDynamics();
	MHRobotManagerSetting();
	demoTask = new demoTaskManager(demoEnv, rManager1);


	srand((unsigned int)time(0));
	vector<unsigned int> random_dummy;
	random_dummy.resize(10);
	for (unsigned int i = 0; i < 10; i++)
		random_dummy[i] = rand() % 1000;
	
	char dummy[1000];
	strcpy(dummy, "Vd0d");



	strcat(dummy, std::to_string(-0.1 + random_dummy[0] / 5000.0).c_str()); strcat(dummy, "d");
	strcat(dummy, std::to_string(-0.05 + random_dummy[1] / 10000.0).c_str()); strcat(dummy, "d");
	strcat(dummy, std::to_string(0.9 + random_dummy[2] / 10000.0).c_str()); strcat(dummy, "d");

	SE3 T_obj = EulerZYX(Vec3(random_dummy[3] / 6000.0 * SR_PI, random_dummy[4] / 6000.0 * SR_PI, random_dummy[5] / 6000.0 * SR_PI));

	for (unsigned int i = 0; i < 9; i++)
	{
		strcat(dummy, std::to_string(T_obj[i]).c_str()); strcat(dummy, "d");
	}
	strcat(dummy, "-0.02d-0.035d-0.0025d0.0025d");

	demoTask->updateEnv(dummy);
	demoTask->setObjectNum();

	bool collision = rManager1->checkCollision();

	if (!collision)
		cout << "collision free!" << endl;
	else
		cout << "collision OCCURED!" << endl;


	/////////////////////////////////////////////////////
	////////////// test planning ////////////////////////
	/////////////////////////////////////////////////////
	demoTask->setRobotRRTManager();
	//demoTask->curObjID = 0;
	Eigen::VectorXd qIKinit(6);
	qIKinit[0] = 0.0417904; qIKinit[1] = 0.283304; qIKinit[2] = -0.513154;
	qIKinit[3] = -0.555694; qIKinit[4] = -1.0634; qIKinit[5] = 0.356608;
	demoTask->lastPlanningJointVal = qIKinit;
	//cout << curobjSE3 << endl;
	//cout << demoTask->curGraspOffset << endl;
	// print home pos
	//cout << "home pos" << endl;
	//cout << rManager1->forwardKin(Eigen::VectorXd::Zero(6), &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], SE3());
	//cout << &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//// check reach pos
	//Eigen::VectorXd qreach = rManager1->inverseKin(curobjSE3 * demoTask->curGraspOffset * SE3(Vec3(0.0, 0.0, -0.00)), &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qInit);
	//cout << flag << endl;
	//// set temp goal
	//SE3 tempGoal = SE3(Vec3(0.1, 0.8, 0.0));
	//Eigen::VectorXd qgoal = rManager1->inverseKin(tempGoal * demoTask->curGraspOffset, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag);
	//cout << flag << endl;
	
	///////////////// check reach task
	/////////////////////////////////////////////////////
	SE3 curobjSE3 = demoTask->curObjectData.objectSE3[demoTask->curObjID];
	cout << curobjSE3 << endl;
	SE3 curGraspOffset = demoTask->curGraspOffset;
	cout << "grasp offset" << endl;
	cout << curGraspOffset << endl;
	SE3 reachOffset = demoTask->reachOffset;
	rrt1_begin = clock();
	demoTask->planBetweenWaypoints(demoTask->homeSE3, curobjSE3 * curGraspOffset * reachOffset);
	rrt1_end = clock();
	//Eigen::VectorXd qtemp = rManager1->inverseKin(demoTask->homeSE3, &MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qIKinit);
	//rManager1->setJointVal(qtemp);
	//cout << flag << endl;
	//demoTask->reachObject(true);
	vector<Eigen::VectorXd> traj1 = demoTask->tempTraj;
	vector<SE3> objTraj1 = demoTask->tempObjTraj;
	///////////////// check move task
	/////////////////////////////////////////////////////
	SE3 goalSE3 = demoTask->goalSE3[0];
	SE3 goalOffset = demoTask->goalOffset;
	demoTask->robotrrtManager->attachObject(demoEnv->objects[demoTask->curObjID], &demoTask->robot->gMarkerLink[MH12_Index::MLINK_GRIP], Inv(curGraspOffset));
	rrt2_begin = clock();
	demoTask->planBetweenWaypoints(curobjSE3 * curGraspOffset, goalSE3 * curGraspOffset * goalOffset);
	rrt2_end = clock();
	//demoTask->moveObject(true);
	vector<Eigen::VectorXd> traj2 = demoTask->tempTraj;
	vector<SE3> objTraj2 = demoTask->tempObjTraj;
	//////////////// check return task
	demoTask->robotrrtManager->detachObject();
	rrt3_begin = clock();
	demoTask->planBetweenWaypoints(goalSE3 * curGraspOffset, demoTask->homeSE3);
	rrt3_end = clock();
	//demoTask->goHomepos(true);
	vector<Eigen::VectorXd> traj3 = demoTask->tempTraj;
	vector<SE3> objTraj3 = demoTask->tempObjTraj;

	vector<Eigen::VectorXd> totalTraj = traj1;
	vector<SE3> totalObjTraj = objTraj1;
	totalTraj.insert(totalTraj.end(), traj2.begin(), traj2.end());
	totalTraj.insert(totalTraj.end(), traj3.begin(), traj3.end());
	totalObjTraj.insert(totalObjTraj.end(), objTraj2.begin(), objTraj2.end());
	totalObjTraj.insert(totalObjTraj.end(), objTraj3.begin(), objTraj3.end());
	demoTask->tempTraj = totalTraj;
	demoTask->tempObjTraj = totalObjTraj;
	//Eigen::VectorXd qInit = rManager1->inverseKin(demoTask->homeSE3, &demoTask->robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qIKinit);
	//cout << flag << endl;
	//cout << qInit.transpose() << endl;
	//rManager1->setJointVal(qInit);
	//Eigen::VectorXd qGoal = rManager1->inverseKin(curobjSE3 * curGraspOffset * reachOffset, &demoTask->robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qIKinit);
	//cout << flag << endl;
	//rManager1->setJointVal(qGoal);
	cout << "수행시간: " << ((rrt3_end - rrt3_begin) + (rrt2_end - rrt2_begin) + (rrt1_end - rrt1_begin)) << endl;

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
	//((srStateJoint*)MHRobot->m_KIN_Joints[activeJointIdx])->m_State.m_rValue[0] = JointVal;
	//((srStateJoint*)MHRobot->m_KIN_Joints[5])->m_State.m_rValue[0] = JointVal;
	//JointVal += 0.01;
	//qval[5] = JointVal;
	//rManager1->setJointVal(qval);
	static int cnt = 0;
	static int trajcnt = 0;
	if (cnt == 0)
	{
		printf("collision: ");
		cout << rManager1->checkCollision() << endl;
	}
	cnt++;
	
		
	int trjIdx = 0;
	if (cnt % 10 == 0)
		trajcnt++;
	if (demoTask->tempTraj.size() > 0)
	{
		trjIdx = trajcnt % demoTask->tempTraj.size();
		rManager1->setJointVal(demoTask->tempTraj[trjIdx]);
		demoTask->demoEnv->objects[demoTask->curObjID]->setBaseLinkFrame(demoTask->tempObjTraj[trjIdx]);
		//if (cnt % 10 == 0)
		//	cout << demoTask->tempObjTraj[trjIdx] % MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame();
	}
		

	
	//cout << MHRobot->gMarkerLink[MH12_Index::MLINK_GRIP].GetFrame() << endl;
	//cout << MHRobot->gLink[MH12_Index::GRIPPER].GetFrame() << endl;
	//rManager1->setJointVal(qval);

	int stop = 1;
}


void MHRobotSetting()
{
	gSpace.AddSystem((srSystem*)MHRobot);
	MHRobot->GetBaseLink()->SetFrame(demoEnv->Trobotbase * demoEnv->Trobotbase2link1);
	MHRobot->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	MHRobot->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	//Trobotbase1 = MHRobot->GetBaseLink()->GetFrame() * MHRobot->TsrLinkbase2robotbase;
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
	rManager1 = new MH12RobotManager(MHRobot, &gSpace);
}

void envSetting()
{
	demoEnv->setEnvironmentInSrSpace(&gSpace);
}

void loadDataFromFile(string loc, char* visiondata)
{
	std::ifstream fin;
	fin.open(loc);

	int i = 0;

	if (fin.is_open())
	{
		while (!fin.eof())
		{
			fin >> visiondata[i];
			i++;
		}
	}
	fin.close();
}