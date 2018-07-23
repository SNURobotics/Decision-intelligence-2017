#define	INVKIN_TRAJ
//#define RRT_TRAJ

#include <cstdio>
#include <cassert>

#include "myRenderer.h"
//#include "2ndRenderer.h"

#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\UR5RobotManager.h"
#include "robotManager\UR5Robot.h"
#include "robotManager\environment_4th.h"
#include <time.h>
#include "common\dataIO.h"
#include "retargettingEnvSetting.h"
#include "robotManager\robotRRTManager.h"

// Robot
// Robot
// Right hand
UR5Robot* URRobot_right = new UR5Robot;
UR5RobotManager* rManager_right;
UR5Robot* URRobot_left = new UR5Robot;
UR5RobotManager* rManager_left;

// environment setting
retargetEnvironment* retargetEnv;



// Given traj
vector<Eigen::VectorXd> right_wrist_data;
vector<Eigen::VectorXd> left_wrist_data;


srSpace gSpace;
myRenderer* renderer;

SE3 Trobotbase1;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void updateFuncPlanning();
void URRobotSetting();
void URRobotManagerSetting();
void envSetting(int taskIdx);
void URrrtSetting();

// generate path with inverse kinematics
vector<Eigen::VectorXd> generateRetargetPath(string data_txt_path, int data_col_num, UR5RobotManager* robotManager, UR5Robot* robot, pair<int, int> taskFrameIdx);

// generate path with RRT
robotRRTManager* RRTManager_right = new robotRRTManager;
robotRRTManager* RRTManager_left = new robotRRTManager;
vector<SE3> waypoint_right(0);
vector<SE3> waypoint_left(0);
pair<vector<Eigen::VectorXd>, vector<Eigen::VectorXd>> rrtInitEndPos_right;
pair<vector<Eigen::VectorXd>, vector<Eigen::VectorXd>> rrtInitEndPos_left;
vector<vector<Eigen::VectorXd>> traj_right(0);
vector<vector<Eigen::VectorXd>> traj_left(0);
Eigen::VectorXd jointVal(6);
Eigen::VectorXd jointAcc(6);
Eigen::VectorXd jointVel(6);

int taskIdx;
string leftRight;
int trajSize_right;
int trajSize_left;


vector<SE3> generateRRTwaypoints(string data_txt_path, int data_col_num, UR5RobotManager* robotManager, UR5Robot* robot, pair<int, int> tasK_frame_idx, int interval);
pair<vector<Eigen::VectorXd>, vector<Eigen::VectorXd>>  RRT_problemSetting(vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, int taskIdx, string leftRight);
void RRTSolve(vector<double> stepsize, string leftRight);


int main(int argc, char **argv)
{
	retargetEnv = new retargetEnvironment();
	// add environment to system
	// task 1: result 1 folder blue connector
	// task 2: result 1 folder red connector
	taskIdx = 1;
	envSetting(taskIdx);
	


	// add robot to system
	URRobotSetting();

	initDynamics();
	URRobotManagerSetting();
	URrrtSetting();

	// data preperation
	// right wrist
	string data_txt_path_right = "D:/Google_Drive/판단지능_ksh_local/4차년도/재평가 관련/생기원 경로/result1/right_wrist_smooth.txt";
	string data_txt_path_left = "D:/Google_Drive/판단지능_ksh_local/4차년도/재평가 관련/생기원 경로/result1/left_wrist_smooth.txt";
	int dataColNum = 3;

#ifdef INVKIN_TRAJ

	pair<int, int> firstTask(0, 150);

	cout << " ===== Right arm motion generating start ======" << endl;
	right_wrist_data = generateRetargetPath(data_txt_path_right, dataColNum, rManager_right, URRobot_right,firstTask);

	// left wrist

	cout << " ===== Left arm motion generating start ======" << endl;
	left_wrist_data = generateRetargetPath(data_txt_path_left, dataColNum, rManager_left, URRobot_left, firstTask);
#endif

#ifdef RRT_TRAJ

	///////////// right hand first task
	//pair<int, int> rightHandFirstTask(0, 150);
	//// waypoint generation
	//int rrtWaypointInterval_right = 10;
	//waypoint_right = generateRRTwaypoints(data_txt_path_right, dataColNum, rManager_right, URRobot_right, rightHandFirstTask, rrtWaypointInterval_right);
	//// rrt problem setting
	//vector<bool> includeOri_right(waypoint_right.size(), true);
	////vector<bool> attachObject_right(waypoint_right.size(), true);
	//vector<bool> attachObject_right(waypoint_right.size(), false);
	//rrtInitEndPos_right = RRT_problemSetting(waypoint_right, includeOri_right, attachObject_right, taskIdx, "right");

	//// rrt solve
	//vector<double> stepsize_right(waypoint_right.size(), 0.05);
	//RRTSolve(stepsize_right, "right");

	/////////// left hand first task
	pair<int, int> leftHandFirstTask(0, 150);
	// waypoint generation
	int rrtWaypointInterval_left = 5;
	waypoint_left = generateRRTwaypoints(data_txt_path_left, dataColNum, rManager_left, URRobot_left, leftHandFirstTask, rrtWaypointInterval_left);
	// rrt problem setting
	//vector<bool> includeOri_left(waypoint_left.size(), true);
	vector<bool> includeOri_left(waypoint_left.size(), false);
	//vector<bool> attachObject_left(waypoint_right.size(), true);
	vector<bool> attachObject_left(waypoint_left.size(), false);
	rrtInitEndPos_left = RRT_problemSetting(waypoint_left, includeOri_left, attachObject_left, taskIdx, "left");

	// rrt solve
	vector<double> stepsize_left(waypoint_left.size(), 0.05);
	RRTSolve(stepsize_left, "left");

	////// Calculate traj size for rendering
	// right
	trajSize_right = 0;
	for (int i = 0; i < traj_right.size(); i++)
	{
		trajSize_right += traj_right[i].size();
	}
	trajSize_left = 0;
	for (int i = 0; i < traj_left.size(); i++)
	{
		trajSize_left += traj_left[i].size();
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
#ifdef INVKIN_TRAJ
	renderer->setUpdateFunc(updateFunc);
#endif
#ifdef RRT_TRAJ
	renderer->setUpdateFunc(updateFuncPlanning);
#endif


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
	switch (taskIdx)
	{
	case 1:
		// right arm motion
		if (right_wrist_data.size() > 0)
		{
			rManager_right->setJointVal(right_wrist_data[trajcnt % right_wrist_data.size()]);
			retargetEnv->m_blueFemaleConnetor->setBaseLinkFrame(rManager_right->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(retargetEnv->TblueFemale2robotEE));
		}

		// left arm motion
		if (left_wrist_data.size() > 0)
		{
			rManager_left->setJointVal(left_wrist_data[trajcnt % left_wrist_data.size()]);
			retargetEnv->m_blueMaleConnetor->setBaseLinkFrame(rManager_left->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(retargetEnv->TblueMale2robotEE));
		}
		break;
	case 2:
		// right arm motion
		if (right_wrist_data.size() > 0)
		{
			rManager_right->setJointVal(right_wrist_data[trajcnt % right_wrist_data.size()]);
			retargetEnv->m_redFemaleConnetor->setBaseLinkFrame(rManager_right->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(retargetEnv->TredFemale2robotEE));
		}

		// left arm motion
		if (left_wrist_data.size() > 0)
		{
			rManager_left->setJointVal(left_wrist_data[trajcnt % left_wrist_data.size()]);
			retargetEnv->m_redMaleConnetor->setBaseLinkFrame(rManager_left->m_activeArmInfo->m_endeffector[0]->GetFrame() * Inv(retargetEnv->TredMale2robotEE));
		}
		break;
			
	}


	


	
}


void URRobotSetting()
{
	// right hand robot
	gSpace.AddSystem((srSystem*)URRobot_right);
	URRobot_right->GetBaseLink()->SetFrame(retargetEnv->Trobotbase_right);
	URRobot_right->SetActType(srJoint::ACTTYPE::HYBRID);

	vector<int> gpIdx(2);
	gpIdx[0] = 0;
	gpIdx[1] = 1;
	URRobot_right->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);

	// left hand robot
	gSpace.AddSystem((srSystem*)URRobot_left);
	URRobot_left->GetBaseLink()->SetFrame(retargetEnv->Trobotbase_left);
	URRobot_left->SetActType(srJoint::ACTTYPE::HYBRID);

	URRobot_left->SetGripperActType(srJoint::ACTTYPE::HYBRID, gpIdx);


}

void URRobotManagerSetting()
{

	rManager_right = new UR5RobotManager(URRobot_right, &gSpace);
	rManager_left = new UR5RobotManager(URRobot_left, &gSpace);

}

vector<Eigen::VectorXd> generateRetargetPath(string data_txt_path, int data_col_num, UR5RobotManager* robotManager, UR5Robot* robot, pair<int, int> taskFrameIdx)
{
	vector<Eigen::VectorXd> data_from_txt = loadDataFromText(data_txt_path, data_col_num);
	vector < Eigen::VectorXd > robotTraj(0);
	int invKinFlag;
	for (int i_frame = taskFrameIdx.first; i_frame < taskFrameIdx.second; i_frame++)
	{
		Vec3 dataXYZ_Vec3_txt;

		for (int i_coord = 0; i_coord < data_from_txt[i_frame].size(); i_coord++)
		{
			dataXYZ_Vec3_txt[i_coord] = data_from_txt[i_frame][i_coord];
		}

		SE3 dataXYZ_SE3 = SE3();
		dataXYZ_SE3.SetPosition(dataXYZ_Vec3_txt);

		//cout << dataXYZ_Vec3_txt << endl;
		//cout << retargetEnv->Tworld2camera << endl;
		//SE3 AAA = retargetEnv->Tworld2camera*dataXYZ_Vec3_txt;
		//cout << AAA << endl;
		SE3 dataXYZfromWorld = retargetEnv->Tworld2camera*dataXYZ_SE3;
		//cout << dataXYZfromWorld << endl;
		SE3 dataXYZIdentity = SE3();
		dataXYZIdentity.SetPosition(dataXYZfromWorld.GetPosition());

		robotTraj.push_back(robotManager->inverseKin(dataXYZfromWorld, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], false, SE3(), invKinFlag, robot->qInvKinInit));
		//robotTraj.push_back(robotManager->inverseKin(dataXYZfromWorld, &robot->gLink[UR5_Index::LINK_6], false, SE3(), invKinFlag, robot->qInvKinInit));
		//robotTraj.push_back(robotManager->inverseKin(dataXYZIdentity, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag, robot->qInvKinInit));
		cout << "inverse kinematics flag: "<< invKinFlag << endl;
		if (invKinFlag != 0)
		{
			cout << "==========================" << endl;
			cout << "Inverse kineamtics flag is not ZERO!!!!!!" << endl;
			cout << "==========================" << endl;
		}
			
	}

	return robotTraj;
}

vector<SE3> generateRRTwaypoints(string data_txt_path, int data_col_num, UR5RobotManager* robotManager, UR5Robot* robot, pair<int, int> tasK_frame_idx, int interval)
{
	vector<Eigen::VectorXd> data_from_txt = loadDataFromText(data_txt_path, data_col_num);
	vector<SE3> waypoints(0);
	
	// check feasibility (first, end point's inverse kin solution exist?)
	int invKinFlag_first, invKinFlag_end;
	Vec3 dataXYZ_vec3_first;
	for (int i_coord = 0; i_coord < data_from_txt[tasK_frame_idx.first].size(); i_coord++)
	{
		dataXYZ_vec3_first[i_coord] = data_from_txt[tasK_frame_idx.first][i_coord];
	}
	SE3 dataXYZ_SE3_txt_first = SE3();
	dataXYZ_SE3_txt_first.SetPosition(dataXYZ_vec3_first);
	SE3 dataXYZfromWorld_first = retargetEnv->Tworld2camera*dataXYZ_SE3_txt_first;
	SE3 dataXYZIdentity_first = SE3();
	dataXYZIdentity_first.SetPosition(dataXYZfromWorld_first.GetPosition());
	//robotManager->inverseKin(dataXYZIdentity_first, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag_first, robot->qInvKinInit);
	robotManager->inverseKin(dataXYZfromWorld_first, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag_first, robot->qInvKinInit);

	Vec3 dataXYZ_vec3_end;
	for (int i_coord = 0; i_coord < data_from_txt[tasK_frame_idx.second-1].size(); i_coord++)
	{
		dataXYZ_vec3_end[i_coord] = data_from_txt[tasK_frame_idx.second-1][i_coord];
	}
	SE3 dataXYZ_SE3_txt_end = SE3();
	dataXYZ_SE3_txt_end.SetPosition(dataXYZ_vec3_end);
	SE3 dataXYZfromWorld_end = retargetEnv->Tworld2camera*dataXYZ_SE3_txt_end;
	SE3 dataXYZIdentity_end = SE3();
	dataXYZIdentity_end.SetPosition(dataXYZfromWorld_end.GetPosition());
	//robotManager->inverseKin(dataXYZIdentity_end, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag_end, robot->qInvKinInit);
	robotManager->inverseKin(dataXYZfromWorld_end, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag_end, robot->qInvKinInit);

	assert(invKinFlag_first == 0 && "first data point does not have invkin solution!");
	assert(invKinFlag_end == 0 && "end data point does not have invkin solution!");
		
	// If the feasibility check is finished, generate rrt waypoints
	int invKinFlag;
	for (int i_frame = tasK_frame_idx.first; i_frame < tasK_frame_idx.second; i_frame++)
	{
		Vec3 dataXYZ_Vec3_txt;
		if (i_frame == tasK_frame_idx.first)
		{
			for (int i_coord = 0; i_coord < data_from_txt[i_frame].size(); i_coord++)
			{
				dataXYZ_Vec3_txt[i_coord] = data_from_txt[i_frame][i_coord];
			}

			SE3 dataXYZ_SE3_txt = SE3();
			dataXYZ_SE3_txt.SetPosition(dataXYZ_Vec3_txt);


			SE3 dataXYZfromWorld = retargetEnv->Tworld2camera*dataXYZ_SE3_txt;



			SE3 dataXYZIdentity = SE3();
			dataXYZIdentity.SetPosition(dataXYZfromWorld.GetPosition());
			//waypoints.push_back(dataXYZIdentity);
			waypoints.push_back(dataXYZfromWorld);

		}
		else if (i_frame % interval == 0)
		{
			for (int i_coord = 0; i_coord < data_from_txt[i_frame].size(); i_coord++)
			{
				dataXYZ_Vec3_txt[i_coord] = data_from_txt[i_frame][i_coord];
			}

			SE3 dataXYZ_SE3_txt = SE3();
			dataXYZ_SE3_txt.SetPosition(dataXYZ_Vec3_txt);


			SE3 dataXYZfromWorld = retargetEnv->Tworld2camera*dataXYZ_SE3_txt;



			SE3 dataXYZIdentity = SE3();
			dataXYZIdentity.SetPosition(dataXYZfromWorld.GetPosition());

			robotManager->inverseKin(dataXYZIdentity, &robot->gMarkerLink[UR5_Index::MLINK_GRIP], true, SE3(), invKinFlag, robot->qInvKinInit);

			if (invKinFlag == 0)
				//waypoints.push_back(dataXYZIdentity);
				waypoints.push_back(dataXYZfromWorld);
		}
		
		else if (i_frame == tasK_frame_idx.second - 1)
		{
			for (int i_coord = 0; i_coord < data_from_txt[i_frame].size(); i_coord++)
			{
				dataXYZ_Vec3_txt[i_coord] = data_from_txt[i_frame][i_coord];
			}

			SE3 dataXYZ_SE3_txt = SE3();
			dataXYZ_SE3_txt.SetPosition(dataXYZ_Vec3_txt);


			SE3 dataXYZfromWorld = retargetEnv->Tworld2camera*dataXYZ_SE3_txt;



			SE3 dataXYZIdentity = SE3();
			dataXYZIdentity.SetPosition(dataXYZfromWorld.GetPosition());
			//waypoints.push_back(dataXYZIdentity);
			waypoints.push_back(dataXYZfromWorld);
		}
		
	}

	return waypoints;
}


void envSetting(int taskIdx)
{

	switch(taskIdx)
	{
	case 1: 
		gSpace.AddSystem(retargetEnv->m_blueFemaleConnetor);
		gSpace.AddSystem(retargetEnv->m_blueMaleConnetor);
		break;
	case 2:
		gSpace.AddSystem(retargetEnv->m_redFemaleConnetor);
		gSpace.AddSystem(retargetEnv->m_redMaleConnetor);
		break;
	}

	//retargetEnv->setEnvironmentInSrSpace(&gSpace);
}

void URrrtSetting()
{
	RRTManager_right->setSpace(&gSpace);
	vector<srStateJoint*> planningJoint(6);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)URRobot_right->gJoint[i];
	RRTManager_right->setSystem(planningJoint);
	RRTManager_right->setStateBound(URRobot_right->getLowerJointLimit(), URRobot_right->getUpperJointLimit());

	RRTManager_left->setSpace(&gSpace);
	for (int i = 0; i < 6; i++)
		planningJoint[i] = (srStateJoint*)URRobot_left->gJoint[i];
	RRTManager_left->setSystem(planningJoint);
	RRTManager_left->setStateBound(URRobot_left->getLowerJointLimit(), URRobot_left->getUpperJointLimit());

}


pair<vector<Eigen::VectorXd>, vector<Eigen::VectorXd>> RRT_problemSetting(vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, int taskIdx, string leftRight)
{

	vector<Eigen::VectorXd> initPos(0);
	vector<Eigen::VectorXd> goalPos(0);
	vector<bool> feas(2);
	int flag;

	switch(taskIdx){
	case 1:
		if (leftRight == "right")
		{
			initPos.push_back(rManager_right->inverseKin(wayPoints[0], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[0], SE3(), flag, URRobot_right->qInvKinInit));
			
			for (unsigned int i = 0; i < wayPoints.size()-1; i++)
			{
				//qInit = initPos[i];
				goalPos.push_back(rManager_right->inverseKin(wayPoints[i+1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i+1], SE3(), flag, URRobot_right->qInvKinInit));
				//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
				if (flag != 0)
					goalPos[i] = rManager_right->inverseKin(wayPoints[i+1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag);
				if (flag != 0)
					goalPos[i] = rManager_right->inverseKin(wayPoints[i+1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, initPos[i]);
				printf("%d-th init inv kin flag: %d\n", i, flag);
				if (i < wayPoints.size() - 2)
					initPos.push_back(goalPos[i]);
				if (attachObject[i + 1])
					RRTManager_right->attachObject(retargetEnv->m_blueFemaleConnetor, &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(retargetEnv->TblueFemale2robotEE));
				else
					RRTManager_right->detachObject();


				feas = RRTManager_right->checkFeasibility(initPos[i], goalPos[i]);
				cout << feas[0] << feas[1] << endl;

			}
		}
		else if (leftRight == "left")
		{
			initPos.push_back(rManager_left->inverseKin(wayPoints[0], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[0], SE3(), flag, URRobot_left->qInvKinInit));

			for (unsigned int i = 0; i < wayPoints.size()-1; i++)
			{
				//qInit = initPos[i];
				goalPos.push_back(rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, URRobot_left->qInvKinInit));
				//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
				if (flag != 0)
					goalPos[i] = rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag);
				if (flag != 0)
					goalPos[i] = rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, initPos[i]);
				printf("%d-th init inv kin flag: %d\n", i, flag);
				if (i < wayPoints.size() - 2)
					initPos.push_back(goalPos[i]);
				if (attachObject[i + 1])
					RRTManager_left->attachObject(retargetEnv->m_blueMaleConnetor, &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(retargetEnv->TblueMale2robotEE));
				else
					RRTManager_left->detachObject();


				feas = RRTManager_left->checkFeasibility(initPos[i], goalPos[i]);
				cout << feas[0] << feas[1] << endl;
			}
		}
		break;
	case 2:
		if (leftRight == "right")
		{
			initPos.push_back(rManager_right->inverseKin(wayPoints[0], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[0], SE3(), flag, URRobot_right->qInvKinInit));

			for (unsigned int i = 0; i < wayPoints.size() - 1; i++)
			{
				//qInit = initPos[i];
				goalPos.push_back(rManager_right->inverseKin(wayPoints[i + 1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, URRobot_right->qInvKinInit));
				//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
				if (flag != 0)
					goalPos[i] = rManager_right->inverseKin(wayPoints[i + 1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag);
				if (flag != 0)
					goalPos[i] = rManager_right->inverseKin(wayPoints[i + 1], &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, initPos[i]);
				printf("%d-th init inv kin flag: %d\n", i, flag);
				if (i < wayPoints.size() - 2)
					initPos.push_back(goalPos[i]);
				if (attachObject[i + 1])
					RRTManager_right->attachObject(retargetEnv->m_redFemaleConnetor, &URRobot_right->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(retargetEnv->TredFemale2robotEE));
				else
					RRTManager_right->detachObject();


				feas = RRTManager_right->checkFeasibility(initPos[i], goalPos[i]);
				cout << feas[0] << feas[1] << endl;

			}
		}
		else if (leftRight == "left")
		{
			initPos.push_back(rManager_left->inverseKin(wayPoints[0], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[0], SE3(), flag, URRobot_left->qInvKinInit));

			for (unsigned int i = 0; i < wayPoints.size() - 1; i++)
			{
				//qInit = initPos[i];
				goalPos.push_back(rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, URRobot_left->qInvKinInit));
				//goalPos.push_back(rManager1->inverseKin(wayPoints[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit)); // ONLY END-EFFECTOR POS/ORI
				if (flag != 0)
					goalPos[i] = rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag);
				if (flag != 0)
					goalPos[i] = rManager_left->inverseKin(wayPoints[i + 1], &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], includeOri[i + 1], SE3(), flag, initPos[i]);
				printf("%d-th init inv kin flag: %d\n", i, flag);
				if (i < wayPoints.size() - 2)
					initPos.push_back(goalPos[i]);
				if (attachObject[i + 1])
					RRTManager_left->attachObject(retargetEnv->m_redMaleConnetor, &URRobot_left->gMarkerLink[UR5_Index::MLINK_GRIP], Inv(retargetEnv->TredMale2robotEE));
				else
					RRTManager_left->detachObject();


				feas = RRTManager_left->checkFeasibility(initPos[i], goalPos[i]);
				cout << feas[0] << feas[1] << endl;
			}
		}
		break;
	}

	pair<vector<Eigen::VectorXd>, vector<Eigen::VectorXd>> output;
	output.first = initPos;
	output.second = goalPos;

	return output;

}

void RRTSolve(vector<double> stepsize, string leftRight)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	int start = 0;		//  >= 0
	int end = stepsize.size(); 		// <= 15
	vector<bool> feas(2);
	if (leftRight == "right")
	{
		traj_right.resize(0);
		for (int i = start; i < end; i++)
		{

			RRTManager_right->setStartandGoal(rrtInitEndPos_right.first[i], rrtInitEndPos_right.second[i]);

			//cout << "initpos:  " << initPos[i].transpose() << endl;
			//cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;


			RRTManager_right->execute(stepsize[i]);
			tempTraj = RRTManager_right->extractPath();
			// check collision
			for (unsigned int j = 0; j < tempTraj.size(); j++)
				if (RRTManager_right->setState(tempTraj[j]))
					cout << "collide at " << j << "th point!!!" << endl;

			traj_right.push_back(tempTraj);
		}
	}
	if (leftRight == "left")
	{
		traj_left.resize(0);
		for (int i = start; i < end; i++)
		{

			RRTManager_left ->setStartandGoal(rrtInitEndPos_left.first[i], rrtInitEndPos_left.second[i]);

			//cout << "initpos:  " << initPos[i].transpose() << endl;
			//cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;


			RRTManager_left->execute(stepsize[i]);
			tempTraj = RRTManager_left->extractPath();
			// check collision
			for (unsigned int j = 0; j < tempTraj.size(); j++)
				if (RRTManager_left->setState(tempTraj[j]))
					cout << "collide at " << j << "th point!!!" << endl;

			traj_left.push_back(tempTraj);
		}
	}
}


void updateFuncPlanning()
{

	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();

	static int cnt = 0;
	static int trjIdx_left = 0;
	static int trjIdx_right = 0;
	static int waypointIdx_left = 0;
	static int waypointIdx_right = 0;

	// both arm motion
	if (traj_right.size() > 0 && traj_left.size() > 0)
	{
		int idx_right = waypointIdx_right % traj_right.size();
		int idx_left = waypointIdx_left % traj_left.size();

		if (idx_right == 0 && idx_left == 0 && cnt > 0)
			cnt = 0;
		if (cnt == 0)
			cnt++;

		// right hand
		rManager_right->setJointVal(traj_right[idx_right][trjIdx_right]);
		trjIdx_right++;
		if (trjIdx_right == traj_right[idx_right].size())
		{
			trjIdx_right = 0;
			waypointIdx_right++;
			cout << "waypointIdx_right: " << waypointIdx_right % traj_right.size() << endl;
		}

		// left arm motion
		if (traj_left.size() > 0)
		{
			rManager_left->setJointVal(traj_left[idx_left][trjIdx_left]);
			trjIdx_left++;
			if (trjIdx_left == traj_left[idx_left].size())
			{
				trjIdx_left = 0;
				waypointIdx_left++;
				cout << "waypointIdx_left: " << waypointIdx_left % traj_left.size() << endl;
			}
		}
	}

	else if (traj_right.size() > 0)
	{
		int idx_right = waypointIdx_right % traj_right.size();

		if (idx_right == 0 && cnt > 0)
			cnt = 0;
		if (cnt == 0)
			cnt++;

		rManager_right->setJointVal(traj_right[idx_right][trjIdx_right]);
		trjIdx_right++;
		if (trjIdx_right == traj_right[idx_right].size())
		{
			trjIdx_right = 0;
			waypointIdx_right++;
			cout << "waypointIdx_right: " << waypointIdx_right % traj_right.size() << endl;
		}
	}

	else if (traj_left.size() > 0)
	{
		int idx_left = waypointIdx_left % traj_left.size();

		if (idx_left == 0 && cnt > 0)
			cnt = 0;
		if (cnt == 0)
			cnt++;

		rManager_left->setJointVal(traj_left[idx_left][trjIdx_left]);
		trjIdx_left++;
		if (trjIdx_left == traj_left[idx_left].size())
		{
			trjIdx_left = 0;
			waypointIdx_left++;
			cout << "waypointIdx_left: " << waypointIdx_left % traj_left.size() << endl;
		}
	}

	//int idx_right = waypointIdx_right % traj_right.size();
	//int idx_left = waypointIdx_left % traj_left.size();

	//if (idx_right == 0 && idx_left == 0 && cnt > 0)
	//	cnt = 0;
	//if (cnt == 0)
	//	cnt++;

	//cnt++;
	//if (cnt % 10 == 0)
	//{
	//	trjIdx_right++;
	//	trjIdx_left++;
	//}
		


	// right arm motion
	//if (traj_right.size() > 0)
	//{
	//	int idx_right = waypointIdx_right % traj_right.size();
	//	rManager_right->setJointVal(traj_right[idx_right][trjIdx_right]);
	//	trjIdx_right++;
	//	if (trjIdx_right == traj_right[idx_right].size())
	//	{
	//		trjIdx_right = 0;
	//		waypointIdx_right++;
	//		cout << "waypointIdx_right: " << waypointIdx_right % traj_right.size() << endl;
	//	}
	//}

	//// left arm motion
	//if (traj_left.size() > 0)
	//{
	//	rManager_left->setJointVal(traj_left[idx_left][trjIdx_left]);
	//	trjIdx_left++;
	//	if (trjIdx_left == traj_left[idx_left].size())
	//	{
	//		trjIdx_left = 0;
	//		waypointIdx_left++;
	//		cout << "waypointIdx_left: " << waypointIdx_left % traj_left.size() << endl;
	//	}
	//}


}