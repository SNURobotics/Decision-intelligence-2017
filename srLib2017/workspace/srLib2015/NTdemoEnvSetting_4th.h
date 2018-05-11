#pragma once
#include "common\dataIO.h"
#include "robotManager\environment_4th.h"
#include "tcp_ip_communication.h"
#include "robotManager\MH12Robot.h"
#include "robotManager\MH12RobotManager.h"

class demoEnvironment
{
public:
	demoEnvironment();
	~demoEnvironment();

	void setObjectFromRobot2ObjectText(string loc, bool print = false);
	void setObjectFromRobot2VisionData(vector<SE3> objectSE3);

public:
	SE3 Trobotbase;
	SE3 Trobotbase2link1;
	SE3 Tworld2camera;
	Bin* bin;
	Table4th* table;
	unsigned int objectNum;
	vector<workingObject*> objects;
	SE3 Trobotbase2camera;
	SE3 Tcamera2robotbase;
	Vec3 Plink12bin;
	Vec3 Plink12table;
};


demoEnvironment::demoEnvironment() {
	
	Trobotbase = SE3();		// same as world frame origin
	Trobotbase2link1 = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, -0.450));		// change to exact value later
	// from data in e-mail 180424
	Tworld2camera = SE3(-0.5810862069, 0.4871189722, 0.6519615994, 
		0.8100935467, 0.4230013330, 0.4059782233, 
		-0.0780209307, 0.7640582303, -0.6404121760, 
		-0.0080738399, -0.0719967823, 0.7784732222);

	// from data in e-mail 180504
	Tcamera2robotbase = SE3(0.977314, 0.073841, -0.198506, 0.027069, -0.886021, -0.462855, -0.210058, 0.457728, -0.863922, 1.095999, -0.310359, 0.925915);
	Trobotbase2camera = Inv(Tcamera2robotbase);
	
	// set bin
	bin = new Bin(0.01);
	Plink12bin = Vec3(0.89, 0.14, 0.45);
	bin->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Plink12bin));	// change to exact value later

	table = new Table4th(0.01);
	Plink12table = Vec3(0.89, 0.0, 0.412);
	table->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(0.0, 0.0, 0.0), Plink12table));	// change to exact value later


	// set objects
	objectNum = 5;
	objects.resize(objectNum);
	for (unsigned int i = 0; i < objectNum; i++)
		objects[i] = new workingObject;
}

demoEnvironment::~demoEnvironment()
{
	delete bin;
	delete table;
	for (unsigned int i = 0; i < objectNum; i++)
		delete objects[i];
}

void demoEnvironment::setObjectFromRobot2ObjectText(string loc, bool print /*= false*/)
{
	std::vector<Eigen::VectorXd> objectloc = loadDataFromText(loc, 6);
	if (objectloc.size() > objectNum)
		printf("object number in the text is larger than preset object number!!!\n");
	for (unsigned int i = 0; i < min(objectloc.size(), objectNum); i++)
	{
		Eigen::VectorXd temp = objectloc[i];
		SE3 tempSE3 = EulerXYZ(Vec3(temp(0), temp(1), temp(2)), Vec3(temp(3), temp(4), temp(5)));
		if (print)
			cout << tempSE3 << endl;
		objects[i]->setBaseLinkFrame(Trobotbase * tempSE3);
	}
}

void demoEnvironment::setObjectFromRobot2VisionData(vector<SE3> objectSE3)
{
	if (objectSE3.size() ==0)
	{
		printf("current object data does not exist!\n");
		return;
	}

	for (unsigned int i = 0; i < objectNum; i++)
	{
		objects[i]->setBaseLinkFrame(Trobotbase * objectSE3[i]);
	}

	return;
}

class SKKUobjectData
{
public:
	SKKUobjectData();
	~SKKUobjectData();

	void setObjectDataFromString(vector<SE3> _objectSE3, vector<bool> _isHead, vector<vector<Vec3>> _objectGraspCandidatePos);
public:
	unsigned int objectNum;
	vector<SE3> objectSE3;
	vector<bool> isHead;
	vector<vector<Vec3>> objectGraspCandidatePos;		// first vector: object ID, second vector: candidates per object
};

SKKUobjectData::SKKUobjectData()
{
	objectNum = 5;
	objectSE3.resize(objectNum);
	isHead.resize(objectNum);
	objectGraspCandidatePos.resize(objectNum);
}

SKKUobjectData::~SKKUobjectData()
{
	objectSE3.clear();
	isHead.clear();
	objectGraspCandidatePos.clear();
}

void SKKUobjectData::setObjectDataFromString(vector<SE3> _objectSE3, vector<bool> _isHead, vector<vector<Vec3>> _objectGraspCandidatePos)
{
	objectSE3.clear();
	isHead.clear();
	objectGraspCandidatePos.clear();

	objectSE3 = _objectSE3;
	isHead = _isHead;
	objectGraspCandidatePos = _objectGraspCandidatePos;
}

class demoTaskManager
{
public:
	demoTaskManager();
	~demoTaskManager();

public:
	// Vision information functions
	void updateEnv(char* stringfromSKKU);		// get vision data from SKKU, and update object locations
	bool setObjectNum(MH12Robot* robot, MH12RobotManager* rManager1);	// select object to move from object SE3 and grasp candidates (return true if object is reachable, false if none of the objects are reachable)
	
	// Pick and place task functions
	bool setGoalNum();		// select goal to place object
	bool doJob(int reachNum, int moveNum);

	// do job functions (all functions send waypoints to robot, and return true when robot moved successfully)
	bool reachObject();		// plan to reach candidate SE3
	bool graspObject();		// go to object and actuate gripper
	bool moveObject();		// plan to goal SE3
	bool releaseObject();	// go to exact goal and release object
	bool returnHomepos();	// return to home position

	
	// Yaskawa client communication functions
	bool goToWaypoint(SE3 Twaypoint);	// send robot waypoint commands after planning
	bool checkWaypointReached();		// check if robot reached to the waypoint

	bool sendError();				// send error when none of the objects are reachable

public:
	// Vision-Robot coordinate change SE3
	SE3 Tworld2camera;
	SE3 Trobotbase2camera;
	SE3 Tcamera2robotbase;

	// SKKU vision data related variables
	SKKUobjectData curObjectData;
	unsigned int objectNum;
	int curObjID;					// current object ID (0~4)
	SE3 curGraspOffset;				// current SE3 from object frame to grasp frame

	// Place task related variables
	vector<SE3> goalSE3;			// goal SE3 of objects (should be predefined and be the same as workspace)
	double posThreshold;			// threshold to decide whether waypoints are reached
	SE3 homePosition;
	SE3 reachOffset;				// offset between grasp point and waypoint right before grasp point (to reach vertically to object)
	SE3 goalOffset;					// offset between goal point and waypoint right before goal point (to reach vertically to object)
	int curGoalID;					// current goal ID 

	// Waypoint moving sub-task related variables
	vector<SE3> curWaypointSet;		// waypoint set of subtasks
	int curWaypointNum;				// current waypoint number of subtask
};


demoTaskManager::demoTaskManager()
{
	// from data in e-mail 180424
	Tworld2camera = SE3(-0.5810862069, 0.4871189722, 0.6519615994,
		0.8100935467, 0.4230013330, 0.4059782233,
		-0.0780209307, 0.7640582303, -0.6404121760,
		-0.0080738399, -0.0719967823, 0.7784732222);

	// from data in e-mail 180504
	Tcamera2robotbase = SE3(0.977314, 0.073841, -0.198506, 0.027069, -0.886021, -0.462855, -0.210058, 0.457728, -0.863922, 1.095999, -0.310359, 0.925915);
	Trobotbase2camera = Inv(Tcamera2robotbase);

	objectNum = 5;
}

demoTaskManager::~demoTaskManager()
{


}


void demoTaskManager::updateEnv(char* stringfromSKKU)
{
	// read vision data by readSKKUvision function from tcp_ip_communication header
	vision_data skku_dataset;
	readSKKUvision(stringfromSKKU, skku_dataset);

	vector<SE3> objectSE3;
	vector<bool> isHead;
	vector<vector<Vec3>> objectGraspCandidatePos;

	objectSE3.resize(objectNum);
	isHead.resize(objectNum);
	objectGraspCandidatePos.resize(objectNum);

	for (unsigned int i = 0; i < objectNum; i++)
	{
		SE3 objSE3_Camera = SE3(skku_dataset.objOri[i][0], skku_dataset.objOri[i][1], skku_dataset.objOri[i][2], skku_dataset.objOri[i][3], skku_dataset.objOri[i][4], skku_dataset.objOri[i][5], skku_dataset.objOri[i][6], skku_dataset.objOri[i][7], skku_dataset.objOri[i][8], skku_dataset.objPos[i][0], skku_dataset.objPos[i][1], skku_dataset.objPos[i][2]);
		
		// coordinate change
		objectSE3[i] = Tcamera2robotbase * objSE3_Camera;
		
		// is z direcition of objectSE3 is upward
		isHead[i] = (objectSE3[i][11] > 0);

		objectGraspCandidatePos[i].resize(skku_dataset.objCand[i]);
		for (unsigned int j = 0; j < (unsigned int)skku_dataset.objCand[i]; j++)
			objectGraspCandidatePos[i][j] = Vec3(skku_dataset.objCandPos[i][j][0], skku_dataset.objCandPos[i][j][1], skku_dataset.objCandPos[i][j][2]);
	}

	curObjectData.setObjectDataFromString(objectSE3, isHead, objectGraspCandidatePos);

	return;
}



bool demoTaskManager::setObjectNum(MH12Robot* robot, MH12RobotManager* rManager1)
{
	int flag;
	int nWay = 3 * objectNum;
	vector<bool> includeOri(nWay, true);

	if (curObjectData.objectSE3.size() == 0)
	{
		printf("current object data does not exist!\n");
		return false;
	}

	for (unsigned int i = 0; i < objectNum; i++)
	{
		for (unsigned int j = 0; j < size(curObjectData.objectGraspCandidatePos[i]); j++)
		{
			SE3 targetObject = curObjectData.objectSE3[i] * SE3(curObjectData.objectGraspCandidatePos[i][j]);
			rManager1->inverseKin(targetObject, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag);
			if (flag == 0)
			{
				curObjID = i;
				curGraspOffset = SE3(curObjectData.objectGraspCandidatePos[i][j]);
				return true;
			}
		}
	}

	return false;
}