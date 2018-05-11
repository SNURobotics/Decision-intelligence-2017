#pragma once
#include "common\dataIO.h"
#include "robotManager\environment_4th.h"
#include "tcp_ip_communication_4th.h"
#include <Windows.h>
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


class demoTaskManager
{
public:
	demoTaskManager();
	~demoTaskManager();

public:
	// Vision information functions
	void updateEnv(char* stringfromSKKU);		// get vision data from SKKU, and update object locations
	bool setObjectNum(MH12Robot* robot, MH12RobotManager* rManager1);	// select object to move from object SE3 and grasp candidates (return true if object is reachable, false if none of the objects are reachable)
	bool sendError();				// send error when none of the objects are reachable

	// Pick and place task functions
	void setGoalNum(int goalNum);		// select goal to place object
	bool doJob(int goalNum);

	// do job functions (all functions send waypoints to robot, and return true when robot moved successfully)
	bool reachObject();		// plan to reach candidate SE3
	bool graspObject();		// go to object and actuate gripper
	bool moveObject();		// plan to goal SE3
	bool releaseObject();	// go to exact goal and release object
	bool goHomepos();	// return to home position

	
	// Yaskawa client communication functions
	bool goToWaypoint(SE3 Twaypoint);	// send robot waypoint commands after planning
	bool goThroughWaypoints(vector<SE3> Twaypoints);
	bool checkWaypointReached(SE3 Twaypoint);		// check if robot reached to the waypoint
	

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
	SE3 homeSE3;
	SE3 reachOffset;				// offset between grasp point and waypoint right before grasp point (to reach vertically to object)
	SE3 goalOffset;					// offset between goal point and waypoint right before goal point (to reach vertically to object)
	


	Eigen::VectorXd curRobotPos;	// current robot pos (Rx, Ry, Rz, px, py, pz)
	int maxTimeDuration;
private:
	int curGoalID;					// current goal ID 
};




