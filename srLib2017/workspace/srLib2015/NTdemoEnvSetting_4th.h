#pragma once
#include "../../VS2013/tcp_ip_server/stdafx.h"
#include "../../VS2013/tcp_ip_server/Server.h"
#include "common\dataIO.h"
#include "robotManager\environment_4th.h"
#include <Windows.h>
#include "robotManager\MH12Robot.h"
#include "robotManager\MH12RobotManager.h"
#include "robotManager/robotRRTManager.h"

#include <string>
#include <cstring>
#include <sstream>



class demoEnvironment
{
public:
	demoEnvironment(unsigned int _objectNum);
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
	demoTaskManager(demoEnvironment* _demoEnv, MH12RobotManager* _rManager);
	~demoTaskManager();

public:
	// Set robotRRTManager
	void setRobotRRTManager();

	// Vision information functions
	void updateEnv(char* stringfromSKKU);		// get vision data from SKKU, and update object locations
	void readSKKUvision(char* hyu_data, vector<SE3>& objectSE3, vector<bool>& isHead, vector<vector<Vec3>>& objectGraspCandidatePos);

	bool setObjectNum();	// select object to move from object SE3 and grasp candidates (return true if object is reachable, false if none of the objects are reachable)
	bool sendError();				// send error when none of the objects are reachable

	// Pick and place task functions
	void setGoalNum(int goalNum);		// select goal to place object
	bool doJob(int goalNum);

	// do job functions (all functions send waypoints to robot, and return true when robot moved successfully)
	bool reachObject(bool usePlanning = false);		// plan to reach candidate SE3
	bool graspObject();		// go to object and actuate gripper
	bool moveObject(bool usePlanning = false);		// plan to goal SE3
	bool releaseObject();	// go to exact goal and release object
	bool goHomepos(bool usePlanning = false);	// return to home position

	vector<SE3> planBetweenWaypoints(SE3 Tinit, SE3 Tgoal, unsigned int midNum = 1);
	
	// Yaskawa client communication functions
	SE3 YKpos2SE3(const Eigen::VectorXd YKpos);
	bool goToWaypoint(SE3 Twaypoint);	// send robot waypoint commands after planning
	bool goThroughWaypoints(vector<SE3> Twaypoints);
	bool checkWaypointReached(SE3 Twaypoint);		// check if robot reached to the waypoint
	void getCurPosSignal();						// send robot read cur pos command and set curRobotPos and TcurRobot
	void setCurPos(vector<double> values);
public:
	// demo environemnt
	demoEnvironment * demoEnv;

	// robotManager and robotRRTManager model
	MH12Robot* robot;
	MH12RobotManager* rManager;
	robotRRTManager* robotrrtManager;

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

	// Robot communication related variables
	Eigen::VectorXd curRobotPos;	// current robot pos (Rx, Ry, Rz, px, py, pz)
	SE3 TcurRobot;
	int maxTimeDuration=10000;
	struct MOVE_POS
	{
		char Rx[256];
		char Ry[256];
		char Rz[256];
		char X[256];
		char Y[256];
		char Z[256];
	};
	bool isGetPos;


	// Planning related variables
	Eigen::VectorXd lastPlanningJointVal;
	SE3 lastObjectSE3;
	vector<Eigen::VectorXd> tempTraj;
	vector<SE3> tempObjTraj;
private:
	int curGoalID;					// current goal ID 
};




