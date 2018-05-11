#include "NTdemoEnvSetting_4th.h"


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

demoTaskManager::demoTaskManager()
{
	int maxTimeDuration = 60000;		// means max 60000ms per movement
	double posThreshold = 0.0001;		// threshold to check if a waypoint is reached
}

demoTaskManager::~demoTaskManager()
{
}

void demoTaskManager::setGoalNum(int goalNum)
{
	curGoalID = goalNum;
}

bool demoTaskManager::doJob(int goalNum)
{
	setGoalNum(goalNum);
	bool reached = reachObject();
	bool grasped = graspObject();
	bool moved = moveObject();
	bool released = releaseObject();
	bool returned = goHomepos();
	return (reached && grasped && moved && released && returned);
}

bool demoTaskManager::reachObject()
{
	return goToWaypoint(curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset);
}

bool demoTaskManager::graspObject()
{
	bool moved = goToWaypoint(curObjectData.objectSE3[curObjID] * curGraspOffset);
	// send message to robot to grasp (gripper command ??)
	bool grasped = false;
	////////////////////////////////////////////////////////
	return moved && grasped;
}

bool demoTaskManager::moveObject()
{
	return goToWaypoint(goalSE3[curGoalID] * goalOffset);
}

bool demoTaskManager::releaseObject()
{
	bool moved = goToWaypoint(goalSE3[curGoalID]);
	// send message to robot to release (gripper command ??)
	bool released = false;
	////////////////////////////////////////////////////////
	return moved && released;
}

bool demoTaskManager::goHomepos()
{
	return goToWaypoint(homeSE3);
}

bool demoTaskManager::goToWaypoint(SE3 Twaypoint)
{
	// send message to robot (imov command) here
	vector<double> tempOri = SO3ToEulerXYZ(Twaypoint.GetOrientation());
	Vec3 tempPos = Twaypoint.GetPosition();
	/////////////////////////////////////////////
	int cnt = 0;
	while (cnt < maxTimeDuration)
	{
		// check if robot has reached its waypoint for every 50 ms
		Sleep(50);
		cnt += 50;
		if (checkWaypointReached(Twaypoint))
			return true;
	}
	return false;
}

bool demoTaskManager::goThroughWaypoints(vector<SE3> Twaypoints)
{
	bool success = true;
	for (unsigned int i = 0; i < Twaypoints.size(); i++)
		success &= goToWaypoint(Twaypoints[i]);
	return success;
}

bool demoTaskManager::checkWaypointReached(SE3 Twaypoint)
{
	// send message to robot (read cur pos command) here
	curRobotPos;
	////////////////////////////////////////////////////
	if (distSE3(Twaypoint, EulerXYZ(Vec3(curRobotPos[0], curRobotPos[1], curRobotPos[2]), Vec3(curRobotPos[3], curRobotPos[4], curRobotPos[5]))) < posThreshold)
		return true;
	return false;
}

bool demoTaskManager::sendError()
{
	return false;
}

SKKUobjectData::SKKUobjectData()
{
}

SKKUobjectData::~SKKUobjectData()
{
}

void SKKUobjectData::setObjectDataFromString(string _string)
{
}
