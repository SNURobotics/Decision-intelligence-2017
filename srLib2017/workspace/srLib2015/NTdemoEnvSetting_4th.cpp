#include "NTdemoEnvSetting_4th.h"

demoEnvironment::demoEnvironment(unsigned int _objectNum) {

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

	// set table
	table = new Table4th(0.01);
	Plink12table = Vec3(0.89, 0.0, 0.41);
	table->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(0.0, 0.0, 0.0), Plink12table));	// change to exact value later
	
																												// set objects
	objectNum = _objectNum;
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


void demoEnvironment::setObjectFromRobot2VisionData(vector<SE3> objectSE3)
{
	if (objectSE3.size() == 0)
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


SKKUobjectData::SKKUobjectData()
{
	objectNum = 1;
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


demoTaskManager::demoTaskManager(demoEnvironment* _demoEnv, MH12RobotManager* _rManager)
{
	int maxTimeDuration = 60000;		// means max 60000ms per movement
	double posThreshold = 0.0001;		// threshold to check if a waypoint is reached
	demoEnv = _demoEnv;
	rManager = _rManager;
	robot = (MH12Robot*)rManager->m_robot;
	robotrrtManager = NULL;
	objectNum = 5;

	// constants for task (should be modified later!!!)
	goalSE3.resize(1);
	goalSE3[0] = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.226026, 0.888197, 0.466843));
	homeSE3 = EulerZYX(Vec3(SR_PI, -SR_PI_HALF, 0.0), Vec3(1.029, 0.0, 0.814));		// where robot goes when job is done (should be modified)
	reachOffset = SE3(Vec3(0.0, 0.0, -0.03));
	goalOffset = SE3(Vec3(0.0, 0.0, 0.0));
}

demoTaskManager::~demoTaskManager()
{
	if (robotrrtManager != NULL)
		delete robotrrtManager;
}

void demoTaskManager::setRobotRRTManager()
{
	robotrrtManager = new robotRRTManager;
	vector<srStateJoint*> planningJoints(DEGREE_OF_FREEDOM_MH12_JOINT);
	robotrrtManager->setSystem(rManager->m_robot);
	robotrrtManager->setSpace(rManager->m_space);
	robotrrtManager->setStateBound(robot->getLowerJointLimit(), robot->getUpperJointLimit());
}

void demoTaskManager::updateEnv(char* stringfromSKKU)
{
	// read vision data by readSKKUvision function from tcp_ip_communication header

	vector<SE3> objectSE3;
	vector<bool> isHead;
	vector<vector<Vec3>> objectGraspCandidatePos;

	objectSE3.resize(objectNum);
	isHead.resize(objectNum);
	objectGraspCandidatePos.resize(objectNum);
	
	readSKKUvision(stringfromSKKU, objectSE3, isHead, objectGraspCandidatePos);

	curObjectData.setObjectDataFromString(objectSE3, isHead, objectGraspCandidatePos);

	demoEnv->setObjectFromRobot2VisionData(curObjectData.objectSE3);

	return;
}

static void Eliminate(char *str, char ch)
{
	for (; *str != '\0'; str++)//종료 문자를 만날 때까지 반복
	{
		if (*str == ch)//ch와 같은 문자일 때
		{
			strcpy(str, str + 1);
			str--;
		}
	}
}

void demoTaskManager::readSKKUvision(char* hyu_data, vector<SE3>& objectSE3, vector<bool>& isHead, vector<vector<Vec3>>& objectGraspCandidatePos)
{
	// read vision data
	Eliminate(hyu_data, 'V');

	// temporary vector variables
	vector<int>	v_objID;
	vector<vector<double>> v_objPos;					//id, (x, y, z)
	vector<vector<double>> v_objOri;					//id, (R_x, R_y, R_z)
	vector<int> v_objCand;							//id, (number of candidate)
	vector<vector<double>> v_objCandPos;		//id, cand_id, (x, y, z)
	vector<vector<double>> v_obsInfo;					//id, (center, size)

	// read char data and save it to vector double format
	char *recv_data = strtok(hyu_data, "d");
	int recv_cnt = 0;
	int nway_cnt = 0;
	v_objID.resize(0);
	v_objPos.resize(0);
	v_objOri.resize(0);
	v_objCand.resize(0);
	v_objCandPos.resize(0);
	v_obsInfo.resize(0);
	int objIdx = 0;
	int objID;
	int obsID = 0;
	int max_recv_cnt = 14;
	bool obsData = false;

	while (recv_data != NULL)
	{
		//cout << recv_data << endl;
		if (!obsData && recv_cnt < 1)
		{
			objID = atoi(recv_data);
			if (objID == -1)
				obsData = true;
			else
			{
				v_objID.push_back(objID);
				v_objPos.resize(objID + 1);
				v_objOri.resize(objID + 1);
				v_objCandPos.resize(objID + 1);
			}
		}
		if (obsData && recv_cnt < 1)
		{
			objID = atoi(recv_data);
			if (objID == 0)
				break;
			else
			{
				vector<double> temp(0);
				v_obsInfo.push_back(temp);
			}
		}

		if (!obsData)
		{
			if (1 <= recv_cnt && recv_cnt < 4)
				v_objPos[objIdx].push_back(atof(recv_data));
			else if (4 <= recv_cnt && recv_cnt < 13)
			{
				v_objOri[objIdx].push_back(atof(recv_data));
			}
			else if (13 <= recv_cnt && recv_cnt < 14)
			{
				v_objCand.push_back(atoi(recv_data));
				max_recv_cnt = 14 + 3 * v_objCand[objIdx];
			}
			else if (14 <= recv_cnt && recv_cnt < max_recv_cnt)
			{
				v_objCandPos[objIdx].push_back(atof(recv_data));
			}

		}
		else if (recv_cnt > 0)
		{
			v_obsInfo[v_obsInfo.size() - 1].push_back(atof(recv_data));
		}

		recv_data = strtok(NULL, "d");
		recv_cnt += 1;

		if (!obsData && recv_cnt == max_recv_cnt)
		{
			recv_cnt = 0;
			objIdx++;
		}
		if (obsData && recv_cnt == 7)
			recv_cnt = 0;
	}



	// transform vector data into Vec3 and SE3 format
	for (unsigned int i = 0; i < objectNum; i++)
	{
		SE3 objSE3_Camera = SE3(v_objOri[i][0], v_objOri[i][1], v_objOri[i][2], v_objOri[i][3], v_objOri[i][4], v_objOri[i][5], v_objOri[i][6], v_objOri[i][7], v_objOri[i][8], v_objPos[i][0], v_objPos[i][1], v_objPos[i][2]);

		// coordinate change
		objectSE3[i] = demoEnv->Tcamera2robotbase * objSE3_Camera;

		// is z direcition of objectSE3 is upward
		isHead[i] = (objectSE3[i][8] > 0);

		objectGraspCandidatePos[i].resize(v_objCand[i]);
		for (unsigned int j = 0; j < (unsigned int)v_objCand[i]; j++)
			objectGraspCandidatePos[i][j] = Vec3(v_objCandPos[i][0 + 3 * j], v_objCandPos[i][1 + 3 * j], v_objCandPos[i][2 + 3 * j]);
	}

}




bool demoTaskManager::setObjectNum()
{
	int flag;
	SE3 headSE3 = SE3();
	Eigen::VectorXd qval;

	if (curObjectData.objectSE3.size() == 0)
	{
		printf("current object data does not exist!\n");
		return false;
	}

	for (unsigned int i = 0; i < objectNum; i++)
	{
		for (unsigned int j = 0; j < size(curObjectData.objectGraspCandidatePos[i]); j++)
		{

			if (curObjectData.isHead[i])
			{
				if (curObjectData.objectGraspCandidatePos[i][j][2] < 0)
				{
					printf("Cannot reach the underside!\n");
					break;
				}
				headSE3 = EulerZYX(Vec3(0.0, 0.0, SR_PI), curObjectData.objectGraspCandidatePos[i][j]);
			}
			else
				headSE3 = SE3(curObjectData.objectGraspCandidatePos[i][j]);
				
			SE3 targetObject = curObjectData.objectSE3[i] * headSE3;
			qval = rManager->inverseKin(targetObject, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag);
						
			if (flag == 0)
			{
				curObjID = i;

				// Need to be deleted at actual simulation
				rManager->setJointVal(qval);

				printf("object number %d is set!\n", curObjID+1);
				cout << qval.transpose() << endl;
				//curGraspOffset = SE3(curObjectData.objectGraspCandidatePos[i][j]);
				curGraspOffset = headSE3;
				return true;
			}
		}
	}

	printf("none of objects are not reachable!\n");
	return false;
}


void demoTaskManager::setGoalNum(int goalNum)
{
	curGoalID = goalNum;
}

bool demoTaskManager::moveJob(int goalNum)
{
	setGoalNum(goalNum);
	bool reached = reachObject();
	bool grasped = graspObject();
	bool lifted1 = moveWorkspaceDisplacement(Vec3(0.0, 0.0, 0.05));
	bool moved = moveObject();
	return (reached && grasped && lifted1 && moved);
}

bool demoTaskManager::returnJob()
{
	bool released = releaseObject();
	bool returned = goHomepos();
	return (released && returned);
}

bool demoTaskManager::reachObject(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning)
		return goToWaypoint(curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset);
	else
	{
		getCurPosSignal();
		while (1)
		{
			if (isGetPos==true)
			{
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset);
				return goThroughWaypoints(Twaypoints);
			}

		}

	}
}
bool demoTaskManager::graspObject()
{
	bool moved = false;
	moved = goToWaypoint(curObjectData.objectSE3[curObjID] * curGraspOffset);

	// send message to robot to grasp (gripper command ??)
	bool grasped = false;
	////////////////////////////////////////////////////////
	return moved && grasped;
}

bool demoTaskManager::moveObject(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning)
		return goToWaypoint(goalSE3[curGoalID] * goalOffset);
	else
	{
		getCurPosSignal();
		while (1)
		{
			if (isGetPos == true)
			{
				robotrrtManager->attachObject(demoEnv->objects[curObjID], &robot->gMarkerLink[MH12_Index::MLINK_GRIP], Inv(curGraspOffset));
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, goalSE3[curGoalID] * goalOffset * curGraspOffset);
				return goThroughWaypoints(Twaypoints);
			}
		}

	}
}

bool demoTaskManager::releaseObject()
{
	// send message to robot to release (gripper command ??)
	bool released = false;
	////////////////////////////////////////////////////////
	return released;
}

bool demoTaskManager::goHomepos(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning)
		return goToWaypoint(homeSE3);
	else
	{
		getCurPosSignal();
		while (1)
		{
			if (isGetPos == true)
			{
				robotrrtManager->detachObject();
				demoEnv->objects[curObjID]->setBaseLinkFrame(goalSE3[curGoalID]);
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, homeSE3);
				return goThroughWaypoints(Twaypoints);
			}
		}
	}
}

bool demoTaskManager::moveWorkspaceDisplacement(Vec3 disp)
{
	getCurPos();
	return goToWaypoint(SE3(disp) * TcurRobot);
}

vector<SE3> demoTaskManager::planBetweenWaypoints(SE3 Tinit, SE3 Tgoal, unsigned int midNum /* = 1*/)
{
	// planning
	tempObjTraj.resize(0);
	int flag;
	Eigen::VectorXd qInit = rManager->inverseKin(Tinit, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, lastPlanningJointVal);
	cout << flag << endl;
	Eigen::VectorXd qGoal = rManager->inverseKin(Tgoal, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, lastPlanningJointVal);
	cout << flag << endl;
	robotrrtManager->setStartandGoal(qInit, qGoal);
	robotrrtManager->execute(0.1);
	
	if (robotrrtManager->isExecuted())
	{
		// save planning results
		tempTraj = robotrrtManager->extractPath();
		lastPlanningJointVal = tempTraj[tempTraj.size() - 1];
		tempObjTraj.resize(tempTraj.size());
		for (unsigned int i = 0; i < tempTraj.size(); i++)
		{
			robotrrtManager->setState(tempTraj[i]);
			tempObjTraj[i] = demoEnv->objects[curObjID]->getBaseLinkFrame();
		}
		lastObjectSE3 = tempObjTraj[tempObjTraj.size() - 1];

		// save waypoints
		if (tempTraj.size() < midNum + 2)
			midNum = tempTraj.size() - 2;
		vector<SE3> TwaypointSet(midNum + 1);
		int bin = tempTraj.size() / (midNum + 1);
		for (unsigned int i = 0; i < midNum; i++)
			TwaypointSet[i] = rManager->forwardKin(tempTraj[(i + 1)*bin], &robot->gMarkerLink[MH12_Index::MLINK_GRIP]);
		TwaypointSet[midNum] = Tgoal;

		return TwaypointSet;
	}
	else
	{
		vector<SE3> TwaypointSet(1, Tgoal);
		return TwaypointSet;
	}
}

SE3 demoTaskManager::YKpos2SE3(const Eigen::VectorXd YKpos)
{
	return EulerXYZ(Vec3(YKpos[0], YKpos[1], YKpos[2]), Vec3(YKpos[3], YKpos[4], YKpos[5]));
}

std::string to_string_custom(double x)
{
	std::ostringstream ss;
	ss << x;
	return ss.str();
}
bool demoTaskManager::goToWaypoint(SE3 Twaypoint)
{
	getCurPosSignal();
	while (1) 
	{
		if (isGetPos == true)
		{
			// send message to robot (imov command) here
			vector<double> tempOri = SO3ToEulerXYZ((TcurRobot % Twaypoint).GetOrientation());
			Vec3 tempPos = (TcurRobot % Twaypoint).GetPosition();

			MOVE_POS posForSend;
			strcpy(posForSend.Rx, to_string_custom(tempOri[0]).c_str());
			strcpy(posForSend.Ry, to_string_custom(tempOri[1]).c_str());
			strcpy(posForSend.Rz, to_string_custom(tempOri[2]).c_str());
			strcpy(posForSend.X, to_string_custom(tempPos[0]).c_str());
			strcpy(posForSend.Y, to_string_custom(tempPos[1]).c_str());
			strcpy(posForSend.Z, to_string_custom(tempPos[2]).c_str());
			HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");

			COPYDATASTRUCT cds;
			cds.dwData = 1;
			cds.cbData = sizeof(posForSend);
			cds.lpData = &posForSend;
			SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
			std::cout << "isGetPos == true and send goToWaypoint message (goToWaypoint())" << std::endl;
			break;

		}
	}

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
	getCurPosSignal();
	while (1)
	{
		if (isGetPos == true)
		{
			std::cout<< "isGetPos == true at checkWaypointReached() function" << std::endl;
			if (distSE3(Twaypoint, TcurRobot) < posThreshold)
				return true;
			return false;
		}
	}

}

void demoTaskManager::getCurPosSignal()
{
	std::cout << "getCurPosSignal() called" << std::endl;
	// send flag 2
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = 2;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	
	// send message to robot (read cur pos command) here
	//curRobotPos;
	//TcurRobot = YKpos2SE3(curRobotPos);
	////////////////////////////////////////////////////

}

void demoTaskManager::setCurPos(vector<double> values)
{
	std::cout << "setCurPos() called" << std::endl;
	curRobotPos.resize(6);
	for (int i = 0; i < 6; i++)
	{
		curRobotPos(i) = values[i];
	}
	TcurRobot = YKpos2SE3(curRobotPos);
	isGetPos = true;

}



bool demoTaskManager::sendError()
{
	return false;
}
