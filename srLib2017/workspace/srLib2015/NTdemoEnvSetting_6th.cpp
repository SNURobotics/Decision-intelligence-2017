#include "NTdemoEnvSetting_6th.h"

demoEnvironment::demoEnvironment(unsigned int _objectNum) {

	Trobotbase = SE3();		// same as world frame origin
	Trobotbase2link1 = EulerZYX(Vec3(SR_PI, 0.0, 0.0), Vec3(0.0, 0.0, -0.450));		// change to exact value later
																					// from data in e-mail 180424
	Tworld2camera = SE3(-0.5810862069, 0.4871189722, 0.6519615994,
		0.8100935467, 0.4230013330, 0.4059782233,
		-0.0780209307, 0.7640582303, -0.6404121760,
		-0.0080738399, -0.0719967823, 0.7784732222);

	// from data in e-mail 180504
	Trobotbase2camera = SE3(0.977314, 0.073841, -0.198506, -0.027069, -0.886021, -0.462855, -0.210058, 0.457728, -0.863922, 1.095999, -0.310359, 0.925915);
	Tcamera2robotbase = Inv(Trobotbase2camera);

	// set bin
	bin = new Bin(0.00);
	bin2 = new Bin(0.00);
	Plink12bin = Vec3(0.575, -0.015, 0.450 + 0.0075);
	bin->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Plink12bin));	// change to exact value later
	bin2->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Plink12bin + Vec3(0, 0.270 + 0.005, 0)));	// change to exact value later

	// set table
	table = new Table4th(0.00);
	Plink12table = Vec3(0.75, 0.23, 0.3);
	table->setBaseLinkFrame(SE3(Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(0.0, 0.0, 0.0), Plink12table));	// change to exact value later

	// set objects																											// set objects
	objectNum = _objectNum;

	objDefaultPositon.resize(objectNum);
	objDefaultPositon2.resize(objectNum);

	for (unsigned int i = 0; i < objectNum; i++) {
		objDefaultPositon[i] =  SE3(Vec3(-1.0, 0.1 * i, 0.0));
		objDefaultPositon2[i] = SE3(Vec3(-1.2, 0.1 * i, 0.0));
	}
	objects.resize(objectNum);
	objects2.resize(objectNum);
	for (unsigned int i = 0; i < objectNum; i++)
	{
		objects[i] = new workingObject;
		objects[i]->setBaseLinkFrame(objDefaultPositon[i]);
		objects2[i] = new workingObject2;
		objects2[i]->setBaseLinkFrame(objDefaultPositon2[i]);
	}


	// set barrier
	barrier1 = new Barrier1;
	barrier1->setBaseLinkFrame(SE3());
	barrier2 = new Barrier2;
	barrier2->setBaseLinkFrame(SE3());
}

demoEnvironment::~demoEnvironment()
{
	delete bin;
	delete bin2;
	delete table;
	for (unsigned int i = 0; i < objectNum; i++)
		delete objects[i];
	delete barrier1;
	delete barrier2;
}

void demoEnvironment::setObjectFromRobot2ObjectText(string loc, bool print /*= false*/)
{
	std::vector<Eigen::VectorXd> objectloc = loadDataFromText(loc, 6);
	if (objectloc.size() > objectNum)
		printf("object number in the text is larger than preset object number!!!\n");
	for (unsigned int i = 0; i < min(objectloc.size(), objectNum); i++)
	{
		Eigen::VectorXd temp = objectloc[i];
		SE3 tempSE3 = EulerZYX(Vec3(temp(2), temp(1), temp(0)), Vec3(temp(3), temp(4), temp(5)));
		if (print)
			cout << tempSE3 << endl;
		objects[i]->setBaseLinkFrame(Trobotbase * tempSE3);
	}
}


void demoEnvironment::setObjectFromRobot2VisionData(vector<SE3> objectSE3, vector<int> objectID, SE3 binSE3)
{
	if (objectSE3.size() == 0)
	{
		printf("current object data does not exist!\n");
		return;
	}
	unsigned int objectNumData = objectSE3.size();

	for (unsigned int i = 0; i < objectNum; i++)
	{
		if (i < objectNumData) {
			if (abs(objectID[i]) == 2)
				objects[i]->setBaseLinkFrame(Trobotbase * objectSE3[i]);
			else if (abs(objectID[i]) == 1)
				objects2[i]->setBaseLinkFrame(Trobotbase * objectSE3[i]);
			else
				objects[i]->setBaseLinkFrame(Trobotbase * objectSE3[i]);
		}
		else
			objects[i]->setBaseLinkFrame(objDefaultPositon[i]);
	}

	bin->setBaseLinkFrame(Trobotbase * binSE3);

	return;
}

void demoEnvironment::setEnvironmentInSrSpace(srSpace * space)
{
	space->AddSystem(bin);
	space->AddSystem(bin2);
	space->AddSystem(table);
	for (unsigned int i = 0; i < objectNum; i++) {
		space->AddSystem(objects[i]);
		space->AddSystem(objects2[i]);
	}
	space->AddSystem(barrier1);
	space->AddSystem(barrier2);
}


SKKUobjectData::SKKUobjectData()
{
	objectNum = 1;
	objectSE3.resize(objectNum);
	isHead.resize(objectNum);
	objectGraspCandidatePos.resize(objectNum);
	binSE3 = SE3();
}

SKKUobjectData::~SKKUobjectData()
{
	objectSE3.clear();
	isHead.clear();
	objectGraspCandidatePos.clear();
	binSE3 = SE3();
}

void SKKUobjectData::setObjectDataFromString(vector<SE3> _objectSE3, vector<bool> _isHead, vector<vector<Vec3>> _objectGraspCandidatePos, vector<int> _objectID, SE3 _binSE3)
{
	objectSE3.clear();
	isHead.clear();
	objectGraspCandidatePos.clear();
	objectID.clear();
	vector<double> length;
	Vec3 temp;
	vector<unsigned int> order;
	unsigned int temp_int;
	length.resize(_objectSE3.size());
	order.resize(_objectSE3.size());

	//for (unsigned int i = 0; i < _objectSE3.size(); i++)
	//{
	//	//temp = _objectSE3[i].GetPosition();
	//	//temp -= _binSE3.GetPosition();
	//	length[i] = (_objectSE3[i].GetPosition() - _binSE3.GetPosition()).Normalize();
	//	order[i] = i;
	//}

	//for (unsigned int i = 0; i < _objectSE3.size(); i++)
	//{
	//	if (length[i] < length[order[0]])
	//	{
	//		order[i] = order[0];
	//		order[0] = i;
	//	}
	//	if (length[i] > length[order[_objectSE3.size() - 1]])
	//	{
	//		order[i] = order[_objectSE3.size() - 1];
	//		order[_objectSE3.size() - 1] = i;
	//	}
	//}

	//if (_objectSE3.size() > 2)
	//{
	//	for (unsigned int i = 1; i < _objectSE3.size() - 1; i++)
	//	{
	//		if (length[order[i]] < length[order[1]])
	//		{
	//			temp_int = order[i];
	//			order[i] = order[1];
	//			order[1] = temp_int;
	//		}
	//		if (length[order[i]] > length[order[_objectSE3.size() - 2]])
	//		{
	//			temp_int = order[i];
	//			order[i] = order[_objectSE3.size() - 2];
	//			order[_objectSE3.size() - 2] = temp_int;
	//		}
	//	}
	//}

	////for (unsigned int i = 0; i < _objectSE3.size(); i++)
	////{
	////	cout << order[i] << endl;
	////	cout << length[i] << endl;
	////}

	//for (unsigned int i = 0; i < _objectSE3.size(); i++)
	//{
	//	objectSE3.push_back(_objectSE3[order[i]]);
	//	isHead.push_back(_isHead[order[i]]);
	//	objectGraspCandidatePos.push_back(_objectGraspCandidatePos[order[i]]);
	//}

	objectSE3 = _objectSE3;
	isHead = _isHead;
	objectGraspCandidatePos = _objectGraspCandidatePos;
	objectID = _objectID;

	binSE3 = _binSE3;
}


demoTaskManager::demoTaskManager(demoEnvironment* _demoEnv, MH12RobotManager* _rManager, bool taskType)
{
	demoEnv = _demoEnv;
	rManager = _rManager;
	robot = (MH12Robot*)rManager->m_robot;
	robotrrtManager = NULL;
	which_task = taskType;

	// constants for task (should be modified later!!!)
	goalSE3.resize(2);
	//goalSE3[0] = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.226026, 0.888197, 0.466843));
	//goalSE3[0] = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), Vec3(0.226026, 0.588197, 0.466843));

	// goal setting (18.05.25.)
	double temp_r = 0.04 - 0.017*0.5 / sqrt(3.0);
	double temp_r2 = 0.001;
	SE3 temp_robot2objhead = EulerZYX(Vec3(-SR_PI / 6.0, 0.0, SR_PI), Vec3(temp_r * cos(SR_PI / 6.0), -temp_r * sin(SR_PI / 6.0), 0.0));
	SE3 temp_robot2objtail = EulerZYX(Vec3(-SR_PI / 6.0, 0.0, 0.0), Vec3(temp_r * cos(SR_PI / 6.0) + temp_r2 * sin(SR_PI / 6.0), -temp_r * sin(SR_PI / 6.0) + temp_r2 * cos(SR_PI / 6.0), 0.004));
	// Transfering Task
	if (taskType == 0) {
		// 0: Head 1: Tail
		goalSE3[0] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.226048, 0.871385, 0.466791)) * temp_robot2objhead; // head case
		goalSE3[1] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.226048, 0.871385, 0.466791)) * temp_robot2objtail; // tail case
	}
	// Alligning Task
	else {
		goalSE3.resize(18);
		goalSE3[0] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.1, 0.08)) * temp_robot2objhead;
		goalSE3[1] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.1, 0.08)) * temp_robot2objtail;
		goalSE3[2] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.05, 0.08)) * temp_robot2objhead;
		goalSE3[3] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.05, 0.08)) * temp_robot2objtail;
		goalSE3[4] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.0, 0.08)) * temp_robot2objhead;
		goalSE3[5] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, -0.0, 0.08)) * temp_robot2objtail;
		goalSE3[6] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.05, 0.08)) * temp_robot2objhead;
		goalSE3[7] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.05, 0.08)) * temp_robot2objtail;
		goalSE3[8] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.1, 0.08)) * temp_robot2objhead;
		goalSE3[9] =  EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.1, 0.08)) * temp_robot2objtail;
		goalSE3[10] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.15, 0.08)) * temp_robot2objhead;
		goalSE3[11] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.15, 0.08)) * temp_robot2objtail;
		goalSE3[12] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.2, 0.08)) * temp_robot2objhead;
		goalSE3[13] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.2, 0.08)) * temp_robot2objtail;
		goalSE3[14] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.25, 0.08)) * temp_robot2objhead;
		goalSE3[15] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.25, 0.08)) * temp_robot2objtail;
		goalSE3[16] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.3, 0.08)) * temp_robot2objhead;
		goalSE3[17] = EulerZYX(Vec3(DEG2RAD(58.4843), DEG2RAD(-0.1395), DEG2RAD(179.8125)), Vec3(0.97, 0.3, 0.08)) * temp_robot2objtail;
	}
	//homeSE3 = EulerZYX(Vec3(SR_PI, -SR_PI_HALF, 0.0), Vec3(1.029, 0.0, 0.814));		// where robot goes when job is done (should be modified)
	homeSE3 = EulerZYX(Vec3(DEG2RAD(-0.8872), DEG2RAD(2.0460), DEG2RAD(179.0799)), Vec3(0.416704, 0.093653, 0.509468));
	homeSE3.SetOrientation(homeSE3.GetOrientation() * Exp(Vec3(0.0, 0.0, DEG2RAD(-130.8872))));
	reachOffset = SE3(Vec3(0.0, 0.0, -0.03));
	goalOffset = SE3(Vec3(0.0, 0.0, 0.0));
	TcurRobot = homeSE3;

	lastPlanningJointVal.setZero(6);
	lastPlanningJointVal[0] = DEG2RAD(0.0);
	lastPlanningJointVal[1] = DEG2RAD(0.0);
	lastPlanningJointVal[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
	lastPlanningJointVal[3] = DEG2RAD(0.0);
	lastPlanningJointVal[4] = DEG2RAD(-90.0);
	lastPlanningJointVal[5] = DEG2RAD(0.0);

	goal_iterator = 0;
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
	vector<int> objectID;
	vector<bool> isHead;
	vector<vector<Vec3>> objectGraspCandidatePos;
	SE3 binSE3;

	readSKKUvision(stringfromSKKU, objectSE3, isHead, objectGraspCandidatePos, objectID, binSE3);

	curObjectData.setObjectDataFromString(objectSE3, isHead, objectGraspCandidatePos, objectID, binSE3);

	demoEnv->setObjectFromRobot2VisionData(curObjectData.objectSE3, curObjectData.objectID, curObjectData.binSE3);

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

void demoTaskManager::readSKKUvision(char* hyu_data, vector<SE3>& objectSE3, vector<bool>& isHead, vector<vector<Vec3>>& objectGraspCandidatePos, vector<int>& objectID, SE3& binSE3)
{
	size_t pos = std::string::npos;
	string str_data = string(hyu_data);
	string str_toDel = "(ind)";

	// Search for the undesired substring & delete
	while ((pos = str_data.find(str_toDel)) != std::string::npos)
		str_data.erase(pos, str_toDel.length());
	strcpy(hyu_data, str_data.c_str());

	// read vision data
	Eliminate(hyu_data, 'V');
	cout << hyu_data << endl;

	// temporary vector variables
	vector<int>				v_objID;
	vector<vector<double>>	v_objPos;					//id, (x, y, z)
	vector<vector<double>>	v_objOri;					//id, (R_x, R_y, R_z)
	vector<vector<int>>		v_objFtr;					//id, (min_x, max_x, min_y, max_y)

	// read char data and save it to vector double format
	char *recv_data = strtok(hyu_data, "d");
	int recv_cnt = 0;
	v_objID.resize(0);
	v_objPos.resize(0);
	v_objOri.resize(0);
	v_objFtr.resize(0);
	int objIdx = 0;
	int objID;
	int max_recv_cnt = 20;
	bool isDummy = false;
	bool isNaN = false;

	while (recv_data != NULL)
	{
		if (recv_cnt < 1)
		{
			if (atof(recv_data) == -10) isNaN = true;
			objID = atoi(recv_data);
			if (abs(objID) == 3) // Dummy Data
				isDummy = true;
			else if (isNaN) {}
			else
			{
				cout << "ID: " << recv_data << endl;
				v_objID.push_back(objID); // 1: FixedContactCover 2: FixedContact
				v_objPos.resize(objIdx + 1);
				v_objOri.resize(objIdx + 1);
				v_objFtr.resize(objIdx + 1);
				v_objFtr[objIdx] = vector<int>{ 0, 0, 0, 0, 0, 0, 0 };
			}
		}

		if (!isDummy && !isNaN)
		{
			if (1 <= recv_cnt && recv_cnt < 10)
			{
				v_objOri[objIdx].push_back(atof(recv_data));
				cout << "Orientation: " << recv_data << endl;
			}
			else if (10 <= recv_cnt && recv_cnt < 13)
			{
				v_objPos[objIdx].push_back(atof(recv_data));
				cout << "Position: " << recv_data << endl;
			}
			else if (13 <= recv_cnt && recv_cnt < 20)
			{
				if (atof(recv_data) != -10)
				{
					v_objFtr[objIdx][atoi(recv_data)] = v_objFtr[objIdx][atoi(recv_data)] + 1;
					cout << "Feature: " << recv_data << endl;
				}
			}
		}

		recv_data = strtok(NULL, "d"); // Update current word
		recv_cnt += 1;

		if (recv_cnt == max_recv_cnt)
		{
			if (!isDummy && !isNaN) {
				objIdx++;
			}
			recv_cnt = 0;
			isDummy = false;
			isNaN = false;
		}
	}
	cout << objIdx << endl;
	objectNumData = objIdx;

	objectSE3.resize(objectNumData);
	isHead.resize(objectNumData);
	objectID.resize(objectNumData);
	objectGraspCandidatePos.resize(objectNumData);


	// transform vector data into Vec3 and SE3 format
	for (unsigned int i = 0; i < objectNumData; i++)
	{
		SE3 objSE3_Camera = SE3(v_objOri[i][0], v_objOri[i][3], v_objOri[i][6], v_objOri[i][1], v_objOri[i][4], v_objOri[i][7], v_objOri[i][2], v_objOri[i][5], v_objOri[i][8], v_objPos[i][0], v_objPos[i][1], v_objPos[i][2]);

		// coordinate change
		//objectSE3[i] = demoEnv->Trobotbase2camera * objSE3_Camera;
		// coordinate change is not needed

		// 2019-9-25 Coordinate change
		if (abs(v_objID[i]) == 2)
		{
			objectSE3[i] = objSE3_Camera;
			objectSE3[i] = objectSE3[i] * Inv(EulerZYX(Vec3(SR_PI_HALF, 0, SR_PI_HALF), Vec3(0, 0, 0)))*EulerZYX(Vec3(SR_PI, 0, 0), Vec3(0, 0, 0));
		}
		else {
			objectSE3[i] = objSE3_Camera;
		}
		objectID[i] = v_objID[i];

		// is z direcition of objectSE3 is upward
		cout << objectSE3[i][8] << endl;
		if (abs(v_objID[i]) == 2) {
			isHead[i] = (objectSE3[i][8] > 0) || (abs(v_objID[i]) / v_objID[i] == 1);
		}
		else if (abs(v_objID[i]) == 1) {
			isHead[i] = (objectSE3[i][4] > 0) || (abs(v_objID[i]) / v_objID[i] == 1);
		}
		cout << "Object No [" << i  << "] is in ";
		if (isHead[i]) cout << "Head" << endl;
		else cout << "Tail" << endl;

		if (isHead[i])
		{
			if (abs(v_objID[i]) == 2) // FixedContactCover
			{
				if (v_objFtr[i][0] && v_objFtr[i][1] && v_objFtr[i][2])
				{
					objectGraspCandidatePos[i].resize(1);
					objectGraspCandidatePos[i][0] = Vec3(-0.02, 0.0, 0.0);
					cout << "Object " << i << ", Head Candidate Pos 1" << endl;
				}
			}
			else if (abs(v_objID[i]) == 1) // FixedContact
			{
				if (v_objFtr[i][0] && v_objFtr[i][1] && v_objFtr[i][2])
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.032, 0.032, -0.070);
					cout << "Object " << i << ", Head Candidate Pos 1" << endl;
				}
				else if (v_objFtr[i][1] && v_objFtr[i][2] && (v_objFtr[i][3] || v_objFtr[i][5]))
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.017, 0.032, -0.036);
					cout << "Object " << i << ", Head Candidate Pos 2" << endl;
				}
				else if (v_objFtr[i][1] && v_objFtr[i][2] && (v_objFtr[i][4] || v_objFtr[i][6]))
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.047, 0.032, -0.036);
					cout << "Object " << i << ", Head Candidate Pos 3" << endl;
				}
			}
			else
			{
				objectGraspCandidatePos[i].resize(1);
				objectGraspCandidatePos[i][0] = Vec3(-0.02, 0.0, 0.0);
			}
		}
		else // isTail[i]
		{
			if (abs(v_objID[i]) == 2) // FixedContactCover
			{
				if (v_objFtr[i][0] && v_objFtr[i][1] && v_objFtr[i][2])
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(-0.02, 0.0, -0.004);
					cout << "Object " << i << ", Tail Candidate Pos 1" << endl;
				}
				if (v_objFtr[i][0] && v_objFtr[i][2] && v_objFtr[i][4])
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.0, 0.02, -0.004);
					cout << "Object " << i << ", Tail Candidate Pos 2" << endl;
				}
				if (v_objFtr[i][0] && v_objFtr[i][1] && v_objFtr[i][3])
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.0, -0.02, -0.004);
					cout << "Object " << i << ", Tail Candidate Pos 3" << endl;
				}
			}
			if (abs(v_objID[i]) == 1) // FixedContact
			{
				if (v_objFtr[i][0] && v_objFtr[i][1] && v_objFtr[i][2])
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.032, 0.030, -0.075);
					cout << "Object " << i << ", Tail Candidate Pos 1" << endl;
				}
				if ((v_objFtr[i][1] || v_objFtr[i][3]) && (v_objFtr[i][2] || v_objFtr[i][4]))
				{
					objectGraspCandidatePos[i].resize(objectGraspCandidatePos[i].size() + 1);
					objectGraspCandidatePos[i][objectGraspCandidatePos[i].size() - 1] = Vec3(0.032, 0.0, -0.04);
					cout << "Object " << i << ", Tail Candidate Pos 2" << endl;
				}
			}
			else
			{
				objectGraspCandidatePos[i].resize(1);
				objectGraspCandidatePos[i][0] = Vec3(-0.02, 0.0, -0.004);
			}
		}

	}

	binSE3 = SE3(demoEnv->Trobotbase2link1.GetPosition()) * EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.0), demoEnv->Plink12bin);
}


bool demoTaskManager::setObjectNumManually(int objNum)
{
	if (curObjectData.objectSE3.size() < objNum)
		return false;
	SE3 headSE3;
	for (unsigned int j = 0; j < size(curObjectData.objectGraspCandidatePos[objNum]); j++)
	{

		if (curObjectData.isHead[objNum])
		{
			if (curObjectData.objectGraspCandidatePos[objNum][j][2] < 0)
			{
				//printf("Cannot reach the underside!\n");
				break;
			}
			headSE3 = EulerZYX(Vec3(0.0, 0.0, SR_PI), curObjectData.objectGraspCandidatePos[objNum][j]);
			//cout << headSE3 << endl;
		}
		else
			headSE3 = SE3(curObjectData.objectGraspCandidatePos[objNum][j]);
	}
	SE3 targetObject = curObjectData.objectSE3[objNum] * headSE3;
	double z_angle_for_zeroq6 = Log(Inv(targetObject.GetOrientation()) * homeSE3.GetOrientation())[2];
	// set goal num (0: head, 1: tail)
	if (curObjectData.isHead[objNum])
	{
		setGoalNum(0);
		double q6_rot = Log(Inv(Exp(Vec3(0.0, 0.0, SR_PI_HALF)) * curObjectData.objectSE3[objNum].GetOrientation()) * goalSE3[0].GetOrientation())[2];
		curGraspOffset = headSE3 * EulerZYX(Vec3(z_angle_for_zeroq6 + 0.5*q6_rot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	}
	else
	{
		setGoalNum(1);
		double q6_rot = Log(Inv(Exp(Vec3(0.0, 0.0, SR_PI_HALF)) * curObjectData.objectSE3[objNum].GetOrientation()) * goalSE3[1].GetOrientation())[2];
		curGraspOffset = headSE3 * EulerZYX(Vec3(z_angle_for_zeroq6 - 0.5*q6_rot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
	}
	return true;
}

bool demoTaskManager::setObjectNum()
{
	int flag;
	bool collision;
	SE3 headSE3 = SE3();
	Eigen::VectorXd qval;

	if (curObjectData.objectSE3.size() == 0)
	{
		printf("current object data does not exist!\n");
		return false;
	}

	for (unsigned int i = 0; i < objectNumData; i++)
	{
		for (unsigned int j = 0; j < size(curObjectData.objectGraspCandidatePos[i]); j++)
		{

			if (curObjectData.isHead[i])
			{
				if (abs(curObjectData.objectID[i]) == 2) {
					if (curObjectData.objectGraspCandidatePos[i][j][2] < 0)
					{
						printf("Cannot reach the underside!\n");
						break;
					}
					headSE3 = EulerZYX(Vec3(0.0, 0.0, SR_PI), curObjectData.objectGraspCandidatePos[i][j]);
					//cout << headSE3 << endl;
				}
				else if (abs(curObjectData.objectID[i]) == 1) {
					if (curObjectData.objectGraspCandidatePos[i][j][1] < 0)
					{
						printf("Cannot reach the underside!\n");
						break;
					}
					headSE3 = EulerZYX(Vec3(SR_PI_HALF, -SR_PI_HALF, 0.0), curObjectData.objectGraspCandidatePos[i][j]);
					//cout << headSE3 << endl;
				}
			}
			else {
				if (abs(curObjectData.objectID[i]) == 2) {
					headSE3 = SE3(curObjectData.objectGraspCandidatePos[i][j]);
				}
				else if (abs(curObjectData.objectID[i]) == 1) {
					headSE3 = EulerZYX(Vec3(-SR_PI_HALF, -SR_PI_HALF, 0.0), curObjectData.objectGraspCandidatePos[i][j]);
				}
			}

			SE3 targetObject = curObjectData.objectSE3[i] * headSE3;

			// calculate z_angle for graspOffset not to exceed joint limit
			double z_angle_for_zeroq6 = Log(Inv(targetObject.GetOrientation()) * homeSE3.GetOrientation())[2];
			SE3 testPos = targetObject;
			testPos.SetOrientation(testPos.GetOrientation() * Exp(Vec3(0.0, 0.0, z_angle_for_zeroq6)));
			// 2019-9-26 Resolve collision error
			testPos = EulerZYX(Vec3(0, 0, 0), Vec3(0, 0, 0.03)) * testPos;
			qval = rManager->inverseKin(testPos, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, lastPlanningJointVal);

			if (flag == 0)
			{
				curObjID = i;

				// Need to be deleted at actual simulation
				rManager->setJointVal(qval);

				collision = rManager->checkCollision();

				if (!collision)
				{
					printf("object number %d is set as goal\n", curObjID + 1);
					printf("candidate pos: %d\n", j + 1);
					//cout << qval.transpose() << endl;
					//curGraspOffset = SE3(curObjectData.objectGraspCandidatePos[i][j]);
					printf("curObject SE3:\n");
					cout << curObjectData.objectSE3[i] << endl;

					// set goal num (0: head, 1: tail)
					if (curObjectData.isHead[i])
					{
						setGoalNum(0);
						double q6_rot = Log(Inv(Exp(Vec3(0.0, 0.0, SR_PI_HALF)) * curObjectData.objectSE3[i].GetOrientation()) * goalSE3[0].GetOrientation())[2];
						curGraspOffset = headSE3 * EulerZYX(Vec3(z_angle_for_zeroq6 + 0.5*q6_rot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
					}
					else
					{
						setGoalNum(1);
						double q6_rot = Log(Inv(Exp(Vec3(0.0, 0.0, SR_PI_HALF)) * curObjectData.objectSE3[i].GetOrientation()) * goalSE3[1].GetOrientation())[2];
						curGraspOffset = headSE3 * EulerZYX(Vec3(z_angle_for_zeroq6 - 0.5*q6_rot, 0.0, 0.0), Vec3(0.0, 0.0, 0.0));
					}
					return true;
				}
			}
		}
	}

	rManager->setJointVal(lastPlanningJointVal);

	printf("none of objects are reachable!\n");
	return false;
}


void demoTaskManager::setGoalNum(int goalNum)
{
	curGoalID = goalNum;
}

int demoTaskManager::getGoalNum()
{
	return curGoalID;
}

bool demoTaskManager::moveJob()
{
	bool reached = false;
	bool reachedGraspPos = false;
	bool grasped = false;
	bool lifted1 = false;
	bool moved = false;
	while (!reached)
		reached = reachObject();
	while (!reachedGraspPos)
		reachedGraspPos = goToGraspPos();
	while (!grasped)
		grasped = graspObject();
	while (!lifted1)
		lifted1 = moveWorkspaceDisplacement(Vec3(0.0, 0.0, 0.15));
	while (!moved)
		moved = moveObject();
	//return (reached && reachedGraspPos && grasped && lifted1 && moved);
	return true;
}

bool demoTaskManager::returnJob()
{
	bool released = false;
	while (!released)
		released = releaseObject();
	bool returned = false;
	while (!returned)
		returned = goHomepos();
	//return (released && returned);
	return true;
}

bool demoTaskManager::reachObject(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning)
		return goToWaypoint(curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset);
	else
	{
		getCurPosSignal();
		std::clock_t start = std::clock();
		while (1)
		{
			if (isGetPos == true)
			{
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, curObjectData.objectSE3[curObjID] * curGraspOffset * reachOffset);
				return goThroughWaypoints(Twaypoints);
			}
			else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
			{
				printf("isGetPos == false and setCurPos failed at reachObject!!!\n");
				return false;
			}
		}
	}
}
bool demoTaskManager::goToGraspPos()
{
	// 2019-10-02 custom z resolve value for FixedContact object
	// 2019-9-26 Resolve z error
	if (abs(curObjectData.objectID[curObjID]) == 2) {
		SE3 temp = curObjectData.objectSE3[curObjID] * curGraspOffset * SE3(Vec3(0.0, 0.0, 0.055));
		if (temp.GetPosition()[2] < 0.015) {
			temp = curObjectData.objectSE3[curObjID] * curGraspOffset * SE3(Vec3(0.0, 0.0, 0.040));
		}
		return goToWaypoint(temp);
	}
	else {
		SE3 temp = curObjectData.objectSE3[curObjID] * curGraspOffset * SE3(Vec3(0.0, 0.0, 0.035));
		if (temp.GetPosition()[2] < 0.015) {
			temp = curObjectData.objectSE3[curObjID] * curGraspOffset * SE3(Vec3(0.0, 0.0, 0.020));
		}
		return goToWaypoint(temp);
	}
}
bool demoTaskManager::graspObject()
{
	// send message to robot to grasp (gripper command ??)
	bool grasped = false;
	LRESULT success = gripperOnSignal();
	Sleep(GRASP_WAIT_TIME);
	if (success == 0)
		return false;
	else
		return true;
}

bool demoTaskManager::moveObject(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning) {
		if (which_task == 0) {
			if (abs(curObjectData.objectID[curObjID]) == 1) { // FixedContact case
				SE3 goal = SE3(Vec3(0.0, 0.0, 0.005)) * goalSE3[curGoalID] * goalOffset * curGraspOffset;
				goal.SetOrientation(EulerZYX(Vec3(0, SR_PI_HALF, 0), Vec3(0, 0, 0)).GetOrientation() * goal.GetOrientation());
				return goToWaypoint(goal);
			}
			else // FixedContactCover case
				return goToWaypoint(SE3(Vec3(0.0, 0.0, 0.005)) * goalSE3[curGoalID] * goalOffset * curGraspOffset);
		}
		else {
			SE3 goal;
			if (abs(curObjectData.objectID[curObjID]) == 1) { // FixedContact case
				goal = SE3(Vec3(0.0, 0.0, 0.005)) * goalSE3[goal_iterator * 2 + curGoalID] * goalOffset * curGraspOffset;
				goal.SetOrientation(EulerZYX(Vec3(0, SR_PI_HALF, 0), Vec3(0, 0, 0)).GetOrientation() * goal.GetOrientation());
			}
			else // FixedContactCover case
				goal = SE3(Vec3(0.0, 0.0, 0.005)) * goalSE3[goal_iterator * 2 + curGoalID] * goalOffset * curGraspOffset;

			if (goal_iterator * 2 + 1 == goalSE3.size()) {
				goal_iterator = 0;
				return goToWaypoint(goal);
			}
			else {
				goal_iterator++;
				return goToWaypoint(goal);
			}
		}
	}
	else
	{
		getCurPosSignal();
		std::clock_t start = std::clock();
		while (1)
		{
			if (isGetPos == true)
			{
				robotrrtManager->attachObject(demoEnv->objects[curObjID], &robot->gMarkerLink[MH12_Index::MLINK_GRIP], Inv(curGraspOffset));
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, goalSE3[curGoalID] * goalOffset * curGraspOffset);
				return goThroughWaypoints(Twaypoints);
			}
			else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
			{
				printf("isGetPos == false and setCurPos failed at moveObject!!!\n");
				return false;
			}
		}

	}
}

bool demoTaskManager::releaseObject()
{
	// send message to robot to release (gripper command ??)
	bool released = false;
	LRESULT success = gripperOffSignal();
	Sleep(GRASP_WAIT_TIME);
	if (success == 0)
		return false;
	else
		return true;
}

bool demoTaskManager::goHomepos(bool usePlanning /*= false*/)
{
	if (robotrrtManager == NULL || !usePlanning)
		return goToWaypoint(homeSE3);
	else
	{
		getCurPosSignal();
		std::clock_t start = std::clock();
		while (1)
		{
			if (isGetPos == true)
			{
				robotrrtManager->detachObject();
				demoEnv->objects[curObjID]->setBaseLinkFrame(goalSE3[curGoalID]);
				vector<SE3> Twaypoints = planBetweenWaypoints(TcurRobot, homeSE3);
				return goThroughWaypoints(Twaypoints);
			}
			else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
			{
				printf("isGetPos == false and setCurPos failed at goHomepos!!!\n");
				return false;
			}
		}
	}
}

bool demoTaskManager::moveWorkspaceDisplacement(Vec3 disp)
{
	getCurPosSignal();
	std::clock_t start = std::clock();
	while (1)
	{
		if (isGetPos == true)
			return goToWaypoint(SE3(disp) * TcurRobot);
		else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
		{
			printf("isGetPos == false and setCurPos failed at moveWorkspaceDisplacement!!!\n");
			return false;
		}
	}
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

void demoTaskManager::printImovCommand(SE3 Tstart, SE3 Tgoal)
{
	vector<double> tempOri = SO3ToEulerZYX((Tgoal.GetOrientation() * Inv(Tstart.GetOrientation())));
	Vec3 tempPos = Tgoal.GetPosition() - Tstart.GetPosition();

	printf("imov command x: %f, y: %f, z: %f, Rx: %f, Ry: %f, Rz: %f\n", tempPos[0] * 1000.0, tempPos[1] * 1000.0, tempPos[2] * 1000.0, tempOri[2] * (180.0 / SR_PI), tempOri[1] * (180.0 / SR_PI), tempOri[0] * (180.0 / SR_PI));
}

LRESULT demoTaskManager::startConnection()
{
	// send flag 0
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = CONNECTION_START_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 10000, temp);
	return success;
}

LRESULT demoTaskManager::endConnection()
{
	// send flag 5
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = CONNECTION_END_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 10000, temp);
	return success;
}

SE3 demoTaskManager::YKpos2SE3(const Eigen::VectorXd YKpos)
{
	return EulerZYX(Vec3(YKpos[2], YKpos[1], YKpos[0]), Vec3(YKpos[3], YKpos[4], YKpos[5]));
}

bool demoTaskManager::checkWaypoint(SE3 Tinit, SE3 Tgoal, int num /*= 10*/)
{
	vector<Vec3> tempVec(num);
	vector<SO3> tempOri(num);
	Vec3 axis = Log(Inv(Tinit.GetOrientation()) * Tgoal.GetOrientation());
	bool okay = true;
	for (int i = 0; i < num; i++)
	{
		tempOri[i] = Tinit.GetOrientation() * Exp(axis * (double)i / (num - 1));
		tempVec[i] = Tinit.GetPosition() * (double)(num - 1 - i) / (num - 1) + Tgoal.GetPosition() * (double)i / (num - 1);

		SE3 Ttemp(tempOri[i], tempVec[i]);
		int flag;
		Eigen::VectorXd qval;
		qval.setZero(6);
		qval[0] = DEG2RAD(0.0);
		qval[1] = DEG2RAD(0.0);
		qval[2] = DEG2RAD(0.0);		// joint 3 15deg error?? robot -15deg 일때랑 여기 0deg일때랑 비슷
		qval[3] = DEG2RAD(0.0);
		qval[4] = DEG2RAD(-90.0);
		qval[5] = DEG2RAD(0.0);
		Eigen::VectorXd q = rManager->inverseKin(Ttemp, &robot->gMarkerLink[MH12_Index::MLINK_GRIP], true, SE3(), flag, qval);
		okay &= (flag == 0);
	}
	return okay;
}

std::string to_string_custom(double x)
{
	std::ostringstream ss;
	ss << x;
	return ss.str();
}
bool demoTaskManager::goToWaypoint(SE3 Twaypoint)
{
	printf("============ go to waypoint ============\n");
	getCurPosSignal();
	Sleep(500);

	MOVE_POS posForSend;
	//vector<double> tempOri = SO3ToEulerZYX(Twaypoint.GetOrientation());
	//Vec3 tempPos = Twaypoint.GetPosition();
	//printf("x: %f, y: %f, z: %f, Rx: %f, Ry: %f, Rz: %f\n", tempPos[0], tempPos[1], tempPos[2], tempOri[2], tempOri[1], tempOri[0]);
	//
	std::clock_t start = std::clock();
	while (1)
	{
		if (isGetPos == true)
		{
			//cout << Twaypoint * Inv(TcurRobot) << endl;
			// send message to robot (imov command) here
			vector<double> tempOri = SO3ToEulerZYX((Twaypoint.GetOrientation() * Inv(TcurRobot.GetOrientation())));
			//std::cout << Inv(TcurRobot)* Twaypoint << std::endl;
			Vec3 tempPos = Twaypoint.GetPosition() - TcurRobot.GetPosition();


			// convert rad -> deg, m -> mm
			posForSend.Rx = tempOri[2] * (180.0 / SR_PI);
			posForSend.Ry = tempOri[1] * (180.0 / SR_PI);
			posForSend.Rz = tempOri[0] * (180.0 / SR_PI);
			posForSend.X = tempPos[0] * 1000.0;
			posForSend.Y = tempPos[1] * 1000.0;
			posForSend.Z = tempPos[2] * 1000.0;
			HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");

			COPYDATASTRUCT cds;
			cds.dwData = MOVE_SIGNAL;
			cds.cbData = sizeof(posForSend);
			cds.lpData = &posForSend;
			//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
			PDWORD_PTR temp = NULL;
			int sleepTime;
			int IMOV_SPEED = 80;
			sleepTime = max(sqrt(posForSend.X * posForSend.X + posForSend.Y * posForSend.Y + posForSend.Z * posForSend.Z), sqrt(posForSend.Rx * posForSend.Rx + posForSend.Ry * posForSend.Ry + posForSend.Rz * posForSend.Rz)) / IMOV_SPEED;
			LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 1000 * (5 + sleepTime), temp);
			printf("isGetPos == true and send goToWaypoint message (goToWaypoint())\n");
			cout << success << endl;
			printf("imov command x: %f, y: %f, z: %f, Rx: %f, Ry: %f, Rz: %f\n", posForSend.X, posForSend.Y, posForSend.Z, posForSend.Rx, posForSend.Ry, posForSend.Rz);
			break;
		}
		else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
		{
			printf("isGetPos == false and setCurPos failed (goToWaypoint())!!!\n");
			return false;
		}
	}

	/////////////////////////////////////////////
	//int cnt = 0;
	//while (cnt < maxTimeDuration)
	//{
	//	// check if robot has reached its waypoint for every 50 ms
	//	Sleep(50);
	//	cnt += 50;
	//	if (checkWaypointReached(Twaypoint))
	//		return true;
	//}
	//return true;
	bool reached = checkWaypointReached(Twaypoint);
	printf("========================================\n");
	return reached;
}

bool demoTaskManager::goThroughWaypoints(vector<SE3> Twaypoints)
{
	bool success = true;
	bool temp_success = true;
	for (unsigned int i = 0; i < Twaypoints.size(); i++)
	{
		bool temp_success = goToWaypoint(Twaypoints[i]);
		while (!temp_success)
			temp_success = goToWaypoint(Twaypoints[i]);
		//success &= goToWaypoint(Twaypoints[i]);
	}
	return success;
}

bool demoTaskManager::checkWaypointReached(SE3 Twaypoint)
{
	getCurPosSignal();
	std::clock_t start = std::clock();
	while (1)
	{
		if (isGetPos == true)
		{
			std::cout << "isGetPos == true at checkWaypointReached() function" << std::endl;
			double dist = distSE3(Twaypoint, TcurRobot);
			if (dist < POSITION_THRESHOLD)
				return true;
			printf("position error: %f\n", dist);
			return false;
		}
		else if ((std::clock() - start) / (double)CLOCKS_PER_SEC > MAX_TIME_DURATION)
		{
			printf("isGetPos == false and setCurPos failed at checkWaypointReached() function!!!\n");
			return false;
		}
	}

}

LRESULT demoTaskManager::getCurPosSignal()
{
	std::cout << "getCurPosSignal() called" << std::endl;
	// send flag 2
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = GET_CURPOS_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 1000, temp);
	while (success == 0)
		success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 1000, temp);
	return success;
	// send message to robot (read cur pos command) here
	//curRobotPos;
	//TcurRobot = YKpos2SE3(curRobotPos);
	////////////////////////////////////////////////////

}

LRESULT demoTaskManager::getCurJointSignal()
{
	std::cout << "getCurJointSignal() called" << std::endl;
	// send flag 2
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = GET_CURJOINT_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 1000, temp);
	while (success == 0)
		success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 1000, temp);
	return success;
	// send message to robot (read cur pos command) here
	//curRobotPos;
	//TcurRobot = YKpos2SE3(curRobotPos);
	////////////////////////////////////////////////////
}

void demoTaskManager::setCurPos(vector<double> values)
{
	std::cout << "setCurPos() called" << std::endl;
	Eigen::VectorXd curRobotPos;	// current robot pos (Rx, Ry, Rz, px, py, pz)
	curRobotPos.resize(6);
	for (int i = 0; i < 6; i++)
	{
		curRobotPos(i) = values[i];
	}
	TcurRobot = YKpos2SE3(curRobotPos);
	isGetPos = true;

}

SE3 demoTaskManager::getTcurRobot()
{
	return TcurRobot;
}

void demoTaskManager::setCurJoint(vector<double> values)
{
	std::cout << "setCurJoint() called" << std::endl;
	Eigen::VectorXd curRobotJoint;
	curRobotJoint.resize(6);
	for (int i = 0; i < 6; i++)
	{
		curRobotJoint(i) = values[i];
	}
	TcurRobot = rManager->forwardKin(curRobotJoint, &robot->gMarkerLink[MH12_Index::MLINK_GRIP]);
	isGetPos = true;
}

LRESULT demoTaskManager::gripperOnSignal()
{
	// send flag 2
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = GRIPPER_ON_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 10000, temp);
	return success;
}

LRESULT demoTaskManager::gripperOffSignal()
{
	// send flag 3
	char dummyMsg[256] = "dummy message";
	isGetPos = false;
	HWND hTargetWnd = FindWindow(NULL, L"ESF_Client_Example_JOB_IMOV");
	COPYDATASTRUCT cds;
	cds.dwData = GRIPPER_OFF_SIGNAL;
	cds.cbData = sizeof(dummyMsg);
	cds.lpData = dummyMsg;
	//SendMessage(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds));
	PDWORD_PTR temp = NULL;
	LRESULT success = SendMessageTimeout(hTargetWnd, WM_COPYDATA, NULL, reinterpret_cast<LPARAM>(&cds), SMTO_NORMAL, 10000, temp);
	return success;
}



bool demoTaskManager::sendError()
{
	return false;
}

void demoTaskManager::setWhichTask(bool taskType)
{
	which_task = taskType;
}
