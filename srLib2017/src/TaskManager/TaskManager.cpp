//#include "SDA20DTaskManager.h"
//#include "common\dataIO.h"
//#include <ctime>
//void SDA20DTaskManager::loadTaskData(string str)
//{
//	// load object and human motion data from dataset folder
//	int numTask = 0;		// number of folders
//	boost::filesystem::path p(str);
//	
//	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
//	{
//		numTask++;
//	}
//
//	_dualArmTaskSet.resize(numTask);
//	_numRightObject.resize(numTask);
//	_numLeftObject.resize(numTask);
//	_objectInitPos.resize(numTask);
//
//	// load task info
//	string innerPath;
//	for (int j = 0; j < numTask; j++)
//	{
//		innerPath = str + "task" + to_string(j+1) + "/objectInfo.txt";
//		_objectInitPos[j].resize(NUM_OF_OBJECTS);
//		loadTaskInfo(innerPath, j);
//	}
//	
//	// save initial and final data of each task
//	_markerTrajBC.resize(numTask + 1);
//	_objectTrajBC.resize(numTask + 1);
//	string humanDataPath;
//	string objDataPath;
//	for (int i = 0; i < numTask; i++)
//	{
//		humanDataPath = str + "task" + std::to_string(i+1) + "/human/";
//		objDataPath = str + "task" + std::to_string(i+1) + "/object/";
//		if (i == numTask - 1)
//		{
//			_robotManager_p->mergeSkeletonTrajectory_init_final(humanDataPath);
//			for (int j = 0; j < 2; j++)
//				_markerTrajBC[i + j] = _robotManager_p->m_markerTraj[j];
//			_robotManager_p->setObjectsTraj_init_final(objDataPath);
//			for (int j = 0; j < 2; j++)
//				_objectTrajBC[i + j] = _robotManager_p->m_objectTraj[j];
//
//
//		}
//		else
//		{
//			_robotManager_p->mergeSkeletonTrajectory(humanDataPath, 0, 0);
//			_markerTrajBC[i] = _robotManager_p->m_markerTraj[0];
//			_robotManager_p->setObjectsTraj(objDataPath, 0, 0);
//			_objectTrajBC[i] = _robotManager_p->m_objectTraj[0];
//
//		}
//	}
//
//	// solve inverse kinematics for each dualArmTask
//	vector<int> dataFeatureIdx(2);
//	dataFeatureIdx[0] = SDA20D_P_Index::MLINK_RIGHT_T;
//	dataFeatureIdx[1] = SDA20D_P_Index::MLINK_LEFT_T;
//	vector<SE3> rightT(numTask + 1);
//	vector<bool> rightIncludeOri(numTask + 1);
//	vector<SE3> leftT(numTask + 1);
//	vector<bool> leftIncludeOri(numTask + 1);
//	Vec3 tmpVec;
//	Vec3 tmpW;
//
//	vector<SE3> Tinit = _robotManager->forwardKin(VectorToVec(_robotManager->generateInitialQ_initguess()));
//	vector<int> featureIdx(2);
//	featureIdx[0] = SDA20D_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx[1] = SDA20D_Index::MLINK_LEFT_GRIPPER;
//	rightT[0] = Tinit[featureIdx[0]];
//	rightIncludeOri[0] = true;
//	leftT[0] = Tinit[featureIdx[1]];
//	leftIncludeOri[0] = true;
//	int tmpR, tmpL;
//	for (int i = 0; i < numTask; i++)
//	{
//		tmpR = _numRightObject[i];
//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::STOP)
//		{
//			rightT[i + 1] = rightT[i];
//			rightIncludeOri[i + 1] = true;
//		}
//		else
//		{
//			for (int j = 0; j < 3; j++)
//			{
//				tmpVec[j] = _objectTrajBC[i+1](tmpR, j);
//				tmpW[j] = _objectTrajBC[i+1](tmpR, j + 3);
//			}
//			rightT[i + 1] = SE3(Exp(tmpW), tmpVec) * _robotManager->m_objectList[tmpR]->m_data2ref;
//			rightIncludeOri[i] = true;
//			if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::REACH)
//				rightT[i + 1] *= Inv(_robotManager->m_objectList[tmpR]->m_robot2objReach);
//			else if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::GRASP || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//				rightT[i + 1] *= Inv(_robotManager->m_objectList[tmpR]->m_robot2obj);
//		}
//		tmpL = _numLeftObject[i];
//		if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::STOP)
//		{
//			leftT[i + 1] = leftT[i];
//			leftIncludeOri[i + 1] = true;
//		}
//		else
//		{
//			if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::REACH || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::GRASP)
//			{
//				// for planning
//				leftT[i + 1] = _objectInitPos[i][tmpL] * _robotManager->m_objectList[tmpL]->m_data2ref;
//			}
//			else
//			{
//				for (int j = 0; j < 3; j++)
//				{
//					tmpVec[j] = _objectTrajBC[i + 1](tmpL, j);
//					tmpW[j] = _objectTrajBC[i + 1](tmpL, j + 3);
//				}
//				leftT[i + 1] = SE3(Exp(tmpW), tmpVec) * _robotManager->m_objectList[tmpL]->m_data2ref;
//			}
//			
//			
//			leftIncludeOri[i + 1] = true;
//			if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::REACH)
//				leftT[i + 1] *= Inv(_robotManager->m_objectList[tmpL]->m_robot2objReach);
//			else if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::GRASP || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//				leftT[i + 1] *= Inv(_robotManager->m_objectList[tmpL]->m_robot2obj);
//		}
//	}
//
//	int tasknumber = 7;
//	int objnumber = 2;
//	SE3 Tprofile = _objectInitPos[tasknumber][0] * _robotManager->m_objectList[0]->m_data2ref;
//	SE3 Tobj = _objectInitPos[tasknumber][objnumber] * _robotManager->m_objectList[objnumber]->m_data2ref;
//	
//	SE3 Tobjdes = SE3(Vec3(-0.05, 0.09, 0.02)) * Tprofile;
//	//SE3 Tobjdes = SE3(Vec3(0.00, 0.00, 0.01)) * Tprofile;
//	//Tobjdes = Inv(Tprofile) * Tobj;
//	cout << Tprofile << endl;
//	cout << _objectInitPos[tasknumber][objnumber] << endl;
//	cout << Tobjdes << endl;
//	SE3 data2ref = Inv(_objectInitPos[tasknumber][objnumber]) * Tobjdes;
//
//
//	// solve inverse kinematics
//
//#ifndef LOADINITIALGUESS
//	humanDataPath = str + "task" + std::to_string(1) + "/human/";
//	vector<Eigen::VectorXd> robotTraj_bf = _robotManager_p->doOptimization_fdf_Phi(humanDataPath, 41, 45);
//	vector<Eigen::VectorXd> saveVector(1);
//	saveVector[0].resize(_robotManager_p->m_phi.size());
//	for (unsigned int i = 0; i < _robotManager_p->m_phi.size(); i++)
//		saveVector[0](i) = _robotManager_p->m_phi[i];
//	saveDataToText(saveVector, "saveData.txt");
//#else
//
//	Eigen::VectorXd temp = loadVectorFromText("saveData.txt", _robotManager_p->getActiveArmInfo()->m_numPJoint);
//	_robotManager_p->m_phi.resize(temp.size());
//	for (int i = 0; i < _robotManager_p->getActiveArmInfo()->m_numPJoint; i++)
//		_robotManager_p->m_phi[i] = temp(i);
//#endif
//
//	vector<SE3> tmpT(2);
//	vector<bool> tmpIncludeOri(2);
//	vector<int> featureIdx_P(2);
//	featureIdx_P[0] = SDA20D_P_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx_P[1] = SDA20D_P_Index::MLINK_LEFT_GRIPPER;
//	Eigen::VectorXd qtemp;
//	int flag;
//	bool considerPosture = true;
//	for (int i = 0; i < numTask + 1; i++)
//	{
//		tmpT[0] = rightT[i];
//		tmpT[1] = leftT[i];
//		tmpIncludeOri[0] = rightIncludeOri[i];
//		tmpIncludeOri[1] = leftIncludeOri[i];
//		//tmpIncludeOri[0] = false;
//		//tmpIncludeOri[1] = false;
//		printf("%d-th task\n", i+1);
//		if (considerPosture)
//		{ 
//			if (i == 0)
//				qtemp = _robotManager_p->inverseKinAddPostureTask(tmpT, featureIdx_P, tmpIncludeOri, _markerTrajBC[i], flag);
//			else
//			{
//				if (i % 2 == 0)
//					qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, qtemp);
//				else
//					qtemp = _robotManager_p->inverseKinAddPostureTask(tmpT, featureIdx_P, tmpIncludeOri, _markerTrajBC[i], flag, qtemp);
//				int iter = 0;
//				while (flag != SDA20DManager::invKinFlag::SOLVED && iter < 5)
//				{
//					printf("ignore posture\n");
//					//if (iter == 0)
//					//	qtemp = _robotManager_p->inverseKinAddPostureTask(tmpT, featureIdx_P, tmpIncludeOri, _markerTrajBC[i], flag);
//					//else
//					//	qtemp = _robotManager_p->inverseKinAddPostureTask(tmpT, featureIdx_P, tmpIncludeOri, _markerTrajBC[i], flag, _robotManager->generateRandomQ());
//					//iter++;
//					//if (iter == 0)
//					//	qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag);
//					//else
//					//	qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, _robotManager->generateRandomQ());
//					//iter++;
//					if (iter == 0)
//						qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, Eigen::VectorXd(), 5000);
//					else
//						qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, _robotManager->generateRandomQ(), 5000);
//					iter++;
//				}
//			}
//		}
//		else
//		{
//			if (i == 0)
//				qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag);
//			else
//			{
//				qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, qtemp);
//				int iter = 0;
//				while (flag != SDA20DManager::invKinFlag::SOLVED && iter < 5)
//				{
//					if (iter == 0)
//						qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, Eigen::VectorXd());
//					else
//						qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, _robotManager->generateRandomQ());
//					iter++;
//				}
//			}
//		}
//		
//			
//		printf("inv kin flag: %d \n", flag);
//		// save solutions
//		if (i == 0)
//			_dualArmTaskSet[i]->_startJointVal = qtemp;
//		else if (i == numTask)
//			_dualArmTaskSet[i - 1]->_goalJointVal = qtemp;
//		else
//		{
//			_dualArmTaskSet[i - 1]->_goalJointVal = qtemp;
//			_dualArmTaskSet[i]->_startJointVal = qtemp;
//		}
//	}
//	for (int i = 0; i < numTask; i++)
//		cout << feasibilityCheck(i) << endl;
//}
//
//dualArmTask::TASKTYPE SDA20DTaskManager::loadTaskType(int taskType)
//{
//	dualArmTask::TASKTYPE type;
//	switch (taskType)
//	{
//	case 0:
//		type = dualArmTask::TASKTYPE::REACH;
//		break;
//	case 1:
//		type = dualArmTask::TASKTYPE::GRASP;
//		break;
//	case 2:
//		type = dualArmTask::TASKTYPE::MOVE;
//		break;
//	case 3:
//		type = dualArmTask::TASKTYPE::PUT;
//		break;
//	case 4:
//		type = dualArmTask::TASKTYPE::STOP;
//		break;
//	default:
//		type = dualArmTask::TASKTYPE::STOP;
//		break;
//	}
//	return type;
//}
//
//bool SDA20DTaskManager::feasibilityCheck(int taskNum)
//{
//	// check feasibility of initial and final condition after setting environment
//	initializeObject(taskNum);
//	bool isFeasibleBCatStart = true;
//	bool isFeasibleBCatGoal = true;
//	vector<double> rJointVal = VectorToVec(_dualArmTaskSet[taskNum]->_startJointVal);
//	_robotManager->setJointValue(rJointVal);
//	//if (_dualArmTaskSet[taskNum]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[taskNum]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//		_robotManager->setLeftGripperValue(0.0125);
//	//else
//	//	_robotManager->setLeftGripperValue(0.0065);
//	//cout << "start task" << taskNum << endl;
//	//for (int i = 0; i < 3; i++)
//	//	cout << _robotManager->m_objectList[i]->GetBaseLink()->GetFrame() << endl;
//	//cout << _dualArmTaskSet[taskNum]->_leftObject->GetBaseLink()->GetFrame() << endl;
//	if (_robotManager->checkCollision())
//		isFeasibleBCatStart = false;
//	rJointVal = VectorToVec(_dualArmTaskSet[taskNum]->_goalJointVal);
//	_robotManager->setJointValue(rJointVal);
//	//cout << "goal task" << taskNum << endl;
//	//for (int i = 0; i < 3; i++)
//	//	cout << _robotManager->m_objectList[i]->GetBaseLink()->GetFrame() << endl;
//	//cout << _dualArmTaskSet[taskNum]->_leftObject->GetBaseLink()->GetFrame() << endl;
//	if (_robotManager->checkCollision())
//		isFeasibleBCatGoal = false;
//	return isFeasibleBCatStart && isFeasibleBCatGoal;
//}
//
//bool SDA20DTaskManager::feasibilityCheck()
//{
//	// check feasibility
//	bool isFeasibleBC = true;
//	for (int tasknum = 0; tasknum < _dualArmTaskSet.size(); tasknum++)
//	{
//		isFeasibleBC &= feasibilityCheck(tasknum);
//		//initializeObject(tasknum);
//		//rJointVal = VectorToVec(_dualArmTaskSet[tasknum]->_startJointVal);
//		//_robotManager->setJointValue(rJointVal);
//		//cout << "start of task " << tasknum << endl;
//		//_robotManager->getSpace()->DYN_MODE_RUNTIME_SIMULATION_LOOP();
//		//_robotManager->updateCollisionManagerFCL();
//		//cout << "col(srLib): " << _robotManager->checkCollision() << endl;
//
//		//rJointVal = VectorToVec(_dualArmTaskSet[tasknum]->_goalJointVal);
//		//_robotManager->setJointValue(rJointVal);
//		//cout << "goal of task " << tasknum << endl;
//		//_robotManager->getSpace()->DYN_MODE_RUNTIME_SIMULATION_LOOP();
//		//_robotManager->updateCollisionManagerFCL();
//		//cout << "col(srLib): " << _robotManager->checkCollision() << endl;
//	}
//	return isFeasibleBC;
//}
//
//bool SDA20DTaskManager::feasibilityCheck_fcl(int taskNum)
//{
//	// check feasibility of initial and final condition after setting environment
//	initializeObject(taskNum);
//	bool isFeasibleBC = true;
//	vector<double> rJointVal = VectorToVec(_dualArmTaskSet[taskNum]->_startJointVal);
//	_robotManager->setJointValue(rJointVal);
//	_robotManager->setLeftGripperValue(0.0125);
//	_robotManager->getSpace()->DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	//cout << "start task" << taskNum << endl;
//	//for (int i = 0; i < 3; i++)
//	//	cout << _robotManager->m_objectList[i]->GetBaseLink()->GetFrame() << endl;
//	if (_robotManager->checkCollisionFCL(robotManager::COL_CHECK::ALL))
//		isFeasibleBC = false;
//	rJointVal = VectorToVec(_dualArmTaskSet[taskNum]->_goalJointVal);
//	_robotManager->setJointValue(rJointVal);
//	_robotManager->getSpace()->DYN_MODE_RUNTIME_SIMULATION_LOOP();
//	//cout << "goal task" << taskNum << endl;
//	//for (int i = 0; i < 3; i++)
//	//	cout << _robotManager->m_objectList[i]->GetBaseLink()->GetFrame() << endl;
//	if (_robotManager->checkCollisionFCL(robotManager::COL_CHECK::ALL))
//		isFeasibleBC = false;
//	return isFeasibleBC;
//}
//
//
//void SDA20DTaskManager::runTask(string str, bool doPlanning)
//{
//	_jointTraj.resize(0);
//
//	for (unsigned int i = 0; i < _dualArmTaskSet.size(); i++)
//	{
//		_jointTraj.push_back(runTask(str, i, doPlanning));
//	}
//}
//
//pair<vector<Eigen::VectorXd>, vector<double>> SDA20DTaskManager::runTask(string str, int taskNum, bool doPlanning)
//{
//	std::string humanDataPath;
//	std::string objDataPath;
//	std::string trajPath;
//	pair<vector<Eigen::VectorXd>, vector<double>> robotTraj;
//
//	
//
//	int i = taskNum;
//	humanDataPath = str + "task" + std::to_string(i+1) + "/human/";
//	objDataPath = str + "task" + std::to_string(i+1) + "/object/";
//	trajPath = str + "task" + std::to_string(i+1) + ".txt";
//
//	initializeObject(i);
//	_robotManager_p->makeMarkerTraj_mode(humanDataPath);
//	_robotManager_p->setObjectsTraj(objDataPath);
//	/////////////////////////////////////////// RETARGETTING /////////////////////////////////////////////
//
//
//	
//	_robotManager_p->clearSubTaskFrames();
//	if (_dualArmTaskSet[i]->_type == dualArmTask::TYPE::FREE)
//	{
//		int rightObjNum;
//		bool rightGrasp = false;
//		bool leftGrasp = false;
//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//		{
//			rightGrasp = true;
//			for (int j = 0; j < NUM_OF_OBJECTS; j++)
//			{
//				if (_dualArmTaskSet[i]->_rightObject == _robotManager->m_objectList[j])
//				{
//					rightObjNum = j;
//					break;
//				}
//			}
//		}
//
//		int leftObjNum;
//		if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//		{
//			leftGrasp = true;
//			for (int j = 0; j < NUM_OF_OBJECTS; j++)
//			{
//				if (_dualArmTaskSet[i]->_leftObject == _robotManager->m_objectList[j])
//				{
//					leftObjNum = j;
//					break;
//				}
//			}
//		}
//
//		if (rightGrasp && leftGrasp)
//		{
//			_robotManager_p->makeEndEffectorTask(rightObjNum, leftObjNum);
//			_robotManager_p->addPostureTask();
//		}
//		else if (rightGrasp && !leftGrasp)
//		{
//			_robotManager_p->makeEndEffectorTask(rightObjNum, true);
//			_robotManager_p->addPostureTask();
//		}
//		else if (!rightGrasp && leftGrasp)
//		{
//			_robotManager_p->makeEndEffectorTask(leftObjNum, false);
//			_robotManager_p->addPostureTask();
//		}
//		else
//		{
//			// give end-effector orientation task
//			// TODO
//			_robotManager_p->makeMarkerTask();
//		}
//	}
//	else if (_dualArmTaskSet[i]->_type == dualArmTask::TYPE::CONSTRAINED)
//	{
//		// maybe later..
//	}
//	
//
//	double retargettingTime = clock();
//	robotTraj = _robotManager_p->closedLoopInverseKinematics(humanDataPath, _dualArmTaskSet[i]->_startJointVal);
//	retargettingTime = clock() - retargettingTime;
//	printf("retargetting time: %.5f\n", retargettingTime);
//
//	///////////////////////////////////////////// PLANNING /////////////////////////////////////////////////
//	bool planning;
//	if (doPlanning)
//		planning = runPlanningTask(i, robotTraj);
//	
//
//	//saveDataToText(robotTraj.first, trajPath);
//
//	return robotTraj;
//}
//
//bool SDA20DTaskManager::runPlanningTask()
//{
//	_jointTraj.resize(0);
//	pair<vector<Eigen::VectorXd>, vector<double>> robotTraj;
//	_planningTime.resize(0);
//	for (unsigned int i = 0; i < _dualArmTaskSet.size(); i++)
//	{
//		initializeObject(i);
//		
//		robotTraj.first.resize(0);
//		robotTraj.second.resize(0);
//		runPlanningTask(i, robotTraj);
//		_jointTraj.push_back(robotTraj);
//	}
//	return false;
//}
//
//bool SDA20DTaskManager::runPlanningTask2()
//{
//	_jointTraj.resize(0);
//	pair<vector<Eigen::VectorXd>, vector<double>> robotTraj;
//	_planningTime.resize(0);
//	for (unsigned int i = 0; i < _dualArmTaskSet.size(); i++)
//	{
//		initializeObject(i);
//
//		robotTraj.first.resize(0);
//		robotTraj.second.resize(0);
//
//		if (i > 0 && i % 2 == 1)
//		{
//			int numTrj = 5;
//			Eigen::VectorXd diff = _dualArmTaskSet[i]->_goalJointVal - _dualArmTaskSet[i]->_startJointVal;
//			for (int j = 0; j < numTrj; j++)
//			{
//				robotTraj.first.push_back(_dualArmTaskSet[i]->_startJointVal + (double)j / (numTrj - 1) * diff);
//				robotTraj.second.push_back((double)j * 0.1);
//			}
//		}
//
//		runPlanningTask(i, robotTraj);
//		_jointTraj.push_back(robotTraj);
//	}
//	return false;
//}
//
//bool SDA20DTaskManager::runPlanningTask(int taskNum, pair<vector<Eigen::VectorXd>, vector<double>>& robotTraj)
//{
//	int i = taskNum;
//	///////////////////////////////////////////// PLANNING /////////////////////////////////////////////////
//	cout << "feasibility of task " << i << ": " << feasibilityCheck(i) << endl;
//	cout << "feasibility of task(fcl) " << i << ": " << feasibilityCheck_fcl(i) << endl;
//	if (feasibilityCheck(i))
//	{
//		// rrt parameters
//		double step_size = 0.05;
//
//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::GRASP || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::GRASP)
//			step_size = 0.005;
//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::PUT || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::PUT)
//			step_size = 0.005;
//
//		// to speed up
//		step_size *= 15;
//		int smoothingNum = 20;
//		// normal
//		//step_size *= 2;
//		//int smoothingNum = 50;
//		int planningMode = SDA20DrrtManager::ALL;
//		printf("start rrt...\n");
//		double t_f = 1.0;
//		if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::STOP && !_robotManager->getActiveArmInfo()->m_useWaist)
//		{
//			if ((_robotManager->getLeftArmJointVal(_dualArmTaskSet[i]->_startJointVal) - _robotManager->getLeftArmJointVal(_dualArmTaskSet[i]->_goalJointVal)).norm() < 1e-5)
//			{
//				_rrtManager->setStartandGoal(_robotManager->getRightArmJointVal(_dualArmTaskSet[i]->_startJointVal), _robotManager->getRightArmJointVal(_dualArmTaskSet[i]->_goalJointVal));
//				planningMode = SDA20DrrtManager::RIGHTARMONLY;
//				_rrtManager->setPlanningMode(planningMode);
//				cout << "plan right arm only..." << endl;
//				if (robotTraj.first.size() > 0)
//				{
//					vector<Eigen::VectorXd> tempTrj(robotTraj.first.size());
//					Eigen::VectorXd tempVec;
//					for (unsigned int j = 0; j < tempTrj.size(); j++)
//					{
//						tempVec = _robotManager->getRightArmJointVal(_dualArmTaskSet[i]->_startJointVal);
//						tempTrj[j] = robotTraj.first[j].head(tempVec.size()) = tempVec;
//					}
//					_rrtManager->setTrajFollowVectorField(tempTrj);
//					t_f = robotTraj.second[robotTraj.second.size() - 1];
//				}
//			}
//		}
//		else if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::STOP && !_robotManager->getActiveArmInfo()->m_useWaist)
//		{
//			if ((_robotManager->getRightArmJointVal(_dualArmTaskSet[i]->_startJointVal) - _robotManager->getRightArmJointVal(_dualArmTaskSet[i]->_goalJointVal)).norm() < 1e-5)
//			{
//				_rrtManager->setStartandGoal(_robotManager->getLeftArmJointVal(_dualArmTaskSet[i]->_startJointVal), _robotManager->getLeftArmJointVal(_dualArmTaskSet[i]->_goalJointVal));
//				planningMode = SDA20DrrtManager::LEFTARMONLY;
//				_rrtManager->setPlanningMode(planningMode);
//				cout << "plan left arm only..." << endl;
//				if (robotTraj.first.size() > 0)
//				{
//					vector<Eigen::VectorXd> tempTrj(robotTraj.first.size());
//					Eigen::VectorXd tempVec;
//					for (unsigned int j = 0; j < tempTrj.size(); j++)
//					{
//						tempVec = _robotManager->getLeftArmJointVal(_dualArmTaskSet[i]->_startJointVal);
//						tempTrj[j] = robotTraj.first[j].tail(tempVec.size()) = tempVec;
//					}
//					_rrtManager->setTrajFollowVectorField(tempTrj);
//					t_f = robotTraj.second[robotTraj.second.size() - 1];
//				}
//			}
//		}
//		else
//		{
//			_rrtManager->setStartandGoal(_dualArmTaskSet[i]->_startJointVal, _dualArmTaskSet[i]->_goalJointVal);
//			planningMode = SDA20DrrtManager::ALL;
//			_rrtManager->setPlanningMode(planningMode);
//			if (robotTraj.first.size() > 0)
//			{
//				_rrtManager->setTrajFollowVectorField(robotTraj.first);
//				t_f = robotTraj.second[robotTraj.second.size() - 1];
//			}
//		}
//		double time = (double) clock();
//		_rrtManager->execute(step_size);
//		robotTraj.first = _rrtManager->extractPath(smoothingNum);
//		_planningTime.push_back((double) clock() - time);
//
//		if (planningMode != SDA20DrrtManager::ALL)
//		{
//			robotTraj.first = expandToDualArmTrj(robotTraj.first, planningMode, _dualArmTaskSet[i]->_startJointVal);
//		}
//
//		int trjNum = robotTraj.first.size();
//		robotTraj.second.resize(trjNum);
//		for (int j = 0; j < trjNum; j++)
//			robotTraj.second[j] = (double)j / (double)(trjNum - 1) * t_f;
//
//		// check feasibility
//		bool colsrLib;
//		bool colfcl;
//		for (unsigned int j = 0; j < trjNum; j++)
//		{
//			_robotManager->setJointValue(VectorToVec(robotTraj.first[j]));
//			colsrLib = _robotManager->checkCollision();
//			colfcl = _robotManager->checkCollisionFCL(robotManager::COL_CHECK::ALL);
//			if (colsrLib || colfcl)
//			{
//				cout << "col(srLib): " << colsrLib << ", col(fcl): " << colfcl << ", trj #: " << i << endl;
//			}
//		}
//
//		// match velocity limit and save
//		//if (robotTraj.first.size() > 2)
//		//	bool adjustStep = _robotManager->adjustTimeStep(robotTraj);
//
//		// check feasibility
//		//trjNum = robotTraj.first.size();
//		//for (unsigned int j = 0; j < trjNum; j++)
//		//{
//		//	_robotManager->setJointValue(VectorToVec(robotTraj.first[i]));
//		//	colsrLib = _robotManager->checkCollision();
//		//	colfcl = _robotManager->checkCollisionFCL(robotManager::COL_CHECK::ALL);
//		//	if (colsrLib || colfcl)
//		//	{
//		//		cout << "col(srLib): " << colsrLib << ", col(fcl): " << colfcl << ", trj #: " << i << endl;
//		//	}
//		//}
//		cout << "trj length = " << trjNum << endl;
//		return true;
//	}
//	else
//	{
//		printf("check initial and final configuration to start rrt... for task %d\n", i + 1);
//		return false;
//	}
//}
//
//bool SDA20DTaskManager::setTaskData(bool loadProblem /* = false*/, int probNum /* = 0*/)
//{
//	/////////////////////////////////////////// TASK HARD CODING ///////////////////////////////////////////
//	int numTask = 8;
//	_dualArmTaskSet.resize(numTask);
//	_numRightObject.resize(numTask);
//	_numLeftObject.resize(numTask);
//	numTask = 0;
//	///////////////////////////////////////// TASK 1 : REACH TO PLATE //////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::REACH);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 1;
//	///////////////////////////////////////// TASK 2 : GRASP PLATE //////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::GRASP);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 1;
//	///////////////////////////////////////// TASK 3 : MOVE PLATE //////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::MOVE);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 1;
//	///////////////////////////////////////// TASK 4 : PUT PLATE //////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::PUT);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 1;
//	///////////////////////////////////////// TASK 5 : REACH TO CHIP ///////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::REACH);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 2;
//	///////////////////////////////////////// TASK 6 : GRASP CHIP ///////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::GRASP);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 2;
//	///////////////////////////////////////// TASK 7 : MOVE CHIP ///////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::MOVE);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 2;
//	///////////////////////////////////////// TASK 8 : PUT CHIP ///////////////////////////////////////////
//	_dualArmTaskSet[numTask] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::PUT);
//	_numRightObject[numTask] = 100;
//	_numLeftObject[numTask++] = 2;
//	/////////////////////////////////////////// TASK 5 : REACH TO DRIVER /////////////////////////////////////
//	//_dualArmTaskSet[4] = new dualArmTask(dualArmTask::TASKTYPE::REACH, dualArmTask::TASKTYPE::REACH);
//	//_numRightObject[4] = 2;
//	//_numLeftObject[4] = 5;
//	/////////////////////////////////////////// TASK 6 : MOVE DRIVER /////////////////////////////////////////
//	//_dualArmTaskSet[5] = new dualArmTask(dualArmTask::TASKTYPE::STOP, dualArmTask::TASKTYPE::MOVE);
//	//_numRightObject[5] = 100;
//	//_numLeftObject[5] = 5;
//
//	_objectInitPos.resize(numTask);
//
//
//	/////////////////// object positions
//	// set as random
//	//_robotManager->m_objectList[0]->setLocalFramefromTable(EulerZYX(Vec3( 0.2*((double)rand()/RAND_MAX - 0.5) , 0.0, 0.0), Vec3(0.1*(double)rand() / RAND_MAX - 0.05, -0.1*(double)rand() / RAND_MAX, static_cast<AlProfile*>(_robotManager->m_objectList[0])->m_AlProfileDim[2])));
//	//_robotManager->m_objectList[1]->setLocalFramefromTable(EulerXYZ(Vec3(0.0, (double)rand() / RAND_MAX * SR_PI_HALF, (double)rand() / RAND_MAX * SR_PI_HALF), Vec3(0.1*(double)rand() / RAND_MAX, 0.2*(double)rand() / RAND_MAX , 0.115)));
//	//_robotManager->m_objectList[2]->setLocalFramefromTable(EulerZYX(Vec3((double)rand() / RAND_MAX * SR_PI_HALF, 0.0, (double)rand() / RAND_MAX * SR_PI_HALF), Vec3(0.1*(double)rand() / RAND_MAX + 0.1, 0.2*(double)rand() / RAND_MAX , 0.065)));
//	////_robotManager->m_objectList[5]->setLocalFramefromTable(SE3(Vec3(0.0, 0.0, 0.0)));
//	double offset = 0.00001;
//	bool collide = true;
//	vector<double> zeroJointVal(_robotManager->getActiveArmInfo()->m_numJoint, 0.0);
//	_robotManager->setJointValue(zeroJointVal);
//	while (collide)
//	{
//		_robotManager->m_objectList[0]->setLocalFramefromTable(
//			EulerZYX(Vec3(0.2*((double)rand() / RAND_MAX - 0.5), 0.0, 0.0),
//				Vec3(0.1*(double)rand() / RAND_MAX, -0.1*(double)rand() / RAND_MAX,
//					static_cast<AlProfile*>(_robotManager->m_objectList[0])->m_AlProfileDim[2] + offset)));
//		_robotManager->m_objectList[1]->setLocalFramefromTable(
//			EulerZYX(Vec3(0.0, SR_PI_HALF, -(double)rand() / RAND_MAX * 0.5* SR_PI_HALF),
//				Vec3(0.05*(double)rand() / RAND_MAX, 0.1*(double)rand() / RAND_MAX + 0.25, 0.115 + offset + 0.2*(double)rand() / RAND_MAX)));
//		//_robotManager->m_objectList[2]->setLocalFramefromTable(
//		//	EulerZYX(Vec3(0.0, SR_PI_HALF, -(double)rand() / RAND_MAX * 0.5 * SR_PI_HALF - 0.25 * SR_PI_HALF),
//		//		Vec3(0.1*(double)rand() / RAND_MAX + 0.1, 0.2*(double)rand() / RAND_MAX + 0.1, 0.065 + offset + 0.07)));
//		_robotManager->m_objectList[2]->setLocalFramefromTable(
//			EulerZYX(Vec3(SR_PI_HALF, 0.0, (double)rand() / RAND_MAX * 0.5 * SR_PI_HALF + 0.25 * SR_PI_HALF),
//				Vec3(0.1*(double)rand() / RAND_MAX + 0.2, 0.2*(double)rand() / RAND_MAX + 0.15, 0.035 + offset + 0.2*(double)rand() / RAND_MAX)));
//
//		// test
//		//_robotManager->m_objectList[0]->setLocalFramefromTable(
//		//	EulerZYX(Vec3(0.2*(0.5 - 0.5), 0.0, 0.0),
//		//		Vec3(0, -0.1,
//		//			static_cast<AlProfile*>(_robotManager->m_objectList[0])->m_AlProfileDim[2] + offset)));
//		//_robotManager->m_objectList[1]->setLocalFramefromTable(
//		//	EulerZYX(Vec3(0.0, SR_PI_HALF, -0.5 * 0.5* SR_PI_HALF),
//		//		Vec3(0.05*0.5, 0.25, 0.115 + offset + 0.01)));
//		//_robotManager->m_objectList[2]->setLocalFramefromTable(
//		//	EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.5 * 0.5 * SR_PI_HALF + 0.25 * SR_PI_HALF),
//		//		Vec3(0.1*0.5 + 0.1, 0.2, 0.035 + offset)));
//
//		vector<SE3> objPosOri(2);
//
//		bool saveProblem = false;
//		vector<Eigen::VectorXd> objData;
//		string fileName = "problem" + to_string(probNum + 1) + ".txt";
//		string dir = "../../../data/problem/";
//		Vec3 tmpVec;
//		Vec3 tmpOri;
//		if (loadProblem)
//		{
//			objData = loadDataFromText(dir + fileName, 6);
//			for (unsigned int j = 0; j < objPosOri.size(); j++)
//			{
//
//				for (int k = 0; k < 3; k++)
//				{
//					tmpVec[k] = objData[j](k);
//					tmpOri[k] = objData[j](k + 3);
//				}
//				objPosOri[j].SetPosition(tmpVec);
//				objPosOri[j].SetOrientation(Exp(tmpOri));
//			}
//		}
//		else
//		{
//			objPosOri[0] = EulerZYX(Vec3(0.0, SR_PI_HALF, -(double)rand() / RAND_MAX * 0.5* SR_PI_HALF),
//				Vec3(0.05*(double)rand() / RAND_MAX - 0.05, 0.1*(double)rand() / RAND_MAX + 0.25, 0.1 + offset + 0.2*(double)rand() / RAND_MAX));
//			objPosOri[1] = EulerZYX(Vec3(SR_PI_HALF, 0.0, (double)rand() / RAND_MAX * 0.5 * SR_PI_HALF + 0.25 * SR_PI_HALF),
//				Vec3(0.1*(double)rand() / RAND_MAX + 0.25, 0.2*(double)rand() / RAND_MAX + 0.20, offset + 0.2*(double)rand() / RAND_MAX));
//
//			//objPosOri[0] = EulerZYX(Vec3(0.0, SR_PI_HALF, -0.5* SR_PI_HALF),
//			//	Vec3(0.0, 0.30, 0.115 + offset + 0.1));
//			//objPosOri[1] = EulerZYX(Vec3(SR_PI_HALF, 0.0, 0.5 * SR_PI_HALF),
//			//	Vec3(0.22, 0.25, offset + 0.1));
//		}
//
//
//		_robotManager->m_objectList[0]->setLocalFramefromTable(
//			EulerZYX(Vec3(0.0, 0.0, 0.0),
//				Vec3(0.0, -0.1,
//					static_cast<AlProfile*>(_robotManager->m_objectList[0])->m_AlProfileDim[2] + offset)));
//		_robotManager->m_objectList[1]->setLocalFramefromTable(
//			objPosOri[0]);
//		_robotManager->m_objectList[2]->setLocalFramefromTable(
//			objPosOri[1]);
//
//
//		if (saveProblem)
//		{
//			vector<Eigen::VectorXd> savePosOri(objPosOri.size(), Eigen::VectorXd::Zero(6));
//			for (unsigned int i = 0; i < savePosOri.size(); i++)
//			{
//				tmpVec = objPosOri[i].GetPosition();
//				tmpOri = Log(objPosOri[i].GetOrientation());
//				for (int j = 0; j < 3; j++)
//				{
//					savePosOri[i](j) = tmpVec[j];
//					savePosOri[i](j + 3) = tmpOri[j];
//				}
//			}
//
//			saveDataToText(savePosOri, dir + fileName);
//		}
//
//		_robotManager->getSpace()->DYN_MODE_RUNTIME_SIMULATION_LOOP();
//		collide = _robotManager->checkCollision();
//	}
//
//
//	SE3 Tprofile = _robotManager->m_objectList[0]->GetBaseLink()->GetFrame();
//	SE3 TplateInit = _robotManager->m_objectList[1]->GetBaseLink()->GetFrame();
//	SE3 TchipInit = _robotManager->m_objectList[2]->GetBaseLink()->GetFrame();
//	SE3 TdriverInit = _robotManager->m_objectList[5]->GetBaseLink()->GetFrame();
//	double offsetToPut = 0.03;
//	SE3 TplateFinal = Tprofile * SE3(Vec3(0.0, 0.0, 0.03));
//	SE3 TplateMiddle = TplateFinal * SE3(Vec3(0.0, 0.0, offsetToPut));
//	offset = 0.005;
//	SE3 TchipFinal = TplateFinal * SE3(Vec3(-0.05 + offset, 0.09 - offset, 0.015));
//	SE3 TchipMiddle = TchipFinal * SE3(Vec3(0.0, 0.0, offsetToPut));
//	SE3 TdriverFinal;
//
//	vector<double> rJointVal(_robotManager->getActiveArmInfo()->m_numJoint, 0.0);
//	Eigen::VectorXd jointVal = _robotManager->generateInitialQ_initguess();
//	for (int i = 0; i < jointVal.size(); i++)
//		rJointVal[i] = jointVal(i);
//	vector<SE3> Tinit = _robotManager->forwardKin(rJointVal);
//	vector<int> featureIdx(2);
//	featureIdx[0] = SDA20D_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx[1] = SDA20D_Index::MLINK_LEFT_GRIPPER;
//	SE3 TrightInit = Tinit[featureIdx[0]];
//	SE3 TleftInit = Tinit[featureIdx[1]];
//	////////////////////////////////////////
//	vector<SE3> Ttemp(2);
//	vector<vector<SE3>> objectPos(NUM_OF_OBJECTS, Ttemp);			// first vector: object number, second vector: object pos sequence
//	for (unsigned int i = 1; i < objectPos.size(); i++)
//	{
//		for (unsigned int j = 0; j < objectPos[i].size(); j++)
//			objectPos[i][j] = SE3(Vec3(-5, 10 * i, 10 * j));
//	}
//	objectPos[0][0] = Tprofile;
//	objectPos[0][1] = Tprofile;
//	objectPos[1].resize(3);
//	objectPos[1][0] = TplateInit;
//	objectPos[1][1] = TplateMiddle;
//	objectPos[1][2] = TplateFinal;
//	objectPos[2].resize(3);
//	objectPos[2][0] = TchipInit;
//	objectPos[2][1] = TchipMiddle;
//	objectPos[2][2] = TchipFinal;
//	//objectPos[5][0] = TdriverInit;
//	//objectPos[5][1] = TdriverFinal;
//	vector<unsigned int> ismoved(NUM_OF_OBJECTS, 0);
//	for (int i = 0; i < numTask; i++)
//	{
//		_objectInitPos[i].resize(NUM_OF_OBJECTS);
//
//		for (int j = 0; j < NUM_OF_OBJECTS; j++)
//		{
//			_objectInitPos[i][j] = objectPos[j][ismoved[j]];
//			if (_numRightObject[i] == j)
//			{
//				_dualArmTaskSet[i]->_rightObject = _robotManager->m_objectList[j];
//				if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//					ismoved[j]++;
//			}
//			if (_numLeftObject[i] == j)
//			{
//				_dualArmTaskSet[i]->_leftObject = _robotManager->m_objectList[j];
//				if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//					ismoved[j]++;
//			}
//		}
//	}
//
//	// solve inverse kinematics
//	vector<SE3> rightT(numTask + 1);
//	vector<bool> rightIncludeOri(numTask + 1, true);
//	vector<SE3> leftT(numTask + 1);
//	vector<bool> leftIncludeOri(numTask + 1, true);
//
//	rightT[0] = TrightInit;
//	leftT[0] = TleftInit;
//	for (int i = 0; i < NUM_OF_OBJECTS; i++)
//		ismoved[i] = 0;
//	int tmpR, tmpL;
//	for (int i = 0; i < numTask; i++)
//	{
//		tmpR = _numRightObject[i];
//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::STOP)
//			rightT[i + 1] = rightT[i];
//		else if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::REACH)
//			rightT[i + 1] = objectPos[tmpR][ismoved[tmpR]] * Inv(_robotManager->m_objectList[tmpR]->m_robot2objReach);
//		else if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::GRASP)
//			rightT[i + 1] = objectPos[tmpR][ismoved[tmpR]] * Inv(_robotManager->m_objectList[tmpR]->m_robot2obj);
//		else if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//			rightT[i + 1] = objectPos[tmpR][++ismoved[tmpR]] * Inv(_robotManager->m_objectList[tmpR]->m_robot2obj);
//		tmpL = _numLeftObject[i];
//		if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::STOP)
//			leftT[i + 1] = leftT[i];
//		else if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::REACH)
//			leftT[i + 1] = objectPos[tmpL][ismoved[tmpL]] * Inv(_robotManager->m_objectList[tmpL]->m_robot2objReach);
//		else if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::GRASP)
//			leftT[i + 1] = objectPos[tmpL][ismoved[tmpL]] * Inv(_robotManager->m_objectList[tmpL]->m_robot2obj);
//		else if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//			leftT[i + 1] = objectPos[tmpL][++ismoved[tmpL]] * Inv(_robotManager->m_objectList[tmpL]->m_robot2obj);
//	}
//
//	vector<SE3> tmpT(2);
//	vector<bool> tmpIncludeOri(2, true);
//	Eigen::VectorXd qtemp;
//	double error;
//	bool welldefined = true;
//	vector<SE3> checkT;
//	se3 tmpse3;
//	int flag;
//	for (int i = 0; i < numTask + 1; i++)
//	{
//		tmpT[0] = rightT[i];
//		tmpT[1] = leftT[i];
//		tmpIncludeOri[0] = rightIncludeOri[i];
//		tmpIncludeOri[1] = leftIncludeOri[i];
//		printf("%d-th task\n", i + 1);
//		if (i == 0)
//			qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag);
//		else
//			qtemp = _robotManager->inverseKin(tmpT, featureIdx, tmpIncludeOri, flag, qtemp);
//		// check inv kin sol
//		checkT = _robotManager->forwardKin(VectorToVec(qtemp));
//		error = 0;
//		for (unsigned int j = 0; j < tmpT.size(); j++)
//		{
//			tmpse3 = Log(checkT[featureIdx[j]] * Inv(tmpT[j]));
//			for (int k = 0; k < 6; k++)
//				error += tmpse3[k] * tmpse3[k];
//		}
//		
//		if (error < 100.0*INVERSEKIN_TOL)
//			welldefined &= true;
//		else
//			welldefined &= false;
//		// save data
//		if (i == 0)
//			_dualArmTaskSet[i]->_startJointVal = qtemp;
//		else if (i == numTask)
//			_dualArmTaskSet[i - 1]->_goalJointVal = qtemp;
//		else
//		{
//			_dualArmTaskSet[i - 1]->_goalJointVal = qtemp;
//			_dualArmTaskSet[i]->_startJointVal = qtemp;
//		}
//		//cout << qtemp.transpose() << endl;
//	}
//	return welldefined;
//}
//
//void SDA20DTaskManager::loadTaskInfo(string str, const int idx)
//{
//	std::ifstream fin;
//	fin.open(str);
//
//	const size_t BUFFER_SIZE = 128;
//	char buffer[BUFFER_SIZE];
//
//	// generate dual arm task
//	int rightArmTaskType, leftArmTaskType;
//	// trash
//	for (int i = 0; i < 3; i++)
//		fin >> buffer;
//	fin >> rightArmTaskType;
//	// trash
//	for (int i = 0; i < 3; i++)
//		fin >> buffer;
//	fin >> leftArmTaskType;
//
//	_dualArmTaskSet[idx] = new dualArmTask(loadTaskType(rightArmTaskType), loadTaskType(leftArmTaskType));
//
//	// set object for each task
//	// trash
//	for (int i = 0; i < 4; i++)
//		fin >> buffer;
//	fin >> _numRightObject[idx];
//	// trash
//	for (int i = 0; i < 4; i++)
//		fin >> buffer;
//	fin >> _numLeftObject[idx];
//	
//	// object index starts from 1 --> should start from 0
//	_numRightObject[idx] -= 1;
//	_numLeftObject[idx] -= 1;
//
//	if (_numRightObject[idx] < NUM_OF_OBJECTS)
//		_dualArmTaskSet[idx]->_rightObject = _robotManager->m_objectList[_numRightObject[idx]];
//	if (_numLeftObject[idx] < NUM_OF_OBJECTS)
//		_dualArmTaskSet[idx]->_leftObject = _robotManager->m_objectList[_numLeftObject[idx]];
//
//	// set object initial position
//	Vec3 tmpVec;
//	Vec3 tmpW;
//	for (int i = 0; i < NUM_OF_OBJECTS; i++)
//	{
//		for (int j = 0; j < 3; j++)
//			fin >> tmpVec[j];
//		for (int j = 0; j < 3; j++)
//			fin >> tmpW[j];
//		_objectInitPos[idx][i] = SE3(Exp(tmpW), tmpVec);
//	}
//	
//}
//
//void SDA20DTaskManager::initializeObject(int taskNum)
//{
//	//int i = taskNum;
//	//// update environment (which object is moving, which object is fixed etc.)
//	//for (int j = 0; j < NUM_OF_OBJECTS; j++)
//	//	_robotManager->getObjectList(j)->GetBaseLink()->SetFrame(_objectInitPos[i][j] * _robotManager->getObjectList(j)->m_data2ref);
//
//	//// attach object for task
//	//_robotManager->clearObjectAttachment();
//	//if (_dualArmTaskSet[i]->_type == dualArmTask::TYPE::FREE)
//	//{
//	//	if (_dualArmTaskSet[i]->_rightObject != NULL)
//	//		if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//	//			_robotManager->attachObject(&_robotManager->getRobot()->gMarkerLink[SDA20D_Index::MLINK_RIGHT_GRIPPER], _dualArmTaskSet[i]->_rightObject);
//	//	
//	//	if (_dualArmTaskSet[i]->_leftObject != NULL)
//	//		if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//	//			_robotManager->attachObject(&_robotManager->getRobot()->gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER], _dualArmTaskSet[i]->_leftObject);
//	//}
//	initializeObject(taskNum, _objectInitPos);
//	
//}
//
//void SDA20DTaskManager::initializeObject(int taskNum, vector<vector<SE3>> objInitPos)
//{
//	int i = taskNum;
//	// update environment (which object is moving, which object is fixed etc.)
//	for (int j = 0; j < NUM_OF_OBJECTS; j++)
//		_robotManager->getObjectList(j)->GetBaseLink()->SetFrame(objInitPos[i][j] * _robotManager->getObjectList(j)->m_data2ref);
//
//	// attach object for task
//	_robotManager->clearObjectAttachment();
//	if (_dualArmTaskSet[i]->_type == dualArmTask::TYPE::FREE)
//	{
//		if (_dualArmTaskSet[i]->_rightObject != NULL)
//			if (_dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_rightArmTaskType == dualArmTask::TASKTYPE::PUT)
//				_robotManager->attachObject(&_robotManager->getRobot()->gMarkerLink[SDA20D_Index::MLINK_RIGHT_GRIPPER], _dualArmTaskSet[i]->_rightObject);
//
//		if (_dualArmTaskSet[i]->_leftObject != NULL)
//			if (_dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::MOVE || _dualArmTaskSet[i]->_leftArmTaskType == dualArmTask::TASKTYPE::PUT)
//				_robotManager->attachObject(&_robotManager->getRobot()->gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER], _dualArmTaskSet[i]->_leftObject);
//	}
//}
//
//pair<vector<Eigen::VectorXd>, vector<unsigned int>> SDA20DTaskManager::extractTrj()
//{
//	vector<Eigen::VectorXd> concatenatedTrj(0);
//	vector<unsigned int> taskNum(0);
//	for (unsigned int i = 0; i < _jointTraj.size(); i++)
//	{
//		for (unsigned int j = 0; j < _jointTraj[i].first.size(); j++)
//		{
//			concatenatedTrj.push_back(_jointTraj[i].first[j]);
//			taskNum.push_back(i);
//		}
//	}
//	
//	_result.first = concatenatedTrj;
//	_result.second = taskNum;
//	return _result;
//}
//
//vector<Eigen::VectorXd> SDA20DTaskManager::expandToDualArmTrj(vector<Eigen::VectorXd>& oneArmTrj, int planningMode, Eigen::VectorXd & startPoint)
//{
//	vector<Eigen::VectorXd> newTrj(oneArmTrj.size(), Eigen::VectorXd::Zero(_robotManager->getActiveArmInfo()->m_numJoint));
//	if (planningMode == SDA20DrrtManager::RIGHTARMONLY)
//	{
//		Eigen::VectorXd augVec = _robotManager->getLeftArmJointVal(startPoint);
//		for (unsigned int j = 0; j < oneArmTrj.size(); j++)
//		{
//			newTrj[j].head(oneArmTrj[j].size()) = oneArmTrj[j];
//			newTrj[j].tail(augVec.size()) = augVec;
//		}
//	}
//	else if (planningMode == SDA20DrrtManager::LEFTARMONLY)
//	{
//		Eigen::VectorXd augVec = _robotManager->getRightArmJointVal(startPoint);
//		for (unsigned int j = 0; j < oneArmTrj.size(); j++)
//		{
//			newTrj[j].head(augVec.size()) = augVec;
//			newTrj[j].tail(oneArmTrj[j].size()) = oneArmTrj[j];
//		}
//	}
//	return newTrj;
//}
//
//void SDA20DTaskManager::saveTaskDataToText(std::string str)
//{
//	// save trj
//	string strtemp = str + "robotTrj.txt";
//	saveDataToText(_result.first, strtemp);
//	vector<Eigen::VectorXd> taskIdx;
//	Eigen::VectorXd tmp(1);
//	vector<vector<Eigen::VectorXd>> obj(NUM_OF_OBJECTS);
//	vector<int> featureIdx(4);
//	featureIdx[0] = SDA20D_Index::MLINK_RIGHT_T;
//	featureIdx[1] = SDA20D_Index::MLINK_LEFT_T;
//	featureIdx[2] = SDA20D_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx[3] = SDA20D_Index::MLINK_LEFT_GRIPPER;
//	vector<vector<Eigen::VectorXd>> robot(featureIdx.size());
//	Eigen::VectorXd tmpPosOri(12);
//	SE3 tmpSE3;
//	SE3 robotBaseSE3 = _robotManager->getRobot()->m_robotBaseFrame;
//	for (unsigned int i = 0; i < featureIdx.size(); i++)
//		robot[i].resize(0);
//	for (int i = 0; i < NUM_OF_OBJECTS; i++)
//		obj[i].resize(0);
//	vector<Eigen::VectorXd> leftGripStatusData(0);
//	Eigen::VectorXd tmpVec(2);
//	// get task idx, robot endeffector, and object trj
//	// get left grip status (for simulator only)
//	for (unsigned int i = 0; i < _result.first.size(); i++)
//	{
//		tmp(0) = _result.second[i];
//		taskIdx.push_back(tmp);
//		initializeObject(_result.second[i]);
//		_robotManager->setJointValue(VectorToVec(_result.first[i]));
//		for (unsigned int j = 0; j < featureIdx.size(); j++)
//		{
//			tmpSE3 = robotBaseSE3 % (&_robotManager->getRobot()->gMarkerLink[featureIdx[j]])->GetFrame();
//			for (int k = 0; k < 12; k++)
//			{
//				tmpPosOri(k) = tmpSE3[k];
//			}
//			robot[j].push_back(tmpPosOri);
//		}
//		for (int j = 0; j < NUM_OF_OBJECTS; j++)
//		{
//			tmpSE3 = robotBaseSE3%_robotManager->m_objectList[j]->GetBaseLink()->GetFrame();
//			for (int k = 0; k < 12; k++)
//			{
//				tmpPosOri(k) = tmpSE3[k];
//			}
//			obj[j].push_back(tmpPosOri);
//		}
//		if (_dualArmTaskSet[_result.second[i]]->_leftArmTaskType == dualArmTask::MOVE || _dualArmTaskSet[_result.second[i]]->_leftArmTaskType == dualArmTask::PUT)
//		{
//			tmpVec(0) = 1;
//			tmpVec(1) = _numLeftObject[_result.second[i]];
//		}
//		else
//		{
//			tmpVec(0) = 0;
//			tmpVec(1) = 100;
//		}
//		leftGripStatusData.push_back(tmpVec);
//	}
//	// save task idx
//	strtemp = str + "idx.txt";
//	saveDataToText(taskIdx, strtemp);
//	// save robot endeffector trj
//	strtemp = str + "robotRight.txt";
//	saveDataToText(robot[0], strtemp);
//	strtemp = str + "robotLeft.txt";
//	saveDataToText(robot[1], strtemp);
//	strtemp = str + "robotRightGripper.txt";
//	saveDataToText(robot[2], strtemp);
//	strtemp = str + "robotLeftGripper.txt";
//	saveDataToText(robot[3], strtemp);
//	// save obj trj
//	vector<Eigen::VectorXd> objinittmp(1);
//	for (int i = 0; i < NUM_OF_OBJECTS; i++)
//	{
//		strtemp = str + "obj" + to_string(i) + ".txt";
//		saveDataToText(obj[i], strtemp);
//		// save obj init pos (for simulator only)
//		objinittmp[0] = obj[i][0];
//		strtemp = str + "obj" + to_string(i) + "init.txt";
//		saveDataToText(objinittmp, strtemp);
//	}
//	// save left grip status
//	saveDataToText(leftGripStatusData, str + "gripStatus.txt");
//
//}
//
//pair<vector<Eigen::VectorXd>, vector<SE3>> SDA20DTaskManager::getJointValFromEndEffectorTrj(vector<Eigen::VectorXd>& endeffectorTrj, vector<int>& gripStatus, vector<int>& gripObject, vector<SE3>& objInitSE3)
//{
//	SE3 robotBaseFrame = _robotManager->getRobot()->m_robotBaseFrame;
//	///////////////////////// values defined w.r.t. robotBaseFrame
//	// endeffectorTrj
//	// objInitSE3
//	// TobjCur
//	///////////////////////// values defined w.r.t. simulatorFrame
//	// Tleft
//
//	// initialize object
//	vector<SE3> TobjCur = objInitSE3;
//	for (unsigned int i = 0; i < objInitSE3.size(); i++)
//		_robotManager->m_objectList[i]->GetBaseLink()->SetFrame(robotBaseFrame * objInitSE3[i]);
//	// set table location
//	Eigen::VectorXd tablePos(3);
//	for (int i = 0; i < 3; i++)
//		tablePos(i) = _robotManager->m_objectList[0]->GetBaseLink()->GetFrame().GetPosition()[i];
//	tablePos(2) -= (0.001 + _robotManager->m_table->m_tableDim[2] * 0.5 + static_cast<AlProfile*>(_robotManager->m_objectList[0])->m_AlProfileDim[2]);
//	_robotManager->m_table->setBaseLinkPosition(tablePos);
//	cout << "table pos" << tablePos << endl;
//	// get joint values from end effector trajectory
//	pair<vector<Eigen::VectorXd>, vector<SE3>> result;
//	vector<Eigen::VectorXd> qTrj(0);
//	vector<SE3> Tleft;
//	vector<SE3> T(2);
//
//	vector<bool> includeOri_(2, true);
//	vector<int> featureIdx(2);
//	vector<bool> solved(endeffectorTrj.size());
//	Eigen::VectorXd qtemp = _robotManager->generateInitialQ_initguess();
//	vector<double> jointVal(_robotManager->getActiveArmInfo()->m_numJoint);
//	for (int i = 0; i < qtemp.size(); i++)
//		jointVal[i] = qtemp(i);
//	vector<SE3> Tinit = _robotManager->forwardKin(jointVal);
//	T[0] = Tinit[SDA20D_Index::MLINK_RIGHT_GRIPPER];
//	featureIdx[0] = SDA20D_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx[1] = SDA20D_Index::MLINK_LEFT_GRIPPER;
//	SE3 Ttemp;
//	int flag;
//	bool wellsolved = true;
//	vector<int> section(0);
//	vector<tuple<int, SE3, vector<SE3>>> changePoint(0);		// index, robot2obj, TobjCur
//	
//	int s = 0;
//	SE3 robot2obj = SE3();
//	bool gripping = false;
//	SE3 Tlefttemp;
//	_robotManager->setLeftGripperValue(0.0125);
//
//
//	
//
//
//	for (unsigned int j = 0; j < endeffectorTrj.size(); j++)
//	{
//		// attach object on left gripper for planning
//		updateObjectStatus(j, endeffectorTrj, gripStatus, gripObject, TobjCur, gripping, robot2obj);
//
//		// solve inverse kinematics
//		for (int k = 0; k < 12; k++)
//			Ttemp[k] = endeffectorTrj[j][k];
//		Ttemp = _robotManager->getRobot()->m_robotBaseFrame*Ttemp;
//		T[1] = Ttemp;
//		qtemp = _robotManager->inverseKin(T, featureIdx, includeOri_, flag, qtemp);
//		_robotManager->setJointValue(VectorToVec(qtemp));
//		if (flag == SDA20DManager::invKinFlag::SOLVED && !_robotManager->checkCollision())
//			solved[j] = true;
//		else
//		{
//			solved[j] = false;
//			printf( "trj # = %d, invkin flag: %d \n", j, flag);
//			if (wellsolved)
//				wellsolved = false;
//		}
//
//		qTrj.push_back(qtemp);
//
//
//		//if (j == 200)
//		//{
//		//	for (int kk = 0; kk < 14; kk++)
//		//		cout << qtemp(kk) << endl;
//		//	for (int kk = 0; kk < 3; kk++)
//		//	{
//		//		cout << robotBaseFrame * TobjCur[kk] << endl;
//		//	}
//		//}
//
//		if (j > 0 && gripStatus[j] != gripStatus[j - 1])
//		{
//			s += 1;
//			tuple<int, SE3, vector<SE3>> tmp = make_tuple(j, robot2obj, TobjCur);
//			changePoint.push_back(tmp);
//		}
//		if (j == 0 || j == endeffectorTrj.size() - 1)
//		{
//			tuple<int, SE3, vector<SE3>> tmp = make_tuple(j, robot2obj, TobjCur);
//			changePoint.push_back(tmp);
//		}
//		section.push_back(s);
//		printf("section: %d\n", section[section.size() - 1]);
//		//cout << qtemp << endl;
//		
//		_robotManager->setJointValue(VectorToVec(qtemp));
//		printf( "%d\n",_robotManager->checkCollision());
//	}
//
//	///////////////////////
//	//wellsolved = true;
//	///////////////////////
//	bool feasiblePlanning = true;
//	if (!wellsolved)
//	{
//		vector<pair<int, int>> planIdx(0);
//		bool colli;
//		// solve for start or end point of grasp 
//		for (unsigned int i = 0; i < changePoint.size(); i++)
//		{
//			int j = get<0>(changePoint[i]);
//			for (int k = 0; k < 12; k++)
//				Ttemp[k] = endeffectorTrj[j][k];
//			Ttemp = _robotManager->getRobot()->m_robotBaseFrame*Ttemp;
//			T[1] = Ttemp;
//			if (!solved[j])
//			{
//				qTrj[j] = _robotManager->inverseKin(T, featureIdx, includeOri_, flag, _robotManager->generateInitialQ_initguess());
//				// set objects and check collision
//				for (int k = 0; k < objInitSE3.size(); k++)
//				{
//					_robotManager->m_objectList[k]->GetBaseLink()->SetFrame(robotBaseFrame * get<2>(changePoint[i])[k]);
//					//_robotManager->m_objectList[k]->GetBaseLink()->SetFrame(get<2>(changePoint[i])[k]); // temp
//				}
//					
//				_robotManager->setJointValue(VectorToVec(qTrj[j]));
//
//				
//				colli = _robotManager->checkCollision();
//				int iter = 0;
//
//				while (flag != SDA20DManager::invKinFlag::SOLVED || colli)
//				{
//					qTrj[j] = _robotManager->inverseKin(T, featureIdx, includeOri_, flag, _robotManager->generateRandomQ());
//					_robotManager->setJointValue(VectorToVec(qTrj[j]));
//					colli = _robotManager->checkCollision();
//					iter += 1;
//					if (iter == 10)
//					{
//						printf("can't go to start or end point of grasp!!!\n");
//						feasiblePlanning = false;
//						break;
//					}
//				}
//				
//				if (flag == SDA20DManager::invKinFlag::SOLVED && !colli)
//					solved[j] = true;
//			}
//		}
//
//		if (!feasiblePlanning)
//		{
//			printf("not feasible trajectory!!!\n");
//			Tleft.resize(qTrj.size());
//			for (unsigned int i = 0; i < qTrj.size(); i++)
//			{
//				T = _robotManager->forwardKin(VectorToVec(qTrj[i]));
//				Tleft[i] = T[3];
//			}
//			result.first = qTrj;
//			result.second = Tleft;
//			return result;
//		}
//
//
//		pair<int, int> tmpIdx;
//		bool counting = false;
//		vector<vector<Eigen::VectorXd>> newTrj(0);
//		
//		
//		gripping = false;
//		TobjCur = objInitSE3;
//		for (unsigned int j = 0; j < endeffectorTrj.size(); j++)
//		{	
//			// attach object on left gripper for planning
//			updateObjectStatus(j, endeffectorTrj, gripStatus, gripObject, TobjCur, gripping,robot2obj);				
//
//			if (!solved[j])
//			{
//				// save start point of planning
//				if (!counting && j > 0 && solved[j - 1])
//				{
//					tmpIdx.first = j - 1;
//					counting = true;
//				}
//			}
//
//			if (counting && solved[j])
//			{
//				// save end point of planning
//				tmpIdx.second = j;
//				planIdx.push_back(tmpIdx);
//
//				/////////////////
//				printf("%d ~ %d\n", tmpIdx.first, tmpIdx.second);
//
//				//////////////////
//				// set attachment
//				if (!gripStatus[tmpIdx.first])
//					_robotManager->clearObjectAttachment();
//
//				// planning for the points where inverse kinematics is not solved
//				Eigen::VectorXd startPoint = qTrj[tmpIdx.first];
//				Eigen::VectorXd goalPoint = qTrj[tmpIdx.second];
//				int planningMode = SDA20DrrtManager::ALL;
//				if (!_robotManager->getActiveArmInfo()->m_useWaist)
//				{
//					if ((_robotManager->getLeftArmJointVal(startPoint) - _robotManager->getLeftArmJointVal(goalPoint)).norm() < 1e-5)
//					{
//						_rrtManager->setStartandGoal(_robotManager->getRightArmJointVal(startPoint), _robotManager->getRightArmJointVal(goalPoint));
//						planningMode = SDA20DrrtManager::RIGHTARMONLY;
//						_rrtManager->setPlanningMode(planningMode);
//						cout << "plan right arm only..." << endl;
//					}
//					else if ((_robotManager->getRightArmJointVal(startPoint) - _robotManager->getRightArmJointVal(goalPoint)).norm() < 1e-5)
//					{
//						cout << "check colli" << endl;
//						cout << startPoint << endl;
//						_robotManager->setJointValue(VectorToVec(startPoint));
//						cout << _robotManager->checkCollision() << endl;
//						for (int i = 0; i < objInitSE3.size(); i++)
//							cout << _robotManager->m_objectList[i]->GetBaseLink()->GetFrame() << endl;
//
//						_robotManager->setJointValue(VectorToVec(goalPoint));
//						cout << _robotManager->checkCollision() << endl;
//						cout << goalPoint << endl;
//						_rrtManager->setStartandGoal(_robotManager->getLeftArmJointVal(startPoint), _robotManager->getLeftArmJointVal(goalPoint));
//						planningMode = SDA20DrrtManager::LEFTARMONLY;
//						_rrtManager->setPlanningMode(planningMode);
//						cout << "plan left arm only..." << endl;
//					}
//					else
//					{
//						_rrtManager->setStartandGoal(startPoint, goalPoint);
//						planningMode = SDA20DrrtManager::ALL;
//						_rrtManager->setPlanningMode(planningMode);
//					}
//				}
//
//				double step_size = 0.07;
//				int smoothingNum = 30;
//				_rrtManager->execute(step_size);
//				vector<Eigen::VectorXd> tempTrj = _rrtManager->extractPath(smoothingNum);
//				newTrj.push_back(tempTrj);
//
//				if (planningMode != SDA20DrrtManager::ALL)
//					newTrj[newTrj.size() - 1] = expandToDualArmTrj(tempTrj, planningMode, startPoint);
//				counting = false;
//			}
//		}
//
//		// insert planned trajectories
//		vector<Eigen::VectorXd> qTrjCopy = qTrj;
//		vector<int> gripStatusCopy = gripStatus;
//		vector<int> gripObjectCopy = gripObject;
//		qTrj.resize(0);
//		gripStatus.resize(0);
//		gripObject.resize(0);
//		int newIdx = 0;
//		for (unsigned int i = 0; i < qTrjCopy.size(); i++)
//		{
//			if ((newIdx == 0 && i < planIdx[newIdx].first) || i < planIdx[newIdx].first && i > planIdx[newIdx - 1].second)
//			{
//				qTrj.push_back(qTrjCopy[i]);
//				gripStatus.push_back(gripStatusCopy[i]);
//				gripObject.push_back(gripObjectCopy[i]);
//			}
//			else if (i == planIdx[newIdx].first)
//			{
//				for (int j = 0; j < newTrj[newIdx].size(); j++)
//				{
//					qTrj.push_back(newTrj[newIdx][j]);
//					gripStatus.push_back(gripStatusCopy[i]);
//					gripObject.push_back(gripObjectCopy[i]);
//				}
//				if (section[planIdx[newIdx].first] != section[planIdx[newIdx].second])
//				{
//					gripStatus[gripStatus.size() - 1] = gripStatusCopy[planIdx[newIdx].second];
//					gripObject[gripStatus.size() - 1] = gripObjectCopy[planIdx[newIdx].second];
//
//				}
//				newIdx += 1;
//			}
//		}
//	}
//
//	Tleft.resize(qTrj.size());
//	for (unsigned int i = 0; i < qTrj.size(); i++)
//	{
//		T = _robotManager->forwardKin(VectorToVec(qTrj[i]));
//		Tleft[i] = T[3];
//	}
//	result.first = qTrj;
//	result.second = Tleft;
//	return result;
//}
//
//pair<vector<Eigen::VectorXd>, vector<SE3>> SDA20DTaskManager::getJointValFromEndEffectorTrj2(vector<Eigen::VectorXd>& endeffectorTrj)
//{
//	SE3 robotBaseFrame = _robotManager->getRobot()->m_robotBaseFrame;
//	///////////////////////// values defined w.r.t. robotBaseFrame
//	// endeffectorTrj
//	///////////////////////// values defined w.r.t. simulatorFrame
//	// Tleft
//
//
//	// get joint values from end effector trajectory
//	pair<vector<Eigen::VectorXd>, vector<SE3>> result;
//	vector<Eigen::VectorXd> qTrj(0);
//	vector<SE3> Tleft;
//	vector<SE3> T(2);
//
//	vector<bool> includeOri_(2, true);
//	vector<int> featureIdx(2);
//	vector<bool> solved(endeffectorTrj.size());
//	Eigen::VectorXd qtemp = _robotManager->generateInitialQ_initguess();
//	vector<double> jointVal(_robotManager->getActiveArmInfo()->m_numJoint);
//	for (int i = 0; i < qtemp.size(); i++)
//		jointVal[i] = qtemp(i);
//	vector<SE3> Tinit = _robotManager->forwardKin(jointVal);
//	T[0] = Tinit[SDA20D_Index::MLINK_RIGHT_GRIPPER];
//	featureIdx[0] = SDA20D_Index::MLINK_RIGHT_GRIPPER;
//	featureIdx[1] = SDA20D_Index::MLINK_LEFT_GRIPPER;
//	SE3 Ttemp;
//	int flag;
//	bool wellsolved = true;
//	vector<int> section(0);
//	vector<tuple<int, SE3, vector<SE3>>> changePoint(0);		// index, robot2obj, TobjCur
//
//	int s = 0;
//	SE3 robot2obj = SE3();
//	bool gripping = false;
//	SE3 Tlefttemp;
//	_robotManager->setLeftGripperValue(0.0125);
//
//
//	for (unsigned int j = 0; j < endeffectorTrj.size(); j++)
//	{
//		// solve inverse kinematics
//		for (int k = 0; k < 12; k++)
//			Ttemp[k] = endeffectorTrj[j][k];
//		Ttemp = _robotManager->getRobot()->m_robotBaseFrame*Ttemp;
//		T[1] = Ttemp;
//		qtemp = _robotManager->inverseKin2(T, featureIdx, includeOri_, flag, qtemp);
//		_robotManager->setJointValue(VectorToVec(qtemp));
//		if (flag == SDA20DManager::invKinFlag::SOLVED && !_robotManager->checkCollision())
//			solved[j] = true;
//		else
//		{
//			solved[j] = false;
//			printf("trj # = %d, flag: %d\n", j, flag);
//			if (wellsolved)
//				wellsolved = false;
//		}
//
//		qTrj.push_back(qtemp);
//	}
//
//	Tleft.resize(qTrj.size());
//	for (unsigned int i = 0; i < qTrj.size(); i++)
//	{
//		T = _robotManager->forwardKin(VectorToVec(qTrj[i]));
//		Tleft[i] = T[3];
//	}
//	result.first = qTrj;
//	result.second = Tleft;
//	return result;
//}
//
//void SDA20DTaskManager::updateObjectStatus(int j, vector<Eigen::VectorXd>& endeffectorTrj, vector<int>& gripStatus, vector<int>& gripObject, vector<SE3>& TobjCur, bool& gripping, SE3& robot2obj)
//{
//	// update object status one step (attach object on left gripper for planning)
//	SE3 robotBaseFrame = _robotManager->getRobot()->m_robotBaseFrame;
//	SE3 Tlefttemp;
//	if (gripStatus[j])
//	{
//		for (int k = 0; k < 12; k++)
//			Tlefttemp[k] = endeffectorTrj[j][k];
//		if (!gripping)
//		{
//			_robotManager->clearObjectAttachment();
//			robot2obj = Tlefttemp % TobjCur[gripObject[j]];
//			cout <<"robot2obj: " << robot2obj << endl;
//			//robot2obj = (_robotManager->getRobot()->m_robotBaseFrame * Tlefttemp) % TobjCur[gripObject[j]]; // temp
//			_robotManager->attachObject(&_robotManager->getRobot()->gMarkerLink[SDA20D_Index::MLINK_LEFT_GRIPPER], _robotManager->m_objectList[gripObject[j]], robot2obj);
//		}
//		TobjCur[gripObject[j]] = Tlefttemp * robot2obj; 
//		//TobjCur[gripObject[j]] = _robotManager->getRobot()->m_robotBaseFrame * Tlefttemp * robot2obj; // temp
//		gripping = true;
//	}
//	else
//	{
//		if (gripping)
//			_robotManager->clearObjectAttachment();
//		gripping = false;
//	}
//	for (int k = 0; k < TobjCur.size(); k++)
//	{
//		_robotManager->m_objectList[k]->GetBaseLink()->SetFrame(robotBaseFrame * TobjCur[k]);
//		//_robotManager->m_objectList[k]->GetBaseLink()->SetFrame(get<2>(changePoint[i])[k]); // temp
//	}
//
//
//
//}
//
//dualArmTask::dualArmTask(TYPE type, TASKTYPE rightArmTaskType, TASKTYPE leftArmTaskType)
//{
//	_type = type;
//	_rightArmTaskType = rightArmTaskType;
//	_leftArmTaskType = leftArmTaskType;
//	_leftObject = NULL;
//	_rightObject = NULL;
//}
//
//dualArmTask::dualArmTask(TASKTYPE rightArmTaskType, TASKTYPE leftArmTaskType)
//{
//	_type = TYPE::FREE;
//	_rightArmTaskType = rightArmTaskType;
//	_leftArmTaskType = leftArmTaskType;
//	_leftObject = NULL;
//	_rightObject = NULL;
//}
