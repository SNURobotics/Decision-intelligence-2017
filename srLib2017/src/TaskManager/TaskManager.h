#pragma once
//#include "SDA20DManager.h"
//#include "SDA20DManager_P.h"
//
//class dualArmTask
//{
//public:
//
//	enum TASKTYPE { REACH = 0, GRASP = 1, MOVE = 2, PUT = 3, STOP = 4 };
//	enum TYPE { FREE, CONSTRAINED };
//
//	dualArmTask(TYPE type, TASKTYPE rightArmTaskType, TASKTYPE leftArmTaskType);
//	dualArmTask(TASKTYPE rightArmTaskType, TASKTYPE leftArmTaskType);
//	~dualArmTask() {};
//
//	TASKTYPE						_leftArmTaskType;
//	TASKTYPE						_rightArmTaskType;
//	TaskObject*						_leftObject;
//	TaskObject*						_rightObject;
//	Eigen::VectorXd					_startJointVal;
//	Eigen::VectorXd					_goalJointVal;
//	TYPE							_type;
//};
//
//class SDA20DTaskManager
//{
//public:
//	SDA20DTaskManager(int ch) { _robotManager = new SDA20DManager(ch); _robotManager_p = new SDA20DManager_P(ch); _rrtManager = new SDA20DrrtManager(_robotManager); _dualArmTaskSet.resize(0); };
//	~SDA20DTaskManager() {};
//
//	void																	loadTaskData(string str);
//	void																	loadTaskInfo(string str, const int idx);
//	dualArmTask::TASKTYPE													loadTaskType(int taskType);
//
//	bool																	feasibilityCheck(int taskNum);
//	bool																	feasibilityCheck();
//	bool																	feasibilityCheck_fcl(int taskNum);
//	void																	runTask(string str, bool doPlanning);
//	pair<vector<Eigen::VectorXd>, vector<double>>							runTask(string str, int taskNum, bool doPlanning);
//	bool																	runPlanningTask();
//	bool																	runPlanningTask2();
//	bool																	runPlanningTask(int taskNum, pair<vector<Eigen::VectorXd>, vector<double>>& robotTraj);
//	bool																	setTaskData(bool loadProblem = false, int probNum = 0);				// set default task
//	
//	void																	initializeObject(int taskNum);			// set object initial position, attach object to robot
//	void																	initializeObject(int taskNum, vector<vector<SE3>> objInitPos);			// set object initial position, attach object to robot
//	pair<vector<Eigen::VectorXd>, vector<unsigned int>>						extractTrj();
//
//	vector<Eigen::VectorXd>													expandToDualArmTrj(vector<Eigen::VectorXd>& oneArmTrj, int planningMode, Eigen::VectorXd& startPoint);
//	void																	saveTaskDataToText(std::string str);
//
//	// convert endeffector trajectory to joint trajectory
//	pair<vector<Eigen::VectorXd>, vector<SE3>>								getJointValFromEndEffectorTrj(vector<Eigen::VectorXd>& endeffectorTrj, vector<int>& gripStatus, vector<int>& gripObject, vector<SE3>& objInitSE3);
//	pair<vector<Eigen::VectorXd>, vector<SE3>>								getJointValFromEndEffectorTrj2(vector<Eigen::VectorXd>& endeffectorTrj);		// use normal inv kin
//	void																	updateObjectStatus(int j, vector<Eigen::VectorXd>& endeffectorTrj, vector<int>& gripStatus, vector<int>& gripObject, vector<SE3>& TobjCur, bool& gripping, SE3& robot2obj);
//
//
//	SDA20DManager*															_robotManager;
//	SDA20DManager_P*														_robotManager_p;
//	SDA20DrrtManager*														_rrtManager;
//	vector<dualArmTask*>													_dualArmTaskSet;
//	vector<vector<SE3>>														_objectInitPos;				// first vector: task number, second vector: object number
//	vector<int>																_numRightObject;			// object number of right arm
//	vector<int>																_numLeftObject;				// object number of left arm
//	vector<Eigen::MatrixXd>													_markerTrajBC;				// set of initial and final posture of each task
//	vector<Eigen::MatrixXd>													_objectTrajBC;				// set of initial and final object of each task
//	vector<pair<vector<Eigen::VectorXd>, vector<double>>>					_jointTraj;					// first vector: task number, second vector: time order
//	pair<vector<Eigen::VectorXd>, vector<unsigned int>>						_result;
//	vector<double>															_planningTime;
//};