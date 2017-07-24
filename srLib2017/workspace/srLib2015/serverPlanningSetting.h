#pragma once

#include "simulationEnvSetting.h"
#include "robotManager\robotRRTManager.h"

robotRRTManager* RRTManager1 = new robotRRTManager;
robotRRTManager* RRTManager2 = new robotRRTManager;
vector<robotRRTManager*> RRTManagerVector(2);
robotRRTManager* RRTManager_twoArm = new robotRRTManager;

// Planning
vector<vector<vector<Eigen::VectorXd>>> renderTraj_multi(2);
vector<vector<Eigen::VectorXd>> renderTraj_twoArm;
vector<vector<Eigen::VectorXd>> qWaypoint(2);
vector<Eigen::VectorXd> qWaypoint_twoArm(0);
vector<Eigen::VectorXd> initPos(0);
vector<Eigen::VectorXd> goalPos(0);

// save last gripper state
int gripState = 0;
vector<int> gripState_multi(2);
vector<int> gripObjectIdx(2, -1);			// save which busbar is moving with each robot during planning
vector<Eigen::VectorXd> lastJointVal_multi(2);// save last joint value
Eigen::VectorXd lastJointVal_multi_twoArm(12);

// save initial and final busbar SE(3)
vector<SE3> TinitObjects_multi(2);
vector<bool> initialObjectSaved(2, false);			// save if initial busbar location is saved (saved when busbar is moving)
vector<SE3>	TlastObjects_multi(2);


// RRT multi-robot
void rrtSetting();
void RRT_problemSettingFromSingleRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag, int robotFlag);
void RRT_problemSetting_SingleRobot(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag, int robotFlag);
void RRTSolve_HYU_SingleRobot(vector<bool> attachObject, vector<double> stepsize, int robotFlag);
// (Two arm simultaneously)
void RRT_problemSettingFromMultiRobotCommand(const vector<desired_dataset> & hyu_desired_dataset, vector<vector<bool>>& attachObject, Eigen::VectorXd init, vector<vector<bool>>& waypointFlag);
void RRT_problemSetting_MultiRobot(Eigen::VectorXd init, vector<vector<SE3>> wayPoints, vector<vector<bool>> includeOri, vector<vector<bool>> attachObject, vector<vector<bool>>& waypointFlag);
void RRTSolve_HYU_multiRobot(vector<vector<bool>> attachObject, vector<double> stepsize);


int getObjectIdx(int robotIdx);
vector<Eigen::VectorXd> calculateJointTorque(vector<vector<Eigen::VectorXd>>& traj, int robotFlag);
bool saveTrajectories = true;
void savePlannedResultToText(unsigned int robotFlag, vector<vector<Eigen::VectorXd>>& traj, vector<vector<bool>>& attachObjectVec);


void rrtSetting()
{
	vector<srStateJoint*> planningJoints1(6);
	vector<srStateJoint*> planningJoints2(6);
	vector<srStateJoint*> planningJoints_twoArm(12);
	for (unsigned int i = 0; i <DEGREE_OF_FREEDOM_INDY_JOINT; i++)
	{
		planningJoints1[i] = (srStateJoint*)robot1->gJoint[i];
		planningJoints2[i] = (srStateJoint*)robot2->gJoint[i];

		planningJoints_twoArm[i] = (srStateJoint*)robot1->gJoint[i];
		planningJoints_twoArm[i + 6] = (srStateJoint*)robot2->gJoint[i];
	}


	RRTManager1->setSystem(planningJoints1);
	RRTManager1->setSpace(&gSpace);
	RRTManager1->setStateBound(robot1->getLowerJointLimit(), robot1->getUpperJointLimit());
	RRTManager2->setSystem(planningJoints2);
	RRTManager2->setSpace(&gSpace);
	RRTManager2->setStateBound(robot2->getLowerJointLimit(), robot2->getUpperJointLimit());

	RRTManager_twoArm->setSystem(planningJoints_twoArm);
	RRTManager_twoArm->setSpace(&gSpace);

	Eigen::VectorXd LowerJointLimit_twoArm(12);
	Eigen::VectorXd UpperJointLimit_twoArm(12);



	Eigen::VectorXd LowerJointLimitRobot1 = robot1->getLowerJointLimit();
	Eigen::VectorXd LowerJointLimitRobot2 = robot2->getLowerJointLimit();
	Eigen::VectorXd UpperJointLimitRobot1 = robot1->getUpperJointLimit();
	Eigen::VectorXd UpperJointLimitRobot2 = robot2->getUpperJointLimit();

	for (int i = 0; i <DEGREE_OF_FREEDOM_INDY_JOINT; i++) {
		LowerJointLimit_twoArm(i) = LowerJointLimitRobot1[i];
		LowerJointLimit_twoArm(i + DEGREE_OF_FREEDOM_INDY_JOINT) = LowerJointLimitRobot2[i];
		UpperJointLimit_twoArm(i) = UpperJointLimitRobot1[i];
		UpperJointLimit_twoArm(i + DEGREE_OF_FREEDOM_INDY_JOINT) = UpperJointLimitRobot2[i];
	}
	RRTManager_twoArm->setStateBound(LowerJointLimit_twoArm, UpperJointLimit_twoArm);


	RRTManagerVector[0] = RRTManager1;
	RRTManagerVector[1] = RRTManager2;
}

void RRT_problemSettingFromSingleRobotCommand(const desired_dataset & hyu_desired_dataset, vector<bool>& attachObject, Eigen::VectorXd init, vector<bool>& waypointFlag, int robotFlag)
{
	unsigned int nWay = hyu_desired_dataset.robot_pos.size() / 3;
	attachObject.resize(nWay);
	vector<bool> includeOri(nWay, true);
	vector<SE3> wayPoints(nWay);
	vector<double> pos(3);
	vector<double> ori(9);
	for (unsigned int i = 0; i < nWay; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			pos[j] = hyu_desired_dataset.robot_pos[i * 3 + j];
			for (int k = 0; k < 3; k++)
				ori[3 * j + k] = hyu_desired_dataset.robot_rot[3 * j + k + 9 * i];
		}
		wayPoints[i] = SKKUtoSE3(ori, pos);
		if (abs(hyu_desired_dataset.robot_gripper[i] - 1.0) < DBL_EPSILON)
			attachObject[i] = true;
		else
			attachObject[i] = false;
	}

	RRT_problemSetting_SingleRobot(init, wayPoints, includeOri, attachObject, waypointFlag, robotFlag);
}

void RRT_problemSetting_SingleRobot(Eigen::VectorXd init, vector<SE3> wayPoints, vector<bool> includeOri, vector<bool> attachObject, vector<bool>& waypointFlag, int robotFlag)
{
	int flag;
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);
	if (attachObject[0] && gripObjectIdx[robotFlag - 1] != -1)
		RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
	else
		RRTManagerVector[robotFlag - 1]->detachObject();
	bool feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(init);
	if (feas != 0)
		printf("initial point not feasible!!!\n");
	Eigen::VectorXd qtemp;
	waypointFlag.resize(wayPoints.size());
	qWaypoint[robotFlag - 1].resize(0);
	for (unsigned int i = 0; i < wayPoints.size(); i++)
	{
		qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, robotVector[robotFlag - 1]->qInvKinInit/*, 500, robotManager::invKinAlg::NR, robotManager::invKinMet::LOG*/);
		//if (flag != 0)
		//	qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, qInit);
		if (flag != 0)
			qtemp = rManagerVector[robotFlag - 1]->inverseKin(TrobotbaseVector[0] * wayPoints[i], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[i], SE3(), flag, initPos[initPos.size() - 1]);
		printf("%d-th init inv kin flag: %d\n", i, flag);

		if (attachObject[i] && gripObjectIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();

		//cout << TrobotbaseVector[0] * wayPoints[i] << endl;
		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(qtemp);
		qWaypoint[robotFlag - 1].push_back(qtemp);
		if (feas == 0 && flag == 0)
		{
			waypointFlag[i] = true;
			goalPos.push_back(qtemp);
			if (i < wayPoints.size() - 1)
				initPos.push_back(goalPos[goalPos.size() - 1]);
		}
		else
		{
			waypointFlag[i] = false;
			if (i == wayPoints.size() - 1)
			{
				if (feas != 0)
					printf("final waypoint not feasible!!! (collision)\n");
				else
					printf("final waypoint not feasible!!! (inversekin)\n");
			}
			else
			{
				if (feas != 0)
					printf("%d-th waypoint not feasible!!! (collision)\n", i + 1);
				else
					printf("%d-th waypoint not feasible!!! (inversekin)\n", i + 1);
			}
			if (i > 0 && attachObject[i] != attachObject[i - 1])
				printf("grasp point is not feasible!!!\n");
		}
	}
}


void RRTSolve_HYU_SingleRobot(vector<bool> attachObject, vector<double> stepsize, int robotFlag)
{
	int nDim = 6;
	vector<Eigen::VectorXd> tempTraj;
	//vector<int> tempIdxTraj(0);
	//vector<SE3> tempTtraj(0);
	int start = 0;
	int end = goalPos.size();
	vector<bool> feas(2);
	vector<vector<Eigen::VectorXd>> traj(0);
	//Ttraj.resize(0);
	//idxTraj.resize(0);
	printf("waypoints: \n");
	for (unsigned int i = 0; i < initPos.size(); i++)
		cout << initPos[i].transpose() << endl;
	if (goalPos.size() > 0)
		cout << goalPos[goalPos.size() - 1].transpose() << endl;
	initialObjectSaved[robotFlag - 1] = false;
	for (int i = start; i < end; i++)
	{
		RRTManagerVector[robotFlag - 1]->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		if (attachObject[i] && gripObjectIdx[robotFlag - 1] != -1)
			RRTManagerVector[robotFlag - 1]->attachObject(objects[gripObjectIdx[robotFlag - 1]], &robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotFlag - 1]]));
		else
			RRTManagerVector[robotFlag - 1]->detachObject();


		feas = RRTManagerVector[robotFlag - 1]->checkFeasibility(initPos[i], goalPos[i]);
		cout << feas[0] << feas[1] << endl;
		/////////////////////
		if ((initPos[i] - goalPos[i]).norm() < 0.2)
			stepsize[i] = 0.01;
		/////////////////////
		RRTManagerVector[robotFlag - 1]->execute(stepsize[i]);
		tempTraj = RRTManagerVector[robotFlag - 1]->extractPath(40);

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManagerVector[robotFlag - 1]->setState(tempTraj[j]))
				printf("collide at %d-th trj, %d-th point!!!\n", i, j);

		traj.push_back(tempTraj);


		if (gripObjectIdx[robotFlag - 1] != -1)
		{
			// set busbar final location
			// if busbar is attached, busbar will be located at last location. 
			// otherwise, busbar will not move, initial and final location will be the same.
			RRTManagerVector[robotFlag - 1]->setState(tempTraj[tempTraj.size() - 1]);
			TlastObjects_multi[robotFlag - 1] = objects[gripObjectIdx[robotFlag - 1]]->GetBaseLink()->GetFrame();
			cout << "TlastBusbar" << endl;
			cout << Trobotbase1 % TlastObjects_multi[robotFlag - 1] << endl;
			if (attachObject[i])
			{
				// set busbar initial location
				if (!initialObjectSaved[robotFlag - 1])
				{
					initialObjectSaved[robotFlag - 1] = true;
					RRTManagerVector[robotFlag - 1]->setState(tempTraj[0]);
					TinitObjects_multi[robotFlag - 1] = objects[gripObjectIdx[robotFlag - 1]]->GetBaseLink()->GetFrame();
					cout << "TinitBusbar" << endl;
					cout << Trobotbase1 % TinitObjects_multi[robotFlag - 1] << endl;
				}
			}
		}

		//tempTtraj.resize(tempTraj.size());
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
	renderTraj_multi[robotFlag - 1] = traj;
	// save last joint val
	if (goalPos.size() > 0)
		lastJointVal_multi[robotFlag - 1] = traj[traj.size() - 1][traj[traj.size() - 1].size() - 1];
	else
		lastJointVal_multi[robotFlag - 1] = initPos[0];

	// save planned result to text file
	if (saveTrajectories)
	{
		vector<vector<bool>> attachObjectVec(0);
		attachObjectVec.push_back(attachObject);
		savePlannedResultToText(robotFlag, traj, attachObjectVec);
	}
}


void RRT_problemSettingFromMultiRobotCommand(const vector<desired_dataset> & hyu_desired_dataset, vector<vector<bool>>& attachObject, Eigen::VectorXd init, vector<vector<bool>>& waypointFlag)
{
	vector<int> nWayVector(2);
	vector<vector<SE3>> wayPoints(2);
	vector<vector<bool>> includeOri(2);
	vector<double> pos(3);
	vector<double> ori(9);

	for (int i = 0; i < 2; i++)
	{
		nWayVector[i] = hyu_desired_dataset[i].robot_pos.size() / 3;
		attachObject[i].resize(nWayVector[i]);
		wayPoints[i].resize(nWayVector[i]);
		includeOri[i].resize(nWayVector[i]);
		for (int j = 0; j < nWayVector[i]; j++)
			includeOri[i][j] = true;
	}

	for (int robotnum = 0; robotnum < 2; robotnum++)
	{
		for (int i = 0; i < nWayVector[robotnum]; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				pos[j] = hyu_desired_dataset[robotnum].robot_pos[i * 3 + j];
				for (int k = 0; k < 3; k++)
					ori[3 * j + k] = hyu_desired_dataset[robotnum].robot_rot[3 * j + k + 9 * i];
			}
			wayPoints[robotnum][i] = SKKUtoSE3(ori, pos);
			if (abs(hyu_desired_dataset[robotnum].robot_gripper[i] - 1.0) < DBL_EPSILON)
				attachObject[robotnum][i] = true;
			else
				attachObject[robotnum][i] = false;
		}
	}


	RRT_problemSetting_MultiRobot(init, wayPoints, includeOri, attachObject, waypointFlag);
}

void RRT_problemSetting_MultiRobot(Eigen::VectorXd init, vector<vector<SE3>> wayPoints, vector<vector<bool>> includeOri, vector<vector<bool>> attachObject, vector<vector<bool>>& waypointFlag)
{


	vector<int> flag(2);
	initPos.resize(0);
	goalPos.resize(0);
	initPos.push_back(init);

	for (int robotnum = 0; robotnum < 2; robotnum++)
	{
		if (attachObject[robotnum][0] && gripObjectIdx[robotnum] != -1)
			RRTManager_twoArm->attachObject(objects[gripObjectIdx[robotnum]], &robotVector[robotnum]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotnum]]));
		else
			RRTManager_twoArm->detachObject();
	}
	bool feas = RRTManager_twoArm->checkFeasibility(init);
	if (feas != 0)
		printf("initial point not feasible!!!\n");
	Eigen::VectorXd qtemp;
	vector<Eigen::VectorXd> qtemp_each_robot(2);
	for (int robotnum = 0; robotnum < 2; robotnum++)
		waypointFlag[robotnum].resize(wayPoints[robotnum].size());
	qWaypoint_twoArm.resize(0);
	for (unsigned int i = 0; i < wayPoints[0].size(); i++) // since the number of two waypoints is the same
	{
		for (unsigned int robotnum = 0; robotnum < 2; robotnum++)
			qtemp_each_robot[robotnum] = rManagerVector[robotnum]->inverseKin(TrobotbaseVector[0] * wayPoints[robotnum][i], &robotVector[robotnum]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[robotnum][i], SE3(), flag[robotnum], robotVector[robotnum]->qInvKinInit);
		if (flag[0] != 0)
			qtemp_each_robot[0] = rManagerVector[0]->inverseKin(TrobotbaseVector[0] * wayPoints[0][i], &robotVector[0]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[0][i], SE3(), flag[0], initPos[initPos.size() - 1].segment(0, 6));
		else if (flag[1] != 0)
			qtemp_each_robot[1] = rManagerVector[1]->inverseKin(TrobotbaseVector[0] * wayPoints[1][i], &robotVector[1]->gMarkerLink[Indy_Index::MLINK_GRIP], includeOri[1][i], SE3(), flag[1], initPos[initPos.size() - 1].segment(6, 6));
		printf("Two robot's %d-th init inv kin flag: %d, %d\n", i, flag[0], flag[1]);
		qtemp = concatenateVec6(qtemp_each_robot[0], qtemp_each_robot[1]);

		for (int robotnum = 0; robotnum < 2; robotnum++)
		{
			if (attachObject[robotnum][i] && gripObjectIdx[robotnum] != -1)
				RRTManager_twoArm->attachObject(objects[gripObjectIdx[robotnum]], &robotVector[robotnum]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotnum]]));
			else
				RRTManager_twoArm->detachObject();
		}

		feas = RRTManager_twoArm->checkFeasibility(qtemp);
		qWaypoint_twoArm.push_back(qtemp);

		if (feas == 0)
		{
			if (flag[0] == 0 && flag[1] == 0)
			{
				waypointFlag[0][i] = true;
				waypointFlag[1][i] = true;
				goalPos.push_back(qtemp);
				if (i < wayPoints[0].size() - 1)
					initPos.push_back(goalPos[goalPos.size() - 1]);
			}
			else if (flag[0] == 0 && flag[1] != 0)
			{
				waypointFlag[0][i] = true;
				waypointFlag[1][i] = false;
			}
			else if (flag[0] == 0 && flag[1] != 0)
			{
				waypointFlag[0][i] = false;
				waypointFlag[1][i] = true;
			}
		}
		else
		{
			waypointFlag[0][i] = false;
			waypointFlag[1][i] = false;
		}
	}
}


void RRTSolve_HYU_multiRobot(vector<vector<bool>> attachObject, vector<double> stepsize)
{
	int nDim = 12;
	vector<Eigen::VectorXd> tempTraj;
	//vector<int> tempIdxTraj(0);
	//vector<SE3> tempTtraj(0);
	int start = 0;
	int end = goalPos.size();
	vector<bool> feas(2);


	vector<vector<Eigen::VectorXd>> traj(0);
	//Ttraj.resize(0);
	//idxTraj.resize(0);
	printf("waypoints: \n");
	for (unsigned int i = 0; i < initPos.size(); i++)
		cout << initPos[i].transpose() << endl;
	if (goalPos.size() > 0)
		cout << goalPos[goalPos.size() - 1].transpose() << endl;

	for (int robotnum = 0; robotnum < 2; robotnum++)
		initialObjectSaved[robotnum] = false;


	for (int i = start; i < end; i++)
	{
		RRTManager_twoArm->setStartandGoal(initPos[i], goalPos[i]);

		cout << "initpos:  " << initPos[i].transpose() << endl;
		cout << "goalPos:  " << goalPos[i].transpose() << endl << endl;;

		for (int robotnum = 0; robotnum < 2; robotnum++)
		{
			if (attachObject[robotnum][i] && gripObjectIdx[robotnum] != -1)
				RRTManager_twoArm->attachObject(objects[gripObjectIdx[robotnum]], &robotVector[robotnum]->gMarkerLink[Indy_Index::MLINK_GRIP], Inv(Tobject2gripper[gripObjectIdx[robotnum]]));
			else
				RRTManager_twoArm->detachObject();
		}

		feas = RRTManager_twoArm->checkFeasibility(initPos[i], goalPos[i]);

		cout << feas[0] << feas[1] << endl;
		/////////////////////
		if ((initPos[i] - goalPos[i]).norm() < 0.2)
			stepsize[i] = 0.01;
		/////////////////////
		RRTManager_twoArm->execute(stepsize[i]);
		tempTraj = RRTManager_twoArm->extractPath(40);

		// check collision
		for (unsigned int j = 0; j < tempTraj.size(); j++)
			if (RRTManager_twoArm->setState(tempTraj[j]))
				printf("collide at %d-th trj, %d-th point!!!\n", i, j);

		traj.push_back(tempTraj);


		for (int robotnum = 0; robotnum < 2; robotnum++)
		{
			if (gripObjectIdx[robotnum] != -1)
			{
				// set busbar final location
				// if busbar is attached, busbar will be located at last location. 
				// otherwise, busbar will not move, initial and final location will be the same.
				RRTManager_twoArm->setState(tempTraj[tempTraj.size() - 1]);
				TlastObjects_multi[robotnum] = objects[gripObjectIdx[robotnum]]->GetBaseLink()->GetFrame();
				cout << "TlastBusbar" << endl;
				cout << Trobotbase1 % TlastObjects_multi[robotnum] << endl;
				if (attachObject[robotnum][i])
				{
					// set busbar initial location
					if (!initialObjectSaved[robotnum])
					{
						initialObjectSaved[robotnum] = true;
						RRTManager_twoArm->setState(tempTraj[0]);
						TinitObjects_multi[robotnum] = objects[gripObjectIdx[robotnum]]->GetBaseLink()->GetFrame();
						cout << "TinitBusbar" << endl;
						cout << Trobotbase1 % TinitObjects_multi[robotnum] << endl;
					}
				}
			}
		}


		//tempTtraj.resize(tempTraj.size());
		//for (unsigned int j = 0; j < traj[i].size(); j++)
		//	tempTtraj[j] = rManager1->forwardKin(traj[i][j], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
		//Ttraj.push_back(tempTtraj);
	}
	renderTraj_twoArm = traj;
	// save last joint val
	if (goalPos.size() > 0)
	{
		lastJointVal_multi_twoArm = traj[traj.size() - 1][traj[traj.size() - 1].size() - 1];
		lastJointVal_multi[0] = lastJointVal_multi_twoArm.segment(0, 6);
		lastJointVal_multi[1] = lastJointVal_multi_twoArm.segment(6, 6);
	}
	else
	{
		lastJointVal_multi_twoArm = initPos[0];
		lastJointVal_multi[0] = initPos[0].segment(0, 6);
		lastJointVal_multi[1] = initPos[0].segment(6, 6);
	}
		

	// save joint trajectory and object trajectories to text
	if (saveTrajectories)
	{
		vector<vector<bool>> attachObjectVec;
		attachObjectVec = attachObject;
		savePlannedResultToText(3, traj, attachObjectVec);
	}

}

int getObjectIdx(int robotIdx)
{
	// robotIdx = 1: robot1, robotIdx = 2: robot2
	if (robotIdx != 1 && robotIdx != 2)
		return -1;
	double mindist = 100.0;
	unsigned int minIdx = 100;
	double tempdist = 100.0;
	for (unsigned int i = 0; i < objects.size(); i++)
	{
		tempdist = Norm(robotVector[robotIdx - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame().GetPosition() - (objects[i]->GetBaseLink()->GetFrame()*Tobject2gripper[i]).GetPosition());
		if (tempdist < mindist)
		{
			mindist = tempdist;
			minIdx = i;
		}
	}
	if (minIdx == -1 || mindist > 0.01)
		printf("check if robot can grip object\n");
	return minIdx;
}

vector<Eigen::VectorXd> calculateJointTorque(vector<vector<Eigen::VectorXd>>& traj, int robotFlag)
{
	vector<Eigen::VectorXd> qTrj(0);
	if (robotFlag != 1 && robotFlag != 2)
		return qTrj;
	else
	{
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
			{
				if (qTrj.size() > 0 && (traj[i][j] - qTrj[qTrj.size() - 1]).norm() > 1.0e-5)
					qTrj.push_back(traj[i][j]);
			}
		}
		vector<double> dt(qTrj.size());
		vector<Eigen::VectorXd> vTrj(qTrj.size());
		vector<Eigen::VectorXd> aTrj(qTrj.size());
		for (unsigned int i = 0; i < qTrj.size() - 1; i++)
		{
			dt[i] = (qTrj[i + 1] - qTrj[i]).cwiseAbs().sum() * 10.0;
			vTrj[i] = (qTrj[i + 1] - qTrj[i]) / dt[i];
		}
		for (unsigned int i = 0; i < qTrj.size() - 1; i++)
			aTrj[i] = (vTrj[i + 1] - vTrj[i]) / dt[i];
		vector<Eigen::VectorXd> tauTrj(qTrj.size() - 1);
		for (unsigned int i = 0; i < qTrj.size() - 1; i++)
			tauTrj[i] = rManagerVector[robotFlag - 1]->inverseDyn(qTrj[i], vTrj[i], aTrj[i]);
		return tauTrj;
	}
}

void savePlannedResultToText(unsigned int robotFlag, vector<vector<Eigen::VectorXd>>& traj, vector<vector<bool>>& attachObjectVec)
{
	// attachObject: first vector - robotnum, second vector - tasknum
	// save joint trajectory and object trajectories to text
	vector<Eigen::VectorXd> saveTraj(0);
	string loc = "../../../data/render_traj/";
	if (robotFlag == 1 || robotFlag == 2)
	{
		// single robot case
		saveTraj.resize(0);
		for (unsigned int i = 0; i < traj.size(); i++)
		{
			for (unsigned int j = 0; j < traj[i].size(); j++)
			{
				saveTraj.push_back(traj[i][j]);
			}
		}
		saveDataToText(saveTraj, loc + "jointVal" + to_string(robotFlag) + ".txt");
		for (unsigned int k = 0; k < objects.size(); k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					if (attachObjectVec[0][i] && k == gripObjectIdx[robotFlag - 1])
					{
						rManagerVector[robotFlag - 1]->setJointVal(traj[i][j]);
						saveTraj.push_back(SE3toVector(robotVector[robotFlag - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tobject2gripper[k])));
					}
					else
						saveTraj.push_back(SE3toVector(objects[k]->GetBaseLink()->GetFrame()));
				}
			}
			saveDataToText(saveTraj, loc + "object" + to_string(k) + ".txt");
		}
	}
	else
	{
		// multi robot case
		for (int k = 0; k < 2; k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					saveTraj.push_back(traj[i][j].segment(6 * k, 6));
				}
			}
			saveDataToText(saveTraj, loc + "jointVal" + to_string(k + 1) + ".txt");
		}

		for (unsigned int k = 0; k < objects.size(); k++)
		{
			saveTraj.resize(0);
			for (unsigned int i = 0; i < traj.size(); i++)
			{
				int robotnum = -1;
				for (int r = 1; r < 3; r++)
				{
					if (attachObjectVec[r - 1][i] && k == gripObjectIdx[r - 1])
					{
						robotnum = r;
						break;
					}
				}

				for (unsigned int j = 0; j < traj[i].size(); j++)
				{
					if (robotnum != -1)
					{
						rManagerVector[robotnum - 1]->setJointVal(traj[i][j]);
						saveTraj.push_back(SE3toVector(robotVector[robotnum - 1]->gMarkerLink[Indy_Index::MLINK_GRIP].GetFrame() * Inv(Tobject2gripper[k])));
					}
					else
						saveTraj.push_back(SE3toVector(objects[k]->GetBaseLink()->GetFrame()));
				}
			}
			saveDataToText(saveTraj, loc + "object" + to_string(k) + ".txt");
		}
	}
}
