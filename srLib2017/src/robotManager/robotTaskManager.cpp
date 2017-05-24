#include "robotTaskManager.h"

robotTaskManager::robotTaskManager(robotManager * _rManager)
{
	m_rManager = _rManager;
	m_rrtManager = new robotRRTManager;
	m_rrtManager->setSpace(m_rManager->m_space);
	m_rrtManager->setSystem(m_rManager->m_activeArmInfo->getActiveStateJoints());
	m_rrtManager->setStateBound(VecToVector(m_rManager->m_lowerJointLimit), VecToVector(m_rManager->m_upperJointLimit));

	m_object = NULL;
	m_link = NULL;
	m_offset = SE3();
}

robotTaskManager::~robotTaskManager()
{
	delete m_rrtManager;
}

void robotTaskManager::attachObject(srSystem * object, srLink * link, SE3 offset)
{
	m_object = object;
	m_link = link;
	m_offset = offset;
}

void robotTaskManager::detachObject()
{
	m_object = NULL;
	m_link = NULL;
	m_offset = SE3();
}

vector<Eigen::VectorXd> robotTaskManager::retargetting(vector<SE3>& eeTraj, srLink * link, bool includeOri, SE3 offset, vector<int> & flag, Eigen::VectorXd initGuess)
{
	// make a collision free & joint limit satisfying robot joint trajectory given end-effector trajectory of a task
	vector<Eigen::VectorXd> qTraj(eeTraj.size());
	flag.resize(0);
	vector<int> flags(eeTraj.size(), 0);
	vector<int> initIdx(0);
	vector<int> goalIdx(0);
	Eigen::VectorXd invInit(m_rManager->m_activeArmInfo->m_numJoint);
	vector<Eigen::VectorXd> tempTraj(0);
	
	if (initGuess.size() == m_rManager->m_activeArmInfo->m_numJoint)
		invInit = initGuess;
	else
		for (int i = 0; i < m_rManager->m_activeArmInfo->m_numJoint; i++)
			invInit(i) = 0.0/*(double)rand() / RAND_MAX * (m_rManager->m_upperJointLimit[i] - m_rManager->m_lowerJointLimit[i]) + m_rManager->m_lowerJointLimit[i]*/;

	for (unsigned int i = 0; i < eeTraj.size(); i++)
	{
		if (i == 0)
			invInit = initGuess;
		else
			invInit = qTraj[i - 1];
		//invInit = initGuess;
		qTraj[i] = m_rManager->inverseKin(eeTraj[i], link, includeOri, offset, flags[i], invInit, 500, robotManager::invKinAlg::QP);
		m_rManager->setJointVal(qTraj[i]);
		m_rManager->m_space->_KIN_UpdateFrame_All_The_Entity_All_The_Systems();

		// update attached object
		if (m_object != NULL)
			m_object->GetBaseLink()->SetFrame(m_link->GetFrame()*m_offset);

		if (m_rManager->checkCollision())
			flags[i] = -1;			// denote collision occurs
		if (i > 0)
		{
			if (flags[i] == 0 && flags[i - 1] != 0)
				goalIdx.push_back(i);
			if (flags[i] != 0 && flags[i - 1] == 0)
				initIdx.push_back(i - 1);
		}
	}

	// check plan index
	bool doPlanning = true;
	if (initIdx.size() == goalIdx.size())
	{
		for (unsigned int i = 0; i < initIdx.size(); i++)
			doPlanning &= (initIdx[i] < goalIdx[i]);
	}
	else
		doPlanning = false;
	if (doPlanning)
	{
		// planning
		if (m_object != NULL)
			m_rrtManager->attachObject(m_object, m_link, m_offset);
		for (int i = initIdx.size() - 1; i >= 0; i--)
		{
			m_rrtManager->setStartandGoal(qTraj[initIdx[i]], qTraj[goalIdx[i]]);
			m_rrtManager->execute(0.01);
			tempTraj = m_rrtManager->extractPath();

			//for (unsigned int j = initIdx[i]; j < goalIdx[i] + 1; j++)
			//{
			//	cout << qTraj[j].transpose() << endl;
			//}

			qTraj.erase(qTraj.begin() + initIdx[i], qTraj.begin() + goalIdx[i] + 1);
			qTraj.insert(qTraj.begin() + initIdx[i], tempTraj.begin(), tempTraj.end());

			//for (unsigned int j = 0; j < tempTraj.size(); j++)
			//	cout << qTraj[j + initIdx[i]].transpose() << endl;


		}
		eeTraj.resize(qTraj.size());

		// calculate forward kin
		for (unsigned int i = 0; i < qTraj.size(); i++)
		{
			eeTraj[i] = m_rManager->forwardKin(qTraj[i], link, offset);
		}
	}
	//cout << "after retargetting..." << endl;
	//cout << qTraj[0].transpose() << endl;
	bool flag_coli;
	bool flag_limit;
	vector<bool> flags_coli = collisionOccur(qTraj, flag_coli);
	vector<bool> flags_limit = exceedJointLimit(qTraj, flag_limit);
	if (flag_coli)
		flag.push_back(retargetFlag::COLLISION);
	if (flag_limit)
		flag.push_back(retargetFlag::LIMIT);
	if (flags[0] != 0)
		flag.push_back(retargetFlag::INFEASIBLE_START);
	if (flags[flags.size() - 1] != 0)
		flag.push_back(retargetFlag::INFEASIBLE_GOAL);

	if (!flag_coli && !flag_limit && (flags[0] == 0) && (flags[flags.size() - 1] == 0))
		flag.push_back(retargetFlag::SOLVED);

	return qTraj;
}

vector<bool> robotTaskManager::collisionOccur(const vector<Eigen::VectorXd>& jointTraj, bool & flag)
{
	// flag = false: collision doesn't occur, true: collision occurs
	// flags[i] = false: collision doesn't occur, true: collision occurs  at i-th traj
	vector<bool> flags(jointTraj.size(), false);
	flag = false;
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		m_rManager->setJointVal(jointTraj[i]);
		m_rManager->m_space->_KIN_UpdateFrame_All_The_Entity_All_The_Systems();

		// update attached object
		if (m_object != NULL)
			m_object->GetBaseLink()->SetFrame(m_link->GetFrame()*m_offset);

		if (m_rManager->checkCollision())
		{
			flags[i] = true;			// collision occurs
			flag = true;
		}
	}
	return flags;
}

vector<bool> robotTaskManager::exceedJointLimit(const vector<Eigen::VectorXd>& jointTraj, bool & flag)
{
	// flag = false: stay in the limit , true: exceed the limit
	// flags[i] = false: stay in the limit , true: exceed the limit   at i-th traj
	vector<bool> flags(jointTraj.size(), false);
	flag = false;
	for (unsigned int i = 0; i < jointTraj.size(); i++)
	{
		if (!m_rManager->checkJointLimit(jointTraj[i]))
		{
			flags[i] = true;		// exceed the limit
			flag = true;
		}
	}
	return flags;
}
