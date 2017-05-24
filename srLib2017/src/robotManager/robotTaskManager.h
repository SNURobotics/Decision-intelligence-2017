#pragma once
#include "robotManager.h"
#include "robotRRTManager.h"

class robotTaskManager
{
	enum retargetFlag
	{
		SOLVED, COLLISION, LIMIT, INFEASIBLE_START, INFEASIBLE_GOAL
	};
public:
	robotTaskManager(robotManager* _rManager);
	~robotTaskManager();

	/////////////////// retargetting
	void						attachObject(srSystem* object, srLink* link, SE3 offset = SE3());
	void						detachObject();
	vector<Eigen::VectorXd>		retargetting(vector<SE3>& eeTraj, srLink* link, bool includeOri, SE3 offset, vector<int>& flag, Eigen::VectorXd initGuess = (Eigen::VectorXd()));		// for 6-dof robot
	vector<bool>				collisionOccur(const vector<Eigen::VectorXd>& jointTraj, bool& flag);
	vector<bool>				exceedJointLimit(const vector<Eigen::VectorXd>& jointTraj, bool& flag);

public:
	robotManager*				m_rManager;
	robotRRTManager*			m_rrtManager;
	srSystem*					m_object;
	srLink*						m_link;
	SE3							m_offset;
};