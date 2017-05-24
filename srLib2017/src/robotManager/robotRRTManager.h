#pragma once
#include "RRTmanager\rrtManager.h"

class robotRRTManager : public rrtManager
{
public:
	robotRRTManager();
	~robotRRTManager();

	void				attachObject(srSystem* object, srLink* link, SE3 offset);
	void				detachObject();
	vector<bool>		checkFeasibility(Eigen::VectorXd& initPos, Eigen::VectorXd& goalPos);

	virtual bool		setState(const Eigen::VectorXd& state);

public:
	srSystem*			m_object;
	srLink*				m_link;
	SE3					m_offset;
};