#include "robotRRTManager.h"

robotRRTManager::robotRRTManager()
{
}

robotRRTManager::~robotRRTManager()
{
}

void robotRRTManager::attachObject(srSystem * object, srLink * link, SE3 offset)
{
	m_object = object;
	m_link = link;
	m_offset = offset;
}

void robotRRTManager::detachObject()
{
	m_object = NULL;
	m_link = NULL;
	m_offset = SE3();
}

bool robotRRTManager::checkFeasibility(Eigen::VectorXd & curPos)
{
	return setState(curPos);
}

vector<bool> robotRRTManager::checkFeasibility(Eigen::VectorXd& initPos, Eigen::VectorXd& goalPos)
{
	vector<bool> result(2);
	result[0] = setState(initPos);
	result[1] = setState(goalPos);
	return result;
}

bool robotRRTManager::setState(const Eigen::VectorXd & state)
{
	// set state
	for (int i = 0; i < nDim; i++)
		pState[i]->m_State.m_rValue[0] = state[i];
	pSpace->_KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	
	// update attached object
	if (m_object != NULL)
		m_object->GetBaseLink()->SetFrame(m_link->GetFrame()*m_offset);
	
	// check collision
	return pSpace->_KIN_COLLISION_RUNTIME_SIMULATION_LOOP();
}
