#pragma once
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn/srWeldJoint.h"
#include "Math\mathOperator.h"

// TODO: function to read robot sensor, joint val, ...

class Object : public srSystem
{
public:
	//Object() {};
	//~Object() {};

	virtual void			AssembleModel() = 0;
	

	void					setBaseLinkPosition(Vec3 position);
	void					setBaseLinkPosition(Eigen::VectorXd position);
	void					setBaseLinkFrame(SE3 T);

	
public:
	std::vector<srLink>					m_ObjLink;
	std::vector<srCollision>			m_ObjCollision;
	std::vector<srWeldJoint>			m_ObjWeldJoint;

	int									m_numLink;
	int									m_numCollision;
	int									m_numWeldJoint;
};

