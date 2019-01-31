#pragma once

#include "environment.h"
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn\srWeldJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srRevoluteJoint.h"
#include "Math\mathOperator.h"

class MovingContact : public Object
{
public:
	MovingContact(double collision_offset = 0.01);
	~MovingContact();
	void AssembleModel();
	double m_collision_offset;
};