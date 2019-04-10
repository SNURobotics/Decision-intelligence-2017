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

class FixedContact : public Object
{
public:
	FixedContact(double collision_offset = 0.01);
	~FixedContact();
	void AssembleModel();
	double m_collision_offset;
};

class BoxForTape : public Object
{
public:
	BoxForTape();
	~BoxForTape();
	void AssembleModel();
};

class Tape : public Object
{
public:
	Tape();
	~Tape();
	void AssembleModel();
};