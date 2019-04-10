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

class HDMI : public Object
{
public:
	HDMI(double collision_offset = 0.0);
	~HDMI();
	void AssembleModel();
	double m_collision_offset;
};

class Power : public Object
{
public:
	Power(double collision_offset = 0.0);
	~Power();
	void AssembleModel();
	double m_collision_offset;
};

class Settop : public Object
{
public:
	Settop(double collision_offset = 0.0);
	~Settop();
	void AssembleModel();
	double m_collision_offset;
};

class Soldering : public Object
{
public:
	Soldering(double collision_offset = 0.0);
	~Soldering();
	void AssembleModel();
	double m_collision_offset;
};

class BoxForTape : public Object
{
public:
	BoxForTape(double collision_offset =0.01);
	~BoxForTape();
	void AssembleModel();
	double m_collision_offset;
};

class Tape : public Object
{
public:
	Tape(double collision_offset = 0.01);
	~Tape();
	void AssembleModel();
	double m_collision_offset;
};