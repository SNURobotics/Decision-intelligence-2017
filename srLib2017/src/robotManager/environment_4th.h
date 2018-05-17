#pragma once

#include "environment.h"
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn\srWeldJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srRevoluteJoint.h"
#include "Math\mathOperator.h"

class Bin : public Object
{
public:
	Bin(double collision_offset = 0.01);
	~Bin();
	void AssembleModel();
	double m_collision_offset;
};

class Table4th : public Object
{
public:
	Table4th(double collision_offset = 0.01);
	~Table4th();
	void AssembleModel();
	double m_collision_offset;
};

// Temporary working object
class workingObject : public Object
{
public:
	workingObject();
	~workingObject();
	void AssembleModel();
};

// Barrier1 (with camera)
class Barrier1 : public Object
{
public:
	Barrier1(double collision_offset = 0.01);
	~Barrier1();
	void AssembleModel();
	double m_collision_offset;
};

// Barrier2
class Barrier2 : public Object
{
public:
	Barrier2(double collision_offset = 0.01);
	~Barrier2();
	void AssembleModel();
	double m_collision_offset;
};