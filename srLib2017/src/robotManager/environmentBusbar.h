#pragma once

#include "environment.h"
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn\srWeldJoint.h"
#include "Math\mathOperator.h"


class TableBusBar_HYU : public Object
{
public:
	TableBusBar_HYU(SE3 tableSurfaceCenter, Vec3 tableDim);
	~TableBusBar_HYU();

	void	AssembleModel();
	
	SE3		m_tableSurfaceCenter;
	Vec3	m_tableDim;
};


// BusBar_HYU parts

class Base_HYU : public Object
{
public:
	Base_HYU();
	~Base_HYU();
	void AssembleModel();
};


class Jig_HYU : public Object
{
public:
	Jig_HYU();
	~Jig_HYU();
	void AssembleModel();
};

class BusBar_HYU : public Object
{
public:
	BusBar_HYU();
	~BusBar_HYU();
	void AssembleModel();
	srLink* busbarLink();
};