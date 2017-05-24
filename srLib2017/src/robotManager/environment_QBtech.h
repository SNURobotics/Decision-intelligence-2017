#pragma once

#include "environment.h"
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn\srWeldJoint.h"
#include "Math\mathOperator.h"


class TableBusbar : public Object
{
public:
	TableBusbar(SE3 tableSurfaceCenter, Vec3 tableDim);
	~TableBusbar();

	void	AssembleModel();

	SE3		m_tableSurfaceCenter;
	Vec3	m_tableDim;
};


// Busbar parts (HAN YANG UNIV)

class Base : public Object
{
public:
	Base();
	~Base();
	void AssembleModel();
};


class Jig : public Object
{
public:
	Jig();
	~Jig();
	void AssembleModel();
};

class BusBar : public Object
{
public:
	BusBar();
	~BusBar();
	void AssembleModel();



};

// Busbar parts (QB tech)

class UpperFrame : public Object
{
public:
	UpperFrame();
	~UpperFrame();
	void AssembleModel();
};

class LowerFrame : public Object
{
public:
	LowerFrame();
	~LowerFrame();
	void AssembleModel();
};



class Jig_QB : public Object
{
public:
	Jig_QB();
	~Jig_QB();
	void AssembleModel();
};

class Insert : public Object
{
public:
	Insert();
	~Insert();
	void AssembleModel();
};

class JigAssem_QB : public Object
{
public:
	JigAssem_QB();
	~JigAssem_QB();
	void AssembleModel();
	vector<SE3> holeCenter;
};