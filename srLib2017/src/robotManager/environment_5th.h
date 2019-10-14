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
	SE3 hdmi_hole;
	SE3 power_hole;
};

class Soldering : public Object
{
public:
	Soldering(double collision_offset = 0.0);
	~Soldering();
	void AssembleModel();
	double m_collision_offset;
};

class PCB : public Object
{
public:
	PCB(double collision_offset = 0.0);
	~PCB();
	void AssembleModel();
	double m_collision_offset;
};

class PCBJig : public Object
{
public:
	PCBJig(double collision_offset = 0.0);
	~PCBJig();
	void AssembleModel();
	double m_collision_offset;
};

class BoxForTape : public Object
{
public:
	BoxForTape(double collision_offset = 0.0);
	~BoxForTape();
	void AssembleModel();
	double m_collision_offset;
};

class Tape : public Object
{
public:
	Tape(double collision_offset = 0.0);
	~Tape();
	void AssembleModel();
	double m_collision_offset;
	srRevoluteJoint re;
};

class WireBlock : public Object
{
public:
	WireBlock(double collision_offset = 0.0);
	~WireBlock();
	void AssembleModel();
	double m_collision_offset;
};

class ACB_FU : public Object
{
public:
	ACB_FU(double collision_offset = 0.01);
	~ACB_FU();
	void AssembleModel();
	double m_collision_offset;
};

class ACB_FL : public Object
{
public:
	ACB_FL(double collision_offset = 0.01);
	~ACB_FL();
	void AssembleModel();
	double m_collision_offset;
};