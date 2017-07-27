#pragma once

#include "environment.h"
#include "srDyn/srDYN.h"
#include "../Eigen/Dense"
#include <vector>
#include "srDyn\srWeldJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srRevoluteJoint.h"
#include "Math\mathOperator.h"

class WorkCell : public Object
{
public:
	WorkCell(int mode = 0, double height = 0.0);
	~WorkCell();
	void AssembleModel();
	void setStageVal(const Eigen::VectorXd& stageVal);
	srLink* getStagePlate() const;

	std::vector<srRevoluteJoint*> rJoint;
	std::vector<srPrismaticJoint*> pJoint;
	int m_mode;			// mode: 0 - no stage, 1 - connect stage to workcell, 2 - attach last stage plate and cylinder
	double m_height;	// elevation height of robot and frames from default model
};

class SphereMarker : public Object
{
public:
	SphereMarker(double dim = 0.1);
	~SphereMarker();
	void AssembleModel();

	double m_dim;
};