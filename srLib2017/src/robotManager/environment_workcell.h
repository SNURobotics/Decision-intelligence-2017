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
	WorkCell();
	~WorkCell();
	void AssembleModel();
	void setStageVal(const Eigen::VectorXd& stageVal);
	srLink* getStagePlate() const;

	std::vector<srRevoluteJoint*> rJoint;
	std::vector<srPrismaticJoint*> pJoint;
};