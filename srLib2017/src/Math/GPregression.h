#pragma once
#include "../Eigen/Core"
#include "mathOperator.h"
#include <srDyn/srSpace.h>
#include <vector>

class GPregression
{
public:
	GPregression();
	~GPregression();

	SO3					getEucWeightedMean(const Eigen::VectorXd weight, const vector<SO3> R);
	double				expKernel(const double t1, const double t2);

	Eigen::MatrixXd		getWeight(double (GPregression::*kernelFunc)(const double t1, const double t2), const vector<double> time, const vector<double> t);
	Eigen::MatrixXd		VecGPregression(const vector<double> time, const Eigen::MatrixXd vecData, const vector<double> t);		// each column of vecData is a vector
	vector<SO3>			SO3GPregression(const vector<double> time, const vector<SO3> SO3Data, const vector<double> t);
	vector<SE3>			SE3GPregression(const vector<double> time, const vector<SE3> SE3Data, const vector<double> t);
	vector<se3>			getSpaceVel(const vector<SE3> SE3reg, const vector<double> t);
	vector<se3>			getSpaceAcc(const vector<se3> spVel, const vector<double> t);
};

