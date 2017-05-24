#pragma once
#include "../Eigen/Core"
#include "mathOperator.h"
#include <srDyn/srSpace.h>
#include <vector>

class cubicSpline
{
public:
	cubicSpline();
	~cubicSpline();

	void					interpolation(vector<double> time, vector<Eigen::VectorXd> controlPoint);
	Eigen::VectorXd			getPosition(double time);
	Eigen::VectorXd			getVelocity(double time);
	Eigen::VectorXd			getAcceleration(double time);

private:
	vector<double>			timeSpan;
	vector<Eigen::VectorXd> a;				// 3rd order term
	vector<Eigen::VectorXd> b;				// 2nd order term
	vector<Eigen::VectorXd> c;				// 1st order term
	vector<Eigen::VectorXd> d;				// 0th order term
	int						K;				// number of control points
	int						dim;			// number of vector dimension
};

class SO3Spline
{
public:
	SO3Spline();
	~SO3Spline();

	void					interpolation(vector<double> time, vector<SO3> controlPoint);
	SO3						getSO3(double time);
	Vec3					getBodyVelocity(double time);
	Vec3					getBodyAcceleration(double time);
	Vec3					getSpaceVelocity(double time);
	Vec3					getSpaceAcceleration(double time);

private:
	vector<double>			timeSpan;
	vector<SO3>				Rspan;
	vector<Vec3>			a;				// 3rd order term
	vector<Vec3>			b;				// 2nd order term
	vector<Vec3>			c;				// 1st order term
	int						K;				// number of control points
};

class SE3Spline
{
public:
	SE3Spline();
	~SE3Spline();

	void					interpolation(vector<double> time, vector<SE3> controlPoint);
	SE3						getSE3(double time);
	se3						getBodyVelocity(double time);
	se3						getBodyAcceleration(double time);
	se3						getSpaceVelocity(double time);
	se3						getSpaceAcceleration(double time);

private:
	SO3Spline		SO3spline;
	cubicSpline		VecSpline;

};