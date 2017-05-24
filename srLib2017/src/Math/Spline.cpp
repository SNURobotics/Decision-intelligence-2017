#include "Spline.h"
#include <math.h>

cubicSpline::cubicSpline()
{
	timeSpan.resize(0);
	a.resize(0);
	b.resize(0);
	c.resize(0);
	d.resize(0);
}

void cubicSpline::interpolation(vector<double> time, vector<Eigen::VectorXd> controlPoint)
{
	if (time.size() != controlPoint.size()){
		cout << "check length of time and control points!!!" << endl;
	}
	timeSpan = time;
	K = time.size();
	dim = controlPoint[0].size();

	a.resize(K);
	b.resize(K);
	c.resize(K);
	d.resize(K);

	for (int i = 0; i < K; i++){
		a[i].resize(dim);
		b[i].resize(dim);
		c[i].resize(dim);
		d[i].resize(dim);
	}

	Eigen::VectorXd B(K - 2);
	Eigen::MatrixXd A(K - 2, K - 2);
	Eigen::VectorXd D(K);
	A.setIdentity();
	A *= 4.0;
	for (int j = 0; j < K - 2; j++){
		if (j < K - 3)
			A(j, j + 1) = 1;
		if (j > 0)
			A(j, j - 1) = 1;
	}

	// interpolation (assume initial and final velocity are zero)
	for (int i = 0; i < dim; i++){
		for (int j = 0; j < K - 2; j++){
			B(j) = 3.0 * (controlPoint[j + 2](i) - controlPoint[j](i));
		}
		D << 0.0, A.inverse()*B, 0.0;

		// save a, b, c, d
		for (int j = 0; j < K - 1; j++){
			d[j](i) = controlPoint[j](i);
			c[j](i) = D(j);
			b[j](i) = 3.0 * (controlPoint[j + 1](i) - controlPoint[j](i)) - 2.0 * D(j) - D(j + 1);
			a[j](i) = 2.0 * (controlPoint[j](i) - controlPoint[j + 1](i)) + D(j) + D(j + 1);
		}
		d[K - 1](i) = controlPoint[K - 1](i);
	}

}

Eigen::VectorXd cubicSpline::getPosition(double time)
{
	Eigen::VectorXd Y(dim);
	
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			Y = d[j - 1] + c[j - 1] * tau + b[j - 1] * tau*tau + a[j - 1] * tau*tau*tau;
			return Y;
		}
	}
	Y = d[K - 1];
	return Y;
}

Eigen::VectorXd cubicSpline::getVelocity(double time)
{
	Eigen::VectorXd Y(dim);

	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			Y = c[j - 1] + b[j - 1] * 2.0 * tau + a[j - 1] * 3.0 * tau*tau;
			return Y;
		}
	}
	Y.setZero();
	return Y;
}

Eigen::VectorXd cubicSpline::getAcceleration(double time)
{
	Eigen::VectorXd Y(dim);

	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			Y = b[j - 1] * 2.0 + a[j - 1] * 6.0 * tau;
			return Y;
		}
	}
	Y.setZero();
	return Y;
}

cubicSpline::~cubicSpline()
{

}


SO3Spline::SO3Spline()
{
	a.resize(0);
	b.resize(0);
	c.resize(0);
	timeSpan.resize(0);
	Rspan.resize(0);
}

SO3Spline::~SO3Spline()
{

}

void SO3Spline::interpolation(vector<double> time, vector<SO3> controlPoint)
{
	if (time.size() != controlPoint.size()){
		cout << "check length of time and control points!!!" << endl;
	}
	timeSpan = time;
	Rspan = controlPoint;
	K = time.size();
	a.resize(K);
	b.resize(K);
	c.resize(K);
	vector<Vec3> r(K);
	for (int i = 0; i < K - 1; i++)
		r[i] = Log(Inv(controlPoint[i]) * controlPoint[i + 1]);
	
	// cf. I.G.KANG AND F.C.PARK, CUBIC SPLINE ALGORITHMS FOR ORIENTATION INTERPOLATION
	// Initialization, assume initial angular acc is zero 
	c[0] = (0.0, 0.0, 0.0); //r[0] * (1.0 / (time[1] - time[0]));  //(0.0, 0.0, 0.0); initial vel 
	b[0] = (0.0, 0.0, 0.0);						// initial acc * 0.5
	a[0] = r[0] - b[0] - c[0];

	for (int i = 1; i < K - 1; i++){
		Vec3 s = r[i];
		Vec3 t = 3.0 * a[i - 1] + 2.0 * b[i - 1] + c[i - 1];
		Vec3 u = 6.0 * a[i - 1] + 2.0 * b[i - 1];
		double s_norm = Norm(s);
		double s_sq_norm = s_norm*s_norm;
		double s_tr_norm = s_sq_norm*s_norm;
		if (s_norm > 0.0000001){
			c[i] = t - (1.0 - cos(s_norm)) / s_sq_norm*Cross(s, t) + (s_norm - sin(s_norm)) / s_tr_norm*Cross(s, Cross(s, t));
			b[i] = 0.5*(u - Inner(s, t) / s_sq_norm / s_sq_norm*(2.0 * cos(s_norm) + s_norm*sin(s_norm) - 2.0)*Cross(s, t) - (1.0 - cos(s_norm)) / s_sq_norm*Cross(s, u) + Inner(s, t) / s_sq_norm / s_tr_norm*(3.0 * sin(s_norm) - s_norm*cos(s_norm) - 2.0 * s_norm)*Cross(s, Cross(s, t)) + (s_norm - sin(s_norm)) / s_tr_norm*(Cross(t, Cross(s, t)) + Cross(s, Cross(s, u))));
			a[i] = s - b[i] - c[i];
		}
		else{
			c[i] = t;
			b[i] = 0.5*u;
			a[i] = s;
		}
	}
}

SO3 SO3Spline::getSO3(double time)
{
	SO3 R;
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			R = Rspan[j - 1] * Exp(a[j - 1] * tau*tau*tau + b[j - 1] * tau*tau + c[j - 1] * tau);
			return R;
		}
	}
	R = Rspan[K - 1];
	return R;
}

Vec3 SO3Spline::getBodyVelocity(double time)
{
	Vec3 w;
	Vec3 r;
	Vec3 dotr;
	Eigen::MatrixXd A(3, 3);
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			double dtau_dt = 1.0 / (timeSpan[j] - timeSpan[j - 1]);
			r = a[j - 1] * tau*tau*tau + b[j - 1] * tau*tau + c[j - 1] * tau;
			dotr = (a[j - 1] * 3.0 *tau*tau + b[j - 1] * 2.0 * tau + c[j - 1])*dtau_dt;
			double r_norm = Norm(r);
			A.setIdentity();
			A -= (1 - cos(r_norm)) / r_norm / r_norm*skewVec3(r);
			A += (r_norm - sin(r_norm)) / r_norm / r_norm / r_norm*skewVec3(r)*skewVec3(r);
			w = VectortoVec3(A*Vec3toVector(r));
			return w;
		}
	}
	w = (0.0, 0.0, 0.0);
	return w;
}

Vec3 SO3Spline::getBodyAcceleration(double time)
{
	Vec3 dotw;
	Vec3 r;
	Vec3 dotr;
	Vec3 ddotr;
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			double dtau_dt = 1.0 / (timeSpan[j] - timeSpan[j - 1]);
			r = a[j - 1] * tau*tau*tau + b[j - 1] * tau*tau + c[j - 1] * tau;
			dotr = (a[j - 1] * 3.0 *tau*tau + b[j - 1] * 2.0 * tau + c[j - 1])*dtau_dt;
			ddotr = (a[j - 1] * 6.0 *tau + b[j - 1] * 2.0)*dtau_dt*dtau_dt;
			double r_norm = Norm(r);
			double r_norm_sq = r_norm*r_norm;
			dotw = ddotr - Inner(r, dotr) / r_norm_sq / r_norm_sq*(2 * cos(r_norm) + r_norm*sin(r_norm) - 2)*Cross(r, dotr) - (1 - cos(r_norm)) / r_norm_sq*Cross(r, ddotr) + Inner(r, dotr) / r_norm_sq / r_norm_sq / r_norm*(3 * sin(r_norm) - r_norm*cos(r_norm) - 2 * r_norm)*Cross(r, Cross(r, dotr)) + (r_norm - sin(r_norm)) / r_norm_sq / r_norm*(Cross(dotr, Cross(r, dotr)) + Cross(r, Cross(r, ddotr)));			
			return dotw;
		}
	}
	dotw = (0.0, 0.0, 0.0);
	return dotw;
}

Vec3 SO3Spline::getSpaceVelocity(double time)
{
	Vec3 w;
	Vec3 r;
	Vec3 dotr;
	Eigen::MatrixXd A(3, 3);
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			double dtau_dt = 1.0 / (timeSpan[j] - timeSpan[j - 1]);
			r = a[j - 1] * tau*tau*tau + b[j - 1] * tau*tau + c[j - 1] * tau;
			dotr = (a[j - 1] * 3.0 *tau*tau + b[j - 1] * 2.0 * tau + c[j - 1])*dtau_dt;
			double r_norm = Norm(r);
			A.setIdentity();
			A += (1 - cos(r_norm)) / r_norm / r_norm*skewVec3(r);
			A += (r_norm - sin(r_norm)) / r_norm / r_norm / r_norm*skewVec3(r)*skewVec3(r);
			w = VectortoVec3(A*Vec3toVector(r));
			return w;
		}
	}
	w = (0.0, 0.0, 0.0);
	return w;
}

Vec3 SO3Spline::getSpaceAcceleration(double time)
{
	Vec3 dotw;
	Vec3 r;
	Vec3 dotr;
	Vec3 ddotr;
	for (int j = 1; j < K; j++){
		if (time < timeSpan[j]){
			double tau = (time - timeSpan[j - 1]) / (timeSpan[j] - timeSpan[j - 1]);
			double dtau_dt = 1.0 / (timeSpan[j] - timeSpan[j - 1]);
			
			r = a[j - 1] * tau*tau*tau + b[j - 1] * tau*tau + c[j - 1] * tau;
			dotr = (a[j - 1] * 3.0 *tau*tau + b[j - 1] * 2.0 * tau + c[j - 1])*dtau_dt;
			ddotr = (a[j - 1] * 6.0 *tau + b[j - 1] * 2.0)*dtau_dt*dtau_dt;
			double r_norm = Norm(r);
			double r_norm_sq = r_norm*r_norm;
			dotw = ddotr + Inner(r, dotr) / r_norm_sq / r_norm_sq*(2 * cos(r_norm) + r_norm*sin(r_norm) - 2)*Cross(r, dotr) + (1 - cos(r_norm)) / r_norm_sq*Cross(r, ddotr) + Inner(r, dotr) / r_norm_sq / r_norm_sq / r_norm*(3 * sin(r_norm) - r_norm*cos(r_norm) - 2 * r_norm)*Cross(r, Cross(r, dotr)) + (r_norm - sin(r_norm)) / r_norm_sq / r_norm*(Cross(dotr, Cross(r, dotr)) + Cross(r, Cross(r, ddotr)));

			return dotw;
		}
	}
	dotw = (0.0, 0.0, 0.0);
	return dotw;
}

SE3Spline::SE3Spline()
{

}

SE3Spline::~SE3Spline()
{

}

void SE3Spline::interpolation(vector<double> time, vector<SE3> controlPoint)
{
	if (time.size() != controlPoint.size()){
		cout << "check length of time and control points!!!" << endl;
	}
	int K = time.size();
	vector<SO3> controlPointOri(K);
	vector<Eigen::VectorXd> controlPointPos(K);

	for (int i = 0; i < K; i++){
		controlPointOri[i] = controlPoint[i].GetOrientation();
		controlPointPos[i] = Vec3toVector(controlPoint[i].GetPosition());
	}

	SO3spline.interpolation(time, controlPointOri);
	VecSpline.interpolation(time, controlPointPos);

}

SE3 SE3Spline::getSE3(double time)
{
	SE3 T;
	T.SetOrientation(SO3spline.getSO3(time));
	T.SetPosition(VectortoVec3(VecSpline.getPosition(time)));
	return T;
}

se3 SE3Spline::getBodyVelocity(double time)
{
	SO3 R = SO3spline.getSO3(time);
	Vec3 wb = SO3spline.getBodyVelocity(time);
	Vec3 vb = Inv(R)*VectortoVec3(VecSpline.getVelocity(time));
	se3 Vb(Vec3toAxis(wb), vb);
	return Vb;
}

se3 SE3Spline::getBodyAcceleration(double time)
{
	SO3 R = SO3spline.getSO3(time);
	Vec3 dotwb = SO3spline.getBodyAcceleration(time);
	Vec3 dotvb = - Cross(SO3spline.getBodyVelocity(time), Inv(R)*VectortoVec3(VecSpline.getVelocity(time))) + Inv(R)*VectortoVec3(VecSpline.getAcceleration(time));
	se3 dotVb(Vec3toAxis(dotwb), dotvb);
	return dotVb;
}

se3 SE3Spline::getSpaceVelocity(double time)
{
	Vec3 ws = SO3spline.getSpaceVelocity(time);
	Vec3 vs = VectortoVec3(VecSpline.getVelocity(time)) - Cross(ws, VectortoVec3(VecSpline.getPosition(time)));
	se3 Vs(Vec3toAxis(ws), vs);
	return Vs;
}

se3 SE3Spline::getSpaceAcceleration(double time)
{
	Vec3 ws = SO3spline.getSpaceVelocity(time);
	Vec3 dotws = SO3spline.getSpaceAcceleration(time);
	Vec3 dotvs = VectortoVec3(VecSpline.getAcceleration(time)) - Cross(ws, VectortoVec3(VecSpline.getVelocity(time))) - Cross(dotws, VectortoVec3(VecSpline.getPosition(time)));
	se3 dotVs(Vec3toAxis(dotws), dotvs);
	return dotVs;
}
