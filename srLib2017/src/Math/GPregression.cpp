#include "GPregression.h"


GPregression::GPregression()
{
}


GPregression::~GPregression()
{
}

SO3 GPregression::getEucWeightedMean(const Eigen::VectorXd weight, const vector<SO3> R)
{
	if (weight.size() != R.size())
		printf("Check size of R!!!\n");
	Eigen::Matrix3d Rsum;
	Rsum.setZero();
	for (int i = 0; i < weight.size(); i++){
		Rsum += weight[i] * SO3toMatrix(R[i]);
	}
	return MatrixtoSO3(Rsum);
}

double GPregression::expKernel(const double t1, const double t2)
{
	double sig = 1.0;
	return exp(-(t1 - t2)*(t1 - t2) / sig / sig);
}

Eigen::MatrixXd GPregression::getWeight(double (GPregression::*kernelFunc)(const double t1, const double t2), const vector<double> time, const vector<double> t)
{
	vector<double> agTime;
	agTime = time;
	agTime.insert(agTime.end(), t.begin(), t.end());
	Eigen::MatrixXd K(agTime.size(), agTime.size());
	for (unsigned int i = 0; i < agTime.size(); i++){
		for (unsigned int j = 0; j < agTime.size(); j++){
			if (i > j)
				K(i, j) = K(j, i);
			else
				K(i, j) = (this->*kernelFunc)(agTime[i], agTime[j]);
		}
	}
	double gamma = 0.01;
	Eigen::MatrixXd I(agTime.size(), agTime.size());
	I.setIdentity();
	K += gamma*I;
	Eigen::MatrixXd weight = K.block(time.size(), 0, t.size(), time.size()) * K.topLeftCorner(time.size(),time.size()).inverse();

	return weight;
}

Eigen::MatrixXd	GPregression::VecGPregression(const vector<double> time, const Eigen::MatrixXd vecData, const vector<double> t)
{
	if (time.size() != vecData.cols())
		printf("Check number of data!!!\n");

	Eigen::MatrixXd	VecGPregResult(vecData.rows(), t.size());

	Eigen::MatrixXd	weight = getWeight(&GPregression::expKernel, time, t);
	VecGPregResult = vecData*weight.transpose();
	return VecGPregResult;
}

vector<SO3> GPregression::SO3GPregression(const vector<double> time, const vector<SO3> SO3Data, const vector<double> t)
{
	if (time.size() != SO3Data.size())
		printf("Check number of data!!!\n");

	Eigen::MatrixXd weight = getWeight(&GPregression::expKernel, time, t);
	vector<SO3> SO3GPregResult(t.size());
	for (unsigned int i = 0; i < t.size(); i++){
		SO3GPregResult[i] = getEucWeightedMean(weight.row(i), SO3Data);
	}
	return SO3GPregResult;
}

vector<SE3> GPregression::SE3GPregression(const vector<double> time, const vector<SE3> SE3Data, const vector<double> t)
{
	if (time.size() != SE3Data.size())
		printf("Check number of data!!!\n");
	
	Eigen::MatrixXd weight = getWeight(&GPregression::expKernel, time, t);
	vector<SE3> SE3GPregResult(t.size());
	vector<SO3> SO3Data(time.size());
	Eigen::MatrixXd vecData(3, time.size());
	
	for (unsigned int i = 0; i < time.size(); i++){
		SO3Data[i] = SE3Data[i].GetOrientation();
		vecData.col(i) = Vec3toVector(SE3Data[i].GetPosition());
	}
	
	Eigen::MatrixXd VecGPregResult = vecData*weight.transpose();

	for (unsigned int i = 0; i < t.size(); i++){
		Vec3 p = VectortoVec3(VecGPregResult.col(i));
		SE3GPregResult[i].SetOrientation(getEucWeightedMean(weight.row(i), SO3Data));
		SE3GPregResult[i].SetPosition(p);
	}
	return SE3GPregResult;
}

vector<se3> GPregression::getSpaceVel(const vector<SE3> SE3reg, const vector<double> t)
{
	// numerical differentiation
	if (SE3reg.size() != t.size())
		printf("Check number of data!!!\n");
	vector<se3> Vs(0);
	for (unsigned int i = 0; i < t.size(); i++){
		if (i < t.size() - 1)
			Vs.push_back(Vectortose3(se3toVector(Log(SE3reg[i + 1] / SE3reg[i])) / (t[i + 1] - t[i])));
		else
			Vs.push_back(se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	}
	return Vs;
}

vector<se3> GPregression::getSpaceAcc(const vector<se3> spVel, const vector<double> t)
{
	// numerical differentiation
	if (spVel.size() != t.size())
		printf("Check number of data!!!\n");
	vector<se3> As(0);
	for (unsigned int i = 0; i < t.size(); i++){
		if (i < t.size() - 1)
			As.push_back(Vectortose3(se3toVector(spVel[i+1] - spVel[i]) / (t[i + 1] - t[i])));
		else
			As.push_back(se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
	}
	return As;
}
