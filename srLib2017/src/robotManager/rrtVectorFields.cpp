#include "rrtVectorFields.h""

trajFollowVectorField::trajFollowVectorField()
{
}

trajFollowVectorField::~trajFollowVectorField()
{
	_refTraj.resize(0);
}

void trajFollowVectorField::setRefTraj(const vector<Eigen::VectorXd>& refTraj)
{
	_refTraj = refTraj;
}

Eigen::VectorXd trajFollowVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_isFeasible)
	{
		Eigen::VectorXd vec(pos1.size());
		double dist;
		unsigned int minIdx = 0;
		double minDist;
		for (unsigned int i = 0; i < _refTraj.size(); i++)
		{
			dist = (pos1 - _refTraj[i]).norm();
			if (i == 0)
				minDist = dist;
			else
			{
				if (dist < minDist)
				{
					minDist = dist;
					minIdx = i;
				}
			}
		}
		vec = _refTraj[minIdx] - pos1;
		//vec.normalize();
		return vec;
	}
	else
		return Eigen::VectorXd();
}

void trajFollowVectorField::checkFeasibility(int nDim)
{
	for (unsigned int i = 0; i < _refTraj.size(); i++)
	{
		if (_refTraj[i].size() != nDim)
		{
			_isFeasible = false;
		}
		if (!_isFeasible)
		{
			printf("check reference trajectory !!!\n");
			break;
		}
	}
}

river2dofVectorField::river2dofVectorField()
{
}

void river2dofVectorField::setBound(const Eigen::VectorXd & lowerBound, const Eigen::VectorXd & upperBound)
{
	_lowerBound = lowerBound;
	_upperBound = upperBound;

}

Eigen::VectorXd river2dofVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (pos1.size() != 2)
		_isFeasible = false;
	if (_isFeasible)
	{
		Eigen::VectorXd vec = Eigen::VectorXd::Zero(2);
		if (pos1(1) < 0.6*(_upperBound(1) - _lowerBound(1)) + _lowerBound(1) && pos1(1) > 0.4*(_upperBound(1) - _lowerBound(1)) + _lowerBound(1))
			vec(0) = 1.0;
		else
			vec(0) = 0.2;
		return vec;
	}
	else
		return Eigen::VectorXd();
}

void river2dofVectorField::checkFeasibility(int nDim)
{
	if (nDim != 2 || _lowerBound.size() < 2 || _upperBound.size() < 2)
		_isFeasible = false;
}
