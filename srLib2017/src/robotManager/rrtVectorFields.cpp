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
		vec *= _C;
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

river2dofVectorField::~river2dofVectorField()
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
		vec *= _C;
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

robotRRTVectorField::robotRRTVectorField()
{
}

robotRRTVectorField::~robotRRTVectorField()
{
}

void robotRRTVectorField::setRobotEndeffector(robotManager * rManager, srLink * link)
{
	_rManager = rManager;
	_link = link;
}

void robotRRTVectorField::checkFeasibility(int nDim)
{
	if (_rManager->m_activeArmInfo->m_numJoint != nDim)
		_isFeasible = false;
}

singularityAvoidanceVectorField::singularityAvoidanceVectorField()
{
	_kind = robotManager::manipKind::MIN;
	_eps = 1e-6;
	_C = 50.0;
}

singularityAvoidanceVectorField::~singularityAvoidanceVectorField()
{
}

void singularityAvoidanceVectorField::setManipulabilityKind(robotManager::manipKind kind)
{
	_kind = kind;
}

Eigen::VectorXd singularityAvoidanceVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_isFeasible)
	{
		// assume the potential function to be in the form of -log(manipulability + epsilon)
		double manip;
		Eigen::VectorXd grad = _rManager->manipulabilityGradient(pos1, _link, manip, _kind);
		grad /= (manip + _eps);
		grad *= _C;
		//cout << "manip: " << manip << endl;
		//cout << grad.transpose() << endl;
		return grad;
	}
	else
		return Eigen::VectorXd();
}

workspaceConstantPositionVectorField::workspaceConstantPositionVectorField()
{
	_C = 1.0;
	_fixOri = false;
}

workspaceConstantPositionVectorField::~workspaceConstantPositionVectorField()
{
}

void workspaceConstantPositionVectorField::setWorkspaceVector(const Eigen::VectorXd & vec)
{
	_workspaceVector = vec;
}

Eigen::VectorXd workspaceConstantPositionVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_isFeasible)
	{
		if (_fixOri)
		{
			Eigen::MatrixXd J = _rManager->getAnalyticJacobian(pos1, _link, true);
			Eigen::VectorXd vec = Eigen::VectorXd::Zero(6);
			vec.tail(3) = _workspaceVector;
			vec = pinv(J) * vec;
			return _C*vec;
		}
		else
		{
			Eigen::MatrixXd J_pos = _rManager->getAnalyticJacobian(pos1, _link);
			Eigen::VectorXd vec = pinv(J_pos) * _workspaceVector;
			return _C*vec;
		}
	}
	else
		return Eigen::VectorXd();
}

void workspaceConstantPositionVectorField::checkFeasibility(int nDim)
{
	if (_rManager->m_activeArmInfo->m_numJoint != nDim)
		_isFeasible = false;

	if (_workspaceVector.size() != 3)
		_isFeasible = false;
}

