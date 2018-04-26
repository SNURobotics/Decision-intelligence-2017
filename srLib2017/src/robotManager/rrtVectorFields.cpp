#include "rrtVectorFields.h""

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
	_centerPoint = Eigen::VectorXd::Zero(3);
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

	if (_workspaceVector.size() != 3 || _centerPoint.size() != 3)
		_isFeasible = false;
}

objectClearanceVectorField::objectClearanceVectorField()
{
}

objectClearanceVectorField::~objectClearanceVectorField()
{
}

void objectClearanceVectorField::setObjects(vector<Vec3> objectLoc)
{
	_objectCenters = objectLoc;
}

void objectClearanceVectorField::setWeights(vector<double> weight)
{
	_objectWeights = weight;
}

Eigen::VectorXd objectClearanceVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_isFeasible)
	{
		Vec3 endeffectorPos = _rManager->forwardKin(pos1, _link, _endeffectorOffset).GetPosition();
		Vec3 workspaceVector(0.0);
		double norm_inv;
		for (unsigned int i = 0; i < _objectCenters.size(); i++)
		{
			norm_inv = 1.0 / Norm(endeffectorPos - _objectCenters[i]);
			workspaceVector += _objectWeights[i] * (endeffectorPos - _objectCenters[i]) * norm_inv * norm_inv * norm_inv;
		}
		Eigen::VectorXd qdot = pinv(_rManager->getAnalyticJacobian(pos1, _link, false, _endeffectorOffset)) * Vec3toVector(workspaceVector);
		qdot *= _C;
		return qdot;
	}
	else
		return Eigen::VectorXd();
}

void objectClearanceVectorField::checkFeasibility(int nDim)
{
	if (_objectCenters.size() != _objectWeights.size())
		_isFeasible = false;
	if (_rManager->m_activeArmInfo->m_numJoint != nDim)
		_isFeasible = false;
	for (unsigned int i = 0; i < _objectWeights.size(); i++)
	{
		if (_objectWeights[i] < 0)
		{
			_isFeasible = false;
			break;
		}
	}
}
