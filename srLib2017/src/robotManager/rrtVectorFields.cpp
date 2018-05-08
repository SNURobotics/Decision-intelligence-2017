#include "rrtVectorFields.h""

robotRRTVectorField::robotRRTVectorField()
{
}

robotRRTVectorField::robotRRTVectorField(robotManager * rManager, srLink * link)
{
	setRobotEndeffector(rManager, link);
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

singularityAvoidanceVectorField::singularityAvoidanceVectorField(robotManager* rManager, srLink* link)
{
	setRobotEndeffector(rManager, link);
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

workspaceConstantPositionVectorField::workspaceConstantPositionVectorField(robotManager* rManager, srLink* link)
{
	setRobotEndeffector(rManager, link);
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

objectClearanceVectorField::objectClearanceVectorField(robotManager* rManager, srLink* link)
{
	setRobotEndeffector(rManager, link);
	_endeffectorOffset = SE3();
	_eps = 1.0e-5;
}

objectClearanceVectorField::~objectClearanceVectorField()
{
}

void objectClearanceVectorField::setObjectLocation(Vec3 objectLoc)
{
	_objectLoc = objectLoc;
}

void objectClearanceVectorField::setWeights(vector<double> weight)
{
	_weights = weight;
}

void objectClearanceVectorField::setLinks(vector<srLink*> links)
{
	_links = links;
}

void objectClearanceVectorField::setOffsets(vector<SE3> offsets)
{
	_offsets = offsets;
}

Eigen::VectorXd objectClearanceVectorField::getVectorField(const Eigen::VectorXd & pos1)
{
	if (_isFeasible)
	{
		Vec3 endeffectorPos = _rManager->forwardKin(pos1, _link, _endeffectorOffset).GetPosition();
		Vec3 workspaceVector(0.0);
		double norm_inv;
		norm_inv = 1.0 / (Norm(endeffectorPos - _objectLoc) + _eps);
		workspaceVector = (endeffectorPos - _objectLoc) * norm_inv * norm_inv * norm_inv;
		Eigen::VectorXd qdot = pinv(_rManager->getAnalyticJacobian(pos1, _link, false, _endeffectorOffset)) * Vec3toVector(workspaceVector);
		vector<SE3> linkPos = _rManager->forwardKin(pos1, _links, _offsets);
		for (unsigned int i = 0; i < _weights.size(); i++)
		{
			//cout << qdot.transpose() << endl;
			norm_inv = 1.0 / (Norm(linkPos[i].GetPosition() - _objectLoc) + _eps);
			workspaceVector = _weights[i] * (linkPos[i].GetPosition() - _objectLoc) * norm_inv * norm_inv * norm_inv;
			qdot += pinv(_rManager->getAnalyticJacobian(pos1, _links[i], false, _offsets[i])) * Vec3toVector(workspaceVector);
		}
		//cout << qdot.transpose() << endl;
		//cout << endl;
		qdot *= _C;
		return qdot;
	}
	else
		return Eigen::VectorXd();
}

void objectClearanceVectorField::checkFeasibility(int nDim)
{
	if (_offsets.size() != _weights.size())
		_isFeasible = false;
	if (_offsets.size() != _links.size())
		_isFeasible = false;
	if (_rManager->m_activeArmInfo->m_numJoint != nDim)
		_isFeasible = false;
	for (unsigned int i = 0; i < _weights.size(); i++)
	{
		if (_weights[i] < 0)
		{
			_isFeasible = false;
			break;
		}
	}
}
