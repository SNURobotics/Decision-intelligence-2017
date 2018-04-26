#include "vfrrtManager.h"

vfrrtManager::vfrrtManager()
{
	_vectorFieldExist = false;
	_vectorFields.resize(0);
	_vectorFieldWeight = 0.7;
	_algorithmMode = MODE::SIMPLE;
	_lambda = 1.0;
}

vfrrtManager::~vfrrtManager()
{
}

bool vfrrtManager::isProblemFeasible()
{
	checkVectorFieldFeasibility();
	return checkStartGoalFeasibility();
}

Eigen::VectorXd vfrrtManager::extendStepSize(const Eigen::VectorXd & vertPos1, const Eigen::VectorXd & vertPos2, double criterion, TARGET_TREE tree)
{
	if (_algorithmMode == MODE::SIMPLE)
		return extendStepSize_simple(vertPos1, vertPos2, criterion, tree);
	else
		return extendStepSize_genuine(vertPos1, vertPos2, criterion, tree);
}

Eigen::VectorXd vfrrtManager::extendStepSize_noVectorField(const Eigen::VectorXd & vertPos1, const Eigen::VectorXd & vertPos2, double criterion)
{
	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
	if (dir_2_random.norm() < criterion)
		return vertPos2;
	else
	{
		dir_2_random.normalize();
		return vertPos1 + step_size * dir_2_random;
	}
}

Eigen::VectorXd vfrrtManager::extendStepSize_simple(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
{
	if (!_vectorFieldExist)
		return extendStepSize_noVectorField(vertPos1, vertPos2, criterion);
	else
	{
		Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
		Eigen::VectorXd vec = getVectorField(vertPos1);
		if (dir_2_random.norm() > 0)
			dir_2_random.normalize();
		if (vec.norm() > 0)
			vec.normalize();
		
		bool isTree1 = (tree == TARGET_TREE::TREE1);
		bool plusVec = isTree1^_isTreeSwaped;		// XOR
		
		if (plusVec)
			vec = dir_2_random + _vectorFieldWeight * vec;
		else
			vec = dir_2_random - _vectorFieldWeight * vec;
		if (vec.norm() > 0)
			vec.normalize();
		return vertPos1 + step_size * vec;
	}
}

Eigen::VectorXd vfrrtManager::extendStepSize_genuine(const Eigen::VectorXd & vertPos1, const Eigen::VectorXd & vertPos2, double criterion, TARGET_TREE tree)
{
	if (!_vectorFieldExist)
		return extendStepSize_noVectorField(vertPos1, vertPos2, criterion);
	else
	{
		Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
		bool isTree1 = (tree == TARGET_TREE::TREE1);
		bool plusVec = isTree1 ^ _isTreeSwaped;		// XOR
		Eigen::VectorXd vec = getVectorField(vertPos1);
		if (!plusVec)
			vec *= -1.0;
		if (dir_2_random.norm() > 0)
			dir_2_random.normalize();
		double vec_norm = vec.norm();
		if (vec.norm() > 0)
			vec.normalize();

		// get weight for input (v_new = v_field + w*v_input)
		double lambda = vec_norm * _lambda;
		double phi_lambda_inv = (1 - exp(-2.0*lambda)) / lambda;
		double sigma = 0.5 * (dir_2_random - vec).squaredNorm();
		double z = -log(1.0 - sigma * lambda*phi_lambda_inv) / lambda;
		double w = sqrt(2.0*z);

		// get v_new = alpha*v_field + beta*v_rand = v_field + w*v_input
		Eigen::VectorXd n = dir_2_random - vec.dot(dir_2_random) * vec;
		n.normalize();
		double theta = 2.0*asin(0.5*w);
		Eigen::VectorXd update_vec = cos(theta) * vec + sin(theta) * n;
		return vertPos1 + step_size * update_vec;
	}
}

void vfrrtManager::connectParentAndChild(rrtVertex* parentVertex, rrtVertex* childVertex)
{
	childVertex->parentVertex = parentVertex;
	childVertex->distance2parent = getDistance(parentVertex->posState, childVertex->posState);
	if (_vectorFieldExist)
		childVertex->cost_bw_parent = getUpstreamCost(parentVertex->posState, childVertex->posState);
	else
		childVertex->cost_bw_parent = 0.0;
}

double vfrrtManager::getCost(rrtVertex * pos1, rrtVertex * pos2)
{
	return getUpstreamCost(pos1->posState, pos2->posState);
}

double vfrrtManager::getRRTpathSmoothingCost(rrtVertex * vertex1, rrtVertex * vertex2, vector<rrtVertex*>& removedVertex)
{
	//double dist_rrtpath = 0;
	double cost_rrtpath = 0;
	rrtVertex* currentVertex = vertex2;
	while (currentVertex != vertex1)
	{
		//dist_rrtpath += currentVertex->distance2parent;
		cost_rrtpath += currentVertex->cost_bw_parent;
		currentVertex = currentVertex->parentVertex;
		removedVertex.push_back(currentVertex);
	}
	return cost_rrtpath;
}

double vfrrtManager::getNewPathSmoothingCost(vector<rrtVertex*> vertices)
{
	double smoothingCost = 0.0;
	for (unsigned int i = 0; i < vertices.size() - 1; i++)
		smoothingCost += getUpstreamCost(vertices[i]->posState, vertices[i + 1]->posState);
	return smoothingCost;
}

bool vfrrtManager::checkVectorFieldFeasibility()
{
	for (unsigned int i = 0; i < _vectorFields.size(); i++)
	{
		_vectorFields[i]->checkFeasibility(nDim);
		if (_vectorFields[i]->_isFeasible)
			_vectorFieldExist = true;
	}
	return false;
}

void vfrrtManager::addVectorField(rrtVectorField * vectorField)
{
	_vectorFields.push_back(vectorField);
}

void vfrrtManager::clearVectorField()
{
	_vectorFieldExist = false;
	_vectorFields.resize(0);
}

Eigen::VectorXd vfrrtManager::getVectorField(const Eigen::VectorXd & pos1)
{
	Eigen::VectorXd vec = Eigen::VectorXd::Zero(pos1.size());
	Eigen::VectorXd temp;
	for (unsigned int i = 0; i < _vectorFields.size(); i++)
	{
		temp = _vectorFields[i]->getVectorField(pos1);
		if (temp.size() == pos1.size())
		{
			vec += temp;
		}
	}
	return vec;

	//if (_vectorField == VECTOR_FIELD::TRAJFOLLOW)
	//	return trajFollowVectorField(pos1, _refTraj);
	//else if (_vectorField == VECTOR_FIELD::RIVER_2DOF)
	//{
	//	Eigen::VectorXd vec = Eigen::VectorXd::Zero(2);
	//	if (pos1(1) < 0.6*(upperBound(1) - lowerBound(1)) + lowerBound(1) && pos1(1) > 0.4*(upperBound(1) - lowerBound(1)) + lowerBound(1))
	//	{
	//		vec(0) = 1.0;
	//	}
	//	else
	//	{
	//		vec(0) = 0.2;
	//	}
	//	return vec;
	//}
	//else
	//	return Eigen::VectorXd::Zero(pos1.size());
}

void vfrrtManager::setVectorFieldWeight(double weight)
{
	if (weight > 0)
		_vectorFieldWeight = weight;
}


double vfrrtManager::getUpstreamCost(const Eigen::VectorXd & vertPos1, const Eigen::VectorXd & vertPos2, int n /* = 10*/)
{
	Eigen::VectorXd dir = vertPos2 - vertPos1;
	double dist = dir.norm() / (double)n;
	dir.normalize();
	Eigen::VectorXd tmpVec;
	double cost = 0.0;
	for (int i = 0; i < n + 1; i++)
	{
		tmpVec = vertPos1 + (double)i / (double)n * (vertPos2 - vertPos1);
		tmpVec = getVectorField(tmpVec);

		if (i == 0 || i == n)
			cost += 0.5*(tmpVec.norm() - tmpVec.transpose()*dir)*dist;
		else
			cost += (tmpVec.norm() - tmpVec.transpose()*dir)*dist;
	}
	return cost;
}


rrtVectorField::rrtVectorField()
{
	_isFeasible = true;
	_C = 1.0;
}

rrtVectorField::~rrtVectorField()
{
}


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