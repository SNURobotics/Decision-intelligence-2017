#include "cbirrtManager.h"

cbirrtManager::cbirrtManager()
{
	rrtConstraints = NULL;
}


cbirrtManager::~cbirrtManager()
{
}

Eigen::VectorXd cbirrtManager::extendStepSize(const Eigen::VectorXd& vertPos1, const Eigen::VectorXd& vertPos2, double criterion, TARGET_TREE tree /* = TARGET_TREE::TREE1*/)
{
	Eigen::VectorXd dir_2_random = (vertPos2 - vertPos1);
	Eigen::VectorXd temp_vertex_pos;

	if (dir_2_random.norm() < criterion)
		temp_vertex_pos = vertPos2;
	else
	{
		dir_2_random.normalize();
		temp_vertex_pos = vertPos1 + step_size * dir_2_random;
	}

	if (rrtConstraints != NULL)
		rrtConstraints->project2ConstraintManifold(temp_vertex_pos);
	return temp_vertex_pos;
}

vector<rrtVertex*> cbirrtManager::getConstrainedPathConnectingTwoVertices(rrtVertex* vertex1, rrtVertex* vertex2, double eps, int maxIter /*= 10000*/)
{
	vector<rrtVertex*> extendedVertex;
	rrtVertex* firstExtenedVertex = new rrtVertex;
	firstExtenedVertex->posState = vertex1->posState;
	extendedVertex.push_back(firstExtenedVertex);


	// find temporary vertex set which connect vertex1 and vertex2
	int iter = 0;
	while (iter < maxIter)
	{
		Eigen::VectorXd diff = vertex2->posState - extendedVertex[extendedVertex.size() - 1]->posState;
		double diffnorm = diff.norm();

		// if two random vertices (vertex1 and vertex2) are close enough, break
		if (diffnorm < eps && !collisionChecking(vertex2->posState, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
			break;

		// if newly extended vertex is farther to vertex2 than previously extended vertex, break
		if (extendedVertex.size() > 2 && diffnorm >(vertex2->posState - extendedVertex[extendedVertex.size() - 2]->posState).norm())
		{
			extendedVertex.resize(extendedVertex.size() - 1);
			break;
		}

		// to get a temp vertex, from vertex1 go a step size to direction to vertex2 and project to constraint manifold
		Eigen::VectorXd temp_vertex = extendedVertex[extendedVertex.size() - 1]->posState + min(step_size, diffnorm)*diff;
		rrtConstraints->project2ConstraintManifold(temp_vertex);

		// if there is no collision, add temp vertex to the set of temporary vertices
		if (!collisionChecking(temp_vertex, extendedVertex[extendedVertex.size() - 1]->posState, step_size*0.1))
		{
			rrtVertex* tempVertex = new rrtVertex;
			tempVertex->posState = temp_vertex;
			//tempVertex->parentVertex = extendedVertex[extendedVertex.size() - 1];
			//tempVertex->distance2parent = (extendedVertex[extendedVertex.size() - 1]->posState - temp_vertex).norm();
			//tempVertex->cost_bw_parent = getCost(extendedVertex[extendedVertex.size() - 1], tempVertex);

			extendedVertex.push_back(tempVertex);
		}
		else
			break;
		iter++;
	}
	return extendedVertex;
}

vector<rrtVertex*> cbirrtManager::getCandidateVertices(vector<rrtVertex*> vertices)
{
	double eps = 0.5*step_size;
	vector<rrtVertex*> extendedVertex = getConstrainedPathConnectingTwoVertices(vertices[0], vertices[1], eps);
	double dist_cur = (vertices[1]->posState - extendedVertex[extendedVertex.size() - 1]->posState).norm();
	if (dist_cur < eps)
	{
		extendedVertex.push_back(vertices[1]);
		return extendedVertex;
	}
	else
	{
		// delete extended vertices
		for (unsigned int i = 0; i < extendedVertex.size(); i++)
			delete extendedVertex[i];
	}
	return vector<rrtVertex*>();
}

bool cbirrtManager::replaceVertices(list<rrtVertex*>& path, vector<rrtVertex*>& tempVertices, vector<rrtVertex*>& removedVertex)
{
	list<rrtVertex*> tmpPath;
	rrtVertex* vertex1 = removedVertex[removedVertex.size() - 1]->parentVertex;
	rrtVertex* vertex2 = tempVertices[tempVertices.size() - 1];
	if (tempVertices.size() == 2)
	{
		vertex2->parentVertex = vertex1;
		vertex2->distance2parent = getDistance(vertex1->posState, vertex2->posState);
		vertex2->cost_bw_parent = getCost(vertex1, vertex2);
	}
	else
	{
		tempVertices[1]->parentVertex = vertex1;
		tempVertices[1]->distance2parent = getDistance(vertex1->posState, tempVertices[1]->posState);
		tempVertices[1]->cost_bw_parent = getCost(vertex1, tempVertices[1]);
		for (unsigned int i = tempVertices.size() - 1; i > 1; i--)
		{
			tempVertices[i]->parentVertex = tempVertices[i - 1];
			tempVertices[i]->distance2parent = getDistance(tempVertices[i - 1]->posState, tempVertices[i]->posState);
			tempVertices[i]->cost_bw_parent = getCost(tempVertices[i - 1], tempVertices[i]);
		}
	}


	for (unsigned int i = 0; i < removedVertex.size() - 1; i++)
		path.remove(removedVertex[i]);

	// add new vertices to the path
	int n = path.size();
	tmpPath.resize(0);
	list<rrtVertex*>::reverse_iterator tmpriter = tmpPath.rbegin();
	list<rrtVertex*>::reverse_iterator riter = path.rbegin();
	tmpPath.push_back(*riter);
	int added_num = tempVertices.size() - 1;
	for (int i = 1; i < n + added_num - 1; i++, tmpriter++)
		tmpPath.push_front((*tmpriter)->parentVertex);
	path = tmpPath;
	delete tempVertices[0];

	return true;
}

void cbirrtManager::setConstraint(rrtConstraint* constraint)
{
	rrtConstraints = constraint;
}

void cbirrtManager::clearConstraints()
{
	rrtConstraints = NULL;
}


rrtConstraint::rrtConstraint()
{
}


rrtConstraint::~rrtConstraint()
{
}