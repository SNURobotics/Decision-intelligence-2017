#pragma once

#include <list>
//#include <boost/thread/shared_mutex.hpp>
//#include <boost/thread/locks.hpp>
#include <mutex>
#include <shared_mutex>
#include "../liegroup/LieGroup.h"

using namespace std;

class Node
{
public:
	Node(void);
	virtual ~Node(void);

	void	addNode(Node* node);
	void	removeNode(Node* node);

	virtual void	glMakeDisplayList() {}
	virtual void	glRender() = 0;
	virtual void    update(SE3 transform) {}
	void    clearNode();


protected:
	//boost::shared_mutex		_node_mutex;
	std::shared_mutex		_node_mutex;
	unsigned int			priority;
	list<Node*>				children;

};
