#include "Node.h"
#include <iostream>

Node::Node(void)
:priority(6)
{
	children.clear();
}

Node::~Node(void)
{
}

void Node::addNode( Node* node )
{

	//boost::unique_lock< boost::shared_mutex > rock(_node_mutex);
	std::unique_lock< std::shared_mutex > rock(_node_mutex);

	list<Node*>::iterator begin, cur;
	
	if ( children.size() < 1 )
	{
		children.push_back(node);
		return;
	}
	
	// priority of node class
	// 1. camera
	// 2. light
	// 3. shader
	// 4. texture
	// 5. leaf
	// 6. group.

	unsigned int prior = node->priority;
		
	begin = children.begin();
	for (cur = begin ; cur != children.end() ; cur++)
	{
		if ( (*cur)->priority > prior )
			break;
	}
	
	children.insert(cur,node);
	

}
void Node::removeNode( Node* node )
{
	//boost::unique_lock< boost::shared_mutex > rock(_node_mutex);
	std::unique_lock< std::shared_mutex > rock(_node_mutex);
	children.remove(node);
	

}


void Node::clearNode()
{
	//boost::unique_lock< boost::shared_mutex > rock(_node_mutex);
	std::unique_lock< std::shared_mutex > rock(_node_mutex);
	list<Node*>::iterator begin, cur;
	begin = children.begin();
	for (cur = begin ; cur != children.end() ; cur++)
	{
		if ( (*cur) !=  NULL )
		{
			(*cur)->clearNode();
			delete *cur ;
			(*cur) = NULL;
		}
	}
	children.clear();

		
}
