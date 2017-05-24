#pragma once
#include "node.h"
#include "Transformation.h"



class Group :  public Node, public Transformation
{
public:
	Group(SE3 trasform = SE3(), double scale = 1);
	virtual ~Group(void);

	virtual void	glRender();
	virtual	void	glMakeDisplayList();

protected:

};
