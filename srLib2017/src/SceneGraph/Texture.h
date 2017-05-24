#pragma once
#include "node.h"

class Texture :
	public Node
{
public:
	Texture(void);
	virtual ~Texture(void);

	virtual void	glRender();
};
