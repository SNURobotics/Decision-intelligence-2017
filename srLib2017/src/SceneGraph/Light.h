#pragma once
#include "node.h"

class Light : public Node
{
public:
	Light(void);
	virtual ~Light(void);

	virtual void	glRender();

	void			setAmbient(float r, float g, float b, float alpha);
	void			setDiffuse(float r, float g, float b, float alpha);
	void			setSpecular(float r, float g, float b, float alpha);
	void			setPosition(float x, float y, float z, float w);

	
private:

	void			initialize();
private:
	

	float			ambient[4];
	float			diffuse[4];
	float			specular[4];
	float			position[4];

	unsigned int	LightNumber;

	static unsigned int		numOfLight;
};
