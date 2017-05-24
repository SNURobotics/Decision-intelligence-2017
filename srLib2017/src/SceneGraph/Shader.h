#pragma once
#include "node.h"

class Shader :
	public Node
{
public:
	Shader(void);
	virtual ~Shader(void);

	virtual void	glRender();

	void			setAmbient(float r, float g, float b, float alpha);
	void			setDiffuse(float r, float g, float b, float alpha);
	void			setSpecular(float r, float g, float b, float alpha);
	void			setShininess(float s);

	void			setColor(float r, float g, float b, float alpha = 1.0f);
	
private:
	float			ambient[4];
	float			diffuse[4];
	float			specular[4];
	float			shininess;

};
