#include "Light.h"
#include "gl.h"

#include <iostream>

using namespace std;

unsigned int Light::numOfLight = 0;
Light::Light(void)
{
	priority = 2;

	////////////////////////////////////////////////////

	glEnable(GL_DEPTH_TEST);
	//glDisable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_LIGHTING);

	numOfLight++;
	if ( numOfLight > 8 )
	{
		cout << "overflow light : last light could be changed " << endl;
		numOfLight--;
		
	}
	
	LightNumber = GL_LIGHT0 + numOfLight - 1;
	

		
		
	const float maxRandomVal = 32767;
	for (int i = 0 ; i < 4 ; i++ )
	{
		ambient[i] = 0.8;
		diffuse[i] = 0.8;
		specular[i] = 0.8;
	}

	for (int i = 0 ; i < 3 ; i++ )
		position[i] = 3;
	position[3] = 1;


	

}

Light::~Light(void)
{
	numOfLight--;
	if ( numOfLight <= 0 )
		glDisable(GL_LIGHTING);
}

void Light::glRender()
{
	glEnable(LightNumber);
	glLightfv(LightNumber,GL_AMBIENT,ambient);
	glLightfv(LightNumber,GL_DIFFUSE,diffuse);
	glLightfv(LightNumber,GL_SPECULAR,specular);
	glLightfv(LightNumber,GL_POSITION,position);
}

void Light::setAmbient( float r, float g, float b, float alpha )
{
	ambient[0] = r;
	ambient[1] = g;
	ambient[2] = b;
	ambient[3] = alpha;

}

void Light::setDiffuse( float r, float g, float b, float alpha )
{
	diffuse[0] = r;
	diffuse[1] = g;
	diffuse[2] = b;
	diffuse[3] = alpha;

}

void Light::setSpecular( float r, float g, float b, float alpha )
{
	specular[0] = r;
	specular[1] = g;
	specular[2] = b;
	specular[3] = alpha;

}

void Light::setPosition( float x, float y, float z, float w )
{
	position[0] = x;
	position[1] = y;
	position[2] = z;
	position[3] = w;
}