#include "Shader.h"
#include "gl.h"

Shader::Shader(void)
{
	priority = 3;
	
	//////////////////////////////////////////////////////////////////////////

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	const float maxRandomVal = 32767;
	for (int i = 0 ; i < 4 ; i++ )
	{
		ambient[i] = rand()/maxRandomVal;
		diffuse[i] = rand()/maxRandomVal;
		specular[i] = rand()/maxRandomVal;
	}

	shininess =  1;

}

Shader::~Shader(void)
{
}
void Shader::glRender()
{
	glMaterialfv(GL_FRONT,GL_AMBIENT,ambient);
	glMaterialfv(GL_FRONT,GL_DIFFUSE,diffuse);
	glMaterialfv(GL_FRONT,GL_SPECULAR,specular);
	glMaterialf(GL_FRONT,GL_SHININESS,shininess);
}

void Shader::setAmbient( float r, float g, float b, float alpha )
{
	ambient[0] = r;
	ambient[1] = g;
	ambient[2] = b;
	ambient[3] = alpha;

}

void Shader::setDiffuse( float r, float g, float b, float alpha )
{
	diffuse[0] = r;
	diffuse[1] = g;
	diffuse[2] = b;
	diffuse[3] = alpha;

}

void Shader::setSpecular( float r, float g, float b, float alpha )
{
	specular[0] = r;
	specular[1] = g;
	specular[2] = b;
	specular[3] = alpha;

}

void Shader::setShininess( float s )
{
	shininess = s;
}

#define SRG_COLOR_DEFAULT_AMBIENT_FACTOR	(1.0f)
#define SRG_COLOR_DEFAULT_DIFFUSE_FACTOR	(1.0f)
#define SRG_COLOR_DEFAULT_SPECULAR_FACTOR	(0.95f)
#define SRG_COLOR_DEFAULT_EMISSION_FACTOR	(0.08f)
#define SRG_COLOR_DEFAULT_SHININESS_FACTOR	(0.55f)

void Shader::setColor( float r, float g, float b, float alpha /*= 1.0f*/ )
{
	ambient[0] = SRG_COLOR_DEFAULT_AMBIENT_FACTOR*r;
	ambient[1] = SRG_COLOR_DEFAULT_AMBIENT_FACTOR*g;
	ambient[2] = SRG_COLOR_DEFAULT_AMBIENT_FACTOR*b;
	ambient[3] = alpha;

	diffuse[0] = SRG_COLOR_DEFAULT_DIFFUSE_FACTOR*r;
	diffuse[1] = SRG_COLOR_DEFAULT_DIFFUSE_FACTOR*g;
	diffuse[2] = SRG_COLOR_DEFAULT_DIFFUSE_FACTOR*b;
	diffuse[3] = alpha;

	specular[0] = SRG_COLOR_DEFAULT_SPECULAR_FACTOR*r;
	specular[1] = SRG_COLOR_DEFAULT_SPECULAR_FACTOR*g;
	specular[2] = SRG_COLOR_DEFAULT_SPECULAR_FACTOR*b;
	specular[3] = alpha;

	//_MaterialColor.Emission[0] = SRG_COLOR_DEFAULT_EMISSION_FACTOR*r;
	//_MaterialColor.Emission[1] = SRG_COLOR_DEFAULT_EMISSION_FACTOR*g;
	//_MaterialColor.Emission[2] = SRG_COLOR_DEFAULT_EMISSION_FACTOR*b;
	//_MaterialColor.Emission[3] = alpha;

	shininess	= SRG_COLOR_DEFAULT_SHININESS_FACTOR*128.0f;
}
