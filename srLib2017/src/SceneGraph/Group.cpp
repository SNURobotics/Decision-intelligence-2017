#include "Group.h"
#include "gl.h"

Group::Group( SE3 trasform, double scale )
:Transformation(trasform,scale)
{
	priority = 6;
	transformMatrix = trasform;
	scaleFactor = scale;
}
Group::~Group(void)
{
}

void Group::glRender()
{
	glPushAttrib(GL_CURRENT_BIT);
	glPushMatrix();
	
	runTransformation();
	
	//boost::shared_lock< boost::shared_mutex > rock(_node_mutex);
	std::shared_lock< std::shared_mutex > rock(_node_mutex);

	list<Node*>::iterator begin, cur;
	begin = children.begin();
	for ( cur = begin; cur != children.end() ; cur++)
		(*cur)->glRender();
	
	glPopMatrix();
	glPopAttrib();

}

void Group::glMakeDisplayList()
{
	list<Node*>::iterator begin, cur;
	begin = children.begin();
	for ( cur = begin; cur != children.end() ; cur++)
		(*cur)->glMakeDisplayList();
}
