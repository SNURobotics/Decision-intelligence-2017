#include "Leaf.h"
#include "gl.h"


#define M_RADIAN	 0.0174532925199432957692	// = pi / 180
#define M_DEGREE	 57.2957795130823208768		// = pi / 180


Leaf::Leaf(void)
{
	priority = 5;
}

Leaf::~Leaf(void)
{
}
Lines::Lines(double lineWidth /*= 2*/)
{
	setLineWidth(lineWidth);
	color[0] = 0.0f;
	color[1] = 1.0f;
	color[2] = 1.0f;
	color[3] = 1.0f;
}
Lines::~Lines(void)
{}
void Lines::glRender()
{
	//boost::mutex::scoped_lock(_mutex);
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LINE_BIT | GL_POINT_BIT);
	//glEnable(GL_BLEND);
	//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glLineWidth(lineWidth);
	glBegin(GL_LINE_STRIP);
	{
		list<PTY>::iterator cur;
		for (cur = arrayPoints.begin(); cur != arrayPoints.end(); cur++){
			glColor4f((*cur).color[0], (*cur).color[1], (*cur).color[2], (*cur).color[3]);
			glVertex3d((*cur).pos[0], (*cur).pos[1], (*cur).pos[2]);
		}
	}
	glEnd();
	//glDisable(GL_BLEND);
	glPopAttrib();
	glEnable(GL_LIGHTING);
	glPopMatrix();
}
void Lines::addPoint(Vec3 point)
{
	PTY pt;
	pt.pos = point;
	pt.color[0] = color[0];
	pt.color[1] = color[1];
	pt.color[2] = color[2];
	pt.color[3] = color[3];

	arrayPoints.push_back(pt);
}

void Lines::clearPoints()
{
	//boost::mutex::scoped_lock(_mutex);
	arrayPoints.clear();
}

void Lines::setColor(float r, float g, float b, float alpha)
{
	color[0] = r;
	color[1] = g;
	color[2] = b;
	color[3] = alpha;

}

void Lines::setLineWidth(double width)
{
	lineWidth = width;
}



Polygon::Polygon(double r /*= 1*/, double g /*= 1 */, double b /*=1*/, double alpha)
{
	this->points = points;
	setColor(r, g, b, alpha);


}

void Polygon::setColor(double r, double g, double b, double alpha)
{
	this->r = r;
	this->g = g;
	this->b = b;
	this->alpha = alpha;
}

void Polygon::setPointSize(double size)
{
	this->size = size;
}

void Polygon::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	glBegin(GL_POLYGON);
	vector<Vec3>::const_iterator iter = points.begin();
	for (iter; iter != points.end(); iter++){
		glVertex3d((*iter)[0], (*iter)[1], (*iter)[2]);
	}
	glEnd();
	glEndList();

}

void Polygon::glRender()
{
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glPointSize(size);
	glColor4d(r, g, b, alpha);
	glPushMatrix();
	glCallList(listID);
	glPopMatrix();
	glDisable(GL_BLEND);
	glPopAttrib();
}

void Polygon::addPoint(const Vec3& point)
{
	points.push_back(point);
}


Sphere::Sphere(double _radius /*=1*/)
{
	radius = _radius;
	slice = 40;
}

void Sphere::glRender()
{
	glPushMatrix();
	runTransformation();
	glCallList(listID);
	glPopMatrix();
}

void Sphere::setSlice(double _slice)
{
	slice = _slice;
}

void Sphere::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	glutSolidSphere(radius, slice, slice);
	glEndList();
}

Ellipsoid::Ellipsoid(double _a, double _b, double _c)
{
	a = _a;
	b = _b;
	c = _c;

}

void Ellipsoid::glRender()
{
	glPushMatrix();

	runTransformation();
	glScaled(a, b, c);
	glutSolidSphere(1, 40, 40);

	glPopMatrix();
}


void TDSNode::glRender()
{

	glPushMatrix();
	targetCheck();
	//runTransformation();
	SE3 new_trans = transformMatrix * localFrame;
	//new_trans.SetPosition( new_trans.GetPosition()*1);
	new_trans.ToArray(m_Model->_T);
	glMultMatrixf(m_Model->_T);
	glCallList(listID);
	
	glPopMatrix();

}

void TDSNode::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	m_Model->Draw(scaleFactor);
	glEndList();
}


void STLNode::glRender()
{
	glPushMatrix();
	targetCheck();
	runTransformation();
	//SE3 new_trans = transformMatrix * localFrame;
	//new_trans.SetPosition( new_trans.GetPosition()*1);
	//new_trans.ToArray(m_Model->_T);
	//glMultMatrixf(m_Model->_T);
	glCallList(listID);

	glPopMatrix();
}

void STLNode::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	m_Model->Draw();
	glEndList();
}

Grid::Grid()
{
	setTransformation(EulerZYX(Vec3(0, 0, 0)));
	setRange(10000);
	setStepSize(10);

}

Grid::Grid(double range, double step /*= 10*/)
{
	//setTransformation(EulerZYX(Vec3(0,0,SR_PI_HALF)));
	setRange(range);
	setStepSize(step);
}

void Grid::setRange(double range)
{
	RANGE = range;
}

void Grid::setStepSize(double step)
{
	STEP = step;
}


void Grid::glRender()
{
	glPushMatrix();
	runTransformation();
	glCallList(listID);
	glPopMatrix();
}


void Grid::glMakeDisplayList(){
	listID = glGenLists(1);

	glNewList(listID, GL_COMPILE);
	glPushAttrib(GL_ALL_ATTRIB_BITS);

	glDisable(GL_LIGHTING);


	glColor3f(0.5f, 0.5f, 0.5f);
	glBegin(GL_LINES);
	for (double i = -RANGE; i <= RANGE; i += STEP)
	{
		//if (i == 0 || (i % 100) == 0) continue;

		glVertex3d(RANGE, i, 0);
		glVertex3d(-RANGE, i, 0);

		glVertex3d(i, -RANGE, 0);
		glVertex3d(i, RANGE, 0);
	}
	glEnd();

	glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
	glBegin(GL_LINES);
	for (double i = -RANGE; i <= RANGE; i += STEP)
	{
		//if (i == 0) continue;

		glVertex3d(RANGE, i, 0);
		glVertex3d(-RANGE, i, 0);

		glVertex3d(i, -RANGE, 0);
		glVertex3d(i, RANGE, 0);
	}
	glEnd();

	glLineWidth(2);
	glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
	glBegin(GL_LINES);
	glVertex3d(RANGE, 0, 0);
	glVertex3d(-RANGE, 0, 0);
	glEnd();

	glBegin(GL_LINES);
	glVertex3d(0, -RANGE, 0);
	glVertex3d(0, RANGE, 0);
	glEnd();
	glLineWidth(1);

	//glEnable(GL_LIGHTING);

	//glPopAttrib();
	glPopAttrib();


	glEndList();
}

//Grid::Grid( int range )
//{
//    setTransformation(EulerZYX(Vec3(0,0,SR_PI_HALF)));
//    setRange(range);
//}

Arrow::Arrow(double _size /*= 1*/)
{
	size = _size;
}

void Arrow::glRender()
{
	glPushMatrix();
	{
		runTransformation();
		double angle = atan2(arrow[1], arrow[0]);
		double arrow_size = size;
		glRotated(angle*M_DEGREE, 0, 0, 1);
		glBegin(GL_LINES);
		glVertex3d(0, 0, 0);
		glVertex3d(Norm(arrow), 0, 0);
		glEnd();
		//glLineWidth(3.0f);
		glBegin(GL_LINE_STRIP);
		glVertex3d(Norm(arrow) - arrow_size*cos(30 * M_RADIAN), arrow_size*sin(30 * M_RADIAN), 0);
		glVertex3d(Norm(arrow), 0, 0);
		glVertex3d(Norm(arrow) - arrow_size*cos(30 * M_RADIAN), -arrow_size*sin(30 * M_RADIAN), 0);
		glEnd();

	}
	glPopMatrix();
}

void Arrow::setArrowVector(Vec3 _arrow)
{
	arrow = _arrow;
}

Strip::Strip(void)
{
	color[0] = 0.0f;
	color[1] = 1.0f;
	color[2] = 1.0f;
	color[3] = 1.0f;
}
void Strip::glRender()
{
	glDisable(GL_LIGHTING);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glPushAttrib(GL_LINE_BIT);
	glLineWidth(2);
	glPushMatrix();
	{
		glBegin(GL_QUAD_STRIP);
		{
			list<PTY>::iterator cur;
			for (cur = arrayPoints.begin(); cur != arrayPoints.end(); cur++){
				glColor4f((*cur).color[0], (*cur).color[1], (*cur).color[2], (*cur).color[3]);
				glVertex3d((*cur).leftPos[0], (*cur).leftPos[1], (*cur).leftPos[2]);
				glVertex3d((*cur).rightPos[0], (*cur).rightPos[1], (*cur).rightPos[2]);
			}
		}
		glEnd();
	}
	glLineWidth(1);
	glPopAttrib();
	glPopMatrix();
	glDisable(GL_BLEND);
	glEnable(GL_LIGHTING);

}

void Strip::setColor(float r, float g, float b, float alpha)
{
	color[0] = r;
	color[1] = g;
	color[2] = b;
	color[3] = alpha;
}
void Strip::addPoint(Vec3 lPoint, Vec3 rPoint)
{

	PTY pt;
	pt.leftPos = lPoint;
	pt.rightPos = rPoint;

	pt.color[0] = color[0];
	pt.color[1] = color[1];
	pt.color[2] = color[2];
	pt.color[3] = color[3];

	arrayPoints.push_back(pt);
}


void Strip::clearPoints()
{
	arrayPoints.clear();
}



Coordinate::Coordinate(double arrowSize /*= 24*/, double sacleFactor /*= 50*/, double lineWidth /*= 1*/)
{
	setArrowSize(arrowSize);
	setScaleFactor(sacleFactor);
	setLineWidth(lineWidth);
}

void Coordinate::setArrowSize(double arrowSize)
{
	this->arrowSize = arrowSize;
}

void Coordinate::setScaleFactor(double sacleFactor)
{
	this->sacleFactor = sacleFactor;
}

void Coordinate::setLineWidth(double lineWidth)
{
	this->lineWidth = lineWidth;
}

void Coordinate::glRender()
{
	// ..1..1..    ..1..1..    ..1111..     
	// ..1..1..    ..1..1..    .....1..     
	// ...11...    ..1..1..    ....1...     
	// ...11...    ..1..1..    ...1....     
	// ..1..1..    ...11...    ..1.....     
	// ..1..1..    ...1....    ..1111..     
	// ........    ...1....    ........     
	// ........    .11.....    ........     

	GLubyte x[] = { 0x00, 0x00, 0x24, 0x24, 0x18, 0x18, 0x24, 0x24 };
	GLubyte y[] = { 0x60, 0x10, 0x10, 0x18, 0x24, 0x24, 0x24, 0x24 };
	GLubyte z[] = { 0x00, 0x00, 0x3C, 0x20, 0x10, 0x08, 0x04, 0x3C };

	glPushAttrib(GL_LINE_BIT);
	glPushMatrix();
	{
		glLineWidth(lineWidth);
		glDisable(GL_LIGHTING);
		runTransformation();
		glScaled(sacleFactor, sacleFactor, sacleFactor);
		glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

		glColor3d(0.9, 0, 0);
		glRasterPos3d(arrowSize, arrowSize / 12, arrowSize / 12);
		glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, x);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(arrowSize, 0, 0);
		glEnd();

		glColor3d(0, 0.9, 0);
		glRasterPos3d(arrowSize / 12, arrowSize, arrowSize / 12);
		glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, y);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, arrowSize, 0);
		glEnd();

		glColor3d(0, 0, 0.9);
		glRasterPos3d(arrowSize / 12, arrowSize / 12, arrowSize);
		glBitmap(8, 8, 0.0, 0.0, 0.0, 0.0, z);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, arrowSize);
		glEnd();
		glEnable(GL_LIGHTING);
		glLineWidth(1.0);
	}
	glPopMatrix();
	glPopAttrib();


}
void varianceStrap2D::glRender()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	//glEnable(GL_COLOR_MATERIAL);
	glDisable(GL_LIGHTING);
	//glEnable(GL_LIGHTING);
	glEnable(GL_BLEND);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);

	glPushMatrix(); {
		runTransformation();
		glColor4f(0.8, 0.8, 0.1, 0.1);
		glBegin(GL_QUAD_STRIP);
		for (int i = 0; i < (int)edgePointList.size() - 1; i++){
			for (int j = 0; j < 2; j++)
				glVertex3d(edgePointList[i][j][0], edgePointList[i][j][1], edgePointList[i][j][2]);
		}
		glEnd();

	}

	glPopMatrix();

	glDisable(GL_BLEND);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);

	glPopAttrib();
}


varianceStrap::varianceStrap()
{
	//color[0] = 0.8;
	//color[1] = 0.8;
	//color[2] = 0.8;
	//color[3] = 0.1;
}

void varianceStrap::setEdgePointList(const vector<vector<Vec3>>& list)
{
	edgePointList = list;
}

void varianceStrap::addEdgePoint(const vector<Vec3>& edgePoint)
{
	edgePointList.push_back(edgePoint);
}

void varianceStrap::glRender()
{
	//glPushAttrib(GL_ALL_ATTRIB_BITS);
	//glEnable(GL_COLOR_MATERIAL);
	//glDisable(GL_LIGHTING);
	//glEnable(GL_LIGHTING);
	//	glEnable(GL_BLEND);

	//	glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
	//glDisable(GL_DEPTH_TEST);
	Vec3 normal;
	glPushMatrix(); {
		runTransformation();

		for (int i = 0; i < (int)edgePointList.size() - 1; i++){
			glBegin(GL_QUAD_STRIP);
			//glColor4f(color[0], color[1], color[2], color[3]);
			for (int j = 0; j < (int)edgePointList[i].size(); j++){
				normal = getNormal(i, j);
				glNormal3d(normal[0], normal[1], normal[2]);
				glVertex3d(edgePointList[i][j][0], edgePointList[i][j][1], edgePointList[i][j][2]);

				normal = getNormal(i + 1, j);
				glNormal3d(normal[0], normal[1], normal[2]);
				glVertex3d(edgePointList[i + 1][j][0], edgePointList[i + 1][j][1], edgePointList[i + 1][j][2]);
			}
			normal = getNormal(i, 0);
			glNormal3d(normal[0], normal[1], normal[2]);
			glVertex3d(edgePointList[i][0][0], edgePointList[i][0][1], edgePointList[i][0][2]);

			normal = getNormal(i + 1, 0);
			glNormal3d(normal[0], normal[1], normal[2]);
			glVertex3d(edgePointList[i + 1][0][0], edgePointList[i + 1][0][1], edgePointList[i + 1][0][2]);
			glEnd();
		}

	}

	glPopMatrix();

	//glDisable(GL_BLEND);
	//glEnable(GL_DEPTH_TEST);


	//glPopAttrib();
}

Vec3 varianceStrap::getNormal(int row, int col)
{
	if (row == edgePointList.size() - 1)
		row -= 1;
	if (col == edgePointList[0].size() - 1)
		col = 0;

	Vec3 dir_1 = edgePointList[row + 1][col] - edgePointList[row][col];
	Vec3 dir_2 = edgePointList[row][col + 1] - edgePointList[row][col];

	Vec3 normal = Cross(dir_1, dir_2);
	normal.Normalize();
	return -normal;
}


Box::Box(double x, double y, double z)
{
	HalfWidth = x;
	HalfHeight = y;
	HalfDepth = z;
}

void Box::glRender()
{
	double x = HalfWidth;
	double y = HalfHeight;
	double z = HalfDepth;
	glPushMatrix();
	{
		runTransformation();
		glBegin(GL_QUADS);
		{
			glNormal3f(0, 1, 0);
			glVertex3d(x, y, -z);
			glVertex3d(-x, y, -z);
			glVertex3d(-x, y, z);
			glVertex3d(x, y, z);

			glNormal3f(1, 0, 0);
			glVertex3d(x, -y, -z);
			glVertex3d(x, y, -z);
			glVertex3d(x, y, z);
			glVertex3d(x, -y, z);

			glNormal3f(0, -1, 0);
			glVertex3d(x, -y, z);
			glVertex3d(-x, -y, z);
			glVertex3d(-x, -y, -z);
			glVertex3d(x, -y, -z);

			glNormal3f(-1, 0, 0);
			glVertex3d(-x, -y, z);
			glVertex3d(-x, y, z);
			glVertex3d(-x, y, -z);
			glVertex3d(-x, -y, -z);

			glNormal3f(0, 0, -1);
			glVertex3d(-x, -y, -z);
			glVertex3d(-x, y, -z);
			glVertex3d(x, y, -z);
			glVertex3d(x, -y, -z);

			glNormal3f(0, 0, 1);
			glVertex3d(-x, -y, z);
			glVertex3d(x, -y, z);
			glVertex3d(x, y, z);
			glVertex3d(-x, y, z);
		} //GL_QUADS
		glEnd();
	}
	glPopMatrix();
}

Capsule::Capsule(double radius, double height)
:DomeBase(radius)
{
	this->height = height;


}

void Capsule::glRender()
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	gluQuadricDrawStyle(obj, GLU_FILL);

	glPushMatrix();
	{
		runTransformation();
		{
			//glRotated(90,1,0,0);
			// Check: Stack을 1로 고정
			drawOpenCylinder();

			glTranslated(0.0f, 0.0f, 0.5f * height);
			//glTranslated(0.0, 0.0, Height);
			drawOpenDome();

			glTranslated(0.0f, 0.0f, -height);
			glRotated(180.0f, 0.0f, 1.0f, 0.0f);
			drawOpenDome();

		}

	}
	glPopMatrix();

}

void Capsule::drawOpenCylinder()
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();
	gluQuadricDrawStyle(obj, GLU_FILL);

	glPushMatrix();
	{
		glTranslatef(0.0f, 0.0f, -(GLfloat)height / 2.0f);
		gluCylinder(obj, (GLfloat)radius, (GLfloat)radius, (GLfloat)height, slice, 1);
	}
	glPopMatrix();
}


Cylinder::Cylinder(double radius, double height)
{
	this->radius = radius;
	this->height = height;

	slice = 25;
}

void Cylinder::glRender()
{
	GLUquadricObj *obj;
	obj = gluNewQuadric();

	gluQuadricDrawStyle(obj, GLU_FILL);
	//gluQuadricDrawStyle(obj, GLU_FILL);

	glPushMatrix();
	{
		runTransformation();
		glTranslatef(0.0f, 0.0f, -(GLfloat)height / 2.0f);
		gluCylinder(obj, (GLfloat)radius, (GLfloat)radius, (GLfloat)height, slice, 1);
		gluQuadricOrientation(obj, GLU_INSIDE);
		gluDisk(obj, 0, radius, slice, 1);
		glTranslatef(0.0f, 0.0f, (GLfloat)height);
		gluQuadricOrientation(obj, GLU_OUTSIDE);
		gluDisk(obj, 0, radius, slice, 1);
	}
	glPopMatrix();
}


void Line::update2(SE3 neighborSE3)
{
	this->neighborSE3 = neighborSE3;
}
void Line::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	Vec3 start = transformMatrix.GetPosition();
	Vec3 end = neighborSE3.GetPosition();
	glPushAttrib(GL_LINE_BIT);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(start[0], start[1], start[2]);
	glVertex3f(end[0], end[1], end[2]);
	glEnd();
	glPopAttrib();
	glEndList();
}


void Line::glRender()
{
	glCallList(listID);
}

Line::Line()
{

}


HalfSphere::HalfSphere(double radius, bool bUpper)
:DomeBase(radius)
{
	this->bUpper = bUpper;

}

void HalfSphere::glRender()
{
	glPushMatrix();
	{
		runTransformation();
		if (!bUpper)
			glRotated(180.0f, 0.0f, 1.0f, 0.0f);
		drawOpenDome();
	}
	glPopMatrix();
}

DomeBase::DomeBase(double radius)
:radius(radius)
{
	slice = 12;
	stack = 12;
}


void DomeBase::drawOpenDome()
{
	// (2pi/Stacks)가 
	GLfloat drho = (GLfloat)(3.141592653589 / stack / 2.0);
	GLfloat dtheta = 2.0f * (GLfloat)(3.141592653589 / slice);

	GLint i, j;     // Looping variables

	GLfloat rho = drho;
	GLfloat srho = (GLfloat)(sin(rho));
	GLfloat crho = (GLfloat)(cos(rho));
	GLfloat srhodrho = (GLfloat)(sin(rho + drho));
	GLfloat crhodrho = (GLfloat)(cos(rho + drho));

	// Many sources of OpenGL sphere drawing code uses a triangle fan
	// for the caps of the sphere. This however introduces texturing 
	// artifacts at the poles on some OpenGL implementations
	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0.0f, 0.0f, (GLfloat)radius);
	glVertex3f(0.0f, 0.0f, (GLfloat)radius);
	for (j = 0; j <= slice; j++)
	{
		GLfloat theta = (j == slice) ? 0.0f : j * dtheta;
		GLfloat stheta = (GLfloat)(-sin(theta));
		GLfloat ctheta = (GLfloat)(cos(theta));

		GLfloat x = srho * stheta;
		GLfloat y = srho * ctheta;
		GLfloat z = crho;

		glNormal3f(x, y, z);
		glVertex3f(x * (GLfloat)radius, y * (GLfloat)radius, z * (GLfloat)radius);
	}
	glEnd();

	for (i = 1; i < stack; i++)
	{
		GLfloat rho = (GLfloat)i * drho;
		GLfloat srho = (GLfloat)(sin(rho));
		GLfloat crho = (GLfloat)(cos(rho));
		GLfloat srhodrho = (GLfloat)(sin(rho + drho));
		GLfloat crhodrho = (GLfloat)(cos(rho + drho));

		// Many sources of OpenGL sphere drawing code uses a triangle fan
		// for the caps of the sphere. This however introduces texturing 
		// artifacts at the poles on some OpenGL implementations
		glBegin(GL_TRIANGLE_STRIP);

		for (j = 0; j <= slice; j++)
		{
			GLfloat theta = (j == slice) ? 0.0f : j * dtheta;
			GLfloat stheta = (GLfloat)(-sin(theta));
			GLfloat ctheta = (GLfloat)(cos(theta));

			GLfloat x = srho * stheta;
			GLfloat y = srho * ctheta;
			GLfloat z = crho;

			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)radius, y * (GLfloat)radius, z * (GLfloat)radius);

			x = srhodrho * stheta;
			y = srhodrho * ctheta;
			z = crhodrho;

			glNormal3f(x, y, z);
			glVertex3f(x * (GLfloat)radius, y * (GLfloat)radius, z * (GLfloat)radius);
		}
		glEnd();
	}
}
ForceVector::ForceVector()
{
	pForce = NULL;
	setScale(15);
}


void ForceVector::setForce(dse3* pForce)
{
	this->pForce = pForce;
}

void ForceVector::setForce(dse3 force)
{
	pForce = NULL;
	this->force = force;
}
void ForceVector::setScale(double scale)
{
	this->scale = scale;
}


void ForceVector::glRender()
{

	glPushMatrix();
	{
		runTransformation();

		Vec3 localCoord;
		if (pForce == NULL)
			localCoord = Vec3(force[3], force[4], force[5]);
		else
			localCoord = Vec3((*pForce)[3], (*pForce)[4], (*pForce)[5]);

		Vec3 end = localCoord * scale;

		glLineWidth(2.0);
		glBegin(GL_LINES);
		glVertex3f(0, 0, 0);
		glVertex3f(end[0], end[1], end[2]);
		glEnd();
		glLineWidth(1.0);
	}
	glPopMatrix();
}


ColorPoint::ColorPoint(double r /*= 1*/, double g /*= 1 */, double b /*=1*/)
{
	setColor(r, g, b);
	setPointSize(1);
}

ColorPoint::ColorPoint(Vec3 pos, double r /*= 1*/, double g /*= 1 */, double b /*=1*/)
{
	setColor(r, g, b);
	setPointSize(1);
	setTransformation(SE3(pos));
}

void ColorPoint::setColor(double r, double g, double b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}


void ColorPoint::setPointSize(double size)
{
	this->size = size;
}


void ColorPoint::glRender()
{
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(size);
	glColor3d(r, g, b);
	glPushMatrix();
	runTransformation();
	glCallList(listID);
	glPopMatrix();
	glPopAttrib();

}

void ColorPoint::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	glBegin(GL_POINTS);
	glVertex3d(0, 0, 0);
	glEnd();
	glEndList();
}



ColorPointGroup::ColorPointGroup(const list<Vec3>& points, double r /*= 1*/, double g /*= 1 */, double b /*=1*/)
{
	this->points = points;
	setColor(r, g, b);



}

void ColorPointGroup::setColor(double r, double g, double b)
{
	this->r = r;
	this->g = g;
	this->b = b;
}

void ColorPointGroup::setPointSize(double size)
{
	this->size = size;
}

void ColorPointGroup::glMakeDisplayList()
{
	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	glBegin(GL_POINTS);
	list<Vec3>::const_iterator iter = points.begin();
	for (iter; iter != points.end(); iter++){
		glVertex3d((*iter)[0], (*iter)[1], (*iter)[2]);
	}
	glEnd();
	glEndList();

}

void ColorPointGroup::glRender()
{
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_LIGHTING_BIT | GL_POINT_BIT);
	glDisable(GL_LIGHTING);
	glPointSize(size);
	glColor3d(r, g, b);
	glPushMatrix();
	/*	glBegin(GL_POINTS);
	list<Vec3>::const_iterator iter = points.begin();
	for (iter ; iter != points.end() ; iter++){
	glVertex3d((*iter)[0],(*iter)[1],(*iter)[2]);
	}
	glEnd();*/
	glCallList(listID);
	glPopMatrix();
	glPopAttrib();
}

Font::Font()
{

}

Font::Font(const string& str)
{
	setString(str);
}

void Font::setString(const string& str)
{
	this->str = str;
}

void Font::glRender()
{
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glDisable(GL_LIGHTING);
	//	glPushMatrix();
	Vec3 pos = transformMatrix.GetPosition();
	glColor3f(0.8f, 0.2f, 0.2f);
	glRasterPos2d(pos[0], pos[1]);
	for (size_t i = 0; i < str.size(); ++i)
	{
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, str[i]);
	}

	//glPopMatrix();
	glPopAttrib();
}

Tube::Tube(const vector<SE3> _frames, double _inner_radius, double _outer_radius)
{
	frames = _frames;
	inner_radius = _inner_radius;
	outer_radius = _outer_radius;

	makeTubeList();

}
void Tube::makeTubeList()
{
	const int resolution = 20;
	double step = SR_TWO_PI / resolution;




	vector<Vec3>						global_inner_circle;
	vector<Vec3>						global_outer_circle;

	for (int i = 0; i <= resolution; i++){
		double x_inner = inner_radius * cos(i*step);
		double y_inner = inner_radius * sin(i*step);

		double x_outer = outer_radius * cos(i*step);
		double y_outer = outer_radius * sin(i*step);

		Vec3 inner_point(x_inner, y_inner, 0);
		Vec3 outer_point(x_outer, y_outer, 0);

		global_inner_circle.push_back(inner_point);
		global_outer_circle.push_back(outer_point);

	}


	for (unsigned i = 0; i < frames.size(); i++){

		vector<Vec3>		innerTube_pointsList_one_step;
		vector<Vec3>		outerTube_pointsList_one_step;
		for (int j = 0; j <= resolution; j++){
			innerTube_pointsList_one_step.push_back(frames[i] * global_inner_circle[j]);
			outerTube_pointsList_one_step.push_back(frames[i] * global_outer_circle[j]);
		}
		innerTube_pointsList.push_back(innerTube_pointsList_one_step);
		outerTube_pointsList.push_back(outerTube_pointsList_one_step);
	}

}

void Tube::glMakeDisplayList()
{

	listID = glGenLists(1);
	glNewList(listID, GL_COMPILE);
	glBegin(GL_QUAD_STRIP);
	for (unsigned i = 0; i < innerTube_pointsList.size() - 1; i++){
		for (unsigned j = 0; j < innerTube_pointsList[i].size(); j++){
			Vec3 point_1 = innerTube_pointsList[i][j];
			Vec3 point_2 = innerTube_pointsList[i + 1][j];
			glVertex3d(point_1[0], point_1[2], point_1[2]);
			glVertex3d(point_2[0], point_2[2], point_2[2]);
		}
	}
	glEnd();

	glBegin(GL_QUAD_STRIP);
	for (unsigned i = 0; i < outerTube_pointsList.size() - 1; i++){
		for (unsigned j = 0; j < outerTube_pointsList[i].size(); j++){
			Vec3 point_1 = outerTube_pointsList[i][j];
			Vec3 point_2 = outerTube_pointsList[i + 1][j];
			glVertex3d(point_1[0], point_1[2], point_1[2]);
			glVertex3d(point_2[0], point_2[2], point_2[2]);
		}
	}
	glEnd();


	glBegin(GL_QUAD_STRIP);
	for (unsigned i = 0; i < innerTube_pointsList[0].size(); i++){
		Vec3 point_1 = innerTube_pointsList[0][i];
		Vec3 point_2 = outerTube_pointsList[0][i];
		glVertex3d(point_1[0], point_1[2], point_1[2]);
		glVertex3d(point_2[0], point_2[2], point_2[2]);
	}
	glEnd();


	glBegin(GL_QUAD_STRIP);
	int N = innerTube_pointsList.size() - 1;
	for (unsigned i = 0; i < innerTube_pointsList[N].size(); i++){
		Vec3 point_1 = innerTube_pointsList[N][i];
		Vec3 point_2 = outerTube_pointsList[N][i];
		glVertex3d(point_1[0], point_1[2], point_1[2]);
		glVertex3d(point_2[0], point_2[2], point_2[2]);
	}
	glEnd();

	glEndList();
}

void Tube::glRender()
{
	glPushMatrix();
	glCallList(listID);
	glPopMatrix();

}


void NothingRenderLeaf::glRender()
{

}
