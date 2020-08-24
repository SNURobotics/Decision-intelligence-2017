#include "Camera.h"
#include "gl.h"

//Camera::Camera( SE3 camera /*= SE3()*/ )
//:Transformation(camera)
//{
//	initializeCamera();
//}

Camera::Camera(double _r, double _phi, double _theta)
:Transformation(SE3())
{
	initializeCamera();

	r = _r;
	phi = _phi;
	theta = _theta;

	setTransformation(getCameraMatrix());



}

Camera::Camera(double _r, double _phi, double _theta, Vec3 focus)
:Transformation(SE3())
{
	initializeCamera(focus);

	r = _r;
	phi = _phi;
	theta = _theta;

	setTransformation(getCameraMatrix());
}


Camera::~Camera(void)
{
}
void Camera::initializeCamera()
{
	priority = 1;

	setClearColor(1.0, 1.0, 1.0, 1.0);

	registerClearColor();
	px = 0;
	py = 0;
	dx = 0;
	dy = 0;
	focus_vec = Vec3(0);
	mode = _NONE;
	bOrtho = false;

	bDrawFocus = false;
}
void Camera::initializeCamera(Vec3 focus)
{
	initializeCamera();
	focus_vec = focus;
}
SE3 Camera::getCameraMatrix()
{
	const Vec3 absolute_z_axis(0, 0,1);

	Vec3 pos(r*cos(phi)*cos(theta), r*cos(phi)*sin(theta), r*sin(phi));
	
	Vec3 z_axis = pos;
	z_axis.Normalize();

	Vec3  x_axis = Cross(absolute_z_axis, z_axis);
	if (cos(phi) < 0)
		x_axis = -x_axis;
	x_axis.Normalize();
	
	Vec3 y_axis = Cross(z_axis, x_axis);

	SO3 R(x_axis[0], x_axis[1], x_axis[2], y_axis[0], y_axis[1], y_axis[2], z_axis[0], z_axis[1], z_axis[2]);
	SO3 invR = Inv(R);
	focus_vec -= R * Vec3(dx, dy, 0);
	//SE3 cameraFrame = EulerXYZ(Vec3(-SR_PI_HALF,0,-SR_PI_HALF),Vec3(0,-0.5,-1.5));
	//SE3(invR,)
	return Inv(SE3(R, pos+focus_vec));

}

void Camera::setClearColor( double r, double g, double b, double alpha )
{
	colearColor[0] = r;
	colearColor[1] = g;
	colearColor[2] = b;
	colearColor[3] = alpha;

}

void Camera::registerClearColor()
{
	glClearColor(colearColor[0],colearColor[1],colearColor[2],colearColor[3]);
}

void Camera::setOrtho( double x1,double x2,double y1,double y2, double z1, double z2 )
{
    bOrtho = true;
    ortho[0] = x1;
    ortho[1] = x2;
    ortho[2] = y1;
    ortho[3] = y2;
    ortho[4] = z1;
    ortho[5] = z2;
}
void Camera::glRender()
{
	
	GLint viewport[4];

	glGetIntegerv(GL_VIEWPORT,viewport);
	GLfloat aspectRatio = viewport[2]/float(viewport[3]);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
    if(bOrtho)
        glOrtho(ortho[0],ortho[1],ortho[2],ortho[3],ortho[4],ortho[5]);
        //gluOrtho2D(ortho[0],ortho[1],ortho[2],ortho[3]);
    else
	    gluPerspective((double)60,aspectRatio,0.01,50);

	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	runTransformation();

	if (bDrawFocus)
		drawCameraFocus(viewport[3]);

}

void Camera::drawCameraFocus(double height)
{
	double scale =  r/height * 5;
	//scale = 0.1;
	glPushMatrix();
	glDisable(GL_LIGHTING);
	glTranslated(focus_vec[0], focus_vec[1], focus_vec[2]);
	glScaled(scale, scale, scale);
	glPushAttrib(GL_COLOR_BUFFER_BIT);
	glColor3d(1, 0, 0);
		glutSolidSphere(1.0f, 20,20);
	glPopAttrib();
	glEnable(GL_LIGHTING);
	glPopMatrix();
}

void Camera::mouse2Camera( GLint Button, GLint State, GLint X, GLint Y )
{

	switch(mode)
	{
	case _NONE :
		break;
	case _TRANSLATE :
	{
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT, viewport);
		float width = viewport[2];
		float height = viewport[3];
		
		dx = (X - px) / width * r;
		dy = -(Y - py) / height * r;

		break;
	}

	case _ROTATE :
	{
	
		GLint viewport[4];
		glGetIntegerv(GL_VIEWPORT,viewport);
		float width = viewport[2];
		float height = viewport[3];
		double d_theta = -(X - px) / width *SR_PI;
		double d_phi = (Y - py) / height*SR_PI;

		cos(phi) > 0 ? theta += d_theta : theta -= d_theta;
		phi += d_phi;

		//setTransformation(getCameraMatrix());
		//glutPostRedisplay();
		break;

	}
	case _ZOOM_IN :
		{
			GLint viewport[4];
			glGetIntegerv(GL_VIEWPORT,viewport);
			float height = viewport[3];

			Vec3 pos =  transformMatrix.GetPosition();
			double val = pos.Normalize();

			Y < py ? r *= 1.05 : r *= 0.95;

			break;
		}

	default :
		break;
	
	}


	if (Button == GLUT_RIGHT_BUTTON && State == GLUT_DOWN)
		mode = _TRANSLATE;
	//else if (Button == GLUT_MIDDLE_BUTTON && State == GLUT_DOWN)
	else if (Button == GLUT_LEFT_BUTTON && State == GLUT_DOWN)
		mode = _ROTATE;
	else if (Button == GLUT_MIDDLE_BUTTON && State == GLUT_DOWN)
		mode = _ZOOM_IN;
	else if (State == GLUT_UP || Button == GLUT_LEFT_BUTTON)
		mode = _NONE;
	else if (Button == GLUT_LEFT_BUTTON && State == GLUT_DOWN )
		mode = _ZOOM_IN;
	else if ( State == GLUT_UP )
		mode = _NONE;
	
	setTransformation(getCameraMatrix());
	
	px = X;
	py = Y;
}

void Camera::switchBDrawFocus()
{
	if (bDrawFocus)
		bDrawFocus = false;
	else
		bDrawFocus = true;
}
