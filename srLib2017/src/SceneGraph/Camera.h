#pragma once
#include "gl.h"
#include "node.h"
#include "Transformation.h"
#include "../liegroup/LieGroup.h"


class Camera :
	public Node , public Transformation
{
public:

	enum CAMERAMODE{ _NONE = 0, _TRANSLATE, _ROTATE, _ZOOM_IN, _ZOOM_OUT};
	Camera(double r, double phi = 0, double theta = 0);
	Camera(double r, double phi, double theta, Vec3 focus);
	//Camera(SE3 camera = SE3());
	virtual ~Camera(void);

	virtual void	glRender();

	void			mouse2Camera(GLint Button, GLint State, GLint X, GLint y);
    void         setOrtho(double x1,double x2,double y1,double y2, double z1, double z2);
	void			setClearColor(double r, double g, double b, double alpha);
	void			registerClearColor();

	void			switchBDrawFocus();
private:
	void			drawCameraFocus(double height);

	void			initializeCamera();
	void			initializeCamera(Vec3 focus);
	SE3			getCameraMatrix();
private:
	
	CAMERAMODE		mode;
	GLint			px;
	GLint			py;
    double          ortho[6];
	GLdouble		colearColor[4];
    bool            bOrtho;

	// ÃÊÁ¡ ÁÂÇ¥
	Vec3			focus_vec;
	double			dx;
	double			dy;
	//  ±¸¸é ÁÂÇ¥°è
	double		   r;
	double			phi;
	double			theta;

	bool				bDrawFocus;
};
