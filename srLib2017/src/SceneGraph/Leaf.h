#pragma once
#include <list>

#include "node.h"
#include "Group.h"
#include "Transformation.h"
#include "Model3DS.h"
#include "ModelSTL.h"


class Leaf : public Node, public Transformation
{

public:
	Leaf(void);
	virtual ~Leaf(void);
	virtual void	glRender() = 0;
	virtual void    update(SE3 transform) { setTransformation(transform); }

protected:
	int listID;
};


class Font : public Leaf
{
public:
	Font();
	Font(const string& str);

	void		setString(const string& str);
	virtual void glRender();

private:
	string			str;
};

class ForceVector : public Leaf
{
public:
	ForceVector();

	void		setForce(dse3*	 pForce);
	void		setForce(dse3	force);
	void		setScale(double scale);
	virtual void glRender();

private:
	dse3*		pForce;
	dse3		force;

	double		scale;
};
class ColorPoint : public Leaf
{
public:
	ColorPoint(double r = 1, double g = 1, double b = 1);
	ColorPoint(Vec3 pos, double r = 1, double g = 1, double b = 1);
	void		setColor(double r, double g, double b);
	void		setPointSize(double size);
	virtual void glRender();
	virtual	void glMakeDisplayList();


private:
	double size;
	double r;
	double g;
	double b;


};

class ColorPointGroup : public Leaf
{

public:

	ColorPointGroup(const list<Vec3>& points, double r = 1, double g = 1, double b = 1);

	void		setColor(double r, double g, double b);
	void		setPointSize(double size);

	virtual	void glMakeDisplayList();
	virtual void glRender();

private:
	list<Vec3>	points;
	double size;
	double r;
	double g;
	double b;
};

class Line : public Leaf
{
public:
	Line();
	void		 update2(SE3 neighborSE3);
	virtual void glRender();
	virtual	void glMakeDisplayList();
private:
	SE3			neighborSE3;


};
class Lines : public Leaf
{
public:
	struct PTY{
		Vec3        pos;
		float       color[4];
	};
public:
	Lines(double lineWidth = 2);
	virtual ~Lines(void);
	virtual void glRender();

	void    setColor(float r, float g, float b, float alpha);
	void    addPoint(Vec3 point);
	void	setLineWidth(double width);
	void    clearPoints();
protected:
	//boost::mutex	_mutex;
	list<PTY>   arrayPoints;
	float       color[4];
	double		lineWidth;
};

class Polygon : public Leaf
{
public:
	Polygon(double r = 1, double g = 1, double b = 1, double alpha = 1);
	void addPoint(const Vec3& point);
	void setColor(double r, double g, double b, double alpha);
	void setPointSize(double size);
	virtual void glMakeDisplayList();
	virtual void glRender();

private:
	vector<Vec3>		points;
	double				size;
	double				r, g, b, alpha;

};
class Tube : public Leaf
{
public:
	Tube(const vector<SE3> frames, double inner_radius, double outer_radius);
	virtual void glMakeDisplayList();
	virtual void glRender();

private:
	void			makeTubeList();
private:

	vector<SE3>			frames;
	double						inner_radius;
	double						outer_radius;

	vector<vector<Vec3>>		innerTube_pointsList;
	vector<vector<Vec3>>		outerTube_pointsList;
};
class Strip : public Leaf
{
public:
	struct PTY{
		Vec3        leftPos;
		Vec3        rightPos;
		float       color[4];
	};
public:
	Strip(void);
	virtual void glRender();

	void    setColor(float r, float g, float b, float alpha);
	void    addPoint(Vec3 lPoint, Vec3 rPoint);
	void    clearPoints();
protected:
	list<PTY>    arrayPoints;
	float          color[4];
};

class Arrow : public Leaf
{
public:
	Arrow(double size = 1);

	void    setArrowVector(Vec3 _arrow);
	virtual void   glRender();

protected:
	Vec3    arrow;
	double size;
};

class Sphere : public Leaf
{
public:
	Sphere(double radius = 1);
	void		setSlice(double _slice);

	virtual void glRender();
	virtual	void glMakeDisplayList();
protected:

	double	radius;

	double slice;
};


class DomeBase : public Leaf
{
public:
	DomeBase(double radius);
	virtual void glRender() = 0;


protected:
	void	drawOpenDome();


protected:
	int		slice;
	int		stack;
	double radius;

};


class Capsule : public DomeBase
{
public:
	Capsule(double radius, double Height);

	virtual void glRender();

protected:
	void	drawOpenCylinder();

protected:
	double height;

};



class HalfSphere : public DomeBase
{
public:
	HalfSphere(double radius, bool bUpper);
	virtual void glRender();
protected:

	bool	bUpper;

};

class Cylinder : public Leaf
{
public:
	Cylinder(double radius, double Height);

	virtual void glRender();



protected:
	double radius;
	double height;

	int		slice;


};

class varianceStrap : public Leaf
{

public:
	varianceStrap();

	virtual	void	glRender();

	void			addEdgePoint(const vector<Vec3>& edgePoint);
	void			setEdgePointList(const vector<vector<Vec3>>& list);
	//void			setColor(double r, double g, double b, double alpha);

protected:

	Vec3		getNormal(int row, int col);
protected:
	vector<vector<Vec3>>		edgePointList;
	//double								color[4];

};

class varianceStrap2D : public varianceStrap
{
	virtual	void	glRender();
};

class Ellipsoid : public Leaf
{
public:
	Ellipsoid(double a, double b, double c);

	virtual void glRender();

protected:

	double	a;
	double	b;
	double	c;
};
class Grid : public Leaf
{
public:
	Grid();
	Grid(double range, double step = 10);
	virtual void glRender();
	virtual	void glMakeDisplayList();
	void    setRange(double range);
	void	setStepSize(double size);
private:
	GLdouble	RANGE;
	GLdouble	STEP;

};
class Coordinate : public Leaf{
public:
	Coordinate(double arrowSize = 1, double sacleFactor = 1, double lineWidth = 1);
	void    setArrowSize(double arrowSize);
	void	setScaleFactor(double sacleFactor);
	void	setLineWidth(double lineWidth);
	virtual void glRender();

private:
	GLdouble	arrowSize;
	GLdouble	sacleFactor;
	GLdouble	lineWidth;
};

class Box : public Leaf{
public:
	Box(double x, double y, double z);

	virtual void glRender();
private:
	double HalfWidth;
	double HalfHeight;
	double HalfDepth;
};


class TDSNode : public Leaf
{
public:

	TDSNode(const char* name, SE3 T = SE3(0.0))
	{
		m_Model = new Model3DS;
		localFrame = T;
		strcpy_s(m_name, name);
		m_Model->Load(m_name);

	}
	~TDSNode()
	{
		delete m_Model;
		//delete [] m_name;
	}

	virtual	void glMakeDisplayList();
	virtual void glRender();

protected:

	char			m_name[_MAX_PATH];
	Model3DS*		m_Model;
	SE3				localFrame;
};

class STLNode : public Leaf
{
public:
	STLNode(const char* name, SE3 T = SE3(0.0))
	{
		m_Model = new ModelSTL;
		localFrame = T;
		strcpy_s(m_name, name);
		m_Model->Load(m_name);
	}
	~STLNode()
	{
		delete m_Model;
	}

	virtual	void glMakeDisplayList();
	virtual void glRender();

protected:
	char		m_name[_MAX_PATH];
	ModelSTL*	m_Model;
	SE3			localFrame;
};

class  NothingRenderLeaf : public Leaf
{
public:

	virtual void glRender();

};