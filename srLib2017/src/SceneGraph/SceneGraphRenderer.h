#pragma once


#include <map>
#include <string>
#include "Node.h"
#include "Camera.h"
#include "Light.h"
#include "Texture.h"
#include "Leaf.h"
#include "Group.h"
#include "Shader.h"
#include "Font.h"




//class srSpace;
//class srLink;
//#include "motionCapture/motionCapture.h"

//#include "CMatrix.h"

class SceneGraphRenderer
{
public:


	enum	NUM_WINDOWS	{SINGLE_WINDOWS, DOUBLE_WINDOWS};

	SceneGraphRenderer(void);
	~SceneGraphRenderer(void);
	void InitializeRenderer( int argc, char *argv[], NUM_WINDOWS numWindow, bool bCapture ); 


	void RunRendering();
	void	addNode(Node*  node, bool bFirstRender = true);



protected:
	virtual		void	updateScene() {}
	static void setScreenShot(bool  bScreenShot);
	static bool getScreeShot() {return bScreenShot; }

	void			 ClearScene();

	virtual		void	keyboardOverloading(unsigned char key, int x, int y);
	virtual		void	keyboard2Overloading(unsigned char key, int x, int y) {}

private:
	static void render();
	static void reshape(int w, int h);
	static void render2();
	static void reshape2(int w, int h);
	static void mouseFunc( GLint Button, GLint State, GLint X, GLint Y );
	static void motionFunc(GLint X, GLint Y);
	static void mouseFunc2( GLint Button, GLint State, GLint X, GLint Y );
	static void motionFunc2(GLint X, GLint Y);
	static void keyboardFunc(unsigned char key, int x, int y);
	static void keyboardFunc2(unsigned char key, int x, int y);
	static void updateRenderState();
	static void _updateScene();


	static void screenShot();
	inline static void SWITCH_GROUP_RENDER(unsigned num);

protected:
	static SceneGraphRenderer*		m_pSceneGraph;

	static Node* m_root;
	static Camera* m_camera;

	static Node* m_root2;
	static Camera* m_camera2;


private:
	static								map<int,Group*>		m_groups;
	static								map<int, bool>			m_bGroupsRendered;

public:	
	static int		window1;
	static int		window2;
protected:
	static	int		w1;
	static	int		h1;

	static	int		w2;
	static	int		h2;

	static	bool	bOverlap;

private:
	static	bool	bScreenShot;
	static  int		screenShotNum;
public:
	static	bool	bRender;



};

//#include <Windows.h>
//#include <gdiplus.h>
//
//bool CaptureScreenShot( int nWidth, int nHeight, const std::wstring& szDestFile, const std::wstring& szEncoderString);
//int GetEncoderClsid(const WCHAR* format, CLSID* pClsid);