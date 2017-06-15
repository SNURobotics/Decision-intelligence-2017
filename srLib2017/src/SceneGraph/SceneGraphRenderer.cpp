

#include "gl2ps.h"
#include "SceneGraphRenderer.h"


SceneGraphRenderer* SceneGraphRenderer::m_pSceneGraph = NULL;

Node* SceneGraphRenderer::m_root = NULL;
Camera* SceneGraphRenderer::m_camera = NULL;

Node*  SceneGraphRenderer::m_root2 = NULL;
Camera*  SceneGraphRenderer::m_camera2 = NULL;

map<int,Group*>		SceneGraphRenderer::m_groups;
map<int, bool>				SceneGraphRenderer::m_bGroupsRendered;

int		SceneGraphRenderer::window1 = 0;
int		SceneGraphRenderer::window2 = 0;

int		SceneGraphRenderer::w1 = 0;
int		SceneGraphRenderer::h1 = 0;

int		SceneGraphRenderer::w2 = 0;
int		SceneGraphRenderer::h2 = 0;

int     SceneGraphRenderer::screenShotNum = 0;

bool	SceneGraphRenderer::bOverlap = false;
bool	SceneGraphRenderer::bScreenShot = false;
bool	SceneGraphRenderer::bRender = true;



//map<string, Node*>	modelRenderer::m_nodes;
//map<string, Coordinate*>	modelRenderer::m_coords;

//map<srLink*, Node*>	SceneGraphRenderer::m_links;



//Lines* SceneGraphRenderer::m_points = NULL;






SceneGraphRenderer::SceneGraphRenderer(void)
{
	m_pSceneGraph = this;
}

SceneGraphRenderer::~SceneGraphRenderer(void)
{

}

void SceneGraphRenderer::InitializeRenderer( int argc, char *argv[], NUM_WINDOWS numWindow, bool bSingleBuffer )
{

	// Normal OpenGL Setup
	glutInit(&argc, argv);


	if (!bSingleBuffer)
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	else
		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB | GLUT_DEPTH);



	// create the first window
	{
		m_root = new Group;

		//glutInitWindowSize(700, 700);
		glutInitWindowSize(1200, 1000);
		window1 = glutCreateWindow("Robotics@SNU Viewer1");
		glutDisplayFunc(render);
		glutReshapeFunc(reshape);
		glutMouseFunc(mouseFunc);
		glutMotionFunc(motionFunc);
		glutKeyboardFunc(keyboardFunc);
	}



	// create the second window
	if (numWindow == DOUBLE_WINDOWS){

		m_root2 = new Group;

		glutInitWindowSize(700, 700);
		window2 = glutCreateWindow("Robitcs@SNU Viewer1");
		glutPositionWindow(800,30);
		glutDisplayFunc(render2);
		glutReshapeFunc(reshape2);
		glutMouseFunc(mouseFunc2);
		glutMotionFunc(motionFunc2);
		glutKeyboardFunc(keyboardFunc2);
	}

	glutIdleFunc(updateRenderState);

}

void SceneGraphRenderer::ClearScene()
{
	if (m_root != NULL ){
		glutSetWindow(window1);
		m_root->glMakeDisplayList();
		if (m_camera)
			m_camera->registerClearColor();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	if (m_root2 != NULL ){
		glutSetWindow(window2);
		m_root2->glMakeDisplayList();
		if (m_camera2)
			m_camera2->registerClearColor();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
}

void SceneGraphRenderer::RunRendering()
{
	// Now start the simulation

	ClearScene();
	glutMainLoop();

}


void SceneGraphRenderer::render()
{

	if (m_root != NULL ){
		_updateScene();
		glutSetWindow(window1);
		if (m_camera != NULL)
			m_camera->registerClearColor();
		if(!bOverlap)
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		m_root->glRender();
		glFlush();
		if(getScreeShot())
			screenShot();
		glutSwapBuffers();

	}


}

void SceneGraphRenderer::reshape(int w, int h)
{
	glutSetWindow(window1);
	w1 = w;
	h1 = h;
	glViewport(0,0,(GLsizei)w, (GLsizei)h);
	render();

}


void SceneGraphRenderer::render2()
{
	if (m_root2 != NULL ){
		glutSetWindow(window2);

		if (m_camera2 != NULL)
			m_camera2->registerClearColor();
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		m_root2->glRender();



		glFlush();
		glutSwapBuffers();

	}

}

void SceneGraphRenderer::reshape2( int w, int h )
{
	glutSetWindow(window2);
	w2 = w;
	h2 = h;
	glViewport(0,0,(GLsizei)w, (GLsizei)h);
	render2();
}


void SceneGraphRenderer::mouseFunc( GLint Button, GLint State, GLint X, GLint Y )
{
	m_camera->mouse2Camera(Button,State,X,Y);
}

void SceneGraphRenderer::motionFunc(GLint X, GLint Y)
{
	m_camera->mouse2Camera(-1,-1,X,Y);
}

void SceneGraphRenderer::mouseFunc2( GLint Button, GLint State, GLint X, GLint Y )
{
	m_camera2->mouse2Camera(Button,State,X,Y);
}

void SceneGraphRenderer::motionFunc2(GLint X, GLint Y)
{
	m_camera2->mouse2Camera(-1,-1,X,Y);
}

void SceneGraphRenderer::keyboardFunc( unsigned char key, int x, int y )
{
	m_pSceneGraph->keyboardOverloading(key,x,y);
}

void SceneGraphRenderer::keyboardFunc2( unsigned char key, int x, int y )
{
	m_pSceneGraph->keyboard2Overloading(key,x,y);
}


void  SceneGraphRenderer::updateRenderState()
{
	if (window1){
		glutSetWindow(window1);
		glutPostRedisplay();
	}
	if (window2){
		glutSetWindow(window2);
		glutPostRedisplay();
	}


}

void SceneGraphRenderer::screenShot()
{
	//char numChar[4];
	//itoa(screenShotNum++,numChar,10);

	//if (window1){
	//	string modelSnap = "model_image_";

	//	modelSnap += numChar;
	//	modelSnap += ".png";

	//	wstring	wstr = L"";
	//	wstr.assign(modelSnap.begin(),modelSnap.end());

	//	glutSetWindow(window1);
	//	CaptureScreenShot(w1,h1,wstr,L"image/png");
	//}


	//if (window2){

	//	string latentSnap = "latentCapture_";
	//	latentSnap += numChar;
	//	latentSnap += ".png";
	//	wstring wstr2 = L"";
	//	wstr2.assign(latentSnap.begin(),latentSnap.end());

	//	glutSetWindow(window2);
	//	CaptureScreenShot(w2,h2,wstr2,L"image/png");
	//}
}


void SceneGraphRenderer::setScreenShot( bool bScreenShot )
{
	m_pSceneGraph->bScreenShot = bScreenShot;
	screenShotNum = 0;
}

void SceneGraphRenderer::addNode(Node* node, bool bFirstRender)
{
	Group*		group = new Group();
	group->addNode(node);

	int n = m_groups.size();
	m_groups[n] = group;
	m_bGroupsRendered[n] = bFirstRender;
	if (bFirstRender)
		m_root->addNode(group);
}


void SceneGraphRenderer::keyboardOverloading(unsigned char key, int x, int y)
{
	switch (key){
		case '1':
			SWITCH_GROUP_RENDER(1);
			break;
		case '2':
			SWITCH_GROUP_RENDER(2);
			break;
		case '3':
			SWITCH_GROUP_RENDER(3);
			break;
		case '4':
			SWITCH_GROUP_RENDER(4);
			break;
		case '5':
			SWITCH_GROUP_RENDER(5);
			break;
		case '6':
			SWITCH_GROUP_RENDER(6);
			break;
		case '7':
			SWITCH_GROUP_RENDER(7);
			break;
		case '8':
			SWITCH_GROUP_RENDER(8);
			break;
		case '9':
			SWITCH_GROUP_RENDER(9);
			break;
		case '0':
			SWITCH_GROUP_RENDER(10);
			break;
		case 'o':
			if (bOverlap)
				bOverlap = false;
			else
				bOverlap = true;
			break;
		case 'c':
		case 'C':
			m_camera->switchBDrawFocus();
			break;
		/*case 's':
			if (getScreeShot())
			setScreenShot(false);
				else
			setScreenShot(true);*/
			break;
		default:
			break;
		}
}

void SceneGraphRenderer::_updateScene()
{
	m_pSceneGraph->updateScene();
}

void SceneGraphRenderer::SWITCH_GROUP_RENDER(unsigned n)
{
	unsigned num = n - 1;
	 if (m_groups.size() > num){
		  if ( m_bGroupsRendered[num] ){
			   m_root->removeNode(m_groups[num]); 
			   m_bGroupsRendered[num] = false;
		 } 
		else{		
			 m_root->addNode(m_groups[num]); 	
			 m_bGroupsRendered[num] = true;
		 }
	 } 
}

//
//bool CaptureScreenShot(	int nWidth, 	int nHeight, 	const std::wstring& szDestFile, 	const std::wstring& szEncoderString)
//{
//	UINT *pixels=new UINT[nWidth * nHeight];
//	memset(pixels, 0, sizeof(UINT)*nWidth*nHeight);
//
//	glFlush(); 
//	//glFinish();
//
//	glReadPixels(0,0,nWidth,nHeight,GL_BGRA_EXT,GL_UNSIGNED_BYTE,pixels);
//
//	if(NULL==pixels)
//		return false;
//
//	// Initialize GDI+
//	static Gdiplus::GdiplusStartupInput gdiplusStartupInput;
//	static ULONG_PTR gdiplusToken;
//	GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
//
//	{
//		// Create the dest image
//		Gdiplus::Bitmap DestBmp(nWidth,nHeight,PixelFormat32bppARGB);
//
//		Gdiplus::Rect rect1(0, 0, nWidth, nHeight);
//
//		Gdiplus::BitmapData bitmapData;
//		memset( &bitmapData, 0, sizeof(bitmapData));
//		DestBmp.LockBits( 
//			&rect1, 
//			Gdiplus::ImageLockModeRead,
//			PixelFormat32bppARGB,
//			&bitmapData );
//
//		int nStride1 = bitmapData.Stride;
//		if( nStride1 < 0 )
//			nStride1 = -nStride1;
//
//		UINT* DestPixels = (UINT*)bitmapData.Scan0;
//
//		if( !DestPixels )
//		{
//			delete [] pixels;
//			return false;
//		}
//
//		for(UINT row = 0; row < bitmapData.Height; ++row)
//		{
//			for(UINT col = 0; col < bitmapData.Width; ++col)
//			{
//				DestPixels[row * nStride1 / 4 + col] = pixels[row * nWidth + col];
//			}
//		}
//
//		DestBmp.UnlockBits( 
//			&bitmapData );
//
//		delete [] pixels;
//		pixels = NULL;
//
//		DestBmp.RotateFlip( Gdiplus::RotateNoneFlipY );
//
//		CLSID Clsid;
//		int result = GetEncoderClsid(szEncoderString.c_str(), &Clsid);
//
//		if( result < 0 )
//			return false;
//
//		Gdiplus::Status status = DestBmp.Save( szDestFile.c_str(), &Clsid );
//	}
//	// Shutdown GDI+
//	//GdiplusShutdown(gdiplusToken);
//
//	return true;
//}
//
//int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
//{
//	UINT  num = 0;          // number of image encoders
//	UINT  size = 0;         // size of the image encoder array in bytes
//
//	Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;
//
//	Gdiplus::GetImageEncodersSize(&num, &size);
//	if(size == 0)
//		return -1;  // Failure
//
//	pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
//	if(pImageCodecInfo == NULL)
//		return -1;  // Failure
//
//	GetImageEncoders(num, size, pImageCodecInfo);
//
//	for(UINT j = 0; j < num; ++j)
//	{
//		if( wcscmp(pImageCodecInfo[j].MimeType, format) == 0 )
//		{
//			*pClsid = pImageCodecInfo[j].Clsid;
//			free(pImageCodecInfo);
//			return j;  // Success
//		}    
//	}
//
//	free(pImageCodecInfo);
//	return -1;  // Failure
//}
