#ifndef __MODELSTL__
#define __MODELSTL__

#include "srGamasot/srSTL.h"
#include "gl.h"

class ModelSTL
{
private:
	gamasot::STLc stl_content;

public:
	void Load(string);
	void Draw();
};

#endif