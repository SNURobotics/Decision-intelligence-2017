#pragma once
#include <cmath>
#include <vector>
#include <iostream>
#include "../Eigen/Dense"
#include "srDyn\srDYN.h"




static std::vector<pair<Vec3, SE3>> makeCylinderWithBoxes(SE3 cylinderCenter, double cylinderRadius, double cylinderHeight, double thickness, double epsilon, int numBox)
{
	// axis of along cylinder height must be aligned to z-axis
	// numBox must be larger than 2

	std::vector<pair<Vec3, SE3>> boxGeomInfo(numBox);

	double tmpR = cylinderRadius - thickness / 2;
	double tmpTheta;
	Vec3 tmpVec3;
	SE3 tmpSE3;
	double axisVal1, axisVal2;


	tmpTheta = 2 * SR_PI * 0.5 / ((double)numBox);

	// if numBox is larger than 2, axisVal1 and axisVal2 have positive values
	axisVal1 = sqrt((tmpR*tmpR) / (1 + tan(tmpTheta)*tan(tmpTheta)));
	axisVal2 = axisVal1 * tan(tmpTheta);

	tmpVec3[0] = thickness;
	tmpVec3[1] = 2 * ((cylinderRadius - thickness) * tan(tmpTheta) - epsilon);
	tmpVec3[2] = cylinderHeight;
	tmpSE3 = RotZ(tmpTheta);
	tmpSE3.SetPosition(Vec3(tmpR*cos(tmpTheta), tmpR*sin(tmpTheta), 0.0));
	tmpSE3 = tmpSE3;

	boxGeomInfo[0].first = tmpVec3;
	boxGeomInfo[0].second = tmpSE3;

	for (int i = 1; i < numBox; i++)
	{
		boxGeomInfo[i].first = tmpVec3;
		tmpSE3 = RotZ(2 * SR_PI / ((double)numBox)) * tmpSE3;
		boxGeomInfo[i].second = tmpSE3;
	}

	for (int i = 0; i < numBox; i++)
		boxGeomInfo[i].second = cylinderCenter * boxGeomInfo[i].second;


	return boxGeomInfo;
}


static std::vector<pair<Vec3, SE3>> makeRectangleHole(SE3 RectangleCenter,  Vec3 RectangleDim, Vec3 HoleCenterFromRC, Vec3 HoleDim)
{
	// rectagle dim_z == hole dim_z ,  hole is cut in z-axis, hole dim should be smaller than rectangle dim
	vector<pair<Vec3, SE3>> boxGeomInfo(4);
	boxGeomInfo[0].first = Vec3(RectangleDim[0], 0.5*RectangleDim[1] - (0.5*HoleDim[1] + HoleCenterFromRC[1]), RectangleDim[2]);
	boxGeomInfo[0].second = RectangleCenter*SE3(Vec3(0.0, 0.5*RectangleDim[1] - 0.5*boxGeomInfo[0].first[1], 0.0));

	boxGeomInfo[1].first = Vec3(RectangleDim[0], 0.5*RectangleDim[1] + (- 0.5*HoleDim[1] + HoleCenterFromRC[1]), RectangleDim[2]);
	boxGeomInfo[1].second = RectangleCenter*SE3(Vec3(0.0, -0.5*RectangleDim[1] + 0.5*boxGeomInfo[1].first[1], 0.0));

	double dim_y = RectangleDim[1] - boxGeomInfo[0].first[1] - boxGeomInfo[1].first[1];
	double pos_y = 0.5*RectangleDim[1] - boxGeomInfo[0].first[1] - 0.5*dim_y;
	boxGeomInfo[2].first = Vec3(0.5*RectangleDim[0] + (HoleCenterFromRC[0] - 0.5*HoleDim[0]), dim_y, RectangleDim[2]);
	boxGeomInfo[2].second = RectangleCenter*SE3(Vec3(-0.5*RectangleDim[0] + 0.5*boxGeomInfo[2].first[0], pos_y, 0.0));

	boxGeomInfo[3].first = Vec3(0.5*RectangleDim[0] - (HoleCenterFromRC[0] + 0.5*HoleDim[0]), dim_y, RectangleDim[2]);
	boxGeomInfo[3].second = RectangleCenter*SE3(Vec3(0.5*RectangleDim[0] - 0.5*boxGeomInfo[3].first[0], pos_y, 0.0));

	return boxGeomInfo;
}