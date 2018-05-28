
// ESF_Client_Example_JOB_IMOV.h : PROJECT_NAME 응용 프로그램에 대한 주 헤더 파일입니다.
//

#pragma once
//#define SR_RADIAN		(0.01745329251994329577)	//< pi / 180
//#define SR_DEGREE		(57.2957795130823208768)	//< 180 / pi
#define EPS__				1e-4
#ifndef __AFXWIN_H__
	#error "PCH에 대해 이 파일을 포함하기 전에 'stdafx.h'를 포함합니다."
#endif

#include "resource.h"		// 주 기호입니다.
#include "../LieGroup/LieGroup.h"
//inline double DEG2RAD(double d) { return (d * SR_RADIAN); }
//inline double RAD2DEG(double r) { return (r * SR_DEGREE); }
inline double distSE3(SE3 T1, SE3 T2) {
	double dist_p = Norm(T1.GetPosition() - T2.GetPosition());
	double dist_R = Norm(Log(Inv(T1.GetOrientation()) * T2.GetOrientation()));
	return dist_p + dist_R;
}
// CESF_Client_Example_JOB_IMOVApp:
// 이 클래스의 구현에 대해서는 ESF_Client_Example_JOB_IMOV.cpp을 참조하십시오.
//

class CESF_Client_Example_JOB_IMOVApp : public CWinApp
{
public:
	CESF_Client_Example_JOB_IMOVApp();

// 재정의입니다.
public:
	virtual BOOL InitInstance();

// 구현입니다.

	DECLARE_MESSAGE_MAP()
};

extern CESF_Client_Example_JOB_IMOVApp theApp;