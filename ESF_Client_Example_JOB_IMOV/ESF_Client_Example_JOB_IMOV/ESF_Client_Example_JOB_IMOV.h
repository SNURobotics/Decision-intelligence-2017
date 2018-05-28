
// ESF_Client_Example_JOB_IMOV.h : PROJECT_NAME ���� ���α׷��� ���� �� ��� �����Դϴ�.
//

#pragma once
//#define SR_RADIAN		(0.01745329251994329577)	//< pi / 180
//#define SR_DEGREE		(57.2957795130823208768)	//< 180 / pi
#define EPS__				1e-4
#ifndef __AFXWIN_H__
	#error "PCH�� ���� �� ������ �����ϱ� ���� 'stdafx.h'�� �����մϴ�."
#endif

#include "resource.h"		// �� ��ȣ�Դϴ�.
#include "../LieGroup/LieGroup.h"
//inline double DEG2RAD(double d) { return (d * SR_RADIAN); }
//inline double RAD2DEG(double r) { return (r * SR_DEGREE); }
inline double distSE3(SE3 T1, SE3 T2) {
	double dist_p = Norm(T1.GetPosition() - T2.GetPosition());
	double dist_R = Norm(Log(Inv(T1.GetOrientation()) * T2.GetOrientation()));
	return dist_p + dist_R;
}
// CESF_Client_Example_JOB_IMOVApp:
// �� Ŭ������ ������ ���ؼ��� ESF_Client_Example_JOB_IMOV.cpp�� �����Ͻʽÿ�.
//

class CESF_Client_Example_JOB_IMOVApp : public CWinApp
{
public:
	CESF_Client_Example_JOB_IMOVApp();

// �������Դϴ�.
public:
	virtual BOOL InitInstance();

// �����Դϴ�.

	DECLARE_MESSAGE_MAP()
};

extern CESF_Client_Example_JOB_IMOVApp theApp;