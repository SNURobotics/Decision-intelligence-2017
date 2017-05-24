#pragma once
#include "environment.h"
#define	NUM_OF_VIEW_FEATUREPOINT		9
#define SHOW_GRIPPER
#define USE_MOCAP_DATA
class FeaturePoint : public Object
{
public:
	FeaturePoint();
	~FeaturePoint();
	void	AssembleModel();
	
};

class Table : public Object
{
public:
	Table();
	~Table();
	void	setLocalFramefromGlobal(Eigen::VectorXd posOri);
	void	setTableHeight(double tableHeight);
	Vec3	getTableCenter();
	double	getTableHeight();
	void	AssembleModel();

	SE3     m_Center;
	Vec3	m_tableCenter;
	Vec3	m_tableDim;
	double	m_tableHeight;

};

class Table2 : public Object
{
public:
	Table2();
	~Table2();
	void	setLocalFramefromGlobal(Eigen::VectorXd posOri);
	void	setTableHeight(double tableHeight);
	Vec3	getTableCenter();
	double	getTableHeight();
	void	AssembleModel();

	SE3     m_Center;			// center of upper face of table
	Vec3	m_tableCenter;
	Vec3	m_tableDim;
	double	m_tableHeight;

};

class TaskObject : public Object
{
public:
	TaskObject() { m_table = NULL; };
	void	setLocalFramefromGlobal(Eigen::VectorXd posOri);
	void	setTable(Table2* table);
	void	setLocalFramefromTable(SE3 frame);
	SE3		m_Center;
	SE3		m_robot2obj;		// how to grip
	SE3		m_robot2objReach;	// approach point before grasp
	SE3		m_data2ref;			// data expressed frame to reference frame
	Table2* m_table;
};

class Supporter1 : public TaskObject //long supporter
{
public:
	Supporter1();
	~Supporter1();
	void	AssembleModel();

	Vec3		m_SupporterBaseLinkDim;

};

class Supporter2 : public TaskObject // short supporter
{
public:
	Supporter2();
	~Supporter2();
	void AssembleModel();

	Vec3		m_Supporter2Dim;

};

class Supporter3 : public TaskObject // vertical supporter
{
public:
	Supporter3();
	~Supporter3();
	void AssembleModel();

	Vec3		m_Supporter3Dim;

};

class Bracket1 : public TaskObject //small bracket
{
public:
	Bracket1();
	~Bracket1();
	void AssembleModel();

	Vec3		m_Bracket1Dim;
};

class Bracket2 : public TaskObject //big bracket
{
public:
	Bracket2();
	~Bracket2();
	void AssembleModel();

	Vec3		m_Bracket2Dim;

};


class AlProfile : public TaskObject // 알류미늄 지지대 (한양대 task 에서만 쓰일 듯)
{
public:
	AlProfile();
	~AlProfile();
	void AssembleModel();

	Vec3		m_AlProfileDim;
	Vec3		m_bracketDim;
};

class AcrylicPlate : public TaskObject //아크릴판
{
public:
	AcrylicPlate();
	~AcrylicPlate();
	void AssembleModel();

	Vec3		m_AcrylicPlateDim;

};

class PCBPlate : public TaskObject //PCB판
{
public:
	PCBPlate();
	~PCBPlate();
	void AssembleModel();

	Vec3		m_PCBPlateDim;

};



class ScrewPlate : public TaskObject //나사판
{
public:
	ScrewPlate();
	~ScrewPlate();
	void AssembleModel();

	Vec3		m_ScrewPlateDim;

};

class Drill : public TaskObject //드릴
{
public:
	Drill();
	~Drill();
	void AssembleModel();

	Vec3		m_DrillDim;
	Vec3		m_DrillHandleDim;
};