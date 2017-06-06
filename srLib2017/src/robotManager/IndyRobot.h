#ifndef _IndyRobot_H
#define _IndyRobot_H

#include "srDyn\srDYN.h"
#include "srDyn\srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srWeldJoint.h"
#include "Eigen/Dense"
#include <vector>


#define NUM_OF_LINK_INDY				13
#define NUM_OF_MARKERLINK_INDY			2
#define NUM_OF_RJOINT_INDY				6
#define NUM_OF_PJOINT_INDY				4
#define NUM_OF_WJOINT_INDY				10000
#define DEGREE_OF_FREEDOM_INDY_JOINT	6
#define NUM_OF_COLLISION_INDY			1000

class Indy_Index
{
public:
	enum Indy_Index_Link_Num
	{
		LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, ENDEFFECTOR, SENSOR, GRIPPER, GRIPPER_FINGER_L, GRIPPER_FINGER_U, GRIPPER_FINGER_L_DUMMY, GRIPPER_FINGER_U_DUMMY
	};
	enum Indy_Index_MarkerLink_Num
	{
		MLINK_GRIP
	};
	enum Indy_Index_JOINT_Num
	{
		JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6
	};
	enum Indy_Index_GRIPPERJOINT_Num
	{
		GRIPJOINT_L, GRIPJOINT_U, GRIPJOINT_L_DUMMY, GRIPJOINT_U_DUMMY
	};
	enum Indy_Index_WELDJOINT_Num
	{
		WELDJOINT_SENSOR, WELDJOINT_GRIPPER, WELDJOINT_GRIP_MARKER
	};
};

class IndyRobot : public srSystem
{
public:
	IndyRobot(double gripperRot = 0.0);
	~IndyRobot();

public:

	srLink						gLink[NUM_OF_LINK_INDY];
	srLink						gMarkerLink[NUM_OF_MARKERLINK_INDY];
	srCollision					gCollision[NUM_OF_COLLISION_INDY];
	srRevoluteJoint*			gJoint[NUM_OF_LINK_INDY];
	srPrismaticJoint*			gPjoint[NUM_OF_PJOINT_INDY];
	srWeldJoint*				gWeldJoint[NUM_OF_WJOINT_INDY];
	

	double UpperJointLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double UpperTorqueLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double LowerTorqueLimit[DEGREE_OF_FREEDOM_INDY_JOINT];
	double UpperPJointLimit[NUM_OF_PJOINT_INDY];
	double LowerPJointLimit[NUM_OF_PJOINT_INDY];




public:
	void	SetActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx);
	void	SetJointLimit();
	void	SetVelocityLimit();
	//	void	SetAccelerationLimit();

	void	SetInitialConfiguration();

	void	AssembleModel(double gripperRot = 0.0);
	void	AssembleCollision();
	void	SetInertia();
	void	SetTorqueLimit();

	Eigen::VectorXd	getLowerJointLimit() const;
	Eigen::VectorXd getUpperJointLimit() const;

public:
	int m_numCollision;
	
};

#endif // !_IndyRobot_H
