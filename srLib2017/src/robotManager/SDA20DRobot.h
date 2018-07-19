#ifndef _SDA20D_H_
#define _SDA20D_H_

#include "srDyn/srDYN.h"
#include "srDyn/srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn/srWeldJoint.h"
#include <vector>
#include "../Eigen/Dense"
#include "Math/mathOperator.h"

#define NUM_OF_LINK_SDA20D						20
#define NUM_OF_RJOINT_SDA20D					15
#define NUM_OF_WJOINT_SDA20D					30
#define NUM_OF_PJOINT_SDA20D					4
#define DEGREE_OF_FREEDOM_SDA20D_JOINT			15
#define DEGREE_OF_FREEDOM_SDA20D_RIGHTARM		7
#define DEGREE_OF_FREEDOM_SDA20D_LEFTARM		7
#define DEGREE_OF_FREEDOM_SDA20D_WAIST			1
#define NUM_OF_COLLISION_SDA20D					1000

//#define SHOW_GRIPPER   // 2-finger gripper

class SDA20D_Index
{
public:
	enum SDA20D_Index_Model_Link_Num
	{
		LINK_BASE, LINK_UPPERBASE,
		LINK_RIGHT_SHOULDER, LINK_RIGHT_UPPERARM,
		LINK_RIGHT_LOWERARM, LINK_RIGHT_ELBOW, LINK_RIGHT_WRIST1, LINK_RIGHT_WRIST2, LINK_RIGHT_GRIPPER,
		LINK_RIGHT_GRIPPER_L, LINK_RIGHT_GRIPPER_U,
		LINK_LEFT_SHOULDER, LINK_LEFT_UPPERARM,
		LINK_LEFT_LOWERARM, LINK_LEFT_ELBOW, LINK_LEFT_WRIST1, LINK_LEFT_WRIST2, LINK_LEFT_GRIPPER,
		LINK_LEFT_GRIPPER_L, LINK_LEFT_GRIPPER_U
	};
	enum SDA20D_Index_Model_Joint_Num
	{
		JOINT_WAIST/*0*/,
		JOINT_RIGHT_S/*1*/, JOINT_RIGHT_L, JOINT_RIGHT_E, JOINT_RIGHT_U, JOINT_RIGHT_R, JOINT_RIGHT_B, JOINT_RIGHT_T,
		JOINT_LEFT_S/*8*/, JOINT_LEFT_L, JOINT_LEFT_E, JOINT_LEFT_U, JOINT_LEFT_R, JOINT_LEFT_B, JOINT_LEFT_T,
		
	};

	enum SDA20D_Index_Model_PJoint_Num
	{
		JOINT_RIGHT_GRIPPER_L, JOINT_RIGHT_GRIPPER_U,
		JOINT_LEFT_GRIPPER_L, JOINT_LEFT_GRIPPER_U
	};

	enum SDA20D_Index_Model_WJoint_Num
	{
		JOINT_TEMP = 0
	};

	enum SDA20D_Index_Model_MarkerLink_Num
	{
		MLINK_RIGHT_T, MLINK_LEFT_T, MLINK_RIGHT_GRIPPER, MLINK_LEFT_GRIPPER
	};
};


class SDA20D : public srSystem
{
public:
	SDA20D();
	~SDA20D();
public:
	
	srLink						gLink[NUM_OF_LINK_SDA20D];
	srCollision					gCollision[NUM_OF_COLLISION_SDA20D];
	srRevoluteJoint*			gJoint[NUM_OF_RJOINT_SDA20D];
	srPrismaticJoint*			gPjoint[NUM_OF_PJOINT_SDA20D];
	srWeldJoint*				gWjoint[4];			// only for end-effector
	srLink						gMarkerLink[4];		// only for end-effector

	double UpperJointLimit[DEGREE_OF_FREEDOM_SDA20D_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_SDA20D_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_SDA20D_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_SDA20D_JOINT];


public:
	void	SetActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx);
	void	SetJointLimit();
	void	SetVelocityLimit();
//	void	SetAccelerationLimit();

	void	SetInitialConfiguration();

	void	AssembleModel();
	void	AssembleCollision();



public:
	int				m_numCollision;
	SE3				TsrLinkbase2robotbase;
	Eigen::VectorXd homePos;
	Eigen::VectorXd qInvKinInit;
};






#endif