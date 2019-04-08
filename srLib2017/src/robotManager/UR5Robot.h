#ifndef _UR5Robot_H
#define _UR5Robot_H

#include "srDyn\srDYN.h"
#include "srDyn\srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srWeldJoint.h"
#include "Eigen/Dense"
#include <vector>

#define _3_Finger

#ifndef _3_Finger
#define NUM_OF_LINK_UR5					16
#define NUM_OF_MARKERLINK_UR5			2
#define NUM_OF_RJOINT_UR5				6
#define NUM_OF_GRIPERJOINT_UR5			3
#define NUM_OF_WJOINT_UR3				10000
#define DEGREE_OF_FREEDOM_UR5_JOINT 	6
#define NUM_OF_COLLISION_UR5			1000
#endif

#ifdef _3_Finger
#define NUM_OF_LINK_UR5					21
#define NUM_OF_MARKERLINK_UR5			2
#define NUM_OF_RJOINT_UR5				6
#define NUM_OF_GRIPERJOINT_UR5			13
#define NUM_OF_WJOINT_UR5				10000
#define DEGREE_OF_FREEDOM_UR5_JOINT 	6
#define NUM_OF_COLLISION_UR5			1000
#endif

class UR5_Index
{
public:
	enum UR5_Index_Link_Num
	{
		LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, ENDEFFECTOR, 
#ifndef _3_Finger
		GRIPPER,
#endif
#ifdef _3_Finger
		GRIPPER_WRIST, GRIPPER_PALM,
		GRIPPER_FINGER_BASE_1, GRIPPER_FINGER_PROX_1, GRIPPER_FINGER_MED_1, GRIPPER_FINGER_DIST_1,
		GRIPPER_FINGER_BASE_2, GRIPPER_FINGER_PROX_2, GRIPPER_FINGER_MED_2, GRIPPER_FINGER_DIST_2,
		GRIPPER_FINGER_BASE_3, GRIPPER_FINGER_PROX_3, GRIPPER_FINGER_MED_3, GRIPPER_FINGER_DIST_3
#endif
	};
	enum UR5_Index_MarkerLink_Num
	{
		MLINK_GRIP
	};
	enum UR5_Index_JOINT_Num
	{
		JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6
	};
#ifdef _3_Finger
	enum UR3_Index_GRIPPERJOINT_Num
	{
		GRIPJOINT_Pal_1, GRIPJOINT_B_1, GRIPJOINT_P_1, GRIPJOINT_M_1,
		GRIPJOINT_Pal_2, GRIPJOINT_B_2, GRIPJOINT_P_2, GRIPJOINT_M_2,
		GRIPJOINT_Pal_3, GRIPJOINT_B_3, GRIPJOINT_P_3, GRIPJOINT_M_3
	};
#endif
	enum UR3_Index_WELDJOINT_Num
	{
#ifndef _3_Finger
		WELDJOINT_COUPLING, WELDJOINT_GRIPJOINT_1, WELDJOINT_GRIPJOINT_2, WELDJOINT_GRIPJOINT_3, WELDJOINT_GRIP_MARKER
#else
		WELDJOINT_COUPLING, GRIPJOINT_W_1, WELDJOINT_GRIP_MARKER
#endif
	};

};

class UR5Robot : public srSystem
{
public:
	UR5Robot(bool elbowUp = true, double gripperRot = 0.0);
	~UR5Robot();

public:

	srLink						gLink[NUM_OF_LINK_UR5];
	srLink						gMarkerLink[NUM_OF_MARKERLINK_UR5];
	srCollision					gCollision[NUM_OF_COLLISION_UR5];
	srRevoluteJoint*			gJoint[NUM_OF_RJOINT_UR5];
	srRevoluteJoint*			gGripJoint[NUM_OF_GRIPERJOINT_UR5];
	srWeldJoint*				gWeldJoint[NUM_OF_WJOINT_UR5];


	double UpperJointLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double UpperTorqueLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double LowerTorqueLimit[DEGREE_OF_FREEDOM_UR5_JOINT];
	double UpperGripJointLimit[NUM_OF_GRIPERJOINT_UR5];
	double LowerGripJointLimit[NUM_OF_GRIPERJOINT_UR5];




public:
	void	SetActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType = srJoint::HYBRID);
	void	SetGripperActType(srJoint::ACTTYPE actType, vector<int> gpJointIdx);
	void	SetJointLimit(bool elbowUp = true);
	void	SetVelocityLimit();
	//	void	SetAccelerationLimit();

	void	SetInitialConfiguration();

	void	AssembleModel(double gripperRot = 0.0);
	void	AssembleCollision();
	void	SetInertia();
	void	SetTorqueLimit();

	Eigen::VectorXd	getLowerJointLimit() const;
	Eigen::VectorXd getUpperJointLimit() const;
	Eigen::VectorXd homePos;
	Eigen::VectorXd qInvKinInit;
	SE3				TsrLinkbase2robotbase;

public:
	int m_numCollision;

};

#endif // !_UR5Robot_H
