#ifndef _UR3Robot_H
#define _UR3Robot_H

#include "srDyn\srDYN.h"
#include "srDyn\srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srWeldJoint.h"
#include "Eigen/Dense"
#include <vector>


#define NUM_OF_LINK_UR3					16
#define NUM_OF_MARKERLINK_UR3			2
#define NUM_OF_RJOINT_UR3				6
#define NUM_OF_GRIPERJOINT_UR3			6
#define NUM_OF_WJOINT_UR3				10000
#define DEGREE_OF_FREEDOM_UR3_JOINT 	6
#define NUM_OF_COLLISION_UR3			1000

class UR3_Index
{
public:
	enum UR3_Index_Link_Num
	{
		LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, ENDEFFECTOR, SENSOR, COUPLING, GRIPPER, GRIPPER_FINGER_M, GRIPPER_FINGER_P, GRIPPER_LINK_1_M, GRIPPER_LINK_1_P, GRIPPER_LINK_2_M, GRIPPER_LINK_2_P
	};
	enum UR3_Index_MarkerLink_Num
	{
		MLINK_GRIP
	};
	enum UR3_Index_JOINT_Num
	{
		JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6
	};
	enum UR3_Index_GRIPPERJOINT_Num
	{
		GRIPJOINT_1_M, GRIPJOINT_1_P, GRIPJOINT_2_M, GRIPJOINT_2_P, GRIPJOINT_F_M, GRIPJOINT_F_P
	};
	enum UR3_Index_WELDJOINT_Num
	{
		WELDJOINT_SENSOR, WELDJOINT_COUPLING, WELDJOINT_GRIPPER, WELDJOINT_GRIP_MARKER
	};

};

class UR3Robot : public srSystem
{
public:
	UR3Robot(bool elbowUp = true, double gripperRot = 0.0);
	~UR3Robot();

public:

	srLink						gLink[NUM_OF_LINK_UR3];
	srLink						gMarkerLink[NUM_OF_MARKERLINK_UR3];
	srCollision					gCollision[NUM_OF_COLLISION_UR3];
	srRevoluteJoint*			gJoint[NUM_OF_RJOINT_UR3];
	srRevoluteJoint*			gGripJoint[NUM_OF_GRIPERJOINT_UR3];
	srWeldJoint*				gWeldJoint[NUM_OF_WJOINT_UR3];
	

	double UpperJointLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double UpperTorqueLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double LowerTorqueLimit[DEGREE_OF_FREEDOM_UR3_JOINT];
	double UpperGripJointLimit[NUM_OF_GRIPERJOINT_UR3];
	double LowerGripJointLimit[NUM_OF_GRIPERJOINT_UR3];




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

#endif // !_IndyRobot_H
