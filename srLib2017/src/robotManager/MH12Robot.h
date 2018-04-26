#ifndef _MH12Robot_H
#define _MH12Robot_H

#include "srDyn\srDYN.h"
#include "srDyn\srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srWeldJoint.h"
#include "Eigen/Dense"
#include <vector>


#define NUM_OF_LINK_MH12					16
#define NUM_OF_MARKERLINK_MH12			2
#define NUM_OF_RJOINT_MH12				6
#define NUM_OF_GRIPERJOINT_MH12			3
#define NUM_OF_WJOINT_UR3				10000
#define DEGREE_OF_FREEDOM_MH12_JOINT 	6
#define NUM_OF_COLLISION_MH12			1000

class MH12_Index
{
public:
	enum MH12_Index_Link_Num
	{
		LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, ENDEFFECTOR, COUPLING, GRIPPER_1, GRIPPER_2, GRIPPER_3
	};
	enum MH12_Index_MarkerLink_Num
	{
		MLINK_GRIP
	};
	enum MH12_Index_JOINT_Num
	{
		JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6
	};
	enum MH12_Index_GRIPPERJOINT_Num
	{
		GRIPJOINT_1, GRIPJOINT_2, GRIPJOINT_3
	};
	enum UR3_Index_WELDJOINT_Num
	{
		WELDJOINT_COUPLING, WELDJOINT_GRIP_MARKER
	};

};

class MH12Robot : public srSystem
{
public:
	MH12Robot(bool elbowUp = true, double gripperRot = 0.0);
	~MH12Robot();

public:

	srLink						gLink[NUM_OF_LINK_MH12];
	srLink						gMarkerLink[NUM_OF_MARKERLINK_MH12];
	srCollision					gCollision[NUM_OF_COLLISION_MH12];
	srRevoluteJoint*			gJoint[NUM_OF_RJOINT_MH12];
	srPrismaticJoint*			gGripJoint[NUM_OF_GRIPERJOINT_MH12];
	srWeldJoint*				gWeldJoint[NUM_OF_WJOINT_UR3];
	

	double UpperJointLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double UpperTorqueLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double LowerTorqueLimit[DEGREE_OF_FREEDOM_MH12_JOINT];
	double UpperGripJointLimit[NUM_OF_GRIPERJOINT_MH12];
	double LowerGripJointLimit[NUM_OF_GRIPERJOINT_MH12];




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

#endif // !_MH12Robot_H
