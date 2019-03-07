#ifndef _Franka_H
#define _Franka_H

#include "srDyn\srDYN.h"
#include "srDyn\srRevoluteJoint.h"
#include "srDyn\srPrismaticJoint.h"
#include "srDyn\srWeldJoint.h"
#include "Eigen/Dense"
#include <vector>


#define NUM_OF_LINK_Franka					16		// fix later
#define NUM_OF_MARKERLINK_Franka			2
#define NUM_OF_RJOINT_Franka				7
#define NUM_OF_GRIPERJOINT_Franka			2
#define NUM_OF_WJOINT_Franka				10000
#define DEGREE_OF_FREEDOM_Franka_JOINT 		7
#define NUM_OF_COLLISION_Franka				1000

class Franka_Index
{
public:
	enum Franka_Index_Link_Num
	{
		LINK_0, LINK_1, LINK_2, LINK_3, LINK_4, LINK_5, LINK_6, LINK_7, HAND	 // add gipper links later
	};
	enum Franka_Index_MarkerLink_Num
	{
		MLINK_GRIP
	};
	enum Franka_Index_JOINT_Num
	{
		JOINT_1, JOINT_2, JOINT_3, JOINT_4, JOINT_5, JOINT_6, JOINT_7
	};
	enum Franka_Index_WELDJOINT_Num
	{
		WELDJOINT_HAND, WELDJOINT_GRIP_MARKER		// add if needed later
	};

};

class Franka : public srSystem
{
public:
	Franka(bool elbowUp = true, double gripperRot = 0.0);
	~Franka();

public:

	srLink						gLink[NUM_OF_LINK_Franka];
	srLink						gMarkerLink[NUM_OF_MARKERLINK_Franka];
	srCollision					gCollision[NUM_OF_COLLISION_Franka];
	srRevoluteJoint*			gJoint[NUM_OF_RJOINT_Franka];
	srRevoluteJoint*			gGripJoint[NUM_OF_GRIPERJOINT_Franka];
	srWeldJoint*				gWeldJoint[NUM_OF_WJOINT_Franka];
	

	double UpperJointLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double LowerJointLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double VelocityLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double AccelerationLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double UpperTorqueLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double LowerTorqueLimit[DEGREE_OF_FREEDOM_Franka_JOINT];
	double UpperGripJointLimit[NUM_OF_GRIPERJOINT_Franka];
	double LowerGripJointLimit[NUM_OF_GRIPERJOINT_Franka];




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
