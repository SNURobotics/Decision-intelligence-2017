#pragma once

#include "srDyn/srConstraint.h"


//**********************************************************************//
// Closed Loop
class ClosedLoop : public Constraint
{
public:
	ClosedLoop()
	{
		nd = 6;
		type1 = true;

		lambda[0] = 0.0;
		lambda[1] = 0.0;
		lambda[2] = 0.0;
		lambda[3] = 0.0;
		lambda[4] = 0.0;
		lambda[5] = 0.0;
	};

	// static members
	static void SetErp(double _erp);
	static double erp_closedloop;

	static void SetAllowedPenetration(double _allowedpenetration);
	static double allowederror;

	static void SetMaximumErpVelocity(double _maximum_erp_velocity);
	static double maximum_erp_velocity;

	// member variables
	static dse3 UnitImp[6]; // constraint jacobian : this is same for all closed loop, hence i chose to use static variable.
	double	closedloopError[6];
	double	lambda[6];

	srSystem	* pSystem;
	srLink	* pLeftMass;
	srLink	* pRightMass;

	SE3		HomeRelativeFrame;
	SE3		RelativeFrame;

	// member functions
	void		GetError(double _recip_timestep);

	// virtual functions
	void		GetInformation(ConstraintInfo * info);
	void		ApplyImpulse(int _idx);
	void		GetDelVelocity(double * sjari);
	void		Excite();
	void		UnExcite();
	void		SetImpulse(double * _lambda);
	srSystem*	UF_Find_Constraint();

};
//**********************************************************************//

