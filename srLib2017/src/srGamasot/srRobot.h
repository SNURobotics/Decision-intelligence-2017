/*******************************************************************/
/*                                                                 */
/*    MADE BY: Keunjun Choi                                        */
/*    Date: 2014-12-14                                             */
/*                                NAMESPACE:     GAMASOT           */
/*                                                                 */
/*******************************************************************/

#ifndef __SR_ROBOT__
#define __SR_ROBOT__

#include <map>
#include <string>
#include <vector>
#include "srDyn/srSpace.h"

using namespace std;

namespace gamasot
{
	class srRobot : public srSystem
	{
	private:
		map < string, srLink* > mLink;
		map < string, srCollision* > mCollision;
		map < string, srJoint* > mJoint;

	public:
		srRobot();
		~srRobot();

		srLink*			getLink(string);
		srCollision*	getCollision(string);
		srJoint*		getJoint(string, srJoint::JOINTTYPE jointType = srJoint::JOINTTYPE::WELD);

		void			setStateJointLimit(vector<double> ulim, vector<double> llim);

	};
}

#endif