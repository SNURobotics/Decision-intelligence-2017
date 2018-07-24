#include "retargettingEnvSetting.h"
#include "retargettingEnvSetting.h"

// world frame: srLib's world frame

retargetEnvironment::retargetEnvironment()
{
	 Tworld2camera = EulerZYX(Vec3(-SR_PI_HALF, 0.0, (120.5 / 180.0) * SR_PI), Vec3(0.50 + 0.53, 0.775 - 0.75, 0.72+ 0.82));
	 Tworld2tableCenter = SE3(Vec3(0.0, 0.0, 0.72));
	 Trobotbase_right= SE3(Vec3(-0.7, -0.3, 0.5));
	 Trobotbase_left = SE3(Vec3(-0.7, 0.55, 0.5));

	 // set table
	 m_table = new TableRetarget();
	 m_table->setBaseLinkFrame(SE3(Vec3(0.0, 0.0, 0.72-0.025)));

	 // set working objects
	 m_blueFemaleConnetor = new BlueFemaleConnector();
	 m_blueMaleConnetor = new BlueMaleConnector();
	 m_redFemaleConnetor = new RedFemaleConnector();
	 m_redMaleConnetor = new RedMaleConnector();

	 // set objects position w.r.t. EE
	 TblueFemale2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.01));
	 TblueMale2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(0.0, 0.0, 0.05));
	 TredFemale2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.03, 0.0, -0.03));
	 TredMale2robotEE = EulerZYX(Vec3(0.0, 0.0, SR_PI_HALF), Vec3(-0.03, 0.0, -0.03));


}

retargetEnvironment::~retargetEnvironment()
{
}


void retargetEnvironment::setEnvironmentInSrSpace(srSpace * space)
{

	space->AddSystem(m_table);
	space->AddSystem(m_blueFemaleConnetor);
	space->AddSystem(m_blueMaleConnetor);
	space->AddSystem(m_redFemaleConnetor);
	space->AddSystem(m_redMaleConnetor);

}
