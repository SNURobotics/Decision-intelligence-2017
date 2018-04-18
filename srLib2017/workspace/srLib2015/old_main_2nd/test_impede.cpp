#include <cstdio>

#include "myRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include "ForceCtrlManager\impedanceCtrlManager.h"
#include <time.h>

srSpace gSpace;
myRenderer* renderer;
gamasot::srRobot* robot = new gamasot::srRobot;
robotManager* rManager = new robotManager;
srLink* ee;
gamasot::srRobot* robot2 = new gamasot::srRobot;
robotManager* rManager2 = new robotManager;
srLink* ee2;
srSystem* object1 = new srSystem;
srSystem* obs = new srSystem;
vector<SE3> r2o(1);
impedanceCtrlManager* iManager = new impedanceCtrlManager;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void setObject(srSystem* system, Vec3 dim, SE3 T = SE3(), srSystem::BASELINKTYPE basetype = srSystem::BASELINKTYPE::FIXED);
void setImpede(SE3 TrobotInit, SE3 Tgoal);
void impedanceControl();
void torqueControl();
void accControl();
int main(int argc, char **argv)
{
	srand((unsigned int)time(0));
	gamasot::urdf2srRobot(robot, "../../../workspace/robot", "ur10", "ur10_robot", false);
	vector<double> ulim(6, 360.0);
	vector<double> llim(6, -360.0);
	robot->setStateJointLimit(ulim, llim);
	gSpace.AddSystem((srSystem*)robot);
	SE3 T = SE3(Vec3(0.0, 0.0, 0.0));
	// object for impedance control
	setObject(object1, Vec3(0.1, 0.1, 0.1), T, srSystem::BASELINKTYPE::DYNAMIC);
	// obstacle
	setObject(obs, Vec3(0.05, 0.5, 0.5), SE3(Vec3(1.0, 0.0, 0.250)));

	// robot2
	//gamasot::urdf2srRobot(robot2, "../../../workspace/robot", "ur10", "ur10_robot", false);
	//gSpace.AddSystem((srSystem*)robot2);

	// initialize srLib
	initDynamics();


	// robotManager
	rManager->setRobot(robot);
	rManager->setSpace(&gSpace);
	ee = robot->getLink("wrist_3_link");
	rManager->setEndeffector(ee);

	// robotManager2
	//rManager2->setRobot(robot2);
	//rManager2->setSpace(&gSpace);
	//ee2 = robot->getLink("wrist_3_link");
	//rManager2->setEndeffector(ee2, srJoint::ACTTYPE::HYBRID);

	int a = rManager->m_activeArmInfo->m_numJoint;

	Eigen::VectorXd q = Eigen::VectorXd::Zero(a);
	Eigen::VectorXd dq = Eigen::VectorXd::Zero(a);
	for (int i = 0; i < q.size(); i++)
		q(i) = (double)rand() / RAND_MAX;
	q(0) = 0.0;
	q(1) = -SR_PI_HALF;
	q(2) = 1.5*SR_PI_HALF;
	q(3) = 0.1;
	q(4) = 0.1;
	q(5) = 0.1;
	
	SE3 TrobotInit = rManager->forwardKin(q, ee);
	rManager->setJointVal(q);
	
	SE3 Tgoalpos = SE3(Vec3(0.9, 0.0, 0.25));
	
	// impedance control
	setImpede(TrobotInit, Tgoalpos);
	//q << 0.008357, 0.069792, 0.634724, 1.047380, 0.113987, -0.519954;
	//dq << -0.095037, -0.374906, 1.046498, -0.329421, 0.017085, -0.350580;
	rManager->setJointValVel(q, dq);
	gSpace.SET_USER_CONTROL_FUNCTION(impedanceControl);
	//gSpace.SET_USER_CONTROL_FUNCTION(torqueControl);
	//gSpace.SET_USER_CONTROL_FUNCTION(accControl);
	rendering(argc, argv);

	return 0;
}

void rendering(int argc, char **argv)
{
	renderer = new myRenderer();

	SceneGraphRenderer::NUM_WINDOWS windows;

	windows = SceneGraphRenderer::SINGLE_WINDOWS;

	renderer->InitializeRenderer(argc, argv, windows, false);
	renderer->InitializeNode(&gSpace);
	renderer->setUpdateFunc(updateFunc);

	renderer->RunRendering();
}

void initDynamics()
{
	gSpace.SetTimestep(0.001);
	gSpace.SetGravity(0.0, 0.0, -0.1);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	gSpace.DYN_MODE_RUNTIME_SIMULATION_LOOP();	
}

void setObject(srSystem* system, Vec3 dim, SE3 T, srSystem::BASELINKTYPE basetype)
{
	srLink* link1 = new srLink;
	srWeldJoint* joint1 = new srWeldJoint;
	srLink* link2 = new srLink;
	srCollision* colli2 = new srCollision;
	system->SetBaseLink(link1);
	link1->GetGeomInfo().SetDimension(Vec3(0.0));
	link2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	link2->GetGeomInfo().SetDimension(dim);
	colli2->GetGeomInfo().SetShape(srGeometryInfo::BOX);
	colli2->GetGeomInfo().SetDimension(dim);
	link2->AddCollision(colli2);
	joint1->SetChildLink(link2);
	joint1->SetChildLinkFrame(SE3());
	joint1->SetParentLink(link1);
	joint1->SetParentLinkFrame(SE3(Vec3(0., 0.0, 0.)));
	link1->SetFrame(T);
	system->SetBaseLinkType(basetype);
	gSpace.AddSystem(system);
}

void setImpede(SE3 TrobotInit, SE3 Tgoalpos)
{
	vector<srLink*> ees(1);
	vector<SE3> ofs(1, SE3());
	
	Eigen::MatrixXd mass = 1000.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd res = 100.0 * Eigen::MatrixXd::Identity(6, 6);
	Eigen::MatrixXd cap = 100.0 * Eigen::MatrixXd::Identity(6, 6);
	se3 zero_se3 = se3(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

	ees[0] = rManager->m_activeArmInfo->m_endeffector[0];
	r2o[0] = SE3(Vec3(0.0, 0.15, 0.0));
	object1->GetBaseLink()->SetFrame(TrobotInit*r2o[0]);
	gSpace._KIN_UpdateFrame_All_The_Entity_All_The_Systems();
	iManager->setSystem(rManager, ees, ofs, r2o);
	iManager->setObject(object1);
	iManager->setInertial(mass);
	iManager->setCapacitive(cap);
	iManager->setResistive(res);
	iManager->setGravity(gSpace.m_Gravity);
	iManager->setRobotPostureAmendThreshold(0.01);
	iManager->setTimeStep(gSpace.m_Timestep_dyn_fixed);
	iManager->setDesiredTrajectory(SE3(object1->GetBaseLink()->GetOrientation(), Tgoalpos.GetPosition()), zero_se3, zero_se3);
}

void impedanceControl()
{
	iManager->impedanceControl(actType);
	//object1->GetBaseLink()->AddUserExternalForce(dse3(0.1));
	//iManager->objectImpedanceControl();
	//cout << object1->GetBaseLink()->m_UserExtForce << endl;
	//object1->GetBaseLink()->m_Acc = se3(0.01);

	//object1->GetBaseLink()->AddUserExternalForce(InvdAd(iManager->objectRefFrame, -iManager->getGravitationalForce()));
	//cout << "Fg1 " << object1->GetBaseLink()->GetExtForce() + object1->GetBaseLink()->m_ChildLinks[0]->GetExtForce() << endl;
	//cout << "Fg2 " << iManager->getGravitationalForce() << endl;
	//cout << "Tb " << object1->GetBaseLink()->GetFrame() << endl;
	//cout << "T1 " << object1->m_KIN_Links[1]->GetFrame() << endl;
	//Inertia II = iManager->objectInertia;
	//for (int i = 5; i < 6; i++)
	//{
	//	((srStateJoint*)robot->m_KIN_Joints[i])->m_State.m_rValue[0] += 0.001;
	//}
	static se3 Vbf = se3(0.0);
	static se3 Vcur = se3(0.0);
	static se3 Vbf_ee = se3(0.0);
	static se3 Vcur_ee = se3(0.0);
	static SE3 Tcur_ee = SE3();
	static SE3 Tbf_ee = SE3();
	Vcur = iManager->object->GetBaseLink()->m_Vel;
	Vcur_ee = rManager->m_activeArmInfo->m_endeffector[0]->m_Vel;
	Tcur_ee = rManager->m_activeArmInfo->m_endeffector[0]->GetFrame();
	se3 Aobm, Arom;
	double diff = 0.0;
	if (iManager->controlStep > 0)
	{
		Aobm = (Vcur - Vbf) * (1. / iManager->timeStep);
		
		Vbf = Vcur;
		Arom = InvAd(r2o[0], Vcur_ee - Vbf_ee) * (1. / iManager->timeStep);
		

		//cout << endl;
		//cout << " tau: " << rManager->getJointCommand().transpose() << endl;
		// print manipulability
		diff = 0.0;
		for (int i = 0; i < 6; i++)
			diff += (Aobm - Arom)[i] * (Aobm - Arom)[i];
		if (diff > 0.0001)
		{
			double manip = rManager->manipulability(rManager->getJointVal(), ee);
			cout << "Aobm: " << Aobm;
			cout << "Arom: " << Arom;
			cout << "   q: " << rManager->getJointVal().transpose() << endl;
			cout << "  dq: " << rManager->getJointVel().transpose() << endl;
			
			//printf("manipulability: %f\n", rManager->manipulability(rManager->getJointVal(), ee));
			//cout << rManager->getSpaceJacobian(rManager->getJointVal(), ee) << endl;
		}
			
		//cout << "Vmes: " << Log(Tbf_ee%Tcur_ee) * (1. / iManager->timeStep);
		//cout << "Vrob: " << Vcur_ee << endl;
		//cout << "Arob: " << (Vcur_ee - Vbf_ee) * (1. / iManager->timeStep) << endl;
		
		Vbf_ee = Vcur_ee;
		Tbf_ee = Tcur_ee;
		
	}


	//cout << object1->GetBaseLink()->m_Vel << endl;
	//double dist = Norm(object1->GetBaseLink()->GetPosition() - robot->getLink("wrist_3_link")->GetPosition());
	//if (iManager->controlStep % 2 == 0)
	//	printf("dist: %f\n", dist);
}

void torqueControl()
{
	Eigen::VectorXd ddq = Eigen::VectorXd::Random(rManager->m_activeArmInfo->m_numJoint);
	Eigen::VectorXd tau = rManager->inverseDyn(rManager->getJointVal(), rManager->getJointVel(), ddq);
	rManager->controlJointTorque(tau);
	static Eigen::VectorXd dqcur = Eigen::VectorXd::Zero(rManager->m_activeArmInfo->m_numJoint);
	static Eigen::VectorXd dqbf = Eigen::VectorXd::Zero(rManager->m_activeArmInfo->m_numJoint);
	static int controlStep = 0;
	dqcur = rManager->getJointVel();
	if (controlStep > 0)
	{
		cout << "ddqmes: " << (dqcur - dqbf).transpose() * (1. / gSpace.m_Timestep_dyn_fixed) << endl;
		//cout << "ddqme2: " << rManager->getJointAcc().transpose() << endl;
		cout << "ddqdes: " << ddq.transpose() << endl;
	}
	dqbf = dqcur;
	controlStep++;
}

void accControl()
{
	Eigen::VectorXd taum = rManager->getJointTorque();
	Eigen::VectorXd ddq = Eigen::VectorXd::Ones(rManager->m_activeArmInfo->m_numJoint);
	Eigen::VectorXd tau = rManager->inverseDyn(rManager->getJointVal(), rManager->getJointVel(), ddq);
	rManager->controlJointAcc(ddq);
	static int controlStep = 0;
	if (controlStep > 0)
	{
		cout << "ddqmes: " << rManager->getJointAcc().transpose() << endl;
		cout << "taumes: " << taum.transpose() << endl;
		cout << "ddqdes " << ddq.transpose() << endl;
		cout << "taudes: " << tau.transpose() << endl;	
	}
	controlStep++;
}
