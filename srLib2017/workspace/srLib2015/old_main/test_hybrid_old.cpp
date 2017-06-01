#include <cstdio>

#include "2ndRenderer.h"
#include "srDyn/srDYN.h"
#include "srGamasot\srURDF.h"
#include "robotManager\robotManager.h"
#include "ForceCtrlManager\hybridPFCtrlManager_old.h"
#include <time.h>

srSpace gSpace;
myRenderer* renderer;
gamasot::srRobot* robot = new gamasot::srRobot;
robotManager* rManager = new robotManager;
srLink* ee;
srSystem* obs = new srSystem;
srJoint::ACTTYPE actType = srJoint::ACTTYPE::TORQUE;
hybridPFCtrlManagerPlane* hctrl = new hybridPFCtrlManagerPlane();
void initDynamics();
void rendering(int argc, char **argv);
void updateFunc();
void setCollision(srLink* link);
void setObject(srSystem* system, Vec3 dim, SE3 T = SE3(), srSystem::BASELINKTYPE basetype = srSystem::BASELINKTYPE::FIXED);
void setSphere(srSystem* system, double dim, SE3 T, Vec3 color);
void setHybridPFCtrl(Eigen::VectorXd q0, Vec3 contactPoint);
void testContactJacobian();
int main(int argc, char **argv)
{
	srand((unsigned int)time(0));

	SE3 Tbase = SE3(Vec3(-0.5, 0.0, 0.0));

	gamasot::urdf2srRobot(robot, "../../../workspace/robot", "ur10", "ur10_robot", false);
	vector<double> ulim(6, 360.0);
	vector<double> llim(6, -360.0);
	robot->setStateJointLimit(ulim, llim);
	robot->GetBaseLink()->SetFrame(Tbase);
	gSpace.AddSystem((srSystem*)robot);

	// attach collision to end-effector
	ee = robot->getLink("wrist_3_link");
	setCollision(ee);
	srLink* circ = new srLink;
	circ->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	circ->GetGeomInfo().SetColor(0.0, 0.0, 0.5, 0.0);
	circ->GetGeomInfo().SetDimension(0.05);
	circ->SetInertia(Inertia(0.0));
	srWeldJoint* wjoint = new srWeldJoint;
	wjoint->SetChildLink(circ);
	wjoint->SetChildLinkFrame(SE3(Vec3(-0.05, 0.0, 0.0)));
	wjoint->SetParentLink(ee);
	wjoint->SetParentLinkFrame(SE3(0.0));

	// obstacle
	setObject(obs, Vec3(0.05, 0.7, 0.7), SE3(Tbase * Vec3(1.0, 0.0, 0.350)));
	srSystem* sph1 = new srSystem;
	setSphere(sph1, 0.05, SE3(), Vec3(1.0, 0.0, 0.0));
	srSystem* sph2 = new srSystem;
	setSphere(sph2, 0.05, SE3(), Vec3(0.0, 1.0, 0.0));

	// initialize srLib
	initDynamics();

	// robotManager
	rManager->setRobot(robot);
	rManager->setSpace(&gSpace);
	rManager->setEndeffector(ee);
	
	// set initial config
	Eigen::VectorXd q_config;
	vector<SE3> Tconfig(1);
	Tconfig[0] = SE3(Vec3(-0.5*(obs->GetBaseLink()->m_ChildLinks[0]->GetGeomInfo().GetDimension()[0] + ee->m_Collisions[0]->GetGeomInfo().GetDimension()[0]), 0.0, 0.0)) * obs->GetBaseLink()->GetFrame();
	vector<srLink*> links(1);
	links[0] = ee;
	vector<bool> includeOri(1, true);
	vector<SE3> offset(1, SE3());
	int flag = 0;
	q_config = rManager->inverseKin(Tconfig, links, includeOri, offset, flag);
	vector<SE3> Tinit(1);
	Tinit[0] = SE3(Vec3(-0.01, 0.0, 0.0))*Tconfig[0];
	Eigen::VectorXd q0 = rManager->inverseKin(Tinit, links, includeOri, offset, flag, q_config);
	cout << "q0: " << q0.transpose() << endl;
	sph1->GetBaseLink()->SetFrame(Tconfig[0]);
	
	Vec3 contactPoint = obs->GetBaseLink()->GetFrame().GetPosition();
	contactPoint[0] -= 0.5*(obs->GetBaseLink()->m_ChildLinks[0]->GetGeomInfo().GetDimension()[0]);
	setHybridPFCtrl(q_config, contactPoint);

	// set desired trajectory (trajectory of the end-effector)
	vector<SE3> Tdes(1);
	Tdes[0] = Tconfig[0] * EulerZYX(Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.1, 0.1));
	cout << "Tdes = " << endl << Tdes[0] << endl;
	vector<dse3> Fdes(1, dse3(0.0));
	Fdes[0][0] = 0.00;
	Fdes[0][1] = 0.0;
	Fdes[0][5] = 0.1;
	sph2->GetBaseLink()->SetFrame(Tdes[0]);
	

	hctrl->isDesTrjSet = hctrl->setDesiredTraj(Tdes, Fdes);
	hctrl->setDesiredJointVal(q_config);
	rManager->setJointValVel(q_config, Eigen::VectorXd::Zero(q_config.size()));


	////////////////// for test ////////////////////////
	//for (int i = 0; i < 5; i++)
	//	testContactJacobian();
	//clock_t start, end;
	//Eigen::MatrixXd AA = Eigen::MatrixXd::Random(6, 6);
	//Eigen::VectorXd bb = Eigen::VectorXd::Random(6);
	//Eigen::VectorXd xx1;
	//Eigen::VectorXd xx2;
	//int num = 10000;
	//start = clock();
	//for (int i = 0; i < num; i++)
	//	xx1 = AA.inverse()*bb;
	//end = clock();
	//printf("t = %f\n", (double)(end - start) / CLOCKS_PER_SEC);
	//start = clock();
	//for (int i = 0; i < num; i++)
	//	xx2 = AA.colPivHouseholderQr().solve(bb);
	//end = clock();
	//printf("t = %f\n", (double)(end - start) / CLOCKS_PER_SEC);
	///////////////////////////////////////////////////
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
	gSpace.SetTimestep(0.01);
	gSpace.SetGravity(0.0, 0.0, -0.0);
	gSpace.SetNumberofSubstepForRendering(1);
	gSpace.DYN_MODE_PRESTEP();
}

void updateFunc()
{
	hctrl->hybridPFControl();
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

void setCollision(srLink * link)
{
	srCollision* colli = new srCollision;
	link->AddCollision(colli);
	if (link->GetGeomInfo().GetShape() < 4)
	{
		colli->GetGeomInfo().SetShape(link->GetGeomInfo().GetShape());
		colli->GetGeomInfo().SetDimension(link->GetGeomInfo().GetDimension());
	}
	else
	{
		colli->GetGeomInfo().SetShape(srGeometryInfo::BOX);
		colli->GetGeomInfo().SetDimension(Vec3(0.1));
	}
}

void setSphere(srSystem * system, double dim, SE3 T, Vec3 color)
{
	srLink* link1 = new srLink;
	srWeldJoint* joint1 = new srWeldJoint;
	srLink* link2 = new srLink;
	srCollision* colli2 = new srCollision;
	system->SetBaseLink(link1);
	link1->GetGeomInfo().SetDimension(dim);
	link1->GetGeomInfo().SetShape(srGeometryInfo::SPHERE);
	link1->GetGeomInfo().SetColor(color[0], color[1], color[2]);
	link1->SetFrame(T);
	system->SetBaseLinkType(srSystem::BASELINKTYPE::FIXED);
	gSpace.AddSystem(system);
}

void setHybridPFCtrl(Eigen::VectorXd q0, Vec3 contactPoint)
{
	hctrl->isSystemSet = hctrl->setSystem(rManager, ee);
	hctrl->setTimeStep(rManager->m_space->m_Timestep_dyn_fixed);
	hctrl->isContactOriSet = hctrl->setContactFrameOri(obs, Vec3(-1.0, 0.0, 0.0));
	hctrl->isICSet = hctrl->setInitialConfig(q0, contactPoint);
	double kv = 0.25e1, kp = 0.25*kv*kv, ki = 1.0e-1;
	hctrl->Kp_v = kp * Eigen::MatrixXd::Identity(3, 3);
	hctrl->Kv_v = kv * Eigen::MatrixXd::Identity(3, 3);
	hctrl->Ki_f = ki * Eigen::MatrixXd::Identity(3, 3);
}

void testContactJacobian()
{
	// set initial config
	Eigen::VectorXd q0;
	vector<SE3> Tgoal(1);
	Tgoal[0] = SE3(Vec3(-0.5*(obs->GetBaseLink()->m_ChildLinks[0]->GetGeomInfo().GetDimension()[0] + ee->m_Collisions[0]->GetGeomInfo().GetDimension()[0]), 0.0, 0.0)) * obs->GetBaseLink()->GetFrame();
	vector<srLink*> links(1);
	links[0] = ee;
	vector<bool> includeOri(1, true);
	vector<SE3> offset(1, SE3());
	int flag = 0;
	q0 = rManager->inverseKin(Tgoal, links, includeOri, offset, flag);
	Vec3 contactPoint = obs->GetBaseLink()->m_ChildLinks[0]->GetFrame().GetPosition();
	contactPoint[0] -= 0.5*(obs->GetBaseLink()->m_ChildLinks[0]->GetGeomInfo().GetDimension()[0]);
	setHybridPFCtrl(q0, contactPoint);

	Eigen::VectorXd dq = Eigen::VectorXd::Random(q0.size());
	Eigen::VectorXd ddq = Eigen::VectorXd::Random(q0.size());
	
	// test contact jacobian
	double dt = 0.0001;
	Eigen::VectorXd dq_next = dq + ddq*dt;
	Vec3 temp;
	temp = hctrl->getEndeffectorToContactFrame(q0).GetPosition();
	SE3 Tc1 = rManager->forwardKin(q0, ee, SE3(temp));
	temp = hctrl->getEndeffectorToContactFrame(q0 + dt*dq).GetPosition();
	SE3 Tc2 = rManager->forwardKin(q0 + dt*dq, ee, SE3(temp));
	cout << Tc1 << endl << endl;
	cout << Tc2 << endl << endl;
	Eigen::VectorXd Vc1_num = se3toVector(Log(Tc1 % Tc2)) * (1.0 / dt);
	SE3 Tec1 = hctrl->getEndeffectorToContactFrame(q0);
	SE3 Tec2 = hctrl->getEndeffectorToContactFrame(q0 + dt*dq);
	se3 Vce = Log(Tec1 % Tec2) * (-1.0 / dt);
	Eigen::MatrixXd Jc1 = hctrl->getJacobianofContactFrame(q0);
	Eigen::VectorXd Vc1 = Jc1 * dq;

	cout << Vc1_num << endl << endl;
	cout << Vc1 << endl << endl;

	Eigen::MatrixXd Jc2 = hctrl->getJacobianofContactFrame(q0 + dt*dq);
	Eigen::MatrixXd dJc_num = (Jc2 - Jc1) / dt;
	Eigen::MatrixXd dJc = hctrl->getJacobianDotofContactFrame(q0, dq, &Tec1, Jc1);
	Eigen::VectorXd Vc2 = Jc2*dq_next;
	cout << Vce << endl;
	cout << dJc_num << endl << endl;
	cout << dJc << endl << endl;

	Eigen::VectorXd dVc1 = Jc1*ddq + dJc*dq;
	Eigen::VectorXd dVc1_num = (Vc2 - Vc1) / dt;

	cout << dVc1.transpose() << endl;
	cout << dVc1_num.transpose() << endl;
}
