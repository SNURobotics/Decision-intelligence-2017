#include "serverPlanningSetting.h"
#include "robotManager/rrtVectorFields.h"
#include "myRenderer.h"
#include <time.h>

myRenderer* renderer;
vector<Eigen::VectorXd> traj1(0);
vector<Eigen::VectorXd> traj2(0);
// rendering functions
void rendering(int argc, char **argv);
void updateFunc();
int main(int argc, char **argv)
{
	srand(time(NULL));
	robot1Setting();
	initDynamics();
	robotManager1Setting();


	rrtSetting();

	singularityAvoidanceVectorField* singAvoidVF = new singularityAvoidanceVectorField;
	singAvoidVF->setRobotEndeffector(rManager1, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);

	double epsilon = 1e-6;
	Eigen::VectorXd testPos(6);
	testPos.setRandom();
	Eigen::VectorXd testVel(6);
	testVel.setZero();

	Eigen::VectorXd singPos(6);
	singPos.setZero();
	singPos[1] = -SR_PI_HALF; singPos[2] = DEG2RAD(0.0); singPos[3] = SR_PI_HALF; singPos[4] = DEG2RAD(0);
	rManager1->setJointVal(singPos);
	cout << rManager1->manipulability(singPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
	cout << rManager1->manipulabilityGradient(singPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]).transpose() << endl;
	
	//Eigen::VectorXd dir = singAvoidVF->getVectorField(singPos);

	//cout << dir.transpose() << endl;

	//for (int i = 0; i < 10; i++)
	//{
	//	testPos.setRandom();
	//	dir = singAvoidVF->getVectorField(testPos);
	//	cout << dir.transpose() << endl;
	//}

	Eigen::VectorXd singPos2(6);
	singPos2.setZero();
	singPos2[1] = -SR_PI_HALF; singPos2[2] = DEG2RAD(0.0); singPos2[3] = SR_PI_HALF; singPos2[4] = DEG2RAD(0);
	
	Eigen::VectorXd initPos = singPos;
	initPos[2] -= 0.5;
	cout << initPos.transpose() << endl;
	cout << rManager1->manipulability(initPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
	cout << initPos.transpose() << endl;
	Eigen::VectorXd goalPos = singPos;
	goalPos[2] += 0.5;
	cout << rManager1->manipulability(goalPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;

	Eigen::MatrixXd Jtest = rManager1->getBodyJacobian(goalPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	Eigen::JacobiSVD<Eigen::MatrixXd> svdtest(Jtest, Eigen::ComputeThinU | Eigen::ComputeThinV);
	cout << "U: " << endl << svdtest.matrixU() << endl;
	cout << "V: " << endl << svdtest.matrixV() << endl;
	cout << "S: " << svdtest.singularValues().transpose() << endl;
	vector<bool> feas = RRTManager1->checkFeasibility(initPos, goalPos);
	RRTManager1->setStartandGoal(initPos, goalPos);

	RRTManager1->execute(0.05);
	traj1 = RRTManager1->extractPath(60);

	rManager1->setJointVal(initPos);

	double manipCost1 = 0.0;
	for (unsigned int i = 0; i < traj1.size(); i++)
	{
		manipCost1 += rManager1->manipulability(traj1[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	}
	cout << "manip cost1: " << manipCost1 << endl;
	
	RRTManager1->addVectorField(singAvoidVF);
	RRTManager1->setVectorFieldWeight(1.0);
	RRTManager1->setStartandGoal(initPos, goalPos);
	RRTManager1->execute(0.05);
	traj2 = RRTManager1->extractPath(60);
	double manipCost2 = 0.0;
	for (unsigned int i = 0; i < traj2.size(); i++)
	{
		manipCost2 += rManager1->manipulability(traj2[i], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	}
	cout << "manip cost2: " << manipCost2 << endl;

	rendering(argc, argv);

	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////
	// check jacobian derivative
	//Eigen::MatrixXd Jb = rManager1->getBodyJacobian(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//Eigen::MatrixXd Jbdot_num;
	//Eigen::MatrixXd Jbdot_ana;
	//cout << "body jacobian: " << endl;
	//cout << Jb << endl << endl;
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jb, Eigen::ComputeThinV);
	//Eigen::VectorXd s = svd.singularValues();
	//Eigen::MatrixXd V = svd.matrixV();
	//Eigen::JacobiSVD<Eigen::MatrixXd> svd2(Jb.transpose()*Jb, Eigen::ComputeThinV);
	//Eigen::VectorXd s2 = svd2.singularValues();
	//Eigen::MatrixXd V2 = svd2.matrixV();
	//Eigen::VectorXd eigGrad(6);
	//cout << "singval: " << s.transpose() << endl;
	//cout << "singvec: " << endl << V << endl;
	//cout << "singval: " << s2.transpose() << endl;
	//cout << "singvec: " << endl << V2 << endl;
	//for (int i = 0; i < 6; i++)
	//{
	//	if (i > 0)
	//	{
	//		testPos[i - 1] -= epsilon;
	//		testVel[i - 1] = 0;
	//	}
	//	testPos[i] += epsilon;
	//	testVel[i] = 1;
	//	cout << "joint " << i << endl;
	//	Eigen::MatrixXd Jbtemp = rManager1->getBodyJacobian(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	Eigen::JacobiSVD<Eigen::MatrixXd> svd(Jbtemp, Eigen::ComputeThinV);
	//	Eigen::VectorXd stemp = svd.singularValues();
	//	//Jbdot_num = (Jbtemp - Jb) / epsilon;
	//	//cout << "numerical deriv: " << endl;
	//	//cout << Jbdot_num << endl;
	//	Jbdot_ana = rManager1->getBodyJacobianDot(testPos, testVel, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	cout << "analytical deriv: " << endl;
	//	cout << Jbdot_ana << endl;
	//}

	/////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////
	// check manipulability gradient
	//for (int testIdx = 0; testIdx < 10; testIdx++)
	//{
	//	testPos.setRandom();
	//	cout << "test pos: " << testPos.transpose() << endl;
	//	double manip0 = rManager1->manipulability(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	
	//	Eigen::VectorXd num_grad(6);
	//	for (int i = 0; i < 6; i++)
	//	{
	//		if (i > 0)
	//			testPos[i - 1] -= epsilon;
	//		testPos[i] += epsilon;
	//		double temp = rManager1->manipulability(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//		num_grad[i] = (temp - manip0) / epsilon;
	//	}
	//	Eigen::VectorXd ana_grad = rManager1->manipulabilityGradient(testPos, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	
	//	cout << "num grad: " << num_grad.transpose() << endl;
	//	cout << "ana grad: " << ana_grad.transpose() << endl << endl;
	//	testPos[5] -= epsilon;
	//	double manip1 = rManager1->manipulability(testPos + ana_grad, &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]);
	//	cout << manip0 << endl;
	//	cout << manip1 << endl;
	//}
	



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

void updateFunc()
{
	static int cnt = 0, trjIdx = 0, trj = 0;
	cnt++;
	if (traj1.size() > 0 && trj%2 == 0)
	{
		if (cnt % 20 == 0)
		{	
			trjIdx++;
			if (trjIdx == 1)
				cout << endl << "trajectory 1" << endl;
			rManager1->setJointVal(traj1[(trjIdx - 1) % traj1.size()]);
			cout << rManager1->manipulability(traj1[(trjIdx - 1) % traj1.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
			if (trjIdx  == traj1.size() + 1)
			{
				trjIdx = 0;
				trj += 1;
			}
		}
	}
	if (traj2.size() > 0 && trj%2 == 1)
	{
		if (cnt % 20 == 0)
		{
			trjIdx++;
			if (trjIdx == 1)
				cout << endl << "trajectory 2" << endl;
			rManager1->setJointVal(traj2[(trjIdx - 1) % traj2.size()]);
			cout << rManager1->manipulability(traj2[(trjIdx - 1) % traj2.size()], &robot1->gMarkerLink[Indy_Index::MLINK_GRIP]) << endl;
			if (trjIdx == traj2.size() + 1)
			{
				trjIdx = 0;
				trj += 1;
			}
		}
	}
}