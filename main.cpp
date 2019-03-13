#include <iostream>
#include <fstream>
#include <string>
#include "vrep_bridge.h"

//for robot model
#include "robot_model.h"

//for controller 
#include "Inverse-dynamics.h"

// for tasks
#include "task-com.h"
#include "task-operational.h"
#include "task-joint-posture.h"
#include "task-joint-bounds.h"
#include "task-transition.h"
#include "task-mobile.h"
#include "task-singularity.h"
// for trajectories 
#include "trajectory-operationalspace.h"
#include "trajectory-jointspace.h"

// for solver
#include "solver-HQP-factory.hxx"
#include "solver-utils.h"

// for contact point
#include "contact-3d.h"

// for util
#include "utils.h"
#include "container.h"
#include <string>
#include <vector>
#include <conio.h> // for keyboard hit
#include <fstream>
FILE *joint = fopen("joint_pos.txt", "w");
FILE *task_error_j = fopen("task_joint_error.txt", "w");
FILE *task_error_ee = fopen("task_ee_error.txt", "w");
FILE *task_error_m = fopen("task_mobile_error.txt", "w");

// for time check
//#define timer
#ifdef timer
#include <windows.h>
	double PCFreq = 0.0;
	__int64 CounterStart = 0;

	void StartCounter()
	{
		LARGE_INTEGER li;
		if (!QueryPerformanceFrequency(&li))
			cout << "QueryPerformanceFrequency failed!\n";

		PCFreq = double(li.QuadPart) / 1000.0;

		QueryPerformanceCounter(&li);
		CounterStart = li.QuadPart;
	}
	double GetCounter()
	{
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return double(li.QuadPart - CounterStart) / PCFreq;
	}
#endif // timer
using namespace std;




HQP::robot::RobotModel * robot_;
HQP::InverseDynamics * invdyn_, *invdyn_total_, *invdyn2_;
HQP::tasks::TaskJointPosture * jointTask, *jointTask2;
HQP::tasks::TaskOperationalSpace * moveTask, *move2Task;
HQP::tasks::TaskJointLimit * jointLimitTask;
HQP::contact::Contact3dPoint * contactTask;
HQP::tasks::TaskSingularityAvoidance * singularTask;
HQP::tasks::TaskMobile * mobileTask;


HQP::trajectories::TrajectoryJointCubic * trajPosture, *trajPosture2;
HQP::trajectories::TrajectoryJointConstant * trajPostureConstant;
HQP::trajectories::TrajectoryOperationCubic * trajEECubic, *trajmobile;
HQP::trajectories::TrajectoryOperationConstant * trajEEConstant;
VectorXd q(dof);
VectorXd qdot(dof);
VectorXd qdes(dof);
VectorXd q_lb(dof + 2); // mobile 2 + robot 7
VectorXd q_ub(dof + 2); // mobile 2 + robot 7
VectorXd wheel_prev(2);
VectorXd wheel_curr(2);
VectorXd tau;
VectorXd dv;
double vrep_time = 0.0;
double Hz = 1000.0;
int na;
int nv;
int nq;
bool add_ = true;
using namespace HQP;
using namespace std;
double beta1;
double beta2;
double wheel_r=0.0;
double wheel_r_prev=0.0;
double wheel_l = 0.0;;
double wheel_l_prev = 0.0;
solver::HQPData HQP_data1;
solver::HQPData HQP_data2;
solver::HQPData HQP_data3;
solver::HQPOutput sol_total_;
solver::HQPOutput sol_1;
solver::HQPOutput sol_2;
solver::HQPOutput sol_3;


using namespace HQP;


int main()
{

	// for robot
	robot_ = new HQP::robot::RobotModel(1);
	na = robot_->na();
	nv = robot_->nv();

	q.setZero();
	qdot.setZero();	

	solver::SolverHQPBase * solver = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog1");
	solver::SolverHQPBase * solver_2 = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog2");
	solver::SolverHQPBase * solver_3 = solver::SolverHQPFactory::createNewSolver(solver::SOLVER_HQP_QPOASES, "solver-eiquadprog3");


	q_lb = -180.0 / 180.0 * 3.14 * VectorXd(dof + 2).setOnes();
	q_ub = -1.0*q_lb;

	q_lb.head(2) = -500.0 * VectorXd(2).setOnes();
	q_ub.head(2) = 500.0 * VectorXd(2).setOnes();

	double kp_jointlimit = 100.0, w_jointlimit = 1.00;
	jointLimitTask = new tasks::TaskJointLimit("joint_limit_task", *robot_);
	jointLimitTask->Kp(kp_jointlimit*VectorXd::Ones(robot_->nv()));
	jointLimitTask->Kd(2.0*jointLimitTask->Kp().cwiseSqrt());
	jointLimitTask->setJointLimit(q_lb, q_ub);

	////////////////// Joint Posture Task ////////////////////
	jointTask = new tasks::TaskJointPosture("joint_control_task", *robot_);
	double kp_posture = 400.0, w_posture = 1.00;
	jointTask->Kp(kp_posture*VectorXd::Ones(robot_->nv()-2));
	jointTask->Kd(2.0*jointTask->Kp().cwiseSqrt());


	jointTask2 = new tasks::TaskJointPosture("joint_control_task2", *robot_);
	jointTask2->Kp(kp_posture*VectorXd::Ones(robot_->nv()-2));
	jointTask2->Kd(2.0*jointTask2->Kp().cwiseSqrt());


	////////////////// Operational Task ////////////////////
	moveTask = new tasks::TaskOperationalSpace("end_effector_task", *robot_, 7);
	double kp_move = 1000.0, w_move = 1.0;
	VectorXd a = VectorXd::Ones(6);
	//a.tail(3) *= 10.0;
	moveTask->Kp(kp_move*a);
	moveTask->Kd(2.0*moveTask->Kp().cwiseSqrt());
	moveTask->setSingular(false);

	move2Task = new tasks::TaskOperationalSpace("end_effector_task2", *robot_, 7);
	move2Task->Kp(kp_move*a);
	move2Task->Kd(2.0*move2Task->Kp().cwiseSqrt());
	move2Task->setSingular(true);

	//////////////////// Mobile Task ///////////////////////
	mobileTask = new tasks::TaskMobile("mobile_task", *robot_);
	double kp_mobile = 100.0, w_mobile = 1.0;
	mobileTask->Kp(kp_mobile*VectorXd::Ones(6));
	mobileTask->Kd(2.0*mobileTask->Kp().cwiseSqrt());
	mobileTask->setOnlyOriCTRL(false);

	trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
	trajectories::TrajectorySample sampleJoint(robot_->nv()-2);
	trajectories::TrajectorySample sampleJoint2(robot_->nv()-2);
	trajectories::TrajectorySample s(12, 6);
	trajectories::TrajectorySample s_mobile(12, 6);
	trajEECubic = new trajectories::TrajectoryOperationCubic("operational_traj");
	trajectories::TrajectorySample sampleEE(12, 6);

	// for v-rep
	VRepBridge vb;

	vb.isSimulationRun = false;
	vb.exitFlag = false;
	double start_time = 0.0;
	VectorXd q_init(dof);
	bool flag = false;
	VectorXd tau(dof);

	while (vb.simConnectionCheck() && !vb.exitFlag)
	{
		if (_kbhit()) {
			int key = _getch();
			switch (key)
			{
			case '\t':
				if (vb.isSimulationRun) {
					cout << "Simulation Pause" << endl;
					vb.isSimulationRun = false;
				}
				else {
					cout << "Simulation Run" << endl;
					vb._cntt = 0;
					vb.isSimulationRun = true;
				}
				break;

			case 'q':
				vb.isSimulationRun = false;
				vb.exitFlag = true;
				simxEndDialog(vb.getClientID(), vb.dialog_handle, simx_opmode_oneshot);
				break;

			default:
				break;
			}
		}
		if (vb.isSimulationRun)
		{
			vb.read();
			
			if (vb._cntt > 0)
			{

				vrep_time = vb._cntt / Hz;

				VectorXd q_current(robot_->na() + 5), qdot_current(robot_->na() + 5);

				q_current.setZero();
				q_current.head<2>() = vb.H_transform_.translation().head(2);
				q_current(2) = vb.euler_(2);
				q_current(3) = 0.0;
				q_current(4) = 0.0;
				q_current.tail<dof>() = vb.current_q_;

				qdot_current.setZero();
				qdot_current.head<2>() = vb.H_vel_.linear().head(2);
				qdot_current(2) = vb.H_vel_.angular()(2);
				qdot_current(3) = vb.current_base_vel_(0);
				qdot_current(4) = vb.current_base_vel_(1);
				qdot_current.tail(dof) = vb.current_qdot_;

				robot_->getUpdateKinematics(q_current, qdot_current);
				robot_->getMobilePos(vb.mobile_center);
				Transform3d Tdes;
				if (vrep_time == 1.0 / Hz) {
					cout << "Start HQP Controller" << endl;
					cout << "T_init" << robot_->getTransformation(7).translation().transpose() << endl;
				

					Transform3d T_endeffector;
					T_endeffector = robot_->getTransformation(7);
					trajEECubic->setInitSample(T_endeffector);

					T_endeffector.translation()(0) += 0.2;

					trajEECubic->setGoalSample(T_endeffector);
					trajEECubic->setDuration(0.0);
					trajEECubic->setStartTime(0.0);
					trajEECubic->setReference(T_endeffector);

					Transform3d T_mobile;
					T_mobile = robot_->getMobileTransformation();

					trajmobile = new trajectories::TrajectoryOperationCubic("op_traj");
					trajmobile->setInitSample(T_mobile);
					T_mobile.translation()(0) += 0.0;
					T_mobile.translation()(1) += 0.0;
					cout << T_mobile.translation() << endl;

					trajmobile->setGoalSample(T_mobile);
					trajmobile->setDuration(1.0);
					trajmobile->setStartTime(start_time);

					q_init = q_current.tail(dof);
					qdes.setZero();
					qdes(1) = M_PI / 4.0;
					qdes(3) = -M_PI / 2.0;
					qdes(5) = M_PI / 2.0;

					trajPosture = new trajectories::TrajectoryJointCubic("joint_traj");
					trajPosture->setInitSample(q_init);
					trajPosture->setGoalSample(qdes);
					trajPosture->setDuration(6.0);
					trajPosture->setStartTime(3.0);
					trajPosture->setReference(qdes);

					qdes.setZero();
					qdes(1) = -M_PI/6.0;
					qdes(3) = -2.0*M_PI / 3.0;
					qdes(5) = M_PI / 2.0;
					trajPosture2 = new trajectories::TrajectoryJointCubic("joint_traj");
					trajPosture2->setInitSample(q_init);
					trajPosture2->setGoalSample(qdes);
					trajPosture2->setDuration(0.0);
					trajPosture2->setStartTime(0.0);
					trajPosture2->setReference(qdes);


					invdyn_total_ = new HQP::InverseDynamics(*robot_);
					invdyn2_ = new HQP::InverseDynamics(*robot_);
					invdyn_ = new HQP::InverseDynamics(*robot_);
					// task 1 : mobile
					// task 2 : joint 
					// task 3 : op 

					invdyn_->addOperationalTask(*moveTask, w_move, 2.0, 0.0);
					invdyn_->addMotionTask(*mobileTask, w_mobile, 1, 0.0);
					invdyn_->addJointPostureTask(*jointTask2, 0.0, 0.0, 0.0); //weight, level, duration

					invdyn2_->addOperationalTask(*moveTask, w_move, 1.0, 0.0);
					invdyn2_->addMotionTask(*mobileTask, w_mobile, 2, 0.0);
					invdyn2_->addJointPostureTask(*jointTask2, 0.0, 0.0, 0.0); //weight, level, duration

					invdyn_total_->addOperationalTask(*moveTask, w_move, 2.0, 0.0);
					invdyn_total_->addMotionTask(*mobileTask, w_mobile, 1, 0.0);
					invdyn_total_->addJointPostureTask(*jointTask2, 1.0, 0.0, 0.0); //weight, level, duration


				}
#ifdef timer
				StartCounter();
#endif
				Transform3d Current = robot_->getTransformation(7);
				trajEECubic->setCurrentTime(vrep_time);
				s = trajEECubic->computeNext();
				moveTask->setReference(s);

				trajPosture->setCurrentTime(vrep_time);
				sampleJoint = trajPosture->computeNext();

				trajmobile->setCurrentTime(vrep_time);
				s_mobile = trajmobile->computeNext();
				mobileTask->setReference(s_mobile);

				trajPosture2->setCurrentTime(vrep_time);
				sampleJoint2 = trajPosture2->computeNext();
				jointTask2->setReference(sampleJoint2);
				double current_time = vrep_time - start_time;


				// first the order of SOT is Task2 < Task1< Task3 
				// then Task2 < Task3 < Task1
				// then Task1 < Task3 < Task2
				// then remove Task3, that means Task1 < Task2

				double ratio = 6.0;
				if (current_time < 10.0) {
	

					moveTask->setTransition(false); 
					mobileTask->setTransition(false); 

					beta1 = h_factor(vrep_time, 5.0 , 0);
					beta2 = 1.0 - beta1;
					mobileTask->setBeta(beta2); 
					moveTask->setBeta(beta1); 

					HQP_data1 = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					HQP_data2 = invdyn2_->computeProblemData(vrep_time, q_current, qdot_current);

					sol_1 = solver->solve(HQP_data1);
					sol_2 = solver_2->solve(HQP_data2);

					moveTask->setTransition(true);
					mobileTask->setTransition(true); 
					mobileTask->setOldSolution(sol_2.x.head(dof + 2)); 
					moveTask->setOldSolution(sol_1.x.head(dof + 2)); 
					
					HQP_data3 = invdyn_total_->computeProblemData(vrep_time, q_current, qdot_current);
					sol_total_ = solver_3->solve(HQP_data3);

					tau = invdyn_total_->getActuatorForces(sol_total_);
					dv = invdyn_total_->getAccelerations(sol_total_);
		

				}
				// // transition # 2 Task 2< Task 3< Task 1 -> Task 1 < Task 3 < Task 2
				else if (current_time >= 10.0 &&  current_time < 20.0) {
					if (current_time == 10.0) {
						cout << "Transition #2: The highest task is mobile task" << "\n" << endl;
						cout << "T_init" << robot_->getTransformation(7).translation().transpose() << endl;

						invdyn_->removeTask("mobile_task");
						invdyn_->removeTask("end_effector_task");
						invdyn_->removeTask("joint_control_task2");
						invdyn_total_->removeTask("mobile_task");
						invdyn_total_->removeTask("end_effector_task");
						invdyn_total_->removeTask("joint_control_task2");

						invdyn_->addOperationalTask(*moveTask, w_move, 1.0, 0.0);
						invdyn_->addMotionTask(*mobileTask, w_mobile, 0, 0.0);
						invdyn_->addJointPostureTask(*jointTask2, 0.0, 2.0, 0.0); //weight, level, duration

						invdyn_total_->addOperationalTask(*moveTask, w_move, 1.0, 0.0);
						invdyn_total_->addMotionTask(*mobileTask, w_mobile, 2, 0.0);
						invdyn_total_->addJointPostureTask(*jointTask2, 0.0, 0.0, 0.0); //weight, level, duration
					}
					mobileTask->setTransition(false); 
					jointTask2->setTransition(false); 

					beta1 = h_factor(vrep_time, 15.0 , 10.0);
					beta2 = 1.0 - beta1;
					mobileTask->setBeta(beta1); 
					jointTask2->setBeta(beta2); 

					HQP_data1 = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					HQP_data2 = invdyn2_->computeProblemData(vrep_time, q_current, qdot_current);
					sol_1 = solver->solve(HQP_data1);
					sol_2 = solver_2->solve(HQP_data2);

					mobileTask->setTransition(true); 
					jointTask2->setTransition(true); 

					mobileTask->setOldSolution(sol_2.x.head(dof + 2));
					jointTask2->setOldSolution(sol_1.x.head(dof + 2)); 


					HQP_data3 = invdyn_total_->computeProblemData(vrep_time, q_current, qdot_current);
					sol_total_ = solver_3->solve(HQP_data3);

					tau = invdyn_total_->getActuatorForces(sol_total_);
					dv = invdyn_total_->getAccelerations(sol_total_);
				}
				 // transition # 3 Task 1 < Task 3 < Task 2 -> Task 1 < Task 2 
				 else if ( current_time >= 20.0 ) {
				 	if ( current_time == 20.0) {
				 		cout << "Transition #3: The highest task is mobile task" << "\n" << endl;

				 		invdyn2_->removeTask("mobile_task");
				 		invdyn2_->removeTask("end_effector_task");
				 		invdyn2_->removeTask("joint_control_task2");
				 		invdyn_total_->removeTask("mobile_task");
				 		invdyn_total_->removeTask("end_effector_task");
				 		invdyn_total_->removeTask("joint_control_task2");

						invdyn2_->addOperationalTask(*moveTask, w_move, 1.0, 0.0);
						invdyn2_->addMotionTask(*mobileTask, w_mobile, 0, 0.0);
						invdyn2_->addJointPostureTask(*jointTask2, 0.0, 2.0, 0.0); //weight, level, duration

						invdyn_total_->addMotionTask(*mobileTask, w_mobile, 0, 0.0);
						invdyn_total_->addJointPostureTask(*jointTask2, 0.0, 1.0, 0.0); //weight, level, duration
				 	}
			
					beta1 = h_factor(vrep_time, 25.0 , 20.0);
					beta2 = 1.0 - beta1;

					moveTask->setBeta(beta2); 

					HQP_data1 = invdyn_->computeProblemData(vrep_time, q_current, qdot_current);
					HQP_data2 = invdyn2_->computeProblemData(vrep_time, q_current, qdot_current);

					sol_1 = solver->solve(HQP_data1);
					sol_2 = solver_2->solve(HQP_data2);

					jointTask2->setTransition(true); 
					mobileTask->setTransition(true); 

					jointTask2->setBeta(beta1); 
					mobileTask->setBeta(beta1); 

					mobileTask->setOldSolution(sol_2.x.head(dof + 2)); 
					jointTask2->setOldSolution(sol_2.x.head(dof + 2)); 

					HQP_data3 = invdyn_total_->computeProblemData(vrep_time, q_current, qdot_current);
					sol_total_ = solver_3->solve(HQP_data3);
				
					if (vb._cntt % 200 == 0) {
						cout << solver::HQPDataToString(HQP_data3, true) << endl;
					}
					tau = invdyn_total_->getActuatorForces(sol_total_);
					dv = invdyn_total_->getAccelerations(sol_total_);

				 }
				 
				 double v;
				 double w;
				 wheel_r = 0.001*dv(1) + wheel_r_prev;
				 wheel_l = 0.001*dv(0) + wheel_l_prev;
				 v = 0.165*(wheel_r + wheel_l) / 2.0;
				 w = 0.165*(wheel_r - wheel_l) / (2.0*0.5);
				 wheel_r_prev = wheel_r;
				 wheel_l_prev = wheel_l;

				 vb.desired_torque_ = tau;// tau;
				vb.desired_base_vel_(0) = dv(0); // dv(0)
				vb.desired_base_vel_(1) = dv(1); // dv(1)
				vb.desired_base_vel_(2) = dv(1); // dv(1)
				vb.desired_base_vel_(3) = dv(0); // dv(0)

				Transform3d Current_;
				Current_ = robot_->getTransformation(7);


				fprintf(joint, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n",vb.H_vel_.linear()(0), v, w, vb.current_q_(0), vb.current_q_(1), vb.current_q_(2), vb.current_q_(3), vb.current_q_(4), vb.current_q_(5), vb.current_q_(6));
				fprintf(task_error_ee, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n", s.pos(0), s.pos(1), s.pos(2), Current_.translation()(0), Current_.translation()(1), Current_.translation()(2));
				fprintf(task_error_j, "%lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t %lf\t \n", sampleJoint2.pos(0), sampleJoint2.pos(1), sampleJoint2.pos(2), sampleJoint2.pos(3), sampleJoint2.pos(4), sampleJoint2.pos(5), sampleJoint2.pos(6), vb.current_q_(0), vb.current_q_(1), vb.current_q_(2), vb.current_q_(3), vb.current_q_(4), vb.current_q_(5), vb.current_q_(6));
				fprintf(task_error_m, "%lf\t %lf\t %lf\t %lf\t  \n", q_current(0), q_current(1), s_mobile.pos(0), s_mobile.pos(1) );

					
				vb.write();
			}
		
			vb.simLoop();
			vb._cntt++;

		}
	}

	return 0;
}