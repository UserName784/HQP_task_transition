#ifndef __HQP__TASK__JOINT_BOUND__
#define __HQP__TASK__JOINT_BOUND__

#include "robot_model.h"
#include "task-base.h"
#include "task-motion.h"
#include "trajectory-base.h"
#include "constraint-inequality.h"

namespace HQP
{
  namespace tasks
  {
	  class TaskJointLimit : public TaskBase
	  {
	  public:
		  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	      typedef constraint::ConstraintInequality ConstraintInequality;
		  typedef robot::RobotModel RobotModel;
		  typedef constraint::ConstraintBase ConstraintBase;

		  TaskJointLimit(const std::string & name, RobotModel & robot);

		  const ConstraintBase & compute(const double t, Cref_vectorXd q, Cref_vectorXd v);
		  const ConstraintBase & getConstraint() const;

		  int dim() const;

		  void setJointLimit(Cref_vectorXd q_low, Cref_vectorXd q_high);
	
		  const VectorXd & mask() const;
		 // void mask(const VectorXd & mask);

		  const VectorXd & Kp();
		  const VectorXd & Kd();
		  void Kp(Cref_vectorXd Kp);
		  void Kd(Cref_vectorXd Kp);
		  void setTransitionState(const bool & transition) { m_transition = transition; }
		  void setPreviousSol(const VectorXd & old_sol) {m_old_sol = old_sol;}
		  double getactivation(const double & lb, const double & ub, const double & alpha, const double & q);

	  protected:
		  VectorXd m_Kp;
		  VectorXd m_Kd;
		  VectorXd m_p_error, m_v_error;
		  VectorXd m_p, m_v;
		  VectorXd m_a_des;
		  VectorXd m_mask;
		  VectorXi m_activeAxes;
		  VectorXd m_old_sol;

		  double m_buffer;
		  VectorXd m_q_lbound, m_q_ubound;
		  ConstraintInequality m_constraint;
		  double m_velocity_limit_;
		  
	  };
  }
}

#endif // __HQP__TASK__JOINT_BOUND__
