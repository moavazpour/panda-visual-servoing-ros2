#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

namespace arm_vs_control
{

class Kinematics
{
public:
  // Build the kinematic chain from the URDF text, between two links.
  bool init(const std::string & urdf, const std::string & base_link, const std::string & tip_link);

  const std::vector<std::string> & joint_names() const {return joint_names_;}
  unsigned int num_joints() const {return chain_.getNrOfJoints();}

  // Forward kinematics: joint angles -> pose of the tip link in the base frame.
  bool forward(const Eigen::VectorXd & q, Eigen::Isometry3d & pose) const;

  // Jacobian: 6 x N matrix that maps joint velocities to tip velocity.
  bool jacobian(const Eigen::VectorXd & q, Eigen::MatrixXd & J) const;

private:
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::vector<std::string> joint_names_;
};

}  // namespace arm_vs_control