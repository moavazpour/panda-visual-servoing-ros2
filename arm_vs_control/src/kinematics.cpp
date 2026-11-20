#include "arm_vs_control/kinematics.hpp"

#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace arm_vs_control
{

bool Kinematics::init(
  const std::string & urdf, const std::string & base_link, const std::string & tip_link)
{
  // 1. URDF text -> KDL tree (the whole robot)
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree)) {
    return false;
  }

  // 2. Tree -> chain (only the links between base and tip)
  if (!tree.getChain(base_link, tip_link, chain_)) {
    return false;
  }

  // 3. Remember the names of the moving joints, in chain order
  joint_names_.clear();
  for (const auto & segment : chain_.segments) {
    if (segment.getJoint().getType() != KDL::Joint::None) {
      joint_names_.push_back(segment.getJoint().getName());
    }
  }

  // 4. Create the solvers
  fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  return true;
}

bool Kinematics::forward(const Eigen::VectorXd & q, Eigen::Isometry3d & pose) const
{
  KDL::JntArray q_kdl(chain_.getNrOfJoints());
  q_kdl.data = q;

  KDL::Frame frame;
  if (fk_solver_->JntToCart(q_kdl, frame) < 0) {
    return false;
  }

  // Convert KDL::Frame -> Eigen::Isometry3d
  pose.setIdentity();
  pose.translation() << frame.p.x(), frame.p.y(), frame.p.z();
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      pose.linear()(r, c) = frame.M(r, c);
    }
  }
  return true;
}

bool Kinematics::jacobian(const Eigen::VectorXd & q, Eigen::MatrixXd & J) const
{
  KDL::JntArray q_kdl(chain_.getNrOfJoints());
  q_kdl.data = q;

  KDL::Jacobian jac(chain_.getNrOfJoints());
  if (jac_solver_->JntToJac(q_kdl, jac) < 0) {
    return false;
  }

  J = jac.data;
  return true;
}

}  // namespace arm_vs_control