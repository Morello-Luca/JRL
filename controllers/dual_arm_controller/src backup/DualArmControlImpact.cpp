#include "DualArmControl.h"

std::pair<
    std::unique_ptr<mc_solver::BoundedSpeedConstr>,
    std::unique_ptr<mc_solver::BoundedSpeedConstr>
    > DualArmControl::boundSpeed(const Eigen::VectorXd & spd){
    
    Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);
    
auto speedLeftConstr = std::make_unique<mc_solver::BoundedSpeedConstr>(
    robots(), leftRobotIndex_, solver().dt());

auto speedRightConstr = std::make_unique<mc_solver::BoundedSpeedConstr>(
    robots(), rightRobotIndex_, solver().dt());
    
    speedLeftConstr->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
    speedRightConstr->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
    
    return {std::move(speedLeftConstr), std::move(speedRightConstr)};
}




double DualArmControl::computeReflectedMassZ(unsigned int robotIndex,
                                             const std::string & eeName)
{
  const auto & robot = robots().robot(robotIndex);

  // Joint-space inertia matrix
  rbd::ForwardDynamics fd(robot.mb());
  fd.computeH(robot.mb(), robot.mbc());
  const Eigen::MatrixXd M = fd.H();

  // Jacobian
  rbd::Jacobian jac(robot.mb(), eeName);
  const Eigen::MatrixXd J =
      jac.jacobian(robot.mb(), robot.mbc()).bottomRows<3>();

  // M^{-1}
  const Eigen::MatrixXd M_inv =
      M.ldlt().solve(Eigen::MatrixXd::Identity(M.rows(), M.cols()));

  // Cartesian inverse inertia
  const Eigen::Matrix3d LambdaInv =
      J * M_inv * J.transpose();

  // Reflected mass along Z
  return 1.0 / LambdaInv(2, 2);
}