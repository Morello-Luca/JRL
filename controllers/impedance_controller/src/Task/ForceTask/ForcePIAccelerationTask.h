#pragma once

#include <mc_tasks/MetaTask.h>
#include <mc_tvm/PositionFunction.h>
#include <mc_solver/TVMQPSolver.h>

#include <tvm/task_dynamics/Proportional.h>

#include <Eigen/Core>

class ForcePIAccelerationTask : public mc_tasks::MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ForcePIAccelerationTask(const mc_rbdyn::RobotFrame & frame);

  void reset() override;

  Eigen::VectorXd eval() const override;
  Eigen::VectorXd speed() const override;

  const sva::ForceVecd & targetWrench() const noexcept;
  void targetWrench(const sva::ForceVecd & wrench);

  const Eigen::Vector3d & kp() const noexcept;
  void kp(const Eigen::Vector3d & kp);

  const Eigen::Vector3d & ki() const noexcept;
  void ki(const Eigen::Vector3d & ki);

  const Eigen::Vector3d & mass() const noexcept;
  void mass(const Eigen::Vector3d & mass);


  void dimWeight(const Eigen::VectorXd & dimW) override;
Eigen::VectorXd dimWeight() const override;

void selectActiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & joints,
    const std::map<std::string, std::vector<std::array<int, 2>>> & dof) override;

void selectUnactiveJoints(
    mc_solver::QPSolver & solver,
    const std::vector<std::string> & joints,
    const std::map<std::string, std::vector<std::array<int, 2>>> & dof) override;

void resetJointsSelector(mc_solver::QPSolver & solver) override;

protected:
  void update(mc_solver::QPSolver & solver) override;
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;

private:
  // Do NOT store RobotFrame by value: it is non-copyable.
  const mc_rbdyn::RobotFrame & frame_;

  std::shared_ptr<mc_tvm::PositionFunction> positionFunction_;

  // Null while the task is not in the solver.
  tvm::TaskWithRequirementsPtr task_;

  sva::ForceVecd targetWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd measuredWrench_ = sva::ForceVecd::Zero();

  Eigen::Vector3d forceIntegral_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d accelerationCommand_ = Eigen::Vector3d::Zero();

  Eigen::Vector3d Kp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d Ki_ = Eigen::Vector3d::Zero();

  // Virtual mass.
  Eigen::Vector3d mass_ = Eigen::Vector3d::Ones();

  // Maximum commanded Cartesian acceleration.
  Eigen::Vector3d maxAcceleration_ =
      Eigen::Vector3d::Constant(1.0);

  Eigen::VectorXd dimWeight_ = Eigen::VectorXd::Ones(3);
};