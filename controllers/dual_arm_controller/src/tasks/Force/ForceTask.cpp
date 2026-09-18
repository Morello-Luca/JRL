#include "ForceTask.h"

namespace mc_tasks
{

ForceTask::ForceTask(const mc_rbdyn::Robots & robots,
                     unsigned int robotIndex,
                     const Eigen::Vector3d & targetForce,
                     double kp,
                     double ki)
: robots_(robots),
  robotIndex_(robotIndex),
  targetForce_(targetForce),
  measuredForce_(Eigen::Vector3d::Zero()),
  commandedForce_(Eigen::Vector3d::Zero()),
  integralError_(Eigen::Vector3d::Zero()),
  dimWeight_(Eigen::Vector3d::Ones()),
  kp_(kp),
  ki_(ki)
{
  type_ = "force";
  name_ = "force_" + robots_.robot(robotIndex_).name();
}

void ForceTask::reset()
{
  measuredForce_.setZero();
  commandedForce_.setZero();
  integralError_.setZero();
}

void ForceTask::addToSolver(mc_solver::QPSolver &)
{
  if(inSolver_) { return; }

  inSolver_ = true;
}

void ForceTask::removeFromSolver(mc_solver::QPSolver &)
{
  if(!inSolver_) { return; }

  inSolver_ = false;
}

void ForceTask::update(mc_solver::QPSolver & solver)
{
  /*
   * For this first simple version, we only compute the
   * force-control command.
   *
   * The actual force measurement can be connected here
   * once the frame/sensor used by your robot is specified.
   */
measuredWrench_ = frame_->wrench();

  measuredForce_ = measuredWrench_.force();


  const double dt = solver.dt();

  const Eigen::Vector3d forceError =
      targetForce_ - measuredForce_;

  integralError_ += forceError * dt;

  commandedForce_ =
      kp_ * forceError +
      ki_ * integralError_;
}

Eigen::VectorXd ForceTask::eval() const
{
  return targetForce_ - measuredForce_;
}

Eigen::VectorXd ForceTask::speed() const
{
  return Eigen::Vector3d::Zero();
}

void ForceTask::dimWeight(const Eigen::VectorXd & weight)
{
  if(weight.size() != 3)
  {
    throw std::runtime_error("ForceTask::dimWeight expects 3 values");
  }

  dimWeight_ = weight;
}

Eigen::VectorXd ForceTask::dimWeight() const
{
  return dimWeight_;
}

void ForceTask::selectActiveJoints(
    mc_solver::QPSolver &,
    const std::vector<std::string> &,
    const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  // Not needed for this simple force task.
}

void ForceTask::selectUnactiveJoints(
    mc_solver::QPSolver &,
    const std::vector<std::string> &,
    const std::map<std::string, std::vector<std::array<int, 2>>> &)
{
  // Not needed for this simple force task.
}

void ForceTask::resetJointsSelector(mc_solver::QPSolver &)
{
  // Not needed for this simple force task.
}

} // namespace mc_tasks