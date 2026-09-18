#include "ForcePIAccelerationTask.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

ForcePIAccelerationTask::ForcePIAccelerationTask(
    const mc_rbdyn::RobotFrame & frame)
: frame_(frame),
  positionFunction_(std::make_shared<mc_tvm::PositionFunction>(frame))
{
  type_ = "force_pi_acceleration";
  name_ = "force_pi_acceleration_" +
          frame.robot().name() + "_" +
          frame.name();
}

void ForcePIAccelerationTask::update(mc_solver::QPSolver & solver)
{
  const double dt = solver.dt();

  if(dt <= 0.0)
  {
    return;
  }

  measuredWrench_ = frame_.wrench();

  // Translational force only.
  //
  // Positive error means:
  // target force > measured force.
  //
  // This is the usual negative-feedback convention.
  const Eigen::Vector3d forceError =
      targetWrench_.force() - measuredWrench_.force();

  forceIntegral_ += forceError * dt;

  const Eigen::Vector3d virtualForce =
      Kp_.cwiseProduct(forceError)
      + Ki_.cwiseProduct(forceIntegral_);

  // M * a = F
  accelerationCommand_ =
      mass_.cwiseInverse().cwiseProduct(virtualForce);

  // Prevent the PI controller from producing unreasonable
  // Cartesian acceleration commands.
  for(int i = 0; i < 3; ++i)
  {
    accelerationCommand_[i] =
        std::clamp(
            accelerationCommand_[i],
            -maxAcceleration_[i],
            maxAcceleration_[i]);
  }

  positionFunction_->refAccel(accelerationCommand_);
}

void ForcePIAccelerationTask::addToSolver(
    mc_solver::QPSolver & solver)
{
  if(task_)
  {
    return;
  }

  tvm::requirements::SolvingRequirements reqs{
      tvm::requirements::PriorityLevel(1),
      tvm::requirements::Weight(1.0)
  };

  /*
   * PositionFunction is a TVM function, not a TVM Task.
   *
   * The TVM task is:
   *
   *      positionFunction == 0
   *
   * with a proportional task dynamics.
   *
   * PositionFunction::refAccel() provides the feed-forward
   * acceleration reference.
   */
  tvm::FunctionPtr errorPtr(
      positionFunction_.get(),
      [](tvm::function::abstract::Function *) {});

Eigen::VectorXd tvmKp(3);
tvmKp.setOnes();

task_ = tvm_solver(solver).problem().add(
    errorPtr == 0.,
    tvm::task_dynamics::P(tvmKp),
    reqs);
}

void ForcePIAccelerationTask::removeFromSolver(
    mc_solver::QPSolver & solver)
{
  if(!task_)
  {
    return;
  }

  tvm_solver(solver).problem().remove(*task_);
  task_.reset();
}

void ForcePIAccelerationTask::reset()
{
  forceIntegral_.setZero();
  accelerationCommand_.setZero();
  measuredWrench_ = sva::ForceVecd::Zero();

  positionFunction_->refAccel(Eigen::Vector3d::Zero());
}

Eigen::VectorXd ForcePIAccelerationTask::eval() const
{
  return accelerationCommand_;
}

Eigen::VectorXd ForcePIAccelerationTask::speed() const
{
  return accelerationCommand_;
}

const sva::ForceVecd &
ForcePIAccelerationTask::targetWrench() const noexcept
{
  return targetWrench_;
}

void ForcePIAccelerationTask::targetWrench(
    const sva::ForceVecd & wrench)
{
  targetWrench_ = wrench;
}

const Eigen::Vector3d &
ForcePIAccelerationTask::kp() const noexcept
{
  return Kp_;
}

void ForcePIAccelerationTask::kp(
    const Eigen::Vector3d & kp)
{
  Kp_ = kp;
}

const Eigen::Vector3d &
ForcePIAccelerationTask::ki() const noexcept
{
  return Ki_;
}

void ForcePIAccelerationTask::ki(
    const Eigen::Vector3d & ki)
{
  Ki_ = ki;
}

const Eigen::Vector3d &
ForcePIAccelerationTask::mass() const noexcept
{
  return mass_;
}

void ForcePIAccelerationTask::mass(
    const Eigen::Vector3d & mass)
{
  if((mass.array() <= 0.0).any())
  {
    mc_rtc::log::error_and_throw(
        "[{}] Mass must be strictly positive", name_);
  }

  mass_ = mass;
}