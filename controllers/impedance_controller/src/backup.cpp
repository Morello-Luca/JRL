#include "Impedance.h"
#include <mc_rtc/constants.h>

namespace mc_control
{

Impedance::Impedance(
    std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
    double dt,
    Backend backend)
: MCController(robot_module, dt, backend)
{
  // ---------------------------------------------------------------------------
  // Solver constraints
  // ---------------------------------------------------------------------------

  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);

  if(compoundJointConstraint)
  {
    solver().addConstraintSet(*compoundJointConstraint);
  }

  // ---------------------------------------------------------------------------
  // Posture task
  // ---------------------------------------------------------------------------

  solver().addTask(postureTask);
  postureTask->stiffness(1.0);
  postureTask->weight(1.0);

  // ---------------------------------------------------------------------------
  // Impedance gains
  // ---------------------------------------------------------------------------

  const Eigen::Vector3d linearMass(1.0, 1.0, 1.0);
  const Eigen::Vector3d angularMass(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearStiffness(2.0, 2.0, 2.0);
  const Eigen::Vector3d angularStiffness(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearDamping(1.5, 1.5, 1.5);
  const Eigen::Vector3d angularDamping(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearForce(0.0, 0.0, 0.0);
  const Eigen::Vector3d angularForce(0.0, 0.0, 0.0);


  impedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>("link7",robots(),robots().robotIndex(),5.0);
  impedanceTask_->weight(100.0);

  auto & gains = impedanceTask_->gains();

  gains.mass() = {linearMass,angularMass};
  gains.spring() = {linearStiffness,angularStiffness};
  gains.damper() = {linearDamping,angularDamping};
  gains.wrench() = {linearForce,angularForce};


  // ---------------------------------------------------------------------------
  // Force Task gains
  // ---------------------------------------------------------------------------


auto forceTask =
    std::make_shared<ForcePIAccelerationTask>(
        robot().frame("link7"));

forceTask->targetWrench(
    sva::ForceVecd(
        Eigen::Vector3d::Zero(),      // torque
        Eigen::Vector3d(0.0, 0.0, 10.0) // force
    ));

forceTask->kp(
    Eigen::Vector3d(0.01, 0.01, 0.01));

forceTask->ki(
    Eigen::Vector3d(0.001, 0.001, 0.001));

forceTask->mass(
    Eigen::Vector3d(1.0, 1.0, 1.0));

solver().addTask(forceTask);

}



void Impedance::reset(
    const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  impedanceTask_->reset();
  solver().addTask(impedanceTask_);

  constexpr double z_offset  = 0.0;
  constexpr double y_Offset  = 0.1;
  constexpr double x_Offset  = 0.1;
  

  // Current end-effector target when the controller starts
  const Eigen::Vector3d initialPosition = impedanceTask_->targetPose().translation();

  orientation_ = impedanceTask_->targetPose().rotation();
  center_ = initialPosition + Eigen::Vector3d(x_Offset, radius_+ y_Offset, z_offset);
  angle_ = 3.0 * mc_rtc::constants::PI / 2.0;
}



bool Impedance::run()
{
  const Eigen::Vector3d targetPosition = circleTrajectory(angle_);
  impedanceTask_->targetPose({ orientation_, targetPosition });

  angle_ += speed_ * solver().dt(); // speed_ is the angular speed [rad/s].

  return MCController::run();
}


Eigen::Vector3d Impedance::circleTrajectory(double angle) const
{
  const Eigen::Vector3d radialOffset( radius_ * std::cos(angle), radius_ * std::sin(angle), 0.0);
  return center_ + radialOffset;
}

} // namespace mc_control


MULTI_CONTROLLERS_CONSTRUCTOR(
    "Impedance",
    mc_control::Impedance(
        rm,
        dt,
        mc_control::MCController::Backend::Tasks),

    "Impedance_TVM",
    mc_control::Impedance(
        rm,
        dt,
        mc_control::MCController::Backend::TVM))