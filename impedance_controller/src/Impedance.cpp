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
  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);
  if(compoundJointConstraint)
  {
    solver().addConstraintSet(*compoundJointConstraint);
  }
  solver().addTask(postureTask);
  postureTask->stiffness(1.0);
  postureTask->weight(1.0);

  /*if(!robot().hasSurface("tool0"))
  {
    mc_rtc::log::error_and_throw(
        "Robot does not have a tool0 surface");
  }*/

  Eigen::Vector3d posM (10.0,10.0,10.0);
  Eigen::Vector3d rotM ( 1.0, 1.0, 1.0);
  
  Eigen::Vector3d posK (500.0,500.0,500.0);
  Eigen::Vector3d rotK (50.0,  50.0, 50.0);
  
  Eigen::Vector3d posD (100.0,100.0,100.0);
  Eigen::Vector3d rotD ( 10.0, 10.0, 10.0);

  impedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(
              "tool0",
              robots(),
              robots().robotIndex(),
              5.0);

  impedanceTask_->weight(100.0);

  auto & gains = impedanceTask_->gains();

  gains.mass()   = { posM, rotM };
  gains.damper() = { posD, rotD };
  gains.spring() = { posK, rotK };

  gains.wrench() =
  {
      Eigen::Vector3d::Ones(),
      Eigen::Vector3d::Ones()
  };
}

void Impedance::reset(
    const ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  impedanceTask_->reset();
  solver().addTask(impedanceTask_);

  center_ = impedanceTask_ ->targetPose().translation() + Eigen::Vector3d(0.0,radius_,0.0);
  orientation_ = impedanceTask_->targetPose().rotation();

  angle_ = 3.0 * mc_rtc::constants::PI / 2.0;
}

bool Impedance::run()
{
  impedanceTask_->targetPose({ orientation_, circleTrajectory(angle_) });
  angle_ += speed_ * solver().dt();
  return MCController::run();
}


Eigen::Vector3d
Impedance::circleTrajectory(double angle)
{
    return center_+ Eigen::Vector3d(radius_ * std::cos(angle),
                                    radius_ * std::sin(angle),
                                    0.0);
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