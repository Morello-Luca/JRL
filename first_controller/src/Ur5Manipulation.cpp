#include "Ur5Manipulation.h"

#include <mc_tasks/MetaTaskLoader.h>
#include <mc_rtc/logging.h>

Ur5Manipulation::Ur5Manipulation(mc_rbdyn::RobotModulePtr rm,
                                 double dt,
                                 const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  // Constraints
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);

  solver().setContacts({{}});

  // Posture task
  postureTask = std::make_shared<mc_tasks::PostureTask>(solver(),0);
  solver().addTask(postureTask);

  postureTask->stiffness(2.0);
  postureTask->weight(100.0);


  // End-effector task
  eeTask_ = std::make_shared<mc_tasks::EndEffectorTask>("tool0",robots(),0);
  solver().addTask(eeTask_);

  eeTask_->stiffness(5.0);
  eeTask_->weight(100.0)

  mc_rtc::log::success("CONTRUCTOR DONE");
}

bool Ur5Manipulation::run()
{
  if(currentState_ != MotionState::IDLE)
  {
    manageStateMachine();
  }

  return mc_control::MCController::run();
}

void Ur5Manipulation::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);

  postureTask->reset();
  eeTask_->reset();

  // Current real EE pose
  homePose_ = realRobot().bodyPosW("tool0");

  // Desired offset
  Eigen::Vector3d translation(-0.07, 0.05, 0.15);

  // Build waypoint
  waypointPose_ = sva::PTransformd(
                  homePose_.rotation(),
                  homePose_.translation() + translation);

  // Set target
  eeTask_->set_ef_pose(waypointPose_);

  currentState_ = MotionState::WAYPOINT;
  mc_rtc::log::success("RESET DONE");

}

void Ur5Manipulation::manageStateMachine()
{
  double error = eeTask_->eval().norm();
  mc_rtc::log::info("EE error: {}", eeTask_->eval().norm());

  if(error < 0.01)
  {
    switch(currentState_)
    {
      case MotionState::WAYPOINT:

        mc_rtc::log::info(
            "Waypoint reached -> returning home");

        eeTask_->set_ef_pose(homePose_);

        currentState_ = MotionState::GO_HOME;

        break;

      case MotionState::GO_HOME:

        mc_rtc::log::info(
            "Home reached -> IDLE");

        currentState_ = MotionState::IDLE;

        break;

      case MotionState::IDLE:
        break;
    }
  }
}

// Register controller
CONTROLLER_CONSTRUCTOR(
    "Ur5Manipulation",
    Ur5Manipulation)