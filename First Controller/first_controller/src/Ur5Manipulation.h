#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/PostureTask.h>
#include <mc_tasks/EndEffectorTask.h>

#include "api.h"

struct Ur5Manipulation_DLLAPI Ur5Manipulation : public mc_control::MCController
{
  Ur5Manipulation(mc_rbdyn::RobotModulePtr rm,
                  double dt,
                  const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

private:

  // Joint posture task
  std::shared_ptr<mc_tasks::PostureTask> postureTask;

  // Cartesian end-effector task
  std::shared_ptr<mc_tasks::EndEffectorTask> eeTask_;

  enum class MotionState
  {
    WAYPOINT,
    GO_HOME,
    IDLE
  };

  MotionState currentState_ = MotionState::WAYPOINT;

  // Cartesian poses
  sva::PTransformd waypointPose_;
  sva::PTransformd homePose_;

  void manageStateMachine();
};