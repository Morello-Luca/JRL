#include "BimanualControl.h"

BimanualControl::BimanualControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(rm, dt)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  solver().setContacts({{}});

  mc_rtc::log::success("BimanualControl init done ");
}

bool BimanualControl::run()
{
  return mc_control::MCController::run();
}

void BimanualControl::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(reset_data);
}

CONTROLLER_CONSTRUCTOR("BimanualControl", BimanualControl)
