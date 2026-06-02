#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/ImpedanceTask.h>

struct Test : public mc_control::MCController
{
public:
  Test(mc_rbdyn::RobotModulePtr rm,
       double dt,
       const mc_rtc::Configuration & config);

  bool run() override;

  void reset(
      const mc_control::ControllerResetData & reset_data) override;

private:
  std::shared_ptr<
      mc_tasks::force::ImpedanceTask>
      impedanceTask_;

private:
  sva::PTransformd targetPose_;
};