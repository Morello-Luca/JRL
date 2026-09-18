#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/ImpedanceTask.h>

#include "../tasks/force/ForcePIAccelerationTask.h"
#include <Eigen/Core>




#include "../api.h"



namespace mc_control
{
struct CustomTask_DLLAPI CustomTask : public mc_control::MCController
{
  CustomTask(mc_rbdyn::RobotModulePtr rm,
             double dt,
             const mc_rtc::Configuration & config);
  bool run() override;

  void reset(
      const ControllerResetData & reset_data) override;

  Eigen::Vector3d
  circleTrajectory(double angle) const;

protected:

  std::shared_ptr<
      mc_tasks::force::ImpedanceTask>
      impedanceTask_;

  double angle_ = 0.0;
  double radius_ = 0.2;
  double speed_ = 0.1;
  double time_ = 0.0;

  Eigen::Vector3d center_ = Eigen::Vector3d::Zero();
    sva::ForceVecd desiredWrench = sva::ForceVecd::Zero();
  Eigen::Matrix3d orientation_ = Eigen::Matrix3d::Identity();

  std::shared_ptr<ForcePIAccelerationTask> forcePITask_;

  
};
} // namespace mc_control


