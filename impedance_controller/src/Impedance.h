#pragma once

#include <mc_control/mc_controller.h>

#include <mc_tasks/ImpedanceTask.h>

#include <Eigen/Core>

namespace mc_control
{

struct Impedance : public MCController
{
  Impedance(std::shared_ptr<mc_rbdyn::RobotModule> robot_module,
            double dt,
            Backend backend);

  bool run() override;

  void reset(
      const ControllerResetData & reset_data) override;

  Eigen::Vector3d
  circleTrajectory(double angle);

protected:

  std::shared_ptr<
      mc_tasks::force::ImpedanceTask>
      impedanceTask_;

  double angle_ = 0.0;

  double radius_ = 0.2;

  double speed_ = 0.5;

  Eigen::Vector3d center_ =
      Eigen::Vector3d(
          0.3,
          0.5,
          1.0);

  Eigen::Matrix3d orientation_ =
      Eigen::Matrix3d::Identity();
};

} // namespace mc_control