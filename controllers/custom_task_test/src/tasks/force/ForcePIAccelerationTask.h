#pragma once

#include <mc_tasks/MetaTask.h>
#include <mc_solver/TVMQPSolver.h>
#include <mc_tvm/RobotFrame.h>

#include <tvm/function/BasicLinearFunction.h>
#include <tvm/task_dynamics/None.h>

#include <mc_filter/LowPass.h>

#include "../../filters/hampelFilter.h"
#include "../../filters/medianFilter.h"
#include "../../filters/butterworth.h"

#include <Eigen/Core>

#include <array>
#include <map>
#include <memory>
#include <string>
#include <vector>

class ForcePIAccelerationTask : public mc_tasks::MetaTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ForcePIAccelerationTask(
    const mc_rbdyn::RobotFrame & frame,
    double dt);

  void reset() override;

  Eigen::VectorXd eval() const override;
  Eigen::VectorXd speed() const override;

  const sva::ForceVecd & targetWrench() const noexcept;
  void targetWrench(const sva::ForceVecd & wrench);

  const Eigen::Vector3d & kp() const noexcept;
  void kp(const Eigen::Vector3d & kp);

  const Eigen::Vector3d & ki() const noexcept;
  void ki(const Eigen::Vector3d & ki);

  const Eigen::Vector3d & mass() const noexcept;
  void mass(const Eigen::Vector3d & mass);

  const Eigen::Vector3d & maxAcceleration() const noexcept;
  void maxAcceleration(const Eigen::Vector3d & aMax);

  void dimWeight(const Eigen::VectorXd & dimW) override;
  Eigen::VectorXd dimWeight() const override;

  void selectActiveJoints( mc_solver::QPSolver & solver, const std::vector<std::string> & joints, const std::map<std::string, std::vector<std::array<int, 2>>> & dof) override;

  void selectUnactiveJoints( mc_solver::QPSolver & solver, const std::vector<std::string> & joints, const std::map<std::string, std::vector<std::array<int, 2>>> & dof) override;

  void resetJointsSelector(mc_solver::QPSolver & solver) override;

  const sva::ForceVecd & measuredWrench() const noexcept;
protected:
  void update(mc_solver::QPSolver & solver) override;
  void addToSolver(mc_solver::QPSolver & solver) override;
  void removeFromSolver(mc_solver::QPSolver & solver) override;
  void addToLogger(mc_rtc::Logger & logger) override;

private:
  // Robot frame associated with the force sensor.
  const mc_rbdyn::RobotFrame & frame_;

  std::shared_ptr<tvm::function::BasicLinearFunction> accelerationFunction_;
  
  tvm::TaskWithRequirementsPtr task_;

  // Force reference and measurement.
  sva::ForceVecd targetWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd measuredWrench_ = sva::ForceVecd::Zero();

  // PI integrator.
  Eigen::Vector3d forceIntegral_ = Eigen::Vector3d::Zero();

  
  // PI gains.
  Eigen::Vector3d Kp_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d Ki_ = Eigen::Vector3d::Zero();

  // Virtual mass used to convert PI force correction into acceleration.
  Eigen::Vector3d mass_ = Eigen::Vector3d::Ones();

  // Cartesian acceleration saturation.
  Eigen::Vector3d maxAcceleration_ = Eigen::Vector3d::Constant(1.0);

  // Task dimensional weight.
  Eigen::VectorXd dimWeight_ = Eigen::VectorXd::Ones(3);



  // filering #include <mc_filter/LowPass.h>
  double cutoffPeriod_ = 0.04; // [s]
  sva::ForceVecd lowPass_filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd hampel_filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd median_filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  sva::ForceVecd butter_filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  
  mc_filter::LowPass<sva::ForceVecd> lowPass_;


// --------------------------------------------------------------------------
// Force-control diagnostics
// --------------------------------------------------------------------------
Eigen::Vector3d targetForceW_{Eigen::Vector3d::Zero()};
Eigen::Vector3d measuredForceW_{Eigen::Vector3d::Zero()};
Eigen::Vector3d forceError_{Eigen::Vector3d::Zero()};

Eigen::Vector3d forceP_{Eigen::Vector3d::Zero()};
Eigen::Vector3d forceI_{Eigen::Vector3d::Zero()};
Eigen::Vector3d virtualForce_{Eigen::Vector3d::Zero()};

Eigen::Vector3d accelerationCommandUnsaturated_{Eigen::Vector3d::Zero()};
Eigen::Vector3d accelerationCommand_{Eigen::Vector3d::Zero()};

Eigen::Vector3d normalAcceleration_{Eigen::Vector3d::Zero()};
Eigen::MatrixXd linearJacobian_;

Eigen::Vector3d accelerationSaturationActive_{Eigen::Vector3d::Zero()};
Eigen::Vector3d antiWindupApplied_{Eigen::Vector3d::Zero()};

const double unsaturated = 0.0;
const Eigen::Vector3d b = Eigen::Vector3d::Zero();
Eigen::Vector3d affineBias_{Eigen::Vector3d::Zero()};
const Eigen::MatrixXd J;

double lastDt_{0.0};

HampelFilter hampel_{40, 8.0, 0.1};
MedianFilter<10> medianFilter_;

ButterworthLowPass2 butterFilter_{1000.0, 20.0};

};