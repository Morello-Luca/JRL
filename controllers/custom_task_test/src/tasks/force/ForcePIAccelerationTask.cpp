#include "ForcePIAccelerationTask.h"

#include <mc_rtc/logging.h>
#include <mc_tvm/Robot.h>
#include <algorithm>
#include <cmath>

ForcePIAccelerationTask::ForcePIAccelerationTask(
    const mc_rbdyn::RobotFrame & frame, double dt)
: frame_(frame),
  lowPass_(dt, cutoffPeriod_)
{
  type_ = "force_pi_acceleration";
  name_ = "force_pi_acceleration_" + frame.robot().name() + "_" + frame.name();

  /*
   * The QP variable for a fixed-base robot is the robot's joint acceleration variable alphaD.
   *
   * build a linear function >> f = A * alphaD + b
   *
   * with:    A = Jv
   *          b = Jdot*qdot - a_cmd
   */
    mc_rtc::log::success(
    "[{}] LowPass initialized with dt = {} s, cutoffPeriod = {} s",
    name_,
    dt,
    cutoffPeriod_);
  
  auto & tvmRobot = frame_.robot().tvmRobot();
  auto alphaD = tvmRobot.alphaD();

  /*
   * Check the Jacobian dimension once here.
   * mc_tvm::RobotFrame::jacobian() is the 6D frame Jacobian.
   * We only control translation, hence the first 3 rows. check because it might be the last 3 
  */

  linearJacobian_ = frame_.tvm_frame().jacobian().bottomRows(3);

  if(linearJacobian_.cols() != alphaD->size()){
    mc_rtc::log::error_and_throw("[{}] Jacobian/alphaD dimension mismatch: J is {}x{}, alphaD has size {}",name_,linearJacobian_.rows(),linearJacobian_.cols(),alphaD->size());
  }

  affineBias_ = frame_.tvm_frame().normalAcceleration().linear();


  accelerationFunction_ =
    std::make_shared<tvm::function::BasicLinearFunction>(
        3,
        alphaD);

accelerationFunction_->A(linearJacobian_);
accelerationFunction_->b(affineBias_);
mc_rtc::log::success(
    "[{}] BasicLinearFunction constructed",
    name_);

mc_rtc::log::success(
    "[{}] BEFORE rSize()",
    name_);

const int n = accelerationFunction_->rSize();

mc_rtc::log::success(
    "[{}] rSize() returned {}",
    name_,
    n);

}

void ForcePIAccelerationTask::update(mc_solver::QPSolver & solver){
  const double dt = solver.dt();


  
  if(dt <= 0.0){
    return;
  }
  
  // --------------------------------------------------------------------------
  // 1. Read force sensor
  // --------------------------------------------------------------------------
    measuredWrench_ = frame_.wrench();
    lowPass_.update(measuredWrench_);
    lowPass_filteredMeasuredWrench_ = lowPass_.eval();
    hampel_filteredMeasuredWrench_ = hampel_.update(measuredWrench_);
    median_filteredMeasuredWrench_ = sva::ForceVecd(medianFilter_.update(measuredWrench_.vector()));
    butter_filteredMeasuredWrench_ = sva::ForceVecd(butterFilter_.update(measuredWrench_.vector()));
  

  const Eigen::Matrix3d R_0_f = frame_.position().rotation();
  measuredForceW_ = R_0_f * butter_filteredMeasuredWrench_.force();
  targetForceW_ = R_0_f * targetWrench_.force();

  // --------------------------------------------------------------------------
  // 2. Force PI
  // --------------------------------------------------------------------------
  forceError_ = measuredForceW_- targetForceW_;
  forceIntegral_ += forceError_ * dt;
  virtualForce_ = Kp_.cwiseProduct(forceError_) + Ki_.cwiseProduct(forceIntegral_);
  accelerationCommand_ = mass_.cwiseInverse().cwiseProduct(virtualForce_);

  // --------------------------------------------------------------------------
  //  3. Acceleration saturation + anti-windup
  // --------------------------------------------------------------------------
  for(int i = 0; i < 3; ++i){
    const double unsaturated = accelerationCommand_[i];
    const double saturated = std::clamp( unsaturated, -maxAcceleration_[i], maxAcceleration_[i]);
    // If the command saturated and the integral action is pushing further into the saturated region, undo the integration performed this cycle.
    const bool pushingIntoSaturation = ((unsaturated > maxAcceleration_[i]) && (forceError_[i] > 0.0)) || ((unsaturated < -maxAcceleration_[i]) && (forceError_[i] < 0.0));
    accelerationCommand_[i] = saturated;
    if(pushingIntoSaturation){
      forceIntegral_[i] -= forceError_[i] * dt;
    }
  }
  
   /* --------------------------------------------------------------------------
   * 4. Update the affine acceleration relation
   *  --------------------------------------------------------------------------
   1. Actual translational Cartesian acceleration  >> a = Jv*qdd + Jdot*qdot
   2. We want                                      >> a = a_PI
   3. Therefore                                    >> Jv*qdd + Jdot*qdot - a_PI = 0.
   4. BasicLinearFunction                          >> f = A*x + b
   5. with:                                           A = Jv
                                                      b = Jdot*qdot - a_PI
   */
  linearJacobian_ = frame_.tvm_frame().jacobian().bottomRows(3);
  normalAcceleration_ = frame_.tvm_frame().normalAcceleration().linear();
  affineBias_ = normalAcceleration_ - accelerationCommand_;
  accelerationFunction_->A(linearJacobian_);
  accelerationFunction_->b(affineBias_);
}



void ForcePIAccelerationTask::addToSolver(mc_solver::QPSolver & solver)
{
  if(task_)
  {
    return;
  }

  mc_rtc::log::success("[{}] addToSolver ENTER", name_);

  tvm::requirements::SolvingRequirements reqs{
      tvm::requirements::PriorityLevel(1),
      tvm::requirements::Weight(1.0),
      tvm::requirements::AnisotropicWeight(dimWeight_)};

  mc_rtc::log::success("[{}] requirements OK", name_);

  /*
   * accelerationFunction_ already represents:
   *
   *   Jv * alphaD + (Jdot*qdot - a_PI) = 0
   *
   * so this is directly an equality task on alphaD.
   *
   * No explicit Task or None dynamics construction is required.
   */
  task_ = tvm_solver(solver).problem().add(
      accelerationFunction_ == Eigen::Vector3d::Zero(),
      reqs);

  mc_rtc::log::success(
      "[{}] problem.add returned, task ptr = {}",
      name_,
      static_cast<const void *>(task_.get()));
}

void ForcePIAccelerationTask::removeFromSolver(mc_solver::QPSolver & solver){
  if(!task_)
  {
    return;
  }

  tvm_solver(solver).problem().remove(*task_);
  task_.reset();
}

void ForcePIAccelerationTask::reset(){
  forceIntegral_.setZero();
  accelerationCommand_.setZero();
  measuredWrench_ = sva::ForceVecd::Zero();
  
  butter_filteredMeasuredWrench_ = sva::ForceVecd::Zero();
  lowPass_.reset(sva::ForceVecd::Zero());

  if(accelerationFunction_){
    linearJacobian_ = frame_.tvm_frame().jacobian().bottomRows(3);
    affineBias_ = frame_.tvm_frame().normalAcceleration().linear();
    accelerationFunction_->A(linearJacobian_);
    accelerationFunction_->b(affineBias_);
  }
}

void ForcePIAccelerationTask::selectActiveJoints(mc_solver::QPSolver &, const std::vector<std::string> &, const std::map<std::string, std::vector<std::array<int, 2>>> &){  
} // Empty

void ForcePIAccelerationTask::selectUnactiveJoints(mc_solver::QPSolver &,const std::vector<std::string> &,const std::map<std::string,std::vector<std::array<int, 2>>> &){
} // Empty

void ForcePIAccelerationTask::resetJointsSelector(mc_solver::QPSolver &){
} // Empty

void ForcePIAccelerationTask::dimWeight( const Eigen::VectorXd & dimW){
  if(dimW.size() != 3){
    mc_rtc::log::error_and_throw("[{}] dimWeight must have size 3, got {}",name_,dimW.size());
  }

  dimWeight_ = dimW;
  if(task_){
    task_->requirements.anisotropicWeight() = dimWeight_;
  }
}

Eigen::VectorXd ForcePIAccelerationTask::dimWeight() const{
  return dimWeight_;
}

Eigen::VectorXd ForcePIAccelerationTask::eval() const{
  return accelerationCommand_;
}

Eigen::VectorXd ForcePIAccelerationTask::speed() const{
  return accelerationCommand_;
} // For compatibility with MetaTask we expose the current acceleration command.

const sva::ForceVecd & ForcePIAccelerationTask::targetWrench() const noexcept{
  return targetWrench_;
}

void ForcePIAccelerationTask::targetWrench(const sva::ForceVecd & wrench){
  targetWrench_ = wrench;
}

const Eigen::Vector3d &ForcePIAccelerationTask::kp() const noexcept{
  return Kp_;
}

void ForcePIAccelerationTask::kp(const Eigen::Vector3d & kp){
  Kp_ = kp;
}

const Eigen::Vector3d &ForcePIAccelerationTask::ki() const noexcept{
  return Ki_;
}

void ForcePIAccelerationTask::ki(const Eigen::Vector3d & ki){
  Ki_ = ki;
}

const Eigen::Vector3d &ForcePIAccelerationTask::mass() const noexcept{
  return mass_;
}

void ForcePIAccelerationTask::mass(const Eigen::Vector3d & mass){
  if((mass.array() <= 0.0).any())
  {
    mc_rtc::log::error_and_throw("[{}] Mass must be strictly positive",name_);
  }
  mass_ = mass;
}

const Eigen::Vector3d &ForcePIAccelerationTask::maxAcceleration() const noexcept{
  return maxAcceleration_;
}

void ForcePIAccelerationTask::maxAcceleration(const Eigen::Vector3d & aMax){
  if((aMax.array() < 0.0).any())
  {
    mc_rtc::log::error_and_throw("[{}] Maximum acceleration must be non-negative",name_);
  }
  maxAcceleration_ = aMax;
}


const sva::ForceVecd & ForcePIAccelerationTask::measuredWrench() const noexcept
{
  return butter_filteredMeasuredWrench_;
}

void ForcePIAccelerationTask::addToLogger(mc_rtc::Logger & logger)
{

  // =========================================================================
  // WRENCH MEASUREMENT
  // =========================================================================

  MC_RTC_LOG_HELPER(name_ + "_targetWrench", targetWrench_);
  MC_RTC_LOG_HELPER(name_ + "_measuredWrench", measuredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_FILTERS_lowpass", lowPass_filteredMeasuredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_FILTERS_hampel", hampel_filteredMeasuredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_FILTERS_median", median_filteredMeasuredWrench_);
  MC_RTC_LOG_HELPER(name_ + "_FILTERS_butter", butter_filteredMeasuredWrench_);

  

  // =========================================================================
  // FORCE CONTROL - WORLD FRAME
  // =========================================================================

  MC_RTC_LOG_HELPER(name_ + "_targetForceW", targetForceW_);
  MC_RTC_LOG_HELPER(name_ + "_measuredForceW", measuredForceW_);
  MC_RTC_LOG_HELPER(name_ + "_forceError", forceError_);


  // =========================================================================
  // ACCELERATION COMMAND
  // =========================================================================

  MC_RTC_LOG_HELPER(name_ + "_accelerationCommandUnsaturated",accelerationCommandUnsaturated_);
  MC_RTC_LOG_HELPER(name_ + "_accelerationCommand",accelerationCommand_);

}



