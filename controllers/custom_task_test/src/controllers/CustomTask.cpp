#include "CustomTask.h"

namespace mc_control
{

CustomTask::CustomTask(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::MCController(
    rm, dt, config,
    mc_control::ControllerParameters{}.backend(mc_control::MCController::Backend::TVM))
{

// ---------------------------------------------------------------------------
// Logging
// ---------------------------------------------------------------------------

logger().addLogEntry("TEST_DESIRED",
                     [this]() { return desiredWrench; });

  // ---------------------------------------------------------------------------
  // Solver constraints
  // ---------------------------------------------------------------------------

  solver().addConstraintSet(kinematicsConstraint);
  solver().addConstraintSet(selfCollisionConstraint);

  if(compoundJointConstraint)
  {
    solver().addConstraintSet(*compoundJointConstraint);
  }



  // in CustomTask ctor, before building forcePITask_
if(solver().backend() != mc_solver::QPSolver::Backend::TVM)
{
  mc_rtc::log::error_and_throw("CustomTask requires the TVM backend, got backend {}", static_cast<int>(solver().backend()));
}
  // ---------------------------------------------------------------------------
  // Posture task
  // ---------------------------------------------------------------------------

  solver().addTask(postureTask);
  postureTask->stiffness(1.0);
  postureTask->weight(1.0);
  mc_rtc::log::success("Known-good PositionTask added");
  
  // ---------------------------------------------------------------------------
  // Impedance gains
  // ---------------------------------------------------------------------------

  const Eigen::Vector3d linearMass(1.0, 1.0, 1.0);
  const Eigen::Vector3d angularMass(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearStiffness(9.0, 9.0, 9.0);
  const Eigen::Vector3d angularStiffness(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearDamping(6, 6, 6);
  const Eigen::Vector3d angularDamping(1.0, 1.0, 1.0);

  const Eigen::Vector3d linearForce(0.0, 0.0, 0.0);
  const Eigen::Vector3d angularForce(0.0, 0.0, 0.0);
 
  impedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>("link7",robots(),robots().robotIndex(),5.0);
  impedanceTask_->weight(1.0);

  auto & gains = impedanceTask_->gains();

  gains.mass() = {linearMass,angularMass};
  gains.spring() = {linearStiffness,angularStiffness};
  gains.damper() = {linearDamping,angularDamping};
  gains.wrench() = {linearForce,angularForce};
  


  // ---------------------------------------------------------------------------
  // Force PI task
  // ---------------------------------------------------------------------------

  auto & robot = robots().robot(robots().robotIndex());

forcePITask_ =
    std::make_shared<ForcePIAccelerationTask>(
        robot.frame("link7"),
        solver().dt());



logger().addLogEntry("TEST_MEASURED",
                     [this]() { return forcePITask_->measuredWrench(); });


  //mc_rtc::log::success("forcePITask_ ptr = {}",static_cast<const void *>(forcePITask_.get()));
  //mc_rtc::log::success("forcePITask_ use_count = {}",forcePITask_.use_count());
  
  forcePITask_->kp( Eigen::Vector3d(0.15, 0.15, 0.15));           // Proportional gain
  forcePITask_->ki( Eigen::Vector3d(0.025, 0.025, 0.025));              // Integral gain
  forcePITask_->mass( Eigen::Vector3d(1.0, 1.0, 1.0));            // Virtual mass used to convert force correction into acceleration
  forcePITask_->maxAcceleration( Eigen::Vector3d(2.0, 2.0, 2.0)); // Acceleration limit 
  forcePITask_->dimWeight( Eigen::Vector3d(0.5, 0.5, 1.5));       // Dimensional weight
  
  
  mc_rtc::log::success("CustomTask init done ");
  
  // ---------------------------------------------------------------------------
  // Add force task to the QP
  // ---------------------------------------------------------------------------
  mc_rtc::log::success(
    "BEFORE addTask ptr = {}",
    static_cast<const void *>(forcePITask_.get()));
  solver().addTask(forcePITask_);
  mc_rtc::log::success( "CustomTask initialized: Force PI acceleration control");
}



void mc_control::CustomTask::reset(const mc_control::ControllerResetData & reset_data)
{
  MCController::reset(reset_data);
  impedanceTask_->reset();
  //forcePITask_->reset();
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

bool mc_control::CustomTask::run()
{
  const double dt = solver().dt();

  // -------------------------------------------------------------------------
  // Sinusoidal desired force
  // -------------------------------------------------------------------------

  constexpr double frequency = 0.1; // Hz
  constexpr double amplitude = 20.0;
  constexpr double offset = -40.0;
  time_ += solver().dt();

  const double omega = 2.0 * mc_rtc::constants::PI * frequency;

  const double Fz_des =
      offset + amplitude * std::cos(omega * time_);

  desiredWrench.force() = Eigen::Vector3d(
      0.0,
      0.0,
      Fz_des
  );

  forcePITask_->targetWrench(desiredWrench);


  //mc_rtc::log::info("Force PI eval = {}",MC_FMT_STREAMED(forcePITask_->eval()));
  //mc_rtc::log::info("Force PI speed = {}",MC_FMT_STREAMED(forcePITask_->speed()));
  //mc_rtc::log::info("Force PI dimWeight = {}",MC_FMT_STREAMED(forcePITask_->dimWeight()));


  const Eigen::Vector3d targetPosition = circleTrajectory(angle_);
  impedanceTask_->targetPose({ orientation_, targetPosition });

  angle_ += speed_ * solver().dt(); // speed_ is the angular speed [rad/s].

  return MCController::run();
}

Eigen::Vector3d CustomTask::circleTrajectory(double angle) const
{
  const Eigen::Vector3d radialOffset( radius_ * std::cos(angle), radius_ * std::sin(angle), 0.0);
  return center_ + radialOffset;
}





} // namespace mc_control

CONTROLLER_CONSTRUCTOR("CustomTask", mc_control::CustomTask)
