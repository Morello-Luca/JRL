#include "Test.h"

#include <mc_rtc/logging.h>

Test::Test(mc_rbdyn::RobotModulePtr rm,
           double dt,
           const mc_rtc::Configuration & config)

: mc_control::MCController(rm, dt)
{
  /*
   * Constraints
   */

  solver().addConstraintSet(contactConstraint);

  solver().addConstraintSet(kinematicsConstraint);

  /*
   * Posture task
   */

  solver().addTask(postureTask);

  /*
   * No contacts
   */

  solver().setContacts({});

  /*
   * Create impedance task
   */

  impedanceTask_ =
      std::make_shared<
          mc_tasks::force::ImpedanceTask>(
              robot().frame("tool0"),
              5.0,
              1000.0);

  /*
   * Current pose
   */

  auto currentPose =
      robot().frame("tool0").position();

  /*
   * IMPORTANT:
   * Keep same orientation.
   * Move only XYZ.
   */

  targetPose_ = currentPose;

  /*
   * Small Cartesian displacement
   */

  targetPose_.translation().x() += 0.05;

  /*
   * Set target pose
   */

  impedanceTask_->targetPose(
      targetPose_);

  /*
   * Zero velocity
   */

  impedanceTask_->targetVel(
      sva::MotionVecd::Zero());

  impedanceTask_->targetAccel(
      sva::MotionVecd::Zero());

  /*
   * IMPORTANT:
   * Gains order is:
   *
   * [ cx cy cz fx fy fz ]
   *
   * angular first
   * linear second
   */

  /*
   * Mass
   */

  impedanceTask_->gains().mass().vec(
      Eigen::Vector6d(
          5.0,
          5.0,
          5.0,
          2.0,
          2.0,
          2.0));

  /*
   * Spring stiffness
   */

  impedanceTask_->gains().spring().vec(
      Eigen::Vector6d(
          300.0,
          300.0,
          300.0,
          100.0,
          100.0,
          100.0));

  /*
   * Critical damping:
   *
   * D = 2 * sqrt(K * M)
   */

  Eigen::Vector6d K =
      impedanceTask_->gains()
          .spring()
          .vec();

  Eigen::Vector6d M =
      impedanceTask_->gains()
          .mass()
          .vec();

  Eigen::Vector6d D;

  for(int i = 0; i < 6; ++i)
  {
    D[i] =
        2.0 * std::sqrt(K[i] * M[i]);
  }

  impedanceTask_->gains().damper().vec(D);

  /*
   * No force tracking
   */

  impedanceTask_->gains().wrench().vec(
      Eigen::Vector6d::Zero());

  /*
   * Compliance limits
   */

  impedanceTask_->hold(false);

  impedanceTask_->cutoffPeriod(0.02);

  /*
   * Add task
   */

  solver().addTask(
      impedanceTask_);

  /*
   * Logger
   */

  logger().addLogEntry(
      "target_pose_x",
      [this]()
      {
        return targetPose_
            .translation()
            .x();
      });

  logger().addLogEntry(
      "current_pose_x",
      [this]()
      {
        return robot()
            .frame("tool0")
            .position()
            .translation()
            .x();
      });

  logger().addLogEntry(
      "position_error_x",
      [this]()
      {
        return targetPose_
                   .translation()
                   .x()
               -
               robot()
                   .frame("tool0")
                   .position()
                   .translation()
                   .x();
      });

  logger().addLogEntry(
      "compliance_pose",
      [this]()
      {
        return impedanceTask_
            ->compliancePose();
      });

  logger().addLogEntry(
      "delta_compliance",
      [this]()
      {
        return impedanceTask_
            ->deltaCompliancePose()
            .translation();
      });

  mc_rtc::log::success(
      "Impedance controller initialized");
}

bool Test::run()
{
  return mc_control::MCController::run();
}

void Test::reset(
    const mc_control::ControllerResetData & reset_data)
{
  mc_control::MCController::reset(
      reset_data);

  impedanceTask_->reset();

  /*
   * Recompute target from current pose
   */

  auto currentPose =
      robot().frame("tool0").position();

  targetPose_ = currentPose;

  targetPose_.translation().x() += 0.05;

  impedanceTask_->targetPose(
      targetPose_);

  impedanceTask_->targetVel(
      sva::MotionVecd::Zero());

  impedanceTask_->targetAccel(
      sva::MotionVecd::Zero());
}

CONTROLLER_CONSTRUCTOR(
    "Test",
    Test)
