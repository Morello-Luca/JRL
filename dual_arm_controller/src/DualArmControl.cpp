#include "DualArmControl.h"
#include <mc_rbdyn/RobotLoader.h>

DualArmControl::DualArmControl(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt, config)
{
  // Configurazione iniziale dei vincoli globali
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);

  postureTask->stiffness(1.0);
  postureTask->weight(0.1);
  solver().setContacts({{}});

  // Caricamento del secondo braccio (xArm7)
  std::vector<std::string> loaderArgs = {"xArm7", "name", "xarm7_2"};
  auto xarm2Module = mc_rbdyn::RobotLoader::get_robot_module(loaderArgs);

  if (!xarm2Module)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Failed to load xarm7_2 module!");
  }

  // Use the correctly matched variable name here
  loadRobot(xarm2Module, "xarm7_2");

  // Posizionamento del secondo braccio nello spazio cartesiano
  robots().robot("xarm7_2").posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0.0, 0.5, 0.0)));
  addCollisions("xarm7", "xarm7_2", {{"*", "*", iDist, sDist, 0}});

  // Stato di sicurezza iniziale (Fai nulla)
  currentState_ = &DualArmControl::stateNoOp;
}

// =========================================================================
// Extract impedance gain setup
// INITIALIZATION OF IMPEDANCE GAINS (M, D, K matrices)
// =========================================================================

void DualArmControl::configureImpedanceGains()
{
  massGains = Eigen::Vector6d::Constant(1.0);
  springGains = Eigen::Vector6d::Constant(200.0);
  damperGains = 2.0 * springGains.cwiseProduct(massGains).cwiseSqrt();
  wrenchGains = Eigen::Vector6d::Constant(1.0);

  forBothImpedanceTasks([&](auto &task)
                        {
    task->gains().mass().vec(massGains);
    task->gains().spring().vec(springGains);
    task->gains().damper().vec(damperGains);
    task->gains().wrench().vec(wrenchGains); });
}

void DualArmControl::reset(const mc_control::ControllerResetData &resetData)
{
  mc_control::MCController::reset(resetData);

  // Cache degli indici dei robot per performance
  leftRobotIndex_ = robots().robotIndex("xarm7");
  rightRobotIndex_ = robots().robotIndex("xarm7_2");

  // ALLOCAZIONE TASK (Zero memory jitter nel loop real-time)
  // --------------------------------------------------------------
  rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), rightRobotIndex_, 1.0, 0.1);
  solver().addTask(rightPostureTask_);

  leftEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), leftRobotIndex_, 1.0);
  rightEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), rightRobotIndex_, 1.0);

  leftImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), leftRobotIndex_, 1.0, 100.0);
  rightImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), rightRobotIndex_, 1.0, 100.0);

  // Gains
  // --------------------------------------------------------------
  configureImpedanceGains();

  // Inizio della FSM (Macchina a Stati)
  transitionTo(&DualArmControl::entryStateIdle, &DualArmControl::stateIdle);
}

// =========================================================================
// GESTORE TRANSIZIONI (Pattern a Doppio Puntatore)
// =========================================================================
void DualArmControl::transitionTo(StateMethod entryMethod, StateMethod runMethod)
{
  if (!entryMethod || !runMethod)
    return;

  stateTimer_ = 0.0; // Reset deterministico del tempo ad ogni cambio stato

  (this->*entryMethod)();    // Esecuzione immediata del setup dello stato
  currentState_ = runMethod; // Switch del puntatore per i cicli successivi
}

// =========================================================================
// STATO 1: IDLE
// =========================================================================
void DualArmControl::entryStateIdle()
{
  mc_rtc::log::info("[FSM] Entering IDLE Phase (Holding for 3.0 seconds)...");
}

void DualArmControl::stateIdle()
{
  stateTimer_ += timeStep;

  if (stateTimer_ >= 3.0)
  {
    mc_rtc::log::success("[FSM] IDLE Phase complete. Switching to INDEPENDENT.");
    transitionTo(&DualArmControl::entryStateIndependent, &DualArmControl::stateIndependent);
  }
}

// =========================================================================
// STATO 2: INDEPENDENT (Raggiungimento Waypoint Singoli con Rotazione)
// =========================================================================
void DualArmControl::entryStateIndependent()
{
  mc_rtc::log::info("[FSM] Entering INDEPENDENT Phase with 90-degree target rotation.");

  solver().addTask(leftEeTask_);
  solver().addTask(rightEeTask_);

  // 1. Cattura gli orientamenti correnti prima della transizione
  Eigen::Matrix3d R_left_start = robots().robot(leftRobotIndex_).bodyPosW(eeName_).rotation();
  Eigen::Matrix3d R_right_start = robots().robot(rightRobotIndex_).bodyPosW(eeName_).rotation();

  // 2. Ruota di 90 gradi attorno all'asse Z relativo
  Eigen::Matrix3d R_rot_90 = sva::RotX(M_PI / 2.0);

  // 3. Calcola le matrici di orientamento target finali
  Eigen::Matrix3d R_left_target = R_rot_90 * R_left_start;
  Eigen::Matrix3d R_right_target = R_rot_90.transpose() * R_right_start;

  // 4. Crea i frame spaziali completi (Posizione + Rotazione)
  sva::PTransformd X_0_leftTarget(R_left_target, Eigen::Vector3d(0.50, 0.13, 0.09));
  sva::PTransformd X_0_rightTarget(R_right_target, Eigen::Vector3d(0.50, 0.38, 0.09));

  // 5. Invia i comandi contemporanei al solutore cinematico
  leftEeTask_->set_ef_pose(X_0_leftTarget);
  rightEeTask_->set_ef_pose(X_0_rightTarget);

  postureTask->stiffness(1.0);
  rightPostureTask_->stiffness(1.0);
}

void DualArmControl::stateIndependent()
{
  // Criteri di convergenza sia cartesiana che di velocità angolare/lineare
  const bool leftConverged = leftEeTask_->eval().norm() < 0.03 && leftEeTask_->speed().norm() < 0.005;
  const bool rightConverged = rightEeTask_->eval().norm() < 0.03 && rightEeTask_->speed().norm() < 0.005;

  if (leftConverged && rightConverged)
  {
    mc_rtc::log::success("[FSM] Independent targets and 90-deg rotation reached. Moving to COLLABORATIVE.");

    solver().removeTask(leftEeTask_);
    solver().removeTask(rightEeTask_);

    transitionTo(&DualArmControl::entryStateCollaborative, &DualArmControl::stateCollaborative);
  }
}

// =========================================================================
// STATO 3: COLLABORATIVE (Manipolazione Oggetto Virtuale)
// =========================================================================

GraspFrame DualArmControl::buildGraspFrame() const
{
  const auto X_left = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const auto X_right = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  const Eigen::Vector3d pL = X_left.translation();
  const Eigen::Vector3d pR = X_right.translation();
  const Eigen::Vector3d center = 0.5 * (pL + pR);

  Eigen::Vector3d x = (pR - pL).normalized();
  Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  if (std::abs(x.dot(z)) > 0.95)
    z = Eigen::Vector3d::UnitY();

  Eigen::Vector3d y = z.cross(x).normalized();
  z = x.cross(y).normalized();

  Eigen::Matrix3d R;
  R.col(0) = x;
  R.col(1) = y;
  R.col(2) = z;

  sva::PTransformd object(Eigen::Quaterniond(R), center);
  return {
      object,
      X_left * object.inv(),
      X_right * object.inv(),
      X_left,
      X_right};
}

void DualArmControl::registerCollaborativeLogs()
{
  if (logsRegistered_)
    return;
  logsRegistered_ = true;

  logger().addLogEntry("error_dual_left_arm_pose_norm", [this]()
                       { return leftImpedanceTask_ ? leftImpedanceTask_->eval().norm() : 0.0; });

  logger().addLogEntry("error_dual_right_arm_pose_norm", [this]()
                       { return rightImpedanceTask_ ? rightImpedanceTask_->eval().norm() : 0.0; });

  logger().addLogEntry("error_dual_object_translation_norm", [this]()
                       { return objectPosError_.norm(); });

  logger().addLogEntry("error_dual_left_arm_force_X", [this]()
                       { return std::abs(leftForceError_(3)); });
  logger().addLogEntry("error_dual_left_arm_force_Y", [this]()
                       { return std::abs(leftForceError_(4)); });
  logger().addLogEntry("error_dual_left_arm_force_Z", [this]()
                       { return std::abs(leftForceError_(5)); });

  logger().addLogEntry("error_dual_right_arm_force_X", [this]()
                       { return std::abs(rightForceError_(3)); });
  logger().addLogEntry("error_dual_right_arm_force_Y", [this]()
                       { return std::abs(rightForceError_(4)); });
  logger().addLogEntry("error_dual_right_arm_force_Z", [this]()
                       { return std::abs(rightForceError_(5)); });

  logger().addLogEntry("error_dual_left_arm_torque_X", [this]()
                       { return leftForceError_(0); });
  logger().addLogEntry("error_dual_left_arm_torque_Y", [this]()
                       { return leftForceError_(1); });
  logger().addLogEntry("error_dual_left_arm_torque_Z", [this]()
                       { return leftForceError_(2); });

  logger().addLogEntry("error_dual_right_arm_torque_X", [this]()
                       { return rightForceError_(0); });
  logger().addLogEntry("error_dual_right_arm_torque_Y", [this]()
                       { return rightForceError_(1); });
  logger().addLogEntry("error_dual_right_arm_torque_Z", [this]()
                       { return rightForceError_(2); });
}

void DualArmControl::entryStateCollaborative()
{
  mc_rtc::log::info("[FSM] Entering COLLABORATIVE Phase (Single Unified Smooth Trajectory).");

  // BUILD GRASP FRAME
  //------------------------------------------

  auto grasp = buildGraspFrame();

  x_0_objectCurrent_ = grasp.object;
  x_0_objectStart_ = grasp.object;

  leftOffset_ = grasp.leftOffset;
  rightOffset_ = grasp.rightOffset;

  x_0_objectWaypoint1_ =
      sva::PTransformd(
          Eigen::Quaterniond(grasp.object.rotation()),
          Eigen::Vector3d(0.50, 0.0, 0.09));

  collaborativeTime_ = 0.0;
  totalTrajectoryDuration_ = 6.0;

  // CONTACT FORCES
  //------------------------------------------
  sva::ForceVecd targetWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -15.0));

  // Set initial targets explicitly to current poses before loop starts
  leftImpedanceTask_->targetPose(grasp.leftEE);
  rightImpedanceTask_->targetPose(grasp.rightEE);
  leftImpedanceTask_->targetWrench(targetWrench);
  rightImpedanceTask_->targetWrench(targetWrench);

  solver().addTask(leftImpedanceTask_);
  solver().addTask(rightImpedanceTask_);

  leftForceError_ = leftImpedanceTask_->targetWrench().vector() - leftImpedanceTask_->measuredWrench().vector();
  rightForceError_ = rightImpedanceTask_->targetWrench().vector() - rightImpedanceTask_->measuredWrench().vector();

  registerCollaborativeLogs();
}

// Helper: Computes the smoothly interpolated desired object pose
sva::PTransformd DualArmControl::computeDesiredObjectPose()
{
  collaborativeTime_ += timeStep;
  double t_norm = std::min(1.0, collaborativeTime_ / totalTrajectoryDuration_);

  if (t_norm >= 1.0)
  {
    return x_0_objectWaypoint1_;
  }

  // Smooth quintic profile
  const double t2 = t_norm * t_norm;
  const double t3 = t2 * t_norm;
  const double s = t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));

  Eigen::Vector3d startPos = x_0_objectStart_.translation();
  Eigen::Vector3d targetPos = x_0_objectWaypoint1_.translation();

  sva::PTransformd desiredPose = x_0_objectCurrent_;
  desiredPose.translation() = startPos + s * (targetPos - startPos);

  Eigen::Quaterniond q_start(x_0_objectStart_.rotation());
  Eigen::Quaterniond q_target(x_0_objectWaypoint1_.rotation());
  desiredPose.rotation() = q_start.slerp(s, q_target).toRotationMatrix();

  return desiredPose;
}

// Helper: Updates error tracking and telemetry data
void DualArmControl::updateTelemetry(const sva::PTransformd &objectDesired)
{
  const auto &leftRobot = robots().robot(leftRobotIndex_);
  const auto &rightRobot = robots().robot(rightRobotIndex_);

  const sva::PTransformd X_0_leftEE = leftRobot.bodyPosW(eeName_);
  const sva::PTransformd X_0_rightEE = rightRobot.bodyPosW(eeName_);

  Eigen::Vector3d actualObjectCenter = 0.5 * (X_0_leftEE.translation() + X_0_rightEE.translation());

  // Position and orientation errors
  objectPosError_ = objectDesired.translation() - actualObjectCenter;
  Eigen::Matrix3d R_error = objectDesired.rotation() * X_0_leftEE.rotation().transpose();
  objectOriError_ = Eigen::AngleAxisd(R_error).angle();

  // Force errors
  leftForceError_ = leftImpedanceTask_->targetWrench().vector() - leftImpedanceTask_->measuredWrench().vector();
  rightForceError_ = rightImpedanceTask_->targetWrench().vector() - rightImpedanceTask_->measuredWrench().vector();
}

// Main State Function
void DualArmControl::stateCollaborative()
{
  // 1. Trajectory generation
  x_0_objectCurrent_ = computeDesiredObjectPose();

  // 2. Telemetry and logging
  updateTelemetry(x_0_objectCurrent_);

  // 3. Position composition for individual impedance targets
  leftImpedanceTask_->targetPose(leftOffset_ * x_0_objectCurrent_);
  rightImpedanceTask_->targetPose(rightOffset_ * x_0_objectCurrent_);
}

bool DualArmControl::run()
{
  (this->*currentState_)();
  return mc_control::MCController::run();
}

CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)
