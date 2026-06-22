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

void DualArmControl::reset(const mc_control::ControllerResetData &resetData)
{
  mc_control::MCController::reset(resetData);

  // Cache degli indici dei robot per performance
  leftRobotIndex_ = robots().robotIndex("xarm7");
  rightRobotIndex_ = robots().robotIndex("xarm7_2");

  // =========================================================================
  // ALLOCAZIONE TASK (Zero memory jitter nel loop real-time)
  // =========================================================================
  rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), rightRobotIndex_, 1.0, 0.1);
  solver().addTask(rightPostureTask_);

  leftEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), leftRobotIndex_, 1.0);
  rightEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), rightRobotIndex_, 1.0);

  leftImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), leftRobotIndex_, 1.0, 100.0);
  rightImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), rightRobotIndex_, 1.0, 100.0);

  // =========================================================================
  // INITIALIZATION OF IMPEDANCE GAINS (M, D, K matrices)
  // =========================================================================
  massGains = Eigen::Vector6d::Constant(1.0);     // Virtual Mass (kg / kg*m^2)
  springGains = Eigen::Vector6d::Constant(200.0); // Virtual Stiffness (N/m)

  // Analytically compute critical damping: D = 2 * sqrt(K * M) to avoid oscillations
  damperGains = 2.0 * springGains.cwiseProduct(massGains).cwiseSqrt();

  // Wrench feedforward/feedback gain matrix (dimensionless scaling factor)
  wrenchGains = Eigen::Vector6d::Constant(1.0);

  // Apply gains to Left Arm Impedance Task
  leftImpedanceTask_->gains().mass().vec(massGains);
  leftImpedanceTask_->gains().spring().vec(springGains);
  leftImpedanceTask_->gains().damper().vec(damperGains);
  leftImpedanceTask_->gains().wrench().vec(wrenchGains);

  // Apply gains to Right Arm Impedance Task
  rightImpedanceTask_->gains().mass().vec(massGains);
  rightImpedanceTask_->gains().spring().vec(springGains);
  rightImpedanceTask_->gains().damper().vec(damperGains);
  rightImpedanceTask_->gains().wrench().vec(wrenchGains);

  // =========================================================================
  // DESIRED CONTACT FORCE SPECIFICATION (10 N)
  // =========================================================================
  /*
  The target wrench is an sva::ForceVecd(couple, force).
  Order: [Torque_X, Torque_Y, Torque_Z, Force_X, Force_Y, Force_Z]

  */

  // Telemetria (Logger)
  logger().addLogEntry("debug_LeftTask_eval_norm", [this]()
                       { return leftEeTask_ ? leftEeTask_->eval().norm() : 0.0; });
  logger().addLogEntry("debug_RightTask_eval_norm", [this]()
                       { return rightEeTask_ ? rightEeTask_->eval().norm() : 0.0; });

  // Interfaccia Grafica (GUI)
  gui()->addElement({"DualArm"}, mc_rtc::gui::Transform("Object Virtual Frame", [this]() -> const sva::PTransformd &
                                                        { return x_0_objectCurrent_; }));

  // =========================================================================
  // ENHANCED TELEMETRY (Structured under error_dual)
  // =========================================================================
  leftForceError_ = Eigen::Vector6d::Zero();
  rightForceError_ = Eigen::Vector6d::Zero();
  objectPosError_ = Eigen::Vector3d::Zero();
  objectOriError_ = 0.0;

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
void DualArmControl::entryStateCollaborative()
{
  mc_rtc::log::info("[FSM] Entering COLLABORATIVE Phase (Single Unified Smooth Trajectory).");

  const sva::PTransformd X_0_leftEE = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const sva::PTransformd X_0_rightEE = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  // =====================================================
  // BUILD GRASP FRAME
  // =====================================================
  const Eigen::Vector3d pL = X_0_leftEE.translation();
  const Eigen::Vector3d pR = X_0_rightEE.translation();
  const Eigen::Vector3d center = 0.5 * (pL + pR);

  Eigen::Vector3d x = (pR - pL).normalized();
  Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  if (std::abs(x.dot(z)) > 0.95)
    z = Eigen::Vector3d::UnitY();
  Eigen::Vector3d y = z.cross(x).normalized();
  z = x.cross(y).normalized();

  Eigen::Matrix3d Robj;
  Robj.col(0) = x;
  Robj.col(1) = y;
  Robj.col(2) = z;

  x_0_objectCurrent_ = sva::PTransformd(Eigen::Quaterniond(Robj), center);
  x_0_objectStart_ = x_0_objectCurrent_;

  // =====================================================
  // CORRECTED RIGID OFFSETS (Transform composition order)
  // =====================================================
  leftOffset_ = X_0_leftEE * x_0_objectCurrent_.inv();
  rightOffset_ = X_0_rightEE * x_0_objectCurrent_.inv();

  // Define target position
  x_0_objectWaypoint1_ = sva::PTransformd(Eigen::Quaterniond(Robj), Eigen::Vector3d(0.50, 0.0, 0.09));

  collaborativeTime_ = 0.0;
  totalTrajectoryDuration_ = 6.0;

  /*
  // =====================================================
  // RESET AND ADD IMPEDANCE TASKS
  // =====================================================
  // Crucial: Reset the impedance tasks to current physical pose to clear any error buffers
  leftImpedanceTask_->reset();
  rightImpedanceTask_->reset();

  // =========================================================================
  // SMOOTH ADMITTANCE INITIALIZATION
  // =========================================================================
  // Rotational components (first 3) can remain relatively firm
  massGains.head<3>() = Eigen::Vector3d::Constant(1.0);
  springGains.head<3>() = Eigen::Vector3d::Constant(100.0);

  // Linear components (last 3) made heavy and soft for impact absorption
  massGains.tail<3>() = Eigen::Vector3d::Constant(4.0);    // Higher inertia (kg)
  springGains.tail<3>() = Eigen::Vector3d::Constant(30.0); // Softer spring (N/m)

  // Critically damped calculation
  damperGains = 2.0 * springGains.cwiseProduct(massGains).cwiseSqrt();

  // Dimensionless wrench scaling
  wrenchGains = Eigen::Vector6d::Constant(1.0);

  // Apply to tasks
  leftImpedanceTask_->gains().mass().vec(massGains);
  leftImpedanceTask_->gains().spring().vec(springGains);
  leftImpedanceTask_->gains().damper().vec(damperGains);
  leftImpedanceTask_->gains().wrench().vec(wrenchGains);
  leftImpedanceTask_->cutoffPeriod(0.02); // Smooth out sensor spikes

  rightImpedanceTask_->gains().mass().vec(massGains);
  rightImpedanceTask_->gains().spring().vec(springGains);
  rightImpedanceTask_->gains().damper().vec(damperGains);
  rightImpedanceTask_->gains().wrench().vec(wrenchGains);
  rightImpedanceTask_->cutoffPeriod(0.02);
  */

  sva::ForceVecd targetWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, -15.0));

  // Set initial targets explicitly to current poses before loop starts
  leftImpedanceTask_->targetPose(X_0_leftEE);
  rightImpedanceTask_->targetPose(X_0_rightEE);
  leftImpedanceTask_->targetWrench(targetWrench);
  rightImpedanceTask_->targetWrench(targetWrench);

  solver().addTask(leftImpedanceTask_);
  solver().addTask(rightImpedanceTask_);

  leftForceError_ = leftImpedanceTask_->targetWrench().vector() - leftImpedanceTask_->measuredWrench().vector();
  rightForceError_ = rightImpedanceTask_->targetWrench().vector() - rightImpedanceTask_->measuredWrench().vector();

  // 1. Task Position/Pose Errors for both arms
  logger().addLogEntry("error_dual_left_arm_pose_norm", [this]()
                       { return leftImpedanceTask_ ? leftImpedanceTask_->eval().norm() : 0.0; });
  logger().addLogEntry("error_dual_right_arm_pose_norm", [this]()
                       { return rightImpedanceTask_ ? rightImpedanceTask_->eval().norm() : 0.0; });

  // 2. Object Translational Error
  logger().addLogEntry("error_dual_object_translation_norm", [this]()
                       { return objectPosError_.norm(); });

  // 3. Force Errors for both arms (Explicit X, Y, Z components to block joint auto-mapping)
  logger().addLogEntry("error_dual_left_arm_force_X", [this]()
                       { return leftForceError_(3).norm(); });
  logger().addLogEntry("error_dual_left_arm_force_Y", [this]()
                       { return leftForceError_(4).norm(); });
  logger().addLogEntry("error_dual_left_arm_force_Z", [this]()
                       { return leftForceError_(5).norm(); });

  logger().addLogEntry("error_dual_right_arm_force_X", [this]()
                       { return rightForceError_(3).norm(); });
  logger().addLogEntry("error_dual_right_arm_force_Y", [this]()
                       { return rightForceError_(4).norm(); });
  logger().addLogEntry("error_dual_right_arm_force_Z", [this]()
                       { return rightForceError_(5).norm(); });

  // 4. Torque Errors for both arms (Optional: components 0, 1, 2)
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

void DualArmControl::stateCollaborative()
{
  collaborativeTime_ += timeStep;
  double t_norm = std::min(1.0, collaborativeTime_ / totalTrajectoryDuration_);

  // Smooth quintic profile
  double s = 10.0 * std::pow(t_norm, 3) - 15.0 * std::pow(t_norm, 4) + 6.0 * std::pow(t_norm, 5);

  Eigen::Vector3d startPos = x_0_objectStart_.translation();
  Eigen::Vector3d targetPos = x_0_objectWaypoint1_.translation();

  // 1. Declare and compute the desired state of your virtual object frame
  sva::PTransformd x_0_objectDesired = x_0_objectCurrent_;
  x_0_objectDesired.translation() = startPos + s * (targetPos - startPos);

  Eigen::Quaterniond q_start(x_0_objectStart_.rotation());
  Eigen::Quaterniond q_target(x_0_objectWaypoint1_.rotation());
  x_0_objectDesired.rotation() = q_start.slerp(s, q_target).toRotationMatrix();

  if (t_norm >= 1.0)
  {
    x_0_objectDesired = x_0_objectWaypoint1_;
  }

  // Update the class variable for the GUI element to render
  x_0_objectCurrent_ = x_0_objectDesired;

  // 2. Fetch current physical End-Effector states to calculate object center estimation
  const sva::PTransformd X_0_leftEE = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const sva::PTransformd X_0_rightEE = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  // Declare actualObjectCenter locally to resolve the compilation error
  Eigen::Vector3d actualObjectCenter = 0.5 * (X_0_leftEE.translation() + X_0_rightEE.translation());

  // 3. Compute and store telemetry data for the logger hooks
  objectPosError_ = x_0_objectDesired.translation() - actualObjectCenter;

  Eigen::Matrix3d R_error = x_0_objectDesired.rotation() * X_0_leftEE.rotation().transpose();
  objectOriError_ = Eigen::AngleAxisd(R_error).angle();

  leftForceError_ = leftImpedanceTask_->targetWrench().vector() - leftImpedanceTask_->measuredWrench().vector();
  rightForceError_ = rightImpedanceTask_->targetWrench().vector() - rightImpedanceTask_->measuredWrench().vector();

  // =====================================================
  // POSITION COMPOSITION FOR IMPEDANCE TARGETS
  // =====================================================
  sva::PTransformd X_0_leftEETarget = leftOffset_ * x_0_objectCurrent_;
  sva::PTransformd X_0_rightEETarget = rightOffset_ * x_0_objectCurrent_;

  leftImpedanceTask_->targetPose(X_0_leftEETarget);
  rightImpedanceTask_->targetPose(X_0_rightEETarget);
}

bool DualArmControl::run()
{
  (this->*currentState_)();
  return mc_control::MCController::run();
}

CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)