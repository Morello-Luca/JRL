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
  // mc_rtc uses 6D vectors ordered as: [Angular_X, Angular_Y, Angular_Z, Linear_X, Linear_Y, Linear_Z]
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
  // sva::ForceVecd targetWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 10.0));
  // leftImpedanceTask_->targetWrench(targetWrench);
  // rightImpedanceTask_->targetWrench(targetWrench);

  // Telemetria (Logger)
  logger().addLogEntry("debug_LeftTask_eval_norm", [this]()
                       { return leftEeTask_ ? leftEeTask_->eval().norm() : 0.0; });
  logger().addLogEntry("debug_RightTask_eval_norm", [this]()
                       { return rightEeTask_ ? rightEeTask_->eval().norm() : 0.0; });

  // Interfaccia Grafica (GUI)
  gui()->addElement({"DualArm"}, mc_rtc::gui::Transform("Object Virtual Frame", [this]() -> const sva::PTransformd &
                                                        { return x_0_objectCurrent_; }));

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
// STATO 2: INDEPENDENT (Raggiungimento Waypoint Singoli)
// =========================================================================
void DualArmControl::entryStateIndependent()
{
  mc_rtc::log::info("[FSM] Entering INDEPENDENT Phase.");

  solver().addTask(leftEeTask_);
  solver().addTask(rightEeTask_);

  leftEeTask_->add_ef_pose({Eigen::Vector3d(0.2, 0.0, 0.4)});
  rightEeTask_->add_ef_pose({Eigen::Vector3d(0.2, -0.0, 0.4)});

  postureTask->stiffness(1.0);
  rightPostureTask_->stiffness(1.0);
}

void DualArmControl::stateIndependent()
{
  // Criteri di convergenza cartesiana e di velocità
  const bool leftConverged = leftEeTask_->eval().norm() < 0.03 && leftEeTask_->speed().norm() < 0.005;
  const bool rightConverged = rightEeTask_->eval().norm() < 0.03 && rightEeTask_->speed().norm() < 0.005;

  if (leftConverged && rightConverged)
  {
    mc_rtc::log::success("[FSM] Independent targets reached. Moving to COLLABORATIVE.");

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
  mc_rtc::log::info("[FSM] Entering COLLABORATIVE Phase (Object Interpolation).");

  const sva::PTransformd X_0_leftEE = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const sva::PTransformd X_0_rightEE = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  // Calcolo del centro geometrico tra i due End Effector
  const Eigen::Vector3d middleTranslation = (X_0_leftEE.translation() + X_0_rightEE.translation()) * 0.5;
  const Eigen::Matrix3d objectRotation = robots().robot(leftRobotIndex_).bodyPosW("link_base").rotation();

  // Inizializzazione del frame virtuale dell'oggetto
  x_0_objectCurrent_ = sva::PTransformd(objectRotation, middleTranslation);

  // Calcolo degli offset relativi rigidi usando l'algebra delle trasformazioni mc_rtc
  leftOffset_ = X_0_leftEE * x_0_objectCurrent_.inv();
  rightOffset_ = X_0_rightEE * x_0_objectCurrent_.inv();

  // Setup dei task di impedenza sulla posizione corrente
  leftImpedanceTask_->targetPose(X_0_leftEE);
  rightImpedanceTask_->targetPose(X_0_rightEE);

  solver().addTask(leftImpedanceTask_);
  solver().addTask(rightImpedanceTask_);

  // Definizione del Target Finale dell'oggetto virtuale
  x_0_objectFinalWaypoint_ = sva::PTransformd(objectRotation, Eigen::Vector3d(0.50, 0.25, 0.09));
}

void DualArmControl::stateCollaborative()
{
  const Eigen::Vector3d errorPos = x_0_objectFinalWaypoint_.translation() - x_0_objectCurrent_.translation();
  const double dist = errorPos.norm();

  const double targetSpeed = 0.05; // 5 cm/s
  const double maxStep = targetSpeed * timeStep;

  // Interpolazione lineare della traslazione (evitando divisioni per zero)
  if (dist > maxStep)
  {
    x_0_objectCurrent_.translation() += errorPos * (maxStep / dist);
  }
  else
  {
    x_0_objectCurrent_ = x_0_objectFinalWaypoint_;
  }

  // Aggiornamento dei target di impedenza preservando la cinematica rigida dell'oggetto
  leftImpedanceTask_->targetPose(leftOffset_ * x_0_objectCurrent_);
  rightImpedanceTask_->targetPose(rightOffset_ * x_0_objectCurrent_);
}

// ==========================================
// MAIN REAL-TIME LOOP
// ==========================================
bool DualArmControl::run()
{
  (this->*currentState_)();
  return mc_control::MCController::run();
}

CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)