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
  sva::ForceVecd targetWrench(Eigen::Vector3d::Zero(), Eigen::Vector3d(0.0, 0.0, 25.0));
  leftImpedanceTask_->targetWrench(targetWrench);
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
    //    transitionTo(&DualArmControl::entryStateIndependent, &DualArmControl::stateIndependent);
    transitionTo(&DualArmControl::entryStateCollaborative, &DualArmControl::stateCollaborative);
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

  leftEeTask_->add_ef_pose({Eigen::Vector3d(0.50, 0.25, 0.09)});
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
  mc_rtc::log::info("[FSM] Entering COLLABORATIVE Phase (True Slerp & Linear Trajectory).");

  const sva::PTransformd X_0_leftEE = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
  const sva::PTransformd X_0_rightEE = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

  // Calcolo frame oggetto iniziale reale
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

  // SALVIAMO IL PUNTO DI PARTENZA REALE
  x_0_objectStart_ = x_0_objectCurrent_;

  // Calcolo offset rigidi fissi
  leftOffset_ = X_0_leftEE * x_0_objectCurrent_.inv();
  rightOffset_ = X_0_rightEE * x_0_objectCurrent_.inv();

  // Reset del timer della traiettoria
  collaborativeTime_ = 0.0;
  collaborativeWaypointIndex_ = 0;

  // Definizione dei Waypoint fissi nel mondo
  Eigen::Quaterniond q0(x_0_objectStart_.rotation());
  x_0_objectWaypoint1_ = sva::PTransformd(q0, Eigen::Vector3d(0.50, 0.25, 0.09));

  Eigen::Quaterniond q_rot(sva::RotZ(M_PI / 4.0));
  x_0_objectWaypoint2_ = sva::PTransformd(q_rot * q0, Eigen::Vector3d(0.50, 0.25, 0.09));

  // Configurazione iniziale task
  const auto &envFrame = robots().env().frame("ground");
  leftImpedanceTask_->targetFrame(envFrame, X_0_leftEE);
  rightImpedanceTask_->targetFrame(envFrame, X_0_rightEE);
  solver().addTask(leftImpedanceTask_);
  solver().addTask(rightImpedanceTask_);
}

void DualArmControl::stateCollaborative()
{
  // Selezioniamo il target corrente
  sva::PTransformd targetWaypoint = (collaborativeWaypointIndex_ == 0) ? x_0_objectWaypoint1_ : x_0_objectWaypoint2_;

  // Avanzamento del tempo dello stato
  collaborativeTime_ += timeStep;

  // Calcolo del parametro normalizzato del tempo t_norm in [0, 1]
  double t_norm = std::min(1.0, collaborativeTime_ / totalTrajectoryDuration_);

  // --- PROFILO QUINTICO MORBIDO (Polinomio di 5° grado per S) ---
  // s(0) = 0, s(1) = 1. Velocità e accelerazioni iniziali/finali sono a zero.
  double s = 10.0 * std::pow(t_norm, 3) - 15.0 * std::pow(t_norm, 4) + 6.0 * std::pow(t_norm, 5);

  // --- INTERPOLAZIONE SINCRONIZZATA ---
  // 1. Traslazione Lineare perfetta basata su s
  Eigen::Vector3d startPos = x_0_objectStart_.translation();
  Eigen::Vector3d targetPos = targetWaypoint.translation();
  x_0_objectCurrent_.translation() = startPos + s * (targetPos - startPos);

  // 2. Rotazione SLERP perfetta basata sullo stesso identico s
  Eigen::Quaterniond q_start(x_0_objectStart_.rotation());
  Eigen::Quaterniond q_target(targetWaypoint.rotation());
  x_0_objectCurrent_.rotation() = q_start.slerp(s, q_target).toRotationMatrix();

  // Controllo fine traiettoria per questo waypoint
  if (t_norm >= 1.0)
  {
    if (collaborativeWaypointIndex_ == 0)
    {
      mc_rtc::log::info("[FSM] Waypoint 1 compleded smoothly. Switching to Waypoint 2.");
      collaborativeWaypointIndex_ = 1;
      collaborativeTime_ = 0.0;              // Resetta il tempo per il prossimo pezzo
      x_0_objectStart_ = x_0_objectCurrent_; // La nuova partenza è dove ci troviamo ora
    }
    else
    {
      // Traiettoria finita, mantieni l'ultimo target stabile
      x_0_objectCurrent_ = targetWaypoint;
    }
  }

  // =========================================================================
  // 4. GENERAZIONE TARGET PER I MANIPOLATORI
  // =========================================================================
  const auto &envFrame = robots().env().frame("ground");

  sva::PTransformd X_0_leftEETarget = leftOffset_ * x_0_objectCurrent_;
  sva::PTransformd X_0_rightEETarget = rightOffset_ * x_0_objectCurrent_;

  leftImpedanceTask_->targetFrame(envFrame, X_0_leftEETarget);
  rightImpedanceTask_->targetFrame(envFrame, X_0_rightEETarget);
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
