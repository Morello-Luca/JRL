#include "DualArmControl.h"
#include <mc_rbdyn/RobotLoader.h>

DualArmControl::DualArmControl(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt, config)
{
  solver().addConstraintSet(contactConstraint);
  solver().addConstraintSet(kinematicsConstraint);
  solver().addTask(postureTask);
  postureTask->stiffness(1.0);
  postureTask->weight(0.1);
  solver().setContacts({{}});

  auto xarm_module = mc_rbdyn::RobotLoader::get_robot_module("xArm7");
  loadRobot(xarm_module, "xarm7_2");
  for (size_t i = 0; i < robots().size(); ++i)
  {
    mc_rtc::log::info("After loadRobot: {} -> {}", i, robots().robot(i).name());
  }

  robots().robot("xarm7_2").posW(sva::PTransformd(sva::RotZ(0.0), Eigen::Vector3d(0, 0.5, 0)));
  mc_rtc::log::success("DualArmControl init done");
  addCollisions("xarm7", "xarm7_2", {{"*", "*", iDist, sDist, 0}});
}

void DualArmControl::reset(const mc_control::ControllerResetData &reset_data)
{
  mc_control::MCController::reset(reset_data);

  leftRobotIndex = robots().robotIndex("xarm7");
  rightRobotIndex = robots().robotIndex("xarm7_2");

  rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), rightRobotIndex, 1.0, 1.0);

  // Lo aggiungiamo subito al solutore in modo che sia attivo fin dall'inizio
  solver().addTask(rightPostureTask_);

  logger().addLogEntry("debug_LeftTask_eval_norm", [this]()
                       { return LeftEndEffectorTask_->eval().norm(); });
  logger().addLogEntry("debug_RightTask_eval_norm", [this]()
                       { return RightEndEffectorTask_->eval().norm(); });
}

void DualArmControl::changeFrame()
{
  if (switch_state == COLLABORATIVE)
  {
    std::string EndEffector = "link7";

    leftAdmittanceTask_ = std::make_shared<mc_tasks::force::AdmittanceTask>(EndEffector, robots(), leftRobotIndex, 200.0, 200);
    rightAdmittanceTask_ = std::make_shared<mc_tasks::force::AdmittanceTask>(EndEffector, robots(), rightRobotIndex, 200.0, 200);

    sva::ForceVecd admittanceGains = sva::ForceVecd::Zero();
    admittanceGains.force().x() = 0.0005;

    leftAdmittanceTask_->admittance(admittanceGains);
    rightAdmittanceTask_->admittance(admittanceGains);

    sva::ForceVecd leftTargetWrench = sva::ForceVecd::Zero();
    leftTargetWrench.force().x() = 0.0;

    sva::ForceVecd rightTargetWrench = sva::ForceVecd::Zero();
    rightTargetWrench.force().x() = 0.0;

    leftAdmittanceTask_->targetWrench(leftTargetWrench);
    rightAdmittanceTask_->targetWrench(rightTargetWrench);

    sva::PTransformd X_0_leftEE = robots().robot(leftRobotIndex).bodyPosW(EndEffector);
    sva::PTransformd X_0_rightEE = robots().robot(rightRobotIndex).bodyPosW(EndEffector);

    Eigen::Vector3d middleTranslation = (X_0_leftEE.translation() + X_0_rightEE.translation()) / 2.0;
    Eigen::Matrix3d objectRotation = robots().robot(leftRobotIndex).bodyPosW("link_base").rotation();
    X_0_objectTarget_ = sva::PTransformd(objectRotation, middleTranslation);

    // MODIFICA: Salviamo gli offset nelle variabili della classe (con il trattino basso _)
    leftOffset_ = X_0_leftEE * X_0_objectTarget_.inv();
    rightOffset_ = X_0_rightEE * X_0_objectTarget_.inv();

    leftAdmittanceTask_->targetPose(leftOffset_ * X_0_objectTarget_);
    rightAdmittanceTask_->targetPose(rightOffset_ * X_0_objectTarget_);

    solver().removeTask(LeftEndEffectorTask_);
    solver().removeTask(RightEndEffectorTask_);
    solver().addTask(leftAdmittanceTask_);
    solver().addTask(rightAdmittanceTask_);

    // Il Robot 0 (sinistro) usa il puntatore predefinito del controller
    postureTask->stiffness(0.5);
    postureTask->weight(0.1);

    // Il Robot 2 (destro) usa il puntatore esplicito che abbiamo creato e aggiunto nel reset
    rightPostureTask_->stiffness(0.5);
    rightPostureTask_->weight(0.1);

    gui()->removeElement({"DualArm"}, "Object Virtual Frame");
    gui()->addElement({"DualArm"},
                      mc_rtc::gui::Transform("Object Virtual Frame",
                                             [this]() -> const sva::PTransformd &
                                             { return X_0_objectTarget_; }));

    mc_rtc::log::success("======================================");
    mc_rtc::log::success("Tasks con offset create correttamente!");
    mc_rtc::log::success("Si passa al DualArm");
    mc_rtc::log::success("======================================");
    phase_ = DUAL;
    switch_state = FREE;
  }
  else
  {
    gui()->removeElement({"DualArm"}, "Object Virtual Frame");
    solver().removeTask(leftAdmittanceTask_);
    solver().removeTask(rightAdmittanceTask_);
    std::string EndEffector = "link7";
    LeftEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(EndEffector, robots(), leftRobotIndex, 1);
    RightEndEffectorTask_ = std::make_shared<mc_tasks::EndEffectorTask>(EndEffector, robots(), rightRobotIndex, 1);
    solver().addTask(LeftEndEffectorTask_);
    solver().addTask(RightEndEffectorTask_);
    mc_rtc::log::success("======================================");
    mc_rtc::log::success("Task normale endeffector");
    mc_rtc::log::success("Si passa al singlearm");
    mc_rtc::log::success("======================================");
    phase_ = SINGLE;
    switch_state = COLLABORATIVE;
  }
}

void DualArmControl::runDual()
{
  // 1. Assegniamo il waypoint una sola volta all'inizio
  if (!waypointSet_)
  {
    X_0_objectTarget_.translation() = Eigen::Vector3d(0.60, 0.7, 0.49);
    sva::MotionVecd maxVelocity(Eigen::Vector3d(0.5, 0.5, 0.5), Eigen::Vector3d(0.1, 0.1, 0.1));
    leftAdmittanceTask_->refVelB(maxVelocity);
    rightAdmittanceTask_->refVelB(maxVelocity);
    waypointSet_ = true;
  }
  // --- LOG ERRORE SNELLO (Usa X_0_objectTarget_ salvato nell'header) ---
  std::string EndEffector = "link7";

  // Posizione reale attuale calcolata al volo dai feedback dei giunti
  Eigen::Vector3d currentPos = (robots().robot(leftRobotIndex).bodyPosW(EndEffector).translation() +
                                robots().robot(rightRobotIndex).bodyPosW(EndEffector).translation()) /
                               2.0;

  // Errore di traslazione puro tra il punto reale attuale e il target nell'header
  double translation_error = (currentPos - X_0_objectTarget_.translation()).norm();

  mc_rtc::log::info("Object Target Error: {} m", translation_error);
  // ---------------------------------------------------------------------

  // 2. I task inseguono il target memorizzato nell'header
  leftAdmittanceTask_->targetPose(leftOffset_ * X_0_objectTarget_);
  rightAdmittanceTask_->targetPose(rightOffset_ * X_0_objectTarget_);
}

void DualArmControl::runIndipendent()
{
  if (!waypointSet_)
  {
    LeftEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.2, 0.2, 0.4)});
    RightEndEffectorTask_->add_ef_pose({Eigen::Vector3d(0.2, -0.2, 0.4)});
    waypointSet_ = true;
  }

  if (LeftEndEffectorTask_->eval().norm() < 0.01 && LeftEndEffectorTask_->speed().norm() < 0.05 &&
      RightEndEffectorTask_->eval().norm() < 0.01 && RightEndEffectorTask_->speed().norm() < 0.05)
  {
    waypointSet_ = false;
    mc_rtc::log::success("Si passa al DualArm");
    changeFrame();
  }
}

bool DualArmControl::run()
{
  switch (phase_)
  {
  case IDLE:
    changeFrame();
    break;

  case DUAL:
    runDual();
    break;

  case SINGLE:

    runIndipendent();
    break;

  default:
    mc_rtc::log::error_and_throw("Errore nella macchina a stati");
  }

  return mc_control::MCController::run();
}

CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)