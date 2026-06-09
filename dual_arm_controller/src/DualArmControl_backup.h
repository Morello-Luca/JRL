#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/AdmittanceTask.h>
#include <mc_rbdyn/RobotFrame.h>

#include "api.h"

enum ControllerPhase
{
  IDLE = 0,
  DUAL,
  SINGLE
};

enum SwitchingState
{
  COLLABORATIVE = 0,
  FREE
};

struct DualArmControl_DLLAPI DualArmControl : public mc_control::MCController
{
  DualArmControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  bool run() override;

  void reset(const mc_control::ControllerResetData &reset_data) override;

private:
  //  Controller state
  ControllerPhase phase_ = IDLE;
  SwitchingState switch_state = FREE;

  // Collision avoidance
  const double iDist = 0.1;
  const double sDist = 0.05;

  // TASKS
  // Position
  std::shared_ptr<mc_tasks::EndEffectorTask> LeftEndEffectorTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> RightEndEffectorTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> LeftOffsetTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> RightOffsetTask_;
  // Force
  std::shared_ptr<mc_tasks::force::AdmittanceTask> leftAdmittanceTask_;
  std::shared_ptr<mc_tasks::force::AdmittanceTask> rightAdmittanceTask_;
  // Posture
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_;

  bool waypointSet_ = false;

  // FUNCTIONS
  void runDual();
  void runIndipendent();
  void changeFrame();

  // Offset Frames
  sva::PTransformd X_0_objectTarget_;
  sva::PTransformd leftOffset_;
  sva::PTransformd rightOffset_;

  unsigned int leftRobotIndex;
  unsigned int rightRobotIndex;

  // Definiamo il tipo per il puntatore a un metodo della classe che non prende argomenti e restituisce void
  using StateMethod = void (DualArmControl::*)();
  // Il puntatore allo stato corrente
  StateMethod currentState_ = nullptr;
  // I metodi che rappresentano gli stati reali
  void stateIdle();
  void stateCollaborative();
  void stateIndependent();
  // Metodo helper per cambiare stato in modo sicuro
  void transitionTo(StateMethod newState);
};