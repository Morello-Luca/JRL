#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/ImpedanceTask.h>

class DualArmControl : public mc_control::MCController
{
public:
  DualArmControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config);

  void reset(const mc_control::ControllerResetData &reset_data) override;
  bool run() override;

private:
  // 1. ==========================================================
  // THE DUAL-POINTER FUNCTION DECLARATIONS
  // ==========================================================
  using StateMethod = void (DualArmControl::*)();
  StateMethod currentState_;

  void stateNoOp() {}
  void transitionTo(StateMethod entryMethod, StateMethod runMethod);

  // State Pairs (Entry and Run)
  void entryStateIdle();
  void stateIdle();

  void entryStateIndependent();
  void stateIndependent();

  void entryStateCollaborative();
  void stateCollaborative();
  void offsetCalc();

  // 2. ==========================================================
  // PRE-ALLOCATED TASK POINTERS (Fixed naming to match .cpp)
  // ==========================================================
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> leftEeTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> rightEeTask_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> leftImpedanceTask_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> rightImpedanceTask_;

  // 3. ==========================================================
  // STATE MEMBER VARIABLES (Fixed lowercase 'x' to match .cpp)
  // ==========================================================
  sva::PTransformd x_0_objectCurrent_;
  sva::PTransformd leftOffset_;
  sva::PTransformd rightOffset_;

  // Gestione Multi-Waypoint Collaborativi
  int collaborativeWaypointIndex_ = 0;   // Traccia il waypoint corrente (0 = Primo, 1 = Secondo)
  sva::PTransformd x_0_objectWaypoint1_; // Vecchio x_0_objectFinalWaypoint_
  sva::PTransformd x_0_objectWaypoint2_; // Nuovo secondo waypoint

  // 4. ==========================================================
  // CONTROLLER CONSTANTS AND UTILITIES
  // ==========================================================
  unsigned int leftRobotIndex_ = 0;
  unsigned int rightRobotIndex_ = 0;

  const std::string eeName_ = "link7";
  double stateTimer_ = 0.0;

  double iDist = 0.05;
  double sDist = 0.01;
  double collaborativeTime_ = 0.0;
  double totalTrajectoryDuration_ = 5.0;
  sva::PTransformd x_0_objectStart_;

  // 4. ==========================================================
  // IMPEDANCE GAINS (M, D, K matrices)
  // ==========================================================
  Eigen::Vector6d massGains;   // Virtual Mass (kg / kg*m^2)
  Eigen::Vector6d springGains; // Virtual Stiffness (N/m)
  // Analytically compute critical damping: D = 2 * sqrt(K * M) to avoid oscillations
  Eigen::Vector6d damperGains;
  // Wrench feedforward/feedback gain matrix (dimensionless scaling factor)
  Eigen::Vector6d wrenchGains;
};
