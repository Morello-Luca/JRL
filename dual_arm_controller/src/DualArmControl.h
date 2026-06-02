#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_solver/CollisionsConstraint.h>
#include <mc_tasks/TransformTask.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_rbdyn/RobotFrame.h>


#include "api.h"


enum ControllerPhase
{
  IDLE = 0,
  DUAL,
  INDIPENDENT
};




struct DualArmControl_DLLAPI DualArmControl : public mc_control::MCController
{
  DualArmControl(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;



  private:
    //  Controller state
        ControllerPhase phase_ = IDLE;

    // Collision avoidance
        const double iDist = 0.1;
        const double sDist = 0.05;

    // TASKS
      // Position
        std::shared_ptr<mc_tasks::EndEffectorTask> LeftEndEffectorTask_;
        std::shared_ptr<mc_tasks::EndEffectorTask> RightEndEffectorTask_;
      // Force 
        std::shared_ptr<mc_tasks::force::ImpedanceTask> leftImpedanceTask_;
        std::shared_ptr<mc_tasks::force::ImpedanceTask> rightImpedanceTask_;
    
    // FUNCTIONS
        void runDual();
        void runIndipendent();  
        void runOffsetFrame();  

    // Offset Frames
        std::shared_ptr<mc_rbdyn::RobotFrame> leftOffsetFrame_;
        std::shared_ptr<mc_rbdyn::RobotFrame> rightOffsetFrame_;

      


};