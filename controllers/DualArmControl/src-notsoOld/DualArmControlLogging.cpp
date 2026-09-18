#include "DualArmControl.h"

void DualArmControl::registerCollaborativeLogs(){
              // =========================================================================
              // FILTER LOGS
              // =========================================================================
              addLog("CurrentState",static_cast<int>(currentFsmState_));
              // =========================================================================
              // FILTER LOGS
              // =========================================================================
              addLog("Filters_AlphaBeta_Smooth_state",       smoothFilter_.state);
              addLog("Filters_AlphaBeta_Balanced_state",     balancedFilter_.state);
              addLog("Filters_AlphaBeta_Fast_state",         fastFilter_.state);
              addLog("Filters_AlphaBeta_Smooth_derivative",  smoothFilter_.derivative);
              addLog("Filters_AlphaBeta_Balanced_derivative",balancedFilter_.derivative);
              addLog("Filters_AlphaBeta_Fast_derivative",    fastFilter_.derivative);

              addLog("Filters_LowPass_VerySmooth",  lambdaFiltered_lowpass_moltosmooth);
              addLog("Filters_LowPass_Smooth",      lambdaFiltered_lowpass_smooth);
              addLog("Filters_LowPass_Mid",         lambdaFiltered_lowpass_mid);
              addLog("Filters_LowPass_High",        lambdaFiltered_lowpass_high);
              addLog("Filters_LowPass_SamplingCut", lambdaFiltered_lowpass_cut);
              // =========================================================================
              // FORCE CONTROL
              // =========================================================================
              addLog("InternalForce_Desired", gains.lambda_desired);
              addLog("InternalForce_Measured", meas);
              addLog("InternalForce_Error", error_internalForce);
              addLog("InternalForce_lambdaCommand", lambdaCommand_);
              addLog("ContactForce_Reference_Left", cmdLeft);
              addLog("ContactForce_Reference_Right", cmdRight);
                     
              addLog("InternalForce_DemandForce", demandforce);
              

              // =========================================================================
              // TRAJECTORY ENDEFFECTORS
              // =========================================================================
              addLog("EndEffectorsTajectory_LeftArm_ActualTranslation",   robots().robot(leftRobotIndex_).bodyPosW(eeName_).translation());
              //addLog("EndEffectorsTajectory_LeftArm_ActualOrientation", robots().robot(leftRobotIndex_).bodyPosW(eeName_).translation());
              addLog("EndEffectorsTajectory_RightArm_ActualTranslation",  robots().robot(rightRobotIndex_).bodyPosW(eeName_).translation());
              //addLog("EndEffectorsTajectory_LeftArm_ActualOrientation", robots().robot(leftRobotIndex_).bodyPosW(eeName_).translation());
              addLog("EndEffectorsTajectory_LeftArm_TargetTranslation",   X_0_leftTarget.translation());
              addLog("EndEffectorsTajectory_RightArm_TargetTranslation",  X_0_rightTarget.translation());            
              // =========================================================================
              // TRAJECTORY OBJECT
              // =========================================================================              
              addLog("ObjectTrajectory_Desired",  x_0_objectCurrent_.translation());
              logger().addLogEntry("ObjectTrajectory_Real",[this](){return Eigen::Vector3d(
                     0.5 * (robots().robot(leftRobotIndex_).bodyPosW(eeName_).translation() +
                            robots().robot(rightRobotIndex_).bodyPosW(eeName_).translation()));});
              //addLog("IfInactive", !(gains.lambda_desired - lambdaMeasured_ < 2));
}