#include "DualArmControl.h"
#include <mc_rbdyn/RobotLoader.h>

DualArmControl::DualArmControl(
    mc_rbdyn::RobotModulePtr rm,
    double dt,
    const mc_rtc::Configuration &config)
    : mc_control::MCController(rm, dt, config){
       // Configurazione iniziale dei vincoli globali
       solver().addConstraintSet(contactConstraint);
       solver().addConstraintSet(kinematicsConstraint);

       solver().setContacts({{}});
       // Caricamento del secondo braccio (xArm7)
       std::vector<std::string> loaderArgs = {"xArm7", "name", "xarm7_2"};
       auto xarm2Module = mc_rbdyn::RobotLoader::get_robot_module(loaderArgs);

       if (!xarm2Module){
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

void DualArmControl::reset(const mc_control::ControllerResetData &resetData){
       mc_control::MCController::reset(resetData);
       // ROBOT INDEX
              leftRobotIndex_ = robots().robotIndex("xarm7");
              rightRobotIndex_ = robots().robotIndex("xarm7_2");

       // TASKS MEMORY ALLOCATION
              rightPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(), rightRobotIndex_,    1.0, 0.1);
              leftPostureTask_ = std::make_shared<mc_tasks::PostureTask>(solver(),  leftRobotIndex_ ,    1.0, 0.1);
              leftPostureTask_->stiffness(0.5);
              rightPostureTask_->stiffness(0.5);
              mc_rtc::log::info("========================================================================");
              mc_rtc::log::info("Backend {}", (int)solver().backend());
              mc_rtc::log::info("========================================================================");

              // Speed bound constraint
              speedLeftConstr_ =std::make_unique<mc_solver::BoundedSpeedConstr>(robots(),leftRobotIndex_,solver().dt());
              speedRightConstr_ =std::make_unique<mc_solver::BoundedSpeedConstr>(robots(),rightRobotIndex_,solver().dt());

              
              leftEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), leftRobotIndex_, 1.0);
              rightEeTask_ = std::make_shared<mc_tasks::EndEffectorTask>(eeName_, robots(), rightRobotIndex_, 1.0);

              leftImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), leftRobotIndex_, 1.0, 100.0);
              rightImpedanceTask_ = std::make_shared<mc_tasks::force::ImpedanceTask>(eeName_, robots(), rightRobotIndex_, 1.0, 100.0);
       // ADD TASKS
              solver().addTask(leftPostureTask_);
              solver().addTask(rightPostureTask_);
       // GAINS
              configureGains();
              registerCollaborativeLogs();

              transitionTo(&DualArmControl::entryStateIdle, &DualArmControl::stateIdle);
}




// =========================================================================
// TRANSITIONS MANAGER
// =========================================================================
void DualArmControl::transitionTo(StateMethod entryMethod, StateMethod runMethod){
       if (!entryMethod || !runMethod)
              return;
       stateTimer_ = 0.0; // Reset deterministico del tempo ad ogni cambio stato
       (this->*entryMethod)();    // Esecuzione immediata del setup dello stato
       currentState_ = runMethod; // Switch del puntatore per i cicli successivi
}
// =========================================================================
// STATO 1: IDLE
// =========================================================================
void DualArmControl::entryStateIdle(){
       currentFsmState_ = FSMState::IDLE;
       mc_rtc::log::info("[FSM] Entering IDLE Phase (Holding for 3.0 seconds)...");
}
void DualArmControl::stateIdle(){
       stateTimer_ += timeStep;
       if (stateTimer_ >= 3.0){
              mc_rtc::log::success("[FSM] IDLE >> ==== INDEPENDENT ====");
             transitionTo(&DualArmControl::entryStateIndependent, &DualArmControl::stateIndependent);
       }
}
// =========================================================================
// STATO 2: INDEPENDENT (Raggiungimento Waypoint Singoli con Rotazione)
// =========================================================================
void DualArmControl::entryStateIndependent(){
       currentFsmState_ = FSMState::INDEPENDENT;
       mc_rtc::log::info("[FSM] Entering INDEPENDENT Phase with 90-degree target rotation.");
       // 0. Tasks allocation
              solver().addTask(leftEeTask_);
              solver().addTask(rightEeTask_);
       // 1. Cattura gli orientamenti correnti prima della transizione
              Eigen::Matrix3d R_left_start = robots().robot(leftRobotIndex_).bodyPosW(eeName_).rotation();
              Eigen::Matrix3d R_right_start = robots().robot(rightRobotIndex_).bodyPosW(eeName_).rotation();
       // 2. Ruota di 90 gradi attorno all'asse Z relativo
              Eigen::Matrix3d R_rot_90           = sva::RotX(M_PI / 2.0);
              Eigen::Matrix3d R_left_target      = R_rot_90 * R_left_start;
              Eigen::Matrix3d R_right_target     = R_rot_90.transpose() * R_right_start;
       // 3. Crea i frame spaziali completi (Posizione + Rotazione)
              X_0_leftTarget = sva::PTransformd(R_left_target, Eigen::Vector3d(0.50, 0.10, 0.08));
              X_0_rightTarget = sva::PTransformd(R_right_target, Eigen::Vector3d(0.50, 0.40, 0.08));
       // 4. Invia i comandi contemporanei al solutore cinematico
              leftEeTask_->set_ef_pose(X_0_leftTarget);
              rightEeTask_->set_ef_pose(X_0_rightTarget);
}
void DualArmControl::stateIndependent(){
       // Criteri di convergenza sia cartesiana che di velocità angolare/lineare
              const bool leftConverged = leftEeTask_  ->eval().norm() < 0.03 && leftEeTask_ ->speed().norm() < 0.005;
              const bool rightConverged = rightEeTask_->eval().norm() < 0.03 && rightEeTask_->speed().norm() < 0.005;
       // SWITCH
              if (leftConverged && rightConverged){
                     mc_rtc::log::success("[FSM] REACHING PHASE COMPLETED. MOVING TO >> ==== COLLABORATIVE ====");
                     solver().removeTask(leftEeTask_);
                     solver().removeTask(rightEeTask_);
                     transitionTo(&DualArmControl::entryStateCollaborative, &DualArmControl::stateCollaborative);
              }
}
// =========================================================================
// STATO 3: COLLABORATIVE 
// =========================================================================



void DualArmControl::entryStateCollaborative(){
       currentFsmState_ = FSMState::COLLABORATIVE;
       mc_rtc::log::info("[FSM] Entering COLLABORATIVE Phase (Single Unified Smooth Trajectory).");
       collabSubState_ = CollabSubState::BUILD_GRASP;
       stateTimer_ = 0;

       leftImpedanceTask_->targetPose(robots().robot(leftRobotIndex_).bodyPosW(eeName_));
       rightImpedanceTask_->targetPose(robots().robot(rightRobotIndex_).bodyPosW(eeName_));
       // Attiviamo i task di impedenza nel solutore QP
       gains.springGains << 10.0, 10.0, 10.0, 40.0, 40.0, 2.0;
       gains.wrenchGains << 0.0, 0.0, 0.0,  0.0,  0.0,  0;
       
       setImpedanceGains(gains.springGains, gains.springGains, gains.wrenchGains, 4);
       solver().addTask(leftImpedanceTask_);
       solver().addTask(rightImpedanceTask_);
       rampTime = 8.0; 
       lambdaStart = 0;

       spd(5) = 0.01;
       speedLeftConstr_->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
       speedRightConstr_->addBoundedSpeed(solver(),eeName_,Eigen::Vector3d::Zero(),dof,-spd,spd);
       solver().addConstraintSet(speedLeftConstr_);
       solver().addConstraintSet(speedRightConstr_);
       

}
void DualArmControl::stateCollaborative(){
       // Incrementiamo il timer interno ad ogni loop
       
       switch (collabSubState_){
              case CollabSubState::BUILD_GRASP:{   
              stateTimer_ += timeStep;           


              double mLeft  = computeReflectedMassZ(leftRobotIndex_, eeName_);
              double mRight = computeReflectedMassZ(rightRobotIndex_, eeName_);

              double m_reflected_z = std::min(mLeft,mRight);

              double r = 0.3; // rapporto massa virtuale/massa riflessa, <1 per garanzia anti-rimbalzo
              double M_virtual_z = r * m_reflected_z;
              gains.massGains(5) = M_virtual_z;   

              Eigen::Vector6d damper  = 2.0 * 3 * gains.springGains.cwiseProduct(gains.massGains).cwiseSqrt();

                     forBothImpedanceTasks([&](auto &task){
                            task->gains().mass().vec(gains.massGains);
                            task->gains().damper().vec(damper);
                     }); 

              const double contactThreshold = 1.7;

              double delta_z = spd(5) * timeStep;

              forBothImpedanceTasks([&](auto &task) {
              // Otteniamo la posa target attuale
              sva::PTransformd targetPose = task->targetPose();
              // Creiamo una trasformazione di sola traslazione lungo l'asse Z locale
              sva::PTransformd localShift(Eigen::Vector3d(0.0, 0.0, delta_z));
              // Moltiplicando a sinistra (nella convenzione SpaceVecAlg), 
              // applichiamo lo spostamento nel frame locale dell'end-effector
              task->targetPose(localShift * targetPose);
              });

              sva::ForceVecd FL = leftImpedanceTask_->measuredWrench();
              sva::ForceVecd FR = rightImpedanceTask_->measuredWrench();
              //double force = gains.contactForce(stateTimer_);
              //leftImpedanceTask_ -> targetWrench(sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d(0.0, 0.0, force)));
              //rightImpedanceTask_-> targetWrench(sva::ForceVecd(Eigen::Vector3d::Zero(),Eigen::Vector3d(0.0, 0.0, force)));


              if(FL.force().norm() > contactThreshold && FR.force().norm() > contactThreshold){
                     mc_rtc::log::success("[FSM] Contact established.");
                     mc_rtc::log::info("FL = {:.2f}, FR = {:.2f}",FL.force().norm(),FR.force().norm());
                     // Blocca la posa attuale dei due end-effector                     
                     leftImpedanceTask_->targetPose(robots().robot(leftRobotIndex_).bodyPosW(eeName_));
                     rightImpedanceTask_->targetPose(robots().robot(rightRobotIndex_).bodyPosW(eeName_));
                     stateTimer_ = 0;
                     collabSubState_ = CollabSubState::TRAJECTORY;
              }
              break;
              }
              case CollabSubState::TRAJECTORY:{   
                     auto grasp = buildGraspFrame();
                     x_0_objectCurrent_ = grasp.object;
                     x_0_objectStart_ = grasp.object;

                     leftOffset_ = grasp.leftOffset;
                     rightOffset_ = grasp.rightOffset; 
                     
                     solver().removeConstraintSet(speedLeftConstr_);
                     solver().removeConstraintSet(speedRightConstr_);
                     currentInternalForce();
                     lambdaStart = lambdaMeasured_;


                     double roll  = M_PI/2; double pitch = 0.0; double yaw   = 0.0;

                     Eigen::Matrix3d R_mondo_desiderata = (
                            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
                            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX())
                     ).toRotationMatrix();

                     //x_0_objectWaypoint1_ = sva::PTransformd(Eigen::Quaterniond(R*R_mondo_desiderata), x_0_objectStart_.translation());
                     x_0_objectWaypoint1_ = sva::PTransformd(Eigen::Quaterniond(x_0_objectStart_.rotation()),Eigen::Vector3d(0.50, 0.0, 0.08));
                     gains.collaborativeTime_ = 0.0;                  
                     // Passiamo al movimento cooperativo vero e proprio
                     mc_rtc::log::info("[FSM - Collaborative] >> ===== STARTING MOVEMENTS ====");

                     collabSubState_ = CollabSubState::COOPERATIVE_MOTION;
              break;
              }
              case CollabSubState::COOPERATIVE_MOTION:{   
                     stateTimer_ += timeStep;

                     double tau = std::clamp(stateTimer_ / rampTime, 0.0, 1.0);

                     double s = 10*pow(tau,3)
                            -15*pow(tau,4)
                            + 6*pow(tau,5);
                     
                     gains.lambda_desired = lambdaStart + (30.0 - lambdaStart) * s;
                     stateTimer_ += timeStep;
                     gains.springGains << 10.0, 10.0, 10.0, 30.0, 30.0, 4.0;
                     gains.massGains = Eigen::Vector6d::Constant(1);
                     gains.damperGains =  2.0 * gains.springGains.cwiseProduct(gains.massGains).cwiseSqrt();
                     gains.wrenchGains << 0.0, 0.0, 0.0,  0.0,  0.0,  0.004;           
                     forBothImpedanceTasks([&](auto &task){
                            task->gains().mass().vec(gains.massGains);
                            task->gains().spring().vec(gains.springGains);
                            task->gains().damper().vec(gains.damperGains);
                            task->gains().wrench().vec(gains.wrenchGains); 
                     });  
                     //updateContactForces();
                     optimize();
                     //if (std::abs(gains.lambda_desired - lambdaMeasured_) < 2){
                     //       gains.collaborativeTime_ += timeStep;
                    // }
                     gains.collaborativeTime_ += timeStep;
                     x_0_objectCurrent_ = computeDesiredObjectPose();

                     

                     

                     // Converte la posa dell'oggetto nelle pose dei singoli end-effector usando gli offset registrati al contatto.
                     // Se l'HOLD è attivo, mc_rtc userà queste pose solo come direzione di movimento, mantenendo intatta 
                     // la compressione iniziale senza generare forze di fuga asimmetriche.
                     leftImpedanceTask_->targetPose(leftOffset_ * x_0_objectCurrent_ );
                     rightImpedanceTask_->targetPose(rightOffset_ * x_0_objectCurrent_);

              break;
              }
       }
}

bool DualArmControl::run(){
       (this->*currentState_)();
       return mc_control::MCController::run();
}
CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)