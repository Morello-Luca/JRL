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
              X_0_leftTarget = sva::PTransformd(R_left_target, Eigen::Vector3d(0.50, 0.10, 0.1));
              X_0_rightTarget = sva::PTransformd(R_right_target, Eigen::Vector3d(0.50, 0.40, 0.1));
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
       gains.springGains << 10.0, 10.0, 10.0, 40.0, 40.0, 0.5;
       gains.wrenchGains << 0.0, 0.0, 0.0,  0.0,  0.0,  0;
       
       setImpedanceGains(gains.springGains, gains.springGains, gains.wrenchGains, 4);
       solver().addTask(leftImpedanceTask_);
       solver().addTask(rightImpedanceTask_);

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
              const double lostContactThreshold = 0.8;        
              



              sva::ForceVecd FL = leftImpedanceTask_->measuredWrench();
              sva::ForceVecd FR = rightImpedanceTask_->measuredWrench();


              double normFL = FL.force().norm();
              double normFR = FR.force().norm();
              bool inContact = (normFL > contactThreshold && normFR > contactThreshold);
              double stableContact = -6.0;

              if(!contactDetected_){
                     // fase di avvicinamento: continua a spingere come prima
                     double delta_z = spd(5) * timeStep;
                     forBothImpedanceTasks([&](auto &task) {
                            sva::PTransformd targetPose = task->targetPose();
                            sva::PTransformd localShift(Eigen::Vector3d(0.0, 0.0, delta_z));
                            task->targetPose(localShift * targetPose);
                     });
                     if(inContact){
                            contactDetected_ = true;
                            settleTimer_ = 0.0;
                            prevFL_ = normFL;
                            prevFR_ = normFR;
                            
                            leftImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.03));
                            rightImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.03));


leftImpedanceTask_->targetWrench(
    sva::ForceVecd(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, stableContact)));

rightImpedanceTask_->targetWrench(
    sva::ForceVecd(
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d(0.0, 0.0, stableContact)));


                            mc_rtc::log::info("[FSM] Contatto rilevato, attendo assestamento...");
                     }

              }
              else{
                     double dFL = std::abs(normFL - prevFL_) / timeStep;
                     double dFR = std::abs(normFR - prevFR_) / timeStep;
                     prevFL_ = normFL;
                     prevFR_ = normFR;

                     bool lostContact = (normFL < lostContactThreshold) || (normFR < lostContactThreshold);

                     bool stable = (dFL < forceRateThreshold_) && (dFR < forceRateThreshold_);

                     if(!lostContact && stable){
                            settleTimer_ += timeStep;
                     }
                     if(settleTimer_ > settleDuration_){
                            mc_rtc::log::success("[FSM] Contact established and settled.");
                            stateTimer_ = 0;
                            contactDetected_ = false;
                            leftImpedanceTask_->targetPose(robots().robot(leftRobotIndex_).bodyPosW(eeName_));
                            rightImpedanceTask_->targetPose(robots().robot(rightRobotIndex_).bodyPosW(eeName_));
                            collabSubState_ = CollabSubState::TRAJECTORY;
                     }
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
                     x_0_objectWaypoint1_ = sva::PTransformd(Eigen::Quaterniond(x_0_objectStart_.rotation()),Eigen::Vector3d(0.50, 0.25, 0.2));
                     x_0_objectWaypoint2_ = sva::PTransformd(Eigen::Quaterniond(x_0_objectWaypoint1_.rotation()),Eigen::Vector3d(0.50, 0.25, 0.1));

                     
                     gains.collaborativeTime_ = 0.0;                  
                     stateTimer_ = 0.0;
                     rampTime = 10.0,

                     mc_rtc::log::info("[FSM - Collaborative] >> ===== STARTING MOVEMENTS ====");
                     collabSubState_ = CollabSubState::COOPERATIVE_MOTION;
              break;
              }
              case CollabSubState::COOPERATIVE_MOTION:{   
                     


                     currentInternalForce();

                     if(!squeezeForceReached_){

                            double tau = std::clamp(stateTimer__ / rampTime, 0.0, 1.0);
                            double s = 10*pow(tau,3)-15*pow(tau,4)+ 6*pow(tau,5);
                            
                            gains.lambda_desired = lambdaStart + (30.0 - lambdaStart) * s;
                            stateTimer__ += timeStep;
                            
                            leftImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.04));
                            rightImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.04));
                            if(std::abs(meas - 30.0) < 0.4){
                                   ++squeezeStableCounter_;
                                   if(squeezeStableCounter_ > 5){
                                          squeezeForceReached_ = true;
                                          motionStarted_ = true;
                                          stateTimer_ = 0.0;
                                          leftImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.03));
                                          rightImpedanceTask_->gains().wrench().vec(Eigen::Vector3d(0,0,0),Eigen::Vector3d(0,0,0.03));
                                   }

                            }
                            else{
                                   squeezeStableCounter_ = 0;
                            }                         
                     }
else{
    stateTimer_ += timeStep;
    gains.collaborativeTime_ += timeStep;

    // Waypoint target corrente in base a target_
    const sva::PTransformd & targetPose =
        (target_ == 1) ? x_0_objectWaypoint1_ : x_0_objectWaypoint2_;

    double posError = (x_0_objectCurrent_.translation() - targetPose.translation()).norm();

    sva::PTransformd w0 = x_0_objectStart_;
    sva::PTransformd w1 = targetPose;

    if(!cycleComplete_ && posError < 0.005)
    {
        gains.collaborativeTime_ = 0.0;
        x_0_objectStart_ = x_0_objectCurrent_;   // nuovo punto di partenza per il prossimo tratto

        if(target_ == 2)
        {
            ++w2Visits_;
            if(w2Visits_ >= maxW2Visits_)
            {
                cycleComplete_ = true;   // stop: N cicli raggiunti
            }
            else
            {
                target_ = 1;             // torna verso W1
            }
        }
        else // target_ == 1, appena arrivato a W1
        {
            target_ = 2;                 // vai verso W2
        }

        w0 = x_0_objectStart_;
        w1 = (target_ == 1) ? x_0_objectWaypoint1_ : x_0_objectWaypoint2_;
    }

    if(!cycleComplete_)
    {
        x_0_objectCurrent_ = computeDesiredObjectPose(w1, w0);
    }
    else
    {
        // resta fermo sull'ultima posa raggiunta (o gestisci qui la transizione di stato)
        // es: transizione ad un altro CollabSubState, oppure semplicemente non aggiornare x_0_objectCurrent_
    }
}

if(movingToWaypoint2_ &&
   (x_0_objectCurrent_.translation() - x_0_objectWaypoint2_.translation()).norm() < 0.005)
{
    motionFinished_ = true;
}
                     // Calcola l'errore relativo dei due EE
                            sva::PTransformd XL = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
                            sva::PTransformd XR = robots().robot(rightRobotIndex_).bodyPosW(eeName_);

                            sx = leftOffset_ * x_0_objectCurrent_;
                            dx = rightOffset_ * x_0_objectCurrent_;

                            Eigen::Vector3d eWL = XL.translation() - sx.translation();
                            Eigen::Vector3d eWR = XR.translation() - dx.translation();    
                                                   
                     // Proiettalo sull'asse di squeeze
                            Eigen::Vector3d eLLocal = sx.rotation().transpose() * eWL;
                            Eigen::Vector3d eRLocal = dx.rotation().transpose() * eWR;
                            ezLeft  = eLLocal.z();
                            ezRight = eRLocal.z();

                            double leftSpr =  alpha(ezLeft , 0.02);
                            double rightSpr = alpha(ezRight, 0.02);


                            double leftWre =  beta(leftSpr,  0.5, 0.1);
                            double rightWre = beta(rightSpr, 0.5, 0.1);


                           // mc_rtc::log::info("ezLeft = {} | eRLocal = {}",ezLeft,ezRight);
                           // mc_rtc::log::info("leftSpr = {:.2f} | rightSpr = {:.2f}",leftSpr,rightSpr);
                           // mc_rtc::log::info("leftWre = {:.2f} | rightWre = {:.2f}",leftWre,rightWre);

                     leftImpedanceTask_->gains().spring().vec(Eigen::Vector3d(10,10,10),Eigen::Vector3d(100,100,0.1));
                     rightImpedanceTask_->gains().spring().vec(Eigen::Vector3d(10,10,10),Eigen::Vector3d(100,100,0.1));

                     leftImpedanceTask_->gains().damper().vec(Eigen::Vector3d(10,10,10),Eigen::Vector3d(20,20,80));
                     rightImpedanceTask_->gains().damper().vec(Eigen::Vector3d(10,10,10),Eigen::Vector3d(20,20,80));



                     Eigen::VectorXd wLMotion = Eigen::VectorXd::Ones(6);
                     Eigen::VectorXd wRMotion = Eigen::VectorXd::Ones(6);
                     wRMotion(5) = 1; 
                     wLMotion(5) = 1;             
                     rightImpedanceTask_->dimWeight(wRMotion);
                     leftImpedanceTask_->dimWeight(wLMotion);                     
                     
                     gains.massGains = Eigen::Vector6d::Constant(1);   
                     forBothImpedanceTasks([&](auto &task){
                            task->gains().mass().vec(gains.massGains);
                     });  

                     optimize(gains.lambda_desired);

                     leftImpedanceTask_->targetPose(sx);
                     rightImpedanceTask_->targetPose(dx);

              break;
              }
       }
}

bool DualArmControl::run(){
       (this->*currentState_)();
       return mc_control::MCController::run();
}
CONTROLLER_CONSTRUCTOR("DualArmControl", DualArmControl)