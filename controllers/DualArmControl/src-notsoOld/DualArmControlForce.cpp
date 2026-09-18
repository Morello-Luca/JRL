#include "DualArmControl.h"

void DualArmControl::filtering(){
       // =====================================================================================
       // 4. ALPHA-BETA FILTER (Kalman-like tracking for Position and Velocity)
       // -------------------------------------------------------------------------------------
       // This steady-state filter smooths the noisy force measurement (lambdaMeasured_)
       // and estimates its first derivative (lambdaDotFiltered_) without a severe lag.
       // Tuning Guide:
       // - alpha_k (0 to 1): Controls position tracking. Higher = faster response but noisy.
       //                                                 Lower  = smoother but introduces lag.
       // - beta_k  (0 to 1): Controls velocity tracking. Higher = reactive but shaky derivative.
       // =====================================================================================
       smoothFilter_.alpha =   0.10;        smoothFilter_.beta  =   0.005;
       balancedFilter_.alpha = 0.25;        balancedFilter_.beta  = 0.02;
       fastFilter_.alpha =     0.45;        fastFilter_.beta  =     0.10;   

       smoothFilter_.update(lambdaMeasured_, timeStep);
       balancedFilter_.update(lambdaMeasured_, timeStep);
       fastFilter_.update(lambdaMeasured_, timeStep);
       // =====================================================================================
       // 4.1 FIRST-ORDER LOW-PASS FILTER (for comparison only)
       // -------------------------------------------------------------------------------------
       // Classical first-order low-pass filter:
       // y(k) = y(k-1) + alpha * (x(k) - y(k-1))
       // Smaller alpha -> stronger smoothing           Larger alpha  -> faster response
       // =====================================================================================
       lowPass(lambdaFiltered_lowpass_moltosmooth, lambdaMeasured_, 0.05);
       lowPass(lambdaFiltered_lowpass_smooth,      lambdaMeasured_, 0.20);
       lowPass(lambdaFiltered_lowpass_mid,         lambdaMeasured_, 0.30);
       lowPass(lambdaFiltered_lowpass_high,        lambdaMeasured_, 0.70);
       double tau = 1.0 / (2.0 * M_PI * gains.cutoffPeriod);
       double alpha_lpcut = timeStep / (tau + timeStep);
       lowPass(lambdaFiltered_lowpass_cut, lambdaMeasured_, alpha_lpcut);
}




void DualArmControl::currentInternalForce(){
       // Rotation Matrix
              auto RL = robots().robot(leftRobotIndex_).bodyPosW(eeName_).rotation();
              auto RR = robots().robot(rightRobotIndex_).bodyPosW(eeName_).rotation();
       // World Frame FORCES
              Eigen::Matrix<double, 6, 1> fL_world;
              Eigen::Matrix<double, 6, 1> fR_world;
              Eigen::Matrix<double, 12, 1> f_meas;
       // Contact Force Measure
              sva::ForceVecd WL = leftImpedanceTask_->measuredWrench();
              sva::ForceVecd WR = rightImpedanceTask_->measuredWrench();
              
              fL_world.head<3>() = RL * WL.couple();
              fL_world.tail<3>() = RL * WL.force();
              fR_world.head<3>() = RR * WR.couple();
              fR_world.tail<3>() = RR * WR.force();
       // Group into one vector
              f_meas.segment<6>(0) = fL_world;
              f_meas.segment<6>(6) = fR_world;
       // Null-Space Projection Matrix
              Eigen::Matrix<double, 12, 12> Pint = Eigen::Matrix<double, 12, 12>::Identity() - Gpinv_ * G_;
              Eigen::Matrix<double, 12, 1> internalForce = Pint * f_meas;

              n_s << 0, 0, 0, 0, 1, 0,  //  Squeeze direction
                     0, 0, 0, 0, -1, 0;
       // Null Space projection of squeeze
              Eigen::JacobiSVD<Eigen::MatrixXd> svd(G_, Eigen::ComputeFullV);
              int rank = svd.rank();
              Eigen::MatrixXd N = svd.matrixV().rightCols(G_.cols() - rank);
              n_squeeze = N * (N.transpose() * n_s);
              n_squeeze.stableNormalize();
       lambdaMeasured_ = n_squeeze.transpose() * internalForce;
       
       filtering();
       meas = lambdaFiltered_lowpass_cut;
}


// theorical peak 10/sqrt3 ≈ 5.7735.pre gain
double DualArmControl::DemandForces(double K) const{
    const double t2 = t_norm * t_norm;
    const double t3 = t2 * t_norm;
    const double t4 = t3 * t_norm;
    //return K * std::abs(60.0 * t_norm - 180.0 * t2 + 120.0 * t3);
    return K * std::abs(30.0 * t2 - 60.0 * t3 + 30.0 * t4);
}



void DualArmControl::lowPassWrench(sva::ForceVecd & filtered, const sva::ForceVecd & raw, double alpha) {
    // Apllica la formula y(k) = y(k-1) + alpha * (x(k) - y(k-1)) sui vettori 3D
    filtered.couple() += alpha * (raw.couple() - filtered.couple());
    filtered.force()  += alpha * (raw.force()  - filtered.force());
}


void DualArmControl::optimize() {
    // 0. Lettura sensori e filtraggio
    double tau = 1.0 / (2.0 * M_PI * gains.cutoffPeriod);
    double alpha_lpcut = timeStep / (tau + timeStep);
    lowPassWrench(WL_filtered_, leftImpedanceTask_->measuredWrench(), alpha_lpcut);
    lowPassWrench(WR_filtered_, rightImpedanceTask_->measuredWrench(), alpha_lpcut);

    // 1. Definiamo i parametri al volo (estratti, ad esempio, dai tuoi oggetti "gains")
    DualArmQPOptimizer::Params qpParams;
    qpParams.alpha    = 1.0; 
    qpParams.beta     = 0.5;
    qpParams.mu       = 0.5;             // Preso dinamicamente da mc_rtc o dal tuo robot
    qpParams.F_static = -15;       // Preso dinamicamente
    qpParams.K_demand = 10.0;

    // 2. Preparazione Dati di Input per il QP
    DualArmQPOptimizer::InputData qpInput;
    qpInput.RL = robots().robot(leftRobotIndex_).bodyPosW(eeName_).rotation();
    qpInput.RR = robots().robot(rightRobotIndex_).bodyPosW(eeName_).rotation();
    
    qpInput.f_meas.segment<3>(0) = qpInput.RL * WL_filtered_.couple();
    qpInput.f_meas.segment<3>(3) = qpInput.RL * WL_filtered_.force();
    qpInput.f_meas.segment<3>(6) = qpInput.RR * WR_filtered_.couple();
    qpInput.f_meas.segment<3>(9) = qpInput.RR * WR_filtered_.force();

    qpInput.Pint = Eigen::Matrix<double, 12, 12>::Identity() - Gpinv_ * G_;
    qpInput.left_local_force = WL_filtered_.force();
    qpInput.right_local_force = WR_filtered_.force();
    qpInput.F_demand = DemandForces(10.0); // K_demand = 10.0

    // 3. Chiamata all'Ottimizzatore Isolato
    Eigen::Matrix<double, 12, 1> f_input;
    bool QP_success = qpOptimizer_.optimize(qpInput, f_input);

    if (!QP_success) {
        mc_rtc::log::error("QLD failed! Fallback applicato internamente.");
    }

    // 4. Estrazione dei Wrench risultanti e invio ai Task
    Eigen::Matrix<double, 6, 1> wL_world = f_input.segment<6>(0);
    Eigen::Matrix<double, 6, 1> wR_world = f_input.segment<6>(6);

    // Trasformazione in coordinate locali ed esecuzione
    cmdLeft.couple() = qpInput.RL.transpose() * wL_world.head<3>();
    cmdLeft.force()  = qpInput.RL.transpose() * wL_world.tail<3>();

    cmdRight.couple() = qpInput.RR.transpose() * wR_world.head<3>();
    cmdRight.force()  = qpInput.RR.transpose() * wR_world.tail<3>();

    leftImpedanceTask_->targetWrench(cmdLeft);
    rightImpedanceTask_->targetWrench(cmdRight);
}