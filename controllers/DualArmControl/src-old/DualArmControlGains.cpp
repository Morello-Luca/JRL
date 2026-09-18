#include "DualArmControl.h"

void DualArmControl::configureGains(){
       // =========================================================================
       // IMPEDANCE GAINS
       // ========================================================================= 
       gains.massGains = Eigen::Vector6d::Constant(1);                           
       gains.springGains << 10.0, 10.0, 10.0, 100.0, 100.0, 100.0;       
       gains.damperGains =  2.0 * gains.springGains.cwiseProduct(gains.massGains).cwiseSqrt(); 
       gains.wrenchGains << 0.0, 0.0, 0.0,  0.0,  0.0,  1.0;   
       gains.cutoffPeriod = 0.9;
       forBothImpedanceTasks([&](auto &task){
              task->gains().mass().vec(gains.massGains);
              task->gains().spring().vec(gains.springGains);
              task->gains().damper().vec(gains.damperGains);
              task->gains().wrench().vec(gains.wrenchGains);
              task->cutoffPeriod(gains.cutoffPeriod);
       });    
       // =========================================================================
       // SLIDING MODE GAINS
       // =========================================================================
       gains.c = 6.0;                // [1/s]: Sliding surface slope. Higher = faster error convergence, but risks instability.
       gains.phi = 10.0;              // [N]  : Boundary layer thickness. Smooths the switching function to eliminate high-frequency actuator chattering. Higher = smoother but less stiff control.
       gains.K = 0.6;                // [N/s]: Discontinuous control gain. Limits the maximum rate of change allowed 
       gains.Ki = 0.6;               // Guadagno integrale
       gains.soglia_errore = 0.15;   // Attiva l'integrale solo se l'errore è sotto il 10% (0.10)
       // =========================================================================
       // DESORED INTERNAL FORCE 
       // =========================================================================      
       gains.lambda_desired = 0;       
       // =========================================================================
       // PRE CONTACT IMPACT SIGMOID GAINS
       // ========================================================================= 
       gains.k = 1.5;
       gains.t0 = 4.0;
       // =========================================================================
       // OBJECT TRAJECTORY TIME
       // ========================================================================= 
       gains.totalTrajectoryDuration_ = 5.0;


}



void DualArmControl::setImpedanceGains(const Eigen::Vector6d & springLeft, 
                                       const Eigen::Vector6d & springRight, 
                                       const Eigen::Vector6d & wrenchGains,
                                       double dampingRatio) 
{
    // Calcolo automatico e CORRETTO dello smorzamento critico (D = 2 * sqrt(K))
    Eigen::Vector6d damperLeft  = 2.0 * dampingRatio * springLeft.cwiseProduct(gains.massGains).cwiseSqrt();
    Eigen::Vector6d damperRight = 2.0 * dampingRatio * springRight.cwiseProduct(gains.massGains).cwiseSqrt();

    // Assegnazione al braccio sinistro
    leftImpedanceTask_->gains().spring().vec(springLeft);
    leftImpedanceTask_->gains().damper().vec(damperLeft);
    leftImpedanceTask_->gains().wrench().vec(wrenchGains);

    // Assegnazione al braccio destro
    rightImpedanceTask_->gains().spring().vec(springRight);
    rightImpedanceTask_->gains().damper().vec(damperRight);
    rightImpedanceTask_->gains().wrench().vec(wrenchGains);
}

double DualArmControl::smoothGain(double t,
                                  double Kmin,
                                  double Kmax,
                                  double t_conv)
{
    
    constexpr double a = 3.0;   // sposta la tanh verso destra
    double tau = t_conv / (2.0 * a);

    double s = (std::tanh(t / tau - a) + std::tanh(a)) /
               (1.0 + std::tanh(a));

    return Kmin + (Kmax - Kmin) * s;
}