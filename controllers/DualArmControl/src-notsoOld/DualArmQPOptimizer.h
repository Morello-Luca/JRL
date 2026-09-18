#pragma once

#include <Eigen/Core>
#include <eigen-qld/QLD.h>
#include <tuple>

class DualArmQPOptimizer
{
public:
    struct Params {
        double alpha = 1.0;       // Minimization gain
        double beta = 0.5;        // Balancing gain
        double mu = 0.5;          // Friction coefficient
        double F_static = -15.0;  // Static baseline [N]
        double K_demand = 10.0;   // Dynamic scaling factor
    };

    struct InputData {
        Eigen::Matrix3d RL;
        Eigen::Matrix3d RR;
        Eigen::Matrix<double, 12, 1> f_meas;
        Eigen::Matrix<double, 12, 12> Pint;
        Eigen::Vector3d left_local_force;
        Eigen::Vector3d right_local_force;
        double F_demand = 0.0;
    };

    DualArmQPOptimizer();

    // Esegue l'ottimizzazione e restituisce il f_input (Wrench ottimo in World Frame, 12x1)
    bool optimize(const InputData& input, const Params& params, Eigen::Matrix<double, 12, 1>& out_f_input);
    
    // Getter per ispezionare i risultati ottimi estratti
    const Eigen::Vector2d& optimalForces() const { return x_opt_; }

private:
    Eigen::QLD qld_;
    Eigen::Vector2d x_opt_;

    // Metodi privati interni per spezzare i blocchi matematici
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> buildQPProblem(
                                                                const Eigen::Matrix<double, 12, 1>& n_squeeze,
                                                                const Eigen::Matrix<double, 12, 12>& Pint,
                                                                const Eigen::Matrix<double, 12, 2>& S_n, 
                                                                const Eigen::Matrix<double, 12, 1>& w_fixed,
                                                                const Params& params)
        
    std::pair<Eigen::Vector2d, Eigen::Vector2d> computeBounds(
        const Eigen::Vector3d& fL, 
        const Eigen::Vector3d& fR, 
        double F_demand);
};