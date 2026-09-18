#include "ContactForceQP.h"
#include <algorithm>

DualArmQPOptimizer::DualArmQPOptimizer() 
{
    qld_.problem(2, 0, 0); // 2 variabili, 0 uguaglianze, 0 disuguaglianze generiche
}


std::pair<Eigen::Matrix2d, Eigen::Vector2d> DualArmQPOptimizer::buildQPProblem(
    const Eigen::Matrix<double, 12, 1>& n_squeeze,
    const Eigen::Matrix<double, 12, 2>& S_n, 
    const Eigen::Matrix<double, 12, 1>& w_fixed,
    const InputData& input,
    const Params& params) 
{
    // 1. Mapping Vector c: c^T = n_squeeze^T * Pint * S_n
    // c è un vettore 2x1 (trasposto diventa colonna)
    Eigen::Vector2d c = (n_squeeze.transpose() * input.Pint * S_n).transpose();

    // 2. Compute scalar offset lambda0
    double lambda0 = n_squeeze.dot(input.Pint * w_fixed);

    // 3. Estrazione dei coefficienti cL e cR per chiarezza matematica
    const double cL = c(0);
    const double cR = c(1);

    // 4. Costruzione della matrice Hessiana H (2x2)
    Eigen::Matrix2d H;
    H << params.alpha * cL * cL + params.beta,  params.alpha * cL * cR - params.beta,
         params.alpha * cL * cR - params.beta,  params.alpha * cR * cR + params.beta;
    H *= 2.0; // Moltiplicazione per 2 richiesta dal solutore QP

    // 5. Costruzione del vettore gradiente g (2x1)
    Eigen::Vector2d g = 2.0 * params.alpha * lambda0 * c;



    // Estensione 
        Eigen::Matrix<double, 6, 1> S_n_L = S_n.block<6,1>(0, 0);
        Eigen::Matrix<double, 6, 1> S_n_R = S_n.block<6,1>(6, 1);

        Eigen::Matrix<double, 6, 1> w_fixed_L = w_fixed.head<6>();
        Eigen::Matrix<double, 6, 1> w_fixed_R = w_fixed.tail<6>();
   
        Eigen::VectorXd b_L = Eigen::VectorXd::Zero(input.J_L.cols());
        Eigen::VectorXd b_R = Eigen::VectorXd::Zero(input.J_R.cols());     

        Eigen::VectorXd tau_fixed_L = Eigen::VectorXd::Zero(input.J_L.cols());
        Eigen::VectorXd tau_fixed_R = Eigen::VectorXd::Zero(input.J_R.cols());

        double b_L_norm_sq = 0.0;
        double b_R_norm_sq = 0.0;
        double tau_fixed_L_dot_b_L = 0.0;
        double tau_fixed_R_dot_b_R = 0.0;

        if (input.J_L.cols() > 0 && input.J_L.rows() == 6) {
            b_L = input.J_L.transpose() * S_n_L;
            tau_fixed_L = input.J_L.transpose() * w_fixed_L;
            b_L_norm_sq = b_L.squaredNorm();
            tau_fixed_L_dot_b_L = tau_fixed_L.dot(b_L);
        }

        if (input.J_R.cols() > 0 && input.J_R.rows() == 6) {
            b_R = input.J_R.transpose() * S_n_R;
            tau_fixed_R = input.J_R.transpose() * w_fixed_R;
            b_R_norm_sq = b_R.squaredNorm();
            tau_fixed_R_dot_b_R = tau_fixed_R.dot(b_R);
        }

        Eigen::Matrix2d H_tau = Eigen::Matrix2d::Zero();
        H_tau(0, 0) = 2.0 * params.gamma_L * b_L_norm_sq;
        H_tau(1, 1) = 2.0 * params.gamma_R * b_R_norm_sq;

        Eigen::Vector2d g_tau = Eigen::Vector2d::Zero();
        g_tau(0) = 2.0 * params.gamma_L * tau_fixed_L_dot_b_L;
        g_tau(1) = 2.0 * params.gamma_R * tau_fixed_R_dot_b_R;

        H += H_tau;
        g += g_tau;

    return {H, g};
}











std::pair<Eigen::Vector2d, Eigen::Vector2d> DualArmQPOptimizer::computeBounds(
    const InputData& input,
    const Params& params) 
{
    double FtL = input.left_local_force.head<2>().norm();
    double FtR = input.right_local_force.head<2>().norm();

    double Fmin_L = std::min(-std::abs(FtL / params.mu), params.F_static - input.F_demand);
    double Fmin_R = std::min(-std::abs(FtR / params.mu), params.F_static - input.F_demand);

    Eigen::Vector2d xl(-80.0, -80.0);
    Eigen::Vector2d xu(Fmin_L, Fmin_R);
    return {xl, xu};
}









bool DualArmQPOptimizer::optimize(const InputData& input,const Params& params, Eigen::Matrix<double, 12, 1>& out_f_input)
{
    // 1. Calcolo S_n (World Frame Normal Directions)
    Eigen::Vector3d u_L_world = input.RL * Eigen::Vector3d(0.0, 0.0, 1.0);
    Eigen::Vector3d u_R_world = input.RR * Eigen::Vector3d(0.0, 0.0, 1.0);

    Eigen::Matrix<double, 12, 2> S_n = Eigen::Matrix<double, 12, 2>::Zero();
    S_n.block<3,1>(3, 0) = u_L_world; 
    S_n.block<3,1>(9, 1) = u_R_world;

    // 2. Calcolo n_squeeze world (estratto da Pint)
    Eigen::Matrix<double, 12, 1> n_s;
    n_s << 0, 0, 0, 0, 1, 0,
           0, 0, 0, 0, -1, 0;
           
    // Proiezione (SVD o via Pint se Pint = I - Gpinv*G)
    // Per efficienza, n_squeeze può essere calcolato usando Pint direttamente:
    Eigen::Matrix<double, 12, 1> n_squeeze = input.Pint * n_s;
    n_squeeze.stableNormalize();

    // 3. Calcolo di w_fixed 
    double FnL_meas = input.left_local_force.z();
    double FnR_meas = input.right_local_force.z();
    Eigen::Vector2d x_meas(FnL_meas, FnR_meas);
    Eigen::Matrix<double, 12, 1> w_fixed = input.f_meas - S_n * x_meas;

    // 4. CHIAMATA AL METODO ISOLATO PER COSTRUIRE IL QP
    auto [H, g] = buildQPProblem(n_squeeze, S_n, w_fixed, input, params);

    // 5. CHIAMATA AL METODO ISOLATO PER I BOUNDS
    auto [xl, xu] = computeBounds(input,params);

    // 6. Soluzione del problema con QD
    Eigen::MatrixXd Aeq(0, 2), Aineq(0, 2);
    Eigen::VectorXd beq(0), bineq(0);

    bool success = qld_.solve(H, g, Aeq, beq, Aineq, bineq, xl, xu, false, 1e-6);

    if (success) {
        x_opt_ = qld_.result();
    } else {
        x_opt_ = xl; // Fallback di sicurezza
    }

    // 7. Ricostruzione del Wrench totale in output
    out_f_input = S_n * x_opt_ + w_fixed;
    return success;
}