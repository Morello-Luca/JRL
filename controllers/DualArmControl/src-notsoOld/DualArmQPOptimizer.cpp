#include "DualArmQPOptimizer.h"
#include <algorithm>

DualArmQPOptimizer::DualArmQPOptimizer(const Params& params) 
    : params_(params) 
{
    qld_.problem(2, 0, 0); // 2 variabili, 0 uguaglianze, 0 disuguaglianze generiche
}


std::pair<Eigen::Matrix2d, Eigen::Vector2d> DualArmQPOptimizer::buildQPProblem(
    const Eigen::Matrix<double, 12, 1>& n_squeeze,
    const Eigen::Matrix<double, 12, 12>& Pint,
    const Eigen::Matrix<double, 12, 2>& S_n, 
    const Eigen::Matrix<double, 12, 1>& w_fixed,
    const Params& params) 
{
    // 1. Mapping Vector c: c^T = n_squeeze^T * Pint * S_n
    // c è un vettore 2x1 (trasposto diventa colonna)
    Eigen::Vector2d c = (n_squeeze.transpose() * Pint * S_n).transpose();

    // 2. Compute scalar offset lambda0
    double lambda0 = n_squeeze.dot(Pint * w_fixed);

    // 3. Estrazione dei coefficienti cL e cR per chiarezza matematica
    const double cL = c(0);
    const double cR = c(1);

    // 4. Costruzione della matrice Hessiana H (2x2)
    Eigen::Matrix2d H;
    H << params_.alpha * cL * cL + params_.beta,  params_.alpha * cL * cR - params_.beta,
         params_.alpha * cL * cR - params_.beta,  params_.alpha * cR * cR + params_.beta;
    H *= 2.0; // Moltiplicazione per 2 richiesta dal solutore QP

    // 5. Costruzione del vettore gradiente g (2x1)
    Eigen::Vector2d g = 2.0 * params_.alpha * lambda0 * c;

    return {H, g};
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> DualArmQPOptimizer::computeBounds(
    const Eigen::Vector3d& fL, 
    const Eigen::Vector3d& fR, 
    double F_demand,
    const Params& params) 
{
    double FtL = fL.head<2>().norm();
    double FtR = fR.head<2>().norm();

    double Fmin_L = std::min(-std::abs(FtL / params.mu), params.F_static - F_demand);
    double Fmin_R = std::min(-std::abs(FtR / params.mu), params.F_static - F_demand);

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
    auto [H, g] = buildQPProblem(n_squeeze, input.Pint, S_n, w_fixed, params);

    // 5. CHIAMATA AL METODO ISOLATO PER I BOUNDS
    auto [xl, xu] = computeBounds(input.left_local_force, input.right_local_force, input.F_demand);

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