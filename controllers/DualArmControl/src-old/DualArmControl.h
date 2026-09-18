#pragma once

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/ImpedanceTask.h>
#include <mc_solver/BoundedSpeedConstr.h>
#include <RBDyn/FD.h>
#include <Tasks/QPSolver.h>



struct GraspFrame
{
  sva::PTransformd object;
  sva::PTransformd leftOffset;
  sva::PTransformd rightOffset;

  sva::PTransformd leftEE;
  sva::PTransformd rightEE;
};

enum class FSMState : int
{
  IDLE = 0,
  INDEPENDENT = 1,
  IMPACT = 2,
  COLLABORATIVE = 3
};

enum class CollabSubState {
        BUILD_GRASP,
        APPROACH_RAMP,
        TRAJECTORY,
        COOPERATIVE_MOTION
    };


struct ImpactGains{
    double k = 2.0;
    double t0 = 4.0;
    double maxForce = -9.0;
    double alpha(double stateTimer) const{
        return 1.0 / (1.0 + std::exp(-k * (stateTimer - t0)));
    }
    double contactForce(double stateTimer) const{
        return maxForce * alpha(stateTimer);
    }
};
struct ImpedanceGains{
    double cutoffPeriod = 0.00;
    Eigen::Vector6d wrenchGains       = Eigen::Vector6d::Zero();

    Eigen::Vector6d massGains         = Eigen::Vector6d::Zero(); // Virtual Mass (kg / kg*m^2)
    Eigen::Vector6d springGains       = Eigen::Vector6d::Zero();    // Virtual Stiffness (N/m)
    Eigen::Vector6d damperGains       = Eigen::Vector6d::Zero();
    
    Eigen::Vector6d massImpactGains   = Eigen::Vector6d::Zero();   // Virtual Mass (kg / kg*m^2)
    Eigen::Vector6d springImpactGains = Eigen::Vector6d::Zero(); // Virtual Stiffness (N/m)
    Eigen::Vector6d damperImpactGains = Eigen::Vector6d::Zero();

    Eigen::Vector6d massCollaborativeGains   = Eigen::Vector6d::Zero();   // Virtual Mass (kg / kg*m^2)
    Eigen::Vector6d springCollaborativeGains = Eigen::Vector6d::Zero(); // Virtual Stiffness (N/m)
    Eigen::Vector6d damperCollaborativeGains = Eigen::Vector6d::Zero();
};
struct TrajectoryGains{
  double collaborativeTime_ = 0.0;
  double totalTrajectoryDuration_ = 5.0;
};
struct ForceGains{
  // Guadagni sliding mode
  double c = 0.0;   // [1/s]
  double phi = 0.0; // [N]
  double K = 0.0;   // [N/s]
  double Ki = 0.0;  // Guadagno integrale
  
  double soglia_errore = 0.0;
  double lambda_desired = 0;
};

struct AlphaBetaFilter
{
    double alpha;
    double beta;
    double state = 0.0;
    double derivative = 0.0;
    void update(double measurement, double dt)
    {
        double predPos = state + dt * derivative;
        double predVel = derivative;
        double innovation = measurement - predPos;
        state = predPos + alpha * innovation;
        derivative = predVel + beta / dt * innovation;
    }
};


inline void lowPass(double &y, double x, double alpha)
{
    y += alpha * (x - y);
}



struct Gains : public ImpedanceGains,
               public ImpactGains,
               public ForceGains,
               public TrajectoryGains
{
};

//struct GainsList
//{
//   ImpedanceGains impedance;
//    ImpactGains impact;
//};




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

  void entryStateImpact();
  void stateImpact();


  void entryStateCollaborative();
  void stateCollaborative();




 void ImpactAware();

  void offsetCalc();

  void registerCollaborativeLogs();
  void updateTelemetry(const sva::PTransformd &objectDesired);

  void configureGains();

 void currentInternalForce();

  void filtering();
  sva::PTransformd computeDesiredObjectPose();

  GraspFrame buildGraspFrame();
  // =========================================================================
  // Apply the same operation to both arms
  // =========================================================================

  template <typename F>
  void forBothImpedanceTasks(F &&f)
  {
    f(leftImpedanceTask_);
    f(rightImpedanceTask_);
  }


Eigen::Vector3d obj;

  // 2. ==========================================================
  // PRE-ALLOCATED TASK POINTERS (Fixed naming to match .cpp)
  // ==========================================================
  std::shared_ptr<mc_tasks::PostureTask> rightPostureTask_;
  std::shared_ptr<mc_tasks::PostureTask> leftPostureTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> leftEeTask_;
  std::shared_ptr<mc_tasks::EndEffectorTask> rightEeTask_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> leftImpedanceTask_;
  std::shared_ptr<mc_tasks::force::ImpedanceTask> rightImpedanceTask_;

  // 3. ==========================================================
  // STATE MEMBER VARIABLES (Fixed lowercase 'x' to match .cpp)
  // ==========================================================

  sva::PTransformd x_0_objectCurrent_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};
  sva::PTransformd leftOffset_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};
  sva::PTransformd rightOffset_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};

  // Gestione Multi-Waypoint Collaborativi
  int collaborativeWaypointIndex_ = 0;   // Traccia il waypoint corrente (0 = Primo, 1 = Secondo)
  sva::PTransformd x_0_objectWaypoint1_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()}; // Vecchio x_0_objectFinalWaypoint_
  sva::PTransformd x_0_objectWaypoint2_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};

  // 4. ==========================================================
  // CONTROLLER CONSTANTS AND UTILITIES
  // ==========================================================
  unsigned int leftRobotIndex_ = 0;
  unsigned int rightRobotIndex_ = 0;

  const std::string eeName_ = "link7";
  double stateTimer_ = 0.0;

  double iDist = 0.05;
  double sDist = 0.01;

  sva::PTransformd x_0_objectStart_{Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero()};

  // 5. ==========================================================
  // GAINS 
  // ==========================================================
  Gains gains;
  AlphaBetaFilter smoothFilter_;
  AlphaBetaFilter balancedFilter_;
  AlphaBetaFilter fastFilter_;

  // 6. ==========================================================
  // logging
  // ==========================================================
  FSMState currentFsmState_ = FSMState::IDLE;

  // 7. ==========================================================
  // Internal Stress
  // ==========================================================
  static Eigen::Matrix3d skew(const Eigen::Vector3d &r);

  Eigen::Matrix<double, 6, 12> G_;
  Eigen::Matrix<double, 12, 6> Gpinv_;
  void updateContactForces();
  double lambdaMeasured_ = 0.0;
  double lambdaErrorIntegral_ = 0.0;
  double lambdaFiltered_ = 0.0;
  double lambdaDotFiltered_ = 0.0;
  
  double lambdaFiltered_lowpass_moltosmooth = 0.0;
  double lambdaFiltered_lowpass_smooth = 0.0;
  double lambdaFiltered_lowpass_mid = 0.0;
  double lambdaFiltered_lowpass_high = 0.0;
  double lambdaFiltered_lowpass_cut = 0.0;

  sva::ForceVecd targetWrench;


  double error_integral = 0.0;   // Accumulatore globale o di classe
  double max_integral = 0.0;     // Ulteriore clamp di sicurezza (Anti-windup classico)
  double errore_normalizzato = 0.0;
  double lambdaCommand_ = 0.0;
   // Attiva l'integrale solo se l'errore è sotto il 10% (0.10)

  // =====================================================================================
  // 4. MULTI-CONFIGURATION ALPHA-BETA FILTER (For Tuning & Plotting)
  // -------------------------------------------------------------------------------------
  // Running three different filter configurations in parallel to compare performance.
  // =====================================================================================
  // Filtro 1: SMOOTH (Molto filtrato, lento)
  double lambdaFiltered_smooth = 0.0;
  double lambdaDotFiltered_smooth = 0.0;

  // Filtro 2: BALANCED (Il tuo attuale, bilanciato)
  double lambdaFiltered_balanced = 0.0;
  double lambdaDotFiltered_balanced = 0.0;

  // Filtro 3: FAST (Molto reattivo, più rumoroso)
  double lambdaFiltered_fast = 0.0;
  double lambdaDotFiltered_fast = 0.0;
// Vettore di appoggio per il logging unificato delle 3 stime
  Eigen::Vector3d lambdaFiltersCompare_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d lambdadotFiltersCompare_ = Eigen::Vector3d::Zero();
double error_internalForce;

sva::ForceVecd targetWrench_{
    Eigen::Vector3d::Zero(), 
    Eigen::Vector3d(0.0, 0.0, 0.0)
};


       sva::ForceVecd WL;
       sva::ForceVecd WR;


template<typename T>
inline void addLog(const std::string &name, const T &var)
{
    logger().addLogEntry(name, [&var]() { return var; });
}

              sva::PTransformd X_0_leftTarget = sva::PTransformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
              sva::PTransformd X_0_rightTarget = sva::PTransformd(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
      
 Eigen::Matrix3d R;

CollabSubState collabSubState_;

sva::ForceVecd cmdLeft{
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero()
};

sva::ForceVecd cmdRight{
    Eigen::Vector3d::Zero(),
    Eigen::Vector3d::Zero()
};


std::unique_ptr<mc_solver::BoundedSpeedConstr> speedLeftConstr_;
std::unique_ptr<mc_solver::BoundedSpeedConstr> speedRightConstr_;
// constraints
std::pair<
    std::unique_ptr<mc_solver::BoundedSpeedConstr>,
    std::unique_ptr<mc_solver::BoundedSpeedConstr>
> boundSpeed(const Eigen::VectorXd & spd);

void setImpedanceGains(const Eigen::Vector6d & springLeft, 
                         const Eigen::Vector6d & springRight, 
                         const Eigen::Vector6d & wrenchGains,
                        double dampingRatio = 1.0);


Eigen::MatrixXd dof = Eigen::MatrixXd::Identity(6,6);

double smoothGain(double t, double Kmin, double Kmax, double t_conv);



double rampTime;
double lambdaStart;


double computeReflectedMassZ(unsigned int robotIndex,
                             const std::string & eeName);


double meas;
Eigen::Matrix<double, 12, 1> n_s;
Eigen::Matrix<double, 12, 1> n_squeeze;


double t_norm = 0;

Eigen::VectorXd spd = Eigen::VectorXd::Zero(6);


//==================================
// optimal control loop

double DemandForces(double K) const;
double demandforce=0;

};