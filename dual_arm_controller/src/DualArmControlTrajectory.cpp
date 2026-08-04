#include "DualArmControl.h"


sva::PTransformd DualArmControl::computeDesiredObjectPose(sva::PTransformd Desired, sva::PTransformd Start ){
       t_norm = std::min(1.0, gains.collaborativeTime_ / gains.totalTrajectoryDuration_);
       if (t_norm >= 1.0) {return Desired;}
       // Smooth quintic profile
              const double t2 = t_norm * t_norm;
              const double t3 = t2 * t_norm;
              const double s = t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));
              Eigen::Vector3d startPos = Start.translation();
              Eigen::Vector3d targetPos = Desired.translation();
              sva::PTransformd desiredPose = x_0_objectCurrent_;
              desiredPose.translation() = startPos + s * (targetPos - startPos);
       // slerp
              Eigen::Quaterniond q_start(Start.rotation());
              Eigen::Quaterniond q_target(Desired.rotation());
       // compose
              desiredPose.rotation() = q_start.slerp(s, q_target).toRotationMatrix();
       return desiredPose;
}

double DualArmControl::alpha(double e, double L,double k)const{
       double z = std::max(0.0, std::min(e / L, 1.0));
       return 0.5 * (1.0 - std::cos(M_PI * std::pow(z, k)));
}

double DualArmControl::beta(double spring, double initial_value, double final_value){
       return initial_value + spring * (final_value - initial_value);
}




    

    