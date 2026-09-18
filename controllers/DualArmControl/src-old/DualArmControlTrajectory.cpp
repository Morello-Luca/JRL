#include "DualArmControl.h"

sva::PTransformd DualArmControl::computeDesiredObjectPose(){
       t_norm = std::min(1.0, gains.collaborativeTime_ / gains.totalTrajectoryDuration_);
       if (t_norm >= 1.0) {return x_0_objectWaypoint1_;}
       // Smooth quintic profile
              const double t2 = t_norm * t_norm;
              const double t3 = t2 * t_norm;
              const double s = t3 * (10.0 + t_norm * (-15.0 + 6.0 * t_norm));
              Eigen::Vector3d startPos = x_0_objectStart_.translation();
              Eigen::Vector3d targetPos = x_0_objectWaypoint1_.translation();
              sva::PTransformd desiredPose = x_0_objectCurrent_;
              desiredPose.translation() = startPos + s * (targetPos - startPos);
       // slerp
              Eigen::Quaterniond q_start(x_0_objectStart_.rotation());
              Eigen::Quaterniond q_target(x_0_objectWaypoint1_.rotation());
       // compose
              desiredPose.rotation() = q_start.slerp(s, q_target).toRotationMatrix();
       return desiredPose;
}


