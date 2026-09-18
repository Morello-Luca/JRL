#include "DualArmControl.h"
GraspFrame DualArmControl::buildGraspFrame(){
       // =========================================================================
       // SAVE END EFFECTOR POSITIONS IN WORLD FRAME
       // =========================================================================
              const auto X_left = robots().robot(leftRobotIndex_).bodyPosW(eeName_);
              const auto X_right = robots().robot(rightRobotIndex_).bodyPosW(eeName_);
              
              const Eigen::Vector3d pL = X_left.translation();
              const Eigen::Vector3d pR = X_right.translation();

       // =========================================================================
       // CALCULATE OBJECT FRAME
       // =========================================================================
              const Eigen::Vector3d center = 0.5 * (pL + pR);

       // =========================================================================
       // OFFSETS FROM CENTER TO END EFFECTORS IN WORLD FRAME
       // =========================================================================
              Eigen::Vector3d rL = pL - center;
              Eigen::Vector3d rR = pR - center;

       // =========================================================================
       // BUILD GRASP FRAME 
       // =========================================================================
              Eigen::Vector3d x = (pR - pL).normalized();
              Eigen::Vector3d z(0,0,1);
              if(std::abs(x.dot(z)) > 0.99) z = Eigen::Vector3d(0,1,0);
              Eigen::Vector3d y = z.cross(x).normalized();
              z = x.cross(y).normalized();
              R.col(0) = x;
              R.col(1) = y;
              R.col(2) = z;
              sva::PTransformd X_world_object(R, center);

       // =========================================================================
       // POSA FRAME END EFFECTORS IN OBJECT FRAME
       // =========================================================================
              sva::PTransformd X_object_left = X_left * X_world_object.inv();
              sva::PTransformd X_object_right = X_right * X_world_object.inv();
              
       // =========================================================================
       // GRASP MATRIX FOR FORCE DECOUPLING
       // =========================================================================
              Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
              Eigen::Matrix3d skew_rL = skew(rL);
              Eigen::Matrix3d skew_rR = skew(rR);
              G_.setZero();
              G_.block<3, 3>(0, 0) = I;
              G_.block<3, 3>(0, 6) = I;
              G_.block<3, 3>(3, 0) = skew_rL;
              G_.block<3, 3>(3, 3) = I;
              G_.block<3, 3>(3, 6) = skew_rR;
              G_.block<3, 3>(3, 9) = I;
       // =========================================================================
       // PSEUDOINVERSE
       // =========================================================================
              Gpinv_.setZero();
              Eigen::Matrix<double,6,6> M = G_ * G_.transpose();
              Gpinv_ = G_.transpose() * M.ldlt().solve(Eigen::Matrix<double,6,6>::Identity());
       // =========================================================================
       // DEBUG FOR FRAMES CHECK AND PROJECTION
       // =========================================================================
              /*
              mc_rtc::log::info("pL = {}", X_left.translation().transpose());
              mc_rtc::log::info("pR = {}", X_right.translation().transpose());
              mc_rtc::log::info("center = {}", center.transpose());
              mc_rtc::log::info("rL(world) = {}", rL.transpose());
              mc_rtc::log::info("rR(world) = {}", rR.transpose());
              mc_rtc::log::info("R_object =\n{}", R);
              mc_rtc::log::info("X_object_left.translation = {}", X_object_left.translation().transpose());
              mc_rtc::log::info("X_object_right.translation = {}",X_object_right.translation().transpose());
              mc_rtc::log::info("X_object_left.rotation =\n{}", X_object_left.rotation());
              mc_rtc::log::info("X_object_right.rotation =\n{}",X_object_right.rotation());
              */
       return {X_world_object,X_object_left,X_object_right,X_left,X_right};
}

Eigen::Matrix3d DualArmControl::skew(const Eigen::Vector3d &r){
       Eigen::Matrix3d m;
       m << 0.0, -r.z(), r.y(),
           r.z(), 0.0, -r.x(),
           -r.y(), r.x(), 0.0;
       return m;
}
