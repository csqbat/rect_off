#include "rl_lib.h"



EulerAngles ToEulerAngles(Quaternion q) {
    // EulerAngles angles;

    // // roll (x-axis rotation)
    // double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    // double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    // angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // // pitch (y-axis rotation)
    // double sinp = 2 * (q.w * q.y - q.z * q.x);
    // // if (std::abs(sinp) >= 1)
    // //     angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of rangeTrajectory_pos,Trajectory_vel,Trajectory_acc
    // //     angles.pitch = std::asin(sinp);

    // // yaw (z-axis rotation)
    // double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    // double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    // angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    // return angles;
}

void quat2eular_show(const Eigen::Quaterniond &quatf)
{
    struct Quaternion quat_show;
    quat_show.w = quatf.w();
    quat_show.x = quatf.x();
    quat_show.y = quatf.y();
    quat_show.z = quatf.z();
    struct EulerAngles euler_show = ToEulerAngles(quat_show);
    // std::cout<<180/M_PI*euler_show.roll<<"\t"<<180/M_PI*euler_show.pitch<<"\t"<<180/M_PI*euler_show.yaw<<std::endl;
}

Eigen::Vector4f quatMultiplication(const Eigen::Vector4f &q, const Eigen::Vector4f &p) {
   Eigen::Vector4f quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3),
          p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
          p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1),
          p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

Eigen::Vector4f rot2Quaternion(const Eigen::Matrix3d &R){
  Eigen::Vector4f quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4f &q){
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3),
    2 * q(1) * q(2) - 2 * q(0) * q(3),
    2 * q(0) * q(2) + 2 * q(1) * q(3),

    2 * q(0) * q(3) + 2 * q(1) * q(2),
    q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
    2 * q(2) * q(3) - 2 * q(0) * q(1),

    2 * q(1) * q(3) - 2 * q(0) * q(2),
    2 * q(0) * q(1) + 2 * q(2) * q(3),
    q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}