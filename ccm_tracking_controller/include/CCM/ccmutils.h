#ifndef CCM_UTILS_H
#define CCM_UTILS_H

#include <Eigen/Dense>
#include <ros/ros.h>
#include <cmath>

namespace ccmutils
{
  void quat2rotM(const Eigen::Vector4d& q, Eigen::Matrix3d &R);
  void rotM2quat(Eigen::Vector4d& q, const Eigen::Matrix3d &R);
  void R2euler_123(const Eigen::Matrix3d &R, Eigen::Vector3d &ang);
  void Euler2R_123(const Eigen::Vector3d &ang, Eigen::Matrix3d &R);
  void R_wb(const Eigen::Vector3d &ang, Eigen::Matrix3d &R);
  void R_eul(const Eigen::Vector3d &ang, Eigen::Matrix3d &R);

}


#endif /* CCM_UTILS_H */
