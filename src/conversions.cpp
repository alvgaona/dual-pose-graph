#include "dual_pose_graph/utils/conversions.hpp"

PoseSE3 convert_to_pose_se3(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation)
{
  PoseSE3 pose;
  pose.position = _position;
  pose.orientation = _orientation;
  return pose;
}

PoseSE3 convert_to_pose_se3(const Eigen::Isometry3d & _isometry)
{
  PoseSE3 pose;
  pose.position = _isometry.translation();
  pose.orientation = Eigen::Quaterniond(_isometry.rotation());
  return pose;
}

Eigen::Isometry3d convert_to_isometry_3d(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation)
{
  Eigen::Isometry3d isometry = Eigen::Translation3d(_position) * _orientation;
  return isometry;
}
