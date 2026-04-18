#ifndef DUAL_POSE_GRAPH__UTILS__CONVERSIONS_HPP_
#define DUAL_POSE_GRAPH__UTILS__CONVERSIONS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <string>

struct PoseSE3
{
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
};

struct IsometryWithID
{
  std::string id;
  Eigen::Isometry3d isometry;
};

struct OdometryInfo
{
  Eigen::Isometry3d increment;
  Eigen::Isometry3d odom_ref;
  Eigen::Isometry3d map_ref;
  Eigen::MatrixXd covariance_matrix;
};

struct OdometryWithCovariance
{
  Eigen::Isometry3d odometry;
  Eigen::MatrixXd covariance;
};

struct FixedObject
{
  enum class Kind { SE3, Point3D };
  std::string id;
  Kind kind = Kind::SE3;
  Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
};

PoseSE3 convert_to_pose_se3(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation);
PoseSE3 convert_to_pose_se3(const Eigen::Isometry3d & _isometry);

Eigen::Isometry3d convert_to_isometry_3d(
  const Eigen::Vector3d & _position,
  const Eigen::Quaterniond & _orientation);

#endif  // DUAL_POSE_GRAPH__UTILS__CONVERSIONS_HPP_
