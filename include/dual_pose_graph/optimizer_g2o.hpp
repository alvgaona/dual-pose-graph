#ifndef DUAL_POSE_GRAPH__OPTIMIZER_G2O_HPP_
#define DUAL_POSE_GRAPH__OPTIMIZER_G2O_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <mutex>
#include <unordered_set>
#include <vector>

#include "dual_pose_graph/graph_g2o.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "dual_pose_graph/utils/conversions.hpp"

struct OptimizerG2OParameters
{
  double main_graph_odometry_distance_threshold = 1.0;
  double main_graph_odometry_orientation_threshold = 1.0;
  double temp_graph_odometry_distance_threshold = 0.1;
  double temp_graph_odometry_orientation_threshold = 0.1;
  double main_graph_odometry_distance_threshold_if_detections = 0.5;
  double map_odom_security_threshold = 2.0;
  double map_odom_transform_alpha = 1.0;
  bool odometry_is_relative = false;
  bool generate_odom_map_transform = false;
  bool calculate_odom_covariance = false;
  bool throttle_detections = true;
  bool use_dual_graph = true;
  Eigen::Isometry3d earth_to_map_transform = Eigen::Isometry3d::Identity();
  std::vector<FixedObject> fixed_objects;
};

class CsvLogger;

class OptimizerG2O
{
public:
  OptimizerG2O();
  ~OptimizerG2O() {}

  void set_csv_logger(CsvLogger * logger) {csv_logger_ = logger;}
  void set_parameters(const OptimizerG2OParameters & _params);

  bool handle_new_odom(const OdometryWithCovariance & _new_odometry);
  void handle_new_object_detection(
    ObjectDetection * _object,
    const OdometryInfo & _detection_odometry_info);
  bool check_adding_new_detection(
    const OdometryWithCovariance & _detection_odometry,
    OdometryInfo & _detection_odometry_info);

  bool generate_odometry_info(
    const OdometryWithCovariance & _new_odometry,
    const OdometryWithCovariance & _last_odometry_added,
    OdometryInfo & _odometry_info);
  bool generate_detection_odometry_info(
    const OdometryWithCovariance & _detection_odometry,
    OdometryInfo & _detection_odometry_info);
  bool check_adding_conditions(
    const OdometryInfo & _odometry,
    double _distance_threshold,
    double _orientation_threshold = 999.0);
  void update_odom_map_transform();
  Eigen::Isometry3d filter_transform(
    const Eigen::Isometry3d & _last_transform,
    const Eigen::Isometry3d & _new_transform);

  Eigen::Isometry3d get_optimized_pose();
  Eigen::Isometry3d get_optimized_map_pose();
  Eigen::Isometry3d get_map_odom_transform();
  Eigen::Isometry3d get_map_transform();

  std::shared_ptr<GraphG2O> main_graph;
  std::shared_ptr<GraphG2O> temp_graph;
  std::mutex graph_mutex_;

private:
  bool temp_graph_generated_ = false;
  bool init_main_graph_ = true;
  double map_odom_transform_alpha_ = 1.0;
  OdometryWithCovariance last_odometry_added_;
  OdometryWithCovariance last_detection_odometry_added_;
  Eigen::Isometry3d map_odom_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d earth_map_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_earth_to_map_transform_ = Eigen::Isometry3d::Identity();

  double main_graph_odometry_distance_threshold_ = 1.0;
  double main_graph_odometry_orientation_threshold_ = 1.0;
  double temp_graph_odometry_distance_threshold_ = 0.1;
  double temp_graph_odometry_orientation_threshold_ = 0.1;
  double main_graph_odometry_distance_threshold_if_detections_ = 0.5;
  double map_odom_security_threshold_ = 2.0;
  bool odometry_is_relative_ = false;
  bool generate_odom_map_transform_ = false;
  bool calculate_odom_covariance_ = false;
  bool throttle_detections_ = true;
  bool use_dual_graph_ = true;
  std::vector<FixedObject> fixed_objects_;
  std::unordered_set<std::string> detections_since_last_keyframe_;
  CsvLogger * csv_logger_ = nullptr;
};

#endif  // DUAL_POSE_GRAPH__OPTIMIZER_G2O_HPP_
