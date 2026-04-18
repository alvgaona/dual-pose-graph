#include "dual_pose_graph/optimizer_g2o.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "dual_pose_graph/graph_g2o.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "dual_pose_graph/utils/conversions.hpp"
#include "dual_pose_graph/utils/csv_logger.hpp"
#include "dual_pose_graph/utils/general_utils.hpp"

OptimizerG2O::OptimizerG2O()
{
  FLAG("STARTING DUAL POSE GRAPH OPTIMIZER");

  main_graph = std::make_shared<GraphG2O>("Main Graph");
  temp_graph = std::make_shared<GraphG2O>("Temp Graph");

  main_graph->init_graph();
  last_odometry_added_.odometry = Eigen::Isometry3d::Identity();
  last_odometry_added_.covariance = Eigen::MatrixXd::Zero(6, 6);
  init_main_graph_ = true;
}

bool OptimizerG2O::generate_odometry_info(
  const OdometryWithCovariance & _new_odometry,
  const OdometryWithCovariance & _last_odometry_added,
  OdometryInfo & _odometry_info)
{
  if (_new_odometry.covariance.isZero()) {
    WARN("Received covariance matrix is zero");
    return false;
  }

  if (odometry_is_relative_) {
    ERROR("RELATIVE ODOMETRY NOT IMPLEMENTED");
    return false;
  }

  _odometry_info.odom_ref = _new_odometry.odometry;
  _odometry_info.increment = _last_odometry_added.odometry.inverse() * _odometry_info.odom_ref;

  if (calculate_odom_covariance_) {
    _odometry_info.covariance_matrix = Eigen::MatrixXd::Zero(6, 6);
    _odometry_info.covariance_matrix(0, 0) =
      std::fabs(_new_odometry.covariance(0, 0) - _last_odometry_added.covariance(0, 0));
    _odometry_info.covariance_matrix(1, 1) =
      std::fabs(_new_odometry.covariance(1, 1) - _last_odometry_added.covariance(1, 1));
    _odometry_info.covariance_matrix(2, 2) =
      std::fabs(_new_odometry.covariance(2, 2) - _last_odometry_added.covariance(2, 2));
    _odometry_info.covariance_matrix(3, 3) = _new_odometry.covariance(3, 3);
    _odometry_info.covariance_matrix(4, 4) = _new_odometry.covariance(4, 4);
    _odometry_info.covariance_matrix(5, 5) = _new_odometry.covariance(5, 5);
  } else {
    _odometry_info.covariance_matrix = _new_odometry.covariance;
  }

  _odometry_info.map_ref = initial_earth_to_map_transform_.inverse() * _odometry_info.odom_ref;

  if (_odometry_info.covariance_matrix.isZero()) {
    WARN("Generated odometry covariance matrix is zero");
    return false;
  }
  return true;
}

bool OptimizerG2O::handle_new_odom(const OdometryWithCovariance & _new_odometry)
{
  OdometryInfo new_odometry_info;
  if (!generate_odometry_info(_new_odometry, last_odometry_added_, new_odometry_info)) {
    return false;
  }

  if (use_dual_graph_ && temp_graph_generated_) {
    if (!check_adding_conditions(new_odometry_info,
        main_graph_odometry_distance_threshold_if_detections_,
        main_graph_odometry_orientation_threshold_))
    {
      return false;
    }
  } else if (!check_adding_conditions(new_odometry_info,
    main_graph_odometry_distance_threshold_,
    main_graph_odometry_orientation_threshold_))
  {
    return false;
  }

  last_odometry_added_.odometry = new_odometry_info.odom_ref;
  last_odometry_added_.covariance = _new_odometry.covariance;

  if (std::isnan(new_odometry_info.odom_ref.translation().x())) {
    WARN("Odometry generated is NaN");
    return false;
  }

  main_graph->add_new_keyframe(
    new_odometry_info.map_ref, new_odometry_info.increment,
    new_odometry_info.covariance_matrix);

  if (!use_dual_graph_) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    detections_since_last_keyframe_.clear();
  }

  if (use_dual_graph_) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (temp_graph_generated_) {
      if (temp_graph->optimize_graph()) {
        // Polymorphic merge: each concrete object node rebuilds its own
        // ObjectDetection from its optimized estimate. No RTTI; adding a new
        // detection kind requires no changes here.
        for (const auto & object : temp_graph->get_object_nodes()) {
          Eigen::MatrixXd cov_matrix = temp_graph->compute_node_covariance(object.second);
          if (cov_matrix.size() == 0) {  // LCOV_EXCL_LINE
            continue;  // LCOV_EXCL_LINE
          }
          auto detection = object.second->build_absolute_detection(cov_matrix);
          if (!detection) {
            continue;
          }
          detection->prepare_measurements(new_odometry_info);
          main_graph->add_new_object_detection(detection.get());
        }
      }
      temp_graph = std::make_shared<GraphG2O>("Temp Graph");
      temp_graph_generated_ = false;
    }
  }

  double chi2_before = main_graph->graph_->chi2();
  auto opt_start = std::chrono::steady_clock::now();
  main_graph->optimize_graph();
  auto opt_end = std::chrono::steady_clock::now();
  double chi2_after = main_graph->graph_->chi2();
  double opt_ms = std::chrono::duration<double, std::milli>(opt_end - opt_start).count();

  if (generate_odom_map_transform_) {
    update_odom_map_transform();
  }

  if (csv_logger_) {
    auto optimized = get_optimized_pose();
    auto corrected = earth_map_transform_ * map_odom_transform_ * last_odometry_added_.odometry;
    csv_logger_->log_keyframe(
      0, 0, last_odometry_added_.odometry,
      static_cast<int>(main_graph->graph_->vertices().size()),
      static_cast<int>(main_graph->graph_->edges().size()),
      chi2_before, chi2_after, opt_ms,
      map_odom_transform_, optimized, corrected);
  }

  return true;
}

bool OptimizerG2O::check_adding_conditions(
  const OdometryInfo & _odometry,
  const double _distance_threshold,
  const double _orientation_threshold)
{
  double translation_distance = _odometry.increment.translation().norm();
  double rotation_distance = Eigen::AngleAxisd(_odometry.increment.rotation()).angle();

  if (translation_distance < _distance_threshold &&
    rotation_distance < _orientation_threshold)
  {
    if (init_main_graph_) {
      init_main_graph_ = false;
      return true;
    }
    return false;
  }
  return true;
}

bool OptimizerG2O::check_adding_new_detection(
  const OdometryWithCovariance & _detection_odometry,
  OdometryInfo & _detection_odometry_info)
{
  std::lock_guard<std::mutex> lock(graph_mutex_);
  if (!temp_graph_generated_) {
    last_detection_odometry_added_ = last_odometry_added_;
  }

  if (!generate_odometry_info(
      _detection_odometry, last_detection_odometry_added_, _detection_odometry_info))
  {
    return false;
  }

  if (!temp_graph_generated_) {
    temp_graph->init_graph(_detection_odometry_info.map_ref);
    temp_graph_generated_ = true;
  } else {
    if (!check_adding_conditions(_detection_odometry_info,
        temp_graph_odometry_distance_threshold_,
        temp_graph_odometry_orientation_threshold_))
    {
      return false;
    }
    temp_graph->add_new_keyframe(
      _detection_odometry_info.map_ref, _detection_odometry_info.increment,
      _detection_odometry_info.covariance_matrix);
  }

  last_detection_odometry_added_.odometry = _detection_odometry_info.odom_ref;
  last_detection_odometry_added_.covariance = _detection_odometry.covariance;
  return true;
}

void OptimizerG2O::handle_new_object_detection(
  ObjectDetection * _object,
  const OdometryInfo & _detection_odometry_info)
{
  if (!_object->prepare_measurements(_detection_odometry_info)) {
    ERROR("Prepare detection ERROR");
    return;
  }

  if (use_dual_graph_) {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    if (!temp_graph_generated_) {return;}
    temp_graph->add_new_object_detection(_object);
  } else {
    std::lock_guard<std::mutex> lock(graph_mutex_);
    std::string id = _object->get_id();
    if (detections_since_last_keyframe_.count(id) > 0) {return;}
    detections_since_last_keyframe_.insert(id);
    main_graph->add_new_object_detection(_object);
  }
}

void OptimizerG2O::set_parameters(const OptimizerG2OParameters & _params)
{
  main_graph_odometry_distance_threshold_ = _params.main_graph_odometry_distance_threshold;
  main_graph_odometry_orientation_threshold_ = _params.main_graph_odometry_orientation_threshold;
  temp_graph_odometry_distance_threshold_ = _params.temp_graph_odometry_distance_threshold;
  temp_graph_odometry_orientation_threshold_ = _params.temp_graph_odometry_orientation_threshold;
  main_graph_odometry_distance_threshold_if_detections_ =
    _params.main_graph_odometry_distance_threshold_if_detections;
  map_odom_security_threshold_ = _params.map_odom_security_threshold;
  odometry_is_relative_ = _params.odometry_is_relative;
  generate_odom_map_transform_ = _params.generate_odom_map_transform;
  fixed_objects_ = _params.fixed_objects;
  initial_earth_to_map_transform_ = _params.earth_to_map_transform;
  map_odom_transform_alpha_ = _params.map_odom_transform_alpha;
  earth_map_transform_ = initial_earth_to_map_transform_;
  calculate_odom_covariance_ = _params.calculate_odom_covariance;
  throttle_detections_ = _params.throttle_detections;
  use_dual_graph_ = _params.use_dual_graph;

  Eigen::MatrixXd earth_to_map_covariance = Eigen::MatrixXd::Identity(6, 6) * 0.0001;
  earth_to_map_covariance(5, 5) = 0.1;

  main_graph->add_new_keyframe(
    initial_earth_to_map_transform_.inverse(),
    initial_earth_to_map_transform_.inverse(),
    earth_to_map_covariance);

  main_graph->set_map_node(main_graph->get_last_odom_node());
  main_graph->set_fixed_objects(fixed_objects_);
}

void OptimizerG2O::update_odom_map_transform()
{
  earth_map_transform_ = get_optimized_map_pose();

  Eigen::Isometry3d new_map_odom_transform =
    earth_map_transform_.inverse() * get_optimized_pose() * last_odometry_added_.odometry.inverse();

  if (map_odom_transform_alpha_ < 1.0) {
    map_odom_transform_ = filter_transform(map_odom_transform_, new_map_odom_transform);
  } else {
    map_odom_transform_ = new_map_odom_transform;
  }
}

Eigen::Isometry3d OptimizerG2O::filter_transform(
  const Eigen::Isometry3d & _last_transform,
  const Eigen::Isometry3d & _new_transform)
{
  Eigen::Vector3d old_translation = _last_transform.translation();
  Eigen::Vector3d new_translation = _new_transform.translation();
  Eigen::Vector3d filtered_translation =
    (1.0 - map_odom_transform_alpha_) * old_translation +
    map_odom_transform_alpha_ * new_translation;

  Eigen::Quaterniond old_rot(_last_transform.rotation());
  Eigen::Quaterniond new_rot(_new_transform.rotation());
  Eigen::Quaterniond filtered_rot = old_rot.slerp(map_odom_transform_alpha_, new_rot);

  Eigen::Isometry3d filtered_transform = Eigen::Isometry3d::Identity();
  filtered_transform.linear() = filtered_rot.toRotationMatrix();
  filtered_transform.translation() = filtered_translation;
  return filtered_transform;
}

Eigen::Isometry3d OptimizerG2O::get_optimized_pose()
{
  return main_graph->get_last_odom_node()->get_pose();
}

Eigen::Isometry3d OptimizerG2O::get_optimized_map_pose()
{
  return main_graph->get_map_node()->get_pose();
}

Eigen::Isometry3d OptimizerG2O::get_map_odom_transform()
{
  return map_odom_transform_;
}

Eigen::Isometry3d OptimizerG2O::get_map_transform()
{
  return earth_map_transform_;
}

bool OptimizerG2O::generate_detection_odometry_info(
  const OdometryWithCovariance & _detection_odometry,
  OdometryInfo & _detection_odometry_info)
{
  return generate_odometry_info(_detection_odometry, last_odometry_added_, _detection_odometry_info);
}
