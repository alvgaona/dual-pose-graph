#ifndef DUAL_POSE_GRAPH__OBJECT_DETECTION_TYPES_HPP_
#define DUAL_POSE_GRAPH__OBJECT_DETECTION_TYPES_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <memory>
#include <string>

#include "dual_pose_graph/graph_edge_types.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/utils/conversions.hpp"
#include "dual_pose_graph/utils/general_utils.hpp"

class ObjectDetection
{
public:
  virtual ~ObjectDetection() = default;
  virtual std::string get_id() = 0;
  virtual std::string get_type() const {return "";}
  virtual Eigen::MatrixXd get_covariance_matrix() = 0;
  virtual Eigen::MatrixXd get_information_matrix() = 0;
  virtual bool prepare_measurements(const OdometryInfo & _detection_odometry) = 0;
  virtual GraphNode * create_node() = 0;
  virtual GraphEdge * create_edge(GraphNode * _node, GraphNode * _detection_node) = 0;
};

class ObjectDetectionBase : public ObjectDetection
{
public:
  ObjectDetectionBase(
    const std::string & _id, const Eigen::MatrixXd & _covariance,
    const bool _detections_are_absolute)
  : id_(_id), covariance_matrix_(_covariance), detections_are_absolute_(_detections_are_absolute)
  {
    information_matrix_ = covariance_matrix_.inverse();
  }
  ~ObjectDetectionBase() override {}

  std::string get_id() override {return id_;}
  Eigen::MatrixXd get_covariance_matrix() override {return covariance_matrix_;}
  Eigen::MatrixXd get_information_matrix() override {return information_matrix_;}

protected:
  std::string id_;
  Eigen::MatrixXd covariance_matrix_;
  Eigen::MatrixXd information_matrix_;
  bool detections_are_absolute_;
};

class ObjectDetectionSE3 : public ObjectDetectionBase
{
public:
  ObjectDetectionSE3(
    const std::string & _id, const Eigen::Isometry3d & _pose,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionBase(_id, _covariance, _detections_are_absolute), measured_pose_(_pose) {}

  bool prepare_measurements(const OdometryInfo & _detection_odometry) override
  {
    if (detections_are_absolute_) {
      edge_measurement_ = _detection_odometry.map_ref.inverse() * measured_pose_;
      node_estimation_ = measured_pose_;
    } else {
      edge_measurement_ = measured_pose_;
      node_estimation_ = _detection_odometry.map_ref * measured_pose_;
    }
    return true;
  }

protected:
  Eigen::Isometry3d measured_pose_;
  Eigen::Isometry3d node_estimation_;
  Eigen::Isometry3d edge_measurement_;
};

class ObjectDetectionPoint3D : public ObjectDetectionBase
{
public:
  ObjectDetectionPoint3D(
    const std::string & _id, const Eigen::Vector3d & _position,
    const Eigen::MatrixXd & _covariance, const bool _detections_are_absolute)
  : ObjectDetectionBase(_id, _covariance, _detections_are_absolute), measured_position_(_position)
  {
  }

  bool prepare_measurements(const OdometryInfo & _detection_odometry) override
  {
    if (detections_are_absolute_) {
      edge_measurement_ = _detection_odometry.odom_ref.inverse() * measured_position_;
      node_estimation_ = measured_position_;
    } else {
      edge_measurement_ = measured_position_;
      node_estimation_ = _detection_odometry.map_ref * measured_position_;
    }
    return true;
  }

protected:
  Eigen::Vector3d measured_position_;
  Eigen::Vector3d node_estimation_;
  Eigen::Vector3d edge_measurement_;
};

/// Default SE3 detection used when a user doesn't need a custom subclass.
/// The type_id is carried forward so the node produced can emit matching
/// absolute-detections on temp→main merge.
class SE3ObjectDetection : public ObjectDetectionSE3
{
public:
  SE3ObjectDetection(
    const std::string & _type_id,
    const std::string & _instance_id,
    const Eigen::Isometry3d & _pose,
    const Eigen::MatrixXd & _covariance,
    const bool _detections_are_absolute)
  : ObjectDetectionSE3(_instance_id, _pose, _covariance, _detections_are_absolute),
    type_id_(_type_id) {}

  std::string get_type() const override {return type_id_;}

  GraphNode * create_node() override
  {
    auto * node = new SE3ObjectNode(type_id_, id_, node_estimation_);
    node->set_covariance(covariance_matrix_);
    return node;
  }

  GraphEdge * create_edge(GraphNode * _node, GraphNode * _detection_node) override
  {
    auto * node_se3 = dynamic_cast<GraphNodeSE3 *>(_node);
    auto * detection_node_se3 = dynamic_cast<GraphNodeSE3 *>(_detection_node);
    if (!node_se3 || !detection_node_se3) {
      ERROR("SE3ObjectDetection::create_edge: incompatible node types");
      return nullptr;
    }
    return new GraphEdgeSE3(
      node_se3, detection_node_se3, edge_measurement_, information_matrix_);
  }

protected:
  std::string type_id_;
};

/// Default Point3D detection.
class Point3DObjectDetection : public ObjectDetectionPoint3D
{
public:
  Point3DObjectDetection(
    const std::string & _type_id,
    const std::string & _instance_id,
    const Eigen::Vector3d & _position,
    const Eigen::MatrixXd & _covariance,
    const bool _detections_are_absolute)
  : ObjectDetectionPoint3D(_instance_id, _position, _covariance, _detections_are_absolute),
    type_id_(_type_id) {}

  std::string get_type() const override {return type_id_;}

  GraphNode * create_node() override
  {
    auto * node = new Point3DObjectNode(type_id_, id_, node_estimation_);
    node->set_covariance(covariance_matrix_);
    return node;
  }

  GraphEdge * create_edge(GraphNode * _node, GraphNode * _detection_node) override
  {
    auto * node_se3 = dynamic_cast<GraphNodeSE3 *>(_node);
    auto * detection_node_point3d = dynamic_cast<GraphNodePoint3D *>(_detection_node);
    if (!node_se3 || !detection_node_point3d) {
      ERROR("Point3DObjectDetection::create_edge: incompatible node types");
      return nullptr;
    }
    return new GraphEdgeSE3Point3D(
      node_se3, detection_node_point3d, edge_measurement_, information_matrix_);
  }

protected:
  std::string type_id_;
};

#endif  // DUAL_POSE_GRAPH__OBJECT_DETECTION_TYPES_HPP_
