#ifndef DUAL_POSE_GRAPH__GRAPH_NODE_TYPES_HPP_
#define DUAL_POSE_GRAPH__GRAPH_NODE_TYPES_HPP_

#include <Eigen/Dense>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/vertex_pointxyz.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <memory>
#include <string>

class ObjectDetection;

class GraphNode
{
public:
  virtual ~GraphNode() = default;
  virtual g2o::HyperGraph::Vertex * get_vertex() = 0;
  virtual void set_fixed() = 0;

  virtual std::string type_id() const {return "";}
  virtual std::string instance_id() const {return "";}

  /// Build an absolute-frame ObjectDetection from this node's current estimate.
  /// Called by the optimizer to merge a temp-graph landmark into the main graph.
  /// Base default returns nullptr. Generic SE3/Point3D object nodes override to
  /// emit a matching detection. Users with custom detection types subclass and
  /// override to produce their own ObjectDetection subclass.
  virtual std::unique_ptr<ObjectDetection> build_absolute_detection(
    const Eigen::MatrixXd & covariance) const;
};

class GraphNodePoint3D : public GraphNode
{
public:
  explicit GraphNodePoint3D(const Eigen::Vector3d & _position)
  {
    vertex_ = new g2o::VertexPointXYZ();
    vertex_->setEstimate(_position);
  }

  g2o::HyperGraph::Vertex * get_vertex() override
  {
    return static_cast<g2o::HyperGraph::Vertex *>(vertex_);
  }

  g2o::VertexPointXYZ * get_vertex_point3d() {return vertex_;}

  void set_fixed() override {vertex_->setFixed(true);}
  Eigen::Vector3d get_position() const {return vertex_->estimate();}
  void set_covariance(const Eigen::MatrixXd & _cov_matrix) {cov_matrix_ = _cov_matrix;}
  Eigen::MatrixXd get_covariance() const {return cov_matrix_;}

protected:
  g2o::VertexPointXYZ * vertex_;
  Eigen::MatrixXd cov_matrix_;
};

class GraphNodeSE3 : public GraphNode
{
public:
  explicit GraphNodeSE3(const Eigen::Isometry3d & _pose)
  {
    vertex_ = new g2o::VertexSE3();
    vertex_->setEstimate(_pose);
  }

  g2o::HyperGraph::Vertex * get_vertex() override
  {
    return static_cast<g2o::HyperGraph::Vertex *>(vertex_);
  }

  g2o::VertexSE3 * get_vertex_se3() {return vertex_;}

  void set_fixed() override {vertex_->setFixed(true);}
  Eigen::Isometry3d get_pose() const {return vertex_->estimate();}
  void set_covariance(const Eigen::MatrixXd & _cov_matrix) {cov_matrix_ = _cov_matrix;}
  Eigen::MatrixXd get_covariance() const {return cov_matrix_;}

protected:
  g2o::VertexSE3 * vertex_;
  Eigen::MatrixXd cov_matrix_;
};

class OdomNode : public GraphNodeSE3
{
public:
  explicit OdomNode(const Eigen::Isometry3d & _pose)
  : GraphNodeSE3(_pose) {}
};

/// Generic SE3 landmark node. Carries a user-supplied type_id so the optimizer
/// can emit matching detections on temp→main merge without any RTTI.
class SE3ObjectNode : public GraphNodeSE3
{
public:
  SE3ObjectNode(
    const std::string & _type_id,
    const std::string & _instance_id,
    const Eigen::Isometry3d & _pose)
  : GraphNodeSE3(_pose), type_id_(_type_id), instance_id_(_instance_id) {}

  std::string type_id() const override {return type_id_;}
  std::string instance_id() const override {return instance_id_;}

  std::unique_ptr<ObjectDetection> build_absolute_detection(
    const Eigen::MatrixXd & covariance) const override;

protected:
  std::string type_id_;
  std::string instance_id_;
};

/// Generic Point3D landmark node.
class Point3DObjectNode : public GraphNodePoint3D
{
public:
  Point3DObjectNode(
    const std::string & _type_id,
    const std::string & _instance_id,
    const Eigen::Vector3d & _position)
  : GraphNodePoint3D(_position), type_id_(_type_id), instance_id_(_instance_id) {}

  std::string type_id() const override {return type_id_;}
  std::string instance_id() const override {return instance_id_;}

  std::unique_ptr<ObjectDetection> build_absolute_detection(
    const Eigen::MatrixXd & covariance) const override;

protected:
  std::string type_id_;
  std::string instance_id_;
};

#endif  // DUAL_POSE_GRAPH__GRAPH_NODE_TYPES_HPP_
