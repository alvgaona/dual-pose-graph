#ifndef DUAL_POSE_GRAPH__GRAPH_EDGE_TYPES_HPP_
#define DUAL_POSE_GRAPH__GRAPH_EDGE_TYPES_HPP_

#include <Eigen/Dense>
#include <g2o/core/hyper_graph.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d/types_slam3d.h>

#include "dual_pose_graph/graph_node_types.hpp"

class GraphEdge
{
public:
  virtual ~GraphEdge() = default;  // LCOV_EXCL_LINE
  virtual g2o::HyperGraph::Edge * get_edge() = 0;
};

class GraphEdgeSE3 : public GraphEdge
{
public:
  GraphEdgeSE3(
    GraphNodeSE3 * _node1,
    GraphNodeSE3 * _node2,
    const Eigen::Isometry3d & _relative_pose,
    const Eigen::MatrixXd & _information_matrix)
  {
    edge_ = new g2o::EdgeSE3();
    edge_->setParameterId(0, 0);
    edge_->setMeasurement(_relative_pose);
    edge_->setInformation(_information_matrix);
    edge_->vertices()[0] = _node1->get_vertex_se3();
    edge_->vertices()[1] = _node2->get_vertex_se3();
  }

  g2o::HyperGraph::Edge * get_edge() override
  {
    return static_cast<g2o::HyperGraph::Edge *>(edge_);
  }

  g2o::EdgeSE3 * get_edge_se3() {return edge_;}

protected:
  g2o::EdgeSE3 * edge_ = nullptr;
};

class GraphEdgeSE3Point3D : public GraphEdge
{
public:
  GraphEdgeSE3Point3D(
    GraphNodeSE3 * _node1,
    GraphNodePoint3D * _node2,
    const Eigen::Vector3d & _relative_position,
    const Eigen::MatrixXd & _information_matrix)
  {
    edge_ = new g2o::EdgeSE3PointXYZ();
    edge_->setParameterId(0, 0);
    edge_->setMeasurement(_relative_position);
    edge_->setInformation(_information_matrix);
    edge_->vertices()[0] = _node1->get_vertex_se3();
    edge_->vertices()[1] = _node2->get_vertex_point3d();
  }

  g2o::HyperGraph::Edge * get_edge() override
  {
    return static_cast<g2o::HyperGraph::Edge *>(edge_);
  }

  g2o::EdgeSE3PointXYZ * get_edge_se3_point3d() {return edge_;}

protected:
  g2o::EdgeSE3PointXYZ * edge_ = nullptr;
};

class OdomEdge : public GraphEdgeSE3
{
public:
  OdomEdge(
    GraphNodeSE3 * _node1,
    GraphNodeSE3 * _node2,
    const Eigen::Isometry3d & _pose,
    const Eigen::MatrixXd & _information_matrix)
  : GraphEdgeSE3(_node1, _node2, _pose, _information_matrix) {}
};

#endif  // DUAL_POSE_GRAPH__GRAPH_EDGE_TYPES_HPP_
