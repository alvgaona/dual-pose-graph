#include "dual_pose_graph/graph_g2o.hpp"

#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/parameter.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "dual_pose_graph/graph_edge_types.hpp"
#include "dual_pose_graph/graph_node_types.hpp"
#include "dual_pose_graph/object_detection_types.hpp"
#include "dual_pose_graph/utils/conversions.hpp"
#include "dual_pose_graph/utils/general_utils.hpp"

G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)

GraphG2O::GraphG2O(const std::string & _name)
: name_(_name)
{
  FLAG_GRAPH("Create " << name_);

  graph_ = std::make_shared<g2o::SparseOptimizer>();
  const std::string solver_type = "lm_var_cholmod";
  g2o::OptimizationAlgorithmFactory * solver_factory =
    g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm * solver = solver_factory->construct(solver_type, solver_property);
  graph_->setAlgorithm(solver);

  g2o::ParameterSE3Offset * sensor_offset = new g2o::ParameterSE3Offset;
  sensor_offset->setOffset(Eigen::Isometry3d::Identity());
  sensor_offset->setId(0);
  graph_->addParameter(sensor_offset);

  if (!graph_->solver()) {  // LCOV_EXCL_START
    std::cerr << "error: failed to allocate g2o solver" << std::endl;
    solver_factory->listSolvers(std::cerr);
    return;
  }  // LCOV_EXCL_STOP
}

void GraphG2O::set_map_node(OdomNode * _map_node)
{
  map_node_ = _map_node;
}

void GraphG2O::set_fixed_objects(const std::vector<FixedObject> & _fixed_objects)
{
  for (const auto & object : _fixed_objects) {
    GraphNode * fixed_node = nullptr;

    switch (object.kind) {
      case FixedObject::Kind::SE3:
        fixed_node = new SE3ObjectNode(object.id, object.id, object.isometry);
        break;
      case FixedObject::Kind::Point3D:
        fixed_node = new Point3DObjectNode(object.id, object.id, object.isometry.translation());
        break;
    }

    if (fixed_node) {
      fixed_node->set_fixed();
      add_node(*fixed_node);
      obj_id2node_[object.id] = fixed_node;
      FLAG_GRAPH("Added fixed object ID: " << object.id);
    } else {
      FLAG_GRAPH("Warning: Unrecognized fixed object kind for ID: " << object.id);
    }
  }
}

void GraphG2O::init_graph(const Eigen::Isometry3d & _initial_pose)
{
  OdomNode * fixed_node = new OdomNode(_initial_pose);
  fixed_node->set_fixed();
  add_node(*fixed_node);
  last_odom_node_ = fixed_node;
}

bool GraphG2O::optimize_graph()
{
  if (graph_->vertices().empty()) {
    ERROR_GRAPH("No vertices in the optimizer! Skipping optimization.");
    return false;
  }
  if (graph_->edges().empty()) {
    ERROR_GRAPH("No edges in the optimizer! Skipping optimization.");
    return false;
  }

  const int num_iterations = 100;
  INFO_GRAPH("nodes: " << graph_->vertices().size() << "   edges: " << graph_->edges().size());
  graph_->initializeOptimization();
  graph_->setVerbose(false);

  graph_->optimize(num_iterations);
  if (std::isnan(graph_->chi2())) {  // LCOV_EXCL_START
    ERROR_GRAPH("GRAPH RETURNED A NAN AFTER OPTIMIZATION");
    return false;
  }  // LCOV_EXCL_STOP
  return true;
}

void GraphG2O::add_node(GraphNode & _node)
{
  int id = n_vertices_++;
  _node.get_vertex()->setId(id);
  graph_->addVertex(_node.get_vertex());
  graph_nodes_.emplace_back(&_node);
}

void GraphG2O::add_edge(GraphEdge & _edge)
{
  int id = n_edges_++;
  _edge.get_edge()->setId(id);
  graph_->addEdge(_edge.get_edge());
  graph_edges_.emplace_back(&_edge);
}

void GraphG2O::add_new_keyframe(
  const Eigen::Isometry3d & _absolute_pose,
  const Eigen::Isometry3d & _relative_pose,
  const Eigen::MatrixXd & _relative_covariance)
{
  OdomNode * odom_node = new OdomNode(_absolute_pose);
  add_node(*odom_node);

  Eigen::MatrixXd information_matrix = _relative_covariance.inverse();
  OdomEdge * odom_edge = new OdomEdge(
    last_odom_node_, odom_node, _relative_pose, information_matrix);
  add_edge(*odom_edge);
  last_odom_node_ = odom_node;
}

void GraphG2O::add_new_object_detection(ObjectDetection * _object_detection)
{
  GraphNode * object_node = obj_id2node_[_object_detection->get_id()];
  if (!object_node) {
    object_node = _object_detection->create_node();
    add_node(*object_node);
    obj_id2node_[_object_detection->get_id()] = object_node;
  }

  GraphEdge * object_edge = _object_detection->create_edge(last_odom_node_, object_node);
  if (object_edge == nullptr) {
    return;
  }
  add_edge(*object_edge);
}

Eigen::MatrixXd GraphG2O::compute_node_covariance(GraphNode * _node)
{
  int node_id = _node->get_vertex()->id();
  g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv;

  auto node_se3 = dynamic_cast<g2o::VertexSE3 *>(_node->get_vertex());
  if (node_se3) {
    graph_->computeMarginals(spinv, node_se3);
  }
  auto node_point3d = dynamic_cast<g2o::VertexPointXYZ *>(_node->get_vertex());
  if (node_point3d) {
    graph_->computeMarginals(spinv, node_point3d);
  }

  if (spinv.nonZeroBlocks() < 1) {  // LCOV_EXCL_LINE
    return Eigen::MatrixXd();  // LCOV_EXCL_LINE
  }

  Eigen::MatrixXd covariance;
  auto block_cols = spinv.blockCols();
  if (node_id < 1 || static_cast<size_t>(node_id - 1) >= block_cols.size()) {  // LCOV_EXCL_START
    ERROR_GRAPH("Node ID " << node_id << " is out of bounds for block_cols");
    return Eigen::MatrixXd();
  }  // LCOV_EXCL_STOP
  auto it = block_cols[node_id - 1].find(node_id - 1);
  if (it != block_cols[node_id - 1].end()) {
    covariance = *it->second;
  }
  return covariance;
}
