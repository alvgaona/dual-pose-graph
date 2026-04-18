#include <gtest/gtest.h>

#include <dual_pose_graph/object_detection_types.hpp>
#include <dual_pose_graph/optimizer_g2o.hpp>

TEST(TempToMainMerge, TempGraphDrainsIntoMainOnNextKeyframe)
{
  OptimizerG2O optimizer;
  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;
  params.main_graph_odometry_orientation_threshold = 10.0;
  params.main_graph_odometry_distance_threshold_if_detections = 0.1;
  params.temp_graph_odometry_distance_threshold = 0.05;
  params.temp_graph_odometry_orientation_threshold = 10.0;
  params.use_dual_graph = true;
  optimizer.set_parameters(params);

  Eigen::MatrixXd odom_cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;
  Eigen::MatrixXd det_cov = Eigen::MatrixXd::Identity(6, 6) * 0.05;

  // Drive along +x, emitting detections of the same generic landmark.
  for (int i = 0; i < 10; ++i) {
    OdometryWithCovariance odom;
    odom.odometry = Eigen::Isometry3d::Identity();
    odom.odometry.translation().x() = 0.2 * i;
    odom.covariance = odom_cov;
    optimizer.handle_new_odom(odom);

    Eigen::Isometry3d lm = Eigen::Isometry3d::Identity();
    lm.translation() = Eigen::Vector3d(5.0, 0.0, 0.0);

    OdometryInfo det_info;
    if (!optimizer.check_adding_new_detection(odom, det_info)) {
      continue;
    }
    SE3ObjectDetection detection("landmark", "lm0", lm, det_cov, true);
    optimizer.handle_new_object_detection(&detection, det_info);
  }

  const auto & main_objs = optimizer.main_graph->get_object_nodes();
  EXPECT_NE(main_objs.find("lm0"), main_objs.end());
}
