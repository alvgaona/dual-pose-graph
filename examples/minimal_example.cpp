#include <iostream>

#include <dual_pose_graph/optimizer_g2o.hpp>
#include <dual_pose_graph/object_detection_types.hpp>

int main()
{
  OptimizerG2O optimizer;

  OptimizerG2OParameters params;
  params.main_graph_odometry_distance_threshold = 0.5;
  params.temp_graph_odometry_distance_threshold = 0.1;
  params.use_dual_graph = true;
  params.generate_odom_map_transform = true;
  optimizer.set_parameters(params);

  const int steps = 20;
  const double radius = 2.0;
  const Eigen::MatrixXd odom_cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;

  for (int i = 0; i < steps; ++i) {
    double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(steps);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() << radius * std::cos(theta), radius * std::sin(theta), 0.0;
    pose.linear() = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    OdometryWithCovariance odom;
    odom.odometry = pose;
    odom.covariance = odom_cov;
    optimizer.handle_new_odom(odom);

    Eigen::Isometry3d landmark = Eigen::Isometry3d::Identity();
    landmark.translation() = pose.translation() + Eigen::Vector3d(0.5, 0.0, 0.0);

    OdometryWithCovariance det_odom;
    det_odom.odometry = pose;
    det_odom.covariance = odom_cov;
    OdometryInfo det_info;
    if (optimizer.check_adding_new_detection(det_odom, det_info)) {
      SE3ObjectDetection detection(
        "landmark", "lm_0", landmark,
        Eigen::MatrixXd::Identity(6, 6) * 0.05,
        /*is_absolute=*/true);
      optimizer.handle_new_object_detection(&detection, det_info);
    }
  }

  auto final_pose = optimizer.get_optimized_pose();
  std::cout << "Final optimized position: "
            << final_pose.translation().transpose() << std::endl;
  return 0;
}
