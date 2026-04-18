#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>

#include <dual_pose_graph/utils/csv_logger.hpp>

namespace fs = std::filesystem;

class CsvLoggerTest : public ::testing::Test
{
protected:
  fs::path dir_;

  void SetUp() override
  {
    dir_ = fs::temp_directory_path() /
      ("dpg_csv_logger_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) +
      "_" + ::testing::UnitTest::GetInstance()->current_test_info()->name());
    fs::create_directories(dir_);
  }

  void TearDown() override
  {
    std::error_code ec;
    fs::remove_all(dir_, ec);
  }
};

TEST_F(CsvLoggerTest, AllLogMethodsWriteExpectedFiles)
{
  {
    CsvLogger logger(dir_.string());

    Eigen::Isometry3d raw = Eigen::Isometry3d::Identity();
    raw.translation() << 1.0, 2.0, 3.0;
    Eigen::Isometry3d corrected = Eigen::Isometry3d::Identity();
    corrected.translation() << 1.1, 2.1, 3.1;
    Eigen::MatrixXd cov = Eigen::MatrixXd::Identity(6, 6) * 0.01;

    // log_odom is throttled every Nth call; drive it enough to flush at least once.
    for (int i = 0; i < 12; ++i) {
      logger.log_odom(static_cast<int64_t>(i), 0, raw, cov, corrected);
    }

    logger.log_keyframe(
      10, 0, raw, /*main_nodes=*/5, /*main_edges=*/4,
      /*chi2_before=*/1.5, /*chi2_after=*/0.5, /*opt_time_ms=*/2.3,
      /*map_odom_tf=*/Eigen::Isometry3d::Identity(),
      /*optimized=*/raw, /*corrected=*/corrected);

    logger.log_detection(
      10, 0, "id1", "landmark",
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      /*is_absolute=*/true);

    logger.log_merge(
      10, 0, "id1",
      Eigen::Vector3d(5.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0),
      /*temp_nodes=*/3, /*temp_edges=*/2, /*temp_chi2=*/0.2);

    logger.log_fixed_object(
      "fixed1", "landmark",
      Eigen::Vector3d(10.0, 0.0, 0.0),
      Eigen::Vector3d(10.0, 0.0, 0.0));
  }  // destructor flushes files

  EXPECT_TRUE(fs::exists(dir_ / "slam_odom.csv"));
  EXPECT_TRUE(fs::exists(dir_ / "slam_keyframes.csv"));
  EXPECT_TRUE(fs::exists(dir_ / "slam_detections.csv"));
  EXPECT_TRUE(fs::exists(dir_ / "slam_merges.csv"));
  EXPECT_TRUE(fs::exists(dir_ / "slam_fixed_objects.csv"));

  // Spot check: the keyframes file has more than just a header line after logging.
  auto kf_size = fs::file_size(dir_ / "slam_keyframes.csv");
  EXPECT_GT(kf_size, 100u);
}
