/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testStereoVisionFrontEnd.cpp
 * @brief  test StereoVisionFrontEnd
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <random>
#include <boost/algorithm/string.hpp>

#include <gflags/gflags.h>
#include <gtest/gtest.h>

#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/logging/Logger.h"

DECLARE_string(test_data_path);

namespace VIO {

static const double tol = 1e-7;
using CsvMat = std::vector<std::vector<std::string>>;

void checkHeader(std::vector<std::string> actual,
    std::vector<std::string> expected) {
  EXPECT_EQ(actual.size(), expected.size());

  for (size_t i=0; i<actual.size(); i++) {
    EXPECT_EQ(actual.at(i), expected.at(i));
  }
}

class CSVReader {
 public:
  CSVReader(std::string sep = ",") : sep_(sep) {}

  CsvMat getData(std::string filename) {
    std::ifstream file(filename);
    CsvMat dataList;
    std::string line = "";

    while (std::getline(file, line)) {
      std::vector<std::string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(sep_));
      dataList.push_back(vec);
    }

    file.close();
    return dataList;
  }

 private:
  std::string sep_;
};

class LoggerFixture : public ::testing::Test {
 public:
  LoggerFixture() : output_path_(
      FLAGS_test_data_path + std::string("/ForLogger/")) {}

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  std::string output_path_;
  CSVReader csv_reader_;
  std::mt19937 rng_;
  std::default_random_engine random_eng_;
};

class BackendLoggerFixture : public LoggerFixture {
 public:
  BackendLoggerFixture() {
    logger_ = VIO::make_unique<BackendLogger>(output_path_);

    // Seed randomness.
    std::srand(0);
  }

 protected:
  std::unique_ptr<BackendLogger> logger_;
};

class FrontendLoggerFixture : public LoggerFixture {
 public:
  FrontendLoggerFixture() {
    logger_ = VIO::make_unique<FrontendLogger>(output_path_);

    // Seed randomness.
    std::srand(0);
  }

 protected:
  std::unique_ptr<FrontendLogger> logger_;
};

class LoopClosureDetectorLoggerFixture : public LoggerFixture {
 public:
  LoopClosureDetectorLoggerFixture() {
    logger_ = VIO::make_unique<LoopClosureDetectorLogger>(output_path_);

    // Seed randomness.
    std::srand(0);
  }

 protected:
  std::unique_ptr<LoopClosureDetectorLogger> logger_;
};

TEST_F(BackendLoggerFixture, logBackendOutput) {
  // Declare all random output members.
  const Timestamp& timestamp = random_eng_();
  gtsam::Values state_values;
  gtsam::Pose3 W_Pose_Blkf = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Vector3 W_Vel_Blkf = gtsam::Vector3::Random();
  ImuBias imu_bias;
  FrameId cur_kf_id = random_eng_();
  int landmark_count = random_eng_();

  logger_->logBackendOutput(BackendOutput(timestamp,
                                          state_values,
                                          W_Pose_Blkf,
                                          W_Vel_Blkf,
                                          imu_bias,
                                          gtsam::Matrix(),
                                          cur_kf_id,
                                          landmark_count,
                                          DebugVioInfo(),
                                          PointsWithIdMap(),
                                          LmkIdToLmkTypeMap()));

  // First check the output_posesVIO.csv results file.
  std::string results_csv = output_path_ + "/traj_vio.csv";
  CsvMat results = csv_reader_.getData(results_csv);

  // Check that only header and one line were logged.
  EXPECT_EQ(results.size(), 2);

  // Check csv header for output_posesVIO.csv.
  std::vector<std::string> actual_results_header = results.at(0);
  std::vector<std::string> expected_results_header =
      {"#timestamp", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz",
       "bgx", "bgy", "bgz", "bax", "bay", "baz"};
  checkHeader(actual_results_header, expected_results_header);

  // Check values of the only result line.
  std::vector<std::string> actual_W_Pose_Blkf = results.at(1);
  Timestamp actual_timestamp = std::stof(actual_W_Pose_Blkf.at(0));
  float actual_x = std::stof(actual_W_Pose_Blkf.at(1));
  float actual_y = std::stof(actual_W_Pose_Blkf.at(2));
  float actual_z = std::stof(actual_W_Pose_Blkf.at(3));
  float actual_qw = std::stof(actual_W_Pose_Blkf.at(4));
  float actual_qx = std::stof(actual_W_Pose_Blkf.at(5));
  float actual_qy = std::stof(actual_W_Pose_Blkf.at(6));
  float actual_qz = std::stof(actual_W_Pose_Blkf.at(7));
  float actual_vx = std::stof(actual_W_Pose_Blkf.at(8));
  float actual_vy = std::stof(actual_W_Pose_Blkf.at(9));
  float actual_vz = std::stof(actual_W_Pose_Blkf.at(10));
  float actual_bgx = std::stof(actual_W_Pose_Blkf.at(11));
  float actual_bgy = std::stof(actual_W_Pose_Blkf.at(12));
  float actual_bgz = std::stof(actual_W_Pose_Blkf.at(13));
  float actual_bax = std::stof(actual_W_Pose_Blkf.at(14));
  float actual_bay = std::stof(actual_W_Pose_Blkf.at(15));
  float actual_baz = std::stof(actual_W_Pose_Blkf.at(16));
  EXPECT_EQ(actual_timestamp, timestamp);
  EXPECT_LT(actual_x - W_Pose_Blkf.translation().x(), tol);
  EXPECT_LT(actual_y - W_Pose_Blkf.translation().y(), tol);
  EXPECT_LT(actual_z - W_Pose_Blkf.translation().z(), tol);
  EXPECT_LT(actual_z - W_Pose_Blkf.translation().z(), tol);
  EXPECT_LT(actual_qx - W_Pose_Blkf.rotation().toQuaternion().x(), tol);
  EXPECT_LT(actual_qy - W_Pose_Blkf.rotation().toQuaternion().y(), tol);
  EXPECT_LT(actual_qz - W_Pose_Blkf.rotation().toQuaternion().z(), tol);
  EXPECT_LT(actual_qw - W_Pose_Blkf.rotation().toQuaternion().w(), tol);
  EXPECT_LT(actual_vx - W_Vel_Blkf(0), tol);
  EXPECT_LT(actual_vy - W_Vel_Blkf(1), tol);
  EXPECT_LT(actual_vz - W_Vel_Blkf(2), tol);
  EXPECT_LT(actual_bgx - imu_bias.gyroscope().transpose()(0), tol);
  EXPECT_LT(actual_bgy - imu_bias.gyroscope().transpose()(1), tol);
  EXPECT_LT(actual_bgz - imu_bias.gyroscope().transpose()(2), tol);
  EXPECT_LT(actual_bax - imu_bias.accelerometer()(0), tol);
  EXPECT_LT(actual_bay - imu_bias.accelerometer()(1), tol);
  EXPECT_LT(actual_baz - imu_bias.accelerometer()(2), tol);


  // Next we check the output_smartFactors.csv results file.
  std::string smart_factors_csv = output_path_ + "/output_smartFactors.csv";
  CsvMat smart_factors = csv_reader_.getData(smart_factors_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(smart_factors.size(), 2);

  // Check csv header for output_smartFactors.csv.
  std::vector<std::string> actual_smart_factors_header = smart_factors.at(0);
  std::vector<std::string> expected_smart_factors_header =
      {"#cur_kf_id", "timestamp_kf", "numSF", "numValid", "numDegenerate",
       "numFarPoints", "numOutliers", "numCheirality", "numNonInitialized",
       "meanPixelError", "maxPixelError", "meanTrackLength", "maxTrackLength",
       "nrElementsInMatrix", "nrZeroElementsInMatrix"};
  checkHeader(actual_smart_factors_header, expected_smart_factors_header);
  // TODO(marcus): check values if you can easily make them nonzero.


  // Next we check the output_pim_navstates.csv results file.
  std::string pim_csv = output_path_ + "/output_pim_navstates.csv";
  CsvMat pim = csv_reader_.getData(pim_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(pim.size(), 2);

  // Check csv header for output_pim_navstates.csv.
  std::vector<std::string> actual_pim_header = pim.at(0);
  std::vector<std::string> expected_pim_header =
      {"#timestamp_kf", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz"};
  checkHeader(actual_pim_header, expected_pim_header);
  // TODO(marcus): check values if you can easily make them nonzero.


  // Next we check the output_backendFactors.csv results file.
  std::string factor_stats_csv = output_path_ + "/output_backendFactors.csv";
  CsvMat factor_stats = csv_reader_.getData(factor_stats_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(factor_stats.size(), 2);

  // Check the csv header for output_backendFactors.csv.
  std::vector<std::string> actual_factor_stats_header = factor_stats.at(0);
  std::vector<std::string> expected_factor_stats_header =
      {"#cur_kf_id", "numAddedSmartF", "numAddedImuF", "numAddedNoMotionF",
       "numAddedConstantF", "numAddedBetweenStereoF", "state_size",
       "landmark_count"};
  checkHeader(actual_factor_stats_header, expected_factor_stats_header);
  // TODO(marcus): check values if you can easily make them nonzero.


  // Next we check the output_backendTiming.csv results file.
  std::string timing_csv = output_path_ + "/output_backendTiming.csv";
  CsvMat timing = csv_reader_.getData(timing_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(timing.size(), 2);

  // Check the csv header for output_backendTiming.csv.
  std::vector<std::string> actual_timing_header = timing.at(0);
  std::vector<std::string> expected_timing_header =
      {"#cur_kf_id", "factorsAndSlotsTime", "preUpdateTime", "updateTime",
       "updateSlotTime", "extraIterationsTime", "linearizeTime",
       "linearSolveTime", "retractTime", "linearizeMarginalizeTime",
       "marginalizeTime"};
  checkHeader(actual_timing_header, expected_timing_header);
  // TODO(marcus): check values if you can easily make them nonzero.
}

TEST_F(FrontendLoggerFixture, logFrontendStats) {
  const Timestamp& timestamp = random_eng_();
  const size_t& nrKeypoints = random_eng_();

  logger_->logFrontendStats(
      timestamp,
      VIO::DebugTrackerInfo(),
      VIO::TrackerStatusSummary(),
      nrKeypoints);

  // First check the output_frontend_stats.csv results file.
  std::string stats_csv = output_path_ + "/output_frontend_stats.csv";
  CsvMat stats = csv_reader_.getData(stats_csv);

  // Check that only header and one line were logged.
  EXPECT_EQ(stats.size(), 2);

  // Check csv header for output_posesVIO.csv.
  std::vector<std::string> actual_results_header = stats.at(0);
  std::vector<std::string> expected_results_header =
      {"#timestamp_lkf", "mono_status", "stereo_status", "nr_keypoints",
       "nrDetectedFeatures", "nrTrackerFeatures", "nrMonoInliers",
       "nrMonoPutatives", "nrStereoInliers", "nrStereoPutatives",
       "monoRansacIters", "stereoRansacIters", "nrValidRKP", "nrNoLeftRectRKP",
       "nrNoRightRectRKP", "nrNoDepthRKP", "nrFailedArunRKP",
       "featureDetectionTime", "featureTrackingTime", "monoRansacTime",
       "stereoRansacTime", "featureSelectionTime", "extracted_corners",
       "need_n_corners"};
  checkHeader(actual_results_header, expected_results_header);

  // Check values of the only result line.
  std::vector<std::string> actual_stats = stats.at(1);

  Timestamp actual_timestamp = std::stof(actual_stats.at(0));
  std::string actual_mono_status = actual_stats.at(1);
  std::string actual_stereo_status = actual_stats.at(2);
  size_t actual_nrkeypoints = std::stof(actual_stats.at(3));

  EXPECT_EQ(actual_timestamp, timestamp);
  EXPECT_EQ(actual_mono_status, "INVALID");
  EXPECT_EQ(actual_stereo_status, "INVALID");
  // TODO(marcus): why doesn't this work:
  // EXPECT_EQ(actual_nrkeypoints, nrKeypoints);

  for (size_t i = 4; i < actual_stats.size(); i++) {
    float val = std::stof(actual_stats.at(i));
    EXPECT_EQ(val, 0);
  }
}

TEST_F(FrontendLoggerFixture, logFrontendRansac) {
  const Timestamp& timestamp = random_eng_();
  gtsam::Pose3 mono_pose = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Pose3 stereo_pose = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());

  logger_->logFrontendRansac(
      timestamp,
      mono_pose,
      stereo_pose);

  // First check the output_frontend_ransac_mono.csv results file.
  std::string ransac_mono_csv = output_path_ +
      "/output_frontend_ransac_mono.csv";
  CsvMat ransac_mono = csv_reader_.getData(ransac_mono_csv);

  // Check that only header and one line were logged.
  EXPECT_EQ(ransac_mono.size(), 2);

  // Check csv header for output_frontend_ransac_mono.csv.
  std::vector<std::string> actual_mono_header = ransac_mono.at(0);
  std::vector<std::string> expected_mono_header =
      {"#timestamp_lkf", "x", "y", "z", "qw", "qx", "qy", "qz"};
  checkHeader(actual_mono_header, expected_mono_header);

  // Check values of the only result line.
  std::vector<std::string> actual_ransac_mono = ransac_mono.at(1);

  Timestamp actual_timestamp = std::stof(actual_ransac_mono.at(0));
  float actual_x = std::stof(actual_ransac_mono.at(1));
  float actual_y = std::stof(actual_ransac_mono.at(2));
  float actual_z = std::stof(actual_ransac_mono.at(3));
  float actual_qw = std::stof(actual_ransac_mono.at(4));
  float actual_qx = std::stof(actual_ransac_mono.at(5));
  float actual_qy = std::stof(actual_ransac_mono.at(6));
  float actual_qz = std::stof(actual_ransac_mono.at(7));
  EXPECT_EQ(actual_timestamp, timestamp);
  EXPECT_LT(actual_x - mono_pose.translation().x(), tol);
  EXPECT_LT(actual_y - mono_pose.translation().y(), tol);
  EXPECT_LT(actual_z - mono_pose.translation().z(), tol);
  EXPECT_LT(actual_qw - mono_pose.rotation().toQuaternion().w(), tol);
  EXPECT_LT(actual_qx - mono_pose.rotation().toQuaternion().x(), tol);
  EXPECT_LT(actual_qy - mono_pose.rotation().toQuaternion().y(), tol);
  EXPECT_LT(actual_qz - mono_pose.rotation().toQuaternion().z(), tol);

  // Lastly do the same checks for the stereo file.
  std::string ransac_stereo_csv = output_path_ +
      "/output_frontend_ransac_stereo.csv";
  CsvMat ransac_stereo = csv_reader_.getData(ransac_stereo_csv);

  EXPECT_EQ(ransac_stereo.size(), 2);

  std::vector<std::string> actual_stereo_header = ransac_stereo.at(0);
  std::vector<std::string> expected_stereo_header =
      {"#timestamp_lkf", "x", "y", "z", "qw", "qx", "qy", "qz"};
  checkHeader(actual_stereo_header, expected_stereo_header);

  std::vector<std::string> actual_ransac_stereo = ransac_stereo.at(1);

  actual_timestamp = std::stof(actual_ransac_stereo.at(0));
  actual_x = std::stof(actual_ransac_stereo.at(1));
  actual_y = std::stof(actual_ransac_stereo.at(2));
  actual_z = std::stof(actual_ransac_stereo.at(3));
  actual_qw = std::stof(actual_ransac_stereo.at(4));
  actual_qx = std::stof(actual_ransac_stereo.at(5));
  actual_qy = std::stof(actual_ransac_stereo.at(6));
  actual_qz = std::stof(actual_ransac_stereo.at(7));
  EXPECT_EQ(actual_timestamp, timestamp);
  EXPECT_LT(actual_x - stereo_pose.translation().x(), tol);
  EXPECT_LT(actual_y - stereo_pose.translation().y(), tol);
  EXPECT_LT(actual_z - stereo_pose.translation().z(), tol);
  EXPECT_LT(actual_qw - stereo_pose.rotation().toQuaternion().w(), tol);
  EXPECT_LT(actual_qx - stereo_pose.rotation().toQuaternion().x(), tol);
  EXPECT_LT(actual_qy - stereo_pose.rotation().toQuaternion().y(), tol);
  EXPECT_LT(actual_qz - stereo_pose.rotation().toQuaternion().z(), tol);
}

TEST_F(LoopClosureDetectorLoggerFixture, logOptimizedTraj) {
  const Timestamp& timestamp_kf = random_eng_();
  const Timestamp& timestamp_query = random_eng_();
  const Timestamp& timestamp_match = random_eng_();
  const FrameId& id_match = random_eng_();
  const FrameId& id_recent = random_eng_();
  gtsam::Pose3 relative_pose = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Pose3 w_pose_map = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Pose3 traj_pose = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Values traj_values;
  traj_values.insert(gtsam::Symbol(0), traj_pose);
  traj_values.insert(gtsam::Symbol(1), traj_pose);

  std::unordered_map<VIO::FrameId, VIO::Timestamp> ts_map;
  ts_map[0] = timestamp_kf;
  ts_map[1] = timestamp_kf;

  logger_->logTimestampMap(ts_map);

  logger_->logOptimizedTraj(LcdOutput(true,
                                      timestamp_kf,
                                      timestamp_query,
                                      timestamp_match,
                                      id_match,
                                      id_recent,
                                      relative_pose,
                                      w_pose_map,
                                      traj_values,
                                      gtsam::NonlinearFactorGraph()));

  // First check the output_frontend_ransac_mono.csv results file.
  std::string opt_traj_csv = output_path_ + "/traj_pgo.csv";
  CsvMat opt_traj = csv_reader_.getData(opt_traj_csv);

  // Check that only header and one line were logged.
  EXPECT_EQ(opt_traj.size(), 2);

  // Check csv header for output_frontend_ransac_mono.csv.
  std::vector<std::string> actual_header = opt_traj.at(0);
  std::vector<std::string> expected_header =
      {"#timestamp_kf", "x", "y", "z", "qw", "qx", "qy", "qz"};
  checkHeader(actual_header, expected_header);

  // Check values of the only result line.
  std::vector<std::string> actual_opt_traj = opt_traj.at(1);

  Timestamp actual_timestamp = std::stof(actual_opt_traj.at(0));
  float actual_x = std::stof(actual_opt_traj.at(1));
  float actual_y = std::stof(actual_opt_traj.at(2));
  float actual_z = std::stof(actual_opt_traj.at(3));
  float actual_qw = std::stof(actual_opt_traj.at(4));
  float actual_qx = std::stof(actual_opt_traj.at(5));
  float actual_qy = std::stof(actual_opt_traj.at(6));
  float actual_qz = std::stof(actual_opt_traj.at(7));
  EXPECT_EQ(actual_timestamp, timestamp_kf);
  EXPECT_LT(actual_x - traj_pose.translation().x(), tol);
  EXPECT_LT(actual_y - traj_pose.translation().y(), tol);
  EXPECT_LT(actual_z - traj_pose.translation().z(), tol);
  EXPECT_LT(actual_qw - traj_pose.rotation().toQuaternion().w(), tol);
  EXPECT_LT(actual_qx - traj_pose.rotation().toQuaternion().x(), tol);
  EXPECT_LT(actual_qy - traj_pose.rotation().toQuaternion().y(), tol);
  EXPECT_LT(actual_qz - traj_pose.rotation().toQuaternion().z(), tol);
}

TEST(testOpenFile, OpenFile) {
  std::ofstream outputFile;
  OpenFile("tmp.txt", &outputFile);
  EXPECT_TRUE(outputFile.is_open());
  outputFile.close();
  EXPECT_TRUE(!outputFile.is_open());
}

// TODO(marcus): add remaining tests for non-critical logs on all three modules.

} // namespace VIO
