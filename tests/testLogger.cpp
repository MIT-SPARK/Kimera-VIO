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
 * @author Antoni Rosinol, Luca Carlone
 */

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <boost/random/mersenne_twister.hpp>

#include "logging/Logger.h"
#include "StereoVisionFrontEnd-definitions.h"
#include "VioBackEnd-definitions.h"

#include <gflags/gflags.h>
#include <gtest/gtest.h>

DECLARE_string(test_data_path);
DECLARE_string(output_path);

static const double tol = 1e-7;

namespace VIO {

using csv_mat = std::vector<std::vector<std::string>>;

class CSVReader {
 public:
	CSVReader(std::string sep = ",")
      : sep_(sep) {}

	csv_mat getData(std::string filename) {
    std::ifstream file(filename);
    csv_mat dataList;
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

class BackendLoggerFixture : public ::testing::Test {
 public:
  BackendLoggerFixture()
      : logger_FLAGS_test_data_path(FLAGS_test_data_path +
                                    std::string("/ForLogger/")) {
    FLAGS_output_path = logger_FLAGS_test_data_path + "backend_output/";
    logger_ = VIO::make_unique<BackendLogger>();

    // Seed randomness.
    std::srand(0);
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

 protected:
  void checkHeader(std::vector<std::string> actual,
      std::vector<std::string> expected) {
    EXPECT_EQ(actual.at(0), expected.at(0));
    EXPECT_EQ(actual.at(1), expected.at(1));
    EXPECT_EQ(actual.at(2), expected.at(2));
    EXPECT_EQ(actual.at(3), expected.at(3));
    EXPECT_EQ(actual.at(4), expected.at(4));
    EXPECT_EQ(actual.at(5), expected.at(5));
    EXPECT_EQ(actual.at(6), expected.at(6));
    EXPECT_EQ(actual.at(7), expected.at(7));
    EXPECT_EQ(actual.at(8), expected.at(8));
    EXPECT_EQ(actual.at(9), expected.at(9));
    EXPECT_EQ(actual.at(10), expected.at(10));
    EXPECT_EQ(actual.at(11), expected.at(11));
    EXPECT_EQ(actual.at(12), expected.at(12));
    EXPECT_EQ(actual.at(13), expected.at(13));
    EXPECT_EQ(actual.at(14), expected.at(14));
    EXPECT_EQ(actual.at(15), expected.at(15));
    EXPECT_EQ(actual.at(16), expected.at(16));
  }

 protected:
  std::string logger_FLAGS_test_data_path;
  std::unique_ptr<BackendLogger> logger_;
  CSVReader csv_reader_;
  boost::random::mt19937 rng_;
};

TEST_F(BackendLoggerFixture, logBackendOutput) {
  // Declare all random output members.
  Timestamp timestamp = rand();
  gtsam::Values state_values;
  gtsam::Pose3 W_Pose_Blkf = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  gtsam::Vector3 W_Vel_Blkf = gtsam::Vector3::Random();
  gtsam::Pose3 B_Pose_leftCam = gtsam::Pose3(
      gtsam::Rot3::Random(rng_), gtsam::Point3::Random());
  ImuBias imu_bias;
  int cur_kf_id = rand();
  int landmark_count = rand();


  logger_->logBackendOutput(
      VioBackEndOutputPayload(
          timestamp,
          state_values,
          W_Pose_Blkf,
          W_Vel_Blkf,
          B_Pose_leftCam,
          imu_bias,
          gtsam::Matrix(),
          cur_kf_id,
          landmark_count,
          DebugVioInfo()));


  // First check the output_posesVIO.csv results file.
  std::string results_csv = FLAGS_output_path + "output_posesVIO.csv";
  csv_mat results = csv_reader_.getData(results_csv);

  // Check that only header and one line were logged.
  EXPECT_EQ(results.size(), 2);

  // Check csv header for output_posesVIO.csv.
  std::vector<std::string> actual_results_header = results.at(0);
  std::vector<std::string> expected_results_header =
      {"timestamp", "x", "y", "z", "qx", "qy", "qz", "qw", "vx", "vy", "vz",
       "bgx", "bgy", "bgz", "bax", "bay", "baz"};
  checkHeader(actual_results_header, expected_results_header);

  // Check values of the only result line.
  std::vector<std::string> actual_W_Pose_Blkf = results.at(1);
  Timestamp actual_timestamp = std::stof(actual_W_Pose_Blkf.at(0));
  float actual_x = std::stof(actual_W_Pose_Blkf.at(1));
  float actual_y = std::stof(actual_W_Pose_Blkf.at(2));
  float actual_z = std::stof(actual_W_Pose_Blkf.at(3));
  float actual_qx = std::stof(actual_W_Pose_Blkf.at(4));
  float actual_qy = std::stof(actual_W_Pose_Blkf.at(5));
  float actual_qz = std::stof(actual_W_Pose_Blkf.at(6));
  float actual_qw = std::stof(actual_W_Pose_Blkf.at(7));
  float actual_vx = std::stof(actual_W_Pose_Blkf.at(8));
  float actual_vy = std::stof(actual_W_Pose_Blkf.at(9));
  float actual_vz = std::stof(actual_W_Pose_Blkf.at(10));
  float actual_bgx = std::stof(actual_W_Pose_Blkf.at(11));
  float actual_bgy = std::stof(actual_W_Pose_Blkf.at(12));
  float actual_bgz = std::stof(actual_W_Pose_Blkf.at(13));
  float actual_bax = std::stof(actual_W_Pose_Blkf.at(14));
  float actual_bay = std::stof(actual_W_Pose_Blkf.at(15));
  float actual_baz = std::stof(actual_W_Pose_Blkf.at(16));
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
  std::cout << "finished all the first ones." << std::endl;



  // Next we check the output_smartFactors.csv results file.
  std::string smart_factors_csv = FLAGS_output_path + "output_smartFactors.csv";
  csv_mat smart_factors = csv_reader_.getData(smart_factors_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(smart_factors.size(), 2);

  // Check csv header for output_smarFactors.csv.
  std::vector<std::string> actual_smart_factors_header = smart_factors.at(0);
  std::vector<std::string> expected_smart_factors_header =
      {"cur_kf_id", "timestamp_kf", "numSF", "numValid", "numDegenerate",
       "numFarPoints", "numOutliers", "numCheirality", "numNonInitialized",
       "meanPixelError", "maxPixelError", "meanTrackLength", "maxTrackLength",
       "nrElementsInMatrix", "nrZeroElementsInMatrix"};
  checkHeader(actual_smart_factors_header, expected_smart_factors_header);
  // TODO(marcus): check values if you can easily make them nonzero.
  std::cout << "finished all the second ones." << std::endl;



  // Next we check the output_pim_navstates.csv results file.
  std::string pim_csv = FLAGS_output_path + "output_pim_navstates_csv";
  csv_mat pim = csv_reader_.getData(pim_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(pim.size(), 2);

  // Check csv header for output_pim_navstates.csv.
  std::vector<std::string> actual_pim_header = pim.at(0);
  std::vector<std::string> expected_pim_header =
      {"timestamp", "x", "y", "z", "qw", "qx", "qy", "qz", "vx", "vy", "vz"};
  checkHeader(actual_pim_header, expected_pim_header);
  // TODO(marcus): check values if you can easily make them nonzero.
  std::cout << "finished all the third ones." << std::endl;



  // Next we check the output_backendFactors.csv results file.
  std::string factor_stats_csv = FLAGS_output_path+"output_backendFactors.csv";
  csv_mat factor_stats = csv_reader_.getData(factor_stats_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(factor_stats.size(), 2);

  // Check the csv header for output_backendFactors.csv.
  std::vector<std::string> actual_factor_stats_header = factor_stats.at(0);
  std::vector<std::string> expected_factor_stats_header =
      {"cur_kf_id", "numAddedSmartF", "numAddedImuF", "numAddedNoMotionF",
       "numAddedConstantF", "numAddedBetweenStereoF", "state_size",
       "landmark_count"};
  checkHeader(actual_factor_stats_header, expected_factor_stats_header);
  // TODO(marcus): check values if you can easily make them nonzero.
  std::cout << "finished all the fourth ones." << std::endl;



  // Next we check the output_backendTiming.csv results file.
  std::string timing_csv = FLAGS_output_path + "output_backendTiming.csv";
  csv_mat timing = csv_reader_.getData(timing_csv);

  // Check that only the header and one line were logged.
  EXPECT_EQ(timing.size(), 2);

  // Check the csv header for output_backendTiming.csv.
  std::vector<std::string> actual_timing_header = timing.at(0);
  std::vector<std::string> expected_timing_header =
      {"cur_kf_id", "factorsAndSlotsTime", "preUpdateTime", "updateTime",
       "updateSlotTime", "extraIterationsTime", "linearizeTime",
       "linearSolveTime", "retractTime", "linearizeMarginalizeTime",
       "marginalizeTime"};
  checkHeader(actual_timing_header, expected_timing_header);
  // TODO(marcus): check values if you can easily make them nonzero.
  std::cout << "finished all the fifth ones." << std::endl;
}

} // namespace VIO
