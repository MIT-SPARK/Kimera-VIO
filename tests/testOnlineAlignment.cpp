/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testOnlineAlignment.cpp
 * @brief  Unit tests for Online Alignment class.
 * @author Sandro Berchier, Luca Carlone
 */


#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ImuFrontEnd-definitions.h"
#include "ImuFrontEnd.h"
#include "OnlineGravityAlignment.h"
#include "utils/ThreadsafeImuBuffer.h"
#include "ETH_parser.h"
#include "test_config.h"

// Add last, since it redefines CHECK, which is first defined by glog.
#include <CppUnitLite/TestHarness.h>

static const std::string data_path(DATASET_PATH + std::string("/ForOnlineAlignment/"));
//int n_begin_dataset = 1185;
//int n_frames = 30;

int n_begin_dataset = 1;
int n_frames = 35;

namespace VIO {

/* -------------------------------------------------------------------------- */
TEST(testOnlineAlignment, GyroscopeBiasEstimation) {  
  // Construct ETH Parser
  std::string reason = "test of gyroscope estimation";
  ETHDatasetParser dataset(reason);

  // Load IMU data and compute pre-integrations
  std::string imu_data_path = "./data/ForOnlineAlignment";
  std::string imu_name = "imu0";
  dataset.parseImuData(data_path,
                       imu_name);

  // Set IMU params
  ImuParams imu_params;
  imu_params.acc_walk_ = 1.0;
  imu_params.acc_noise_ = 1.0;
  imu_params.gyro_walk_ = 1.0;
  imu_params.gyro_noise_ = 1.0;
  imu_params.n_gravity_ << 0.0, 0.0, 0.0; // This is needed for online alignment
  imu_params.imu_integration_sigma_ = 1.0;
  Vector3 bias_acc (0.0, 0.0, 0.0);
  Vector3 bias_gyr (0.0, 0.0, 0.0);
  ImuBias imu_bias (bias_acc, bias_gyr);

  // Create ImuFrontEnd
  //ImuFrontEnd imu_frontend(imu_params, imu_bias);

  // Load ground-truth poses
  std::string gt_name = "gt0";
  dataset.parseGTdata(data_path,
                      gt_name);
  GroundTruthData gtData();

  // Get GT poses and IMU pims
  Timestamp timestamp_last_frame;
  Timestamp timestamp_frame_k;
  ImuMeasurements imu_meas;

  // Variables for online alignment
  AlignmentPoses estimated_poses;
  AlignmentPims pims;
  std::vector<double> delta_t_poses;
  estimated_poses.clear();
  pims.clear();
  delta_t_poses.clear();

  // Extract first element in the map
  std::map<long long, gtNavState>::iterator it;
  it = dataset.gtData_.mapToGt_.begin();
  for (int i = 0; i < n_begin_dataset; i++) {
    it++; // Move iterator to desired spot
  }
  timestamp_last_frame = it->first;
  gtsam::Pose3 gt_pose_k = it->second.pose();
  estimated_poses.push_back(gt_pose_k);

  it++; // Move to the second one
  while (it != dataset.gtData_.mapToGt_.end())
  {
    // Get GT information
    timestamp_frame_k = it->first;
    gt_pose_k = it->second.pose();

    // Get PIM information
    dataset.imuData_.imu_buffer_.getImuDataInterpolatedUpperBorder(
          timestamp_last_frame,
          timestamp_frame_k,
          &imu_meas.timestamps_,
          &imu_meas.measurements_);
    ImuFrontEnd imu_frontend(imu_params, imu_bias);
    const auto& pim = imu_frontend.preintegrateImuMeasurements(
                imu_meas.timestamps_, imu_meas.measurements_);

    // Buffer for online alignment
    estimated_poses.push_back(gt_pose_k);
    delta_t_poses.push_back(UtilsOpenCV::NsecToSec(
              timestamp_frame_k - timestamp_last_frame));
    pims.push_back(pim);
    if (pims.size() > n_frames) {
      break;
    }

    // Move to next element in map
    timestamp_last_frame = timestamp_frame_k;
    it++;
  }

  // Initialize OnlineAlignment
  gtsam::Vector3 gyro_bias = imu_bias.gyroscope();
  //gtsam::Vector3 gyro_bias = imu_frontend.getCurrentImuBias().gyroscope();
  gtsam::Vector3 g_iter;
  gtsam::Pose3 init_pose;
  CHECK_DOUBLE_EQ(gyro_bias.norm(), 0.0);
  // imu_params.n_gravity_ << 0.0, 0.0, -9.81; // This is needed for online
  // alignment
  OnlineGravityAlignment initial_alignment(
                      estimated_poses, 
                      delta_t_poses,
                      pims, 
                      imu_params.n_gravity_);

  // Compute Gyroscope Bias
  initial_alignment.alignVisualInertialEstimates(&gyro_bias, &g_iter,
                                                 &init_pose);

  // Final test check
  DOUBLES_EQUAL(bias_gyr.norm(), gyro_bias.norm(), 1e-2);
}

} // End of VIO namespace.

/* ************************************************************************* */
int main(int argc, char *argv[]) {
  // Initialize Google's flags library.
  google::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  TestResult tr; 
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
