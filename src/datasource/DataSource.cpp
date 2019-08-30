/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#include "datasource/DataSource.h"

#include <gflags/gflags.h>

#include "RegularVioBackEndParams.h"
#include "VioBackEndParams.h"
#include "VioFrontEndParams.h"
#include "imu-frontend/ImuFrontEnd-definitions.h"

DEFINE_string(vio_params_path, "", "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path, "",
              "Path to tracker user-defined parameters.");
DEFINE_int32(backend_type, 0,
             "Type of vioBackEnd to use:\n"
             "0: VioBackEnd\n"
             "1: RegularVioBackEnd");

DEFINE_int64(initial_k, 50,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_k, 10000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_string(dataset_path, "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");

namespace VIO {

DataProvider::DataProvider() :
    initial_k_(FLAGS_initial_k),
    final_k_(FLAGS_final_k),
    dataset_path_(FLAGS_dataset_path) {

  CHECK(final_k_ > initial_k_)
      << "Value for final_k (" << final_k_
      << ") is smaller than value for"
      << " initial_k (" << initial_k_ << ").";

  LOG(INFO) << "Running dataset between frame " << initial_k_
          << " and frame " << final_k_;
}

DataProvider::~DataProvider() {
  LOG(INFO) << "Data provider destructor called.";
}

void DataProvider::registerVioCallback(
    std::function<void(const StereoImuSyncPacket&)> callback) {
  vio_callback_ = std::move(callback);
}

bool DataProvider::spin() {
  // Dummy example:
  // 1) Check that the vio_callback_ has been registered, aka that the user has
  // called the function registerVioCallback, in order to store the callback
  // function.
  CHECK(vio_callback_);

  // 2) Loop over the dataset and:
  //  a) Create StereoImuSyncPacket packets out of the data.
  //  This one is dummy since it is filled with empty images, parameters,
  //  imu data, etc.
  StereoImuSyncPacket vio_packet(
      StereoFrame(1, 1, cv::Mat(), CameraParams(), cv::Mat(), CameraParams(),
                  gtsam::Pose3(), StereoMatchingParams()),
      ImuStampS(), ImuAccGyrS());
  //  b) Call the vio callback in order to start processing the packet.
  vio_callback_(vio_packet);

  // 3) Once the dataset spin has finished, exit.
  // You can return false if something went wrong.
  return true;
}

/* -------------------------------------------------------------------------- */
void DataProvider::parseBackendParams() {
  switch (FLAGS_backend_type) {
    case 0: {
      pipeline_params_.backend_params_ = std::make_shared<VioBackEndParams>();
      break;
    }
    case 1: {
      pipeline_params_.backend_params_ = std::make_shared<RegularVioBackEndParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend type: " << FLAGS_backend_type << "."
                   << " 0: normalVio, 1: RegularVio.";
    }
  }

  pipeline_params_.backend_type_ = FLAGS_backend_type;

  // Read/define vio params.
  if (FLAGS_vio_params_path.empty()) {
    LOG(WARNING) << "No vio parameters specified, using default.";
    // Default params with IMU stats from dataset.
    pipeline_params_.backend_params_->gyroNoiseDensity_ = pipeline_params_.imu_params_.gyro_noise_;
    pipeline_params_.backend_params_->accNoiseDensity_ = pipeline_params_.imu_params_.acc_noise_;
    pipeline_params_.backend_params_->gyroBiasSigma_ = pipeline_params_.imu_params_.gyro_walk_;
    pipeline_params_.backend_params_->accBiasSigma_ = pipeline_params_.imu_params_.acc_walk_;
  } else {
    VLOG(100) << "Using user-specified VIO parameters: "
              << FLAGS_vio_params_path;
    pipeline_params_.backend_params_->parseYAML(FLAGS_vio_params_path);
  }
  // TODO(Toni) make this cleaner! pipeline_params_.imu_params_ are parsed all
  // around, it's a mess!! They are basically parsed from backend params... but
  // they should be on their own mostly. Here we just override gravity and
  // imu_integration sigma from backend_params_ But one may be able to overwrite
  // all of them using backend_params_
  CHECK_DOUBLE_EQ(pipeline_params_.imu_params_.acc_walk_,
                  pipeline_params_.backend_params_->accBiasSigma_);
  CHECK_DOUBLE_EQ(pipeline_params_.imu_params_.acc_noise_,
                  pipeline_params_.backend_params_->accNoiseDensity_);
  CHECK_DOUBLE_EQ(pipeline_params_.imu_params_.gyro_walk_,
                  pipeline_params_.backend_params_->gyroBiasSigma_);
  CHECK_DOUBLE_EQ(pipeline_params_.imu_params_.gyro_noise_,
                  pipeline_params_.backend_params_->gyroNoiseDensity_);
  pipeline_params_.imu_params_.imu_integration_sigma_ =
      pipeline_params_.backend_params_->imuIntegrationSigma_;
  pipeline_params_.imu_params_.n_gravity_ =
      pipeline_params_.backend_params_->n_gravity_;
  CHECK(pipeline_params_.backend_params_);
}

/* -------------------------------------------------------------------------- */
void DataProvider::parseFrontendParams() {
  // Read/define tracker params.
  if (FLAGS_tracker_params_path.empty()) {
    LOG(WARNING) << "No tracker parameters specified, using default";
    pipeline_params_.frontend_params_ = VioFrontEndParams();  // default params
  } else {
    VLOG(100) << "Using user-specified tracker parameters: "
              << FLAGS_tracker_params_path;
    pipeline_params_.frontend_params_.parseYAML(FLAGS_tracker_params_path);
  }
}

}  // namespace VIO
