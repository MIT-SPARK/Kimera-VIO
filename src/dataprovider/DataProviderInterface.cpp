/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderInterface.cpp
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/dataprovider/DataProviderInterface.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/backend/RegularVioBackEndParams.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"
#include "kimera-vio/pipeline/PipelineParams.h"

DEFINE_bool(parallel_run, true, "Run VIO parallel or sequential");
DEFINE_string(left_cam_params_path, "", "Path to Left Camera parameters.");
DEFINE_string(right_cam_params_path, "", "Path to Right Camera parameters.");
DEFINE_string(imu_params_path, "", "Path to IMU parameters.");
DEFINE_string(backend_params_path, "", "Path to VIO backend parameters.");
DEFINE_string(frontend_params_path, "", "Path to VIO frontend parameters.");
DEFINE_string(lcd_params_path, "", "Path to loop-closure-detector parameters.");
DEFINE_int32(backend_type,
             0,
             "Type of vioBackEnd to use:\n"
             "0: VioBackEnd\n"
             "1: RegularVioBackEnd");
DEFINE_int32(frontend_type,
             0,
             "Type of VIO Frontend to use:\n"
             "0: StereoImu");
DEFINE_int64(initial_k,
             50,
             "Initial frame to start processing dataset, "
             "previous frames will not be used.");
DEFINE_int64(final_k,
             10000,
             "Final frame to finish processing dataset, "
             "subsequent frames will not be used.");
DEFINE_string(dataset_path,
              "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");

namespace VIO {

DataProviderInterface::DataProviderInterface(
    const int& initial_k,
    const int& final_k,
    const bool& parallel_run,
    const std::string& dataset_path,
    const std::string& left_cam_params_path,
    const std::string& right_cam_params_path,
    const std::string& imu_params_path,
    const std::string& backend_params_path,
    const std::string& frontend_params_path,
    const std::string& lcd_params_path)
    : pipeline_params_(),
      initial_k_(initial_k),
      final_k_(final_k),
      dataset_path_(dataset_path),
      left_cam_params_path_(left_cam_params_path),
      right_cam_params_path_(right_cam_params_path),
      imu_params_path_(imu_params_path),
      backend_params_path_(backend_params_path),
      frontend_params_path_(frontend_params_path),
      lcd_params_path_(lcd_params_path) {
  parseParams();

  // Get whether we run the pipeline in parallel mode or in sequential mode;
  pipeline_params_.parallel_run_ = parallel_run;

  // Start processing dataset from frame initial_k.
  // Useful to skip a bunch of images at the beginning (imu calibration).
  CHECK_GE(initial_k_, 0);
  CHECK_GE(initial_k_, 10)
      << "initial_k should be >= 10 for IMU bias initialization";

  // Finish processing dataset at frame final_k.
  // Last frame to process (to avoid processing the entire dataset),
  // skip last frames.
  CHECK_GT(final_k_, 0);

  CHECK(final_k_ > initial_k_)
      << "Value for final_k (" << final_k_ << ") is smaller than value for"
      << " initial_k (" << initial_k_ << ").";
  LOG(INFO) << "Running dataset between frame " << initial_k_ << " and frame "
            << final_k_;
}

DataProviderInterface::DataProviderInterface()
    : DataProviderInterface(FLAGS_initial_k,
                            FLAGS_final_k,
                            FLAGS_parallel_run,
                            FLAGS_dataset_path,
                            FLAGS_left_cam_params_path,
                            FLAGS_right_cam_params_path,
                            FLAGS_imu_params_path,
                            FLAGS_backend_params_path,
                            FLAGS_frontend_params_path,
                            FLAGS_lcd_params_path) {}

DataProviderInterface::~DataProviderInterface() {
  LOG(INFO) << "Data provider destructor called.";
}

bool DataProviderInterface::spin() {
  // Dummy example:
  // 1) Check that the callbacks have been registered, aka that the user has
  // called the function registerVioCallback, in order to store the callback
  // function.
  //! Check only the imu callback that you want to use:
  //! - single: only one message at a time.
  //! - multi: multiple message at a time.
  CHECK(imu_single_callback_);
  CHECK(imu_multi_callback_);
  CHECK(left_frame_callback_);
  CHECK(right_frame_callback_);

  // 2) Loop over the dataset and:
  //  a) Create data packets out of the data.
  //  This one is dummy since it is filled with empty images, parameters,
  //  imu data, etc.
  //  b) Call the callbacks in order to send the data.
  left_frame_callback_(
      VIO::make_unique<Frame>(0, 0, CameraParams(), cv::Mat()));
  right_frame_callback_(
      VIO::make_unique<Frame>(0, 0, CameraParams(), cv::Mat()));
  //! Usually you would use only one of these
  imu_single_callback_(ImuMeasurement());
  imu_multi_callback_(ImuMeasurements());

  // 3) Once the dataset spin has finished, exit with false.
  return false;
}

void DataProviderInterface::parseParams() {
  // Parse Sensor parameters
  pipeline_params_.camera_params_.push_back(
      parseCameraParams(left_cam_params_path_));
  pipeline_params_.camera_params_.push_back(
      parseCameraParams(right_cam_params_path_));
  parseImuParams();

  // Parse backend/frontend/lcd parameters
  parseBackendParams();
  parseFrontendParams();
  parseLCDParams();
}

void DataProviderInterface::parseBackendParams() {
  pipeline_params_.backend_type_ = static_cast<BackendType>(FLAGS_backend_type);
  switch (pipeline_params_.backend_type_) {
    case BackendType::kStereoImu: {
      pipeline_params_.backend_params_ = std::make_shared<VioBackEndParams>();
      break;
    }
    case BackendType::kStructuralRegularities: {
      pipeline_params_.backend_params_ =
          std::make_shared<RegularVioBackEndParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend type: "
                 << static_cast<int>(pipeline_params_.backend_type_) << "."
                 << " 0: normalVio, 1: RegularVio.";
    }
  }

  // Read/define vio params.
  CHECK(pipeline_params_.backend_params_);
  parsePipelineParams(backend_params_path_,
                      // TODO(Toni): a bit ugly this get()
                      pipeline_params_.backend_params_.get());
}

void DataProviderInterface::parseFrontendParams() {
  pipeline_params_.frontend_type_ =
      static_cast<FrontendType>(FLAGS_frontend_type);
  // Read/define tracker params.
  parsePipelineParams(frontend_params_path_,
                      &pipeline_params_.frontend_params_);
}

void DataProviderInterface::parseLCDParams() {
  parsePipelineParams(lcd_params_path_, &pipeline_params_.lcd_params_);
}

// Parse camera params for a given dataset
CameraParams DataProviderInterface::parseCameraParams(
    const std::string& filename) {
  CameraParams camera_params;
  parsePipelineParams(filename, &camera_params);
  return camera_params;
}

void DataProviderInterface::parseImuParams() {
  parsePipelineParams(imu_params_path_, &pipeline_params_.imu_params_);
}

}  // namespace VIO
