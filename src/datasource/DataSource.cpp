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

#include "kimera-vio/datasource/DataSource.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/backend/RegularVioBackEndParams.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEnd-definitions.h"

DEFINE_string(vio_params_path, "", "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path,
              "",
              "Path to tracker user-defined parameters.");
DEFINE_string(lcd_params_path,
              "",
              "Path to loop-closure-detector user-defined parameters.");
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

DataProviderInterface::DataProviderInterface(int initial_k,
                                             int final_k,
                                             const std::string& dataset_path)
    : pipeline_params_(),
      initial_k_(initial_k),
      final_k_(final_k),
      dataset_path_(dataset_path) {
  CHECK(final_k_ > initial_k_)
      << "Value for final_k (" << final_k_ << ") is smaller than value for"
      << " initial_k (" << initial_k_ << ").";
  LOG(INFO) << "Running dataset between frame " << initial_k_ << " and frame "
            << final_k_;
}

DataProviderInterface::DataProviderInterface()
    : DataProviderInterface(FLAGS_initial_k,
                            FLAGS_final_k,
                            FLAGS_dataset_path) {}

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
      VIO::make_unique<Frame>(0, 0, CameraParams("left_cam"), cv::Mat()));
  right_frame_callback_(
      VIO::make_unique<Frame>(0, 0, CameraParams("right_cam"), cv::Mat()));
  //! Usually you would use only one of these
  imu_single_callback_(ImuMeasurement());
  imu_multi_callback_(ImuMeasurements());

  // 3) Once the dataset spin has finished, exit.
  // You can return false if something went wrong.
  return true;
}

/* -------------------------------------------------------------------------- */
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
  if (FLAGS_vio_params_path.empty()) {
    LOG(WARNING) << "No vio parameters specified, using default.";
  } else {
    VLOG(100) << "Using user-specified VIO parameters: "
              << FLAGS_vio_params_path;
    pipeline_params_.backend_params_->parseYAML(FLAGS_vio_params_path);
  }
  CHECK(pipeline_params_.backend_params_);
}

/* -------------------------------------------------------------------------- */
void DataProviderInterface::parseFrontendParams() {
  pipeline_params_.frontend_type_ =
      static_cast<FrontendType>(FLAGS_frontend_type);
  // Read/define tracker params.
  if (FLAGS_tracker_params_path.empty()) {
    LOG(WARNING) << "No tracker parameters specified, using default";
    pipeline_params_.frontend_params_ = VioFrontEndParams();  // default params
  } else {
    VLOG(100) << "Using user-specified tracker parameters: "
              << FLAGS_tracker_params_path;
    pipeline_params_.frontend_params_.parseYAML(FLAGS_tracker_params_path);
  }
  CHECK_NOTNULL(&pipeline_params_.frontend_params_);
}

bool DataProviderInterface::parseImuParams(const std::string& imu_yaml_path,
                                           ImuParams* imu_params) {
  CHECK_NOTNULL(imu_params);

  YamlParser::Ptr yaml_parser_ = std::make_shared<YamlParser>(imu_yaml_path);

  // Rows and cols are redundant info, since the pose 4x4, but we parse just
  // to check we are all on the same page.
  // TODO(Toni): don't use the getYamlFileStorage function...
  cv::FileStorage fs = yaml_parser_->getYamlFileStorage();
  CHECK_EQ(static_cast<int>(fs["T_BS"]["rows"]), 4u);
  CHECK_EQ(static_cast<int>(fs["T_BS"]["cols"]), 4u);
  std::vector<double> vector_pose;
  fs["T_BS"]["data"] >> vector_pose;
  const gtsam::Pose3& body_Pose_cam =
      UtilsOpenCV::poseVectorToGtsamPose3(vector_pose);

  // Sanity check: IMU is usually chosen as the body frame.
  LOG_IF(FATAL, !body_Pose_cam.equals(gtsam::Pose3()))
      << "parseImuData: we expected identity body_Pose_cam_: is everything ok?";

  int rate = 0;
  yaml_parser_->getYamlParam("rate_hz", &rate);
  CHECK_GT(rate, 0u);
  imu_params->nominal_rate_ = 1 / static_cast<double>(rate);

  // IMU PARAMS
  yaml_parser_->getYamlParam("gyroscope_noise_density",
                             &imu_params->gyro_noise_);
  yaml_parser_->getYamlParam("accelerometer_noise_density",
                             &imu_params->acc_noise_);
  yaml_parser_->getYamlParam("gyroscope_random_walk", &imu_params->gyro_walk_);
  yaml_parser_->getYamlParam("accelerometer_random_walk",
                             &imu_params->acc_walk_);
  std::vector<double> n_gravity;
  yaml_parser_->getYamlParam("n_gravity", &n_gravity);
  CHECK_EQ(n_gravity.size(), 3);
  for (int k = 0; k < 3; k++) imu_params->n_gravity_(k) = n_gravity[k];

  fs.release();
  return true;
}

void DataProviderInterface::parseLCDParams() {
  // Read/define LCD params.
  if (FLAGS_lcd_params_path.empty()) {
    VLOG(100) << "No LoopClosureDetector parameters specified, using default";
    pipeline_params_.lcd_params_ = LoopClosureDetectorParams();
  } else {
    VLOG(100) << "Using user-specified LoopClosureDetector parameters: "
              << FLAGS_lcd_params_path;
    pipeline_params_.lcd_params_.parseYAML(FLAGS_lcd_params_path);
  }
  CHECK_NOTNULL(&pipeline_params_.lcd_params_);
}

}  // namespace VIO
