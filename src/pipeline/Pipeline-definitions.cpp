/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline-definitions.h
 * @brief  Definitions for VIO pipeline.
 * @author Antoni Rosinol
 */

#include "kimera-vio/pipeline/Pipeline-definitions.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/visualizer/OpenCvDisplay.h"  // for ocv display params...

DEFINE_bool(use_external_odometry, false, "Use an external odometry input.");

namespace VIO {

decltype(VioParams::kPipelineFilename) constexpr VioParams::kPipelineFilename;
decltype(VioParams::kImuFilename) constexpr VioParams::kImuFilename;
decltype(
    VioParams::kLeftCameraFilename) constexpr VioParams::kLeftCameraFilename;
decltype(
    VioParams::kRightCameraFilename) constexpr VioParams::kRightCameraFilename;
decltype(VioParams::kFrontendFilename) constexpr VioParams::kFrontendFilename;
decltype(VioParams::kBackendFilename) constexpr VioParams::kBackendFilename;
decltype(VioParams::kLcdFilename) constexpr VioParams::kLcdFilename;
decltype(VioParams::kDisplayFilename) constexpr VioParams::kDisplayFilename;
decltype(VioParams::kOdometryFilename) constexpr VioParams::kOdometryFilename;

VioParams::VioParams(const std::string& params_folder_path)
    : VioParams(params_folder_path + '/' + kPipelineFilename,
                params_folder_path + '/' + kImuFilename,
                params_folder_path + '/' + kLeftCameraFilename,
                params_folder_path + '/' + kRightCameraFilename,
                params_folder_path + '/' + kFrontendFilename,
                params_folder_path + '/' + kBackendFilename,
                params_folder_path + '/' + kLcdFilename,
                params_folder_path + '/' + kDisplayFilename,
                FLAGS_use_external_odometry
                    ? params_folder_path + '/' + kOdometryFilename
                    : "",
                !params_folder_path.empty()) {}

VioParams::VioParams(const std::string& params_folder_path,
                     const std::string& sensor_folder_path)
    : VioParams(params_folder_path + '/' + kPipelineFilename,
                sensor_folder_path + '/' + kImuFilename,
                sensor_folder_path + '/' + kLeftCameraFilename,
                sensor_folder_path + '/' + kRightCameraFilename,
                params_folder_path + '/' + kFrontendFilename,
                params_folder_path + '/' + kBackendFilename,
                params_folder_path + '/' + kLcdFilename,
                params_folder_path + '/' + kDisplayFilename,
                FLAGS_use_external_odometry
                    ? sensor_folder_path + '/' + kOdometryFilename
                    : "",
                !params_folder_path.empty()) {}


VioParams::VioParams(const std::string& pipeline_params_filepath,
                     const std::string& imu_params_filepath,
                     const std::string& left_cam_params_filepath,
                     const std::string& right_cam_params_filepath,
                     const std::string& frontend_params_filepath,
                     const std::string& backend_params_filepath,
                     const std::string& lcd_params_filepath,
                     const std::string& display_params_filepath,
                     const std::string& odom_params_filepath,
                     bool should_parse)
    : PipelineParams("VioParams"),
      // Actual VIO Parameters
      imu_params_(),
      camera_params_(),
      frontend_params_(),
      backend_params_(std::make_shared<BackendParams>()),
      lcd_params_(),
      display_params_(std::make_shared<DisplayParams>(DisplayType::kOpenCV)),
      frontend_type_(FrontendType::kStereoImu),
      backend_type_(BackendType::kStructuralRegularities),
      parallel_run_(true),
      // Filepaths, keep defaults unless you changed file names.
      pipeline_params_filepath_(pipeline_params_filepath),
      imu_params_filepath_(imu_params_filepath),
      left_cam_params_filepath_(left_cam_params_filepath),
      right_cam_params_filepath_(right_cam_params_filepath),
      frontend_params_filepath_(frontend_params_filepath),
      backend_params_filepath_(backend_params_filepath),
      lcd_params_filepath_(lcd_params_filepath),
      display_params_filepath_(display_params_filepath),
      odom_params_filepath_(odom_params_filepath) {
  if (should_parse) {
    parseYAML("");
  } else {
    LOG(WARNING)
        << "Calling VioParams constructor without parsing parameters..."
           "Using default Vio Parameters.";
  }
}

bool VioParams::parseYAML(const std::string&) {
  // Create a parser for pipeline params.
  YamlParser yaml_parser(pipeline_params_filepath_);
  int backend_type;
  yaml_parser.getYamlParam("backend_type", &backend_type);
  backend_type_ = static_cast<BackendType>(backend_type);
  int frontend_type;
  yaml_parser.getYamlParam("frontend_type", &frontend_type);
  frontend_type_ = static_cast<FrontendType>(frontend_type);
  int display_type;
  yaml_parser.getYamlParam("display_type", &display_type);
  display_type_ = static_cast<DisplayType>(display_type);
  yaml_parser.getYamlParam("parallel_run", &parallel_run_);

  // Parse IMU params
  parsePipelineParams(imu_params_filepath_, &imu_params_);

  // Parse Camera parameters
  camera_params_.push_back(parseCameraParams(left_cam_params_filepath_));
  if (frontend_type_ == FrontendType::kStereoImu) {
    camera_params_.push_back(parseCameraParams(right_cam_params_filepath_));
  }

  // Parse Backend params, needs a bit of help with backend_type
  switch (backend_type_) {
    case BackendType::kStereoImu: {
      backend_params_ = std::make_shared<BackendParams>();
      break;
    }
    case BackendType::kStructuralRegularities: {
      backend_params_ = std::make_shared<RegularVioBackendParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized Backend type: "
                 << static_cast<int>(backend_type_) << "."
                 << " 0: normalVio, 1: RegularVio.";
    }
  }
  CHECK(backend_params_);
  parsePipelineParams(backend_params_filepath_, backend_params_.get());

  // Parse Frontend params.
  parsePipelineParams(frontend_params_filepath_, &frontend_params_);

  // Parse LcdParams
  parsePipelineParams(lcd_params_filepath_, &lcd_params_);

  // Parse DisplayParams
  switch (display_type_) {
    case DisplayType::kOpenCV: {
      display_params_ = std::make_shared<OpenCv3dDisplayParams>();
      break;
    }
    case DisplayType::kPangolin: {
      // We don't have Pangolin specific params so far...
      display_params_ = std::make_shared<DisplayParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized display type: "
                 << static_cast<int>(display_type_) << "."
                 << " 0: OpenCV, 1: Pangolin.";
    }
  }
  CHECK(display_params_);
  parsePipelineParams(display_params_filepath_, display_params_.get());
  display_params_->display_type_ = display_type_;

  if (!odom_params_filepath_.empty()) {
    OdometryParams odom_params;
    parsePipelineParams(odom_params_filepath_, &odom_params);
    odom_params_ = odom_params;
  }

  return true;
}

void VioParams::print() const {
  LOG(INFO) << std::string(10, '*') << " VIO PARAMS " << std::string(10, '*');
  imu_params_.print();
  for (const auto& cam : camera_params_) cam.print();
  frontend_params_.print();
  CHECK(backend_params_);
  backend_params_->print();
  lcd_params_.print();
  CHECK(display_params_);
  display_params_->print();
  LOG(INFO) << "Frontend Type: " << VIO::to_underlying(frontend_type_);
  LOG(INFO) << "Backend Type: " << VIO::to_underlying(backend_type_);
  LOG(INFO) << "Display Type: " << VIO::to_underlying(display_type_);
  LOG(INFO) << "Running VIO in " << (parallel_run_ ? "parallel" : "sequential")
            << " mode.";
}

//! Helper function to parse camera params.
CameraParams VioParams::parseCameraParams(const std::string& filename) const {
  CameraParams camera_params;
  parsePipelineParams(filename, &camera_params);
  return camera_params;
}
}  // namespace VIO
