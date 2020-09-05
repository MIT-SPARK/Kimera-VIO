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

#include "kimera-vio/backend/RegularVioBackEndParams.h"
#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

namespace VIO {

VioParams::VioParams(const std::string& params_folder_path)
    : VioParams(params_folder_path,
                "PipelineParams.yaml",
                "ImuParams.yaml",
                "LeftCameraParams.yaml",
                "RightCameraParams.yaml",
                "FrontendParams.yaml",
                "BackendParams.yaml",
                "LcdParams.yaml") {}

VioParams::VioParams(const std::string& params_folder_path,
                     const std::string& pipeline_params_filename,
                     const std::string& imu_params_filename,
                     const std::string& left_cam_params_filename,
                     const std::string& right_cam_params_filename,
                     const std::string& frontend_params_filename,
                     const std::string& backend_params_filename,
                     const std::string& lcd_params_filename)
    : PipelineParams("VioParams"),
      // Actual VIO Parameters
      imu_params_(),
      camera_params_(),
      frontend_params_(),
      backend_params_(std::make_shared<BackendParams>()),
      lcd_params_(),
      frontend_type_(FrontendType::kStereoImu),
      backend_type_(BackendType::kStructuralRegularities),
      parallel_run_(true),
      // Filepaths, keep defaults unless you changed file names.
      pipeline_params_filename_(pipeline_params_filename),
      imu_params_filename_(imu_params_filename),
      left_cam_params_filename_(left_cam_params_filename),
      right_cam_params_filename_(right_cam_params_filename),
      frontend_params_filename_(frontend_params_filename),
      backend_params_filename_(backend_params_filename),
      lcd_params_filename_(lcd_params_filename) {
  if (!params_folder_path.empty()) {
    parseYAML(params_folder_path);
  } else {
    LOG(WARNING)
        << "Calling VioParams constructor without parsing parameters..."
           "Using default Vio Parameters.";
  }
}

bool VioParams::parseYAML(const std::string& folder_path) {
  // Create a parser for pipeline params.
  YamlParser yaml_parser(folder_path + '/' + pipeline_params_filename_);
  int backend_type;
  yaml_parser.getYamlParam("backend_type", &backend_type);
  backend_type_ = static_cast<BackendType>(backend_type);
  int frontend_type;
  yaml_parser.getYamlParam("frontend_type", &frontend_type);
  frontend_type_ = static_cast<FrontendType>(frontend_type);
  yaml_parser.getYamlParam("parallel_run", &parallel_run_);

  // Parse IMU params
  parsePipelineParams(folder_path + '/' + imu_params_filename_, &imu_params_);

  // Parse Camera parameters
  camera_params_.push_back(
      parseCameraParams(folder_path + '/' + left_cam_params_filename_));
  camera_params_.push_back(
      parseCameraParams(folder_path + '/' + right_cam_params_filename_));

  // Parse backend params, needs a bit of help with backend_type
  switch (backend_type_) {
    case BackendType::kStereoImu: {
      backend_params_ = std::make_shared<BackendParams>();
      break;
    }
    case BackendType::kStructuralRegularities: {
      backend_params_ = std::make_shared<RegularVioBackEndParams>();
      break;
    }
    default: {
      LOG(FATAL) << "Unrecognized backend type: "
                 << static_cast<int>(backend_type_) << "."
                 << " 0: normalVio, 1: RegularVio.";
    }
  }
  CHECK(backend_params_);
  parsePipelineParams(folder_path + '/' + backend_params_filename_,
                      backend_params_.get());

  // Parse frontend params.
  parsePipelineParams(folder_path + '/' + frontend_params_filename_,
                      &frontend_params_);

  // Parse LcdParams
  parsePipelineParams(folder_path + '/' + lcd_params_filename_, &lcd_params_);

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
  LOG(INFO) << "Frontend Type: " << VIO::to_underlying(frontend_type_);
  LOG(INFO) << "Backend Type: " << VIO::to_underlying(backend_type_);
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
