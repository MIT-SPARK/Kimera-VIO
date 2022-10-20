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

#pragma once

#include <gflags/gflags.h>

#include "kimera-vio/backend/RegularVioBackendParams.h"
#include "kimera-vio/backend/VioBackend-definitions.h"
#include "kimera-vio/backend/VioBackendParams.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/OdometryParams.h"
#include "kimera-vio/frontend/VisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontendParams.h"
#include "kimera-vio/imu-frontend/ImuFrontendParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"
#include "kimera-vio/visualizer/DisplayParams.h"

DECLARE_bool(use_external_odometry);

namespace VIO {

/**
 * @brief The VioParams struct
 * Contains all necessary parameters for the VIO pipeline to run.
 */
struct VioParams : public PipelineParams {
  KIMERA_POINTER_TYPEDEFS(VioParams);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief VioParams Default constructor
   * @param params_folder_path if left empty ("") no params will be parsed, and
   * it returns default values for all parameters. If a path is passed, then it
   * will parse the params from inside the given folder path. It assumes the
   * following system layout inside params_folder_path.
   *
   * Default filesystem layout:
   *  where params_folder_path = "Euroc" in this example
   *
   * Euroc/
   * ├── ImuParams.yaml
   * ├── LeftCameraParams.yaml
   * ├── RightCameraParams.yaml
   * ├── FrontendParams.yaml
   * ├── BackendParams.yaml
   * ├── LcdParams.yaml
   * └── DisplayParams.yaml
   *
   * NOTE: If you wish to parse filenames different than the ones above, you can
   * always use the ctor below.
   */
  explicit VioParams(const std::string& params_folder_path);

  /**
   * @brief VioParams constructor which provides freedom to choose the parsed
   * params on a per-config basis.
   *
   * To parse the parameters from a folder with yaml files, use:
   * ```
   * VioParams vio_params(folder_path);
   * ```
   * where folder_path is the path to the folder containing all the YAML params.
   *
   * @param imu_params_filepath IMU params YAML file path.
   * @param left_camera_params_filepath  Left Camera params YAML file path.
   * @param right_camera_params_filepath Right Camera params YAML file path.
   * @param frontend_params_filepath Frontend params YAML file path.
   * @param backend_params_filepath Backend params YAML file path.
   * @param lcd_params_filepath Loop closure params YAML file path.
   * @param display_params_filepath Display params YAML file path.
   * @param pipeline_params_filepath Pipeline params YAML file path.
   * @param odom_params_filepath External odometry params file path
   * @param should_parse Whether or not to attempt to read from the YAML files
   */
  VioParams(const std::string& pipeline_params_filepath,
            const std::string& imu_params_filepath,
            const std::string& left_cam_params_filepath,
            const std::string& right_cam_params_filepath,
            const std::string& frontend_params_filepath,
            const std::string& backend_params_filepath,
            const std::string& lcd_params_filepath,
            const std::string& display_params_filepath,
            boost::optional<std::string> odom_params_filepath,
            bool should_parse = true);

  virtual ~VioParams() = default;

  /**
   * @brief parseYAML
   * - ImuParams
   * - CameraParams
   * - FrontendParams
   * - BackendParams
   * - LcdParams
   * - DisplayParams
   * - ExternalOdometryParams
   *
   * @return true if all parsing went ok.
   */
  bool parseYAML(const std::string&) override;

  /**
   * @brief print all the parsed parameters.
   */
  void print() const override;

 public:
  static constexpr char kPipelineFilename[] = "PipelineParams.yaml";
  static constexpr char kImuFilename[] = "ImuParams.yaml";
  static constexpr char kLeftCameraFilename[] = "LeftCameraParams.yaml";
  static constexpr char kRightCameraFilename[] = "RightCameraParams.yaml";
  static constexpr char kFrontendFilename[] = "FrontendParams.yaml";
  static constexpr char kBackendFilename[] = "BackendParams.yaml";
  static constexpr char kLcdFilename[] = "LcdParams.yaml";
  static constexpr char kDisplayFilename[] = "DisplayParams.yaml";
  static constexpr char kOdometryFilename[] = "ExternalOdometryParams.yaml";

  // The actual VIO parameters:
  //! Sensor parameters
  ImuParams imu_params_;
  MultiCameraParams camera_params_;
  //! Pipeline Modules paramters
  FrontendParams frontend_params_;
  //! Mind that this is shared btw the vio pipeline and dataprovider,
  //!  so that any changes to this pointer will affect both.
  BackendParams::Ptr backend_params_;
  LoopClosureDetectorParams lcd_params_;
  DisplayParams::Ptr display_params_;
  //! General Pipeline parameters
  FrontendType frontend_type_;
  BackendType backend_type_;
  DisplayType display_type_;
  boost::optional<OdometryParams> odom_params_;
  bool parallel_run_;

 protected:
  //! Helper function to parse camera params.
  CameraParams parseCameraParams(const std::string& filename) const;

  bool equals(const PipelineParams& obj) const override {
    const auto& rhs = static_cast<const VioParams&>(obj);
    return imu_params_ == rhs.imu_params_ &&
           camera_params_ == rhs.camera_params_ &&
           frontend_params_ == rhs.frontend_params_ &&
           backend_params_ == rhs.backend_params_ &&
           frontend_type_ == rhs.frontend_type_ &&
           backend_type_ == rhs.backend_type_ &&
           display_type_ == rhs.display_type_ &&
           lcd_params_ == rhs.lcd_params_ &&
           display_params_ == rhs.display_params_ &&
           parallel_run_ == rhs.parallel_run_;
  }

  //! Names of the YAML files with the parameters.
  std::string pipeline_params_filepath_;
  std::string imu_params_filepath_;
  std::string left_cam_params_filepath_;
  std::string right_cam_params_filepath_;
  std::string frontend_params_filepath_;
  std::string backend_params_filepath_;
  std::string lcd_params_filepath_;
  std::string display_params_filepath_;
  boost::optional<std::string> odom_params_filepath_;
};

//! Callback called when the VIO pipeline has shut down.
typedef std::function<void()> ShutdownPipelineCallback;
}  // namespace VIO
