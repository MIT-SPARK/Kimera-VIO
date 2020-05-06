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

#include "kimera-vio/backend/RegularVioBackEndParams.h"
#include "kimera-vio/backend/VioBackEnd-definitions.h"
#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VisionFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

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
   * ├── LcdParams.yaml
   * ├── BackendParams.yaml
   * └── FrontendParams.yaml
   *
   * NOTE: If you wish to parse filenames different than the ones above, you can
   * always use the ctor below.
   */
  VioParams(const std::string& params_folder_path);

  /**
   * @brief VioParams constructor which provides freedom to choose the parsed
   * params on a per-file basis.
   *
   * To parse the parameters from a folder with yaml files, use:
   * ```
   * VioParams vio_params;
   * parsePipelineParams(folder_path, vio_params);
   * ```
   * where folder_path is the path to the folder containing all the YAML params,
   * and vio_params is an instance of this class.
   *
   * @param params_folder_path A path to the folder containing YAML files below.
   * @param pipeline_params_filename Pipeline params YAML file name.
   * @param imu_params_filename IMU params YAML file name.
   * @param left_camera_params_filename  Left Camera params YAML file name.
   * @param right_camera_params_filename Right Camera params YAML file name.
   * @param frontend_params_filename Frontend params YAML file name.
   * @param backend_params_filename Backend params YAML file name.
   */
  VioParams(const std::string& params_folder_path,
            const std::string& pipeline_params_filename,
            const std::string& imu_params_filename,
            const std::string& left_cam_params_filename,
            const std::string& right_cam_params_filename,
            const std::string& frontend_params_filename,
            const std::string& lcd_params_filename,
            const std::string& backend_params_filename);
  virtual ~VioParams() = default;

  /**
   * @brief parseYAML
   * @param folder_path Path to the folder containing all VIO params, i.e:
   * - ImuParams
   * - CameraParams
   * - FrontendParams
   * - BackendParams
   * - LcdParams
   *
   * @return true if all parsing went ok.
   */
  bool parseYAML(const std::string& folder_path) override;

  /**
   * @brief print all the parsed parameters.
   */
  void print() const override;

 public:
  // The actual VIO parameters:
  //! Sensor parameters
  ImuParams imu_params_;
  MultiCameraParams camera_params_;
  //! Pipeline Modules paramters
  FrontendParams frontend_params_;
  BackendParams::Ptr backend_params_;
  LoopClosureDetectorParams lcd_params_;
  //! General Pipeline parameters
  FrontendType frontend_type_;
  BackendType backend_type_;
  bool parallel_run_;
  //! Log output path where to store VIO's output in CSV files.
  //! If empty, no output log will be generated.
  std::string log_output_path_;

 protected:
  //! Helper function to parse camera params.
  CameraParams parseCameraParams(const std::string& filename) const;

  bool equals(const PipelineParams& obj) const override {
    const auto& rhs = static_cast<const VioParams&>(obj);
    return imu_params_ == rhs.imu_params_ &&
        camera_params_ == rhs.camera_params_ &&
        frontend_params_ == rhs.frontend_params_ &&
        backend_params_ == rhs.backend_params_ &&
        lcd_params_ == rhs.lcd_params_ &&
        frontend_type_ == rhs.frontend_type_ &&
        backend_type_ == rhs.backend_type_ &&
        parallel_run_ == rhs.parallel_run_ &&
        log_output_path_ == rhs.log_output_path_;
  }

  //! Names of the YAML files with the parameters.
  std::string pipeline_params_filename_;
  std::string imu_params_filename_;
  std::string left_cam_params_filename_;
  std::string right_cam_params_filename_;
  std::string frontend_params_filename_;
  std::string backend_params_filename_;
  std::string lcd_params_filename_;
};

//! Callback called when the VIO pipeline has shut down.
typedef std::function<void()> ShutdownPipelineCallback;
}  // namespace VIO
