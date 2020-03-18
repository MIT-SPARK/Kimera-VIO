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
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/imu-frontend/ImuFrontEndParams.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

/**
 * @brief The VioParams struct
 * Contains all necessary parameters for the VIO pipeline to run.
 */
struct VioParams : public PipelineParams {
  /**
   * @brief VioParams ctor
   * @param use_gflags true if you want to parse parameters using gflags
   *  or false if you just want a default VioParams class.
   */
  VioParams(const bool& use_gflags);

  /**
   * @brief VioParams constructor: typically used without arguments.
   *
   * @param frontend_type: the frontend to be used
   * @param backend_type: the backend to be used
   * @param parallel_run: if the pipeline should be run in parallel or
   * sequential mode
   * Params below are all paths.
   *
   * To parse the parameters, use:
   * ```
   * VioParams vio_params;
   * parsePipelineParams(folder_path, vio_params);
   * ```
   * where folder_path is the path to the folder containing all the YAML params,
   * and vio_params is an instance of this class.
   *
   * Parameters below are optional and default to the following filesystem
   * layout: where folder_path = "Euroc" here, but can be changed when calling
   * parseYAML or the static function parsePipelineParams() on this struct (i.e.
   * Euroc/
   * ├── ImuParams.yaml
   * ├── LeftCameraParams.yaml
   * ├── RightCameraParams.yaml
   * ├── LcdParams.yaml
   * ├── BackendParams.yaml
   * └── FrontendParams.yaml
   *
   * NOTE: If you wish to parse filenames different than the ones above, you can
   * always override the defaults below:
   *
   * @param imu_params_filename IMU params YAML file
   * @param left_camera_params_filename  Left Camera params YAML file
   * @param right_camera_params_filename Right Camera params YAML file
   * @param frontend_params_filename Frontend params YAML file
   * @param backend_params_filename Backend params YAML file
   */
  VioParams(const FrontendType& frontend_type,
            const BackendType& backend_type,
            const bool& parallel_run,
            const std::string& params_folder_path,
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
  VioFrontEndParams frontend_params_;
  VioBackEndParams::Ptr backend_params_;
  LoopClosureDetectorParams lcd_params_;
  //! General Pipeline parameters
  FrontendType frontend_type_;
  BackendType backend_type_;
  bool parallel_run_;

 protected:
  //! Helper function to parse camera params.
  CameraParams parseCameraParams(const std::string& filename) const;

  //! Names of the YAML files with the parameters.
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
