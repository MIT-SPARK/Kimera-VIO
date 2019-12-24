/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataProviderInterface.h
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>

#include "kimera-vio/frontend/Camera.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(DataProviderInterface);
  //! IMU input callbacks come in two flavours:
  //! - Single: allows to add only one single measurement at a time.
  //! - Multi: allows to add a bunck of measurements at a time.
  typedef std::function<void(const ImuMeasurement&)> ImuSingleInputCallback;
  typedef std::function<void(const ImuMeasurements&)> ImuMultiInputCallback;
  typedef std::function<void(Frame::UniquePtr)> FrameInputCallback;

  /**
   * @brief DataProviderInterface
   * @param initial_k
   * @param final_k
   * @param parallel_run: if the pipeline should be run in parallel or
   * sequential mode
   * Params below are all paths.
   * @param dataset_path
   * @param left_cam_params_path
   * @param right_cam_params_path
   * @param imu_params_path
   * @param backend_params_path
   * @param frontend_params_path
   * @param lcd_params_path
   */
  DataProviderInterface(const int& initial_k,
                        const int& final_k,
                        const bool& parallel_run,
                        const std::string& dataset_path,
                        const std::string& left_cam_params_path,
                        const std::string& right_cam_params_path,
                        const std::string& imu_params_path,
                        const std::string& backend_params_path,
                        const std::string& frontend_params_path,
                        const std::string& lcd_params_path);
  // Ctor from gflags. Calls regular ctor with gflags values.
  DataProviderInterface();
  virtual ~DataProviderInterface();

  // The derived classes need to implement this function!
  /**
   * @brief spin Spins the dataset until it finishes.
   * - If set in sequential mode, it should return each time a frame is sent.
   * - In parallel mode, it should not return until it finishes.
   * A dummy example is provided as implementation (instead of being pure
   * virtual)
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin();

  // Register a callback function for IMU data
  inline void registerImuSingleCallback(
      const ImuSingleInputCallback& callback) {
    imu_single_callback_ = callback;
  }
  inline void registerImuMultiCallback(const ImuMultiInputCallback& callback) {
    imu_multi_callback_ = callback;
  }
  inline void registerLeftFrameCallback(const FrameInputCallback& callback) {
    left_frame_callback_ = callback;
  }
  inline void registerRightFrameCallback(const FrameInputCallback& callback) {
    right_frame_callback_ = callback;
  }

 protected:
  void parseParams();

  // TODO(Toni): Create a separate params only parser!
  //! Helper functions to parse user-specified parameters.
  //! These are agnostic to dataset type.
  void parseBackendParams();
  void parseFrontendParams();
  void parseLCDParams();

  //! Functions to parse dataset dependent parameters.
  // Parse camera params for a given dataset
  CameraParams parseCameraParams(const std::string& filename);
  void parseImuParams();

 public:
  // Init Vio parameters.
  VioParams pipeline_params_;

 protected:
  // Vio callbacks. These functions should be called once data is available for
  // processing.
  ImuSingleInputCallback imu_single_callback_;
  ImuMultiInputCallback imu_multi_callback_;
  FrameInputCallback left_frame_callback_;
  FrameInputCallback right_frame_callback_;

  FrameId initial_k_;  // start frame
  FrameId final_k_;    // end frame
  const std::string dataset_path_;
  const std::string left_cam_params_path_;
  const std::string right_cam_params_path_;
  const std::string imu_params_path_;
  const std::string backend_params_path_;
  const std::string frontend_params_path_;
  const std::string lcd_params_path_;
};

}  // namespace VIO
