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
#include "kimera-vio/frontend/rgbd/RgbdFrame.h"
#include "kimera-vio/pipeline/Pipeline-definitions.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(DataProviderInterface);
  KIMERA_POINTER_TYPEDEFS(DataProviderInterface);
  //! IMU input callbacks come in two flavours:
  //! - Single: allows to add only one single measurement at a time.
  //! - Multi: allows to add a bunch of measurements at a time.
  typedef std::function<void(const ImuMeasurement&)> ImuSingleInputCallback;
  typedef std::function<void(const ImuMeasurements&)> ImuMultiInputCallback;
  typedef std::function<void(Frame::UniquePtr)> FrameInputCallback;
  typedef std::function<void(DepthFrame::UniquePtr)> DepthFrameInputCallback;

  DataProviderInterface() = default;
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

  /**
   * @brief shutdown Call if you want to explicitly stop the data provider.
   * This is thread-safe, since shutdown_ is an atomic bool.
   * NOTE: this is only setting the shutdown_ flag to any class inheriting from
   * this one will have to implement the logic around the usage of shutdown_!
   * See example usage in other DataProvider classes.
   * Other classes might want to override the simple behavior of shutdown();
   */
  virtual void shutdown();

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
  inline void registerDepthFrameCallback(
      const DepthFrameInputCallback& callback) {
    depth_frame_callback_ = callback;
  }

 protected:
  // Vio callbacks. These functions should be called once data is available for
  // processing.
  ImuSingleInputCallback imu_single_callback_;
  ImuMultiInputCallback imu_multi_callback_;
  FrameInputCallback left_frame_callback_;
  FrameInputCallback right_frame_callback_;
  DepthFrameInputCallback depth_frame_callback_;

  // Shutdown switch to stop data provider.
  std::atomic_bool shutdown_ = {false};
};

}  // namespace VIO
