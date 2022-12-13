/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OAKDataProvider.h
 * @brief  Parse OAK Device Streams
 * @author Sachin Guruswamy
 */

#pragma once

#include <map>
#include <string>
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <depthai/depthai.hpp>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/dataprovider/DataProviderInterface.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {
enum class ImuSyncMethod { COPY, LINEAR_INTERPOLATE_GYRO, LINEAR_INTERPOLATE_ACCEL };

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class OAKDataProvider : public DataProviderInterface {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(OAKDataProvider);
  KIMERA_POINTER_TYPEDEFS(OAKDataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  OAKDataProvider(const VioParams& vio_params);
  
  // //! Ctor from gflags
  // explicit OAKDataProvider(const VioParams& vio_params);

  virtual ~OAKDataProvider();

 public:
  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

  /**
   * @brief IMU Callback to connect to IMU QUeue
   * 
   */ 
  void imuCallback(std::string name, std::shared_ptr<dai::ADatatype> data);

  /**
   * @brief Callback to connect to Undistorted left Queue
   * 
   */ 
  void leftImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data);

  /**
   * @brief Callback to connect to Undistorted right Queue
   * 
   */ 
  void rightImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data);

  // // Retrieve absolute gt pose at *approx* timestamp.
  // inline gtsam::Pose3 getGroundTruthPose(const Timestamp& timestamp) const {
  //   return getGroundTruthState(timestamp).pose_;
  // }
  private:
    void FillImuDataLinearInterpolation(std::vector<IMUPacket>& imuPackets);
    
    void sendImuMeasurement(dai::IMUReportAccelerometer accel, dai::IMUReportGyroscope gyro);

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  // virtual bool spinOnce();

  /**
   * @brief sendImuData We send IMU data first (before frames) so that the VIO
   * pipeline can query all IMU data between frames.
   */
  void sendImuData() const;


// Get timestamp of a given pair of stereo images & IMU (synchronized).
Timestamp timestampAtFrame(const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& timestamp);



 protected:
  VioParams vio_params_;

  /// Images data.
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  // This matches the names of the folders in the dataset
  std::vector<std::string> camera_names_;
  
  //! Pre-stored imu-measurements
  std::vector<ImuMeasurement> imu_measurements_;
  ImuSyncMethod sync_mode_ = ImuSyncMethod::LINEAR_INTERPOLATE_ACCEL;
  // FIXME(Saching): Replace the EurocGtLogger later)
  // EurocGtLogger::UniquePtr logger_;
};

}  // namespace VIO