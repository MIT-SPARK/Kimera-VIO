/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OAKDataProvider.h
 * @brief  Parse OAK device left, right and IMU Streams.
 * @author Sachin Guruswamy
 */

#pragma once

#include <map>
#include <string>
#include <stack> // for syncing
#include <queue> // for syncing

#include <unordered_map> // for syncing
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include "depthai/depthai.hpp"

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/common/VioNavState.h"
#include "kimera-vio/dataprovider/DataProviderInterface-definitions.h"
#include "kimera-vio/dataprovider/OAKDataProvider.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class OAKStereoDataProvider : public OAKDataProvider {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(OAKStereoDataProvider);
  KIMERA_POINTER_TYPEDEFS(OAKStereoDataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  OAKStereoDataProvider(const VioParams& vio_params);
  
  // //! Ctor from gflags
  // explicit OAKDataProvider(const VioParams& vio_params);

  virtual ~OAKStereoDataProvider();

  void setRightQueue(std::shared_ptr<dai::DataOutputQueue> right_queue);

  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

  /**
   * @brief Callback to connect to Undistorted right Queue
   * 
   */ 
  void rightImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data);

  void syncImageGrab(std::shared_ptr<dai::ADatatype> left_msg, std::shared_ptr<dai::ADatatype> right_msg);


 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  // virtual bool spinOnce();

  // TODO(saching): Update the Sync functionality to be something more reliant
  // void syncImageSend(std::shared_ptr<dai::ADatatype> left_msg, std::shared_ptr<dai::ADatatype> right_msg);

 protected:
  /// Images data.
  // TODO(Toni): remove camera_names_ and camera_image_lists_...
  // This matches the names of the folders in the dataset
  CameraParams& right_cam_info_;

  // TODO(Saching): move these to the backend Queues later. 
  // std::stack<std::pair<std::shared_ptr<dai::ADatatype>, std::shared_ptr<dai::ADatatype>>> sync_msgs_;
  std::queue<std::tuple<std::shared_ptr<dai::ADatatype>, std::shared_ptr<dai::ADatatype>>> sync_msgs_;
  std::queue<std::shared_ptr<dai::ADatatype>> left_sync_queue_, right_sync_queue_;


  std::shared_ptr<dai::DataOutputQueue> right_queue_;
  // FIXME(Saching): Replace the EurocGtLogger later)
  // EurocGtLogger::UniquePtr logger_;
};

}  // namespace VIO