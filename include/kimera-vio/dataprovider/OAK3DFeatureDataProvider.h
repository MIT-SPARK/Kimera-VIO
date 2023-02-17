/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OAK3DFeatureDataProvider.h
 * @brief  Parse OAK Device Depth, Mono and Feature Streams
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
class OAK3DFeatureDataProvider : public OAKDataProvider {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(OAK3DFeatureDataProvider);
  KIMERA_POINTER_TYPEDEFS(OAK3DFeatureDataProvider);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  //! Ctor with params.
  OAK3DFeatureDataProvider(const VioParams& vio_params);
  
  // //! Ctor from gflags
  // explicit OAKDataProvider(const VioParams& vio_params);

  virtual ~OAK3DFeatureDataProvider();

  void setDepthFeatureQueues(std::shared_ptr<dai::DataOutputQueue> depth_queue, std::shared_ptr<dai::DataOutputQueue> features_queue);


  /**
   * @brief spin Spins the dataset until it finishes. If set in sequential mode,
   * it will return each time a frame is sent. In parallel mode, it will not
   * return until it finishes.
   * @return True if the dataset still has data, false otherwise.
   */
  virtual bool spin() override;

  /**
   * @brief Callback to get the Image and Feature map to be added to 
   * a frame and send to the RGBDDataProviderModule.
   * 
   */ 
  void leftImageFeatureCallback(std::shared_ptr<dai::ADatatype> image, std::shared_ptr<dai::ADatatype> feature_map);

  /**
   * @brief Callback to connect to get the depth image into pipeline
   * 
   */ 
  void depthImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data);

 protected:
  /**
   * @brief spinOnce Send data to VIO pipeline on a per-frame basis
   * @return if the dataset finished or not
   */
  // virtual bool spinOnce();

  void syncImageFeatureGrab(std::shared_ptr<dai::ADatatype> image, std::shared_ptr<dai::ADatatype> depth, std::shared_ptr<dai::ADatatype> feature_map);

 protected:
  // TODO(Saching): move these to the backend Queues later. 
  std::queue<std::tuple<std::shared_ptr<dai::ADatatype>, std::shared_ptr<dai::ADatatype>, std::shared_ptr<dai::ADatatype>>> sync_msgs_;
  std::queue<std::shared_ptr<dai::ADatatype>> left_sync_queue_, depth_sync_queue_, feature_sync_queue_;

  int depth_image_count_, feature_msg_count_, depth_image_fps_, feature_msg_fps_;
  Timestamp recent_depth_image_timestamp_, recent_feature_timestamp_;

  std::shared_ptr<dai::DataOutputQueue> depth_queue_, feature_queue_;
  // FIXME(Saching): Replace the EurocGtLogger later)
  // EurocGtLogger::UniquePtr logger_;
};

}  // namespace VIO