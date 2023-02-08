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

#include "kimera-vio/dataprovider/OAKStereoDataProvider.h"

#include <algorithm>  // for max
#include <chrono>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <utility>  // for pair<>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gflags/gflags.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/imu-frontend/ImuFrontend-definitions.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/utils/YamlParser.h"


namespace VIO {

/* -------------------------------------------------------------------------- */
OAKStereoDataProvider::OAKStereoDataProvider(const VioParams& vio_params)
    : OAKDataProvider(vio_params),
      right_cam_info_(vio_params_.camera_params_.at(1)){}

/* -------------------------------------------------------------------------- */
OAKStereoDataProvider::~OAKStereoDataProvider() {
  LOG(INFO) << "OAKStereoDataProvider destructor called.";
}

void OAKStereoDataProvider::setRightQueue(std::shared_ptr<dai::DataOutputQueue> right_queue){
    right_queue_ = right_queue;
}

/* -------------------------------------------------------------------------- */
bool OAKStereoDataProvider::spin() {
    // Spin.
CHECK_EQ(vio_params_.camera_params_.size(), 2u);
  LOG(INFO) << "Data OAKStereoDataProvider Interface: <-------------- Spinning -------------->";

    while (!shutdown_) {
        std::shared_ptr<dai::ADatatype> left_image       = left_queue_->get<dai::ADatatype>();
        std::shared_ptr<dai::ADatatype> right_image      = right_queue_->get<dai::ADatatype>();
        std::shared_ptr<dai::ADatatype> imu_measurements = imu_queue_->get<dai::ADatatype>();
        
        std::string name = "";
        imuCallback(name, imu_measurements);
        syncImageGrab(left_image, right_image);
        while(!sync_msgs_.empty()) {
            std::shared_ptr<dai::ADatatype> synced_left, synced_right;
            std::tie(synced_left, synced_right) = sync_msgs_.front();
            leftImageCallback("left",   synced_left);
            rightImageCallback("right", synced_right);
        }
        if (!vio_params_.parallel_run_) {
            return true;
        }
    //   std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  LOG_IF(INFO, shutdown_) << "OAK StereoDataProvider shutdown requested.";
  return false;
}

void OAKStereoDataProvider::syncImageGrab(std::shared_ptr<dai::ADatatype> left_msg, std::shared_ptr<dai::ADatatype> right_msg){
    left_sync_queue_.push(left_msg);
    right_sync_queue_.push(right_msg);
    if (left_sync_queue_.size() > 8 || right_sync_queue_.size() > 8){
        LOG(ERROR) << "Queue Sizes exceeded the max of "<< left_sync_queue_.size() << " and " << right_sync_queue_.size();
        // left_sync_queue_.clear();
        // right_sync_queue_.clear();
    }
    while(!left_sync_queue_.empty() && !right_sync_queue_.empty()){
        std::shared_ptr<dai::ImgFrame> left_msg  = std::dynamic_pointer_cast<dai::ImgFrame>(left_sync_queue_.front());
        std::shared_ptr<dai::ImgFrame> right_msg = std::dynamic_pointer_cast<dai::ImgFrame>(right_sync_queue_.front());
        int64_t leftSeqNum = left_msg->getSequenceNum();
        int64_t rightSeqNum = right_msg->getSequenceNum();
        if (leftSeqNum == rightSeqNum){
            right_msg->setTimestamp(left_msg->getTimestamp());
            sync_msgs_.push(std::make_tuple(left_sync_queue_.front(), right_sync_queue_.front()));
            left_sync_queue_.pop();
            right_sync_queue_.pop();
        }
        else if (leftSeqNum > rightSeqNum){
            right_sync_queue_.pop();
        }
        else if (rightSeqNum > leftSeqNum){
            left_sync_queue_.pop();
        }
    }
}

void OAKStereoDataProvider::rightImageCallback(std::string name, std::shared_ptr<dai::ADatatype> data){
    CHECK(right_frame_callback_) << "Did you forget to register the right image callback to the VIO Pipeline?";
    auto daiDataPtr = std::dynamic_pointer_cast<dai::ImgFrame>(data);
    cv::Mat imageFrame = daiDataPtr->getCvFrame();
    Timestamp localTimestamp = timestampAtFrame(daiDataPtr->getTimestamp());
    
    // TODO(saching): Add option to equalize the image from histogram
    right_frame_callback_(
        VIO::make_unique<Frame>(daiDataPtr->getSequenceNum(),
                                localTimestamp,
                                right_cam_info_,
                                imageFrame));
}

}  // namespace VIO
