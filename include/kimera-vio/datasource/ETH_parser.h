/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ETH_parser.h
 * @brief  Parse EUROC dataset.
 * @author Antoni Rosinol,
 * @author Yun Chang,
 * @author Luca Carlone
 */

#pragma once

#include <stdlib.h>
#include <algorithm>  // for max
#include <fstream>
#include <map>
#include <string>
#include <utility>  // for make_pair
#include <vector>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/datasource/DataSource-definitions.h"
#include "kimera-vio/datasource/DataSource.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoImuSyncPacket.h"
#include "kimera-vio/frontend/StereoMatchingParams.h"

namespace VIO {

/*
 * Parse all images and camera calibration for an ETH dataset.
 */
class ETHDatasetParser : public DataProviderInterface {
 public:
  // Ctor with params.
  ETHDatasetParser(const int& initial_k,
                   const int& final_k,
                   const std::string& dataset_path,
                   const std::string& left_cam_params_path,
                   const std::string& right_cam_params_path,
                   const std::string& imu_params_path,
                   const std::string& backend_params_path,
                   const std::string& frontend_params_path,
                   const std::string& lcd_params_path);
  // Ctor from gflags
  ETHDatasetParser();
  virtual ~ETHDatasetParser();

 public:
  virtual bool spin() override;

  // Print info about dataset.
  void print() const;

 public:
  // Ground truth data.
  GroundTruthData gt_data_;

 private:
  // Parses EuRoC data
  void parse();

  // Send data to VIO pipeline on a per-frame basis
  void spinOnce(const FrameId& k,
                const CameraParams& left_cam_info,
                const CameraParams& right_cam_info,
                const bool& equalize_image);

  // Parse camera, gt, and imu data if using different Euroc format.
  bool parseDataset();

  //! Parsers
  bool parseImuData(const std::string& input_dataset_path,
                    const std::string& imu_name);

  bool parseGTdata(const std::string& input_dataset_path,
                   const std::string& gtSensorName);

  bool parseCameraData(const std::string& cam_name,
                       CameraImageLists* cam_list_i);

  //! Getters.
  std::string getDatasetName();
  inline std::string getLeftImgName(const size_t& k) const {
    return getImgName("cam0", k);
  }
  inline std::string getRightImgName(const size_t& k) const {
    return getImgName("cam1", k);
  }

  // Retrieve relative pose between timestamps.
  gtsam::Pose3 getGroundTruthRelativePose(
      const Timestamp& previousTimestamp,
      const Timestamp& currentTimestamp) const;

  // Retrieve absolute pose at timestamp.
  VioNavState getGroundTruthState(const Timestamp& timestamp) const;

  // Compute initialization errors and stats.
  const InitializationPerformance getInitializationPerformance(
      const std::vector<Timestamp>& timestamps,
      const std::vector<gtsam::Pose3>& poses_ba,
      const VioNavState& init_nav_state,
      const gtsam::Vector3& init_gravity);

  inline size_t getNumImages() const {
    const std::string& camera_name = camera_names_.at(0);
    const auto& iter = camera_image_lists_.find(camera_name);
    CHECK(iter != camera_image_lists_.end());
    return iter->second.getNumImages();
  }
  inline std::string getImgName(const std::string& id, const size_t& k) const {
    const std::string& camera_name = camera_names_.at(0);
    const auto& iter = camera_image_lists_.find(id);
    CHECK(iter != camera_image_lists_.end());
    return iter->second.img_lists_.at(k).second;
  }
  // Retrieve absolute pose at timestamp.
  inline gtsam::Pose3 getGroundTruthPose(const Timestamp& timestamp) const {
    return getGroundTruthState(timestamp).pose_;
  }

  //! Sanity checks
  bool sanityCheckCameraData(
      const std::vector<std::string>& camera_names,
      std::map<std::string, CameraImageLists>* camera_image_lists) const;

  // Sanity check: nr images is the same for left and right camera
  // Resizes left/right img lists to the minimum number of frames in case of
  // different list sizes.
  bool sanityCheckCamSize(CameraImageLists::ImgLists* left_img_lists,
                          CameraImageLists::ImgLists* right_img_lists) const;

  // Sanity check: time stamps are the same for left and right camera
  bool sanityCheckCamTimestamps(
      const CameraImageLists::ImgLists& left_img_lists,
      const CameraImageLists::ImgLists& right_img_lists,
      const CameraParams& left_cam_info) const;

  //! Utilities
  // Check if the ground truth is available.
  // (i.e., the timestamp is after the first gt state)
  bool isGroundTruthAvailable(const Timestamp& timestamp) const;

  // Get if the dataset has ground truth.
  bool isGroundTruthAvailable() const;

  // Get timestamp of a given pair of stereo images (synchronized).
  Timestamp timestampAtFrame(const FrameId& frame_number);

  // Clip final frame to the number of images in the dataset.
  void clipFinalFrame();

 private:
  /// Images data.
  // This matches the names of the folders in the dataset
  std::vector<std::string> camera_names_;
  // Map from camera name to its images
  std::map<std::string, CameraImageLists> camera_image_lists_;

  bool is_gt_available_;
  std::string dataset_name_;

  // Number of initial frames to skip for
  // IMU calibration. We only use the IMU information and discard the frames.
  int skip_n_start_frames_ = 0;
  // Number of final frames to skip for IMU calibration. We only use the IMU
  // information and discard the frames.
  int skip_n_end_frames_ = 0;

  const std::string kLeftCamName = "cam0";
  const std::string kRightCamName = "cam1";
  const std::string kImuName = "imu0";
};

}  // namespace VIO
