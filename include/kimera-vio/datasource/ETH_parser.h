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
  ETHDatasetParser(int initial_k,
                   int final_k,
                   const std::string& dataset_path,
                   int skip_n_start_frames,
                   int skip_n_end_frames);
  // Ctor from gflags params.
  ETHDatasetParser();
  virtual ~ETHDatasetParser();

 public:
  virtual bool spin() override;

  void spinOnce(const FrameId& k,
                const StereoMatchingParams& stereo_matchiong_params,
                const bool equalize_image,
                const CameraParams& left_cam_info,
                const CameraParams& right_cam_info,
                Timestamp* timestamp_last_frame);

  // Parses EuRoC data, as well as the frontend and backend parameters
  void parse();

  // Parse camera, gt, and imu data if using different Euroc format.
  bool parseDataset(const std::string& input_dataset_path,
                    const std::string& left_cam_name,
                    const std::string& right_cam_name,
                    const std::string& imu_name,
                    const std::string& gt_sensor_name,
                    bool parse_images = true);

  // Check if the ground truth is available.
  // (i.e., the timestamp is after the first gt state)
  bool isGroundTruthAvailable(const Timestamp& timestamp) const;

  // Get if the dataset has ground truth.
  bool isGroundTruthAvailable() const;

  // Get timestamp of a given pair of stereo images (synchronized).
  Timestamp timestampAtFrame(const FrameId& frame_number);

  /// Getters
  inline std::string getDatasetName() const { return dataset_name_; }
  inline std::string getLeftImgName(const size_t& k) const {
    return getImgName("cam0", k);
  }
  inline std::string getRightImgName(const size_t& k) const {
    return getImgName("cam1", k);
  }
  inline ImuParams getImuParams() const {
    return pipeline_params_.imu_params_;
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

  // Print info about dataset.
  void print() const;

 public:
  // Ground truth data.
  GroundTruthData gt_data_;

  // IMU data.
  ImuData imu_data_;

 private:
  bool parseImuData(const std::string& input_dataset_path,
                    const std::string& imu_name);

  bool parseGTdata(const std::string& input_dataset_path,
                   const std::string& gtSensorName);

  // Parse cam0, cam1 of a given dataset.
  virtual bool parseCameraParams(const std::string& input_dataset_path,
                                 const std::string& left_cam_name,
                                 const std::string& right_cam_name,
                                 bool parse_images,
                                 MultiCameraParams* multi_cam_params) override;

  virtual bool parseImuParams(const std::string& input_dataset_path,
                              const std::string& imu_name,
                              ImuParams* imu_params) override;

  /// Getters.
  inline size_t getNumImages() const {
    return camera_image_lists_.at(camera_names_.at(0)).getNumImages();
  }
  inline std::string getImgName(const std::string& id, const size_t& k) const {
    return camera_image_lists_.at(id).img_lists_.at(k).second;
  }
  // Retrieve absolute pose at timestamp.
  inline gtsam::Pose3 getGroundTruthPose(const Timestamp& timestamp) const {
    return getGroundTruthState(timestamp).pose_;
  }

  bool sanityCheckCameraData(const std::vector<std::string>& camera_names,
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
};

}  // namespace VIO
