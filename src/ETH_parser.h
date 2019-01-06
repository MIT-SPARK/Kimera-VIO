/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ETH_parser.h
 * @brief  Reads ETH's Euroc dataset
 * @author Luca Carlone
 */

#ifndef ETH_parser_H_
#define ETH_parser_H_

#include <stdlib.h>
#include <fstream>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "Frame.h"
#include "ImuFrontEnd.h"
#include "VioFrontEndParams.h"
#include "VioBackEndParams.h"

namespace VIO {

/*
 * Class describing the imu parameters, to be read from the ETH dataset
 */
class ImuData {
public:
  ImuData() : imu_buffer_(INT64_MAX) {} // imu buffer with virtually infinite memory!

  gtsam::Pose3 body_Pose_cam_;
  double imu_rate_;
  double nominal_imu_rate_;
  double imu_rate_std_;
  double imu_rate_maxMismatch_;
  double gyro_noise_;
  double gyro_walk_;
  double acc_noise_;
  double acc_walk_;

  ImuFrontEnd imu_buffer_;
public:
  void print();
};

/*
 * compact storage of state
 */
class gtNavState {
public:
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity;
  gtsam::imuBias::ConstantBias imuBias;
public:
  void print(const std::string message = " ") const {
    if (VLOG_IS_ON(10)) {
      LOG(INFO) << "--- " << message << "--- ";
      pose.print("\n pose: \n");
      LOG(INFO) << "\n velocity: \n" << velocity.transpose();
      imuBias.print("\n imuBias: \n");
    }
  }
};

/*
 * Store GT poses and GT info
 */
class GroundTruthData {
public:
  gtsam::Pose3 body_Pose_cam_;  // Sensor extrinsics wrt. the body-frame
  double gt_rate_; // data rate in seconds, for debug
  std::map<long long, gtNavState> mapToGt_; // map from timestamp to gtNavState
public:
  // display all params
  void print();
};

/*
 * Store a list of image names and provide functionalities to parse them
 */
class CameraImageLists {
public:
  std::string image_folder_path_;
  typedef std::vector<std::pair<long long, std::string>> ImgLists;
  ImgLists img_lists;
public:
  bool parseCamImgList(const std::string& folderpath,
                       const std::string& filename);
  inline int getNumImages() {
      return img_lists.size();
  }
  void print();
};

/*
 * Parse all images and camera calibration for an ETH dataset
 */
class ETHDatasetParser {
public:
  ETHDatasetParser() : imuData_() {}

  // images data
  std::vector<std::string> camera_names_; // this matches the names of the folders in the dataset
  std::map<std::string, CameraParams> camera_info_; // map from camera name to its parameters
  std::map<std::string, CameraImageLists> camera_image_lists_; // map from camera name to its images

  gtsam::Pose3 camL_Pose_camR_;

  // gt data
  GroundTruthData gtData_;

  // imu data
  ImuData imuData_;

  // Getters
  inline std::string getDatasetName() {
      return dataset_name_;
  }
  inline std::string getLeftImgName(const size_t& k) const {
      return getImgName("cam0", k);
  }
  inline std::string getRightImgName(const size_t& k) const {
      return getImgName("cam1", k);
  }
  inline const CameraParams& getLeftCamInfo() const {
      return camera_info_.at("cam0");
  }
  inline const CameraParams& getRightCamInfo() const {
      return camera_info_.at("cam1");
  }

private:
  bool is_gt_available_;
  std::string dataset_path_;
  std::string dataset_name_;

public:
  // Helper function to parse dataset
  void parse(size_t* initial_k, size_t* final_k);

  // Helper function to parse user-specified parameters.
  void parseParams(
      VioBackEndParamsPtr vioParams,
      const ImuData& imu_data,
      VioFrontEndParams* trackerParams);

  // parse camera, gt, and imu
  bool parseDataset(const std::string& input_dataset_path,
                    const std::string& leftCameraName,
                    const std::string& rightCameraName,
                    const std::string& imuName,
                    const std::string& gtSensorName,
                    bool doParseImages = true);

  // Parse cam0, cam1 of a given dataset
  bool parseCameraData(const std::string& input_dataset_path,
                       const std::string& leftCameraName,
                       const std::string& rightCameraName,
                       const bool doParseImages = true);

  // Parse imu data of a given dataset
  bool parseImuData(const std::string& input_dataset_path,
                    const std::string& imuName);

  // Parse ground truth data
  bool parseGTdata(const std::string& input_dataset_path,
                   const std::string& gtSensorName);

  // Retrieve relative pose between timestamps
  gtsam::Pose3 getGroundTruthRelativePose(
      const Timestamp& previousTimestamp,
      const Timestamp& currentTimestamp) const;

  // retrieve absolute pose at timestamp
  gtNavState getGroundTruthState(const Timestamp& timestamp) const;

  // retrieve absolute pose at timestamp
  inline gtsam::Pose3 getGroundTruthPose(const Timestamp& timestamp) const {
      return getGroundTruthState(timestamp).pose;
  }

  // check if the ground truth is available (i.e., the timestamp is after the first gt state)
  bool isGroundTruthAvailable(const Timestamp& timestamp) const;

  // Get if the dataset has ground truth.
  bool isGroundTruthAvailable() const;

  // compute error on the relative pose between two time stamps, compared with the relative pose from ground trutu
  std::pair<double,double> computePoseErrors(
          const gtsam::Pose3 lkf_T_k_body,
          const bool isTrackingValid,
          const Timestamp& previousTimestamp,
          const Timestamp& currentTimestamp,
          const bool upToScale = false) const;

  // get number of images
  size_t nrImages(){
    std::string leftCameraName = camera_names_[0];
    return camera_image_lists_[leftCameraName].getNumImages();
  }

  // get timestamp of a given pair of stereo images (synchronized)
  Timestamp timestampAtFrame(const Timestamp timestamp){
    std::string leftCameraName = camera_names_[0];
    return camera_image_lists_[leftCameraName].img_lists[timestamp].first;
  }

private:
  inline std::string getImgName(const std::string& id, const size_t& k) const {
      return camera_image_lists_.at(id).img_lists.at(k).second;
  }

  bool sanityCheckCameraData(
      const std::vector<std::string>& camera_names,
      const std::map<std::string, CameraParams>& camera_info,
      std::map<std::string, CameraImageLists>* camera_image_lists) const;

  // Sanity check: nr images is the same for left and right camera
  // Resizes left/right img lists to the minimum number of frames in case of
  // different list sizes.
  bool sanityCheckCamSize(
    CameraImageLists::ImgLists* left_img_lists,
    CameraImageLists::ImgLists* right_img_lists) const;

  // Sanity check: time stamps are the same for left and right camera
  bool sanityCheckCamTimestamps(
      const CameraImageLists::ImgLists& left_img_lists,
      const CameraImageLists::ImgLists& right_img_lists,
      const CameraParams& left_cam_info) const;

public:
  // print info about dataset
  void print();
};

} // namespace VIO

#endif /* ETH_parser_H_ */
