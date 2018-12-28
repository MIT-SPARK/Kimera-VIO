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

// #define ETH_PARSER_DEBUG_COUT

namespace VIO {

/*
 * Class describing the imu parameters, to be read from the ETH dataset
 */
class ImuData {
public:
  ImuData() : imu_buffer_(1e20) {} // imu buffer with virtually infinite memory!

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
  void print(const std::string message = " "){
#ifdef ETH_PARSER_DEBUG_COUT
    std::cout << "--- " << message << "--- " << std::endl;
    pose.print("\n pose: \n");
    std::cout << "\n velocity: \n" << velocity.transpose() << std::endl;
    imuBias.print("\n imuBias: \n");
#endif
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
  std::vector<std::pair<long long, std::string> > img_lists;
public:
  bool parseCameraImageList(const std::string folderpath,const std::string filename);
  int getNumImages(){return img_lists.size();}
  void print();
};

/*
 * Parse all images and camera calibration for an ETH dataset
 */
class ETHDatasetParser {
public:
  ETHDatasetParser() : imuData_() {}

  std::string dataset_path_;
  std::string dataset_name_;

  // images data
  std::vector<std::string> camera_names; // this matches the names of the folders in the dataset
  std::map<std::string, CameraParams> camera_info; // map from camera name to its parameters
  std::map<std::string, CameraImageLists> camera_image_lists; // map from camera name to its images

  gtsam::Pose3 camL_Pose_calR;

  // gt data
  GroundTruthData gtData_;

  // imu data
  ImuData imuData_;

private:
  bool is_gt_available_;

public:
  // Helper function to parse dataset and user-specified parameters.
  void parseDatasetAndParams(VioBackEndParamsPtr vioParams,
                             VioFrontEndParams* trackerParams,
                             size_t* initial_k, size_t* final_k);

  // parse camera, gt, and imu
  bool parseDataset(const std::string& input_dataset_path,
                    const std::string& leftCameraName,
                    const std::string& rightCameraName,
                    const std::string& imuName,
                    const std::string& gtSensorName,
                    bool doParseImages = true);

  // Parse cam0, cam1 of a given dataset
  bool parseCameraData(const std::string input_dataset_path, const std::string leftCameraName,
      const std::string rightCameraName, const bool doParseImages = true);

  // Parse imu data of a given dataset
  bool parseImuData(const std::string input_dataset_path, const std::string imuName);

  // Parse ground truth data
  bool parseGTdata(const std::string input_dataset_path, const std::string gtSensorName);

  // Retrieve relative pose between timestamps
  gtsam::Pose3 getGroundTruthRelativePose(const long long previousTimestamp, const long long currentTimestamp) const;

  // retrieve absolute pose at timestamp
  gtNavState getGroundTruthState(const long long timestamp) const;

  // retrieve absolute pose at timestamp
  gtsam::Pose3 getGroundTruthPose(const long long timestamp) const { return getGroundTruthState(timestamp).pose; }

  // check if the ground truth is available (i.e., the timestamp is after the first gt state)
  bool isGroundTruthAvailable(const long long timestamp) const;

  // Get if the dataset has ground truth.
  bool isGroundTruthAvailable() const;

  // compute error on the relative pose between two time stamps, compared with the relative pose from ground trutu
  std::pair<double,double> computePoseErrors(const gtsam::Pose3 lkf_T_k_body, const bool isTrackingValid,
      long long previousTimestamp, long long currentTimestamp, const bool upToScale = false) const;

  // get number of images
  size_t nrImages(){
    std::string leftCameraName = camera_names[0];
    return camera_image_lists[leftCameraName].getNumImages();
  }

  // get timestamp of a given pair of stereo images (synchronized)
  long long timestampAtFrame(const long long timestamp){
    std::string leftCameraName = camera_names[0];
    return camera_image_lists[leftCameraName].img_lists[timestamp].first;
  }

  // print info about dataset
  void print();
};

} // namespace VIO

#endif /* ETH_parser_H_ */
