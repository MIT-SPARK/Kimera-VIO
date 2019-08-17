/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   DataSource.h
 * @brief  Base implementation of a data provider for the VIO pipeline.
 * @author Antoni Rosinol
 */

#pragma once

#include <functional>
#include <string>

#include "ImuFrontEnd.h"
#include "RegularVioBackEndParams.h"
#include "StereoImuSyncPacket.h"
#include "VioBackEndParams.h"
#include "VioFrontEndParams.h"

#include <gtsam/navigation/ImuBias.h>

#include <gflags/gflags.h>

//########### SPARK_VIO_ROS ############################################
namespace VIO {

// TODO make new file for Ground Truth Data and the like,
// because it is used by the backend and the feature selector.
// Leaving it in the parser forces these modules to include a parser which is
// at the very least weird.

/*
 * Compact storage of state.
 */
class gtNavState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  gtNavState(){}

  gtNavState(const gtsam::Pose3& pose,
             const gtsam::Vector3& velocity,
             const gtsam::imuBias::ConstantBias& imu_bias);

  gtNavState(const gtsam::NavState& nav_state,
             const gtsam::imuBias::ConstantBias& imu_bias);

  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias imu_bias_;

  void print(const std::string& message = " ") const;

  gtsam::Pose3 pose() const { return pose_; };
};

// Struct for performance in initialization
struct InitializationPerformance {
  public:
    // Default constructor
    InitializationPerformance(
      const Timestamp init_timestamp,
      const int init_n_frames,
      const double avg_rotationErrorBA,
      const double avg_tranErrorBA,
      const gtNavState init_nav_state,
      const gtsam::Vector3 init_gravity,
      const gtNavState gt_nav_state,
      const gtsam::Vector3 gt_gravity);

    void print() const;

  public:
    const Timestamp init_timestamp_;
    const int init_n_frames_;
    const double avg_rotationErrorBA_;
    const double avg_tranErrorBA_;
    const gtNavState init_nav_state_;
    const gtsam::Vector3 init_gravity_;
    const gtNavState gt_nav_state_;
    const gtsam::Vector3 gt_gravity_;
};

/*
 * Store GT poses and GT info.
 */
class GroundTruthData {
 public:
  // Display all params.
  void print() const;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Sensor extrinsics wrt. the body-frame
  gtsam::Pose3 body_Pose_cam_;

  // Data rate in seconds, for debug.
  double gt_rate_;

  // Map from timestamp to gtNavState.
  std::map<long long, gtNavState> mapToGt_;
};

/*
 * Store a list of image names and provide functionalities to parse them.
 */
class CameraImageLists {
 public:
  bool parseCamImgList(const std::string& folderpath,
                       const std::string& filename);
  inline size_t getNumImages() const { return img_lists.size(); }
  void print() const;

 public:
  std::string image_folder_path_;
  typedef std::vector<std::pair<long long, std::string> > ImgLists;
  ImgLists img_lists;
};

struct PipelineParams {
  VioFrontEndParams frontend_params_;
  VioBackEndParamsPtr backend_params_;
  ImuParams imu_params_;
  int backend_type_;
};

class DataProvider {
 public:
  DataProvider();
  virtual ~DataProvider();

  // The derived classes need to implement this function!
  // Spin the dataset: processes the input data and constructs a Stereo Imu
  // Synchronized Packet which contains the minimum amount of information
  // for the VIO pipeline to do one processing iteration.
  // A Dummy example is provided as an implementation.
  virtual bool spin();
  // Init Vio parameters.
  PipelineParams pipeline_params_;

  // Register a callback function that will be called once a StereoImu Synchro-
  // nized packet is available for processing.
  void registerVioCallback(
      std::function<void(const StereoImuSyncPacket&)> callback);

 protected:
  // Vio callback. This function should be called once a StereoImuSyncPacket
  // is available for processing.
  std::function<void(const StereoImuSyncPacket&)> vio_callback_;

  int initial_k_; // start frame
  int final_k_; // end frame
  std::string dataset_path_;

protected:
  // Helper function to parse user-specified parameters.
  void parseBackendParams();
  void parseFrontendParams();
};

}  // namespace VIO
