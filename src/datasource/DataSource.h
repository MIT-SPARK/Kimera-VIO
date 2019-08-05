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

#include "StereoImuSyncPacket.h"
#include "Tracker.h"
#include "VioFrontEndParams.h"
#include "VioBackEndParams.h"
#include "RegularVioBackEndParams.h"
#include "ImuFrontEnd.h"

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
  gtNavState() = default;
  gtNavState(const gtsam::Pose3& pose,
             const gtsam::Vector3& velocity,
             const gtsam::imuBias::ConstantBias& imu_bias)
    : pose_(pose),
      velocity_(velocity),
      imu_bias_(imu_bias) {}
  gtNavState(const gtsam::NavState& nav_state,
             const gtsam::imuBias::ConstantBias& imu_bias)
    : pose_(nav_state.pose()),
      velocity_(nav_state.velocity()),
      imu_bias_(imu_bias) {}


  gtsam::Pose3 pose_;
  gtsam::Vector3 velocity_;
  gtsam::imuBias::ConstantBias imu_bias_;

  void print(const std::string message = " ") const {
    if (VLOG_IS_ON(10)) {
      LOG(INFO) << "--- " << message << "--- ";
      pose_.print("\n pose: \n");
      LOG(INFO) << "\n velocity: \n" << velocity_.transpose();
      imu_bias_.print("\n imuBias: \n");
    }
  }

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
      const gtsam::Vector3 gt_gravity)
      : init_timestamp_(init_timestamp),
        init_n_frames_(init_n_frames),
        avg_rotationErrorBA_(avg_rotationErrorBA),
        avg_tranErrorBA_(avg_tranErrorBA),
        init_nav_state_(init_nav_state),
        init_gravity_(init_gravity),
        gt_nav_state_(gt_nav_state),
        gt_gravity_(gt_gravity) {}

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
  inline size_t getNumImages() const {return img_lists.size();}
  void print() const;

public:
  std::string image_folder_path_;
  typedef std::vector<std::pair<long long, std::string> > ImgLists;
  ImgLists img_lists;
};

// Struct to deal with getting values out of the spin
struct SpinOutputContainer {
  // Default constructor
  SpinOutputContainer(
      const Timestamp& timestamp_kf, const gtsam::Pose3& W_Pose_Blkf,
      const Vector3& W_Vel_Blkf, const ImuBias& imu_bias_lkf,
      const gtsam::Matrix state_covariance_lkf = gtsam::zeros(15, 15),
      const DebugTrackerInfo debug_tracker_info = DebugTrackerInfo())
      : timestamp_kf_(timestamp_kf),
        W_Pose_Blkf_(W_Pose_Blkf),
        W_Vel_Blkf_(W_Vel_Blkf),
        imu_bias_lkf_(imu_bias_lkf),
        debug_tracker_info_(debug_tracker_info) {
    // TODO: Create a better assert for this covariance matrix
    CHECK_EQ(state_covariance_lkf.rows(), 15);
    CHECK_EQ(state_covariance_lkf.cols(), 15);
    state_covariance_lkf_ = state_covariance_lkf;
  }

  // Trivial constructor (do not publish)
  SpinOutputContainer()
    : timestamp_kf_(0),
      W_Pose_Blkf_(gtsam::Pose3()),
      W_Vel_Blkf_(gtsam::Vector3()),
      imu_bias_lkf_(gtsam::imuBias::ConstantBias()),
      state_covariance_lkf_(gtsam::zeros(15,15)),
      debug_tracker_info_(DebugTrackerInfo()) {}

  Timestamp timestamp_kf_;
  gtsam::Pose3 W_Pose_Blkf_;
  Vector3 W_Vel_Blkf_;
  ImuBias imu_bias_lkf_;
  gtsam::Matrix state_covariance_lkf_;
  DebugTrackerInfo debug_tracker_info_;

  inline const Timestamp getTimestamp() const { return timestamp_kf_; }

  inline const gtsam::Pose3 getEstimatedPose() const { return W_Pose_Blkf_; }

  inline const Vector3 getEstimatedVelocity() const { return W_Vel_Blkf_; }

  inline const gtsam::Matrix6 getEstimatedPoseCov() const {
    return gtsam::sub(state_covariance_lkf_, 0, 6, 0, 6);
  }

  inline const gtsam::Matrix3 getEstimatedVelCov() const {
    return gtsam::sub(state_covariance_lkf_, 6, 9, 6, 9);
  }

  inline const ImuBias getEstimatedBias() const { return imu_bias_lkf_; }

  inline const gtsam::Matrix6 getEstimatedBiasCov() const {
    return gtsam::sub(state_covariance_lkf_, 9, 15, 9, 15);
  }

  inline const DebugTrackerInfo getTrackerInfo() const { return debug_tracker_info_; }
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
  void parseParams();
};

}  // namespace VIO
