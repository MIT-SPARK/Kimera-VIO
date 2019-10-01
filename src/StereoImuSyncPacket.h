/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuSyncPacket.h
 * @brief  Class describing the minimum input for VIO to run
 * Contains a Stereo Frame with Imu data synchronized from last
 * Keyframe timestamp to the current stereo frame timestamp.
 * @author Antoni Rosinol
 */

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include "StereoFrame.h"
#include "Tracker-definitions.h"
#include "VioBackEnd-definitions.h"
#include "imu-frontend/ImuFrontEnd-definitions.h"
#include "mesh/Mesh.h"

namespace VIO {

struct ReinitPacket {
  ReinitPacket(const bool& reinit_flag_ext = false,
               const Timestamp& timestamp_ext = 0,
               const gtsam::Pose3& W_Pose_Bext = gtsam::Pose3(),
               const gtsam::Vector3& W_Vel_Bext = gtsam::zero(3),
               const ImuBias& imu_bias_ext = gtsam::imuBias::ConstantBias())
      : reinit_flag_ext_(reinit_flag_ext),
        timestamp_ext_(timestamp_ext),
        W_Pose_Bext_(W_Pose_Bext),
        W_Vel_Bext_(W_Vel_Bext),
        imu_bias_ext_(imu_bias_ext) {}

 private:
  // Member variables of reinitialization packet
  bool reinit_flag_ext_;
  Timestamp timestamp_ext_;
  gtsam::Pose3 W_Pose_Bext_;
  gtsam::Vector3 W_Vel_Bext_;
  ImuBias imu_bias_ext_;

 public:
  // Careful, returning references to members can lead to dangling refs.
  inline const bool& getReinitFlag() const { return reinit_flag_ext_; }
  inline const Timestamp& getReinitTime() const { return timestamp_ext_; }
  inline const gtsam::Pose3& getReinitPose() const { return W_Pose_Bext_; }
  inline const gtsam::Vector3& getReinitVel() const { return W_Vel_Bext_; }
  inline const ImuBias& getReinitBias() const { return imu_bias_ext_; }

  // Setting pose
  inline void setReinitFlag(bool& reinit_flag) {
    reinit_flag_ext_ = reinit_flag;
  }
  inline void resetReinitFlag() { reinit_flag_ext_ = false; }
  inline void setReinitPose(gtsam::Pose3& external_pose) {
    W_Pose_Bext_ = external_pose;
  }

  void print() const {
    if (getReinitFlag()) {
      LOG(INFO) << "VIO to be reinitialised with:\n"
                << "TIMESTAMP : \n"
                << getReinitTime() << '\n'
                << "POSE : \n"
                << getReinitPose() << '\n'
                << "VELOCITY : \n"
                << getReinitVel() << '\n'
                << "BIAS : \n"
                << getReinitBias();
    }
  }
};

class StereoImuSyncPacket {
 public:
  // WARNING do not use constructor params after being moved with
  // std::move as they are left in an invalid state!!
  StereoImuSyncPacket() = delete;
  StereoImuSyncPacket(StereoFrame stereo_frame,
                      ImuStampS imu_stamps,
                      ImuAccGyrS imu_accgyr,
                      ReinitPacket reinit_packet = ReinitPacket());
  ~StereoImuSyncPacket() = default;

  // TODO delete copy-constructor because it is used in some places!

  // Careful, returning references to members can lead to dangling refs.
  inline const StereoFrame& getStereoFrame() const { return stereo_frame_; }
  inline const ImuStampS& getImuStamps() const { return imu_stamps_; }
  inline const ImuAccGyrS& getImuAccGyr() const { return imu_accgyr_; }
  inline const ReinitPacket& getReinitPacket() const { return reinit_packet_; }
  inline const bool getReinitFlag() const {
    return reinit_packet_.getReinitFlag();
  }

  // This enforces the frame to be a keyframe
  void setAsKeyframe() { stereo_frame_.setIsKeyframe(true); }

  void print() const {
    LOG(INFO) << "Stereo Frame timestamp: " << stereo_frame_.getTimestamp()
              << '\n'
              << "STAMPS IMU rows : \n"
              << imu_stamps_.rows() << '\n'
              << "STAMPS IMU cols : \n"
              << imu_stamps_.cols() << '\n'
              << "STAMPS IMU: \n"
              << imu_stamps_ << '\n'
              << "ACCGYR IMU rows : \n"
              << imu_accgyr_.rows() << '\n'
              << "ACCGYR IMU cols : \n"
              << imu_accgyr_.cols() << '\n'
              << "ACCGYR IMU: \n"
              << imu_accgyr_;
    if (reinit_packet_.getReinitFlag() == true) {
      LOG(INFO) << "\nVIO Re-Initialized at: " << reinit_packet_.getReinitTime()
                << '\n'
                << "POSE : \n"
                << reinit_packet_.getReinitPose() << '\n'
                << "VELOCITY : \n"
                << reinit_packet_.getReinitVel() << '\n'
                << "BIAS : \n"
                << reinit_packet_.getReinitBias();
    }
  }

  // Use only unique ptr since this class should be moved around,
  // not copied!
  typedef std::unique_ptr<StereoImuSyncPacket> UniquePtr;
  typedef std::unique_ptr<const StereoImuSyncPacket> ConstUniquePtr;

 private:
  // TODO(Toni): this should be const as well, but the initialization is
  // modifying it and setting as keyframe....
  StereoFrame stereo_frame_;
  const ImuStampS imu_stamps_;
  const ImuAccGyrS imu_accgyr_;
  const ReinitPacket reinit_packet_;
};

// Struct to deal with getting values out of the spin
struct SpinOutputPacket {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Default constructor
  SpinOutputPacket(
      const Timestamp& timestamp_kf,
      const gtsam::Pose3& W_Pose_Blkf,
      const gtsam::Vector3& W_Vel_Blkf,
      const ImuBias& imu_bias_lkf,
      const Mesh2D& mesh_2d,
      const Mesh3D& mesh_3d,
      const cv::Mat& mesh_2d_img,
      const PointsWithIdMap& points_with_id_VIO,
      const LmkIdToLmkTypeMap& lmk_id_to_lmk_type_map,
      const gtsam::Matrix state_covariance_lkf = gtsam::zeros(15, 15),
      const DebugTrackerInfo debug_tracker_info = DebugTrackerInfo())
      : timestamp_kf_(timestamp_kf),
        W_Pose_Blkf_(W_Pose_Blkf),
        W_Vel_Blkf_(W_Vel_Blkf),
        imu_bias_lkf_(imu_bias_lkf),
        mesh_2d_(mesh_2d),
        mesh_3d_(mesh_3d),
        mesh_2d_img_(mesh_2d_img),
        points_with_id_VIO_(points_with_id_VIO),
        lmk_id_to_lmk_type_map_(lmk_id_to_lmk_type_map),
        debug_tracker_info_(debug_tracker_info) {
    // TODO: Create a better assert for this covariance matrix
    CHECK_EQ(state_covariance_lkf.rows(), 15);
    CHECK_EQ(state_covariance_lkf.cols(), 15);
    state_covariance_lkf_ = state_covariance_lkf;
  }

  // Trivial constructor
  SpinOutputPacket()
      : timestamp_kf_(0),
        W_Pose_Blkf_(gtsam::Pose3()),
        W_Vel_Blkf_(gtsam::Vector3()),
        imu_bias_lkf_(gtsam::imuBias::ConstantBias()),
        mesh_2d_(),
        mesh_3d_(),
        mesh_2d_img_(),
        points_with_id_VIO_(),
        lmk_id_to_lmk_type_map_(),
        state_covariance_lkf_(gtsam::zeros(15, 15)),
        debug_tracker_info_(DebugTrackerInfo()) {}

  // Define getters for output values
  // TODO(Toni, Sandro): I don't think we need getters for these guys, they
  // should be public members instead and just follow the philosophy of a
  // struct.
  inline Timestamp getTimestamp() const { return timestamp_kf_; }
  inline gtsam::Pose3 getEstimatedPose() const { return W_Pose_Blkf_; }
  inline gtsam::Vector3 getEstimatedVelocity() const { return W_Vel_Blkf_; }
  inline gtsam::Matrix6 getEstimatedPoseCov() const {
    return gtsam::sub(state_covariance_lkf_, 0, 6, 0, 6);
  }
  inline gtsam::Matrix3 getEstimatedVelCov() const {
    return gtsam::sub(state_covariance_lkf_, 6, 9, 6, 9);
  }
  inline ImuBias getEstimatedBias() const { return imu_bias_lkf_; }
  inline gtsam::Matrix6 getEstimatedBiasCov() const {
    return gtsam::sub(state_covariance_lkf_, 9, 15, 9, 15);
  }
  inline DebugTrackerInfo getTrackerInfo() const { return debug_tracker_info_; }

 public:
  Mesh2D mesh_2d_;
  Mesh3D mesh_3d_;
  cv::Mat mesh_2d_img_;
  PointsWithIdMap points_with_id_VIO_;
  LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_;

 private:
  Timestamp timestamp_kf_;
  gtsam::Pose3 W_Pose_Blkf_;
  gtsam::Vector3 W_Vel_Blkf_;
  ImuBias imu_bias_lkf_;
  gtsam::Matrix state_covariance_lkf_;
  DebugTrackerInfo debug_tracker_info_;
};

}  // namespace VIO
