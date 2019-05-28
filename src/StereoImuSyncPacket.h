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

#include "StereoFrame.h"
#include "ImuFrontEnd.h"

namespace VIO {

struct ReinitPacket {
  ReinitPacket(const bool& reinit_flag_ext = false,
                         const Timestamp& timestamp_ext = 0,
                         const gtsam::Pose3& W_Pose_Bext = gtsam::Pose3(),
                         const Vector3& W_Vel_Bext = gtsam::zero(3),
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
    Vector3 W_Vel_Bext_;
    ImuBias imu_bias_ext_;

  public:

    // Careful, returning references to members can lead to dangling refs.
    inline const bool& getReinitFlag() const {return reinit_flag_ext_;}
    inline const Timestamp& getReinitTime() const {return timestamp_ext_;}
    inline const gtsam::Pose3& getReinitPose() const {return W_Pose_Bext_;}
    inline const gtsam::Vector3& getReinitVel() const {return W_Vel_Bext_;}
    inline const ImuBias& getReinitBias() const {return imu_bias_ext_;}

    // Setting pose
    inline void setReinitFlag(bool& reinit_flag) { reinit_flag_ext_ = reinit_flag;}
    inline void resetReinitFlag() { reinit_flag_ext_ = false;}
    inline void setReinitPose(gtsam::Pose3& external_pose) { W_Pose_Bext_ = external_pose;}

    void print() const {
      if (getReinitFlag()) {
        LOG(INFO) 
          << "VIO to be reinitialised with:\n"
          << "TIMESTAMP : \n"<< getReinitTime() << '\n'
          << "POSE : \n" << getReinitPose() << '\n'
          << "VELOCITY : \n" << getReinitVel() << '\n'
          << "BIAS : \n" << getReinitBias();
      }
    }
    
};

class StereoImuSyncPacket {
public:
  StereoImuSyncPacket() = delete;
  StereoImuSyncPacket(StereoFrame stereo_frame,
                      ImuStampS imu_stamps,
                      ImuAccGyrS imu_accgyr,
                      ReinitPacket reinit_packet = ReinitPacket());
  ~StereoImuSyncPacket() = default;

  // Careful, returning references to members can lead to dangling refs.
  inline const StereoFrame& getStereoFrame() const {return stereo_frame_;}
  inline const ImuStampS& getImuStamps() const {return imu_stamps_;}
  inline const ImuAccGyrS& getImuAccGyr() const {return imu_accgyr_;}
  inline const ReinitPacket& getReinitPacket() const {return reinit_packet_;}

  void print() const {
    LOG(INFO)
        << "Stereo Frame timestamp: " << stereo_frame_.getTimestamp() << '\n'
        << "STAMPS IMU rows : \n" << imu_stamps_.rows()  << '\n'
        << "STAMPS IMU cols : \n" << imu_stamps_.cols() << '\n'
        << "STAMPS IMU: \n" << imu_stamps_ << '\n'
        << "ACCGYR IMU rows : \n" << imu_accgyr_.rows() << '\n'
        << "ACCGYR IMU cols : \n" << imu_accgyr_.cols() << '\n'
        << "ACCGYR IMU: \n" << imu_accgyr_;
    if (reinit_packet_.getReinitFlag() == true) {
      LOG(INFO) 
        << "\nVIO Re-Initialized at: "<< reinit_packet_.getReinitTime() << '\n'
        << "POSE : \n" << reinit_packet_.getReinitPose() << '\n'
        << "VELOCITY : \n" << reinit_packet_.getReinitVel() << '\n'
        << "BIAS : \n" << reinit_packet_.getReinitBias();
    }
  }

  // Use only unique ptr since this class should be moved around,
  // not copied!
  typedef std::unique_ptr<StereoImuSyncPacket> UniquePtr;
  typedef std::unique_ptr<const StereoImuSyncPacket> ConstUniquePtr;

private:
  StereoFrame stereo_frame_;
  ImuStampS imu_stamps_;
  ImuAccGyrS imu_accgyr_;
  ReinitPacket reinit_packet_;
};


} // End of VIO namespace.
