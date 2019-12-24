/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file  VisionFrontEndModule.cpp
 * @brief
 * @author Antoni Rosinol
 */

#include "kimera-vio/frontend/VisionFrontEndModule.h"

namespace VIO {

StereoVisionFrontEndModule::StereoVisionFrontEndModule(
    InputQueue* input_queue,
    bool parallel_run,
    StereoVisionFrontEnd::UniquePtr vio_frontend)
    : SIMO(input_queue, "VioFrontEnd", parallel_run),
      vio_frontend_(std::move(vio_frontend)) {
  CHECK(vio_frontend_);
}

StereoVisionFrontEndModule::OutputUniquePtr
StereoVisionFrontEndModule::spinOnce(StereoImuSyncPacket::UniquePtr input) {
  CHECK(input);
  return vio_frontend_->spinOnce(*input);
}

//! Frontend Initialization
StereoFrame StereoVisionFrontEndModule::processFirstStereoFrame(
    const StereoFrame& first_frame) {
  return vio_frontend_->processFirstStereoFrame(first_frame);
}

void StereoVisionFrontEndModule::resetFrontendAfterOnlineAlignment(
    const gtsam::Vector3& gravity,
    gtsam::Vector3& gyro_bias) {
  vio_frontend_->resetFrontendAfterOnlineAlignment(gravity, gyro_bias);
}

void StereoVisionFrontEndModule::prepareFrontendForOnlineAlignment() {
  vio_frontend_->prepareFrontendForOnlineAlignment();
}

// Check values in frontend for initial bundle adjustment for online alignment
void StereoVisionFrontEndModule::checkFrontendForOnlineAlignment() {
  vio_frontend_->checkFrontendForOnlineAlignment();
}

};  // namespace VIO
