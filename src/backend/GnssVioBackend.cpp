/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssVioBackend.h
 * @brief  Derived class from RegularVioBackend which uses gnss factors
 * in vio factor graph.
 *
 * A. Rosinol, T. Sattler, M. Pollefeys, and L. Carlone. Incremental
 * Visual-Inertial 3D Mesh Generation with Structural Regularities. IEEE Intl.
 * Conf. on Robotics and Automation (ICRA), 2019
 *
 * @author Elizaveta Karaseva
 */

#include "kimera-vio/backend/GnssVioBackend.h"

#include <vector>

#include "kimera-vio/factors/GnssFactor.h"

namespace VIO {

GnssVioBackend::GnssVioBackend(const Pose3& B_Pose_leftCamRect,
                               const StereoCalibPtr& stereo_calibration,
                               const BackendParams& backend_params,
                               const ImuParams& imu_params,
                               const BackendOutputParams& backend_output_params,
                               const bool& log_output,
                               std::optional<OdometryParams> odom_params)
    : RegularVioBackend(B_Pose_leftCamRect,
                        stereo_calibration,
                        backend_params,
                        imu_params,
                        backend_output_params,
                        log_output,
                        odom_params),
      gnss_vio_params_(GnssVioBackendParams::safeCast(backend_params)) {
  auto base_model =
      gtsam::noiseModel::Isotropic::Sigma(3, gnss_vio_params_.gnssNoiseSigma_);
  selectNormType(&gnss_noise_,
                 base_model,
                 gnss_vio_params_.gnssNormType_,
                 gnss_vio_params_.gnssNormParam_);
}

void GnssVioBackend::beforeOptimizeHook(const Timestamp& ts,
                                        std::optional<GnssPoint> gnss_point) {
  if (!gnss_point.has_value()) return;

  addGnssFactor(curr_kf_id_, *gnss_point, &new_imu_prior_and_other_factors_);
}

void GnssVioBackend::addGnssFactor(const FrameId& frame_id,
                                   const GnssPoint& gnss_point,
                                   gtsam::NonlinearFactorGraph* graph) {
  CHECK_NOTNULL(graph);
  const gtsam::Symbol pose_key('x', frame_id);
  graph->add(gtsam::GnssFactor(pose_key, gnss_point, gnss_noise_));
}

/* ------------------------------------------------------------------------*/
// Output a noise model with a selected norm type:
// norm_type = 0: l-2.
// norm_type = 1: Huber.
// norm_type = 2: Tukey.
void GnssVioBackend::selectNormType(
    gtsam::SharedNoiseModel* noise_model_output,
    const gtsam::SharedNoiseModel& noise_model_input,
    const size_t& norm_type,
    const double& norm_type_parameter) {
  CHECK_NOTNULL(noise_model_output);
  switch (norm_type) {
    case 0: {
      VLOG(1) << "Using l-2 norm.";
      *noise_model_output = noise_model_input;
      break;
    }
    case 1: {
      VLOG(1) << "Using Huber norm, with parameter value: "
              << norm_type_parameter;
      *noise_model_output = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(
              norm_type_parameter,
              gtsam::noiseModel::mEstimator::Huber::Scalar),  // Default is
                                                              // Block
          noise_model_input);
      break;
    }
    case 2: {
      VLOG(1) << "Using Tukey norm, with parameter value: "
              << norm_type_parameter;
      *noise_model_output = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Tukey::Create(
              norm_type_parameter,
              gtsam::noiseModel::mEstimator::Tukey::Scalar),  // Default is
                                                              // Block
          noise_model_input);                                 // robust
      break;
    }
    default: {
      LOG(ERROR) << "Wrong norm_type passed...";
      break;
    }
  }
}

}  // namespace VIO
