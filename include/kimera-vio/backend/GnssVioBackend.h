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

#pragma once

#include <vector>

#include "kimera-vio/backend/GnssVioBackendParams.h"
#include "kimera-vio/backend/RegularVioBackend.h"
#include "kimera-vio/frontend/Gnss.h"

namespace VIO {

class GnssVioBackend : public RegularVioBackend {
 public:
  GnssVioBackend(const Pose3& B_Pose_leftCamRect,
                 const StereoCalibPtr& stereo_calibration,
                 const BackendParams& backend_params,
                 const ImuParams& imu_params,
                 const BackendOutputParams& backend_output_params,
                 const bool& log_output,
                 std::optional<OdometryParams> odom_params = std::nullopt);

  virtual ~GnssVioBackend() = default;

  void beforeOptimizeHook(
      const Timestamp& ts,
      std::optional<GnssPoint> gnss_point = std::nullopt) override;

 private:
  const GnssVioBackendParams gnss_vio_params_;
  gtsam::SharedNoiseModel gnss_noise_;
  /* ------------------------------------------------------------------------ */
  // Output a noise model with a selected norm type:
  // norm_type = 0: l-2.
  // norm_type = 1: Huber.
  // norm_type = 2: Tukey.
  void selectNormType(gtsam::SharedNoiseModel* noise_model_output,
    const gtsam::SharedNoiseModel& noise_model_input,
    const size_t& norm_type,
    const double& norm_type_parameter);

  void addGnssFactor(const FrameId& frame_id,
                     const GnssPoint& gnss_point,
                     gtsam::NonlinearFactorGraph* graph);
};

}  // namespace VIO
