/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OnlineGravityAlignment.h
 * @brief  Contains initial Online Gravity Alignment functions.
 *
 * Qin, Tong, and Shaojie Shen.
 * Robust initialization of monocular visual-inertial estimation on aerial
 * robots. International Conference on Intelligent Robots and Systems (IROS).
 * IEEE, 2017.
 *
 * @author Antoni Rosinol
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#pragma once

#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/AHRSFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "kimera-vio/imu-frontend/ImuFrontend.h"
#include "kimera-vio/initial/OnlineGravityAlignment-definitions.h"

namespace VIO {

// Typedefs for online initialization
typedef std::vector<gtsam::Pose3> AlignmentPoses;
typedef std::vector<ImuFrontend::PimPtr> AlignmentPims;
typedef std::vector<VisualInertialFrame> VisualInertialFrames;
typedef std::vector<gtsam::AHRSFactor::PreintegratedMeasurements>
    InitialAHRSPims;

// Class with functions for online initialization
class OnlineGravityAlignment {
 public:
  /* ------------------------------------------------------------------------ */
  OnlineGravityAlignment(const AlignmentPoses &estimated_body_poses,
                         const std::vector<double> &delta_t_camera,
                         const AlignmentPims &pims,
                         const gtsam::Vector3 &g_world,
                         const InitialAHRSPims &ahrs_pims = InitialAHRSPims());

  /* ------------------------------------------------------------------------ */
  ~OnlineGravityAlignment() = default;

 public:
  /* ------------------------------------------------------------------------ */
  bool alignVisualInertialEstimates(gtsam::Vector3 *gyro_bias,
                                    gtsam::Vector3 *g_iter,
                                    gtsam::NavState *init_navstate,
                                    const bool &estimate_bias = true);

  /* ------------------------------------------------------------------------ */
  static gtsam::Matrix createTangentBasis(const gtsam::Vector3 &g0);

  /* ------------------------------------------------------------------------ */
  bool estimateGyroscopeBiasOnly(gtsam::Vector3 *gyro_bias,
                                 const bool &use_ahrs_estimator = false);

  /* ------------------------------------------------------------------------ */
  gtsam::Vector3 estimateGyroscopeResiduals(
      const VisualInertialFrames &vi_frames);

 private:
  /* ------------------------------------------------------------------------ */
  void constructVisualInertialFrames(const AlignmentPoses &estimated_body_poses,
                                     const std::vector<double> &delta_t_camera,
                                     const AlignmentPims &pims,
                                     VisualInertialFrames *vi_frames);

  /* ------------------------------------------------------------------------ */
  bool estimateBiasAndUpdateStates(const AlignmentPims &pims,
                                   const InitialAHRSPims &ahrs_pims,
                                   gtsam::Vector3 *gyro_bias,
                                   VisualInertialFrames *vi_frames,
                                   const bool &use_ahrs_estimator);

  /* ------------------------------------------------------------------------ */
  void estimateGyroscopeBias(const VisualInertialFrames &vi_frames,
                             gtsam::Vector3 *gyro_bias);

  /* ------------------------------------------------------------------------ */
  void estimateGyroscopeBiasAHRS(const VisualInertialFrames &vi_frames,
                                 const InitialAHRSPims &ahrs_pims,
                                 gtsam::Vector3 *gyro_bias);

  /* ------------------------------------------------------------------------ */
  void updateDeltaStates(const AlignmentPims &pims,
                         const gtsam::Vector3 &delta_bg,
                         VisualInertialFrames *vi_frames);

  /* ------------------------------------------------------------------------ */
  bool alignEstimatesLinearly(const VisualInertialFrames &vi_frames,
                              const gtsam::Vector3 &g_world,
                              gtsam::Vector3 *g_iter,
                              gtsam::Velocity3 *init_vel);

  /* ------------------------------------------------------------------------ */
  void refineGravity(const VisualInertialFrames &vi_frames,
                     const gtsam::Vector3 &g_world,
                     gtsam::Vector3 *g_iter,
                     gtsam::Velocity3 *init_vel);

 private:
  const AlignmentPims pims_;
  const AlignmentPoses estimated_body_poses_;
  const std::vector<double> delta_t_camera_;
  const gtsam::Vector3 g_world_;
  const InitialAHRSPims ahrs_pims_;
};

}  // namespace VIO
