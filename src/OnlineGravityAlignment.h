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
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#ifndef OnlineGravityAlignment_H_
#define OnlineGravityAlignment_H_

#include <boost/foreach.hpp>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/Values.h>

#include "ImuFrontEnd.h"
#include "OnlineGravityAlignment-definitions.h"

namespace VIO {

// Typedefs for online initialization
typedef std::vector<gtsam::Pose3> AlignmentPoses;
typedef std::vector<gtsam::PreintegratedImuMeasurements> AlignmentPims;
typedef std::vector<VisualInertialFrame> VisualInertialFrames;

// Class with functions for online initialization
class OnlineGravityAlignment {
public:
  /* ------------------------------------------------------------------------ */
  OnlineGravityAlignment(const AlignmentPoses &estimated_body_poses,
                         const std::vector<double> &delta_t_camera,
                         const AlignmentPims &pims,
                         const gtsam::Vector3 &g_world);

  /* ------------------------------------------------------------------------ */
  ~OnlineGravityAlignment() = default;

public:
  /* ------------------------------------------------------------------------ */
  bool alignVisualInertialEstimates(gtsam::Vector3 *gyro_bias, 
                                    gtsam::Vector3 *g_iter);

private:
  /* ------------------------------------------------------------------------ */
  void constructVisualInertialFrames(const AlignmentPoses &estimated_body_poses,
                              const std::vector<double> &delta_t_camera,
                              const AlignmentPims &pims,
                              VisualInertialFrames *vi_frames);

  /* ------------------------------------------------------------------------ */
  bool estimateGyroscopeBias(const VisualInertialFrames &vi_frames,
                              gtsam::Vector3 *gyro_bias);

  /* ------------------------------------------------------------------------ */
  void updateDeltaStates(const AlignmentPims &pims,
                         const gtsam::Vector3 &delta_bg,
                         VisualInertialFrames *vi_frames);

  /* ------------------------------------------------------------------------ */
  bool alignEstimatesLinearly(const VisualInertialFrames &vi_frames,
                              const gtsam::Vector3 &g_world,
                              gtsam::Vector3 *g_iter);

  /* ------------------------------------------------------------------------ */
  gtsam::Matrix createTangentBasis(const gtsam::Vector3 &g0);

  /* ------------------------------------------------------------------------ */
  void refineGravity(const VisualInertialFrames &vi_frames,
                     const gtsam::Vector3 &g_world, gtsam::Vector3 *g_iter);

private:
  const AlignmentPims &pims_;
  const AlignmentPoses &estimated_body_poses_;
  const std::vector<double> &delta_t_camera_;
  const gtsam::Vector3 &g_world_;
};

} // namespace VIO

#endif /* OnlineGravityAlignment_H_ */
