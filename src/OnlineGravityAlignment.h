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

#include "ImuFrontEnd.h"

namespace VIO {

class AlignmentFrame {
public:
  AlignmentFrame(const gtsam::Pose3 &current_body_pose,
                 const gtsam::Pose3 &previous_body_pose,
                 gtsam::NavState &delta_state, const double &delta_time)
      : current_body_pose_(current_body_pose),
        previous_body_pose_(previous_body_pose), delta_state_(delta_state),
        delta_time_(delta_time){};

  ~AlignmentFrame() = default;

public:
  /* ------------------------------------------------------------------------ */
  inline double dt() const { return delta_time_; }
  /* ------------------------------------------------------------------------ */
  inline gtsam::Matrix R_mat() const {
    return current_body_pose_.rotation().matrix();
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Vector p() const { return current_body_pose_.translation(); }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Matrix prev_R_mat() const {
    return previous_body_pose_.rotation().matrix();
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Vector prev_p() const {
    return previous_body_pose_.translation();
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Matrix delta_R_mat() const {
    return delta_state_.pose().rotation().matrix();
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Vector delta_p() const {
    return delta_state_.pose().translation();
  }

  /* ------------------------------------------------------------------------ */
  inline gtsam::Vector delta_v() const { return delta_state_.velocity(); }

  /* ------------------------------------------------------------------------ */
  inline gtsam::NavState navstate() const { return delta_state_; }

  /* ------------------------------------------------------------------------ */
  inline void updateDeltaState(const gtsam::NavState &delta_state) {
    delta_state_ = delta_state;
  }

private:
  const gtsam::Pose3 &current_body_pose_;
  const gtsam::Pose3 &previous_body_pose_;
  gtsam::NavState &delta_state_;
  const double &delta_time_;
};

typedef std::vector<gtsam::Pose3> AlignmentPoses;
typedef std::vector<gtsam::NavState> AlignmentDeltaStates;
typedef std::vector<gtsam::PreintegratedImuMeasurements> AlignmentPims;
typedef std::vector<AlignmentFrame> AlignmentFrames;
typedef Vector3 GyroBias;

class OnlineGravityAlignment {
public:
  /* ------------------------------------------------------------------------ */
  OnlineGravityAlignment(const AlignmentPoses &estimated_body_poses,
                         const AlignmentPims &pims,
                         const gtsam::Vector3 &g_world, GyroBias *gyro_bias);

  /* ------------------------------------------------------------------------ */
  ~OnlineGravityAlignment() = default;

public:
  /* ------------------------------------------------------------------------ */
  bool alignVisualInertialEstimates();

private:
  /* ------------------------------------------------------------------------ */
  void constructFrames(const AlignmentPoses &estimated_body_poses,
                       const AlignmentPims &pims, AlignmentFrames *frames);

  /* ------------------------------------------------------------------------ */
  bool estimateGyroscopeBias(const AlignmentFrames &frames,
                             const AlignmentPims &pims, GyroBias *gyro_bias);

  /* ------------------------------------------------------------------------ */
  bool updateDeltaStates(const AlignmentPims &pims, const GyroBias &delta_bg,
                         AlignmentFrames *frames);

  /* ------------------------------------------------------------------------ */
  bool alignEstimatesLinearly(const AlignmentFrames &frames,
                              const gtsam::Vector3 &g_world,
                              gtsam::Vector3 *g_iter);

  /* ------------------------------------------------------------------------ */
  gtsam::Matrix createTangentBasis(const gtsam::Vector3 &g0);

  /* ------------------------------------------------------------------------ */
  void refineGravity(const AlignmentFrames &frames,
                     const gtsam::Vector3 &g_world, gtsam::Vector3 *g_iter);

private:
  const AlignmentPims &pims_;
  const AlignmentPoses &estimated_body_poses_;
  const gtsam::Vector3 &g_world_;
  GyroBias &gyro_bias_;
};

} // namespace VIO

#endif /* OnlineGravityAlignment_H_ */
