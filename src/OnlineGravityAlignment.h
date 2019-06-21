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

namespace VIO {

// Class that deals with single frames in initialization
class VisualInertialFrame {
public:
  VisualInertialFrame(const gtsam::Pose3 &curr_body_pose,
                 const gtsam::Pose3 &prev_body_pose,
                 const double &delta_time_camera,
                 gtsam::NavState &delta_state,
                 const gtsam::Matrix3& dbg_Jacobian_dR,
                 const double &delta_time_pim)
      : curr_body_pose_(curr_body_pose),
        prev_body_pose_(prev_body_pose),
        delta_time_camera_(delta_time_camera), 
        delta_state_(delta_state),
        dbg_Jacobian_dR_(dbg_Jacobian_dR),
        delta_time_pim_(delta_time_pim){};

  ~VisualInertialFrame() = default;

public:
  /* ------------------------------------------------------------------------------- */
  inline double cam_dt() const { return delta_time_camera_; }
  /* ------------------------------------------------------------------------------- */
  inline double pim_dt() const { return delta_time_pim_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix b0_R_bkp1() const { return curr_body_pose_.rotation().matrix(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix b0_R_bk() const { return prev_body_pose_.rotation().matrix(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector curr_p() const { return curr_body_pose_.translation(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector prev_p() const { return prev_body_pose_.translation(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Rot3 bk_gamma_bkp1() const { return delta_state_.pose().rotation(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector delta_p() const { return delta_state_.pose().translation(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector delta_v() const { return delta_state_.velocity(); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix3 dbg_jacobian_dR() const { return dbg_Jacobian_dR_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Rot3 bk_R_bkp1() const {
    return gtsam::Rot3(b0_R_bk().transpose() * b0_R_bkp1());
  }
  /* ------------------------------------------------------------------------------- */
  inline void updateDeltaState(const gtsam::NavState &delta_state) {
    delta_state_ = delta_state;
  }

private:
  const gtsam::Pose3 curr_body_pose_;
  const gtsam::Pose3 prev_body_pose_;
  const gtsam::Matrix3 dbg_Jacobian_dR_;
  gtsam::NavState delta_state_;
  const double delta_time_camera_;
  const double delta_time_pim_;
};

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
                         const gtsam::Vector3 &g_world,
                         gtsam::Vector3 *gyro_bias);

  /* ------------------------------------------------------------------------ */
  ~OnlineGravityAlignment() = default;

public:
  /* ------------------------------------------------------------------------ */
  bool alignVisualInertialEstimates();

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
  bool updateDeltaStates(const AlignmentPims &pims,
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
  gtsam::Vector3 &gyro_bias_;
};

} // namespace VIO

#endif /* OnlineGravityAlignment_H_ */
