/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   OnlineGravityAlignment-definitions.h
 * @brief  Contains the visual inertial alignment frame definition.
 * @author Antoni Rosinol
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#pragma once

#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/NavState.h>

namespace VIO {

// Class that deals with single frames in initialization
class VisualInertialFrame {
 public:
  VisualInertialFrame(const gtsam::Pose3 &curr_body_pose,
                      const gtsam::Pose3 &prev_body_pose,
                      const double &cam_dt,
                      const gtsam::NavState &delta_state,
                      const gtsam::Matrix3 &dbg_Jacobian_dR,
                      const double &pim_dt)
      : b0_R_bk_(prev_body_pose.rotation().matrix()),
        b0_p_bk_(prev_body_pose.translation()),
        b0_R_bkp1_(curr_body_pose.rotation().matrix()),
        b0_p_bkp1_(curr_body_pose.translation()),
        bk_alpha_bkp1_(delta_state.pose().translation()),
        bk_beta_bkp1_(delta_state.velocity()),
        bk_gamma_bkp1_(delta_state.pose().rotation().matrix()),
        dbg_Jacobian_dR_(dbg_Jacobian_dR),
        cam_dt_(cam_dt),
        pim_dt_(pim_dt),
        dt_bk_(0.5 * (cam_dt + pim_dt)){};  // TODO(Toni): remove hardcoded.

  ~VisualInertialFrame() = default;

 public:
  inline double camDt() const { return cam_dt_; }
  inline double pimDt() const { return pim_dt_; }
  inline double dtBk() const { return dt_bk_; }
  inline gtsam::Matrix3 b0Rbkp1() const { return b0_R_bkp1_; }
  inline gtsam::Matrix3 b0Rbk() const { return b0_R_bk_; }
  inline gtsam::Vector3 b0Pbkp1() const { return b0_p_bkp1_; }
  inline gtsam::Vector3 b0Pbk() const { return b0_p_bk_; }
  gtsam::Matrix3 bkRbkp1() const { return (b0_R_bk_.transpose() * b0_R_bkp1_); }
  gtsam::Vector3 bkPbkp1() const { return (b0_p_bkp1_ - b0_p_bk_); }
  inline gtsam::Matrix3 bkGammaBkp1() const { return bk_gamma_bkp1_; }
  inline gtsam::Vector3 bkAlphaBkp1() const { return bk_alpha_bkp1_; }
  inline gtsam::Vector3 bkBetaBkp1() const { return bk_beta_bkp1_; }
  inline gtsam::Matrix3 dbgJacobianDr() const { return dbg_Jacobian_dR_; }
  gtsam::Matrix3 A11() const { return (-dt_bk_ * b0_R_bk_.transpose()); }
  gtsam::Matrix3 A13() const {
    return (0.5 * dt_bk_ * dt_bk_ * b0_R_bk_.transpose());
  }
  gtsam::Vector3 b1() const {
    return (bk_alpha_bkp1_ - (b0_R_bk_.transpose() * bkPbkp1()));
  }
  gtsam::Matrix3 A21() const { return (-b0_R_bk_.transpose()); }
  gtsam::Matrix3 A22() const { return b0_R_bk_.transpose(); }
  gtsam::Matrix3 A23() const { return (dt_bk_ * b0_R_bk_.transpose()); }
  inline gtsam::Vector3 b2() const { return bk_beta_bkp1_; }
  gtsam::Pose3 b0Tbk() const {
    return gtsam::Pose3(gtsam::Rot3(b0_R_bk_), b0_p_bk_);
  }
  // Updates delta state with new delta state in visual inertial frame.
  // [in]: New pim delta state to be inserted in visual inertial frame.
  void updateDeltaState(const gtsam::NavState &delta_state);

 private:
  const gtsam::Matrix3 b0_R_bk_;
  const gtsam::Vector3 b0_p_bk_;
  const gtsam::Matrix3 b0_R_bkp1_;
  const gtsam::Vector3 b0_p_bkp1_;
  gtsam::Vector3 bk_alpha_bkp1_;
  gtsam::Vector3 bk_beta_bkp1_;
  gtsam::Matrix3 bk_gamma_bkp1_;
  const gtsam::Matrix3 dbg_Jacobian_dR_;
  const double cam_dt_;
  const double pim_dt_;
  const double dt_bk_;
};

}  // namespace VIO
