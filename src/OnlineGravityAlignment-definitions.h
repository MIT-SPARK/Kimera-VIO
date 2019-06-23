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
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#ifndef OnlineGravityAlignmentdefinitions_H_
#define OnlineGravityAlignmentdefinitions_H_

#include <boost/foreach.hpp>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>

namespace VIO {

// Class that deals with single frames in initialization
class VisualInertialFrame {
public:
  VisualInertialFrame(const gtsam::Pose3 &curr_body_pose,
                 const gtsam::Pose3 &prev_body_pose,
                 const double &cam_dt,
                 gtsam::NavState &delta_state,
                 const gtsam::Matrix3& dbg_Jacobian_dR,
                 const double &pim_dt)
      : cam_dt_(cam_dt),
        b0_R_bk_(prev_body_pose.rotation().matrix()),
        b0_p_bk_(prev_body_pose.translation()),
        b0_R_bkp1_(curr_body_pose.rotation().matrix()),
        b0_p_bkp1_(curr_body_pose.translation()),
        bk_alpha_bkp1_(delta_state.pose().translation()),
        bk_beta_bkp1_(delta_state.velocity()),
        bk_gamma_bkp1_(delta_state.pose().rotation().matrix()),
        dbg_Jacobian_dR_(dbg_Jacobian_dR),
        pim_dt_(pim_dt),
        dt_bk_(0.5*(cam_dt + pim_dt)) {};

  ~VisualInertialFrame() = default;

public:
  /* ------------------------------------------------------------------------------- */
  inline double cam_dt() const { return cam_dt_; }
  /* ------------------------------------------------------------------------------- */
  inline double pim_dt() const { return pim_dt_; }
  /* ------------------------------------------------------------------------------- */
  inline double dt_bk() const { return dt_bk_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix3 b0_R_bkp1() const { return b0_R_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix3 b0_R_bk() const { return b0_R_bk_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector3 b0_p_bkp1() const { return b0_p_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector3 b0_p_bk() const { return b0_p_bk_; }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 bk_R_bkp1() const { return (b0_R_bk_.transpose() * b0_R_bkp1_); }
  /* ------------------------------------------------------------------------------- */
  gtsam::Vector3 bk_p_bkp1() const { return (b0_p_bkp1_ - b0_p_bk_); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix3 bk_gamma_bkp1() const { return bk_gamma_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector3 bk_alpha_bkp1() const { return bk_alpha_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector3 bk_beta_bkp1() const { return bk_beta_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Matrix3 dbg_jacobian_dR() const { return dbg_Jacobian_dR_; }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 A_11() const { return (-dt_bk_*b0_R_bk_); }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 A_13() const { return (0.5 * dt_bk_ * dt_bk_ * b0_R_bk_); }
  /* ------------------------------------------------------------------------------- */
  gtsam::Vector3 b_1() const { return (bk_alpha_bkp1_ - (b0_R_bk_ * bk_p_bkp1())); }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 A_21() const { return (-b0_R_bk_); }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 A_22() const { return b0_R_bk_; }
  /* ------------------------------------------------------------------------------- */
  gtsam::Matrix3 A_23() const { return (dt_bk_ * b0_R_bk_); }
  /* ------------------------------------------------------------------------------- */
  inline gtsam::Vector3 b_2() const { return bk_beta_bkp1_; }
  /* ------------------------------------------------------------------------------- */
  void updateDeltaState(const gtsam::NavState &delta_state) {
    bk_alpha_bkp1_ = gtsam::Vector3(delta_state.pose().translation());
    bk_beta_bkp1_ = delta_state.velocity();
    bk_gamma_bkp1_ = delta_state.pose().rotation().matrix();
  }

private:
  const gtsam::Matrix3 b0_R_bk_;
  const gtsam::Vector3 b0_p_bk_; 
  const gtsam::Matrix3 b0_R_bkp1_;
  const gtsam::Vector3 b0_p_bkp1_;
  const double cam_dt_;
  gtsam::Vector3 bk_alpha_bkp1_;
  gtsam::Vector3 bk_beta_bkp1_;
  gtsam::Matrix3 bk_gamma_bkp1_;
  const gtsam::Matrix3 dbg_Jacobian_dR_;
  const double pim_dt_;
  const double dt_bk_;
};

} // namespace VIO

#endif /* OnlineGravityAlignmentdefinitions_H_ */
