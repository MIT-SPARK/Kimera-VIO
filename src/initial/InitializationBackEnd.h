/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   InitializationBackEnd.h
 * @brief  Derived class from VioBackEnd for bundle adjustment and alignment
 * for the online initialization
 * @author Sandro Berchier
 * @author Luca Carlone
 */

#ifndef InitializationBackEnd_H_
#define InitializationBackEnd_H_

#include <queue>

#include "utils/Timer.h"
#include "VioBackEnd.h"
#include "InitializationBackEnd-definitions.h"

namespace VIO {

class InitializationBackEnd: public VioBackEnd {
public:

  /* ------------------------------------------------------------------------ */
  // Create and initialize InitializationBackEnd, without initiaing pose.
  InitializationBackEnd(const Pose3& leftCamPose,
             const Cal3_S2& leftCameraCalRectified,
             const double& baseline,
             const VioBackEndParams& vioParams,
             const bool log_output = false);

  /* ------------------------------------------------------------------------ */
  ~InitializationBackEnd() = default;

public:
  /* ------------------------------------------------------------------------ */
  // Perform Bundle-Adjustment and initial gravity alignment
  bool bundleAdjustmentAndGravityAlignment(
    std::queue<InitializationInputPayload>& output_frontend,
      gtsam::Vector3 *gyro_bias,
      gtsam::Vector3 *g_iter_b0,
      gtsam::NavState *init_navstate);

public:
  /* ------------------------------------------------------------------------ */
  std::vector<gtsam::Pose3> addInitialVisualStatesAndOptimize(
      const std::vector<std::shared_ptr<VioBackEndInputPayload>> &input);

private:
  /* --------------------------------------------------------------------------
   */
  // Adding of states for bundle adjustment used in initialization.
  // [in] timestamp_kf_nsec, keyframe timestamp.
  // [in] status_smart_stereo_measurements_kf, vision data.
  // [in] stereo_ransac_body_pose, inertial data.
  virtual void addInitialVisualState(
      const Timestamp &timestamp_kf_nsec,
      const StatusSmartStereoMeasurements &status_smart_stereo_measurements_kf,
      std::vector<Plane> *planes,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose,
      const int verbosity);

  /* --------------------------------------------------------------------------
   */
  std::vector<gtsam::Pose3> optimizeInitialVisualStates(
      const Timestamp &timestamp_kf_nsec, const FrameId &cur_id,
      const size_t &max_extra_iterations,
      const std::vector<size_t> &extra_factor_slots_to_delete =
          std::vector<size_t>(),
      const int verbosity = 0);

};

} // namespace VIO

#endif /* InitializationBackEnd_H_ */