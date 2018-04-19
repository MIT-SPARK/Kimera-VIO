/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackEnd.h
 * @brief  Derived class from VioBackEnd which enforces regularity constraints
 * on the factor graph.
 * @author Toni Rosinol
 * @author Luca Carlone
 */

#ifndef RegularVioBackEnd_H_
#define RegularVioBackEnd_H_

#include <VioBackEnd.h>
#include <glog/logging.h>
#include <gtsam/slam/StereoFactor.h>

namespace VIO {

class RegularVioBackEnd: public VioBackEnd {
public:
  /* ------------------------------------------------------------------------ */
  RegularVioBackEnd(const Pose3& leftCamPose,
                    const Cal3_S2& leftCameraCalRectified,
                    const double& baseline,
                    const VioBackEndParams& vioParams = VioBackEndParams());

  /* ------------------------------------------------------------------------ */
  ~RegularVioBackEnd() = default;

public:
  /* ------------------------------------------------------------------------ */
  void addVisualInertialStateAndOptimize(
      const Timestamp& timestamp_kf_nsec, // Keyframe timestamp.
      const StatusSmartStereoMeasurements&
                            status_smart_stereo_measurements_kf, // Vision data.
      const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr,  // Inertial data.
      const LandmarkIds& mesh_lmk_ids_ground_cluster,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

  /* ------------------------------------------------------------------------ */
  // TODO Virtualize this appropriately,
  void addLandmarksToGraph(const LandmarkIds& landmarks_kf);

  /* ------------------------------------------------------------------------ */
  void addLandmarkToGraph(const LandmarkId& lm_id, const FeatureTrack& lm);

  /* ------------------------------------------------------------------------ */
  void updateLandmarkInGraph(const LandmarkId& lm_id,
                             const std::pair<FrameId, StereoPoint2>& newObs);
private:
  using GenericProjectionFactor = gtsam::GenericStereoFactor<Pose3, Point3>;
  // Map from lmk ID to corresponding factor type, true: smart.
  using LmkIdIsSmart = gtsam::FastMap<LandmarkId, bool>;

  /// Members
  LmkIdIsSmart lmk_id_is_smart_;
  std::vector<size_t> delete_slots_converted_factors_;

private:
  /* ------------------------------------------------------------------------ */
  void isLandmarkSmart(const LandmarkIds& lmk_kf,
                       const LandmarkIds& mesh_lmk_ids,
                       LmkIdIsSmart* lmk_id_is_smart);

  /* ------------------------------------------------------------------------ */
  void addRegularityFactors(const LandmarkIds& mesh_lmk_ids);

};

} // namespace VIO

#endif /* RegularVioBackEnd_H_ */

