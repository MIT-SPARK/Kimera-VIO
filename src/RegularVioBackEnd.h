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
  using GenericProjectionFactor = gtsam::GenericStereoFactor<Pose3, Point3>;

  public:
  RegularVioBackEnd(const Pose3& leftCamPose,
                    const Cal3_S2& leftCameraCalRectified,
                    const double& baseline,
                    const VioBackEndParams& vioParams = VioBackEndParams()) :
    VioBackEnd(leftCamPose, leftCameraCalRectified, baseline, vioParams) {
    LOG(INFO) << "Using Regular VIO backend.\n";
  }

  ~RegularVioBackEnd() = default;

  public:
  // Map from ldmrk ID to corresponding factor type, true: smart.
  using LmkIdIsSmart = gtsam::FastMap<LandmarkId, bool>;
  LmkIdIsSmart lmk_id_is_smart_;
  std::vector<size_t> delete_slots_converted_factors_;

  // TODO Virtualize this appropriately,
  void addLandmarksToGraph(LandmarkIds landmarks_kf);

  void addLandmarkToGraph(const LandmarkId& lm_id, FeatureTrack& lm);

  void updateLandmarkInGraph(const LandmarkId& lm_id,
                             const std::pair<FrameId, StereoPoint2>& newObs);

  void addVisualInertialStateAndOptimize(const Timestamp& timestamp_kf_nsec, // keyframe timestamp
   const StatusSmartStereoMeasurements&
   status_smart_stereo_measurements_kf, // vision data
   const ImuStamps& imu_stamps, const ImuAccGyr& imu_accgyr, // inertial data
   const LandmarkIds& mesh_lmk_ids_ground_cluster,
   boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

  void addRegularityFactors(const LandmarkIds& mesh_lmk_ids_ground_cluster);


};

} // namespace VIO
#endif /* RegularVioBackEnd_H_ */

