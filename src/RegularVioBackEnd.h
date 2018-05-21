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
  void addLandmarksToGraph(const LandmarkIds& lmks_kf);

  /* ------------------------------------------------------------------------ */
  void addLandmarkToGraph(const LandmarkId& lm_id, const FeatureTrack& lm);

  /* ------------------------------------------------------------------------ */
  void updateLandmarkInGraph(const LandmarkId& lm_id,
                             const std::pair<FrameId, StereoPoint2>& newObs);
private:
  typedef size_t Slot;

  // Type of handled regularities.
  enum class RegularityType{
    POINT_PLANE
  };

  using GenericProjectionFactor = gtsam::GenericStereoFactor<Pose3, Point3>;
  // Map from lmk ID to corresponding factor type, true: smart.
  using LmkIdIsSmart = gtsam::FastMap<LandmarkId, bool>;

  /// Members
  LmkIdIsSmart lmk_id_is_smart_; // TODO GROWS UNBOUNDED, use the loop in getMapLmkIdsTo3dPointsInTimeHorizon();
  std::map<LandmarkId, RegularityType> lmk_id_to_regularity_type_map_;
  std::vector<size_t> delete_slots_of_converted_smart_factors_;

  gtsam::SharedNoiseModel mono_noise_;
  boost::shared_ptr<Cal3_S2> mono_cal_;

private:
  /* ------------------------------------------------------------------------ */
  void isLandmarkSmart(const LandmarkIds& lmks_kf,
                       const LandmarkIds& mesh_lmk_ids,
                       LmkIdIsSmart* lmk_id_is_smart);

  /* ------------------------------------------------------------------------ */
  void updateExistingSmartFactor(const LandmarkId& lmk_id,
                                 const std::pair<FrameId, StereoPoint2>& new_obs,
                                 LandmarkIdSmartFactorMap* new_smart_factors,
                                 SmartFactorMap* old_smart_factors);
  /* ------------------------------------------------------------------------ */
  bool convertSmartToProjectionFactor(
      const LandmarkId& lmk_id,
      SmartFactorMap* old_smart_factors,
      gtsam::Values* new_values,
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors,
      std::vector<size_t>* delete_slots_of_converted_smart_factors);

  /* ------------------------------------------------------------------------ */
  void addProjectionFactor(
      const LandmarkId& lmk_id,
      const std::pair<FrameId, StereoPoint2>& new_obs,
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);

  /* ------------------------------------------------------------------------ */
  void addRegularityFactors(
      const LandmarkIds& mesh_lmk_ids,
      gtsam::Symbol* plane_symbol,
      std::vector<std::pair<Slot, LandmarkId>>* idx_of_point_plane_factors_to_add);

  /* ------------------------------------------------------------------------ */
  void removeOldRegularityFactors_Slow(
      const gtsam::Symbol& plane_symbol,
      const std::vector<std::pair<Slot, LandmarkId>>& idx_of_point_plane_factors_to_add,
      const LandmarkIds& mesh_lmk_ids,
      std::vector<size_t>* delete_slots);

  /* ------------------------------------------------------------------------ */
  void fillDeleteSlots(const std::vector<std::pair<Slot, LandmarkId> >& point_plane_factor_slots,
      std::vector<size_t>* delete_slots);

  /* ------------------------------------------------------------------------ */
  // Remove as well the factors that are going to be added in this iteration.
  void deleteNewSlots(
      const std::vector<std::pair<Slot, LandmarkId>>& idx_of_point_plane_factors_to_add,
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors_);

  /* ------------------------------------------------------------------------ */
  void cleanNullPtrsFromGraph(
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);
};

} // namespace VIO

#endif /* RegularVioBackEnd_H_ */

