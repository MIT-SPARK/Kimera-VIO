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
      std::vector<Plane>* planes = nullptr,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

  /* ------------------------------------------------------------------------ */
  // TODO Virtualize this appropriately,
  void addLandmarksToGraph(const LandmarkIds& lmks_kf,
                           const LandmarkIds& lmk_ids_with_regularity);

  /* ------------------------------------------------------------------------ */
  void addLandmarkToGraph(const LandmarkId& lm_id, const FeatureTrack& lm);

  /* ------------------------------------------------------------------------ */
  void updateLandmarkInGraph(const LandmarkId& lmk_id,
                             const bool& is_lmk_smart,
                             const std::pair<FrameId, StereoPoint2>& new_obs);
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

  // For Stereo and Projection factors.
  gtsam::SharedNoiseModel stereo_noise_;
  gtsam::SharedNoiseModel mono_noise_;
  boost::shared_ptr<Cal3_S2> mono_cal_;

  // For regularity factors.
  gtsam::SharedNoiseModel point_plane_regularity_noise_;

private:
  /* ------------------------------------------------------------------------ */
  bool isLandmarkSmart(const LandmarkId& lmk_id,
                       const LandmarkIds& lmk_ids_with_regularity,
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
  void convertExtraSmartFactorToProjFactor(
      const LandmarkIds& lmk_ids_with_regularity);

  /* ------------------------------------------------------------------------ */
  virtual void deleteLmkFromExtraStructures(const LandmarkId& lmk_id);

  /* ------------------------------------------------------------------------ */
  void addProjectionFactor(
      const LandmarkId& lmk_id,
      const std::pair<FrameId, StereoPoint2>& new_obs,
      gtsam::NonlinearFactorGraph* new_imu_prior_and_other_factors);

  /* ------------------------------------------------------------------------ */
  void addRegularityFactors(
      const LandmarkIds& mesh_lmk_ids,
      const Plane& plane,
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
  // Output a noise model with a selected norm type:
  // norm_type = 0: l-2.
  // norm_type = 1: Huber.
  // norm_type = 2: Tukey.
  void selectNormType(
      gtsam::SharedNoiseModel* noise_model_output,
      const gtsam::SharedNoiseModel& noise_model_input,
      const size_t& norm_type);

  /* ------------------------------------------------------------------------ */
  // Extract all lmk ids, wo repetition, from the set of planes.
  void extractLmkIdsFromPlanes(const std::vector<Plane>& planes,
                               LandmarkIds* lmk_ids_with_regularity) const;

  /* ------------------------------------------------------------------------ */
  // Update plane normal and distance if the plane could be found in the state.
  // Otherwise, erase the plane.
  void updatePlaneEstimates(std::vector<Plane>* planes);

};

} // namespace VIO

#endif /* RegularVioBackEnd_H_ */

