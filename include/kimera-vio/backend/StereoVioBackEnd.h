/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVioBackEnd.h
 * @brief  Visual-Inertial Odometry pipeline, as described in these papers:
 *
 * C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza.
 * On-Manifold Preintegration Theory for Fast and Accurate Visual-Inertial
 * Navigation. IEEE Trans. Robotics, 33(1):1-21, 2016.
 *
 * L. Carlone, Z. Kira, C. Beall, V. Indelman, and F. Dellaert.
 * Eliminating Conditionally Independent Sets in Factor Graphs: A Unifying
 * Perspective based on Smart Factors. In IEEE Intl. Conf. on Robotics and
 * Automation (ICRA), 2014.
 *
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include "kimera-vio/backend/VioBackEnd.h"

#include <gtsam/geometry/StereoCamera.h>
#include <gtsam/geometry/StereoPoint2.h>

namespace VIO {

class StereoVioBackEnd : public VioBackEnd {
 public:
  KIMERA_DELETE_COPY_CONSTRUCTORS(StereoVioBackEnd);
  KIMERA_POINTER_TYPEDEFS(StereoVioBackEnd);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoVioBackEnd(const Pose3& B_Pose_leftCam,
             const StereoCalibPtr& stereo_calibration,
             const BackendParams& backend_params,
             const ImuParams& imu_params,
             const BackendOutputParams& backend_output_params,
             bool log_output);

  virtual ~StereoVioBackEnd() { LOG(INFO) << "StereoBackEnd destructor called."; }

 public:
  /* ------------------------------------------------------------------------ */
  BackendOutput::UniquePtr spinOnce(const BackendInput& input) override;

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points - TODO: this copies the graph.
  // void get3DPoints(std::vector<gtsam::Point3>* points_3d) const override;

  /* ------------------------------------------------------------------------ */
  // Get valid 3D points and corresponding lmk id.
  // Warning! it modifies old_smart_factors_!!
  PointsWithIdMap getMapLmkIdsTo3dPointsInTimeHorizon(
      LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map = nullptr,
      const size_t& min_age = 2);

 protected:
  /* ------------------------------------------------------------------------ */
  // Uses landmark table to add factors in graph.
  virtual void addLandmarksToGraph(const LandmarkIds& landmarks_kf) override;

  /* ------------------------------------------------------------------------ */
  // Adds a landmark to the graph for the first time.
  virtual void addLandmarkToGraph(const LandmarkId& lm_id,
                                  const FeatureTrack& lm) override;

  /* ------------------------------------------------------------------------ */
  // Store stereo frame info into landmarks table:
  // returns landmarks observed in current frame.
  void addStereoMeasurementsToFeatureTracks(
      const int& frameNum,
      const StereoMeasurements& stereoMeasurements_kf,
      LandmarkIds* landmarks_kf);

  /* ------------------------------------------------------------------------ */
  virtual bool addVisualInertialStateAndOptimize(const BackendInput& input);

  /* ------------------------------------------------------------------------ */
  // Workhorse that stores data and optimizes at each keyframe.
  // [in] timestamp_kf_nsec, keyframe timestamp.
  // [in] status_smart_stereo_measurements_kf, vision data.
  // [in] stereo_ransac_body_pose, inertial data.
  virtual bool addVisualInertialStateAndOptimize(
      const Timestamp& timestamp_kf_nsec,
      const StatusStereoMeasurements& status_smart_stereo_measurements_kf,
      const gtsam::PreintegrationType& pim,
      boost::optional<gtsam::Pose3> stereo_ransac_body_pose = boost::none);

  /* ------------------------------------------------------------------------ */
  virtual void updateLandmarkInGraph(
      const LandmarkId& lmk_id,
      const std::pair<FrameId, StereoPoint2>& new_measurement);

  /* ------------------------------------------------------------------------ */
  void print() const override;

 protected:
  // Pose of the left camera wrt body
  const Pose3 B_Pose_leftCam_;
  // Stores calibration, baseline.
  const gtsam::Cal3_S2Stereo::shared_ptr stereo_cal_;
};

}  // namespace VIO
