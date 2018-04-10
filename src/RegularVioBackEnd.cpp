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
 */

#include "RegularVioBackEnd.h"

namespace VIO {

void RegularVioBackEnd::addVisualInertialStateAndOptimize(
    const Timestamp timestamp_kf_nsec,
    const StatusSmartStereoMeasurements status_smart_stereo_measurements_kf,
    ImuStamps imu_stamps, ImuAccGyr imu_accgyr,
    boost::optional<gtsam::Pose3> stereo_ransac_body_pose)
{
  debugInfo_.resetAddedFactorsStatistics();

  if (verbosity_ >= 7) { StereoVisionFrontEnd::PrintStatusStereoMeasurements(status_smart_stereo_measurements_kf); }

  // Features and IMU line up --> do iSAM update
  last_id_ = cur_id_;
  ++cur_id_;

  timestamp_kf_ = UtilsOpenCV::NsecToSec(timestamp_kf_nsec);

  std::cout << "RegularVIO: adding keyframe " << cur_id_ << " at timestamp:" << timestamp_kf_ << " (sec)" << std::endl;

  /////////////////// MANAGE IMU MEASUREMENTS ///////////////////////////
  // Predict next step, add initial guess
  integrateImuMeasurements(imu_stamps, imu_accgyr);
  addImuValues(cur_id_);

  // add imu factors between consecutive keyframe states
  addImuFactor(last_id_, cur_id_);

  // add between factor from RANSAC
  if(stereo_ransac_body_pose){
    std::cout << "RegularVIO: adding between " << std::endl;
    (*stereo_ransac_body_pose).print();
    addBetweenFactor(last_id_, cur_id_, *stereo_ransac_body_pose);
  }

  /////////////////// MANAGE VISION MEASUREMENTS ///////////////////////////
  SmartStereoMeasurements smartStereoMeasurements_kf = status_smart_stereo_measurements_kf.second;

  // if stereo ransac failed, remove all right pixels:
  Tracker::TrackingStatus kfTrackingStatus_stereo = status_smart_stereo_measurements_kf.first.kfTrackingStatus_stereo_;
  // if(kfTrackingStatus_stereo == Tracker::TrackingStatus::INVALID){
  //   for(size_t i = 0; i < smartStereoMeasurements_kf.size(); i++)
  //     smartStereoMeasurements_kf[i].uR = std::numeric_limits<double>::quiet_NaN();;
  //}

  // extract relevant information from stereo frame
  LandmarkIds landmarks_kf = addStereoMeasurementsToFeatureTracks(cur_id_, smartStereoMeasurements_kf);
  if (verbosity_ >= 8) { printFeatureTracks(); }

  // decide which factors to add
  Tracker::TrackingStatus kfTrackingStatus_mono = status_smart_stereo_measurements_kf.first.kfTrackingStatus_mono_;
  switch(kfTrackingStatus_mono){
  case Tracker::TrackingStatus::LOW_DISPARITY :  // vehicle is not moving
    if (verbosity_ >= 7) {printf("Add zero velocity and no motion factors\n");}
    addZeroVelocityPrior(cur_id_);
    addNoMotionFactor(last_id_, cur_id_);
    break;

    // This did not improve in any case
    //  case Tracker::TrackingStatus::INVALID :// ransac failed hence we cannot trust features
    //    if (verbosity_ >= 7) {printf("Add constant velocity factor (monoRansac is INVALID)\n");}
    //    addConstantVelocityFactor(last_id_, cur_id_);
    //    break;

  default: // Tracker::TrackingStatus::VALID, FEW_MATCHES, INVALID, DISABLED : // we add features in VIO
    addLandmarksToGraph(landmarks_kf);
    break;
  }

  imu_bias_prev_kf_ = imu_bias_lkf_; // this lags 1 step behind to mimic hw
  // TODO add conversion from Smart factor to regular.
  optimize(cur_id_, vioParams_.numOptimize_);
}

void RegularVioBackEnd::addLandmarkToGraph(LandmarkId lm_id, FeatureTrack& ft)
{
  if(ft.in_ba_graph_)
    throw std::runtime_error("addLandmarkToGraph: feature already in the graph!");

  ft.in_ba_graph_ = true;

  // We use a unit pinhole projection camera for the smart factors to be more efficient.
  SmartStereoFactor::shared_ptr new_factor(
      new SmartStereoFactor(smart_noise_, smartFactorsParams_, B_Pose_leftCam_));

  if (verbosity_ >= 9) {std::cout << "Adding landmark with: " << ft.obs_.size() << " landmarks to graph, with keys: ";}
  if (verbosity_ >= 9){new_factor->print();}

  // add observations to smart factor
  for (const std::pair<FrameId,StereoPoint2>& obs : ft.obs_)
  {
    new_factor->add(obs.second, gtsam::Symbol('x', obs.first), stereoCal_);
    if (verbosity_ >= 9) {std::cout << " " <<  obs.first;}
  }
  if (verbosity_ >= 9) {std::cout << std::endl;}
  // add new factor to suitable structures:
  new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
  old_smart_factors_.insert(std::make_pair(lm_id, std::make_pair(new_factor, -1)));
  // This is the ONLY difference wrt VioBackEnd: add to the map of lmkID (at the
  // beginning they are all smart factors) whether it is smart of not.
  lmk_id_is_smart_.insert(std::make_pair(lm_id, true));
}

void RegularVioBackEnd::updateLandmarkInGraph(const LandmarkId lm_id, const std::pair<FrameId, StereoPoint2>& newObs)
{
  bool is_lmk_smart = lmk_id_is_smart_.at(lm_id);
  if (is_lmk_smart == true) {
    // Update existing smart-factor
    auto old_smart_factors_it = old_smart_factors_.find(lm_id);
    if (old_smart_factors_it == old_smart_factors_.end())
      throw std::runtime_error("updateLandmarkInGraph: landmark not found in old_smart_factors_\n");

    SmartStereoFactor::shared_ptr old_factor = old_smart_factors_it->second.first;
    SmartStereoFactor::shared_ptr new_factor = boost::make_shared<SmartStereoFactor>(*old_factor); // clone old factor
    new_factor->add(newObs.second, gtsam::Symbol('x', newObs.first), stereoCal_);

    // update the factor
    if (old_smart_factors_it->second.second != -1){// if slot is still -1, it means that the factor has not been inserted yet in the graph
      new_smart_factors_.insert(std::make_pair(lm_id, new_factor));
    }else{
      throw std::runtime_error("updateLandmarkInGraph: when calling update the slot should be already != -1! \n");
    }
    old_smart_factors_it->second.first = new_factor;
    if (verbosity_ >= 8) {std::cout << "updateLandmarkInGraph: added observation to point: " << lm_id << std::endl;}
  } else {
    // If it is not smart, just add current measurement.
    new_imu_and_prior_factors_.push_back(gtsam::GenericStereoFactor<Pose3, Point3>
                                         (newObs.second, smart_noise_,
                                          gtsam::Symbol('x', newObs.first),
                                          gtsam::Symbol('l', lm_id),
                                          stereoCal_, B_Pose_leftCam_));
  }
}

} // namespace VIO
