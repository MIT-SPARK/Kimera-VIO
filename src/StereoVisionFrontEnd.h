/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoVisionFrontEnd.h
 * @brief  Class describing a stereo tracker
 * @author Luca Carlone
 */

#ifndef StereoVisionFrontEnd_H_
#define StereoVisionFrontEnd_H_

#include "StereoFrame.h"
#include <boost/shared_ptr.hpp>        // used for opengv
#include <time.h>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "Tracker.h"

using namespace std;
using namespace cv;

namespace VIO {

///////////////////////////////////////////////////////////////////////////////////////
class TrackerStatusSummary{

public:
  gtsam::Pose3 lkf_T_k_mono_;
  gtsam::Pose3 lkf_T_k_stereo_;
  Tracker::TrackingStatus kfTrackingStatus_mono_;
  Tracker::TrackingStatus kfTrackingStatus_stereo_;
  gtsam::Matrix3 infoMatStereoTranslation_;

public:
  TrackerStatusSummary() :
    kfTrackingStatus_mono_(Tracker::TrackingStatus::INVALID),
    kfTrackingStatus_stereo_(Tracker::TrackingStatus::INVALID),
    lkf_T_k_mono_(gtsam::Pose3()),
    lkf_T_k_stereo_(gtsam::Pose3()),
    infoMatStereoTranslation_(gtsam::Matrix3::Zero()) {}
};

using StatusSmartStereoMeasurements = std::pair<TrackerStatusSummary,SmartStereoMeasurements>;

///////////////////////////////////////////////////////////////////////////////////////
class StereoVisionFrontEnd{
public:
  // Constructor.
  StereoVisionFrontEnd(
      const VioFrontEndParams& trackerParams = VioFrontEndParams(),
      int saveImages = 1,
      const std::string& dataset_name = "");

  // verbosity_ explanation (TODO: include this)
  /*
   * 0: no display
   * 1: show images
   * 2: write images (at each keyframe)
   * 3: write video
   * 4: write an image for each feature matched between left and right
   */

  // Stereo Frames
  std::shared_ptr<StereoFrame> stereoFrame_k_;   // Current frame
  std::shared_ptr<StereoFrame> stereoFrame_km1_; // Last frame
  std::shared_ptr<StereoFrame> stereoFrame_lkf_; // Last keyframe

  // Counters
  int frame_count_;		      // Frame counter
  int keyframe_count_;        // keyframe counter
  int last_landmark_count_;	      // Previous number of landmarks (used for what is new landmark)
  Timestamp last_keyframe_timestamp_; // Timestamp of last keyframe

  // set of functionalities for tracking
  Tracker tracker_;

  // debug flag
  const int saveImages_; // 0: don't show, 1: show, 2: write & save

  // summary of information from the tracker, e.g., relative pose estimates and status of mono and stereo ransac
  TrackerStatusSummary trackerStatusSummary_;

  // this is not const as for debugging we want to redirect the image save path where we like
  std::string outputImagesPath_;

public:
  /* ------------------------------------------------------------------------ */
  // Frontend initialization.
  void processFirstStereoFrame(StereoFrame& firstFrame);

  /* ------------------------------------------------------------------------ */
  // Frontend main function.
  StatusSmartStereoMeasurements processStereoFrame(
      StereoFrame cur_frame, // Pass by value and use move semantics!
      boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu = boost::none);

  /* ------------------------------------------------------------------------ */
  // Returns extracted left and right rectified features in a suitable format
  // for VIO.
  SmartStereoMeasurements getSmartStereoMeasurements(
      const StereoFrame& stereoFrame_kf) const;

  /* ------------------------------------------------------------------------ */
  // visualize quality of temporal and stereo matching
  void displayStereoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // visualize quality of temporal and stereo matching
  void displayMonoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const;

  /* ------------------------------------------------------------------------ */
  // Static function to display output of stereo tracker
  static void PrintStatusStereoMeasurements(
      const StatusSmartStereoMeasurements& statusStereoMeasurements) {
    LOG(INFO) << " SmartStereoMeasurements with status:";
    logTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_mono_,
                      "mono");
    logTrackingStatus(statusStereoMeasurements.first.kfTrackingStatus_stereo_,
                      "stereo");
    LOG(INFO) << " observing landmarks:";
    const SmartStereoMeasurements& smartStereoMeas = statusStereoMeasurements.second;
    for(const auto& smart_stereo_meas: smartStereoMeas) {
      std::cout << " " << smart_stereo_meas.first << " ";
    }
    std::cout << std::endl;
  }

  static std::string asString(const Tracker::TrackingStatus& status) {
    switch(status) {
    case Tracker::TrackingStatus::VALID : return "VALID";
    case Tracker::TrackingStatus::INVALID : return "INVALID";
    case Tracker::TrackingStatus::DISABLED : return "DISABLED";
    case Tracker::TrackingStatus::FEW_MATCHES : return "FEW_MATCHES";
    case Tracker::TrackingStatus::LOW_DISPARITY : return "LOW_DISPARITY";
    }
  }

private:
  inline static void logTrackingStatus(const Tracker::TrackingStatus& status,
                                       const std::string& type = "mono") {
    LOG(INFO) << "Status " << type << ": " << asString(status);
  }
};

} // namespace VIO
#endif /* StereoVisionFrontEnd_H_ */

