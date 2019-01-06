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
    kfTrackingStatus_mono_(Tracker::INVALID),
    kfTrackingStatus_stereo_(Tracker::INVALID),
    lkf_T_k_mono_(gtsam::Pose3()), lkf_T_k_stereo_(gtsam::Pose3()),
    infoMatStereoTranslation_(gtsam::Matrix3::Zero()) {}
};

using StatusSmartStereoMeasurements = std::pair<TrackerStatusSummary,SmartStereoMeasurements>;

///////////////////////////////////////////////////////////////////////////////////////
class StereoVisionFrontEnd{
public:
  // Constructor.
  StereoVisionFrontEnd(
      const VioFrontEndParams trackerParams = VioFrontEndParams(),
      const VioBackEndParams vioParams = VioBackEndParams(),
      int saveImages = 1,
      const std::string& dataset_name = "") :
    frame_count_(0), keyframe_count_(0), last_landmark_count_(0),
    tracker_(trackerParams,saveImages), saveImages_(saveImages),
    trackerStatusSummary_(TrackerStatusSummary()),
    outputImagesPath_("./outputImages/") // only for debugging and visualization
  {
    if (saveImages > 0) {
      outputImagesPath_ = "./outputStereoTrackerImages-" +
          dataset_name;
      tracker_.outputImagesPath_ = "./outputTrackerImages-" +
          dataset_name;
    }
    tracker_.trackerParams_.print();
  }

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
  virtual void processFirstStereoFrame(StereoFrame& firstFrame);
  StatusSmartStereoMeasurements processStereoFrame(StereoFrame& cur_Frame,
      boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu = boost::none);

  /* ------------------------------------------------------------------------ */
  // returns extracted left and right rectified features in a suitable format for VIO
  SmartStereoMeasurements getSmartStereoMeasurements(
      const StereoFrame& stereoFrame_kf) const;

  /* ------------------------------------------------------------------------ */
  // visualize quality of temporal and stereo matching
  void displayStereoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // visualize quality of temporal and stereo matching
  void displayMonoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // return relative pose between last (lkf) and current keyframe (k) - MONO RANSAC
  gtsam::Pose3 getRelativePoseBodyMono() const {
    // lkfBody_T_kBody = lkfBody_T_lkfCamera *  lkfCamera_T_kCamera_ * kCamera_T_kBody =
    // body_Pose_cam_ * lkf_T_k_mono_ * body_Pose_cam_^-1
    gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->B_Pose_camLrect; // of the left camera!!
    return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_mono_ * body_Pose_cam_.inverse();
  }

  /* ------------------------------------------------------------------------ */
  // return relative pose between last (lkf) and current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const {
    gtsam::Pose3 body_Pose_cam_ = stereoFrame_lkf_->B_Pose_camLrect; // of the left camera!!
    return body_Pose_cam_ * trackerStatusSummary_.lkf_T_k_stereo_ * body_Pose_cam_.inverse();
  }

  /* ------------------------------------------------------------------------ */
  // static function to display output of stereo tracker
  static void PrintStatusStereoMeasurements(
      const StatusSmartStereoMeasurements& statusStereoMeasurements) {
    std::cout << " SmartStereoMeasurements with mono status " << statusStereoMeasurements.first.kfTrackingStatus_mono_
        << " , stereo status " << statusStereoMeasurements.first.kfTrackingStatus_stereo_
        << " observing landmarks:" << std::endl;
    const SmartStereoMeasurements& smartStereMeas = statusStereoMeasurements.second;
    for(size_t k=0; k<smartStereMeas.size();k++){
      std::cout << " " << smartStereMeas[k].first << " ";
    }
    std::cout << std::endl;
  }
};

} // namespace VIO
#endif /* StereoVisionFrontEnd_H_ */

