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
 * @author Antoni Rosinol, Luca Carlone
 */

#ifndef StereoVisionFrontEnd_H_
#define StereoVisionFrontEnd_H_

#include <boost/shared_ptr.hpp> // used for opengv

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "StereoFrame.h"
#include "StereoImuSyncPacket.h"
#include "Tracker.h"
#include "utils/Timer.h"
#include "utils/Statistics.h"
#include "utils/ThreadsafeQueue.h"
#include "ImuFrontEnd-definitions.h"
#include "ImuFrontEnd.h"
#include "Tracker-definitions.h"
#include "StereoVisionFrontEnd-definitions.h"
#include "VioBackEnd-definitions.h"

namespace VIO {

class StereoVisionFrontEnd {
public:
  using StereoFrontEndInputPayload = StereoImuSyncPacket;
  //using StereoFrontEndOutputPayload = VioBackEndInputPayload;

public:
  StereoVisionFrontEnd(
      const ImuParams& imu_params,
      const ImuBias& imu_initial_bias,
      const VioFrontEndParams& trackerParams = VioFrontEndParams(),
      int saveImages = 1,
      const std::string& dataset_name = "",
      bool log_output = false);

  /* ------------------------------------------------------------------------ */
  bool spin(ThreadsafeQueue<StereoFrontEndInputPayload>& input_queue,
            ThreadsafeQueue<StereoFrontEndOutputPayload>& output_queue,
            bool parallel_run = true);

  /* ------------------------------------------------------------------------ */
  // Shutdown spin.
  inline void shutdown() {
    LOG_IF(WARNING, shutdown_) << "Shutdown requested, but Frontend was already "
                                  "shutdown.";
    LOG(INFO) << "Shutting down Frontend.";
    shutdown_ = true;
  }

  /* ------------------------------------------------------------------------ */
  // Query if thread is working and not waiting on input queue to be filled.
  inline bool isWorking() const {return is_thread_working_;}

public:
  /* ------------------------------------------------------------------------ */
  // Update Imu Bias. This is thread-safe as imu_frontend_->updateBias is
  // thread-safe.
  inline void updateImuBias(const ImuBias& imu_bias) const {
    imu_frontend_->updateBias(imu_bias);
  }

  /* ************************************************************************ */
  // NOT THREAD-SAFE METHODS
  /* ************************************************************************ */

  /* ------------------------------------------------------------------------ */
  // Frontend initialization.
  StereoFrame processFirstStereoFrame(const StereoFrame& firstFrame);

  /* ------------------------------------------------------------------------ */
  // Returns extracted left and right rectified features in a suitable format
  // for VIO.
  SmartStereoMeasurements getSmartStereoMeasurements(
      const StereoFrame& stereoFrame_kf) const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - MONO RANSAC.
  gtsam::Pose3 getRelativePoseBodyMono() const;

  /* ------------------------------------------------------------------------ */
  // Return relative pose between last (lkf) and
  // current keyframe (k) - STEREO RANSAC
  gtsam::Pose3 getRelativePoseBodyStereo() const;

private:
  /* ------------------------------------------------------------------------ */
  StereoFrontEndOutputPayload spinOnce(
      const std::shared_ptr<StereoFrontEndInputPayload>& input);

  /* ------------------------------------------------------------------------ */
  // Frontend main function.
  StatusSmartStereoMeasurements processStereoFrame(
      StereoFrame cur_frame, // Pass by value and use move semantics!
      boost::optional<gtsam::Rot3> calLrectLkf_R_camLrectKf_imu = boost::none);

  /* ------------------------------------------------------------------------ */
  inline static void logTrackingStatus(const TrackingStatus& status,
                                       const std::string& type = "mono") {
    LOG(INFO) << "Status " << type << ": "
              << TrackerStatusSummary::asString(status);
  }

  /* ------------------------------------------------------------------------ */
  // Static function to display output of stereo tracker
  static void printStatusStereoMeasurements(
      const StatusSmartStereoMeasurements& statusStereoMeasurements);

  // verbosity_ explanation (TODO: include this)
  /*
   * 0: no display
   * 1: show images
   * 2: write images (at each keyframe)
   * 3: write video
   * 4: write an image for each feature matched between left and right
   */
  /* ------------------------------------------------------------------------ */
  // Visualize quality of temporal and stereo matching
  void displayStereoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  // Visualize quality of temporal and stereo matching
  void displayMonoTrack(const int& verbosity) const;

  /* ------------------------------------------------------------------------ */
  void displaySaveImage(
      const cv::Mat& img_left,
      const std::string& text_on_img = "",
      const std::string& imshow_name = "mono tracking visualization (1 frame)",
      const std::string& folder_name_append = "-monoMatching1frame",
      const std::string& img_name_prepend = "monoTrackerDisplay1Keyframe_",
      const int verbosity = 0) const;

private:
  // TODO MAKE THESE GUYS std::unique_ptr, we do not want to have multiple
  // owners, instead they should be passed around.
  // Stereo Frames

  // Current frame
  std::shared_ptr<StereoFrame> stereoFrame_k_;
  // Last frame
  std::shared_ptr<StereoFrame> stereoFrame_km1_;
  // Last keyframe
  std::shared_ptr<StereoFrame> stereoFrame_lkf_;

  // Counters.
  // Frame counte.
  int frame_count_;
  // keyframe counte.
  int keyframe_count_;
  // Previous number of landmarks (used for what is new landmark.
  int last_landmark_count_;
  // Timestamp of last keyframe.
  Timestamp last_keyframe_timestamp_;

  // Set of functionalities for tracking.
  Tracker tracker_;

  // IMU frontend.
  std::unique_ptr<ImuFrontEnd> imu_frontend_;

  // Debug flag.
  const int save_images_option_; // 0: don't show, 1: show, 2: write & save

  // Summary of information from the tracker, e.g., relative pose estimates and
  // status of mono and stereo ransac
  TrackerStatusSummary trackerStatusSummary_;

  // This is not const as for debugging we want to redirect the image save path
  // where we like
  std::string output_images_path_;

  // Thread related members.
  std::atomic_bool shutdown_ = {false};
  std::atomic_bool is_thread_working_ = {false};

  // Frontend logger.
  const bool log_output_ = {false};
};

} // namespace VIO
#endif /* StereoVisionFrontEnd_H_ */

