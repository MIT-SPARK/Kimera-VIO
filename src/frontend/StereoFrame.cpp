/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Frame.h
 * @brief  Class describing a pair of stereo images
 * @author Antoni Rosinol, Luca Carlone
 */

#include "kimera-vio/frontend/StereoFrame.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>

DEFINE_bool(images_rectified, false, "Input image data already rectified.");

namespace VIO {

/* -------------------------------------------------------------------------- */
StereoFrame::StereoFrame(const FrameId& id,
                         const Timestamp& timestamp,
                         const Frame& left_frame,
                         const Frame& right_frame,
                         const StereoMatchingParams& stereo_matching_params)
    : id_(id),
      timestamp_(timestamp),
      // TODO(Toni): these copies are the culprits of all evil...
      left_frame_(left_frame),
      right_frame_(right_frame),
      is_rectified_(FLAGS_images_rectified),
      is_keyframe_(false),
      // TODO(Toni): completely useless to copy params all the time...
      sparse_stereo_params_(stereo_matching_params),
      baseline_(0.0) {
  initialize(left_frame_.cam_param_, right_frame_.cam_param_);
  CHECK_EQ(id_, left_frame_.id_);
  CHECK_EQ(id_, right_frame_.id_);
  CHECK_EQ(timestamp_, left_frame_.timestamp_);
  CHECK_EQ(timestamp_, right_frame_.timestamp_);
}

/* -------------------------------------------------------------------------- */
StereoFrame::StereoFrame(const FrameId& id,
                         const Timestamp& timestamp,
                         const cv::Mat& left_image,
                         const CameraParams& cam_param_left,
                         const cv::Mat& right_image,
                         const CameraParams& cam_param_right,
                         const StereoMatchingParams& stereo_matching_params)
    : id_(id),
      timestamp_(timestamp),
      left_frame_(id, timestamp, cam_param_left, left_image),
      right_frame_(id, timestamp, cam_param_right, right_image),
      is_rectified_(FLAGS_images_rectified),
      is_keyframe_(false),
      sparse_stereo_params_(stereo_matching_params),
      baseline_(0.0) {
  initialize(cam_param_left, cam_param_right);
  CHECK_EQ(id_, left_frame_.id_);
  CHECK_EQ(id_, right_frame_.id_);
  CHECK_EQ(timestamp_, left_frame_.timestamp_);
  CHECK_EQ(timestamp_, right_frame_.timestamp_);
}

void StereoFrame::initialize(const CameraParams& cam_param_left,
                             const CameraParams& cam_param_right) {
  // If input is rectified already
  if (is_rectified_) {
    left_img_rectified_ = left_frame_.img_;
    right_img_rectified_ = right_frame_.img_;
    left_undistRectCameraMatrix_ =
        UtilsOpenCV::Cvmat2Cal3_S2(left_frame_.cam_param_.P_);
    right_undistRectCameraMatrix_ =
        UtilsOpenCV::Cvmat2Cal3_S2(right_frame_.cam_param_.P_);
    // TODO(Toni): remove assumption on stereo cameras being x-aligned!
    baseline_ =
        cam_param_left.body_Pose_cam_.between(cam_param_right.body_Pose_cam_)
            .x();
  } else {
    // TODO(Toni): the undistRect maps should be computed once and cached!!
    computeRectificationParameters(
        &left_frame_.cam_param_, &right_frame_.cam_param_, &B_Pose_camLrect_);
    // TODO REMOVE ASSUMPTION ON x aligned stereo camera, can't we just take the
    // norm?
    baseline_ = left_frame_.cam_param_.body_Pose_cam_
                    .between(right_frame_.cam_param_.body_Pose_cam_)
                    .x();
    left_undistRectCameraMatrix_ =
        UtilsOpenCV::Cvmat2Cal3_S2(left_frame_.cam_param_.P_);
    right_undistRectCameraMatrix_ =
        UtilsOpenCV::Cvmat2Cal3_S2(right_frame_.cam_param_.P_);
    //! Sanity check that rectified baseline remains within 10% of the
    //! expected baseline.
    const double& nominal_baseline = sparse_stereo_params_.nominal_baseline_;
    static constexpr double kBaselineTolerance = 0.10;
    if (baseline_ > (1.0 + kBaselineTolerance) * nominal_baseline ||
        baseline_ < (1.0 - kBaselineTolerance) * nominal_baseline) {
      LOG(FATAL) << "Baseline after rectification differs from nominal: \n"
                 << "- Abnormal baseline: " << baseline_ << '\n'
                 << "- Nominal baseline: " << nominal_baseline << '\n'
                 << "(not within +/-10% bounds)";
    }
    //! Rectify and undistort images
    cv::remap(left_frame_.img_,
              left_img_rectified_,
              left_frame_.cam_param_.undistRect_map_x_,
              left_frame_.cam_param_.undistRect_map_y_,
              cv::INTER_LINEAR);
    cv::remap(right_frame_.img_,
              right_img_rectified_,
              right_frame_.cam_param_.undistRect_map_x_,
              right_frame_.cam_param_.undistRect_map_y_,
              cv::INTER_LINEAR);
    is_rectified_ = true;
  }
  VLOG(10) << "- size before (left): " << left_frame_.img_.rows << " x "
           << left_frame_.img_.cols << '\n'
           << "- size after  (left): " << left_img_rectified_.rows << " x "
           << left_img_rectified_.cols
           << "\n- size before (right): " << right_frame_.img_.rows << " x "
           << right_frame_.img_.cols << '\n'
           << "- size after  (right): " << right_img_rectified_.rows << " x "
           << right_img_rectified_.cols;
}

/* -------------------------------------------------------------------------- */
// TODO: Clean up RGBD
// TODO: this should be in StereoMatcher
void StereoFrame::sparseStereoMatching(const int verbosity) {
  if (verbosity > 0) {
    cv::Mat leftImgWithKeypoints =
        UtilsOpenCV::DrawCircles(left_frame_.img_, left_frame_.keypoints_);
    showImagesSideBySide(leftImgWithKeypoints, right_frame_.img_,
                         "unrectifiedLeftWithKeypoints_", verbosity);
  }

  CHECK(is_rectified_);

  // Get rectified left keypoints.
  StatusKeypointsCV left_keypoints_rectified;
  undistortRectifyPoints(left_frame_.keypoints_,
                         left_frame_.cam_param_,
                         left_undistRectCameraMatrix_,
                         &left_keypoints_rectified);
  // TODO (actually this is compensated later on in the pipeline): This should
  // be correct but largely hinders the performance of RANSAC compensate versors
  // for rectification
  //  gtsam::Rot3 camLrect_R_camL =
  //  UtilsOpenCV::Cvmat2rot(left_frame_.cam_param_.R_rectify_); std::cout <<
  //  "left_frame_.cam_param_.R_rectify_ << " <<
  //  camLrect_R_camL.matrix().determinant() << std::endl; for(size_t i = 0; i <
  //  left_frame_.versors_.size(); i++){ // for each versor
  //    left_frame_.versors_.at(i) =
  //    camLrect_R_camL.rotate(gtsam::Unit3(left_frame_.versors_.at(i))).unitVector();
  //    // rotate versor
  //    // R_rectify = camL_R_camLrect' = camL_R_camLrect => camLrect_versor =
  //    camLrect_R_camL * camL_versor double normV =
  //    left_frame_.versors_.at(i).norm(); left_frame_.versors_.at(i) =
  //    left_frame_.versors_.at(i) / normV;
  //    // renormalize to reduce error propagation
  //  }

  // **** now we get corresponding keypoints in right image
  // ***************************
  //////////////////////////////////////////////////////////////////////////
  // another way is to find for sparse correspondences in right image using
  // optical flow COMMENT: this does not work, probably the baseline is too
  // large getRightKeypointsLKunrectified();
  //////////////////////////////////////////////////////////////////////////
  // another way is to extract corners in right and try to match with left
  // COMMENT: unfortunately, from visual inspection it seems that we do not get
  // the same matches KeypointsCV right_keypoints =
  // UtilsOpenCV::ExtractCorners(right_frame_.img_); cv::Mat
  // rightImgWithKeypoints = UtilsOpenCV::DrawCircles(right_frame_.img_,
  // right_keypoints);
  // showImagesSideBySide(leftImgWithKeypoints,rightImgWithKeypoints,"unrectifiedWithKeypoints",
  // verbosity);

  //////////////////////////////////////////////////////////////////////////
  // the way we use is to find for sparse correspondences in right image using
  // patch correlation along (horizontal epipolar lines)
  double fx = left_undistRectCameraMatrix_.fx();
  // std::cout << "nr of template matching calls: " <<
  // left_frame_.getNrValidKeypoints() << std::endl;

  // Options for stereo and RGB-D
  StatusKeypointsCV right_keypoints_rectified;
  switch (sparse_stereo_params_.vision_sensor_type_) {
    case VisionSensorType::STEREO:
      right_keypoints_rectified = getRightKeypointsRectified(
          left_img_rectified_, right_img_rectified_, left_keypoints_rectified,
          fx, getBaseline());
      break;
    case VisionSensorType::RGBD:  // just use depth to "fake right pixel
                                  // matches"
      right_keypoints_rectified = getRightKeypointsRectifiedRGBD(
          left_img_rectified_, right_img_rectified_, left_keypoints_rectified,
          fx, getBaseline(), getMapDepthFactor(), getMinDepthFactor());
      break;
    default:
      LOG(FATAL) << "sparseStereoMatching: only works when "
                    "VisionSensorType::STEREO or RGBD";
      break;
  }

  // Compute the depth for each keypoints.
  keypoints_depth_ = getDepthFromRectifiedMatches(
      left_keypoints_rectified, right_keypoints_rectified, fx, getBaseline());
  // Display.
  if (verbosity > 0) {
    cv::Mat left_rectifiedWithKeypoints =
        UtilsOpenCV::DrawCircles(left_img_rectified_, left_keypoints_rectified);
    drawEpipolarLines(left_rectifiedWithKeypoints, right_img_rectified_, 20,
                      verbosity);
    cv::Mat right_rectifiedWithKeypoints = UtilsOpenCV::DrawCircles(
        right_img_rectified_, right_keypoints_rectified, keypoints_depth_);
    showImagesSideBySide(left_rectifiedWithKeypoints,
                         right_rectifiedWithKeypoints,
                         "rectifiedWithKeypointsAndDepth_", verbosity);
  }

  // Store point pixels and statuses: for visualization
  // and to populate the statuses.
  // TODO remove tie, potential copies being made.
  std::tie(right_frame_.keypoints_, right_keypoints_status_) =
      distortUnrectifyPoints(right_keypoints_rectified,
                             right_frame_.cam_param_.undistRect_map_x_,
                             right_frame_.cam_param_.undistRect_map_y_);

  // Sanity check.
  CHECK_EQ(keypoints_depth_.size(), left_frame_.versors_.size())
      << "sparseStereoMatching: keypoints_depth_ & versors_ sizes are wrong!";

  // Get 3D points and populate structures.
  keypoints_3d_.clear();
  keypoints_3d_.reserve(right_keypoints_rectified.size());
  left_keypoints_rectified_.clear();
  left_keypoints_rectified_.reserve(right_keypoints_rectified.size());
  right_keypoints_rectified_.clear();
  right_keypoints_rectified_.reserve(right_keypoints_rectified.size());

  // IMPORTANT: keypoints_3d_ are expressed in the rectified left frame, so we
  // have to compensate for rectification. We do not do this for the versors to
  // avoid adding numerical errors (we are using very tight thresholds on
  // 5-point RANSAC)
  gtsam::Rot3 camLrect_R_camL =
      UtilsOpenCV::cvMatToGtsamRot3(left_frame_.cam_param_.R_rectify_);
  for (size_t i = 0; i < right_keypoints_rectified.size(); i++) {
    left_keypoints_rectified_.push_back(left_keypoints_rectified.at(i).second);
    right_keypoints_rectified_.push_back(
        right_keypoints_rectified.at(i).second);
    if (right_keypoints_rectified[i].first == KeypointStatus::VALID) {
      Vector3 versor = camLrect_R_camL.rotate(left_frame_.versors_[i]);
      CHECK_GE(versor(2), 1e-3)
          << "sparseStereoMatching: found point with nonpositive depth!";
      // keypoints_depth_ is not the norm of the vector, it is the z component.
      keypoints_3d_.push_back(versor * keypoints_depth_[i] / versor(2));
    } else {
      keypoints_3d_.push_back(Vector3::Zero());
    }
  }

  // Visualize statistics on the performance of the sparse stereo matching.
  if (VLOG_IS_ON(20)) {
    displayKeypointStats(right_keypoints_rectified);
  }

  // Sanity check.
  VLOG(10) << "Sanity check stereo frame...";
  checkStereoFrame();
  VLOG(10) << "Finished sanity check stereo frame.";
}

/* -------------------------------------------------------------------------- */
void StereoFrame::checkStereoFrame() const {
  const size_t nrLeftKeypoints = left_frame_.keypoints_.size();
  CHECK_EQ(left_frame_.scores_.size(), nrLeftKeypoints)
      << "checkStereoFrame: left_frame_.scores.size()";
  CHECK_EQ(right_frame_.keypoints_.size(), nrLeftKeypoints)
      << "checkStereoFrame: right_frame_.keypoints_.size()";
  CHECK_EQ(right_keypoints_status_.size(), nrLeftKeypoints)
      << "checkStereoFrame: right_keypoints_status_.size()";
  CHECK_EQ(keypoints_depth_.size(), nrLeftKeypoints)
      << "checkStereoFrame: keypoints_depth_.size()";
  CHECK_EQ(keypoints_3d_.size(), nrLeftKeypoints)
      << "checkStereoFrame: keypoints_3d_.size()";
  CHECK_EQ(left_keypoints_rectified_.size(), nrLeftKeypoints)
      << "checkStereoFrame: left_keypoints_rectified_.size()";
  CHECK_EQ(right_keypoints_rectified_.size(), nrLeftKeypoints)
      << "checkStereoFrame: right_keypoints_rectified_.size()";

  double tol = 1e-4;
  for (size_t i = 0; i < nrLeftKeypoints; i++) {
    if (right_keypoints_status_[i] == KeypointStatus::VALID) {
      CHECK_LE(fabs(right_keypoints_rectified_[i].y -
                    left_keypoints_rectified_[i].y),
               3)
          << "checkStereoFrame: rectified keypoints have different y "
          << right_keypoints_rectified_[i].y << " vs. "
          << left_keypoints_rectified_[i].y;
    }

    CHECK_LE(fabs(keypoints_3d_[i](2) - keypoints_depth_[i]), tol)
        << "keypoints_3d_[i] has wrong depth " << keypoints_3d_[i](2) << " vs. "
        << keypoints_depth_[i];

    if (right_keypoints_status_[i] == KeypointStatus::VALID) {
      CHECK_NE(fabs(right_frame_.keypoints_[i].x) +
                   fabs(right_frame_.keypoints_[i].y),
               0)
          << "checkStereoFrame: right_frame_.keypoints_[i] is zero.";
      // Also: cannot have zero depth.
      CHECK_GT(keypoints_depth_[i], 0)
          << "checkStereoFrame: keypoints_3d_[i] has nonpositive "
             "for valid point: "
          << keypoints_depth_[i] << '\n'
          << "right_keypoints_status_[i] " << right_keypoints_status_[i] << '\n'
          << "left_frame_.keypoints_[i] " << left_frame_.keypoints_[i] << '\n'
          << "right_frame_.keypoints_[i] " << right_frame_.keypoints_[i] << '\n'
          << "left_keypoints_rectified_[i] " << left_keypoints_rectified_[i]
          << '\n'
          << "right_keypoints_rectified_[i] " << right_keypoints_rectified_[i]
          << '\n'
          << "keypoints_depth_[i] " << keypoints_depth_[i];
    } else {
      CHECK_LE(keypoints_depth_[i], 0)
          << "checkStereoFrame: keypoints_3d_[i] has positive "
             "for nonvalid point: "
          << keypoints_depth_[i];
    }
  }
}

/* -------------------------------------------------------------------------- */
// TODO: this should be in Camera
std::pair<KeypointsCV, std::vector<KeypointStatus>>
StereoFrame::distortUnrectifyPoints(
    const StatusKeypointsCV& keypoints_rectified,
    const cv::Mat map_x,
    const cv::Mat map_y) {
  std::vector<KeypointStatus> pointStatuses;
  KeypointsCV points;
  for (size_t i = 0; i < keypoints_rectified.size(); i++) {
    pointStatuses.push_back(keypoints_rectified[i].first);
    if (keypoints_rectified[i].first == KeypointStatus::VALID) {
      KeypointCV px = keypoints_rectified[i].second;
      auto x = map_x.at<float>(round(px.y), round(px.x));
      auto y = map_y.at<float>(round(px.y), round(px.x));
      points.push_back(KeypointCV(x, y));
    } else {
      points.push_back(KeypointCV(0.0, 0.0));
    }
  }
  return std::make_pair(points, pointStatuses);
}

/* -------------------------------------------------------------------------- */
// TODO: this should be in Camera
void StereoFrame::undistortRectifyPoints(
    const KeypointsCV& left_keypoints_unrectified,
    const CameraParams& cam_param,
    const gtsam::Cal3_S2& rectCameraMatrix,
    StatusKeypointsCV* left_keypoints_rectified) const {
  CHECK_NOTNULL(left_keypoints_rectified)
      ->resize(left_keypoints_unrectified.size());

  int invalid_count = 0;
  size_t idx = 0;
  for (const KeypointCV& px : left_keypoints_unrectified) {
    // The following undistort to a versor,
    // then we can project by the new camera matrix.
    Vector3 calibrated_versor = Frame::calibratePixel(px, cam_param);

    // Compensate for rectification.
    gtsam::Rot3 R_rect = UtilsOpenCV::cvMatToGtsamRot3(cam_param.R_rectify_);
    calibrated_versor = R_rect.matrix() * calibrated_versor;

    // Normalize to unit z.
    LOG_IF(FATAL, std::fabs(calibrated_versor(2)) < 1e-4)
        << "undistortRectifyPoints: versor with zero depth";
    calibrated_versor = calibrated_versor / calibrated_versor(2);

    const double& fx = rectCameraMatrix.fx();
    const double& cx = rectCameraMatrix.px();
    const double& fy = rectCameraMatrix.fy();
    const double& cy = rectCameraMatrix.py();

    // rectified_versor = rectCameraMatrix * calibrated_versor; -> this is done
    // manually since the matrix and the versor are incompatible types (opencv
    // vs. gtsam)
    KeypointCV px_undistRect(fx * calibrated_versor(0) + cx,
                             fy * calibrated_versor(1) + cy);
    bool cropped = UtilsOpenCV::cropToSize(&px_undistRect,
                                           cam_param.undistRect_map_x_.size());

    float x_check = 0.0f;
    float y_check = 0.0f;
    // TODO(Toni): should be in camera_params
    if (!FLAGS_images_rectified) {
      // sanity check: you can go back to the original image accurately (if
      // original not rectified)
      x_check = cam_param.undistRect_map_x_.at<float>(round(px_undistRect.y),
                                                      round(px_undistRect.x));
      y_check = cam_param.undistRect_map_y_.at<float>(round(px_undistRect.y),
                                                      round(px_undistRect.x));
    } else {
      x_check = px.x;
      y_check = px.y;
    }

    // Tolerance in pixels
    // This tolerance is huge no??
    static constexpr float kTolInPixels = 2.0;
    if (cropped || std::fabs(px.x - x_check) > kTolInPixels ||
        std::fabs(px.y - y_check) > kTolInPixels) {
      // Mark as invalid all pixels that were undistorted out of the frame
      // and for which the undistroted rectified keypoint remaps close to
      // the distorted unrectified pixel.
      VLOG(10) << "undistortRectifyPoints: pixel mismatch\n"
               << "px.x " << px.x << " x_check " << x_check << '\n'
               << "px.y " << px.y << " y_check " << y_check << '\n'
               << "px_undistRect " << px_undistRect << '\n'
               << "rounded " << round(px_undistRect.y) << '\n'
               << " " << round(px_undistRect.x) << '\n'
               << "cam_param.undistRect_map_x_ "
               << cam_param.undistRect_map_x_.size() << '\n'
               << "map type " << cam_param.undistRect_map_x_.type() << '\n'
               << "fx " << fx << " cx " << cx << " fy " << fy << " cy " << cy
               << '\n'
               << "calibrated_versor\n"
               << calibrated_versor;
      invalid_count += 1;
      // Invalid points.
      left_keypoints_rectified->at(idx) =
          std::make_pair(KeypointStatus::NO_LEFT_RECT, px_undistRect);
    } else {
      // Point is valid!
      left_keypoints_rectified->at(idx) =
          std::make_pair(KeypointStatus::VALID, px_undistRect);
    }
    idx++;
  }
  VLOG_IF(10, invalid_count > 0) << "undistortRectifyPoints: unable to match "
                                 << invalid_count << " keypoints";
}

/* -------------------------------------------------------------------------- */
// TODO: this should be in StereoMatcher
cv::Mat StereoFrame::getDisparityImage() const {
  const cv::Mat imgLeft = left_img_rectified_.clone();
  const cv::Mat imgRight = right_img_rectified_.clone();
  // TODO (Toni): remove hardcoded.
  bool isDebug = true;

  cv::Mat imgDisparity(imgLeft.rows, imgLeft.cols, CV_16S);

  // Call the constructor for StereoBM.
  int ndisparities = 16 * 5; /**< Range of disparity */
  int SADWindowSize = 21;    /**< Size of the block window. Must be odd */
  cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create(ndisparities, SADWindowSize);

  // Calculate the disparity image.
  sbm->compute(imgLeft, imgRight, imgDisparity);

  if (isDebug) {
    cv::Mat imgDisparity8U(imgLeft.rows, imgLeft.cols, CV_8UC1);

    // Check its extreme values.
    double minVal;
    double maxVal;
    minMaxLoc(imgDisparity, &minVal, &maxVal);
    LOG(INFO) << "getDisparityImage: minVal = " << minVal << " maxVal "
              << maxVal;

    // Display it as a CV_8UC1 image.
    imgDisparity.convertTo(imgDisparity8U, CV_8UC1, 255 / (maxVal - minVal));
    cv::namedWindow("disparity", cv::WINDOW_NORMAL);
    cv::imshow("disparity", imgDisparity8U);
    cv::waitKey(1);
  }

  return imgDisparity;
}

/* -------------------------------------------------------------------------- */
void StereoFrame::setIsKeyframe(bool is_kf) {
  is_keyframe_ = is_kf;
  left_frame_.isKeyframe_ = is_kf;
  right_frame_.isKeyframe_ = is_kf;
}

/* -------------------------------------------------------------------------- */
void StereoFrame::setIsRectified(bool is_rectified) {
  is_rectified_ = is_rectified;
}


/* -------------------------------------------------------------------------- */
// TODO: this should be in Mesher.
// Removes triangles in the 2d mesh that have more than "max_keypoints_with_
// gradient" keypoints with higher gradient than "gradient_bound".
// Input the original triangulation: original_triangulation_2D
// Output the filtered triangulation wo high-gradient triangles:
// filtered_triangulation_2D.
// If gradient_bound < 0, the check is disabled.
void StereoFrame::filterTrianglesWithGradients(
    const std::vector<cv::Vec6f>& original_triangulation_2D,
    std::vector<cv::Vec6f>* filtered_triangulation_2D,
    const float& gradient_bound,
    const size_t& max_keypoints_with_gradient) const {
  CHECK_NOTNULL(filtered_triangulation_2D);
  CHECK_NE(filtered_triangulation_2D, &original_triangulation_2D)
      << "Input original_triangulation_2D should be different that the object "
      << "pointed by filtered_triangulation_2D. Input=*Output error.";

  if (gradient_bound == -1) {
    // Skip filter.
    *filtered_triangulation_2D = original_triangulation_2D;
    LOG_FIRST_N(WARNING, 1) << "Filter triangles with gradients is disabled.";
    return;
  }

  // Compute img gradients.
  cv::Mat img_grads;
  computeImgGradients(left_frame_.img_, &img_grads);

  // For each triangle, set to full the triangles that have near-zero gradient.
  // triangulation2Dobs_.reserve(triangulation2D.size());
  // TODO far too many loops over triangles.
  for (const cv::Vec6f& triangle : original_triangulation_2D) {
    // Find all pixels with a gradient higher than gradBound.
    std::vector<std::pair<KeypointCV, double>> keypoints_with_high_gradient =
        UtilsOpenCV::FindHighIntensityInTriangle(img_grads, triangle,
                                                 gradient_bound);

    // If no high-grad pixels exist,
    // then this triangle is assumed to be a plane.
    if (keypoints_with_high_gradient.size() <= max_keypoints_with_gradient) {
      filtered_triangulation_2D->push_back(triangle);
    }
  }
}

void StereoFrame::computeImgGradients(const cv::Mat& img,
                                      cv::Mat* img_grads) const {
  CHECK_NOTNULL(img_grads);

  // Compute image gradients to check intensity gradient in each triangle.
  // UtilsOpenCV::ImageLaplacian(ref_frame.img_);
  *img_grads = UtilsOpenCV::EdgeDetectorCanny(img);

  static constexpr bool visualize_gradients = false;
  if (visualize_gradients) {
    cv::imshow("left_img_grads", *img_grads);
    cv::waitKey(1);
  }
}

/* -------------------------------------------------------------------------- */
void StereoFrame::cloneRectificationParameters(const StereoFrame& sf) {
  left_frame_.cam_param_.R_rectify_ = sf.left_frame_.cam_param_.R_rectify_;
  right_frame_.cam_param_.R_rectify_ = sf.right_frame_.cam_param_.R_rectify_;
  B_Pose_camLrect_ = sf.B_Pose_camLrect_;
  baseline_ = sf.baseline_;
  left_frame_.cam_param_.undistRect_map_x_ =
      sf.left_frame_.cam_param_.undistRect_map_x_.clone();
  left_frame_.cam_param_.undistRect_map_y_ =
      sf.left_frame_.cam_param_.undistRect_map_y_.clone();
  right_frame_.cam_param_.undistRect_map_x_ =
      sf.right_frame_.cam_param_.undistRect_map_x_.clone();
  right_frame_.cam_param_.undistRect_map_y_ =
      sf.right_frame_.cam_param_.undistRect_map_y_.clone();
  left_frame_.cam_param_.P_ = sf.left_frame_.cam_param_.P_.clone();
  right_frame_.cam_param_.P_ = sf.right_frame_.cam_param_.P_.clone();
  left_undistRectCameraMatrix_ = sf.left_undistRectCameraMatrix_;
  right_undistRectCameraMatrix_ = sf.right_undistRectCameraMatrix_;
  is_rectified_ = true;
  VLOG(10) << "cloned undistRect maps and other rectification parameters!";
}

/* -------------------------------------------------------------------------- */
// note also computes the rectification maps
// TODO(Toni): this should be done much earlier and only once...
void StereoFrame::computeRectificationParameters(
    CameraParams* left_cam_params,
    CameraParams* right_cam_params,
    gtsam::Pose3* B_Pose_camLrect) {
  CHECK_NOTNULL(left_cam_params);
  CHECK_NOTNULL(right_cam_params);
  CHECK_NOTNULL(B_Pose_camLrect);

  // Get extrinsics in open CV format.
  cv::Mat L_Rot_R, L_Tran_R;

  //! Extrinsics of the stereo (not rectified) relative pose between cameras
  gtsam::Pose3 camL_Pose_camR = (left_cam_params->body_Pose_cam_)
                                    .between(right_cam_params->body_Pose_cam_);
  // NOTE: openCV pose convention is the opposite, that's why we have to invert
  boost::tie(L_Rot_R, L_Tran_R) =
      UtilsOpenCV::Pose2cvmats(camL_Pose_camR.inverse());

  //////////////////////////////////////////////////////////////////////////////
  // get rectification matrices
  CameraParams& left_camera_info = *left_cam_params;
  CameraParams& right_camera_info = *right_cam_params;

  // P1 and P2 are the new camera matrices, but with an extra 0 0 0 column
  cv::Mat P1, P2, Q;

  if (left_camera_info.distortion_model_ == "radtan" ||
      left_camera_info.distortion_model_ == "radial-tangential") {
    // Get stereo rectification
    VLOG(10) << "Stereo camera distortion for rectification: radtan";
    cv::stereoRectify(
        // Input
        left_camera_info.camera_matrix_,
        left_camera_info.distortion_coeff_,
        right_camera_info.camera_matrix_,
        right_camera_info.distortion_coeff_,
        left_camera_info.image_size_,
        L_Rot_R,
        L_Tran_R,
        // Output
        left_camera_info.R_rectify_,
        right_camera_info.R_rectify_,
        P1,
        P2,
        Q);
  } else if (left_camera_info.distortion_model_ == "equidistant") {
    // Get stereo rectification
    VLOG(10) << "Stereo camera distortion for rectification: equidistant";
    cv::fisheye::stereoRectify(left_camera_info.camera_matrix_,
                               left_camera_info.distortion_coeff_,
                               right_camera_info.camera_matrix_,
                               right_camera_info.distortion_coeff_,
                               left_camera_info.image_size_,
                               L_Rot_R,
                               L_Tran_R,
                               // following are output
                               left_camera_info.R_rectify_,
                               right_camera_info.R_rectify_,
                               P1,
                               P2,
                               Q,
                               // TODO: Flag to maximise area???
                               cv::CALIB_ZERO_DISPARITY);
  } else {
    LOG(ERROR)
        << "Stereo camera distortion model not found for stereo rectification!";
  }

  VLOG(10) << "RESULTS OF RECTIFICATION: \n"
           << "left_camera_info.R_rectify_\n"
           << left_camera_info.R_rectify_ << '\n'
           << "right_camera_info.R_rectify_\n"
           << right_camera_info.R_rectify_;

  // Left camera pose after rectification
  const gtsam::Rot3& camL_Rot_camLrect =
      UtilsOpenCV::cvMatToGtsamRot3(left_camera_info.R_rectify_).inverse();
  //! Fix camL position to camLrect.
  //! aka use gtsam::Point3()
  gtsam::Pose3 camL_Pose_camLrect(camL_Rot_camLrect, gtsam::Point3::Zero());
  *B_Pose_camLrect =
      left_camera_info.body_Pose_cam_.compose(camL_Pose_camLrect);

  // right camera pose after rectification
  const gtsam::Rot3& camR_Rot_camRrect =
      UtilsOpenCV::cvMatToGtsamRot3(right_camera_info.R_rectify_).inverse();
  gtsam::Pose3 camR_Pose_camRrect(camR_Rot_camRrect, gtsam::Point3());
  gtsam::Pose3 B_Pose_camRrect =
      (right_camera_info.body_Pose_cam_).compose(camR_Pose_camRrect);

  // Relative pose after rectification
  gtsam::Pose3 camLrect_Pose_calRrect =
      B_Pose_camLrect->between(B_Pose_camRrect);

  // Sanity check.
  LOG_IF(FATAL,
         gtsam::Rot3::Logmap(camLrect_Pose_calRrect.rotation()).norm() > 1e-5)
      << "camL_Pose_camR log: "
      << gtsam::Rot3::Logmap(camL_Pose_camR.rotation()).norm() << '\n'
      << "camLrect_Pose_calRrect log: "
      << gtsam::Rot3::Logmap(camLrect_Pose_calRrect.rotation()).norm() << '\n'
      << "Vio constructor: camera poses do not seem to be rectified (rot)";
  LOG_IF(FATAL, fabs(camLrect_Pose_calRrect.translation().y()) > 1e-3 ||
                fabs(camLrect_Pose_calRrect.translation().z()) > 1e-3)
      << "Vio constructor: camera poses do not seem to be rectified (tran) \n"
      << "camLrect_Poe_calRrect: " << camLrect_Pose_calRrect;

  //////////////////////////////////////////////////////////////////////////////
  // TODO: Unit tests for this sections!!!!!

  // Left camera
  if (left_camera_info.distortion_model_ == "radtan" ||
      left_camera_info.distortion_model_ == "radial-tangential") {
    // Get rectification & undistortion maps. (radtan dist. model)
    VLOG(10) << "Left camera distortion: radtan";
    cv::initUndistortRectifyMap(left_camera_info.camera_matrix_,
                                left_camera_info.distortion_coeff_,
                                left_camera_info.R_rectify_,
                                P1,
                                left_camera_info.image_size_,
                                CV_32FC1,
                                // output:
                                left_camera_info.undistRect_map_x_,
                                left_camera_info.undistRect_map_y_);
  } else if (left_camera_info.distortion_model_ == "equidistant") {
    // Get rectification & undistortion maps. (equi dist. model)
    VLOG(10) << "Left camera distortion: equidistant";
    cv::fisheye::initUndistortRectifyMap(left_camera_info.camera_matrix_,
                                         left_camera_info.distortion_coeff_,
                                         left_camera_info.R_rectify_,
                                         P1,
                                         left_camera_info.image_size_,
                                         CV_32F,
                                         // output:
                                         left_camera_info.undistRect_map_x_,
                                         left_camera_info.undistRect_map_y_);
  } else {
    LOG(ERROR) << "Camera distortion model not found for left camera!";
  }

  // Right camera
  if (right_camera_info.distortion_model_ == "radtan" ||
      right_camera_info.distortion_model_ == "radial-tangential") {
    // Get rectification & undistortion maps. (radtan dist. model)
    VLOG(10) << "Right camera distortion: radtan";
    cv::initUndistortRectifyMap(right_camera_info.camera_matrix_,
                                right_camera_info.distortion_coeff_,
                                right_camera_info.R_rectify_,
                                P2,
                                right_camera_info.image_size_,
                                CV_32FC1,
                                // Output:
                                right_camera_info.undistRect_map_x_,
                                right_camera_info.undistRect_map_y_);
  } else if (right_camera_info.distortion_model_ == "equidistant") {
    // Get rectification & undistortion maps. (equi dist. model)
    VLOG(10) << "Right camera distortion: equidistant";
    cv::fisheye::initUndistortRectifyMap(right_camera_info.camera_matrix_,
                                         right_camera_info.distortion_coeff_,
                                         right_camera_info.R_rectify_,
                                         P2,
                                         right_camera_info.image_size_,
                                         CV_32F,
                                         // Output:
                                         right_camera_info.undistRect_map_x_,
                                         right_camera_info.undistRect_map_y_);
  } else {
    LOG(ERROR) << "Camera distortion model not found for right camera!";
  }

  // Store intermediate results from rectification.
  // contains an extra column to project in homogeneous coordinates
  left_camera_info.P_ = P1;
  // contains an extra column to project in homogeneous coordinates
  right_camera_info.P_ = P2;
  VLOG(10) << "Storing undistRect maps and other rectification parameters!";
}

/* -------------------------------------------------------------------------- */
void StereoFrame::getRightKeypointsLKunrectified() {
  Frame& ref_frame = left_frame_;
  Frame& cur_frame = right_frame_;

  if (left_frame_.keypoints_.size() == 0)
    LOG(FATAL) << "computeStereo: no keypoints found";

  // get correspondences on right image by using Lucas Kanade
  // Parameters
  int klt_max_iter = 40;
  int klt_win_size = 31;

  ////////////////////////////////////////////////////////////////////////
  // Setup termination criteria for optical flow
  std::vector<uchar> status;
  std::vector<float> error;
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                            klt_max_iter, klt_max_iter);

  // Fill up structure for reference pixels and their labels
  KeypointsCV px_ref;
  px_ref.reserve(ref_frame.keypoints_.size());
  for (size_t i = 0; i < ref_frame.keypoints_.size(); ++i)
    px_ref.push_back(ref_frame.keypoints_[i]);

  // Initialize to old locations
  KeypointsCV px_cur = px_ref;
  if (px_cur.size() > 0) {
    // Do the actual tracking, so px_cur becomes the new pixel locations
    cv::calcOpticalFlowPyrLK(ref_frame.img_, cur_frame.img_, px_ref, px_cur,
                             status, error,
                             cv::Size2i(klt_win_size, klt_win_size), 4,
                             termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);
  } else {
    LOG(FATAL)
        << "computeStereo: no available keypoints for stereo computation";
  }

  cur_frame.keypoints_.clear();
  int nrValidDepths = 0;
  for (int i = 0; i < px_ref.size(); i++)  // fill in right frame
  {
    cur_frame.keypoints_.push_back(px_cur[i]);

    if (status[i] != 0) {  // we correctly tracked the point
      nrValidDepths += 1;
    } else {
      ref_frame.landmarks_[i] = -1;  // make point invalid
    }
  }

  if (cur_frame.keypoints_.size() != ref_frame.keypoints_.size())
    LOG(FATAL)
        << "computeStereo: error -  length of computeStereo is incorrect";

  std::cout << "stereo matching: matched  " << nrValidDepths << " out of "
            << ref_frame.keypoints_.size() << " keypoints" << std::endl;
}

/* -------------------------------------------------------------------------- */
StatusKeypointsCV StereoFrame::getRightKeypointsRectified(
    const cv::Mat left_rectified,
    const cv::Mat right_rectified,
    const StatusKeypointsCV& left_keypoints_rectified,
    const double& fx,
    const double& baseline) const {
  int verbosity = 0;  // Change back to 0
  bool writeImageLeftRightMatching = false;

  // The stripe has to be places in the right image, on the left-hand-side wrt
  // the x of the left feature, since: disparity = left_px.x - right_px.x, hence
  // we check: right_px.x < left_px.x a stripe to select in the right image
  // (this must contain match as epipolar lines are horizontal)
  int stripe_rows =
      sparse_stereo_params_.templ_rows_ +
      sparse_stereo_params_
          .stripe_extra_rows_;  // must be odd; p/m stripe_extra_rows/2 pixels
                                // to deal with rectification error
  // dimension of the search space in right camera is defined by min depth:
  // depth = fx * b / disparity => max disparity = fx * b / minDepth;
  int stripe_cols =
      std::round(fx * baseline / sparse_stereo_params_.min_point_dist_) +
      sparse_stereo_params_.templ_cols_ + 4;  // 4 is a tolerance
  // std::cout << "stripe_cols " << stripe_cols << " stripe_rows " <<
  // stripe_rows << " right_rectified.cols "<< right_rectified.cols
  //    << " minPointDist " << sparseStereoParams_.minPointDist << std::endl;
  if (stripe_cols % 2 != 1) {
    stripe_cols += 1;
  }  // make it odd, if it is not
  if (stripe_cols > right_rectified.cols) {
    stripe_cols = right_rectified.cols;
  }  // if we exagerated with the stripe columns

  // for each point in the (rectified) left image we try to get the pixel which
  // maximizes correlation with (rectified) right image along the (horizontal)
  // epipolar line
#ifdef USE_OMP
  Matrixf right_keypoints_rectified_matrix =
      Matrixf::Zero(2, left_keypoints_rectified.size());
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>
      status_right_keypoints_rectified_matrix =
          Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Zero(
              1, left_keypoints_rectified.size());
#pragma omp parallel for
  for (size_t i = 0; i < left_keypoints_rectified.size(); ++i) {
    // std::cout << "Using " << omp_get_num_threads() << " parallel cores" <<
    // std::endl;
    if (left_keypoints_rectified[i].first !=
        Kstatus::VALID) {  // skip invalid points (fill in with placeholders in
                           // right)
      continue;
    }

    // Do left->right matching
    KeypointCV left_rectified_i = left_keypoints_rectified[i].second;
    StatusKeypointCV right_rectified_i_candidate;
    double matchingVal_LR;
    std::tie(right_rectified_i_candidate, matchingVal_LR) =
        findMatchingKeypointRectified(
            left_rectified, left_rectified_i, right_rectified,
            sparseStereoParams_.templ_cols, sparseStereoParams_.templ_rows,
            stripe_cols, stripe_rows,
            sparseStereoParams_.toleranceTemplateMatching,
            writeImageLeftRightMatching);

    right_keypoints_rectified_matrix(0, i) =
        right_rectified_i_candidate.second.x;
    right_keypoints_rectified_matrix(1, i) =
        right_rectified_i_candidate.second.y;
    status_right_keypoints_rectified_matrix(0, i) =
        right_rectified_i_candidate.first;
  }
  StatusKeypointsCV right_keypoints_rectified;
  right_keypoints_rectified.reserve(left_keypoints_rectified.size());
  for (size_t i = 0; i < left_keypoints_rectified.size(); ++i) {
    if (left_keypoints_rectified[i].first != Kstatus::VALID) {
      right_keypoints_rectified.push_back(std::make_pair(
          left_keypoints_rectified[i].first, KeypointCV(0.0, 0.0)));
    } else {
      right_keypoints_rectified.push_back(std::make_pair(
          static_cast<Kstatus>(status_right_keypoints_rectified_matrix(i)),
          KeypointCV(right_keypoints_rectified_matrix(0, i),
                     right_keypoints_rectified_matrix(1, i))));
    }
  }
#endif
  StatusKeypointsCV right_keypoints_rectified;
  right_keypoints_rectified.reserve(left_keypoints_rectified.size());

  // Serial version
  for (size_t i = 0; i < left_keypoints_rectified.size(); ++i) {
    // check if we already have computed the right kpt, in which case we avoid
    // recomputing
    if (left_keypoints_rectified_.size() > i + 1 &&
        right_keypoints_rectified_.size() > i + 1 &&
        right_keypoints_status_.size() > i + 1 &&  // if we stored enough points
        left_keypoints_rectified[i].second.x ==
            left_keypoints_rectified_[i]
                .x &&  // the query point matches the one we stored
        left_keypoints_rectified[i].second.y ==
            left_keypoints_rectified_[i].y) {
      // we already stored the rectified pixel in the stereo frame
      right_keypoints_rectified.push_back(std::make_pair(
          right_keypoints_status_[i], right_keypoints_rectified_[i]));
      continue;
    }

    // if the left point is invalid, we also set the right point to be invalid
    // and we move on
    if (left_keypoints_rectified[i].first !=
        KeypointStatus::VALID) {  // skip invalid points (fill in with
                                  // placeholders in
                                  // right)
      right_keypoints_rectified.push_back(std::make_pair(
          left_keypoints_rectified[i].first, KeypointCV(0.0, 0.0)));
      continue;
    }

    // Do left->right matching
    KeypointCV left_rectified_i = left_keypoints_rectified[i].second;
    StatusKeypointCV right_rectified_i_candidate;
    double matchingVal_LR;
    // TODO remove tie, potential copies being made.
    std::tie(right_rectified_i_candidate, matchingVal_LR) =
        findMatchingKeypointRectified(
            left_rectified, left_rectified_i, right_rectified,
            sparse_stereo_params_.templ_cols_,
            sparse_stereo_params_.templ_rows_, stripe_cols, stripe_rows,
            sparse_stereo_params_.tolerance_template_matching_,
            writeImageLeftRightMatching);

    // perform bidirectional check: disabled!
    // if(sparseStereoParams_.bidirectionalMatching &&
    // right_rectified_i_candidate.first == Kstatus::VALID){
    //
    //  throw std::runtime_error("getRightKeypointsRectified: bidirectional
    //  matching was not updated to deal with small stripe size");
    //  StatusKeypointCV left_rectified_i_candidate; double matchingVal_RL;
    //  std::tie(left_rectified_i_candidate,matchingVal_RL) =
    //  findMatchingKeypointRectified(right_rectified,
    //  right_rectified_i_candidate.second, left_rectified,
    //      sparseStereoParams_.templ_cols, sparseStereoParams_.templ_rows,
    //      stripe_cols, stripe_rows,
    //      sparseStereoParams_.toleranceTemplateMatching);
    //
    //  if(fabs(left_rectified_i_candidate.second.x - left_rectified_i.x) > 5 //
    //  if matching is not bidirectional
    //      ||  fabs(left_rectified_i_candidate.second.y - left_rectified_i.y) >
    //      5)
    //   // ||  fabs(matchingVal_LR-matchingVal_RL) > 0.1 * matchingVal_RL ) //
    //   and score is not similar in the two directions (found unnecessary)
    //  {
    //    right_rectified_i_candidate.first = Kstatus::NO_RIGHT_RECT;
    //    if(verbosity>0)
    //    {
    //      std::cout << "-------------------------------------" <<std::endl;
    //      std::cout << "matchingVal_LR " << matchingVal_LR <<std::endl;
    //      std::cout << "matchingVal_RL " << matchingVal_RL <<std::endl;
    //      std::cout << "left_rectified_i_candidate " <<
    //      left_rectified_i_candidate.second <<std::endl; std::cout <<
    //      "left_rectified_i " << left_rectified_i <<std::endl; std::cout <<
    //      "-------------------------------------" <<std::endl;
    //    }
    //  }
    //}
    right_keypoints_rectified.push_back(right_rectified_i_candidate);
  }

  if (verbosity > 0) {
    cv::Mat imgL_withKeypoints =
        UtilsOpenCV::DrawCircles(left_rectified, left_keypoints_rectified);
    cv::Mat imgR_withKeypoints =
        UtilsOpenCV::DrawCircles(right_rectified, right_keypoints_rectified);
    showImagesSideBySide(imgL_withKeypoints, imgR_withKeypoints,
                         "result_getRightKeypointsRectified", verbosity);
  }
  return right_keypoints_rectified;
}

/* ---------------------------------------------------------------------------------------
 */
StatusKeypointsCV StereoFrame::getRightKeypointsRectifiedRGBD(
    const cv::Mat left_rectified, const cv::Mat right_rectified,
    const StatusKeypointsCV& left_keypoints_rectified, const double& fx,
    const double& baseline, const double& depth_map_factor,
    const double& min_depth) const {
  int verbosity = 0;
  bool writeImageLeftRightMatching = false;

  StatusKeypointsCV right_keypoints_rectified;
  right_keypoints_rectified.reserve(left_keypoints_rectified.size());

  // Serial version
  for (size_t i = 0; i < left_keypoints_rectified.size(); ++i) {
    // check if we already have computed the right kpt, in which case we avoid
    // recomputing
    if (left_keypoints_rectified_.size() > i + 1 &&
        right_keypoints_rectified_.size() > i + 1 &&
        right_keypoints_status_.size() > i + 1 &&  // if we stored enough points
        left_keypoints_rectified[i].second.x ==
            left_keypoints_rectified_[i]
                .x &&  // the query point matches the one we stored
        left_keypoints_rectified[i].second.y ==
            left_keypoints_rectified_[i].y) {
      // we already stored the rectified pixel in the stereo frame
      right_keypoints_rectified.push_back(std::make_pair(
          right_keypoints_status_[i], right_keypoints_rectified_[i]));
      continue;
    }

    // if the left point is invalid, we also set the right point to be invalid
    // and we move on
    if (left_keypoints_rectified[i].first !=
        KeypointStatus::VALID) {  // skip invalid points (fill in with
                                  // placeholders in
                                  // right)
      right_keypoints_rectified.push_back(std::make_pair(
          left_keypoints_rectified[i].first, KeypointCV(0.0, 0.0)));
      continue;
    }

    // fake left->right matching using depth image
    KeypointCV left_rectified_i = left_keypoints_rectified[i].second;

    // get depth from RGBD
    float depth_from_RGBD =
        depth_map_factor *
        float(right_img_rectified_.at<u_int16_t>(round(left_rectified_i.y),
                                                 round(left_rectified_i.x)));

    // get disparity from RGBD
    float disparityFromRGBD;
    if (depth_from_RGBD > 0.0) {
      // compute disparity from the rgbd depth
      disparityFromRGBD = fx * baseline / depth_from_RGBD;
    } else {
      disparityFromRGBD = 0.0;  // This will get discarded anyway
    }

    StatusKeypointCV right_rectified_i_candidate;

    // Adapted the conditions for a valid disparity
    if ((disparityFromRGBD >= 0.5) && (depth_from_RGBD >= min_depth) &&
        ((left_rectified_i.x - disparityFromRGBD) >
         0)) {  // valid disparity (> 0 pixels), depth > min_depth, in the right
                // image
                // Use correct definition of disparity in VIO code
      right_rectified_i_candidate =
          std::make_pair(KeypointStatus::VALID,
                         KeypointCV(left_rectified_i.x - disparityFromRGBD,
                                    left_rectified_i.y));
    } else {  // invalid disparity
      right_rectified_i_candidate =
          std::make_pair(KeypointStatus::NO_DEPTH, KeypointCV(0.0, 0.0));
    }
    right_keypoints_rectified.push_back(right_rectified_i_candidate);
  }

  if (verbosity > 0) {
    cv::Mat right_rectified_adapted;
    right_rectified.copyTo(right_rectified_adapted);
    right_rectified_adapted.convertTo(right_rectified_adapted, CV_8UC1);
    cv::Mat imgL_withKeypoints =
        UtilsOpenCV::DrawCircles(left_rectified, left_keypoints_rectified);
    cv::Mat imgR_withKeypoints = UtilsOpenCV::DrawCircles(
        right_rectified_adapted, right_keypoints_rectified);
    showImagesSideBySide(imgL_withKeypoints, imgR_withKeypoints,
                         "result_getRightKeypointsRectified", verbosity);
  }
  return right_keypoints_rectified;
}

/* -------------------------------------------------------------------------- */
std::pair<StatusKeypointCV, double> StereoFrame::findMatchingKeypointRectified(
    const cv::Mat left_rectified, const KeypointCV& left_rectified_i,
    const cv::Mat right_rectified, const int templ_cols, const int templ_rows,
    const int stripe_cols, const int stripe_rows, const double tol_corr,
    const bool debugStereoMatching) const {
  /// correlation matrix
  int result_cols = stripe_cols - templ_cols + 1;
  int result_rows = stripe_rows - templ_rows + 1;
  cv::Mat result;
  // result.create( result_rows, result_cols, CV_32FC1 );

  int rounded_left_rectified_i_x = round(left_rectified_i.x);
  int rounded_left_rectified_i_y = round(left_rectified_i.y);

  /// CORRECTLY PLACE THE TEMPLATE (IN LEFT IMAGE) ///////////////////////////
  int temp_corner_y =
      rounded_left_rectified_i_y -
      (templ_rows - 1) / 2;  // y-component of upper left corner of template
  if (temp_corner_y < 0 ||
      temp_corner_y + templ_rows >
          left_rectified.rows -
              1) {  // template exceeds bottom or top of the image
    return std::make_pair(
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0)),
        -1.0);  // skip point too close to up or down boundary
  }
  int offset_temp = 0;  // compensate when the template falls off the image
  int temp_corner_x = rounded_left_rectified_i_x - (templ_cols - 1) / 2;
  if (temp_corner_x <
      0) {  // template exceeds on the left-hand-side of the image
    offset_temp = temp_corner_x;  // offset_temp a bit to make the template
                                  // inside the image
    temp_corner_x = 0;  // because of the offset_temp, the template corner ends
                        // up on the image border
  }
  if (temp_corner_x + templ_cols >
      left_rectified.cols -
          1) {  // template exceeds on the right-hand-side of the image
    if (offset_temp != 0)
      LOG(FATAL) << "findMatchingKeypointRectified: offset_temp cannot exceed "
                    "in both directions!";
    offset_temp = (temp_corner_x + templ_cols) -
                  (left_rectified.cols - 1);  // amount that exceeds
    temp_corner_x -= offset_temp;  // corner has to be offset_temp to the left
                                   // by the amount that exceeds
  }
  // create template
  cv::Rect templ_selector =
      cv::Rect(temp_corner_x, temp_corner_y, templ_cols, templ_rows);
  cv::Mat templ(left_rectified, templ_selector);
  // templ.convertTo(templ, CV_8U);

  // std::cout << "template: \n" << templ << std::endl;

  /// CORRECTLY PLACE THE STRIPE (IN RIGHT IMAGE) ///////////////////////////
  int stripe_corner_y =
      rounded_left_rectified_i_y -
      (stripe_rows - 1) / 2;  // y-component of upper left corner of stripe
  if (stripe_corner_y < 0 ||
      stripe_corner_y + stripe_rows >
          right_rectified.rows -
              1) {  // stripe exceeds bottom or top of the image
    return std::make_pair(
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, KeypointCV(0.0, 0.0)),
        -1.0);  // skip point too close to boundary
  }
  int offset_stripe = 0;  // compensate when the template falls off the image
  int stripe_corner_x =
      rounded_left_rectified_i_x + (templ_cols - 1) / 2 -
      stripe_cols;  // y-component of upper left corner of stripe
  if (stripe_corner_x + stripe_cols >
      right_rectified.cols - 1) {  // stripe exceeds on the right of image
    offset_stripe = (stripe_corner_x + stripe_cols) -
                    (right_rectified.cols - 1);  // amount that exceeds
    stripe_corner_x -= offset_stripe;
  }
  if (stripe_corner_x < 0)  // stripe exceeds on the left of the image
    stripe_corner_x = 0;    // set to left-most column
  // create stripe
  cv::Rect stripe_selector =
      cv::Rect(stripe_corner_x, stripe_corner_y, stripe_cols, stripe_rows);
  cv::Mat stripe(right_rectified, stripe_selector);
  // stripe.convertTo(stripe, CV_8U);

  // find template and normalize results
  double minVal;
  double maxVal;
  cv::Point minLoc;
  cv::Point maxLoc;

  cv::matchTemplate(stripe, templ, result, CV_TM_SQDIFF_NORMED);
  // result.convertTo(result, CV_32F);
  /// Localizing the best match with minMaxLoc
  cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

  // normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() ); // TODO:
  // do we need to normalize??

  cv::Point matchLoc = minLoc;  // position within the result matrix
  matchLoc.x += stripe_corner_x + (templ_cols - 1) / 2 + offset_temp;
  matchLoc.y += stripe_corner_y + (templ_rows - 1) / 2;  // from result to image
  KeypointCV match_px(matchLoc.x, matchLoc.y);  // our desired pixel match

  // debug:
  // int result_cols =  stripe_cols - templ_cols + 1;
  // int result_rows = stripe_rows - templ_rows + 1;
  //  if(debugStereoMatching){
  //    KeypointsCV left_rectified_i_draw;
  //    left_rectified_i_draw.push_back(left_rectified_i); cv::Mat
  //    left_rectifiedWithKeypoints = UtilsOpenCV::DrawCircles(left_rectified,
  //    left_rectified_i_draw); std::cout << "Rectangle 1" << std::endl;
  //    rectangle( left_rectifiedWithKeypoints, templ_selector, cv::Scalar( 0,
  //    255, 255 )); cv::Mat right_rectified_rect;
  //    right_rectified.copyTo(right_rectified_rect);
  //    cv::cvtColor(right_rectified_rect, right_rectified_rect,
  //    cv::COLOR_GRAY2BGR); std::cout << "converted to color" << std::endl;
  //    cv::circle(right_rectified_rect, matchLoc, 3, cv::Scalar(0, 0, 255), 2);
  //    cv::Point textLoc = matchLoc + cv::Point(-10,-5);
  //    cv::putText(right_rectified_rect,
  //    UtilsOpenCV::To_string_with_precision(minVal),
  //        textLoc, CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(0, 0, 255));
  //    std::cout << "put text" << std::endl;
  //    rectangle( right_rectified_rect, stripe_selector, cv::Scalar( 0, 255,
  //    255 )); std::cout << "Rectangle 2" << std::endl; std::string img_title =
  //    "rectifiedWithKeypointsAndRects_" + std::to_string(std::rand()) + "_" ;
  //    std::cout << img_title << std::endl;
  //    showImagesSideBySide(left_rectifiedWithKeypoints,right_rectified_rect,img_title,
  //    2);
  //  }

  // Refine keypoint with subpixel accuracy.
  if (sparse_stereo_params_.subpixel_refinement_) {
    cv::TermCriteria criteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    cv::Size winSize(10, 10);
    cv::Size zeroZone(-1, -1);
    std::vector<cv::Point2f> corner;
    corner.push_back(match_px);  // make in correct format for cornerSubPix
    cv::cornerSubPix(right_rectified, corner, winSize, zeroZone, criteria);
    match_px = corner[0];
  }

  if (minVal < tol_corr) {  // Valid point with small mismatch wrt template
    return std::make_pair(std::make_pair(KeypointStatus::VALID, match_px),
                          minVal);
  } else {
    return std::make_pair(
        std::make_pair(KeypointStatus::NO_RIGHT_RECT, match_px), minVal);
  }
}

/* -------------------------------------------------------------------------- */
StereoFrame::LandmarkInfo StereoFrame::getLandmarkInfo(
    const LandmarkId& i) const {
  // output to populate:
  LandmarkInfo lInfo;
  CHECK_EQ(left_frame_.landmarks_.size(), keypoints_3d_.size())
      << "StereoFrame: getLandmarkKeypointAgekeypoint_3d size mismatch";
  CHECK_EQ(left_frame_.landmarks_.size(), left_frame_.scores_.size())
      << "StereoFrame: scores_ size mismatch";

  for (size_t ind = 0; ind < left_frame_.landmarks_.size(); ind++) {
    if (left_frame_.landmarks_.at(ind) == i) {  // this is the desired landmark
      lInfo.keypoint = left_frame_.keypoints_.at(ind);
      lInfo.score = left_frame_.scores_.at(ind);
      lInfo.age = left_frame_.landmarksAge_.at(ind);
      lInfo.keypoint_3d = keypoints_3d_.at(ind);
      return lInfo;
    }
  }
  // If we got here without finding the landmark there is something wrong.
  LOG(FATAL) << "getLandmarkKeypointAgeVersor: landmark not found";
}

/* -------------------------------------------------------------------------- */
std::vector<double> StereoFrame::getDepthFromRectifiedMatches(
    StatusKeypointsCV& left_keypoints_rectified,
    StatusKeypointsCV& right_keypoints_rectified, const double& fx,
    const double& b) const {
  // depth = fx * baseline / disparity (should be fx = focal * sensorsize)
  double fx_b = fx * b;

  std::vector<double> disparities;
  std::vector<double> depths;
  CHECK_EQ(left_keypoints_rectified.size(), right_keypoints_rectified.size())
      << "getDepthFromRectifiedMatches: size mismatch!";

  int nrValidDepths = 0;
  // disparity = left_px.x - right_px.x, hence we check: right_px.x < left_px.x
  for (size_t i = 0; i < left_keypoints_rectified.size(); i++) {
    if (left_keypoints_rectified[i].first == KeypointStatus::VALID &&
        right_keypoints_rectified[i].first == KeypointStatus::VALID) {
      KeypointCV left_px = left_keypoints_rectified[i].second;
      KeypointCV right_px = right_keypoints_rectified[i].second;
      double disparity = left_px.x - right_px.x;
      if (disparity >= 0.0) {
        // Valid.
        nrValidDepths += 1;
        double depth = fx_b / disparity;
        if (depth < sparse_stereo_params_.min_point_dist_ ||
            depth > sparse_stereo_params_.max_point_dist_) {
          right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
          depths.push_back(0.0);
        } else {
          depths.push_back(depth);
        }
      } else {
        // Right match was wrong.
        right_keypoints_rectified[i].first = KeypointStatus::NO_DEPTH;
        depths.push_back(0.0);
      }
    } else {
      // Something is wrong.
      if (left_keypoints_rectified[i].first != KeypointStatus::VALID) {
        // We cannot have a valid right, without a valid left keypoint.
        right_keypoints_rectified[i].first = left_keypoints_rectified[i].first;
      }
      depths.push_back(0.0);
    }
  }
  CHECK_EQ(left_keypoints_rectified.size(), depths.size())
      << "getDepthFromRectifiedMatches: depths size mismatch!";

  return depths;
}

/* -------------------------------------------------------------------------- */
void StereoFrame::print() const {
  LOG(INFO) << "=====================\n"
            << "id_: " << id_ << '\n'
            << "timestamp_: " << timestamp_ << '\n'
            << "isRectified_: " << is_rectified_ << '\n'
            << "isKeyframe_: " << is_keyframe_ << '\n'
            << "nr keypoints in left: " << left_frame_.keypoints_.size() << '\n'
            << "nr keypoints in right: " << right_frame_.keypoints_.size()
            << '\n'
            << "nr keypoints_depth_: " << keypoints_depth_.size() << '\n'
            << "nr keypoints_3d_: " << keypoints_3d_.size() << '\n'
            << "left_frame_.cam_param_.body_Pose_cam_: "
            << left_frame_.cam_param_.body_Pose_cam_ << '\n'
            << "right_frame_.cam_param_.body_Pose_cam_: "
            << right_frame_.cam_param_.body_Pose_cam_;
}

/* -------------------------------------------------------------------------- */
void StereoFrame::showOriginal(const int verbosity) const {
  CHECK(!is_rectified_) << "showOriginal: but images are already rectified";
  showImagesSideBySide(left_frame_.img_, right_frame_.img_,
                       "original: left-right", verbosity);
}

/* -------------------------------------------------------------------------- */
// TODO visualization (aka imshow/waitKey) must be done in the main thread...
void StereoFrame::showRectified(const int verbosity) const {
  CHECK(is_rectified_) << "showRectified: please perform rectification before "
                          "asking to visualize rectified images";
  // showImagesSideBySide(left_frame_.img_,
  //                      right_frame_.img_,
  //                      "rectified: left-right");
  cv::Mat canvas_undistort =
      drawEpipolarLines(left_frame_.img_, right_frame_.img_, 15);
  if (verbosity > 1) {
    std::string img_name =
        "./outputImages/rectified_" + std::to_string(id_) + ".png";
    cv::imwrite(img_name, canvas_undistort);
  }
  cv::imshow("Rectified!", canvas_undistort);
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
// TODO visualization (aka imshow/waitKey) must be done in the main thread...
void StereoFrame::showImagesSideBySide(const cv::Mat imL, const cv::Mat imR,
                                       const std::string& title,
                                       const int& verbosity) const {
  if (verbosity == 0) return;

  cv::Mat originalLR = UtilsOpenCV::ConcatenateTwoImages(imL, imR);
  if (verbosity == 1) {
    cv::namedWindow(
        title, cv::WINDOW_AUTOSIZE);  // moved in here to allow saving images
    cv::imshow("originalLR", originalLR);
    cv::waitKey(1);
  } else if (verbosity == 2) {
    std::string img_name =
        "./outputImages/" + title + std::to_string(id_) + ".png";
    cv::imwrite(img_name, originalLR);
  }
}

/* -------------------------------------------------------------------------- */
cv::Mat StereoFrame::drawEpipolarLines(const cv::Mat img1, const cv::Mat img2,
                                       const int& numLines,
                                       const int& verbosity) const {
  cv::Mat canvas = UtilsOpenCV::ConcatenateTwoImages(img1, img2);
  int lineGap = canvas.rows / (numLines + 1);
  for (int l = 0; l < numLines; l++) {
    float yPos = (l + 1) * lineGap;
    cv::line(canvas, cv::Point2f(0, yPos), cv::Point2f(canvas.cols - 1, yPos),
             cv::Scalar(0, 255, 0));
  }
  if (verbosity > 1) {
    std::string img_name =
        "./outputImages/drawEpipolarLines_" + std::to_string(id_) + ".png";
    cv::imwrite(img_name, canvas);
  }
  return canvas;
}

/* -------------------------------------------------------------------------- */
void StereoFrame::displayLeftRightMatches() const {
  CHECK_EQ(left_frame_.keypoints_.size(), right_frame_.keypoints_.size())
      << "displayLeftRightMatches: error -  nr of corners in left and right "
         "cameras must be the same";

  // Draw the matchings: assumes that keypoints in the left and right keyframe
  // are ordered in the same way
  std::vector<cv::DMatch> matches;
  for (size_t i = 0; i < left_frame_.keypoints_.size(); i++) {
    matches.push_back(cv::DMatch(i, i, 0));
  }
  cv::Mat match_vis = UtilsOpenCV::DrawCornersMatches(
      left_frame_.img_, left_frame_.keypoints_, right_frame_.img_,
      right_frame_.keypoints_, matches);
  cv::imshow("match_visualization", match_vis);
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
void StereoFrame::displayKeypointStats(
    const StatusKeypointsCV& right_keypoints_rectified) const {
  int nrValid = 0;
  int nrNoLeftRect = 0;
  int nrNoRightRect = 0;
  int nrNoDepth = 0;
  int nrFailedArunRKP = 0;
  for (const StatusKeypointCV& right_keypoint_rectified :
       right_keypoints_rectified) {
    switch (right_keypoint_rectified.first) {
      case KeypointStatus::VALID: {
        nrValid++;
        break;
      }
      case KeypointStatus::NO_LEFT_RECT: {
        nrNoLeftRect++;
        break;
      }
      case KeypointStatus::NO_RIGHT_RECT: {
        nrNoRightRect++;
        break;
      }
      case KeypointStatus::NO_DEPTH: {
        nrNoDepth++;
        break;
      }
      case KeypointStatus::FAILED_ARUN: {
        nrFailedArunRKP++;
        break;
      }
    }
  }
  LOG(INFO) << "Nr of right keypoints: " << right_keypoints_rectified.size()
            << " of which:\n"
            << "nrValid: " << nrValid << "\n"
            << "nrNoLeftRect: " << nrNoLeftRect << "\n"
            << "nrNoRightRect: " << nrNoRightRect << "\n"
            << "nrNoDepth: " << nrNoDepth;
}

}  // namespace VIO
