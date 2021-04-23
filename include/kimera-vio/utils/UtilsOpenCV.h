/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UtilsOpenCV.h
 * @brief  Utilities to interface GTSAM with OpenCV
 * @author Antoni Rosinol
 * @author Luca Carlone
 */

#pragma once

#include <iostream>
#include <string>

#include <glog/logging.h>

#include <Eigen/Core>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/opencv.hpp>

#include <gtsam/geometry/Unit3.h>
#include <gtsam/inference/Symbol.h>

// This is only added so that everybody that adds
// this file to have Timestamp for example does
// not complain for now.
#include "kimera-vio/common/vio_types.h"

// TODO(Toni): do not forward declare that much!!
// Forward declare classes.
namespace gtsam {
class Pose3;
class Rot3;
class Cal3_S2;
}

namespace cv {
template <typename T>
class Affine3;
typedef Affine3<float> Affine3f;
}

namespace opengv {
typedef Eigen::Matrix<double, 3, 4> transformation_t;
}

// TODO remove most of these definitions...
namespace VIO {
// Holds the ids of the mesh triangles that should be clustered together.
struct TriangleCluster {
  // Ids of the triangles in the cluster.
  // Increasing id assigned to each triangle.
  // Maybe ask for Lmk ids of vertices in triangle? Later needed to know
  // which lmks have regularity constraints and which do not.
  std::vector<int> triangle_ids_;  // TODO refactor

  // Id of the cluster, determines the color of the cluster when plotting.
  int cluster_id_;

  // Direction of the normal defining the cluster.
  cv::Point3f cluster_direction_;
};

// TODO this should not be a class, should a bunch of functions
// under the appropriate namespace
class UtilsOpenCV {
 public:
  /* ------------------------------------------------------------------------ */
  // Returns the type of an OpenCV matrix in string format.
  static std::string typeToString(int type);

  /* ------------------------------------------------------------------------ */
  // compares 2 cv::Mat
  static bool compareCvMatsUpToTol(const cv::Mat& mat1,
                                   const cv::Mat& mat2,
                                   const double& tol = 1e-7);

  /* ------------------------------------------------------------------------ */
  // comparse 2 cvPoints
  static bool CvPointCmp(const cv::Point2f& p1,
                         const cv::Point2f& p2,
                         const double& tol = 1e-7);

  /* ------------------------------------------------------------------------ */
  // Converts a gtsam::Unit3 to a cv::Point3d.
  static inline cv::Point3d unit3ToPoint3d(const gtsam::Unit3& unit3) {
    return cv::Point3d(
        unit3.point3().x(), unit3.point3().y(), unit3.point3().z());
  }

  /* ------------------------------------------------------------------------ */
  // Converts a cv::Point3d to a gtsam::Unit3.
  static inline gtsam::Unit3 point3dToUnit3(const cv::Point3d& point_3d) {
    CHECK_NEAR(cv::norm(point_3d), 1.0, 1e-5);  // Expect unit norm.
    return gtsam::Unit3(point_3d.x, point_3d.y, point_3d.z);
  }

  /* ------------------------------------------------------------------------ */
  // Converts a vector of 16 elements listing the elements of a 4x4 3D pose
  // matrix by rows into a pose3 in gtsam
  static gtsam::Pose3 poseVectorToGtsamPose3(
      const std::vector<double>& vector_pose);

  /* ------------------------------------------------------------------------ */
  // Converts a gtsam pose3 to a 3x3 rotation matrix and translation vector
  // in opencv format (note: the function only extracts R and t, without
  // changing them)
  static std::pair<cv::Mat, cv::Mat> Pose2cvmats(const gtsam::Pose3& pose);

  /* ------------------------------------------------------------------------ */
  // Converts a gtsam rotation in matrix form to a opencv mat.
  static cv::Mat gtsamMatrix3ToCvMat(const gtsam::Matrix3& rot);

  /* ------------------------------------------------------------------------ */
  // Converts a gtsam translation to a opencv mat.
  static cv::Mat gtsamVector3ToCvMat(const gtsam::Vector3& tran);
  static cv::Point3d gtsamVector3ToCvPoint3(const gtsam::Vector3& tran);

  /* ------------------------------------------------------------------------ */
  // Converts a gtsam pose3 to a opencv Affine3d
  static cv::Affine3d gtsamPose3ToCvAffine3d(const gtsam::Pose3& pose);

  /* ------------------------------------------------------------------------ */
  // Converts a rotation matrix and translation vector from opencv to gtsam
  // pose3
  static gtsam::Pose3 cvMatsToGtsamPose3(const cv::Mat& R, const cv::Mat& T);

  /* ------------------------------------------------------------------------ */
  // Converts a 3x3 rotation matrix from opencv to gtsam Rot3
  static gtsam::Rot3 cvMatToGtsamRot3(const cv::Mat& R);

  /* ------------------------------------------------------------------------ */
  // Converts a 3x1 OpenCV matrix to gtsam Point3
  static gtsam::Point3 cvMatToGtsamPoint3(const cv::Mat& cv_t);

  /* ------------------------------------------------------------------------ */
  // Converts a camera matrix from opencv to gtsam::Cal3_S2
  static gtsam::Cal3_S2 Cvmat2Cal3_S2(const cv::Mat& M);

  /* ------------------------------------------------------------------------ */
  // Converts a camera matrix from opencv to gtsam::Cal3_S2
  static cv::Mat Cal3_S2ToCvmat(const gtsam::Cal3_S2& M);

  /* ------------------------------------------------------------------------ */
  // converts an opengv transformation (3x4 [R t] matrix) to a gtsam::Pose3
  static gtsam::Pose3 openGvTfToGtsamPose3(const opengv::transformation_t& RT);

  /* ------------------------------------------------------------------------ */
  // Crops pixel coordinates avoiding that it falls outside image
  static bool cropToSize(cv::Point2f* px, const cv::Size& size);

  /* ------------------------------------------------------------------------ */
  // crop to size and round pixel coordinates to integers
  static bool roundAndCropToSize(cv::Point2f* px, const cv::Size& size);

  /* ------------------------------------------------------------------------ */
  // Get good features to track from image (wrapper for opencv
  // goodFeaturesToTrack)
  static bool ExtractCorners(const cv::Mat& img,
                             std::vector<cv::Point2f>* corners,
                             const int& max_n_corners = 100,
                             const double& qualityLevel = 0.01,
                             const double& minDistance = 10,
                             const int& blockSize = 3,
                             const double& k = 0.04,
                             const bool& useHarrisDetector = false);

  /* --------------------------------------------------------------------------
   */
  // creates pose by aligning initial gravity vector estimates
  static gtsam::Rot3 AlignGravityVectors(
      const gtsam::Vector3& local_gravity_dir,
      const gtsam::Vector3& global_gravity_dir,
      bool round);

  /* ------------------------------------------------------------------------ */
  // rounds entries in a unit3, such that largest entry is saturated to +/-1 and
  // the other become 0
  static gtsam::Unit3 RoundUnit3(const gtsam::Unit3& x);

  /* ------------------------------------------------------------------------ */
  // Generate random vector using random number generator between -sigma and
  // sigma
  static gtsam::Vector3 RandomVectorGenerator(const double sigma);

  /* ------------------------------------------------------------------------ */
  // Generates random noisy pose around identity with rad_sigma and pos_sigma
  static gtsam::Pose3 RandomPose3(const double rad_sigma,
                                  const double pos_sigma);

  /* ------------------------------------------------------------------------ */
  // given two gtsam::Pose3 computes the relative rotation and translation
  // errors: rotError,tranError
  static std::pair<double, double> ComputeRotationAndTranslationErrors(
      const gtsam::Pose3& expectedPose,
      const gtsam::Pose3& actualPose,
      const bool upToScale = false);

  /* ------------------------------------------------------------------------ */
  // reads image and converts to 1 channel image
  static cv::Mat ReadAndConvertToGrayScale(const std::string& img_name,
                                           bool const equalize = false);
  /* ------------------------------------------------------------------------ */
  // reorder block entries of covariance from state: [bias, vel, pose] to [pose
  // vel bias]
  static gtsam::Matrix Covariance_bvx2xvb(const gtsam::Matrix& COV_bvx);

  /* ------------------------------------------------------------------------ */
  static void PlainMatchTemplate(const cv::Mat stripe,
                                 const cv::Mat templ,
                                 cv::Mat& result);

  /* ------------------------------------------------------------------------ */
  // add circles in the image at desired position/size/color
  static void DrawCirclesInPlace(
      cv::Mat& img,
      const KeypointsCV& image_points,
      const cv::Scalar& color = cv::Scalar(0, 255, 0),
      const double& msize = 3,
      const std::vector<int>& point_ids = std::vector<int>(),
      const int& rem_id = 1e9);

  /* ------------------------------------------------------------------------ */
  // add squares in the image at desired position/size/color
  static void DrawSquaresInPlace(
      cv::Mat& img,
      const std::vector<cv::Point2f>& imagePoints,
      const cv::Scalar& color = cv::Scalar(0, 255, 0),
      const double msize = 10,
      const std::vector<int>& pointIds = std::vector<int>(),
      const int remId = 1e9);

  /* ------------------------------------------------------------------------ */
  // add x in the image at desired position/size/color
  static void DrawCrossesInPlace(
      cv::Mat& img,
      const std::vector<cv::Point2f>& imagePoints,
      const cv::Scalar& color = cv::Scalar(0, 255, 0),
      const double msize = 0.2,
      const std::vector<int>& pointIds = std::vector<int>(),
      const int remId = 1e9);

  /* ------------------------------------------------------------------------ */
  // add text (vector of doubles) in the image at desired position/size/color
  static void DrawTextInPlace(
      cv::Mat& img,
      const std::vector<cv::Point2f>& imagePoints,
      const cv::Scalar& color = cv::Scalar(0, 255, 0),
      const double msize = 0.4,
      const std::vector<double>& textDoubles = std::vector<double>());

  /* ------------------------------------------------------------------------ */
  // Concatenate two images and return results as a new mat.
  // Clones the two images.
  static cv::Mat concatenateTwoImages(const cv::Mat& imL_in,
                                      const cv::Mat& imR_in);

  /* ------------------------------------------------------------------------ */
  // Draw corner matches and return results as a new mat.
  static cv::Mat DrawCornersMatches(const cv::Mat& img1,
                                    const KeypointsCV& corners1,
                                    const cv::Mat& img2,
                                    const KeypointsCV& corners2,
                                    const DMatchVec& matches,
                                    const bool& randomColor = false);

  /* ------------------------------------------------------------------------ */
  // Draw corner matches and return results as a new mat.
  static cv::Mat DrawCornersMatches(const cv::Mat& img1,
                                    const StatusKeypointsCV& corners1,
                                    const cv::Mat& img2,
                                    const StatusKeypointsCV& corners2,
                                    const DMatchVec& matches,
                                    const bool& randomColor = false);

  /* ------------------------------------------------------------------------ */
  static void showImagesSideBySide(const cv::Mat& img_left,
                                   const cv::Mat& img_right,
                                   const std::string& title,
                                   const bool& show_images,
                                   const bool& save_images);

  cv::Scalar getColorFromKeypointStatus(
      const KeypointStatus& kpt_status);

  /* ------------------------------------------------------------------------ */
  static cv::Mat DrawCircles(
      const cv::Mat img,
      const StatusKeypointsCV& imagePoints,
      const std::vector<double>& circleSizes = std::vector<double>());

  /**
   * @brief DrawCircles
   * @param img
   * @param imagePoints
   * @param circle_colors
   * @param circle_sizes
   * @param display_with_size
   * if true size of circles is proportional to depth
   * @param display_with_text
   * if true display text with depth
   * @return
   */
  static cv::Mat DrawCircles(
      const cv::Mat img,
      const KeypointsCV& image_points,
      const std::vector<cv::Scalar>& circle_colors = std::vector<cv::Scalar>(),
      const std::vector<double>& circle_sizes = std::vector<double>(),
      const bool& display_with_size = false,
      const bool& display_with_text = true);

  /* ------------------------------------------------------------------------ */
  static void DrawCornersMatchesOneByOne(
      const cv::Mat img1,
      const std::vector<cv::Point2f>& corners1,
      const cv::Mat img2,
      const std::vector<cv::Point2f>& corners2,
      const DMatchVec& matches);

  /* ------------------------------------------------------------------------ */
  //  find max absolute value of matrix entry
  static double MaxAbsValue(gtsam::Matrix M);

  /* ------------------------------------------------------------------------ */
  // compute image gradients (TODO: untested: taken from
  // http://www.coldvision.io/2016/03/18/image-gradient-sobel-operator-opencv-3-x-cuda/)
  static cv::Mat ImageLaplacian(const cv::Mat& img);

  /* ------------------------------------------------------------------------ */
  // compute canny edges (TODO: untested: taken from
  // https://github.com/opencv/opencv/blob/master/samples/cpp/edge.cpp)
  static cv::Mat EdgeDetectorCanny(const cv::Mat& img);

  /* ------------------------------------------------------------------------ */
  // compute max intensity of pixels within a triangle specified by the pixel
  // location of its vertices
  // If intensityThreshold is < 0, then the check is disabled.
  static std::vector<std::pair<KeypointCV, double>> FindHighIntensityInTriangle(
      const cv::Mat img,
      const cv::Vec6f& px_vertices,
      const float intensityThreshold);

  /* ------------------------------------------------------------------------ */
  // Returns a OpenCV file storage in a safely manner, warning about potential
  // exceptions thrown.
  // Param: check_opened, throws if the file cannot be opened.
  static void safeOpenCVFileStorage(cv::FileStorage* fs,
                                    const std::string& filename,
                                    const bool check_opened = true);

  /**
   * @brief getDisparityVis Allows visualization of disparity image (which is
   * usually outputed as CV_16S or CV_32F from stereo depth reconstruction).
   * @param src
   * @param dst
   * @param scale
   * @param unknown_disparity
   */
  static void getDisparityVis(cv::InputArray src,
                              cv::OutputArray dst,
                              int unknown_disparity = 16320);
};

/* -------------------------------------------------------------------------- */
// TODO(Toni): remove this aberration
template <typename T>
struct myGreaterThanPtr {
  bool operator()(const std::pair<const T*, T> a,
                  const std::pair<const T*, T> b) const {
    return *(a.first) > *(b.first);
  }
};

/* -------------------------------------------------------------------------- */
// A Plane defined by a gtsam::Symbol, a normal, a distance, and the set
// of lmk ids that are part of the plane.
// TODO define Plane in a Plane.cpp/.h not in utilsOpenCV
struct Plane {
 public:
  typedef cv::Point3d Normal;

 public:
  Plane(const gtsam::Symbol& plane_symbol,
        const Normal& normal = Normal(),
        const double& distance = 0.0,
        const LandmarkIds& lmk_ids = LandmarkIds(),
        const int& cluster_id = 0)
      : plane_symbol_(plane_symbol),
        normal_(normal),
        distance_(distance),
        lmk_ids_(lmk_ids) {
    // TODO remove, only used for consistency.
    // triangle_cluster is only used for visualziation.
    triangle_cluster_.cluster_id_ = cluster_id;
    triangle_cluster_.cluster_direction_ = normal;
  }

  inline const gtsam::Symbol& getPlaneSymbol() const { return plane_symbol_; }

  bool geometricEqual(const Plane& rhs,
                      const double& normal_tolerance,
                      const double& distance_tolerance) const {
    return (isNormalStrictlyEqual(rhs.normal_, normal_, normal_tolerance) &&
            // TODO implement a better test for distance tolerance... as it can
            // be large for small normal difference and distances are big.
            isPlaneDistanceStrictlyEqual(
                rhs.distance_, distance_, distance_tolerance)) ||
           // Check also the other possible case.
           (isNormalStrictlyEqual(rhs.normal_, -normal_, normal_tolerance) &&
            isPlaneDistanceStrictlyEqual(
                rhs.distance_, -distance_, distance_tolerance));
  }

 private:
  gtsam::Symbol plane_symbol_;

 public:
  Normal normal_;
  double distance_;
  LandmarkIds lmk_ids_;

  // TODO remove, only used for visualization...
  TriangleCluster triangle_cluster_;

 private:
  typedef cv::Point3d NormalInternal;
  /* ------------------------------------------------------------------------ */
  // Is normal equal? True whenever normals are parallel,
  // indep of their direction.
  bool isNormalEqual(const NormalInternal& axis,
                     const NormalInternal& normal,
                     const double& tolerance) const {
    // TODO typedef normals and axis to Normal, and use cv::Point3d instead.
    CHECK_NEAR(cv::norm(axis), 1.0, 1e-5);    // Expect unit norm.
    CHECK_NEAR(cv::norm(normal), 1.0, 1e-5);  // Expect unit norm.
    CHECK_GT(tolerance, 0.0);                 // Tolerance is positive.
    CHECK_LT(tolerance, 1.0);                 // Tolerance is positive.
    // Dot product should be close to 1 or -1 if axis is aligned with normal.
    return (std::fabs(normal.ddot(axis)) > 1.0 - tolerance);
  }

  /* ------------------------------------------------------------------------ */
  // Is normal equal? True whenever normals are aligned.
  bool isNormalStrictlyEqual(const NormalInternal& axis,
                             const NormalInternal& normal,
                             const double& tolerance) const {
    // TODO typedef normals and axis to Normal, and use cv::Point3d instead.
    CHECK_NEAR(cv::norm(axis), 1.0, 1e-5);    // Expect unit norm.
    CHECK_NEAR(cv::norm(normal), 1.0, 1e-5);  // Expect unit norm.
    CHECK_GT(tolerance, 0.0);                 // Tolerance is positive.
    CHECK_LT(tolerance, 1.0);                 // Tolerance is positive.
    // Dot product should be close to 1 or -1 if axis is aligned with normal.
    return (normal.ddot(axis) > 1.0 - tolerance);
  }

  /* ------------------------------------------------------------------------ */
  // Is distance equal? true whenever distances are of similar absolute value.
  bool isPlaneDistanceEqual(const double& dist_1,
                            const double& dist_2,
                            const double& tolerance) const {
    return std::fabs((std::fabs(dist_1) - std::fabs(dist_2)) < tolerance);
  }

  /* ------------------------------------------------------------------------ */
  // Is distance equal? true whenever distances are of similar absolute value.
  bool isPlaneDistanceStrictlyEqual(const double& dist_1,
                                    const double& dist_2,
                                    const double& tolerance) const {
    return std::fabs(dist_1 - dist_2) < tolerance;
  }
};

}  // namespace VIO
