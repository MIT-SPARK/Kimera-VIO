/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   UtilsOpenCV.cpp
 * @brief  Utilities to interface GTSAM with OpenCV
 * @author Antoni Rosinol, Luca Carlone
 */

#include "kimera-vio/utils/UtilsOpenCV.h"

#include <stdlib.h>
#include <sys/time.h>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <opengv/point_cloud/methods.hpp>

#include <opencv2/core/eigen.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3DS2.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

namespace VIO {

/* -------------------------------------------------------------------------- */
std::string UtilsOpenCV::typeToString(int type) {
  std::string r;
  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);
  switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
  }
  r += "C";
  r += (chans + '0');
  return r;
}

/* -------------------------------------------------------------------------- */
// Compares two opencv matrices up to a certain tolerance error.
bool UtilsOpenCV::compareCvMatsUpToTol(const cv::Mat& mat1,
                                       const cv::Mat& mat2,
                                       const double& tol) {
  // treat two empty mat as identical as well
  if (mat1.empty() && mat2.empty()) {
    LOG(WARNING) << "CvMatCmp: asked comparison of 2 empty matrices.";
    return true;
  }

  // if dimensionality of two mats are not identical, these two mats are not
  // identical
  if (mat1.cols != mat2.cols || mat1.rows != mat2.rows ||
      mat1.dims != mat2.dims) {
    return false;
  }

  // Compare the two matrices!
  cv::Mat diff = mat1 - mat2;
  return cv::checkRange(diff, true, 0, -tol, tol);
}

/* -------------------------------------------------------------------------- */
// comparse 2 cvPoints
bool UtilsOpenCV::CvPointCmp(const cv::Point2f& p1,
                             const cv::Point2f& p2,
                             const double& tol) {
  return std::abs(p1.x - p2.x) <= tol && std::abs(p1.y - p2.y) <= tol;
}

/* -------------------------------------------------------------------------- */
// Converts a vector of 16 elements listing the elements of a 4x4 3D pose matrix
// by rows into a pose3 in gtsam
gtsam::Pose3 UtilsOpenCV::poseVectorToGtsamPose3(
    const std::vector<double>& vector_pose) {
  CHECK_EQ(vector_pose.size(), 16u);
  DCHECK_EQ(vector_pose[12], 0.0);
  DCHECK_EQ(vector_pose[13], 0.0);
  DCHECK_EQ(vector_pose[14], 0.0);
  DCHECK_EQ(vector_pose[15], 1.0);
  return gtsam::Pose3(Eigen::Matrix4d(vector_pose.data()).transpose());
}

/* -------------------------------------------------------------------------- */
// Converts a gtsam pose3 to a 3x3 rotation matrix and translation vector
// in opencv format (note: the function only extracts R and t, without changing
// them)
std::pair<cv::Mat, cv::Mat> UtilsOpenCV::Pose2cvmats(const gtsam::Pose3& pose) {
  const gtsam::Matrix3& rot = pose.rotation().matrix();
  const gtsam::Vector3& tran = pose.translation();
  return std::make_pair(gtsamMatrix3ToCvMat(rot), gtsamVector3ToCvMat(tran));
}

cv::Mat UtilsOpenCV::gtsamMatrix3ToCvMat(const gtsam::Matrix3& rot) {
  cv::Mat R = cv::Mat(3, 3, CV_64F);
  cv::eigen2cv(rot, R);
  return R;
}

cv::Mat UtilsOpenCV::gtsamVector3ToCvMat(const gtsam::Vector3& tran) {
  cv::Mat T = cv::Mat(3, 1, CV_64F);
  cv::eigen2cv(tran, T);
  return T;
}

/* -------------------------------------------------------------------------- */
// Converts a gtsam pose3 to a opencv Affine3d
cv::Affine3d UtilsOpenCV::gtsamPose3ToCvAffine3d(const gtsam::Pose3& pose) {
  cv::Mat RT(4, 4, CV_64F);
  cv::eigen2cv(pose.matrix(), RT);
  return cv::Affine3d(RT);
}
/* -------------------------------------------------------------------------- */
// Converts a rotation matrix and translation vector from opencv to gtsam pose3
gtsam::Pose3 UtilsOpenCV::cvMatsToGtsamPose3(const cv::Mat& R,
                                             const cv::Mat& T) {
  return gtsam::Pose3(cvMatToGtsamRot3(R), cvMatToGtsamPoint3(T));
}

/* -------------------------------------------------------------------------- */
// Converts a 3x3 rotation matrix from opencv to gtsam Rot3
gtsam::Rot3 UtilsOpenCV::cvMatToGtsamRot3(const cv::Mat& R) {
  CHECK_EQ(R.rows, 3);
  CHECK_EQ(R.cols, 3);
  gtsam::Matrix rot_mat = gtsam::Matrix::Identity(3, 3);
  cv::cv2eigen(R, rot_mat);
  return gtsam::Rot3(rot_mat);
}

/* -------------------------------------------------------------------------- */
// Converts a 3x1 OpenCV matrix to gtsam Point3
gtsam::Point3 UtilsOpenCV::cvMatToGtsamPoint3(const cv::Mat& cv_t) {
  CHECK_EQ(cv_t.rows, 3);
  CHECK_EQ(cv_t.cols, 1);
  gtsam::Point3 gtsam_t;
  gtsam_t << cv_t.at<double>(0, 0), cv_t.at<double>(1, 0),
      cv_t.at<double>(2, 0);
  return gtsam_t;
}

/* -------------------------------------------------------------------------- */
// Converts a 3x3 (or 3xM, M > 3) camera matrix from opencv to gtsam::Cal3_S2.
gtsam::Cal3_S2 UtilsOpenCV::Cvmat2Cal3_S2(const cv::Mat& M) {
  CHECK_EQ(M.rows, 3);  // We expect homogeneous camera matrix.
  CHECK_GE(M.cols, 3);  // We accept extra columns (which we do not use).
  const double& fx = M.at<double>(0, 0);
  const double& fy = M.at<double>(1, 1);
  const double& s = M.at<double>(0, 1);
  const double& u0 = M.at<double>(0, 2);
  const double& v0 = M.at<double>(1, 2);
  return gtsam::Cal3_S2(fx, fy, s, u0, v0);
}

/* -------------------------------------------------------------------------- */
// Converts a camera matrix from opencv to gtsam::Cal3_S2
cv::Mat UtilsOpenCV::Cal3_S2ToCvmat(const gtsam::Cal3_S2& M) {
  cv::Mat C = cv::Mat::eye(3, 3, CV_64F);
  C.at<double>(0, 0) = M.fx();
  C.at<double>(1, 1) = M.fy();
  C.at<double>(0, 1) = M.skew();
  C.at<double>(0, 2) = M.px();
  C.at<double>(1, 2) = M.py();
  return C;
}

/* -------------------------------------------------------------------------- */
// Converts an opengv transformation (3x4 [R t] matrix) to a gtsam::Pose3
gtsam::Pose3 UtilsOpenCV::openGvTfToGtsamPose3(
    const opengv::transformation_t& RT) {
  gtsam::Matrix poseMat = gtsam::Matrix::Identity(4, 4);
  poseMat.block<3, 4>(0, 0) = RT;
  return gtsam::Pose3(poseMat);
}

/* -------------------------------------------------------------------------- */
// Crops pixel coordinates avoiding that it falls outside image
bool UtilsOpenCV::cropToSize(cv::Point2f* px, const cv::Size& size) {
  CHECK_NOTNULL(px);
  bool cropped = false;
  float max_width = static_cast<float>(size.width - 1);
  if (px->x > max_width) {
    px->x = max_width;
    cropped = true;
  } else if (px->x < 0.0f) {
    px->x = 0.0f;
    cropped = true;
  }
  float max_height = static_cast<float>(size.height - 1);
  if (px->y > max_height) {
    px->y = max_height;
    cropped = true;
  } else if (px->y < 0.0f) {
    px->y = 0.0f;
    cropped = true;
  }
  return cropped;
}
/* -------------------------------------------------------------------------- */
/** @brief Crop to size and round pixel coordinates to integers
 * @param
 * @return whether the pixel was cropped or not.
 */
bool UtilsOpenCV::roundAndCropToSize(cv::Point2f* px, const cv::Size& size) {
  CHECK_NOTNULL(px);
  px->x = round(px->x);
  px->y = round(px->y);
  return cropToSize(px, size);
}

/* -------------------------------------------------------------------------- */
bool UtilsOpenCV::ExtractCorners(const cv::Mat& img,
                                 std::vector<cv::Point2f>* corners,
                                 const double& qualityLevel,
                                 const double& minDistance,
                                 const int& blockSize,
                                 const double& k,
                                 const bool& useHarrisDetector) {
  CHECK_NOTNULL(corners)->clear();
  // TODO(Toni): wtf are all these hardcoded things :(
  // please remove...
  static const cv::TermCriteria criteria(
      CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
  static const cv::Size winSize(10, 10);
  static const cv::Size zeroZone(-1, -1);
  try {
    // Extract the corners
    cv::goodFeaturesToTrack(img,
                            *corners,
                            100,
                            qualityLevel,
                            minDistance,
                            cv::noArray(),
                            blockSize,
                            useHarrisDetector,
                            k);
    cv::cornerSubPix(img, *corners, winSize, zeroZone, criteria);
  } catch (...) {
    LOG(ERROR) << "ExtractCorners: no corner found in image.";
    return false;
  }
  return true;
}

/* -------------------------------------------------------------------------- */
template <typename T>
bool UtilsOpenCV::myGreaterThanPtr<T>::operator()(
    const std::pair<const T*, T> a, const std::pair<const T*, T> b) const {
  return *(a.first) > *(b.first);
}
/* -------------------------------------------------------------------------- */
// TODO(Toni): VIT this function should be optimized and cleaned.
// Refactor corners_with_scores using typedef.
// Use std::vector<std::pair<cv::Point2f, double>>
// Or  std::vector<std::pair<KeypointCV, Score>> but not a pair of vectors.
// Remove hardcoded parameters (there are a ton).
void UtilsOpenCV::MyGoodFeaturesToTrackSubPix(
    const cv::Mat& image,
    const int& maxCorners,
    const double& qualityLevel,
    double minDistance,
    const cv::Mat& mask,
    const int& blockSize,
    const bool& useHarrisDetector,
    const double& harrisK,
    std::pair<std::vector<cv::Point2f>, std::vector<double>>*
        corners_with_scores) {
  CHECK_NOTNULL(corners_with_scores);
  try {
    cv::Mat eig, tmp;
    double maxVal = 0;
    double minVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    // get eigenvalue image & get peak
    if (useHarrisDetector) {
      cv::cornerHarris(image, eig, blockSize, 3, harrisK);
    } else {
      cv::cornerMinEigenVal(image, eig, blockSize, 3);
    }

    // Cut off corners below quality level.
    cv::minMaxLoc(eig, &minVal, &maxVal, &minLoc, &maxLoc, mask);
    // Cut stuff below quality.
    cv::threshold(eig, eig, maxVal * qualityLevel, 0, CV_THRESH_TOZERO);
    cv::dilate(eig, tmp, cv::Mat());

    const cv::Size& imgsize = image.size();

    // Create corners.
    std::vector<std::pair<const float*, float>> tmpCornersScores;

    // collect list of pointers to features - put them into temporary image
    for (int y = 1; y < imgsize.height - 1; y++) {
      const float* eig_data = (const float*)eig.ptr(y);
      const float* tmp_data = (const float*)tmp.ptr(y);
      const uchar* mask_data = mask.data ? mask.ptr(y) : nullptr;

      for (int x = 1; x < imgsize.width - 1; x++) {
        const float& val = eig_data[x];
        // TODO this takes a ton of time 12ms each time...
        if (val != 0 && val == tmp_data[x] && (!mask_data || mask_data[x])) {
          tmpCornersScores.push_back(std::make_pair(eig_data + x, val));
        }
      }
    }

    std::sort(tmpCornersScores.begin(), tmpCornersScores.end(),
              myGreaterThanPtr<float>());

    // Put sorted corner in other struct.
    size_t j;
    size_t total = tmpCornersScores.size();
    size_t ncorners = 0;

    if (minDistance >= 1) {
      // Partition the image into larger grids
      int w = image.cols;
      int h = image.rows;

      const int cell_size = cvRound(minDistance);
      const int grid_width = (w + cell_size - 1) / cell_size;
      const int grid_height = (h + cell_size - 1) / cell_size;

      std::vector<std::vector<cv::Point2f>> grid(grid_width * grid_height);

      minDistance *= minDistance;

      for (size_t i = 0; i < total; i++) {
        int ofs = (int)((const uchar*)tmpCornersScores[i].first - eig.data);
        int y = (int)(ofs / eig.step);
        int x = (int)((ofs - y * eig.step) / sizeof(float));
        double eigVal = double(tmpCornersScores[i].second);

        bool good = true;

        int x_cell = x / cell_size;
        int y_cell = y / cell_size;

        int x1 = x_cell - 1;
        int y1 = y_cell - 1;
        int x2 = x_cell + 1;
        int y2 = y_cell + 1;

        // boundary check
        x1 = std::max(0, x1);
        y1 = std::max(0, y1);
        x2 = std::min(grid_width - 1, x2);
        y2 = std::min(grid_height - 1, y2);

        for (int yy = y1; yy <= y2; yy++) {
          for (int xx = x1; xx <= x2; xx++) {
            const std::vector<cv::Point2f>& m = grid[yy * grid_width + xx];

            if (m.size()) {
              for (j = 0; j < m.size(); j++) {
                const float& dx = x - m[j].x;
                const float& dy = y - m[j].y;

                if (dx * dx + dy * dy < minDistance) {
                  good = false;
                  goto break_out;
                }
              }
            }
          }
        }

      break_out:

        if (good) {
          // printf("%d: %d %d -> %d %d, %d, %d -- %d %d %d %d, %d %d, c=%d\n",
          //    i,x, y, x_cell, y_cell, (int)minDistance, cell_size,x1,y1,x2,y2,
          //    grid_width,grid_height,c);
          grid[y_cell * grid_width + x_cell].push_back(
              cv::Point2f((float)x, (float)y));
          corners_with_scores->first.push_back(cv::Point2f((float)x, (float)y));
          corners_with_scores->second.push_back(eigVal);
          ++ncorners;
          if (maxCorners > 0 && (int)ncorners == maxCorners) {
            break;
          }
        }
      }
    } else {
      for (size_t i = 0; i < total; i++) {
        int ofs = (int)((const uchar*)tmpCornersScores[i].first - eig.data);
        int y = (int)(ofs / eig.step);
        int x = (int)((ofs - y * eig.step) / sizeof(float));
        double eigVal = double(tmpCornersScores[i].second);
        corners_with_scores->first.push_back(cv::Point2f((float)x, (float)y));
        corners_with_scores->second.push_back(eigVal);
        ++ncorners;
        if (maxCorners > 0 && (int)ncorners == maxCorners) {
          break;
        }
      }
    }

    // subpixel accuracy: TODO: create function for the next 4 lines
    // TODO(Toni): REMOVE all these hardcoded stuff...
    static const cv::TermCriteria criteria(
        CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001);
    static const cv::Size winSize(10, 10);
    static const cv::Size zeroZone(-1, -1);

    // TODO this takes a ton of time 27ms each time...
    cv::cornerSubPix(image, corners_with_scores->first, winSize, zeroZone,
                     criteria);
  } catch (...) {
    // Corners remains empty.
    LOG(WARNING) << "ExtractCorners: no corner found in image.";
  }
}

/* -------------------------------------------------------------------------- */
// Returns rotation that aligs two vectors.
gtsam::Rot3 UtilsOpenCV::AlignGravityVectors(
    const gtsam::Vector3& local_gravity_dir,
    const gtsam::Vector3& global_gravity_dir,
    bool round) {
  // a = localGravity (we measure the opposite of gravity).
  gtsam::Unit3 localGravityDir(local_gravity_dir);
  // b
  gtsam::Unit3 globalGravityDir(global_gravity_dir);

  if (round) {
    // Align vectors to dominant axis: e.g. [0.01 0.1 1] becomes [0 0 1]
    localGravityDir = UtilsOpenCV::RoundUnit3(localGravityDir);
    globalGravityDir = UtilsOpenCV::RoundUnit3(globalGravityDir);
  }

  // a x b
  gtsam::Unit3 cross_product = localGravityDir.cross(globalGravityDir);
  double c = localGravityDir.dot(globalGravityDir);
  // compute rotation such that R * a = b
  // http://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d/476311#476311
  gtsam::Rot3 R;
  if (std::fabs(1 - c) < 1e-3) {
    // Already aligned.
    R = gtsam::Rot3();
  } else if (std::fabs(1 + c) < 1e-3) {
    // Degenerate condition a = -b
    // Compute cross product with any nonparallel vector.
    gtsam::Unit3 perturbedGravity(localGravityDir.unitVector() +
                                  gtsam::Vector3(1, 2, 3));
    cross_product = localGravityDir.cross(perturbedGravity);

    // If the resulting vector is still not a number
    // (i.e. perturbedGravity // localGravityDir)
    if (std::isnan(cross_product.unitVector()(0))) {
      // Compute cross product with any nonparallel vector.
      perturbedGravity =
          gtsam::Unit3(localGravityDir.unitVector() + gtsam::Vector3(3, 2, 1));
      cross_product = localGravityDir.cross(perturbedGravity);
    }
    // 180 rotation around an axis perpendicular to both vectors
    R = gtsam::Rot3::Expmap(cross_product.unitVector() * M_PI);
  } else {
    R = gtsam::Rot3::AlignPair(
        cross_product, globalGravityDir, localGravityDir);
  }

  return R;
}

/* -------------------------------------------------------------------------- */
// rounds entries in a unit3, such that largest entry is saturated to +/-1 and the other become 0
gtsam::Unit3 UtilsOpenCV::RoundUnit3(const gtsam::Unit3& x){

  gtsam::Vector3 x_vect_round = gtsam::Vector3::Zero();
  gtsam::Vector3 x_vect = x.unitVector();
  double max_x = (x_vect.cwiseAbs()).maxCoeff();  // max absolute value
  for (size_t i = 0; i < 3; i++) {
    if (fabs(fabs(x_vect(i)) - max_x) < 1e-4) {  // found max element
      x_vect_round(i) = x_vect(i) / max_x;       // can be either -1 or +1
      break;  // tie breaker for the case in which multiple elements attain the
              // max
    }
  }
  return gtsam::Unit3(x_vect_round);
}
/* -------------------------------------------------------------------------- */
// rounds number to a specified number of decimal digits
// (digits specifies the number of digits to keep AFTER the decimal point)
double UtilsOpenCV::RoundToDigit(const double x, const int digits) {
  double dec = pow(10, digits);  // 10^digits
  double y = double(round(x * dec)) / dec;
  return y;
}
/* ------------------------------------------------------------------------ */
// Generate random float using random number generator between -sigma and sigma
double UtilsOpenCV::RandomFloatGenerator(const double sigma) {
  return ((double)rand() / RAND_MAX) * sigma - sigma/2.0;
}
/* ------------------------------------------------------------------------ */
// Generate random vector using random number generator between -sigma and sigma
gtsam::Vector3 UtilsOpenCV::RandomVectorGenerator(const double sigma) {
  double x = RandomFloatGenerator(sigma);
  double y = RandomFloatGenerator(sigma);
  double z = RandomFloatGenerator(sigma);
  return gtsam::Vector3(x, y, z);
}
/* ------------------------------------------------------------------------ */
// Generates random noisy pose around identity with rad_sigma and pos_sigma
gtsam::Pose3 UtilsOpenCV::RandomPose3(const double rad_sigma, const double pos_sigma) {
  gtsam::Vector3 rot = RandomVectorGenerator(rad_sigma);
  gtsam::Vector3 pos = RandomVectorGenerator(pos_sigma);
  return gtsam::Pose3(gtsam::Rot3::RzRyRx(rot.x(), rot.y(), rot.z()), pos);
}
/* -------------------------------------------------------------------------- */
// converts doulbe to sting with desired number of digits (total number of
// digits)
std::string UtilsOpenCV::To_string_with_precision(const double a_value,
                                                  const int n) {
  std::ostringstream out;
  out << std::setprecision(n) << a_value;
  return out.str();
}
/* -------------------------------------------------------------------------- */
// converts time from nanoseconds to seconds
double UtilsOpenCV::NsecToSec(const Timestamp& timestamp) {
  return double(timestamp) * 1e-9;
}
/* -------------------------------------------------------------------------- */
// (NOT TESTED): converts time from seconds to nanoseconds
std::int64_t UtilsOpenCV::SecToNsec(const double timeInSec) {
  return double(timeInSec * 1e9);
}
/* -------------------------------------------------------------------------- */
// (NOT TESTED): get current time in seconds
double UtilsOpenCV::GetTimeInSeconds() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  int64_t time_usec = tv.tv_sec * (int64_t)1e6 + tv.tv_usec;
  return ((double)time_usec * 1e-6);
}
/* -------------------------------------------------------------------------- */
// given two gtsam::Pose3 computes the relative rotation and translation errors:
// rotError,tranError
std::pair<double, double> UtilsOpenCV::ComputeRotationAndTranslationErrors(
    const gtsam::Pose3& expectedPose, const gtsam::Pose3& actualPose,
    const bool upToScale) {
  // compute errors
  gtsam::Rot3 rotErrorMat =
      (expectedPose.rotation()).between(actualPose.rotation());
  gtsam::Vector3 rotErrorVector = gtsam::Rot3::Logmap(rotErrorMat);
  double rotError = rotErrorVector.norm();

  gtsam::Vector3 actualTranslation = actualPose.translation().vector();
  gtsam::Vector3 expectedTranslation = expectedPose.translation().vector();
  if (upToScale) {
    double normExpected = expectedTranslation.norm();
    double normActual = actualTranslation.norm();
    if (normActual > 1e-5)
      actualTranslation = normExpected * actualTranslation /
                          normActual;  // we manually add the scale here
  }
  gtsam::Vector3 tranErrorVector = expectedTranslation - actualTranslation;
  double tranError = tranErrorVector.norm();
  return std::make_pair(rotError, tranError);
}
/* -------------------------------------------------------------------------- */
// Reads image and converts to 1 channel image.
cv::Mat UtilsOpenCV::ReadAndConvertToGrayScale(const std::string& img_name,
                                               bool equalize) {
  cv::Mat img = cv::imread(img_name, cv::IMREAD_ANYCOLOR);
  if (img.channels() > 1) cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
  if (equalize) {  // Apply Histogram Equalization
    LOG(INFO) << "- Histogram Equalization for image: " << img_name;
    cv::equalizeHist(img, img);
  }
  return img;
}

/* -------------------------------------------------------------------------- */
// reorder block entries of covariance from state: [bias, vel, pose] to [pose
// vel bias]
gtsam::Matrix UtilsOpenCV::Covariance_bvx2xvb(const gtsam::Matrix& COV_bvx) {
  gtsam::Matrix cov_xvb = COV_bvx;
  // fix diagonals: poses
  cov_xvb.block<6, 6>(0, 0) = COV_bvx.block<6, 6>(9, 9);
  // fix diagonals: velocity: already in place
  // fix diagonals: biases
  cov_xvb.block<6, 6>(9, 9) = COV_bvx.block<6, 6>(0, 0);

  // off diagonal, pose-vel
  cov_xvb.block<6, 3>(0, 6) = COV_bvx.block<6, 3>(9, 6);
  cov_xvb.block<3, 6>(6, 0) = (cov_xvb.block<6, 3>(0, 6)).transpose();
  // off diagonal, pose-bias
  cov_xvb.block<6, 6>(0, 9) = COV_bvx.block<6, 6>(9, 0);
  cov_xvb.block<6, 6>(9, 0) = (cov_xvb.block<6, 6>(0, 9)).transpose();
  // off diagonal, vel-bias
  cov_xvb.block<3, 6>(6, 9) = COV_bvx.block<3, 6>(6, 0);
  cov_xvb.block<6, 3>(9, 6) = (cov_xvb.block<3, 6>(6, 9)).transpose();

  return cov_xvb;
}
/* -------------------------------------------------------------------------- */
void UtilsOpenCV::PlainMatchTemplate(const cv::Mat stripe, const cv::Mat templ,
                                     cv::Mat& result) {
  int result_cols = stripe.cols - templ.cols + 1;
  int result_rows = stripe.rows - templ.rows + 1;

  result.create(result_rows, result_cols, CV_32FC1);
  float diffSq = 0, tempSq = 0, stripeSq = 0;
  for (int ii = 0; ii < templ.rows; ii++) {
    for (int jj = 0; jj < templ.cols; jj++) {
      tempSq += pow((int)templ.at<uchar>(ii, jj), 2);
      // std::cout << " templ.at<double>(ii,jj) " << (int)
      // templ.at<uchar>(ii,jj) << std::endl;
    }
  }
  for (size_t i = 0; i < result_rows; i++) {
    for (size_t j = 0; j < result_cols; j++) {
      diffSq = 0;
      stripeSq = 0;

      for (int ii = 0; ii < templ.rows; ii++) {
        for (int jj = 0; jj < templ.cols; jj++) {
          diffSq += pow((int)templ.at<uchar>(ii, jj) -
                            (int)stripe.at<uchar>(i + ii, j + jj),
                        2);
          stripeSq += pow((int)stripe.at<uchar>(i + ii, j + jj), 2);
        }
      }
      result.at<float>(i, j) = diffSq / sqrt(tempSq * stripeSq);
    }
  }
}
/* -------------------------------------------------------------------------- */
// add circles in the image at desired position/size/color
void UtilsOpenCV::DrawCirclesInPlace(
    cv::Mat& img, const std::vector<cv::Point2f>& imagePoints,
    const cv::Scalar& color, const double msize,
    const std::vector<int>& pointIds, const int remId) {
  cv::Point2f textOffset = cv::Point2f(-10, -5);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    cv::circle(img, imagePoints[i], msize, color, 2);
    if (pointIds.size() == imagePoints.size())  // we also have text
      cv::putText(img, std::to_string(pointIds[i] % remId),
                  imagePoints[i] + textOffset, CV_FONT_HERSHEY_COMPLEX, 0.5,
                  color);
  }
}
/* -------------------------------------------------------------------------- */
// add squares in the image at desired position/size/color
void UtilsOpenCV::DrawSquaresInPlace(
    cv::Mat& img, const std::vector<cv::Point2f>& imagePoints,
    const cv::Scalar& color, const double msize,
    const std::vector<int>& pointIds, const int remId) {
  cv::Point2f textOffset = cv::Point2f(-10, -5);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    cv::Rect square = cv::Rect(imagePoints[i].x - msize / 2,
                               imagePoints[i].y - msize / 2, msize, msize);
    rectangle(img, square, color, 2);
    if (pointIds.size() == imagePoints.size())  // we also have text
      cv::putText(img, std::to_string(pointIds[i] % remId),
                  imagePoints[i] + textOffset, CV_FONT_HERSHEY_COMPLEX, 0.5,
                  color);
  }
}
/* -------------------------------------------------------------------------- */
// add x in the image at desired position/size/color
void UtilsOpenCV::DrawCrossesInPlace(
    cv::Mat& img, const std::vector<cv::Point2f>& imagePoints,
    const cv::Scalar& color, const double msize,
    const std::vector<int>& pointIds, const int remId) {
  cv::Point2f textOffset = cv::Point2f(-10, -5);         // text offset
  cv::Point2f textOffsetToCenter = cv::Point2f(-3, +3);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    cv::putText(img, "X", imagePoints[i] + textOffsetToCenter,
                CV_FONT_HERSHEY_COMPLEX, msize, color, 2);
    if (pointIds.size() == imagePoints.size())  // we also have text
      cv::putText(img, std::to_string(pointIds[i] % remId),
                  imagePoints[i] + textOffset, CV_FONT_HERSHEY_COMPLEX, 0.5,
                  color);
  }
}
/* -------------------------------------------------------------------------- */
// add text (vector of doubles) in the image at desired position/size/color
void UtilsOpenCV::DrawTextInPlace(cv::Mat& img,
                                  const std::vector<cv::Point2f>& imagePoints,
                                  const cv::Scalar& color, const double msize,
                                  const std::vector<double>& textDoubles) {
  cv::Point2f textOffset = cv::Point2f(-12, -5);  // text offset
  if (img.channels() < 3) cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
  for (size_t i = 0; i < imagePoints.size(); i++) {
    if (imagePoints.size() == textDoubles.size())  // write text
      cv::putText(img, To_string_with_precision(textDoubles.at(i), 3),
                  imagePoints[i] + textOffset, CV_FONT_HERSHEY_COMPLEX, msize,
                  color);
  }
}
/* -------------------------------------------------------------------------- */
// Concatenate two images and return results as a new mat.
// Clones the two images.
cv::Mat UtilsOpenCV::ConcatenateTwoImages(const cv::Mat& imL_in,
                                          const cv::Mat& imR_in) {
  cv::Mat imL = imL_in.clone();
  if (imL.channels() == 1) {
    cv::cvtColor(imL, imL, cv::COLOR_GRAY2BGR);
  }
  cv::Mat imR = imR_in.clone();
  if (imR.channels() == 1) {
    cv::cvtColor(imR, imR, cv::COLOR_GRAY2BGR);
  }
  cv::Size szL = imL.size();
  cv::Size szR = imR.size();
  cv::Mat originalLR(szL.height, szL.width + szR.width, CV_8UC3);
  cv::Mat left(originalLR, cv::Rect(0, 0, szL.width, szL.height));
  imL.copyTo(left);
  cv::Mat right(originalLR, cv::Rect(szL.width, 0, szR.width, szR.height));
  imR.copyTo(right);
  return originalLR;
}

/* -------------------------------------------------------------------------- */
// Draw corner matches and return results as a new mat.
cv::Mat UtilsOpenCV::DrawCornersMatches(
    const cv::Mat& img1, const std::vector<cv::Point2f>& corners1,
    const cv::Mat& img2, const std::vector<cv::Point2f>& corners2,
    const std::vector<cv::DMatch>& matches, const bool randomColor) {
  cv::Mat canvas = UtilsOpenCV::ConcatenateTwoImages(img1, img2);
  cv::Point2f ptOffset = cv::Point2f(img1.cols, 0);
  cv::RNG rng(12345);
  for (int i = 0; i < matches.size(); i++) {
    cv::Scalar color;
    if (randomColor)
      color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),
                         rng.uniform(0, 255));
    else
      color = cv::Scalar(0, 255, 0);
    cv::line(canvas, corners1[matches[i].queryIdx],
             corners2[matches[i].trainIdx] + ptOffset, color);
    cv::circle(canvas, corners1[matches[i].queryIdx], 3, color, 2);
    cv::circle(canvas, corners2[matches[i].trainIdx] + ptOffset, 3, color, 2);
  }
  return canvas;
}
/* -------------------------------------------------------------------------- */
cv::Mat UtilsOpenCV::DrawCircles(const cv::Mat img,
                                 const StatusKeypointsCV& imagePoints,
                                 const std::vector<double>& circleSizes) {
  KeypointsCV valid_imagePoints;
  std::vector<cv::Scalar> circleColors;
  for (size_t i = 0; i < imagePoints.size(); i++) {
    if (imagePoints[i].first ==
        KeypointStatus::VALID) {  // it is a valid point!
      valid_imagePoints.push_back(imagePoints[i].second);
      circleColors.push_back(cv::Scalar(0, 255, 0));  // green
    } else if (imagePoints[i].first ==
               KeypointStatus::NO_RIGHT_RECT) {  // template matching did not
                                                 // pass
      valid_imagePoints.push_back(imagePoints[i].second);
      circleColors.push_back(cv::Scalar(0, 0, 255));
    } else {
      valid_imagePoints.push_back(
          imagePoints[i].second);  // disparity turned out negative
      circleColors.push_back(cv::Scalar(0, 0, 255));
    }
  }
  return UtilsOpenCV::DrawCircles(img, valid_imagePoints, circleColors,
                                  circleSizes);
}
/* -------------------------------------------------------------------------- */
cv::Mat UtilsOpenCV::DrawCircles(const cv::Mat img,
                                 const KeypointsCV& imagePoints,
                                 const std::vector<cv::Scalar>& circleColors,
                                 const std::vector<double>& circleSizes) {
  bool displayWithSize =
      false;  // if true size of circles is proportional to depth
  bool displayWithText = true;     // if true display text with depth
  KeypointCV textOffset(-10, -5);  // text offset
  cv::Mat img_color = img.clone();
  if (img_color.channels() < 3) {
    cv::cvtColor(img_color, img_color, cv::COLOR_GRAY2BGR);
  }
  for (size_t i = 0; i < imagePoints.size(); i++) {
    double circleSize = 3;
    cv::Scalar circleColor = cv::Scalar(0, 255, 0);

    if (displayWithSize && circleSizes.size() == imagePoints.size())
      circleSize = 5 * std::max(circleSizes[i], 0.5);

    if (circleColors.size() == imagePoints.size())
      circleColor = circleColors[i];

    cv::circle(img_color, imagePoints[i], circleSize, circleColor, 2);

    if (displayWithText && circleSizes.size() == imagePoints.size() &&
        circleSizes[i] != -1) {
      cv::putText(img_color,
                  UtilsOpenCV::To_string_with_precision(circleSizes[i]),
                  imagePoints[i] + textOffset, CV_FONT_HERSHEY_COMPLEX, 0.4,
                  circleColor);
    }
  }
  return img_color;
}
/* -------------------------------------------------------------------------- */
void UtilsOpenCV::DrawCornersMatchesOneByOne(
    const cv::Mat img1, const std::vector<cv::Point2f>& corners1,
    const cv::Mat img2, const std::vector<cv::Point2f>& corners2,
    const std::vector<cv::DMatch>& matches) {
  cv::Mat canvas = UtilsOpenCV::ConcatenateTwoImages(img1, img2);
  cv::Point2f ptOffset = cv::Point2f(img1.cols, 0);

  for (int i = 0; i < matches.size(); i++) {
    cv::Mat baseCanvas = canvas.clone();
    printf("Match %d\n", i);
    cv::line(baseCanvas, corners1[matches[i].queryIdx],
             corners2[matches[i].trainIdx] + ptOffset, cv::Scalar(0, 255, 0));
    cv::imshow("Match one by one", baseCanvas);
    cv::waitKey(1);
  }
}
/* -------------------------------------------------------------------------- */
//  find max absolute value of matrix entry
double UtilsOpenCV::MaxAbsValue(gtsam::Matrix M) {
  double maxVal = 0;
  for (size_t i = 0; i < M.rows(); i++) {
    for (size_t j = 0; j < M.cols(); j++) {
      maxVal = std::max(maxVal, fabs(M(i, j)));
    }
  }
  return maxVal;
}
/* -------------------------------------------------------------------------- */
// compute image gradients (TODO: untested: taken from
// http://www.coldvision.io/2016/03/18/image-gradient-sobel-operator-opencv-3-x-cuda/)
cv::Mat UtilsOpenCV::ImageLaplacian(const cv::Mat& img) {
  // duplicate image to preserve const input
  cv::Mat input = img.clone();

  // blur the input image to remove the noise
  cv::GaussianBlur(input, input, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

  // convert it to grayscale (CV_8UC3 -> CV_8UC1)
  cv::Mat input_gray;
  if (input.channels() > 1)
    cv::cvtColor(input, input_gray, cv::COLOR_RGB2GRAY);
  else
    input_gray = input.clone();

  // compute the gradients on both directions x and y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;
  int scale = 1;
  int delta = 0;

  // Scharr( input_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
  cv::Sobel(input_gray, grad_x, CV_16S, 1, 0, 3, scale, delta,
            cv::BORDER_DEFAULT);
  cv::convertScaleAbs(grad_x, abs_grad_x);  // CV_16S -> CV_8U

  // Scharr( input_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
  cv::Sobel(input_gray, grad_y, CV_16S, 0, 1, 3, scale, delta,
            cv::BORDER_DEFAULT);
  cv::convertScaleAbs(grad_y, abs_grad_y);  // CV_16S -> // CV_16S -> CV_8U

  // create the output by adding the absolute gradient images of each x and y
  // direction
  cv::Mat output;
  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, output);
  return output;
}
/* -------------------------------------------------------------------------- */
// compute canny edges (TODO: untested: taken from
// https://github.com/opencv/opencv/blob/master/samples/cpp/edge.cpp)
cv::Mat UtilsOpenCV::EdgeDetectorCanny(const cv::Mat& img) {
  // duplicate image to preserve const input
  cv::Mat input = img.clone();
  cv::equalizeHist(input, input);

  // blur the input image to remove the noise
  cv::GaussianBlur(input, input, cv::Size(3, 3), 0, 0, cv::BORDER_DEFAULT);

  // convert it to grayscale (CV_8UC3 -> CV_8UC1)
  cv::Mat input_gray;
  if (input.channels() > 1)
    cv::cvtColor(input, input_gray, cv::COLOR_RGB2GRAY);
  else
    input_gray = input.clone();

  // Run the edge detector on grayscale
  cv::Mat edges;
  double edgeThresh = 40;
  cv::Canny(input_gray, edges, edgeThresh, edgeThresh * 3, 3);
  return edges;
}
/* -------------------------------------------------------------------------- */
// compute max intensity of pixels within a triangle specified by the pixel
// location of its vertices If intensityThreshold is < 0, then the check is
// disabled.
std::vector<std::pair<KeypointCV, double>>
UtilsOpenCV::FindHighIntensityInTriangle(const cv::Mat img,
                                         const cv::Vec6f& px_vertices,
                                         const float intensityThreshold) {
  std::vector<std::pair<KeypointCV, double>> keypointsWithIntensities;
  if (intensityThreshold < 0) {  // check is disabled
    return keypointsWithIntensities;
  }

  static constexpr bool isDebug = false;

  // parse input vertices
  int x0 = std::round(px_vertices[0]);
  int y0 = std::round(px_vertices[1]);
  int x1 = std::round(px_vertices[2]);
  int y1 = std::round(px_vertices[3]);
  int x2 = std::round(px_vertices[4]);
  int y2 = std::round(px_vertices[5]);

  // get bounding box
  int topLeft_x = std::min(x0, std::min(x1, x2));
  int topLeft_y = std::min(y0, std::min(y1, y2));
  int botRight_x = std::max(x0, std::max(x1, x2));
  int botRight_y = std::max(y0, std::max(y1, y2));

  double min, max;  // for debug
  cv::Mat imgCopy;  // for debug
  if (isDebug) {
    std::vector<cv::Point> pts(3);
    cv::minMaxLoc(img, &min, &max);
    imgCopy = img.clone();
    cv::cvtColor(imgCopy, imgCopy, cv::COLOR_GRAY2BGR);
    pts[0] = cv::Point(x0, y0);
    pts[1] = cv::Point(x1, y1);
    pts[2] = cv::Point(x2, y2);
    cv::rectangle(imgCopy, cv::Point(topLeft_x, topLeft_y),
                  cv::Point(botRight_x, botRight_y), cv::Scalar(0, 255, 0));
    cv::line(imgCopy, pts[0], pts[1], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
    cv::line(imgCopy, pts[1], pts[2], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
    cv::line(imgCopy, pts[2], pts[0], cv::Scalar(0, 255, 0), 1, CV_AA, 0);
  }

  for (int r = topLeft_y; r < botRight_y; r++) {
    // find smallest col inside triangle:
    int min_x = botRight_x;  // initialized to largest
    int max_x = topLeft_x;   // initialized to smallest
    int margin = 4;

    // check triangle 01:
    if (y0 != y1) {  // in this case segment is horizontal and we can skip it
      double lambda01 = double(r - y1) / double(y0 - y1);
      if (lambda01 >= 0 && lambda01 <= 1) {  // intersection belongs to segment
        int x =
            std::round((lambda01) * double(x0) + (1 - lambda01) * double(x1));
        min_x = std::min(min_x, x);  // try to expand segment to the left
        max_x = std::max(max_x, x);  // try to expand segment to the right
      }
    }

    // check triangle 12:
    if (y1 != y2) {  // in this case segment is horizontal and we can skip it
      double lambda12 = double(r - y2) / double(y1 - y2);
      if (lambda12 >= 0 && lambda12 <= 1) {  // intersection belongs to segment
        int x =
            std::round((lambda12) * double(x1) + (1 - lambda12) * double(x2));
        min_x = std::min(min_x, x);  // try to expand segment to the left
        max_x = std::max(max_x, x);  // try to expand segment to the right
      }
    }

    // check triangle 20:
    if (y2 != y0) {  // in this case segment is horizontal and we can skip it
      double lambda20 = double(r - y0) / double(y2 - y0);
      if (lambda20 >= 0 && lambda20 <= 1) {  // intersection belongs to segment
        int x =
            std::round((lambda20) * double(x2) + (1 - lambda20) * double(x0));
        min_x = std::min(min_x, x);  // try to expand segment to the left
        max_x = std::max(max_x, x);  // try to expand segment to the right
      }
    }

    // sanity check
    CHECK(min_x >= topLeft_x && max_x <= botRight_x)
        << min_x << " " << topLeft_x << " " << max_x << " " << botRight_x
        << '\n'
        << "FindHighIntensityInTriangle: inconsistent extrema.";

    for (int c = min_x + margin; c < max_x - margin; c++) {
      float intensity_rc = float(img.at<uint8_t>(r, c));

      if (isDebug) {
        LOG(INFO) << "intensity_rc (r,c): " << intensity_rc << " (" << r << ","
                  << c << ")";
        LOG(INFO) << "min: " << min << " max " << max;
        cv::circle(imgCopy, cv::Point(c, r), 1, cv::Scalar(255, 0, 0),
                   CV_FILLED, CV_AA, 0);
      }

      if (intensity_rc > intensityThreshold) {
        keypointsWithIntensities.push_back(
            std::make_pair(cv::Point(c, r), intensity_rc));
        if (isDebug) {
          cv::circle(imgCopy, cv::Point(c, r), 1, cv::Scalar(0, 0, 255),
                     CV_FILLED, CV_AA, 0);
        }
      }
    }
  }

  if (isDebug) {
    cv::imshow("imgCopy", imgCopy);
    cv::waitKey(1);
  }

  return keypointsWithIntensities;
} // namespace VIO

/* ------------------------------------------------------------------------ */
// Returns a OpenCV file storage in a safely manner, warning about potential
// exceptions thrown.
void UtilsOpenCV::safeOpenCVFileStorage(cv::FileStorage* fs,
                                        const std::string& file_path,
                                        const bool check_opened) {
  CHECK_NOTNULL(fs);
  try {
    *fs = cv::FileStorage(file_path, cv::FileStorage::READ);
  } catch (cv::Exception& e) {
    LOG(FATAL) << "Cannot open file: " << file_path << '\n'
               << "OpenCV error code: " << e.msg;
  }

  if (check_opened) {
    CHECK(fs->isOpened()) << "Cannot open file: " << file_path << " (remember "
                          << "that first line of yaml file must be: %YAML:1.0)";
  }
}

}  // namespace VIO
