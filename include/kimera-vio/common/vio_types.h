#pragma once

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>  // for forward
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <opencv2/core.hpp>

namespace VIO {
using Timestamp = std::int64_t;

// Typedefs of commonly used Eigen matrices and vectors.
using Point2 = gtsam::Point2;
using Point3 = gtsam::Point3;
using Vector3 = gtsam::Vector3;
using Vector6 = gtsam::Vector6;
using Matrix3 = gtsam::Matrix33;
using Matrix6 = gtsam::Matrix66;
using Matrices3 =
    std::vector<gtsam::Matrix3, Eigen::aligned_allocator<gtsam::Matrix3>>;
using Vectors3 = std::vector<Vector3, Eigen::aligned_allocator<Vector3>>;

// float version
using Vector3f = Eigen::Matrix<float, 3, 1>;
using Vector6f = Eigen::Matrix<float, 6, 1>;
using Matrix3f = Eigen::Matrix<float, 3, 3>;
using Matrix6f = Eigen::Matrix<float, 6, 6>;
using Matrixf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>;
using Matrices3f = std::vector<Matrix3f, Eigen::aligned_allocator<Matrix3f>>;
using Vectors3f = std::vector<Vector3f, Eigen::aligned_allocator<Vector3f>>;

enum class KeypointStatus {
  VALID,
  NO_LEFT_RECT,
  NO_RIGHT_RECT,
  NO_DEPTH,
  FAILED_ARUN
};

using Depth = double;
using Depths = std::vector<Depth>;

using DMatchVec = std::vector<cv::DMatch>;

// Definitions relevant to frame types
using FrameId = std::uint64_t;  // Frame id is used as the index of gtsam symbol
                                // (not as a gtsam key).
using PlaneId = std::uint64_t;
using LandmarkId = long int;  // -1 for invalid landmarks. // int would be too
                            // small if it is 16 bits!
using LandmarkIds = std::vector<LandmarkId>;
using Landmark = gtsam::Point3;
using Landmarks = std::vector<Landmark, Eigen::aligned_allocator<Landmark>>;
using LandmarkCV = cv::Point3d;
using LandmarksCV = std::vector<LandmarkCV>;
enum class LandmarkType { SMART, PROJECTION };
using KeypointCV = cv::Point2f;
using KeypointsCV = std::vector<KeypointCV>;
using StatusKeypointCV = std::pair<KeypointStatus, KeypointCV>;
using StatusKeypointsCV = std::vector<StatusKeypointCV>;
using BearingVector = gtsam::Vector3;
using BearingVectors =
    std::vector<BearingVector, Eigen::aligned_allocator<BearingVector>>;

// Add way of printing strongly typed enums (enum class).
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

template <typename Derived, typename Base>
std::unique_ptr<Derived> castUnique(std::unique_ptr<Base>&& base) {
  auto derived = dynamic_cast<Derived*>(base.get());
  if (!derived) {
    return nullptr;
  }

  base.release();
  return std::unique_ptr<Derived>(derived);
}

using LandmarksMap = std::unordered_map<LandmarkId, gtsam::Point3>;

}  // namespace VIO
