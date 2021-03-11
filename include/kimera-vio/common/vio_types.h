#pragma once

#include <cstdint>
#include <memory>
#include <utility>  // for forward
#include <vector>

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>

#include <glog/logging.h>

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
using LandmarkCV = cv::Point3d;
using LandmarksCV = std::vector<LandmarkCV>;
enum class LandmarkType { SMART, PROJECTION };
using KeypointCV = cv::Point2f;
using KeypointsCV = std::vector<KeypointCV>;
using StatusKeypointCV = std::pair<KeypointStatus, KeypointCV>;
using StatusKeypointsCV = std::vector<StatusKeypointCV>;
using BearingVectors =
    std::vector<gtsam::Vector3, Eigen::aligned_allocator<gtsam::Vector3>>;

// TODO(Toni): we wouldn't need this if we didn't use vector of pairs.
static void getValidKeypointsFromStatusKeypointsCV(
    const StatusKeypointsCV& status_kpts_cv,
    KeypointsCV* valid_kpts) {
  CHECK_NOTNULL(valid_kpts)->clear();
  valid_kpts->resize(status_kpts_cv.size());
  for (const StatusKeypointCV& status_kpt: status_kpts_cv) {
    if (status_kpt.first == KeypointStatus::VALID) {
      valid_kpts->push_back(status_kpt.second);
    }
  }
}

// TODO(Toni): move make unique and  to underlying to another file...
// Add compatibility for c++11's lack of make_unique.
template <typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}

// Add way of printing strongly typed enums (enum class).
template <typename E>
constexpr typename std::underlying_type<E>::type to_underlying(E e) noexcept {
  return static_cast<typename std::underlying_type<E>::type>(e);
}

// Safely downcast and deal with errors. This works on raw types (not pointers)
// NOTE: this will create a copy
template <class Base, class Derived>
inline Derived safeCast(const Base& base) {
  try {
    return dynamic_cast<const Derived&>(base);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting an object that is not "
                  "the one you expected!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when dynamic casting.";
  }
}

// Safely downcast shared pointers. Will not create copies of underlying data,
// but returns a new pointer. 
template <typename Base, typename Derived>
inline std::shared_ptr<Derived> safeCast(std::shared_ptr<Base> base_ptr) {
  CHECK(base_ptr);
  try {
    return std::dynamic_pointer_cast<Derived>(base_ptr);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting an object that is not "
                  "the one you expected!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when dynamic casting.";
  }
}

// Safely downcast unique pointers.
// NOTE: pass by rvalue because this prevents the copy of the pointer and 
// is faster. All others don't transfer ownership (aren't using move types)
// and so don't need to pass by rvalue.
template <typename Base, typename Derived>
inline std::unique_ptr<Derived> safeCast(std::unique_ptr<Base>&& base_ptr) {
  std::unique_ptr<Derived> derived_ptr;
  Derived* tmp = nullptr;
  try {
    tmp = dynamic_cast<Derived*>(base_ptr.get());
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting an object that is not "
                  "the one you expected!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when dynamic casting.";
  }
  if (!tmp) return nullptr;
  base_ptr.release();
  derived_ptr.reset(tmp);
  return derived_ptr;
}

}  // namespace VIO
