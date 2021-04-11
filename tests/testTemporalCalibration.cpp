/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testTemporalCalibration.cpp
 * @brief  Unit tests for time alignment
 * @author Nathan Hughes
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/initial/CrossCorrTimeAligner.h"
#include "kimera-vio/initial/TimeAlignerBase.h"

#include <Eigen/Dense>
#include <vector>

namespace VIO {

using ::testing::_;
using ::testing::Invoke;
using ::testing::NotNull;
using ::testing::AllArgs;
using ::testing::Ne;

typedef std::pair<TrackingStatus, gtsam::Pose3> RansacResult;

class MockTracker : public Tracker {
 public:
  MockTracker()
      : Tracker(FrontendParams(), std::make_shared<Camera>(CameraParams())) {}

  ~MockTracker() = default;

  MOCK_METHOD(RansacResult,
              geometricOutlierRejectionMono,
              (Frame*, Frame*),
              (override));
};

class ReturnHelper {
 public:
  ReturnHelper(const std::vector<RansacResult>& values)
      : vec_(values), vec_iter_(values.begin()) {}

  RansacResult getNext(Frame* /* ref */, Frame* /* curr */) {
    if (vec_iter_ == vec_.end()) {
      return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
    }
    RansacResult to_return = *vec_iter_;
    std::next(vec_iter_);
    return to_return;
  }

 private:
  std::vector<RansacResult> vec_;
  std::vector<RansacResult>::const_iterator vec_iter_;
};

TEST(temporalCalibration, mockedTracker) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(
      TrackingStatus::VALID,
      gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 1).finished()));
  results.emplace_back(
      TrackingStatus::VALID,
      gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 2).finished()));
  results.emplace_back(
      TrackingStatus::VALID,
      gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 3).finished()));

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejectionMono(NotNull(), NotNull()))
      .With(AllArgs(Ne()))
      .Times(3)
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  Frame test_prev(1, 1, CameraParams(), cv::Mat());
  Frame test_curr(1, 1, CameraParams(), cv::Mat());
  tracker.geometricOutlierRejectionMono(&test_prev, &test_curr);
  tracker.geometricOutlierRejectionMono(&test_prev, &test_curr);
  tracker.geometricOutlierRejectionMono(&test_prev, &test_curr);
}

}  // namespace VIO
