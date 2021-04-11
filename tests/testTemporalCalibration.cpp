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

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/initial/CrossCorrTimeAligner.h"
#include "kimera-vio/initial/TimeAlignerBase.h"

#include <Eigen/Dense>
#include <vector>

namespace VIO {

using ::testing::_;
using ::testing::AllArgs;
using ::testing::Invoke;
using ::testing::Ne;
using ::testing::NotNull;

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
  ReturnHelper(const std::vector<RansacResult>& values) : vec_(values) {
    vec_iter_ = vec_.begin();
  }

  RansacResult getNext(Frame* /* ref */, Frame* /* curr */) {
    if (vec_iter_ == vec_.end()) {
      return std::make_pair(TrackingStatus::INVALID, gtsam::Pose3());
    }
    RansacResult to_return = *vec_iter_;
    ++vec_iter_;
    return to_return;
  }

 private:
  std::vector<RansacResult> vec_;
  std::vector<RansacResult>::const_iterator vec_iter_;
};

// basic sanity check that mocking works as expected
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

FrontendOutputPacketBase::UniquePtr make_output(
    Timestamp timestamp,
    FrontendType frontend_type = FrontendType::kStereoImu) {
  Frame fake_frame(1, timestamp, CameraParams(), cv::Mat());
  if (frontend_type == FrontendType::kMonoImu) {
    return std::move(
        make_unique<MonoFrontendOutput>(false,
                                        StatusMonoMeasurementsPtr(nullptr),
                                        TrackingStatus::VALID,
                                        gtsam::Pose3(),
                                        gtsam::Pose3(),
                                        fake_frame,
                                        ImuFrontend::PimPtr(nullptr),
                                        ImuAccGyrS(6, 1),
                                        cv::Mat(),
                                        DebugTrackerInfo()));
  } else {
    StereoFrame fake_stereo(
        fake_frame.id_, fake_frame.timestamp_, fake_frame, fake_frame);
    return std::move(
        make_unique<StereoFrontendOutput>(false,
                                          StatusStereoMeasurementsPtr(nullptr),
                                          TrackingStatus::VALID,
                                          gtsam::Pose3(),
                                          gtsam::Pose3(),
                                          gtsam::Pose3(),
                                          fake_stereo,
                                          ImuFrontend::PimPtr(nullptr),
                                          ImuAccGyrS(6, 1),
                                          cv::Mat(),
                                          DebugTrackerInfo()));
  }
}

TEST(temporalCalibration, testBadRansacStatus) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(
      TrackingStatus::INVALID,
      gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 1).finished()));
  results.emplace_back(
      TrackingStatus::DISABLED,
      gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 2).finished()));

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejectionMono(NotNull(), NotNull()))
      .With(AllArgs(Ne()))
      .Times(2)
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  CrossCorrTimeAligner aligner(params);

  FrontendOutputPacketBase::UniquePtr output = make_output(1);
  ImuStampS times(1, 0);
  ImuAccGyrS values(6, 0);

  // Set initial frame
  TimeAlignerBase::Result result =
      aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if RANSAC is invalid (first result)
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if 5pt RANSAC is disabled (second result)
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testEmptyImu) {
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
      .Times(1)
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  CrossCorrTimeAligner aligner(params);

  FrontendOutputPacketBase::UniquePtr output = make_output(1);
  ImuStampS times(1, 0);
  ImuAccGyrS values(6, 0);

  // Set initial frame
  TimeAlignerBase::Result result =
      aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if the IMU isn't present between frames
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testLessThanWindow) {
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
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.time_alignment_window_size_ = 10;
  CrossCorrTimeAligner aligner(params);

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::UniquePtr output = make_output(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testLessThanWindowFrameRate) {
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
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.time_alignment_window_size_ = 10;
  params.do_imu_rate_time_alignment_ = false;
  CrossCorrTimeAligner aligner(params);

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::UniquePtr output = make_output(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testLowVariance) {
  MockTracker tracker;

  ImuParams params;
  params.gyro_noise_density_ = 1.0;
  params.time_alignment_window_size_ = 3;
  params.do_imu_rate_time_alignment_ = false;
  CrossCorrTimeAligner aligner(params);

  std::vector<RansacResult> results;
  for (size_t i = 0; i < params.time_alignment_window_size_; ++i) {
    results.emplace_back(
        TrackingStatus::VALID,
        gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 1).finished()));
  }

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejectionMono(NotNull(), NotNull()))
      .With(AllArgs(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::UniquePtr output = make_output(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);

    // We get false either from not having enough data or not having enough
    // variance
    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testEnoughVariance) {
  MockTracker tracker;

  ImuParams params;
  params.gyro_noise_density_ = 0.0;
  params.time_alignment_window_size_ = 3;
  params.do_imu_rate_time_alignment_ = false;
  CrossCorrTimeAligner aligner(params);

  std::vector<RansacResult> results;
  for (size_t i = 0; i < params.time_alignment_window_size_; ++i) {
    results.emplace_back(
        TrackingStatus::VALID,
        gtsam::Pose3(gtsam::Rot3(), (Eigen::Vector3d() << 0, 0, 1).finished()));
  }

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejectionMono(NotNull(), NotNull()))
      .With(AllArgs(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::UniquePtr output = make_output(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    if (i < results.size()) {
      // We get false from not having enough data
      EXPECT_FALSE(result.valid);
      EXPECT_EQ(0.0, result.imu_time_shift);
    } else {
      EXPECT_TRUE(result.valid);
      // result needs to be somewhere with the min and max possible time
      EXPECT_GE(UtilsNumerical::NsecToSec(results.size() - 1),
                result.imu_time_shift);
      EXPECT_LE(UtilsNumerical::NsecToSec(-results.size() + 1),
                result.imu_time_shift);
    }
  }
}

TEST(temporalCalibration, testWellFormedNoDelay) {
  MockTracker tracker;

  ImuParams params;
  params.gyro_noise_density_ = 0.0;
  params.time_alignment_window_size_ = 10;
  params.do_imu_rate_time_alignment_ = false;
  params.nominal_sampling_time_s_ = 1.0;
  CrossCorrTimeAligner aligner(params);

  const double rotation_scale = 0.1;  // radians per sample

  std::vector<RansacResult> results;
  for (size_t i = 0; i < params.time_alignment_window_size_; ++i) {
    if (i < 5) {
      results.emplace_back(TrackingStatus::VALID,
                           gtsam::Pose3(gtsam::Rot3::Rz(rotation_scale * i),
                                        Eigen::Vector3d::Zero()));
    } else {
      results.emplace_back(
          TrackingStatus::VALID,
          gtsam::Pose3(gtsam::Rot3::Rz(rotation_scale * (10 - i)),
                       Eigen::Vector3d::Zero()));
    }
  }

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejectionMono(NotNull(), NotNull()))
      .With(AllArgs(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  FrontendOutputPacketBase::UniquePtr output = make_output(0);
  ImuStampS times(1, 1);
  times << 0;
  ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);
  aligner.estimateTimeAlignment(tracker, *output, times, values);

  for (size_t i = 0; i < results.size(); ++i) {
    FrontendOutputPacketBase::UniquePtr output = make_output(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);
    if (i < 5) {
      values(3, 0) = rotation_scale * i;  // create some signal
    } else {
      values(3, 0) = rotation_scale * (10 - i);
    }
    if (i != results.size() - 1) {
      aligner.estimateTimeAlignment(tracker, *output, times, values);
    } else {
      TimeAlignerBase::Result result =
          aligner.estimateTimeAlignment(tracker, *output, times, values);
      EXPECT_TRUE(result.valid);
      EXPECT_EQ(0.0, result.imu_time_shift);
    }
  }
}

}  // namespace VIO
