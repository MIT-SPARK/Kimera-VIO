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

#include <Eigen/Dense>
#include <vector>

#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/StereoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/Tracker-definitions.h"
#include "kimera-vio/initial/CrossCorrTimeAligner.h"
#include "kimera-vio/initial/TimeAlignerBase.h"
#include "kimera-vio/utils/UtilsNumerical.h"

namespace VIO {

using ::testing::_;
using ::testing::Args;
using ::testing::Invoke;
using ::testing::Ne;
using ::testing::NotNull;

typedef std::pair<TrackingStatus, gtsam::Pose3> RansacResult;

class MockTracker : public Tracker {
 public:
  MockTracker()
      : Tracker(TrackerParams(), std::make_shared<Camera>(CameraParams())) {}

  ~MockTracker() = default;

  MOCK_METHOD(RansacResult,
              geometricOutlierRejection2d2d,
              (Frame*, Frame*, const gtsam::Pose3&),
              (override));
};

class ReturnHelper {
 public:
  ReturnHelper(const std::vector<RansacResult>& values) : vec_(values) {
    vec_iter_ = vec_.begin();
  }

  RansacResult getNext(Frame* /* ref */,
                       Frame* /* curr */,
                       const gtsam::Pose3& /* cam_lkf_Pose_cam_kf */) {
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

FrontendOutputPacketBase::Ptr makeOutput(
    Timestamp timestamp,
    double frame_period = 1.0e-9,
    FrontendType frontend_type = FrontendType::kStereoImu) {
  Frame fake_frame(1, timestamp, CameraParams(), cv::Mat());
  fake_frame.cam_param_.frame_rate_ = frame_period;
  if (frontend_type == FrontendType::kMonoImu) {
    return std::make_shared<MonoFrontendOutput>(
        false,
        StatusMonoMeasurementsPtr(nullptr),
        gtsam::Pose3(),
        fake_frame,
        ImuFrontend::PimPtr(nullptr),
        ImuAccGyrS(6, 1),
        cv::Mat(),
        DebugTrackerInfo());
  } else {
    StereoFrame fake_stereo(
        fake_frame.id_, fake_frame.timestamp_, fake_frame, fake_frame);
    return std::make_shared<StereoFrontendOutput>(
        false,
        StatusStereoMeasurementsPtr(nullptr),
        gtsam::Pose3(),
        gtsam::Pose3(),
        fake_stereo,
        ImuFrontend::PimPtr(nullptr),
        ImuAccGyrS(6, 1),
        cv::Mat(),
        DebugTrackerInfo());
  }
}

struct TestData {
  ImuParams params;
  std::vector<RansacResult> results;
  std::vector<FrontendOutputPacketBase::Ptr> outputs;
  std::vector<ImuStampS> imu_stamps;
  std::vector<ImuAccGyrS> imu_values;
  double expected_delay = 0.0;
};

void addFirstFrame(TestData& data) {
  ImuStampS times = ImuStampS::Zero(1, 1);
  ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);

  data.outputs.push_back(makeOutput(0));
  data.imu_stamps.push_back(times);
  data.imu_values.push_back(values);
}

struct SignalData {
  std::vector<Timestamp> vision_times;
  std::vector<double> vision_angles;
  std::vector<Timestamp> imu_times;
  std::vector<double> imu_angles;
};

SignalData generateSignal(int num_frames,
                          int num_imu_per,
                          int num_delay,
                          double rotation_scale,
                          double imu_period_s) {
  SignalData signal;
  // make flat signal at start
  if (num_delay < 0) {
    for (int i = 0; i < std::abs(num_delay); ++i) {
      signal.imu_times.push_back(i);
      signal.imu_angles.push_back(0.0);
    }
  }

  for (size_t i = 1; i <= num_frames; ++i) {
    double angle;  // rotation angle for image
    // make triangle wave
    if (i <= num_frames / 2) {
      angle = rotation_scale * i;
    } else {
      angle = rotation_scale * (num_frames - i);
    }

    signal.vision_times.push_back(i * num_imu_per);
    signal.vision_angles.push_back(angle);

    // set IMU angular velocity to be ratio of image rotation angle
    for (size_t k = 0; k < num_imu_per; ++k) {
      double imu_angle = (angle / num_imu_per) / imu_period_s;
      signal.imu_times.push_back(
          signal.imu_times.empty() ? 0 : signal.imu_times.back() + 1);
      signal.imu_angles.push_back(imu_angle);
    }
  }

  // make flat signal at end
  if (num_delay > 0) {
    for (int i = 0; i < num_delay; ++i) {
      signal.imu_angles.push_back(0.0);
      signal.imu_times.push_back(signal.imu_times.back() + 1);
    }
  } else {
    signal.imu_angles.push_back(0.0);
    signal.imu_times.push_back(signal.imu_times.back() + 1);
  }

  return signal;
}

TestData makeTestData(size_t num_frames = 10,
                      size_t num_imu_per = 5,
                      double rotation_scale = 0.1,
                      bool imu_rate = true,
                      int num_delay = 0) {
  TestData to_return;
  // set up some important data
  to_return.params.gyro_noise_density_ = 0.0;
  to_return.params.do_imu_rate_time_alignment_ = imu_rate;
  to_return.params.time_alignment_window_size_s_ =
      (imu_rate ? num_frames * num_imu_per : num_frames) * 1.0e-9;
  to_return.params.nominal_sampling_time_s_ = 1.0e-9;

  // correlation should ideally produce this
  if (imu_rate) {
    to_return.expected_delay =
        to_return.params.nominal_sampling_time_s_ * num_delay;
  } else {
    double imu_multiplier = static_cast<double>(num_imu_per);
    int delay_periods = std::round(num_delay / imu_multiplier);
    to_return.expected_delay = to_return.params.nominal_sampling_time_s_ *
                               imu_multiplier * delay_periods;
  }

  // add the first frame used to start the process
  addFirstFrame(to_return);

  SignalData signal = generateSignal(num_frames,
                                     num_imu_per,
                                     num_delay,
                                     rotation_scale,
                                     to_return.params.nominal_sampling_time_s_);
  for (size_t i = 0; i < signal.vision_angles.size(); ++i) {
    // this is actually a different axis, but the transform doesn't matter
    gtsam::Pose3 pose(gtsam::Rot3::Rz(signal.vision_angles[i]),
                      Eigen::Vector3d::Zero());
    to_return.results.emplace_back(TrackingStatus::VALID, pose);
    to_return.outputs.emplace_back(
        makeOutput(signal.vision_times[i], num_imu_per * 1.0e-9));
  }

  Timestamp first_imu_time =
      num_delay > 0 ? signal.imu_times[num_delay] : signal.imu_times.front();

  for (size_t i = 0; i < num_frames; ++i) {
    ImuStampS times = ImuStampS::Zero(1, num_imu_per + 1);
    ImuAccGyrS values = ImuAccGyrS::Zero(6, num_imu_per + 1);

    size_t offset =
        num_delay > 0 ? num_imu_per * i + num_delay : num_imu_per * i;
    for (size_t k = 0; k <= num_imu_per; ++k) {
      times(0, k) = signal.imu_times[k + offset] - first_imu_time;
      values(3, k) = signal.imu_angles[k + offset];
    }

    to_return.imu_stamps.emplace_back(times);
    to_return.imu_values.emplace_back(values);
  }

  return to_return;
}

class MockTimeAligner : public TimeAlignerBase {
 public:
  MockTimeAligner() {}
  ~MockTimeAligner() = default;

  using TimeAlignerBase::mergeImuData;

  void addPrevious(const ImuStampS& stamps, const ImuAccGyrS& values) {
    imu_stamp_cache_.push_back(stamps);
    imu_value_cache_.push_back(values);
  }

 protected:
  void doFirstFrameSetup(const Frame& frame) override {}

  TimeAlignerBase::Result attemptEstimation(
      const std::vector<Timestamp>& image_stamps,
      const gtsam::Pose3& T_ref_cur,
      const ImuStampS& imu_stamps,
      const ImuAccGyrS& imu_accgyrs,
      FrontendLogger* logger = nullptr) {
    return {true, 0.0};
  }
};

TEST(temporalCalibration, testMergeImuData) {
  MockTimeAligner aligner;

  ImuStampS first_stamps(1, 3);
  first_stamps << 1, 2, 3;
  ImuAccGyrS first_values(6, 3);
  first_values.block<1, 3>(2, 0) << 1, 2, 3;
  aligner.addPrevious(first_stamps, first_values);

  ImuStampS second_stamps(1, 2);
  second_stamps << 4, 5;
  ImuAccGyrS second_values(6, 2);
  second_values.block<1, 2>(2, 0) << 4, 5;
  aligner.addPrevious(second_stamps, second_values);

  ImuStampS third_stamps(1, 4);
  third_stamps << 6, 7, 8, 9;
  ImuAccGyrS third_values(6, 4);
  third_values.block<1, 4>(2, 0) << 6, 7, 8, 9;

  ImuStampS merged_stamps;
  ImuAccGyrS merged_values;
  aligner.mergeImuData(
      third_stamps, third_values, &merged_stamps, &merged_values);

  ASSERT_EQ(9, merged_stamps.cols());
  ASSERT_EQ(1, merged_stamps.rows());
  ASSERT_EQ(9, merged_values.cols());
  ASSERT_EQ(6, merged_values.rows());

  for (int i = 0; i < merged_stamps.cols(); ++i) {
    EXPECT_EQ(i + 1, merged_stamps(0, i));
    EXPECT_EQ(i + 1, merged_values(2, i));
  }
}

TEST(temporalCalibration, testGetMaxFromN) {
  { // test case 1: single max
    std::vector<double> values{1.0, 2.0, 3.0, 2.0, 1.0};
    for (size_t i = 0; i < values.size(); ++i) {
      const auto pos = CrossCorrTimeAligner::getMaxFromN(values, i);
      EXPECT_EQ(pos, 2u);
    }
  }

  { // test case 2: multiple max values (left-hand side)
    std::vector<double> values{3.0, 2.0, 2.0, 3.0, 2.0, 2.0, 1.0};
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 3), 3u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 2), 3u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 1), 0u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 0), 0u);
  }

  { // test case 2: multiple max values (right-hand side)
    std::vector<double> values{0.0, 2.0, 2.0, 3.0, 2.0, 2.0, 3.0};
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 3), 3u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 0), 3u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 6), 6u);
    EXPECT_EQ(CrossCorrTimeAligner::getMaxFromN(values, 5), 6u);
  }
}

TEST(temporalCalibration, testBadRansacStatus) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(TrackingStatus::INVALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::DISABLED, gtsam::Pose3());

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(2)
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.nominal_sampling_time_s_ = 1.0;
  CrossCorrTimeAligner aligner(params);

  FrontendOutputPacketBase::Ptr output = makeOutput(1);
  ImuStampS times(1, 0);
  ImuAccGyrS values(6, 0);

  // Set initial frame
  TimeAlignerBase::Result result =
      aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if RANSAC is invalid (first result)
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if 5pt RANSAC is disabled (second result)
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testEmptyImu) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(1)
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.nominal_sampling_time_s_ = 1.0;
  CrossCorrTimeAligner aligner(params);

  FrontendOutputPacketBase::Ptr output = makeOutput(1);
  ImuStampS times(1, 0);
  ImuAccGyrS values(6, 0);

  // Set initial frame
  TimeAlignerBase::Result result =
      aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_FALSE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);

  // Time alignment "succeeds" if the IMU isn't present between frames
  result = aligner.estimateTimeAlignment(tracker, *output, times, values);
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testLessThanWindow) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.nominal_sampling_time_s_ = 1.0e-9;
  params.time_alignment_window_size_s_ = 10.0e-9;
  CrossCorrTimeAligner aligner(params);

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::Ptr output = makeOutput(i);
    ImuStampS times(1, 1);
    times << (i == 0 ? 0 : i - 1);  // first IMU stamp is discarded
    ImuAccGyrS values(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testLessThanWindowFrameRate) {
  MockTracker tracker;

  std::vector<RansacResult> results;
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  ImuParams params;
  params.time_alignment_window_size_s_ = 10.0e-9;
  params.do_imu_rate_time_alignment_ = false;
  params.nominal_sampling_time_s_ = 1.0e-9;
  CrossCorrTimeAligner aligner(params);

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::Ptr output = makeOutput(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testLowVariance) {
  MockTracker tracker;

  ImuParams params;
  params.gyro_noise_density_ = 1.0;
  params.time_alignment_window_size_s_ = 3.0e-9;
  params.do_imu_rate_time_alignment_ = false;
  params.nominal_sampling_time_s_ = 1.0e-9;
  CrossCorrTimeAligner aligner(params);

  std::vector<RansacResult> results;
  for (size_t i = 0; i < 3; ++i) {
    results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  }

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::Ptr output = makeOutput(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);

    // We get false either from not having enough data or not having enough
    // variance
    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }
}

TEST(temporalCalibration, testEnoughVariance) {
  MockTracker tracker;

  ImuParams params;
  params.gyro_noise_density_ = 0.0;
  params.time_alignment_window_size_s_ = 3.0e-9;
  params.do_imu_rate_time_alignment_ = false;
  params.nominal_sampling_time_s_ = 1.0e-9;
  CrossCorrTimeAligner aligner(params);

  std::vector<RansacResult> results;
  for (size_t i = 0; i < 3; ++i) {
    results.emplace_back(TrackingStatus::VALID, gtsam::Pose3());
  }

  ReturnHelper helper(results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  for (size_t i = 0; i <= results.size(); ++i) {
    FrontendOutputPacketBase::Ptr output = makeOutput(i);
    ImuStampS times(1, 1);
    times << i;
    ImuAccGyrS values = ImuAccGyrS::Zero(6, 1);

    TimeAlignerBase::Result result =
        aligner.estimateTimeAlignment(tracker, *output, times, values);
    if (i < results.size()) {
      // We get false from not having enough data
      EXPECT_FALSE(result.valid);
      EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
    } else {
      EXPECT_TRUE(result.valid);
      // result needs to be within the min and max possible time
      EXPECT_GE(UtilsNumerical::NsecToSec(results.size() - 1),
                result.imu_time_shift);
      EXPECT_LE(UtilsNumerical::NsecToSec(-results.size() + 1),
                result.imu_time_shift);
    }
  }
}

TEST(temporalCalibration, testWellFormedNoDelay) {
  TestData data = makeTestData(10, 1, 0.1, true);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testWellFormedMultiImuNoDelayImuRate) {
  TestData data = makeTestData(10, 5, 0.1, true);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testWellFormedMultiImuNoDelayFrameRate) {
  TestData data = makeTestData(10, 5, 0.1, false);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
}

TEST(temporalCalibration, testNegDelayImuRate) {
  TestData data = makeTestData(10, 5, 0.1, true, -8);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

TEST(temporalCalibration, testPosDelayImuRate) {
  TestData data = makeTestData(10, 5, 0.1, true, 7);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

TEST(temporalCalibration, testNegDelayFrameRate) {
  TestData data = makeTestData(10, 5, 0.1, false, -8);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

TEST(temporalCalibration, testPosDelayFrameRate) {
  TestData data = makeTestData(10, 5, 0.1, false, 7);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

TEST(temporalCalibration, testPosDelayLowDisparity) {
  TestData data = makeTestData(10, 5, 0.1, true, 7);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  data.results[4].first =
      TrackingStatus::LOW_DISPARITY;  // force caching of IMU
  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

TEST(temporalCalibration, testPosDelayLowDisparityFrameRate) {
  TestData data = makeTestData(10, 5, 0.1, false, 7);

  MockTracker tracker;
  CrossCorrTimeAligner aligner(data.params);

  data.results[4].first =
      TrackingStatus::LOW_DISPARITY;  // force caching of IMU
  ReturnHelper helper(data.results);
  EXPECT_CALL(tracker, geometricOutlierRejection2d2d(NotNull(), NotNull(), _))
      .With(Args<0, 1>(Ne()))
      .Times(data.results.size())
      .WillRepeatedly(Invoke(&helper, &ReturnHelper::getNext));

  TimeAlignerBase::Result result;
  for (size_t i = 0; i < data.results.size(); ++i) {
    result = aligner.estimateTimeAlignment(
        tracker, *(data.outputs[i]), data.imu_stamps[i], data.imu_values[i]);
    EXPECT_FALSE(result.valid);
    EXPECT_DOUBLE_EQ(0.0, result.imu_time_shift);
  }

  result = aligner.estimateTimeAlignment(tracker,
                                         *(data.outputs.back()),
                                         data.imu_stamps.back(),
                                         data.imu_values.back());
  EXPECT_TRUE(result.valid);
  EXPECT_DOUBLE_EQ(data.expected_delay, result.imu_time_shift);
}

}  // namespace VIO
