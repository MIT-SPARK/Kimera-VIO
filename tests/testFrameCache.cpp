/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testFrameCache.cpp
 * @brief  Unit tests for FrameCache and LCD frame serialization
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include "kimera-vio/loopclosure/FrameCache.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

namespace cv {

bool operator==(const KeyPoint& lhs, const KeyPoint& rhs) {
  return lhs.angle == rhs.angle && lhs.class_id == rhs.class_id &&
         lhs.octave == rhs.octave && lhs.pt.x == rhs.pt.x &&
         lhs.pt.y == rhs.pt.y && lhs.response == rhs.response &&
         lhs.size == rhs.size;
}

}  // namespace cv

namespace VIO {

template <typename T>
std::string formatElem(const T& elem) {
  std::stringstream ss;
  ss << "(" << elem << ")";
  return ss.str();
}

template <>
std::string formatElem<Eigen::Vector3d>(const Eigen::Vector3d& elem) {
  std::stringstream ss;
  ss << "(" << elem.x() << ", " << elem.y() << ", " << elem.z() << ")";
  return ss.str();
}

template <>
std::string formatElem<cv::KeyPoint>(const cv::KeyPoint& pt) {
  std::stringstream ss;
  ss << "{";
  ss << "x: " << pt.pt.x << ", y: " << pt.pt.y << ", angle: " << pt.angle
     << ", octave: " << pt.octave << ", class_id: " << pt.class_id
     << ", response: " << pt.response << ", size: " << pt.size;
  ss << "}";
  return ss.str();
}

template <>
std::string formatElem<StatusKeypointCV>(const StatusKeypointCV& pt) {
  std::stringstream ss;
  ss << "{status: ";
  switch (pt.first) {
    case KeypointStatus::VALID:
      ss << "VALID";
      break;
    case KeypointStatus::NO_LEFT_RECT:
      ss << "NO_LEFT_RECT";
      break;
    case KeypointStatus::NO_RIGHT_RECT:
      ss << "NO_RIGHT_RECT";
      break;
    case KeypointStatus::NO_DEPTH:
      ss << "NO_DEPTH";
      break;
    case KeypointStatus::FAILED_ARUN:
      ss << "FAILED_ARUN";
      break;
    default:
      ss << "UNKNOWN";
      break;
  }

  ss << ", point: (" << pt.second.x << ", " << pt.second.y << ")}";
  return ss.str();
}

template <typename T, typename Alloc>
std::string printVec(const std::vector<T, Alloc>& vec) {
  std::stringstream ss;
  ss << "[";
  auto iter = vec.begin();
  while (iter != vec.end()) {
    ss << formatElem(*iter);
    ++iter;
    if (iter != vec.end()) {
      ss << ", ";
    }
  }
  ss << "]";
  return ss.str();
}

std::ostream& operator<<(std::ostream& out, const LCDFrame& frame) {
  out << "frame " << frame.id_ << " (kf: " << frame.id_kf_ << ") @ "
      << frame.timestamp_ << " [ns]:\n";
  out << "  - keypoints: " << printVec(frame.keypoints_) << "\n";
  out << "  - landmarks: " << printVec(frame.keypoints_3d_) << "\n";
  out << "  - bearing_vectors: " << printVec(frame.bearing_vectors_) << "\n";
  return out;
}

std::ostream& operator<<(std::ostream& out, const StereoLCDFrame& frame) {
  out << static_cast<const LCDFrame&>(frame);
  out << "  - left_keypoints_rect: "
      << printVec(frame.left_keypoints_rectified_) << "\n";
  out << "  - right_keypoints_rect: "
      << printVec(frame.right_keypoints_rectified_) << "\n";
  return out;
}

bool operator==(const LCDFrame& lhs, const LCDFrame& rhs) {
  const auto valid = lhs.timestamp_ == rhs.timestamp_ && lhs.id_ == rhs.id_ &&
                     lhs.id_kf_ == rhs.id_kf_ &&
                     lhs.keypoints_ == rhs.keypoints_ &&
                     lhs.keypoints_3d_ == rhs.keypoints_3d_ &&
                     lhs.bearing_vectors_ == rhs.bearing_vectors_;

  if (!valid) {
    return false;
  }

  if (!UtilsOpenCV::compareCvMatsUpToTol(lhs.descriptors_mat_,
                                         rhs.descriptors_mat_)) {
    return false;
  }

  if (lhs.descriptors_vec_.size() != rhs.descriptors_vec_.size()) {
    return false;
  }

  for (size_t i = 0; i < lhs.descriptors_vec_.size(); ++i) {
    if (!UtilsOpenCV::compareCvMatsUpToTol(lhs.descriptors_vec_[i],
                                           rhs.descriptors_vec_[i])) {
      return false;
    }
  }

  return true;
}

bool operator==(const StereoLCDFrame& lhs, const StereoLCDFrame& rhs) {
  if (!(static_cast<const LCDFrame&>(lhs) ==
        static_cast<const LCDFrame&>(rhs))) {
    return false;
  }

  return lhs.left_keypoints_rectified_ == rhs.left_keypoints_rectified_ &&
         lhs.right_keypoints_rectified_ == rhs.right_keypoints_rectified_;
}

TEST(testFrameCache, EmptyFrameRoundTripCorrect) {
  LCDFrame frame;

  std::ostringstream s_out;
  frame.save(s_out);

  std::istringstream s_in(s_out.str());
  auto result = LCDFrame::load(s_in);
  ASSERT_TRUE(result);
  EXPECT_EQ(frame, *result);
}

TEST(testFrameCache, NonEmptyFrameRoundTripCorrect) {
  std::vector<cv::KeyPoint> keypoints{{4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9, 10}};
  Landmarks landmarks{{11.0, 12.0, 13.0}, {14.0, 15.0, 16.0}};
  BearingVectors bearings{
      {25.0, 26.0, 27.0}, {28.0, 29.0, 30.0}, {31.0, 32.0, 33.0}};

  OrbDescriptorVec vec{cv::Mat(17, 18, CV_64FC1),
                       cv::Mat(19, 20, CV_32SC1),
                       cv::Mat(21, 22, CV_8UC3)};
  cv::randu(vec[0], -10.0, 10.0);
  cv::randu(vec[1], -10, 10);
  cv::randu(vec[2], 0, 255);

  cv::Mat descriptors(23, 24, CV_16SC2);
  cv::randu(descriptors, 0, 50);

  LCDFrame frame(1, 2, 3, keypoints, landmarks, vec, descriptors, bearings);

  std::ostringstream s_out;
  frame.save(s_out);

  std::istringstream s_in(s_out.str());
  auto result = LCDFrame::load(s_in);
  ASSERT_TRUE(result);
  EXPECT_EQ(frame, *result);
}

TEST(testFrameCache, EmptyStereoFrameRoundTripCorrect) {
  StereoLCDFrame frame;

  std::ostringstream s_out;
  frame.save(s_out);

  std::istringstream s_in(s_out.str());
  auto result = LCDFrame::load(s_in);
  ASSERT_TRUE(result);
  EXPECT_EQ(frame, *result);
}

TEST(testFrameCache, NonEmptyStereoFrameRoundTripCorrect) {
  std::vector<cv::KeyPoint> keypoints{{4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9, 10}};
  Landmarks landmarks{{11.0, 12.0, 13.0}, {14.0, 15.0, 16.0}};
  BearingVectors bearings{
      {25.0, 26.0, 27.0}, {28.0, 29.0, 30.0}, {31.0, 32.0, 33.0}};

  OrbDescriptorVec vec{cv::Mat(17, 18, CV_64FC1),
                       cv::Mat(19, 20, CV_32SC1),
                       cv::Mat(21, 22, CV_8UC3)};
  cv::randu(vec[0], -10.0, 10.0);
  cv::randu(vec[1], -10, 10);
  cv::randu(vec[2], 0, 255);

  cv::Mat descriptors(23, 24, CV_16SC2);
  cv::randu(descriptors, 0, 50);

  StatusKeypointsCV left_keypoints{
      std::make_pair(KeypointStatus::NO_LEFT_RECT, cv::Point2f(5.0f, 6.0f))};
  StatusKeypointsCV right_keypoints{
      std::make_pair(KeypointStatus::NO_RIGHT_RECT, cv::Point2f(6.0f, 7.0f)),
      std::make_pair(KeypointStatus::NO_DEPTH, cv::Point2f(9.0f, 10.0f))};

  StereoLCDFrame frame(1,
                       2,
                       3,
                       keypoints,
                       landmarks,
                       vec,
                       descriptors,
                       bearings,
                       left_keypoints,
                       right_keypoints);

  std::ostringstream s_out;
  frame.save(s_out);

  std::istringstream s_in(s_out.str());
  auto result = LCDFrame::load(s_in);
  ASSERT_TRUE(result);
  EXPECT_EQ(frame, *result);
}

struct FrameCacheTestConfig {
  FrameCacheConfig config;
  size_t num_frames;
  std::vector<std::pair<size_t, bool>> access_order;
};

struct FrameCacheFixture : public testing::TestWithParam<FrameCacheTestConfig> {
  FrameCacheFixture() {}
  virtual ~FrameCacheFixture() {}

  void SetUp() override {}
};

TEST_P(FrameCacheFixture, AccessCorrect) {
  const auto info = GetParam();
  FrameCache cache(info.config);
  for (size_t i = 0; i < info.num_frames; ++i) {
    cache.addFrame(std::make_shared<LCDFrame>());
  }

  for (auto&& [id, valid] : info.access_order) {
    const auto frame = cache.getFrame(id);
    if (!valid) {
      EXPECT_FALSE(frame);
      continue;
    }

    ASSERT_TRUE(frame);
    EXPECT_EQ(frame->id_, id);
  }
}

std::vector<std::pair<size_t, bool>> getAccessAllOrder(size_t total) {
  std::vector<std::pair<size_t, bool>> order;
  for (size_t i = 0; i < total; ++i) {
    order.push_back({i, true});
  }

  return order;
}

const FrameCacheTestConfig test_cases[] = {
    {{}, 10, getAccessAllOrder(10)},
    {{10, "/tmp", ".kimera_lcd_frames", 2}, 10, getAccessAllOrder(10)},
    {{}, 10, {{10, false}, {11, false}, {12, false}}},
    {{10, "/tmp", ".kimera_lcd_frames", 2}, 10, {{10, false}, {11, false}}},
};

INSTANTIATE_TEST_SUITE_P(CacheAccessCorrect,
                         FrameCacheFixture,
                         testing::ValuesIn(test_cases));

}  // namespace VIO
