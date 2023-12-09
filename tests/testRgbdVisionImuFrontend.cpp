/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   testRgbdVisionImuFrontend.cpp
 * @brief  test RgbdisionImuFrontend (based on testStereoVisionImuFrontend)
 * @author Antoni Rosinol
 * @author Luca Carlone
 * @author Marcus Abate
 * @author Nathan Hughes
 */

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <random>
#include <string>

#include "kimera-vio/frontend/CameraParams.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend.h"
#include "kimera-vio/frontend/Tracker.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

DECLARE_string(test_data_path);
DECLARE_bool(display);

namespace VIO {

using Point2Vec = std::vector<gtsam::Point2>;

class RgbdVisionImuFrontendFixture : public ::testing::Test {
 public:
  RgbdVisionImuFrontendFixture()
      : rgbd_data_path(FLAGS_test_data_path + "/ForRgbd/") {
    initializeData();
  }

 protected:
  virtual void SetUp() {}
  virtual void TearDown() {}

  cv::Mat loadRgbImage(const std::string& name) {
    std::string image_path = rgbd_data_path + name;
    return UtilsOpenCV::ReadAndConvertToGrayScale(image_path);
  }

  Frame::Ptr loadRgbFrame(FrameId id,
                          Timestamp stamp,
                          const std::string& name) {
    return std::make_shared<Frame>(
        id, stamp, camera->getCamParams(), loadRgbImage(name));
  }

  DepthFrame::Ptr loadDepthFrame(FrameId id,
                                 Timestamp stamp,
                                 const std::string& name) {
    std::string image_path = rgbd_data_path + name;
    cv::Mat depth_img = cv::imread(image_path, cv::IMREAD_UNCHANGED);
    return std::make_shared<DepthFrame>(id, stamp, depth_img);
  }

  RgbdFrame::Ptr loadRgbdFrame(FrameId id,
                               Timestamp stamp,
                               const std::string& rgb_name,
                               const std::string& depth_name) {
    return std::make_shared<RgbdFrame>(id,
                                       stamp,
                                       *loadRgbFrame(id, stamp, rgb_name),
                                       *loadDepthFrame(id, stamp, depth_name));
  }

  void initializeData() {
    CameraParams left_cam;
    left_cam.parseYAML(rgbd_data_path + "/sensorLeft.yaml");
    camera.reset(new RgbdCamera(left_cam));

    // TODO(nathan) grab from rgbd frames?
    // Data for testing "geometricOutlierRejection2d2d"
    ref_frame = loadRgbFrame(id_ref, timestamp_ref, "left_img_0.png");
    cur_frame = loadRgbFrame(id_cur, timestamp_cur, "left_img_1.png");

    // TODO(nathan) fix depth extension
    ref_rgbd_frame = loadRgbdFrame(
        id_ref, timestamp_ref, "left_img_0.png", "depth_img_0.tiff");
    cur_rgbd_frame = loadRgbdFrame(
        id_ref, timestamp_ref, "left_img_1.png", "depth_img_1.tiff");

    // Imu Params
    imu_params.acc_random_walk_ = 1;
    imu_params.gyro_random_walk_ = 1;
    imu_params.acc_noise_density_ = 1;
    imu_params.gyro_noise_density_ = 1;
    imu_params.imu_integration_sigma_ = 1;

    // Set randomness!
    srand(0);
  }

  std::vector<double> loadDepth(const std::string& filepath) {
    std::vector<double> depth;
    std::ifstream fin(filepath);
    int num_corners;
    fin >> num_corners;
    depth.reserve(num_corners);
    for (int i = 0; i < num_corners; i++) {
      double d;
      fin >> d;
      // Convert points coordinates from MATLAB convention to c++ convention:
      depth.push_back(d);
    }
    return depth;
  }

  std::string rgbd_data_path;

  // Data
  std::shared_ptr<Frame> ref_frame;
  std::shared_ptr<Frame> cur_frame;
  std::shared_ptr<RgbdFrame> ref_rgbd_frame;
  std::shared_ptr<RgbdFrame> cur_rgbd_frame;
  const FrameId id_ref = 0u;
  const FrameId id_cur = 1u;
  const Timestamp timestamp_ref = 1000u;
  const Timestamp timestamp_cur = 2000u;

  RgbdCamera::Ptr camera;
  ImuParams imu_params;
};

TEST_F(RgbdVisionImuFrontendFixture, fillSmartStereoMeasurements) {
  RgbdVisionImuFrontend frontend(
      FrontendParams(), imu_params, ImuBias(), camera);

  // Parameters for the synthesis!
  const int num_valid = 12;
  const int num_landmark_invalid = 12;
  const int num_right_missing = 12;

  // Synthesize the input data!

  auto stereo_frame = ref_rgbd_frame->getStereoFrame();
  stereo_frame->setIsRectified(true);
  stereo_frame->setIsKeyframe(true);

  auto& left_frame = stereo_frame->left_frame_;

  // valid!
  for (int i = 0; i < num_valid; i++) {
    double uL = rand() % 800;
    double v = rand() % 600;
    left_frame.keypoints_.push_back(cv::Point2f(uL, v));
    left_frame.versors_.push_back(gtsam::Vector3(uL, v, 1.0).normalized());
    left_frame.landmarks_.push_back(i);
    left_frame.scores_.push_back(1.0);
    stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
  }

  // right keypoints invalid!
  for (int i = 0; i < num_right_missing; i++) {
    double uL = rand() % 800;
    double v = rand() % 600;
    left_frame.keypoints_.push_back(cv::Point2f(uL, v));
    left_frame.versors_.push_back(gtsam::Vector3(uL, v, 1.0).normalized());
    left_frame.landmarks_.push_back(i + num_valid);
    left_frame.scores_.push_back(1.0);
    stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::NO_RIGHT_RECT, cv::Point2f(uL, v)));
  }

  // landmark missing!
  for (int i = 0; i < num_landmark_invalid; i++) {
    double uL = rand() % 800;
    double v = rand() % 600;
    left_frame.keypoints_.push_back(cv::Point2f(uL, v));
    left_frame.versors_.push_back(gtsam::Vector3(uL, v, 1.0).normalized());
    left_frame.landmarks_.push_back(-1);
    left_frame.scores_.push_back(1.0);
    stereo_frame->left_keypoints_rectified_.push_back(
        StatusKeypointCV(KeypointStatus::VALID, cv::Point2f(uL, v)));
  }

  ref_rgbd_frame->fillStereoFrame(*camera, *stereo_frame);

  // Call the function!
  StereoMeasurements stereo_measurements;
  frontend.fillSmartStereoMeasurements(*stereo_frame, &stereo_measurements);

  // Verify the correctness of the results!
  EXPECT_EQ(stereo_measurements.size(), num_valid + num_right_missing);
  std::set<int> landmark_set;

  const auto& lkp_rect = stereo_frame->left_keypoints_rectified_;
  const auto& rkp_rect = stereo_frame->right_keypoints_rectified_;
  for (auto s : stereo_measurements) {
    // No order requirement for the entries in ssm.
    // To avoid searching for the location for a landmark, the data is
    // synthesized following a simple convention:
    //         landmark_[i] = i; for landmark_[i] != -1;
    const auto landmark_id = s.first;
    EXPECT_EQ(lkp_rect.at(landmark_id).second.x, s.second.uL());
    EXPECT_EQ(lkp_rect.at(landmark_id).second.y, s.second.v());
    if (rkp_rect.at(landmark_id).first == KeypointStatus::VALID) {
      EXPECT_EQ(rkp_rect.at(landmark_id).second.x, s.second.uR());
    } else {
      EXPECT_TRUE(std::isnan(s.second.uR()));
    }

    // Verify that there is no replicated entries in landmark_set.
    EXPECT_EQ(landmark_set.find(landmark_id), landmark_set.end());
    landmark_set.insert(landmark_id);
  }
}

}  // namespace VIO
