#include "kimera-vio/playground/EurocPlayground.h"
#include "kimera-vio/mesh/MeshUtils.h"
#include "kimera-vio/visualizer/OpenCvDisplay.h"
#include "kimera-vio/visualizer/OpenCvDisplayParams.h"
#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

EurocPlayground::EurocPlayground(const std::string& dataset_path,
                                 const std::string& params_path,
                                 const int& initial_k,
                                 const int& final_k,
                                 const size_t& subsample_n)
    : vio_params_(params_path),
      visualizer_3d_(nullptr),
      stereo_camera_(nullptr),
      stereo_matcher_(nullptr),
      display_module_(nullptr),
      display_input_queue_("display_input_queue"),
      dataset_path_(dataset_path),
      subsample_n(subsample_n),
      feature_detector_(nullptr),
      euroc_data_provider_(nullptr),
      imu_data_(),
      left_frame_queue_("left_frame_queue"),
      right_frame_queue_("right_frame_queue") {
  // Set sequential mode
  vio_params_.parallel_run_ = false;

  // Create euroc data parser
  euroc_data_provider_ = VIO::make_unique<EurocDataProvider>(
      dataset_path, initial_k, final_k, vio_params_);

  // Register Callbacks
  euroc_data_provider_->registerImuSingleCallback(
      std::bind(&EurocPlayground::fillImuQueue, this, std::placeholders::_1));
  euroc_data_provider_->registerLeftFrameCallback(std::bind(
      &EurocPlayground::fillLeftFrameQueue, this, std::placeholders::_1));
  euroc_data_provider_->registerRightFrameCallback(std::bind(
      &EurocPlayground::fillRightFrameQueue, this, std::placeholders::_1));

  // Parse Euroc dataset.
  // Since we run in sequential mode, we need to spin it till it finishes.
  while (euroc_data_provider_->spin()) {
  };  // Fill queues.

  // Create 3D visualizer
  VisualizationType viz_type = VisualizationType::kPointcloud;
  BackendType backend_type = BackendType::kStereoImu;
  visualizer_3d_ = VIO::make_unique<OpenCvVisualizer3D>(viz_type, backend_type);

  // Create Displayer
  CHECK(vio_params_.display_params_);
  OpenCv3dDisplayParams modified_display_params =
      VIO::safeCast<DisplayParams, OpenCv3dDisplayParams>(
          *vio_params_.display_params_);
  modified_display_params.hold_3d_display_ = true;
  DisplayParams::Ptr new_display_params =
      std::make_shared<OpenCv3dDisplayParams>(modified_display_params);
  display_module_ = VIO::make_unique<DisplayModule>(
      &display_input_queue_,
      nullptr,
      vio_params_.parallel_run_,
      VIO::make_unique<OpenCv3dDisplay>(new_display_params, nullptr));

  // Create Feature detector
  FeatureDetectorParams feature_detector_params;
  feature_detector_params.feature_detector_type_ = FeatureDetectorType::ORB;
  feature_detector_ =
      VIO::make_unique<FeatureDetector>(feature_detector_params);

  // Create Stereo Camera
  stereo_camera_ = VIO::make_unique<StereoCamera>(
      vio_params_.camera_params_.at(0),
      vio_params_.camera_params_.at(1));
  stereo_matcher_ = VIO::make_unique<StereoMatcher>(
      stereo_camera_, vio_params_.frontend_params_.stereo_matching_params_);
}

void EurocPlayground::visualizeGtData(const bool& viz_traj,
                                      const bool& viz_img_in_frustum,
                                      const bool& viz_pointcloud) {
  VisualizerOutput::UniquePtr output = VIO::make_unique<VisualizerOutput>();
  output->visualization_type_ = VisualizationType::kPointcloud;

  // Draw the global frame of reference
  visualizer_3d_->visualizeGlobalFrameOfReference(&output->widgets_);

  if (viz_traj) {
    CHECK_GT(vio_params_.camera_params_.size(), 0);
    const gtsam::Pose3& body_Pose_cam =
        vio_params_.camera_params_.at(0).body_Pose_cam_;
    LOG_IF(ERROR, euroc_data_provider_->gt_data_.map_to_gt_.size() == 0)
        << "Empty ground-truth trajectory.";
    for (const auto& kv : euroc_data_provider_->gt_data_.map_to_gt_) {
      const VioNavState& state = kv.second;
      const cv::Affine3d& left_cam_pose = UtilsOpenCV::gtsamPose3ToCvAffine3d(
          state.pose_.compose(body_Pose_cam));
      visualizer_3d_->addPoseToTrajectory(left_cam_pose);
    }
    visualizer_3d_->visualizeTrajectory3D(&output->widgets_);
  }

  if (viz_img_in_frustum) {
    CHECK_GT(vio_params_.camera_params_.size(), 0);
    const auto& K = vio_params_.camera_params_.at(0).K_;
    Frame::UniquePtr left_frame = nullptr;
    Frame::UniquePtr right_frame = nullptr;
    size_t color_counter = 0u;
    while (left_frame_queue_.pop(left_frame) &&
           right_frame_queue_.pop(right_frame)) {
      CHECK(left_frame);
      CHECK(right_frame);
      CHECK_EQ(left_frame->timestamp_, right_frame->timestamp_);
      StereoFrame stereo_frame(
          left_frame->id_,
          left_frame->timestamp_,
          *left_frame,
          *right_frame);

      CHECK(stereo_camera_);
      if ((left_frame->id_ % subsample_n) == 0u) {
        // Add frame to frustum
        const cv::Affine3d& left_cam_rect_pose =
            UtilsOpenCV::gtsamPose3ToCvAffine3d(
                euroc_data_provider_->getGroundTruthPose(left_frame->timestamp_)
                    .compose(stereo_camera_->getBodyPoseLeftCamRect()));
        const cv::Affine3d& right_cam_rect_pose =
            UtilsOpenCV::gtsamPose3ToCvAffine3d(
                euroc_data_provider_
                    ->getGroundTruthPose(right_frame->timestamp_)
                    .compose(stereo_camera_->getBodyPoseRightCamRect()));

        cv::Mat smaller_img;
        static constexpr double kScaleFactor = 0.1;
        cv::resize(left_frame->img_,
                   smaller_img,
                   cv::Size(),
                   kScaleFactor,
                   kScaleFactor);
        cv::Mat resize_K = cv::Mat::eye(3, 3, CV_64FC1);
        resize_K.at<double>(0, 0) = kScaleFactor;
        resize_K.at<double>(1, 1) = kScaleFactor;
        cv::Mat resized_K = resize_K * K;
        visualizer_3d_->visualizePoseWithImgInFrustum(
            smaller_img,
            left_cam_rect_pose,
            &output->widgets_,
            "Camera id: " + std::to_string(left_frame->id_),
            resized_K);

        // Compute depth map just to see.
        cv::Mat disp_img =
            cv::Mat(left_frame->img_.rows, left_frame->img_.cols, CV_32F);
        CHECK(stereo_frame.isRectified());
        stereo_camera_->undistortRectifyStereoFrame(&stereo_frame);
        CHECK(stereo_matcher_);
        stereo_matcher_->denseStereoReconstruction(
            stereo_frame.getLeftImgRectified(),
            stereo_frame.getRightImgRectified(),
            &disp_img);
        cv::Mat disp_viz_img;
        UtilsOpenCV::getDisparityVis(disp_img, disp_viz_img, 1.0);
        cv::imshow("Left Image Rectified", stereo_frame.getLeftImgRectified());
        cv::imshow("Right Image Rectified",
                   stereo_frame.getRightImgRectified());
        cv::imshow("Disparity Image", disp_viz_img);

        // Check
        // https://github.com/opencv/opencv/blob/master/samples/cpp/stereo_match.cpp
        cv::Mat floatDisp;
        disp_img.convertTo(floatDisp, CV_32F, 1.0f / 16.0f);
        // disp_img.convertTo(floatDisp, CV_32F, 1.0f / 16.0f);
        disp_img = floatDisp;

        // I think this is the perfect container for mesh optimization
        // since it encodes in (u, v) => (x, y, z).
        // Maybe ideally it should be (u, v) => 1/z
        cv::Mat_<cv::Point3f> depth_map;
        // Need to move all points according to pose of stereo camera!
        stereo_camera_->backProjectDisparityTo3D(disp_img, &depth_map);
        CHECK_EQ(depth_map.type(), CV_32FC3);
        // Would that work? interpret xyz as rgb?
        cv::imshow("Depth Image", depth_map);
        cv::waitKey(1);

        // Store depth maps for mesh optimization.
        MeshPacket mesh_packet_;
        // Shouldn't we send rectified camera pose?
        mesh_packet_.world_pose_body_ =
            euroc_data_provider_->getGroundTruthPose(left_frame->timestamp_);
        mesh_packet_.left_cam_rect_pose_ = left_cam_rect_pose;
        // Shouldn't we send rectified images?
        mesh_packet_.left_image_rect_ =
            stereo_frame.getLeftImgRectified().clone();
        mesh_packet_.right_cam_rect_pose_ = right_cam_rect_pose;
        mesh_packet_.right_image_rect =
            stereo_frame.getRightImgRectified().clone();

        LOG(INFO) << "Converting depth to pcl.";
        // Reshape as a list of 3D points, same channels,
        // 1 row, N cols (doesn't work cause we need to clean depth...)
        // cv::Mat_<cv::Point3f> depth_reshaped = depth.reshape(0, 1);

        // Depth image contains INFs. We have to remove them:
        CHECK_EQ(left_frame->img_.type(), CV_8UC1);  // for color
        cv::Mat_<cv::Point3f> valid_depth = cv::Mat(1, 0, CV_32FC3);
        cv::Mat valid_colors = cv::Mat(1, 0, CV_8UC3, cv::viz::Color::red());
        for (int32_t v = 0; v < depth_map.rows; ++v) {
          for (int32_t u = 0; u < depth_map.cols; ++u) {
            const cv::Point3f& xyz = depth_map(v, u);
            if (isValidPoint(xyz)) {
              valid_depth.push_back(xyz);
              const auto& grey_value =
                  mesh_packet_.left_image_rect_.at<uint8_t>(v, u);
              // We recolor the depth map for disambiguation of original
              // viewpoint
              valid_colors.push_back(
                  cv::Vec3b(color_counter == 0u ? grey_value : 0u,
                            color_counter == 1u ? grey_value : 0u,
                            color_counter == 2u ? grey_value : 0u));
            }
          }
        }
        color_counter++;
        mesh_packet_.depth_map_ = depth_map;
        mesh_packets_[left_frame->timestamp_] = mesh_packet_;

        LOG(INFO) << "Send pcl to viz.";
        visualizer_3d_->visualizePointCloud(
            valid_depth, &output->widgets_, left_cam_rect_pose, valid_colors);
      }
    }
  }

  if (viz_pointcloud) {
    static const std::string pcl_ply_filename =
        dataset_path_ + "/mav0/pointcloud0/data.ply";
    visualizer_3d_->visualizePlyMesh(pcl_ply_filename, &output->widgets_);
  }

  display_module_->spinOnce(std::move(output));
}

void EurocPlayground::projectVisibleLandmarksToCam(
    const StereoCamera& stereo_cam,
    const Landmarks& lmks) {
  KeypointsCV left_kpts, right_kpts;
  LOG(ERROR) << "Landmarks size: " << lmks.size();
  // stereo_cam.project(lmks, &left_kpts, &right_kpts);
  // Visualize keypoints (use drawPixelImg in visualizer from mesh opt).
}

//! Fill one IMU measurement only
void EurocPlayground::fillImuQueue(const ImuMeasurement& imu_measurement) {
  imu_data_.imu_buffer_.addMeasurement(imu_measurement.timestamp_,
                                       imu_measurement.acc_gyr_);
}

//! Callbacks to fill queues: they should be all lighting fast.
void EurocPlayground::fillLeftFrameQueue(Frame::UniquePtr left_frame) {
  CHECK(left_frame);
  left_frame_queue_.push(std::move(left_frame));
}

//! Callbacks to fill queues: they should be all lighting fast.
void EurocPlayground::fillRightFrameQueue(Frame::UniquePtr left_frame) {
  CHECK(left_frame);
  right_frame_queue_.push(std::move(left_frame));
}
}
