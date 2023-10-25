/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RgbdImuPipeline.cpp
 * @brief  Implements RgbdVIO pipeline workflow.
 * @author Nathan Hughes
 */

#include "kimera-vio/pipeline/RgbdImuPipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "kimera-vio/backend/VioBackendFactory.h"
#include "kimera-vio/dataprovider/RgbdDataProviderModule.h"
#include "kimera-vio/frontend/RgbdVisionImuFrontend.h"
#include "kimera-vio/loopclosure/LcdFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

DECLARE_bool(do_coarse_imu_camera_temporal_sync);
DECLARE_bool(do_fine_imu_camera_temporal_sync);

namespace VIO {

RgbdImuPipeline::RgbdImuPipeline(const VioParams& params,
                                 Visualizer3D::UniquePtr&& visualizer,
                                 DisplayBase::UniquePtr&& displayer,
                                 PreloadedVocab::Ptr&& preloaded_vocab)
    : Pipeline(params) {
  CHECK_GE(params.camera_params_.size(), 1u)
      << "Need at least one camera for RgbdImuPipeline.";
  camera_ = std::make_shared<RgbdCamera>(params.camera_params_.at(0));

  data_provider_module_ = std::make_unique<RgbdDataProviderModule>(
      &frontend_input_queue_, "Rgbd Data Provider", parallel_run_);

  if (FLAGS_do_coarse_imu_camera_temporal_sync) {
    data_provider_module_->doCoarseImuCameraTemporalSync();
  }
  if (!FLAGS_do_fine_imu_camera_temporal_sync) {
    if (FLAGS_do_coarse_imu_camera_temporal_sync) {
      LOG(WARNING) << "The manually provided IMU time shift will be applied on "
                      "top of whatever the coarse time alignment calculates. "
                      "This may or may not be what you want!";
    }
    data_provider_module_->setImuTimeShift(imu_params_.imu_time_shift_);
  }

  if (params.odom_params_) {
    data_provider_module_->setExternalOdometryTimeShift(
        params.odom_params_.value().time_shift_s_);
  }

  data_provider_module_->registerVioPipelineCallback(
      std::bind(&RgbdImuPipeline::spinOnce, this, std::placeholders::_1));

  LOG_IF(FATAL, !params.frontend_params_.use_stereo_tracking_)
      << "useStereoTracking is set to false, but is required for RGBD!";
  vio_frontend_module_ = std::make_unique<VisionImuFrontendModule>(
      &frontend_input_queue_,
      parallel_run_,
      std::make_unique<RgbdVisionImuFrontend>(
          params.frontend_params_,
          params.imu_params_,
          gtsam::imuBias::ConstantBias(),
          camera_,
          FLAGS_visualize ? &display_input_queue_ : nullptr,
          FLAGS_log_output,
          params.odom_params_));
  vio_frontend_module_->registerImuTimeShiftUpdateCallback(
      [&](double imu_time_shift_s) {
        data_provider_module_->setImuTimeShift(imu_time_shift_s);
      });

  // TODO(nathan) we can probably delete this and bind to this
  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue](const FrontendOutputPacketBase::Ptr& output) {
        auto converted_output =
            std::dynamic_pointer_cast<RgbdFrontendOutput>(output);
        CHECK(converted_output);
        if (converted_output->is_keyframe_) {
          backend_input_queue.push(std::make_unique<BackendInput>(
              converted_output->frame_lkf_.timestamp_,
              converted_output->status_stereo_measurements_,
              converted_output->pim_,
              converted_output->imu_acc_gyrs_,
              converted_output->body_lkf_OdomPose_body_kf_,
              converted_output->body_kf_world_OdomVel_body_kf_));
        } else {
          VLOG(5) << "Frontend did not output a keyframe, skipping Backend "
                     "input.";
        }
      });

  // TODO(Toni): put this into Backend params.
  BackendOutputParams backend_output_params(
      static_cast<VisualizationType>(FLAGS_viz_type) !=
          VisualizationType::kNone,
      FLAGS_min_num_obs_for_mesher_points,
      FLAGS_visualize && FLAGS_visualize_lmk_type);

  CHECK(backend_params_);
  vio_backend_module_ = std::make_unique<VioBackendModule>(
      &backend_input_queue_,
      parallel_run_,
      BackendFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          camera_->getBodyPoseCam(),
          camera_->getFakeStereoCalib(),
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output,
          params.odom_params_));
  vio_backend_module_->registerOnFailureCallback(
      std::bind(&RgbdImuPipeline::signalBackendFailure, this));
  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));
  vio_backend_module_->registerMapUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateMap,
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  if (static_cast<VisualizationType>(FLAGS_viz_type) ==
      VisualizationType::kMesh2dTo3dSparse) {
    mesher_module_ = std::make_unique<MesherModule>(
        parallel_run_,
        MesherFactory::createMesher(
            MesherType::PROJECTIVE,
            MesherParams(camera_->getBodyPoseCam(),
                         params.camera_params_[0].image_size_)));

    vio_backend_module_->registerOutputCallback(
        std::bind(&MesherModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(mesher_module_.get())),
                  std::placeholders::_1));

    auto& mesher_module = mesher_module_;
    vio_frontend_module_->registerOutputCallback(
        [&mesher_module](const FrontendOutputPacketBase::Ptr& base_output) {
          CHECK(base_output->frontend_type_ == FrontendType::kRgbdImu)
              << "Frontend output packet found that isn't an RGBD packet!";

          RgbdFrontendOutput::Ptr output =
              std::dynamic_pointer_cast<RgbdFrontendOutput>(base_output);
          CHECK(output);

          // mesher only takes stereo frame as input from frontend, so we pick
          // a minimal set of fields to fill
          CHECK_NOTNULL(mesher_module.get())
              ->fillFrontendQueue(std::make_shared<StereoFrontendOutput>(
                  output->is_keyframe_,
                  nullptr,
                  gtsam::Pose3(),
                  gtsam::Pose3(),
                  output->frame_lkf_,
                  nullptr,
                  output->imu_acc_gyrs_,
                  cv::Mat(),
                  output->debug_tracker_info_));
        });
  }

  // TODO(nathan) LCD
  if (FLAGS_use_lcd) {
    lcd_module_ = std::make_unique<LcdModule>(
        parallel_run_,
        LcdFactory::createLcd(LoopClosureDetectorType::BoW,
                              params.lcd_params_,
                              camera_->getCamParams(),
                              camera_->getBodyPoseCam(),
                              std::nullopt,
                              std::nullopt,
                              camera_,
                              FLAGS_log_output,
                              std::move(preloaded_vocab)));
    //! Register input callbacks
    vio_backend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
    vio_frontend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
  }

  if (FLAGS_visualize) {
    visualizer_module_ = std::make_unique<VisualizerModule>(
        &display_input_queue_,
        parallel_run_,
        FLAGS_use_lcd,
        visualizer ? std::move(visualizer)
                   : VisualizerFactory::createVisualizer(
                         VisualizerType::OpenCV,
                         static_cast<VisualizationType>(FLAGS_viz_type),
                         static_cast<BackendType>(params.backend_type_)));

    vio_backend_module_->registerOutputCallback(
        std::bind(&VisualizerModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));

    auto& visualizer_module = visualizer_module_;
    vio_frontend_module_->registerOutputCallback(
        [&visualizer_module](const FrontendOutputPacketBase::Ptr& output) {
          CHECK_NOTNULL(visualizer_module.get())->fillFrontendQueue(output);
        });

    if (mesher_module_) {
      mesher_module_->registerOutputCallback(
          std::bind(&VisualizerModule::fillMesherQueue,
                    std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                    std::placeholders::_1));
    } else {
      visualizer_module->disableMesherQueue();
    }

    //! Actual displaying of visual data is done in the main thread.
    CHECK(params.display_params_);
    display_module_ = std::make_unique<DisplayModule>(
        &display_input_queue_,
        nullptr,
        parallel_run_,
        // Use given displayer if any
        displayer ? std::move(displayer)
                  : DisplayFactory::makeDisplay(
                        params.display_params_->display_type_,
                        params.display_params_,
                        std::bind(&RgbdImuPipeline::shutdown, this)));
  }

  launchThreads();
}

void RgbdImuPipeline::fillDepthFrameQueue(DepthFrame::UniquePtr frame) {
  CHECK(data_provider_module_);
  CHECK(frame);
  auto provider =
      dynamic_cast<RgbdDataProviderModule*>(data_provider_module_.get());
  CHECK_NOTNULL(provider)->fillDepthFrameQueue(std::move(frame));
}

}  // namespace VIO
