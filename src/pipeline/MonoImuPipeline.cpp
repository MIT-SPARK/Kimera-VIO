/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuPipeline.cpp
 * @brief  Implements MonoVIO pipeline workflow.
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/MonoImuPipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "kimera-vio/backend/VioBackendFactory.h"
#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontendFactory.h"
#include "kimera-vio/loopclosure/LcdFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

DECLARE_bool(do_coarse_imu_camera_temporal_sync);
DECLARE_bool(do_fine_imu_camera_temporal_sync);

namespace VIO {

MonoImuPipeline::MonoImuPipeline(
    const VioParams& params,
    Visualizer3D::UniquePtr&& visualizer,
    DisplayBase::UniquePtr&& displayer,
    PreloadedVocab::Ptr&& preloaded_vocab)
    : Pipeline(params), camera_(nullptr) {
  // CHECK_EQ(params.camera_params_.size(), 1u) << "Need one camera for
  // MonoImuPipeline.";
  camera_ = std::make_shared<Camera>(params.camera_params_.at(0));

  data_provider_module_ = VIO::make_unique<MonoDataProviderModule>(
      &frontend_input_queue_, "Mono Data Provider", parallel_run_);
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
      std::bind(&MonoImuPipeline::spinOnce, this, std::placeholders::_1));

  LOG_IF(FATAL, params.frontend_params_.use_stereo_tracking_)
      << "useStereoTracking is set to true, but this is a mono pipeline!";
  vio_frontend_module_ = VIO::make_unique<VisionImuFrontendModule>(
      &frontend_input_queue_,
      parallel_run_,
      VisionImuFrontendFactory::createFrontend(
          params.frontend_type_,
          params.imu_params_,
          gtsam::imuBias::ConstantBias(),
          params.frontend_params_,
          camera_,
          FLAGS_visualize ? &display_input_queue_ : nullptr,
          FLAGS_log_output,
          params.odom_params_));
  vio_frontend_module_->registerImuTimeShiftUpdateCallback(
      [&](double imu_time_shift_s) {
        data_provider_module_->setImuTimeShift(imu_time_shift_s);
      });

  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback(
      [&backend_input_queue](const FrontendOutputPacketBase::Ptr& output) {
        MonoFrontendOutput::Ptr converted_output =
            VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(output);
        if (converted_output->is_keyframe_) {
          //! Only push to Backend input queue if it is a keyframe!
          backend_input_queue.push(VIO::make_unique<BackendInput>(
              converted_output->frame_lkf_.timestamp_,
              converted_output->status_mono_measurements_,
              converted_output->pim_,
              converted_output->imu_acc_gyrs_,
              converted_output->body_lkf_OdomPose_body_kf_,
              converted_output->body_kf_world_OdomVel_body_kf_));
        } else {
          VLOG(5)
              << "Frontend did not output a keyframe, skipping Backend input.";
        }
      });

  //! Params for what the Backend outputs.
  // TODO(Toni): put this into Backend params.
  BackendOutputParams backend_output_params(
      static_cast<VisualizationType>(FLAGS_viz_type) !=
          VisualizationType::kNone,
      FLAGS_min_num_obs_for_mesher_points,
      FLAGS_visualize && FLAGS_visualize_lmk_type);

  //! Create Backend
  // TODO(marcus): get rid of fake stereocam
  LOG_IF(FATAL, params.backend_params_->addBetweenStereoFactors_)
      << "addBetweenStereoFactors is set to true, but this is a mono pipeline!";
  const gtsam::Cal3_S2& calib = camera_->getCalibration();
  StereoCalibPtr stereo_calib = boost::make_shared<gtsam::Cal3_S2Stereo>(
      calib.fx(),
      calib.fy(),
      calib.skew(),
      calib.px(),
      calib.py(),
      0.1);  // TODO(marcus): hardcoded baseline!
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackendModule>(
      &backend_input_queue_,
      parallel_run_,
      BackendFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          camera_->getBodyPoseCam(),
          stereo_calib,
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output,
          params.odom_params_));

  vio_backend_module_->registerOnFailureCallback(
      std::bind(&MonoImuPipeline::signalBackendFailure, this));

  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  vio_backend_module_->registerMapUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateMap,
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  // TOOD(marcus): enable use of mesher for mono pipeline
  // if (static_cast<VisualizationType>(FLAGS_viz_type) ==
  //     VisualizationType::kMesh2dTo3dSparse) {
  //   mesher_module_ = VIO::make_unique<MesherModule>(
  //       parallel_run_,
  //       MesherFactory::createMesher(
  //           MesherType::PROJECTIVE,
  //           MesherParams(camera_->getBodyPoseCam(),
  //                        params.camera_params_.at(0u).image_size_)));

  //   //! Register input callbacks
  //   vio_backend_module_->registerOutputCallback(
  //       std::bind(&MesherModule::fillBackendQueue,
  //                 std::ref(*CHECK_NOTNULL(mesher_module_.get())),
  //                 std::placeholders::_1));

  //   vio_frontend_module_->registerOutputCallback(
  //       std::bind(&MesherModule::fillFrontendQueue,
  //                 std::ref(*CHECK_NOTNULL(mesher_module_.get())),
  //                 std::placeholders::_1));
  // }

  if (FLAGS_use_lcd) {
    lcd_module_ = VIO::make_unique<LcdModule>(
        parallel_run_,
        LcdFactory::createLcd(LoopClosureDetectorType::BoW,
                              params.lcd_params_,
                              camera_->getCamParams(),
                              camera_->getBodyPoseCam(),
                              boost::none,
                              boost::none,
                              boost::none,
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
    visualizer_module_ = VIO::make_unique<VisualizerModule>(
        //! Send ouput of visualizer to the display_input_queue_
        &display_input_queue_,
        parallel_run_,
        FLAGS_use_lcd,
        // Use given visualizer if any
        visualizer ? std::move(visualizer)
                   : VisualizerFactory::createVisualizer(
                         VisualizerType::OpenCV,
                         // TODO(Toni): bundle these three params in
                         // VisualizerParams...
                         // NOTE: use kNone or kPointCloud for now because
                         //   mesher isn't enabled.
                         static_cast<VisualizationType>(FLAGS_viz_type),
                         static_cast<BackendType>(params.backend_type_)));

    //! Register input callbacks
    CHECK(vio_backend_module_);
    vio_backend_module_->registerOutputCallback(
        std::bind(&VisualizerModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));

    auto& visualizer_module = visualizer_module_;
    vio_frontend_module_->registerOutputCallback(
        [&visualizer_module](const FrontendOutputPacketBase::Ptr& output) {
          MonoFrontendOutput::Ptr converted_output =
              VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(
                  output);
          CHECK_NOTNULL(visualizer_module.get())
              ->fillFrontendQueue(converted_output);
        });

    // if (mesher_module_) {
    //   mesher_module_->registerOutputCallback(
    //       std::bind(&VisualizerModule::fillMesherQueue,
    //                 std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
    //                 std::placeholders::_1));
    // }

    if (lcd_module_) {
      lcd_module_->registerOutputCallback(
          std::bind(&VisualizerModule::fillLcdQueue,
                    std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                    std::placeholders::_1));
    }

    //! Actual displaying of visual data is done in the main thread.
    CHECK(params.display_params_);
    display_module_ = VIO::make_unique<DisplayModule>(
        &display_input_queue_,
        nullptr,
        parallel_run_,
        // Use given displayer if any
        displayer ? std::move(displayer)
                  : DisplayFactory::makeDisplay(
                        params.display_params_->display_type_,
                        params.display_params_,
                        std::bind(&MonoImuPipeline::shutdown, this)));
  }

  launchThreads();
}

}  // namespace VIO
