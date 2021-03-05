/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoImuPipeline.cpp
 * @brief  Implements StereoVIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/StereoImuPipeline.h"

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/backend/VioBackendFactory.h"
#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/VisionImuFrontendFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

// TODO(marcus): clean this and put things in the base ctor
StereoImuPipeline::StereoImuPipeline(const VioParams& params,
                               Visualizer3D::UniquePtr&& visualizer,
                               DisplayBase::UniquePtr&& displayer)
    : Pipeline(params),
      stereo_camera_(nullptr) {
  //! Create Stereo Camera
  CHECK_EQ(params.camera_params_.size(), 2u) << "Need two cameras for StereoImuPipeline.";
  stereo_camera_ = std::make_shared<StereoCamera>(
      params.camera_params_.at(0),
      params.camera_params_.at(1));

  //! Create DataProvider
  data_provider_module_ = VIO::make_unique<StereoDataProviderModule>(
      &frontend_input_queue_,
      "Stereo Data Provider",
      parallel_run_,
      // TODO(Toni): these params should not be sent...
      params.frontend_params_.stereo_matching_params_);

  data_provider_module_->registerVioPipelineCallback(
      std::bind(&StereoImuPipeline::spinOnce, this, std::placeholders::_1));

  //! Create Frontend
  vio_frontend_module_ = VIO::make_unique<VisionImuFrontendModule>(
      &frontend_input_queue_,
      parallel_run_,
      VisionImuFrontendFactory::createFrontend(
          params.frontend_type_,
          params.imu_params_,
          gtsam::imuBias::ConstantBias(),
          params.frontend_params_,
          stereo_camera_,
          FLAGS_visualize ? &display_input_queue_ : nullptr,
          FLAGS_log_output));
  auto& backend_input_queue = backend_input_queue_;  //! for the lambda below
  vio_frontend_module_->registerOutputCallback([&backend_input_queue](
      const FrontendOutputPacketBase::Ptr& output) {
    StereoFrontendOutput::Ptr converted_output =
        VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(output);

    if (converted_output && converted_output->is_keyframe_) {
      //! Only push to Backend input queue if it is a keyframe!
      backend_input_queue.push(VIO::make_unique<BackendInput>(
          converted_output->stereo_frame_lkf_.timestamp_,
          converted_output->status_stereo_measurements_,
          converted_output->tracker_status_,
          converted_output->pim_,
          converted_output->imu_acc_gyrs_,
          converted_output->relative_pose_body_stereo_));
    } else {
      VLOG(5) << "Frontend did not output a keyframe, skipping Backend input.";
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
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackendModule>(
      &backend_input_queue_,
      parallel_run_,
      BackendFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          stereo_camera_->getBodyPoseLeftCamRect(),
          stereo_camera_->getStereoCalib(),
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output));
  vio_backend_module_->registerOnFailureCallback(
      std::bind(&StereoImuPipeline::signalBackendFailure, this));
  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  if (static_cast<VisualizationType>(FLAGS_viz_type) ==
      VisualizationType::kMesh2dTo3dSparse) {
    mesher_module_ = VIO::make_unique<MesherModule>(
        parallel_run_,
        MesherFactory::createMesher(
            MesherType::PROJECTIVE,
            MesherParams(stereo_camera_->getBodyPoseLeftCamRect(),
                         params.camera_params_.at(0u).image_size_)));
    //! Register input callbacks
    vio_backend_module_->registerOutputCallback(
        std::bind(&MesherModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(mesher_module_.get())),
                  std::placeholders::_1));

    auto& mesher_module = mesher_module_;
    vio_frontend_module_->registerOutputCallback(
        [&mesher_module](const FrontendOutputPacketBase::Ptr& output) {
          StereoFrontendOutput::Ptr converted_output =
              VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(output);
          CHECK_NOTNULL(mesher_module.get())
              ->fillFrontendQueue(converted_output);
        });
  }

  if (FLAGS_use_lcd) {
    lcd_module_ = VIO::make_unique<LcdModule>(
        parallel_run_,
        LcdFactory::createLcd(LoopClosureDetectorType::BoW,
                              params.lcd_params_,
                              stereo_camera_,
                              params.frontend_params_.stereo_matching_params_,
                              FLAGS_log_output));
    //! Register input callbacks
    vio_backend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));

    auto& lcd_module = lcd_module_;
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
                         static_cast<VisualizationType>(FLAGS_viz_type),
                         static_cast<BackendType>(params.backend_type_)));
    //! Register input callbacks
    vio_backend_module_->registerOutputCallback(
        std::bind(&VisualizerModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));

    auto& visualizer_module = visualizer_module_;
    vio_frontend_module_->registerOutputCallback(
        [&visualizer_module](const FrontendOutputPacketBase::Ptr& output) {
          StereoFrontendOutput::Ptr converted_output =
              VIO::safeCast<FrontendOutputPacketBase, StereoFrontendOutput>(output);
          CHECK_NOTNULL(visualizer_module.get())
              ->fillFrontendQueue(converted_output);
        });

    if (mesher_module_) {
      mesher_module_->registerOutputCallback(
          std::bind(&VisualizerModule::fillMesherQueue,
                    std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                    std::placeholders::_1));
    }

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
                        std::bind(&StereoImuPipeline::shutdown, this)));
  }

  // All modules are ready, launch threads! If the parallel_run flag is set to
  // false this will not do anything.
  launchThreads();
}

}  // namespace VIO
