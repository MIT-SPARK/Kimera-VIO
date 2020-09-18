/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   StereoPipeline.cpp
 * @brief  Implements StereoVIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/StereoPipeline.h"

#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/backend/VioBackEndFactory.h"
#include "kimera-vio/dataprovider/StereoDataProviderModule.h"
#include "kimera-vio/frontend/VisionFrontEndFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3D.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

// TODO(marcus): clean this and put things in the base ctor
StereoPipeline::StereoPipeline(const VioParams& params,
                               Visualizer3D::UniquePtr&& visualizer,
                               DisplayBase::UniquePtr&& displayer)
    : Pipeline(params),
      stereo_camera_(nullptr) {
  //! Create Stereo Camera
  CHECK_EQ(params.camera_params_.size(), 2u) << "Need two cameras for StereoPipeline.";
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
      std::bind(&StereoPipeline::spinOnce, this, std::placeholders::_1));

  //! Create frontend
  vio_frontend_module_ = VIO::make_unique<StereoVisionFrontEndModule>(
      &frontend_input_queue_,
      parallel_run_,
      VisionFrontEndFactory::createFrontend(
          params.frontend_type_,
          params.imu_params_,
          gtsam::imuBias::ConstantBias(),
          params.frontend_params_,
          stereo_camera_,
          FLAGS_visualize ? &display_input_queue_ : nullptr,
          FLAGS_log_output));
  auto& backend_input_queue = backend_input_queue_;  //! for the lambda below
  vio_frontend_module_->registerOutputCallback([&backend_input_queue](
      const StereoFrontendOutput::Ptr& output) {
    CHECK(output);
    if (output->is_keyframe_) {
      //! Only push to backend input queue if it is a keyframe!
      backend_input_queue.push(VIO::make_unique<BackendInput>(
          output->stereo_frame_lkf_.timestamp_,
          output->status_stereo_measurements_,
          output->tracker_status_,
          output->pim_,
          output->imu_acc_gyrs_,
          output->relative_pose_body_stereo_));
    } else {
      VLOG(5) << "Frontend did not output a keyframe, skipping backend input.";
    }
  });

  //! Params for what the backend outputs.
  // TODO(Toni): put this into backend params.
  BackendOutputParams backend_output_params(
      static_cast<VisualizationType>(FLAGS_viz_type) !=
          VisualizationType::kNone,
      FLAGS_min_num_obs_for_mesher_points,
      FLAGS_visualize && FLAGS_visualize_lmk_type);

  //! Create backend
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackEndModule>(
      &backend_input_queue_,
      parallel_run_,
      BackEndFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          stereo_camera_->getBodyPoseLeftCamRect(),
          stereo_camera_->getStereoCalib(),
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output));
  vio_backend_module_->registerOnFailureCallback(
      std::bind(&StereoPipeline::signalBackendFailure, this));
  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&StereoVisionFrontEndModule::updateImuBias,
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
    vio_frontend_module_->registerOutputCallback(
        std::bind(&MesherModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(mesher_module_.get())),
                  std::placeholders::_1));
  }

  if (FLAGS_visualize) {
    visualizer_module_ = VIO::make_unique<VisualizerModule>(
        //! Send ouput of visualizer to the display_input_queue_
        &display_input_queue_,
        parallel_run_,
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
    vio_frontend_module_->registerOutputCallback(
        std::bind(&VisualizerModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));
    if (mesher_module_) {
      mesher_module_->registerOutputCallback(
          std::bind(&VisualizerModule::fillMesherQueue,
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
                        std::bind(&StereoPipeline::shutdown, this)));
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
    vio_frontend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
  }

  // All modules are ready, launch threads! If the parallel_run flag is set to
  // false this will not do anything.
  launchThreads();
}

/* -------------------------------------------------------------------------- */
StereoPipeline::~StereoPipeline() {
  if (!shutdown_) {
    shutdown();
  } else {
    LOG(INFO) << "Manual shutdown was requested.";
  }
}

/* -------------------------------------------------------------------------- */
bool StereoPipeline::shutdownWhenFinished(const int& sleep_time_ms,
                                          const bool& print_stats) {
  // This is a very rough way of knowing if we have finished...
  // Since threads might be in the middle of processing data while we
  // query if the queues are empty.
  // Check every 0.5 seconds if all queues are empty.
  // Time to sleep between queries to the queues [in milliseconds].
  LOG(INFO) << "Shutting down VIO pipeline once processing has finished.";

  bool lcd_and_lcd_input_finished = true;
  if (lcd_module_) {
    lcd_and_lcd_input_finished = false;
  }

  CHECK(data_provider_module_);
  CHECK(vio_frontend_module_);
  CHECK(vio_backend_module_);

  while (
      !shutdown_ &&         // Loop while not explicitly shutdown.
      is_backend_ok_ &&     // Loop while backend is fine.
      (!isInitialized() ||  // Pipeline is not initialized and
                            // data is not yet consumed.
       !(!data_provider_module_->isWorking() &&
         (frontend_input_queue_.isShutdown() ||
          frontend_input_queue_.empty()) &&
         !vio_frontend_module_->isWorking() &&
         (backend_input_queue_.isShutdown() || backend_input_queue_.empty()) &&
         !vio_backend_module_->isWorking() &&
         (mesher_module_ ? !mesher_module_->isWorking() : true) &&
         (lcd_module_ ? !lcd_module_->isWorking() : true) &&
         (visualizer_module_ ? !visualizer_module_->isWorking() : true) &&
         (display_input_queue_.isShutdown() || display_input_queue_.empty()) &&
         (display_module_ ? !display_module_->isWorking() : true)))) {
    // Note that the values in the log below might be different than the
    // evaluation above since they are separately evaluated at different
    // times.
    VLOG(1) << "shutdown_: " << shutdown_ << '\n'
            << "VIO pipeline status: \n"
            << "Pipeline initialized? " << isInitialized() << '\n'
            << "Frontend initialized? " << vio_frontend_module_->isInitialized()
            << '\n'
            << "Backend initialized? " << vio_backend_module_->isInitialized()
            << '\n'
            << "Data provider is working? "
            << data_provider_module_->isWorking() << '\n'
            << "Frontend input queue shutdown? "
            << frontend_input_queue_.isShutdown() << '\n'
            << "Frontend input queue empty? " << frontend_input_queue_.empty()
            << '\n'
            << "Frontend is working? " << vio_frontend_module_->isWorking()
            << '\n'
            << "Backend Input queue shutdown? "
            << backend_input_queue_.isShutdown() << '\n'
            << "Backend Input queue empty? " << backend_input_queue_.empty()
            << '\n'
            << "Backend is working? " << vio_backend_module_->isWorking()
            << '\n'
            << "Mesher is working? "
            << (mesher_module_ ? mesher_module_->isWorking() : false) << '\n'
            << "LCD is working? "
            << (lcd_module_ ? lcd_module_->isWorking() : false) << '\n'
            << "Visualizer is working? "
            << (visualizer_module_ ? visualizer_module_->isWorking() : false)
            << '\n'
            << "Display Input queue shutdown? "
            << display_input_queue_.isShutdown() << '\n'
            << "Display Input queue empty? " << display_input_queue_.empty()
            << '\n'
            << "Displayer is working? "
            << (display_module_ ? display_module_->isWorking() : false);

    VLOG_IF(5, mesher_module_)
        << "Mesher is working? " << mesher_module_->isWorking();

    VLOG_IF(5, lcd_module_)
        << "LoopClosureDetector is working? " << lcd_module_->isWorking();

    VLOG_IF(5, visualizer_module_)
        << "Visualizer is working? " << visualizer_module_->isWorking();

    VLOG_IF(5, display_module_)
        << "Visualizer is working? " << display_module_->isWorking();

    // Print all statistics
    LOG(INFO) << utils::Statistics::Print();
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
  }
  LOG(INFO) << "Shutting down VIO, reason: input is empty and threads are "
               "idle.";
  VLOG(1) << "shutdown_: " << shutdown_ << '\n'
          << "VIO pipeline status: \n"
          << "Pipeline initialized? " << isInitialized() << '\n'
          << "Frontend initialized? " << vio_frontend_module_->isInitialized()
          << '\n'
          << "Backend initialized? " << vio_backend_module_->isInitialized()
          << '\n'
          << "Data provider is working? " << data_provider_module_->isWorking()
          << '\n'
          << "Frontend input queue shutdown? "
          << frontend_input_queue_.isShutdown() << '\n'
          << "Frontend input queue empty? " << frontend_input_queue_.empty()
          << '\n'
          << "Frontend is working? " << vio_frontend_module_->isWorking()
          << '\n'
          << "Backend Input queue shutdown? "
          << backend_input_queue_.isShutdown() << '\n'
          << "Backend Input queue empty? " << backend_input_queue_.empty()
          << '\n'
          << "Backend is working? " << vio_backend_module_->isWorking() << '\n'
          << "Mesher is working? "
          << (mesher_module_ ? mesher_module_->isWorking() : false) << '\n'
          << "LCD is working? "
          << (lcd_module_ ? lcd_module_->isWorking() : false) << '\n'
          << "Visualizer is working? "
          << (visualizer_module_ ? visualizer_module_->isWorking() : false)
          << '\n'
          << "Display Input queue shutdown? "
          << display_input_queue_.isShutdown() << '\n'
          << "Display Input queue empty? " << display_input_queue_.empty()
          << '\n'
          << "Displayer is working? "
          << (display_module_ ? display_module_->isWorking() : false);
  if (!shutdown_) shutdown();
  return true;
}

/* -------------------------------------------------------------------------- */
void StereoPipeline::shutdown() {
  Pipeline<StereoImuSyncPacket, StereoFrontendOutput>::shutdown();
  // Second: stop data provider
  CHECK(data_provider_module_);
  data_provider_module_->shutdown();

  // Third: stop VIO's threads
  stopThreads();
  if (parallel_run_) {
    joinThreads();
  }
  LOG(INFO) << "VIO Pipeline's threads shutdown successfully.\n"
            << "VIO Pipeline successful shutdown.";
}

void StereoPipeline::spinSequential() {
  // Spin once each pipeline module.
  CHECK(data_provider_module_);
  data_provider_module_->spin();

  CHECK(vio_frontend_module_);
  vio_frontend_module_->spin();

  CHECK(vio_backend_module_);
  vio_backend_module_->spin();

  if (mesher_module_) mesher_module_->spin();

  if (lcd_module_) lcd_module_->spin();

  if (visualizer_module_) visualizer_module_->spin();

  if (display_module_) display_module_->spin();
}

}  // namespace VIO
