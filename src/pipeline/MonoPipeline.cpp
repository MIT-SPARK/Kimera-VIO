/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoPipeline.cpp
 * @brief  Implements MonoVIO pipeline workflow.
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/MonoPipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "kimera-vio/backend/VioBackEndFactory.h"
#include "kimera-vio/frontend/MonoVisionFrontEnd-definitions.h"
#include "kimera-vio/frontend/VisionFrontEndFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/DisplayFactory.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

MonoPipeline::MonoPipeline(const VioParams& params,
                           Visualizer3D::UniquePtr&& visualizer,
                           DisplayBase::UniquePtr&& displayer)
    : Pipeline(params),
      camera_(nullptr) {
  // TODO(marcus): specify separate params for mono
  // CHECK_EQ(params.camera_params_.size(), 1u) << "Need one camera for MonoPipeline.";
  camera_ = std::make_shared<Camera>(params.camera_params_.at(0));

  data_provider_module_ = VIO::make_unique<MonoDataProviderModule>(
      &frontend_input_queue_,
      "Mono Data Provider",
      parallel_run_);

  data_provider_module_->registerVioPipelineCallback(
    std::bind(&MonoPipeline::spinOnce, this, std::placeholders::_1));

  LOG_IF(FATAL, params.frontend_params_.useStereoTracking_) 
      << "useStereoTracking is set to true, but this is a mono pipeline!";
  vio_frontend_module_ = VIO::make_unique<VisionFrontEndModule>(
      &frontend_input_queue_,
      parallel_run_,
      VisionFrontEndFactory::createFrontend(
        params.frontend_type_,
        params.imu_params_,
        gtsam::imuBias::ConstantBias(),
        params.frontend_params_,
        camera_,
        FLAGS_visualize ? &display_input_queue_ : nullptr,
        FLAGS_log_output));

  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback([&backend_input_queue](
      const FrontendOutputPacketBase::Ptr& output) {
    MonoFrontendOutput::Ptr converted_output = 
        VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(output);

    if (converted_output->is_keyframe_) {
      //! Only push to backend input queue if it is a keyframe!
      backend_input_queue.push(VIO::make_unique<BackendInput>(
          converted_output->frame_lkf_.timestamp_,
          converted_output->status_mono_measurements_,
          converted_output->tracker_status_,
          converted_output->pim_,
          converted_output->imu_acc_gyrs_,
          boost::none));  // don't pass stereo pose to backend!
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
  // TODO(marcus): get rid of fake stereocam
  LOG_IF(FATAL, params.backend_params_->addBetweenStereoFactors_)
      << "addBetweenStereoFactors is set to true, but this is a mono pipeline!";
  VIO::StereoCamera stereo_cam(params.camera_params_.at(0),
                               params.camera_params_.at(1));
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackEndModule>(
      &backend_input_queue_,
      parallel_run_,
      BackEndFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          camera_->getBodyPoseCam(),
          stereo_cam.getStereoCalib(),
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output));

  vio_backend_module_->registerOnFailureCallback(
      std::bind(&MonoPipeline::signalBackendFailure, this));

  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&VisionFrontEndModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
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
                         // NOTE: use kNone for now because mesher isn't enabled
                         // TODO(marcus): handle in params instead!
                         static_cast<VisualizationType>(VisualizationType::kNone),
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
                        std::bind(&MonoPipeline::shutdown, this)));
  }

  // TODO(marcus): enable use of lcd with mono pipeline
  // if (FLAGS_use_lcd) {
  //   lcd_module_ = VIO::make_unique<LcdModule>(
  //       parallel_run_,
  //       LcdFactory::createLcd(LoopClosureDetectorType::BoW,
  //                             params.lcd_params_,
  //                             camera_,
  //                             params.frontend_params_.stereo_matching_params_,
  //                             FLAGS_log_output));
  //   //! Register input callbacks
  //   vio_backend_module_->registerOutputCallback(
  //       std::bind(&LcdModule::fillBackendQueue,
  //                 std::ref(*CHECK_NOTNULL(lcd_module_.get())),
  //                 std::placeholders::_1));
  //   vio_frontend_module_->registerOutputCallback(
  //       std::bind(&LcdModule::fillFrontendQueue,
  //                 std::ref(*CHECK_NOTNULL(lcd_module_.get())),
  //                 std::placeholders::_1));
  // }

  launchThreads();
}

}  // namespace VIO
