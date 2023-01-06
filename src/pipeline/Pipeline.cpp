/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.cpp
 * @brief  Implements abstract VIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/Pipeline.h"

DEFINE_bool(log_output, false, "Log output to CSV files.");
DEFINE_bool(extract_planes_from_the_scene,
            false,
            "Whether to use structural regularities in the scene,"
            "currently only planes.");

DEFINE_bool(visualize, true, "Enable overall visualization.");
DEFINE_bool(visualize_lmk_type, false, "Enable landmark type visualization.");
DEFINE_int32(viz_type,
             0,
             "0: MESH2DTo3Dsparse, get a 3D mesh from a 2D triangulation of "
             "the (right-VALID).\n"
             "1: POINTCLOUD, visualize 3D VIO points (no repeated point)\n"
             "are re-plotted at every frame).\n"
             "keypoints in the left frame and filters out triangles \n"
             "2: NONE, does not visualize map.");

DEFINE_bool(deterministic_random_number_generator,
            false,
            "If true the random number generator will consistently output the "
            "same sequence of pseudo-random numbers for every run (use it to "
            "have repeatable output). If false the random number generator "
            "will output a different sequence for each run.");
DEFINE_int32(min_num_obs_for_mesher_points,
             4,
             "Minimum number of observations for a smart factor's landmark to "
             "to be used as a 3d point to consider for the mesher.");

DEFINE_bool(use_lcd,
            false,
            "Enable LoopClosureDetector processing in pipeline.");
DEFINE_bool(
    do_coarse_imu_camera_temporal_sync,
    false,
    "Compute the static offset between the IMU and camera timestamps. This is "
    "only really suitable for time offsets that have no physical meaning as it "
    "only computes the time difference between the last IMU and image "
    "measurement, and applies that to all IMU measurements going forward.");
DEFINE_bool(
    do_fine_imu_camera_temporal_sync,
    false,
    "Estimate the delay between the IMU and the camera. This enables "
    "estimating the time delay between the IMU and the camera (currently by "
    "cross-correlation between relative rotation angles).");

namespace VIO {

Pipeline::Pipeline(const VioParams& params)
    : backend_params_(params.backend_params_),
      frontend_params_(params.frontend_params_),
      imu_params_(params.imu_params_),
      parallel_run_(params.parallel_run_),
      data_provider_module_(nullptr),
      vio_frontend_module_(nullptr),
      frontend_input_queue_("frontend_input_queue"),
      vio_backend_module_(nullptr),
      backend_input_queue_("backend_input_queue"),
      mesher_module_(nullptr),
      lcd_module_(nullptr),
      visualizer_module_(nullptr),
      display_input_queue_("display_input_queue"),
      display_module_(nullptr),
      shutdown_pipeline_cb_(nullptr),
      frontend_thread_(nullptr),
      backend_thread_(nullptr),
      mesher_thread_(nullptr),
      lcd_thread_(nullptr),
      visualizer_thread_(nullptr) {
  if (FLAGS_deterministic_random_number_generator) {
    setDeterministicPipeline();
  }
}

Pipeline::~Pipeline() {
  if (!shutdown_) {
    shutdown();
  } else {
    LOG(INFO) << "Manual shutdown was requested.";
  }
}

bool Pipeline::spin() {
  // Feed data to the pipeline
  CHECK(data_provider_module_);
  LOG(INFO) << "Spinning Kimera-VIO.";
  return data_provider_module_->spin();
}

bool Pipeline::spinViz() {
  if (display_module_) {
    return display_module_->spin();
  }
  return true;
}

std::string Pipeline::printStatus() const {
  std::stringstream ss;
  ss << "shutdown_: " << shutdown_ << '\n'
     << "VIO pipeline status: \n"
     << "Pipeline initialized? " << isInitialized() << '\n'
     << "Frontend initialized? " << vio_frontend_module_->isInitialized()
     << '\n'
     << "Backend initialized? " << vio_backend_module_->isInitialized() << '\n'
     << "Data provider is working? " << data_provider_module_->isWorking()
     << '\n'
     << "Frontend input queue shutdown? " << frontend_input_queue_.isShutdown()
     << '\n'
     << "Frontend input queue empty? " << frontend_input_queue_.empty() << '\n'
     << "Frontend is working? " << vio_frontend_module_->isWorking() << '\n'
     << "Backend Input queue shutdown? " << backend_input_queue_.isShutdown()
     << '\n'
     << "Backend Input queue empty? " << backend_input_queue_.empty() << '\n'
     << "Backend is working? " << vio_backend_module_->isWorking() << '\n'
     << (mesher_module_
             ? ("Mesher is working? " +
                std::string(mesher_module_->isWorking() ? "Yes" : "No"))
             : "No mesher module.")
     << '\n'
     << (lcd_module_ ? ("LCD is working? " +
                        std::string(lcd_module_->isWorking() ? "Yes" : "No"))
                     : "No LCD module.")
     << '\n'
     << (visualizer_module_
             ? ("Visualizer is working? " +
                std::string(visualizer_module_->isWorking() ? "Yes" : "No"))
             : "No visualizer module.")
     << '\n'
     << "Display Input queue shutdown? " << display_input_queue_.isShutdown()
     << '\n'
     << "Display Input queue empty? " << display_input_queue_.empty() << '\n'
     << (display_module_
             ? ("Displayer is working? " +
                std::string(display_module_->isWorking() ? "Yes" : "No"))
             : "No display module.");
  return ss.str();
}

/* -------------------------------------------------------------------------- */
bool Pipeline::shutdownWhenFinished(const int& sleep_time_ms,
                                    const bool& print_stats) {
  // defaults to only checking whether the pipeline itself has finished
  return waitForShutdown(
      []() -> bool { return true; }, sleep_time_ms, print_stats);
}

/* -------------------------------------------------------------------------- */
bool Pipeline::waitForShutdown(const std::function<bool()>& data_done_cb,
                               const int& sleep_time_ms,
                               const bool& print_stats) {
  LOG_IF(INFO, parallel_run_)
      << "Shutting down VIO pipeline once processing has finished.";

  CHECK(data_provider_module_);
  CHECK(vio_frontend_module_);
  CHECK(vio_backend_module_);

  while (!hasFinished() || !data_done_cb()) {
    // Note that the values in the log below might be different than the
    // evaluation above since they are separately evaluated at different times.
    VLOG(5) << printStatus();

    // Print all statistics
    LOG_IF(INFO, print_stats) << utils::Statistics::Print();

    // Time to sleep between queries to the queues [in milliseconds].
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));

    if (!parallel_run_) {
      // Don't break, otw we will shutdown the pipeline.
      return false;
    }
  }
  LOG(INFO) << "Shutting down VIO, reason: input is empty and threads are "
               "idle.";
  VLOG(5) << printStatus();
  if (!shutdown_) shutdown();
  return true;
}

void Pipeline::spinSequential() {
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

bool Pipeline::hasFinished() const {
  CHECK(data_provider_module_);
  CHECK(vio_frontend_module_);
  CHECK(vio_backend_module_);

  const bool fqueue_done =
      frontend_input_queue_.isShutdown() || frontend_input_queue_.empty();
  const bool bqueue_done =
      backend_input_queue_.isShutdown() || backend_input_queue_.empty();
  const bool dqueue_done =
      display_input_queue_.isShutdown() || display_input_queue_.empty();
  const bool mesher_done =
      mesher_module_ != nullptr ? !mesher_module_->isWorking() : true;
  const bool lcd_done =
      lcd_module_ != nullptr ? !lcd_module_->isWorking() : true;
  const bool visualizer_done =
      visualizer_module_ != nullptr ? !visualizer_module_->isWorking() : true;
  const bool display_done =
      display_module_ != nullptr ? !display_module_->isWorking() : true;

  VLOG(10) << std::endl
          << std::boolalpha
          << "  - data: " << data_provider_module_->isWorking() << std::endl
          << "  - frontend_input_queue: " << fqueue_done << std::endl
          << "  - frontend: " << vio_frontend_module_->isWorking() << std::endl
          << "  - backend_input_queue: " << bqueue_done << std::endl
          << "  - backend: " << vio_backend_module_->isWorking() << std::endl
          << "  - mesher: " << mesher_done << std::endl
          << "  - lcd: " << lcd_done << std::endl
          << "  - visualizer: " << visualizer_done << std::endl
          << "  - display_input_queue: " << dqueue_done << std::endl
          << "  - display: " << display_done << std::endl;

  // This is a very rough way of knowing if we have finished...
  // Since threads might be in the middle of processing data while we
  // query if the queues are empty.
  return !(                 // Negate everything (too lazy to negate everything)
      !shutdown_ &&         // Loop while not explicitly shutdown.
      is_backend_ok_ &&     // Loop while Backend is fine.
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
         (display_module_ ? !display_module_->isWorking() : true))));
}

void Pipeline::shutdown() {
  LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                              "shutdown.";
  LOG(INFO) << "Shutting down VIO pipeline.";
  shutdown_ = true;

  // First: call registered shutdown callbacks, these are typically to signal
  // data providers that they should now die.
  if (shutdown_pipeline_cb_) {
    LOG(INFO) << "Calling registered shutdown callbacks...";
    // Mind that this will raise a SIGSEGV seg fault if the callee is
    // destroyed.
    shutdown_pipeline_cb_();
  }

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

  if (FLAGS_log_output) {
    PipelineLogger logger;
    // TODO(nathan) consider adding actual elapsed time
    logger.logPipelineOverallTiming(std::chrono::milliseconds(0));
  }
}

void Pipeline::resume() {
  LOG(INFO) << "Restarting Frontend workers and queues...";
  frontend_input_queue_.resume();

  LOG(INFO) << "Restarting Backend workers and queues...";
  backend_input_queue_.resume();
}

void Pipeline::spinOnce(FrontendInputPacketBase::UniquePtr input) {
  CHECK(input);
  if (!shutdown_) {
    // Push to Frontend input queue.
    VLOG(2) << "Push input payload to Frontend.";
    frontend_input_queue_.pushBlockingIfFull(std::move(input), 5u);

    if (!parallel_run_) {
      // Run the pipeline sequentially.
      spinSequential();
    }
  } else {
    LOG(WARNING) << "Not spinning pipeline as it's been shutdown.";
  }
}

void Pipeline::launchThreads() {
  if (parallel_run_) {
    frontend_thread_ = VIO::make_unique<std::thread>(
        &VisionImuFrontendModule::spin,
        CHECK_NOTNULL(vio_frontend_module_.get()));

    backend_thread_ = VIO::make_unique<std::thread>(
        &VioBackendModule::spin, CHECK_NOTNULL(vio_backend_module_.get()));

    if (mesher_module_) {
      mesher_thread_ = VIO::make_unique<std::thread>(
          &MesherModule::spin, CHECK_NOTNULL(mesher_module_.get()));
    }

    if (lcd_module_) {
      lcd_thread_ = VIO::make_unique<std::thread>(
          &LcdModule::spin, CHECK_NOTNULL(lcd_module_.get()));
    }

    if (visualizer_module_) {
      visualizer_thread_ = VIO::make_unique<std::thread>(
          &VisualizerModule::spin, CHECK_NOTNULL(visualizer_module_.get()));
    }
    LOG(INFO) << "Pipeline Modules launched (parallel_run set to "
              << parallel_run_ << ").";
  } else {
    LOG(INFO) << "Pipeline Modules running in sequential mode"
              << " (parallel_run set to " << parallel_run_ << ").";
  }
}

void Pipeline::stopThreads() {
  VLOG(1) << "Stopping workers and queues...";

  backend_input_queue_.shutdown();
  CHECK(vio_backend_module_);
  vio_backend_module_->shutdown();

  frontend_input_queue_.shutdown();
  CHECK(vio_frontend_module_);
  vio_frontend_module_->shutdown();

  if (mesher_module_) mesher_module_->shutdown();
  if (lcd_module_) lcd_module_->shutdown();
  if (visualizer_module_) visualizer_module_->shutdown();
  if (display_module_) {
    display_input_queue_.shutdown();
    display_module_->shutdown();
  }

  VLOG(1) << "Sent stop flag to all module and queues...";
}

void Pipeline::joinThreads() {
  LOG_IF(WARNING, !parallel_run_)
      << "Asked to join threads while in sequential mode, this is ok, but "
      << "should not happen.";
  VLOG(1) << "Joining threads...";

  joinThread("Backend", backend_thread_.get());
  joinThread("Frontend", frontend_thread_.get());
  joinThread("mesher", mesher_thread_.get());
  joinThread("lcd", lcd_thread_.get());
  joinThread("visualizer", visualizer_thread_.get());

  VLOG(1) << "All threads joined.";
}

void Pipeline::joinThread(const std::string& thread_name, std::thread* thread) {
  if (thread) {
    VLOG(1) << "Joining " << thread_name.c_str() << " thread...";
    if (thread->joinable()) {
      thread->join();
      VLOG(1) << "Joined " << thread_name.c_str() << " thread...";
    } else {
      LOG_IF(ERROR, parallel_run_)
          << thread_name.c_str() << " thread is not joinable...";
    }
  } else {
    LOG(WARNING) << "No " << thread_name.c_str() << " thread, not joining.";
  }
}

}  // namespace VIO
