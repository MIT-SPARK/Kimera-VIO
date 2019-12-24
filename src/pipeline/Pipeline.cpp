/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.cpp
 * @brief  Implements VIO pipeline workflow.
 * @author Antoni Rosinol
 */

#include "kimera-vio/pipeline/Pipeline.h"

#include <future>
#include <string>
#include <utility>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>

#include "kimera-vio/backend/VioBackEndFactory.h"
#include "kimera-vio/frontend/VisionFrontEndFactory.h"
#include "kimera-vio/initial/InitializationBackEnd.h"
#include "kimera-vio/initial/InitializationFromImu.h"
#include "kimera-vio/initial/OnlineGravityAlignment.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/visualizer/Visualizer3DFactory.h"

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

DEFINE_bool(use_feature_selection, false, "Enable smart feature selection.");

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

DEFINE_int32(num_frames_vio_init,
             25,
             "Minimum number of frames for the online "
             "gravity-aligned initialization.");

// TODO(Sandro): Create YAML file for initialization and read in!
DEFINE_double(smart_noise_sigma_bundle_adjustment,
              1.5,
              "Smart noise sigma for bundle adjustment"
              " in initialization.");
DEFINE_double(outlier_rejection_bundle_adjustment,
              30,
              "Outlier rejection for bundle adjustment"
              " in initialization.");
DEFINE_double(between_translation_bundle_adjustment,
              0.5,
              "Between factor precision for bundle adjustment"
              " in initialization.");
DEFINE_int32(max_time_allowed_for_keyframe_callback,
             5u,
             "Maximum time allowed for processing keyframe rate callback "
             "(in ms).");

DEFINE_bool(use_lcd,
            false,
            "Enable LoopClosureDetector processing in pipeline.");

namespace VIO {

Pipeline::Pipeline(const VioParams& params)
    : backend_type_(static_cast<BackendType>(params.backend_type_)),
      stereo_camera_(nullptr),
      data_provider_module_(nullptr),
      vio_frontend_module_(nullptr),
      feature_selector_(nullptr),
      vio_backend_module_(nullptr),
      lcd_module_(nullptr),
      backend_params_(params.backend_params_),
      frontend_params_(params.frontend_params_),
      imu_params_(params.imu_params_),
      mesher_module_(nullptr),
      visualizer_module_(nullptr),
      frontend_thread_(nullptr),
      backend_thread_(nullptr),
      mesher_thread_(nullptr),
      lcd_thread_(nullptr),
      visualizer_thread_(nullptr),
      parallel_run_(params.parallel_run_),
      stereo_frontend_input_queue_("stereo_frontend_input_queue"),
      initialization_frontend_output_queue_(
          "initialization_frontend_output_queue"),
      backend_input_queue_("backend_input_queue") {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();

  //! Create Stereo Camera
  CHECK_EQ(params.camera_params_.size(), 2u) << "Only stereo camera support.";
  stereo_camera_ = VIO::make_unique<StereoCamera>(
      params.camera_params_.at(0),
      params.camera_params_.at(1),
      params.frontend_params_.stereo_matching_params_);

  //! Create DataProvider
  data_provider_module_ = VIO::make_unique<DataProviderModule>(
      &stereo_frontend_input_queue_,
      "Data Provider",
      parallel_run_,
      // TODO(Toni): these params should not be sent...
      params.frontend_params_.stereo_matching_params_);

  data_provider_module_->registerVioPipelineCallback(
      std::bind(&Pipeline::spinOnce, this, std::placeholders::_1));

  //! Create frontend
  vio_frontend_module_ = VIO::make_unique<StereoVisionFrontEndModule>(
      &stereo_frontend_input_queue_,
      parallel_run_,
      VisionFrontEndFactory::createFrontend(params.frontend_type_,
                                            params.imu_params_,
                                            gtsam::imuBias::ConstantBias(),
                                            params.frontend_params_,
                                            FLAGS_log_output));
  auto& backend_input_queue = backend_input_queue_;  //! for the lambda below
  vio_frontend_module_->registerCallback(
      [&backend_input_queue](const FrontendOutput::Ptr& output) {
        if (output->is_keyframe_) {
          //! Only push to backend input queue if it is a keyframe!
          backend_input_queue.push(VIO::make_unique<BackendInput>(
              output->stereo_frame_lkf_.getTimestamp(),
              output->status_stereo_measurements_,
              output->tracker_status_,
              output->pim_,
              output->relative_pose_body_stereo_));
        }
      });

  //! Params for what the backend outputs.
  // TODO(Toni): put this into backend params.
  BackendOutputParams backend_output_params(
      static_cast<VisualizationType>(FLAGS_viz_type) ==
          VisualizationType::kMesh2dTo3dSparse,
      FLAGS_min_num_obs_for_mesher_points,
      FLAGS_visualize_lmk_type);

  //! Create backend
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackEndModule>(
      &backend_input_queue_,
      parallel_run_,
      BackEndFactory::createBackend(backend_type_,
                                    // These two should be given by parameters.
                                    stereo_camera_->getLeftCamPose(),
                                    stereo_camera_->getStereoCalib(),
                                    *backend_params_,
                                    imu_params_,
                                    backend_output_params,
                                    FLAGS_log_output));
  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&StereoVisionFrontEndModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  // TODO(Toni): only create if used.
  mesher_module_ = VIO::make_unique<MesherModule>(
      parallel_run_,
      MesherFactory::createMesher(
          MesherType::PROJECTIVE,
          MesherParams(stereo_camera_->getLeftCamPose(),
                       params.camera_params_.at(0).image_size_)));
  //! Register input callbacks
  vio_backend_module_->registerCallback(
      std::bind(&MesherModule::fillBackendQueue,
                std::ref(*CHECK_NOTNULL(mesher_module_.get())),
                std::placeholders::_1));
  vio_frontend_module_->registerCallback(
      std::bind(&MesherModule::fillFrontendQueue,
                std::ref(*CHECK_NOTNULL(mesher_module_.get())),
                std::placeholders::_1));

  if (FLAGS_visualize) {
    visualizer_module_ = VIO::make_unique<VisualizerModule>(
        parallel_run_,
        VisualizerFactory::createVisualizer(
            VisualizerType::OpenCV,
            // TODO(Toni): bundle these three params in VisualizerParams...
            static_cast<VisualizationType>(FLAGS_viz_type),
            backend_type_));
    //! Register input callbacks
    vio_backend_module_->registerCallback(
        std::bind(&VisualizerModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));
    vio_frontend_module_->registerCallback(
        std::bind(&VisualizerModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));
    mesher_module_->registerCallback(
        std::bind(&VisualizerModule::fillMesherQueue,
                  std::ref(*CHECK_NOTNULL(visualizer_module_.get())),
                  std::placeholders::_1));
    //! Actual displaying of visual data is done in the main thread.
    visualizer_module_->registerCallback(std::bind(
        &Pipeline::spinDisplayOnce, std::cref(*this), std::placeholders::_1));
  }

  if (FLAGS_use_lcd) {
    lcd_module_ = VIO::make_unique<LcdModule>(
        parallel_run_,
        LcdFactory::createLcd(LoopClosureDetectorType::BoW,
                              params.lcd_params_,
                              FLAGS_log_output));
    //! Register input callbacks
    vio_backend_module_->registerCallback(
        std::bind(&LcdModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
    vio_frontend_module_->registerCallback(
        std::bind(&LcdModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
  }

  // Instantiate feature selector: not used in vanilla implementation.
  if (FLAGS_use_feature_selection) {
    feature_selector_ =
        VIO::make_unique<FeatureSelector>(frontend_params_, *backend_params_);
  }
}

/* -------------------------------------------------------------------------- */
Pipeline::~Pipeline() {
  LOG(INFO) << "Pipeline destructor called.";
  // Shutdown pipeline if it is not already down.
  if (!shutdown_) {
    shutdown();
  } else {
    LOG(INFO) << "Manual shutdown was requested.";
  }
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinOnce(StereoImuSyncPacket::UniquePtr stereo_imu_sync_packet) {
  CHECK(stereo_imu_sync_packet);
  CHECK(!shutdown_) << "Pipeline is shutdown.";
  // Check if we have to re-initialize
  checkReInitialize(*stereo_imu_sync_packet);
  // Initialize pipeline if not initialized
  if (!is_initialized_) {
    // Launch frontend thread
    if (!is_launched_) {
      launchFrontendThread();
      is_launched_ = true;
      init_frame_id_ = stereo_imu_sync_packet->getStereoFrame().getFrameId();
    }
    CHECK(is_launched_);

    // Initialize pipeline.
    // TODO this is very brittle, because we are accumulating IMU data, but
    // not using it for initialization, because accumulated and actual IMU data
    // at init is the same...
    if (initialize(*stereo_imu_sync_packet)) {
      LOG(INFO) << "Before launching threads.";
      launchRemainingThreads();
      LOG(INFO) << " launching threads.";
      is_initialized_ = true;
    } else {
      LOG(INFO) << "Not yet initialized...";
    }
  } else {
    // SEND INFO TO FRONTEND, here is when we are in nominal mode.
    // TODO Warning: we do not accumulate IMU measurements for the first
    // packet... Spin.
    CHECK(is_initialized_);

    // Push to stereo frontend input queue.
    VLOG(2) << "Push input payload to Frontend.";
    stereo_frontend_input_queue_.push(std::move(stereo_imu_sync_packet));

    // Run the pipeline sequentially.
    if (!parallel_run_) spinSequential();
  }

  return;
}

// Returns whether the visualizer_ is running or not. While in parallel mode,
// it does not return unless shutdown.
bool Pipeline::spinViz() {
  if (visualizer_module_) {
    return visualizer_module_->spin();
  }
  return true;
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinSequential() {
  // Spin once each pipeline module.
  // CHECK(data_provider_module_);
  // data_provider_module_->spin();

  CHECK(vio_frontend_module_);
  vio_frontend_module_->spin();

  CHECK(vio_backend_module_);
  vio_backend_module_->spin();

  if (mesher_module_) mesher_module_->spin();

  if (lcd_module_) lcd_module_->spin();

  if (visualizer_module_) visualizer_module_->spin();
}

// TODO: Adapt this function to be able to cope with new initialization
/* -------------------------------------------------------------------------- */
bool Pipeline::shutdownWhenFinished() {
  // This is a very rough way of knowing if we have finished...
  // Since threads might be in the middle of processing data while we
  // query if the queues are empty.
  // Check every second if all queues are empty.
  // Time to sleep between queries to the queues [in seconds].
  LOG(INFO) << "Shutting down VIO pipeline once processing has finished.";
  static constexpr int sleep_time = 1;

  bool lcd_and_lcd_input_finished = true;
  if (lcd_module_) {
    lcd_and_lcd_input_finished = false;
  }

  CHECK(data_provider_module_);
  CHECK(vio_frontend_module_);
  CHECK(vio_backend_module_);

  while (!shutdown_ &&         // Loop while not explicitly shutdown.
         (!is_initialized_ ||  // Loop while not initialized
                               // Or, once init, data is not yet consumed.
          !(!data_provider_module_->isWorking() &&
            stereo_frontend_input_queue_.empty() &&
            !vio_frontend_module_->isWorking() &&
            backend_input_queue_.empty() && !vio_backend_module_->isWorking() &&
            (mesher_module_ ? !mesher_module_->isWorking() : true) &&
            (lcd_module_ ? !lcd_module_->isWorking() : true) &&
            (visualizer_module_ ? !visualizer_module_->isWorking() : true)))) {
    VLOG(5) << "shutdown_: " << shutdown_ << '\n'
            << "VIO pipeline status: \n"
            << "Initialized? " << is_initialized_ << '\n'
            << "Data provider is working? "
            << data_provider_module_->isWorking() << '\n'
            << "Frontend input queue empty? "
            << stereo_frontend_input_queue_.empty() << '\n'
            << "Frontend is working? " << vio_frontend_module_->isWorking()
            << '\n'
            << "Backend Input queue empty? " << backend_input_queue_.empty()
            << '\n'
            << "Backend is working? "
            << (is_initialized_ ? vio_backend_module_->isWorking() : false);

    VLOG_IF(5, mesher_module_)
        << "Mesher is working? " << mesher_module_->isWorking();

    VLOG_IF(5, lcd_module_)
        << "LoopClosureDetector is working? " << lcd_module_->isWorking();

    VLOG_IF(5, visualizer_module_)
        << "Visualizer is working? " << visualizer_module_->isWorking();

    std::this_thread::sleep_for(std::chrono::seconds(sleep_time));
  }
  LOG(INFO) << "Shutting down VIO, reason: input is empty and threads are "
               "idle.";
  VLOG(10) << "shutdown_: " << shutdown_ << '\n'
           << "VIO pipeline status: \n"
           << "Initialized? " << is_initialized_ << '\n'
           << "Data provider is working? " << data_provider_module_->isWorking()
           << '\n'
           << "Frontend input queue empty? "
           << stereo_frontend_input_queue_.empty() << '\n'
           << "Frontend is working? " << vio_frontend_module_->isWorking()
           << '\n'
           << "Backend Input queue empty? " << backend_input_queue_.empty()
           << '\n'
           << "Backend is working? "
           << (is_initialized_ ? vio_backend_module_->isWorking() : false);
  if (!shutdown_) shutdown();
  return true;
}

/* -------------------------------------------------------------------------- */
void Pipeline::shutdown() {
  LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                              "shutdown.";
  LOG(INFO) << "Shutting down VIO pipeline.";
  shutdown_ = true;
  stopThreads();
  if (parallel_run_) {
    joinThreads();
  }
  LOG(INFO) << "Pipeline destructor finished.";
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initialize(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  switch (backend_params_->autoInitialize_) {
    case 0:
      // If the gtNavState is identity, the params provider probably did a
      // mistake, although it can happen that the ground truth initial pose is
      // identity! But if that is the case, create another autoInitialize value
      // for this case and send directly a identity pose...
      CHECK(!backend_params_->initial_ground_truth_state_.equals(VioNavState()))
          << "Requested initialization from Ground-Truth pose but got an "
             "identity pose: did you parse your ground-truth correctly?";
      return initializeFromGroundTruth(
          stereo_imu_sync_packet, backend_params_->initial_ground_truth_state_);
    case 1:
      return initializeFromIMU(stereo_imu_sync_packet);
    case 2:
      // Initialization using online gravity alignment.
      return initializeOnline(stereo_imu_sync_packet);
    default:
      LOG(FATAL) << "Wrong initialization mode.";
  }
  return false;
}

/* -------------------------------------------------------------------------- */
// TODO: Adapt and create better re-initialization (online) function
void Pipeline::checkReInitialize(
    const StereoImuSyncPacket& stereo_imu_sync_packet) {
  // Re-initialize pipeline if requested
  if (is_initialized_ &&
      stereo_imu_sync_packet.getReinitPacket().getReinitFlag()) {
    LOG(WARNING) << "Re-initialization triggered!";
    // Shutdown pipeline first
    shutdown();

    // Reset shutdown flags
    shutdown_ = false;
    // Set initialization flag to false
    is_initialized_ = false;
    // Set launch thread flag to false
    is_launched_ = false;
    // Reset initial id to current id
    init_frame_id_ = stereo_imu_sync_packet.getStereoFrame().getFrameId();

    // Resume threads
    CHECK(vio_frontend_module_);
    vio_frontend_module_->restart();
    CHECK(vio_backend_module_);
    vio_backend_module_->restart();
    mesher_module_->restart();
    if (lcd_module_) lcd_module_->restart();
    visualizer_module_->restart();
    // Resume pipeline
    resume();
    initialization_frontend_output_queue_.resume();
  }
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initializeFromGroundTruth(
    const StereoImuSyncPacket& stereo_imu_sync_packet,
    const VioNavState& initial_ground_truth_state) {
  LOG(INFO) << "------------------- Initialize Pipeline with frame k = "
            << stereo_imu_sync_packet.getStereoFrame().getFrameId()
            << "--------------------";

  // Initialize Stereo Frontend.
  CHECK(vio_frontend_module_);
  const StereoFrame& stereo_frame_lkf =
      vio_frontend_module_->processFirstStereoFrame(
          stereo_imu_sync_packet.getStereoFrame());

  // Initialize Backend using ground-truth.
  CHECK(vio_backend_module_);
  vio_backend_module_->initializeBackend(VioNavStateTimestamped(
      stereo_frame_lkf.getTimestamp(), initial_ground_truth_state));

  return true;
}

/* -------------------------------------------------------------------------- */
// Assumes Zero Velocity & upright vehicle.
bool Pipeline::initializeFromIMU(
    const StereoImuSyncPacket& stereo_imu_sync_packet) {
  LOG(INFO) << "------------------- Initialize Pipeline with frame k = "
            << stereo_imu_sync_packet.getStereoFrame().getFrameId()
            << "--------------------";

  // Guess pose from IMU, assumes vehicle to be static.
  VioNavState initial_state_estimate =
      InitializationFromImu::getInitialStateEstimate(
          stereo_imu_sync_packet.getImuAccGyr(),
          imu_params_.n_gravity_,
          backend_params_->roundOnAutoInitialize_);

  // Initialize Stereo Frontend.
  CHECK(vio_frontend_module_);
  const StereoFrame& stereo_frame_lkf =
      vio_frontend_module_->processFirstStereoFrame(
          stereo_imu_sync_packet.getStereoFrame());

  // Initialize Backend using IMU data.
  CHECK(vio_backend_module_);
  vio_backend_module_->initializeBackend(VioNavStateTimestamped(
      stereo_frame_lkf.getTimestamp(), initial_state_estimate));

  return true;
}

/* -------------------------------------------------------------------------- */
// TODO (Toni): move this as much as possible inside initialization...
bool Pipeline::initializeOnline(
    const StereoImuSyncPacket& stereo_imu_sync_packet) {
  int frame_id = stereo_imu_sync_packet.getStereoFrame().getFrameId();
  LOG(INFO) << "------------------- Initializing Pipeline with frame k = "
            << frame_id << "--------------------";

  CHECK(vio_frontend_module_);
  CHECK_GE(frame_id, init_frame_id_);
  CHECK_GE(init_frame_id_ + FLAGS_num_frames_vio_init, frame_id);

  // TODO(Sandro): Find a way to optimize this
  // Create ImuFrontEnd with non-zero gravity (zero bias)
  ImuFrontEnd imu_frontend_real(
      imu_params_,
      gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
  CHECK_DOUBLE_EQ(imu_frontend_real.getPreintegrationGravity().norm(),
                  imu_params_.n_gravity_.norm());

  // Enforce stereo frame as keyframe for initialization
  StereoFrame stereo_frame = stereo_imu_sync_packet.getStereoFrame();
  stereo_frame.setIsKeyframe(true);
  // TODO: this is copying the packet implicitly, just to set a flag to true.
  StereoImuSyncPacket::UniquePtr stereo_imu_sync_init =
      VIO::make_unique<StereoImuSyncPacket>(
          stereo_frame,
          stereo_imu_sync_packet.getImuStamps(),
          stereo_imu_sync_packet.getImuAccGyr(),
          stereo_imu_sync_packet.getReinitPacket());

  FrontendOutput::ConstPtr frontend_output = nullptr;
  /////////////////// FIRST FRAME //////////////////////////////////////////////
  if (frame_id == init_frame_id_) {
    // Set trivial bias, gravity and force 5/3 point method for initialization
    vio_frontend_module_->prepareFrontendForOnlineAlignment();
    // Initialize Stereo Frontend.
    vio_frontend_module_->processFirstStereoFrame(
        stereo_imu_sync_init->getStereoFrame());

    //! Register frontend output queue for the initializer.
    vio_frontend_module_->registerCallback(
        [&frontend_output](const FrontendOutput::ConstPtr& output) {
          frontend_output = output;
        });
    return false;
  } else {
    // Check trivial bias and gravity vector for online initialization
    vio_frontend_module_->checkFrontendForOnlineAlignment();
    // Spin frontend once with enforced keyframe and 53-point method
    vio_frontend_module_->spinOnce(std::move(stereo_imu_sync_init));
    // TODO(Sandro): Optionally add AHRS PIM
    CHECK(frontend_output);
    initialization_frontend_output_queue_.push(
        VIO::make_unique<InitializationInputPayload>(
            frontend_output->is_keyframe_,
            frontend_output->status_stereo_measurements_,
            frontend_output->tracker_status_,
            frontend_output->relative_pose_body_stereo_,
            frontend_output->stereo_frame_lkf_,
            frontend_output->pim_,
            frontend_output->debug_tracker_info_));

    // TODO(Sandro): Find a way to optimize this
    // This queue is used for the the backend optimization
    const ImuStampS& imu_stamps = stereo_imu_sync_packet.getImuStamps();
    const ImuAccGyrS& imu_accgyr = stereo_imu_sync_packet.getImuAccGyr();
    ImuFrontEnd::PimPtr pim =
        imu_frontend_real.preintegrateImuMeasurements(imu_stamps, imu_accgyr);
    // This queue is used for the backend after initialization
    VLOG(2) << "Initialization: Push input payload to Backend.";
    backend_input_queue_.push(VIO::make_unique<BackendInput>(
        frontend_output->stereo_frame_lkf_.getTimestamp(),
        frontend_output->status_stereo_measurements_,
        frontend_output->tracker_status_,
        pim,
        frontend_output->relative_pose_body_stereo_));

    // Only process set of frontend outputs after specific number of frames
    if (frame_id < (init_frame_id_ + FLAGS_num_frames_vio_init)) {
      return false;
    } else {
      ///////////////////////////// ONLINE INITIALIZER //////////////////////
      auto tic_full_init = utils::Timer::tic();

      // Create empty output variables
      gtsam::Vector3 gyro_bias, g_iter_b0;
      gtsam::NavState init_navstate;

      // Get frontend output to backend input for online initialization
      InitializationBackEnd::InitializationQueue output_frontend;
      CHECK(initialization_frontend_output_queue_.batchPop(&output_frontend));
      // Shutdown the initialization input queue once used
      initialization_frontend_output_queue_.shutdown();

      // Adjust parameters for Bundle Adjustment
      // TODO(Sandro): Create YAML file for initialization and read in!
      VioBackEndParams backend_params_init(*backend_params_);
      backend_params_init.smartNoiseSigma_ =
          FLAGS_smart_noise_sigma_bundle_adjustment;
      backend_params_init.outlierRejection_ =
          FLAGS_outlier_rejection_bundle_adjustment;
      backend_params_init.betweenTranslationPrecision_ =
          FLAGS_between_translation_bundle_adjustment;

      // Create initial backend
      CHECK(stereo_camera_);
      InitializationBackEnd initial_backend(
          stereo_camera_->getLeftCamPose(),
          stereo_camera_->getStereoCalib(),
          backend_params_init,
          imu_params_,
          BackendOutputParams(false, 0, false),
          FLAGS_log_output);

      // Enforce zero bias in initial propagation
      // TODO(Sandro): Remove this, once AHRS is implemented
      vio_frontend_module_->updateAndResetImuBias(
          gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
      gyro_bias = vio_frontend_module_->getCurrentImuBias().gyroscope();

      // Initialize if successful
      if (initial_backend.bundleAdjustmentAndGravityAlignment(
              output_frontend, &gyro_bias, &g_iter_b0, &init_navstate)) {
        LOG(INFO) << "Bundle adjustment and alignment successful!";

        // Reset frontend with non-trivial gravity and remove 53-enforcement.
        // Update frontend with initial gyro bias estimate.
        vio_frontend_module_->resetFrontendAfterOnlineAlignment(
            imu_params_.n_gravity_, gyro_bias);
        LOG(WARNING) << "Time used for initialization: "
                     << utils::Timer::toc(tic_full_init).count() << " (ms).";

        ///////////////////////////// BACKEND ////////////////////////////////
        // Initialize backend with pose estimate from gravity alignment
        // Create initial state for initialization from online gravity
        VioNavState initial_state_OGA(init_navstate,
                                      ImuBias(gtsam::Vector3(), gyro_bias));
        // Initialize Backend using IMU data.
        CHECK(vio_backend_module_);
        vio_backend_module_->initializeBackend(VioNavStateTimestamped(
            frontend_output->stereo_frame_lkf_.getTimestamp(),
            initial_state_OGA));
        LOG(INFO) << "Initialization finalized.";

        // TODO(Sandro): Create check-return for function
        return true;
      } else {
        // Reset initialization
        LOG(ERROR) << "Bundle adjustment or alignment failed!";
        init_frame_id_ = stereo_imu_sync_packet.getStereoFrame().getFrameId();
        initialization_frontend_output_queue_.shutdown();
        initialization_frontend_output_queue_.resume();
        return false;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinDisplayOnce(const VisualizerOutput::Ptr& viz_output) const {
  // Display 3D window.
  if (viz_output->visualization_type_ != VisualizationType::kNone) {
    VLOG(10) << "Spin Visualize 3D output.";
    // visualizer_output_payload->window_.spin();
    CHECK(!viz_output->window_.wasStopped());
    viz_output->window_.spinOnce(1, true);
  }

  // Display 2D images.
  for (const ImageToDisplay& img_to_display : viz_output->images_to_display_) {
    cv::imshow(img_to_display.name_, img_to_display.image_);
  }
  VLOG(10) << "Spin Visualize 2D output.";
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
StatusStereoMeasurements Pipeline::featureSelect(
    const VioFrontEndParams& tracker_params,
    const FeatureSelectorParams& feature_selector_params,
    const Timestamp& timestamp_k,
    const Timestamp& timestamp_lkf,
    const gtsam::Pose3& W_Pose_Blkf,
    double* feature_selection_time,
    std::shared_ptr<StereoFrame>& stereoFrame_km1,
    const StatusStereoMeasurements& status_stereo_meas,
    int cur_kf_id,
    int save_image_selector,
    const gtsam::Matrix& curr_state_cov,
    const Frame& left_frame) {  // last one for visualization only
  CHECK_NOTNULL(feature_selection_time);

  // ------------ DATA ABOUT CURRENT AND FUTURE ROBOT STATE ------------- //
  size_t nrKfInHorizon =
      round(feature_selector_params.featureSelectionHorizon_ /
            tracker_params.intra_keyframe_time_);
  VLOG(100) << "nrKfInHorizon for selector: " << nrKfInHorizon;

  // Future poses are gt and might be far from the vio pose: we have to
  // attach the *relative* poses from the gt to the latest vio estimate.
  // W_Pose_Bkf_gt    : ground truth pose at previous keyframe.
  // vio->W_Pose_Blkf_: vio pose at previous keyframe.
  // More important than the time, it is important that
  // it is the same time as vio->W_Pose_Blkf_
  KeyframeToStampedPose posesAtFutureKeyframes;
  Pose3 W_Pose_Bkf_gt;

  VLOG(100) << "Starting feature selection...";
  SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
  std::tie(trackedAndSelectedSmartStereoMeasurements, *feature_selection_time) =
      feature_selector_->splitTrackedAndNewFeatures_Select_Display(
          stereoFrame_km1,
          status_stereo_meas.second,
          cur_kf_id,
          save_image_selector,
          feature_selector_params.featureSelectionCriterion_,
          feature_selector_params.featureSelectionNrCornersToSelect_,
          tracker_params.maxFeatureAge_,
          posesAtFutureKeyframes,
          curr_state_cov,
          "",
          left_frame);  // last 2 are for visualization
  VLOG(100) << "Feature selection completed.";

  // Same status as before.
  TrackerStatusSummary status = status_stereo_meas.first;
  return std::make_pair(status, trackedAndSelectedSmartStereoMeasurements);
}

/* -------------------------------------------------------------------------- */
void Pipeline::launchThreads() {
  LOG(INFO) << "Launching threads.";
  launchFrontendThread();
  launchRemainingThreads();
}

/* -------------------------------------------------------------------------- */
void Pipeline::launchFrontendThread() {
  if (parallel_run_) {
    // Start frontend_thread.
    frontend_thread_ = VIO::make_unique<std::thread>(
        &StereoVisionFrontEndModule::spin,
        CHECK_NOTNULL(vio_frontend_module_.get()));
    LOG(INFO) << "Frontend launched (parallel_run set to " << parallel_run_
              << ").";
  } else {
    LOG(INFO) << "Frontend running in sequential mode (parallel_run set to "
              << parallel_run_ << ").";
  }
}

/* -------------------------------------------------------------------------- */
void Pipeline::launchRemainingThreads() {
  if (parallel_run_) {
    backend_thread_ = VIO::make_unique<std::thread>(
        &VioBackEndModule::spin, CHECK_NOTNULL(vio_backend_module_.get()));

    mesher_thread_ = VIO::make_unique<std::thread>(
        &MesherModule::spin, CHECK_NOTNULL(mesher_module_.get()));

    if (lcd_module_) {
      lcd_thread_ = VIO::make_unique<std::thread>(
          &LcdModule::spin, CHECK_NOTNULL(lcd_module_.get()));
    }

    // TODO(Toni): visualizer thread is run in main thread.
    //// Start visualizer_thread.
    // if (visualizer_module_) {
    //  visualizer_thread_ = VIO::make_unique<std::thread>(
    //      &VisualizerModule::spin, CHECK_NOTNULL(visualizer_module_.get()));
    //}

    LOG(INFO) << "Backend, mesher and visualizer launched (parallel_run set to "
              << parallel_run_ << ").";
  } else {
    LOG(INFO) << "Backend, mesher and visualizer running in sequential mode"
              << " (parallel_run set to " << parallel_run_ << ").";
  }
}

/* -------------------------------------------------------------------------- */
// Resume all workers and queues
void Pipeline::resume() {
  LOG(INFO) << "Restarting frontend workers and queues...";
  stereo_frontend_input_queue_.resume();

  LOG(INFO) << "Restarting backend workers and queues...";
  backend_input_queue_.resume();

  // Re-launch threads
  /*if (parallel_run_) {
    launchThreads();
  } else {
    LOG(INFO) << "Running in sequential mode (parallel_run set to "
              << parallel_run_<< ").";
  }
  is_launched_ = true; */
}

/* -------------------------------------------------------------------------- */
void Pipeline::stopThreads() {
  LOG(INFO) << "Stopping workers and queues...";

  LOG(INFO) << "Stopping data provider module...";
  CHECK(data_provider_module_);
  data_provider_module_->shutdown();

  LOG(INFO) << "Stopping backend module and queues...";
  backend_input_queue_.shutdown();
  CHECK(vio_backend_module_);
  vio_backend_module_->shutdown();

  // Shutdown workers and queues.
  LOG(INFO) << "Stopping frontend module and queues...";
  stereo_frontend_input_queue_.shutdown();
  CHECK(vio_frontend_module_);
  vio_frontend_module_->shutdown();

  LOG(INFO) << "Stopping mesher module and queues...";
  if (mesher_module_) mesher_module_->shutdown();

  LOG(INFO) << "Stopping loop closure module and queues...";
  if (lcd_module_) lcd_module_->shutdown();

  LOG(INFO) << "Stopping visualizer module and queues...";
  if (visualizer_module_) visualizer_module_->shutdown();

  LOG(INFO) << "Sent stop flag to all module and queues...";
}

/* -------------------------------------------------------------------------- */
void Pipeline::joinThreads() {
  LOG_IF(WARNING, !parallel_run_)
      << "Asked to join threads while in sequential mode, this is ok, but "
      << "should not happen.";
  LOG(INFO) << "Joining threads...";

  if (backend_thread_) {
    LOG(INFO) << "Joining backend thread...";
    if (backend_thread_->joinable()) {
      backend_thread_->join();
      LOG(INFO) << "Joined backend thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Backend thread is not joinable...";
    }
  } else {
    VLOG(1) << "No Backend thread, not joining.";
  }

  if (frontend_thread_) {
    LOG(INFO) << "Joining frontend thread...";
    if (frontend_thread_->joinable()) {
      frontend_thread_->join();
      LOG(INFO) << "Joined frontend thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Frontend thread is not joinable...";
    }
  } else {
    VLOG(1) << "No Frontend thread, not joining.";
  }

  if (mesher_thread_) {
    LOG(INFO) << "Joining mesher thread...";
    if (mesher_thread_->joinable()) {
      mesher_thread_->join();
      LOG(INFO) << "Joined mesher thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Mesher thread is not joinable...";
    }
  } else {
  }

  if (lcd_thread_) {
    LOG(INFO) << "Joining loop closure thread...";
    if (lcd_thread_->joinable()) {
      lcd_thread_->join();
      LOG(INFO) << "Joined loop closure thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Loop closure thread is not joinable...";
    }
  } else {
    VLOG(1) << "No LCD thread, not joining.";
  }

  if (visualizer_thread_) {
    LOG(INFO) << "Joining visualizer thread...";
    if (visualizer_thread_->joinable()) {
      visualizer_thread_->join();
      LOG(INFO) << "Joined visualizer thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "visualizer thread is not joinable...";
    }
  } else {
    VLOG(1) << "No Visualizer thread, not joining.";
  }

  LOG(INFO) << "All threads joined.";
}

}  // namespace VIO
