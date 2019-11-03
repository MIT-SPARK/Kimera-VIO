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

#include "kimera-vio/backend/RegularVioBackEnd.h"
#include "kimera-vio/backend/VioBackEndFactory.h"
#include "kimera-vio/frontend/StereoVisionFrontEnd.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"

#include "kimera-vio/initial/InitializationBackEnd.h"
#include "kimera-vio/initial/InitializationFromImu.h"
#include "kimera-vio/initial/OnlineGravityAlignment.h"

DEFINE_bool(log_output, false, "Log output to CSV files.");
DEFINE_bool(extract_planes_from_the_scene, false,
            "Whether to use structural regularities in the scene,"
            "currently only planes.");

DEFINE_bool(visualize, true, "Enable overall visualization.");
DEFINE_bool(visualize_lmk_type, false, "Enable landmark type visualization.");
DEFINE_int32(viz_type, 0,
             "\n0: POINTCLOUD, visualize 3D VIO points (no repeated point)\n"
             "are re-plotted at every frame)\n"
             "1: MESH2D, only visualizes 2D mesh on image\n"
             "2: MESH2Dsparse, visualize a 2D mesh of (right-valid) keypoints "
             "discarding "
             "triangles corresponding to non planar obstacles\n"
             "3: MESH2DTo3Dsparse, get a 3D mesh from a 2D triangulation of "
             "the (right-VALID) "
             "keypoints in the left frame and filters out triangles \n"
             "4: NONE, does not visualize map\n");
DEFINE_bool(record_video_for_viz_3d, false,
            "Record a video as a sequence of "
            "screenshots of the 3d viz window");

DEFINE_bool(use_feature_selection, false, "Enable smart feature selection.");

DEFINE_bool(deterministic_random_number_generator, false,
            "If true the random number generator will consistently output the "
            "same sequence of pseudo-random numbers for every run (use it to "
            "have repeatable output). If false the random number generator "
            "will output a different sequence for each run.");
DEFINE_int32(min_num_obs_for_mesher_points, 4,
             "Minimum number of observations for a smart factor's landmark to "
             "to be used as a 3d point to consider for the mesher.");

DEFINE_int32(num_frames_vio_init, 25,
             "Minimum number of frames for the online "
             "gravity-aligned initialization.");

// TODO(Sandro): Create YAML file for initialization and read in!
DEFINE_double(smart_noise_sigma_bundle_adjustment, 1.5,
              "Smart noise sigma for bundle adjustment"
              " in initialization.");
DEFINE_double(outlier_rejection_bundle_adjustment, 30,
              "Outlier rejection for bundle adjustment"
              " in initialization.");
DEFINE_double(between_translation_bundle_adjustment, 0.5,
              "Between factor precision for bundle adjustment"
              " in initialization.");
DEFINE_int32(max_time_allowed_for_keyframe_callback,
             5u,
             "Maximum time allowed for processing keyframe rate callback "
             "(in ms).");

DEFINE_bool(use_lcd, false,
            "Enable LoopClosureDetector processing in pipeline.");

namespace VIO {

Pipeline::Pipeline(const PipelineParams& params)
    : backend_type_(static_cast<BackendType>(params.backend_type_)),
      stereo_camera_(nullptr),
      vio_frontend_module_(nullptr),
      feature_selector_(nullptr),
      vio_backend_module_(nullptr),
      loop_closure_detector_(nullptr),
      backend_params_(params.backend_params_),
      frontend_params_(params.frontend_params_),
      imu_params_(params.imu_params_),
      mesher_(nullptr),
      visualizer_(nullptr),
      stereo_frontend_thread_(nullptr),
      wrapped_thread_(nullptr),
      backend_thread_(nullptr),
      mesher_thread_(nullptr),
      parallel_run_(params.parallel_run_),
      stereo_frontend_input_queue_("stereo_frontend_input_queue"),
      stereo_frontend_output_queue_("stereo_frontend_output_queue"),
      initialization_frontend_output_queue_(
          "initialization_frontend_output_queue"),
      backend_input_queue_("backend_input_queue"),
      backend_output_queue_("backend_output_queue"),
      mesher_input_queue_("mesher_input_queue"),
      mesher_output_queue_("mesher_output_queue"),
      lcd_input_queue_("lcd_input_queue"),
      null_lcd_output_queue_("null_lcd_output_queue"),
      visualizer_input_queue_("visualizer_input_queue"),
      null_visualizer_output_queue_("visualizer_output_queue") {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();

  //! Create Stereo Camera
  CHECK_EQ(params.camera_params_.size(), 2u) << "Only stereo camera support.";
  stereo_camera_ = VIO::make_unique<StereoCamera>(
      params.camera_params_.at(0),
      params.camera_params_.at(1),
      params.frontend_params_.stereo_matching_params_);

  // Instantiate stereo tracker (class that tracks implements estimation
  // front-end) and print parameters.
  vio_frontend_module_ = VIO::make_unique<StereoVisionFrontEndModule>(
      &stereo_frontend_input_queue_,
      &stereo_frontend_output_queue_,
      parallel_run_,
      FrontEndFactory::createFrontend(params.frontend_type_,
                                      params.imu_params_,
                                      gtsam::imuBias::ConstantBias(),
                                      params.frontend_params_,
                                      FLAGS_log_output));

  // These two should be given by parameters...
  vio_backend_module_ = VIO::make_unique<VioBackEndModule>(
      &backend_input_queue_,
      &backend_output_queue_,
      parallel_run_,
      BackEndFactory::createBackend(backend_type_,
                                    stereo_camera_->getLeftCamPose(),
                                    stereo_camera_->getStereoCalib(),
                                    *CHECK_NOTNULL(backend_params_),
                                    FLAGS_log_output));
  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&StereoVisionFrontEndModule::updateImuBias,
                // Send a cref: constant reference because vio_frontend_ is
                // not copyable.
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  // TODO(Toni): only create if used.
  mesher_ = VIO::make_unique<Mesher>(
      &mesher_input_queue_, &mesher_output_queue_, parallel_run_);

  // TODO(Toni): only create if used.
  visualizer_ = VIO::make_unique<Visualizer3D>(
      &visualizer_input_queue_,
      &null_visualizer_output_queue_,
      // TODO(Toni): bundle these three params in VisualizerParams...
      static_cast<VisualizationType>(FLAGS_viz_type),
      backend_type_,
      std::bind(&Pipeline::spinDisplayOnce, this, std::placeholders::_1),
      parallel_run_);

  if (FLAGS_use_lcd) {
    loop_closure_detector_ =
        VIO::make_unique<LoopClosureDetector>(&lcd_input_queue_,
                                              &null_lcd_output_queue_,
                                              params.lcd_params_,
                                              parallel_run_,
                                              FLAGS_log_output);
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
void Pipeline::spin(
    StereoImuSyncPacket::ConstUniquePtr stereo_imu_sync_packet) {
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
    // TODO Warning: we do not accumulate IMU measurements for the first
    // packet... Spin.
    spinOnce(std::move(stereo_imu_sync_packet));
  }
  return;
}

/* -------------------------------------------------------------------------- */
// Spin the pipeline only once.
void Pipeline::spinOnce(
    StereoImuSyncPacket::ConstUniquePtr stereo_imu_sync_packet) {
  CHECK(is_initialized_);
  ////////////////////////////// FRONT-END /////////////////////////////////////
  // Push to stereo frontend input queue.
  VLOG(2) << "Push input payload to Frontend.";
  stereo_frontend_input_queue_.push(std::move(stereo_imu_sync_packet));

  // Run the pipeline sequentially.
  if (!parallel_run_) spinSequential();
}

/* -------------------------------------------------------------------------- */
void Pipeline::processKeyframe(
    const StatusStereoMeasurements& status_stereo_measurements,
    const StereoFrame& last_stereo_keyframe,
    const ImuFrontEnd::PreintegratedImuMeasurements& pim,
    const TrackingStatus& kf_tracking_status_stereo,
    const gtsam::Pose3& relative_pose_body_stereo,
    const DebugTrackerInfo&
        debug_tracker_info) {  // Only for output of pipeline
  //////////////////// BACK-END ////////////////////////////////////////////////
  // Push to backend input.
  // This should be done inside the frontend!!!!
  // Or the backend should pull from the frontend!!!!
  VLOG(2) << "Push input payload to Backend.";
  backend_input_queue_.push(VIO::make_unique<VioBackEndInputPayload>(
      last_stereo_keyframe.getTimestamp(),
      status_stereo_measurements,
      kf_tracking_status_stereo,
      pim,
      relative_pose_body_stereo,
      &planes_));

  // This should be done inside those who need the backend results
  // IN this case the logger!!!!!
  // But there are many more people that want backend results...
  // Pull from backend.
  VLOG(2) << "Waiting payload from Backend.";
  VioBackEndOutputPayload::UniquePtr backend_output_payload;
  backend_output_queue_.popBlocking(backend_output_payload);
  LOG_IF(WARNING, !backend_output_payload) << "Missing backend output payload.";

  // Optionally, push to lcd input.
  // TODO(marcus): not robust to cases where frontend and backend processing
  // happen in different methods. This only works if they're done one after
  // another. Mixing input from two different outputs is bad form.
  // It is unfortunately the only option in this architecture.
  if (FLAGS_use_lcd) {
    VLOG(2) << "Push input payload to LoopClosureDetector.";
    lcd_input_queue_.push(
        std::move(VIO::make_unique<LoopClosureDetectorInputPayload>(
            last_stereo_keyframe.getTimestamp(),
            backend_output_payload->cur_kf_id_,
            last_stereo_keyframe,
            backend_output_payload->W_State_Blkf_.pose_)));
  }

  ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////////
  PointsWithIdMap points_with_id_VIO;
  LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
  MesherOutputPayload::UniquePtr mesher_output_payload;
  VisualizationType visualization_type =
      static_cast<VisualizationType>(FLAGS_viz_type);
  // Compute 3D mesh
  if (visualization_type == VisualizationType::MESH2DTo3Dsparse) {
    // Create and fill data packet for mesher.
    // Points_with_id_VIO contains all the points in the optimization,
    // (encoded as either smart factors or explicit values), potentially
    // restricting to points seen in at least min_num_obs_fro_mesher_points
    // keyframes (TODO restriction is not enforced for projection factors).
    // THREAD_SAFE only as long as this is running in the backend/wrapped
    // thread!!!
    // TODO(Toni) Ideally, should be sent along backend output payload, but
    // it is computationally expensive to compute.
    points_with_id_VIO =
        vio_backend_module_
            ->getMapLmkIdsTo3dPointsInTimeHorizon(  // not thread-safe
                FLAGS_visualize_lmk_type ? &lmk_id_to_lmk_type_map : nullptr,
                FLAGS_min_num_obs_for_mesher_points);  // copy, thread safe,
                                                       // read-only. // This
                                                       // should be a
                                                       // popBlocking...
    // Push to queue.
    // In another thread, mesher is running, consuming mesher payloads.
    VLOG(2) << "Push input payload to Mesher.";
    mesher_input_queue_.push(VIO::make_unique<MesherInputPayload>(
        points_with_id_VIO,
        last_stereo_keyframe,  // not really thread safe, read only.
        backend_output_payload->W_State_Blkf_.pose_.compose(
            last_stereo_keyframe
                .getBPoseCamLRect())));  // TODO(Toni) this is constant and
                                         // should not be sent via output
                                         // payload.

    // In the mesher thread push queue with meshes for visualization.
    // Use blocking to avoid skipping frames.
    VLOG(2) << "Waiting payload from Mesher.";
    LOG_IF(WARNING, !mesher_output_queue_.popBlocking(mesher_output_payload))
        << "Mesher output queue did not pop a payload.";

    // Do this after popBlocking from Mesher so we do it sequentially, since
    // planes_ are not thread-safe.
    // Find regularities in the mesh if we are using RegularVIO backend.
    // TODO create a new class that is mesh segmenter or plane extractor.
    // This is NOT THREAD_SAFE, do it after popBlocking the mesher...
    if (FLAGS_extract_planes_from_the_scene) {
      // Use Regular VIO
      CHECK(backend_type_ == BackendType::StructuralRegularities);
      mesher_->clusterPlanesFromMesh(&planes_, points_with_id_VIO);
    } else {
      RegularVioBackEndParams regular_vio_backend_params =
          RegularVioBackEndParams::safeCast(*backend_params_);
      LOG_IF_EVERY_N(
          WARNING,
          backend_type_ == BackendType::StructuralRegularities &&
              (regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::STRUCTURELESS_AND_PROJECTION ||
               regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::PROJECTION_AND_REGULARITY ||
               regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::
                       STRUCTURELESS_PROJECTION_AND_REGULARITY),
          10)
          << "Using Regular VIO without extracting planes from the scene. "
             "Set flag extract_planes_from_the_scene to true to enforce "
             "regularities.";
    }
  }

  if (keyframe_rate_output_callback_) {
    auto tic = utils::Timer::tic();
    VLOG(2) << "Call keyframe callback with spin output payload.";
    keyframe_rate_output_callback_(
        SpinOutputPacket(backend_output_payload->W_State_Blkf_,
                         mesher_output_payload->mesh_2d_,
                         mesher_output_payload->mesh_3d_,
                         Visualizer3D::visualizeMesh2D(
                             mesher_output_payload->mesh_2d_filtered_for_viz_,
                             last_stereo_keyframe.getLeftFrame().img_),
                         points_with_id_VIO,
                         lmk_id_to_lmk_type_map,
                         backend_output_payload->state_covariance_lkf_,
                         debug_tracker_info));
    auto toc = utils::Timer::toc(tic);
    LOG_IF(WARNING, toc.count() > FLAGS_max_time_allowed_for_keyframe_callback)
        << "Keyframe Rate Output Callback is taking longer than it should: "
           "make sure your callback is fast!";
  }

  if (FLAGS_visualize) {
    // Push data for visualizer thread.
    // WHO Should be pushing to the visualizer input queue????????
    // This cannot happen at all from a single module, because visualizer
    // takes input from mesher and backend right now...
    VLOG(2) << "Push input payload to Visualizer.";
    visualizer_input_queue_.push(VIO::make_unique<VisualizerInputPayload>(
        // Pose for trajectory viz.
        backend_output_payload->W_State_Blkf_.pose_ *
            last_stereo_keyframe
                .getBPoseCamLRect(),  // This should be pass at ctor level...
        // For visualizeMesh2D and visualizeMesh2DStereo.
        last_stereo_keyframe,
        // visualizeConvexHull & visualizeMesh3DWithColoredClusters
        std::move(mesher_output_payload),
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        visualization_type == VisualizationType::POINTCLOUD
            ? vio_backend_module_
                  ->getMapLmkIdsTo3dPointsInTimeHorizon()  // not thread-safe
            : points_with_id_VIO,
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        lmk_id_to_lmk_type_map,
        planes_,  // visualizeMesh3DWithColoredClusters
        vio_backend_module_->getFactorsUnsafe(),  // For plane constraints viz.
                                                  // // not thread-safe
        backend_output_payload->state_  // For planes and plane constraints viz.
        ));
  }
}

// Returns whether the visualizer_ is running or not. While in parallel mode,
// it does not return unless shutdown.
bool Pipeline::spinViz() {
  if (FLAGS_visualize) {
    CHECK(visualizer_);
    return visualizer_->spin();
  }
  return true;
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinSequential() {
  // Spin once frontend.
  CHECK(vio_frontend_module_);
  vio_frontend_module_->spin();

  // Pop from frontend.
  StereoFrontEndOutputPayload::UniquePtr stereo_frontend_output_payload;
  stereo_frontend_output_queue_.pop(stereo_frontend_output_payload);
  if (!stereo_frontend_output_payload) {
    // Frontend hasn't reach a keyframe, return and wait frontend to create a
    // keyframe.
    return;
  }
  CHECK(stereo_frontend_output_payload->is_keyframe_);

  // We have a keyframe. Push to backend.
  backend_input_queue_.push(VIO::make_unique<VioBackEndInputPayload>(
      stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp(),
      stereo_frontend_output_payload->status_stereo_measurements_,
      stereo_frontend_output_payload->tracker_status_,
      stereo_frontend_output_payload->pim_,
      stereo_frontend_output_payload->relative_pose_body_stereo_,
      &planes_));

  // Spin once backend. Do not run in parallel.
  CHECK(vio_backend_module_);
  vio_backend_module_->spin();

  // Pop blocking from backend.
  VioBackEndOutputPayload::UniquePtr backend_output_payload;
  CHECK(backend_output_queue_.popBlocking(backend_output_payload));

  // Push keyframe to LCD.
  lcd_input_queue_.push(VIO::make_unique<LoopClosureDetectorInputPayload>(
      stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp(),
      backend_output_payload->cur_kf_id_,
      stereo_frontend_output_payload->stereo_frame_lkf_,
      backend_output_payload->W_State_Blkf_.pose_));

  // Spin once LCD. Do not run in parallel.
  if (FLAGS_use_lcd) {
    CHECK(loop_closure_detector_);
    loop_closure_detector_->spin();
  }

  const auto& stereo_keyframe =
      stereo_frontend_output_payload->stereo_frame_lkf_;
  ////////////////// CREATE 3D MESH //////////////////////////////////////////
  PointsWithIdMap points_with_id_VIO;
  LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
  MesherOutputPayload::UniquePtr mesher_output_payload;
  VisualizationType visualization_type =
      static_cast<VisualizationType>(FLAGS_viz_type);
  if (visualization_type == VisualizationType::MESH2DTo3Dsparse) {
    // Push to mesher.
    points_with_id_VIO =
        vio_backend_module_->getMapLmkIdsTo3dPointsInTimeHorizon(
            FLAGS_visualize_lmk_type ? &lmk_id_to_lmk_type_map : nullptr,
            FLAGS_min_num_obs_for_mesher_points);  // copy, thread safe,
                                                   // read-only. // This should
                                                   // be a popBlocking...
    // Push to queue.
    // In another thread, mesher is running, consuming mesher payloads.
    CHECK(mesher_input_queue_.push(VIO::make_unique<MesherInputPayload>(
        points_with_id_VIO,
        stereo_keyframe,  // not really thread safe, read only.
        backend_output_payload->W_State_Blkf_.pose_.compose(
            stereo_keyframe.getBPoseCamLRect()))));

    // Spin once mesher.
    mesher_->spin();

    // Find regularities in the mesh if we are using RegularVIO backend.
    // TODO create a new class that is mesh segmenter or plane extractor.
    if (FLAGS_extract_planes_from_the_scene) {
      CHECK(backend_type_ == BackendType::StructuralRegularities);
      mesher_->clusterPlanesFromMesh(&planes_, points_with_id_VIO);
    } else {
      RegularVioBackEndParams regular_vio_backend_params =
          RegularVioBackEndParams::safeCast(*backend_params_);
      LOG_IF_EVERY_N(
          WARNING,
          backend_type_ == BackendType::StructuralRegularities &&
              (regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::STRUCTURELESS_AND_PROJECTION ||
               regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::PROJECTION_AND_REGULARITY ||
               regular_vio_backend_params.backend_modality_ ==
                   RegularBackendModality::
                       STRUCTURELESS_PROJECTION_AND_REGULARITY),
          10)
          << "Using Regular VIO without extracting planes from the scene. "
             "Set flag extract_planes_from_the_scene to true to enforce "
             "regularities.";
    }

    // Pop from mesher.
    LOG_IF(WARNING, !mesher_output_queue_.popBlocking(mesher_output_payload))
        << "Mesher output queue did not pop a payload.";
  }
  ////////////////////////////////////////////////////////////////////////////

  if (keyframe_rate_output_callback_) {
    auto tic = utils::Timer::tic();
    VLOG(2) << "Call keyframe callback with spin output payload.";
    keyframe_rate_output_callback_(SpinOutputPacket(
        backend_output_payload->W_State_Blkf_,
        mesher_output_payload->mesh_2d_,
        mesher_output_payload->mesh_3d_,
        Visualizer3D::visualizeMesh2D(
            mesher_output_payload->mesh_2d_filtered_for_viz_,
            stereo_frontend_output_payload->stereo_frame_lkf_.getLeftFrame()
                .img_),
        points_with_id_VIO,
        lmk_id_to_lmk_type_map,
        backend_output_payload->state_covariance_lkf_,
        stereo_frontend_output_payload->debug_tracker_info_));
    auto toc = utils::Timer::toc(tic);
    LOG_IF(WARNING, toc.count() > FLAGS_max_time_allowed_for_keyframe_callback)
        << "Keyframe Rate Output Callback is taking longer than it should: "
           "make sure your callback is fast!";
  }

  if (FLAGS_visualize) {
    // Push data for visualizer thread.
    // WHO Should be pushing to the visualizer input queue????????
    // This cannot happen at all from a single module, because visualizer
    // takes input from mesher and backend right now...
    visualizer_input_queue_.push(VIO::make_unique<VisualizerInputPayload>(
        // Pose for trajectory viz.
        backend_output_payload->W_State_Blkf_
                .pose_ *  // The visualizer needs backend results
            stereo_keyframe
                .getBPoseCamLRect(),  // This should be pass at ctor level....
        // For visualizeMesh2D and visualizeMesh2DStereo.
        stereo_keyframe,
        // visualizeConvexHull & visualizeMesh3DWithColoredClusters
        // WARNING using move explicitly!
        std::move(mesher_output_payload),
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        visualization_type == VisualizationType::POINTCLOUD
            ?  // This is to avoid recomputing MapLmkIdsTo3d...
            vio_backend_module_->getMapLmkIdsTo3dPointsInTimeHorizon()
            : points_with_id_VIO,
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        lmk_id_to_lmk_type_map,
        planes_,  // visualizeMesh3DWithColoredClusters
        vio_backend_module_->getFactorsUnsafe(),  // For plane constraints viz.
        backend_output_payload->state_  // For planes and plane constraints viz.
        ));

    // Spin visualizer.
    CHECK(visualizer_);
    visualizer_->spin();
  }
}

// TODO: Adapt this function to be able to cope with new initialization
/* -------------------------------------------------------------------------- */
void Pipeline::shutdownWhenFinished() {
  // This is a very rough way of knowing if we have finished...
  // Since threads might be in the middle of processing data while we
  // query if the queues are empty.
  // Check every second if all queues are empty.
  // Time to sleep between queries to the queues [in seconds].
  LOG(INFO) << "Shutting down VIO pipeline once processing has finished.";
  static constexpr int sleep_time = 1;

  while (
      !shutdown_ &&         // Loop while not explicitly shutdown.
      (!is_initialized_ ||  // Loop while not initialized
                            // Or, once init, data is not yet consumed.
       !(stereo_frontend_input_queue_.empty() &&
         stereo_frontend_output_queue_.empty() &&
         !vio_frontend_module_->isWorking() && backend_input_queue_.empty() &&
         backend_output_queue_.empty() && !vio_backend_module_->isWorking() &&
         mesher_input_queue_.empty() && mesher_output_queue_.empty() &&
         (mesher_ ? !mesher_->isWorking() : true) && lcd_input_queue_.empty() &&
         (loop_closure_detector_ ? !loop_closure_detector_->isWorking()
                                 : true) &&
         visualizer_input_queue_.empty() &&
         null_visualizer_output_queue_.empty() &&
         (visualizer_ ? !visualizer_->isWorking() : true)))) {
    VLOG_EVERY_N(10, 100) << "shutdown_: " << shutdown_ << '\n'
                          << "VIO pipeline status: \n"
                          << "Initialized? " << is_initialized_ << '\n'
                          << "Frontend input queue empty?"
                          << stereo_frontend_input_queue_.empty() << '\n'
                          << "Frontend output queue empty?"
                          << stereo_frontend_output_queue_.empty() << '\n'
                          << "Frontend is working? "
                          << vio_frontend_module_->isWorking() << '\n'
                          << "Backend Input queue empty?"
                          << backend_input_queue_.empty() << '\n'
                          << "Backend Output queue empty?"
                          << backend_output_queue_.empty() << '\n'
                          << "Backend is working? "
                          << (is_initialized_ ? vio_backend_module_->isWorking()
                                              : false);

    VLOG_IF_EVERY_N(10, mesher_, 100)
        << "Mesher input queue empty?" << mesher_input_queue_.empty() << '\n'
        << "Mesher output queue empty?" << mesher_output_queue_.empty() << '\n'
        << "Mesher is working? " << mesher_->isWorking();

    VLOG_IF_EVERY_N(10, loop_closure_detector_, 100)
        << "LoopClosureDetector input queue empty?" << lcd_input_queue_.empty()
        << '\n'
        << "LoopClosureDetector output queue empty?"
        << null_lcd_output_queue_.empty() << '\n'
        << "LoopClosureDetector is working? "
        << loop_closure_detector_->isWorking();

    // NOLINTNEXTLINE
    VLOG_IF_EVERY_N(10, visualizer_, 100)
        << "Visualizer input queue empty?" << visualizer_input_queue_.empty()
        << '\n'
        << "Visualizer output queue empty?"
        << null_visualizer_output_queue_.empty() << '\n'
        << "Visualizer is working? " << visualizer_->isWorking();  // NOLINT

    std::this_thread::sleep_for(std::chrono::seconds(sleep_time));
  }
  LOG(INFO) << "Shutting down VIO, reason: input is empty and threads are "
               "idle.";
  if (!shutdown_) shutdown();
}

/* -------------------------------------------------------------------------- */
void Pipeline::shutdown() {
  LOG_IF(ERROR, shutdown_) << "Shutdown requested, but Pipeline was already "
                              "shutdown.";
  LOG(INFO) << "Shutting down VIO pipeline.";
  shutdown_ = true;
  stopThreads();
  // if (parallel_run_) {
  joinThreads();
  //}
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
    mesher_->restart();
    if (loop_closure_detector_) loop_closure_detector_->restart();
    visualizer_->restart();
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
          backend_params_->n_gravity_,
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
  gtsam::PreintegratedImuMeasurements::Params imu_params =
      ImuFrontEnd::convertImuParams(imu_params_);
  imu_params.n_gravity = backend_params_->n_gravity_;
  ImuFrontEnd imu_frontend_real(
      imu_params,
      gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
  CHECK_DOUBLE_EQ(imu_frontend_real.getPreintegrationGravity().norm(),
                  imu_params.n_gravity.norm());

  // Enforce stereo frame as keyframe for initialization
  StereoFrame stereo_frame = stereo_imu_sync_packet.getStereoFrame();
  stereo_frame.setIsKeyframe(true);
  // TODO: this is copying the packet implicitly, just to set a flag to true.
  StereoImuSyncPacket stereo_imu_sync_init(
      stereo_frame,
      stereo_imu_sync_packet.getImuStamps(),
      stereo_imu_sync_packet.getImuAccGyr(),
      stereo_imu_sync_packet.getReinitPacket());

  /////////////////// FIRST FRAME //////////////////////////////////////////////
  if (frame_id == init_frame_id_) {
    // Set trivial bias, gravity and force 5/3 point method for initialization
    vio_frontend_module_->prepareFrontendForOnlineAlignment();
    // Initialize Stereo Frontend.
    vio_frontend_module_->processFirstStereoFrame(
        stereo_imu_sync_init.getStereoFrame());
    return false;
  } else {
    // Check trivial bias and gravity vector for online initialization
    vio_frontend_module_->checkFrontendForOnlineAlignment();
    // Spin frontend once with enforced keyframe and 53-point method
    const StereoFrontEndOutputPayload::UniquePtr& frontend_output =
        vio_frontend_module_->spinOnce(stereo_imu_sync_init);
    // TODO(Sandro): Optionally add AHRS PIM
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
    const auto& imu_stamps = stereo_imu_sync_packet.getImuStamps();
    const auto& imu_accgyr = stereo_imu_sync_packet.getImuAccGyr();
    const auto& pim =
        imu_frontend_real.preintegrateImuMeasurements(imu_stamps, imu_accgyr);
    // This queue is used for the backend after initialization
    VLOG(2) << "Initialization: Push input payload to Backend.";
    stereo_frontend_output_queue_.push(
        VIO::make_unique<StereoFrontEndOutputPayload>(
            frontend_output->is_keyframe_,
            frontend_output->status_stereo_measurements_,
            frontend_output->tracker_status_,
            frontend_output->relative_pose_body_stereo_,
            frontend_output->stereo_frame_lkf_,
            pim,
            frontend_output->debug_tracker_info_));

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
      InitializationBackEnd initial_backend(stereo_camera_->getLeftCamPose(),
                                            stereo_camera_->getStereoCalib(),
                                            backend_params_init,
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
            backend_params_->n_gravity_, gyro_bias);
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
        stereo_frontend_output_queue_.shutdown();
        initialization_frontend_output_queue_.shutdown();
        stereo_frontend_output_queue_.resume();
        initialization_frontend_output_queue_.resume();
        return false;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinDisplayOnce(
    VisualizerOutputPayload& visualizer_output_payload) {
  // Display 3D window.
  if (visualizer_output_payload.visualization_type_ !=
      VisualizationType::NONE) {
    VLOG(10) << "Spin Visualize 3D output.";
    // visualizer_output_payload->window_.spin();
    CHECK(!visualizer_output_payload.window_.wasStopped());
    visualizer_output_payload.window_.spinOnce(1, true);
    // TODO this is not very thread-safe!!! Since recordVideo might modify
    // window_ in this thread, while it might also be called in viz thread.
    if (FLAGS_record_video_for_viz_3d) {
      visualizer_->recordVideo();
    }
  }

  // Display 2D images.
  for (const ImageToDisplay& img_to_display :
       visualizer_output_payload.images_to_display_) {
    cv::imshow(img_to_display.name_, img_to_display.image_);
  }
  VLOG(10) << "Spin Visualize 2D output.";
  cv::waitKey(1);
}

/* -------------------------------------------------------------------------- */
StatusStereoMeasurements Pipeline::featureSelect(
    const VioFrontEndParams& tracker_params,
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
  size_t nrKfInHorizon = round(tracker_params.featureSelectionHorizon_ /
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
          tracker_params.featureSelectionCriterion_,
          tracker_params.featureSelectionNrCornersToSelect_,
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
void Pipeline::processKeyframePop() {
  // TODO (Sandro): Adapt to be able to batch pop frames for batch backend
  // Pull from stereo frontend output queue.
  LOG(INFO) << "Spinning wrapped thread.";
  while (!shutdown_) {
    // Here we are inside the WRAPPED THREAD //
    VLOG(2) << "Waiting payload from Frontend.";
    StereoFrontEndOutputPayload::UniquePtr stereo_frontend_output_payload;
    stereo_frontend_output_queue_.popBlocking(stereo_frontend_output_payload);
    if (!stereo_frontend_output_payload) {
      LOG(WARNING) << "Missing frontend output payload.";
      continue;
    }
    CHECK(stereo_frontend_output_payload->is_keyframe_);

    ////////////////////////////////////////////////////////////////////////////
    // So from this point on, we have a keyframe.
    // Pass info to VIO
    // Actual keyframe processing. Call to backend.
    ////////////////////////////// BACK-END
    ///////////////////////////////////////
    VLOG(2) << "Process Keyframe in BackEnd";
    // TODO(Toni): There is a potential 800 element vector copying to pass
    // the visual feature tracks between frontend and backend!
    processKeyframe(stereo_frontend_output_payload->status_stereo_measurements_,
                    stereo_frontend_output_payload->stereo_frame_lkf_,
                    stereo_frontend_output_payload->pim_,
                    stereo_frontend_output_payload->tracker_status_,
                    stereo_frontend_output_payload->relative_pose_body_stereo_,
                    stereo_frontend_output_payload->debug_tracker_info_);
  }
  LOG(INFO) << "Shutdown wrapped thread.";
}

/* --------------------------------------------------------------------------
 */
void Pipeline::launchThreads() {
  LOG(INFO) << "Launching threads.";
  launchFrontendThread();
  launchRemainingThreads();
}

/* -------------------------------------------------------------------------- */
void Pipeline::launchFrontendThread() {
  if (parallel_run_) {
    // Start frontend_thread.
    stereo_frontend_thread_ = VIO::make_unique<std::thread>(
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
    wrapped_thread_ =
        VIO::make_unique<std::thread>(&Pipeline::processKeyframePop, this);

    backend_thread_ = VIO::make_unique<std::thread>(
        &VioBackEndModule::spin, CHECK_NOTNULL(vio_backend_module_.get()));

    mesher_thread_ = VIO::make_unique<std::thread>(
        &Mesher::spin, CHECK_NOTNULL(mesher_.get()));

    if (FLAGS_use_lcd) {
      lcd_thread_ = VIO::make_unique<std::thread>(
          &LoopClosureDetector::spin,
          CHECK_NOTNULL(loop_closure_detector_.get()));
    }

    // Start visualizer_thread.
    // visualizer_thread_ = std::thread(&Visualizer3D::spin,
    //                                 &visualizer_,
    //                                 std::ref(visualizer_input_queue_),
    //                                 std::ref(visualizer_output_queue_));
    LOG(INFO) << "Backend, mesher and visualizer launched (parallel_run set to "
              << parallel_run_ << ").";
  } else {
    LOG(INFO) << "Backend, mesher and visualizer running in sequential mode"
              << " (parallel_run set to " << parallel_run_ << ").";
  }
}

/* --------------------------------------------------------------------------
 */
// Resume all workers and queues
void Pipeline::resume() {
  LOG(INFO) << "Restarting frontend workers and queues...";
  stereo_frontend_input_queue_.resume();
  stereo_frontend_output_queue_.resume();

  LOG(INFO) << "Restarting backend workers and queues...";
  backend_input_queue_.resume();
  backend_output_queue_.resume();

  LOG(INFO) << "Restarting mesher workers and queues...";
  mesher_input_queue_.resume();
  mesher_output_queue_.resume();

  LOG(INFO) << "Restarting loop closure workers and queues...";
  lcd_input_queue_.resume();

  LOG(INFO) << "Restarting visualizer workers and queues...";
  visualizer_input_queue_.resume();
  null_visualizer_output_queue_.resume();

  // Re-launch threads
  /*if (parallel_run_) {
    launchThreads();
  } else {
    LOG(INFO) << "Running in sequential mode (parallel_run set to "
              << parallel_run_<< ").";
  }
  is_launched_ = true; */
}

/* --------------------------------------------------------------------------
 */
void Pipeline::stopThreads() {
  LOG(INFO) << "Stopping workers and queues...";

  LOG(INFO) << "Stopping backend workers and queues...";
  backend_input_queue_.shutdown();
  backend_output_queue_.shutdown();
  CHECK(vio_backend_module_);
  vio_backend_module_->shutdown();

  // Shutdown workers and queues.
  LOG(INFO) << "Stopping frontend workers and queues...";
  stereo_frontend_input_queue_.shutdown();
  stereo_frontend_output_queue_.shutdown();
  CHECK(vio_frontend_module_);
  vio_frontend_module_->shutdown();

  LOG(INFO) << "Stopping mesher workers and queues...";
  mesher_input_queue_.shutdown();
  mesher_output_queue_.shutdown();
  if (mesher_) mesher_->shutdown();

  LOG(INFO) << "Stopping loop closure workers and queues...";
  lcd_input_queue_.shutdown();
  if (loop_closure_detector_) loop_closure_detector_->shutdown();

  LOG(INFO) << "Stopping visualizer workers and queues...";
  visualizer_input_queue_.shutdown();
  null_visualizer_output_queue_.shutdown();
  if (visualizer_) visualizer_->shutdown();

  LOG(INFO) << "Sent stop flag to all workers and queues...";
}

/* -------------------------------------------------------------------------- */
void Pipeline::joinThreads() {
  LOG(INFO) << "Joining threads...";

  LOG(INFO) << "Joining backend thread...";
  if (backend_thread_ && backend_thread_->joinable()) {
    backend_thread_->join();
    LOG(INFO) << "Joined backend thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Backend thread is not joinable...";
  }

  LOG(INFO) << "Joining frontend thread...";
  if (stereo_frontend_thread_ && stereo_frontend_thread_->joinable()) {
    stereo_frontend_thread_->join();
    LOG(INFO) << "Joined frontend thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Frontend thread is not joinable...";
  }

  LOG(INFO) << "Joining wrapped thread...";
  if (wrapped_thread_ && wrapped_thread_->joinable()) {
    wrapped_thread_->join();
    LOG(INFO) << "Joined wrapped thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Wrapped thread is not joinable...";
  }

  LOG(INFO) << "Joining mesher thread...";
  if (mesher_thread_ && mesher_thread_->joinable()) {
    mesher_thread_->join();
    LOG(INFO) << "Joined mesher thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Mesher thread is not joinable...";
  }

  LOG(INFO) << "Joining loop closure thread...";
  if (lcd_thread_) {
    if (lcd_thread_->joinable()) {
      lcd_thread_->join();
      LOG(INFO) << "Joined loop closure thread...";
    } else {
      LOG_IF(ERROR, parallel_run_) << "Loop closure thread is not joinable...";
    }
  } else {
    LOG(WARNING) << "No lcd thread registered.";
  }

  // visualizer_thread_.join();

  LOG(INFO) << "All threads joined.";
}

}  // namespace VIO
