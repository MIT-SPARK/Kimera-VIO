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

#include "pipeline/Pipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <future>

#include "RegularVioBackEnd.h"
#include "StereoVisionFrontEnd.h"
#include "utils/Statistics.h"
#include "utils/Timer.h"

#include "OnlineGravityAlignment.h"

DEFINE_bool(log_output, false, "Log output to matlab.");
DEFINE_int32(regular_vio_backend_modality, 4u,
             "Modality for regular Vio backend, currently supported:\n"
             "0: Structureless (equiv to normal VIO)\n"
             "1: Projection (as if it was a typical VIO backend with projection"
             "factors\n"
             "2: Structureless and projection, sets to projection factors the "
             "structureless factors that are supposed to be in a regularity.\n"
             "3: Projection and regularity, sets all structureless factors to"
             "projection factors and adds regularity factors to a subset.\n"
             "4: structureless, projection and regularity factors used.");
DEFINE_bool(extract_planes_from_the_scene, false,
            "Whether to use structural regularities in the scene,"
            "currently only planes");

DEFINE_bool(visualize, true, "Enable overall visualization.");
DEFINE_bool(visualize_lmk_type, false, "Enable landmark type visualization.");
DEFINE_int32(
    viz_type, 0,
    "\n0: POINTCLOUD, visualize 3D VIO points (no repeated point)\n"
    "1: POINTCLOUD_REPEATEDPOINTS, visualize VIO points as point clouds "
    "(points "
    "are re-plotted at every frame)\n"
    "2: MESH2D, only visualizes 2D mesh on image\n"
    "3: MESH2Dsparse, visualize a 2D mesh of (right-valid) keypoints "
    "discarding "
    "triangles corresponding to non planar obstacles\n"
    "4: MESH2DTo3D, get a 3D mesh from a 2D triangulation of the (right-VALID) "
    "keypoints in the left frame\n"
    "5: MESH2DTo3Dsparse, same as MESH2DTo3D but filters out triangles "
    "corresponding to non planar obstacles\n"
    "6: NONE, does not visualize map\n");
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
             "to be used as a 3d point to consider for the mesher");

DEFINE_int32(numb_frames_oga, 15,
             "Minimum number of frames for the online "
             "gravity-aligned initialization");
DEFINE_int32(initialization_mode, 1,
             "Initialization mode to be choosen "
             "0: default (GT/IMU), 1: online initialisation");

namespace VIO {

// TODO VERY BAD TO SEND THE DATASET PARSER AS A POINTER (at least for thread
// safety!), but this is done now because the logger heavily relies on the
// dataset parser, especially the grount-truth! Feature selector also has some
// dependency with this...
Pipeline::Pipeline(ETHDatasetParser* dataset, const ImuParams& imu_params,
                   bool parallel_run)
    : dataset_(CHECK_NOTNULL(dataset)),
      vio_frontend_(nullptr),
      vio_backend_(nullptr),
      mesher_(),
      visualizer_(static_cast<VisualizationType>(FLAGS_viz_type),
                  dataset->getBackendType()),
      stereo_frontend_thread_(nullptr),
      wrapped_thread_(nullptr),
      backend_thread_(nullptr),
      mesher_thread_(nullptr),
      parallel_run_(parallel_run),
      stereo_frontend_input_queue_("stereo_frontend_input_queue"),
      stereo_frontend_output_queue_("stereo_frontend_output_queue"),
      backend_input_queue_("backend_input_queue"),
      backend_output_queue_("backend_output_queue"),
      mesher_input_queue_("mesher_input_queue"),
      mesher_output_queue_("mesher_output_queue"),
      visualizer_input_queue_("visualizer_input_queue"),
      visualizer_output_queue_("visualizer_output_queue") {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();
  if (FLAGS_log_output) logger_.openLogFiles();

  frontend_params_ = dataset_->getFrontendParams();
  backend_params_ = dataset_->getBackendParams();

  // initialization_packet_.clear();

  // Instantiate stereo tracker (class that tracks implements estimation
  // front-end) and print parameters.
  // TODO remove hardcoded saveImages, use gflag.
  static constexpr int saveImages =
      0;  // 0: don't show, 1: show, 2: write & save
  vio_frontend_ = VIO::make_unique<StereoVisionFrontEnd>(
      imu_params,
      // This should not be asked!
      dataset->getGroundTruthState(dataset_->timestamp_first_lkf_).imu_bias_,
      frontend_params_, saveImages, dataset_->getDatasetName(),
      FLAGS_log_output);

  // Instantiate feature selector: not used in vanilla implementation.
  if (FLAGS_use_feature_selection) {
    feature_selector_ =
        FeatureSelector(dataset_->getFrontendParams(), *backend_params_);
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
SpinOutputContainer Pipeline::spin(
    const StereoImuSyncPacket& stereo_imu_sync_packet) {
  // Initialize pipeline if not initialized
  if (!is_initialized_) {

    // Launch frontend thread
    if (!is_launched_) {
      launchFrontendThread();
      is_launched_ = true;
    }
    CHECK(is_launched_);

    // Initialize pipeline.
    // TODO this is very brittle, because we are accumulating IMU data, but
    // not using it for initialization, because accumulated and actual IMU data
    // at init is the same...
    if (initialize(stereo_imu_sync_packet)) {
      launchRemainingThreads();
      is_initialized_ = true;
      return getSpinOutputContainer();
    } else {
      LOG(INFO) << "Not yet initialized...";
      // TODO: Don't publish if it is the trivial output
      return SpinOutputContainer();
    }

  } else if (stereo_imu_sync_packet.getReinitPacket().getReinitFlag()) {
    // TODO: Add option to autoinitialize, but re-initialize from ext. pose
    // (flag) Shutdown pipeline first
    shutdown();
    // Re-initialize pipeline
    reInitialize(stereo_imu_sync_packet);
    // Resume pipeline
    resume();
    return getSpinOutputContainer();

  } else {
    // TODO Warning: we do not accumulate IMU measurements for the first
    // packet... Spin.
    spinOnce(stereo_imu_sync_packet);
    return getSpinOutputContainer();
  }
}

/* -------------------------------------------------------------------------- */
// Get spin output container
SpinOutputContainer Pipeline::getSpinOutputContainer() {
  return SpinOutputContainer(getTimestamp(), getEstimatedPose(),
                             getEstimatedVelocity(), getEstimatedBias(),
                             getEstimatedStateCovariance(), getTrackerInfo());
}

/* -------------------------------------------------------------------------- */
// Spin the pipeline only once.
void Pipeline::spinOnce(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  CHECK(is_initialized_);
  ////////////////////////////// FRONT-END /////////////////////////////////////
  // Push to stereo frontend input queue.
  stereo_frontend_input_queue_.push(stereo_imu_sync_packet);

  // Run the pipeline sequentially.
  if (!parallel_run_) spinSequential();
}

/* -------------------------------------------------------------------------- */
void Pipeline::processKeyframe(
    const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
    const StereoFrame& last_stereo_keyframe,
    const ImuFrontEnd::PreintegratedImuMeasurements& pim,
    const TrackingStatus& kf_tracking_status_stereo,
    const gtsam::Pose3& relative_pose_body_stereo) {
  //////////////////// BACK-END ////////////////////////////////////////////////
  // Push to backend input.
  // This should be done inside the frontend!!!!
  // Or the backend should pull from the frontend!!!!
  backend_input_queue_.push(VioBackEndInputPayload(
      last_stereo_keyframe.getTimestamp(), statusSmartStereoMeasurements,
      kf_tracking_status_stereo, pim, relative_pose_body_stereo, &planes_));

  // This should be done inside those who need the backend results
  // IN this case the logger!!!!!
  // But there are many more people that want backend results...
  // Pull from backend.
  std::shared_ptr<VioBackEndOutputPayload> backend_output_payload =
      backend_output_queue_.popBlocking();
  LOG_IF(WARNING, !backend_output_payload) << "Missing backend output payload.";

  ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////////
  VioBackEnd::PointsWithIdMap points_with_id_VIO;
  VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
  MesherOutputPayload mesher_output_payload;
  VisualizationType visualization_type =
      static_cast<VisualizationType>(FLAGS_viz_type);
  // Compute 3D mesh
  if (visualization_type == VisualizationType::MESH2DTo3Dsparse) {
    pushToMesherInputQueue(&points_with_id_VIO, &lmk_id_to_lmk_type_map,
                           last_stereo_keyframe);

    // Find regularities in the mesh if we are using RegularVIO backend.
    // TODO create a new class that is mesh segmenter or plane extractor.
    if (FLAGS_extract_planes_from_the_scene) {
      CHECK_EQ(dataset_->getBackendType(), 1u);  // Use Regular VIO
      mesher_.clusterPlanesFromMesh(&planes_, points_with_id_VIO);
    } else {
      LOG_IF_EVERY_N(WARNING,
                     dataset_->getBackendType() == 1u &&
                         (FLAGS_regular_vio_backend_modality == 2u ||
                          FLAGS_regular_vio_backend_modality == 3u ||
                          FLAGS_regular_vio_backend_modality == 4u),
                     10)
          << "Using Regular VIO without extracting planes from the scene. "
             "Set flag extract_planes_from_the_scene to true to enforce "
             "regularities.";
    }

    // In the mesher thread push queue with meshes for visualization.
    // Use blocking to avoid skipping frames.
    LOG_IF(WARNING, !mesher_output_queue_.popBlocking(mesher_output_payload))
        << "Mesher output queue did not pop a payload.";
  }

  if (FLAGS_visualize) {
    // Push data for visualizer thread.
    // WHO Should be pushing to the visualizer input queue????????
    // This cannot happen at all from a single module, because visualizer
    // takes input from mesher and backend right now...
    visualizer_input_queue_.push(VisualizerInputPayload(
        // Pose for trajectory viz.
        vio_backend_->getWPoseBLkf() *  // The visualizer needs backend results
            last_stereo_keyframe
                .getBPoseCamLRect(),  // This should be pass at ctor level....
        // For visualizeMesh2D and visualizeMesh2DStereo.
        last_stereo_keyframe,
        // visualizeConvexHull & visualizeMesh3DWithColoredClusters
        // WARNING using move explicitly!
        std::move(mesher_output_payload),
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        visualization_type == VisualizationType::POINTCLOUD
            ? vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon()
            : points_with_id_VIO,
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        lmk_id_to_lmk_type_map,
        planes_,                           // visualizeMesh3DWithColoredClusters
        vio_backend_->getFactorsUnsafe(),  // For plane constraints viz.
        vio_backend_->getState(),  // For planes and plane constraints viz.
        visualization_type == VisualizationType::POINTCLOUD_REPEATEDPOINTS
            ? vio_backend_->get3DPoints()
            : std::vector<Point3>()));
  }
}

void Pipeline::pushToMesherInputQueue(
    VioBackEnd::PointsWithIdMap* points_with_id_VIO,
    VioBackEnd::LmkIdToLmkTypeMap* lmk_id_to_lmk_type_map,
    const StereoFrame& last_stereo_keyframe) {
  CHECK_NOTNULL(points_with_id_VIO);
  CHECK_NOTNULL(lmk_id_to_lmk_type_map);
  // Points_with_id_VIO contains all the points in the optimization,
  // (encoded as either smart factors or explicit values), potentially
  // restricting to points seen in at least min_num_obs_fro_mesher_points
  // keyframes (TODO restriction is not enforced for projection factors).
  *points_with_id_VIO = vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon(
      FLAGS_visualize_lmk_type ? lmk_id_to_lmk_type_map : nullptr,
      FLAGS_min_num_obs_for_mesher_points);

  // Create and fill data packet for mesher.
  // Push to queue.
  // In another thread, mesher is running, consuming mesher payloads.
  CHECK(mesher_input_queue_.push(MesherInputPayload(
      *points_with_id_VIO,  // copy, thread safe, read-only. // This should be a
                            // popBlocking...
      last_stereo_keyframe,  // not really thread safe, read only.
      vio_backend_->getWPoseBLkf().compose(
          vio_backend_->getBPoseLeftCam()))));  // Get camera pose.
}

void Pipeline::spinViz(bool parallel_run) {
  if (FLAGS_visualize) {
    visualizer_.spin(
        visualizer_input_queue_, visualizer_output_queue_,
        std::bind(&Pipeline::spinDisplayOnce, this, std::placeholders::_1),
        parallel_run);
  }
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinSequential() {
  // Spin once frontend.
  CHECK(vio_frontend_);
  vio_frontend_->spin(stereo_frontend_input_queue_,
                      stereo_frontend_output_queue_,
                      false);  // Do not run in parallel.

  // Pop from frontend.
  const auto& stereo_frontend_output_payload =
      stereo_frontend_output_queue_.pop();
  if (!stereo_frontend_output_payload) {
    // Frontend hasn't reach a keyframe, return and wait frontend to create a
    // keyframe.
    return;
  }
  CHECK(stereo_frontend_output_payload->is_keyframe_);

  // Pass info for resiliency
  debug_tracker_info_ = stereo_frontend_output_payload->getTrackerInfo();

  // Get timestamp of key-frame
  timestamp_lkf_ =
      stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp();

  // We have a keyframe. Push to backend.
  backend_input_queue_.push(VioBackEndInputPayload(
      stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp(),
      stereo_frontend_output_payload->statusSmartStereoMeasurements_,
      stereo_frontend_output_payload->tracker_status_,
      stereo_frontend_output_payload->pim_,
      stereo_frontend_output_payload->relative_pose_body_stereo_, &planes_));

  // Spin once backend. Do not run in parallel.
  CHECK(vio_backend_);
  vio_backend_->spin(backend_input_queue_, backend_output_queue_, false);

  // Pop blocking from backend.
  const auto& backend_output_payload = backend_output_queue_.popBlocking();
  CHECK(backend_output_payload);

  const auto& stereo_keyframe =
      stereo_frontend_output_payload->stereo_frame_lkf_;
  ////////////////// CREATE 3D MESH //////////////////////////////////////////
  VioBackEnd::PointsWithIdMap points_with_id_VIO;
  VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
  MesherOutputPayload mesher_output_payload;
  VisualizationType visualization_type =
      static_cast<VisualizationType>(FLAGS_viz_type);
  if (visualization_type == VisualizationType::MESH2DTo3Dsparse) {
    // Push to mesher.
    pushToMesherInputQueue(&points_with_id_VIO, &lmk_id_to_lmk_type_map,
                           stereo_keyframe);

    // Spin once mesher.
    mesher_.spin(mesher_input_queue_, mesher_output_queue_, false);

    // Find regularities in the mesh if we are using RegularVIO backend.
    // TODO create a new class that is mesh segmenter or plane extractor.
    if (FLAGS_extract_planes_from_the_scene) {
      CHECK_EQ(dataset_->getBackendType(), 1);  // Use Regular VIO
      mesher_.clusterPlanesFromMesh(&planes_, points_with_id_VIO);
    } else {
      LOG_IF_EVERY_N(WARNING,
                     dataset_->getBackendType() == 1u &&
                         (FLAGS_regular_vio_backend_modality == 2u ||
                          FLAGS_regular_vio_backend_modality == 3u ||
                          FLAGS_regular_vio_backend_modality == 4u),
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

  if (FLAGS_visualize) {
    // Push data for visualizer thread.
    // WHO Should be pushing to the visualizer input queue????????
    // This cannot happen at all from a single module, because visualizer
    // takes input from mesher and backend right now...
    visualizer_input_queue_.push(VisualizerInputPayload(
        // Pose for trajectory viz.
        vio_backend_->getWPoseBLkf() *  // The visualizer needs backend results
            stereo_keyframe
                .getBPoseCamLRect(),  // This should be pass at ctor level....
        // For visualizeMesh2D and visualizeMesh2DStereo.
        stereo_keyframe,
        // visualizeConvexHull & visualizeMesh3DWithColoredClusters
        // WARNING using move explicitly!
        std::move(mesher_output_payload),
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        visualization_type == VisualizationType::POINTCLOUD
            ? vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon()
            : points_with_id_VIO,
        // visualizeMesh3DWithColoredClusters & visualizePoints3D
        lmk_id_to_lmk_type_map,
        planes_,                           // visualizeMesh3DWithColoredClusters
        vio_backend_->getFactorsUnsafe(),  // For plane constraints viz.
        vio_backend_->getState(),  // For planes and plane constraints viz.
        visualization_type == VisualizationType::POINTCLOUD_REPEATEDPOINTS
            ? vio_backend_->get3DPoints()
            : std::vector<Point3>()));

    // Spin visualizer.
    visualizer_.spin(
        visualizer_input_queue_, visualizer_output_queue_,
        std::bind(&Pipeline::spinDisplayOnce, this, std::placeholders::_1),
        false);
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
  while (!is_initialized_ ||  // Loop while not initialized
         !(stereo_frontend_input_queue_
               .empty() &&  // Or, once init, data is not yet consumed.
           stereo_frontend_output_queue_.empty() &&
           !vio_frontend_->isWorking() && backend_input_queue_.empty() &&
           backend_output_queue_.empty() && !vio_backend_->isWorking() &&
           mesher_input_queue_.empty() && mesher_output_queue_.empty() &&
           !mesher_.isWorking() && visualizer_input_queue_.empty() &&
           visualizer_output_queue_.empty() && !visualizer_.isWorking())) {
    VLOG_EVERY_N(10, 100)
        << "VIO pipeline status: \n"
        << "Initialized? " << is_initialized_ << '\n'
        << "Frontend input queue empty?" << stereo_frontend_input_queue_.empty()
        << '\n'
        << "Frontend output queue empty?"
        << stereo_frontend_output_queue_.empty() << '\n'
        << "Frontend is working? " << vio_frontend_->isWorking() << '\n'
        << "Backend Input queue empty?" << backend_input_queue_.empty() << '\n'
        << "Backend Output queue empty?" << backend_output_queue_.empty()
        << '\n'
        << "Backend is working? "
        << (is_initialized_ ? vio_backend_->isWorking() : false) << '\n'
        << "Mesher input queue empty?" << mesher_input_queue_.empty() << '\n'
        << "Mesher output queue empty?" << mesher_output_queue_.empty() << '\n'
        << "Mesher is working? " << mesher_.isWorking() << '\n'
        << "Visualizer input queue empty?" << visualizer_input_queue_.empty()
        << '\n'
        << "Visualizer output queue empty?" << visualizer_output_queue_.empty()
        << '\n'
        << "Visualizer is working? " << visualizer_.isWorking();
    std::this_thread::sleep_for(std::chrono::seconds(sleep_time));
  }
  shutdown();
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
  if (FLAGS_log_output) {
    LOG(INFO) << "Closing log files";
    logger_.closeLogFiles();
  }
  LOG(INFO) << "Pipeline destructor finished.";
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initialize(const StereoImuSyncPacket &stereo_imu_sync_packet) {
  // Switch initialization mode
  switch (FLAGS_initialization_mode) {
  case 0: // Initialization using IMU or GT only
    return initializeFromIMUorGT(stereo_imu_sync_packet);
    break;
  case 1: // Initialization using online gravity alignment
    return initializeOnline(stereo_imu_sync_packet);
    break;
  default:
    LOG(ERROR) << "Initialization mode doesn't exist.";
    return false;
    break;
  }
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initializeFromIMUorGT(
    const StereoImuSyncPacket &stereo_imu_sync_packet) {
  LOG(INFO) << "------------------- Initialize Pipeline with frame k = "
            << stereo_imu_sync_packet.getStereoFrame().getFrameId()
            << "--------------------";

  /////////////////// FRONTEND /////////////////////////////////////////////////
  // Initialize Stereo Frontend.
  CHECK(vio_frontend_);
  const StereoFrame& stereo_frame_lkf = vio_frontend_->processFirstStereoFrame(
      stereo_imu_sync_packet.getStereoFrame());

  ///////////////////////////// BACKEND ////////////////////////////////////////
  // Initialize Backend using GT if available.
  std::shared_ptr<gtNavState> initialStateGT =
      dataset_->isGroundTruthAvailable()
          ? std::make_shared<gtNavState>(dataset_->getGroundTruthState(
                stereo_imu_sync_packet.getStereoFrame().getTimestamp()))
          : std::shared_ptr<gtNavState>(nullptr);

  // TODO: Include flag to start from external pose estimate (ROS)
  // if (flag_init_gt = 0) {
  //    LOG(INFO) << "Initialize pipeline with possible GT.";
  //
  //} else {
  //    // Initialize Backend using External Pose Estimate if available.
  //    LOG(INFO) << "Initialize pipeline with external navstate estimate.";
  //    initialStateGT = std::make_shared<gtNavState>(gtNavState(
  //          stereo_imu_sync_packet.getReinitPacket().getReinitPose(),
  //          stereo_imu_sync_packet.getReinitPacket().getReinitVel(),
  //          stereo_imu_sync_packet.getReinitPacket().getReinitBias()))
  //}

  initBackend(
      &vio_backend_, stereo_frame_lkf.getBPoseCamLRect(),
      stereo_frame_lkf.getLeftUndistRectCamMat(),
      stereo_frame_lkf.getBaseline(), *backend_params_, &initialStateGT,
      stereo_imu_sync_packet.getStereoFrame().getTimestamp(),
      stereo_imu_sync_packet.getImuAccGyr());  // No timestamps needed for IMU?
  vio_backend_->registerImuBiasUpdateCallback(
      std::bind(&StereoVisionFrontEnd::updateImuBias,
                // Send a cref: constant reference because vio_frontend_ is
                // not copyable.
                std::cref(*vio_frontend_), std::placeholders::_1));

  ////////////////// DEBUG INITIALIZATION //////////////////////////////////
  if (FLAGS_log_output) {
    logger_.displayInitialStateVioInfo(
        *dataset_, vio_backend_, *CHECK_NOTNULL(initialStateGT.get()),
        stereo_imu_sync_packet.getImuAccGyr(),
        stereo_imu_sync_packet.getStereoFrame().getTimestamp());
    // Store latest pose estimate.
    logger_.W_Pose_Bprevkf_vio_ = vio_backend_->getWPoseBLkf();
  }

  return true;
}

// TODO(Sandro): Need to get pre-integration without gravity!!!!!!
// TODO(Sandro): Set gravity in pre-integration to zero
// TODO(Sandro): Reset values at end of this function
// TODO(Sandro): Reset spinOnce (frontend) as private function
// TODO(Sandro): Fix issues with autoinitialize flag
  // Change flag to be int and not bool
  // Change BackEnd constructor depending on 1 or 2
// TODO(Sandro): Use only one backend for initialization??
// TODO(Sandro): Find way to destruct initBackend?
// TODO(Sandro): Create function for initial bundle adjustment in pipeline
  // Skips from batchpop to initial alignment --> directly in backend initialization
  // Adapt initializeIMUorGT to use same code base for backend initialization
// TODO(Sandro): Insert poses from initialization and optimize for kf (batch
// insert and optimize once)
/* -------------------------------------------------------------------------- */
bool Pipeline::initializeOnline(
    const StereoImuSyncPacket &stereo_imu_sync_packet) {
  int frame_id = stereo_imu_sync_packet.getStereoFrame().getFrameId();
  LOG(INFO) << "------------------- Initializing Pipeline with frame k = "
            << frame_id << "--------------------";

  CHECK(vio_frontend_);
  CHECK_GE(frame_id, 1);
  CHECK_GE(FLAGS_numb_frames_oga, frame_id);

  // Enforce stereo frame as keyframe for initialization
  StereoImuSyncPacket stereo_imu_sync_init = stereo_imu_sync_packet;
  stereo_imu_sync_init.setAsKeyframe();

  /////////////////// FIRST FRAME //////////////////////////////////////////////
  if (frame_id == 1) {

    // Set IMU bias in frontend to trivial value. This values
    // are reset at the end of initializeOnline().
    vio_frontend_->updateAndResetImuBias(
            gtsam::imuBias::ConstantBias(
                Vector3::Zero(), Vector3::Zero()));
    vio_frontend_->resetGravity(Vector3::Zero());
    vio_frontend_->forceFiveThreePointMethod(true);
    CHECK_DOUBLE_EQ(vio_frontend_->getGravity().norm(),0.0);
    CHECK_DOUBLE_EQ(vio_frontend_->getCurrentImuBias().gyroscope().norm(),0.0);

    // Initialize Stereo Frontend.
    StereoFrame stereo_frame_lkf =
        vio_frontend_->processFirstStereoFrame(
            stereo_imu_sync_init.getStereoFrame());
    // Get timestamp for bookkeeping
    timestamp_lkf_ = stereo_imu_sync_init.getStereoFrame().getTimestamp();

    return false;

  /////////////////// FRONTEND //////////////////////////////////////////////////
  } else {

    CHECK_DOUBLE_EQ(vio_frontend_->getCurrentImuBias().gyroscope().norm(), 0.0);

    // Spin frontend once with enforced keyframe and 53-point method
    auto frontend_output = vio_frontend_->spinOnce(
        std::make_shared<StereoImuSyncPacket>(stereo_imu_sync_init));
    stereo_frontend_output_queue_.push(frontend_output);
    StereoFrame stereo_frame_lkf = frontend_output.stereo_frame_lkf_;

    // Only process set of frontend outputs after specific number of frames
    if (frame_id < FLAGS_numb_frames_oga) {
      return false;
    } else {
      ///////////////////////////// ONLINE INITIALIZER //////////////////////

      // Run Bundle-Adjustment and initial gravity alignment
      // TODO(Sandro): Clean up this interface
      gtsam::Vector3 gyro_bias;
      CHECK(bundleAdjustmentAndGravityAlignment(
                      stereo_imu_sync_init,
                      stereo_frame_lkf,
                      &gyro_bias));

      // Reset frontend with non-trivial gravity and remove 53-enforcement.
      // Update frontend with initial gyro bias estimate.
      vio_frontend_->forceFiveThreePointMethod(false);
      vio_frontend_->resetGravity(backend_params_->n_gravity_);
      gtsam::imuBias::ConstantBias final_bias(gtsam::Vector3::Zero(),
                                              gyro_bias);
      vio_frontend_->updateAndResetImuBias(final_bias);
      CHECK_DOUBLE_EQ(vio_frontend_->getGravity().norm(),
                      backend_params_->n_gravity_.norm());
      CHECK_DOUBLE_EQ(vio_frontend_->getCurrentImuBias().gyroscope().norm(),
                      gyro_bias.norm());

      ///////////////////////////// BACKEND ///////////////////////////////////
      // TODO(Sandro): This part can be unified with initializeIMUorGT()

      std::shared_ptr<gtNavState> initialStateOGA =
          std::make_shared<gtNavState>(gtNavState());

      initBackend(
          &vio_backend_, stereo_frame_lkf.getBPoseCamLRect(),
          stereo_frame_lkf.getLeftUndistRectCamMat(),
          stereo_frame_lkf.getBaseline(), *backend_params_, &initialStateOGA,
          stereo_imu_sync_init.getStereoFrame().getTimestamp(),
          stereo_imu_sync_init.getImuAccGyr()); // No timestamps needed for IMU?
      vio_backend_->registerImuBiasUpdateCallback(
          std::bind(&StereoVisionFrontEnd::updateImuBias,
                    // Send a cref: constant reference because vio_frontend_ is
                    // not copyable.
                    std::cref(*vio_frontend_), std::placeholders::_1));

      ////////////////// DEBUG INITIALIZATION //////////////////////////////////
      if (FLAGS_log_output) {
        logger_.displayInitialStateVioInfo(
            *dataset_, vio_backend_, *CHECK_NOTNULL(initialStateOGA.get()),
            stereo_imu_sync_init.getImuAccGyr(),
            stereo_imu_sync_init.getStereoFrame().getTimestamp());
        // Store latest pose estimate.
        logger_.W_Pose_Bprevkf_vio_ = vio_backend_->getWPoseBLkf();
      }

      // TODO(Sandro): Create check-return for function
      return true;
    }
  }
}

/* -------------------------------------------------------------------------- */
// Perform Bundle-Adjustment and initial gravity alignment.
// [out]: Initial navState and imu bias estimates for backend.
// TODO(Sandro): Modify this to have 
bool Pipeline::bundleAdjustmentAndGravityAlignment(
                      StereoImuSyncPacket &stereo_imu_sync_init,
                      StereoFrame &stereo_frame_lkf,
                      gtsam::Vector3 *gyro_bias) {

    // Get frontend output to backend input for online initialization
    auto output_frontend = stereo_frontend_output_queue_.batchPop();
    const StereoFrame &stereo_frame_fkf =
        output_frontend.back()->stereo_frame_lkf_;
    std::vector<std::shared_ptr<VioBackEndInputPayload>> inputs_backend;
    inputs_backend.clear();

    // Create inputs for online gravity alignment
    std::vector<gtsam::PreintegratedImuMeasurements> pims;
    pims.clear();
    std::vector<double> delta_t_camera;
    delta_t_camera.clear();
    for (int i = 0; i < output_frontend.size(); i++) {
      // Create input for backend
      std::shared_ptr<VioBackEndInputPayload> input_backend =
          std::make_shared<VioBackEndInputPayload>(VioBackEndInputPayload(
              output_frontend.at(i)->stereo_frame_lkf_.getTimestamp(),
              output_frontend.at(i)->statusSmartStereoMeasurements_,
              output_frontend.at(i)->tracker_status_,
              output_frontend.at(i)->pim_,
              output_frontend.at(i)->relative_pose_body_stereo_, &planes_));
      inputs_backend.push_back(input_backend);
      pims.push_back(output_frontend.at(i)->pim_);
      // Bookkeeping for timestamps
      Timestamp timestamp_kf = 
              output_frontend.at(i)->stereo_frame_lkf_.getTimestamp();
      delta_t_camera.push_back(UtilsOpenCV::NsecToSec(
                      timestamp_kf - timestamp_lkf_));
      timestamp_lkf_ = timestamp_kf;
      // Logging
      VLOG(10) << "Frame: "
                << output_frontend.at(i)->stereo_frame_lkf_.getFrameId()
                << " for initialization is "
                << (output_frontend.at(i)->is_keyframe_ ? "a keyframe."
                                                        : "not a keyframe.");
    }
    // Logging
    VLOG(10) << "N frames for initial alignment: " << output_frontend.size();

    // Run initial Bundle Adjustment and retrieve body poses 
    // wrt. to initial camera frame (v0_T_bk, for k in 0:N)
    std::shared_ptr<gtNavState> trivial_state =
        std::make_shared<gtNavState>(gtNavState(
            gtsam::Pose3(),
            gtsam::Vector3::Zero(),
            gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero())));
    VioBackEnd initial_backend(
        stereo_frame_lkf.getBPoseCamLRect(),
        stereo_frame_lkf.getLeftUndistRectCamMat(),
        stereo_frame_lkf.getBaseline(), &trivial_state,
        stereo_imu_sync_init.getStereoFrame().getTimestamp(),
        stereo_imu_sync_init.getImuAccGyr(), *backend_params_,
        FLAGS_log_output);
    std::vector<gtsam::Pose3> estimated_poses =
        initial_backend.addInitialVisualStatesAndOptimize(inputs_backend);
    // Logging
    LOG(INFO) << "Initial bundle adjustment terminated.";

    // Destruct initial backend
    // initial_backend.~VioBackEnd();

    // Enforce zero bias in initial propagation
    vio_frontend_->updateAndResetImuBias(
        gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
    // Run initial visual-inertial alignment(OGA)
    *gyro_bias = vio_frontend_->getCurrentImuBias().gyroscope();
    OnlineGravityAlignment initial_alignment(
                              estimated_poses, 
                              delta_t_camera,
                              pims, 
                              backend_params_->n_gravity_,
                              gyro_bias);
    initial_alignment.alignVisualInertialEstimates();

    // TODO(Sandro): Create check-return for function
    return true;
}

/* -------------------------------------------------------------------------- */
// TODO: Adapt and create better re-initialization (online) function
bool Pipeline::reInitialize(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  // Reset shutdown flags
  shutdown_ = false;

  CHECK(vio_frontend_);
  vio_frontend_->restart();

  CHECK(vio_backend_);
  vio_backend_->restart();

  mesher_.restart();

  visualizer_.restart();

  // Use default initialization function
  return initialize(stereo_imu_sync_packet);
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initBackend(std::unique_ptr<VioBackEnd>* vio_backend,
                           const gtsam::Pose3& B_Pose_camLrect,
                           const gtsam::Cal3_S2& left_undist_rect_cam_mat,
                           const double& baseline,
                           const VioBackEndParams& vio_params,
                           std::shared_ptr<gtNavState>* initial_state_gt,
                           const Timestamp& timestamp_k,
                           const ImuAccGyrS& imu_accgyr) {
  CHECK_NOTNULL(vio_backend);
  // Create VIO.
  switch (dataset_->getBackendType()) {
    case 0: {
      LOG(INFO) << "\e[1m Using Normal VIO. \e[0m";
      *vio_backend = VIO::make_unique<VioBackEnd>(
          B_Pose_camLrect, left_undist_rect_cam_mat, baseline, initial_state_gt,
          timestamp_k, imu_accgyr, vio_params, FLAGS_log_output);
      break;
    }
    case 1: {
      LOG(INFO) << "\e[1m Using Regular VIO with modality "
                << FLAGS_regular_vio_backend_modality << "\e[0m";
      *vio_backend = VIO::make_unique<RegularVioBackEnd>(
          B_Pose_camLrect, left_undist_rect_cam_mat, baseline, initial_state_gt,
          timestamp_k, imu_accgyr, vio_params, FLAGS_log_output,
          static_cast<RegularVioBackEnd::BackendModality>(
              FLAGS_regular_vio_backend_modality));
      break;
    }
    default: {
      LOG(FATAL) << "Requested backend type is not supported.\n"
                 << "Currently supported backend types:\n"
                 << "0: normal VIO\n"
                 << "1: regular VIO\n"
                 << " but requested backend: " << dataset_->getBackendType();
    }
  }
  return true;
}

/* -------------------------------------------------------------------------- */
void Pipeline::spinDisplayOnce(
    VisualizerOutputPayload& visualizer_output_payload) {
  // Display 3D window.
  if (visualizer_output_payload.visualization_type_ !=
      VisualizationType::NONE) {
    VLOG(10) << "Spin Visualize 3D output.";
    // visualizer_output_payload->window_.spin();
    visualizer_output_payload.window_.spinOnce(1, true);
    // TODO this is not very thread-safe!!! Since recordVideo might modify
    // window_ in this thread, while it might also be called in viz thread.
    if (FLAGS_record_video_for_viz_3d) {
      visualizer_.recordVideo();
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
StatusSmartStereoMeasurements Pipeline::featureSelect(
    const VioFrontEndParams& tracker_params, const ETHDatasetParser& dataset,
    const Timestamp& timestamp_k, const Timestamp& timestamp_lkf,
    const gtsam::Pose3& W_Pose_Blkf, double* feature_selection_time,
    std::shared_ptr<StereoFrame>& stereoFrame_km1,
    const StatusSmartStereoMeasurements& status_smart_stereo_meas,
    int cur_kf_id, int save_image_selector, const gtsam::Matrix& curr_state_cov,
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
  if (dataset.isGroundTruthAvailable()) {
    W_Pose_Bkf_gt = dataset.getGroundTruthState(timestamp_lkf).pose_;

    for (size_t kk = 0; kk < nrKfInHorizon + 1; kk++) {
      // Including current pose.
      Timestamp timestamp_kk =
          timestamp_k +
          UtilsOpenCV::SecToNsec(kk * tracker_params.intra_keyframe_time_);

      // Relative pose wrt ground truth at last kf.
      Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(
          dataset.getGroundTruthState(timestamp_kk).pose_);
      posesAtFutureKeyframes.push_back(
          StampedPose(W_Pose_Blkf.compose(poseGT_km1_kk),
                      UtilsOpenCV::NsecToSec(timestamp_kk)));
    }
  }

  VLOG(100) << "Starting feature selection...";
  SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
  std::tie(trackedAndSelectedSmartStereoMeasurements, *feature_selection_time) =
      feature_selector_.splitTrackedAndNewFeatures_Select_Display(
          stereoFrame_km1, status_smart_stereo_meas.second, cur_kf_id,
          save_image_selector, tracker_params.featureSelectionCriterion_,
          tracker_params.featureSelectionNrCornersToSelect_,
          tracker_params.maxFeatureAge_,
          posesAtFutureKeyframes,  // TODO Luca: can we make this optional, for
                                   // the case where we do not have ground
                                   // truth?
          curr_state_cov, dataset.getDatasetName(),
          left_frame);  // last 2 are for visualization
  VLOG(100) << "Feature selection completed.";

  // Same status as before.
  TrackerStatusSummary status = status_smart_stereo_meas.first;
  return std::make_pair(status, trackedAndSelectedSmartStereoMeasurements);
}

/* -------------------------------------------------------------------------- */
void Pipeline::processKeyframePop() {
  // Pull from stereo frontend output queue.
  LOG(INFO) << "Spinning wrapped thread.";
  while (!shutdown_) {
    std::shared_ptr<StereoFrontEndOutputPayload>
        stereo_frontend_output_payload =
            stereo_frontend_output_queue_.popBlocking();
    /*
    // TODO(Sandro): Adapt this!! 
    // We should also be able to batch process for the backend
    // Either just popBlocking or adapt processKeyframe for multiple
    const auto &stereo_frontend_output_payload_vector =
        stereo_frontend_output_queue_.batchPopBlocking();
    if (stereo_frontend_output_payload_vector.size() != 1) {
      LOG(INFO) << "Queue output vector size: 1";
    } else {
      LOG(WARNING) << "Queue output vector size: "
                   << stereo_frontend_output_payload_vector.size();
    }
    // TODO: Adapt to be able to push in blocks into backend
    const auto &stereo_frontend_output_payload =
        stereo_frontend_output_payload_vector.front();
    // std::shared_ptr<StereoFrontEndOutputPayload>
    // stereo_frontend_output_payload
    //    = stereo_frontend_output_queue_.popBlocking();
    CHECK(stereo_frontend_output_payload); */
    if (!stereo_frontend_output_payload) {
      LOG(WARNING) << "No StereoFrontEnd Output Payload received.";
      return;
    }
    
    CHECK(stereo_frontend_output_payload->is_keyframe_);

    // Pass info for resiliency
    debug_tracker_info_ = stereo_frontend_output_payload->getTrackerInfo();

    // Get timestamp of key-frame
    timestamp_lkf_ =
        stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp();

    ////////////////////////////////////////////////////////////////////////////
    // So from this point on, we have a keyframe.
    // Pass info to VIO
    // Actual keyframe processing. Call to backend.
    ////////////////////////////// BACK-END ////////////////////////////////////
    processKeyframe(
        stereo_frontend_output_payload->statusSmartStereoMeasurements_,
        stereo_frontend_output_payload->stereo_frame_lkf_,
        stereo_frontend_output_payload->pim_,
        stereo_frontend_output_payload->tracker_status_,
        stereo_frontend_output_payload->relative_pose_body_stereo_);
  }
  LOG(INFO) << "Shutdown wrapped thread.";
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
    stereo_frontend_thread_ = VIO::make_unique<std::thread>(
        &StereoVisionFrontEnd::spin, CHECK_NOTNULL(vio_frontend_.get()),
        std::ref(stereo_frontend_input_queue_),
        std::ref(stereo_frontend_output_queue_), true);
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

    // Start backend_thread.
    backend_thread_ = VIO::make_unique<std::thread>(
        &VioBackEnd::spin,
        // Returns the pointer to vio_backend_.
        CHECK_NOTNULL(vio_backend_.get()), std::ref(backend_input_queue_),
        std::ref(backend_output_queue_), true);

    // Start mesher_thread.
    mesher_thread_ = VIO::make_unique<std::thread>(
        &Mesher::spin, &mesher_, std::ref(mesher_input_queue_),
        std::ref(mesher_output_queue_), true);

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

/* -------------------------------------------------------------------------- */
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

  LOG(INFO) << "Restarting visualizer workers and queues...";
  visualizer_input_queue_.resume();
  visualizer_output_queue_.resume();

  // Re-launch threads
  if (parallel_run_) {
    launchThreads();
  } else {
    LOG(INFO) << "Running in sequential mode (parallel_run set to "
              << parallel_run_ << ").";
  }
  is_initialized_ = true;
}

/* -------------------------------------------------------------------------- */
void Pipeline::stopThreads() {
  LOG(INFO) << "Stopping workers and queues...";

  LOG(INFO) << "Stopping backend workers and queues...";
  backend_input_queue_.shutdown();
  backend_output_queue_.shutdown();
  CHECK(vio_backend_);
  vio_backend_->shutdown();

  // Shutdown workers and queues.
  LOG(INFO) << "Stopping frontend workers and queues...";
  stereo_frontend_input_queue_.shutdown();
  stereo_frontend_output_queue_.shutdown();
  CHECK(vio_frontend_);
  vio_frontend_->shutdown();

  LOG(INFO) << "Stopping mesher workers and queues...";
  mesher_input_queue_.shutdown();
  mesher_output_queue_.shutdown();
  mesher_.shutdown();

  LOG(INFO) << "Stopping visualizer workers and queues...";
  visualizer_input_queue_.shutdown();
  visualizer_output_queue_.shutdown();
  visualizer_.shutdown();

  LOG(INFO) << "Sent stop flag to all workers and queues...";
}

/* -------------------------------------------------------------------------- */
void Pipeline::joinThreads() {
  LOG(INFO) << "Joining threads...";

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

  LOG(INFO) << "Joining backend thread...";
  if (backend_thread_ && backend_thread_->joinable()) {
    backend_thread_->join();
    LOG(INFO) << "Joined backend thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Backend thread is not joinable...";
  }

  LOG(INFO) << "Joining mesher thread...";
  if (mesher_thread_ && mesher_thread_->joinable()) {
    mesher_thread_->join();
    LOG(INFO) << "Joined mesher thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Mesher thread is not joinable...";
  }
  // visualizer_thread_.join();

  LOG(INFO) << "All threads joined.";
}

}  // namespace VIO
