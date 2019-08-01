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
#include "InitializationBackEnd.h"
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

DEFINE_int32(num_frames_vio_init, 25,
             "Minimum number of frames for the online "
             "gravity-aligned initialization");

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
    initialization_frontend_output_queue_("initialization_frontend_output_queue"),
    backend_input_queue_("backend_input_queue"),
    backend_output_queue_("backend_output_queue"),
    mesher_input_queue_("mesher_input_queue"),
    mesher_output_queue_("mesher_output_queue"),
    lcd_input_queue_("lcd_input_queue"),
    lcd_output_queue_("lcd_output_queue"),
    visualizer_input_queue_("visualizer_input_queue"),
    visualizer_output_queue_("visualizer_output_queue") {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();
  if (FLAGS_log_output) logger_.openLogFiles();

  frontend_params_ = dataset_->getFrontendParams();
  backend_params_ = dataset_->getBackendParams();
  lcd_params_ = dataset_->getLCDParams();

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

  loop_closure_detector_ = VIO::make_unique<LoopClosureDetector>(
      lcd_params_,
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

// TODO MARCUS: Make sure loop_closure_detector_ is properly integrated to all of the methods below

/* -------------------------------------------------------------------------- */
void Pipeline::spin(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  // Check if we have to re-initialize
  checkReInitialize(stereo_imu_sync_packet);
  // Initialize pipeline if not initialized
  if (!is_initialized_) {
    // Launch frontend thread
    if (!is_launched_) {
      launchFrontendThread();
      is_launched_ = true;
      init_frame_id_ = stereo_imu_sync_packet.getStereoFrame().getFrameId();
    }
    CHECK(is_launched_);

    // Initialize pipeline.
    // TODO this is very brittle, because we are accumulating IMU data, but
    // not using it for initialization, because accumulated and actual IMU data
    // at init is the same...
    if (initialize(stereo_imu_sync_packet)) {
      LOG(INFO) << "Before launching threads.";
      launchRemainingThreads();
      LOG(INFO) << " launching threads.";
      is_initialized_ = true;
      return;
    } else {
      LOG(INFO) << "Not yet initialized...";
      // TODO: Don't publish if it is the trivial output
      return;
    }
  } else {
    // TODO Warning: we do not accumulate IMU measurements for the first packet...
    // Spin.
    spinOnce(stereo_imu_sync_packet);
    return;
  }
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

  // Push to lcd. Also should be done inside frontend!!!
  // TODO: Need to make this robust to slow backend.
  lcd_input_queue_.push(LoopClosureDetectorInputPayload(
      last_stereo_keyframe.getTimestamp(), // TODO: This should be different?
      last_stereo_keyframe));

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

    // In the mesher thread push queue with meshes for visualization.
    // Use blocking to avoid skipping frames.
    LOG_IF(WARNING, !mesher_output_queue_.popBlocking(mesher_output_payload))
        << "Mesher output queue did not pop a payload.";

    // Do this after popBlocking from Mesher so we do it sequentially, since
    // planes_ are not thread-safe.
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
  // TODO Marcus: Need to add a flag for using LCD in sequential, it makes things slow

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

  // Push keyframe to LCD.
  lcd_input_queue_.push(LoopClosureDetectorInputPayload(
      stereo_frontend_output_payload->stereo_frame_lkf_.getTimestamp(),
      stereo_frontend_output_payload->stereo_frame_lkf_));

  // Spin once LCD. Do not run in parallel.
  loop_closure_detector_->spin(lcd_input_queue_, lcd_output_queue_, false);

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
      DCHECK_EQ(dataset_->getBackendType(), 1);  // Use Regular VIO
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
           !mesher_.isWorking() && lcd_input_queue_.empty() &&
           lcd_output_queue_.empty() && !loop_closure_detector_->isWorking() &&
           visualizer_input_queue_.empty() && visualizer_output_queue_.empty()
           && !visualizer_.isWorking())) {
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
        << "LoopClosureDetector input queue empty?"
        << lcd_input_queue_.empty() << '\n'
        << "LoopClosureDetector output queue empty?"
        << lcd_output_queue_.empty() << '\n'
        << "LoopClosureDetector is working? "
        << loop_closure_detector_->isWorking() << '\n'
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
  switch (backend_params_->autoInitialize_) {
  case 0 ... 1: // Initialization using IMU or GT only
    return initializeFromIMUorGT(stereo_imu_sync_packet);
    break;
  case 2: // Initialization using online gravity alignment
    return initializeOnline(stereo_imu_sync_packet);
    break;
  default:
    LOG(ERROR) << "Initialization mode doesn't exist.";
    return false;
    break;
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
    CHECK(vio_frontend_);
    vio_frontend_->restart();
    CHECK(vio_backend_);
    vio_backend_->restart();
    mesher_.restart();
    loop_closure_detector_->restart();
    visualizer_.restart();
    // Resume pipeline
    resume();
    initialization_frontend_output_queue_.resume();
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

  ///////////////////////////// GT ////////////////////////////////////////////
  // Initialize Backend using GT if available.
  std::shared_ptr<gtNavState> initialStateGT =
      dataset_->isGroundTruthAvailable()?
        std::make_shared<gtNavState>(dataset_->getGroundTruthState(
                     stereo_imu_sync_packet.getStereoFrame().getTimestamp())) :
                     std::shared_ptr<gtNavState>(nullptr);

  ///////////////////////////// BACKEND //////////////////////////////////////
  // Initialize backend with pose estimate from gravity alignment
  initializeVioBackend(stereo_imu_sync_packet, initialStateGT,
                        stereo_frame_lkf);

  return true;
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initializeOnline(
    const StereoImuSyncPacket &stereo_imu_sync_packet) {
  int frame_id = stereo_imu_sync_packet.getStereoFrame().getFrameId();
  LOG(INFO) << "------------------- Initializing Pipeline with frame k = "
            << frame_id << "--------------------";

  CHECK(vio_frontend_);
  CHECK_GE(frame_id, init_frame_id_);
  CHECK_GE(init_frame_id_ + FLAGS_num_frames_vio_init, frame_id);

  // TODO(Sandro): Find a way to optimize this
  // Create ImuFrontEnd with non-zero gravity (zero bias)
  gtsam::PreintegratedImuMeasurements::Params imu_params =
      vio_frontend_->getImuFrontEndParams();
  imu_params.n_gravity = backend_params_->n_gravity_;
  ImuFrontEnd imu_frontend_real(imu_params,
      gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
  CHECK_DOUBLE_EQ(imu_frontend_real.getPreintegrationGravity().norm(),
        imu_params.n_gravity.norm());
  ///////

  // Enforce stereo frame as keyframe for initialization
  StereoImuSyncPacket stereo_imu_sync_init = stereo_imu_sync_packet;
  stereo_imu_sync_init.setAsKeyframe();

  /////////////////// FIRST FRAME //////////////////////////////////////////////
  if (frame_id == init_frame_id_) {
    // Set trivial bias, gravity and force 5/3 point method for initialization
    vio_frontend_->prepareFrontendForOnlineAlignment();
    // Initialize Stereo Frontend.
    StereoFrame stereo_frame_lkf =
        vio_frontend_->processFirstStereoFrame(
            stereo_imu_sync_init.getStereoFrame());
    return false;

  /////////////////// FRONTEND //////////////////////////////////////////////////
  } else {

    // Check trivial bias and gravity vector for online initialization
    vio_frontend_->checkFrontendForOnlineAlignment();
    // Spin frontend once with enforced keyframe and 53-point method
    auto frontend_output = vio_frontend_->spinOnce(
        std::make_shared<StereoImuSyncPacket>(stereo_imu_sync_init));
    const StereoFrame stereo_frame_lkf = frontend_output.stereo_frame_lkf_;
    // TODO(Sandro): Optionally add AHRS PIM
    InitializationInputPayload frontend_init_output(
      frontend_output.is_keyframe_,
      frontend_output.statusSmartStereoMeasurements_,
      frontend_output.tracker_status_,
      frontend_output.relative_pose_body_stereo_,
      frontend_output.stereo_frame_lkf_,
      frontend_output.pim_,
      frontend_output.debug_tracker_info_);
    initialization_frontend_output_queue_.push(frontend_init_output);

    // TODO(Sandro): Find a way to optimize this
    // This queue is used for the the backend optimization
    const auto& imu_stamps = stereo_imu_sync_packet.getImuStamps();
    const auto& imu_accgyr = stereo_imu_sync_packet.getImuAccGyr();
    const auto& pim =
      imu_frontend_real.preintegrateImuMeasurements(imu_stamps, imu_accgyr);
    StereoFrontEndOutputPayload frontend_real_output(
      frontend_output.is_keyframe_,
      frontend_output.statusSmartStereoMeasurements_,
      frontend_output.tracker_status_,
      frontend_output.relative_pose_body_stereo_,
      frontend_output.stereo_frame_lkf_,
      pim,
      frontend_output.debug_tracker_info_);
    // This queue is used for the backend after initialization
    stereo_frontend_output_queue_.push(frontend_real_output);
    /////////


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
      std::queue<InitializationInputPayload> output_frontend;
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
      InitializationBackEnd initial_backend(
          output_frontend.front().stereo_frame_lkf_.getBPoseCamLRect(),
          output_frontend.front().stereo_frame_lkf_.getLeftUndistRectCamMat(),
          output_frontend.front().stereo_frame_lkf_.getBaseline(),
          backend_params_init,
          FLAGS_log_output);

      // Enforce zero bias in initial propagation
      // TODO(Sandro): Remove this, once AHRS is implemented
      vio_frontend_->updateAndResetImuBias(
          gtsam::imuBias::ConstantBias(Vector3::Zero(), Vector3::Zero()));
      gyro_bias = vio_frontend_->getCurrentImuBias().gyroscope();

      // Initialize if successful
      if (initial_backend.bundleAdjustmentAndGravityAlignment(
                                    output_frontend,
                                    &gyro_bias,
                                    &g_iter_b0,
                                    &init_navstate)) {
        LOG(INFO) << "Bundle adjustment and alignment successful!";

        // Create initial state for initialization from online gravity
        std::shared_ptr<gtNavState> initial_state_OGA =
            std::make_shared<gtNavState>(init_navstate,
              ImuBias(gtsam::Vector3(), gyro_bias));

        // Reset frontend with non-trivial gravity and remove 53-enforcement.
        // Update frontend with initial gyro bias estimate.
        const gtsam::Vector3 gravity = backend_params_->n_gravity_;
        vio_frontend_->resetFrontendAfterOnlineAlignment(gravity, gyro_bias);

        auto full_init_duration =
            utils::Timer::toc<std::chrono::nanoseconds>(tic_full_init).count();
        LOG(INFO) << "Time used for initialization: "
                  << (double(full_init_duration) / double(1e6)) << " (ms).";

        ///////////////////////////// BACKEND ////////////////////////////////
        // Initialize backend with pose estimate from gravity alignment
        initializeVioBackend(stereo_imu_sync_packet, initial_state_OGA,
                          stereo_frame_lkf);
        LOG(INFO) << "Initialization finalized.";

        // TODO(Sandro): Create check-return for function
        return true;
      } else {
        // Reset initialization
        LOG(WARNING) << "Bundle adjustment or alignment failed!";
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
// TODO(Sandro): Unify both functions below (init backend)
//////////////////// UNIFY
bool Pipeline::initializeVioBackend(
                              const StereoImuSyncPacket &stereo_imu_sync_packet,
                              std::shared_ptr<gtNavState> initial_state,
                              const StereoFrame &stereo_frame_lkf) {
    ///////////////////////////// BACKEND ///////////////////////////////////
    initBackend(
        &vio_backend_, stereo_frame_lkf.getBPoseCamLRect(),
        stereo_frame_lkf.getLeftUndistRectCamMat(),
        stereo_frame_lkf.getBaseline(), *backend_params_, &initial_state,
        stereo_imu_sync_packet.getStereoFrame().getTimestamp(),
        stereo_imu_sync_packet.getImuAccGyr()); // No timestamps needed for IMU?
    vio_backend_->registerImuBiasUpdateCallback(
        std::bind(&StereoVisionFrontEnd::updateImuBias,
                  // Send a cref: constant reference because vio_frontend_ is
                  // not copyable.
                  std::cref(*vio_frontend_), std::placeholders::_1));

    ////////////////// DEBUG INITIALIZATION //////////////////////////////////
    if (FLAGS_log_output) {
      logger_.displayInitialStateVioInfo(
          *dataset_, vio_backend_, *CHECK_NOTNULL(initial_state.get()),
          stereo_imu_sync_packet.getImuAccGyr(),
          stereo_imu_sync_packet.getStereoFrame().getTimestamp());
      // Store latest pose estimate.
      logger_.W_Pose_Bprevkf_vio_ = vio_backend_->getWPoseBLkf();
    }
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
//////////////////// UNIFY

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
  // TODO(marcus): send these to the LCD input thread as well
  // TODO (Sandro): Adapt to be able to batch pop frames for batch backend
  // Pull from stereo frontend output queue.
  LOG(INFO) << "Spinning wrapped thread.";
  while(!shutdown_) {
    std::shared_ptr<StereoFrontEndOutputPayload> stereo_frontend_output_payload;
    stereo_frontend_output_payload = stereo_frontend_output_queue_.pop();
    if (!stereo_frontend_output_payload) {
      VLOG(100) << "Missing frontend output payload.";
      continue;
    }
    CHECK(stereo_frontend_output_payload);
    if (!stereo_frontend_output_payload->is_keyframe_) {
      continue;
    }
    CHECK(stereo_frontend_output_payload->is_keyframe_);

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

    backend_thread_ = VIO::make_unique<std::thread>(
        &VioBackEnd::spin,
        // Returns the pointer to vio_backend_.
        CHECK_NOTNULL(vio_backend_.get()), std::ref(backend_input_queue_),
        std::ref(backend_output_queue_), true);

    mesher_thread_ = VIO::make_unique<std::thread>(
        &Mesher::spin, &mesher_, std::ref(mesher_input_queue_),
        std::ref(mesher_output_queue_), true);

    lcd_thread_ = VIO::make_unique<std::thread>(
        &LoopClosureDetector::spin, CHECK_NOTNULL(loop_closure_detector_.get()),
        std::ref(lcd_input_queue_), std::ref(lcd_output_queue_), true);

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

    LOG(INFO) << "Restarting loop closure workers and queues...";
    lcd_input_queue_.resume();
    lcd_output_queue_.resume();

    LOG(INFO) << "Restarting visualizer workers and queues...";
    visualizer_input_queue_.resume();
    visualizer_output_queue_.resume();

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

  LOG(INFO) << "Stopping loop closure workers and queues...";
  lcd_input_queue_.shutdown();
  lcd_output_queue_.shutdown();
  loop_closure_detector_->shutdown();

  LOG(INFO) << "Stopping visualizer workers and queues...";
  visualizer_input_queue_.shutdown();
  visualizer_output_queue_.shutdown();
  visualizer_.shutdown();

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
  if (lcd_thread_ && lcd_thread_->joinable()) {
    lcd_thread_->join();
    LOG(INFO) << "Joined loop closure thread...";
  } else {
    LOG_IF(ERROR, parallel_run_) << "Loop closure thread is not joinable...";
  }

  // visualizer_thread_.join();

  LOG(INFO) << "All threads joined.";
}

}  // namespace VIO
