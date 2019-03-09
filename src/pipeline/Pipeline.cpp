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

#include <future>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "utils/Timer.h"
#include "utils/Statistics.h"
#include "RegularVioBackEnd.h"

DEFINE_bool(log_output, false, "Log output to matlab.");
DEFINE_bool(parallel_run, false, "Run parallelized pipeline.");
DEFINE_int32(regular_vio_backend_modality, 4,
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
DEFINE_int32(viz_type, 0,
  "\n0: POINTCLOUD, visualize 3D VIO points (no repeated point)\n"
  "1: POINTCLOUD_REPEATEDPOINTS, visualize VIO points as point clouds (points "
    "are re-plotted at every frame)\n"
  "2: MESH2D, only visualizes 2D mesh on image\n"
  "3: MESH2DTo3D, get a 3D mesh from a 2D triangulation of the (right-VALID) "
    "keypoints in the left frame\n"
  "4: MESH2Dsparse, visualize a 2D mesh of (right-valid) keypoints discarding "
    "triangles corresponding to non planar obstacles\n"
  "5: MESH2DTo3Dsparse, same as MESH2DTo3D but filters out triangles "
    "corresponding to non planar obstacles\n"
  "6: MESH3D, 3D mesh from CGAL using VIO points (requires #define USE_CGAL!)\n"
  "7: NONE, does not visualize map\n");
DEFINE_bool(record_video_for_viz_3d, false, "Record a video as a sequence of "
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

namespace VIO {

// TODO VERY BAD TO SEND THE DATASET PARSER AS A POINTER (at least for thread
// safety!), but this is done now because the logger heavily relies on the
// dataset parser, especially the grount-truth! Feature selector also has some
// dependency with this...
Pipeline::Pipeline(ETHDatasetParser* dataset,
                   const ImuParams& imu_params)
  : dataset_(CHECK_NOTNULL(dataset)) {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();
  if (FLAGS_log_output) logger_.openLogFiles();

  frontend_params_ = dataset_->getFrontendParams();
  backend_params_ = dataset_->getBackendParams();

  // Instantiate stereo tracker (class that tracks implements estimation
  // front-end) and print parameters.
  // TODO remove hardcoded saveImages, use gflag.
  static constexpr int saveImages = 0; // 0: don't show, 1: show, 2: write & save
  vio_frontend_ = VIO::make_unique<StereoVisionFrontEnd>(
        imu_params,
        // This should not be asked!
        dataset->getGroundTruthState(dataset_->timestamp_first_lkf_).imu_bias_,
        frontend_params_,
        saveImages,
        dataset_->getDatasetName(),
        FLAGS_log_output);

  // Instantiate feature selector: not used in vanilla implementation.
  if (FLAGS_use_feature_selection) {
    feature_selector_ = FeatureSelector(dataset_->getFrontendParams(),
                                        *backend_params_);
  }
}

/* -------------------------------------------------------------------------- */
Pipeline::~Pipeline() {
  // Shutdown pipeline if it is not already down.
  if (!shutdown_) shutdown();
}

/* -------------------------------------------------------------------------- */
bool Pipeline::spin(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  static bool is_initialized = false;
  if (!is_initialized) {
    LOG(INFO) << "Initialize VIO pipeline.";
    // Initialize pipeline.
    // TODO this is very brittle, because we are accumulating IMU data, but
    // not using it for initialization, because accumulated and actual IMU data
    // at init is the same...
    initialize(stereo_imu_sync_packet);
    // Launch threads.
    launchThreads();
    is_initialized = true;
    return true;
  }

  // TODO Warning: we do not accumulate IMU measurements for the first packet...
  // Spin.
  VLOG(10) << "Spin pipeline once.";
  auto tic = utils::Timer::tic();
  spinOnce(stereo_imu_sync_packet);
  auto spin_duration = utils::Timer::toc(tic).count();
  LOG(WARNING) << "Current overall Pipeline frequency: "
               << 1000.0 / spin_duration << " Hz. ("
               << spin_duration << " ms).";
  utils::StatsCollector stats_pipeline("Pipeline overall Timing [ms]");
  stats_pipeline.AddSample(spin_duration);
  return true;
}

/* -------------------------------------------------------------------------- */
// Spin the pipeline only once.
void Pipeline::spinOnce(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  ////////////////////////////// FRONT-END /////////////////////////////////////
  // Push to stereo frontend input queue.
  stereo_frontend_input_queue_.push(StereoFrontEndInputPayload(
                                      stereo_imu_sync_packet));

  // Pull from stereo frontend output queue.
  std::shared_ptr<StereoFrontEndOutputPayload> stereo_frontend_output_payload;
  if (FLAGS_parallel_run) {
     stereo_frontend_output_payload = stereo_frontend_output_queue_.pop();
  } else {
     stereo_frontend_output_payload = stereo_frontend_output_queue_.popBlocking();
  }
  if (!stereo_frontend_output_payload) {
    LOG(WARNING) << "Missing frontend output payload.";
    return;
  }
  CHECK(stereo_frontend_output_payload);
  if (!stereo_frontend_output_payload->is_keyframe_) {
    return;
  }
  //////////////////////////////////////////////////////////////////////////////
  // So from this point on, we have a keyframe.
  // Pass info to VIO
  // !!!!! Don't use stereo_imu_sync_packet passed this point because it is a
  // packet that is way further ahead in time than the keyframe packer since
  // the pipeline keeps spinning while the frontend works   !!!!!!!


  //////////////////////////////////////////////////////////////////////////////
  // Actual keyframe processing. Call to backend.
  ////////////////////////////// BACK-END //////////////////////////////////////
  processKeyframe(
        stereo_frontend_output_payload->statusSmartStereoMeasurements_,
        stereo_frontend_output_payload->stereo_frame_lkf_,
        stereo_frontend_output_payload->pim_,
        stereo_frontend_output_payload->tracker_status_,
        stereo_frontend_output_payload->relative_pose_body_stereo_);
}

/* -------------------------------------------------------------------------- */
void Pipeline::processKeyframe(
    const StatusSmartStereoMeasurements& statusSmartStereoMeasurements,
    const StereoFrame& last_stereo_keyframe,
    const ImuFrontEnd::PreintegratedImuMeasurements& pim,
    const Tracker::TrackingStatus& kf_tracking_status_stereo,
    const gtsam::Pose3& relative_pose_body_stereo) {
  // At this point stereoFrame_km1 == stereoFrame_lkf_ !
  const Frame& last_left_keyframe = last_stereo_keyframe.getLeftFrame();

  //////////////////// BACK-END ////////////////////////////////////////////////
  // Push to backend input.
  // This should be done inside the frontend!!!!
  // Or the backend should pull from the frontend!!!!
  backend_input_queue_.push(
        VioBackEndInputPayload(
          last_stereo_keyframe.getTimestamp(),
          statusSmartStereoMeasurements,
          kf_tracking_status_stereo,
          pim,
          &planes_,
          relative_pose_body_stereo));

  // This should be done inside those who need the backend results
  // IN this case the logger!!!!!
  // But there are many more people that want backend results...
  // Pull from backend.
  std::shared_ptr<VioBackEndOutputPayload> backend_output_payload =
      backend_output_queue_.popBlocking();
  LOG_IF(WARNING, !backend_output_payload) << "Missing backend output payload.";

  ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////////
  std::vector<cv::Vec6f> mesh_2d;
  VioBackEnd::PointsWithIdMap points_with_id_VIO;
  VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
  MesherOutputPayload mesher_output_payload;
  std::vector<Point3> points_3d;
  VioBackEnd::PointsWithIdMap points_with_id;

  // Decide mesh computation:
  VisualizationType visualization_type =
      static_cast<VisualizationType>(FLAGS_viz_type);
  switch (visualization_type) {
  case VisualizationType::MESH2D: {
    mesh_2d = last_left_keyframe.createMesh2D(); // TODO pass the visualization flag inside the while loop of frontend and call this.
    break;
  }
    // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
  case VisualizationType::MESH2Dsparse: {
    last_stereo_keyframe.createMesh2dStereo(&mesh_2d); // TODO same as above.
    break;
  }
  case VisualizationType::MESH2DTo3Dsparse: {
    // Points_with_id_VIO contains all the points in the optimization,
    // (encoded as either smart factors or explicit values), potentially
    // restricting to points seen in at least min_num_obs_fro_mesher_points keyframes
    // (TODO restriction is not enforced for projection factors).
    vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon(
          &points_with_id_VIO,
          FLAGS_visualize_lmk_type?
            &lmk_id_to_lmk_type_map:nullptr,
          FLAGS_min_num_obs_for_mesher_points);

    // Get camera pose.
    gtsam::Pose3 W_Pose_camlkf_vio =
        vio_backend_->getWPoseBLkf().compose(vio_backend_->getBPoseLeftCam());

    // Create and fill data packet for mesher.

    // Push to queue.
    // In another thread, mesher is running, consuming mesher payloads.
    CHECK(mesher_input_queue_.push(MesherInputPayload (
                                     points_with_id_VIO, //copy, thread safe, read-only. // This should be a popBlocking...
                                     last_stereo_keyframe, // not really thread safe, read only.
                                     W_Pose_camlkf_vio))); // copy, thread safe, read-only. // Same, popBlocking

    // Find regularities in the mesh if we are using RegularVIO backend.
    // TODO create a new class that is mesh segmenter or plane extractor.
    if (dataset_->getBackendType() == 1 &&
        FLAGS_extract_planes_from_the_scene) {
      mesher_.clusterPlanesFromMesh(&planes_,
                                    points_with_id_VIO);
    }

    // In the mesher thread push queue with meshes for visualization.
    if (!mesher_output_queue_.popBlocking(mesher_output_payload)) { //Use blocking to avoid skipping frames.
      LOG(WARNING) << "Mesher output queue did not pop a payload.";
    }
    break;
  }
  case VisualizationType::POINTCLOUD_REPEATEDPOINTS: {// visualize VIO points as point clouds (points are replotted at every frame)
    points_3d = vio_backend_->get3DPoints();
    break;
  }
    // Computes and visualizes a 3D point cloud.
  case VisualizationType::POINTCLOUD: {// visualize VIO points  (no repeated point)
    vio_backend_->getMapLmkIdsTo3dPointsInTimeHorizon(&points_with_id_VIO);
    break;
  }
  case VisualizationType::NONE: {break;}
  }

  if (FLAGS_visualize) {
    LOG_IF(WARNING, semantic_mesh_segmentation_callback_)
        << "Coloring the mesh using semantic segmentation colors.";
    // Push data for visualizer thread.
    // WHO Should be pushing to the visualizer input queue????????
    // This cannot happen at all from a single module, because visualizer
    // takes input from mesher and backend right now...
    visualizer_input_queue_.push(
          VisualizerInputPayload(
            visualization_type, // This should be passed at ctor level....
            dataset_->getBackendType(),// This should be passed at ctor level....
            // Pose for trajectory viz.
            vio_backend_->getWPoseBLkf() * // The visualizer needs backend results
            last_stereo_keyframe.getBPoseCamLRect(), // This should be pass at ctor level....
            // For visualizeMesh2D and visualizeMesh2DStereo
            mesh_2d, // The visualizer needs mesher results
            // Call semantic mesh segmentation if someone registered a callback.
            semantic_mesh_segmentation_callback_? // This callback should be given to the visualizer at ctor level....
              semantic_mesh_segmentation_callback_(
                last_left_keyframe.timestamp_,
                last_left_keyframe.img_,
                mesher_output_payload.mesh_2d_, // The visualizer needs mesher results, but we are already passing mesher_output_payload, visualizer should popBlocking that...
                mesher_output_payload.mesh_3d_) :
              Visualizer3D::texturizeMesh3D(
                last_left_keyframe.timestamp_,
                last_left_keyframe.img_,
                mesher_output_payload.mesh_2d_,
                mesher_output_payload.mesh_3d_),
            // For visualizeMesh2D and visualizeMesh2DStereo.
            last_left_keyframe,
            // visualizeConvexHull & visualizeMesh3DWithColoredClusters
            // WARNING using move explicitly!
            std::move(mesher_output_payload),
            // visualizeMesh3DWithColoredClusters & visualizePoints3D
            points_with_id_VIO,
            // visualizeMesh3DWithColoredClusters & visualizePoints3D
            lmk_id_to_lmk_type_map,
            planes_,  // visualizeMesh3DWithColoredClusters
            vio_backend_->getFactorsUnsafe(), // For plane constraints viz.
            vio_backend_->getState(), // For planes and plane constraints viz.
            points_3d,
            last_stereo_keyframe.getTimestamp()
            ));

      // Get data from visualizer thread.
      // We use non-blocking pop() because no one depends on the output
      // of the visualizer. This way the pipeline can keep running, while the
      // visualization is only done when there is data available.
      spinDisplayOnce(visualizer_output_queue_.popBlocking());
  }
}

/* -------------------------------------------------------------------------- */
bool Pipeline::spinSequential() {
  LOG(FATAL) << "Spin sequential is not yet available.";
}

/* -------------------------------------------------------------------------- */
void Pipeline::shutdownWhenFinished() {
  // This is a very rough way of knowing if we have finished...
  // Since threads might be in the middle of processing data while we
  // query if the queues are empty.
  // Check every second if all queues are empty.
  // Time to sleep between queries to the queues [in seconds].
  LOG(INFO) << "Shutting down VIO pipeline once processing has finished.";
  static constexpr int sleep_time = 1;
  while(!(stereo_frontend_input_queue_.empty() &&
          stereo_frontend_output_queue_.empty() &&
          !vio_frontend_->isWorking() &&
          backend_input_queue_.empty() &&
          backend_output_queue_.empty() &&
          !vio_backend_->isWorking() &&
          mesher_input_queue_.empty() &&
          mesher_output_queue_.empty() &&
          !mesher_.isWorking() &&
          visualizer_input_queue_.empty() &&
          visualizer_output_queue_.empty() &&
          !visualizer_.isWorking())) {
    VLOG_EVERY_N(10, 100)
        << "VIO pipeline status: \n"
        << "Frontend input queue empty?" << stereo_frontend_input_queue_.empty() << '\n'
        << "Frontend output queue empty?" << stereo_frontend_output_queue_.empty()<< '\n'
        << "Frontend is working? " <<  vio_frontend_->isWorking()<< '\n'
        << "Backend Input queue empty?" <<  backend_input_queue_.empty()<< '\n'
        << "Backend Output queue empty?" <<  backend_output_queue_.empty()<< '\n'
        << "Backend is working? " <<  vio_backend_->isWorking()<< '\n'
        << "Mesher input queue empty?" <<  mesher_input_queue_.empty()<< '\n'
        << "Mesher output queue empty?" <<  mesher_output_queue_.empty()<< '\n'
        << "Mesher is working? " <<  mesher_.isWorking()<< '\n'
        << "Visualizer input queue empty?" << visualizer_input_queue_.empty()<< '\n'
        << "Visualizer is working? " << visualizer_.isWorking();
        std::this_thread::sleep_for (std::chrono::seconds(sleep_time));
  }
  shutdown();
}

/* -------------------------------------------------------------------------- */
void Pipeline::shutdown() {
  LOG(INFO) << "Shutting down VIO pipeline.";
  shutdown_ = true;
  stopThreads();
  joinThreads();
  LOG(INFO) << "All threads joined, successful VIO pipeline shutdown.";
  if (FLAGS_log_output) {
    logger_.closeLogFiles();
  }
}

/* -------------------------------------------------------------------------- */
bool Pipeline::initialize(const StereoImuSyncPacket& stereo_imu_sync_packet) {
  LOG(INFO) << "------------------- Initialize Pipeline with frame k = "
            << stereo_imu_sync_packet.getStereoFrame().getFrameId()
            << "--------------------";

  /////////////////// FRONTEND /////////////////////////////////////////////////
  // Initialize Stereo Frontend.
  const StereoFrame& stereo_frame_lkf =
      vio_frontend_->processFirstStereoFrame(
        stereo_imu_sync_packet.getStereoFrame());

  ///////////////////////////// BACKEND ////////////////////////////////////////
  // Initialize Backend.
  std::shared_ptr<gtNavState> initialStateGT =
      dataset_->isGroundTruthAvailable()?
        std::make_shared<gtNavState>(dataset_->getGroundTruthState(
                     stereo_imu_sync_packet.getStereoFrame().getTimestamp())) :
                     std::shared_ptr<gtNavState>(nullptr);

  initBackend(&vio_backend_,
              stereo_frame_lkf.getBPoseCamLRect(),
              stereo_frame_lkf.getLeftUndistRectCamMat(),
              stereo_frame_lkf.getBaseline(),
              *backend_params_,
              &initialStateGT,
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
          *dataset_,
          vio_backend_,
          *CHECK_NOTNULL(initialStateGT.get()),
          stereo_imu_sync_packet.getImuAccGyr(),
          stereo_imu_sync_packet.getStereoFrame().getTimestamp());
    // Store latest pose estimate.
    logger_.W_Pose_Bprevkf_vio_ = vio_backend_->getWPoseBLkf();
  }

  return true;
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
  switch(dataset_->getBackendType()) {
  case 0: {
    LOG(INFO) << "\e[1m Using Normal VIO. \e[0m";
    *vio_backend = VIO::make_unique<VioBackEnd>(B_Pose_camLrect,
                                           left_undist_rect_cam_mat,
                                           baseline,
                                           initial_state_gt,
                                           timestamp_k,
                                           imu_accgyr,
                                           vio_params,
                                           FLAGS_log_output);
    break;
  }
  case 1: {
    LOG(INFO) << "\e[1m Using Regular VIO with modality "
              << FLAGS_regular_vio_backend_modality << "\e[0m";
    *vio_backend = VIO::make_unique<RegularVioBackEnd>(
          B_Pose_camLrect,
          left_undist_rect_cam_mat,
          baseline,
          initial_state_gt,
          timestamp_k,
          imu_accgyr,
          vio_params, FLAGS_log_output,
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
    const std::shared_ptr<VisualizerOutputPayload>& visualizer_output_payload) {
  // Display only if the visualizer has done its work.
  if (visualizer_output_payload) {
    // Display 3D window.
    if (visualizer_output_payload->visualization_type_ !=
        VisualizationType::NONE) {
      VLOG(10) << "Spin Visualize 3D output.";
      //visualizer_output_payload->window_.spin();
      visualizer_output_payload->window_.spinOnce(1, true);
      // TODO this is not very thread-safe!!! Since recordVideo might modify
      // window_ in this thread, while it might also be called in viz thread.
      if (FLAGS_record_video_for_viz_3d) {
        visualizer_.recordVideo();
      }
    }

    // Display 2D images.
    for (const ImageToDisplay& img_to_display:
         visualizer_output_payload->images_to_display_) {
      cv::imshow(img_to_display.name_, img_to_display.image_);
    }
    VLOG(10) << "Spin Visualize 2D output.";
    cv::waitKey(1);
  } else {
    LOG(WARNING) << "Visualizer is lagging behind pipeline processing.";
  }
}

/* -------------------------------------------------------------------------- */
StatusSmartStereoMeasurements Pipeline::featureSelect(
    const VioFrontEndParams& tracker_params,
    const ETHDatasetParser& dataset,
    const Timestamp& timestamp_k,
    const Timestamp& timestamp_lkf,
    const gtsam::Pose3& W_Pose_Blkf,
    double* feature_selection_time,
    std::shared_ptr<StereoFrame>& stereoFrame_km1,
    const StatusSmartStereoMeasurements& status_smart_stereo_meas,
    int cur_kf_id,
    int save_image_selector,
    const gtsam::Matrix& curr_state_cov,
    const Frame& left_frame) { // last one for visualization only
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
    W_Pose_Bkf_gt= dataset.getGroundTruthState(timestamp_lkf).pose_;

    for (size_t kk = 0; kk < nrKfInHorizon + 1; kk++) {
      // Including current pose.
      Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(
            kk * tracker_params.intra_keyframe_time_);

      // Relative pose wrt ground truth at last kf.
      Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(
            dataset.getGroundTruthState(timestamp_kk).pose_);
      posesAtFutureKeyframes.push_back(
            StampedPose(W_Pose_Blkf.compose(poseGT_km1_kk),
                        UtilsOpenCV::NsecToSec(timestamp_kk)) );
    }
  }

  VLOG(100) << "Starting feature selection...";
  SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
  std::tie(trackedAndSelectedSmartStereoMeasurements, *feature_selection_time)
      = feature_selector_.splitTrackedAndNewFeatures_Select_Display(
        stereoFrame_km1,
        status_smart_stereo_meas.second,
        cur_kf_id, save_image_selector,
        tracker_params.featureSelectionCriterion_,
        tracker_params.featureSelectionNrCornersToSelect_,
        tracker_params.maxFeatureAge_,
        posesAtFutureKeyframes, // TODO Luca: can we make this optional, for the case where we do not have ground truth?
        curr_state_cov,
        dataset.getDatasetName(),
        left_frame); // last 2 are for visualization
  VLOG(100) << "Feature selection completed.";

  // Same status as before.
  TrackerStatusSummary status = status_smart_stereo_meas.first;
  return std::make_pair(status, trackedAndSelectedSmartStereoMeasurements);
}

/* -------------------------------------------------------------------------- */
void Pipeline::launchThreads() {
  LOG(INFO) << "Launching threads...";

  // Start frontend_thread.
  stereo_frontend_thread_ = std::thread(
                              &StereoVisionFrontEnd::spin,
                              CHECK_NOTNULL(vio_frontend_.get()),
                              std::ref(stereo_frontend_input_queue_),
                              std::ref(stereo_frontend_output_queue_));

  // Start backend_thread.
  backend_thread_ = std::thread(&VioBackEnd::spin,
                                // Returns the pointer to vio_backend_.
                                CHECK_NOTNULL(vio_backend_.get()),
                                std::ref(backend_input_queue_),
                                std::ref(backend_output_queue_));

  // Start mesher_thread.
  mesher_thread_ = std::thread(&Mesher::spin,
                               &mesher_,
                               std::ref(mesher_input_queue_),
                               std::ref(mesher_output_queue_));

  // Start visualizer_thread.
  visualizer_thread_ = std::thread(&Visualizer3D::spin,
                                   &visualizer_,
                                   std::ref(visualizer_input_queue_),
                                   std::ref(visualizer_output_queue_));
}

/* -------------------------------------------------------------------------- */
void Pipeline::stopThreads() {
  // Shutdown workers and queues.
  stereo_frontend_input_queue_.shutdown();
  stereo_frontend_output_queue_.shutdown();
  vio_frontend_->shutdown();

  backend_input_queue_.shutdown();
  backend_output_queue_.shutdown();
  vio_backend_->shutdown();

  mesher_input_queue_.shutdown();
  mesher_output_queue_.shutdown();
  mesher_.shutdown();

  visualizer_input_queue_.shutdown();
  visualizer_output_queue_.shutdown();
  visualizer_.shutdown();
}

/* -------------------------------------------------------------------------- */
void Pipeline::joinThreads() {
  stereo_frontend_thread_.join();
  backend_thread_.join();
  mesher_thread_.join();
  visualizer_thread_.join();
}

} // End of VIO namespace
