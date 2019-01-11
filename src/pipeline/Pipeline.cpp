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
#include <chrono>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>

#include "RegularVioBackEnd.h"

DEFINE_bool(log_output, false, "Log output to matlab.");
DEFINE_bool(parallel_run, true, "Run parallelized pipeline.");
DEFINE_int32(backend_type, 0, "Type of vioBackEnd to use:\n"
                                 "0: VioBackEnd\n"
                                 "1: RegularVioBackEnd");
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

DEFINE_bool(use_feature_selection, false, "Enable smart feature selection.");

DEFINE_bool(deterministic_random_number_generator, false,
            "If true the random number generator will consistently output the "
            "same sequence of pseudo-random numbers for every run (use it to "
            "have repeatable output). If false the random number generator "
            "will output a different sequence for each run.");
DEFINE_int32(min_num_obs_for_mesher_points, 4,
             "Minimum number of observations for a smart factor's landmark to "
             "to be used as a 3d point to consider for the mesher");

// Add compatibility for c++11's lack of make_unique.
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique( Args&& ...args ) {
  return std::unique_ptr<T>( new T( std::forward<Args>(args)... ) );
}

namespace VIO {
Pipeline::Pipeline() {
  if (FLAGS_deterministic_random_number_generator) setDeterministicPipeline();
  // Set backend params.
  setBackendType(FLAGS_backend_type, &vio_params_);

  // Parse dataset.
  dataset_.parse(&initial_k_, &final_k_);
  // Parse parameters.
  dataset_.parseParams(vio_params_, dataset_.imuData_, &tracker_params_);

  // Instantiate stereo tracker (class that tracks implements estimation
  // front-end) and print parameters.
  static constexpr int saveImages = 0; // 0: don't show, 1: show, 2: write & save
  stereo_vision_frontend_ = make_unique<StereoVisionFrontEnd>(
        tracker_params_, *vio_params_, // vioParams used by feature selection
        saveImages, dataset_.getDatasetName());

  // Instantiate feature selector: not used in vanilla implementation.
  feature_selector_ = FeatureSelector(tracker_params_, *vio_params_);

  // Open log files.
  if (FLAGS_log_output) {
    logger_.openLogFiles();
  }

  // Timestamp 10 frames before the first (for imu calibration)
  // TODO: remove hardcoded 10
  static constexpr size_t frame_offset_for_imu_calib = 10;
  CHECK_GE(initial_k_, frame_offset_for_imu_calib)
      << "Initial frame " << initial_k_ << " has to be larger than "
      << frame_offset_for_imu_calib << " (needed for IMU calibration)";
  timestamp_lkf_ = dataset_.timestampAtFrame(
        initial_k_ - frame_offset_for_imu_calib);
}

bool Pipeline::spin() {
  // Initialize pipeline.
  initialize(initial_k_);

  // Launch threads.
  launchThreads();

  // Run main loop.
  auto start = std::chrono::high_resolution_clock::now();
  for(size_t k = initial_k_ + 1; k < final_k_; k++) { // for each image
    spinOnce(k);
  }
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  LOG(WARNING) << "Spin took: " << duration_ms.count() << " milliseconds.";

  // Shutdown.
  shutdown();

  LOG(INFO) << "Spin completed successfully!";
  return true;
}

// Spin the pipeline only once.
void Pipeline::spinOnce(size_t k) {
  LOG(INFO) << "------------------- Processing frame k = " << k << "--------------------";
  const std::string& left_img_name (dataset_.getLeftImgName(k));
  const std::string& right_img_name (dataset_.getRightImgName(k));
  timestamp_k_ = dataset_.timestampAtFrame(k); // same in both images

  /////////////////// FRONTEND ///////////////////////////////////////////
  // Load stereo images.
  start_time_ = UtilsOpenCV::GetTimeInSeconds();
  StereoFrame stereoFrame_k(
        k, timestamp_k_,
        left_img_name, right_img_name,
        dataset_.getLeftCamInfo(), dataset_.getRightCamInfo(),
        dataset_.camL_Pose_camR_,
        tracker_params_.getStereoMatchingParams());

  if (FLAGS_log_output) {
    logger_.timing_loadStereoFrame_ =
        UtilsOpenCV::GetTimeInSeconds() - start_time_;
  }

  ////////////////////////////////////////////////////////////////////////////
  // For k > 1
  // Integrate rotation measurements (rotation is used in RANSAC).
  std::tie(imu_stamps_, imu_accgyr_) = dataset_.imuData_.imu_buffer_.
      getBetweenValuesInterpolated(timestamp_lkf_,
                                   timestamp_k_);
  gtsam::Rot3 calLrectLkf_R_camLrectKf_imu = vio_backend_->
      preintegrateGyroMeasurements(imu_stamps_,
                                   imu_accgyr_);

  ////////////////////////////// FRONT-END ///////////////////////////////////
  // Main function for tracking.
  start_time_ = UtilsOpenCV::GetTimeInSeconds();

  // Rotation used in 1 and 2 point ransac.
  StatusSmartStereoMeasurements statusSmartStereoMeasurements =
      stereo_vision_frontend_->processStereoFrame(stereoFrame_k,
                                                  calLrectLkf_R_camLrectKf_imu);

  if (FLAGS_log_output) {
    logger_.timing_processStereoFrame_ =
        UtilsOpenCV::GetTimeInSeconds() - start_time_;
  }

  // Pass info to VIO if it's keyframe.
  start_time_ = UtilsOpenCV::GetTimeInSeconds();
  if (stereo_vision_frontend_->stereoFrame_km1_->isKeyframe_) {
    // It's a keyframe!
    LOG(INFO) << "Keyframe " << k << " with: "
              << statusSmartStereoMeasurements.second.size()
              << " smart measurements";

    ////////////////////////////// FEATURE SELECTOR //////////////////////////
    if (FLAGS_use_feature_selection) {
      start_time_ = UtilsOpenCV::GetTimeInSeconds();
      // !! featureSelect is not thread safe !! Do not use when running in
      // parallel mode.
      static constexpr int saveImagesSelector = 1; // 0: don't show, 2: write & save
      statusSmartStereoMeasurements = featureSelect(
            tracker_params_,
            dataset_,
            timestamp_k_,
            vio_backend_->W_Pose_Blkf_,
            &(stereo_vision_frontend_->tracker_.debugInfo_.featureSelectionTime_),
            *(stereo_vision_frontend_->stereoFrame_km1_),
            statusSmartStereoMeasurements,
            vio_backend_->cur_kf_id_, (FLAGS_visualize?saveImagesSelector:0),
            vio_backend_->getCurrentStateCovariance(),
            stereo_vision_frontend_->stereoFrame_lkf_->left_frame_); // last one for visualization only
      if (FLAGS_log_output) {
        VLOG(100)
            << "Overall selection time (logger.timing_featureSelection_) "
            << logger_.timing_featureSelection_ << '\n'
            << "actual selection time (stereoTracker.tracker_.debugInfo_."
            << "featureSelectionTime_) "
            << stereo_vision_frontend_->tracker_.debugInfo_.featureSelectionTime_;
        logger_.timing_featureSelection_ =
            UtilsOpenCV::GetTimeInSeconds() - start_time_;
      }
    } else {
      VLOG(100) << "Not using feature selection.";
    }

    ////////////////// DEBUG INFO FOR FRONT-END //////////////////////////////
    start_time_ = UtilsOpenCV::GetTimeInSeconds();

    if (FLAGS_log_output) {
      logger_.logFrontendResults(dataset_, *stereo_vision_frontend_, timestamp_lkf_, // TODO this copies the whole frontend!!
                                 timestamp_k_);
      logger_.timing_loggerFrontend_ =
          UtilsOpenCV::GetTimeInSeconds() - start_time_;
    }
    ////////////////////////////////////////////////////////////////////////////

    // Get IMU data.
    std::tie(imu_stamps_, imu_accgyr_) = dataset_.imuData_.imu_buffer_
        .getBetweenValuesInterpolated(timestamp_lkf_,
                                      timestamp_k_);

    //////////////////// BACK-END ////////////////////////////////////////////
    // Push to backend input.
    backend_input_queue_.push(
          VioBackEndInputPayload(
            timestamp_k_,
            statusSmartStereoMeasurements,
            stereo_vision_frontend_->trackerStatusSummary_.kfTrackingStatus_stereo_,
            imu_stamps_,
            imu_accgyr_,
            &planes_,
            stereo_vision_frontend_->getRelativePoseBodyStereo()));

    // Pull from backend.
    std::shared_ptr<VioBackEndOutputPayload> backend_output_payload =
            backend_output_queue_.popBlocking();

    // Log backend results.
    if (FLAGS_log_output) {
      logger_.timing_vio_ = UtilsOpenCV::GetTimeInSeconds() - start_time_;

      ////////////////// DEBUG INFO FOR BACK-END /////////////////////////////
      start_time_ = UtilsOpenCV::GetTimeInSeconds();
      logger_.logBackendResults(dataset_, *stereo_vision_frontend_,
                                backend_output_payload,
                                vio_params_->horizon_,
                                timestamp_lkf_, timestamp_k_,k);
      logger_.W_Pose_Bprevkf_vio_ = vio_backend_->W_Pose_Blkf_;
      logger_.timing_loggerBackend_ =
          UtilsOpenCV::GetTimeInSeconds() - start_time_;
      logger_.displayOverallTiming();
    }

    ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////
    std::vector<cv::Vec6f> mesh_2d;
    VioBackEnd::PointsWithIdMap points_with_id_VIO;
    VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
    MesherOutputPayload mesher_output_payload;
    vector<Point3> points_3d;
    VioBackEnd::PointsWithIdMap points_with_id;

    // Decide mesh computation:
    VisualizationType visualization_type =
        static_cast<VisualizationType>(FLAGS_viz_type);
    switch (visualization_type) {
    case VisualizationType::MESH2D: {
      stereo_vision_frontend_->stereoFrame_lkf_->left_frame_.createMesh2D( // TODO pass the visualization flag inside the while loop of frontend and call this.
                                                                           &mesh_2d);
      break;
    }
      // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
    case VisualizationType::MESH2Dsparse: {
      stereo_vision_frontend_->stereoFrame_lkf_->createMesh2dStereo(&mesh_2d); // TODO same as above.
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
          vio_backend_->W_Pose_Blkf_.compose(vio_backend_->B_Pose_leftCam_);

      // Create and fill data packet for mesher.

      // Push to queue.
      // In another thread, mesher is running, consuming mesher payloads.
      CHECK(mesher_input_queue_.push(MesherInputPayload (
                                       points_with_id_VIO, //copy, thread safe, read-only. // This should be a popBlocking...
                                       *(stereo_vision_frontend_->stereoFrame_lkf_), // not really thread safe, read only.
                                       W_Pose_camlkf_vio))); // copy, thread safe, read-only. // Same, popBlocking

      // Find regularities in the mesh if we are using RegularVIO backend.
      // TODO create a new class that is mesh segmenter or plane extractor.
      if (FLAGS_backend_type == 1) {
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
      // Push data for visualizer thread.
      visualizer_input_queue_.push(VisualizerInputPayload(
                                     visualization_type, FLAGS_backend_type,
                                     vio_backend_->W_Pose_Blkf_ * stereo_vision_frontend_->stereoFrame_km1_->B_Pose_camLrect_, // pose for trajectory viz.
                                     mesh_2d, // for visualizeMesh2D and visualizeMesh2DStereo
                                     stereo_vision_frontend_->stereoFrame_lkf_->left_frame_, // for visualizeMesh2D and visualizeMesh2DStereo
                                     mesher_output_payload, // visualizeConvexHull & visualizeMesh3DWithColoredClusters
                                     points_with_id_VIO, // visualizeMesh3DWithColoredClusters & visualizePoints3D
                                     lmk_id_to_lmk_type_map, // visualizeMesh3DWithColoredClusters & visualizePoints3D
                                     planes_,  // visualizeMesh3DWithColoredClusters
                                     vio_backend_->smoother_->getFactors(), // For plane constraints viz.
                                     vio_backend_->state_, // For planes and plane constraints viz.
                                     points_3d, timestamp_k_));

      // Get data from visualizer thread.
      // We use non-blocking pop() because no one depends on the output
      // of the visualizer. This way the pipeline can keep running, while the
      // visualization is only done when there is data available.
      std::shared_ptr<VisualizerOutputPayload> visualizer_output_payload =
          visualizer_output_queue_.popBlocking();
      // Display only if the visualizer has done its work.
      if (visualizer_output_payload) {
        // Display 3D window.
        if (visualization_type != VisualizationType::NONE) {
          visualizer_output_payload->window_.spinOnce(1, true);
        }

        // Display 2D images.
        for (const ImageToDisplay& img_to_display:
             visualizer_output_payload->images_to_display_) {
          cv::imshow(img_to_display.name_, img_to_display.image_);
        }
        cv::waitKey(1);
      } else {
        LOG(WARNING) << "Visualizer is lagging behind pipeline processing.";
      }
    }

    timestamp_lkf_ = timestamp_k_;
  }
}

bool Pipeline::spinSequential() {
  LOG(FATAL) << "Spin sequential is not yet available.";
}

void Pipeline::shutdown() {
  LOG(INFO) << "Shutting down VIO pipeline.";
  shutdown_ = true;
  stopThreads();
  joinThreads();
  if (FLAGS_log_output) {
    logger_.closeLogFiles();
  }
}

void Pipeline::setBackendType(int backend_type,
                              std::shared_ptr<VioBackEndParams>* vioParams) const {
  CHECK_NOTNULL(vioParams);
  switch(backend_type) {
  case 0: {*vioParams = std::make_shared<VioBackEndParams>(); break; }
  case 1: {*vioParams = std::make_shared<RegularVioBackEndParams>(); break; }
  default: { CHECK(false) << "Unrecognized backend type: "
                          << backend_type << "."
                          << " 0: normalVio, 1: RegularVio.";
  }
  }
}

bool Pipeline::initialize(size_t k) {
  CHECK_EQ(k, initial_k_);
  LOG(INFO) << "------------------- Initialize Pipeline with frame k = " << k << "--------------------";
  const std::string& left_img_name (dataset_.getLeftImgName(k));
  const std::string& right_img_name (dataset_.getRightImgName(k));
  timestamp_k_ = dataset_.timestampAtFrame(k); // same in both images

  // Load stereo images.
  start_time_ = UtilsOpenCV::GetTimeInSeconds();
  StereoFrame stereoFrame_k(
        k, timestamp_k_,
        left_img_name, right_img_name,
        dataset_.getLeftCamInfo(), dataset_.getRightCamInfo(),
        dataset_.camL_Pose_camR_,
        tracker_params_.getStereoMatchingParams());

  /////////////////// FRONTEND /////////////////////////////////////////////////
  // Initialize Frontend.
  initFrontend(timestamp_lkf_, timestamp_k_,
               &stereoFrame_k, stereo_vision_frontend_.get(),
               &(dataset_.imuData_.imu_buffer_), &imu_stamps_, &imu_accgyr_);

  ///////////////////////////// BACKEND ////////////////////////////////////////
  // Initialize Backend.
  std::shared_ptr<gtNavState> initialStateGT =
      dataset_.isGroundTruthAvailable()?
        std::make_shared<gtNavState>(dataset_.getGroundTruthState(timestamp_k_)) :
        std::shared_ptr<gtNavState>(nullptr);

  initBackend(&vio_backend_,
              stereo_vision_frontend_->stereoFrame_km1_->B_Pose_camLrect_,
              stereo_vision_frontend_->stereoFrame_km1_->left_undistRectCameraMatrix_,
              stereo_vision_frontend_->stereoFrame_km1_->baseline_,
              *CHECK_NOTNULL(vio_params_.get()),
              &initialStateGT,
              timestamp_k_,
              imu_accgyr_);

  ////////////////// DEBUG INITIALIZATION //////////////////////////////////
  if (FLAGS_log_output) {
    logger_.displayInitialStateVioInfo(dataset_, vio_backend_,
                                       *CHECK_NOTNULL(initialStateGT.get()),
                                       imu_accgyr_, timestamp_k_);
    // Store latest pose estimate
    logger_.W_Pose_Bprevkf_vio_ = vio_backend_->W_Pose_Blkf_;
  }

  // Store latest keyframe timestamp
  timestamp_lkf_ = timestamp_k_;
  return true;
}

bool Pipeline::initFrontend(const Timestamp& timestamp_lkf,
                            const Timestamp& timestamp_k,
                            StereoFrame* stereoFrame_k,
                            StereoVisionFrontEnd* stereo_vision_frontend,
                            ImuFrontEnd* imu_buffer,
                            ImuStamps* imu_stamps, ImuAccGyr* imu_accgyr) const {
  return initStereoFrontend(stereoFrame_k, stereo_vision_frontend) &&
         initImuFrontend(timestamp_lkf, timestamp_k,
                         imu_buffer, imu_stamps, imu_accgyr);
}

bool Pipeline::initStereoFrontend(StereoFrame* stereo_frame_k,
                        StereoVisionFrontEnd* stereo_vision_frontend) const {
  CHECK_NOTNULL(stereo_frame_k);
  CHECK_NOTNULL(stereo_vision_frontend);
  // Process first stereo frame.
  stereo_vision_frontend->processFirstStereoFrame(*stereo_frame_k);
  return true;
}


bool Pipeline::initImuFrontend(const Timestamp & timestamp_lkf,
                     const Timestamp & timestamp_k,
                     ImuFrontEnd* imu_buffer,
                     ImuStamps* imu_stamps, ImuAccGyr* imu_accgyr) const {
  CHECK_NOTNULL(imu_buffer);
  CHECK_NOTNULL(imu_stamps);
  CHECK_NOTNULL(imu_accgyr);
  // Get IMU data.
  std::tie(*imu_stamps, *imu_accgyr) =
          imu_buffer->getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);
  return true;
}

bool Pipeline::initBackend(std::shared_ptr<VioBackEnd>* vio_backend,
                           const gtsam::Pose3& B_Pose_camLrect,
                           const gtsam::Cal3_S2& left_undist_rect_cam_mat,
                           const double& baseline,
                           const VioBackEndParams& vio_params,
                           std::shared_ptr<gtNavState>* initial_state_gt,
                           const Timestamp& timestamp_k,
                           const ImuAccGyr& imu_accgyr) {
  CHECK_NOTNULL(vio_backend);
  // Create VIO.
  switch(FLAGS_backend_type) {
    case 0: {
      LOG(INFO) << "\e[1m Using Normal VIO. \e[0m";
      *vio_backend = std::make_shared<VioBackEnd>(B_Pose_camLrect,
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
      *vio_backend = std::make_shared<RegularVioBackEnd>(
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
                 << " but requested backend: " << FLAGS_backend_type;
      break;
    }
  }
  return true;
}

StatusSmartStereoMeasurements Pipeline::featureSelect(
    const VioFrontEndParams& tracker_params,
    const ETHDatasetParser& dataset,
    const Timestamp& timestamp_k,
    const gtsam::Pose3& W_Pose_Blkf,
    double* feature_selection_time,
    const StereoFrame& stereoFrame_km1,
    const StatusSmartStereoMeasurements& status_smart_stereo_meas,
    const double& cur_kf_id,
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
  if (dataset_.isGroundTruthAvailable()) {
    W_Pose_Bkf_gt= dataset_.getGroundTruthState(timestamp_lkf_).pose_;

    for (size_t kk = 0; kk < nrKfInHorizon + 1; kk++) {
      // Including current pose.
      Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(
            kk * tracker_params.intra_keyframe_time_);

      // Relative pose wrt ground truth at last kf.
      Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(
            dataset_.getGroundTruthState(timestamp_kk).pose_);
      posesAtFutureKeyframes.push_back(
            StampedPose(W_Pose_Blkf.compose(poseGT_km1_kk),
                        UtilsOpenCV::NsecToSec(timestamp_kk)) );
    }
  }

  VLOG(100) << "Starting feature selection...";
  SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
  std::tie(trackedAndSelectedSmartStereoMeasurements, *feature_selection_time)
      = feature_selector_.splitTrackedAndNewFeatures_Select_Display(
        stereo_vision_frontend_->stereoFrame_km1_,
        status_smart_stereo_meas.second,
        cur_kf_id, save_image_selector,
        tracker_params.featureSelectionCriterion_,
        tracker_params.featureSelectionNrCornersToSelect_,
        tracker_params.maxFeatureAge_,
        posesAtFutureKeyframes, // TODO Luca: can we make this optional, for the case where we do not have ground truth?
        vio_backend_->getCurrentStateCovariance(),
        dataset_.getDatasetName(),
        stereo_vision_frontend_->stereoFrame_lkf_->left_frame_); // last 2 are for visualization
  VLOG(100) << "Feature selection completed.";

  // Same status as before.
  TrackerStatusSummary status = status_smart_stereo_meas.first;
  return std::make_pair(status, trackedAndSelectedSmartStereoMeasurements);
}

void Pipeline::launchThreads() {
  // Start backend_thread.
  backend_thread_ = std::thread(&VioBackEnd::spin,
                                std::ref(*vio_backend_),
                                std::ref(backend_input_queue_),
                                std::ref(backend_output_queue_));

  // Start mesher_thread.
  mesher_thread_ = std::thread(&Mesher::run, std::ref(mesher_),
                               std::ref(mesher_input_queue_),
                               std::ref(mesher_output_queue_));

  // Start visualizer_thread.
  visualizer_thread_ = std::thread(&Visualizer3D::spin, std::ref(visualizer_),
                                   std::ref(visualizer_input_queue_),
                                   std::ref(visualizer_output_queue_));
}

void Pipeline::stopThreads() {
  // Shutdown workers and queues.
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

void Pipeline::joinThreads() {
  backend_thread_.join();
  mesher_thread_.join();
  visualizer_thread_.join();
}

} // End of VIO namespace
