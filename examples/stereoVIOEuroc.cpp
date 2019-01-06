/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   stereoVIOEuroc.cpp
 * @brief  example of VIO pipeline running on the Euroc dataset
 * @author Luca Carlone
 */

//#define USE_CGAL

#include <memory>
#include <thread>
#include <future>
#include <chrono>

#include "RegularVioBackEnd.h"
#include "VioBackEnd.h"
#include "RegularVioBackEndParams.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include "factors/PointPlaneFactor.h"

#include "ETH_parser.h"
#include "mesh/Mesher.h"
#include "utils/ThreadsafeQueue.h"
#include "StereoVisionFrontEnd.h"
#include "FeatureSelector.h"
#include "LoggerMatlab.h"
#include "Visualizer3D.h"

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


DEFINE_bool(deterministic_random_number_generator, false,
            "If true the random number generator will consistently output the "
            "same sequence of pseudo-random numbers for every run (use it to "
            "have repeatable output). If false the random number generator "
            "will output a different sequence for each run.");
DEFINE_int32(min_num_obs_for_mesher_points, 4,
             "Minimum number of observations for a smart factor's landmark to "
             "to be used as a 3d point to consider for the mesher");

using namespace std;
using namespace gtsam;
using namespace VIO;

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {
  auto start = std::chrono::high_resolution_clock::now();

  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  if (FLAGS_deterministic_random_number_generator) {
    // Initialize random seed for repeatability (only on the same machine).
    // Still does not make RANSAC REPEATABLE across different machines.
    srand(0);
  }

  // Init dataset parser.
  ETHDatasetParser dataset;
  size_t initial_k, final_k; // initial and final frame: useful to skip a bunch of images at the beginning (imu calibration)
  // Parse dataset.
  dataset.parse(&initial_k, &final_k);

  // Init Vio parameters (should be done inside VIO).
  VioBackEndParamsPtr vioParams;
  switch(FLAGS_backend_type) {
    case 0: {
      vioParams = std::make_shared<VioBackEndParams>();
      break;
    }
    case 1: {
      vioParams = std::make_shared<RegularVioBackEndParams>();
      break;
    }
    default: {
      CHECK(false) << "Unrecognized backend type: " << FLAGS_backend_type << "."
                   << " 0: normalVio, 1: RegularVio.";
      break;
    }
  }
  VioFrontEndParams trackerParams;
  // Parse parameters.
  dataset.parseParams(vioParams, dataset.imuData_, &trackerParams);

  // instantiate stereo tracker (class that tracks implements estimation
  // front-end) and print parameters
  static constexpr int saveImages = 0; // 0: don't show, 1: show, 2: write & save
  StereoVisionFrontEnd stereoVisionFrontEnd(
              trackerParams, *vioParams, // vioParams used by feature selection
              saveImages, dataset.getDatasetName());

  // instantiate feature selector: not used in vanilla implementation
  FeatureSelector featureSelector(trackerParams, *vioParams);

  // Create VIO: class that implements estimation back-end.
  std::shared_ptr<VioBackEnd> vioBackEnd;

  // Create class to visualize mesh:
  Mesher mesher;

  // Thread-safe queue for the mesher.
  ThreadsafeQueue<MesherInputPayload> mesher_input_queue;
  ThreadsafeQueue<MesherOutputPayload> mesher_output_queue;

  // Start mesher_thread.
  std::thread mesher_thread (&Mesher::run, std::ref(mesher),
                             std::ref(mesher_input_queue),
                             std::ref(mesher_output_queue));

  // Visualization process.
  Visualizer3D visualizer;

  // Thread-safe queue for the visualizer.
  ThreadsafeQueue<VisualizerInputPayload> visualizer_input_queue;
  ThreadsafeQueue<VisualizerOutputPayload> visualizer_output_queue;

  // Start visualizer_thread.
  std::thread visualizer_thread (&Visualizer3D::run, std::ref(visualizer),
                                 std::ref(visualizer_input_queue),
                                 std::ref(visualizer_output_queue));

  // structures to be filled with imu data
  ImuStamps imu_stamps;
  ImuAccGyr imu_accgyr;

  // instantiate Logger class (stores data for matlab visualization)
  LoggerMatlab logger;
  if (FLAGS_log_output) {
    logger.openLogFiles();
  }

  // timestamp 10 frames before the first (for imu calibration)
  // TODO: remove hardcoded 10
  static constexpr size_t frame_offset_for_imu_calib = 10;
  CHECK_GE(initial_k, frame_offset_for_imu_calib)
          << "Initial frame " << initial_k << " has to be larger than "
          << frame_offset_for_imu_calib << " (needed for IMU calibration)";
  Timestamp timestamp_lkf = dataset.timestampAtFrame(
              initial_k - frame_offset_for_imu_calib);
  Timestamp timestamp_k;

  double startTime; // to log timing results

  /// Set of planes in the scene.
  std::vector<Plane> planes;

  static bool is_pipeline_successful = false;
  // start actual processing of the dataset
  for(size_t k = initial_k; k < final_k; k++) { // for each image

    LOG(INFO) << "------------------- Processing frame k = " << k
              << "--------------------";
    std::string leftImageName = dataset.camera_image_lists["cam0"]
                                                           .img_lists[k].second;
    std::string rightImageName = dataset.camera_image_lists["cam1"]
                                                           .img_lists[k].second;
    timestamp_k = dataset.timestampAtFrame(k); // same in both images

    // load stereo images
    startTime = UtilsOpenCV::GetTimeInSeconds();
    StereoFrame stereoFrame_k(
          k, timestamp_k,
          leftImageName, rightImageName,
          dataset.camera_info["cam0"], dataset.camera_info["cam1"],
          dataset.camL_Pose_calR,
          trackerParams.getStereoMatchingParams());

    if (FLAGS_log_output) {
      logger.timing_loadStereoFrame_ =
          UtilsOpenCV::GetTimeInSeconds() - startTime;
    }

    ////////////////////////////////////////////////////////////////////////////
    // For k == 1 (initial frame).
    if (k == initial_k) {
      // Process frame.
      stereoVisionFrontEnd.processFirstStereoFrame(stereoFrame_k);

      // Get IMU data.
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_
                      .getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

      // Create VIO.
      switch(FLAGS_backend_type) {
        case 0: {
          LOG(INFO) << "\e[1m Using Normal VIO. \e[0m";
          vioBackEnd = std::make_shared<VioBackEnd>(
           stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
           stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
           stereoVisionFrontEnd.stereoFrame_km1_->baseline_, *vioParams,
                         FLAGS_log_output);

          break;
        }
        case 1: {
          LOG(INFO) << "\e[1m Using Regular VIO with modality "
                    << FLAGS_regular_vio_backend_modality << "\e[0m";
          vioBackEnd = std::make_shared<RegularVioBackEnd>(
            stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
            stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
            stereoVisionFrontEnd.stereoFrame_km1_->baseline_,
            *vioParams, FLAGS_log_output,
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

      // Initialize VIO.
      gtNavState initialStateGT;

      // Use initial IMU measurements to guess first pose
      if (vioParams->autoInitialize_ || !dataset.isGroundTruthAvailable()) {
        LOG_IF(WARNING, !vioParams->autoInitialize_)
            << "Could not initialize from ground truth, since it is not "
               "available";
        initialStateGT.pose = vioBackEnd->guessPoseFromIMUmeasurements(
                                imu_accgyr, vioParams->n_gravity_,
                                vioParams->roundOnAutoInitialize_);
        vioBackEnd->initializeStateAndSetPriors(timestamp_k,
                                                initialStateGT.pose,
                                                imu_accgyr);
        // Only for display later on
        initialStateGT.velocity = vioBackEnd->W_Vel_Blkf_;
        // Only for display
        initialStateGT.imuBias = vioBackEnd->imu_bias_lkf_;
      } else {
        initialStateGT = dataset.getGroundTruthState(timestamp_k);
        vioBackEnd->initializeStateAndSetPriors(timestamp_k,
                                                initialStateGT.pose,
                                                initialStateGT.velocity,
                                                initialStateGT.imuBias);
      }
      ////////////////// DEBUG INITIALIZATION //////////////////////////////////
      if (FLAGS_log_output) {
        logger.displayInitialStateVioInfo(dataset, vioBackEnd, initialStateGT,
                                          imu_accgyr, timestamp_k);
        // Store latest pose estimate
        logger.W_Pose_Bprevkf_vio_ = vioBackEnd->W_Pose_Blkf_;
      }

      // Store latest keyframe timestamp
      timestamp_lkf = timestamp_k;
      continue;
    }

    ////////////////////////////////////////////////////////////////////////////
    // For k > 1
    // For visualization purposes.
    Frame frame_km1_debug = Frame(stereoVisionFrontEnd.
                                  stereoFrame_lkf_->left_frame_);

    // Integrate rotation measurements (rotation is used in RANSAC).
    std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_.
                                     getBetweenValuesInterpolated(timestamp_lkf,
                                                                  timestamp_k);
    gtsam::Rot3 calLrectLkf_R_camLrectKf_imu = vioBackEnd->
                                       preintegrateGyroMeasurements(imu_stamps,
                                                                    imu_accgyr);

    ////////////////////////////// FRONT-END ///////////////////////////////////
    // Main function for tracking.
    startTime = UtilsOpenCV::GetTimeInSeconds();

    // Rotation used in 1 and 2 point ransac.
    StatusSmartStereoMeasurements statusSmartStereoMeasurements =
        stereoVisionFrontEnd.processStereoFrame(stereoFrame_k,
                                                calLrectLkf_R_camLrectKf_imu);

    if (FLAGS_log_output) {
      logger.timing_processStereoFrame_ =
          UtilsOpenCV::GetTimeInSeconds() - startTime;
    }

    // Pass info to VIO if it's keyframe.
    startTime = UtilsOpenCV::GetTimeInSeconds();
    if (stereoVisionFrontEnd.stereoFrame_km1_->isKeyframe_) {
      // It's a keyframe!
      std::cout << "Keyframe " << k << " with: "
                << statusSmartStereoMeasurements.second.size()
                << " smart measurements" << std::endl;

      ////////////////////////////// FEATURE SELECTOR //////////////////////////
      // ------------ DATA ABOUT CURRENT AND FUTURE ROBOT STATE ------------- //
      KeyframeToStampedPose posesAtFutureKeyframes;
      // Consider using static here.
      size_t nrKfInHorizon =
          round(stereoVisionFrontEnd.tracker_.trackerParams_
                .featureSelectionHorizon_ /
                stereoVisionFrontEnd.tracker_.trackerParams_
                .intra_keyframe_time_);
      VLOG(100) << "nrKfInHorizon for selector: " << nrKfInHorizon;

      // Future poses are gt and might be far from the vio pose: we have to
      // attach the *relative* poses from the gt to the latest vio estimate.
      // W_Pose_Bkf_gt    : ground truth pose at previous keyframe.
      // vio->W_Pose_Blkf_: vio pose at previous keyframe.
      // More important than the time, it is important that
      // it is the same time as vio->W_Pose_Blkf_
      Pose3 W_Pose_Bkf_gt;
      if (dataset.isGroundTruthAvailable()) {
         W_Pose_Bkf_gt= dataset.getGroundTruthState(timestamp_lkf).pose;

        for (size_t kk = 0; kk < nrKfInHorizon + 1; kk++) {
          // Including current pose.
          Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(
                                     kk * stereoVisionFrontEnd.tracker_.
                                     trackerParams_.intra_keyframe_time_);

          // Relative pose wrt ground truth at last kf.
          Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(dataset.getGroundTruthState(
                                                        timestamp_kk).pose);
          posesAtFutureKeyframes.push_back(
                StampedPose(vioBackEnd->W_Pose_Blkf_.compose(poseGT_km1_kk),
                            UtilsOpenCV::NsecToSec(timestamp_kk)) );
        }
      }

      startTime = UtilsOpenCV::GetTimeInSeconds();
      SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;

      VLOG(100) << "Starting feature selection...";
      // ToDo init to invalid value.
      gtsam::Matrix curr_state_cov;
      if (trackerParams.featureSelectionCriterion_ !=
          VioFrontEndParams::FeatureSelectionCriterion::QUALITY) {
        VLOG(100) << "Using feature selection criterion diff than QUALITY ";
        try {
          curr_state_cov = vioBackEnd->getCurrentStateCovariance();
        } catch(const gtsam::IndeterminantLinearSystemException& e) {
          LOG(ERROR) << "Error when calculating current state covariance.";
        }
      } else {
        VLOG(100) << "Using QUALITY as feature selection criterion";
      }

      static constexpr int saveImagesSelector = 1; // 0: don't show, 2: write & save
      std::tie(trackedAndSelectedSmartStereoMeasurements,
               stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_) =
          featureSelector.splitTrackedAndNewFeatures_Select_Display(
            stereoVisionFrontEnd.stereoFrame_km1_,
            statusSmartStereoMeasurements.second,
            vioBackEnd->cur_kf_id_, (FLAGS_visualize?saveImagesSelector:0),
            trackerParams.featureSelectionCriterion_,
            trackerParams.featureSelectionNrCornersToSelect_,
            trackerParams.maxFeatureAge_,
            posesAtFutureKeyframes, // TODO Luca: can we make this optional, for the case where we do not have ground truth?
            curr_state_cov,
            dataset.getDatasetName(),
            frame_km1_debug); // last 2 are for visualization
      VLOG(100) << "Feature selection completed.";

      if (FLAGS_log_output) {
        logger.timing_featureSelection_ =
            UtilsOpenCV::GetTimeInSeconds() - startTime;
      }

      TrackerStatusSummary status = statusSmartStereoMeasurements.first;
      statusSmartStereoMeasurements = std::make_pair(
                                     status, // same status as before
                                     trackedAndSelectedSmartStereoMeasurements);

      VLOG_IF(100, FLAGS_log_output)
          << "Overall selection time (logger.timing_featureSelection_) "
          << logger.timing_featureSelection_ << '\n'
          << "actual selection time (stereoTracker.tracker_.debugInfo_."
          << "featureSelectionTime_) "
          << stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_;

      ////////////////// DEBUG INFO FOR FRONT-END //////////////////////////////
      startTime = UtilsOpenCV::GetTimeInSeconds();

      if (FLAGS_log_output) {
        logger.logFrontendResults(dataset, stereoVisionFrontEnd, timestamp_lkf,
                                  timestamp_k);
        logger.timing_loggerFrontend_ =
            UtilsOpenCV::GetTimeInSeconds() - startTime;
      }

      //////////////////// BACK-END ////////////////////////////////////////////
      // Get IMU data.
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_
                                    .getBetweenValuesInterpolated(timestamp_lkf,
                                                                  timestamp_k);

      // Process data with VIO.
      startTime = UtilsOpenCV::GetTimeInSeconds();

      if (vioParams->addBetweenStereoFactors_ == true &&
          stereoVisionFrontEnd.trackerStatusSummary_.kfTrackingStatus_stereo_ ==
                                                              Tracker::VALID ) {
        VLOG(10) << "Add visual inertial state and optimize,"
                    " using stereo between factor.";
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k, // Current time for fixed lag smoother.
              statusSmartStereoMeasurements, // Vision data.
              imu_stamps, imu_accgyr, // Inertial data.
              &planes,
              stereoVisionFrontEnd.getRelativePoseBodyStereo()); // optional: pose estimate from stereo ransac
        VLOG(10) << "Finished addVisualInertialStateAndOptimize.";
      } else {
        VLOG(10) << "Add visual inertial state and optimize,"
                    " without using stereo between factor.";
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k,
              statusSmartStereoMeasurements,
              imu_stamps, imu_accgyr,
              &planes); // Same but no pose.
        VLOG(10) << "Finished addVisualInertialStateAndOptimize.";
      }

      if (FLAGS_log_output) {
        logger.timing_vio_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

        ////////////////// DEBUG INFO FOR BACK-END /////////////////////////////
        startTime = UtilsOpenCV::GetTimeInSeconds();
        logger.logBackendResults(dataset, stereoVisionFrontEnd, vioBackEnd,
                                 timestamp_lkf, timestamp_k,k);
        logger.W_Pose_Bprevkf_vio_ = vioBackEnd->W_Pose_Blkf_;
        logger.timing_loggerBackend_ =
            UtilsOpenCV::GetTimeInSeconds() - startTime;
        logger.displayOverallTiming();
      }
      //////////////////////////////////////////////////////////////////////////

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
        stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.createMesh2D(
              &mesh_2d);
        break;
      }
      case VisualizationType::MESH2Dsparse: { // visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
        stereoVisionFrontEnd.stereoFrame_lkf_->createMesh2dStereo(&mesh_2d);
        break;
      }
      case VisualizationType::MESH2DTo3Dsparse: {
        // Points_with_id_VIO contains all the points in the optimization,
        // (encoded as either smart factors or explicit values), potentially
        // restricting to points seen in at least min_num_obs_fro_mesher_points keyframes
        // (TODO restriction is not enforced for projection factors).
        vioBackEnd->getMapLmkIdsTo3dPointsInTimeHorizon(
              &points_with_id_VIO,
              FLAGS_visualize_lmk_type?
                &lmk_id_to_lmk_type_map:nullptr,
              FLAGS_min_num_obs_for_mesher_points);

        // Get camera pose.
        gtsam::Pose3 W_Pose_camlkf_vio =
            vioBackEnd->W_Pose_Blkf_.compose(vioBackEnd->B_Pose_leftCam_);

        // Create and fill data packet for mesher.

        // Push to queue.
        // In another thread, mesher is running, consuming mesher payloads.
        CHECK(mesher_input_queue.push(MesherInputPayload (
                                        points_with_id_VIO, //copy, thread safe, read-only.
                                        *(stereoVisionFrontEnd.stereoFrame_lkf_), // not really thread safe, read only.
                                        W_Pose_camlkf_vio))); // copy, thread safe, read-only.

        // Find regularities in the mesh if we are using RegularVIO backend.
        // TODO create a new class that is mesh segmenter or plane extractor.
        if (FLAGS_backend_type == 1) {
          mesher.clusterPlanesFromMesh(&planes,
                                       points_with_id_VIO);
        }

        // In the mesher thread push queue with meshes for visualization.
        if (!mesher_output_queue.popBlocking(mesher_output_payload)) { //Use blocking to avoid skipping frames.
          LOG(WARNING) << "Mesher output queue did not pop a payload.";
        }
        break;
      }
      case VisualizationType::POINTCLOUD_REPEATEDPOINTS: {// visualize VIO points as point clouds (points are replotted at every frame)
        points_3d = vioBackEnd->get3DPoints();
        break;
      }
        // Computes and visualizes a 3D point cloud.
      case VisualizationType::POINTCLOUD: {// visualize VIO points  (no repeated point)
        vioBackEnd->getMapLmkIdsTo3dPointsInTimeHorizon(&points_with_id_VIO);
        break;
      }
      case VisualizationType::NONE: {break;}
      }

      if (FLAGS_visualize) {
        // Decide corresponding visualization type:
        visualizer_input_queue.push(VisualizerInputPayload(
            visualization_type, FLAGS_backend_type,
            vioBackEnd->W_Pose_Blkf_ * stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect, // pose for trajectory viz.
            mesh_2d, // for visualizeMesh2D and visualizeMesh2DStereo
            stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_, // for visualizeMesh2D and visualizeMesh2DStereo
            mesher_output_payload, // visualizeConvexHull & visualizeMesh3DWithColoredClusters
            points_with_id_VIO, // visualizeMesh3DWithColoredClusters & visualizePoints3D
            lmk_id_to_lmk_type_map, // visualizeMesh3DWithColoredClusters & visualizePoints3D
            planes,  // visualizeMesh3DWithColoredClusters
            vioBackEnd->smoother_->getFactors(), // For plane constraints viz.
            vioBackEnd->state_, // For planes and plane constraints viz.
            points_3d, timestamp_k));
        std::shared_ptr<VisualizerOutputPayload> visualizer_output_payload =
            visualizer_output_queue.popBlocking();
        for (const ImageToDisplay& img_to_display:
             visualizer_output_payload->images_to_display_) {
          cv::imshow(img_to_display.name_, img_to_display.image_);
        }
        if (visualization_type != VisualizationType::NONE) {
          visualizer_output_payload->window_.spinOnce(1, true);
        }
        cv::waitKey(1);
      }

      timestamp_lkf = timestamp_k;
    }

    if (k == final_k - 1) {
      LOG(INFO) << "stereoVIOExample completed successfully!";
      is_pipeline_successful = true;
    }
  }

  // Shutdown workers and queues.
  mesher_input_queue.shutdown();
  mesher_output_queue.shutdown();
  mesher.shutdown();
  mesher_thread.join();

  visualizer_input_queue.shutdown();
  visualizer_output_queue.shutdown();
  visualizer.shutdown();
  visualizer_thread.join();


  if (FLAGS_log_output) {
    logger.closeLogFiles();
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration_ms =
          std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
  LOG(WARNING) << "main() took" << duration_ms.count() << " milliseconds.";

  return is_pipeline_successful? EXIT_SUCCESS : EXIT_FAILURE;
}
