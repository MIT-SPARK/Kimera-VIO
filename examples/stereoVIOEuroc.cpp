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

#include "RegularVioBackEnd.h"
#include "VioBackEnd.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/OrientedPlane3Factor.h>
#include "factors/PointPlaneFactor.h"

#include "ETH_parser.h"
#include "mesh/Mesher.h"
#include "StereoVisionFrontEnd.h"
#include "FeatureSelector.h"
#include "LoggerMatlab.h"
#include "Visualizer3D.h"

DEFINE_string(dataset_path, "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_string(vio_params_path, "",
              "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path, "",
              "Path to tracker user-defined parameters.");
DEFINE_bool(log_output, false, "Log output to matlab.");
DEFINE_int32(backend_type, 0, "Type of vioBackEnd to use:\n"
                                 "0: VioBackEnd\n"
                                 "1: RegularVioBackEnd");
DEFINE_bool(visualize, true, "Enable visualization.");
DEFINE_int32(viz_type, 3,
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
DEFINE_int32(initial_k, 50, "Initial frame to start processing dataset, "
                            "previous frames will not be used.");
DEFINE_int32(final_k, 2812, "Final frame to finish processing dataset, "
                            "subsequent frames will not be used.");

using namespace std;
using namespace gtsam;
using namespace VIO;

// Helper function to parse dataset and user-specified parameters.
void parseDatasetAndParams(ETHDatasetParser* dataset,
                           VioBackEndParams* vioParams,
                           VioFrontEndParams* trackerParams,
                           size_t* initial_k, size_t* final_k) {
  CHECK_NOTNULL(dataset);
  CHECK_NOTNULL(vioParams);
  CHECK_NOTNULL(trackerParams);
  CHECK_NOTNULL(initial_k);
  CHECK_NOTNULL(final_k);

  // Dataset path.
  VLOG(100) << "Using dataset path: " << FLAGS_dataset_path;

  // Parse the dataset (ETH format).
  static const std::string leftCameraName = "cam0";
  static const std::string rightCameraName = "cam1";
  static const std::string imuName = "imu0";
  static const std::string gtSensorName = "state_groundtruth_estimate0";

  dataset->parseDataset(FLAGS_dataset_path, leftCameraName, rightCameraName,
                       imuName, gtSensorName);
  dataset->print();

  // Read/define vio params.
  if (FLAGS_vio_params_path.empty()) {
    VLOG(100) << "No vio parameters specified, using default.";
    // Default params with IMU stats from dataset.
    *vioParams = VioBackEndParams(dataset->imuData_.gyro_noise_,
                                  dataset->imuData_.acc_noise_,
                                  dataset->imuData_.gyro_walk_,
                                  dataset->imuData_.acc_walk_);
  } else {
    VLOG(100) << "Using user-specified VIO parameters: "
              << FLAGS_vio_params_path;
    vioParams->parseYAML(FLAGS_vio_params_path);
  }

  // Read/define tracker params.
  if (FLAGS_tracker_params_path.empty()) {
    VLOG(100) << "No tracker parameters specified, using default";
    *trackerParams = VioFrontEndParams(); // default params
  } else {
    VLOG(100) << "Using user-specified tracker parameters: "
              << FLAGS_tracker_params_path;
    trackerParams->parseYAML(FLAGS_tracker_params_path);
  }

  // Start processing dataset from frame initial_k.
  // Useful to skip a bunch of images at the beginning (imu calibration).
  *initial_k = FLAGS_initial_k;
  CHECK_GE(*initial_k, 0);
  CHECK_GE(*initial_k, 10)
      << "initial_k should be >= 10 for IMU bias initialization";

  // Finish processing dataset at frame final_k.
  // Last frame to process (to avoid processing the entire dataset),
  // skip last frames.
  *final_k = FLAGS_final_k;
  CHECK_GT(*final_k, 0);
  const size_t& nr_images = dataset->nrImages();
  CHECK(*final_k < nr_images)
      << "Value for final_k, " << *final_k << " is larger than total"
      << " number of frames in dataset " << nr_images;
  CHECK(*final_k > *initial_k)
      << "Value for final_k (" << *final_k << ") is smaller than value for"
      << " initial_k (" << *initial_k << ").";

  LOG(INFO) << "Running dataset between frame " << *initial_k
            << " and frame " <<  *final_k;
}

////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample
////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[]) {

  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // initialize random seed for repeatability (only on the same machine)
  srand(0); // still does not make RANSAC REPEATABLE across different machines
  static constexpr int saveImages = 0; // 0: don't show, 1: show, 2: write & save
  static constexpr int saveImagesSelector = 1;          // 0: don't show, >0 write & save
  VisualizationType visualization_type = static_cast<VisualizationType>(
        FLAGS_viz_type);

  ETHDatasetParser dataset;
  VioBackEndParams vioParams;
  VioFrontEndParams trackerParams;
  size_t initial_k, final_k; // initial and final frame: useful to skip a bunch of images at the beginning (imu calibration)
  parseDatasetAndParams(&dataset, &vioParams, &trackerParams,
                        &initial_k, &final_k);

  // instantiate stereo tracker (class that tracks implements estimation front-end) and print parameters
  StereoVisionFrontEnd stereoVisionFrontEnd(trackerParams, vioParams, saveImages); // vioParams used by feature selection
  stereoVisionFrontEnd.tracker_.trackerParams_.print();
  if (saveImages > 0) {
    stereoVisionFrontEnd.outputImagesPath_ = "./outputStereoTrackerImages-" + dataset.dataset_name_;
    stereoVisionFrontEnd.tracker_.outputImagesPath_ = "./outputTrackerImages-" + dataset.dataset_name_;
  }

  // instantiate feature selector: not used in vanilla implementation
  FeatureSelector featureSelector(trackerParams, vioParams);

  // Create VIO: class that tracks implements estimation back-end
  boost::shared_ptr<VioBackEnd> vioBackEnd;

  // create class to visualize 3D points and mesh:
  Mesher mesher;
  Visualizer3D visualizer;

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
  Timestamp timestamp_lkf = dataset.timestampAtFrame(initial_k - 10);
  Timestamp timestamp_k;

  double startTime; // to log timing results

  /// Lmk ids that are considered to be in the same cluster.
  LandmarkIds mesh_lmk_ids_ground_cluster;

  // start actual processing of the dataset
  for(size_t k = initial_k; k < final_k; k++) { // for each image

    std::cout << "------------------- Processing frame k="<< k
              << "--------------------" << std::endl;
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
          vioBackEnd = boost::make_shared<VioBackEnd>(
           stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
           stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
           stereoVisionFrontEnd.stereoFrame_km1_->baseline_, vioParams);

          break;
        }
        case 1: {
          LOG(INFO) << "\e[1m Using Regular VIO. \e[0m";
          vioBackEnd = boost::make_shared<RegularVioBackEnd>(
           stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
           stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
           stereoVisionFrontEnd.stereoFrame_km1_->baseline_, vioParams);
          break;
        }
      }

      // Initialize VIO.
      gtNavState initialStateGT;

      // Use initial IMU measurements to guess first pose
      if (vioParams.autoInitialize_) {
          initialStateGT.pose = vioBackEnd->guessPoseFromIMUmeasurements(
                                              imu_accgyr, vioParams.n_gravity_,
                                              vioParams.roundOnAutoInitialize_);
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
      Pose3 W_Pose_Bkf_gt = dataset.getGroundTruthState(timestamp_lkf).pose;

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

      std::tie(trackedAndSelectedSmartStereoMeasurements,
               stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_) =
          featureSelector.splitTrackedAndNewFeatures_Select_Display(
            stereoVisionFrontEnd.stereoFrame_km1_,
            statusSmartStereoMeasurements.second,
            vioBackEnd->cur_kf_id_, saveImagesSelector,
            trackerParams.featureSelectionCriterion_,
            trackerParams.featureSelectionNrCornersToSelect_,
            trackerParams.maxFeatureAge_, posesAtFutureKeyframes,
            curr_state_cov,
            dataset.dataset_name_,
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

      if (vioParams.addBetweenStereoFactors_ == true &&
          stereoVisionFrontEnd.trackerStatusSummary_.kfTrackingStatus_stereo_ ==
                                                              Tracker::VALID ) {
        VLOG(10) << "Add visual inertial state and optimize,"
                    " using stereo between factor.";
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k, // Current time for fixed lag smoother.
              statusSmartStereoMeasurements, // Vision data.
              imu_stamps, imu_accgyr, // Inertial data.
              mesh_lmk_ids_ground_cluster,
              stereoVisionFrontEnd.getRelativePoseBodyStereo()); // optional: pose estimate from stereo ransac
        VLOG(10) << "Finished addVisualInertialStateAndOptimize.";
      } else {
        VLOG(10) << "Add visual inertial state and optimize,"
                    " without using stereo between factor.";
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k,
              statusSmartStereoMeasurements,
              imu_stamps, imu_accgyr,
              mesh_lmk_ids_ground_cluster); // Same but no pose.
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
      // Get camera pose.
      gtsam::Pose3 W_Pose_camlkf_vio =
          vioBackEnd->W_Pose_Blkf_.compose(vioBackEnd->B_Pose_leftCam_);

      switch (visualization_type) {
        // Computes and visualizes 2D mesh.
        // vertices: all leftframe kps with lmkId != -1 and inside the image
        // triangles: all the ones with edges inside images as produced by cv::subdiv
        case VisualizationType::MESH2D: {
          stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.createMesh2D();
          stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_
              .visualizeMesh2D(100);
          break;
        }

          // Computes and visualizes a 3D mesh from 2D triangulation.
          // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and inside the image
          // triangles: all the ones with edges inside images as produced by cv::subdiv
          // (updateMesh3D also filters out geometrically)
        case VisualizationType::MESH2DTo3D: {
          CHECK(false)
              << "This is the same as Mesh2Dto3Dsparse except the fact"
              << "that we do not restrict triangles to be on planar surfaces."
              << "Deprecated for simplicity, it can be executed by running mesh2dto3dsparse with "
              << "maxGradInTriangle being very large.";
          break;
        }

          // Computes and visualizes 2D mesh.
          // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and inside the image
          // triangles: all the ones with edges inside images as produced by cv::subdiv, which have uniform gradient
        case VisualizationType::MESH2Dsparse: {// visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
          std::vector<cv::Vec6f> mesh_2d;
          stereoVisionFrontEnd.stereoFrame_lkf_->createMesh2dStereo(&mesh_2d);
          stereoVisionFrontEnd.stereoFrame_lkf_->visualizeMesh2DStereo(mesh_2d,
                                                                       100);
          break;
        }

          // Computes and visualizes 3D mesh from 2D triangulation.
          // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and inside the image
          // triangles: all the ones with edges inside images as produced by cv::subdiv, which have uniform gradient
          // (updateMesh3D also filters out geometrically)
          // same as MESH2DTo3D but filters out triangles corresponding to non planar obstacles
        case VisualizationType::MESH2DTo3Dsparse: {
          VLOG(10) << "Mesh2Dtype::MESH2DTo3Dsparse";

          // only select points which have been tracked for minKfValidPoints keyframes
          // If this is 0 it breaks!!!!!!!!!!!!!!!!!!!!!!
          static constexpr int minKfValidPoints = 4;

          // Points_with_id_VIO contains all the points in the optimization,
          // (encoded as either smart factors or explicit values), potentially
          // restricting to points seen in at least minKfValidPoints keyframes
          // (TODO restriction is not enforced for projection factors).
          VioBackEnd::PointsWithIdMap points_with_id_VIO;
          VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map;
          static constexpr bool visualize_lmk_type = true;
          if (visualize_lmk_type) {
            vioBackEnd->getMapLmkIdsTo3dPointsInTimeHorizon(
                  &points_with_id_VIO,
                  &lmk_id_to_lmk_type_map,
                  minKfValidPoints);
          } else {
            vioBackEnd->getMapLmkIdsTo3dPointsInTimeHorizon(
                  &points_with_id_VIO,
                  nullptr,
                  minKfValidPoints);
          }

          static constexpr float maxGradInTriangle = -1; //50.0;
          static constexpr double minRatioBetweenLargestAnSmallestSide = 0.5; // TODO: this check should be improved
          static constexpr double min_elongation_ratio = 0.5;  // TODO: this check should be improved
          static constexpr double maxTriangleSide = 0.5;

          // Create mesh.
          mesher.updateMesh3D(
                points_with_id_VIO,
                stereoVisionFrontEnd.stereoFrame_lkf_,
                W_Pose_camlkf_vio,
                maxGradInTriangle,
                minRatioBetweenLargestAnSmallestSide,
                min_elongation_ratio, maxTriangleSide,
                FLAGS_visualize);

          // Find regularities in the mesh.
          // Currently only triangles in the ground floor.
          std::vector<TriangleCluster> triangle_clusters;
          gtsam::OrientedPlane3 plane;
          static const gtsam::Point3 plane_normal (0.0, 0.0, 1.0);
          static constexpr double plane_distance = -0.15;

          static constexpr bool use_expectation_maximization = true;
          if(use_expectation_maximization &&
             vioBackEnd->getEstimateOfKey<gtsam::OrientedPlane3>(
               gtsam::Symbol('P', 0).key(), &plane)) {
            // Use the plane estimate of the backend.
            // TODO this can lead to issues, when the plane estimate gets
            // quite crazy, but at the same time it will avoid crashing the
            // optimization, because otherwise we are providing huge outliers.
            static constexpr bool use_normal_estimation = false;
            mesher.clusterMesh(&triangle_clusters,
                               use_normal_estimation?plane.normal().point3():plane_normal,
                               plane.distance());
          } else {
            // Try to cluster the ground plane.
            mesher.clusterMesh(&triangle_clusters,
                               plane_normal,
                               plane_distance);
          }


          mesher.extractLmkIdsFromTriangleCluster(triangle_clusters.at(0),
                                                  &mesh_lmk_ids_ground_cluster);

          if (FLAGS_visualize) {
            VLOG(10) << "Starting mesh visualization...";
            cv::Mat vertices_mesh;
            cv::Mat polygons_mesh;
            mesher.getVerticesMesh(&vertices_mesh);
            mesher.getPolygonsMesh(&polygons_mesh);

            static VioBackEnd::PointsWithIdMap points_with_id_VIO_prev;
            static VioBackEnd::LmkIdToLmkTypeMap lmk_id_to_lmk_type_map_prev;
            static cv::Mat vertices_mesh_prev;
            static cv::Mat polygons_mesh_prev;
            static std::vector<TriangleCluster> triangle_clusters_prev;

            static constexpr bool visualize_mesh = true;
            if (visualize_mesh) {
              visualizer.visualizeMesh3DWithColoredClusters(triangle_clusters_prev,
                                                            vertices_mesh_prev,
                                                            polygons_mesh_prev);
            }

            static constexpr bool visualize_point_cloud = true;
            if (visualize_point_cloud) {
              visualizer.visualizePoints3D(points_with_id_VIO_prev,
                                           lmk_id_to_lmk_type_map_prev);
            }

            static constexpr bool visualize_planes = true;
            static constexpr bool visualize_plane_constraints = true;
            if (visualize_planes) {
              static const std::string plane_id = "Plane 0.";
              static bool is_plane_in_window = false;
              gtsam::OrientedPlane3 plane;
              if (vioBackEnd->getEstimateOfKey<gtsam::OrientedPlane3>(
                    gtsam::Symbol('P', 0).key(), &plane)) {
                const Point3& normal = plane.normal().point3();
                if (visualize_plane_constraints) {
                  const gtsam::NonlinearFactorGraph& graph =
                      vioBackEnd->smoother_->getFactors();

                  LandmarkIds lmk_ids_in_current_pp_factors;
                  for (const auto& g : graph) {
                    const auto& ppf =
                        boost::dynamic_pointer_cast<PointPlaneFactor>(g);
                    if (ppf) {
                      // We found a PointPlaneFactor.
                      // Get point key.
                      Key point_key = ppf->getPointKey();
                      LandmarkId lmk_id = gtsam::Symbol(point_key).index();
                      lmk_ids_in_current_pp_factors.push_back(lmk_id);
                      // Get point estimate.
                      gtsam::Point3 point;
                      CHECK(vioBackEnd->getEstimateOfKey(point_key, &point));

                      // Visualize.
                      Key plane_key = ppf->getPlaneKey();
                      CHECK(plane_key == gtsam::Symbol('P', 0).key());

                      visualizer.visualizePlaneConstraints(
                            normal, plane.distance(),
                            lmk_id, point);
                    }
                  }

                  // Remove lines that are not representing a point plane factor
                  // in the current graph.
                  visualizer.removeOldLines(lmk_ids_in_current_pp_factors);
                }

                // Visualize plane.
                visualizer.visualizePlane(plane_id,
                                          normal.x(),
                                          normal.y(),
                                          normal.z(),
                                          plane.distance());
                is_plane_in_window = true;
              } else {
                if (visualize_plane_constraints) {
                  visualizer.removePlaneConstraintsViz();
                }
                if (is_plane_in_window) {
                  visualizer.removeWidget(plane_id)?
                        is_plane_in_window = false : is_plane_in_window = true;
                }
              }
            }

            // Render current window.
            visualizer.renderWindow(1, true);

            // Store current mesh for display later.
            vertices_mesh_prev = vertices_mesh;
            polygons_mesh_prev = polygons_mesh;
            triangle_clusters_prev = triangle_clusters;
            points_with_id_VIO_prev = points_with_id_VIO;
            lmk_id_to_lmk_type_map_prev = lmk_id_to_lmk_type_map;
            VLOG(10) << "Finished mesh visualization.";
          } // FLAGS_visualize.

          break;
        }

          // Computes and visualizes 3D mesh.
          // vertices: all VALID VIO points (that can be triangulated)
          // triangles: the ones produced by CGAL
        case VisualizationType::MESH3D: {// 3D mesh from CGAL using VIO points
#ifdef USE_CGAL
          VioBackEnd::PointsWithId pointsWithId;
          vioBackEnd->get3DPointsAndLmkIds(&pointsWithId);
          mesher.updateMap3D(pointsWithId);
          visualizer.visualizeMesh3D(mesher.mapPoints3d_,
                                     Mesher_cgal::CreateMesh3D_MapPointId(
                                       mesher.mapPoints3d_));
          break;
#else
          throw std::runtime_error("VisualizationType::MESH3D requires flag USE_CGAL to be true");
          break;
#endif
        }

          // Computes and visualizes a 3D point cloud.
        case VisualizationType::POINTCLOUD_REPEATEDPOINTS: {// visualize VIO points as point clouds (points are replotted at every frame)
          vector<Point3> points3d = vioBackEnd->get3DPoints();
          visualizer.visualizeMap3D(points3d);
          visualizer.renderWindow();
          break;
        }

          // Computes and visualizes a 3D point cloud.
        case VisualizationType::POINTCLOUD: {// visualize VIO points  (no repeated point)
          VioBackEnd::PointsWithIdMap pointsWithId;
          vioBackEnd->getMapLmkIdsTo3dPointsInTimeHorizon(&pointsWithId);
          //mesher.updateMap3D(pointsWithId);
          //visualizer.visualizePoints3D(pointsWithId, mesher.map_points_3d_);
          break;
        }

        case VisualizationType::NONE: {
          break;
        }

        default: {
          throw std::runtime_error("stereoVIOEuroc: unknown visualizationType");
          break;
        }
      }

      // Visualize trajectory.
      if (FLAGS_visualize) {
        VLOG(10) << "Starting trajectory visualization...";
        visualizer.addPoseToTrajectory(vioBackEnd->W_Pose_Blkf_);
        visualizer.visualizeTrajectory3D(
              &(stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.img_));
        visualizer.renderWindow();
        VLOG(10) << "Finsihed trajectory visualization.";
      }

      timestamp_lkf = timestamp_k;
    }

    if (k == final_k - 1) {
      LOG(INFO) << "stereoVIOExample completed successfully!";
    }
  }

  if (FLAGS_log_output) {
    logger.closeLogFiles();
  }

  return 0;
}
