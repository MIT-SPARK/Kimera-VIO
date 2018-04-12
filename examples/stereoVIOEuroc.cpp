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
#define USE_REGULAR_VIO

#ifdef USE_REGULAR_VIO
  #include "RegularVioBackEnd.h"
#else
  #include "VioBackEnd.h"
#endif

#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ETH_parser.h"
#include "mesh/Mesher.h"
#include "StereoVisionFrontEnd.h"
#include "FeatureSelector.h"
#include "LoggerMatlab.h"
#include <gtsam/geometry/Pose3.h>
#include "../src/Visualizer3D.h"

DEFINE_string(dataset_path, "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_string(vio_params_path, "",
              "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path, "",
              "Path to tracker user-defined parameters.");
DEFINE_bool(log_output, false, "Log output to matlab.");
DEFINE_bool(viz, true, "Enable visualization.");
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

using namespace std;
using namespace gtsam;
using namespace VIO;

// helper function to parse dataset and user-specified parameters
void parseDatasetAndParams(const int argc, const char * const *argv,
    //output:
                         ETHDatasetParser& dataset, VioBackEndParams& vioParams,
                         VioFrontEndParams& trackerParams,
                         size_t& initial_k, size_t& final_k) {

  // dataset path
  VLOG(100) << "stereoVIOexample: using dataset path: " << FLAGS_dataset_path;

  // parse the dataset (ETH format)
  std::string leftCameraName = "cam0";
  std::string rightCameraName = "cam1";
  std::string imuName = "imu0";
  std::string gtSensorName = "state_groundtruth_estimate0";

  dataset.parseDataset(FLAGS_dataset_path, leftCameraName, rightCameraName,
                       imuName, gtSensorName);
  dataset.print();

  // read/define vio params
  if (FLAGS_vio_params_path.empty()) {
    VLOG(100) << "stereoVIOexample: no vio parameters specified, "
                 "using default.";
    // Default params with IMU stats from dataset
    vioParams = VioBackEndParams(dataset.imuData_.gyro_noise_,
                                 dataset.imuData_.acc_noise_,
                                 dataset.imuData_.gyro_walk_,
                                 dataset.imuData_.acc_walk_);
  } else {
    VLOG(100) << "stereoVIOexample: using user-specified vio parameters: "
              << FLAGS_vio_params_path;
    vioParams.parseYAML(FLAGS_vio_params_path);
  }

  // read/define tracker params
  if (FLAGS_tracker_params_path.empty()){
    VLOG(100) << "stereoVIOexample: no tracker parameters specified, "
                 "using default";
    trackerParams = VioFrontEndParams(); // default params
  }else{
    VLOG(100) << "stereoVIOexample: using user-specified tracker parameters: "
              << FLAGS_tracker_params_path;
    trackerParams.parseYAML(FLAGS_tracker_params_path);
  }

  // start processing dataset from frame initial_k
  initial_k = 50; // useful to skip a bunch of images at the beginning (imu calibration)
  if (argc >= 5){
    initial_k = size_t(stoi(argv[4]));
    std::cout << "stereoVIOexample: using user-specified initial_k: "
              << initial_k << std::endl;
  }

  // finish processing dataset at frame final_k
  final_k = dataset.nrImages() - 100; // last frame to process (to avoid processing the entire dataset), skip last frames
  if (argc >= 6){
    final_k = size_t(stoi(argv[5]));
    std::cout << "stereoVIOexample: using user-specified final_k "
              << "(may be saturated later): " << final_k << std::endl;
  }
  final_k = std::min(final_k,dataset.nrImages());
  std::cout << "Running dataset between frame " << initial_k << " and frame "
            <<  final_k << std::endl;

  if (initial_k < 10) {
    throw std::runtime_error(
      "stereoVIOExample: initial_k should be > 10 for IMU bias initialization");
  }
}

#ifdef USE_REGULAR_VIO
  typedef RegularVioBackEnd MyVioBackEnd;
#else
  typedef VioBackEnd MyVioBackEnd;
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  // Initialize Google's flags library.
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);

  // initialize random seed for repeatability (only on the same machine)
  // srand(0); // still does not make RANSAC REPEATABLE across different machines
  const int saveImages = 0;         // 0: don't show, 1: show, 2: write & save
  const int saveImagesSelector = 1; // 0: don't show, >0 write & save
  VisualizationType visualizationType = static_cast<VisualizationType>(
        FLAGS_viz_type); // MESH2Dobs MESH3D MESH2DTo3Dobs

  ETHDatasetParser dataset;
  VioBackEndParams vioParams;
  VioFrontEndParams trackerParams;
  size_t initial_k, final_k; // initial and final frame: useful to skip a bunch of images at the beginning (imu calibration)
  parseDatasetAndParams(argc, argv, dataset, vioParams, trackerParams,
                        initial_k, final_k);

  // instantiate stereo tracker (class that tracks implements estimation front-end) and print parameters
  StereoVisionFrontEnd stereoVisionFrontEnd(trackerParams, vioParams, saveImages); // vioParams used by feature selection
  stereoVisionFrontEnd.tracker_.trackerParams_.print();
  if(saveImages>0) {
    stereoVisionFrontEnd.outputImagesPath_ = "./outputStereoTrackerImages-" + dataset.dataset_name_;
    stereoVisionFrontEnd.tracker_.outputImagesPath_ = "./outputTrackerImages-" + dataset.dataset_name_;
  }

  // instantiate feature selector: not used in vanilla implementation
  FeatureSelector featureSelector(trackerParams, vioParams);

  // Create VIO: class that tracks implements estimation back-end
  boost::shared_ptr<MyVioBackEnd> vioBackEnd;

  // create class to visualize 3D points and mesh:
  Mesher mesher;
  Visualizer3D visualizer;

  // structures to be filled with imu data
  ImuStamps imu_stamps;
  ImuAccGyr imu_accgyr;

  // instantiate Logger class (stores data for matlab visualization)
  LoggerMatlab logger;
  if (FLAGS_log_output)
    logger.openLogFiles();

  // timestamp 10 frames before the first (for imu calibration)
  // TODO: remove hardcoded 10
  Timestamp timestamp_lkf = dataset.timestampAtFrame(initial_k - 10);
  Timestamp timestamp_k;

  bool didFirstOptimization = false;
  double startTime; // to log timing results

  /// Lmk ids that are considered to be in the same cluster.
  static LandmarkIds mesh_lmk_ids_ground_cluster;

  // start actual processing of the dataset
  for(size_t k = initial_k; k < final_k; k++)  // for each image
  {
    std::cout << "------------------- Processing frame k="<< k
              << "--------------------" << std::endl;
    std::string leftImageName = dataset.camera_image_lists["cam0"]
                                                           .img_lists[k].second;
    std::string rightImageName = dataset.camera_image_lists["cam1"]
                                                           .img_lists[k].second;
    timestamp_k = dataset.timestampAtFrame(k); // same in both images

    // load stereo images
    startTime = UtilsOpenCV::GetTimeInSeconds();
    StereoFrame stereoFrame_k(k, timestamp_k, leftImageName, rightImageName,
        dataset.camera_info["cam0"], dataset.camera_info["cam1"],
        dataset.camL_Pose_calR, trackerParams.getStereoMatchingParams());
    if (FLAGS_log_output) logger.timing_loadStereoFrame_ =
                                    UtilsOpenCV::GetTimeInSeconds() - startTime;

    ////////////////////////////////////////////////////////////////////////////
    // for k == 1 (initial frame)
    if (k == initial_k) {
      // process frame
      stereoVisionFrontEnd.processFirstStereoFrame(stereoFrame_k);

      // get IMU data
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_
                      .getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

      // create VIO
      vioBackEnd = boost::make_shared<MyVioBackEnd>(
            stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
            stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
            stereoVisionFrontEnd.stereoFrame_km1_->baseline_, vioParams);

      // initialize Vio
      gtNavState initialStateGT;
      // Use initial IMU measurements to guess first pose
      if (vioParams.autoInitialize_) {
          initialStateGT.pose = vioBackEnd->GuessPoseFromIMUmeasurements(
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
    // for k > 1
    // For visualization purposes
    Frame frame_km1_debug = Frame(stereoVisionFrontEnd.
                                  stereoFrame_lkf_->left_frame_);

    // integrate rotation measurements (rotation is used in RANSAC)
    std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_.
        getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);
    gtsam::Rot3 calLrectLkf_R_camLrectKf_imu = vioBackEnd->
        preintegrateGyroMeasurements(imu_stamps, imu_accgyr);

    ////////////////// FRONT-END ///////////////////////////////////////////////
    // main function for tracking
    startTime = UtilsOpenCV::GetTimeInSeconds();
    // Rotation used in 1 and 2 point ransac
    StatusSmartStereoMeasurements statusSmartStereoMeasurements =
        stereoVisionFrontEnd.processStereoFrame(stereoFrame_k,
                                                calLrectLkf_R_camLrectKf_imu);
    if (FLAGS_log_output) logger.timing_processStereoFrame_ =
                                    UtilsOpenCV::GetTimeInSeconds() - startTime;

    // pass info to vio if it's keyframe
    startTime = UtilsOpenCV::GetTimeInSeconds();
    if (stereoVisionFrontEnd.stereoFrame_km1_->isKeyframe_) {// it's a keyframe!
      std::cout << "Keyframe " << k << " with: "
                << statusSmartStereoMeasurements.second.size()
                << " smart measurements" << std::endl;

      //////////////////////////////////// FEATURE SELECTOR ////////////////////
      // ------------ DATA ABOUT CURRENT AND FUTURE ROBOT STATE ------------- //
      KeyframeToStampedPose posesAtFutureKeyframes;
      // Consider using static here
      size_t nrKfInHorizon =
          round(stereoVisionFrontEnd.tracker_.trackerParams_
                .featureSelectionHorizon_ /
                stereoVisionFrontEnd.tracker_.trackerParams_
                .intra_keyframe_time_);
      std::cout << "nrKfInHorizon for selector: " << nrKfInHorizon << std::endl;
      // Future poses are gt and might be far from the vio pose: we have to
      // attach the *relative* poses from the gt to the latest vio estimate
      // W_Pose_Bkf_gt    : ground truth pose at previous keyframe
      // vio->W_Pose_Blkf_: vio pose at previous keyframe
      Pose3 W_Pose_Bkf_gt = dataset.getGroundTruthState(timestamp_lkf).pose; // more important than the time, it is important that
      // it is the same time as vio->W_Pose_Blkf_

      for (size_t kk = 0; kk < nrKfInHorizon+1; kk++) {// including current pose
        Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(kk *
             stereoVisionFrontEnd.tracker_.trackerParams_.intra_keyframe_time_);
        // Relative pose wrt ground truth at last kf
        Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(dataset.getGroundTruthState(
                                                        timestamp_kk).pose);
        posesAtFutureKeyframes.push_back(
              StampedPose(vioBackEnd->W_Pose_Blkf_.compose(poseGT_km1_kk),
                          UtilsOpenCV::NsecToSec(timestamp_kk)) );
      }

      startTime = UtilsOpenCV::GetTimeInSeconds();
      SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
      std::tie(trackedAndSelectedSmartStereoMeasurements,
               stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_) =
          featureSelector.splitTrackedAndNewFeatures_Select_Display(
            stereoVisionFrontEnd.stereoFrame_km1_,
            statusSmartStereoMeasurements.second,
            vioBackEnd->cur_id_, saveImagesSelector,
            trackerParams.featureSelectionCriterion_,
            trackerParams.featureSelectionNrCornersToSelect_,
            trackerParams.maxFeatureAge_, posesAtFutureKeyframes,
            vioBackEnd->getCurrentStateCovariance(), dataset.dataset_name_,
            frame_km1_debug); // last 2 are for visualization

      if (FLAGS_log_output) logger.timing_featureSelection_ =
          UtilsOpenCV::GetTimeInSeconds() - startTime;

      TrackerStatusSummary status = statusSmartStereoMeasurements.first;
      statusSmartStereoMeasurements = std::make_pair(
            status, // same status as before
            trackedAndSelectedSmartStereoMeasurements);

      VLOG_IF(100, FLAGS_log_output)
          << "overall selection time (logger.timing_featureSelection_) "
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
      // get IMU data
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_
          .getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

      // process data with VIO
      startTime = UtilsOpenCV::GetTimeInSeconds();


      std::cout << " Size of mesh_lmk_ids_ground_cluster : " << mesh_lmk_ids_ground_cluster.size() << std::endl; // TODO remove
      if (vioParams.addBetweenStereoFactors_ == true &&
          stereoVisionFrontEnd.trackerStatusSummary_.kfTrackingStatus_stereo_ ==
                                                              Tracker::VALID ) {
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k, // current time for fixed lag smoother
              statusSmartStereoMeasurements, // vision data
              imu_stamps, imu_accgyr, // inertial data
              mesh_lmk_ids_ground_cluster,
              stereoVisionFrontEnd.getRelativePoseBodyStereo()); // optional: pose estimate from stereo ransac
      } else {
        vioBackEnd->addVisualInertialStateAndOptimize(
              timestamp_k,
              statusSmartStereoMeasurements,
              imu_stamps, imu_accgyr,
              mesh_lmk_ids_ground_cluster); // same but no pose
      }

      if (FLAGS_log_output) {
        logger.timing_vio_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

        ////////////////// DEBUG INFO FOR BACK-END /////////////////////////////
        startTime = UtilsOpenCV::GetTimeInSeconds();
        logger.logBackendResults(dataset, stereoVisionFrontEnd, vioBackEnd,
                                 timestamp_lkf,timestamp_k,k);
        logger.W_Pose_Bprevkf_vio_ = vioBackEnd->W_Pose_Blkf_;
        logger.timing_loggerBackend_ =
                                    UtilsOpenCV::GetTimeInSeconds() - startTime;
        logger.displayOverallTiming();
      }
      //////////////////////////////////////////////////////////////////////////

      ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////
      if (FLAGS_viz) {
        // DEBUG: this is the image from which the triangulation is computed
        //cv::Mat img = stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.img_.clone();
        //cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        //UtilsOpenCV::DrawCrossesInPlace(img, stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.getValidKeypoints(), cv::Scalar(0, 0, 255),0.4);
        //cv::imshow("Valid keypoints", img);
        //cv::waitKey(100);

        // get camera pose
        gtsam::Pose3 W_Pose_camlkf_vio =
                vioBackEnd->W_Pose_Blkf_.compose(vioBackEnd->B_Pose_leftCam_);

        switch (visualizationType) {
        // computes and visualizes 2D mesh
        // vertices: all leftframe kps with lmkId != -1 and inside the image
        // triangles: all the ones with edges inside images as produced by cv::subdiv
        case VisualizationType::MESH2D: {
          stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.createMesh2D();
          stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_
                                                          .visualizeMesh2D(100);
          break;
        }
        // computes and visualizes a 3D mesh from 2D triangulation
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
        // computes and visualizes 2D mesh
        // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and inside the image
        // triangles: all the ones with edges inside images as produced by cv::subdiv, which have uniform gradient
        case VisualizationType::MESH2Dsparse: {// visualize a 2D mesh of (right-valid) keypoints discarding triangles corresponding to non planar obstacles
          std::vector<cv::Vec6f> mesh_2d;
          stereoVisionFrontEnd.stereoFrame_lkf_->createMesh2dStereo(&mesh_2d);
          stereoVisionFrontEnd.stereoFrame_lkf_->visualizeMesh2DStereo(mesh_2d,
                                                                       100);
          break;
        }
        // computes and visualizes 3D mesh from 2D triangulation
        // vertices: all leftframe kps with right-VALID (3D), lmkId != -1 and inside the image
        // triangles: all the ones with edges inside images as produced by cv::subdiv, which have uniform gradient
        // (updateMesh3D also filters out geometrically)
        // same as MESH2DTo3D but filters out triangles corresponding to non planar obstacles
        case VisualizationType::MESH2DTo3Dsparse: {
          std::cout << "Mesh2Dtype::VALIDKEYPOINTS" << std::endl;

          static constexpr int  minKfValidPoints = 0; // only select points which have been tracked for minKfValidPoints keyframes

          // Points_with_id_VIO contains all the points in the optimization,
          // (encoded as either smart factors or explicit values), potentially
          // restricting to points seen in at least minKfValidPoints keyframes
          // (TODO restriction is not enforced for projection factors).
          MyVioBackEnd::PointsWithId points_with_id_VIO;
          vioBackEnd->get3DPointsAndLmkIds(&points_with_id_VIO,
                                           minKfValidPoints);

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
                min_elongation_ratio, maxTriangleSide);

          // Find regularities in the mesh.
          // Currently only triangles in the ground floor.
          std::vector<TriangleCluster> triangle_clusters;
          mesher.clusterMesh(&triangle_clusters);

          mesher.extractLmkIdsFromTriangleCluster(triangle_clusters.at(0),
                                                  &mesh_lmk_ids_ground_cluster);

          cv::Mat vertices_mesh;
          cv::Mat polygons_mesh;
          mesher.getVerticesMesh(&vertices_mesh);
          mesher.getPolygonsMesh(&polygons_mesh);
          visualizer.visualizeMesh3DWithColoredClusters(triangle_clusters,
                                                        vertices_mesh,
                                                        polygons_mesh);
          break;
        }

        // computes and visualizes 3D mesh
        // vertices: all VALID VIO points (that can be triangulated)
        // triangles: the ones produced by CGAL
        case VisualizationType::MESH3D: {// 3D mesh from CGAL using VIO points
#ifdef USE_CGAL
          MyVioBackEnd::PointsWithId pointsWithId;
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
        // computes and visualizes a 3D point cloud
        case VisualizationType::POINTCLOUD_REPEATEDPOINTS: {// visualize VIO points as point clouds (points are replotted at every frame)
          vector<Point3> points3d = vioBackEnd->get3DPoints();
          visualizer.visualizeMap3D(points3d);
          break;
        }
        // computes and visualizes a 3D point cloud
        case VisualizationType::POINTCLOUD: {// visualize VIO points  (no repeated point)
          MyVioBackEnd::PointsWithId pointsWithId;
          vioBackEnd->get3DPointsAndLmkIds(&pointsWithId);
          //mesher.updateMap3D(pointsWithId);
          //visualizer.visualizePoints3D(pointsWithId, mesher.map_points_3d_);
          break;
        }
        case VisualizationType::NONE: {
          break;
        }
        default:
          throw std::runtime_error("stereoVIOEuroc: unknown visualizationType");
          break;
        }
        // visualize trajectory
        visualizer.addPoseToTrajectory(vioBackEnd->W_Pose_Blkf_);
        visualizer.visualizeTrajectory3D(
                      &(stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.img_));
      }

      didFirstOptimization = true;
      timestamp_lkf = timestamp_k;
    }

    if (k == final_k-1)
      std::cout << "stereoVIOExample completed successfully!" << std::endl;
  }

  if (FLAGS_log_output)
    logger.closeLogFiles();
  return 0;
}
