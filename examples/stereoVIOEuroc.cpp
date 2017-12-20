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

#include "ETH_parser.h"
#include "StereoVisionFrontEnd.h"
#include "FeatureSelector.h"
#include "Visualizer.h"
#include "LoggerMatlab.h"
#include "VioBackEnd.h"
#include <gtsam/geometry/Pose3.h>

using namespace std;
using namespace gtsam;
using namespace VIO;

// helper function to parse dataset and user-specified parameters
void parseDatasetAndParams(const int argc, const char *argv[],
    //output:
    ETHDatasetParser& dataset,VioBackEndParams& vioParams,VioFrontEndParams& trackerParams,
    size_t& initial_k, size_t& final_k){

  // dataset path
  std::string dataset_path;
  if (argc < 2){
    dataset_path = "/Users/Luca/data/MH_01_easy";
    std::cout << "stereoVIOexample: no dataset path specified, using default path: " << dataset_path << std::endl;
  }else{
    dataset_path = argv[1];
    std::cout << "stereoVIOexample: using user-specified dataset path: " << dataset_path << std::endl;
  }

  // parse the dataset (ETH format)
  std::string leftCameraName = "cam0";
  std::string rightCameraName = "cam1";
  std::string imuName = "imu0";
  std::string gtSensorName = "state_groundtruth_estimate0";

  dataset.parseDataset(dataset_path, leftCameraName, rightCameraName, imuName, gtSensorName);
  dataset.print();

  // read/define vio params
  if (argc < 3){
    std::cout << "stereoVIOexample: no vio parameters specified, using default" << std::endl;
    vioParams = VioBackEndParams(dataset.imuData_.gyro_noise_, dataset.imuData_.acc_noise_,
        dataset.imuData_.gyro_walk_, dataset.imuData_.acc_walk_); // default params with IMU stats from dataset
  }else{
    std::string vioParams_path = argv[2];
    std::cout << "stereoVIOexample: using user-specified vio parameters: " << vioParams_path << std::endl;
    vioParams.parseYAML(vioParams_path);
  }

  // read/define tracker params
  if (argc < 4){
    std::cout << "stereoVIOexample: no tracker parameters specified, using default" << std::endl;
    trackerParams = VioFrontEndParams(); // default params
  }else{
    std::string trackerParams_path = argv[3];
    std::cout << "stereoVIOexample: using user-specified vio parameters: " << trackerParams_path << std::endl;
    trackerParams.parseYAML(trackerParams_path);
  }

  // start processing dataset from frame initial_k
  initial_k = 50; // useful to skip a bunch of images at the beginning (imu calibration)
  if (argc >= 5){
    initial_k = size_t(stoi(argv[4]));
    std::cout << "stereoVIOexample: using user-specified initial_k: " << initial_k << std::endl;
  }

  // finish processing dataset at frame final_k
  final_k = dataset.nrImages() - 100; // last frame to process (to avoid processing the entire dataset), skip last frames
  if (argc >= 6){
    final_k = size_t(stoi(argv[5]));
    std::cout << "stereoVIOexample: using user-specified final_k (may be saturated later): " << final_k << std::endl;
  }
  final_k = std::min(final_k,dataset.nrImages());
  std::cout << "Running dataset between frame " << initial_k << " and frame " <<  final_k << std::endl;

  if(initial_k < 10)
    throw std::runtime_error("stereoVIOExample: initial_k should be > 10 for IMU bias initialization");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// stereoVIOexample
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int main(const int argc, const char *argv[])
{
  // initialize random seed for repeatability (only on the same machine)
  // srand(0); // still does not make RANSAC REPEATABLE across different machines
  const int saveImages = 0;         // 0: don't show, 1: show, 2: write & save
  const int saveImagesSelector = 1; // 0: don't show, >0 write & save
  const bool doVisualize = true;
  VisualizationType visualizationType = VisualizationType::MESH3D; // MESH3D; //POINTCLOUD

  ETHDatasetParser dataset;
  VioBackEndParams vioParams;
  VioFrontEndParams trackerParams;
  size_t initial_k, final_k; // initial and final frame: useful to skip a bunch of images at the beginning (imu calibration)
  parseDatasetAndParams(argc,argv, dataset, vioParams, trackerParams, initial_k, final_k);

  // instantiate stereo tracker (class that tracks implements estimation front-end) and print parameters
  StereoVisionFrontEnd stereoVisionFrontEnd(trackerParams,vioParams,saveImages); // vioParams used by feature selection
  stereoVisionFrontEnd.tracker_.trackerParams_.print();
  if(saveImages>0){
    stereoVisionFrontEnd.outputImagesPath_ = "./outputStereoTrackerImages-" + dataset.dataset_name_;
    stereoVisionFrontEnd.tracker_.outputImagesPath_ = "./outputTrackerImages-" + dataset.dataset_name_;
  }

  // instantiate feature selector: not used in vanilla implementation
  FeatureSelector featureSelector(trackerParams, vioParams);

  // Create VIO: class that tracks implements estimation back-end
  boost::shared_ptr<VioBackEnd> vioBackEnd;

  // create class to visualize 3D points and mesh:
  Mesher mesher;
  Visualizer visualizer;

  // structures to be filled with imu data
  ImuStamps imu_stamps;
  ImuAccGyr imu_accgyr;

  // instantiate Logger class (stores data for matlab visualization)
  LoggerMatlab logger;
  logger.openLogFiles();

  Timestamp timestamp_lkf = dataset.timestampAtFrame(initial_k-10); //  timestamp 10 frames before the first (for imu calibration) //TODO: remove hardcoded 10
  Timestamp timestamp_k;

  bool didFirstOptimization = false;
  double startTime; // to log timing results

  // start actual processing of the dataset
  for(size_t k = initial_k; k < final_k; k++)  // for each image
  {
    std::cout << "------------------- Processing frame k="<< k << "--------------------" << std::endl;
    std::string leftImageName = dataset.camera_image_lists["cam0"].img_lists[k].second;
    std::string rightImageName = dataset.camera_image_lists["cam1"].img_lists[k].second;
    timestamp_k = dataset.timestampAtFrame(k); // same in both images

    // load stereo images
    startTime = UtilsOpenCV::GetTimeInSeconds();
    StereoFrame stereoFrame_k(k,timestamp_k,leftImageName,rightImageName,
        dataset.camera_info["cam0"],dataset.camera_info["cam1"],
        dataset.camL_Pose_calR, trackerParams.getStereoMatchingParams());
    logger.timing_loadStereoFrame_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

    ////////////////////////////////////////////////////////////////
    // for k == 1 (initial frame)
    if(k==initial_k){
      // process frame
      stereoVisionFrontEnd.processFirstStereoFrame(stereoFrame_k);

      // get IMU data
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_.getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

      // create VIO
      vioBackEnd = boost::make_shared<VioBackEnd>(stereoVisionFrontEnd.stereoFrame_km1_->B_Pose_camLrect,
          stereoVisionFrontEnd.stereoFrame_km1_->left_undistRectCameraMatrix_,
          stereoVisionFrontEnd.stereoFrame_km1_->baseline_, vioParams);

      // initialize Vio
      gtNavState initialStateGT;
      if(vioParams.autoInitialize_){ // use initial IMU measurements to guess first pose
        initialStateGT.pose = vioBackEnd->GuessPoseFromIMUmeasurements(imu_accgyr, vioParams.n_gravity_,vioParams.roundOnAutoInitialize_);
        vioBackEnd->initializeStateAndSetPriors(timestamp_k, initialStateGT.pose,imu_accgyr);
        initialStateGT.velocity = vioBackEnd->W_Vel_Blkf_; // only for display later on
        initialStateGT.imuBias = vioBackEnd->imu_bias_lkf_;// only for display
      }
      else{
        initialStateGT = dataset.getGroundTruthState(timestamp_k);
        vioBackEnd->initializeStateAndSetPriors(timestamp_k, initialStateGT.pose,initialStateGT.velocity,initialStateGT.imuBias);
      }
      ////////////////// DEBUG INITIALIZATION ////////////////////////////////////////////////
      logger.displayInitialStateVioInfo(dataset,vioBackEnd,initialStateGT,imu_accgyr,timestamp_k);
      logger.W_Pose_Bprevkf_vio_ = vioBackEnd->W_Pose_Blkf_; // store latest pose estimate

      timestamp_lkf = timestamp_k; // store latest keyframe time stamp
      continue;
    }

    ////////////////////////////////////////////////////////////////
    // for k > 1
    Frame frame_km1_debug = Frame(stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_);  // for visualization purposes

    // integrate rotation measurements (rotation is used in RANSAC)
    std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_.getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);
    gtsam::Rot3 calLrectLkf_R_camLrectKf_imu = vioBackEnd->preintegrateGyroMeasurements(imu_stamps, imu_accgyr);

    ////////////////// FRONT-END ////////////////////////////////////////////////
    // main function for tracking
    startTime = UtilsOpenCV::GetTimeInSeconds();
    StatusSmartStereoMeasurements statusSmartStereoMeasurements = stereoVisionFrontEnd.processStereoFrame(stereoFrame_k, calLrectLkf_R_camLrectKf_imu); // rotation used in 1 and 2 point ransac
    logger.timing_processStereoFrame_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

    // pass info to vio if it's keyframe
    startTime = UtilsOpenCV::GetTimeInSeconds();
    if(stereoVisionFrontEnd.stereoFrame_km1_->isKeyframe_) // it's a keyframe!
    {
      std::cout << "Keyframe " << k << " with: " << statusSmartStereoMeasurements.second.size() << " smart measurements" << std::endl;

      //////////////////////////////////// FEATURE SELECTOR ////////////////////////////////////
      // ------------------ DATA ABOUT CURRENT AND FUTURE ROBOT STATE ----------------- //
      KeyframeToStampedPose posesAtFutureKeyframes;
      size_t nrKfInHorizon =
          round(stereoVisionFrontEnd.tracker_.trackerParams_.featureSelectionHorizon_ / stereoVisionFrontEnd.tracker_.trackerParams_.intra_keyframe_time_);
      std::cout << "nrKfInHorizon for selector: " << nrKfInHorizon << std::endl;
      // Future poses are gt and might be far from the vio pose: we have to attach the *relative* poses from the gt to the latest vio estimate
      // W_Pose_Bkf_gt    : ground truth pose at previous keyframe
      // vio->W_Pose_Blkf_: vio pose at previous keyframe
      Pose3 W_Pose_Bkf_gt = dataset.getGroundTruthState(timestamp_lkf).pose; // more important than the time, it is important that
      // it is the same time as vio->W_Pose_Blkf_
      for(size_t kk = 0; kk < nrKfInHorizon+1; kk++){// including current pose
        Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(kk * stereoVisionFrontEnd.tracker_.trackerParams_.intra_keyframe_time_);
        Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(dataset.getGroundTruthState(timestamp_kk).pose); // relative pose wrt ground truth at last kf
        posesAtFutureKeyframes.push_back( StampedPose( vioBackEnd->W_Pose_Blkf_.compose(poseGT_km1_kk) , UtilsOpenCV::NsecToSec(timestamp_kk)) );
      }
      startTime = UtilsOpenCV::GetTimeInSeconds();
      SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements;
      std::tie(trackedAndSelectedSmartStereoMeasurements, stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_) =
          featureSelector.splitTrackedAndNewFeatures_Select_Display(stereoVisionFrontEnd.stereoFrame_km1_,statusSmartStereoMeasurements.second,
          vioBackEnd->cur_id_, saveImagesSelector,
          trackerParams.featureSelectionCriterion_, trackerParams.featureSelectionNrCornersToSelect_, trackerParams.maxFeatureAge_,
          posesAtFutureKeyframes, vioBackEnd->getCurrentStateCovariance(),
          dataset.dataset_name_, frame_km1_debug); // last 2 are for visualization
      logger.timing_featureSelection_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
      TrackerStatusSummary status = statusSmartStereoMeasurements.first;
      statusSmartStereoMeasurements = std::make_pair(status, // same status as before
              trackedAndSelectedSmartStereoMeasurements);
      std::cout << "overall selection time (logger.timing_featureSelection_) " << logger.timing_featureSelection_ << std::endl;
      std::cout << "actual selection time (stereoTracker.tracker_.debugInfo_.featureSelectionTime_) " << stereoVisionFrontEnd.tracker_.debugInfo_.featureSelectionTime_ << std::endl;

      ////////////////// DEBUG INFO FOR FRONT-END /////////////////////////////////////////////////////////////////
      startTime = UtilsOpenCV::GetTimeInSeconds();
      logger.logFrontendResults(dataset,stereoVisionFrontEnd,timestamp_lkf,timestamp_k);
      logger.timing_loggerFrontend_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

      //////////////////// BACK-END /////////////////////////////////////////////////////////////////////
      // get IMU data
      std::tie(imu_stamps, imu_accgyr) = dataset.imuData_.imu_buffer_.getBetweenValuesInterpolated(timestamp_lkf, timestamp_k);

      // process data with VIO
      startTime = UtilsOpenCV::GetTimeInSeconds();

      if(vioParams.addBetweenStereoFactors_ == true && stereoVisionFrontEnd.trackerStatusSummary_.kfTrackingStatus_stereo_ == Tracker::VALID ){
        vioBackEnd->addVisualInertialStateAndOptimize(
            timestamp_k, // current time for fixed lag smoother
            statusSmartStereoMeasurements, // vision data
            imu_stamps, imu_accgyr, // inertial data
            stereoVisionFrontEnd.getRelativePoseBodyStereo()); // optional: pose estimate from stereo ransac
      }else{
        vioBackEnd->addVisualInertialStateAndOptimize(timestamp_k,statusSmartStereoMeasurements,imu_stamps, imu_accgyr); // same but no pose
      }
      logger.timing_vio_ = UtilsOpenCV::GetTimeInSeconds() - startTime;

      ////////////////// DEBUG INFO FOR BACK-END /////////////////////////////////////////////////////////////////////
      startTime = UtilsOpenCV::GetTimeInSeconds();
      logger.logBackendResults(dataset,stereoVisionFrontEnd,vioBackEnd,timestamp_lkf,timestamp_k,k);
      logger.W_Pose_Bprevkf_vio_ = vioBackEnd->W_Pose_Blkf_;
      logger.timing_loggerBackend_ = UtilsOpenCV::GetTimeInSeconds() - startTime;
      logger.displayOverallTiming();
      ///////////////////////////////////////////////////////////////////////////////////////////////////////

      ////////////////// CREATE AND VISUALIZE MESH /////////////////////////////////////////////////////////////////////
      if(doVisualize){
        //cv::Mat img = stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.img_.clone();
        //cv::cvtColor(img, img, cv::COLOR_GRAY2BGR);
        //UtilsOpenCV::DrawCrossesInPlace(img, stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_.getValidKeypoints(), cv::Scalar(0, 0, 255),0.4);
        //cv::imshow("Valid keypoints", img);
        //cv::waitKey(100);

        // vector<Vec6f> triangulation2D = Mesher::CreateMesh2D(stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_);
        // std::cout <<"visualizing mesh:" << std::endl;
        // Mesher::VisualizeMesh2D(stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_, triangulation2D, 100);

        // visualize points 3D
        // vector<Point3> points3d = vioBackEnd->get3DPoints();
        // mesher.visualizeMap3D_repeatedPoints(points3d);

        // visualize points 3D without repetition
        if(visualizationType == VisualizationType::POINTCLOUD){
          VioBackEnd::PointsWithId pointsWithId = vioBackEnd->get3DPointsAndLmkIds();
          mesher.updateMap3D(pointsWithId);
          visualizer.visualizePoints3D(pointsWithId,mesher);
        }

        if(visualizationType == VisualizationType::MESH2DTo3D){
          VioBackEnd::PointsWithId pointsWithId = vioBackEnd->get3DPointsAndLmkIds();
          mesher.updateMesh3D(pointsWithId,stereoVisionFrontEnd.stereoFrame_lkf_->left_frame_);
          visualizer.visualizeMesh3D(mesher);
        }

        if(visualizationType == VisualizationType::MESH3D){
          VioBackEnd::PointsWithId pointsWithId = vioBackEnd->get3DPointsAndLmkIds();
          mesher.updateMap3D(pointsWithId);
          visualizer.visualizeMesh3D(mesher.mapPoints3d_, Mesher_cgal::CreateMesh3D_MapPointId(mesher.mapPoints3d_));
        }

        // visualize trajectory
        std::cout <<"add pose" << std::endl;
        visualizer.addPoseToTrajectory(vioBackEnd->W_Pose_Blkf_);
        std::cout <<"visualizeTrajectory" << std::endl;
        visualizer.visualizeTrajectory();
        std::cout <<"spinOnce" << std::endl;
        visualizer.myWindow_.spinOnce(50);
      }

      didFirstOptimization = true;
      timestamp_lkf = timestamp_k;
    }
    if(k == final_k-1)
      std::cout << "stereoVIOExample completed successfully!" << std::endl;
  }

  logger.closeLogFiles();
  return 0;
}
