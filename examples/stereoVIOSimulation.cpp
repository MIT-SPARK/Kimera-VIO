/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   stereoVIOSimulation.cpp
 * @brief  example of VIO pipeline running on a simulated dataset
 * @author Luca Carlone
 */

#include "ETH_parser.h"
#include "StereoVisionFrontEnd.h"
#include "VioBackEnd.h"
#include <gtsam/nonlinear/Marginals.h>
#include "FeatureSelector.h"
#include <algorithm>    // std::shuffle
#include <array>        // std::array
#include <random>       // std::default_random_engine
#include <chrono>       // std::chrono::system_clock

using namespace std;
using namespace gtsam;
using namespace VIO;

struct FeatureObservation{
  LandmarkId lmkId;
  gtsam::Point2 px;
  double sigma;
};
typedef vector<FeatureObservation> FeatureObservations;

// TO CALL: ./stereoVIOExampleSimulation selector seed lazy, where seed is an integer and selector is 0 (QUALITY), 1 (MIN_EIG), or 2 (LOGDET), and lazy is 0 or 1
int main(const int argc, const char *argv[])
{
  bool addNoise = true;
  unsigned seed = 0; // std::chrono::system_clock::now().time_since_epoch().count();
  if (argc >= 3){
    seed = stoul(argv[2]);
  }
  std::cout << "stereoVIOexampleSimulation: seed = " << seed << std::endl;
  srand(seed);

  // dataset name
  std::string dataset_path = "../python-simulator/Square_01_easy/";

  // store output data and debug info:
  std::ofstream outputFile;               UtilsOpenCV::OpenFile("./output.txt",outputFile);
  std::ofstream outputFile_posesVIO;      UtilsOpenCV::OpenFile("./output_posesVIO.txt",outputFile_posesVIO);
  std::ofstream outputFile_posesGT;       UtilsOpenCV::OpenFile("./output_posesGT.txt",outputFile_posesGT);
  std::ofstream outputFile_smartFactors;  UtilsOpenCV::OpenFile("./output_smartFactors.txt",outputFile_smartFactors);
  std::ofstream outputFile_timingVIO;     UtilsOpenCV::OpenFile("./output_timingVIO.txt",outputFile_timingVIO);
  std::ofstream outputFile_timingTracker; UtilsOpenCV::OpenFile("./output_timingTracker.txt",outputFile_timingTracker);
  std::ofstream outputFile_statsTracker;  UtilsOpenCV::OpenFile("./output_statsTracker.txt",outputFile_statsTracker);
  std::ofstream outputFile_statsFactors;  UtilsOpenCV::OpenFile("./output_statsFactors.txt",outputFile_statsFactors);

  ////////////////////////////// PARSE DATASET //////////////////////////////
  // parse the simulated dataset: parses
  // groundtruth_states
  std::string leftCameraName = "cam0";
  std::string rightCameraName = "cam1";
  std::string imuName = "imu0";
  std::string gtSensorName = "state_groundtruth_estimate0";
  ETHDatasetParser dataset;
  bool doParseImages = false;
  dataset.parseDataset(dataset_path, leftCameraName, rightCameraName, imuName, gtSensorName,doParseImages);
  dataset.print();

  double baseline = 0.12; // TODO: does this matter?
  ////////////////////////////// PARSE FEATURE DATA //////////////////////////////
  // LANDMARKS
  std::map<LandmarkId,gtsam::Point3> landmarkPositions;
  { // only thing that gets updated here is landmarkPositions
    const std::string landmarkFilename = dataset_path + "/mav0/landmarks.csv";
    std::ifstream fin(landmarkFilename.c_str());
    if (!fin.is_open()) { std::cout << "Cannot open file: " << landmarkFilename << std::endl; exit(-1); }
    // skip the first line, containing the header
    std::string line; std::getline(fin, line);
    // read/store list of image names
    while (std::getline(fin, line)) {
      LandmarkId landmarkId;
      gtsam::Point3 landmarkPosition;
      for(size_t i=0; i < 4; i++){
        int idx = line.find_first_of(',');
        if (i==0)
          landmarkId = std::stoll(line.substr(0, idx));
        else
          landmarkPosition(i-1) = std::stod(line.substr(0, idx));
        line = line.substr(idx+1);
      }
      landmarkPositions[landmarkId] = landmarkPosition;
    }
    fin.close();
  }
  long long int landmarkCount = landmarkPositions.size();

  // FEATURE OBSERVATIONS:
  std::vector<Timestamp> keyframesTimestamps;
  std::map<Timestamp,FeatureObservations> timestampToFeatureObservations;
  { // only thing that gets updated here are keyframesTimestamps and timestampToFeatureObservations
    const std::string featureTrackFilename = dataset_path + "/mav0/cam0_tracks.csv";
    std::ifstream fin(featureTrackFilename.c_str());
    if (!fin.is_open()) { std::cout << "Cannot open file: " << featureTrackFilename << std::endl; exit(-1); }
    // skip the first line, containing the header
    std::string line; std::getline(fin, line);
    // read/store list of image names
    Timestamp lastTimestampParsed = -1;
    while (std::getline(fin, line)) {
      FeatureObservation fo_i;
      double x,y;
      Timestamp time_i;
      for(size_t i=0; i < 5; i++){
        int idx = line.find_first_of(',');
        if (i==0)
          time_i = std::stoll(line.substr(0, idx));
        else if(i == 1)
          fo_i.lmkId = std::stod(line.substr(0, idx));
        else if(i == 2)
          x = std::stod(line.substr(0, idx));
        else if(i == 3)
          y = std::stod(line.substr(0, idx));
        else
          fo_i.sigma = std::stod(line.substr(0, idx));
        line = line.substr(idx+1);
      }
      fo_i.px = gtsam::Point2(x,y);
      if(time_i != lastTimestampParsed) // new keyframe
        keyframesTimestamps.push_back(time_i);
      timestampToFeatureObservations[time_i].push_back(fo_i);
      lastTimestampParsed = time_i;
    }
    fin.close();
  }
  size_t nrKeyframes = keyframesTimestamps.size();
  std::cout << "nrKeyframes: " << nrKeyframes << std::endl;

  //////////////////////////////  START PROCESSING ///////////////////////////////
  // read/define vio params
  VioBackEndParams vioParams = VioBackEndParams();
  vioParams.outlierRejection_ = 1e9; // disabled
  vioParams.landmarkDistanceThreshold_ = 1e9; // disabled
  vioParams.accBiasSigma_ = 1e-5; // constant bias
  vioParams.gyroBiasSigma_ = 1e-5; // constant bias

  // read/define tracker params
  VioFrontEndParams trackerParams = VioFrontEndParams();
  trackerParams.useStereoTracking_ = false;
  trackerParams.maxFeaturesPerFrame_ = 20;
  trackerParams.intra_keyframe_time_ = 0.4;
  trackerParams.featureSelectionCriterion_ = VioFrontEndParams::FeatureSelectionCriterion::QUALITY;
  if (argc >= 2){
    switch(stoi(argv[1])){
    case 1 : trackerParams.featureSelectionCriterion_ = VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG; break;
    case 2 : trackerParams.featureSelectionCriterion_ = VioFrontEndParams::FeatureSelectionCriterion::LOGDET; break;
      default :  break;
    }
  }
  std::cout << "Feature selection criterion: " << trackerParams.featureSelectionCriterion_ << std::endl;

  trackerParams.featureSelectionUseLazyEvaluation_ = true;
  if (argc >= 4){
    if(stoi(argv[3]) == 0){
      trackerParams.featureSelectionUseLazyEvaluation_ = false;
    }
  }
  std::cout << "featureSelectionUseLazyEvaluation_: " << trackerParams.featureSelectionUseLazyEvaluation_ << std::endl;

  // to generate measurement noise:
  std::cout << "dataset.imuData_.imu_rate_ : " << dataset.imuData_.imu_rate_ << std::endl;
  std::default_random_engine generator;
  std::normal_distribution<double> pixelNoise(0.0,1.0); // 0 mean and suitable std (vioParams.smartNoiseSigma_)
  std::normal_distribution<double> accDiscreteNoise(0.0,vioParams.accNoiseDensity_ * sqrt(dataset.imuData_.imu_rate_)); // 0 mean and suitable std
  std::normal_distribution<double> gyroDiscreteNoise(0.0,vioParams.gyroNoiseDensity_ * sqrt(dataset.imuData_.imu_rate_)); // 0 mean and suitable std

  // Create VIO: class that tracks entire history
  boost::shared_ptr<VioBackEnd> vio;

  // create Feature selector
  FeatureSelector featureSelector = FeatureSelector(trackerParams,vioParams);

  // structures to be filled by imu data
  ImuStampS imu_stamps;
  ImuAccGyrS imu_accgyr;

  size_t initial_k = 10; // useful to skip a bunch of images at the beginning
  if(initial_k < 10)
    throw std::runtime_error("stereoVIOExample: initial_k should be > 10 for IMU bias initialization");

  size_t final_k = nrKeyframes - 10; // otherwise the feature selector will query unknown poses

  Timestamp timestamp_lkf = keyframesTimestamps.at(initial_k-10); // timestamp 10 frames before the first (for imu calibration)
  Timestamp timestamp_k;
  gtsam::Pose3 W_Pose_Bprevkf_vio; // for debugging
  gtsam::Pose3 W_Pose_Bkf_gt; // for debugging
  double vioRotError,vioTranError;
  bool didFirstOptimization = false;
  std::map<LandmarkId,int> lmkIdToAge;
  std::map<LandmarkId,LandmarkId> lmkIdToNewLmkId;
  SmartStereoMeasurements previousSmartStereoMeasurements;
  for(size_t k = initial_k; k < final_k; k++)  // for each keyframe
  {
    timestamp_k = keyframesTimestamps.at(k);
    std::cout << "Time between keyframes: "<< UtilsOpenCV::NsecToSec(timestamp_k - keyframesTimestamps.at(k-1)) << std::endl;

    ////////////////////////////////////////////////////////////////
    // for k == 1 (initial frame)
    if(k==initial_k){

      // create and initialize VIO
      std::shared_ptr<gtNavState> initialStateGT =
              std::make_shared<gtNavState>(dataset.getGroundTruthState(timestamp_k));
      initialStateGT->print("initialStateGT\n");
      gtsam::Vector3 rpy_gt = initialStateGT->pose_.rotation().rpy(); // such that R = Rot3::Ypr(y,p,r)
      std::cout << "yaw= " << rpy_gt(2) << " pitch= " << rpy_gt(1) << " roll= "<< rpy_gt(0) << std::endl;
      Vector3 localGravity = initialStateGT->pose_.rotation().inverse().matrix() * vio->getBackEndParams().n_gravity_;
      std::cout << "gravity in local frame: \n" << localGravity << std::endl;
      std::cout << "expected initial acc measurement: \n" << -localGravity + initialStateGT->imu_bias_.accelerometer()  << std::endl;
      std::cout << "actual initial acc measurement: \n" << imu_accgyr.block<3,1>(0,0) << std::endl;
      std::cout << "expected initial gyro measurement: \n" << initialStateGT->imu_bias_.gyroscope()  << std::endl;
      std::cout << "actual initial gyro measurement: \n" << imu_accgyr.block<3,1>(3,0) << std::endl;
      vio = boost::make_shared<VioBackEnd>(dataset.getLeftCamInfo().body_Pose_cam_,
          UtilsOpenCV::Cvmat2Cal3_S2(dataset.getLeftCamInfo().camera_matrix_), // no distortion nor rectification needed
          baseline, &initialStateGT, timestamp_k, imu_accgyr, vioParams);
      vio->print();

      std::tie(vioRotError,vioTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(initialStateGT->pose_, vio->getWPoseBLkf());
      if(vioRotError > 1e-4 || vioTranError > 1e-4)
        throw std::runtime_error("stereoVIOExample: wrong initialization");

      // for comparison: gt bias:
      dataset.gtData_.mapToGt_[timestamp_k].imu_bias_.print("Ground truth initial bias: \n");

      W_Pose_Bprevkf_vio = vio->getWPoseBLkf();
      timestamp_lkf = timestamp_k;
      continue;
    }

    ////////////////////////////////////////////////////////////////
    // for k > 1
    // GENERATE FEATURE OBSERVATIONS:
    SmartStereoMeasurements trackedSmartStereoMeasurements,newSmartStereoMeasurements;
    vector<bool> isNewFeature;
    FeatureObservations fobs_t = timestampToFeatureObservations[timestamp_k];
    vector<gtsam::Vector3> trackedkeypoints_3d;
    vector<size_t> trackerLandmarksAge;

    for(FeatureObservation fobs_t_i : fobs_t){
      // expected:
      LandmarkId lid = fobs_t_i.lmkId;

      /// LIMIT FEATURE TRACKS TO maxFeatureAge_
      // check if we have seen it before
      if (lmkIdToAge.find(lid) == lmkIdToAge.end()){ // first time seen
        lmkIdToNewLmkId[lid] = lid; // we start calling it with its own id
        lmkIdToAge[ lid ] = 0; // initialize age to zero
      }else{
        lmkIdToAge[ lid ] += 1; // seen in one more frame
      }
      // if feature has age > 0, but it was not seen in the last frame, it is a loop closure and we have to discard it
      if(lmkIdToAge[ lid ] > 0){
        bool seenInPreviosKeyframe = false;
        for(auto smartStereoMeas : previousSmartStereoMeasurements){
          if(lmkIdToNewLmkId[ lid ] == smartStereoMeas.first){
            seenInPreviosKeyframe = true; break;
          }
        }
        if(!seenInPreviosKeyframe) // it is a loop, closure, we have to give a new id, since we do not detect lc
          lmkIdToAge[ lid ] = trackerParams.maxFeatureAge_+1; // this will be "refreshed" in the following if
      }
      // check if feature track was too long
      if(lmkIdToAge[ lid ] > trackerParams.maxFeatureAge_){
        // create new landmark
        landmarkCount++;
        lmkIdToNewLmkId[lid] = landmarkCount; // new name
        lmkIdToAge[ lid ] = 0; // new history
        // std::cout << "landmarkCount " << landmarkCount << std::endl;
      }

      gtsam::Point3 lposition = landmarkPositions[lid];
      // ground truth left camera
      gtsam::Pose3 camLPose = dataset.getGroundTruthState(timestamp_k).pose_.compose(dataset.getLeftCamInfo().body_Pose_cam_);
      gtsam::PinholeCamera<gtsam::Cal3_S2> camL(camLPose,
          UtilsOpenCV::Cvmat2Cal3_S2(dataset.getLeftCamInfo().camera_matrix_));
      gtsam::Point2 expected_px = camL.project(lposition);
      // actual
      gtsam::Point2 actual_px = fobs_t_i.px;
      if( (expected_px - actual_px).norm() > 1e-2) //  check that we understood measurement conventions correctly
        throw std::runtime_error("stereoVIOExampleSimulation: pixel projection mismatch");

      // pack into smartStereoMeasurements
      double uL = actual_px.x();
      double v = actual_px.y();
      if(addNoise){
        double noiseU = pixelNoise(generator);
        uL += noiseU;
        double noiseV = pixelNoise(generator);
        v += noiseV;
        // std::cout << "noiseU: " << noiseU << " noiseV : " << noiseV << std::endl;
      }
      double uR = std::numeric_limits<double>::quiet_NaN(); // missing pixel information
      gtsam::StereoPoint2 stereo_px(uL,uR,v);

      // if age =  0, it is a newly detected feature, that we can select, otherwise it's a feature we are tracking
      if(lmkIdToAge[ lid ] == 0)
        newSmartStereoMeasurements.push_back(std::make_pair( lmkIdToNewLmkId[lid] , stereo_px));
      else{
        trackedSmartStereoMeasurements.push_back(std::make_pair( lmkIdToNewLmkId[lid] , stereo_px));
        trackedkeypoints_3d.push_back( camLPose.transform_to(lposition) );
        trackerLandmarksAge.push_back(lmkIdToAge[ lid ]);
      }
    }

    /////////////////////// FEATURE SELECTION: ///////////////////////
    std::cout << "Nr of tracked: " << trackedSmartStereoMeasurements.size() << std::endl;
    std::cout << "Nr of new features (before selection): " << newSmartStereoMeasurements.size() << std::endl;
    VioFrontEndParams::FeatureSelectionCriterion criterion = trackerParams.featureSelectionCriterion_; // for brevity
    std::cout << "criterion: "  << criterion << std::endl;
    double featureSelectionTime = 0;
    int need_nr_features = std::max( trackerParams.maxFeaturesPerFrame_ - int(trackedkeypoints_3d.size()), int (0) );
    if(newSmartStereoMeasurements.size() > need_nr_features) // if we have to select something
    {
      if(criterion == VioFrontEndParams::FeatureSelectionCriterion::QUALITY || !didFirstOptimization){
        // in simulation there is no quality, hence we pick max nr of features at random:
        unsigned seedShuffle = rand(); // randomized seed, but fixed if we fix srand
        std::shuffle ( newSmartStereoMeasurements.begin(), newSmartStereoMeasurements.end(), std::default_random_engine(seedShuffle) ); // randomize order
        newSmartStereoMeasurements.resize(need_nr_features); // take only first maxFeaturesPerFrame_
      }else{ // we have to do some feature selection and we already did the first optimization (otherwise we do not have initial covariance in smoother)
        FeatureSelectorData featureSelectionData;
        KeyframeToStampedPose posesAtFutureKeyframes;
        size_t nrKfInHorizon = round(trackerParams.featureSelectionHorizon_ / trackerParams.intra_keyframe_time_);
        std::cout << "nrKfInHorizon for selector: " << nrKfInHorizon << std::endl;
        // DATA ABOUT CURRENT AND FUTURE ROBOT STATE
        // Future poses are gt and might be far from the vio pose: we have to attach the *relative* poses from the gt to the latest vio estimate
        // W_Pose_Bkf_gt    : ground truth pose at previous time stamp (k-1)
        // vio->W_Pose_Blkf_: vio pose at previous time stamp (k-1)
        for(size_t kk = 0; kk < nrKfInHorizon+1; kk++){// including current pose
          Timestamp timestamp_kk = timestamp_k + UtilsOpenCV::SecToNsec(kk * trackerParams.intra_keyframe_time_);
          Pose3 poseGT_km1_kk = W_Pose_Bkf_gt.between(dataset.getGroundTruthState(timestamp_kk).pose_); // relative pose wrt ground truth at time k-1
          posesAtFutureKeyframes.push_back( StampedPose( vio->getWPoseBLkf().compose(poseGT_km1_kk) , UtilsOpenCV::NsecToSec(timestamp_kk)) );
        }
        std::cout << "getting covariance" << std::endl;
        featureSelectionData.currentNavStateCovariance = vio->getCurrentStateCovariance(); // covariance of current state
        featureSelectionData.posesAtFutureKeyframes = posesAtFutureKeyframes;
        // DATA ABOUT FEATURES WE ARE TRACKING:
        std::cout << "selector: populating data about existing feature tracks" << std::endl;
        featureSelectionData.keypoints_3d = trackedkeypoints_3d; // current 3D points
        featureSelectionData.keypointLife.reserve(trackerLandmarksAge.size());
        for(int age : trackerLandmarksAge) // compute age as maxFeatureAge_ - current age
          featureSelectionData.keypointLife.push_back(trackerParams.maxFeatureAge_-age); // this is life
        if(featureSelectionData.keypoints_3d.size() != featureSelectionData.keypointLife.size())
          throw std::runtime_error("processStereoFrame: keypoint age inconsistent with keypoint 3D");
        featureSelectionData.body_P_leftCam = dataset.getLeftCamInfo().body_Pose_cam_;
        featureSelectionData.body_P_rightCam = dataset.getRightCamInfo().body_Pose_cam_;
        featureSelectionData.left_undistRectCameraMatrix = UtilsOpenCV::Cvmat2Cal3_S2(dataset.getLeftCamInfo().camera_matrix_);
        featureSelectionData.right_undistRectCameraMatrix = UtilsOpenCV::Cvmat2Cal3_S2(dataset.getRightCamInfo().camera_matrix_);
        // DATA ABOUT NEW FEATURES:
        std::cout << "selector: populating data about new feature tracks" << std::endl;
        KeypointsCV corners; std::vector<double> successProbabilities, availableCornerDistances;
        for(auto stereoMeas : newSmartStereoMeasurements){
          corners.push_back(KeypointCV(stereoMeas.second.uL(), stereoMeas.second.v()));
          successProbabilities.push_back(1.0);
          availableCornerDistances.push_back(0.0); // not available
        }
        std::vector<size_t> selectedIndices; // only for debug
        std::vector<double> selectedGains;
        std::cout << "selector: calling selector" << std::endl;
        // featureSelectionData.print();
        double startTime = UtilsOpenCV::GetTimeInSeconds();
        std::tie(corners,selectedIndices,selectedGains) = featureSelector.featureSelectionLinearModel(
            corners,
            successProbabilities,// for each corner, in [0,1]
            availableCornerDistances, // 0 if not available
            dataset.getLeftCamInfo(),
            need_nr_features,featureSelectionData,criterion);
        featureSelectionTime = UtilsOpenCV::GetTimeInSeconds() - startTime;
        std::cout << "selector: done, featureSelectionTime " << featureSelectionTime << std::endl;
        // populate SmartStereoMeasurements with results
        SmartStereoMeasurements tmp = newSmartStereoMeasurements;
        newSmartStereoMeasurements.resize(0); // delete everything and repopulate
        for(auto i : selectedIndices){
          newSmartStereoMeasurements.push_back(tmp.at(i));
        }
        std::cout << "selector: populated SmartStereoMeasurements" << std::endl;
      }
    }
    std::cout << "New of new features (after selection): " << newSmartStereoMeasurements.size() << std::endl;
    SmartStereoMeasurements smartStereoMeasurements = trackedSmartStereoMeasurements; // always use tracked features
    smartStereoMeasurements.insert( smartStereoMeasurements.end(), newSmartStereoMeasurements.begin(),
        newSmartStereoMeasurements.end() ); // and use the new features that passed the selection

    // pack measurements for VIO
    previousSmartStereoMeasurements = smartStereoMeasurements;
    TrackerStatusSummary trackerStatusSummary;
    trackerStatusSummary.kfTrackingStatus_mono_ = Tracker::TrackingStatus::VALID;
    trackerStatusSummary.kfTrackingStatus_stereo_ = Tracker::TrackingStatus::INVALID;
    StatusSmartStereoMeasurements statusSmartStereoMeasurements = std::make_pair(trackerStatusSummary, smartStereoMeasurements);
    ////////////////// DEBUG FRONT-END ////////////////////////////////////////////////
    double relativeRotError,relativeTranError;
    outputFile << 0 << " "  <<  0 << " " << 0 << " " << 0 << " "; // fill in with zeros since we have no front end
    outputFile << 0 << " " <<  0 << " " << 0 << " " << 0 << " ";
    //////////////////////////////////////////////////////////////////////////////////////

    // Get IMU data.
    dataset.imuData_.imu_buffer_.getImuDataInterpolatedBorders(timestamp_lkf,
                                                               timestamp_k,
                                                               &imu_stamps,
                                                               &imu_accgyr);

    // debug:
    // StereoTracker::PrintStatusStereoMeasurements(statusSmartStereoMeasurements);

    // add noise to inertial data
    if(addNoise){
      size_t nrMeasured = imu_accgyr.cols();
      for(size_t i=0; i < nrMeasured; i++){
        for(size_t j=0; j<3; j++){
          double noiseAcc = accDiscreteNoise(generator);
          imu_accgyr(j,i)  += noiseAcc; // accelerometer
          double noiseGyro = gyroDiscreteNoise(generator);
          imu_accgyr(3+j,i) += noiseGyro; // gyroscope
        }
      }
    }

    // process data with VIO
    vio->addVisualInertialStateAndOptimize(
        timestamp_k, // current time for fixed lag smoother
        statusSmartStereoMeasurements, // vision data
        imu_stamps, imu_accgyr); // inertial data
    didFirstOptimization = true;

    ////////////////// DEBUG BACK-END ////////////////////////////////////////////////
    W_Pose_Bkf_gt = (dataset.getGroundTruthState(timestamp_k)).pose_;
    std::tie(vioRotError,vioTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(W_Pose_Bkf_gt, vio->getWPoseBLkf());
    std::cout << "vioRotError " << vioRotError << ", vioTranError " << vioTranError << std::endl;
    // Absolute vio errors
    outputFile << vio->getCurrKfId() << " "
               << vioRotError << " "
               << vioTranError << " "
               << vio->getLandmarkCount() << " ";

    // RPY vio errors
    gtsam::Vector3 rpy_gt = W_Pose_Bkf_gt.rotation().rpy(); // such that R = Rot3::Ypr(y,p,r)
    gtsam::Vector3 rpy_vio = vio->getWPoseBLkf().rotation().rpy();
    outputFile << rpy_gt(0) << " " <<  rpy_gt(1) << " " << rpy_gt(2) << " "
        << rpy_vio(0) << " " <<  rpy_vio(1) << " " << rpy_vio(2)  << " ";

    // relative vio errors
    gtsam::Pose3 Bprevkf_Pose_Bkf_vio = W_Pose_Bprevkf_vio.between(vio->getWPoseBLkf());
    boost::tie(relativeRotError,relativeTranError) =
        dataset.computePoseErrors(Bprevkf_Pose_Bkf_vio, true, timestamp_lkf, timestamp_k); // always VALID = TRUE
    outputFile << relativeRotError << " " << relativeTranError << " " << std::endl;

    // debug smart factors:
    DebugVioInfo debug_info = vio->getCurrentDebugVioInfo();
    outputFile_smartFactors
        << vio->getCurrKfId() << " "
        << k << " " << UtilsOpenCV::NsecToSec(timestamp_k) // keyframe id, frame id, timestamp
        << " " << debug_info.numSF_ << " " << debug_info.numValid_
        << " " << debug_info.numDegenerate_ << " " << debug_info.numFarPoints_
        << " " << debug_info.numOutliers_ << " " << debug_info.numCheirality_
        << " " << debug_info.meanPixelError_ << " " << debug_info.maxPixelError_
        << " " << debug_info.meanTrackLength_ << " " << debug_info.maxTrackLength_ <<  std::endl;

    // we log the camera since we will display camera poses in matlab
    gtsam::Pose3 W_Pose_camlkf_vio = vio->getWPoseBLkf().compose(vio->getBPoseLeftCam());
    outputFile_posesVIO
        << vio->getCurrKfId() << " "
        << W_Pose_camlkf_vio.translation().transpose() << " "
        << W_Pose_camlkf_vio.rotation().matrix().row(0) << " "
        << W_Pose_camlkf_vio.rotation().matrix().row(1) << " "
        << W_Pose_camlkf_vio.rotation().matrix().row(2) << " "
        << vio->getWVelBLkf().transpose() 				        << " "
        << vio->getLatestImuBias().accelerometer().transpose() << " "
        << vio->getLatestImuBias().gyroscope().transpose() << std::endl;

    // we log the camera since we will display camera poses in matlab
    gtsam::Pose3 W_Pose_camlkf_gt = W_Pose_Bkf_gt.compose(vio->getBPoseLeftCam());
    Vector3 W_Vel_camlkf_gt = (dataset.getGroundTruthState(timestamp_k)).velocity_;
    ImuBias imu_bias_lkf_gt = (dataset.getGroundTruthState(timestamp_k)).imu_bias_;
    outputFile_posesGT
        << vio->getCurrKfId() << " "
        << W_Pose_camlkf_gt.translation().transpose() << " "
        << W_Pose_camlkf_gt.rotation().matrix().row(0) << " "
        << W_Pose_camlkf_gt.rotation().matrix().row(1) << " "
        << W_Pose_camlkf_gt.rotation().matrix().row(2) << " "
        << W_Vel_camlkf_gt.transpose() 				<< " "
        << imu_bias_lkf_gt.accelerometer().transpose() << " "
        << imu_bias_lkf_gt.gyroscope().transpose() << std::endl;

    // log timing for benchmarking and performance profiling
    outputFile_timingVIO << vio->getCurrKfId() << " "
                         << debug_info.factorsAndSlotsTime_ << " "
                         << debug_info.preUpdateTime_ << " "
                         << debug_info.updateTime_ << " "
                         << debug_info.updateSlotTime_ << " "
                         << debug_info.extraIterationsTime_ << " "
                         << debug_info.printTime_ << std::endl;

    // fake front end info:
    outputFile_timingTracker << vio->getCurrKfId() << " " <<
        -1 << " " <<  -1 << " " << -1 << " " <<  -1 << " " << -1 << " " << -1 << " " << featureSelectionTime << " " << std::endl;

    // log performance of tracker (currently we only log info at keyframes!!)
    outputFile_statsTracker << vio->getCurrKfId() << " " <<
        -1 << " " <<
        -1 << " " << -1 << " " << -1 << " " << -1<< " " << -1<< " " << -1 << " " <<
        -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1 << " " << -1  << std::endl;

    // statistics about factors added to the graph
    outputFile_statsFactors << vio->getCurrKfId() << " " <<
        debug_info.numAddedSmartF_ << " " <<
        debug_info.numAddedImuF_ << " " <<
        debug_info.numAddedNoMotionF_ << " " <<
        debug_info.numAddedConstantVelF_ << std::endl;

    std::cout << "data written to file" << std::endl;
    //////////////////////////////////////////////////////////////////////////
    W_Pose_Bprevkf_vio = vio->getWPoseBLkf();
    timestamp_lkf = timestamp_k;
  }
  outputFile.close();
  outputFile_posesVIO.close();
  outputFile_posesGT.close();
  outputFile_smartFactors.close();
  outputFile_timingVIO.close();
  outputFile_timingTracker.close();
  outputFile_statsTracker.close();
  outputFile_statsFactors.close();
  return 0;
}
