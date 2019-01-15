/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   ETH_parser.h
 * @brief  Reads ETH's Euroc dataset
 * @author Luca Carlone
 */

#include "ETH_parser.h"

#include <gflags/gflags.h>

DEFINE_string(dataset_path, "/Users/Luca/data/MH_01_easy",
              "Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).");
DEFINE_string(vio_params_path, "",
              "Path to vio user-defined parameters.");
DEFINE_string(tracker_params_path, "",
              "Path to tracker user-defined parameters.");
DEFINE_int32(initial_k, 50, "Initial frame to start processing dataset, "
                            "previous frames will not be used.");
DEFINE_int32(final_k, 10000, "Final frame to finish processing dataset, "
                            "subsequent frames will not be used.");

using namespace VIO;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS CameraImageLists              ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* --------------------------------------------------------------------------------------- */
bool CameraImageLists::parseCamImgList(const std::string& folderpath,
                                       const std::string& filename) {
  image_folder_path_ = folderpath; // stored, only for debug
  const std::string fullname = folderpath + "/" + filename;
  std::ifstream fin(fullname.c_str());
  if (!fin.is_open()) {
    std::cout << "Cannot open file: " << fullname << std::endl;
    throw std::runtime_error("parseCameraImageList: cannot open file");
  }
  // skip the first line, containing the header
  std::string item;
  std::getline(fin, item);

  // read/store list of image names
  while (std::getline(fin, item)) {
    // print the item!
    int idx = item.find_first_of(',');
    Timestamp timestamp = std::stoll(item.substr(0, idx));
    std::string imageFilename = folderpath + "/data/" + item.substr(0, idx) + ".png";
    // strangely, on mac, it does not work if we use: item.substr(idx + 1);
    // maybe different string termination characters???
    img_lists.push_back(make_pair(timestamp, imageFilename));
  }
  fin.close();
  return true;
}

/* --------------------------------------------------------------------------------------- */
void CameraImageLists::print()
{
  std::cout << "------------ CameraImageLists::print -------------" << std::endl;
  std::cout << "image_folder_path: " << image_folder_path_ << std::endl;
  std::cout << "img_lists size: " << img_lists.size() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ETHDatasetParser              //////////
////////////////////////////////////////////////////////////////////////////////
/* -------------------------------------------------------------------------- */
void GroundTruthData::print() {
  std::cout << "------------ GroundTruthData::print -------------" << std::endl;
  body_Pose_cam_.print("body_Pose_cam_: \n");
  std::cout << "\n gt_rate: " << gt_rate_ << std::endl;
  std::cout << "nr of gtStates: " << mapToGt_.size() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ImuData                       //////////
////////////////////////////////////////////////////////////////////////////////
/* -------------------------------------------------------------------------- */
void ImuData::print()
{
  std::cout << "------------ ImuData::print    -------------" << std::endl;
  body_Pose_cam_.print("body_Pose_cam_: \n");
  std::cout << "\n nominal_imu_rate: " << nominal_imu_rate_ << std::endl;
  std::cout << "imu_rate: " << imu_rate_ << std::endl;
  std::cout << "imu_rate_std: " << imu_rate_std_ << std::endl;
  std::cout << "imu_rate_maxMismatch: " << imu_rate_maxMismatch_ << std::endl;
  std::cout << "gyroscope_noise_density: " << gyro_noise_ << std::endl;
  std::cout << "gyroscope_random_walk: " << gyro_walk_ << std::endl;
  std::cout << "accelerometer_noise_density: " << acc_noise_ << std::endl;
  std::cout << "accelerometer_random_walk: " << acc_walk_ << std::endl;
  std::cout << "nr of imu measurements: " << imu_buffer_.size() << std::endl;
}

////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ETHDatasetParser              //////////
////////////////////////////////////////////////////////////////////////////////
/* -------------------------------------------------------------------------- */
void ETHDatasetParser::parse(size_t* initial_k, size_t* final_k) {
  CHECK_NOTNULL(initial_k);
  CHECK_NOTNULL(final_k);

  VLOG(100) << "Using dataset path: " << FLAGS_dataset_path;
  // Parse the dataset (ETH format).
  static const std::string leftCameraName = "cam0";
  static const std::string rightCameraName = "cam1";
  static const std::string imuName = "imu0";
  static const std::string gtSensorName = "state_groundtruth_estimate0";
  parseDataset(FLAGS_dataset_path,
               leftCameraName, rightCameraName,
               imuName, gtSensorName);
  print();

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

  const size_t& nr_images = nrImages();
  if (*final_k > nr_images) {
    LOG(WARNING) << "Value for final_k, " << *final_k << " is larger than total"
                 << " number of frames in dataset " << nr_images;
    // Skip last frames which are typically problematic
    // (IMU bumps, FOV occluded)...
    static constexpr size_t skip_n_end_frames = 100;
    *final_k = nr_images - skip_n_end_frames;
    LOG(WARNING) << "Using final_k = " << *final_k << ", where we removed "
                 << skip_n_end_frames << " frames to avoid bad IMU readings.";
  }
  CHECK(*final_k > *initial_k)
      << "Value for final_k (" << *final_k << ") is smaller than value for"
      << " initial_k (" << *initial_k << ").";

  LOG(INFO) << "Running dataset between frame " << *initial_k
            << " and frame " <<  *final_k;
}

void ETHDatasetParser::parseParams(
        VioBackEndParamsPtr vioParams,
        const ImuData& imu_data,
        VioFrontEndParams* trackerParams) {
  CHECK(vioParams);
  CHECK_NOTNULL(trackerParams);

  // Read/define vio params.
  if (FLAGS_vio_params_path.empty()) {
    VLOG(100) << "No vio parameters specified, using default.";
    // Default params with IMU stats from dataset.
    vioParams->gyroNoiseDensity_ = imu_data.gyro_noise_;
    vioParams->accNoiseDensity_ = imu_data.acc_noise_;
    vioParams->gyroBiasSigma_ = imu_data.acc_noise_;
    vioParams->accBiasSigma_ = imu_data.acc_walk_;
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
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseImuData(const std::string& input_dataset_path,
                                    const std::string& imuName) {
  ///////////////// PARSE SENSOR FILE ////////////////////////////////////////////////////////
  std::string filename_sensor = input_dataset_path + "/mav0/" + imuName + "/sensor.yaml";
  // parse sensor parameters
  // make sure that each YAML file has %YAML:1.0 as first line
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filename_sensor);
  if (!fs.isOpened()) {
    std::cout << "Cannot open file in parseImuData: " << filename_sensor << std::endl;
    throw std::runtime_error("parseImuData: cannot open file");
  }

  // body_Pose_cam_: sensor to body transformation
  int n_rows = (int)fs["T_BS"]["rows"];
  int n_cols = (int)fs["T_BS"]["cols"];
  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  imuData_.body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec,n_rows,n_cols);

  // sanity check: IMU is usually chosen as the body frame
  gtsam::Pose3 identityPose = gtsam::Pose3();
  if(!imuData_.body_Pose_cam_.equals(gtData_.body_Pose_cam_))
    throw std::runtime_error("parseImuData: we expected identity body_Pose_cam_: is everything ok?");

  int rate = fs["rate_hz"];
  imuData_.nominal_imu_rate_ = 1 / double(rate);
  imuData_.gyro_noise_ = fs["gyroscope_noise_density"];
  imuData_.gyro_walk_ = fs["gyroscope_random_walk"];
  imuData_.acc_noise_ = fs["accelerometer_noise_density"];
  imuData_.acc_walk_ = fs["accelerometer_random_walk"];

  fs.release();

  ///////////////// PARSE ACTUAL DATA ////////////////////////////////////////////////////////
  //#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],
  // a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
  std::string filename_data = input_dataset_path + "/mav0/" + imuName + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  if (!fin.is_open()) {
    std::cout << "Cannot open file: " << filename_data << std::endl;
    throw std::runtime_error("parseImuData: cannot open file");
  }
  // skip the first line, containing the header
  std::string line;
  std::getline(fin, line);

  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  double stdDelta = 0;
  double imu_rate_maxMismatch = 0;
  double maxNormAcc = 0, maxNormRotRate = 0; // only for debugging
  Timestamp previous_timestamp = -1;
  // read/store imu measurements, line by line
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    gtsam::Vector6 gyroAccData;
    for(size_t i=0; i < 7; i++){
      int idx = line.find_first_of(',');
      if (i==0) {
        timestamp = std::stoll(line.substr(0, idx));
      } else {
        gyroAccData(i-1) = std::stod(line.substr(0, idx));
      }
      line = line.substr(idx+1);
    }
    Vector6 imu_accgyr;
    imu_accgyr << gyroAccData.tail(3), gyroAccData.head(3); // acceleration first!

    double normAcc = gyroAccData.tail(3).norm();
    if(normAcc > maxNormAcc)
      maxNormAcc = normAcc;

    double normRotRate = gyroAccData.head(3).norm();
    if(normRotRate > maxNormRotRate)
      maxNormRotRate = normRotRate;

    imuData_.imu_buffer_.insert(timestamp, imu_accgyr);
    if(previous_timestamp == -1){
      previous_timestamp = timestamp; // do nothing
    }else{
      sumOfDelta += (timestamp - previous_timestamp);
      double deltaMismatch = fabs( double(timestamp - previous_timestamp - imuData_.nominal_imu_rate_) * 1e-9 );
      stdDelta += pow( deltaMismatch , 2);
      imu_rate_maxMismatch = std::max(imu_rate_maxMismatch, deltaMismatch);
      deltaCount += 1u;
      previous_timestamp = timestamp;
    }
  }
  if (deltaCount != imuData_.imu_buffer_.size() - 1u) {
    std::cout << "parseImuData: wrong nr of deltaCount: deltaCount "
              << deltaCount << " nr lines "
              << imuData_.imu_buffer_.size() << std::endl;
    throw std::runtime_error("parseImuData: wrong nr of deltaCount");
  }
  imuData_.imu_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9; // converted to seconds
  imuData_.imu_rate_std_ = sqrt( stdDelta / double(deltaCount-1u) );
  imuData_.imu_rate_maxMismatch_ = imu_rate_maxMismatch;
  fin.close();

  std::cout << "Maximum measured rotation rate (norm):" << maxNormRotRate << " - Maximum measured acceleration (norm): " << maxNormAcc << std::endl;
  return true;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::parseGTdata(const std::string &input_dataset_path,
                                   const std::string &gtSensorName) {
  ///////////////// PARSE SENSOR FILE //////////////////////////////////////////
  std::string filename_sensor =
      input_dataset_path + "/mav0/" + gtSensorName + "/sensor.yaml";

  // Make sure that each YAML file has %YAML:1.0 as first line.
  cv::FileStorage fs;
  UtilsOpenCV::safeOpenCVFileStorage(&fs, filename_sensor);
  if (!fs.isOpened()) {
    std::cout << "Cannot open file in parseGTYAML: "
              << filename_sensor << std::endl;
    std::cout << "Assuming dataset has no ground truth...";
    return false;
  }

  // body_Pose_cam_: usually this is the identity matrix as the GT "sensor" is
  // at the body frame
  int n_rows = (int)fs["T_BS"]["rows"];
  int n_cols = (int)fs["T_BS"]["cols"];

  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  gtData_.body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec, n_rows, n_cols);

  // sanity check: usually this is the identity matrix as the GT "sensor"
  // is at the body frame
  gtsam::Pose3 identityPose = gtsam::Pose3();

  if(!gtData_.body_Pose_cam_.equals(identityPose)) {
    throw std::runtime_error("parseGTdata: we expected identity body_Pose_cam_:"
                             " is everything ok?");
  }

  fs.release();

  ///////////////// PARSE ACTUAL DATA //////////////////////////////////////////
  //#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
  //q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
  //v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
  //b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
  //b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]
  std::string filename_data = input_dataset_path + "/mav0/" + gtSensorName
                              + "/data.csv";
  std::ifstream fin (filename_data.c_str());
  if (!fin.is_open()) {
    std::cout << "Cannot open file: " << filename_data << std::endl;
    std::cout << "Assuming dataset has no ground truth...";
    return false;
  }

  // Skip the first line, containing the header.
  std::string line;
  std::getline(fin, line);

  size_t deltaCount = 0u;
  Timestamp sumOfDelta = 0;
  Timestamp previous_timestamp = -1;

  // Read/store gt, line by line.
  double maxGTvel = 0;
  while (std::getline(fin, line)) {
    Timestamp timestamp = 0;
    std::vector<double> gtDataRaw;
    for (size_t i = 0; i < 17; i++) {
      int idx = line.find_first_of(',');
      if (i == 0)
        timestamp = std::stoll(line.substr(0, idx));
      else
        gtDataRaw.push_back(std::stod(line.substr(0, idx)));
      line = line.substr(idx+1);
    }
    if (previous_timestamp == -1) {
      previous_timestamp = timestamp; // do nothing
    } else {
      sumOfDelta += (timestamp - previous_timestamp);
      // std::cout << "time diff (sec): " << (timestamp - previous_timestamp) * 1e-9 << std::endl;
      deltaCount += 1u;
      previous_timestamp = timestamp;
    }

    gtNavState gt_curr;
    gtsam::Point3 position(gtDataRaw[0], gtDataRaw[1], gtDataRaw[2]);
    // Quaternion w x y z.
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(
                        gtDataRaw[3], gtDataRaw[4],gtDataRaw[5],gtDataRaw[6]);

    // Sanity check.
    gtsam::Vector q = rot.quaternion();
    if(fabs(q(0) + gtDataRaw[3]) < fabs(q(0) - gtDataRaw[3])) {// figure out sign for quaternion
      q = -q;
    }

    if ((fabs(q(0) - gtDataRaw[3]) > 1e-3) ||
        (fabs(q(1) - gtDataRaw[4]) > 1e-3) ||
        (fabs(q(2) - gtDataRaw[5]) > 1e-3) ||
        (fabs(q(3) - gtDataRaw[6]) > 1e-3)) {
      std::cout << "(" << q(0)  << "," <<  gtDataRaw[3] << ") " <<
          "(" << q(1)  << "," <<  gtDataRaw[4] << ") "
          "(" << q(2)  << "," <<  gtDataRaw[5] << ") "
          "(" << q(3)  << "," <<  gtDataRaw[6] << ") " << std::endl;
      throw std::runtime_error("parseGTdata: wrong quaternion conversion");
    }

    gt_curr.pose_ = gtsam::Pose3(rot, position).compose(gtData_.body_Pose_cam_);
    gt_curr.velocity_ = gtsam::Vector3(gtDataRaw[7], gtDataRaw[8], gtDataRaw[9]);
    gtsam::Vector3 gyroBias =
        gtsam::Vector3(gtDataRaw[10], gtDataRaw[11], gtDataRaw[12]);
    gtsam::Vector3 accBias =
        gtsam::Vector3(gtDataRaw[13], gtDataRaw[14], gtDataRaw[15]);
    gt_curr.imu_bias_ = gtsam::imuBias::ConstantBias(accBias, gyroBias);

    gtData_.mapToGt_.insert(
          std::pair<Timestamp, gtNavState>(timestamp,gt_curr));

    double normVel = gt_curr.velocity_.norm();
    if (normVel > maxGTvel) maxGTvel = normVel;
  }
  if (deltaCount != gtData_.mapToGt_.size() - 1u) {
    std::cout << "parseGTdata: wrong nr of deltaCount: deltaCount "
              << deltaCount << " nrPoses " << gtData_.mapToGt_.size() << '\n';
    throw std::runtime_error("parseGTdata: wrong nr of deltaCount");
  }
  CHECK_NE(deltaCount, 0u);
  gtData_.gt_rate_ =
      (double(sumOfDelta) / double(deltaCount)) * 1e-9; // Converted in seconds.
  fin.close();

  std::cout << "Maximum ground truth velocity: " << maxGTvel << std::endl;
  return true;
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseDataset(const std::string& input_dataset_path,
                                    const std::string& leftCameraName,
                                    const std::string& rightCameraName,
                                    const std::string& imuName,
                                    const std::string& gtSensorName,
                                    bool doParseImages) {
  dataset_path_ = input_dataset_path;
  parseCameraData(dataset_path_, leftCameraName, rightCameraName, doParseImages);
  parseImuData(dataset_path_, imuName);
  is_gt_available_ = parseGTdata(dataset_path_, gtSensorName);

  // find and store actual name (rather than path) of the dataset
  std::size_t foundLastSlash = dataset_path_.find_last_of("/\\");
  std::string dataset_path_tmp = dataset_path_;
  dataset_name_ = dataset_path_tmp.substr(foundLastSlash+1);
  if (foundLastSlash >= dataset_path_tmp.size() - 1){ // the dataset name has a slash at the very end
    dataset_path_tmp = dataset_path_tmp.substr(0,foundLastSlash); // cut the last slash
    foundLastSlash = dataset_path_tmp.find_last_of("/\\"); // repeat the search
    dataset_name_ = dataset_path_tmp.substr(foundLastSlash+1); // try to pick right name
  }
  std::cout << "dataset_name " << dataset_name_ << std::endl;

  return true;
}

bool ETHDatasetParser::sanityCheckCameraData(
    const std::vector<std::string>& camera_names,
    const std::map<std::string, CameraParams>& camera_info,
    std::map<std::string, CameraImageLists>* camera_image_lists) const {
  CHECK_NOTNULL(camera_image_lists);
  const auto& left_cam_info = camera_info.at(camera_names.at(0));
  auto& left_img_lists = camera_image_lists->at(camera_names.at(0)).img_lists;
  auto& right_img_lists = camera_image_lists->at(camera_names.at(1)).img_lists;
  return sanityCheckCamSize(&left_img_lists, &right_img_lists) &&
      sanityCheckCamTimestamps(left_img_lists, right_img_lists, left_cam_info);
}

bool ETHDatasetParser::sanityCheckCamSize(
    CameraImageLists::ImgLists* left_img_lists,
    CameraImageLists::ImgLists* right_img_lists) const {
  CHECK_NOTNULL(left_img_lists);
  CHECK_NOTNULL(right_img_lists);
  size_t nr_left_cam_imgs = left_img_lists->size();
  size_t nr_right_cam_imgs = right_img_lists->size();
  if (nr_left_cam_imgs!= nr_right_cam_imgs) {
    LOG(WARNING) << "Different number of images in left and right camera!\n"
                 << "Left: " << nr_left_cam_imgs << "\n"
                 << "Right: " << nr_right_cam_imgs;
    size_t nrCommonImages = std::min(nr_left_cam_imgs, nr_right_cam_imgs);
    left_img_lists->resize(nrCommonImages);
    right_img_lists->resize(nrCommonImages);
  }
  return true;
}

bool ETHDatasetParser::sanityCheckCamTimestamps(
  const CameraImageLists::ImgLists& left_img_lists,
  const CameraImageLists::ImgLists& right_img_lists,
  const CameraParams& left_cam_info) const {
  double stdDelta = 0;
  double frame_rate_maxMismatch = 0;
  size_t deltaCount = 0u;
  for (size_t i = 0; i < left_img_lists.size(); i++) {
    if (i > 0) {
      deltaCount++;
      const Timestamp& timestamp = left_img_lists.at(i).first;
      const Timestamp& previous_timestamp = left_img_lists.at(i-1).first;
      double deltaMismatch = fabs(double(timestamp - previous_timestamp - left_cam_info.frame_rate_) * 1e-9 );
      stdDelta += pow( deltaMismatch , 2);
      frame_rate_maxMismatch = std::max(frame_rate_maxMismatch, deltaMismatch);
    }

    if (left_img_lists.at(i).first != right_img_lists.at(i).first) {
        LOG(FATAL) << "Different timestamp for left and right image!\n"
        << "left: " <<  left_img_lists.at(i).first << '\n'
        << "right: " << right_img_lists.at(i).first << '\n'
        << " for image " << i << " of " << left_img_lists.size();
        return false;
    }
  }

  LOG(INFO) << "nominal frame rate: " << left_cam_info.frame_rate_ << '\n'
            << "frame rate std: " << sqrt(stdDelta / double(deltaCount-1u)) << '\n'
            << "frame rate maxMismatch: " << frame_rate_maxMismatch;
  return true;
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseCameraData(
        const std::string& input_dataset_path,
        const std::string& left_cam_name,
        const std::string& right_cam_name,
        const bool parse_imgs) {
  // Default names: match names of the corresponding folders.
  camera_names_.resize(2);
  camera_names_[0] = left_cam_name;
  camera_names_[1] = right_cam_name;

  // Read camera info and list of images.
  camera_info_.clear();
  camera_image_lists_.clear();
  for (const std::string& cam_name: camera_names_) {
    LOG(INFO) << "reading camera: " << cam_name;
    CameraParams cam_info_i;
    cam_info_i.parseYAML(input_dataset_path + "/mav0/" + cam_name + "/sensor.yaml");
    camera_info_[cam_name] = cam_info_i;

    CameraImageLists cam_list_i;
    if (parse_imgs) {
      cam_list_i.parseCamImgList(input_dataset_path + "/mav0/" + cam_name, "data.csv");
    }
    camera_image_lists_[cam_name] = cam_list_i;
  }

  CHECK(sanityCheckCameraData(camera_names_, camera_info_,
                              &camera_image_lists_));

  // Set extrinsic for the stereo.
  const CameraParams& left_camera_info = camera_info_.at(camera_names_.at(0));
  const CameraParams& right_camera_info = camera_info_.at(camera_names_.at(1));

  // Extrinsics of the stereo (not rectified)
  // relative pose between cameras
  camL_Pose_camR_ = (left_camera_info.body_Pose_cam_).between(right_camera_info.body_Pose_cam_);
  return true;
}

/* --------------------------------------------------------------------------------------- */
gtsam::Pose3 ETHDatasetParser::getGroundTruthRelativePose(
    const Timestamp& previousTimestamp,
    const Timestamp& currentTimestamp) const {
  gtsam::Pose3 previousPose = getGroundTruthPose(previousTimestamp);
  gtsam::Pose3 currentPose = getGroundTruthPose(currentTimestamp);
  return previousPose.between(currentPose);
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::isGroundTruthAvailable(const Timestamp& timestamp) const {
  auto it_begin = gtData_.mapToGt_.begin();
  return timestamp > it_begin->first;
}

/* -------------------------------------------------------------------------- */
bool ETHDatasetParser::isGroundTruthAvailable() const {
  return is_gt_available_;
}

/* --------------------------------------------------------------------------------------- */
gtNavState ETHDatasetParser::getGroundTruthState(const Timestamp& timestamp) const {
  auto it_low_up = gtData_.mapToGt_.equal_range(timestamp);
  auto it_low = it_low_up.first; // closest, non-lesser

  // sanity check
  double delta_low = double(it_low->first - timestamp) * 1e-9;
  auto it_begin = gtData_.mapToGt_.begin();
  if (timestamp > it_begin->first && delta_low > 0.01) {
    std::cout << "\n getGroundTruthState: something wrong "
              << delta_low << '\n';
    throw std::runtime_error("getGroundTruthState: something wrong");
  }
  return it_low->second;
}

/* --------------------------------------------------------------------------------------- */
std::pair<double, double> ETHDatasetParser::computePoseErrors(
    const gtsam::Pose3& lkf_T_k_body,
    const bool isTrackingValid,
    const Timestamp& previousTimestamp,
    const Timestamp& currentTimestamp,
    const bool upToScale) const {
  double relativeRotError = -1.0;
  double relativeTranError = -1.0;
  if (isTrackingValid && isGroundTruthAvailable(previousTimestamp)) {
    gtsam::Pose3 lkf_T_k_gt = getGroundTruthRelativePose(previousTimestamp,
                                                         currentTimestamp);
    // Compute errors.
    std::tie(relativeRotError, relativeTranError) =
        UtilsOpenCV::ComputeRotationAndTranslationErrors(
        lkf_T_k_gt, lkf_T_k_body, upToScale);
  }
  return std::make_pair(relativeRotError, relativeTranError);
}

/* -------------------------------------------------------------------------- */
void ETHDatasetParser::print() {
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "------------------ ETHDatasetParser::print ---------------------------------------------------------------------------------" << std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "Displaying info for dataset: " << dataset_path_ << std::endl;
  camL_Pose_camR_.print("camL_Pose_calR \n");
  std::cout << std::endl;
  for (int i = 0; i < camera_names_.size(); i++) { // for each of the 2 cameras
    std::string leftOrRight;
    if(i==0)
      leftOrRight = "Left";
    else
      leftOrRight = "Right";

    std::cout << "\n" << leftOrRight << " camera name: " << camera_names_[i] << ", with params: " << std::endl;
    camera_info_[camera_names_[i]].print();
    camera_image_lists_[camera_names_[i]].print();
  }
  gtData_.print();
  imuData_.print();
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
}
