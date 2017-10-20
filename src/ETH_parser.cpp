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

using namespace VIO;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS CameraImageLists              ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* --------------------------------------------------------------------------------------- */
bool CameraImageLists::parseCameraImageList(const std::string folderpath, const std::string filename)
{
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
    long long timestamp = std::stoll(item.substr(0, idx));
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
#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "------------ CameraImageLists::print -------------" << std::endl;
  std::cout << "image_folder_path: " << image_folder_path_ << std::endl;
  std::cout << "img_lists size: " << img_lists.size() << std::endl;
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ETHDatasetParser              ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* --------------------------------------------------------------------------------------- */
void GroundTruthData::print()
{
#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "------------ GroundTruthData::print -------------" << std::endl;
  body_Pose_cam_.print("body_Pose_cam_: \n");
  std::cout << "\n gt_rate: " << gt_rate_ << std::endl;
  std::cout << "nr of gtStates: " << mapToGt_.size() << std::endl;
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ImuData                       ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* --------------------------------------------------------------------------------------- */
void ImuData::print()
{
#ifdef ETH_PARSER_DEBUG_COUT
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
#endif
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////// FUNCTIONS OF THE CLASS ETHDatasetParser              ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseImuData(const std::string input_dataset_path, const std::string imuName)
{
  ///////////////// PARSE SENSOR FILE ////////////////////////////////////////////////////////
  std::string filename_sensor = input_dataset_path + "/mav0/" + imuName + "/sensor.yaml";
  // parse sensor parameters
  // make sure that each YAML file has %YAML:1.0 as first line
  cv::FileStorage fs(filename_sensor, cv::FileStorage::READ);
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

  double deltaCount = 0.0;
  long long sumOfDelta = 0;
  double stdDelta = 0;
  double imu_rate_maxMismatch = 0;
  double maxNormAcc = 0, maxNormRotRate = 0; // only for debugging
  long long int previous_timestamp = -1;
  // read/store imu measurements, line by line
  while (std::getline(fin, line)) {
    long long timestamp;
    gtsam::Vector6 gyroAccData;
    for(size_t i=0; i < 7; i++){
      int idx = line.find_first_of(',');
      if (i==0)
        timestamp = std::stoll(line.substr(0, idx));
      else
        gyroAccData(i-1) = std::stod(line.substr(0, idx));
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
      deltaCount += 1.0;
      previous_timestamp = timestamp;
    }
  }
  if(deltaCount != imuData_.imu_buffer_.size()-1){
    std::cout << "parseImuData: wrong nr of deltaCount: deltaCount " << deltaCount << " nr lines " << imuData_.imu_buffer_.size() << std::endl;
    throw std::runtime_error("parseImuData: wrong nr of deltaCount");
  }
  imuData_.imu_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9; // converted to seconds
  imuData_.imu_rate_std_ = sqrt( stdDelta / double(deltaCount-1) );
  imuData_.imu_rate_maxMismatch_ = imu_rate_maxMismatch;
  fin.close();

#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "Maximum measured rotation rate (norm):" << maxNormRotRate << " - Maximum measured acceleration (norm): " << maxNormAcc << std::endl;
#endif
  return true;
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseGTdata(const std::string input_dataset_path, const std::string gtSensorName)
{
  ///////////////// PARSE SENSOR FILE ////////////////////////////////////////////////////////
  std::string filename_sensor = input_dataset_path + "/mav0/" + gtSensorName + "/sensor.yaml";

  // make sure that each YAML file has %YAML:1.0 as first line
  cv::FileStorage fs(filename_sensor, cv::FileStorage::READ);
  if (!fs.isOpened()) {
    std::cout << "Cannot open file in parseGTYAML: " << filename_sensor << std::endl;
    throw std::runtime_error("parseGTdata: cannot open file");
  }

  // body_Pose_cam_: usually this is the identity matrix as the GT "sensor" is at the body frame
  int n_rows = (int)fs["T_BS"]["rows"];
  int n_cols = (int)fs["T_BS"]["cols"];

  std::vector<double> vec;
  fs["T_BS"]["data"] >> vec;
  gtData_.body_Pose_cam_ = UtilsOpenCV::Vec2pose(vec,n_rows,n_cols);

  // sanity check: usually this is the identity matrix as the GT "sensor" is at the body frame
  gtsam::Pose3 identityPose = gtsam::Pose3();

  if(!gtData_.body_Pose_cam_.equals(gtData_.body_Pose_cam_))
    throw std::runtime_error("parseGTdata: we expected identity body_Pose_cam_: is everything ok?");

  fs.release();

  ///////////////// PARSE ACTUAL DATA ////////////////////////////////////////////////////////
  //#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m],
  //q_RS_w [], q_RS_x [], q_RS_y [], q_RS_z [],
  //v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1],
  //b_w_RS_S_x [rad s^-1], b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1],
  //b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], b_a_RS_S_z [m s^-2]
  std::string filename_data = input_dataset_path + "/mav0/" + gtSensorName + "/data.csv";
  std::ifstream fin(filename_data.c_str());
  if (!fin.is_open()) {
    std::cout << "Cannot open file: " << filename_data << std::endl;
    throw std::runtime_error("parseGTdata: cannot open file?");
  }
  // skip the first line, containing the header
  std::string line;
  std::getline(fin, line);

  double deltaCount = 0.0;
  long long sumOfDelta = 0;
  long long int previous_timestamp = -1;
  // read/store gt, line by line
  double maxGTvel = 0;
  while (std::getline(fin, line)) {
    long long timestamp;
    std::vector<double> gtDataRaw;
    for(size_t i=0; i < 17; i++){
      int idx = line.find_first_of(',');
      if (i==0)
        timestamp = std::stoll(line.substr(0, idx));
      else
        gtDataRaw.push_back(std::stod(line.substr(0, idx)));
      line = line.substr(idx+1);
    }
    if(previous_timestamp == -1){
      previous_timestamp = timestamp; // do nothing
    }else{
      sumOfDelta += (timestamp - previous_timestamp);
      // std::cout << "time diff (sec): " << (timestamp - previous_timestamp) * 1e-9 << std::endl;
      deltaCount += 1.0;
      previous_timestamp = timestamp;
    }

    gtNavState gt_curr;
    gtsam::Point3 position(gtDataRaw[0],gtDataRaw[1],gtDataRaw[2]);
    gtsam::Rot3 rot = gtsam::Rot3::Quaternion(gtDataRaw[3],gtDataRaw[4],gtDataRaw[5],gtDataRaw[6]); // quaternion w x y z

    // sanity check
    gtsam::Vector q = rot.quaternion();
    if(fabs(q(0) + gtDataRaw[3]) < fabs(q(0) - gtDataRaw[3])) // figure out sign for quaternion
      q = -q;

    if( (fabs(q(0) - gtDataRaw[3]) > 1e-3) || (fabs(q(1) - gtDataRaw[4]) > 1e-3) ||
        (fabs(q(2) - gtDataRaw[5]) > 1e-3)  || (fabs(q(3) - gtDataRaw[6]) > 1e-3) ){
      std::cout << "(" << q(0)  << "," <<  gtDataRaw[3] << ") " <<
          "(" << q(1)  << "," <<  gtDataRaw[4] << ") "
          "(" << q(2)  << "," <<  gtDataRaw[5] << ") "
          "(" << q(3)  << "," <<  gtDataRaw[6] << ") " << std::endl;
      throw std::runtime_error("parseGTdata: wrong quaternion conversion");
    }
    gt_curr.pose = gtsam::Pose3(rot, position).compose(gtData_.body_Pose_cam_);
    gt_curr.velocity = gtsam::Vector3(gtDataRaw[7],gtDataRaw[8],gtDataRaw[9]);
    gtsam::Vector3 gyroBias = gtsam::Vector3(gtDataRaw[10],gtDataRaw[11],gtDataRaw[12]);
    gtsam::Vector3 accBias = gtsam::Vector3(gtDataRaw[13],gtDataRaw[14],gtDataRaw[15]);
    gt_curr.imuBias = gtsam::imuBias::ConstantBias(accBias, gyroBias);

    gtData_.mapToGt_.insert(std::pair<long long, gtNavState>(timestamp,gt_curr));

    double normVel = gt_curr.velocity.norm();
    if(normVel > maxGTvel)
      maxGTvel = normVel;
  }
  if(deltaCount != gtData_.mapToGt_.size()-1){
    std::cout << "parseGTdata: wrong nr of deltaCount: deltaCount " << deltaCount << " nrPoses " << gtData_.mapToGt_.size() << std::endl;
    throw std::runtime_error("parseGTdata: wrong nr of deltaCount");
  }
  gtData_.gt_rate_ = (double(sumOfDelta) / double(deltaCount)) * 1e-9; // converted in seconds
  fin.close();

#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "Maximum ground truth velocity: " << maxGTvel << std::endl;
#endif
  return true;
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseDataset(const std::string input_dataset_path,
    const std::string leftCameraName, const std::string rightCameraName,
    const std::string imuName, const std::string gtSensorName, const bool doParseImages)
{
  dataset_path_ = input_dataset_path;
  parseCameraData(dataset_path_,leftCameraName,rightCameraName,doParseImages);
  parseImuData(dataset_path_,imuName);
  parseGTdata(dataset_path_,gtSensorName);

  // find and store actual name (rather than path) of the dataset
  std::size_t foundLastSlash = dataset_path_.find_last_of("/\\");
  std::string dataset_path_tmp = dataset_path_;
  dataset_name_ = dataset_path_tmp.substr(foundLastSlash+1);
  if (foundLastSlash >= dataset_path_tmp.size() - 1){ // the dataset name has a slash at the very end
    dataset_path_tmp = dataset_path_tmp.substr(0,foundLastSlash); // cut the last slash
    foundLastSlash = dataset_path_tmp.find_last_of("/\\"); // repeat the search
    dataset_name_ = dataset_path_tmp.substr(foundLastSlash+1); // try to pick right name
  }
#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "dataset_name " << dataset_name_ << std::endl;
#endif

  return true;
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::parseCameraData(const std::string input_dataset_path,
    const std::string leftCameraName, const std::string rightCameraName,
    const bool doParseImages)
{
  // default names: match names of the corresponding folders
  camera_names.resize(2);
  camera_names[0] = leftCameraName;
  camera_names[1] = rightCameraName;

  // read camera info and list of images
  camera_info.clear();
  camera_image_lists.clear();
  for (int i = 0; i < camera_names.size(); i++) { // for each of the 2 cameras
#ifdef ETH_PARSER_DEBUG_COUT
    std::cout << "reading camera: " << i <<std::endl;
#endif
    CameraParams cam_info_i;
    cam_info_i.parseYAML(input_dataset_path + "/mav0/" +
        camera_names[i] + "/sensor.yaml");
    camera_info[camera_names[i]] = cam_info_i;

    CameraImageLists cam_list_i;
    if(doParseImages){
      cam_list_i.parseCameraImageList(input_dataset_path + "/mav0/" +
          camera_names[i], "data.csv");
    }
    camera_image_lists[camera_names[i]] = cam_list_i;
  }

  // SANITY CHECK: nr images is the same for left and right camera
  if(camera_image_lists[leftCameraName].img_lists.size() != camera_image_lists[rightCameraName].img_lists.size()){
#ifdef ETH_PARSER_DEBUG_COUT
    std::cout << "\n\n\n\n WARNING: parseCameraData: different number of images in left and right camera!!" << std::endl;
    std::cout << "Left: " << camera_image_lists[leftCameraName].img_lists.size() << " Right: " <<
        camera_image_lists[rightCameraName].img_lists.size() << "\n\n\n\n" << std::endl;
#endif
    size_t nrCommonImages = std::min(camera_image_lists[leftCameraName].img_lists.size(),camera_image_lists[rightCameraName].img_lists.size());
    camera_image_lists[leftCameraName].img_lists.resize(nrCommonImages);
    camera_image_lists[rightCameraName].img_lists.resize(nrCommonImages);
    //  throw std::runtime_error("parseCameraData: different number of images in left and right camera!!");
  }

  // SANITY CHECK: time stamps are the same for left and right camera
  double stdDelta = 0;
  double frame_rate_maxMismatch = 0;
  int deltaCount = 0;
  for(size_t i=0; i<camera_image_lists[leftCameraName].img_lists.size(); i++)
  {
    if(i>0){
      deltaCount++;
      Timestamp timestamp = camera_image_lists[leftCameraName].img_lists[i].first;
      Timestamp previous_timestamp = camera_image_lists[leftCameraName].img_lists[i-1].first;
      double deltaMismatch = fabs( double(timestamp - previous_timestamp - camera_info[leftCameraName].frame_rate_) * 1e-9 );
      stdDelta += pow( deltaMismatch , 2);
      frame_rate_maxMismatch = std::max(frame_rate_maxMismatch, deltaMismatch);
    }

    if(camera_image_lists[leftCameraName].img_lists[i].first !=
        camera_image_lists[rightCameraName].img_lists[i].first){

      std::cout << "left: " << camera_image_lists[leftCameraName].img_lists[i].first <<
          " right: " << camera_image_lists[rightCameraName].img_lists[i].first << " for image " << i << " of " <<
          camera_image_lists[leftCameraName].img_lists.size() <<  std::endl;
      throw std::runtime_error("parseCameraData: different timestamp for left and right image!!");
    }
  }
#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "nominal frame rate: " << camera_info[leftCameraName].frame_rate_ << std::endl;
  std::cout << "frame rate std: " << sqrt( stdDelta / double(deltaCount-1) ) << std::endl;
  std::cout << "frame rate maxMismatch: " << frame_rate_maxMismatch << std::endl;
#endif

  // Set extrinsic for the sterep
  CameraParams& left_camera_info = camera_info[camera_names[0]];
  CameraParams& right_camera_info = camera_info[camera_names[1]];

  // Extrinsics of the stereo (not rectified)
  camL_Pose_calR = (left_camera_info.body_Pose_cam_).between(right_camera_info.body_Pose_cam_); // relative pose between cameras
  return true;
}

/* --------------------------------------------------------------------------------------- */
gtsam::Pose3 ETHDatasetParser::getGroundTruthRelativePose(const long long previousTimestamp,
    const long long currentTimestamp) const
{
  gtsam::Pose3 previousPose = getGroundTruthPose(previousTimestamp);
  gtsam::Pose3 currentPose = getGroundTruthPose(currentTimestamp);
  return previousPose.between(currentPose);
}

/* --------------------------------------------------------------------------------------- */
bool ETHDatasetParser::isGroundTruthAvailable(const long long timestamp) const
{
  auto it_begin = gtData_.mapToGt_.begin();
  return timestamp > it_begin->first;
}

/* --------------------------------------------------------------------------------------- */
gtNavState ETHDatasetParser::getGroundTruthState(const long long timestamp) const
{
  auto it_low_up = gtData_.mapToGt_.equal_range(timestamp);
  auto it_low = it_low_up.first; // closest, non-lesser

  // sanity check
  double delta_low = double(it_low->first - timestamp) * 1e-9;
  auto it_begin = gtData_.mapToGt_.begin();
  if(timestamp > it_begin->first && delta_low > 0.01){
    std::cout << "\n getGroundTruthState: something wrong " << delta_low  << std::endl;
    throw std::runtime_error("getGroundTruthState: something wrong");
  }
  return it_low->second;
}

/* --------------------------------------------------------------------------------------- */
std::pair<double,double> ETHDatasetParser::computePoseErrors(const gtsam::Pose3 lkf_T_k_body, const bool isTrackingValid,
    long long previousTimestamp, long long currentTimestamp, const bool upToScale) const
{
  double relativeRotError = -1.0;
  double relativeTranError = -1.0;
  if(isTrackingValid && isGroundTruthAvailable(previousTimestamp)){
    // lkf_T_k_body.print("\n lkf_T_k_body \n");
    gtsam::Pose3 lkf_T_k_gt = getGroundTruthRelativePose(previousTimestamp,currentTimestamp);
    // lkf_T_k_gt.print("\n lkf_T_k_gt \n");

    // compute errors
    std::tie(relativeRotError,relativeTranError) = UtilsOpenCV::ComputeRotationAndTranslationErrors(
        lkf_T_k_gt, lkf_T_k_body, upToScale); // true = translation comparison is up to scale
  }
  return std::make_pair(relativeRotError,relativeTranError);
}

/* --------------------------------------------------------------------------------------- */
void ETHDatasetParser::print()
{
#ifdef ETH_PARSER_DEBUG_COUT
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "------------------ ETHDatasetParser::print ---------------------------------------------------------------------------------" << std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "Displaying info for dataset: " << dataset_path_ << std::endl;
  camL_Pose_calR.print("camL_Pose_calR \n");
  std::cout << std::endl;
  for (int i = 0; i < camera_names.size(); i++) { // for each of the 2 cameras
    std::string leftOrRight;
    if(i==0)
      leftOrRight = "Left";
    else
      leftOrRight = "Right";

    std::cout << "\n" << leftOrRight << " camera name: " << camera_names[i] << ", with params: " << std::endl;
    camera_info[camera_names[i]].print();
    camera_image_lists[camera_names[i]].print();
  }
  gtData_.print();
  imuData_.print();
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
  std::cout << "----------------------------------------------------------------------------------------------------------------------------"<< std::endl;
#endif
}
