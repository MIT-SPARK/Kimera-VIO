/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   KittiDataSource.h
 * @brief  Kitti dataset parser.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>
#include <functional>
#include "datasource/DataSource.h"
#include "StereoImuSyncPacket.h"

namespace VIO {

class KittiDataProvider: public DataProvider {
public:
  KittiDataProvider(const std::string& kitti_dataset_path);
  virtual ~KittiDataProvider();
  virtual bool spin();

private:
  struct KittiData {
    inline size_t getNumberOfImages() const {return left_img_names_.size();}
    std::vector<std::string> left_img_names_;
    std::vector<std::string> right_img_names_;
    std::vector<double> timestamps_;
    // Dummy check to ensure data is correctly parsed.
    explicit operator bool() const;
  };

private:
  cv::Mat readKittiImage(const std::string& img_name);
  void parseData(const std::string& kitti_sequence_path,
                 KittiData* kitti_data) const;

  bool parseCameraData(const std::string& input_dataset_path,
                       const std::string& left_cam_name,
                       const std::string& right_cam_name);

private:
  KittiData kitti_data_;
};

} // End of VIO namespace.
