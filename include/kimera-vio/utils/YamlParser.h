/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   YamlParser.h
 * @brief  Base class for YAML parsers.
 * @author Antoni Rosinol
 */

#pragma once

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <glog/logging.h>

#include <opencv2/core/core.hpp>

#include "kimera-vio/utils/Macros.h"

namespace VIO {

class YamlParser {
 public:
  KIMERA_POINTER_TYPEDEFS(YamlParser);

  // Don't make explicit, bcs strings get truncated...
  YamlParser(const std::string& filepath)
      : fs_(), filepath_(filepath) {
    openFile(filepath, &fs_);
  }
  virtual ~YamlParser() { closeFile(&fs_); }

  /**
   * @brief getYamlParam reads a yaml parameter, but breaks if the parameter
   * is not found. This is useful for required parameters.
   * @param id String identifying the parameter in the yaml file.
   * @param output Returned object read from the yaml file.
   */
  template <class T>
  void getYamlParam(const std::string& id, T* output) const {
    CHECK(!id.empty());
    const cv::FileNode& file_handle = fs_[id];
    CHECK_NE(file_handle.type(), cv::FileNode::NONE)
        << "Missing parameter: " << id.c_str()
        << " in file: " << filepath_.c_str();
    file_handle >> *CHECK_NOTNULL(output);
  }

  /**
   * @brief getOptionalYamlParam reads a yaml parameter only if it could be
   * found.
   * @param id String identifying the parameter in the yaml file.
   * @param output Returned object read from the yaml file.
   * @return True if the parameter could be found, false otherwise.
   */
  template <class T>
  bool getOptionalYamlParam(const std::string& id, T* output) const {
    CHECK(!id.empty());
    const cv::FileNode& file_handle = fs_[id];
    if (file_handle.type() == cv::FileNode::NONE) {
      VLOG(5) << "Missing optional parameter: " << id.c_str()
              << " in file: " << filepath_.c_str();
      return false;
    } else {
      file_handle >> *CHECK_NOTNULL(output);
      return true;
    }
  }

  template <class T>
  void getNestedYamlParam(const std::string& id,
                          const std::string& id_2,
                          T* output) const {
    CHECK(!id.empty());
    CHECK(!id_2.empty());
    const cv::FileNode& file_handle = fs_[id];
    CHECK_NE(file_handle.type(), cv::FileNode::NONE)
        << "Missing parameter: " << id.c_str()
        << " in file: " << filepath_.c_str();
    const cv::FileNode& file_handle_2 = file_handle[id_2];
    CHECK_NE(file_handle_2.type(), cv::FileNode::NONE)
        << "Missing nested parameter: " << id_2.c_str() << " inside "
        << id.c_str() << '\n'
        << " in file: " << filepath_.c_str();
    CHECK(file_handle.isMap())
        << "I think that if this is not a map, we can't use >>";
    file_handle_2 >> *CHECK_NOTNULL(output);
  }

 private:
  void openFile(const std::string& filepath, cv::FileStorage* fs) const {
    CHECK(!filepath.empty()) << "Empty filepath!";
    try {
      CHECK_NOTNULL(fs)->open(filepath, cv::FileStorage::READ);
    } catch (cv::Exception& e) {
      LOG(FATAL) << "Cannot open file: " << filepath << '\n'
                 << "OpenCV error code: " << e.msg;
    }
    LOG_IF(FATAL, !fs->isOpened())
        << "Cannot open file in parseYAML: " << filepath
        << " (remember that the first line should be: %YAML:1.0)";
  }

  inline void closeFile(cv::FileStorage* fs) const {
    CHECK_NOTNULL(fs)->release();
  }

 private:
  cv::FileStorage fs_;
  std::string filepath_;
};

}  // namespace VIO
