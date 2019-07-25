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

#include <fstream>
#include <iostream>
#include <memory>
#include <stdlib.h>
#include <string>

#include <glog/logging.h>

#include <opencv2/core/core.hpp>

namespace VIO {

class YamlParser {
public:
  bool parse(const std::string &filepath) {
    cv::FileStorage fs;
    openFile(filepath, &fs);
    bool result = parseYAML(fs);
    closeFile(&fs);
    return result;
  }

  virtual ~YamlParser() = default;

protected:
  YamlParser() = default;
  virtual bool parseYAML(const cv::FileStorage &fs) = 0;

  template <class T>
  void getYamlParam(const cv::FileStorage &fs, const std::string &id,
                    T *output) const {
    const cv::FileNode &file_handle = fs[id];
    CHECK_NE(file_handle.type(), cv::FileNode::NONE)
        << "Missing parameter: " << id.c_str();
    file_handle >> *CHECK_NOTNULL(output);
  }

private:
  void openFile(const std::string &filepath, cv::FileStorage *fs) const {
    CHECK_NOTNULL(fs)->open(filepath, cv::FileStorage::READ);
    LOG_IF(FATAL, !fs->isOpened())
        << "Cannot open file in parseYAML: " << filepath
        << " (remember that the first line should be: %YAML:1.0)";
  }

  inline void closeFile(cv::FileStorage *fs) const {
    CHECK_NOTNULL(fs)->release();
  }
};

} // namespace VIO
