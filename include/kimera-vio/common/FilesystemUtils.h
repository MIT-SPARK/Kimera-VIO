#pragma once

#include <direct.h>

#include <glog/logging.h>

namespace VIO {
namespace common {

static std::string pathAppend(const std::string& p1, const std::string& p2) {
  char sep = '/';
  std::string tmp = p1;
  if (p1[p1.length()] != sep) {  // Need to add a
    tmp += sep;                  // path separator
    return (tmp + p2);
  } else {
    return (p1 + p2);
  }
}

static bool createDirectory(const std::string& dir_path,
                            const std::string& dir_name,
                            bool override_if_exists = false) {
  std::string path = pathAppend(dir_path, dir_name);
  // Give Read/write/search permissions for owner.
  int status = _mkdir(path.c_str());
  if (status == 0) {
    LOG(INFO) << "Directory created: " << path;
    return true;
  } else {
    LOG(ERROR) << "Directory creation failed for: " << path << '\n'
               << "Error code: " << strerror(errno);
    return false;
  }
}

}  // namespace common
}  // namespace VIO
