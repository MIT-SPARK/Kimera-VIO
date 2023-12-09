#include "kimera-vio/utils/FilesystemUtils.h"

#include <glog/logging.h>
#include <sys/stat.h>
#include <sys/types.h>

namespace VIO {
namespace common {

std::string pathAppend(const std::string& p1, const std::string& p2) {
  char sep = '/';
  std::string tmp = p1;
  if (p1[p1.length()] != sep) {  // Need to add a
    tmp += sep;                  // path separator
    return (tmp + p2);
  } else {
    return (p1 + p2);
  }
}

bool createDirectory(const std::string& dir_path,
                     const std::string& dir_name,
                     bool override_if_exists) {
  std::string path = pathAppend(dir_path, dir_name);
  // Give Read/write/search permissions for owner.
  int status = mkdir(path.c_str(), S_IRWXU);
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
