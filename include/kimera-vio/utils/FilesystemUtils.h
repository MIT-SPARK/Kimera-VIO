#pragma once
#include <string>

namespace VIO {
namespace common {

std::string pathAppend(const std::string& p1, const std::string& p2);

bool createDirectory(const std::string& dir_path,
                     const std::string& dir_name,
                     bool override_if_exists = false);

}  // namespace common
}  // namespace VIO
