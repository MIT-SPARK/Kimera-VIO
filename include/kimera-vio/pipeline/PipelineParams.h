/**
 * @file   PipelinePayload.h
 * @brief  Base class for the payloads shared between pipeline modules.
 * @author Antoni Rosinol
 */

#pragma once

#include <iomanip>
#include <ostream>
#include <sstream>
#include <string>

#include <glog/logging.h>

#include "kimera-vio/frontend/FrontendType.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

/**
 * @brief The PipelineParams base class
 * Sets a common base class for parameters of the pipeline
 * for easy parsing/printing. All parameters in VIO should inherit from
 * this class and implement the print/parseYAML virtual functions.
 */
class PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineParams);
  explicit PipelineParams(const std::string& name);
  virtual ~PipelineParams() = default;

 public:
  // Parameters of the pipeline must specify how to be parsed.
  virtual bool parseYAML(const std::string& filepath) = 0;

  // Parameters of the pipeline must specify how to be printed.
  virtual void print() const = 0;

  // Parameters of the pipeline must specify how to be compard, they need
  // to implement the equals function below.
  friend bool operator==(const PipelineParams& lhs, const PipelineParams& rhs);
  friend bool operator!=(const PipelineParams& lhs, const PipelineParams& rhs);

 protected:
  // Parameters of the pipeline must specify how to be compared.
  virtual bool equals(const PipelineParams& obj) const = 0;

  /** @brief print Helper function to print arguments in a standardized way
   * @param[out] Stream were the printed output will be written. You can then
   * log that to cout or glog, i.e: `LOG(INFO) << out.c_str()`;
   * @param[in] *Even* list of arguments, where we assume the first to be the
   * name
   * and the second a value (both can be of any type (with operator <<)).
   * Note: uneven list of arguments will raise compilation errors.
   * Usage:
   * ```
   * std::stringstream out;
   * int my_first_param = 2;
   * bool my_second_param = 2;
   * print(out, "My first param", my_first_param,
   *  "My second param", my_second_param);
   * std::cout << out.c_str() << std::endl;
   * ```
   */
  template <typename... args>
  void print(std::stringstream& out, args... to_print) const {
    out.str("");  // clear contents.

    // Add title.
    out.width(kTotalWidth);
    size_t center =
        (kTotalWidth - name_.size() - 2u) / 2u;  // -2u for ' ' chars
    out << '\n'
        << std::string(center, '*').c_str() << ' ' << name_.c_str() << ' '
        << std::string(center, '*').c_str() << '\n';

    // Add columns' headers.
    out.width(kNameWidth);  // Remove hardcoded, need to pre-calc width.
    out.setf(std::ios::left, std::ios::adjustfield);
    out << "Name";
    out.setf(std::ios::right, std::ios::adjustfield);
    out.width(kValueWidth);
    out << "Value\n";

    // Add horizontal separator
    out.width(kTotalWidth);  // Remove hardcoded, need to pre-calc width.
    out << std::setfill('-') << "\n";

    // Reset fill to dots
    out << std::setfill('.');
    printImpl(out, to_print...);

    // Add horizontal separator
    out.width(kTotalWidth);  // Remove hardcoded, need to pre-calc width.
    out << std::setfill('-') << "\n";
  }

 public:
  std::string name_;
  static constexpr size_t kTotalWidth = 60;
  static constexpr size_t kNameWidth = 40;
  static constexpr size_t kValueWidth = 20;

 private:
  template <typename TName, typename TValue>
  void printImpl(std::stringstream& out, TName name, TValue value) const {
    out.width(kNameWidth);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << name;
    out.width(kValueWidth);
    out.setf(std::ios::right, std::ios::adjustfield);
    out << value << '\n';
  }

  template <typename TName, typename TValue, typename... Args>
  void printImpl(std::stringstream& out,
                 TName name,
                 TValue value,
                 Args... next) const {
    out.width(kNameWidth);
    out.setf(std::ios::left, std::ios::adjustfield);
    out << name;
    out.width(kValueWidth);
    out.setf(std::ios::right, std::ios::adjustfield);
    out << value << '\n';
    printImpl(out, next...);
  }
};

inline bool operator==(const PipelineParams& lhs, const PipelineParams& rhs) {
  // Allow to compare only instances of the same dynamic type
  return typeid(lhs) == typeid(rhs) && lhs.name_ == rhs.name_ &&
         lhs.equals(rhs);
}
inline bool operator!=(const PipelineParams& lhs, const PipelineParams& rhs) {
  return !(lhs == rhs);
}

template <class T>
inline void parsePipelineParams(const std::string& params_path,
                                T* pipeline_params) {
  CHECK_NOTNULL(pipeline_params);
  static_assert(std::is_base_of<PipelineParams, T>::value,
                "T must be a class that derives from PipelineParams.");
  // Read/define tracker params.
  if (params_path.empty()) {
    LOG(WARNING) << "No " << pipeline_params->name_
                 << " parameters specified, using default.";
    *pipeline_params = T();  // default params
  } else {
    VLOG(100) << "Using user-specified " << pipeline_params->name_
              << " parameters: " << params_path;
    pipeline_params->parseYAML(params_path);
  }
}

}  // namespace VIO
