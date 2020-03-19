/**
 * @file   PipelinePayload.h
 * @brief  Base class for the payloads shared between pipeline modules.
 * @author Antoni Rosinol
 */

#pragma once

#include <string>

#include <glog/logging.h>

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

 public:
  std::string name_;
};

inline bool operator==(const PipelineParams& lhs, const PipelineParams& rhs) {
  // Allow to compare only instances of the same dynamic type
  return typeid(lhs) == typeid(rhs) &&
      lhs.name_ == rhs.name_ &&
      lhs.equals(rhs);
}
inline bool operator!=(const PipelineParams& lhs, const PipelineParams& rhs) {
  return !(lhs == rhs);
}

template <class T>
static void parsePipelineParams(const std::string& params_path,
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
