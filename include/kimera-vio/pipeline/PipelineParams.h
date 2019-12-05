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
 * for easy parsing/printing.
 */
class PipelineParams {
 public:
  KIMERA_POINTER_TYPEDEFS(PipelineParams);
  PipelineParams(const std::string& name);
  virtual ~PipelineParams() = default;

 public:
  // Parameters of the pipeline must specify how to be parsed.
  virtual bool parseYAML(const std::string& filepath) = 0;

  // Parameters of the pipeline must specify how to be printed.
  virtual void print() const = 0;

 public:
  std::string name_;
};

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
