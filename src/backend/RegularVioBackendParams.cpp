/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   RegularVioBackendParams.cpp
 * @brief  Class collecting the parameters of the Visual Inertial odometry
 * pipeline for the RegularVIO implementation.
 * @author Antoni Rosinol
 */

#include "kimera-vio/backend/RegularVioBackendParams.h"

#include <utility>

namespace VIO {

RegularVioBackendParams::RegularVioBackendParams() : BackendParams() {
  // Trivial sanity checks.
  CHECK_GE(monoNoiseSigma_, 0.0);
  CHECK_GE(stereoNoiseSigma_, 0.0);
  CHECK_GE(regularityNoiseSigma_, 0.0);
  CHECK_GE(monoNormType_, 0);
  CHECK_GE(stereoNormType_, 0);
  CHECK_GE(regularityNormType_, 0);
  CHECK_GE(huberParam_, 0.0);
  CHECK_GE(tukeyParam_, 0.0);
}

bool RegularVioBackendParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  return parseYAMLVioBackendParams(yaml_parser) &&
         parseYAMLRegularVioBackendParams(yaml_parser);
}

bool RegularVioBackendParams::equals(const BackendParams& vp2,
                                     double tol) const {
  return equalsVioBackendParams(vp2, tol) &&
         equalsRegularVioBackendParams(vp2, tol);
}

void RegularVioBackendParams::print() const {
  printVioBackendParams();
  printRegularVioBackendParams();
}

RegularVioBackendParams RegularVioBackendParams::safeCast(
    const BackendParams& params) {
  try {
    return dynamic_cast<const RegularVioBackendParams&>(params);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting VioBackendParams to "
                  "RegularVioBackendParams, but this object is not "
                  "a RegularVioBackendParams!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when casting to RegularVioBackendParams.";
  }
}

bool RegularVioBackendParams::parseYAMLRegularVioBackendParams(
    const YamlParser& yaml_parser) {
  yaml_parser.getYamlParam("monoNoiseSigma", &monoNoiseSigma_);
  yaml_parser.getYamlParam("monoNormType", &monoNormType_);
  yaml_parser.getYamlParam("monoNormParam", &monoNormParam_);
  yaml_parser.getYamlParam("stereoNoiseSigma", &stereoNoiseSigma_);
  yaml_parser.getYamlParam("stereoNormType", &stereoNormType_);
  yaml_parser.getYamlParam("stereoNormParam", &stereoNormParam_);
  yaml_parser.getYamlParam("regularityNoiseSigma", &regularityNoiseSigma_);
  yaml_parser.getYamlParam("regularityNormParam", &regularityNormParam_);
  yaml_parser.getYamlParam("regularityNormType", &regularityNormType_);
  int backend_modality = 0;
  yaml_parser.getYamlParam("backend_modality", &backend_modality);
  backend_modality_ = static_cast<RegularBackendModality>(backend_modality);
  return true;
}

bool RegularVioBackendParams::equalsRegularVioBackendParams(
    const BackendParams& vp2,
    double tol) const {
  RegularVioBackendParams rvp2 = RegularVioBackendParams::safeCast(vp2);
  return (fabs(monoNoiseSigma_ - rvp2.monoNoiseSigma_) <= tol) &&
         (monoNormType_ == rvp2.monoNormType_) &&
         (fabs(stereoNoiseSigma_ - rvp2.stereoNoiseSigma_) <= tol) &&
         (stereoNormType_ == rvp2.stereoNormType_) &&
         (fabs(regularityNoiseSigma_ - rvp2.regularityNoiseSigma_) <= tol) &&
         (fabs(monoNormParam_ - rvp2.monoNormParam_) <= tol) &&
         (fabs(stereoNormParam_ - rvp2.stereoNormParam_) <= tol) &&
         (fabs(regularityNormParam_ - rvp2.regularityNormParam_) <= tol) &&
         (regularityNormType_ == rvp2.regularityNormType_) &&
         (backend_modality_ == rvp2.backend_modality_);
}

}  // namespace VIO
