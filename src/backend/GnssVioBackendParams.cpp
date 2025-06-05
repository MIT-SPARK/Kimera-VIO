/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   GnssVioBackendParams.cpp
 * @brief  Class collecting the parameters of the Gnss Visual Inertial odometry
 * pipeline for the GnssVIO implementation.
 * @author Elisaveta Karaseva
 */

#include "kimera-vio/backend/GnssVioBackendParams.h"

#include <string>  // for string

namespace VIO {

GnssVioBackendParams::GnssVioBackendParams() : RegularVioBackendParams() {
  // Trivial sanity checks.
  CHECK_GE(gnssNoiseSigma_, 0.0);
  CHECK_GE(stereoNoiseSigma_, 0.0);
  CHECK_GE(regularityNoiseSigma_, 0.0);
  CHECK_GE(monoNormType_, 0);
  CHECK_GE(stereoNormType_, 0);
  CHECK_GE(regularityNormType_, 0);
  CHECK_GE(huberParam_, 0.0);
  CHECK_GE(tukeyParam_, 0.0);
}

bool GnssVioBackendParams::parseYAML(const std::string& filepath) {
  YamlParser yaml_parser(filepath);
  return parseYAMLVioBackendParams(yaml_parser) &&
         parseYAMLRegularVioBackendParams(yaml_parser) &&
         parseYAMLGnssVioBackendParams(yaml_parser);
}

bool GnssVioBackendParams::parseYAMLGnssVioBackendParams(const YamlParser& yaml_parser) {
  yaml_parser.getYamlParam("initialGnssPoseSigma", &initialGnssPoseSigma_);
  yaml_parser.getYamlParam("gnssNoiseSigma", &gnssNoiseSigma_);
  yaml_parser.getYamlParam("gnssNormType", &gnssNormType_);
  yaml_parser.getYamlParam("gnssNormParam", &gnssNormParam_);
  return true;
}

bool GnssVioBackendParams::equals(const BackendParams& vp2, double tol) const {
  return equalsVioBackendParams(vp2, tol) &&
         equalsRegularVioBackendParams(vp2, tol) &&
         equalsGnssVioBackendParams(vp2, tol);
}

bool GnssVioBackendParams::equalsGnssVioBackendParams(const BackendParams& vp2,
                                                      double tol) const {
  GnssVioBackendParams rvp2 = GnssVioBackendParams::safeCast(vp2);
  return fabs(initialGnssPoseSigma_ - rvp2.initialGnssPoseSigma_) <= tol &&
         fabs(gnssNoiseSigma_ - rvp2.gnssNoiseSigma_) <= tol &&
         gnssNormType_ == rvp2.gnssNormType_ &&
         fabs(gnssNormParam_ - rvp2.gnssNormParam_) <= tol;
}

void GnssVioBackendParams::print() const {
  printVioBackendParams();
  printRegularVioBackendParams();
  printGnssVioBackendParams();
}

void GnssVioBackendParams::printGnssVioBackendParams() const {
  LOG(INFO) << "** GNSS Parameters **";
  LOG(INFO) << "initialGnssPoseSigma: " << initialGnssPoseSigma_;
  LOG(INFO) << "gnssNoiseSigma: " << gnssNoiseSigma_;
  LOG(INFO) << "gnssNormType: " << gnssNormType_;
  LOG(INFO) << "gnssNormParam: " << gnssNormParam_;
}

GnssVioBackendParams GnssVioBackendParams::safeCast(
    const BackendParams& params) {
  try {
    return dynamic_cast<const GnssVioBackendParams&>(params);
  } catch (const std::bad_cast& e) {
    LOG(ERROR) << "Seems that you are casting VioBackendParams to "
                  "GnssVioBackendParams, but this object is not "
                  "a GnssVioBackendParams!";
    LOG(FATAL) << e.what();
  } catch (...) {
    LOG(FATAL) << "Exception caught when casting to GnssVioBackendParams.";
  }
}

}  // namespace VIO
