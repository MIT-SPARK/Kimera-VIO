#pragma once
#include <gtsam/nonlinear/LinearContainerFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>

#include "kimera-vio/factors/PointPlaneFactor.h"

namespace VIO {

using SmartPoseFactor = gtsam::SmartStereoProjectionPoseFactor;
using PlanePrior = gtsam::PriorFactor<gtsam::OrientedPlane3>;
using PointPrior = gtsam::PriorFactor<gtsam::Point3>;

class FactorFormatter {
 public:
  struct Options {
    gtsam::KeyFormatter key_format = gtsam::DefaultKeyFormatter;
    std::string key_separator = ", ";
    std::string key_prefix = "[";
    std::string key_suffix = "]";
  };

  static FactorFormatter& instance();

  template <typename T>
  static std::string format(const T& factor, const Options* options = nullptr) {
    auto& fformat = instance();
    if (options) {
      fformat.options_ = *options;
    }

    const auto ret = fformat.toString(factor);
    fformat.options_ = fformat.defaults_;
    return ret;
  }

 private:
  template <typename T>
  std::string toString(const T& factor) const;

  std::string getKeyString(const gtsam::Factor& factor) const;

  FactorFormatter();

  Options options_;
  Options defaults_;
  static std::unique_ptr<FactorFormatter> instance_;
};

template <>
std::string FactorFormatter::toString<SmartPoseFactor>(
    const SmartPoseFactor& factor) const;

template <>
std::string FactorFormatter::toString<gtsam::PointPlaneFactor>(
    const gtsam::PointPlaneFactor& factor) const;

template <>
std::string FactorFormatter::toString<PlanePrior>(
    const PlanePrior& factor) const;

template <>
std::string FactorFormatter::toString<PointPrior>(
    const PointPrior& factor) const;

template <>
std::string FactorFormatter::toString<gtsam::LinearContainerFactor>(
    const gtsam::LinearContainerFactor& factor) const;

}  // namespace VIO
