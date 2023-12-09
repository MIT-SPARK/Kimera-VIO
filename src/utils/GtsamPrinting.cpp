#include "kimera-vio/utils/GtsamPrinting.h"

namespace VIO {

std::unique_ptr<FactorFormatter> FactorFormatter::instance_;

FactorFormatter& FactorFormatter::instance() {
  if (!instance_) {
    instance_.reset(new FactorFormatter());
  }

  return *instance_;
}

FactorFormatter::FactorFormatter() {}

std::string FactorFormatter::getKeyString(const gtsam::Factor& factor) const {
  std::stringstream ss;
  ss << options_.key_prefix;

  auto iter = factor.begin();
  while (iter != factor.end()) {
    ss << options_.key_format(*iter);
    ++iter;
    if (iter != factor.end()) {
      ss << options_.key_separator;
    }
  }

  ss << options_.key_suffix;
  return ss.str();
}

template <>
std::string FactorFormatter::toString<SmartPoseFactor>(
    const SmartPoseFactor& factor) const {
  std::stringstream ss;
  ss << std::boolalpha << "Smart Factor (valid: " << factor.isValid()
     << ", deg: " << factor.isDegenerate()
     << " isCheir: " << factor.isPointBehindCamera() << "): \t"
     << getKeyString(factor);
  return ss.str();
}

template <>
std::string FactorFormatter::toString<gtsam::PointPlaneFactor>(
    const gtsam::PointPlaneFactor& factor) const {
  std::stringstream ss;
  ss << "Point Plane Factor: plane key "
     << options_.key_format(factor.getPlaneKey()) << ", point key "
     << options_.key_format(factor.getPointKey());
  return ss.str();
}

template <>
std::string FactorFormatter::toString<PlanePrior>(
    const PlanePrior& factor) const {
  std::stringstream ss;
  ss << "Plane Prior: plane key \t" << getKeyString(factor);
  return ss.str();
}

template <>
std::string FactorFormatter::toString<PointPrior>(
    const PointPrior& factor) const {
  std::stringstream ss;
  ss << "Point Prior: point key \t" << getKeyString(factor);
  return ss.str();
}

template <>
std::string FactorFormatter::toString<gtsam::LinearContainerFactor>(
    const gtsam::LinearContainerFactor& factor) const {
  std::stringstream ss;
  ss << "Linear Container Factor: \t" << getKeyString(factor);
  return ss.str();
}

}  // namespace VIO
