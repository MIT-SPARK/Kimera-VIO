/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureSelector.h
 * @brief  Class describing a greedy feature selection strategy, based on the paper:
 *
 * L. Carlone and S. Karaman. Attention and Anticipation in Fast Visual-Inertial Navigation.
 * In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages 3886-3893, 2017.
 *
 * @author Luca Carlone
 */

#ifndef FeatureSelector_H_
#define FeatureSelector_H_

#include <random>

// TODO clean number of include files, adds extra dependencies for ppl that
// want to use this kimera_vio library...
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/RegularImplicitSchurFactor.h>
#include <gtsam/geometry/Cal3Bundler.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/inference/Symbol.h>

#include "kimera-vio/backend/VioBackEndParams.h"
#include "kimera-vio/frontend/Frame.h"
#include "kimera-vio/frontend/StereoFrame.h"
#include "kimera-vio/frontend/VioFrontEndParams.h"
#include "kimera-vio/utils/UtilsOpenCV.h"

//#define useSpectra
#ifdef useSpectra
#include <Eigen/Core>
#include <SymEigsSolver.h>  // Also includes <MatOp/DenseGenMatProd.h>
#include <SymEigsShiftSolver.h>
#include <MatOp/DenseSymShiftSolve.h>
#endif

//#define FEATURE_SELECTOR_DEBUG_COUT

namespace VIO {
struct StampedPose{
public:
  StampedPose(const gtsam::Pose3 p, const double t);

public:
  gtsam::Pose3 pose;
  double timestampInSec;
};
// Map future keyframe id to StampedPose, with the assumption that.
using KeyframeToStampedPose = std::vector<StampedPose>;

////////////////////////////////////////////////////////////////////////////////
struct FeatureSelectorData{
public:
  gtsam::Matrix currentNavStateCovariance; // covariance matrix of current navigation state [rot, position, vel, biasAcc, biasOmega]
  KeyframeToStampedPose posesAtFutureKeyframes;

  gtsam::Pose3 body_P_leftCam;
  gtsam::Pose3 body_P_rightCam;

  gtsam::Cal3_S2 left_undistRectCameraMatrix;
  gtsam::Cal3_S2 right_undistRectCameraMatrix;

  std::vector<gtsam::Vector3> keypoints_3d;
  std::vector<int> keypointLife; // for each keypoint, based on their age and the max track length

  FeatureSelectorData();

  void print() const;
  bool isEmpty() const; // empty if there are no poses in horizon
};

////////////////////////////////////////////////////////////////////////////////
class FeatureSelector{
public:
  // Definitions.
  typedef gtsam::PinholeCamera<gtsam::Cal3_S2> Camera;
  typedef std::vector<Camera> Cameras;
  typedef gtsam::Matrix69 MatrixZD;
  typedef std::vector<MatrixZD> FBlocks;

  // Variables.
  double accVarianceDiscTime_;
  double biasAccVarianceDiscTime_;
  double integrationVar_;
  double sqrtInfoVision_;
  double imuDeltaT_;
  bool useStereo_;
  bool useLazyEvaluation_;
  double featureSelectionDefaultDepth_, featureSelectionCosineNeighborhood_,
  landmarkDistanceThreshold_;
  bool useSuccessProbabilities_;

  // Constructor.
  FeatureSelector(const VioFrontEndParams& trackerParams = VioFrontEndParams(),
                  const VioBackEndParams& vioParams = VioBackEndParams(),
                  const FeatureSelectorParams& feature_select_params =
                      FeatureSelectorParams());

  void print() const;

  /* ------------------------------------------------------------------------ */
  // Multiply hessian in hessian factor ptr by nonnegative double
  static void MultiplyHessianInPlace(gtsam::HessianFactor::shared_ptr Deltaj,
                                     const double c);

  /* ------------------------------------------------------------------------ */
  // cam_param contains the distorted, unrectified camera calibration used to
  // undistorRectify availableCorners.
  std::tuple<KeypointsCV, std::vector<size_t>, std::vector<double>>
  featureSelectionLinearModel(
      const KeypointsCV& availableCorners,
      const std::vector<double>& successProbabilities,
      const std::vector<double>& availableCornersDistances,
      const CameraParams& cam_param,
      const int need_n_corners,
      const FeatureSelectorData& featureSelectionData,
      const FeatureSelectorParams::FeatureSelectionCriterion& criterion) const;

  /* ------------------------------------------------------------------------ */
  static bool Comparator(const std::pair<size_t,double>& l,
                         const std::pair<size_t,double>& r);

  /* ------------------------------------------------------------------------ */
  // Sort upperBounds in descending order, storing the corresponding indices in the second
  // output argument (as in matlab sort).
  static std::pair< std::vector<size_t>,std::vector<double> >
  SortDescending(const std::vector<double>& upperBounds);

  /* ************************************************************************ */
  static double Logdet(const gtsam::Matrix& M);

  /* ************************************************************************ */
  static boost::tuple<int, double, gtsam::Vector> SmallestEigs(
      const gtsam::Matrix& A);

  /* ------------------------------------------------------------------------ */
  static std::tuple<std::vector<size_t>, std::vector<double>, double>
  OrderByUpperBound(
      const gtsam::GaussianFactorGraph::shared_ptr& bestOmegaBar,
      const std::vector<gtsam::HessianFactor::shared_ptr>& Deltas,
      const FeatureSelectorParams::FeatureSelectionCriterion& criterion);

  /* ------------------------------------------------------------------------ */
  static std::pair<std::vector<size_t>, std::vector<double>> GreedyAlgorithm(
      const gtsam::GaussianFactorGraph::shared_ptr& OmegaBar,
      const std::vector<gtsam::HessianFactor::shared_ptr>& Deltas,
      const int need_n_corners,
      const FeatureSelectorParams::FeatureSelectionCriterion& criterion,
      const bool useLazyEval = true);

  /* ------------------------------------------------------------------------ */
  static boost::tuple<int, double, gtsam::Vector>
  SmallestEigsPowerIter(const gtsam::Matrix& M);

  /* ------------------------------------------------------------------------ */
  static boost::tuple<int, double, gtsam::Vector>
  SmallestEigsSpectra(const gtsam::Matrix& M);

  /* ------------------------------------------------------------------------ */
  static boost::tuple<int, double, gtsam::Vector>
  SmallestEigsSpectraShift(const gtsam::Matrix& M);

  /* ------------------------------------------------------------------------ */
  static double EvaluateGain(
      const gtsam::GaussianFactorGraph::shared_ptr& OmegaBar,
      const gtsam::HessianFactor::shared_ptr& Deltaj,
      const FeatureSelectorParams::FeatureSelectionCriterion& criterion,
      bool useDenseMatrices = true);

  /* ------------------------------------------------------------------------ */
  std::pair<Cameras,Cameras> getCameras(
      const FeatureSelectorData& featureSelectionData) const;

  /* ------------------------------------------------------------------------ */
  gtsam::JacobianFactor createPrior(
      const FeatureSelectorData& featureSelectionData) const;

  /* ------------------------------------------------------------------------ */
  std::pair<gtsam::Matrix, gtsam::Matrix> createMatricesLinearImuFactor(
      const StampedPose& poseStamped_i,
      const StampedPose& poseStamped_j) const;

  /* ------------------------------------------------------------------------ */
  gtsam::GaussianFactorGraph createOmegaBarImuAndPrior(
      const FeatureSelectorData& featureSelectionData) const;

  /* ------------------------------------------------------------------------ */
  // Create projection factors on the Linear navigation state: position, vel, accBias
  // This "emulates" an actual smart factor and marginalizes out point
  // 1) Models measurement std and max distance
  // 2) does not mode outliers, since we "predict" ground truth measurements
  gtsam::HessianFactor::shared_ptr createLinearVisionFactor(
      const gtsam::Point3& pworld_l,
      const Cameras& left_cameras,
      const Cameras& right_cameras,
      double &debugFETime, double &debugSVDTime, double &debugSchurTime, // debug
      const int keypointLife = 1e9, bool hasRightPixel = true) const;

  /* ------------------------------------------------------------------------ */
  gtsam::GaussianFactorGraph::shared_ptr createOmegaBar(
      const FeatureSelectorData& featureSelectionData,
      const Cameras& left_cameras,
      const Cameras& right_cameras) const;

  /* ------------------------------------------------------------------------ */
  std::vector<gtsam::HessianFactor::shared_ptr> createDeltas(
      const std::vector<gtsam::Vector3>& availableVersors,
      const std::vector<double>& availableCornersDistances,
      const FeatureSelectorData& featureSelectionData,
      const Cameras& left_cameras, const Cameras& right_cameras) const;

  /* ------------------------------------------------------------------------ */
  // check if point, expressed in world frame, is within fov of camera (fov defined in terms of
  // angle and maxDistance). If so returns a versor corresponding to the direction to the point
  static boost::optional<gtsam::Unit3> GetVersorIfInFOV(
      const Camera& cam,
      const gtsam::Point3& pworld,
      const double& maxDistance = 1e9);

  /* ------------------------------------------------------------------------ */
  /**
   * Do Schur complement, given Jacobian as Fs,E,P, return SymmetricBlockMatrix
   * G = F' * F - F' * E * P * E' * F
   * g = zero
   * Fixed size version
   */
  static gtsam::SymmetricBlockMatrix SchurComplement(
      const FBlocks& Fs,
      const gtsam::Matrix& E,
      const gtsam::Matrix3& P,
      const gtsam::Vector& b);

  /* ------------------------------------------------------------------------ */
  // Before starting feature selection: returns selected smart measurements
  // (included tracked ones) and actual time it took for the selection.
  std::pair<SmartStereoMeasurements, double>
  splitTrackedAndNewFeatures_Select_Display(
      std::shared_ptr<StereoFrame>&
          stereoFrame_km1,  // not constant since we discard nonselected lmks
      const SmartStereoMeasurements& smartStereoMeasurements,
      const int& vio_cur_id,
      const int& saveImagesSelector,
      const FeatureSelectorParams::FeatureSelectionCriterion& criterion,
      const int& nrFeaturesToSelect,
      const int& maxFeatureAge,
      const KeyframeToStampedPose& posesAtFutureKeyframes,
      const gtsam::Matrix& curr_state_cov,
      const std::string& dataset_name,
      const Frame& frame_km1);
};

} // End of VIO namespace.

#endif /* FeatureSelector_H_ */
