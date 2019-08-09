/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology, * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   FeatureSelector.h
 * @brief  Class describing a greedy feature selection strategy, based on the
 * paper:
 *
 * L. Carlone and S. Karaman. Attention and Anticipation in Fast Visual-Inertial
 * Navigation. In IEEE Intl. Conf. on Robotics and Automation (ICRA), pages
 * 3886-3893, 2017.
 *
 * @author Luca Carlone
 */

#include <boost/filesystem.hpp> // to create folders
#include <glog/logging.h>

#include "FeatureSelector.h"

namespace VIO {

// TODO REMOVE GLOBAL VARS!
static const size_t D = 9;     // "camera" size
static const size_t ZDim = 6;  // "measurement" size
static const size_t N = 3;     // size of a point

static bool debug = false;
static bool testing = false;

static double numericalUpperBound = std::numeric_limits<double>::max();
// min instead will return a tiny positive number.
static double numericalLowerBound = -numericalUpperBound;

//////////////////////////////////////////////////////////////////////////////
StampedPose::StampedPose(const gtsam::Pose3 p, const double t)
    : pose(p), timestampInSec(t) {}

using KeyframeToStampedPose =
    std::vector<StampedPose>;  // map future keyframe id to StampedPose, with
                               // the assumption that

//////////////////////////////////////////////////////////////////////////////
FeatureSelectorData::FeatureSelectorData()
    : body_P_leftCam(gtsam::Pose3()),
      body_P_rightCam(gtsam::Pose3()),
      left_undistRectCameraMatrix(gtsam::Cal3_S2()),
      right_undistRectCameraMatrix(gtsam::Cal3_S2()) {}

void FeatureSelectorData::print() const {
  std::cout << "currentNavStateCovariance \n"
            << currentNavStateCovariance << std::endl;
  std::cout << "posesAtFutureKeyframes " << std::endl;
  for (size_t i = 0; i < posesAtFutureKeyframes.size(); i++) {
    std::cout << "\n timestampInSec: "
              << posesAtFutureKeyframes.at(i).timestampInSec << std::endl;
    posesAtFutureKeyframes.at(i).pose.print();
  }
  body_P_leftCam.print("\n body_P_leftCam \n");
  body_P_rightCam.print("\n body_P_rightCam \n");
  left_undistRectCameraMatrix.print("\n left_undistRectCameraMatrix \n");
  right_undistRectCameraMatrix.print("\n right_undistRectCameraMatrix \n");
  for (size_t i = 0; i < keypoints_3d.size(); i++) {
    std::cout << "keypoints_3d: " << keypoints_3d.at(i).transpose()
              << " with life " << keypointLife.at(i) << std::endl;
  }
}

bool FeatureSelectorData::isEmpty() const {
  return (posesAtFutureKeyframes.size() == 0);
}  // empty if there are no poses in horizon

//////////////////////////////////////////////////////////////////////////////
FeatureSelector::FeatureSelector(const VioFrontEndParams& trackerParams,
                                 const VioBackEndParams& vioParams) {
  imuDeltaT_ = trackerParams.featureSelectionImuRate_;
  // Variance, converted to discrete time, see ImuFactor.cpp
  accVarianceDiscTime_ = pow(vioParams.accNoiseDensity_, 2) / imuDeltaT_;
  // Variance, converted to discrete time, see CombinedImuFactor.cpp
  biasAccVarianceDiscTime_ = pow(vioParams.accBiasSigma_, 2) * imuDeltaT_;
  // Inverse of std of vision measurements
  sqrtInfoVision_ = 1 / vioParams.smartNoiseSigma_;  // TODO: this * 1000 should
                                                     // be fx of the calibration
  // Predict that we'll have stereo measurement or not
  useStereo_ = trackerParams.useStereoTracking_;
  // Variance of integration noise, converted to discrete time, see
  // ImuFactor.cpp (?)
  integrationVar_ = pow(vioParams.imuIntegrationSigma_, 2) * imuDeltaT_;
  featureSelectionDefaultDepth_ = trackerParams.featureSelectionDefaultDepth_;
  featureSelectionCosineNeighborhood_ =
      trackerParams.featureSelectionCosineNeighborhood_;
  landmarkDistanceThreshold_ = vioParams.landmarkDistanceThreshold_;
  useLazyEvaluation_ = trackerParams.featureSelectionUseLazyEvaluation_;
  useSuccessProbabilities_ = trackerParams.useSuccessProbabilities_;
  print();
}

void FeatureSelector::print() const {
  LOG(INFO) << "=============== FEATURE SELECTOR =============" << '\n'
            << "imuDeltaT_: " << imuDeltaT_ << '\n'
            << "accVarianceDiscTime_: " << accVarianceDiscTime_ << '\n'
            << "biasAccVarianceDiscTime_: " << biasAccVarianceDiscTime_ << '\n'
            << "sqrtInfoVision_: " << sqrtInfoVision_ << '\n'
            << "integrationVar_: " << integrationVar_ << '\n'
            << "useStereo_: " << useStereo_ << '\n'
            << "featureSelectionDefaultDepth_: "
            << featureSelectionDefaultDepth_ << '\n'
            << "featureSelectionCosineNeighborhood_: "
            << featureSelectionCosineNeighborhood_ << '\n'
            << "landmarkDistanceThreshold_: " << landmarkDistanceThreshold_
            << '\n'
            << "useLazyEvaluation_: " << useLazyEvaluation_ << '\n'
            << "useSuccessProbabilities_: " << useSuccessProbabilities_;
}

/* ------------------------------------------------------------------------ */
// Multiply hessian in hessian factor ptr by nonnegative double
void FeatureSelector::MultiplyHessianInPlace(
    gtsam::HessianFactor::shared_ptr Deltaj, const double c) {
  if (c < 0)
    throw std::runtime_error(
        "MultiplyHessianInPlace: cannot multiply by negative number");

  // do multiplication blockwise:
  gtsam::FastVector<gtsam::Key> keys = Deltaj->keys();
  gtsam::SymmetricBlockMatrix& cH = Deltaj->info();
  gtsam::Matrix B;
  for (size_t i = 0; i < keys.size() + 1;
       i++) {  // +1 since there is also a vector attached
    B = cH.block(i, i);
    cH.setDiagonalBlock(i, c * B);
    for (size_t j = i + 1; j < keys.size() + 1;
         j++) {  // +1 since there is also a vector attached
      B = cH.block(i, j);
      cH.setOffDiagonalBlock(i, j, c * B);
    }
  }
}

/* ------------------------------------------------------------------------ */
// cam_param contains the distorted, unrectified camera calibration used to
// undistorRectify availableCorners.
std::tuple<KeypointsCV, std::vector<size_t>, std::vector<double>>
FeatureSelector::featureSelectionLinearModel(
    const KeypointsCV& availableCorners,
    const std::vector<double>& successProbabilities,
    const std::vector<double>& availableCornersDistances,
    const CameraParams& cam_param, const int need_n_corners,
    const FeatureSelectorData& featureSelectionData,
    const VioFrontEndParams::FeatureSelectionCriterion& criterion) const {
#ifdef FEATURE_SELECTOR_DEBUG_COUT
  double startTime = UtilsOpenCV::GetTimeInSeconds();
#endif
  // create cameras to test reprojection
  if (debug) {
    std::cout << "featureSelectionLinearModel: getCameras" << std::endl;
  }
  Cameras left_cameras, right_cameras;
  std::tie(left_cameras, right_cameras) = getCameras(featureSelectionData);

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "getCameras time: "
            << UtilsOpenCV::GetTimeInSeconds() - startTime << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // create OmegaBar: includes IMU and existing vision measurements
  if (debug) {
    std::cout << "featureSelectionLinearModel: createOmegaBar" << std::endl;
  }
  gtsam::GaussianFactorGraph::shared_ptr OmegaBar =
      createOmegaBar(featureSelectionData, left_cameras, right_cameras);

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "OmegaBar time: " << UtilsOpenCV::GetTimeInSeconds() - startTime
            << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // get directions from each (UNCALIBRATED) available corner in left camera at
  // time 0 (cam_param includes also distortion)
  if (debug) {
    std::cout << "featureSelectionLinearModel: availableVersors" << std::endl;
  }
  std::vector<gtsam::Vector3> availableVersors;
  availableVersors.reserve(availableCorners.size());
  for (size_t l = 0; l < availableCorners.size(); l++)
    availableVersors.push_back(
        Frame::CalibratePixel(availableCorners.at(l), cam_param));

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "known points time: "
            << UtilsOpenCV::GetTimeInSeconds() - startTime << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // create Deltas for each direction
  if (debug) {
    std::cout << "featureSelectionLinearModel: createDeltas" << std::endl;
  }
  std::vector<gtsam::HessianFactor::shared_ptr> Deltas =
      createDeltas(availableVersors, availableCornersDistances,
                   featureSelectionData, left_cameras, right_cameras);

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "createDeltas time: "
            << UtilsOpenCV::GetTimeInSeconds() - startTime
            << "(nr factors, maybe empty: " << Deltas.size() << ")"
            << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  if (useSuccessProbabilities_) {
    // UtilsOpenCV::PrintVector(successProbabilities,"successProbabilities in
    // selector");
    for (size_t j = 0; j < Deltas.size(); j++) {
      if (!Deltas.at(j)->empty())
        MultiplyHessianInPlace(Deltas.at(j), successProbabilities.at(j));
    }
  }

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "success time: " << UtilsOpenCV::GetTimeInSeconds() - startTime
            << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // apply greedy algorithm to select need_n_corners out of the available
  // corners
  if (debug) {
    std::cout << "featureSelectionLinearModel: GreedyAlgorithm" << std::endl;
  }
  std::vector<size_t> selectedIndices;
  std::vector<double> selectedMarginalGains;
  std::tie(selectedIndices, selectedMarginalGains) = GreedyAlgorithm(
      OmegaBar, Deltas, need_n_corners, criterion, useLazyEvaluation_);

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "Overall time greedy alg: "
            << UtilsOpenCV::GetTimeInSeconds() - startTime << std::endl;
#endif

  if (debug) {
    std::cout << "featureSelectionLinearModel: selectedCorners" << std::endl;
  }
  KeypointsCV selectedCorners;
  selectedCorners.reserve(selectedIndices.size());
  for (auto ind : selectedIndices)
    selectedCorners.push_back(availableCorners.at(ind));

  return std::make_tuple(selectedCorners, selectedIndices,
                         selectedMarginalGains);
}

/* ------------------------------------------------------------------------ */
bool FeatureSelector::Comparator(const std::pair<size_t, double>& l,
                                 const std::pair<size_t, double>& r) {
  return l.second > r.second;
}

/* ------------------------------------------------------------------------ */
// Sort upperBounds in descending order, storing the corresponding indices in
// the second output argument (as in matlab sort).
std::pair<std::vector<size_t>, std::vector<double>>
FeatureSelector::SortDescending(const std::vector<double>& upperBounds) {
  size_t N = upperBounds.size();

  // apply sort to vector of <size_t,double> pairs, with smarted comparison rule
  std::vector<std::pair<size_t, double>> indValuePairs;
  indValuePairs.reserve(N);
  for (size_t j = 0; j < N; j++) {
    indValuePairs.push_back(std::make_pair(j, upperBounds.at(j)));
  }
  // sort
  std::sort(indValuePairs.begin(), indValuePairs.end(), Comparator);

  // unpack into 2 vectors
  std::vector<size_t> ordering;
  ordering.reserve(N);
  std::vector<double> sortedUpperBound;
  sortedUpperBound.reserve(N);
  for (size_t j = 0; j < N; j++) {
    ordering.push_back(indValuePairs.at(j).first);
    sortedUpperBound.push_back(indValuePairs.at(j).second);
  }
  return std::make_pair(ordering, sortedUpperBound);
}

/* ************************************************************************ */
double FeatureSelector::Logdet(const gtsam::Matrix& M) {
  double logDet = 0;
  gtsam::Matrix llt = gtsam::LLt(M);  // lower triangular
  for (size_t i = 0; i < llt.rows(); ++i) logDet += log(llt(i, i));
  logDet *= 2;
  return logDet;
}

/* ************************************************************************ */
boost::tuple<int, double, gtsam::Vector> FeatureSelector::SmallestEigs(
    const gtsam::Matrix& A) {
  // Check size of A
  size_t n = A.rows(), p = A.cols(), m = std::min(n, p);

  // Do SVD on A
  Eigen::JacobiSVD<gtsam::Matrix> svd(A, Eigen::ComputeFullV);
  gtsam::Vector s = svd.singularValues();
  gtsam::Matrix V = svd.matrixV();

  // Find rank
  size_t rank = 0;
  for (size_t j = 0; j < m; j++)
    if (s(j) > 1e-4) rank++;

  // Return rank, error, and corresponding column of V
  double error = m < p ? 0 : s(m - 1);
  return boost::tuple<int, double, gtsam::Vector>(
      (int)rank, error, gtsam::Vector(gtsam::column(V, p - 1)));
}

/* ------------------------------------------------------------------------ */
std::tuple<std::vector<size_t>, std::vector<double>, double>
FeatureSelector::OrderByUpperBound(
    const gtsam::GaussianFactorGraph::shared_ptr& bestOmegaBar,
    const std::vector<gtsam::HessianFactor::shared_ptr>& Deltas,
    const VioFrontEndParams::FeatureSelectionCriterion& criterion) {
  std::vector<double> upperBounds;
  upperBounds.reserve(N);  // for lazy evaluation
  size_t sizeBestOmegaBar = bestOmegaBar->size();

  int rank;
  double eigValue;
  gtsam::Vector eigVector;
  double gainOmegaBar;
  gtsam::VectorValues xx, yy;
  switch (criterion) {
    case VioFrontEndParams::FeatureSelectionCriterion::MIN_EIG:
      // get eigenvector and smallest eigenvalue of bestOmegaBar
      boost::tie(rank, eigValue, eigVector) =
          SmallestEigs(bestOmegaBar->hessian().first);
      gainOmegaBar = eigValue;
      for (auto key : bestOmegaBar->keys()) {
        xx.insert(key, eigVector.segment<9>(9 * key));
      }
      // compute upper bounds lambdaMin(A + B) = lambdaMin(A) + norm(B *
      // mu_min_A)
      for (size_t j = 0; j < Deltas.size(); j++) {
        yy.setZero();
        Deltas.at(j)->multiplyHessianAdd(1.0, xx, yy);  // y += 1 * Hessian * xx
        upperBounds.push_back(eigValue + yy.norm());    //
      }
      break;
    case VioFrontEndParams::FeatureSelectionCriterion::
        LOGDET:  // picks the best features that maximize the logdet of the
                 // covariance
      gainOmegaBar = EvaluateGain(
          bestOmegaBar, boost::make_shared<gtsam::HessianFactor>(), criterion);
      // compute upper bounds det(M) = prod (Mii)
      for (size_t j = 0; j < Deltas.size(); j++) {  // for each delta
        if (!Deltas.at(j)->empty()) bestOmegaBar->push_back(Deltas.at(j));
        gtsam::Vector hessianDiagonal =
            bestOmegaBar->hessianDiagonal().vector();
        double sumLogDiag = 0;
        for (size_t k = 0; k < hessianDiagonal.size(); k++) {
          sumLogDiag += log(hessianDiagonal(k));
        }
        upperBounds.push_back(sumLogDiag);
        bestOmegaBar->resize(
            sizeBestOmegaBar);  // since bestOmegaBar is a shared pointer, it
                                // would carry Deltas.at(j)
      }
      break;
    default:
      throw std::runtime_error("OrderByUpperBound: wrong choice of criterion");
      break;
  }
  // check that upper bounds are valid
  if (testing) {
    for (size_t j = 0; j < Deltas.size(); j++) {
      std::cout.precision(20);
      std::cout << "upb: " << upperBounds.at(j) << " gain: "
                << EvaluateGain(bestOmegaBar, Deltas.at(j), criterion)
                << " criterion " << criterion << std::endl;
      // check with some relative (numerical) tolerance
      if (upperBounds.at(j) <
          (1 - 1e-7) * EvaluateGain(bestOmegaBar, Deltas.at(j), criterion)) {
        std::cout << "WATCH OUT : upb: " << upperBounds.at(j) << " gain: "
                  << EvaluateGain(bestOmegaBar, Deltas.at(j), criterion)
                  << " criterion " << criterion << std::endl;
        throw std::runtime_error("OrderByUpperBound: invalid upper bound");
      }
    }
  }
  // sort in descending order by upper bound
  std::vector<size_t> ordering;
  std::vector<double> sortedUpperBounds;
  std::tie(ordering, sortedUpperBounds) = SortDescending(upperBounds);
  return std::make_tuple(ordering, sortedUpperBounds, gainOmegaBar);
}

/* ------------------------------------------------------------------------ */
std::pair<std::vector<size_t>, std::vector<double>>
FeatureSelector::GreedyAlgorithm(
    const gtsam::GaussianFactorGraph::shared_ptr& OmegaBar,
    const std::vector<gtsam::HessianFactor::shared_ptr>& Deltas,
    const int need_n_corners,
    const VioFrontEndParams::FeatureSelectionCriterion& criterion,
    const bool useLazyEval) {
  size_t N = Deltas.size();  // nr of available features

  // initialize set: we haven't picked any feature yet
  gtsam::GaussianFactorGraph::shared_ptr bestOmegaBar = OmegaBar->cloneToPtr();
  std::vector<size_t> selectedIndices;
  std::vector<double> selectedMarginalGains;
  std::vector<size_t> ordering;
  ordering.reserve(N);  // for lazy evaluation
  std::vector<double> upperBounds;
  upperBounds.reserve(N);                           // for lazy evaluation
  gtsam::Vector inserted = gtsam::Vector::Zero(N);  // 0 = not inserted
  // run greedy
  double nrGainEval = 0;
  double gainOmegaBar = 0;
  double timeOrdering = 0, timeGreedy = 0;
  double startTime;
  for (size_t i = 0; i < need_n_corners;
       i++) {  // for each feature we have to add

#ifdef FEATURE_SELECTOR_DEBUG_COUT
    startTime = UtilsOpenCV::GetTimeInSeconds();
#endif
    if (useLazyEval) {
      std::tie(ordering, upperBounds, gainOmegaBar) =
          OrderByUpperBound(bestOmegaBar, Deltas, criterion);
    } else {
      ordering.clear();
      upperBounds.clear();
      for (size_t j = 0; j < N; j++) {
        ordering.push_back(j);  // process sequentially
        upperBounds.push_back(numericalUpperBound);
      }
      gainOmegaBar = 0;
    }
#ifdef FEATURE_SELECTOR_DEBUG_COUT
    timeOrdering += UtilsOpenCV::GetTimeInSeconds() - startTime;
#endif

    int best_j = -1;
    double best_gain_j = numericalLowerBound;
    gtsam::Vector gains = gtsam::Vector::Zero(N);  // only for debug
#ifdef FEATURE_SELECTOR_DEBUG_COUT
    startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

    // greedly select best index
    for (size_t indj = 0; indj < N;
         indj++) {  // loop over all available features and compute gain

      // check lazy stopping condition
      if (useLazyEval && (best_gain_j > upperBounds.at(indj)))
        break;  // lazy evaluation is current best is better than sorted upper
                // bound, then we can stop

      size_t j = ordering.at(
          indj);  // make sure that we look according to descending upperbounds

      // std::cout << " best_gain_j " << best_gain_j << " best_j " << best_j <<
      // " j "<< j << " nr keys: " << Deltas.at(j).keys().size() << std::endl;
      if (inserted(j) == 0 &&
          (Deltas.at(j)->keys().size() > 0 ||
           !useLazyEval)) {  // not inserted yet and Delta_j has some info
        gains(j) = EvaluateGain(bestOmegaBar, Deltas.at(j), criterion);
        nrGainEval += 1;
        // std::cout << " gains(j) " << gains(j) << " best_gain_j " <<
        // best_gain_j << " best_j " << best_j << " j "<< j << std::endl; update
        // max
        if (gains(j) > best_gain_j) {
          best_j = j;
          best_gain_j = gains(j);
        }
      } else {          // already selected or empty Delta_j
        gains(j) = -1;  // we already included this so cannot be selected again
        nrGainEval += 1;
      }
    }
    // if no feature won, pick the first that was not taken (features are
    // ordered by quality)
    if (best_gain_j == numericalLowerBound) {
      for (size_t j = 0; j < N; j++) {
        if (inserted(j) == 0) {
          best_j = j;
          break;  // pick first nonzero
        }
      }
    }
    // std::cout << "gains.sortDescending(): " << gains.sortDescending() <<
    // std::endl;

#ifdef FEATURE_SELECTOR_DEBUG_COUT
    timeGreedy += UtilsOpenCV::GetTimeInSeconds() - startTime;
#endif
    // std::cout << "gains: " << gains.transpose() << std::endl;
    // add to current set
    bestOmegaBar->push_back(Deltas.at(best_j));
    selectedIndices.push_back(best_j);
    selectedMarginalGains.push_back(best_gain_j -
                                    gainOmegaBar);  // marginal gains
    // mark as inserted to avoid that is reselected later on
    inserted(best_j) = 1;
  }

  double relNrGainEval = nrGainEval / double(N * need_n_corners);
#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "-- -- greedyAlgorithm: Ordering time: " << timeOrdering
            << std::endl;
  std::cout << "-- -- greedyAlgorithm: Greedy time: " << timeGreedy
            << std::endl;
#endif
  std::cout << "-- -- greedyAlgorithm: nrGainEval: " << nrGainEval << "/"
            << N * need_n_corners << " (relative: " << relNrGainEval << ") "
            << std::endl;
  return std::make_pair(selectedIndices, selectedMarginalGains);
}

/* ------------------------------------------------------------------------ */
boost::tuple<int, double, gtsam::Vector> FeatureSelector::SmallestEigsPowerIter(
    const gtsam::Matrix& M) {
  int maxIter = 1e7;
  double tol = 1e-4;

  // initialize vector
  gtsam::Vector xold = gtsam::Vector::Zero(M.cols());
  gtsam::Vector x = gtsam::Vector::Ones(M.cols());
  for (size_t i = 0; i < M.cols(); ++i) x(i) = rand();
  x = x / x.norm();

  // get max eigenvalue:Ax = lambda x
  // Basic power iteration from: https://en.wikipedia.org/wiki/Power_iteration
  bool reachedMaxIter = true;
  double normx;
  for (size_t iter = 0; iter < maxIter; ++iter) {
    x = M * x;
    normx = x.norm();
    x = x / normx;
    if ((x - xold).norm() < tol) {
      std::cout << "Largest eig: " << normx << "(iters: " << iter << ")"
                << std::endl;
      reachedMaxIter = false;
      break;
    }
    xold = x;
  }
  double lambdaMax = normx;
  if (reachedMaxIter)
    std::cout << "LargestEigsFast: reached maximum number of iterations"
              << std::endl;

  for (size_t i = 0; i < M.cols(); ++i) x(i) = rand();
  // B = -(A - lambdaMax * I) = -A + lambdaMax*I
  gtsam::Matrix B = -M;
  for (size_t i = 0; i < B.cols(); i++) B(i, i) += lambdaMax;

  // Basic power iteration from: https://en.wikipedia.org/wiki/Power_iteration
  // Ax = lambda x
  reachedMaxIter = true;
  tol = 1e-6;  // here we need accuracy
  for (size_t iter = 0; iter < maxIter; ++iter) {
    x = B * x;
    normx = x.norm();
    x = x / normx;
    if ((x - xold).norm() < tol) {
      std::cout << "Smallest eig: " << normx << "(iters: " << iter << ")"
                << std::endl;
      reachedMaxIter = false;
      break;
    }
    xold = x;
  }
  if (reachedMaxIter)
    std::cout << "SmallestEigsPowerIter: reached maximum number of iterations"
              << std::endl;

  int rank = -1;
  double error = -normx + lambdaMax;
  return boost::tuple<int, double, gtsam::Vector>(rank, error, -x);
}

/* ------------------------------------------------------------------------ */
boost::tuple<int, double, gtsam::Vector> FeatureSelector::SmallestEigsSpectra(
    const gtsam::Matrix& M) {
#ifdef useSpectra
  // Construct matrix operation object using the wrapper class DenseGenMatProd
  Spectra::DenseSymMatProd<double> op(M);

  // Construct eigen solver object, requesting the largest three eigenvalues
  Spectra::SymEigsSolver<double, Spectra::SMALLEST_MAGN,
                         Spectra::DenseSymMatProd<double>>
      eigs(&op, 1, round(M.cols() / 2));  //

  // Initialize and compute
  eigs.init();
  int nconv = eigs.compute(1000, 1e-4);

  // Retrieve results
  if (eigs.info() != Spectra::SUCCESSFUL)
    throw std::runtime_error("spectra: eigs was not successful :-(");

  Eigen::VectorXd minEigs = eigs.eigenvalues();
  double minEig = minEigs(0);
  // Eigen::VectorXd actualVects = eigs.eigenvectors();
  // gtsam::Vector actualVect = actualVects.col(0);
  gtsam::Vector eigenvector = eigs.eigenvectors();

  return boost::tuple<int, double, gtsam::Vector>(-1, minEig, eigenvector);
#else
  return SmallestEigs(M);
#endif
}

/* ------------------------------------------------------------------------ */
boost::tuple<int, double, gtsam::Vector>
FeatureSelector::SmallestEigsSpectraShift(const gtsam::Matrix& M) {
#ifdef useSpectra
  // Construct matrix operation object using the wrapper class DenseGenMatProd
  Spectra::DenseSymShiftSolve<double> op(M);

  Spectra::SymEigsShiftSolver<double, Spectra::LARGEST_MAGN,
                              Spectra::DenseSymShiftSolve<double>>
      eigs(&op, 1, round(M.cols() / 2), 0.0);
  eigs.init();
  eigs.compute(1000, 1e-4);

  // Retrieve results
  if (eigs.info() != Spectra::SUCCESSFUL)
    throw std::runtime_error("spectraShift: eigs was not successful :-(");

  Eigen::VectorXd minEigs = eigs.eigenvalues();
  double minEig = minEigs(0);
  // Eigen::VectorXd actualVects = eigs.eigenvectors();
  // gtsam::Vector actualVect = actualVects.col(0);
  gtsam::Vector eigenvector = -eigs.eigenvectors();

  return boost::tuple<int, double, gtsam::Vector>(-1, minEig, eigenvector);
#else
  return SmallestEigs(M);
#endif
}

/* ------------------------------------------------------------------------ */
double FeatureSelector::EvaluateGain(
    const gtsam::GaussianFactorGraph::shared_ptr& OmegaBar,
    const gtsam::HessianFactor::shared_ptr& Deltaj,
    const VioFrontEndParams::FeatureSelectionCriterion& criterion,
    bool useDenseMatrices) {
  // augment graph
  size_t sizeOmegaBar = OmegaBar->size();
  // gtsam::GaussianFactorGraph::shared_ptr OmegaBar_U_Deltaj = OmegaBar;
  if (!Deltaj->empty())  // if contains something
    OmegaBar->push_back(Deltaj);

  // compute gain
  double gain;
  boost::shared_ptr<gtsam::GaussianBayesTree> bayesTree;
  int rank;
  gtsam::Vector eigVector;
  switch (criterion) {
    case VioFrontEndParams::FeatureSelectionCriterion::
        MIN_EIG:  // picks the best features that maximize the smallest
                  // eigenvalue of the covariance
      if (!useDenseMatrices) {
        std::cout << "EvaluateGain: useDenseMatrices is deprecated"
                  << std::endl;
        // NOTE (Luca): 2x slower than SmallestEigs
        boost::tie(rank, gain, eigVector) =
            SmallestEigsPowerIter(OmegaBar->hessian().first);
        // boost::tie(rank,gain,eigVector) =
        // SmallestEigsSpectra(OmegaBar->hessian().first);
        // boost::tie(rank,gain,eigVector) =
        // SmallestEigsSpectraShift(OmegaBar->hessian().first);
      } else {  // use eigen with dense matrices
        boost::tie(rank, gain, eigVector) =
            SmallestEigs(OmegaBar->hessian().first);
      }
      break;
    case VioFrontEndParams::FeatureSelectionCriterion::
        LOGDET:  // picks the best features that maximize the logdet of the
                 // covariance
      if (!useDenseMatrices) {
        std::cout << "EvaluateGain: useDenseMatrices is deprecated"
                  << std::endl;
        // NOTE (Luca): this seems slightly slower than
        // Logdet(OmegaBar->hessian().first): 0.032 vs 0.030 also, sparse
        // version sometimes gives: terminate called after throwing an instance
        // of 'gtsam::InconsistentEliminationRequested' what():  An inference
        // algorithm was called with inconsistent arguments.  The factor graph,
        // ordering, or variable index were inconsistent with each other, or a
        // full elimination routine was called with an ordering that does not
        // include all of the variables.
        bayesTree = OmegaBar->eliminateMultifrontal();
        gain = 2 * bayesTree->logDeterminant();
        // pow(bayesTree->determinant(),2); // Bayes tree determinant returns
        // det(R), where R'R = our hessian
      } else {  // use eigen with dense matrices
        gain = Logdet(OmegaBar->hessian().first);
        // throw std::runtime_error("EvaluateGain: det computation leads to very
        // large numbers"); gain =
        // OmegaBar_U_Deltaj.hessian().first.determinant();
      }
      break;
    default:
      throw std::runtime_error("EvaluateGain: wrong choice of criterion");
      break;
  }
  OmegaBar->resize(sizeOmegaBar);  // without this, since we work with shared
                                   // ptrs, we would also carry Deltaj
  return gain;
}

/* ------------------------------------------------------------------------ */
std::pair<FeatureSelector::Cameras, FeatureSelector::Cameras>
FeatureSelector::getCameras(
    const FeatureSelectorData& featureSelectionData) const {
  // future poses:
  const KeyframeToStampedPose& posesAtFutureKeyframes =
      featureSelectionData.posesAtFutureKeyframes;

  // nr of cameras to be created
  size_t nrKeyframesInHorizon = posesAtFutureKeyframes.size();

  // create cameras
  Cameras left_cameras, right_cameras;
  left_cameras.reserve(nrKeyframesInHorizon);
  right_cameras.reserve(nrKeyframesInHorizon);
  for (size_t i = 0; i < nrKeyframesInHorizon; i++) {
    const gtsam::Pose3 bodyPose = posesAtFutureKeyframes.at(i).pose;
    left_cameras.push_back(
        Camera(bodyPose.compose(featureSelectionData.body_P_leftCam),
               featureSelectionData.left_undistRectCameraMatrix));
    right_cameras.push_back(
        Camera(bodyPose.compose(featureSelectionData.body_P_rightCam),
               featureSelectionData.right_undistRectCameraMatrix));
  }
  return std::make_pair(left_cameras, right_cameras);
}

/* ------------------------------------------------------------------------ */
gtsam::JacobianFactor FeatureSelector::createPrior(
    const FeatureSelectorData& featureSelectionData) const {
  // get prior over nonlinear state: pose, velocity, biases
  gtsam::Matrix9 covariancePriorLocal =
      featureSelectionData.currentNavStateCovariance.block<9, 9>(
          3, 3);  // happens to be the matrix we want

  // get keys of current state
  gtsam::Key stateIndex = 0;

  // convert to position covariance to global frame:
  const StampedPose poseStamped0 =
      featureSelectionData.posesAtFutureKeyframes.at(stateIndex);
  Matrix3 R0 = poseStamped0.pose.rotation().matrix();
  gtsam::Matrix9 RII = gtsam::Matrix9::Identity();
  RII.block<3, 3>(0, 0) = R0;
  RII.block<3, 3>(3, 3) = R0;
  gtsam::Matrix covariancePrior = RII * covariancePriorLocal * RII.transpose();
  gtsam::noiseModel::Gaussian::shared_ptr noisePrior =
      gtsam::noiseModel::Gaussian::Covariance(covariancePrior);

  // Since GTSAM only supports diagonal noise model with Jacobians, we have to
  // whiten
  // ||x - xBar||_Omega = ||sqrtOmega x - sqrtOmega xBar ||
  gtsam::Matrix A_prior = gtsam::Matrix9::Identity();
  noisePrior->WhitenInPlace(
      A_prior);  // Now A_prior contains the sq info matrix

  // values in the vector are irrelevant
  return gtsam::JacobianFactor(stateIndex, A_prior, gtsam::Vector::Zero(9));
  ;
}

/* ------------------------------------------------------------------------ */
std::pair<gtsam::Matrix, gtsam::Matrix>
FeatureSelector::createMatricesLinearImuFactor(
    const StampedPose& poseStamped_i, const StampedPose& poseStamped_j) const {
  double Deltaij = poseStamped_j.timestampInSec -
                   poseStamped_i.timestampInSec;  // clearly in seconds

  // sanity check
  if (Deltaij < 0)
    throw std::runtime_error(
        "createMatricesLinearImuFactor: Deltaij is negative!");

  int nrImuMeasurements = round(Deltaij / imuDeltaT_);
  double imuRate_ij = double(
      Deltaij / nrImuMeasurements);  // small adjustment to accounts the Deltaij
                                     // is not multiple of imuDeltaT_

  // sanity check
  if (std::abs(imuRate_ij - imuDeltaT_) > imuDeltaT_)
    throw std::runtime_error(
        "createOmegaBar: imuRate_ij too different from imuDeltaT_");
  if (std::abs(nrImuMeasurements * imuRate_ij - Deltaij) >
      1e-4)  // we did all this to make Deltaij a multiple of imuRate_ij
    throw std::runtime_error(
        "createOmegaBar: Deltaij inconsistent with imuRate_ij");

  // 1.1) create matrices Mij and Nij and Covariance: covImu =
  // [accVarianceDiscTime_ *  CCt   0 ;  0  biasAccVarianceDiscTime_ * I33]
  gtsam::Rot3 Ri_rot = poseStamped_i.pose.rotation();
  gtsam::Rot3 Rj_rot = poseStamped_j.pose.rotation();
  gtsam::Rot3 Rij_rot = Ri_rot.inverse() * Rj_rot;
  gtsam::Vector3 rotVec_ij = gtsam::Rot3::Logmap(Rij_rot);
  gtsam::Matrix3 Rimu =
      gtsam::Rot3::Expmap(rotVec_ij / nrImuMeasurements).matrix();
  gtsam::Matrix3 Ri = Ri_rot.matrix();
  gtsam::Matrix3 Mij = gtsam::Matrix3::Zero();
  gtsam::Matrix3 Nij = gtsam::Matrix3::Zero();
  double CCt_11 = 0, CCt_12 = 0;  // block entries of CCt

  double imuRate_ij_2 = imuRate_ij * imuRate_ij;      // squared
  double imuRate_ij_3 = imuRate_ij * imuRate_ij_2;    // power 3
  double imuRate_ij_4 = imuRate_ij_2 * imuRate_ij_2;  // power 4
  gtsam::Matrix3 Rh = Ri;
  for (size_t h = 0; h < nrImuMeasurements;
       h++) {  // note wrt draft: k=0 and j-1 = nrImuMeasurements
    Mij += Rh;

    double jkh = (nrImuMeasurements - h - 0.5);
    Nij += jkh * Rh;

    // entries of CCt
    CCt_11 += jkh * jkh;
    CCt_12 += jkh;

    Rh = Rh * Rimu;
  }
  Nij = Nij * imuRate_ij_2;
  Mij = Mij * imuRate_ij;

  gtsam::Matrix9 covImu = gtsam::Matrix9::Zero();
  covImu.block<3, 3>(0, 0) =
      gtsam::I_3x3 * (nrImuMeasurements * integrationVar_ +
                      CCt_11 * imuRate_ij_4 * accVarianceDiscTime_);
  covImu.block<3, 3>(0, 3) =
      gtsam::I_3x3 * CCt_12 * imuRate_ij_3 * accVarianceDiscTime_;
  covImu.block<3, 3>(3, 0) = covImu.block<3, 3>(0, 3).transpose();
  covImu.block<3, 3>(3, 3) =
      gtsam::I_3x3 * nrImuMeasurements * imuRate_ij_2 * accVarianceDiscTime_;
  covImu.block<3, 3>(6, 6) =
      gtsam::I_3x3 * nrImuMeasurements *
      biasAccVarianceDiscTime_;  //  this depends on the imu bias random walk

  // sanity check
  if ((Rj_rot.matrix() - Rh).norm() > 1e-2) {
    std::cout << "Rj_rot \n"
              << Rj_rot.matrix() << "\n Rh \n"
              << Rh << std::endl;
    throw std::runtime_error("createOmegaBar: Rh integration is inconsistent");
  }

  gtsam::Matrix Ai = -gtsam::Matrix9::Identity();
  Ai.block<3, 3>(0, 3) = -gtsam::I_3x3 * Deltaij;
  Ai.block<3, 3>(0, 6) = Nij;
  Ai.block<3, 3>(3, 6) = Mij;

  return std::make_pair(Ai, covImu);
}

/* ------------------------------------------------------------------------ */
gtsam::GaussianFactorGraph FeatureSelector::createOmegaBarImuAndPrior(
    const FeatureSelectorData& featureSelectionData) const {
  const KeyframeToStampedPose& posesAtFutureKeyframes =
      featureSelectionData.posesAtFutureKeyframes;

  // create OmegaBar: requires rotations at imu timestamps in horizon (we get
  // this by interpolation of frame )
  gtsam::GaussianFactorGraph OmegaBar;
  ////////////////////////////////////////////////////////////////////////////////////////
  // 0) create prior on first state: note: here the state is in Real{9}:
  // position, velocity, biasAcc
  OmegaBar.push_back(createPrior(featureSelectionData));

  ////////////////////////////////////////////////////////////////////////////////////////
  // 1) add linear imu factors between consecutive states
  size_t nrKeyframesInHorizon = posesAtFutureKeyframes.size();
  for (size_t i = 0; i < nrKeyframesInHorizon - 1; i++) {
    const StampedPose& poseStamped_i = posesAtFutureKeyframes.at(i);
    const StampedPose& poseStamped_j = posesAtFutureKeyframes.at(i + 1);

    gtsam::Matrix Ai, covImu;
    std::tie(Ai, covImu) =
        createMatricesLinearImuFactor(poseStamped_i, poseStamped_j);

    // create noise model
    gtsam::noiseModel::Gaussian::shared_ptr noiseImu =
        gtsam::noiseModel::Gaussian::Covariance(covImu);

    // this coefficient matrix is always the identity
    gtsam::Matrix Aj = gtsam::Matrix9::Identity();

    // whiten and add to graph
    noiseImu->WhitenInPlace(Ai);
    noiseImu->WhitenInPlace(Aj);

    OmegaBar.push_back(
        gtsam::JacobianFactor(i, Ai, i + 1, Aj, gtsam::Vector::Zero(9)));
  }
  return OmegaBar;
}

/* ------------------------------------------------------------------------ */
// Create projection factors on the Linear navigation state: position, vel,
// accBias This "emulates" an actual smart factor and marginalizes out point 1)
// Models measurement std and max distance 2) does not mode outliers, since we
// "predict" ground truth measurements
gtsam::HessianFactor::shared_ptr FeatureSelector::createLinearVisionFactor(
    const gtsam::Point3& pworld_l, const Cameras& left_cameras,
    const Cameras& right_cameras, double& debugFETime, double& debugSVDTime,
    double& debugSchurTime,  // debug
    const int keypointLife, bool hasRightPixel) const {
  // compute track lenght, depending on horizon and keypointLife
  size_t nrKeyframesInHorizon =
      std::min(left_cameras.size(), size_t(keypointLife));

  // create quantities to define hessian factor
  gtsam::FastVector<gtsam::Key> keys;
  keys.reserve(nrKeyframesInHorizon);
  std::vector<gtsam::Matrix69> FBlocks;
  FBlocks.reserve(nrKeyframesInHorizon);  // 3 + 3 = 6 measurements
  gtsam::Matrix E = gtsam::Matrix::Zero(6 * nrKeyframesInHorizon, 3);
  gtsam::Matrix69 ZeroMat69 = gtsam::Matrix69::Zero();
  gtsam::Vector b = gtsam::Vector::Zero(6 * nrKeyframesInHorizon);

  if (debug) {
    std::cout << "createLinearVisionFactor: before loop " << std::endl;
  }
  // reproject in each camera and build corresponding Jacobian

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  double startTime = UtilsOpenCV::GetTimeInSeconds();
#endif
  bool featureTrackInterrupted = false;  // unfortunately, we always populate
                                         // the data structures, even with zeros
  for (size_t c = 0; c < nrKeyframesInHorizon; c++) {
    keys.push_back(c);
    FBlocks.push_back(ZeroMat69);  // initialization
    // try to project point to left camera: if point is in FOV, add to Jacobians
    boost::optional<gtsam::Unit3> uij_left = FeatureSelector::GetVersorIfInFOV(
        left_cameras.at(c), pworld_l, landmarkDistanceThreshold_);
    if (uij_left &&
        !featureTrackInterrupted) {  // if point was in field of view
      gtsam::Matrix3 uijx_RkRcamTran =
          sqrtInfoVision_ * uij_left->skew() *
          (left_cameras.at(c)).pose().rotation().matrix().transpose();
      FBlocks.at(c).block<3, 3>(0, 0) = -uijx_RkRcamTran;
      E.block<3, 3>(6 * c, 0) = uijx_RkRcamTran;
    } else {
      featureTrackInterrupted = true;  // we lost the feature track!
    }
    if (!hasRightPixel || !useStereo_) {
      continue;
    }  // otherwise those blocks remain zeros
    // try to project point to left camera: if point is in FOV, add to Jacobians
    boost::optional<gtsam::Unit3> uij_right = FeatureSelector::GetVersorIfInFOV(
        right_cameras.at(c), pworld_l, landmarkDistanceThreshold_);
    if (uij_right &&
        !featureTrackInterrupted) {  // if point was in field of view
      gtsam::Matrix3 uijx_RkRcamTran =
          sqrtInfoVision_ * uij_right->skew() *
          (right_cameras.at(c)).pose().rotation().matrix().transpose();
      FBlocks.at(c).block<3, 3>(3, 0) = -uijx_RkRcamTran;
      E.block<3, 3>(6 * c + 3, 0) = uijx_RkRcamTran;
    }
  }
  if (debug && featureTrackInterrupted) {
    std::cout << "Feature track was broken before " << nrKeyframesInHorizon
              << " keyframes" << std::endl;
  }
  if (debug) {
    std::cout << "createLinearVisionFactor: after loop " << std::endl;
  }
#ifdef FEATURE_SELECTOR_DEBUG_COUT
  debugFETime += UtilsOpenCV::GetTimeInSeconds() - startTime;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // do Schur complement and populate Hessian factor
  gtsam::Matrix3 Pinv = E.transpose() * E;
  if (debug) {
    std::cout << "Pinv \n " << Pinv << std::endl;
  }
  int rank;
  double minSv;
  gtsam::Vector eigVector;
  boost::tie(rank, minSv, eigVector) = SmallestEigs(Pinv);
  // std::cout << " minSv " << minSv << std::endl;
  if (minSv < 1e-9)
    return boost::make_shared<gtsam::HessianFactor>();  // point cannot be
                                                        // triangulated, factor
                                                        // is useless

  gtsam::Matrix3 P = Pinv.inverse();
  if (debug) {
    std::cout << "P \n " << P << std::endl;
  }

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  debugSVDTime += UtilsOpenCV::GetTimeInSeconds() - startTime;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  // get Hessian matrix from Schur Complement (get rid of point pworld_l)
  gtsam::SymmetricBlockMatrix H = SchurComplement(FBlocks, E, P, b);

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  debugSchurTime += UtilsOpenCV::GetTimeInSeconds() - startTime;
#endif

  return boost::make_shared<gtsam::HessianFactor>(keys, H);
}

/* ------------------------------------------------------------------------ */
gtsam::GaussianFactorGraph::shared_ptr FeatureSelector::createOmegaBar(
    const FeatureSelectorData& featureSelectionData,
    const Cameras& left_cameras, const Cameras& right_cameras) const {
  // 1) add imu factors

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  double startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  if (debug) {
    std::cout << "createOmegaBar: createOmegaBar" << std::endl;
  }
  gtsam::GaussianFactorGraph OmegaBar =
      createOmegaBarImuAndPrior(featureSelectionData);
  if (testing) {
    if (OmegaBar.size() != left_cameras.size()) {
      throw std::runtime_error(
          "createOmegaBar: createOmegaBarImuAndPrior returned wrong size ");
    }
  }

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "-- createOmegaBar: createOmegaBar time: "
            << UtilsOpenCV::GetTimeInSeconds() - startTime << std::endl;
  startTime = UtilsOpenCV::GetTimeInSeconds();
#endif

  double debugFETime = 0;
  double debugSVDTime = 0;
  double debugSchurTime = 0;
  // 2) add linear vision factors for existing features
  for (size_t l = 0; l < featureSelectionData.keypoints_3d.size(); l++) {
    // 3D point in local frame of first camera
    gtsam::Point3 p_l_camL0 =
        gtsam::Point3(featureSelectionData.keypoints_3d.at(l));
    // convert to global frame (point are expressed wrt camera 0)
    gtsam::Point3 pworld_l =
        left_cameras.at(0).pose() *
        p_l_camL0;  // overload for tranform_from (converts to global frame)
    // add to graph
    gtsam::HessianFactor::shared_ptr H_l = createLinearVisionFactor(
        pworld_l, left_cameras, right_cameras, debugFETime, debugSVDTime,
        debugSchurTime, featureSelectionData.keypointLife.at(l));
    if (!H_l->empty())  // if not empty
      OmegaBar.push_back(H_l);
  }

#ifdef FEATURE_SELECTOR_DEBUG_COUT
  std::cout << "-- createOmegaBar: createLinearVisionFactor time : "
            << UtilsOpenCV::GetTimeInSeconds() - startTime
            << "(nr factors: " << featureSelectionData.keypoints_3d.size()
            << ")" << std::endl;
  std::cout << "-- -- createLinearVisionFactor: debugFETime_ " << debugFETime
            << std::endl;
  std::cout << "-- -- createLinearVisionFactor: debugSVDTime_ " << debugSVDTime
            << std::endl;
  std::cout << "-- -- createLinearVisionFactor: debugSchurTime_ "
            << debugSchurTime << std::endl;
  std::cout << "-- -- createLinearVisionFactor: sum "
            << debugFETime + debugSVDTime + debugSchurTime << std::endl;
#endif

  if (debug) {
    std::cout << "createOmegaBar: done createLinearVisionFactor" << std::endl;
  }
  gtsam::HessianFactor OmegaBar_H = gtsam::HessianFactor(
      OmegaBar);  // convert the factor graph into a single factor
  OmegaBar.resize(0);
  OmegaBar.push_back(OmegaBar_H);
  if (testing) {
    if (OmegaBar.size() != 1) {
      throw std::runtime_error(
          "createOmegaBar: createOmegaBarImuAndPrior has wrong size after "
          "hessian factor remake");
    }
  }
  return boost::make_shared<gtsam::GaussianFactorGraph>(OmegaBar);
}

/* ------------------------------------------------------------------------ */
std::vector<gtsam::HessianFactor::shared_ptr> FeatureSelector::createDeltas(
    const std::vector<gtsam::Vector3>& availableVersors,
    const std::vector<double>& availableCornersDistances,
    const FeatureSelectorData& featureSelectionData,
    const Cameras& left_cameras, const Cameras& right_cameras) const {
  // sanity check:
  if (availableVersors.size() != availableCornersDistances.size())
    throw std::runtime_error("createDeltas: distance vector size mismatch");

  std::vector<gtsam::HessianFactor::shared_ptr> Deltas;
  // for each candidate versor (keypoint)
  double tol = 0.01;  // 1 cm
  bool hasRightPixel = true;
  for (size_t l = 0; l < availableVersors.size(); l++) {
    gtsam::Vector3 versor_l = availableVersors.at(l);  // candidate versor

    double distance = availableCornersDistances.at(l);  // 0 if not available
    if (distance <
        tol) {  // if zero, i.e., if not available, we try to guess it:
      hasRightPixel =
          false;  // we guess the distance, but only predict left measurements
      distance = featureSelectionDefaultDepth_;  // start by setting to default
      for (size_t o = 0; o < featureSelectionData.keypoints_3d.size();
           o++) {  // search for a nearby known feature with valid depth
        double normo = featureSelectionData.keypoints_3d.at(o).norm();
        // if norm is nonzero and dot product = cos * depth is large enough
        if (normo > tol &&
            gtsam::dot(versor_l, featureSelectionData.keypoints_3d.at(o)) >
                featureSelectionCosineNeighborhood_ * normo) {
          // product is cos * norm, therefore the inequality asks for the sin to
          // be > featureSelectionCosineNeighborhood_
          distance = normo;  // use same depth as point o
          break;
        }
      }
    }
    if (fabs(versor_l.norm() - 1) > tol)  // not unit norm
      throw std::runtime_error("createDeltas: not unit norm versor");

    // std::cout << "versor_l " << versor_l.transpose() << std::endl;
    gtsam::Point3 p_l_camL0 = gtsam::Point3(versor_l * distance);
    // convert to global frame (point are expressed wrt camera 0)

    gtsam::Point3 pworld_l =
        left_cameras.at(0).pose() *
        p_l_camL0;  // overload for tranform_from (converts to global frame)

    // add to Deltas
    // NOTE: we always need to push_back to Deltas since order is important here
    double debugFETime = 0;
    double debugSVDTime = 0;
    double debugSchurTime = 0;
    Deltas.push_back(createLinearVisionFactor(
        pworld_l, left_cameras, right_cameras, debugFETime, debugSVDTime,
        debugSchurTime, 1e9,  // infinite life
        hasRightPixel));
  }
  return Deltas;
}
/* ------------------------------------------------------------------------ */
// check if point, expressed in world frame, is within fov of camera (fov
// defined in terms of angle and maxDistance). If so returns a versor
// corresponding to the direction to the point
boost::optional<gtsam::Unit3> FeatureSelector::GetVersorIfInFOV(
    const Camera& cam, const gtsam::Point3& pworld, const double& maxDistance) {
  // check if a point p (expressed in global frame) falls in the FOV of the
  // camera cam, in which case the function returns a unit3 corresponding to the
  // direction at which p is observed
  if (pworld.vector().norm() <= 1e-3)
    return boost::none;  // point is at zero (invalid)

  gtsam::Point2 px;
  try {
    px = cam.project(pworld);
  } catch (gtsam::CheiralityException) {
    return boost::none;  // point is behind the camera
  }
  if (px.x() < 0 || px.y() < 0 || px.x() > 2 * cam.calibration().px() ||
      px.y() > 2 * cam.calibration().py()) {
    return boost::none;  // point is outside the FOV
  }
  // point (as a vector) in camera frame
  Vector3 pcam = (cam.pose().transform_to(pworld)).vector();
  if (pcam.norm() > maxDistance) {  // beyond max distance
    return boost::none;
  }
  return gtsam::Unit3(pcam);
}
/* ------------------------------------------------------------------------ */
/**
 * Do Schur complement, given Jacobian as Fs,E,P, return SymmetricBlockMatrix
 * G = F' * F - F' * E * P * E' * F
 * g = zero
 * Fixed size version
 */
gtsam::SymmetricBlockMatrix FeatureSelector::SchurComplement(
    const FBlocks& Fs, const gtsam::Matrix& E, const gtsam::Matrix3& P,
    const gtsam::Vector& b) {
  // a single point is observed in m cameras
  size_t m = Fs.size();

  // Create a SymmetricBlockMatrix
  size_t M1 = D * m + 1;
  std::vector<gtsam::DenseIndex> dims(m + 1);  // this also includes the b term
  std::fill(dims.begin(), dims.end() - 1, D);
  dims.back() = 1;
  gtsam::SymmetricBlockMatrix augmentedHessian(dims,
                                               gtsam::Matrix::Zero(M1, M1));

  // Blockwise Schur complement
  for (size_t i = 0; i < m; i++) {  // for each camera

    const MatrixZD& Fi = Fs[i];
    const auto FiT = Fi.transpose();
    const Eigen::Matrix<double, ZDim, N> Ei_P =  //
        E.block(ZDim * i, 0, ZDim, N) * P;

    // D = (Dx2) * ZDim
    augmentedHessian.setOffDiagonalBlock(
        i, m,
        FiT * b.segment<ZDim>(ZDim * i)  // F' * b
            -
            FiT * (Ei_P *
                   (E.transpose() *
                    b)));  // D = (DxZDim) * (ZDimx3) * (N*ZDimm) * (ZDimm x 1)

    // (DxD) = (DxZDim) * ( (ZDimxD) - (ZDimx3) * (3xZDim) * (ZDimxD) )
    augmentedHessian.setDiagonalBlock(
        i, FiT * (Fi - Ei_P * E.block(ZDim * i, 0, ZDim, N).transpose() * Fi));

    // upper triangular part of the hessian
    for (size_t j = i + 1; j < m; j++) {  // for each camera
      const MatrixZD& Fj = Fs[j];

      // (DxD) = (Dx2) * ( (2x2) * (2xD) )
      augmentedHessian.setOffDiagonalBlock(
          i, j, -FiT * (Ei_P * E.block(ZDim * j, 0, ZDim, N).transpose() * Fj));
    }
  }  // end of for over cameras

  augmentedHessian.diagonalBlock(m)(0, 0) += b.squaredNorm();
  return augmentedHessian;
}

/* ------------------------------------------------------------------------ */
// Before starting feature selection: returns selected smart measurements
// (included tracked ones) and actual time it took for the selection.

std::pair<SmartStereoMeasurements, double>
FeatureSelector::splitTrackedAndNewFeatures_Select_Display(
    std::shared_ptr<StereoFrame>&
        stereoFrame_km1,  // not constant since we discard nonselected lmks
    const SmartStereoMeasurements& smartStereoMeasurements,
    const int& vio_cur_id, const int& saveImagesSelector,
    const VioFrontEndParams::FeatureSelectionCriterion& criterion,
    const int& nrFeaturesToSelect, const int& maxFeatureAge,
    const KeyframeToStampedPose& posesAtFutureKeyframes,
    const gtsam::Matrix& curr_state_cov, const std::string& dataset_name,
    const Frame& frame_km1) {
  // ToDo init to invalid value.
  gtsam::Matrix currNavStateCovariance;
  if (criterion != VioFrontEndParams::FeatureSelectionCriterion::QUALITY) {
    VLOG(100) << "Using feature selection criterion diff than QUALITY ";
    try {
      currNavStateCovariance = curr_state_cov;
    } catch (const gtsam::IndeterminantLinearSystemException& e) {
      LOG(ERROR) << "Error when calculating current state covariance.";
    }
  } else {
    VLOG(100) << "Using QUALITY as feature selection criterion";
  }

  //////////////////////////////////////////////////////////////////
  // 1) split tracked VS new features:
  //////////////////////////////////////////////////////////////////
  // tracked features (observed in more than 2 kf)
  KeypointsCV trackedKeypoints;  // only for visualization
  LandmarkIds trackedLmks;       // only for visualization
  SmartStereoMeasurements trackedSmartStereoMeasurements;
  std::vector<gtsam::Vector3> trackedkeypoints_3d;
  std::vector<size_t> trackedLandmarksAge;

  // tracked features (observed in exacly 2 kf - after 1 kf they are
  // uninformative)
  KeypointsCV newlyAvailableKeypoints;  // only for visualization
  LandmarkIds newlyAvailableLmks;       // only for visualization
  SmartStereoMeasurements newSmartStereoMeasurements;
  std::vector<double> newlyAvailableKeypointsScore;
  std::vector<double> newlyAvailableKeypointsDistance;

  for (auto ssm :
       smartStereoMeasurements) {  // for each smart stereo measurement:
    LandmarkId lmkId = ssm.first;
    StereoFrame::LandmarkInfo lmkInfo = stereoFrame_km1->getLandmarkInfo(lmkId);
    // Age can be 1, 2, ...: at age 1 we do not do anything,
    // at age 2 we consider them new measurements,
    // > 2 we consider them tracked
    if (lmkInfo.age ==
        2) {  // new feature (already observed in two consecutive frames)
      newlyAvailableLmks.push_back(lmkId);
      newlyAvailableKeypoints.push_back(lmkInfo.keypoint);
      newSmartStereoMeasurements.push_back(ssm);  // it is a new measurement.
      newlyAvailableKeypointsScore.push_back(lmkInfo.score);
      newlyAvailableKeypointsDistance.push_back(lmkInfo.keypoint_3d.norm());
    } else if (lmkInfo.age > 2) {
      // if age = 0 they are not informative for pose estimation
      trackedKeypoints.push_back(lmkInfo.keypoint);  // for visualization
      trackedLmks.push_back(lmkId);
      trackedSmartStereoMeasurements.push_back(ssm);
      trackedkeypoints_3d.push_back(lmkInfo.keypoint_3d);
      trackedLandmarksAge.push_back(lmkInfo.age);  // for feature selection
    }
  }

  VLOG(20) << "Split features into tracked and new smart measurements.";

  // Sanity check:
  if (newlyAvailableKeypointsScore.size() > 0 &&
      newlyAvailableKeypointsScore.at(0) <= 0) {
    throw std::runtime_error("stereoVioExample: max score is zero");
  }

  for (size_t ni = 1; ni < newlyAvailableKeypointsScore.size(); ++ni) {
    if (newlyAvailableKeypointsScore.at(ni - 1) <
        newlyAvailableKeypointsScore.at(ni)) {
      throw std::runtime_error(
          "stereoVioExample: scores are not in descending order");
    }
  }

  // Debugging.
  double ratioOfNewKeypointsWithDistance = 0;
  for (size_t ni = 0; ni < newlyAvailableKeypointsDistance.size(); ++ni) {
    if (newlyAvailableKeypointsDistance.at(ni) > 1e-2)
      ratioOfNewKeypointsWithDistance += 1;
  }
  if (newlyAvailableKeypointsDistance.size() != 0) {
    ratioOfNewKeypointsWithDistance =
        ratioOfNewKeypointsWithDistance /
        double(newlyAvailableKeypointsDistance.size());
  } else {
    ratioOfNewKeypointsWithDistance =
        std::numeric_limits<double>::signaling_NaN();
  }
  VLOG(10) << "ratioOfNewKeypointsWithDistance: "
           << ratioOfNewKeypointsWithDistance;
  // UtilsOpenCV::PrintVector(newlyAvailableKeypointsDistance,"newlyAvailableKeypointsDistance");

  //////////////////////////////////////////////////////////////////
  // 2) select best among the newSmartStereoMeasurements
  //////////////////////////////////////////////////////////////////
  // 2.1) figure out how many features we need!
  int need_nr_features =
      std::max(nrFeaturesToSelect - int(trackedkeypoints_3d.size()),
               int(0));  // at least zero
  VLOG(10) << "Nr features tracked: " << trackedSmartStereoMeasurements.size()
           << "\n"
           << "Nr new features needed: " << need_nr_features << "\n"
           << "Nr new features before selector: "
           << newSmartStereoMeasurements.size();

  // 2.2) run a feature selector of your choice
  KeypointsCV selectedKeypoints;  // only for visualization
  LandmarkIds selectedLmks;       // only for visualization
  std::vector<size_t> selectedIndices;
  std::vector<double> selectedGains;

  double featureSelectionTime =
      0.0;  // later saved to:
            // stereoTracker.tracker_.debugInfo_.featureSelectionTime_
  if (newSmartStereoMeasurements.size() > need_nr_features) {
    // If we have to select something.
    //////////////////////////////////////////////////////////////////////////
    if (criterion == VioFrontEndParams::FeatureSelectionCriterion::QUALITY) {
      // Features were already inserted in order of quality.
      for (size_t ii = 0; ii < need_nr_features; ii++) {
        // These are the landmarks we are going to select.
        selectedIndices.push_back(ii);
      }

      //////////////////////////////////////////////////////////////////////////
    } else if (criterion ==
               VioFrontEndParams::FeatureSelectionCriterion::RANDOM) {
      // Reshuffle.
      for (size_t ii = 0; ii < newSmartStereoMeasurements.size(); ii++) {
        // These are the landmarks we are going to select.
        selectedIndices.push_back(ii);
      }

      // Randomized seed, but fixed if we fix srand.
      unsigned seedShuffle = rand();
      // Randomize order.
      std::shuffle(selectedIndices.begin(), selectedIndices.end(),
                   std::default_random_engine(seedShuffle));
      // Take only first need_nr_features.
      selectedIndices.resize(need_nr_features);
      //////////////////////////////////////////////////////////////////////////
      // Use our more clever feature selection algorithms:
    } else {
      FeatureSelectorData featureSelectionData;
      // Covariance of current state.
      featureSelectionData.currentNavStateCovariance = currNavStateCovariance;
      featureSelectionData.posesAtFutureKeyframes = posesAtFutureKeyframes;

      // ----------------- DATA ABOUT FEATURES WE ARE TRACKING ------------ //
      VLOG(20) << "Selector: populating data about existing feature tracks.";
      // Current 3D points.
      featureSelectionData.keypoints_3d = trackedkeypoints_3d;
      featureSelectionData.keypointLife.reserve(trackedLandmarksAge.size());
      for (const int& age : trackedLandmarksAge) {
        // Compute age as maxFeatureAge_ - current age.
        // This is life.
        featureSelectionData.keypointLife.push_back(maxFeatureAge - age);
      }

      if (featureSelectionData.keypoints_3d.size() !=
          featureSelectionData.keypointLife.size())
        throw std::runtime_error(
            "stereoVioExample: keypoint age inconsistent with keypoint 3D");
      featureSelectionData.body_P_leftCam = stereoFrame_km1->getBPoseCamLRect();
      featureSelectionData
          .body_P_rightCam =  // rectified right camera only has a translation
                              // along x = baseline
          stereoFrame_km1->getBPoseCamLRect().compose(gtsam::Pose3(
              gtsam::Rot3(),
              gtsam::Point3(stereoFrame_km1->getBaseline(), 0.0, 0.0)));
      featureSelectionData.left_undistRectCameraMatrix =
          stereoFrame_km1->getLeftUndistRectCamMat();
      featureSelectionData.right_undistRectCameraMatrix =
          stereoFrame_km1->getRightUndistRectCamMat();
      // ------------------ DATA ABOUT NEW FEATURES: ----------------- //
      std::cout << "selector: populating data about new feature tracks"
                << std::endl;
      KeypointsCV corners;
      std::vector<double> successProbabilities;
      if (newlyAvailableKeypointsScore.size() !=
          newSmartStereoMeasurements.size())
        throw std::runtime_error("stereoVioExample: wrong score vector size");

      for (size_t ni = 0; ni < newSmartStereoMeasurements.size(); ++ni) {
        corners.push_back(
            KeypointCV(newSmartStereoMeasurements.at(ni).second.uL(),
                       newSmartStereoMeasurements.at(ni)
                           .second.v()));  // undistorted, rectified
        successProbabilities.push_back(
            newlyAvailableKeypointsScore.at(ni) /
            newlyAvailableKeypointsScore.at(
                0));  // normalized wrt largest (starting from 1 and decreasing)
      }
      UtilsOpenCV::PrintVector<double>(successProbabilities,
                                       "successProbabilities");

      // featureSelectionData.print();
      const gtsam::Cal3_S2& K = stereoFrame_km1->getLeftUndistRectCamMat();
      CameraParams cam_param;
      cam_param.calibration_ =
          gtsam::Cal3DS2(K.fx(), K.fy(), 0.0, K.px(), K.py(), 0.0, 0.0);
      cam_param.camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
      cam_param.camera_matrix_.at<double>(0, 0) = K.fx();
      cam_param.camera_matrix_.at<double>(1, 1) = K.fy();
      cam_param.camera_matrix_.at<double>(0, 2) = K.px();
      cam_param.camera_matrix_.at<double>(1, 2) = K.py();
      cam_param.distortion_coeff_ =
          cv::Mat::zeros(1, 5, CV_64F);  // undistorted points

      std::cout << "selector: calling selector" << std::endl;
      double startTime = UtilsOpenCV::GetTimeInSeconds();
      std::tie(corners, selectedIndices, selectedGains) =
          featureSelectionLinearModel(
              corners,                          // undistorted and rectified
              successProbabilities,             // in [0,1] for each corner
              newlyAvailableKeypointsDistance,  // 0 if not available
              cam_param,  // note: corners are undistorted and rectified
              need_nr_features, featureSelectionData, criterion);
      featureSelectionTime = UtilsOpenCV::GetTimeInSeconds() - startTime;
      UtilsOpenCV::PrintVector<size_t>(selectedIndices, "selectedIndices");
      UtilsOpenCV::PrintVector<double>(selectedGains, "selectedGains");
    }

    //////////////////////////////////////////////////////////////////////////
    // 3) POPULATE NEW SMART STEREO MEASUREMENTS
    //////////////////////////////////////////////////////////////////////////
    SmartStereoMeasurements tmp = newSmartStereoMeasurements;
    // Delete everything and repopulate.
    newSmartStereoMeasurements.resize(0);
    // Preallocate.
    newSmartStereoMeasurements.reserve(selectedIndices.size());
    for (const auto& i : selectedIndices) {
      newSmartStereoMeasurements.push_back(tmp.at(i));
      selectedKeypoints.push_back(newlyAvailableKeypoints.at(i));
      selectedLmks.push_back(newlyAvailableLmks.at(i));
    }

    VLOG(20) << "Selector: populated SmartStereoMeasurements.";

    //////////////////////////////////////////////////////////////////
    // 4) REMOVE NONSELECTED FEATURES FROM FRAME
    //////////////////////////////////////////////////////////////////
    LandmarkIds discardedLmks;
    std::vector<size_t> nonSelectedIndices;
    // All new measurements.
    for (size_t ii = 0; ii < tmp.size(); ii++) {
      if (std::find(selectedIndices.begin(), selectedIndices.end(), ii) ==
          selectedIndices.end()) {
        // ii was not selected.
        nonSelectedIndices.push_back(ii);
        const LandmarkId& lmkId = tmp.at(ii).first;
        discardedLmks.push_back(lmkId);
      }
    }
    // UtilsOpenCV::PrintVector<size_t>(nonSelectedIndices,"nonSelectedIndices");
    // UtilsOpenCV::PrintVector<LandmarkId>(discardedLmks,"discardedLmks");

    stereoFrame_km1->getLeftFrameMutable()->setLandmarksToMinus1(discardedLmks);

    // Sanity check:
    if (discardedLmks.size() + newSmartStereoMeasurements.size() !=
        tmp.size()) {
      throw std::runtime_error("stereoVIOexample: wrong feature selection");
    }
  } else {
    // We have less features than what we need:
    selectedKeypoints = newlyAvailableKeypoints;
    selectedLmks = newlyAvailableLmks;
  }

  VLOG(20) << "Nr new features after selector: "
           << newSmartStereoMeasurements.size();

  // Put only the selected features in statusSmartStereoMeasurements.
  SmartStereoMeasurements trackedAndSelectedSmartStereoMeasurements =
      trackedSmartStereoMeasurements;
  trackedAndSelectedSmartStereoMeasurements.insert(
      trackedAndSelectedSmartStereoMeasurements.end(),
      newSmartStereoMeasurements.begin(), newSmartStereoMeasurements.end());
  VLOG(20) << "Nr features tracked & selected "
           << trackedAndSelectedSmartStereoMeasurements.size();

  //////////////////////////////////////////////////////////////////
  // 5) DISPLAY FEATURE SELECTION:
  //////////////////////////////////////////////////////////////////
  static constexpr bool visualize_selection_results = false;
  if (visualize_selection_results) {
    static constexpr int remId = 1000;  // landmark ids are visualized modulo
                                        // this number to improve visualization
    cv::Mat img = stereoFrame_km1->getLeftFrame().img_.clone();
    // UtilsOpenCV::DrawCrossesInPlace(img, newlyAvailableKeypoints,
    // cv::Scalar(0, 0, 255),0.4,newlyAvailableLmks,remId);
    // UtilsOpenCV::DrawCirclesInPlace(img, selectedKeypoints, cv::Scalar(0,
    // 255, 255), 4,selectedLmks,remId); UtilsOpenCV::DrawSquaresInPlace(img,
    // trackedKeypoints, cv::Scalar(0, 255, 0),10,trackedLmks,remId);
    std::vector<int> emptyVect;
    UtilsOpenCV::DrawCrossesInPlace(img, newlyAvailableKeypoints,
                                    cv::Scalar(0, 0, 255), 0.4, emptyVect,
                                    remId);
    UtilsOpenCV::DrawCirclesInPlace(
        img, selectedKeypoints, cv::Scalar(0, 255, 255), 4, emptyVect, remId);
    UtilsOpenCV::DrawSquaresInPlace(
        img, trackedKeypoints, cv::Scalar(0, 255, 0), 10, emptyVect, remId);
    for (size_t ii = 0; ii < trackedLmks.size(); ii++) {
      KeypointCV px_cur = trackedKeypoints.at(ii);
      KeypointCV px_ref;
      for (size_t jj = 0; jj < frame_km1.landmarks_.size(); jj++) {
        // Current landmark in previous frame.
        if (frame_km1.landmarks_.at(jj) == trackedLmks.at(ii)) {
          px_ref = frame_km1.keypoints_.at(jj);
          break;
        }
        continue;  // if we did not find the point, we skip the line
      }
      cv::line(img, px_cur, px_ref, cv::Scalar(0, 255, 0), 1);
    }

    // Plot text with keyframe id.
    cv::putText(img,
                "kf:" + std::to_string(vio_cur_id) +
                    " #tracked:" + std::to_string(trackedLmks.size()) +
                    " #selected:" + std::to_string(selectedLmks.size()),
                KeypointCV(10, 15), CV_FONT_HERSHEY_COMPLEX, 0.4,
                cv::Scalar(0, 255, 255));

    VLOG(20) << "trackedKeypoints: " << trackedKeypoints.size()
             << " newlyAvailableKeypoints " << newlyAvailableKeypoints.size()
             << " selectedKeypoints " << selectedKeypoints.size();

    cv::imshow("Selection results", img);

    if (saveImagesSelector == 2) {
      // Create output folders:
      std::string folderName =
          "./result-FeatureSelection-" + dataset_name + "-" +
          VioFrontEndParams::FeatureSelectionCriterionStr(criterion) + "/";
      boost::filesystem::path stereoTrackerDir(folderName.c_str());
      boost::filesystem::create_directory(stereoTrackerDir);
      // Write image.
      std::string img_name =
          folderName + "img_" + std::to_string(vio_cur_id) + ".png";
      cv::imwrite(img_name, img);

      // Create output folders:
      folderName = "./result-original-" + dataset_name + "-" +
                   VioFrontEndParams::FeatureSelectionCriterionStr(criterion) +
                   "/";
      boost::filesystem::path originalImgDir(folderName.c_str());
      boost::filesystem::create_directory(originalImgDir);

      // Write image.
      img_name = folderName + "img_" + std::to_string(vio_cur_id) + ".png";
      cv::imwrite(img_name, stereoFrame_km1->getLeftFrame().img_);
    }
  }

  return std::make_pair(trackedAndSelectedSmartStereoMeasurements,
                        featureSelectionTime);
}

}  // namespace VIO
