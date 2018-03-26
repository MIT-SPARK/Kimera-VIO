/**
 * @file OdometryExample.cpp
 * @brief Simple robot motion example, with prior and two odometry measurements
 * @author Frank Dellaert
 */

/**
 * Example of a simple 2D localization example
 *  - Robot poses are facing along the X axis (horizontal, to the right in 2D)
 *  - The robot moves 2 meters each step
 *  - We have full odometry between poses
 */

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/OrientedPlane3.h>

// In GTSAM, measurement functions are represented as 'factors'. Several common factors
// have been provided with the library for solving robotics/SLAM/Bundle Adjustment problems.
// Here we will use Between factors for the relative motion described by odometry measurements.
// Also, we will initialize the robot at the origin using a Prior factor.
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>

// Factor between a point landmark and a plane.
#include "factors/RegularPlane3Factor.h"

// When the factors are created, we will add them to a Factor Graph. As the factors we are using
// are nonlinear factors, we will need a Nonlinear Factor Graph.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

using namespace std;
using namespace gtsam;

int main(int argc, char** argv) {

  // Create an empty nonlinear factor graph
  NonlinearFactorGraph graph;

  // Add a prior on the first point, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)
  Point3 priorMean1(0.0, 0.0, 1.0); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  graph.emplace_shared<PriorFactor<Point3> >(1, priorMean1, priorNoise);
  Point3 priorMean2(1.0, 0.0, 1.0); // prior at origin
  graph.emplace_shared<PriorFactor<Point3> >(2, priorMean2, priorNoise);
  Point3 priorMean3(0.0, 1.0, 1.0); // prior at origin
  graph.emplace_shared<PriorFactor<Point3> >(3, priorMean3, priorNoise);

  //OrientedPlane3 priorMeanPlane(0.0, 0.0, 1.0, 1.0); // prior at origin
  //noiseModel::Diagonal::shared_ptr priorNoisePlane = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
  //graph.emplace_shared<PriorFactor<OrientedPlane3> >(4, priorMeanPlane, priorNoisePlane);

  // For simplicity, we will use the same noise model for each factor
  noiseModel::Diagonal::shared_ptr regularityNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1));
  // Notice there is no measurement argument!
  //graph.emplace_shared<RegularPlane3Factor>(1, 2, 3, 4, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(1, 4, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(2, 4, regularityNoise);
  graph.emplace_shared<BasicRegularPlane3Factor>(3, 4, regularityNoise);

  graph.print("\nFactor Graph:\n"); // print

  // Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initial;
  initial.insert(1, Point3(0.0, 0.0, 3.0));
  initial.insert(2, Point3(1.0, 2.0, 2.0));
  initial.insert(3, Point3(0.3, 1.0, 8.0));
  initial.insert(4, OrientedPlane3(0.0, 1, 0.0, 0));
  initial.print("\nInitial Estimate:\n"); // print

  // Optimize
  //ISAM2 isam2;
  //ISAM2Result result = isam2.update(graph, initial);
  //Values result_values = isam2.calculateEstimate();
  //result_values.print("Final Result:\n");
  //OrientedPlane3 optimized_plane_landmark = result_values.at<OrientedPlane3>(
  //    lm_sym);

  // optimize using Levenberg-Marquardt optimization
  Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
  result.print("Final Result:\n");

  // Calculate and print marginal covariances for all variables
  //cout.precision(2);
  //Marginals marginals(graph, result_values);
  //cout << "l1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  //cout << "l2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  //cout << "l3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  //cout << "p4 covariance:\n" << marginals.marginalCovariance(4) << endl;

  return 0;
}
