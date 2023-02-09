/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.cpp
 * @brief  Build and visualize 3D mesh from 2D mesh
 * @author Antoni Rosinol
 */

#include "kimera-vio/mesh/Mesher.h"

#include <utility>  // for make_pair
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <math.h>
#include <algorithm>
#include <opencv2/imgproc.hpp>

// For serialization of meshes
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
#include "kimera-vio/utils/UtilsNumerical.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <triangle/triangle.h>

#ifdef __cplusplus
}
#endif

// General functionality for the mesher.
DEFINE_bool(add_extra_lmks_from_stereo,
            false,
            "Add extra landmarks that are stereo triangulated to the mesh. "
            "WARNING this is computationally expensive.");
DEFINE_bool(reduce_mesh_to_time_horizon,
            true,
            "Reduce mesh vertices to the "
            "landmarks available in current optimization's time horizon.");
DEFINE_bool(compute_per_vertex_normals,
            false,
            "Compute per-vertex normals,"
            "this is for visualization in RVIZ, it is costly!");

// Mesh 2D return, for semantic segmentation.
// TODO REMOVE THIS FLAG MAKE MESH_2D Optional!
DEFINE_bool(return_mesh_2d,
            false,
            "Return mesh 2d with pixel positions, i.e."
            " for semantic segmentation");

// Visualization.
DEFINE_bool(visualize_histogram_1D, false, "Visualize 1D histogram.");
DEFINE_bool(log_histogram_1D,
            false,
            "Log 1D histogram to file."
            " It logs the raw and smoothed histogram.");
DEFINE_bool(visualize_histogram_2D, false, "Visualize 2D histogram.");
DEFINE_bool(log_histogram_2D,
            false,
            "Log 2D histogram to file."
            " It logs the raw and smoothed histogram.");

// Mesh filters.
DEFINE_double(min_ratio_btw_largest_smallest_side,
              0.5,
              "Minimum ratio between largest and smallest "
              "side of a triangle.");  // TODO: this check should be improved
DEFINE_double(min_elongation_ratio,
              0.5,
              "Minimum allowed elongation "
              "ratio for a triangle.");  // TODO: this check should be improved
DEFINE_double(max_triangle_side,
              0.5,
              "Maximum allowed side for "
              "a triangle.");

// Association.
DEFINE_double(normal_tolerance_polygon_plane_association,
              0.011,
              "Tolerance for a polygon's normal and a plane's normal to be "
              "considered equal (0.087 === 10 deg. aperture).");
DEFINE_double(distance_tolerance_polygon_plane_association,
              0.10,
              "Tolerance for a polygon vertices to be considered close to a "
              "plane.");
DEFINE_double(normal_tolerance_plane_plane_association,
              0.011,
              "Normal tolerance for a plane to be associated to another plane "
              "(0.087 === 10 deg. aperture).");
DEFINE_double(distance_tolerance_plane_plane_association,
              0.20,
              "Distance tolerance for a plane to be associated to another "
              "plane.");
DEFINE_bool(do_double_association,
            true,
            "Do double plane association of Backend plane with multiple "
            "segmented planes. Otherwise search for another possible "
            "Backend plane for the segmented plane.");
DEFINE_bool(only_associate_a_polygon_to_a_single_plane,
            true,
            "Limit association of a particular polygon to a single plane. "
            "Otherwise, a polygon can be associated to different planes "
            " depending on the tolerance given by the thresholds set for "
            "association.");

// Segmentation.
DEFINE_double(normal_tolerance_horizontal_surface,
              0.011,
              "Normal tolerance for a polygon to be considered parallel to the "
              "ground (0.087 === 10 deg. aperture).");
DEFINE_double(normal_tolerance_walls,
              0.0165,
              "Normal tolerance for a polygon to be considered perpendicular to"
              " the vertical direction.");
DEFINE_bool(only_use_non_clustered_points,
            true,
            "Only use points that have not been clustered in a plane already "
            "when filling both histograms.");

// Histogram 2D.
DEFINE_int32(hist_2d_gaussian_kernel_size,
             3,
             "Kernel size for gaussian blur of 2D histogram.");
DEFINE_int32(hist_2d_nr_of_local_max,
             2,
             "Number of local maximums to extract in 2D histogram.");
DEFINE_int32(hist_2d_min_support,
             20,
             "Minimum number of votes to consider a local maximum in 2D "
             "histogram a valid peak.");
DEFINE_int32(
    hist_2d_min_dist_btw_local_max,
    5,
    "Minimum distance between local maximums to be considered different.");
DEFINE_int32(hist_2d_theta_bins, 40, ".");
DEFINE_int32(hist_2d_distance_bins, 40, ".");
DEFINE_double(hist_2d_theta_range_min, 0, ".");
DEFINE_double(hist_2d_theta_range_max, M_PI, ".");
DEFINE_double(hist_2d_distance_range_min, -6.0, ".");
DEFINE_double(hist_2d_distance_range_max, 6.0, ".");

// Z histogram.
DEFINE_int32(z_histogram_bins, 512, "Number of bins for z histogram.");
DEFINE_double(z_histogram_min_range, -0.75, "Minimum z value for z histogram.");
DEFINE_double(z_histogram_max_range, 3.0, "Maximum z value for z histogram.");
DEFINE_int32(z_histogram_window_size,
             3,
             "Window size of z histogram to "
             "calculate derivatives, not sure in fact.");
DEFINE_double(z_histogram_peak_per,
              0.5,
              "Extra peaks in the z histogram will be only considered if it "
              "has a value of peak_per (< 1) times the value of the max peak"
              " in the histogram.");
DEFINE_double(z_histogram_min_support,
              50,
              "Minimum number of votes for a value in the z histogram to be "
              "considered a peak.");
DEFINE_double(z_histogram_min_separation,
              0.1,
              "If two peaks in the z histogram lie within min_separation "
              ", only the one with maximum support will be taken "
              "(sisable by setting < 0).");
DEFINE_int32(z_histogram_gaussian_kernel_size,
             5,
             "Kernel size for gaussian blur of z histogram (should be odd).");
DEFINE_int32(z_histogram_max_number_of_peaks_to_select,
             3,
             "Maximum number of peaks to select in z histogram.");

namespace VIO {

/* -------------------------------------------------------------------------- */
Mesher::Mesher(const MesherParams& mesher_params, const bool& serialize_meshes)
    : mesh_2d_(),
      mesh_3d_(),
      mesher_params_(mesher_params),
      mesher_logger_(nullptr),
      serialize_meshes_(serialize_meshes) {
  mesher_logger_ = VIO::make_unique<MesherLogger>();

  // Create z histogram.
  std::vector<int> hist_size = {FLAGS_z_histogram_bins};
  // We cannot use an array of doubles here bcs the function cv::calcHist asks
  // for floats ... It is not templated.
  std::array<float, 2> z_range = {
      static_cast<float>(FLAGS_z_histogram_min_range),
      static_cast<float>(FLAGS_z_histogram_max_range)};
  std::vector<std::array<float, 2>> ranges = {z_range};
  std::vector<int> channels = {0};
  z_hist_ =
      Histogram(1, channels, cv::Mat(), 1, hist_size, ranges, true, false);

  // Create 2d histogram.
  std::vector<int> hist_2d_size = {FLAGS_hist_2d_theta_bins,
                                   FLAGS_hist_2d_distance_bins};
  std::array<float, 2> theta_range = {
      static_cast<float>(FLAGS_hist_2d_theta_range_min),
      static_cast<float>(FLAGS_hist_2d_theta_range_max)};
  std::array<float, 2> distance_range = {
      static_cast<float>(FLAGS_hist_2d_distance_range_min),
      static_cast<float>(FLAGS_hist_2d_distance_range_max)};
  std::vector<std::array<float, 2>> ranges_2d = {theta_range, distance_range};
  std::vector<int> channels_2d = {0, 1};
  hist_2d_ = Histogram(
      1, channels_2d, cv::Mat(), 2, hist_2d_size, ranges_2d, true, false);
}

MesherOutput::UniquePtr Mesher::spinOnce(const MesherInput& input) {
  MesherOutput::UniquePtr mesher_output_payload =
      VIO::make_unique<MesherOutput>(input.timestamp_);
  updateMesh3D(
      input,
      // TODO REMOVE THIS FLAG MAKE MESH_2D Optional!
      FLAGS_return_mesh_2d ? &(mesher_output_payload->mesh_2d_) : nullptr,
      // These are more or less
      // the same info as mesh_2d_
      &(mesher_output_payload->mesh_2d_for_viz_));
  // Serialize 2D/3D Mesh if requested
  if (serialize_meshes_) {
    LOG_FIRST_N(WARNING, 1) << "Mesh serialization enabled.";
    serializeMeshes();
  }
  // TODO(Toni): remove these calls, since all info is in mesh_3d_...
  getVerticesMesh(&(mesher_output_payload->vertices_mesh_));
  getPolygonsMesh(&(mesher_output_payload->polygons_mesh_));
  mesher_output_payload->mesh_3d_ = mesh_3d_;
  return mesher_output_payload;
}

std::vector<cv::Vec6f> Mesher::computeDelaunayTriangulation(
    const KeypointsCV& keypoints,
    MeshIndices* vtx_indices) {
  // input/output structure for triangulation
  struct triangulateio in, out;
  int32_t k;

  // Input number of points and point list
  in.numberofpoints = keypoints.size();
  in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));
  k = 0;
  for (int32_t i = 0; i < static_cast<int32_t>(keypoints.size()); i++) {
    in.pointlist[k++] = keypoints[i].x;
    in.pointlist[k++] = keypoints[i].y;
  }
  in.numberofpointattributes = 0;
  in.pointattributelist = NULL;
  in.pointmarkerlist = NULL;
  in.numberofsegments = 0;
  in.numberofholes = 0;
  in.numberofregions = 0;
  in.regionlist = NULL;

  // outputs
  out.pointlist = NULL;
  out.pointattributelist = NULL;
  out.pointmarkerlist = NULL;
  out.trianglelist = NULL;
  out.triangleattributelist = NULL;
  out.neighborlist = NULL;
  out.segmentlist = NULL;
  out.segmentmarkerlist = NULL;
  out.edgelist = NULL;
  out.edgemarkerlist = NULL;

  // do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary
  // markers)
  char parameters[] = "zneQB";
  ::triangulate(parameters, &in, &out, NULL);

  // put resulting triangles into vector tri
  // triangle structure is an array that holds the triangles 3 corners
  // Stores one point and the remainder in a counterclockwise order
  std::vector<cv::Vec6f> tri;
  k = 0;
  TriVtxIndices tri_vtx_indices;
  for (int32_t i = 0; i < out.numberoftriangles; i++) {
    // Find vertex ids!
    if (vtx_indices) {
      tri_vtx_indices[0] = out.trianglelist[k];
      tri_vtx_indices[1] = out.trianglelist[k + 1];
      tri_vtx_indices[2] = out.trianglelist[k + 2];
      vtx_indices->push_back(tri_vtx_indices);
    }
    tri.push_back(cv::Vec6f(keypoints[out.trianglelist[k]].x,
                            keypoints[out.trianglelist[k]].y,
                            keypoints[out.trianglelist[k + 1]].x,
                            keypoints[out.trianglelist[k + 1]].y,
                            keypoints[out.trianglelist[k + 2]].x,
                            keypoints[out.trianglelist[k + 2]].y));
    k += 3;
  }

  // free memory used for triangulation
  free(in.pointlist);
  free(out.pointlist);
  free(out.trianglelist);

  // return triangles
  return tri;
}

/* -------------------------------------------------------------------------- */
// For a triangle defined by the 3d points p1, p2, and p3
// compute ratio between largest side and smallest side (how elongated it is).
double Mesher::getRatioBetweenSmallestAndLargestSide(
    const double& d12,
    const double& d23,
    const double& d31,
    boost::optional<double&> minSide_out,
    boost::optional<double&> maxSide_out) const {
  // Measure sides.
  double minSide = std::min(d12, std::min(d23, d31));
  double maxSide = std::max(d12, std::max(d23, d31));

  if (minSide_out && maxSide_out) {
    *minSide_out = minSide;
    *maxSide_out = maxSide;
  }

  // Compute and return ratio.
  return minSide / maxSide;
}

/* -------------------------------------------------------------------------- */
// TODO this only works for current points in the current frame!!!
// Not for the landmarks in time horizon, since they can be behind the camera!!!
// for a triangle defined by the 3d points mapPoints3d_.at(rowId_pt1),
// mapPoints3d_.at(rowId_pt2), mapPoints3d_.at(rowId_pt3), compute ratio between
// largest side and smallest side (how elongated it is)
double Mesher::getRatioBetweenTangentialAndRadialDisplacement(
    const Vertex3D& p1,
    const Vertex3D& p2,
    const Vertex3D& p3,
    const gtsam::Pose3& leftCameraPose) const {
  std::vector<gtsam::Point3> points;

  // get 3D points
  gtsam::Point3 p1_C = gtsam::Point3(static_cast<double>(p1.x),
                                     static_cast<double>(p1.y),
                                     static_cast<double>(p1.z));
  points.push_back(
      leftCameraPose.transformTo(p1_C));  // checks elongation in *camera frame*

  gtsam::Point3 p2_C = gtsam::Point3(static_cast<double>(p2.x),
                                     static_cast<double>(p2.y),
                                     static_cast<double>(p2.z));
  points.push_back(
      leftCameraPose.transformTo(p2_C));  // checks elongation in *camera frame*

  gtsam::Point3 p3_C = gtsam::Point3(static_cast<double>(p3.x),
                                     static_cast<double>(p3.y),
                                     static_cast<double>(p3.z));
  points.push_back(
      leftCameraPose.transformTo(p3_C));  // checks elongation in *camera frame*

  return UtilsGeometry::getRatioBetweenTangentialAndRadialDisplacement(points);
}

/* -------------------------------------------------------------------------- */
// Try to reject bad triangles, corresponding to outliers
// TODO filter out bad triangle without s, and use it in reduce Mesh.
// TODO filter before and not using the mesh itself because there are lmks
// that might not be seen in the current frame!
void Mesher::filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                                   double minRatioBetweenLargestAnSmallestSide,
                                   double min_elongation_ratio,
                                   double maxTriangleSide) {
  Mesh3D mesh_output;

  // Loop over each face in the mesh.
  Mesh3D::Polygon polygon;

  for (size_t i = 0; i < mesh_3d_.getNumberOfPolygons(); i++) {
    CHECK(mesh_3d_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), 3) << "Expecting 3 vertices in triangle";
    // Check if triangle is good.
    if (!isBadTriangle(polygon,
                       leftCameraPose,
                       minRatioBetweenLargestAnSmallestSide,
                       min_elongation_ratio,
                       maxTriangleSide)) {
      mesh_output.addPolygonToMesh(polygon);
    }
  }

  mesh_3d_ = mesh_output;
}

/* -------------------------------------------------------------------------- */
// Try to reject bad triangles, corresponding to outliers.
bool Mesher::isBadTriangle(
    const Mesh3D::Polygon& polygon,
    const gtsam::Pose3& left_camera_pose,
    const double& min_ratio_between_largest_an_smallest_side,
    const double& min_elongation_ratio,
    const double& max_triangle_side) const {
  CHECK_EQ(polygon.size(), 3) << "Expecting 3 vertices in triangle";
  const Vertex3D& p1 = polygon.at(0).getVertexPosition();
  const Vertex3D& p2 = polygon.at(1).getVertexPosition();
  const Vertex3D& p3 = polygon.at(2).getVertexPosition();

  double ratioSides_i = 0;
  double ratioTangentialRadial_i = 0;
  double maxTriangleSide_i = 0;

  // Check geometric dimensions.
  // Measure sides.
  double d12 = cv::norm(p1 - p2);
  double d23 = cv::norm(p2 - p3);
  double d31 = cv::norm(p3 - p1);

  // If threshold is disabled, avoid computation.
  if (min_ratio_between_largest_an_smallest_side > 0.0) {
    ratioSides_i = getRatioBetweenSmallestAndLargestSide(d12, d23, d31);
  }

  // If threshold is disabled, avoid computation.
  if (min_elongation_ratio > 0.0) {
    ratioTangentialRadial_i = getRatioBetweenTangentialAndRadialDisplacement(
        p1, p2, p3, left_camera_pose);
  }

  // If threshold is disabled, avoid computation.
  if (max_triangle_side > 0.0) {
    std::array<double, 3> sidesLen;
    sidesLen.at(0) = d12;
    sidesLen.at(1) = d23;
    sidesLen.at(2) = d31;
    const auto& it = std::max_element(sidesLen.begin(), sidesLen.end());
    DCHECK(it != sidesLen.end());
    maxTriangleSide_i = *it;
  }

  // Check if triangle is not elongated.
  if ((ratioSides_i >= min_ratio_between_largest_an_smallest_side) &&
      (ratioTangentialRadial_i >= min_elongation_ratio) &&
      (maxTriangleSide_i <= max_triangle_side)) {
    return false;
  } else {
    return true;
  }
}

/* -------------------------------------------------------------------------- */
// Create a 3D mesh from 2D corners in an image, keeps the mesh in time horizon.
// Optionally returns the 2D mesh that links with the 3D mesh via the
// landmarks ids.
void Mesher::populate3dMeshTimeHorizon(
    // cv::Vec6f assumes triangular mesh.
    const std::vector<cv::Vec6f>& mesh_2d_pixels,
    const PointsWithIdMap& points_with_id_map,
    const KeypointsCV& keypoints,
    const LandmarkIds& landmarks,
    const gtsam::Pose3& left_cam_pose,
    double min_ratio_largest_smallest_side,
    double min_elongation_ratio,
    double max_triangle_side,
    Mesh2D* mesh_2d) {
  VLOG(10) << "Starting populate3dMeshTimeHorizon...";
  VLOG(10) << "Starting populate3dMesh...";
  populate3dMesh(mesh_2d_pixels,
                 points_with_id_map,
                 keypoints,
                 landmarks,
                 left_cam_pose,
                 min_ratio_largest_smallest_side,
                 min_elongation_ratio,
                 max_triangle_side,
                 mesh_2d);
  VLOG(10) << "Finished populate3dMesh.";
  // Remove faces in the mesh that have vertices which are not in
  // points_with_id_map anymore.
  VLOG(10) << "Starting updatePolygonMeshToTimeHorizon...";
  updatePolygonMeshToTimeHorizon(points_with_id_map,
                                 left_cam_pose,
                                 min_ratio_largest_smallest_side,
                                 max_triangle_side,
                                 FLAGS_reduce_mesh_to_time_horizon);
  VLOG(10) << "Finished updatePolygonMeshToTimeHorizon.";
  VLOG(10) << "Finished populate3dMeshTimeHorizon.";
}

/* -------------------------------------------------------------------------- */
// Create a 3D mesh from 2D corners in an image.
void Mesher::populate3dMesh(const std::vector<cv::Vec6f>& mesh_2d_pixels,
                            const PointsWithIdMap& points_with_id_map,
                            const KeypointsCV& keypoints,
                            const LandmarkIds& landmarks,
                            const gtsam::Pose3& left_cam_pose,
                            double min_ratio_largest_smallest_side,
                            double min_elongation_ratio,
                            double max_triangle_side,
                            Mesh2D* mesh_2d) {
  // Iterate over each face in the 2d mesh, and generate the 3d mesh.
  // TODO to retrieve lmk id from pixels, do it in the stereo frame! not here.

  LOG_IF(WARNING, points_with_id_map.size() == 0u)
      << "Missing landmark information for the Mesher: "
         "cannot generate 3D Mesh.";

  // Create face and add it to the 2d mesh.
  Mesh2D::Polygon face;
  face.resize(3);

  // Clean output 2d mesh.
  if (mesh_2d != nullptr) mesh_2d->clearMesh();

  // Create polygon and add it to the 3d mesh.
  Mesh3D::Polygon polygon;
  polygon.resize(3);

  // Iterate over the 2d mesh triangles.
  for (size_t i = 0u; i < mesh_2d_pixels.size(); i++) {
    const cv::Vec6f& triangle_2d = mesh_2d_pixels.at(i);

    // Iterate over each vertex (pixel) of the triangle.
    // Triangle_2d.rows = 3.
    for (size_t j = 0u; j < triangle_2d.rows / 2u; j++) {
      // Extract pixel.
      const cv::Point2f pixel(triangle_2d[j * 2u], triangle_2d[j * 2u + 1u]);

      // Extract landmark id corresponding to this pixel.
      const LandmarkId& lmk_id(
          Frame::findLmkIdFromPixel(pixel, keypoints, landmarks));
      if (lmk_id == -1) {
        // CHECK_NE(lmk_id, -1) << "Could not find lmk_id: " << lmk_id
        //  << " for pixel: " << pixel << " in keypoints:\n "
        //  << keypoints;
        LOG(ERROR) << "Could not find lmk_id: " << lmk_id
                   << " for pixel: " << pixel;
        break;
      }

      // Try to find this landmark id in points_with_id_map.
      const auto& lmk_it = points_with_id_map.find(lmk_id);
      if (lmk_it != points_with_id_map.end()) {
        // We found the landmark.
        // Extract 3D position of the landmark.
        const gtsam::Point3& point(lmk_it->second);
        cv::Point3f lmk(static_cast<float>(point.x()),
                        static_cast<float>(point.y()),
                        static_cast<float>(point.z()));
        // Add landmark as one of the vertices of the current polygon in 3D.
        DCHECK_LT(j, polygon.size());
        polygon.at(j) = Mesh3D::VertexType(lmk_id, lmk);
        if (mesh_2d != nullptr) {
          face.at(j) = Mesh2D::VertexType(lmk_id, pixel);
        }
        static const size_t loop_end = triangle_2d.rows / 2u - 1u;
        if (j == loop_end) {
          // Last iteration.
          // Filter out bad polygons.
          if (!isBadTriangle(polygon,
                             left_cam_pose,
                             min_ratio_largest_smallest_side,
                             min_elongation_ratio,
                             max_triangle_side)) {
            // Save the valid triangular polygon, since it has all vertices in
            // points_with_id_map.
            mesh_3d_.addPolygonToMesh(polygon);
            if (mesh_2d != nullptr) {
              mesh_2d->addPolygonToMesh(face);
            }
          }
        }
      } else {
        // Do not save current polygon, since it has at least one vertex that
        // is not in points_with_id_map.
        LOG(ERROR) << "Landmark with id : " << lmk_id
                   << ", could not be found in points_with_id_map. "
                   << "But it should have been.\n";
        break;
      }
    }
  }
}

/* -------------------------------------------------------------------------- */
// TODO the polygon_mesh has repeated faces...
// And this seems to slow down quite a bit the for loop!
void Mesher::updatePolygonMeshToTimeHorizon(
    const PointsWithIdMap& points_with_id_map,
    const gtsam::Pose3& leftCameraPose,
    double min_ratio_largest_smallest_side,
    double max_triangle_side,
    const bool& reduce_mesh_to_time_horizon) {
  VLOG(10) << "Starting updatePolygonMeshToTimeHorizon...";
  LOG_IF(WARNING, points_with_id_map.size() == 0u)
      << "Missing landmark information for the Mesher: "
         "cannot trim 3D mesh to time horizon.";
  const auto& end = points_with_id_map.end();

  // Loop over each face in the mesh.
  Mesh3D::Polygon polygon;
  Mesh3D mesh_output;
  for (size_t i = 0; i < mesh_3d_.getNumberOfPolygons(); i++) {
    CHECK(mesh_3d_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    bool save_polygon = true;
    for (Mesh3D::VertexType& vertex : polygon) {
      const auto& point_with_id_it = points_with_id_map.find(vertex.getLmkId());
      if (point_with_id_it == end) {
        // Vertex of current polygon is not in points_with_id_map
        if (reduce_mesh_to_time_horizon) {
          // We want to reduce the mesh to time horizon.
          // Delete the polygon by not adding it to the new mesh.
          save_polygon = false;
          break;
        } else {
          // We do not want to reduce the mesh to time horizon.
          save_polygon = true;
        }
      } else {
        // Update the vertex with newest landmark position.
        // This is to ensure we have latest update, the previous
        // addPolygonToMesh only updates the positions of the vertices in the
        // visible frame.
        vertex.setVertexPosition(Vertex3D(point_with_id_it->second.x(),
                                          point_with_id_it->second.y(),
                                          point_with_id_it->second.z()));
      }
    }

    if (save_polygon) {
      // Refilter polygons, as the updated vertices might make it unvalid.
      if (!isBadTriangle(
              polygon,
              leftCameraPose,
              min_ratio_largest_smallest_side,
              -1.0,  // elongation test is invalid, no per-frame concept
              max_triangle_side)) {
        // Finally add the polygon to the mesh.
        mesh_output.addPolygonToMesh(polygon);
      }
    }
  }

  mesh_3d_ = mesh_output;
  VLOG(10) << "Finished updatePolygonMeshToTimeHorizon.";
}

/* -------------------------------------------------------------------------- */
// Calculate normals of polygonMesh.
// TODO(Toni): put this inside the mesh itself...
// TODO(Toni): the mesh has already a computePerVertexNormals.
// although here we are interested instead on a per-face normal.
void Mesher::calculateNormals(std::vector<cv::Point3f>* normals) {
  CHECK_NOTNULL(normals);
  CHECK_EQ(mesh_3d_.getMeshPolygonDimension(), 3)
      << "Expecting 3 vertices in triangle.";

  // Brute force, ideally only call when a new triangle appears...
  normals->clear();
  normals->resize(
      mesh_3d_.getNumberOfPolygons());  // TODO Assumes we have triangles...

  // Loop over each polygon face in the mesh.
  // TODO there are far too many loops over the total number of Polygon faces...
  // Should put them all in the same loop!
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_3d_.getNumberOfPolygons(); i++) {
    CHECK(mesh_3d_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    DCHECK_EQ(polygon.size(), 3);
    const Vertex3D& p1 = polygon.at(0).getVertexPosition();
    const Vertex3D& p2 = polygon.at(1).getVertexPosition();
    const Vertex3D& p3 = polygon.at(2).getVertexPosition();

    cv::Point3f normal;
    CHECK(calculateNormal(p1, p2, p3, &normal));
    // Mat normal2;
    // viz::computeNormals(mesh, normal2);
    // https://github.com/zhoushiwei/Viz-opencv/blob/master/Viz/main.cpp

    // Store normal to triangle i.
    normals->at(i) = normal;
  }
}

/* -------------------------------------------------------------------------- */
// Calculate normal of a triangle, and return whether it was possible or not.
// Calculating the normal of aligned points in 3D is not possible...
bool Mesher::calculateNormal(const Vertex3D& p1,
                             const Vertex3D& p2,
                             const Vertex3D& p3,
                             // Make this a cv::vector3f
                             cv::Point3f* normal) const {
  CHECK_NOTNULL(normal);
  // TODO what if p2 = p1 or p3 = p1?
  // Calculate vectors of the triangle.
  cv::Point3f v21 = p2 - p1;
  cv::Point3f v31 = p3 - p1;

  // Normalize vectors.
  double v21_norm = cv::norm(v21);
  CHECK_GT(v21_norm, 0.0);
  v21 /= v21_norm;

  double v31_norm = cv::norm(v31);
  CHECK_GT(v31_norm, 0.0);
  v31 /= v31_norm;

  // Check that vectors are not aligned, dot product should not be 1 or -1.
  static constexpr double epsilon = 1e-3;  // 2.5 degrees aperture.
  if (std::fabs(v21.ddot(v31)) >= 1.0 - epsilon) {
    // Dot prod very close to 1.0 or -1.0...
    // We have a degenerate configuration with aligned vectors.
    LOG(WARNING) << "Cross product of aligned vectors.";
    return false;
  } else {
    // Calculate normal (cross product).
    *normal = v21.cross(v31);

    // Normalize.
    double norm = cv::norm(*normal);
    CHECK_GT(norm, 0.0);
    *normal /= norm;
    CHECK_NEAR(cv::norm(*normal), 1.0, 1e-5);  // Expect unit norm.
    return true;
  }
}

/* -------------------------------------------------------------------------- */
// Clusters normals given an axis, a set of normals and a
// tolerance. The result is a vector of indices of the given set of normals
// that are in the cluster.
void Mesher::clusterNormalsAroundAxis(const cv::Point3f& axis,
                                      const std::vector<cv::Point3f>& normals,
                                      const double& tolerance,
                                      std::vector<int>* cluster_normals_idx) {
  size_t idx = 0;
  // TODO, this should be in the same loop as the one calculating
  // the normals...
  for (const cv::Point3f& normal : normals) {
    if (isNormalAroundAxis(axis, normal, tolerance))
      cluster_normals_idx->push_back(idx);
    idx++;
  }
}

/* -------------------------------------------------------------------------- */
// Is normal around axis?
bool Mesher::isNormalAroundAxis(const cv::Point3f& axis,
                                const cv::Point3f& normal,
                                const double& tolerance) const {
  // TODO typedef normals and axis to Normal, and use cv::Point3d instead.
  CHECK_NEAR(cv::norm(axis), 1.0, 1e-5);    // Expect unit norm.
  CHECK_NEAR(cv::norm(normal), 1.0, 1e-5);  // Expect unit norm.
  DCHECK_GT(tolerance, 0.0);                // Tolerance is positive.
  DCHECK_LT(tolerance, 1.0);  // Tolerance is lower than maximum dot product.
  // Dot product should be close to 1 or -1 if axis is aligned with normal.
  return (std::fabs(normal.ddot(axis)) > 1.0 - tolerance);
}

/* -------------------------------------------------------------------------- */
// Clusters normals perpendicular to an axis. Given an axis, a set of normals
// and a tolerance. The result is a vector of indices of the given set of
// normals that are in the cluster.
void Mesher::clusterNormalsPerpendicularToAxis(
    const cv::Point3f& axis,
    const std::vector<cv::Point3f>& normals,
    const double& tolerance,
    std::vector<int>* cluster_normals_idx) {
  size_t idx = 0;
  std::vector<cv::Point3f> cluster_normals;
  for (const cv::Point3f& normal : normals) {
    if (isNormalPerpendicularToAxis(axis, normal, tolerance)) {
      cluster_normals_idx->push_back(idx);
    }
    idx++;
  }
}

/* -------------------------------------------------------------------------- */
// Is normal perpendicular to axis?
bool Mesher::isNormalPerpendicularToAxis(const cv::Point3f& axis,
                                         const cv::Point3f& normal,
                                         const double& tolerance) const {
  CHECK_NEAR(cv::norm(axis), 1.0, 1e-5);    // Expect unit norm.
  CHECK_NEAR(cv::norm(normal), 1.0, 1e-5);  // Expect unit norm.
  DCHECK_GT(tolerance, 0.0);                // Tolerance is positive.
  DCHECK_LT(tolerance, 1.0);  // Tolerance is lower than maximum dot product.
  // Dot product should be close to 0 if axis is perpendicular to normal.
  return (cv::norm(normal.ddot(axis)) < tolerance);
}

/* -------------------------------------------------------------------------- */
// Checks whether all points in polygon are closer than tolerance to the plane.
bool Mesher::isPolygonAtDistanceFromPlane(
    const Mesh3D::Polygon& polygon,
    const double& plane_distance,
    const cv::Point3f& plane_normal,
    const double& distance_tolerance) const {
  CHECK_NEAR(cv::norm(plane_normal), 1.0, 1e-05);  // Expect unit norm.
  DCHECK_GE(distance_tolerance, 0.0);
  for (const Mesh3D::VertexType& vertex : polygon) {
    if (!isPointAtDistanceFromPlane(vertex.getVertexPosition(),
                                    plane_distance,
                                    plane_normal,
                                    distance_tolerance)) {
      return false;
    }
  }
  // All lmks are close to the plane.
  return true;
}

/* -------------------------------------------------------------------------- */
// Checks whether the point is closer than tolerance to the plane.
bool Mesher::isPointAtDistanceFromPlane(
    const Vertex3D& point,
    const double& plane_distance,
    const cv::Point3f& plane_normal,
    const double& distance_tolerance) const {
  CHECK_NEAR(cv::norm(plane_normal), 1.0, 1e-05);  // Expect unit norm.
  DCHECK_GE(distance_tolerance, 0.0);
  // The lmk is closer to the plane than given tolerance.
  return (std::fabs(plane_distance - point.ddot(plane_normal)) <=
          distance_tolerance);
}

/* -------------------------------------------------------------------------- */
// Cluster planes from Mesh.
// Points_with_id_vio are only used when add_extra_lmks_from_stereo is true, so
// that we only extract lmk ids that are in the optimization time horizon.
void Mesher::clusterPlanesFromMesh(std::vector<Plane>* planes,
                                   const PointsWithIdMap& points_with_id_vio) {
  CHECK_NOTNULL(planes);
  // Segment planes in the mesh, using seeds.
  VLOG(10) << "Starting plane segmentation...";
  std::vector<Plane> new_planes;
  segmentPlanesInMesh(planes,
                      &new_planes,
                      points_with_id_vio,
                      FLAGS_normal_tolerance_polygon_plane_association,
                      FLAGS_distance_tolerance_polygon_plane_association,
                      FLAGS_normal_tolerance_horizontal_surface,
                      FLAGS_normal_tolerance_walls);
  VLOG(10) << "Finished plane segmentation.";
  // Do data association between the planes given and the ones segmented.
  VLOG(10) << "Starting plane association...";
  std::vector<Plane> new_non_associated_planes;
  associatePlanes(new_planes,
                  *planes,
                  &new_non_associated_planes,
                  FLAGS_normal_tolerance_plane_plane_association,
                  FLAGS_distance_tolerance_plane_plane_association);
  VLOG(10) << "Finished plane association.";
  if (new_non_associated_planes.size() > 0) {
    // Update lmk ids of the newly added planes.

    // TODO delete this loop by customizing histograms!!
    // WARNING Here we are updating lmk ids in new non-associated planes,
    // BUT it requires another loop over mesh, and recalculates normals!!!
    // Very unefficient.
    VLOG(10) << "Starting update plane lmk ids for new non-associated planes.";
    updatePlanesLmkIdsFromMesh(
        &new_non_associated_planes,
        FLAGS_normal_tolerance_polygon_plane_association,
        FLAGS_distance_tolerance_polygon_plane_association,
        points_with_id_vio);
    VLOG(10) << "Finished update plane lmk ids for new non-associated planes.";

    // Append new planes that where not associated to original planes.
    planes->insert(planes->end(),
                   new_non_associated_planes.begin(),
                   new_non_associated_planes.end());
  } else {
    VLOG(10)
        << "Avoid extra loop over mesh, since there are no new non-associated"
           " planes to be updated.";
  }
}

/* -------------------------------------------------------------------------- */
// Segment planes in the mesh: updates seed_planes and extracts new_planes.
// Points_with_id_vio are only used if add_extra_lmks_from_stereo is true,
// They are used by extractLmkIdsFromTriangleCluster to extract only lmk ids
// that are in the time horizon (aka points_with_id_vio).
void Mesher::segmentPlanesInMesh(
    std::vector<Plane>* seed_planes,
    std::vector<Plane>* new_planes,
    const PointsWithIdMap& points_with_id_vio,
    const double& normal_tolerance_polygon_plane_association,
    const double& distance_tolerance_polygon_plane_association,
    const double& normal_tolerance_horizontal_surface,
    const double& normal_tolerance_walls) {
  CHECK_NOTNULL(seed_planes);
  CHECK_NOTNULL(new_planes);

  // Clean seed_planes of lmk_ids:
  for (Plane& seed_plane : *seed_planes) {
    seed_plane.lmk_ids_.clear();
    seed_plane.triangle_cluster_.triangle_ids_.clear();
  }

  static constexpr size_t mesh_polygon_dim = 3;
  CHECK_EQ(mesh_3d_.getMeshPolygonDimension(), mesh_polygon_dim)
      << "Expecting 3 vertices in triangle.";

  // Cluster new lmk ids for seed planes.
  // Loop over the mesh only once.
  Mesh3D::Polygon polygon;
  cv::Mat z_components(1, 0, CV_32F);
  cv::Mat walls(0, 0, CV_32FC2);
  for (size_t i = 0; i < mesh_3d_.getNumberOfPolygons(); i++) {
    CHECK(mesh_3d_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), mesh_polygon_dim);
    const Vertex3D& p1 = polygon.at(0).getVertexPosition();
    const Vertex3D& p2 = polygon.at(1).getVertexPosition();
    const Vertex3D& p3 = polygon.at(2).getVertexPosition();

    // Calculate normal of the triangle in the mesh.
    // The normals are in the world frame of reference.
    cv::Point3f triangle_normal;
    if (calculateNormal(p1, p2, p3, &triangle_normal)) {
      ////////////////////////// Update seed planes ////////////////////////////
      // Update seed_planes lmk_ids field with ids of vertices of polygon if the
      // polygon is on the plane.
      bool is_polygon_on_a_plane = updatePlanesLmkIdsFromPolygon(
          seed_planes,
          polygon,
          i,
          triangle_normal,
          normal_tolerance_polygon_plane_association,
          distance_tolerance_polygon_plane_association,
          points_with_id_vio,
          FLAGS_only_associate_a_polygon_to_a_single_plane);

      ////////////////// Build Histogram for new planes ////////////////////////
      /// Values for Z Histogram.///////////////////////////////////////////////
      // Collect z values of vertices of polygon which is not already on a plane
      // and which has the normal aligned with the vertical direction so that we
      // can build an histogram.
      static const cv::Point3f vertical(0, 0, 1);
      if ((FLAGS_only_use_non_clustered_points ? !is_polygon_on_a_plane
                                               : true) &&
          isNormalAroundAxis(
              vertical, triangle_normal, normal_tolerance_horizontal_surface)) {
        // We have a triangle with a normal aligned with gravity, which is not
        // already clustered in a plane.
        // Store z components to build histogram.
        // TODO instead of storing z_components, use the accumulate flag in
        // calcHist and add them straight.
        z_components.push_back(p1.z);
        z_components.push_back(p2.z);
        z_components.push_back(p3.z);
      } else if ((FLAGS_only_use_non_clustered_points ? !is_polygon_on_a_plane
                                                      : true) &&
                 isNormalPerpendicularToAxis(
                     vertical, triangle_normal, normal_tolerance_walls)) {
        /// Values for walls Histogram./////////////////////////////////////////
        // WARNING if we do not normalize, we'll have two peaks for the same
        // plane, no?
        // Store theta.
        double theta = getLongitude(triangle_normal, vertical);

        // Store distance.
        // Using triangle_normal.
        double distance = p1.ddot(triangle_normal);
        if (theta < 0) {
          VLOG(10) << "Normalize theta: " << theta
                   << " and distance: " << distance;
          // Say theta is -pi/2, then normalized theta is pi/2.
          theta = theta + M_PI;
          // Change distance accordingly.
          distance = -distance;
          VLOG(10) << "New normalized theta: " << theta
                   << " and distance: " << distance;
        }
        walls.push_back(cv::Point2f(theta, distance));
        // WARNING should we instead be using projected triangle normal
        // on equator, and taking average of three distances...
        // NORMALIZE if a theta is positive and distance negative, it is the
        // same as if theta is 180 deg from it and distance positive...
      }
    }
  }

  VLOG(10) << "Number of polygons potentially on a wall: " << walls.rows;

  // Segment new planes.
  // Currently using lmks that were used by the seed_planes...
  segmentNewPlanes(new_planes, z_components, walls);
}

/* -------------------------------------------------------------------------- */
// Output goes from (-pi to pi], as we are using atan2, which looks at sign
// of arguments.
double Mesher::getLongitude(const cv::Point3f& triangle_normal,
                            const cv::Point3f& vertical) const {
  // A get projection on equatorial plane.
  // i get projection of triangle normal on vertical
  CHECK_NEAR(cv::norm(triangle_normal), 1.0, 1e-5);  // Expect unit norm.
  CHECK_NEAR(cv::norm(vertical), 1.0, 1e-5);         // Expect unit norm.
  cv::Point3f equatorial_proj =
      triangle_normal - vertical.ddot(triangle_normal) * vertical;
  CHECK_NEAR(equatorial_proj.ddot(vertical), 0.0, 1e-5);
  CHECK(equatorial_proj.y != 0 || equatorial_proj.x != 0);
  return std::atan2(equatorial_proj.y, equatorial_proj.x);
}

/* -------------------------------------------------------------------------- */
// Update plane lmk ids field, by looping over the mesh and stoting lmk ids of
// the vertices of the polygons that are close to the plane.
// It will append lmk ids to the ones already present in the plane.
// Points with id vio, only used if we are using stereo points to build the
// mesh.
void Mesher::updatePlanesLmkIdsFromMesh(
    std::vector<Plane>* planes,
    double normal_tolerance,
    double distance_tolerance,
    const PointsWithIdMap& points_with_id_vio) const {
  CHECK_NOTNULL(planes);
  static constexpr size_t mesh_polygon_dim = 3;
  CHECK_EQ(mesh_3d_.getMeshPolygonDimension(), mesh_polygon_dim)
      << "Expecting 3 vertices in triangle.";
  Mesh3D::Polygon polygon;
  for (size_t i = 0; i < mesh_3d_.getNumberOfPolygons(); i++) {
    CHECK(mesh_3d_.getPolygon(i, &polygon)) << "Could not retrieve polygon.";
    CHECK_EQ(polygon.size(), mesh_polygon_dim);
    const Vertex3D& p1 = polygon.at(0).getVertexPosition();
    const Vertex3D& p2 = polygon.at(1).getVertexPosition();
    const Vertex3D& p3 = polygon.at(2).getVertexPosition();

    // Calculate normal of the triangle in the mesh.
    // The normals are in the world frame of reference.
    cv::Point3f triangle_normal;
    if (calculateNormal(p1, p2, p3, &triangle_normal)) {
      // Loop over newly segmented planes, and update lmk ids field if
      // the current polygon is on the plane.
      updatePlanesLmkIdsFromPolygon(
          planes,
          polygon,
          i,
          triangle_normal,
          normal_tolerance,
          distance_tolerance,
          points_with_id_vio,
          FLAGS_only_associate_a_polygon_to_a_single_plane);
    }
  }
}

/* -------------------------------------------------------------------------- */
// Updates planes lmk ids field with a polygon vertices ids if this polygon
// is part of the plane according to given tolerance.
// points_with_id_vio is only used if we are using stereo points...
bool Mesher::updatePlanesLmkIdsFromPolygon(
    std::vector<Plane>* seed_planes,
    const Mesh3D::Polygon& polygon,
    const size_t& triangle_id,
    const cv::Point3f& triangle_normal,
    double normal_tolerance,
    double distance_tolerance,
    const PointsWithIdMap& points_with_id_vio,
    bool only_associate_a_polygon_to_a_single_plane) const {
  CHECK_NOTNULL(seed_planes);
  bool is_polygon_on_a_plane = false;
  for (Plane& seed_plane : *seed_planes) {
    // Only cluster if normal and distance of polygon are close to plane.
    // WARNING: same polygon is being possibly clustered in multiple planes.
    // Break loop when polygon is in a plane?
    if (isNormalAroundAxis(
            seed_plane.normal_, triangle_normal, normal_tolerance) &&
        isPolygonAtDistanceFromPlane(polygon,
                                     seed_plane.distance_,
                                     seed_plane.normal_,
                                     distance_tolerance)) {
      // I guess we can comment out the check below since it can happen that
      // a poygon is segmented in two very close planes? Better we enable
      // it again!
      // CHECK(!is_polygon_on_a_plane) << "Polygon was already in a plane,"
      //                                 " are we having similar planes?";

      // Update lmk_ids of seed plane.
      // Points_with_id_vio are only used for stereo.
      appendLmkIdsOfPolygon(polygon, &seed_plane.lmk_ids_, points_with_id_vio);

      // TODO Remove, only used for visualization...
      seed_plane.triangle_cluster_.triangle_ids_.push_back(triangle_id);

      // Acknowledge that the polygon is at least in one plane, to avoid
      // sending this polygon to segmentation and segment the same plane
      // again.
      is_polygon_on_a_plane = true;
      if (only_associate_a_polygon_to_a_single_plane) {
        break;
      }
    }
  }
  return is_polygon_on_a_plane;
}

/* -------------------------------------------------------------------------- */
// Segment new planes in the mesh.
// Currently segments horizontal planes using z_components, which is
// expected to be a cv::Mat z_components (1, 0, CV_32F);
// And walls perpendicular to the ground, using a cv::Mat which is expected to
// be a cv::Mat walls (0, 0, CV_32FC2), with first channel being theta (yaw
// angle of the wall) and the second channel the distance of it.
// points_with_id_vio is only used if we are using stereo points...
void Mesher::segmentNewPlanes(std::vector<Plane>* new_segmented_planes,
                              const cv::Mat& z_components,
                              const cv::Mat& walls) {
  CHECK_NOTNULL(new_segmented_planes);
  new_segmented_planes->clear();

  // Segment horizontal planes.
  static size_t plane_id = 0;
  static const Plane::Normal vertical(0, 0, 1);
  segmentHorizontalPlanes(
      new_segmented_planes, &plane_id, vertical, z_components);

  // Segment vertical planes.
  segmentWalls(new_segmented_planes, &plane_id, walls);
}

/* -------------------------------------------------------------------------- */
// Segment wall planes.
// plane_id, starting id for new planes, it gets increased every time we add a
// new plane.
void Mesher::segmentWalls(std::vector<Plane>* wall_planes,
                          size_t* plane_id,
                          const cv::Mat& walls) {
  CHECK_NOTNULL(wall_planes);
  CHECK_NOTNULL(plane_id);
  ////////////////////////////// 2D Histogram //////////////////////////////////
  VLOG(10) << "Starting to calculate 2D histogram...";
  hist_2d_.calculateHistogram(walls, FLAGS_log_histogram_2D);
  VLOG(10) << "Finished to calculate 2D histogram.";

  /// Added by me
  // cv::GaussianBlur(histImg, histImg, cv::Size(9, 9), 0);
  ///
  VLOG(10) << "Starting get local maximum for 2D histogram...";
  std::vector<Histogram::PeakInfo2D> peaks2;
  static const cv::Size kernel_size_2d(FLAGS_hist_2d_gaussian_kernel_size,
                                       FLAGS_hist_2d_gaussian_kernel_size);
  hist_2d_.getLocalMaximum2D(&peaks2,
                             kernel_size_2d,
                             FLAGS_hist_2d_nr_of_local_max,
                             FLAGS_hist_2d_min_support,
                             FLAGS_hist_2d_min_dist_btw_local_max,
                             FLAGS_visualize_histogram_2D,
                             FLAGS_log_histogram_2D);
  VLOG(10) << "Finished get local maximum for 2D histogram.";

  VLOG(0) << "# of peaks in 2D histogram = " << peaks2.size();
  size_t i = 0;
  for (const Histogram::PeakInfo2D& peak : peaks2) {
    double plane_theta = peak.x_value_;
    double plane_distance = peak.y_value_;
    cv::Point3f plane_normal(std::cos(plane_theta), std::sin(plane_theta), 0);
    VLOG(0) << "Peak #" << i << " in bin with coords: "
            << " x= " << peak.pos_.x << " y= " << peak.pos_.y
            << ". So peak with theta = " << plane_theta
            << " (normal: x= " << plane_normal.x << " and y= " << plane_normal.y
            << " )"
            << " and distance = " << plane_distance;

    // WARNING we are not giving lmk ids to this plane!
    // We should either completely customize the histogram calc to pass lmk ids
    // or do another loop over the mesh to cluster new triangles.
    const gtsam::Symbol plane_symbol('P', *plane_id);
    static constexpr int cluster_id =
        1;  // Only used for visualization. 1 = walls.
    VLOG(10) << "Segmented a wall plane with:\n"
             << "\t normal: " << plane_normal
             << "\t distance: " << plane_distance << "\n\t plane id: "
             << gtsam::DefaultKeyFormatter(plane_symbol.key())
             << "\n\t cluster id: " << cluster_id;
    wall_planes->push_back(Plane(plane_symbol,
                                 plane_normal,
                                 plane_distance,
                                 // Currently filled after this function...
                                 LandmarkIds(),  // We should fill this!!!
                                 cluster_id));
    (*plane_id)++;  // CRITICAL TO GET THIS RIGHT: ensure no duplicates,

    i++;
  }
}

/* -------------------------------------------------------------------------- */
// Segment new planes horizontal.
// plane_id, starting id for new planes, it gets increased every time we add a
// new plane.
void Mesher::segmentHorizontalPlanes(std::vector<Plane>* horizontal_planes,
                                     size_t* plane_id,
                                     const Plane::Normal& normal,
                                     const cv::Mat& z_components) {
  CHECK_NOTNULL(horizontal_planes);
  CHECK_NOTNULL(plane_id);
  ////////////////////////////// 1D Histogram //////////////////////////////////
  VLOG(10) << "Starting calculate 1D histogram.";
  z_hist_.calculateHistogram(z_components, FLAGS_log_histogram_1D);
  VLOG(10) << "Finished calculate 1D histogram.";

  VLOG(10) << "Starting get local maximum for 1D.";
  static const cv::Size kernel_size(1, FLAGS_z_histogram_gaussian_kernel_size);
  std::vector<Histogram::PeakInfo> peaks =
      z_hist_.getLocalMaximum1D(kernel_size,
                                FLAGS_z_histogram_window_size,
                                FLAGS_z_histogram_peak_per,
                                FLAGS_z_histogram_min_support,
                                FLAGS_visualize_histogram_1D,
                                FLAGS_log_histogram_1D);
  VLOG(10) << "Finished get local maximum for 1D.";

  LOG(WARNING) << "# of peaks in 1D histogram = " << peaks.size();
  size_t i = 0;
  std::vector<Histogram::PeakInfo>::iterator previous_peak_it;
  double previous_plane_distance = -DBL_MAX;
  for (std::vector<Histogram::PeakInfo>::iterator peak_it = peaks.begin();
       peak_it != peaks.end();) {
    // Make sure it is below min possible value for distance.
    double plane_distance = peak_it->value_;
    VLOG(10) << "Peak #" << i << " in bin " << peak_it->pos_
             << " has distance = " << plane_distance << " with a support of "
             << peak_it->support_ << " points";

    // Remove duplicates, and, for peaks that are too close, take the one with
    // maximum support.
    // Assuming repeated peaks are ordered...
    if (i > 0 && *peak_it == peaks.at(i - 1)) {
      // Repeated element, delete it.
      LOG(WARNING) << "Deleting repeated peak for peak # " << i << " in bin "
                   << peak_it->pos_;
      peak_it = peaks.erase(peak_it);
      i--;
    } else if (i > 0 &&
               std::fabs(previous_plane_distance - plane_distance) <
                   FLAGS_z_histogram_min_separation) {
      // Not enough separation between planes, delete the one with less support.
      if (previous_peak_it->support_ < peak_it->support_) {
        // Delete previous_peak.
        LOG(WARNING) << "Deleting peak in bin " << previous_peak_it->pos_;
        // Iterators, pointers and references pointing to position (or first)
        // and
        // beyond are invalidated, with all iterators, pointers and references
        // to elements before position (or first) are guaranteed to keep
        // referring to the same elements they were referring to before the
        // call.
        peaks.erase(previous_peak_it);
        peak_it = peaks.begin() + i - 1;
        i--;
      } else {
        // Delete peak_it.
        LOG(WARNING) << "Deleting too close peak # " << i << " in bin "
                     << peak_it->pos_;
        peak_it = peaks.erase(peak_it);
        i--;
      }
    } else {
      previous_peak_it = peak_it;
      previous_plane_distance = plane_distance;
      peak_it++;
      i++;
    }
  }

  for (int peak_nr = 0;
       peak_nr < FLAGS_z_histogram_max_number_of_peaks_to_select;
       peak_nr++) {
    // Get the peaks in order of max support.
    std::vector<Histogram::PeakInfo>::iterator it =
        std::max_element(peaks.begin(), peaks.end());
    if (it != peaks.end()) {
      double plane_distance = it->value_;
      // WARNING we are not giving lmk ids to this plane!
      // We should either completely customize the histogram calc to pass lmk
      // ids or do another loop over the mesh to cluster new triangles.
      const gtsam::Symbol plane_symbol('P', *plane_id);
      static constexpr int cluster_id =
          2;  // Only used for visualization. 2 = ground.
      VLOG(10) << "Segmented an horizontal plane with:\n"
               << "\t distance: " << plane_distance << "\n\t plane id: "
               << gtsam::DefaultKeyFormatter(plane_symbol.key())
               << "\n\t cluster id: " << cluster_id;
      horizontal_planes->push_back(
          Plane(plane_symbol,
                normal,
                plane_distance,
                // Currently filled after this function...
                LandmarkIds(),  // We should fill this!!!
                cluster_id));
      (*plane_id)++;  // CRITICAL TO GET THIS RIGHT: ensure no duplicates,
      // no wrong ids...

      // Delete current peak from set of peaks, so that we can find next
      // maximum.
      peaks.erase(it);
    } else {
      if (peaks.size() == 0) {
        VLOG(10) << "No more peaks available.";
      } else {
        VLOG(10) << "Could not find a maximum among the list of "
                 << peaks.size() << " peaks in histogram of horizontal planes.";
      }
      break;
    }
  }
}

/* -------------------------------------------------------------------------- */
// Data association between planes.
void Mesher::associatePlanes(const std::vector<Plane>& segmented_planes,
                             const std::vector<Plane>& planes,
                             std::vector<Plane>* non_associated_planes,
                             const double& normal_tolerance,
                             const double& distance_tolerance) const {
  CHECK_NOTNULL(non_associated_planes);
  non_associated_planes->clear();
  if (planes.size() == 0) {
    // There are no previous planes, data association unnecessary, just copy
    // segmented planes to output planes.
    VLOG(0) << "No planes in Backend, just copy the " << segmented_planes.size()
            << " segmented planes to the set of "
            << "Backend planes, skipping data association.";
    *non_associated_planes = segmented_planes;
  } else {
    // Planes tmp will contain the new segmented planes.
    // Both the ones that could be associated, in which case only the landmark
    // ids are updated (symbol, norm, distance remain the same).
    // WARNING maybe we should do the Union of both the lmk ids of the new plane
    // and the old plane........................
    // And the ones that could not be associated, in which case they are added
    // as new planes, with a new symbol.
    if (segmented_planes.size() == 0) {
      LOG(WARNING) << "No segmented planes.";
    }
    // std::vector<Plane> planes_tmp;
    // To avoid  associating several segmented planes to the same
    // plane_backend
    std::vector<uint64_t> associated_plane_ids;
    for (const Plane& segmented_plane : segmented_planes) {
      bool is_segmented_plane_associated = false;
      for (const Plane& plane_backend : planes) {
        // Check if normals are close or 180 degrees apart.
        // Check if distance is similar in absolute value.
        // TODO check distance given the difference in normals.
        if (plane_backend.geometricEqual(
                segmented_plane, normal_tolerance, distance_tolerance)) {
          // We found a plane association
          uint64_t backend_plane_index = plane_backend.getPlaneSymbol().index();
          // Check that it was not associated before.
          if (std::find(associated_plane_ids.begin(),
                        associated_plane_ids.end(),
                        backend_plane_index) == associated_plane_ids.end()) {
            // It is the first time we associate this plane.
            // Update lmk ids in plane.
            VLOG(10) << "Plane from Backend with id "
                     << gtsam::DefaultKeyFormatter(
                            plane_backend.getPlaneSymbol().key())
                     << " has been associated with segmented plane: "
                     << gtsam::DefaultKeyFormatter(
                            segmented_plane.getPlaneSymbol().key());
            // Update plane.
            // WARNING maybe do the union between previous and current lmk_ids?
            // Or just don't do anything, cause the plane should be there with
            // its own lmk ids already...
            // Actually YES DO IT, so we can spare one loop over the mesh!
            // the first one ! (aka go back to the so-called naive
            // implementation! Not entirely true, since we still need to loop
            // over the mesh to get the points for extracting new planes...
            // plane_backend.lmk_ids_ = segmented_plane.lmk_ids_;
            // WARNING TODO should we also update the normal & distance??
            // Acknowledge that we have an association.
            associated_plane_ids.push_back(backend_plane_index);

            is_segmented_plane_associated = true;
            break;
          } else {
            // Continue, to see if we can associate the current segmented plane
            // to another Backend plane.
            LOG(ERROR) << "Double plane association of Backend plane: "
                       << gtsam::DefaultKeyFormatter(
                              plane_backend.getPlaneSymbol().key())
                       << " with another segmented plane: "
                       << gtsam::DefaultKeyFormatter(
                              segmented_plane.getPlaneSymbol().key())
                       << "\n.";
            if (FLAGS_do_double_association) {
              LOG(ERROR) << "Doing double plane association of Backend plane.";
              is_segmented_plane_associated = true;
              break;
            } else {
              LOG(ERROR)
                  << "Avoiding double plane association of Backend plane. "
                  << "Searching instead for another possible Backend plane for "
                     "this"
                     " segmented plane.";
              continue;
            }
          }
        } else {
          VLOG(0) << "Plane " << gtsam::DefaultKeyFormatter(
                                     plane_backend.getPlaneSymbol().key())
                  << " from Backend not associated to new segmented plane "
                  << gtsam::DefaultKeyFormatter(
                         segmented_plane.getPlaneSymbol())
                  << "\n\tSegmented normal: " << segmented_plane.normal_
                  << " ( vs normal: " << plane_backend.normal_
                  << ") \n\tSegmented distance: " << segmented_plane.distance_
                  << " ( vs distance: " << plane_backend.distance_ << ").";
        }
      }

      if (!is_segmented_plane_associated) {
        // The segmented plane could not be associated to any existing plane
        // in the Backend...
        // Add it as a new plane.
        VLOG(0) << "Add plane with id "
                << gtsam::DefaultKeyFormatter(segmented_plane.getPlaneSymbol())
                << " as a new plane for the Backend.";
        // WARNING by pushing we are also associating segmented planes between
        // them, do we want this? Maybe yes because we have some repeated
        // segmented planes that are very similar, but then what's up with the
        // lmk ids, which should we keep?
        non_associated_planes->push_back(segmented_plane);
      }
    }

    // Update planes.
    // Cleans planes that have not been updated
    // Which is not necessarily good, because the segmenter might differ from
    // Backend and we will not do expectation maximization anymore... plus
    // it makes the visualizer not work...
    //*planes = planes_tmp;
  }
}

/* -------------------------------------------------------------------------- */
// Update mesh: update structures keeping memory of the map before visualization
// Optional parameter is the mesh in 2D for visualization.
void Mesher::updateMesh3D(const PointsWithIdMap& points_with_id_VIO,
                          const KeypointsCV& keypoints,
                          const std::vector<KeypointStatus>& keypoints_status,
                          const BearingVectors& keypoints_3d,
                          const LandmarkIds& landmarks,
                          const gtsam::Pose3& left_camera_pose,
                          Mesh2D* mesh_2d,
                          std::vector<cv::Vec6f>* mesh_2d_for_viz) {
  VLOG(10) << "Starting updateMesh3D...";
  LOG_IF(WARNING, points_with_id_VIO.size() == 0u)
      << "Missing landmark information to build 3D Mesh.";
  const PointsWithIdMap* points_with_id_all = &points_with_id_VIO;

  // Get points in stereo camera that are not in vio but have lmk id:
  PointsWithIdMap points_with_id_stereo;
  // TODO(Toni): allow for only seeing stereo mesh
  if (FLAGS_add_extra_lmks_from_stereo) {
    // Append vio points.
    // WARNING some stereo and vio lmks share the same id, so adding order
    // matters! first add vio points, then stereo, so that vio points have
    // preference over stereo ones if they are repeated!
    static constexpr bool kAppendStereoLmks = true;
    if (kAppendStereoLmks) {
      points_with_id_stereo = points_with_id_VIO;
    }
    appendNonVioStereoPoints(landmarks,
                             keypoints_status,
                             keypoints_3d,
                             left_camera_pose,
                             &points_with_id_stereo);
    VLOG(20) << "Number of stereo landmarks used for the mesh: "
             << points_with_id_stereo.size() << "\n"
             << "Number of VIO landmarks used for the mesh: "
             << points_with_id_VIO.size();

    points_with_id_all = &points_with_id_stereo;
  }
  LOG_IF(WARNING, points_with_id_all->size() == 0u)
      << "Missing landmark information for the Mesher!";
  VLOG(20) << "Total number of landmarks used for the mesh: "
           << points_with_id_all->size();

  // Build 2D mesh.
  std::vector<cv::Vec6f> mesh_2d_pixels;
  createMesh2dVIO(&mesh_2d_pixels,
                  landmarks,
                  keypoints_status,
                  keypoints,
                  mesher_params_.img_size_,
                  *points_with_id_all);
  if (mesh_2d_for_viz) *mesh_2d_for_viz = mesh_2d_pixels;
  LOG_IF(WARNING, mesh_2d_pixels.size() == 0) << "2D Mesh is empty!";

  populate3dMeshTimeHorizon(mesh_2d_pixels,
                            *points_with_id_all,
                            keypoints,
                            landmarks,
                            left_camera_pose,
                            FLAGS_min_ratio_btw_largest_smallest_side,
                            FLAGS_min_elongation_ratio,
                            FLAGS_max_triangle_side,
                            mesh_2d);

  // Calculate 3d mesh normals.
  if (FLAGS_compute_per_vertex_normals) mesh_3d_.computePerVertexNormals();

  VLOG(10) << "Finished updateMesh3D.";
}

/* -------------------------------------------------------------------------- */
// Update mesh, but in a thread-safe way.
// TODO(TONI): this seems completely unnecessary
void Mesher::updateMesh3D(const MesherInput& mesher_payload,
                          Mesh2D* mesh_2d,
                          std::vector<cv::Vec6f>* mesh_2d_for_viz) {
  const StereoFrame& stereo_frame =
      mesher_payload.frontend_output_->stereo_frame_lkf_;
  const StatusKeypointsCV& right_keypoints = 
      stereo_frame.right_keypoints_rectified_;
  std::vector<KeypointStatus> right_keypoint_status;
  right_keypoint_status.reserve(right_keypoints.size());
  for (const StatusKeypointCV& kpt : right_keypoints) {
    right_keypoint_status.push_back(kpt.first);
  }

  updateMesh3D(mesher_payload.backend_output_->landmarks_with_id_map_,
               stereo_frame.left_frame_.keypoints_,
               right_keypoint_status,
               stereo_frame.keypoints_3d_,
               stereo_frame.left_frame_.landmarks_,
               mesher_payload.backend_output_->W_State_Blkf_.pose_.compose(
                   mesher_params_.B_Pose_camLrect_),
               mesh_2d,
               mesh_2d_for_viz);
}

/* -------------------------------------------------------------------------- */
// Attempts to insert new points in the map, but does not override if there
// is already a point with the same lmk id.
void Mesher::appendNonVioStereoPoints(
    const LandmarkIds& landmarks,
    const std::vector<KeypointStatus>& keypoints_status,
    const BearingVectors& keypoints_3d,
    const gtsam::Pose3& left_cam_pose,
    PointsWithIdMap* points_with_id_stereo) const {
  CHECK_NOTNULL(points_with_id_stereo);
  CHECK_EQ(landmarks.size(), keypoints_status.size())
      << "Landmarks and keypoints_status should have same dimension...";
  CHECK_EQ(landmarks.size(), keypoints_3d.size())
      << "Landmarks and keypoints_3d should have same dimension...";
  for (size_t i = 0; i < landmarks.size(); i++) {
    const LandmarkId& landmark_id = landmarks.at(i);
    if (keypoints_status.at(i) == KeypointStatus::VALID && landmark_id != -1) {
      const gtsam::Point3& p_i_global =
          left_cam_pose.transformFrom(gtsam::Point3(keypoints_3d.at(i)));
      // Use insert() instead of [] operator, to make sure that if there is
      // already a point with the same lmk_id, we do not override it.
      points_with_id_stereo->insert(std::make_pair(landmark_id, p_i_global));
    }
  }
}

/* -------------------------------------------------------------------------- */
// TODO avoid this loop by enforcing to pass the lmk id of the vertex of the
// triangle in the triangle cluster.
// In case we are using extra lmks from stereo,
// then it makes sure that the lmk ids are used in the optimization
// (they are present in time horizon).
void Mesher::extractLmkIdsFromVectorOfTriangleClusters(
    const std::vector<TriangleCluster>& triangle_clusters,
    const PointsWithIdMap& points_with_id_vio,
    LandmarkIds* lmk_ids) const {
  VLOG(10) << "Starting extract lmk ids for vector of triangle cluster...";
  CHECK_NOTNULL(lmk_ids);
  lmk_ids->resize(0);

  for (const TriangleCluster& triangle_cluster : triangle_clusters) {
    extractLmkIdsFromTriangleCluster(
        triangle_cluster, points_with_id_vio, lmk_ids);
  }
  VLOG(10) << "Finished extract lmk ids for vector of triangle cluster.";
}

/* -------------------------------------------------------------------------- */
// Extracts lmk ids from triangle cluster. In case we are using extra lmks
// from stereo, then it makes sure that the lmk ids are used in the optimization
// (they are present in time horizon: meaning it checks that we can find the
// lmk id in points_with_id_vio...
void Mesher::extractLmkIdsFromTriangleCluster(
    const TriangleCluster& triangle_cluster,
    const PointsWithIdMap& points_with_id_vio,
    LandmarkIds* lmk_ids) const {
  VLOG(10) << "Starting extractLmkIdsFromTriangleCluster...";
  CHECK_NOTNULL(lmk_ids);
  lmk_ids->resize(0);

  Mesh3D::Polygon polygon;
  for (const size_t& polygon_idx : triangle_cluster.triangle_ids_) {
    CHECK(mesh_3d_.getPolygon(polygon_idx, &polygon))
        << "Polygon, with idx " << polygon_idx << ", is not in the mesh.";
    appendLmkIdsOfPolygon(polygon, lmk_ids, points_with_id_vio);
  }
  VLOG(10) << "Finished extractLmkIdsFromTriangleCluster.";
}

/* -------------------------------------------------------------------------- */
// Extracts lmk ids from a mesh polygon and appends them to lmk_ids.
// In case we are using extra lmks from stereo, then it makes sure that the lmk
// ids are used in the optimization (they are present in time horizon: meaning
// it checks that we can find the lmk id in points_with_id_vio...
// WARNING: this function won't check that the original lmk_ids are in the
// optimization (time-horizon)...
void Mesher::appendLmkIdsOfPolygon(
    const Mesh3D::Polygon& polygon,
    LandmarkIds* lmk_ids,
    const PointsWithIdMap& points_with_id_vio) const {
  CHECK_NOTNULL(lmk_ids);
  for (const Mesh3D::VertexType& vertex : polygon) {
    // Ensure we are not adding more than once the same lmk_id.
    const auto& it =
        std::find(lmk_ids->begin(), lmk_ids->end(), vertex.getLmkId());
    if (it == lmk_ids->end()) {
      // The lmk id is not present in the lmk_ids vector, add it.
      if (FLAGS_add_extra_lmks_from_stereo) {
        // Only add lmks that are used in the Backend (time-horizon).
        // This is just needed when adding extra lmks from stereo...
        // We are assuming lmk_ids has already only points in time-horizon,
        // so no need to check them as well.
        if (points_with_id_vio.find(vertex.getLmkId()) !=
            points_with_id_vio.end()) {
          lmk_ids->push_back(vertex.getLmkId());
        }
      } else {
        lmk_ids->push_back(vertex.getLmkId());
      }
    } else {
      // The lmk id is already in the lmk_ids vector, do not add it.
      continue;
    }
  }
}

/* -------------------------------------------------------------------------- */
void Mesher::getVerticesMesh(cv::Mat* vertices_mesh) const {
  CHECK_NOTNULL(vertices_mesh);
  mesh_3d_.getVerticesMeshToMat(vertices_mesh);
}
void Mesher::getPolygonsMesh(cv::Mat* polygons_mesh) const {
  CHECK_NOTNULL(polygons_mesh);
  mesh_3d_.getPolygonsMeshToMat(polygons_mesh);
}

void Mesher::serializeMeshes() {
  CHECK(mesher_logger_);
  mesher_logger_->serializeMesh(mesh_3d_, "mesh_3d");
  mesher_logger_->serializeMesh(mesh_2d_, "mesh_2d");
}

void Mesher::deserializeMeshes() {
  CHECK(mesher_logger_);
  mesher_logger_->deserializeMesh("mesh_3d", &mesh_3d_);
  mesher_logger_->deserializeMesh("mesh_2d", &mesh_2d_);
}

/* -------------------------------------------------------------------------- */
void Mesher::createMesh2dVIO(
    std::vector<cv::Vec6f>* triangulation_2D,
    const LandmarkIds& landmarks,
    const std::vector<KeypointStatus>& keypoints_status,
    const KeypointsCV& keypoints,
    const cv::Size& img_size,
    const PointsWithIdMap& pointsWithIdVIO) {
  CHECK_NOTNULL(triangulation_2D);

  // Pick left frame.
  // Sanity check.
  CHECK_EQ(landmarks.size(), keypoints_status.size())
      << "Wrong dimension for the landmarks";
  CHECK_EQ(landmarks.size(), keypoints.size())
      << "Wrong dimension for the keypoints";

  // Create mesh including indices of keypoints with valid 3D.
  // (which have right px).
  std::vector<cv::Point2f> keypoints_for_mesh;
  // TODO this double loop is quite expensive.
  LOG_IF(WARNING, pointsWithIdVIO.empty())
      << "List of Keypoints with associated Landmarks is empty.";
  for (const auto& point_with_id : pointsWithIdVIO) {
    for (size_t j = 0u; j < landmarks.size(); j++) {
      // If we are seeing a VIO point in left and right frame, add to keypoints
      // to generate the mesh in 2D.
      if (landmarks.at(j) == point_with_id.first &&
          keypoints_status.at(j) == KeypointStatus::VALID) {
        // Add keypoints for mesh 2d.
        keypoints_for_mesh.push_back(keypoints.at(j));
      }
    }
  }

  // Get a triangulation for all valid keypoints.
  *triangulation_2D = createMesh2dImpl(img_size, keypoints_for_mesh);
}

/* -------------------------------------------------------------------------- */
// Create a 2D mesh from 2D corners in an image
// Returns the actual keypoints used to perform the triangulation.
std::vector<cv::Vec6f> Mesher::createMesh2dImpl(
    const cv::Size& img_size,
    const KeypointsCV& keypoints_to_triangulate,
    MeshIndices* vtx_indices) {
  // Nothing to triangulate.
  if (keypoints_to_triangulate.size() == 0) return std::vector<cv::Vec6f>();

  // Rectangle to be used with Subdiv2D.
  // https://answers.opencv.org/question/180984/out-of-range-error-in-delaunay-triangulation/
  static const cv::Rect2f rect(0.0, 0.0, img_size.width, img_size.height);
  // subdiv has the delaunay triangulation function
  cv::Subdiv2D subdiv(rect);
  subdiv.initDelaunay(rect);
  // subdiv.swapEdges()

  // TODO Luca: there are kpts outside image, probably from tracker. This
  // check should be in the tracker. Actually seems to be issue in subdiv...
  // IT IS OPENCV subdiv that is messing up...
  // -> Make sure we only pass keypoints inside the image!
  KeypointsCV keypoints_inside_image;
  keypoints_inside_image.reserve(keypoints_to_triangulate.size());
  for (const KeypointCV& kp : keypoints_to_triangulate) {
    if (rect.contains(kp)) {
      // TODO(Toni): weirdly enough rect.contains does not quite work with
      // (-0.0 - epsilon) values... it still considers them inside...
      if (kp.x >= 0.0 && kp.y >= 0.0) {
        keypoints_inside_image.push_back(kp);
      } else {
        LOG(ERROR) << "Keypoint with negative coords: \n"
                   << "x: " << kp.x << ", y:" << kp.y;
      }
    } else {
      VLOG(1) << "createMesh2D - error, keypoint out of image frame: \n"
              << "Keypoint 2D: " << kp
              << "Rectangle Image size (x, y, height, width): " << rect.x
              << ", " << rect.y << ", " << rect.height << ", " << rect.width;
    }
  }

  // Perform 2D Delaunay triangulation.
  try {
    subdiv.insert(keypoints_inside_image);
  } catch (...) {
    LOG(ERROR) << "Keypoints supposedly inside the image:";
    for (const KeypointCV& kp : keypoints_inside_image) {
      LOG(ERROR) << "x: " << kp.x << ", y: " << kp.y;
    }
    LOG(ERROR) << "CreateMesh2D: subdiv.insert error (2). "
               << "A point is outside of the triangulation specified rect...\n"
               << "Rectangle size (x, y, height, width): " << rect.x << ", "
               << rect.y << ", " << rect.height << ", " << rect.width << '\n'
               << "Keypoints to triangulate: "
               << keypoints_to_triangulate.size() << '\n'
               << "Keypoints inside image: " << keypoints_inside_image.size();
  }

  // getTriangleList returns some spurious triangle with vertices outside
  // image
  // TODO I think that the spurious triangles are due to ourselves sending
  // keypoints out of the image... Compute actual triangulation.
  std::vector<cv::Vec6f> tri_2d;
  subdiv.getTriangleList(tri_2d);

  // If requested, also return the unique ids of the vertices of each triangle.
  // subdiv.locate has a bug, let us just use hashes of vertices...
  if (vtx_indices) {
    vtx_indices->clear();
    vtx_indices->reserve(tri_2d.size());
  }

  // Retrieve "good triangles" (all vertices are inside image).
  std::vector<cv::Vec6f> good_triangulation;
  good_triangulation.reserve(tri_2d.size());
  for (const cv::Vec6f& tri : tri_2d) {
    if (rect.contains(cv::Point2f(tri[0], tri[1])) &&
        rect.contains(cv::Point2f(tri[2], tri[3])) &&
        rect.contains(cv::Point2f(tri[4], tri[5]))) {
      // Triangle with all vertices inside image
      good_triangulation.push_back(tri);

      // Find vertex ids!
      if (vtx_indices) {
        TriVtxIndices tri_vtx_indices;
        // Iterate over each vertex (pixel) of the triangle.
        for (size_t j = 0u; j < tri.rows / 2u; j++) {
          // Extract vertex == pixel.
          const cv::Point2f pixel(tri[j * 2u], tri[j * 2u + 1u]);
          // Find vertex ids
          tri_vtx_indices[j] =
              UtilsNumerical::hashPair(std::make_pair(pixel.x, pixel.y));
        }
        vtx_indices->push_back(tri_vtx_indices);
      }

    } else {
      LOG_EVERY_N(ERROR, 100)
          << "Delaunay Triangle out of image (size: x: " << rect.x
          << ", y: " << rect.y << ", height: " << rect.height << ", width "
          << rect.width << "\n Triangle: x, y: \n"
          << tri[0] << ", " << tri[1] << '\n'
          << tri[2] << ", " << tri[3] << '\n'
          << tri[4] << ", " << tri[5];
    }
  }
  return good_triangulation;
}

/* -------------------------------------------------------------------------- */
// Create a 2D mesh from 2D corners in an image
std::vector<cv::Vec6f> Mesher::createMesh2D(
    const Frame& frame,
    const std::vector<size_t>& selected_indices) {
  // Sanity check.
  const size_t& n_landmarks = frame.landmarks_.size();
  const size_t& n_keypoints = frame.keypoints_.size();
  CHECK_EQ(n_landmarks, n_keypoints)
      << "Frame: wrong dimension for the landmarks";

  const cv::Size& frame_size = frame.img_.size();
  cv::Rect2i rect(0, 0, frame_size.width, frame_size.height);

  // Add points from Frame.
  std::vector<cv::Point2f> keypoints_to_triangulate;
  for (const size_t& i : selected_indices) {
    CHECK_LT(i, n_landmarks);
    CHECK_LT(i, n_keypoints);
    const KeypointCV& keypoint_i = frame.keypoints_.at(i);
    if (frame.landmarks_.at(i) != -1 && rect.contains(keypoint_i)) {
      // Only for valid keypoints (some keypoints may
      // end up outside image after tracking which causes subdiv to crash).
      keypoints_to_triangulate.push_back(keypoint_i);
    }
  }
  return createMesh2dImpl(frame_size, keypoints_to_triangulate);
}

/* -------------------------------------------------------------------------- */
void Mesher::createMesh2dStereo(
    std::vector<cv::Vec6f>* triangulation_2D,
    const LandmarkIds& landmarks,
    const std::vector<KeypointStatus>& keypoints_status,
    const KeypointsCV& keypoints,
    const BearingVectors& keypoints_3d,
    const cv::Size& img_size,
    std::vector<std::pair<LandmarkId, gtsam::Point3>>* lmk_with_id_stereo) {
  // triangulation_2D is compulsory, lmk_with_id_stereo is optional.
  CHECK_NOTNULL(triangulation_2D);

  // Sanity check.
  CHECK_EQ(landmarks.size(), keypoints_status.size())
      << "StereoFrame: wrong dimension for the landmarks.";

  // Create mesh including indices of keypoints with valid 3D
  // (which have right px).
  std::vector<cv::Point2f> keypoints_for_mesh;
  for (size_t i = 0u; i < landmarks.size(); i++) {
    if (keypoints_status.at(i) == KeypointStatus::VALID &&
        landmarks.at(i) != -1) {
      // Add keypoints for mesh 2d.
      keypoints_for_mesh.push_back(keypoints.at(i));

      // Store corresponding landmarks.
      // These points are in stereo camera and are not in VIO, but have lmk id.
      if (lmk_with_id_stereo != nullptr) {
        const gtsam::Point3& p_i_camera_left =
            gtsam::Point3(keypoints_3d.at(i));
        lmk_with_id_stereo->push_back(
            std::make_pair(landmarks.at(i), p_i_camera_left));
      }
    }
  }

  // Get a triangulation for all valid keypoints.
  *triangulation_2D = createMesh2dImpl(img_size, keypoints_for_mesh);
}

}  // namespace VIO
