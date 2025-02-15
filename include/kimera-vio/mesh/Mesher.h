/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Mesher.h
 * @brief  Build 3D mesh from 2D mesh.
 * @author Antoni Rosinol
 */

#pragma once

#include <atomic>
#include <cstdlib>
#include <limits>  // for numeric_limits<>
#include <opencv2/opencv.hpp>
#include <optional>
#include <utility>  // for move
#include <vector>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/logging/Logger.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/Mesher-definitions.h"
#include "kimera-vio/utils/Histogram.h"
#include "kimera-vio/utils/Macros.h"

namespace VIO {

class Mesher {
 public:
  KIMERA_POINTER_TYPEDEFS(Mesher);
  KIMERA_DELETE_COPY_CONSTRUCTORS(Mesher);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Vector of triplets of vertex indices, each element corresponds to 3
  // triangle vertices ids.
  using VertexIdx = size_t;  //! A hash value (can be very large!).
  using TriVtxIndices = std::array<VertexIdx, 3>;
  using MeshIndices = std::vector<TriVtxIndices>;

 public:
  explicit Mesher(const MesherParams& mesher_params,
                  const bool& serialize_meshes = false);
  virtual ~Mesher() = default;

 public:
  /**
   * @brief Mesher::spinOnce Process only one minimal packet of information.
   * @param input Minimal input for the Mesher to do its work.
   * @return Mesher's output
   */
  virtual MesherOutput::UniquePtr spinOnce(const MesherInput& input);

 public:
  // Update mesh: update structures keeping memory of the map before
  // visualization. It also returns a mesh_2d which represents the triangulation
  // in the 2d image.
  //
  // Also provides an image of the 2d triangulation,
  // as well as a mesh2D that is linked to the mesh3D via the
  // landmark ids. Note that the mesh2D only contains those triangles
  // that have a corresponding polygon face in 3D.
  // Iterate over the mesh 2D, and use mesh3D getVertex to get the
  // 3D face from the 2D triangle.
  void updateMesh3D(const PointsWithIdMap& points_with_id_VIO,
                    const KeypointsCV& keypoints,
                    const std::vector<KeypointStatus>& keypoints_status,
                    const BearingVectors& keypoints_3d,
                    const LandmarkIds& landmarks,
                    const gtsam::Pose3& left_camera_pose,
                    Mesh2D* mesh_2d = nullptr,
                    std::vector<cv::Vec6f>* mesh_2d_for_viz = nullptr);

  /* ------------------------------------------------------------------------ */
  // Update mesh, but in a thread-safe way.
  void updateMesh3D(const MesherInput& mesher_payload,
                    Mesh2D* mesh_2d = nullptr,
                    std::vector<cv::Vec6f>* mesh_2d_for_viz = nullptr);

  /* ------------------------------------------------------------------------ */
  // Cluster planes from the mesh.
  void clusterPlanesFromMesh(std::vector<Plane>* planes,
                             const PointsWithIdMap& points_with_id_vio);

  /* ------------------------------------------------------------------------ */
  void appendNonVioStereoPoints(
      const LandmarkIds& landmarks,
      const std::vector<KeypointStatus>& keypoints_status,
      const BearingVectors& keypoints_3d,
      const gtsam::Pose3& left_cam_pose,
      PointsWithIdMap* points_with_id_stereo) const;

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from triangle cluster.
  void extractLmkIdsFromTriangleClusters(
      const std::vector<TriangleCluster>& triangle_cluster,
      const PointsWithIdMap& points_with_id_vio,
      LandmarkIds* lmk_ids) const;

  /**
   * @brief serializeMeshes Write meshes to file so that they can be loaded
   * later (Note: the function is not const bcs serialize needs to be non-const)
   * But we do not modify the meshes.
   */
  void serializeMeshes();

  /**
   * @brief deserializeMeshes Load meshes from a file where the meshes were
   * previously serialized.
   */
  void deserializeMeshes();

  static std::vector<cv::Vec6f> createMesh2D(
      const Frame& frame,
      const std::vector<size_t>& selected_indices);

  /**
   * @brief createMesh2D Creates a 2D Mesh out of given keypoints inside the
   * given image size. It effectively calls Delaunay triangulation over the
   * 2D keypoints and returns a Mesh2D (without lmk ids).
   * @param keypoints
   * @param image_size
   * @param mesh_2d 2D mesh that is delaunay triangulation of the given
   * keypoints.
   */
  static void createMesh2D(const KeypointsCV& keypoints, Mesh2D* mesh_2d) {
    CHECK_NOTNULL(mesh_2d);
    CHECK_EQ(mesh_2d->getNumberOfPolygons(), 0u);
    CHECK_EQ(mesh_2d->getNumberOfUniqueVertices(), 0u);
    MeshIndices vtx_indices;
    std::vector<cv::Vec6f> tri =
        Mesher::computeDelaunayTriangulation(keypoints, &vtx_indices);

    // Iterate over the 2d mesh triangles.
    for (size_t i = 0u; i < tri.size(); i++) {
      const cv::Vec6f& triangle_2d = tri.at(i);
      const TriVtxIndices& tri_vtx_indices = vtx_indices.at(i);

      // Iterate over each vertex (pixel) of the triangle.
      Mesh2D::Polygon polygon_2d;
      for (size_t j = 0u; j < triangle_2d.rows / 2u; j++) {
        const cv::Point2f vtx(triangle_2d[j * 2u], triangle_2d[j * 2u + 1u]);
        polygon_2d.push_back(Mesh2D::VertexType(tri_vtx_indices[j], vtx));
      }
      CHECK_EQ(polygon_2d.size(), mesh_2d->getMeshPolygonDimension());

      // Finally, add polygon to mesh
      mesh_2d->addPolygonToMesh(polygon_2d);
    }
  }

  /*
   * Triangulate support points for the requested image
   * Usage of Triangle inspired From: LIBELAS
   * http://www.cvlibs.net/software/libelas/
   */
  static std::vector<cv::Vec6f> computeDelaunayTriangulation(
      const KeypointsCV& keypoints,
      MeshIndices* vtx_indices = nullptr);

 private:
  // Provide Mesh 3D in read-only mode.
  // Not the nicest to send a const &, should maybe use shared_ptr
  inline const Mesh3D& get3DMesh() const { return mesh_3d_; }

  /* ------------------------------------------------------------------------ */
  // Reduce the 3D mesh to the current VIO lmks only.
  void updatePolygonMeshToTimeHorizon(
      const PointsWithIdMap& points_with_id_map,
      const gtsam::Pose3& leftCameraPose,
      double min_ratio_largest_smallest_side,
      double max_triangle_side,
      const bool& reduce_mesh_to_time_horizon = true);

  /* ------------------------------------------------------------------------ */
  // For a triangle defined by the 3d points p1, p2, and p3
  // compute ratio between largest side and smallest side (how elongated it is).
  double getRatioBetweenSmallestAndLargestSide(
      const double& d12,
      const double& d23,
      const double& d31,
      double* minSide_out = nullptr,
      double* maxSide_out = nullptr) const;

  /* ------------------------------------------------------------------------ */
  // For a triangle defined by the 3d points p1, p2, and p3
  // compute ratio between largest side and smallest side (how elongated it is)
  double getRatioBetweenTangentialAndRadialDisplacement(
      const Vertex3D& p1,
      const Vertex3D& p2,
      const Vertex3D& p3,
      const gtsam::Pose3& leftCameraPose) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers.
  void filterOutBadTriangles(const gtsam::Pose3& leftCameraPose,
                             double minRatioBetweenLargestAnSmallestSide,
                             double min_elongation_ratio,
                             double maxTriangleSide);

  /* ------------------------------------------------------------------------ */
  // Create a 3D mesh from a 2d mesh in pixel coordinates.
  // The 3D mesh is constructed by finding the 3D landmark corresponding to the
  // pixel in the 2d mesh. The correspondence is found using the frame
  // parameter. The 3D mesh contains, at any given time, only points that are in
  // points_with_id_map.
  void populate3dMeshTimeHorizon(  // cv::Vec6f assumes triangular mesh.
      const std::vector<cv::Vec6f>& mesh_2d_pixels,
      const PointsWithIdMap& points_with_id_map,
      const KeypointsCV& keypoints,
      const LandmarkIds& landmarks,
      const gtsam::Pose3& left_cam_pose,
      double min_ratio_largest_smallest_side,
      double min_elongation_ratio,
      double max_triangle_side,
      Mesh2D* mesh_2d = nullptr);

  /* ------------------------------------------------------------------------ */
  // Create a 3D mesh from 2D corners in an image.
  void populate3dMesh(  // cv::Vec6f assumes triangular mesh.
      const std::vector<cv::Vec6f>& mesh_2d_pixels,
      const PointsWithIdMap& points_with_id_map,
      const KeypointsCV& keypoints,
      const LandmarkIds& landmarks,
      const gtsam::Pose3& left_cam_pose,
      double min_ratio_largest_smallest_side,
      double min_elongation_ratio,
      double max_triangle_side,
      Mesh2D* mesh_2d = nullptr);

  /* ------------------------------------------------------------------------ */
  // Calculate normals of each polygon in the mesh.
  void calculateNormals(std::vector<cv::Point3f>* normals);

  /* ------------------------------------------------------------------------ */
  // Calculate normal of a triangle, and return whether it was possible or not.
  // Calculating the normal of aligned points in 3D is not possible...
  bool calculateNormal(const Vertex3D& p1,
                       const Vertex3D& p2,
                       const Vertex3D& p3,
                       cv::Point3f* normal) const;

  /* ------------------------------------------------------------------------ */
  // Is normal perpendicular to axis?
  bool isNormalPerpendicularToAxis(const cv::Point3f& axis,
                                   const cv::Point3f& normal,
                                   const double& tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Is normal around axis?
  bool isNormalAroundAxis(const cv::Point3f& axis,
                          const cv::Point3f& normal,
                          const double& tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Clusters normals given an axis, a set of normals and a
  // tolerance. The result is a vector of indices of the given set of normals
  // that are in the cluster.
  void clusterNormalsAroundAxis(const cv::Point3f& axis,
                                const std::vector<cv::Point3f>& normals,
                                const double& tolerance,
                                std::vector<int>* triangle_cluster);

  /* ------------------------------------------------------------------------ */
  // Clusters normals perpendicular to an axis. Given an axis, a set of normals
  // and a tolerance. The result is a vector of indices of the given set of
  // normals that are in the cluster.
  void clusterNormalsPerpendicularToAxis(
      const cv::Point3f& axis,
      const std::vector<cv::Point3f>& normals,
      const double& tolerance,
      std::vector<int>* cluster_normals_idx);

  /* ------------------------------------------------------------------------ */
  // Checks whether all points in polygon are closer than tolerance to the
  // plane.
  bool isPolygonAtDistanceFromPlane(const Mesh3D::Polygon& polygon,
                                    const double& plane_distance,
                                    const cv::Point3f& plane_normal,
                                    const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Checks whether the point is closer than tolerance to the plane.
  bool isPointAtDistanceFromPlane(const Vertex3D& point,
                                  const double& plane_distance,
                                  const cv::Point3f& plane_normal,
                                  const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Try to reject bad triangles, corresponding to outliers.
  bool isBadTriangle(const Mesh3D::Polygon& polygon,
                     const gtsam::Pose3& left_camera_pose,
                     const double& min_ratio_between_largest_an_smallest_side,
                     const double& min_elongation_ratio,
                     const double& max_triangle_side) const;

  /* ------------------------------------------------------------------------ */
  // Segment planes in the mesh:
  // Updates seed_planes lmk ids of the plane by using initial plane seeds.
  // Extracts new planes from the mesh.
  // WARNING: data association must be performed between seed_planes and
  // new_planes since both structures might have the same planes.
  void segmentPlanesInMesh(
      std::vector<Plane>* seed_planes,
      std::vector<Plane>* new_planes,
      const PointsWithIdMap& points_with_id_vio,
      const double& normal_tolerance_polygon_plane_association,
      const double& distance_tolerance_polygon_plane_association,
      const double& normal_tolerance_horizontal_surface,
      const double& normal_tolerance_walls);

  /* --------------------------------------------------------------------------
   */
  // Updates planes lmk ids field with a polygon vertices ids if this polygon
  // Output goes from 0 to 2*pi, as we are using atan2, which looks at sign
  // of arguments.
  double getLongitude(const cv::Point3f& triangle_normal,
                      const cv::Point3f& vertical) const;

  /* --------------------------------------------------------------------------
   */
  // Update plane lmk ids field, by looping over the mesh and stoting lmk ids of
  // the vertices of the polygons that are close to the plane.
  // It will append lmk ids to the ones already present in the plane.
  void updatePlanesLmkIdsFromMesh(
      std::vector<Plane>* planes,
      double normal_tolerance,
      double distance_tolerance,
      const PointsWithIdMap& points_with_id_vio) const;

  /* ------------------------------------------------------------------------ */
  // Updates planes lmk ids field with a polygon vertices ids if this polygon
  // is part of the plane according to given tolerance.
  // It can either associate a polygon only once to the first plane it matches,
  // or it can associate to multiple planes, depending on the flag passed.
  bool updatePlanesLmkIdsFromPolygon(
      std::vector<Plane>* seed_planes,
      const Mesh3D::Polygon& polygon,
      const size_t& triangle_id,
      const cv::Point3f& triangle_normal,
      double normal_tolerance,
      double distance_tolerance,
      const PointsWithIdMap& points_with_id_vio,
      bool only_associate_a_polygon_to_a_single_plane = false) const;

  /* --------------------------------------------------------------------------
   */
  // Segment new planes in the mesh.
  // Currently segments horizontal planes using z_components, which is
  // expected to be a cv::Mat z_components (1, 0, CV_32F);
  // And walls perpendicular to the ground, using a cv::Mat which is expected to
  // be a cv::Mat walls (0, 0, CV_32FC2), with first channel being theta (yaw
  // angle of the wall) and the second channel the distance of it.
  // points_with_id_vio is only used if we are using stereo points...
  void segmentNewPlanes(std::vector<Plane>* new_segmented_planes,
                        const cv::Mat& z_components,
                        const cv::Mat& walls);

  /* ------------------------------------------------------------------------ */
  // Segment wall planes.
  void segmentWalls(std::vector<Plane>* wall_planes,
                    size_t* plane_id,
                    const cv::Mat& walls);

  /* ------------------------------------------------------------------------ */
  // Segment new planes horizontal.
  void segmentHorizontalPlanes(std::vector<Plane>* horizontal_planes,
                               size_t* plane_id,
                               const Plane::Normal& normal,
                               const cv::Mat& z_components);

  /* ------------------------------------------------------------------------ */
  // Data association between planes:
  // It just outputs the set of planes that could not be associated.
  // It does not change the original planes.
  void associatePlanes(const std::vector<Plane>& segmented_planes,
                       const std::vector<Plane>& planes,
                       std::vector<Plane>* non_associated_planes,
                       const double& normal_tolerance,
                       const double& distance_tolerance) const;

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from a vector of triangle clusters.
  void extractLmkIdsFromVectorOfTriangleClusters(
      const std::vector<TriangleCluster>& triangle_cluster,
      const PointsWithIdMap& points_with_id_vio,
      LandmarkIds* lmk_ids) const;

  /* ------------------------------------------------------------------------ */
  // Extract lmk ids from triangle cluster.
  void extractLmkIdsFromTriangleCluster(
      const TriangleCluster& triangle_cluster,
      const PointsWithIdMap& points_with_id_vio,
      LandmarkIds* lmk_ids) const;

  /* ------------------------------------------------------------------------ */
  // Extracts lmk ids from a mesh polygon.
  // In case we are using extra lmks from stereo, then it makes sure that the
  // lmk ids are used in the optimization (they are present in time horizon:
  // meaning it checks that we can find the lmk id in points_with_id_vio...
  // WARNING: this function won't check that the original lmk_ids are in the
  // optimization (time-horizon)...
  void appendLmkIdsOfPolygon(const Mesh3D::Polygon& polygon,
                             LandmarkIds* lmk_ids,
                             const PointsWithIdMap& points_with_id_vio) const;

  /* ------------------------------------------------------------------------ */
  // Clones underlying data structures encoding the mesh.
  void getVerticesMesh(cv::Mat* vertices_mesh) const;
  void getPolygonsMesh(cv::Mat* polygons_mesh) const;

  /**
   * @brief createMesh2dImpl
   * @param img_size
   * @param keypoints_to_triangulate
   * @param vtx_indices Returns each vertex id of the vertices of the triangles
   * in order
   * @return
   */
  static std::vector<cv::Vec6f> createMesh2dImpl(
      const cv::Size& img_size,
      const KeypointsCV& keypoints_to_triangulate,
      MeshIndices* vtx_indices = nullptr);

  static void createMesh2dVIO(
      std::vector<cv::Vec6f>* triangulation_2D,
      const LandmarkIds& landmarks,
      const std::vector<KeypointStatus>& keypoints_status,
      const KeypointsCV& keypoints,
      const cv::Size& img_size,
      const PointsWithIdMap& pointsWithIdVIO);

  static void createMesh2dStereo(
      std::vector<cv::Vec6f>* triangulation_2D,
      const LandmarkIds& landmarks,
      const std::vector<KeypointStatus>& keypoints_status,
      const KeypointsCV& keypoints,
      const BearingVectors& keypoints_3d,
      const cv::Size& img_size,
      std::vector<std::pair<LandmarkId, gtsam::Point3>>* lmk_with_id_stereo =
          nullptr);

 private:
  // The current per-frame 2D mesh.
  Mesh2D mesh_2d_;
  // The 3D mesh.
  Mesh3D mesh_3d_;
  // The histogram of z values for vertices of polygons parallel to ground.
  Histogram z_hist_;
  // The 2d histogram of theta angle (latitude) and distance of polygons
  // perpendicular to the vertical (aka parallel to walls).
  Histogram hist_2d_;

  const MesherParams mesher_params_;
  std::unique_ptr<MesherLogger> mesher_logger_;
  const bool serialize_meshes_;
};

}  // namespace VIO
