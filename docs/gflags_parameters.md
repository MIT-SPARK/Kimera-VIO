## SparkVIO GFLAGS

The following are the gflags accepted by SparkVIO, this has been generated using the SparkVIO executable with the `--help` flag: `./build/stereoVIOEuroc --help`

[...] Skipping gflags and glog related flags...

  * Flags from SparkVio.cpp:

    * dataset_type (Type of parser to use:
      0: EuRoC
      1: Kitti) type: int32 default: 0
    * parallel_run (Run parallelized pipeline.) type: bool default: false

  * Flags from LoggerMatlab.cpp:

    * output_path (Path where to store VIO's log output.) type: string
      default: "./"

  * Flags from RegularVioBackend.cpp:

    * convert_extra_smart_factors_to_proj_factors (Whether to convert all smart
      factors in time horizon to projection factors, instead of just the ones in current frame.) type: bool default: true

    * max_parallax (Maximum parallax to be considered correct. This is a patch to remove outliers when using mono and stereo projection factors.) type: double default: 150

    * min_num_obs_for_proj_factor (If the smart factor has less than x number of observations, then do not consider the landmark as valid to transform to proj. This param is different from the one counting the track length as it controls only the quality of the subsequent proj factors, and has no effect on how the smart factors are created. This param is very correlated to the one we use to get lmks in time horizon (aka min_age) since only the ones that have reached the mesher and have been clustered will have the possibility of being here, so this param must be higher than the min_age one to have any impact.) type: int32 default: 4 

    * min_num_of_observations (Minimum number of observations for a feature track to be added in the optimization problem (corresponds to number of measurements in smart factors. Only insert feature tracks of length at least 2 (otherwise uninformative).) type: int32 default: 2

     * min_num_of_plane_constraints_to_add_factors (Minimum number of plane constraints to ) type: int32 default: 20

    * min_num_of_plane_constraints_to_avoid_seg_fault (Minimum number of
      constraints from landmark to plane to keep in order to avoid seg fault when removing factors for a specific plane. If all the factors are removed, then ISAM2 will seg fault, check issue:https://github.mit.edu/lcarlone/VIO/issues/32.) type: int32 default: 3

    * min_num_of_plane_constraints_to_remove_factors (Number of constraints for a plane to be considered underconstrained when trying to remove old regularity factors. If a plane is thought to be underconstrained, we'll try to remove it from the optimization or set a prior to it (depending on the use_unstable_plane_removal flag.) type: int32 default: 10

     * prior_noise_sigma_distance (Sigma for the noise model of the prior on the distance of the plane.) type: double default: 0.10000000000000001 

     * prior_noise_sigma_normal (Sigma for the noise model of the prior on the normal of the plane.) type: double default: 0.10000000000000001 

     * remove_old_reg_factors (Remove regularity factors for those landmarks that were originally associated to the plane, but which are not anymore.) type: bool default: true

    * use_unstable_plane_removal (Remove planes from optimization using unstable implementation, which tries to remove all factors attached to the plane so that ISAM2 deletes it. Unfortunately, ISAM2 has a bug and leads to seg faults if we do so. The stable implementation instead puts a prior on the plane and removes as many factors from the plane as possible to avoid seg fault.) type: bool default: false

  * Flags from StereoFrame.cpp:

    * images_rectified (Input image data already rectified.) type: bool
      default: false

  * Flags from VioBackend.cpp:
    * compute_state_covariance (Flag to compute state covariance from
      optimization Backend) type: bool default: false
    * debug_graph_before_opt (Store factor graph before optimization for later
      printing if the optimization fails.) type: bool default: false
    * max_number_of_cheirality_exceptions (Sets the maximum number of times we
      process a cheirality exception for a given optimization problem. This is
      to avoid too many recursive calls to update the smoother) type: int32
      default: 5
    * process_cheirality (Handle cheirality exception by removing problematic
      landmarks and re-running optimization.) type: bool default: false

  * Flags from Visualizer3D.cpp:

    * displayed_trajectory_length (Set length of plotted trajectory.If -1 then all the trajectory is plotted.) type: int32 default: 50

    * log_accumulated_mesh (Accumulate the mesh when logging.) type: bool
      default: false
    * log_mesh (Log the mesh at time horizon.) type: bool default: false
    * mesh_representation (Mesh representation:
       0: Points, 1: Surface, 2: Wireframe) type: int32 default: 1
    * mesh_shading (Mesh shading:
       0: Flat, 1: Gouraud, 2: Phong) type: int32 default: 0
    * set_mesh_ambient (Whether to use ambient light for the mesh.) type: bool
      default: false
    * set_mesh_lighting (Whether to use lighting for the mesh.) type: bool
      default: false
    * texturize_3d_mesh (Whether you want to add texture to the 3dmesh. The
      texture is taken from the image frame.) type: bool default: false
    * visualize_convex_hull (Enable convex hull visualization.) type: bool
      default: false
    * visualize_load_mesh_filename (Load a mesh in the visualization, i.e. to
      visualize ground-truth point cloud from Euroc's Vicon dataset.)
      type: string default: ""
    * visualize_mesh (Enable 3D mesh visualization.) type: bool default: false
    * visualize_mesh_2d (Visualize mesh 2D.) type: bool default: false
    * visualize_mesh_2d_filtered (Visualize mesh 2D filtered.) type: bool
      default: false
    * visualize_mesh_in_frustum (Enable mesh visualization in camera frustum.)
      type: bool default: false
    * visualize_mesh_with_colored_polygon_clusters (Color the polygon clusters
      according to their cluster id.) type: bool default: false
    * visualize_plane_constraints (Enable plane constraints visualization.)
      type: bool default: false
    * visualize_plane_label (Enable plane label visualization.) type: bool
      default: false
    * visualize_planes (Enable plane visualization.) type: bool default: false
    * visualize_point_cloud (Enable point cloud visualization.) type: bool
      default: true
    * visualize_semantic_mesh (Color the 3d mesh according to their semantic
      labels.) type: bool default: false

  * Flags from DataSource.cpp:
    * backend_type (Type of vioBackend to use:
      0: VioBackend
      1: RegularVioBackend) type: int32 default: 0
    * dataset_path (Path of dataset (i.e. Euroc, /Users/Luca/data/MH_01_easy).)
      type: string default: "/Users/Luca/data/MH_01_easy"
    * final_k (Final frame to finish processing dataset, subsequent frames will
      not be used.) type: int64 default: 10000
    * initial_k (Initial frame to start processing dataset, previous frames will
      not be used.) type: int64 default: 50
    * tracker_params_path (Path to tracker user-defined parameters.)
      type: string default: ""
    * vio_params_path (Path to vio user-defined parameters.) type: string
      default: ""

  * Flags from ETH_parser.cpp:
    * skip_n_end_frames (Number of final frames to skip.) type: int32
      default: 100
    * skip_n_start_frames (Number of initial frames to skip.) type: int32
      default: 10

  * Flags from OnlineGravityAlignment.cpp:
    * camera_pim_delta_difference (Maximum tolerable difference in time
      interval.) type: double default: 0.0050000000000000001
    * gravity_tolerance_linear (Maximum gravity tolerance for linear alignment.)
      type: double default: 9.9999999999999995e-07
    * gravity_tolerance_refinement (Maximum gravity tolerance for refinement.)
      type: double default: 0.10000000000000001
    * gyroscope_residuals (Maximum allowed gyroscope residual after
      pre-integrationcorrection.) type: double default: 0.050000000000000003
    * num_iterations_gravity_refinement (Number of iterations for gravity
      refinement.) type: int32 default: 4
    * rotation_noise_prior (Rotation noise for Rot3 priors in AHRS gyroscope
      estimation.) type: double default: 0.01
    * use_ahrs_estimator (Use AHRS gyroscope bias estimator instead of linear
      1st order approximation.) type: bool default: false

  * Flags from Mesher.cpp:
    * add_extra_lmks_from_stereo (Add extra landmarks that are stereo
      triangulated to the mesh. WARNING this is computationally expensive.)
      type: bool default: false
    * compute_per_vertex_normals (Compute per-vertex normals,this is for
      visualization in RVIZ, it is costly!) type: bool default: false
    * distance_tolerance_plane_plane_association (Distance tolerance for a plane
      to be associated to another plane.) type: double
      default: 0.20000000000000001
    * distance_tolerance_polygon_plane_association (Tolerance for a polygon
      vertices to be considered close to a plane.) type: double
      default: 0.10000000000000001
    * do_double_association (Do double plane association of Backend plane with
      multiple segmented planes. Otherwise search for another possible Backend
      plane for the segmented plane.) type: bool default: true
    * hist_2d_distance_bins (.) type: int32 default: 40
    * hist_2d_distance_range_max (.) type: double default: 6
    * hist_2d_distance_range_min (.) type: double default: -6
    * hist_2d_gaussian_kernel_size (Kernel size for gaussian blur of 2D
      histogram.) type: int32 default: 3
    * hist_2d_min_dist_btw_local_max (Minimum distance between local maximums to
      be considered different.) type: int32 default: 5
    * hist_2d_min_support (Minimum number of votes to consider a local maximum
      in 2D histogram a valid peak.) type: int32 default: 20
    * hist_2d_nr_of_local_max (Number of local maximums to extract in 2D
      histogram.) type: int32 default: 2
    * hist_2d_theta_bins (.) type: int32 default: 40
    * hist_2d_theta_range_max (.) type: double default: 3.1415926535897931
    * hist_2d_theta_range_min (.) type: double default: 0
    * log_histogram_1D (Log 1D histogram to file. It logs the raw and smoothed
      histogram.) type: bool default: false
    * log_histogram_2D (Log 2D histogram to file. It logs the raw and smoothed
      histogram.) type: bool default: false
    * max_triangle_side (Maximum allowed side for a triangle.) type: double
      default: 0.5
    * min_elongation_ratio (Minimum allowed elongation ratio for a triangle.)
      type: double default: 0.5
    * min_ratio_btw_largest_smallest_side (Minimum ratio between largest and
      smallest side of a triangle.) type: double default: 0.5
    * normal_tolerance_horizontal_surface (Normal tolerance for a polygon to be
      considered parallel to the ground (0.087 === 10 deg. aperture).)
      type: double default: 0.010999999999999999
    * normal_tolerance_plane_plane_association (Normal tolerance for a plane to
      be associated to another plane (0.087 === 10 deg. aperture).)
      type: double default: 0.010999999999999999
    * normal_tolerance_polygon_plane_association (Tolerance for a polygon's
      normal and a plane's normal to be considered equal (0.087 === 10 deg.
      aperture).) type: double default: 0.010999999999999999
    * normal_tolerance_walls (Normal tolerance for a polygon to be considered
      perpendicular to the vertical direction.) type: double
      default: 0.016500000000000001
    * only_associate_a_polygon_to_a_single_plane (Limit association of a
      particular polygon to a single plane. Otherwise, a polygon can be
      associated to different planes  depending on the tolerance given by the
      thresholds set for association.) type: bool default: true
    * only_use_non_clustered_points (Only use points that have not been
      clustered in a plane already when filling both histograms.) type: bool
      default: true
    * reduce_mesh_to_time_horizon (Reduce mesh vertices to the landmarks
      available in current optimization's time horizon.) type: bool
      default: true
    * return_mesh_2d (Return mesh 2d with pixel positions, i.e. for semantic
      segmentation) type: bool default: false
    * visualize_histogram_1D (Visualize 1D histogram.) type: bool default: false
    * visualize_histogram_2D (Visualize 2D histogram.) type: bool default: false
    * z_histogram_bins (Number of bins for z histogram.) type: int32
      default: 512
    * z_histogram_gaussian_kernel_size (Kernel size for gaussian blur of z
      histogram (should be odd).) type: int32 default: 5
    * z_histogram_max_number_of_peaks_to_select (Maximum number of peaks to
      select in z histogram.) type: int32 default: 3
    * z_histogram_max_range (Maximum z value for z histogram.) type: double
      default: 3
    * z_histogram_min_range (Minimum z value for z histogram.) type: double
      default: -0.75
    * z_histogram_min_separation (If two peaks in the z histogram lie within
      min_separation , only the one with maximum support will be taken (sisable
      by setting < 0).) type: double default: 0.10000000000000001
    * z_histogram_min_support (Minimum number of votes for a value in the z
      histogram to be considered a peak.) type: double default: 50
    * z_histogram_peak_per (Extra peaks in the z histogram will be only
      considered if it has a value of peak_per (< 1) times the value of the max
      peak in the histogram.) type: double default: 0.5
    * z_histogram_window_size (Window size of z histogram to calculate
      derivatives, not sure in fact.) type: int32 default: 3

  * Flags from Pipeline.cpp:
    * between_translation_bundle_adjustment (Between factor precision for bundle
      adjustment in initialization.) type: double default: 0.5
    * deterministic_random_number_generator (If true the random number generator
      will consistently output the same sequence of pseudo-random numbers for
      every run (use it to have repeatable output). If false the random number
      generator will output a different sequence for each run.) type: bool
      default: false
    * extract_planes_from_the_scene (Whether to use structural regularities in
      the scene,currently only planes.) type: bool default: false
    * log_output (Log output to matlab.) type: bool default: false
    * min_num_obs_for_mesher_points (Minimum number of observations for a smart
      factor's landmark to to be used as a 3d point to consider for the
      mesher.) type: int32 default: 4
    * num_frames_vio_init (Minimum number of frames for the online
      gravity-aligned initialization.) type: int32 default: 25
    * outlier_rejection_bundle_adjustment (Outlier rejection for bundle
      adjustment in initialization.) type: double default: 30
    * record_video_for_viz_3d (Record a video as a sequence of screenshots of
      the 3d viz window) type: bool default: false
    * regular_vio_backend_modality (Modality for regular Vio Backend, currently
      supported:
      0: Structureless (equivalent to normal VIO)
      1: Projection (as if it was a typical VIO Backend with projectionfactors)
      2: Structureless and Projection, sets to projection factors the
      structureless factors that are supposed to be in a regularity.
      3: Projection and Regularity, sets all structureless factors toprojection
      factors and adds regularity factors to a subset.
      4: Structureless, Projection and Regularity factors used.) type: int32
      default: 4
    * smart_noise_sigma_bundle_adjustment (Smart noise sigma for bundle
      adjustment in initialization.) type: double default: 1.5
    * use_feature_selection (Enable smart feature selection.) type: bool
      default: false
    * use_lcd (Enable LoopClosureDetector processing in pipeline.) type: bool
      default: false
    * visualize (Enable overall visualization.) type: bool default: true
    * visualize_lmk_type (Enable landmark type visualization.) type: bool
      default: false
    * viz_type (
      0: POINTCLOUD, visualize 3D VIO points (no repeated point)
      are re-plotted at every frame)
      1: MESH2D, only visualizes 2D mesh on image
      2: MESH2Dsparse, visualize a 2D mesh of (right-valid) keypoints
      discarding triangles corresponding to non planar obstacles
      3: MESH2DTo3Dsparse, get a 3D mesh from a 2D triangulation of the
      (right-VALID) keypoints in the left frame and filters out triangles 
      4: NONE, does not visualize map
      ) type: int32 default: 0
