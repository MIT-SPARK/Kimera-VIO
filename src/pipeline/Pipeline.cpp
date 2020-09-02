/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   Pipeline.cpp
 * @brief  Implements abstract VIO pipeline workflow.
 * @author Antoni Rosinol
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/Pipeline.h"

DEFINE_bool(log_output, false, "Log output to CSV files.");
DEFINE_bool(extract_planes_from_the_scene,
            false,
            "Whether to use structural regularities in the scene,"
            "currently only planes.");

DEFINE_bool(visualize, true, "Enable overall visualization.");
DEFINE_bool(visualize_lmk_type, false, "Enable landmark type visualization.");
DEFINE_int32(viz_type,
             0,
             "0: MESH2DTo3Dsparse, get a 3D mesh from a 2D triangulation of "
             "the (right-VALID).\n"
             "1: POINTCLOUD, visualize 3D VIO points (no repeated point)\n"
             "are re-plotted at every frame).\n"
             "keypoints in the left frame and filters out triangles \n"
             "2: NONE, does not visualize map.");

DEFINE_bool(deterministic_random_number_generator,
            false,
            "If true the random number generator will consistently output the "
            "same sequence of pseudo-random numbers for every run (use it to "
            "have repeatable output). If false the random number generator "
            "will output a different sequence for each run.");
DEFINE_int32(min_num_obs_for_mesher_points,
             4,
             "Minimum number of observations for a smart factor's landmark to "
             "to be used as a 3d point to consider for the mesher.");

DEFINE_bool(use_lcd,
            false,
            "Enable LoopClosureDetector processing in pipeline.");
