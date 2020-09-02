/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VisionFrontEnd.cpp
 * @brief  Class describing an abstract VIO Frontend
 * @author Marcus Abate
 */

#include "kimera-vio/frontend/VisionFrontEnd.h"

DEFINE_bool(visualize_feature_tracks, true, "Display feature tracks.");
DEFINE_bool(visualize_frontend_images,
            false,
            "Display images in frontend logger for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(save_frontend_images,
            false,
            "Save images in frontend logger to disk for debugging (only use "
            "if in sequential mode, otherwise expect segfaults). ");
DEFINE_bool(log_feature_tracks, false, "Display/Save feature tracks images.");
DEFINE_bool(log_mono_tracking_images,
            false,
            "Display/Save stereo tracking rectified and unrectified images.");
