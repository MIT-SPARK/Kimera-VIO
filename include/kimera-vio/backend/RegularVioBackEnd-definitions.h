/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   VioBackEnd-definitions.h
 * @brief  Definitions for VioBackEnd.
 * @author Antoni Rosinol
 */

#pragma once

namespace VIO {

enum class RegularBackendModality {
  //! Only use structureless factors, equiv to normal Vio.
  STRUCTURELESS = 0,
  //! Converts all structureless factors to projection factors
  PROJECTION = 1,
  //! Projection factors used for regularities.
  STRUCTURELESS_AND_PROJECTION = 2,
  //! Projection Vio + regularity factors.
  PROJECTION_AND_REGULARITY = 3,
  //! All types of factors used.
  STRUCTURELESS_PROJECTION_AND_REGULARITY = 4
};

}  // namespace VIO
