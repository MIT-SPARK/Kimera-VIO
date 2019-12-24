/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   QueueSynchronizer.cpp
 * @brief  Implements temporal synchronization of queues.
 * @author Antoni Rosinol
 */

#include "kimera-vio/pipeline/QueueSynchronizer.h"

#include <numeric>  // for numeric_limits
#include <string>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/pipeline/PipelinePayload.h"

namespace VIO {}  // namespace VIO
