/********************************************************************************
Copyright (c) 2015 Dorian Galvez-Lopez. http://doriangalvez.com
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
3. The original author of the work must be notified of any
   redistribution of source code or in binary form.
4. Neither the name of copyright holders nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL COPYRIGHT HOLDERS OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/

/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file ThirdPartyWrapper.h
 * @brief Wrapper for functions of DLoopDetector (see copyright notice above).
 * @author Marcus Abate
 */

#pragma once

#include <vector>

#include <DBoW2/QueryResults.h>

#include "kimera-vio/common/vio_types.h"
#include "kimera-vio/loopclosure/LoopClosureDetectorParams.h"

namespace VIO {

class LcdThirdPartyWrapper {
 public:
  KIMERA_POINTER_TYPEDEFS(LcdThirdPartyWrapper);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LcdThirdPartyWrapper);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /* ------------------------------------------------------------------------ */
  /** @brief Constructor: wrapper for functions of DLoopDetector that are used
   *  in the loop-closure-detection pipeline by LoopClosureDetector.
   * @param[in] params The LoopClosureDetectorParams object with all relevant
   *  parameters.
   */
  explicit LcdThirdPartyWrapper(const LoopClosureDetectorParams& params);

  /* ------------------------------------------------------------------------ */
  virtual ~LcdThirdPartyWrapper();

  /* ------------------------------------------------------------------------ */
  /** @brief Determines whether a frame meets the temoral constraint given by
   *  a MatchIsland.
   * @param[in] id The frame ID of the frame being processed in the database.
   * @param[in] island A MatchIsland representing several potential matches.
   * @return True if the constraint is met, false otherwise.
   */
  // TODO(marcus): unit tests
  bool checkTemporalConstraint(const FrameId& id, const MatchIsland& island);

  /* ------------------------------------------------------------------------ */
  /** @brief Computes the various islands created by a QueryResult, which is
   *  given by the OrbDatabase.
   * @param[in] q A QueryResults object containing all the resulting possible
   *  matches with a frame.
   * @param[out] A vector of MatchIslands, each of which is an island of
   *  nearby possible matches with the frame being queried.
   */
  // TODO(marcus): unit tests
  void computeIslands(DBoW2::QueryResults* q,
                      std::vector<MatchIsland>* islands) const;

 private:
  /* ------------------------------------------------------------------------ */
  /** @brief Compute the overall score of an island.
   * @param[in] q A QueryResults object containing all the possible matches
   *  with a frame.
   * @param[in] start_id The frame ID that starts the island.
   * @param[in] end_id The frame ID that ends the island.
   * @reutrn The score of the island.
   */
  double computeIslandScore(const DBoW2::QueryResults& q,
                            const FrameId& start_id,
                            const FrameId& end_id) const;

 private:
  LoopClosureDetectorParams lcd_params_;
  int temporal_entries_;
  MatchIsland latest_matched_island_;
  FrameId latest_query_id_;
};  // class LcdThirdPartyWrapper

}  // namespace VIO
