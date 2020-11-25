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
 * @file ThirdPartyWrapper.cpp
 * @brief Wrapper for functions of DLoopDetector (see copyright notice above).
 * @author Marcus Abate
 */

#include <algorithm>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/loopclosure/LcdThirdPartyWrapper.h"

namespace VIO {

/* ------------------------------------------------------------------------ */
LcdThirdPartyWrapper::LcdThirdPartyWrapper(
    const LoopClosureDetectorParams& params)
    : lcd_params_(params),
      temporal_entries_(0),
      latest_matched_island_(),
      latest_query_id_(FrameId(0)) {}

/* ------------------------------------------------------------------------ */
LcdThirdPartyWrapper::~LcdThirdPartyWrapper() {
  LOG(INFO) << "LcdThirdPartyWrapper desctuctor called.";
}

/* ------------------------------------------------------------------------ */
bool LcdThirdPartyWrapper::checkTemporalConstraint(const FrameId& id,
                                                   const MatchIsland& island) {
  // temporal_entries_ starts at zero and counts the number of
  if (temporal_entries_ == 0 || static_cast<int>(id - latest_query_id_) >
                                    lcd_params_.max_nrFrames_between_queries_) {
    temporal_entries_ = 1;
  } else {
    int a1 = static_cast<int>(latest_matched_island_.start_id_);
    int a2 = static_cast<int>(latest_matched_island_.end_id_);
    int b1 = static_cast<int>(island.start_id_);
    int b2 = static_cast<int>(island.end_id_);

    // Check that segments (a1, a2) and (b1, b2) have some overlap
    bool overlap = (b1 <= a1 && a1 <= b2) || (a1 <= b1 && b1 <= a2);
    bool gap_is_small = false;
    if (!overlap) {
      // Compute gap between segments (one of the two is negative)
      int d1 = static_cast<int>(latest_matched_island_.start_id_) -
               static_cast<int>(island.end_id_);
      int d2 = static_cast<int>(island.start_id_) -
               static_cast<int>(latest_matched_island_.end_id_);

      int gap = (d1 > d2 ? d1 : d2);  // Choose positive gap
      gap_is_small = gap <= lcd_params_.max_nrFrames_between_islands_;
    }

    if (overlap || gap_is_small) {
      temporal_entries_++;
    } else {
      temporal_entries_ = 1;
    }
  }

  latest_matched_island_ = island;
  latest_query_id_ = id;
  return temporal_entries_ > lcd_params_.min_temporal_matches_;
}

/* ------------------------------------------------------------------------ */
void LcdThirdPartyWrapper::computeIslands(
    DBoW2::QueryResults* q,
    std::vector<MatchIsland>* islands) const {
  CHECK_NOTNULL(q);
  CHECK_NOTNULL(islands);
  islands->clear();

  // The case of one island is easy to compute and is done separately
  if (q->size() == 1) {
    const DBoW2::Result& result = (*q)[0];
    const DBoW2::EntryId& result_id = result.Id;
    MatchIsland island(result_id, result_id, result.Score);
    island.best_id_ = result_id;
    island.best_score_ = result.Score;
    islands->push_back(island);
  } else if (!q->empty()) {
    // sort query results in ascending order of frame ids
    std::sort(q->begin(), q->end(), DBoW2::Result::ltId);

    // create long enough islands
    DBoW2::QueryResults::const_iterator dit = q->begin();
    int first_island_entry = static_cast<int>(dit->Id);
    int last_island_entry = static_cast<int>(dit->Id);

    // these are indices of q
    FrameId i_first = 0;
    FrameId i_last = 0;

    double best_score = dit->Score;
    DBoW2::EntryId best_entry = dit->Id;

    ++dit;
    for (FrameId idx = 1; dit != q->end(); ++dit, ++idx) {
      if (static_cast<int>(dit->Id) - last_island_entry <
          lcd_params_.max_intraisland_gap_) {
        last_island_entry = dit->Id;
        i_last = idx;
        if (dit->Score > best_score) {
          best_score = dit->Score;
          best_entry = dit->Id;
        }
      } else {
        // end of island reached
        int length = last_island_entry - first_island_entry + 1;
        if (length >= lcd_params_.min_matches_per_island_) {
          MatchIsland island =
              MatchIsland(first_island_entry,
                          last_island_entry,
                          computeIslandScore(*q, i_first, i_last));

          islands->push_back(island);
          islands->back().best_score_ = best_score;
          islands->back().best_id_ = best_entry;
        }

        // prepare next island
        first_island_entry = last_island_entry = dit->Id;
        i_first = i_last = idx;
        best_score = dit->Score;
        best_entry = dit->Id;
      }
    }
    // add last island
    // TODO: do we need this? why isn't it handled in prev for loop?
    if (last_island_entry - first_island_entry + 1 >=
        lcd_params_.min_matches_per_island_) {
      MatchIsland island = MatchIsland(first_island_entry,
                                       last_island_entry,
                                       computeIslandScore(*q, i_first, i_last));

      islands->push_back(island);
      islands->back().best_score_ = best_score;
      islands->back().best_id_ = static_cast<FrameId>(best_entry);
    }
  }
}

/* ------------------------------------------------------------------------ */
double LcdThirdPartyWrapper::computeIslandScore(const DBoW2::QueryResults& q,
                                                const FrameId& start_id,
                                                const FrameId& end_id) const {
  CHECK_GT(q.size(), start_id);
  CHECK_GT(q.size(), end_id);
  double score_sum = 0.0;
  for (FrameId id = start_id; id <= end_id; id++) {
    score_sum += q.at(id).Score;
  }

  return score_sum;
}

}  // namespace VIO
