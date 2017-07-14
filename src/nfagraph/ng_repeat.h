/*
 * Copyright (c) 2015-2017, Intel Corporation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of Intel Corporation nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 * \brief Bounded repeat analysis.
 */

#ifndef NG_REPEAT_H
#define NG_REPEAT_H

#include "ng_holder.h"
#include "ue2common.h"
#include "nfa/repeat_internal.h"
#include "util/depth.h"
#include "util/flat_containers.h"

#include <map>
#include <vector>

namespace ue2 {

class NGHolder;
class ReportManager;
struct Grey;

/**
 * \brief Everything you need to know about a bounded repeat that we have
 * transformed.
 */
struct BoundedRepeatData {
    BoundedRepeatData(enum RepeatType type_in, const depth &a, const depth &z,
                      u32 minPeriod_in, NFAVertex cyc, NFAVertex pos,
                      const std::vector<NFAVertex> &tug_in)
        : type(type_in), repeatMin(a), repeatMax(z), minPeriod(minPeriod_in),
          cyclic(cyc), pos_trigger(pos), tug_triggers(tug_in) {}

    BoundedRepeatData() = delete; // no default construction allowed.

    enum RepeatType type;  //!< selected type based on bounds and structure
    depth repeatMin;       //!< minimum repeat bound
    depth repeatMax;       //!< maximum repeat bound
    u32 minPeriod;         //!< min trigger period
    NFAVertex cyclic;      //!< cyclic vertex representing repeat in graph
    NFAVertex pos_trigger; //!< positive trigger vertex
    std::vector<NFAVertex> tug_triggers; //!< list of tug trigger vertices
};

/**
 * \brief Run the bounded repeat analysis and transform the graph where
 * bounded repeats are found.
 *
 * \param h
 *      Graph to operate on.
 * \param rm
 *      ReportManager, or nullptr if the graph's reports are internal (e.g. for
 *      Rose use).
 * \param fixed_depth_tops
 *      Map of top to possible trigger depth.
 * \param triggers
 *      Map of top to the vector of triggers (i.e. preceding literals/masks)
 * \param repeats
 *      Repeat info is filled in for caller here.
 * \param streaming
 *      True if we're in streaming mode.
 * \param simple_model_selection
 *      Don't perform complex (and slow) model selection analysis, e.g.
 *      determining whether the repeat is sole entry.
 * \param grey
 *      Grey box object.
 * \param reformed_start_ds
 *      If supplied, this will be set to true if the graph was optimised for a
 *      leading first repeat, resulting in the output graph having no self-loop
 *      on startDs.
 */
void analyseRepeats(NGHolder &h, const ReportManager *rm,
            const std::map<u32, u32> &fixed_depth_tops,
            const std::map<u32, std::vector<std::vector<CharReach>>> &triggers,
            std::vector<BoundedRepeatData> *repeats, bool streaming,
            bool simple_model_selection, const Grey &grey,
            bool *reformed_start_ds = nullptr);

/**
 * \brief Information on repeats in a holder, returned from \ref findRepeats.
 */
struct GraphRepeatInfo {
    depth repeatMin; /**< minimum bound */
    depth repeatMax; /**< effective max bound */
    std::vector<NFAVertex> vertices; /**< vertices involved in repeat */
};

/**
 * \brief Provides information on repeats in the graph.
 */
void findRepeats(const NGHolder &h, u32 minRepeatVertices,
                 std::vector<GraphRepeatInfo> *repeats_out);

struct PureRepeat {
    CharReach reach;
    DepthMinMax bounds;
    flat_set<ReportID> reports;

    bool operator==(const PureRepeat &a) const {
        return reach == a.reach && bounds == a.bounds && reports == a.reports;
    }

    bool operator!=(const PureRepeat &a) const { return !(*this == a); }

    bool operator<(const PureRepeat &a) const {
        if (reach != a.reach) {
            return reach < a.reach;
        }
        if (bounds != a.bounds) {
            return bounds < a.bounds;
        }
        return reports < a.reports;
    }
};

/**
 * \brief Returns true and fills the given PureRepeat structure if the graph is
 * wholly a repeat over a single character class.
 *
 * For example, something like:
 *
 *     /^[a-z]{10,20}/
 *
 * - Note: graph must not use SDS or EOD.
 * - Note: \p PureRepeat::bounds::max is set to infinity if there is no upper
 *   bound on the repeat.
 */
bool isPureRepeat(const NGHolder &h, PureRepeat &r);

} // namespace ue2

#endif // NG_REPEAT_H
