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
 * \brief Rose mask construction from NGHolder.
 */
#include "ng_fixed_width.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "rose/rose_build.h"
#include "util/container.h"
#include "ue2common.h"

#include <algorithm>
#include <iterator>
#include <set>

using namespace std;

namespace ue2 {

static
bool findMask(const NGHolder &g, vector<CharReach> *mask, bool *anchored,
              flat_set<ReportID> *reports) {
    DEBUG_PRINTF("looking for a mask pattern\n");
    set<NFAVertex> s_succ;
    insert(&s_succ, adjacent_vertices(g.start, g));

    set<NFAVertex> sds_succ;
    insert(&sds_succ, adjacent_vertices(g.startDs, g));

    *anchored = sds_succ.size() == 1; /* sds itself */
    bool floating = is_subset_of(s_succ, sds_succ);

    DEBUG_PRINTF("sds %zu s %zu%s%s\n", sds_succ.size(), s_succ.size(),
                 *anchored ? " anchored" : "", floating ? " floating" : "");
    if (!*anchored && !floating) {
        DEBUG_PRINTF("semi-anchored\n");
        return false;
    }

    set<NFAVertex> &succs = *anchored ? s_succ : sds_succ;
    succs.erase(g.startDs);
    if (succs.size() != 1) {
        DEBUG_PRINTF("branchy root\n");
        return false;
    }

    NFAVertex u = *anchored ? g.start : g.startDs;
    NFAVertex v = *succs.begin();

    while (true) {
        DEBUG_PRINTF("validating vertex %zu\n", g[v].index);

        assert(v != g.acceptEod);

        // If we've reached an accept, we MAY have found a valid Rose pattern
        if (v == g.accept) {
            DEBUG_PRINTF("accept\n");
            insert(reports, g[u].reports);
            return true;
        }

        mask->push_back(g[v].char_reach);

        if (out_degree(v, g) != 1) {
            DEBUG_PRINTF("out_degree != 1\n");
            return false; /* not a chain */
        }

        u = v;
        v = *adjacent_vertices(v, g).first;

        if (in_degree(v, g) != 1) {
            DEBUG_PRINTF("blargh\n"); /* picks up cases where there is no path
                                        * to case accept (large cycles),
                                        * ensures term */
            return false;
        }
    }
}

bool handleFixedWidth(RoseBuild &rose, const NGHolder &g, const Grey &grey) {
    if (!grey.roseMasks) {
        return false;
    }

    if (in_degree(g.acceptEod,g) != 1) {
        DEBUG_PRINTF("EOD anchoring not supported\n");
        return false;
    }

    flat_set<ReportID> reports;
    bool anchored = false;
    vector<CharReach> mask;

    if (!findMask(g, &mask, &anchored, &reports)) {
        return false;
    }

    DEBUG_PRINTF("%smasky masky\n", anchored ? "anchored " : "");

    assert(!mask.empty());
    assert(!reports.empty());

    if (rose.add(anchored, mask, reports)) {
        DEBUG_PRINTF("added as rose mask\n");
        return true;
    } else {
        DEBUG_PRINTF("failed to add masky\n");
        return false;
    }
}

} // namespace ue2
