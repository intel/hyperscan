/*
 * Copyright (c) 2015, Intel Corporation
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
 * \brief Short Exhaustible Passthroughs.
 *
 * Analysis code for determining whether a graph should be treated specially
 * because it is short and contains exhaustible reports; typically we turn
 * these into outfixes rather than risk them becoming Rose literals.
 *
 * For example, the pattern:
 *
 *     /[a-f]/H
 *
 * ... is far better suited to becoming a small outfix that generates one match
 * and goes dead than being split into six one-byte Rose literals that end up
 * in the literal matcher.
 */
#include "ng_sep.h"

#include "grey.h"
#include "ng_holder.h"
#include "ng_reports.h"
#include "ng_util.h"
#include "ue2common.h"
#include "util/graph_range.h"

using namespace std;

namespace ue2 {

static
bool checkFromVertex(const NGHolder &g, NFAVertex start) {
    for (auto v : adjacent_vertices_range(start, g)) {
        if (v == g.startDs) {
            continue;
        }

        assert(!is_special(v, g)); /* should not be vacuous */

        if (!edge(g.startDs, v, g).second) { /* only floating starts */
            return false;
        } else if (out_degree(v, g) == 1
                   && edge(v, g.accept, g).second) { /* only floating end */
            ; /* possible sep */
        } else {
            return false;
        }
    }
    return true;
}

bool isSEP(const NGHolder &g, const ReportManager &rm, const Grey &grey) {
    if (!grey.mergeSEP || !can_exhaust(g, rm)) {
        return false;
    }

    if (!checkFromVertex(g, g.start) || !checkFromVertex(g, g.startDs)) {
        return false;
    }

    assert(out_degree(g.start, g) || proper_out_degree(g.startDs, g));

    DEBUG_PRINTF("graph is an SEP\n");
    return true;
}

} // namespace ue2
