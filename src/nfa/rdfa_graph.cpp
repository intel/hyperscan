/*
 * Copyright (c) 2016, Intel Corporation
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


#include "rdfa_graph.h"

#include "rdfa.h"
#include "util/container.h"

#include <vector>

using namespace std;

namespace ue2 {

RdfaGraph::RdfaGraph(const raw_dfa &rdfa) {
    RdfaGraph &g = *this;

    vector<RdfaGraph::vertex_descriptor> verts;
    verts.reserve(rdfa.states.size());
    for (dstate_id_t i = 0; i < rdfa.states.size(); i++) {
        verts.push_back(add_vertex(g));
        assert(g[verts.back()].index == i);
    }

    symbol_t symbol_end = rdfa.alpha_size - 1;

    flat_set<dstate_id_t> local_succs;
    for (dstate_id_t i = 0; i < rdfa.states.size(); i++) {
        local_succs.clear();
        for (symbol_t s = 0; s < symbol_end; s++) {
            dstate_id_t next = rdfa.states[i].next[s];
            if (contains(local_succs, next)) {
                continue;
            }
            DEBUG_PRINTF("%hu->%hu\n", i, next);
            add_edge(verts[i], verts[next], g);
            local_succs.insert(next);
        }
    }
}

}
