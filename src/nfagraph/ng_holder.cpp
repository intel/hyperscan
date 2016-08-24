/*
 * Copyright (c) 2015-2016, Intel Corporation
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

#include "ng_holder.h"

#include "ng_util.h"
#include "ue2common.h"

using namespace std;

namespace ue2 {

// internal use only
static NFAVertex addSpecialVertex(NGHolder &g, SpecialNodes id) {
    NFAVertex v(add_vertex(g));
    g[v].index = id;
    return v;
}

NGHolder::NGHolder(nfa_kind k)
 : kind (k),
   // add initial special nodes
   start(addSpecialVertex(*this, NODE_START)),
   startDs(addSpecialVertex(*this, NODE_START_DOTSTAR)),
   accept(addSpecialVertex(*this, NODE_ACCEPT)),
   acceptEod(addSpecialVertex(*this, NODE_ACCEPT_EOD)) {

    // wire up some fake edges for the stylized bits of the NFA
    add_edge(start, startDs, *this);
    add_edge(startDs, startDs, *this);
    add_edge(accept, acceptEod, *this);

    (*this)[start].char_reach.setall();
    (*this)[startDs].char_reach.setall();
}

NGHolder::~NGHolder(void) {
    DEBUG_PRINTF("destroying holder @ %p\n", this);
}

void clear_graph(NGHolder &h) {
    NGHolder::vertex_iterator vi, ve;
    for (tie(vi, ve) = vertices(h); vi != ve;) {
        NFAVertex v = *vi;
        ++vi;

        clear_vertex(v, h);
        if (!is_special(v, h)) {
            remove_vertex(v, h);
        }
    }

    assert(num_vertices(h) == N_SPECIALS);
    renumber_vertices(h); /* ensure that we reset our next allocated index */
    renumber_edges(h);

    // Recreate special stylised edges.
    add_edge(h.start, h.startDs, h);
    add_edge(h.startDs, h.startDs, h);
    add_edge(h.accept, h.acceptEod, h);
}

NFAVertex NGHolder::getSpecialVertex(u32 id) const {
    switch (id) {
    case NODE_START:         return start;
    case NODE_START_DOTSTAR: return startDs;
    case NODE_ACCEPT:        return accept;
    case NODE_ACCEPT_EOD:    return acceptEod;
    default:                 return null_vertex();
    }
}

}
