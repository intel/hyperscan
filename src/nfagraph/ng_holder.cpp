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

#include "ng_holder.h"

#include "ng_util.h"
#include "ue2common.h"

using namespace std;

namespace ue2 {

// internal use only
static NFAVertex addSpecialVertex(NFAGraph &g, SpecialNodes id) {
    NFAVertex v = add_vertex(g);
    g[v].index = id;
    return v;
}

NGHolder::NGHolder(void)
 : g(),
   // add initial special nodes
   start(addSpecialVertex(g, NODE_START)),
   startDs(addSpecialVertex(g, NODE_START_DOTSTAR)),
   accept(addSpecialVertex(g, NODE_ACCEPT)),
   acceptEod(addSpecialVertex(g, NODE_ACCEPT_EOD)),
   // misc data
   numVertices(N_SPECIALS),
   numEdges(0),
   isValidNumEdges(true),
   isValidNumVertices(true) {

    // wire up some fake edges for the stylized bits of the NFA
    add_edge(start, startDs, *this);
    add_edge(startDs, startDs, *this);
    add_edge(accept, acceptEod, *this);

    g[start].char_reach.setall();
    g[startDs].char_reach.setall();
}

NGHolder::NGHolder(nfa_kind k)
 : kind (k), g(),
   // add initial special nodes
   start(addSpecialVertex(g, NODE_START)),
   startDs(addSpecialVertex(g, NODE_START_DOTSTAR)),
   accept(addSpecialVertex(g, NODE_ACCEPT)),
   acceptEod(addSpecialVertex(g, NODE_ACCEPT_EOD)),
   // misc data
   numVertices(N_SPECIALS),
   numEdges(0),
   isValidNumEdges(true),
   isValidNumVertices(true) {

    // wire up some fake edges for the stylized bits of the NFA
    add_edge(start, startDs, *this);
    add_edge(startDs, startDs, *this);
    add_edge(accept, acceptEod, *this);

    g[start].char_reach.setall();
    g[startDs].char_reach.setall();
}

NGHolder::~NGHolder(void) {
    DEBUG_PRINTF("destroying holder @ %p\n", this);
}

size_t num_edges(NGHolder &h) {
    if (!h.isValidNumEdges) {
        h.numEdges = num_edges(h.g);
        h.isValidNumEdges = true;
    }
    return h.numEdges;
}

size_t num_edges(const NGHolder &h) {
    if (!h.isValidNumEdges) {
        return num_edges(h.g);
    }
    return h.numEdges;
}

size_t num_vertices(NGHolder &h) {
    if (!h.isValidNumVertices) {
        h.numVertices = num_vertices(h.g);
        h.isValidNumVertices = true;
    }
    return h.numVertices;
}

size_t num_vertices(const NGHolder &h) {
    if (!h.isValidNumVertices) {
        return num_vertices(h.g);
    }
    return h.numVertices;
}

void remove_edge(const NFAEdge &e, NGHolder &h) {
    remove_edge(e, h.g);
    assert(!h.isValidNumEdges || h.numEdges > 0);
    h.numEdges--;
}

void remove_edge(NFAVertex u, NFAVertex v, NGHolder &h) {
    remove_edge(u, v, h.g);
    assert(!h.isValidNumEdges || h.numEdges > 0);
    h.numEdges--;
}

void remove_vertex(NFAVertex v, NGHolder &h) {
    remove_vertex(v, h.g);
    assert(!h.isValidNumVertices || h.numVertices > 0);
    h.numVertices--;
}

void clear_vertex(NFAVertex v, NGHolder &h) {
    h.isValidNumEdges = false;
    clear_vertex_faster(v, h.g);
}

void clear_in_edges(NFAVertex v, NGHolder &h) {
    h.isValidNumEdges = false;
    clear_in_edges(v, h.g);
}

void clear_out_edges(NFAVertex v, NGHolder &h) {
    h.isValidNumEdges = false;
    clear_out_edges(v, h.g);
}

void clear_graph(NGHolder &h) {
    NFAGraph::vertex_iterator vi, ve;
    for (tie(vi, ve) = vertices(h); vi != ve;) {
        NFAVertex v = *vi;
        ++vi;

        clear_vertex(v, h);
        if (!is_special(v, h)) {
            remove_vertex(v, h);
        }
    }

    assert(num_vertices(h) == N_SPECIALS);

    // Recreate special stylised edges.
    add_edge(h.start, h.startDs, h);
    add_edge(h.startDs, h.startDs, h);
    add_edge(h.accept, h.acceptEod, h);
}

std::pair<NFAEdge, bool> add_edge(NFAVertex u, NFAVertex v, NGHolder &h) {
    assert(edge(u, v, h.g).second == false);
    pair<NFAEdge, bool> e = add_edge(u, v, h.g);
    h.g[e.first].index = h.numEdges++;
    assert(!h.isValidNumEdges || h.numEdges > 0); // no wrapping
    h.g[e.first].top = 0;
    return e;
}

std::pair<NFAEdge, bool> add_edge(NFAVertex u, NFAVertex v,
                                  const NFAGraph::edge_property_type &ep,
                                  NGHolder &h) {
    assert(edge(u, v, h.g).second == false);
    pair<NFAEdge, bool> e = add_edge(u, v, ep, h.g);
    h.g[e.first].index = h.numEdges++;
    assert(!h.isValidNumEdges || h.numEdges > 0); // no wrapping
    return e;
}

NFAVertex add_vertex(NGHolder &h) {
    NFAVertex v = add_vertex(h.g);
    h[v].index = h.numVertices++;
    assert(h.numVertices > 0); // no wrapping
    return v;
}

NFAVertex add_vertex(const NFAGraph::vertex_property_type &vp, NGHolder &h) {
    NFAVertex v = add_vertex(h);
    u32 i = h.g[v].index; /* preserve index */
    h.g[v] = vp;
    h.g[v].index = i;
    return v;
}

void NGHolder::renumberEdges() {
    numEdges = renumberGraphEdges(g);
    isValidNumEdges = true;
}

void NGHolder::renumberVertices() {
    numVertices = renumberGraphVertices(g);
    isValidNumVertices = true;
}

NFAVertex NGHolder::getSpecialVertex(u32 id) const {
    switch (id) {
        case NODE_START:         return start;
        case NODE_START_DOTSTAR: return startDs;
        case NODE_ACCEPT:        return accept;
        case NODE_ACCEPT_EOD:    return acceptEod;
        default:                 return nullptr;
    }
}

}
