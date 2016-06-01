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
 * \brief Equivalence class graph reduction pass.
 */

#include "ng_equivalence.h"

#include "grey.h"
#include "ng_depth.h"
#include "ng_holder.h"
#include "ng_util.h"
#include "util/compile_context.h"
#include "util/graph_range.h"
#include "util/order_check.h"

#include <algorithm>
#include <set>
#include <stack>
#include <vector>

#include <boost/ptr_container/ptr_vector.hpp>

using namespace std;
using boost::ptr_vector;

namespace ue2 {

enum EquivalenceType {
    LEFT_EQUIVALENCE = 0,
    RIGHT_EQUIVALENCE,
    MAX_EQUIVALENCE
};

namespace {
class VertexInfo;

// custom comparison functor for unordered_set and flat_set
struct VertexInfoPtrCmp {
    // for flat_set
    bool operator()(const VertexInfo *a, const VertexInfo *b) const;
    // for unordered_set
    size_t operator()(const VertexInfo *a) const;
};

/** Precalculated (and maintained) information about a vertex. */
class VertexInfo {
public:
    VertexInfo(NFAVertex v_in, const NGHolder &g)
        : v(v_in), vert_index(g[v].index), cr(g[v].char_reach), edge_top(~0),
          equivalence_class(~0), vertex_flags(g[v].assert_flags) {}

    flat_set<VertexInfo *, VertexInfoPtrCmp> pred; //!< predecessors of this vertex
    flat_set<VertexInfo *, VertexInfoPtrCmp> succ; //!< successors of this vertex
    NFAVertex v;
    u32 vert_index;
    CharReach cr;
    CharReach pred_cr;
    CharReach succ_cr;
    unsigned edge_top;
    unsigned equivalence_class;
    unsigned vertex_flags;
};

}

typedef ue2::unordered_set<VertexInfo *, VertexInfoPtrCmp> VertexInfoSet;
typedef ue2::unordered_map<unsigned, VertexInfoSet> ClassMap;

// compare two vertex info pointers on their vertex index
bool VertexInfoPtrCmp::operator()(const VertexInfo *a,
                                  const VertexInfo *b) const {
    return a->vert_index < b->vert_index;
}
// provide a "hash" for vertex info pointer by returning its vertex index
size_t VertexInfoPtrCmp::operator()(const VertexInfo *a) const {
    return a->vert_index;
}

namespace {

// to avoid traversing infomap each time we need to check the class during
// partitioning, we will cache the information pertaining to a particular class
class ClassInfo {
public:
    struct ClassDepth {
        ClassDepth() {}
        ClassDepth(const NFAVertexDepth &d)
            : d1(d.fromStart), d2(d.fromStartDotStar) {}
        ClassDepth(const NFAVertexRevDepth &rd)
            : d1(rd.toAccept), d2(rd.toAcceptEod) {}
        DepthMinMax d1;
        DepthMinMax d2;
    };
    ClassInfo(const NGHolder &g, VertexInfo &vi, ClassDepth &d_in,
              EquivalenceType eq)
        : vertex_flags(vi.vertex_flags), edge_top(vi.edge_top), cr(vi.cr),
          depth(d_in) {

        // hackety-hack!
        node_type = g[vi.v].index;
        if (node_type > N_SPECIALS) {
            // we treat all regular vertices the same
            node_type = N_SPECIALS;
        }

        // get all the adjacent vertices' CharReach
        adjacent_cr = eq == LEFT_EQUIVALENCE ? vi.pred_cr : vi.succ_cr;

        if (eq == RIGHT_EQUIVALENCE) {
            rs = g[vi.v].reports;
        }
    }

    bool operator<(const ClassInfo &b) const;

private:
    flat_set<ReportID> rs; /* for right equiv only */
    unsigned vertex_flags;
    u32 edge_top;
    CharReach cr;
    CharReach adjacent_cr;
    unsigned node_type;
    ClassDepth depth;
};

// work queue class. this contraption has two goals:
// 1. uniqueness of elements
// 2. FILO operation
class WorkQueue {
public:
    explicit WorkQueue(unsigned c) {
        q.reserve(c);
    }
    // unique push
    void push(unsigned id) {
        if (ids.insert(id).second) {
            q.push_back(id);
        }
    }

    // pop
    unsigned pop() {
        unsigned id = q.back();
        ids.erase(id);
        q.pop_back();
        return id;
    }

    void append(WorkQueue &other) {
        for (const auto &e : other) {
            push(e);
        }
    }

    void clear() {
        ids.clear();
        q.clear();
    }

    bool empty() const {
        return ids.empty();
    }

    vector<unsigned>::const_iterator begin() const {
        return q.begin();
    }

    vector<unsigned>::const_iterator end() const {
       return q.end();
    }

    size_t capacity() const {
        return q.capacity();
    }
private:
    set<unsigned> ids; //!< stores id's, for uniqueness
    vector<unsigned> q; //!< vector of id's that we use as FILO.
};

}

bool ClassInfo::operator<(const ClassInfo &b) const {
    const ClassInfo &a = *this;

    ORDER_CHECK(node_type);
    ORDER_CHECK(depth.d1);
    ORDER_CHECK(depth.d2);
    ORDER_CHECK(cr);
    ORDER_CHECK(adjacent_cr);
    ORDER_CHECK(edge_top);
    ORDER_CHECK(vertex_flags);
    ORDER_CHECK(rs);
    return false;
}

static
bool outIsIrreducible(NFAVertex &v, const NGHolder &g) {
    unsigned nonSpecialVertices = 0;
    for (auto w : adjacent_vertices_range(v, g)) {
        if (!is_special(w, g) && w != v) {
            nonSpecialVertices++;
        }
    }
    return nonSpecialVertices == 1;
}

static
bool inIsIrreducible(NFAVertex &v, const NGHolder &g) {
    unsigned nonSpecialVertices = 0;
    for (auto u : inv_adjacent_vertices_range(v, g)) {
        if (!is_special(u, g) && u != v) {
            nonSpecialVertices++;
        }
    }
    return nonSpecialVertices == 1;
}

/** Cheaply check whether this graph can't be reduced at all, because it is
 * just a chain of vertices with no other edges. */
static
bool isIrreducible(const NGHolder &g) {
    for (auto v : vertices_range(g)) {
        // skip specials
        if (is_special(v, g)) {
            continue;
        }

        // we want meaningful in_degree to be 1. we also want to make sure we
        // don't count self-loop + 1 incoming edge as not irreducible
        if (in_degree(v, g) != 1 && !inIsIrreducible(v, g)) {
            return false;
        }
        // we want meaningful out_degree to be 1. we also want to make sure we
        // don't count self-loop + 1 outgoing edge as not irreducible
        if (out_degree(v, g) != 1 && !outIsIrreducible(v, g)) {
            return false;
        }
    }

    return true;
}

#ifndef NDEBUG
static
bool hasEdgeAsserts(NFAVertex v, const NGHolder &g) {
    for (const auto &e : in_edges_range(v, g)) {
        if (g[e].assert_flags != 0) {
            return true;
        }
    }
    for (const auto &e : out_edges_range(v, g)) {
        if (g[e].assert_flags != 0) {
            return true;
        }
    }
    return false;
}
#endif

// populate VertexInfo table
static
void getVertexInfos(const NGHolder &g, ptr_vector<VertexInfo> &infos) {
    vector<VertexInfo *> vertex_map; // indexed by vertex_index property
    vertex_map.resize(num_vertices(g));

    for (auto v : vertices_range(g)) {
        VertexInfo *vi = new VertexInfo(v, g);

        // insert our new shiny VertexInfo into the info map
        infos.push_back(vi);

        vertex_map[g[v].index] = vi;
    }

    // now, go through each vertex and populate its predecessor and successor lists
    for (VertexInfo &cur_vi : infos) {
        // find predecessors
        for (const auto &e : in_edges_range(cur_vi.v, g)) {
            NFAVertex u = source(e, g);
            VertexInfo *vmi = vertex_map[g[u].index];

            cur_vi.pred_cr |= vmi->cr;
            cur_vi.pred.insert(vmi);

            // also set up edge tops
            if (is_triggered(g) && u == g.start) {
                cur_vi.edge_top = g[e].top;
            }
        }

        // find successors
        for (auto w : adjacent_vertices_range(cur_vi.v, g)) {
            VertexInfo *vmi = vertex_map[g[w].index];
            cur_vi.succ_cr |= vmi->cr;
            cur_vi.succ.insert(vmi);
        }
        assert(!hasEdgeAsserts(cur_vi.v, g));
    }
}

// store equivalence class in VertexInfo for each vertex
static
void partitionGraph(ptr_vector<VertexInfo> &infos, ClassMap &classes,
                    WorkQueue &work_queue, const NGHolder &g,
                    EquivalenceType eq) {
    map<ClassInfo, unsigned> classinfomap;

    // get distances from start (or accept) for all vertices
    // only one of them is used at a time, never both
    vector<NFAVertexDepth> depths;
    vector<NFAVertexRevDepth> rdepths;

    if (eq == LEFT_EQUIVALENCE) {
        calcDepths(g, depths);
    } else {
        calcDepths(g, rdepths);
    }

    // partition the graph based on CharReach
    for (VertexInfo &vi : infos) {
        ClassInfo::ClassDepth depth;

        if (eq == LEFT_EQUIVALENCE) {
            depth = depths[vi.vert_index];
        } else {
            depth = rdepths[vi.vert_index];
        }
        ClassInfo ci(g, vi, depth, eq);

        auto ii = classinfomap.find(ci);
        if (ii == classinfomap.end()) {
            unsigned new_class = classinfomap.size();
            vi.equivalence_class = new_class;

            classinfomap[ci] = new_class;

            // insert this vertex into the class map
            VertexInfoSet &vertices = classes[new_class];
            vertices.insert(&vi);
        } else {
            unsigned eq_class = ii->second;
            vi.equivalence_class = eq_class;

            // insert this vertex into the class map
            VertexInfoSet &vertices = classes[eq_class];
            vertices.insert(&vi);

            // we now know that this particular class has more than one
            // vertex, so we add it to the work queue
            work_queue.push(eq_class);
        }
    }
    DEBUG_PRINTF("partitioned, %zu equivalence classes\n", classinfomap.size());
}

// generalized equivalence processing (left and right)
// basically, goes through every vertex in a class and checks if all successor or
// predecessor classes match in all vertices. if classes mismatch, a vertex is
// split into a separate class, along with all vertices having the same set of
// successor/predecessor classes. the opposite side (successors for left
// equivalence, predecessors for right equivalence) classes get revalidated in
// case of a split.
static
void equivalence(ClassMap &classmap, WorkQueue &work_queue,
                 EquivalenceType eq_type) {
    // now, go through the work queue until it's empty
    map<flat_set<unsigned>, VertexInfoSet> tentative_classmap;
    flat_set<unsigned> cur_classes;
    // local work queue, to store classes we want to revalidate in case of split
    WorkQueue reval_queue(work_queue.capacity());

    while (!work_queue.empty()) {

        // dequeue our class from the work queue
        unsigned cur_class = work_queue.pop();

        // get all vertices in current equivalence class
        VertexInfoSet &cur_class_vertices = classmap[cur_class];

        if (cur_class_vertices.size() < 2) {
            continue;
        }

        // clear data from previous iterations
        tentative_classmap.clear();

        DEBUG_PRINTF("doing equivalence pass for class %u, %zd vertices\n",
                     cur_class, cur_class_vertices.size());

        // go through vertices in this class
        for (VertexInfo *vi : cur_class_vertices) {
            cur_classes.clear();

            // get vertex lists for equivalence vertices and vertices for
            // revalidation in case of split
            const auto &eq_vertices =
                (eq_type == LEFT_EQUIVALENCE) ? vi->pred : vi->succ;
            const auto &reval_vertices =
                (eq_type == LEFT_EQUIVALENCE) ? vi->succ : vi->pred;

            // go through equivalence and note the classes
            for (const VertexInfo *tmp : eq_vertices) {
                cur_classes.insert(tmp->equivalence_class);
            }

            // note all the classes that need to be reevaluated
            for (const VertexInfo *tmp : reval_vertices) {
                reval_queue.push(tmp->equivalence_class);
            }

            VertexInfoSet &tentative_classes = tentative_classmap[cur_classes];
            tentative_classes.insert(vi);
        }

        // if we found more than one class, split and revalidate everything
        if (tentative_classmap.size() > 1) {
            auto tmi = tentative_classmap.begin();

            // start from the second class
            for (++tmi; tmi != tentative_classmap.end(); ++tmi) {
                unsigned new_class = classmap.size();
                const VertexInfoSet &vertices_to_split = tmi->second;
                VertexInfoSet &new_class_vertices = classmap[new_class];

                for (VertexInfo *vi : vertices_to_split) {
                    vi->equivalence_class = new_class;
                    cur_class_vertices.erase(vi);
                    new_class_vertices.insert(vi);
                }
                if (tmi->first.find(cur_class) != tmi->first.end()) {
                    reval_queue.push(new_class);
                }
            }
            work_queue.append(reval_queue);
        }
        reval_queue.clear();
    }
}

static
bool require_separate_eod_vertex(const VertexInfoSet &vert_infos,
                                 const NGHolder &g) {
    /* We require separate eod and normal accept vertices for a class if we have
     * both normal accepts and eod accepts AND the reports are different for eod
     * and non-eod reports. */

    flat_set<ReportID> non_eod;
    flat_set<ReportID> eod;

    for (const VertexInfo *vi : vert_infos) {
        NFAVertex v = vi->v;

        if (edge(v, g.accept, g).second) {
            insert(&non_eod, g[v].reports);
        }

        if (edge(v, g.acceptEod, g).second) {
            insert(&eod, g[v].reports);
        }
    }

    if (non_eod.empty() || eod.empty()) {
        return false;
    }

    return non_eod != eod;

}

static
void mergeClass(ptr_vector<VertexInfo> &infos, NGHolder &g, unsigned eq_class,
                VertexInfoSet &cur_class_vertices, set<NFAVertex> *toRemove) {
    DEBUG_PRINTF("Replacing %zd vertices from equivalence class %u with a "
                 "single vertex.\n", cur_class_vertices.size(), eq_class);

    // replace equivalence class with a single vertex:
    // 1. create new vertex with matching properties
    // 2. wire all predecessors to new vertex
    // 2a. update info for new vertex with new predecessors
    // 2b. update each predecessor's successor list
    // 3. wire all successors to new vertex
    // 3a. update info for new vertex with new successors
    // 3b. update each successor's predecessor list
    // 4. remove old vertex

    // any differences between vertex properties were resolved during
    // initial partitioning, so we assume that every vertex in equivalence
    // class has the same CharReach et al.
    // so, we find the first vertex in our class and get all its properties

    /* For left equivalence, if the members have different reporting behaviour
     * we sometimes require two vertices to be created (one connected to accept
     * and one to accepteod) */

    NFAVertex old_v = (*cur_class_vertices.begin())->v;
    NFAVertex new_v = clone_vertex(g, old_v); /* set up new vertex with same
                                               * props */
    g[new_v].reports.clear(); /* populated as we pull in succs */

    VertexInfo *new_vertex_info = new VertexInfo(new_v, g);
    // store this vertex in our global vertex list
    infos.push_back(new_vertex_info);

    NFAVertex new_v_eod = NGHolder::null_vertex();
    VertexInfo *new_vertex_info_eod = nullptr;

    if (require_separate_eod_vertex(cur_class_vertices, g)) {
        new_v_eod = clone_vertex(g, old_v);
        g[new_v_eod].reports.clear();
        new_vertex_info_eod = new VertexInfo(new_v_eod, g);
        infos.push_back(new_vertex_info_eod);
    }

    const unsigned edgetop = (*cur_class_vertices.begin())->edge_top;
    for (VertexInfo *old_vertex_info : cur_class_vertices) {
        assert(old_vertex_info->equivalence_class == eq_class);

        // mark this vertex for removal
        toRemove->insert(old_vertex_info->v);

        // for each predecessor, add edge to new vertex and update info
        for (VertexInfo *pred_info : old_vertex_info->pred) {
            // update info for new vertex
            new_vertex_info->pred.insert(pred_info);
            if (new_vertex_info_eod) {
                new_vertex_info_eod->pred.insert(pred_info);
            }

            // update info for predecessor
            pred_info->succ.erase(old_vertex_info);

            // if edge doesn't exist, create it
            NFAEdge e = add_edge_if_not_present(pred_info->v, new_v, g).first;

            // put edge top, if applicable
            if (edgetop != (unsigned) -1) {
                g[e].top = edgetop;
            }

            pred_info->succ.insert(new_vertex_info);

            if (new_v_eod) {
                NFAEdge ee = add_edge_if_not_present(pred_info->v, new_v_eod,
                                                     g).first;

                // put edge top, if applicable
                if (edgetop != (unsigned) -1) {
                    g[ee].top = edgetop;
                }

                pred_info->succ.insert(new_vertex_info_eod);
            }
        }

        // for each successor, add edge from new vertex and update info
        for (VertexInfo *succ_info : old_vertex_info->succ) {
            NFAVertex succ_v = succ_info->v;

            // update info for successor
            succ_info->pred.erase(old_vertex_info);

            if (new_v_eod && succ_v == g.acceptEod) {
                // update info for new vertex
                new_vertex_info_eod->succ.insert(succ_info);
                insert(&g[new_v_eod].reports,
                       g[old_vertex_info->v].reports);

                add_edge_if_not_present(new_v_eod, succ_v, g);
                succ_info->pred.insert(new_vertex_info_eod);
            } else {
                // update info for new vertex
                new_vertex_info->succ.insert(succ_info);

                // if edge doesn't exist, create it
                add_edge_if_not_present(new_v, succ_v, g);
                succ_info->pred.insert(new_vertex_info);

                if (is_any_accept(succ_v, g)) {
                    insert(&g[new_v].reports,
                           g[old_vertex_info->v].reports);
                }
            }
        }
    }

    // update classmap
    new_vertex_info->equivalence_class = eq_class;
    cur_class_vertices.insert(new_vertex_info);
}

// walk through vertices of an equivalence class and replace them with a single
// vertex (or, in rare cases for left equiv, a pair if we cannot satisfy the
// report behaviour with a single vertex).
static
bool mergeEquivalentClasses(ClassMap &classmap, ptr_vector<VertexInfo> &infos,
                            NGHolder &g) {
    bool merged = false;
    set<NFAVertex> toRemove;

    // go through all classes and merge classes with more than one vertex
    for (auto &cm : classmap) {
        // get all vertices in current equivalence class
        unsigned eq_class = cm.first;
        VertexInfoSet &cur_class_vertices = cm.second;

        // we don't care for single-vertex classes
        if (cur_class_vertices.size() > 1) {
            merged = true;
            mergeClass(infos, g, eq_class, cur_class_vertices, &toRemove);
        }
    }

    // remove all dead vertices
    DEBUG_PRINTF("removing %zd vertices.\n", toRemove.size());
    remove_vertices(toRemove, g);

    return merged;
}

bool reduceGraphEquivalences(NGHolder &g, const CompileContext &cc) {
    if (!cc.grey.equivalenceEnable) {
        DEBUG_PRINTF("equivalence processing disabled in grey box\n");
        return false;
    }
    g.renumberVertices();

    // Cheap check: if all the non-special vertices have in-degree one and
    // out-degree one, there's no redundancy in this here graph and we can
    // vamoose.
    if (isIrreducible(g)) {
        DEBUG_PRINTF("skipping equivalence processing, graph is irreducible\n");
        return false;
    }

    // take note if we have merged any vertices
    bool merge = false;

    for (int eqi = 0; eqi < MAX_EQUIVALENCE; ++eqi) {
        // map of all information pertaining a vertex
        ptr_vector<VertexInfo> infos;
        ClassMap classes;

        // create a list of equivalence classes to check
        WorkQueue work_queue(num_vertices(g));
        EquivalenceType eq_type = (EquivalenceType) eqi;

        // resize the vector, make room for twice the vertices we have
        infos.reserve(num_vertices(g) * 2);

        // get information on every vertex in the graph
        // new vertices are allocated here, and stored in infos
        getVertexInfos(g, infos);

        // partition the graph
        partitionGraph(infos, classes, work_queue, g, eq_type);

        // do equivalence processing
        equivalence(classes, work_queue, eq_type);

        // replace equivalent classes with single vertices
        // new vertices are (possibly) allocated here, and stored in infos
        merge |= mergeEquivalentClasses(classes, infos, g);
    }

    return merge;
}

} // namespace ue2
