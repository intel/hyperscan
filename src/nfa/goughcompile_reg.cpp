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

#include "goughcompile.h"
#include "goughcompile_dump.h"
#include "goughcompile_internal.h"
#include "gough_internal.h"
#include "grey.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph.h"
#include "util/graph_range.h"
#include "util/order_check.h"

#include "ue2common.h"

#include <algorithm>
#include <boost/graph/depth_first_search.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_values;

namespace ue2 {

template<typename VarP, typename VarQ>
void push_back_all_raw(vector<VarP> *out, const vector<VarQ> &in) {
    for (const auto &var : in) {
        out->push_back(var.get());
    }
}

static
void all_vars(const GoughGraph &g, vector<GoughSSAVar *> *out) {
    for (auto v : vertices_range(g)) {
        push_back_all_raw(out, g[v].vars);
    }
    for (const auto &e : edges_range(g)) {
        push_back_all_raw(out, g[e].vars);
    }
}

namespace {
struct GoughGraphAux {
    map<const GoughSSAVar *, GoughVertex> containing_v;
    map<const GoughSSAVar *, GoughEdge> containing_e;
    map<const GoughSSAVar *, set<GoughVertex> > reporters;
};
}

static never_inline
void fill_aux(const GoughGraph &g, GoughGraphAux *aux) {
    for (auto v : vertices_range(g)) {
        for (const auto &var : g[v].vars) {
            aux->containing_v[var.get()] = v;
            DEBUG_PRINTF("%u is on vertex %u\n", var->slot, g[v].state_id);
        }

        for (GoughSSAVar *var : g[v].reports | map_values) {
            aux->reporters[var].insert(v);
        }

        for (GoughSSAVar *var : g[v].reports_eod | map_values) {
            aux->reporters[var].insert(v);
        }
    }
    for (const auto &e : edges_range(g)) {
        for (const auto &var : g[e].vars) {
            aux->containing_e[var.get()] = e;
            DEBUG_PRINTF("%u is on edge %u->%u\n", var->slot,
                         g[source(e, g)].state_id, g[target(e, g)].state_id);
        }
    }
}

static
bool is_block_local(const GoughGraph &cfg, GoughSSAVar *var,
                    const GoughGraphAux &aux) {
    /* if var used as a report, it cannot be considered block local */
    if (contains(aux.reporters, var)) {
        return false;
    }

    /* (useful) vertex/join vars never local - they are terminal in blocks
     * and so should be read by another block. */
    if (!contains(aux.containing_e, var)) {
        return false;
    }

    /* for other cases, require that all uses of var are later in the same edge
     * or on the target AND if on target it is sole on flow coming from the
     * edge in question. */
    const GoughEdge &e = aux.containing_e.at(var);
    GoughVertex t = target(e, cfg);

    size_t seen_outputs = 0;
    const flat_set<GoughSSAVarWithInputs *> &out = var->get_outputs();
    bool seen_var = false;
    for (const auto &e_var : cfg[e].vars) {
        if (seen_var) {
            GoughSSAVarWithInputs *w
                = dynamic_cast<GoughSSAVarWithInputs *>(e_var.get());
            if (contains(out, w)) {
                seen_outputs++;
            }
        } else {
            seen_var = var == e_var.get();
        }
    }
    assert(seen_var);

    for (const auto &t_var : cfg[t].vars) {
        if (contains(out, t_var.get())) {
            seen_outputs++;
            const flat_set<GoughEdge> &flow = t_var->get_edges_for_input(var);
            if (flow.size() != 1 || *flow.begin() != e) {
                /* this var is used by the target join var BUT on a different
                 * flow, so this is not a block local variable */
                return false;
            }
        }
    }

    assert(seen_outputs <= out.size());
    return seen_outputs == out.size();
}

static
void handle_pending_edge(const GoughGraph &g, const GoughEdge &e,
                         GoughSSAVar *start, set<GoughVertex> &pending_vertex,
                         set<const GoughSSAVar *> &rv) {
    const vector<shared_ptr<GoughSSAVar> > &vars = g[e].vars;
    bool marking = !start;
    DEBUG_PRINTF("  ---checking edge %u->%u %s %zu\n", g[source(e, g)].state_id,
                 g[target(e, g)].state_id, marking ? "full" : "partial",
                 vars.size());
    for (auto it = vars.rbegin(); it != vars.rend(); ++it) {
        GoughSSAVar *var = it->get();
        if (contains(rv, var)) {
            DEBUG_PRINTF("somebody has already processed this vertex [%u]\n",
                         var->slot);
            return;
        }
        if (var == start) {
            assert(!marking);
            marking = true;
            continue;
        }
        if (marking) {
            rv.insert(var);
        }
    }
    assert(marking);
    GoughVertex s = source(e, g);
    for (const auto &var : g[s].vars) {
        DEBUG_PRINTF("interferes %u\n", var->slot);
        rv.insert(var.get());
    }
    pending_vertex.insert(s);
}

static
void handle_pending_vars(GoughSSAVar *def, const GoughGraph &g,
                         const GoughGraphAux &aux,
                         const flat_set<GoughSSAVarWithInputs *> &pending_var,
                         set<GoughVertex> &pending_vertex,
                         set<const GoughSSAVar *> &rv) {
    for (GoughSSAVarWithInputs *var : pending_var) {
        if (contains(aux.containing_v, var)) {
            /* def is used by join vertex, value only needs to be live on some
             * incoming edges */
            GoughSSAVarJoin *vj = (GoughSSAVarJoin *)var;
            const flat_set<GoughEdge> &live_edges
                = vj->get_edges_for_input(def);
            for (const auto &e : live_edges) {
                handle_pending_edge(g, e, nullptr, pending_vertex, rv);
            }
            continue;
        }
        const GoughEdge &e = aux.containing_e.at(var);
        handle_pending_edge(g, e, var, pending_vertex, rv);
    }
}

static
void handle_pending_vertex(GoughVertex def_v, const GoughGraph &g,
                           GoughVertex current,
                           set<GoughVertex> &pending_vertex,
                           set<const GoughSSAVar *> &rv) {
    DEBUG_PRINTF("---checking vertex %u\n", g[current].state_id);
    if (def_v == current) {
        DEBUG_PRINTF("contains target vertex\n");
        return; /* we have reached def */
    }
    for (const auto &e : in_edges_range(current, g)) {
        handle_pending_edge(g, e, nullptr, pending_vertex, rv);
    }
}

static
void handle_pending_vertices(GoughSSAVar *def, const GoughGraph &g,
                             const GoughGraphAux &aux,
                             set<GoughVertex> &pending_vertex,
                             set<const GoughSSAVar *> &rv) {
    if (pending_vertex.empty()) {
        return;
    }

    GoughVertex def_v = GoughGraph::null_vertex();
    if (contains(aux.containing_v, def)) {
        def_v = aux.containing_v.at(def);
    }
    unordered_set<GoughVertex> done;
    while (!pending_vertex.empty()) {
        GoughVertex current = *pending_vertex.begin();
        pending_vertex.erase(current);
        if (contains(done, current)) {
            continue;
        }
        done.insert(current);
        handle_pending_vertex(def_v, g, current, pending_vertex, rv);
    }
}

/* returns set of labels that the given def is live at */
static never_inline
set<const GoughSSAVar *> live_during(GoughSSAVar *def, const GoughGraph &g,
                                     const GoughGraphAux &aux) {
    DEBUG_PRINTF("checking who is defined during %u lifetime\n", def->slot);
    set<GoughVertex> pending_vertex;

    set<const GoughSSAVar *> rv;
    rv.insert(def);

    if (contains(aux.reporters, def)) {
        DEBUG_PRINTF("--> gets reported\n");
        const set<GoughVertex> &reporters = aux.reporters.at(def);
        for (auto v : reporters) {
            pending_vertex.insert(v);
            for (const auto &var : g[v].vars) {
                DEBUG_PRINTF("interferes %u\n", var->slot);
                rv.insert(var.get());
            }
        }
    }

    handle_pending_vars(def, g, aux, def->get_outputs(), pending_vertex, rv);
    handle_pending_vertices(def, g, aux, pending_vertex, rv);

    rv.erase(def);
    return rv;
}

template<typename VarP>
void set_initial_slots(const vector<VarP> &vars, u32 *next_slot) {
    for (auto &var : vars) {
        assert(var->slot == INVALID_SLOT);
        var->slot = (*next_slot)++;
    }
}

/* crude, deterministic assignment of symbolic register slots.
 * returns number of slots given out
 */
static
u32 initial_slots(const GoughGraph &g) {
    u32 next_slot = 0;
    for (auto v : vertices_range(g)) {
        set_initial_slots(g[v].vars, &next_slot);
    }
    for (const auto &e : edges_range(g)) {
        set_initial_slots(g[e].vars, &next_slot);
    }

    return next_slot;
}

#define NO_COLOUR (~0U)

static
u32 available_colour(const flat_set<u32> &bad_colours) {
    u32 rv = 0;
    for (const u32 &colour : bad_colours) {
        if (colour != rv) {
            assert(colour > rv);
            break;
        }
        rv = colour + 1;
    }

    assert(rv != NO_COLOUR);
    return rv;
}

static
void poison_colours(const set<const GoughSSAVar *> &live, u32 c,
                    const vector<u32> &colour_map,
                    vector<flat_set<u32> > *bad_colour) {
    for (const GoughSSAVar *var : live) {
        u32 var_index = var->slot;
        if (colour_map[var_index] != NO_COLOUR) {
            assert(c != colour_map[var_index]);
        } else {
            (*bad_colour)[var_index].insert(c);
        }
    }
}

static
void find_bad_due_to_live(const set<const GoughSSAVar *> &live,
                          const vector<u32> &colour_map, flat_set<u32> *out) {
    for (const GoughSSAVar *var : live) {
        u32 var_index = var->slot;
        if (colour_map[var_index] != NO_COLOUR) {
            out->insert(colour_map[var_index]);
        }
    }
}

static
void sequential_vertex_colouring(const GoughGraph &g, const GoughGraphAux &aux,
                                 const vector<GoughSSAVar *> &order,
                                 vector<u32> &colour_map) {
    assert(order.size() < NO_COLOUR);
    colour_map.clear();
    colour_map.resize(order.size(), NO_COLOUR);
    vector<u32> temp(order.size(), ~0U);
    vector<flat_set<u32> > bad_colour(order.size());

    for (GoughSSAVar *var : order) {
        u32 var_index = var->slot;
        if (is_block_local(g, var, aux)) {
            DEBUG_PRINTF("%u is block local\n", var_index);
            /* ignore variable whose lifetime is limited to their local block
             * there is no need to assign stream state to these variables */
            continue;
        }
        assert(colour_map[var_index] == NO_COLOUR);
        set<const GoughSSAVar *> live = live_during(var, g, aux);
        flat_set<u32> &local_bad = bad_colour[var_index];
        find_bad_due_to_live(live, colour_map, &local_bad);
        DEBUG_PRINTF("colouring %u\n", var_index);
        u32 c = available_colour(local_bad);
        colour_map[var_index] = c;
        assert(!contains(bad_colour[var_index], c));
        poison_colours(live, c, colour_map, &bad_colour);

        flat_set<u32> temp_set;
        local_bad.swap(temp_set);
        DEBUG_PRINTF("    %u coloured %u\n", var_index, c);
    }
}

template<typename VarP>
void add_to_dom_ordering(const vector<VarP> &vars,
                         vector<GoughSSAVar *> *out) {
    for (const auto &var : vars) {
        out->push_back(var.get());
    }
}

namespace {
class FinishVisitor : public boost::default_dfs_visitor {
public:
    explicit FinishVisitor(vector<GoughVertex> *o) : out(o) {}
    void finish_vertex(const GoughVertex v, const GoughGraph &) {
        out->push_back(v);
    }
    vector<GoughVertex> *out;
};
}

static
void find_dom_ordering(const GoughGraph &cfg, vector<GoughSSAVar *> *out) {
    vector<GoughVertex> g_order;

    /* due to construction quirks, default vertex order provides entry points */
    depth_first_search(cfg, visitor(FinishVisitor(&g_order))
                        .root_vertex(cfg[boost::graph_bundle].initial_vertex));

    for (auto it = g_order.rbegin(); it != g_order.rend(); ++it) {
        add_to_dom_ordering(cfg[*it].vars, out);
        for (const auto &e : out_edges_range(*it, cfg)) {
            add_to_dom_ordering(cfg[e].vars, out);
        }
    }
}

static
void create_slot_mapping(const GoughGraph &cfg, UNUSED u32 old_slot_count,
                         vector<u32> *old_new) {
    /* Interference graphs from SSA form are chordal -> optimally colourable in
     * poly time.
     *
     * Chordal graphs can be coloured by walking in perfect elimination order.
     * If the SSA CFG is iterated over in a way that respects dominance
     * relationship, the interference graph will be iterated in a perfect
     * elimination order.
     *
     * We can avoid creating the full interference graph and use liveness
     * information as we iterate over the definitions to perform the colouring.
     *
     * See S Hack various 2006-
     */
    vector<GoughSSAVar *> dom_order;

    GoughGraphAux aux;
    fill_aux(cfg, &aux);

    find_dom_ordering(cfg, &dom_order);
    assert(dom_order.size() == old_slot_count);
    sequential_vertex_colouring(cfg, aux, dom_order, *old_new);
}

static
void update_local_slots(GoughGraph &g, set<GoughSSAVar *> &locals,
                        u32 local_base) {
    DEBUG_PRINTF("%zu local variables\n", locals.size());
    /* local variables only occur on edges (joins are never local) */

    u32 allocated_count = 0;
    for (const auto &e : edges_range(g)) {
        u32 next_slot = local_base;
        for (auto &var : g[e].vars) {
            if (contains(locals, var.get())) {
                DEBUG_PRINTF("updating slot %u using local %u\n", var->slot,
                             next_slot);
                var->slot = next_slot++;
                allocated_count++;
            }
        }
    }

    assert(allocated_count == locals.size());
}

static never_inline
u32 update_slots(GoughGraph &g, const vector<u32> &old_new,
                 UNUSED u32 old_slot_count) {
    vector<GoughSSAVar *> vars;
    set<GoughSSAVar *> locals;
    all_vars(g, &vars);
    u32 slot_count = 0;
    for (GoughSSAVar *v : vars) {
        assert(v->slot < old_new.size());
        DEBUG_PRINTF("updating slot %u to %u\n", v->slot, old_new[v->slot]);
        if (old_new[v->slot] != NO_COLOUR) { /* not local, assign final slot */
            v->slot = old_new[v->slot];
            ENSURE_AT_LEAST(&slot_count, v->slot + 1);
        } else {
            locals.insert(v);
        }
    }
    assert(slot_count <= old_slot_count);
    DEBUG_PRINTF("reduce stream slots from %u to %u\n", old_slot_count,
                 slot_count);
    update_local_slots(g, locals, slot_count);

    return slot_count;
}

u32 assign_slots(GoughGraph &cfg, const Grey &grey) {
    u32 slot_count = initial_slots(cfg);

    if (!grey.goughRegisterAllocate) {
        return slot_count;
    }
    dump(cfg, "slots_pre", grey);

    vector<u32> old_new;
    create_slot_mapping(cfg, slot_count, &old_new);
    slot_count = update_slots(cfg, old_new, slot_count);

    return slot_count;
}

} // namespace ue2
