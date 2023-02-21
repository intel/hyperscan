/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "accel.h"
#include "goughcompile_dump.h"
#include "goughcompile_internal.h"
#include "gough_internal.h"
#include "grey.h"
#include "mcclellancompile.h"
#include "nfa_internal.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/flat_containers.h"
#include "util/graph_range.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/verify_types.h"

#include "ue2common.h"

#include <algorithm>
#include <boost/dynamic_bitset.hpp>
#include <boost/range/adaptor/map.hpp>

using namespace std;
using boost::adaptors::map_keys;
using boost::adaptors::map_values;
using boost::vertex_index;

namespace ue2 {

void raw_som_dfa::stripExtraEodReports(void) {
    /* if a state generates a given report as a normal accept - then it does
     * not also need to generate an eod report for it */
    for (vector<dstate_som>::iterator it = state_som.begin();
         it != state_som.end(); ++it) {
        for (const som_report &sr : it->reports) {
            it->reports_eod.erase(sr);
        }
        dstate &norm = states[it - state_som.begin()];
        norm.reports_eod.clear();
        for (const som_report &sr : it->reports_eod) {
            norm.reports_eod.insert(sr.report);
        }
    }
}

namespace {

class gough_build_strat : public mcclellan_build_strat {
public:
    gough_build_strat(
        raw_som_dfa &r, const GoughGraph &g, const ReportManager &rm_in,
        const map<dstate_id_t, gough_accel_state_info> &accel_info)
        : mcclellan_build_strat(r, rm_in, false), rdfa(r), gg(g),
          accel_gough_info(accel_info) {}
    unique_ptr<raw_report_info> gatherReports(vector<u32> &reports /* out */,
                            vector<u32> &reports_eod /* out */,
                            u8 *isSingleReport /* out */,
                            ReportID *arbReport  /* out */) const override;
    AccelScheme find_escape_strings(dstate_id_t this_idx) const override;
    size_t accelSize(void) const override { return sizeof(gough_accel); }
    void buildAccel(dstate_id_t this_idx, const AccelScheme &info,
                    void *accel_out) override;
    u32 max_allowed_offset_accel() const override { return 0; }
    DfaType getType() const override { return Gough; }

    raw_som_dfa &rdfa;
    const GoughGraph &gg;
    map<dstate_id_t, gough_accel_state_info> accel_gough_info;
    map<gough_accel *, dstate_id_t> built_accel;
};

}

GoughSSAVar::~GoughSSAVar() {
}

void GoughSSAVar::clear_outputs() {
    for (GoughSSAVarWithInputs *var : outputs) {
        var->remove_input_raw(this);
    }
    outputs.clear();
}

void GoughSSAVarWithInputs::clear_all() {
    clear_inputs();
    clear_outputs();
}

void GoughSSAVarMin::clear_inputs() {
    for (GoughSSAVar *var : inputs) {
        assert(contains(var->outputs, this));
        var->outputs.erase(this);
    }
    inputs.clear();
}

void GoughSSAVarMin::replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) {
    assert(contains(inputs, old_v));
    inputs.erase(old_v);
    old_v->outputs.erase(this);
    inputs.insert(new_v);
    new_v->outputs.insert(this);
}

static
void translateRawReports(UNUSED GoughGraph &cfg, UNUSED const raw_som_dfa &raw,
                         const flat_map<u32, GoughSSAVarJoin *> &joins_at_s,
                         UNUSED GoughVertex s,
                         const set<som_report> &reports_in,
                         vector<pair<ReportID, GoughSSAVar *> > *reports_out) {
    for (const som_report &sr : reports_in) {
        DEBUG_PRINTF("state %u: report %u slot %d\n", cfg[s].state_id,
                     sr.report, sr.slot);
        GoughSSAVar *var = nullptr;
        if (sr.slot == CREATE_NEW_SOM) {
            assert(!generates_callbacks(raw.kind));
        } else {
            var = joins_at_s.at(sr.slot);
        }
        reports_out->push_back(make_pair(sr.report, var));
    }
}

static
void makeCFG_reports(GoughGraph &cfg, const raw_som_dfa &raw,
                     const vector<flat_map<u32, GoughSSAVarJoin *> > &joins,
                     const vector<GoughVertex> &vertices) {
     for (u32 i = 1; i < raw.states.size(); ++i) {
         GoughVertex s = vertices[i];
         const flat_map<u32, GoughSSAVarJoin *> &joins_at_s
             = joins[get(vertex_index, cfg, s)];
         translateRawReports(cfg, raw, joins_at_s, s,
                             raw.state_som[i].reports, &cfg[s].reports);
         translateRawReports(cfg, raw, joins_at_s, s,
                             raw.state_som[i].reports_eod, &cfg[s].reports_eod);
     }
}

static never_inline
void makeCFG_top_edge(GoughGraph &cfg, const vector<GoughVertex> &vertices,
                      const vector<flat_map<u32, GoughSSAVarJoin *> > &joins,
                      u32 trigger_slot, const som_tran_info &src_slots,
                      const som_tran_info &dest_slot_pred,
                      dstate_id_t i, dstate_id_t n, const GoughEdge &e) {
    GoughVertex s = vertices[i];
    GoughVertex t = vertices[n];
    const flat_map<u32, GoughSSAVarJoin *> &joins_at_s
        = joins[get(vertex_index, cfg, s)];
    const flat_map<u32, GoughSSAVarJoin *> &joins_at_t
        = joins[get(vertex_index, cfg, t)];

    DEBUG_PRINTF("top for %u -> %u\n", i, n);

    for (som_tran_info::const_iterator it = dest_slot_pred.begin();
         it != dest_slot_pred.end(); ++it) {
        /* for ordering, need to ensure that new values feeding directly
         * into mins come first */
        u32 slot_id = it->first;

        shared_ptr<GoughSSAVarNew> vnew;
        if (slot_id == trigger_slot) {
            vnew = make_shared<GoughSSAVarNew>(0U);
            cfg[e].vars.push_back(vnew);
        } else {
            assert(contains(src_slots, slot_id));
        }

        GoughSSAVar *final_var;
        if (vnew && !contains(src_slots, slot_id)) {
            final_var = vnew.get();
            DEBUG_PRINTF("bypassing min on join %u\n", slot_id);
        } else if (!vnew) {
            final_var = joins_at_s.at(slot_id);
            DEBUG_PRINTF("bypassing min on join %u\n", slot_id);
        } else {
            assert(vnew);
            assert(contains(src_slots, slot_id));

            shared_ptr<GoughSSAVarMin> vmin = make_shared<GoughSSAVarMin>();
            if (!vmin) {
                assert(0);
                throw std::bad_alloc();
            }
            cfg[e].vars.push_back(vmin);
            final_var = vmin.get();

            DEBUG_PRINTF("slot %u gets a new value\n", slot_id);
            vmin->add_input(vnew.get());

            DEBUG_PRINTF("slot %u is constant\n", slot_id);
            vmin->add_input(joins_at_s.at(slot_id));
        }

        /* wire to destination target */
        GoughSSAVarJoin *vk = joins_at_t.at(slot_id);
        vk->add_input(final_var, e);
    }
}

static never_inline
void makeCFG_edge(GoughGraph &cfg, const map<u32, u32> &som_creators,
                  const vector<GoughVertex> &vertices,
                  const vector<flat_map<u32, GoughSSAVarJoin *> > &joins,
                  const som_tran_info &src_slots,
                  const som_tran_info &dest_slot_pred, dstate_id_t i,
                  dstate_id_t n, const GoughEdge &e) {
    GoughVertex s = vertices[i];
    GoughVertex t = vertices[n];
    const flat_map<u32, GoughSSAVarJoin *> &joins_at_s
        = joins[get(vertex_index, cfg, s)];
    const flat_map<u32, GoughSSAVarJoin *> &joins_at_t
        = joins[get(vertex_index, cfg, t)];

    map<u32, shared_ptr<GoughSSAVarNew> > vnew_by_adj;
    for (som_tran_info::const_iterator it = dest_slot_pred.begin();
         it != dest_slot_pred.end(); ++it) {
        /* for ordering, need to ensure that new values feeding directly
         * into mins come first */
        u32 slot_id = it->first;

        if (contains(som_creators, slot_id) && !som_creators.at(slot_id)) {
            continue;
        }

        shared_ptr<GoughSSAVarNew> vnew;
        const vector<u32> &inputs = it->second;
        u32 useful_input_count = 0;
        u32 first_useful_input = ~0U;

        for (const u32 &input_slot : inputs) {
            if (!contains(src_slots, input_slot)) {
                continue;
            }
            DEBUG_PRINTF("%u is useful\n", input_slot);

            if (!vnew || !contains(som_creators, input_slot)) {
                useful_input_count++;
                if (useful_input_count == 1) {
                    first_useful_input = input_slot;
                }
            }

            if (contains(som_creators, input_slot)) {
                u32 adjust = som_creators.at(input_slot);

                if (vnew && vnew->adjust >= adjust) {
                    DEBUG_PRINTF("skipping %u as domininated by adj%u\n",
                                 adjust, vnew->adjust);
                    continue; /* deeper starts can be seen to statically
                                 dominate */
                }

                if (contains(vnew_by_adj, adjust)) {
                    vnew = vnew_by_adj[adjust];
                } else {
                    vnew = make_shared<GoughSSAVarNew>(adjust);
                    cfg[e].vars.push_back(vnew);
                    vnew_by_adj[adjust] = vnew;
                }
                assert(vnew);
            }
        }

        /* If we have a new start of match (with no offset or 1 byte offset) and
         * other variables coming in, the new will always be dominated by the
         * existing variables (as they must be at least one byte into the match)
         * -- and so can be dropped. */
        if (vnew && vnew->adjust < 2 && useful_input_count > 1) {
            useful_input_count--;
            vnew.reset();

            /* need to reestablish the first useful input */
            for (const u32 &input_slot : inputs) {
                if (!contains(src_slots, input_slot)) {
                    continue;
                }
                if (!contains(som_creators, input_slot)) {
                    first_useful_input = input_slot;
                }
            }

        }

        GoughSSAVar *final_var;
        if (useful_input_count == 1) {
            if (vnew) {
                final_var = vnew.get();
            } else {
                assert(first_useful_input != ~0U);
                final_var = joins_at_s.at(first_useful_input);
            }
            DEBUG_PRINTF("bypassing min on join %u\n", slot_id);
        } else {
            shared_ptr<GoughSSAVarMin> vmin = make_shared<GoughSSAVarMin>();
            if (!vmin) {
                assert(0);
                throw std::bad_alloc();
            }
            cfg[e].vars.push_back(vmin);
            final_var = vmin.get();

            if (vnew) {
                vmin->add_input(vnew.get());
            }

            /* wire the normal inputs to the min */
            for (const u32 &input_slot : inputs) {
                if (!contains(src_slots, input_slot)) {
                    continue;
                }
                if (!contains(som_creators, input_slot)) {
                    vmin->add_input(joins_at_s.at(input_slot));
                }
            }
            assert(vmin->get_inputs().size() > 1);
            DEBUG_PRINTF("wire min to join %u\n", slot_id);
        }

        GoughSSAVarJoin *vk = joins_at_t.at(slot_id);
        assert(final_var);
        vk->add_input(final_var, e);
    }
}

static never_inline
unique_ptr<GoughGraph> makeCFG(const raw_som_dfa &raw) {
    vector<GoughVertex> vertices;
    vertices.reserve(raw.states.size());
    unique_ptr<GoughGraph> cfg = ue2::make_unique<GoughGraph>();
    u32 min_state = !is_triggered(raw.kind);

    if (min_state) {
        vertices.push_back(GoughGraph::null_vertex()); /* skip dead state */
    }

    vector<flat_map<u32, GoughSSAVarJoin *> > joins(raw.states.size());
    for (u32 i = min_state; i < raw.states.size(); ++i) {
        GoughVertex v = add_vertex(GoughVertexProps(i), *cfg);
        vertices.push_back(v);

        /* create JOIN variables */
        for (som_tran_info::const_iterator it = raw.state_som[i].preds.begin();
             it != raw.state_som[i].preds.end(); ++it) {
            u32 slot_id = it->first;
            if (!contains(raw.new_som_nfa_states, slot_id)
                || raw.new_som_nfa_states.at(slot_id)) {
                (*cfg)[v].vars.push_back(make_shared<GoughSSAVarJoin>());
                joins[get(vertex_index, *cfg, v)][slot_id]
                    = (*cfg)[v].vars.back().get();
                DEBUG_PRINTF("dfa %u:: slot %u\n", i, slot_id);
            }
        }
    }

    u16 top_sym = raw.alpha_remap[TOP];
    DEBUG_PRINTF("top: %hu, kind %s\n", top_sym, to_string(raw.kind).c_str());

    /* create edges, JOIN variables (on edge targets) */
    map<dstate_id_t, GoughEdge> seen;
    for (u32 i = min_state; i < raw.states.size(); ++i) {
        seen.clear(); /* seen is really local to each state */

        DEBUG_PRINTF("creating edges out of %u/%zu\n", i, raw.states.size());
        GoughVertex s = vertices[i];
        const vector<dstate_id_t> &next = raw.states[i].next;
        for (u32 j = 0; j < next.size(); ++j) {
            if (!is_triggered(raw.kind) && j == top_sym) {
                continue;
            }

            dstate_id_t n = next[j];
            DEBUG_PRINTF(" edge to %hu out on %u\n", n, j);
            assert(n < raw.states.size());
            GoughVertex t = vertices[n];

            if (j == top_sym) {
                GoughEdge e = add_edge(s, t, *cfg).first;
                (*cfg)[e].top = true;
                makeCFG_top_edge(*cfg, vertices, joins, raw.trigger_nfa_state,
                                 raw.state_som[i].preds, raw.state_som[n].preds,
                                 i, n, e);
            } else {
                if (contains(seen, n)) {
                    const GoughEdge &e = seen[n];
                    (*cfg)[e].reach.set(j);
                    continue;
                }

                GoughEdge e = add_edge(s, t, *cfg).first;
                (*cfg)[e].reach.set(j);

                seen[n] = e;

                makeCFG_edge(*cfg, raw.new_som_nfa_states, vertices, joins,
                             raw.state_som[i].preds, raw.state_som[n].preds,
                             i, n, e);
            }
        }
    }

    /* populate reports */
    makeCFG_reports(*cfg, raw, joins, vertices);

    using boost::graph_bundle;
    if (is_triggered(raw.kind)) {
        (*cfg)[graph_bundle].initial_vertex = vertices[DEAD_STATE];
    } else {
        (*cfg)[graph_bundle].initial_vertex = vertices[raw.start_anchored];
    }

    return cfg;
}

static
void copy_propagate_report_set(vector<pair<ReportID, GoughSSAVar *> > &rep) {
    vector<pair<ReportID, GoughSSAVar *> >::iterator it = rep.begin();
    while (it != rep.end()) {
        GoughSSAVar *var = it->second;
        if (!var) {
            ++it;
            continue;
        }
        const flat_set<GoughSSAVar *> &inputs = var->get_inputs();
        if (inputs.size() != 1) {
            ++it;
            continue;
        }
        it->second = *inputs.begin(); /* note may result in dupes,
                                         filter later */
    }
}

template<typename VarP>
void copy_propagate_update_vars(vector<VarP> &vars,  bool *changes) {
    for (u32 i = 0; i < vars.size(); i++) {
        GoughSSAVar *vp = vars[i].get();
        const flat_set<GoughSSAVar *> &inputs = vp->get_inputs();

        /* no need to worry about data coming from self; ignore self loops */
        GoughSSAVar *new_input = nullptr;

        if (inputs.size() == 1) {
            new_input = *inputs.begin();
        } else if (inputs.size() == 2) {
            flat_set<GoughSSAVar *>::const_iterator jt = inputs.begin();
            GoughSSAVar *i_0 = *jt;
            GoughSSAVar *i_1 = *++jt;

            if (i_0 == vp) {
                new_input = i_1;
            } else if (i_1 == vp) {
                new_input = i_0;
            }
        }

        if (!new_input) {
            continue;
        }

        assert(new_input != vp);

        /* copy set as it will be modified by iteration */
        const flat_set<GoughSSAVarWithInputs *> outputs = vp->get_outputs();

        for (GoughSSAVar *curr : outputs) {
            curr->replace_input(vp, new_input);
            *changes = true;
        }
    }
}

static
void copy_propagation(GoughGraph &g, const Grey &grey) {
    if (!grey.goughCopyPropagate) {
        return;
    }
    /* TODO order visit of variables sensibly */
    bool changes = false;
    do {
        DEBUG_PRINTF("new iteration\n");
        changes = false;
        for (auto v : vertices_range(g)) {
            copy_propagate_update_vars(g[v].vars, &changes);
        }
        for (const auto &e : edges_range(g)) {
            copy_propagate_update_vars(g[e].vars, &changes);
        }
    } while(changes);

    /* see if any reports can also be moved along */
    for (auto v : vertices_range(g)) {
        copy_propagate_report_set(g[v].reports);
        copy_propagate_report_set(g[v].reports_eod);
    }
}

static
void mark_live_reports(const vector<pair<ReportID, GoughSSAVar *> > &reps,
                       vector<GoughSSAVar *> *queue) {
    for (const auto &r : reps) {
        GoughSSAVar *var = r.second;
        if (!var || var->seen) {
            continue;
        }
        var->seen = true;
        queue->push_back(var);
    }
}

static
void remove_dead(GoughGraph &g) {
    vector<GoughSSAVar *> queue;

    for (auto v : vertices_range(g)) {
        mark_live_reports(g[v].reports, &queue);
        mark_live_reports(g[v].reports_eod, &queue);
    }

    while (!queue.empty()) {
        GoughSSAVar *v = queue.back();
        queue.pop_back();
        for (GoughSSAVar *var : v->get_inputs()) {
            if (var->seen) {
                continue;
            }
            var->seen = true;
            queue.push_back(var);
        }
    }

    /* remove unused variables */
    for (auto v : vertices_range(g)) {
        for (u32 i = 0; i < g[v].vars.size(); i++) {
            GoughSSAVar *var = g[v].vars[i].get();
            if (var->seen) {
                continue;
            }
            var->clear_all();
            g[v].vars.erase(g[v].vars.begin() + i);
            i--;
        }
    }
    for (const auto &e : edges_range(g)) {
        for (u32 i = 0; i < g[e].vars.size(); i++) {
            GoughSSAVar *var = g[e].vars[i].get();
            if (var->seen) {
                continue;
            }
            var->clear_all();
            g[e].vars.erase(g[e].vars.begin() + i);
            i--;
        }
    }
}

static
gough_ins make_gough_ins(u8 op, u32 dest = INVALID_SLOT,
                         u32 src = INVALID_SLOT) {
    assert(dest != INVALID_SLOT || op == GOUGH_INS_END);
    assert(src != INVALID_SLOT || op == GOUGH_INS_END || op == GOUGH_INS_NEW);
    gough_ins rv;
    rv.op = op;
    rv.dest = dest;
    rv.src = src;
    return rv;
}

void GoughSSAVarNew::generate(vector<gough_ins> *out) const {
    assert(slot != INVALID_SLOT);
    out->push_back(make_gough_ins(GOUGH_INS_NEW, slot, adjust));
}

#ifndef NDEBUG
template<typename C, typename K>
bool contains_loose(const C &container, const K &key) {
    for (const auto &elem : container) {
        if (elem == key) {
            return true;
        }
    }
    return false;
}
#endif

void GoughSSAVarMin::generate(vector<gough_ins> *out) const {
    assert(slot != INVALID_SLOT);
    assert(!inputs.empty());
    // assert(inputs.size() > 1);
    vector<u32> input_slots; /* for determinism */
    bool first = true;
    for (const GoughSSAVar *var : inputs) {
        assert(contains_loose(var->outputs, this));
        if (var->slot == slot) {
            /* if the destination is one of the sources, no need to move it */
            first = false;
        } else {
            input_slots.push_back(var->slot);
        }
    }

    sort(input_slots.begin(), input_slots.end());

    for (const u32 &input_slot : input_slots) {
        if (first) {
            out->push_back(make_gough_ins(GOUGH_INS_MOV, slot, input_slot));
            first = false;
        } else {
            out->push_back(make_gough_ins(GOUGH_INS_MIN, slot, input_slot));
        }
    }
}

void GoughSSAVarMin::remove_input_raw(GoughSSAVar *v) {
    assert(contains(inputs, v));
    inputs.erase(v);
}

void GoughSSAVarJoin::generate(UNUSED vector<gough_ins> *out) const {
    assert(0);
}

GoughSSAVar *GoughSSAVarJoin::get_input(const GoughEdge &prev) const {
    for (const auto &var_edge : input_map) {
        if (contains(var_edge.second, prev)) {
            return var_edge.first;
        }
    }
    assert(0);
    return nullptr;
}

const flat_set<GoughEdge> &GoughSSAVarJoin::get_edges_for_input(
                                                 GoughSSAVar *input) const {
    return input_map.at(input);
}

const map<GoughSSAVar *, flat_set<GoughEdge> > &GoughSSAVarJoin::get_input_map()
    const {
    return input_map;
}

void GoughSSAVarJoin::clear_inputs() {
    for (GoughSSAVar *var : input_map | map_keys) {
        assert(contains(var->outputs, this));
        var->outputs.erase(this);
    }
    input_map.clear();
    inputs.clear();
}

void GoughSSAVarJoin::replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) {
    assert(contains(input_map, old_v));
    assert(contains(inputs, old_v));
    if (old_v == new_v) {
        assert(0);
        return;
    }
    insert(&input_map[new_v], input_map[old_v]);
    input_map.erase(old_v);
    inputs.erase(old_v);
    inputs.insert(new_v);
    old_v->outputs.erase(this);
    new_v->outputs.insert(this);
}

void GoughSSAVarJoin::add_input(GoughSSAVar *v, GoughEdge prev) {
    input_map[v].insert(prev);
    inputs.insert(v);
    v->outputs.insert(this);
}

void GoughSSAVarJoin::remove_input_raw(GoughSSAVar *v) {
     assert(contains(inputs, v));
     assert(contains(input_map, v));
     input_map.erase(v);
     inputs.erase(v);
}

static
u32 highest_slot_used(const vector<gough_ins> &program) {
    u32 rv = INVALID_SLOT;
    for (const gough_ins &ins : program) {
        if (rv == INVALID_SLOT) {
            rv = ins.dest;
        } else if (ins.dest != INVALID_SLOT) {
            ENSURE_AT_LEAST(&rv, ins.dest);
        }
        if (rv == INVALID_SLOT) {
            rv = ins.src;
        } else if (ins.src != INVALID_SLOT) {
            ENSURE_AT_LEAST(&rv, ins.src);
        }
    }
    assert(rv != INVALID_SLOT);
    return rv;
}

static
u32 highest_slot_used(const map<gough_edge_id, vector<gough_ins> > &blocks) {
    u32 rv = INVALID_SLOT;
    for (const vector<gough_ins> &ins_list : blocks | map_values) {
        u32 used = highest_slot_used(ins_list);
        if (rv == INVALID_SLOT) {
            rv = used;
        } else if (used != INVALID_SLOT) {
            ENSURE_AT_LEAST(&rv, used);
        }
    }
    return rv;
}

static
void add_to_block(const vector<shared_ptr<GoughSSAVar> > &vars,
                  vector<gough_ins> *out) {
    for (const auto &var : vars) {
        var->generate(out);
    }
}

namespace {
struct edge_join_info {
    bool empty() const { return dest_to_src.empty(); }

    void insert(u32 src, u32 dest) {
        assert(!contains(dest_to_src, dest));
        assert(src != dest);
        dest_to_src[dest] = src;
        src_to_dest[src].insert(dest);
    }

    void erase(u32 src, u32 dest) {
        assert(dest_to_src.at(dest) == src);
        dest_to_src.erase(dest);
        src_to_dest[src].erase(dest);

        if (src_to_dest[src].empty()) {
            src_to_dest.erase(src);
        }
    }

    bool is_src(u32 v) const {
        bool rv = contains(src_to_dest, v);
        assert(!rv || !src_to_dest.at(v).empty());
        return rv;
    }

    bool is_dest(u32 v) const {
        return contains(dest_to_src, v);
    }

    void remap_src(u32 old_src, u32 new_src) {
        assert(is_src(old_src));
        assert(!is_src(new_src));

        for (const u32 &e : src_to_dest[old_src]) {
            assert(e != new_src);
            dest_to_src[e] = new_src;
        }
        src_to_dest[new_src].swap(src_to_dest[old_src]);
        src_to_dest.erase(old_src);

        assert(!is_src(old_src));
        assert(is_src(new_src));
    }

    /* returns an arbitrary unresolved entry */
    void get_pending(u32 *src, u32 *dest) {
        assert(!empty());
        *dest = dest_to_src.begin()->first;
        *src = dest_to_src.begin()->second;
    }

    const map<u32, u32> &get_dest_mapping() const { return dest_to_src; }

private:
    map<u32, set<u32> > src_to_dest;
    map<u32, u32> dest_to_src;
};

}

static
void prep_joins_for_generation(const GoughGraph &g, GoughVertex v,
                               map<GoughEdge, edge_join_info> *edge_info) {
    DEBUG_PRINTF("writing out joins for %u\n", g[v].state_id);
    for (const auto &var : g[v].vars) {
        u32 dest_slot = var->slot;
        for (const auto &var_edges : var->get_input_map()) {
            u32 input = var_edges.first->slot;
            if (dest_slot == input) {
                continue;
            }

            for (const GoughEdge &incoming_edge : var_edges.second) {
                (*edge_info)[incoming_edge].insert(input, dest_slot);
                DEBUG_PRINTF("need %u<-%u\n", dest_slot, input);
            }
        }
    }
}

static
void add_simple_joins(edge_join_info &eji, vector<gough_ins> *out) {
    /* any slot whose value we don't need can be written to immediately */
    const map<u32, u32> &dest_to_src = eji.get_dest_mapping();

    bool changed;
    do {
        changed = false;
        for (map<u32, u32>::const_iterator it = dest_to_src.begin();
             it != dest_to_src.end();) {
            u32 src = it->second;
            u32 dest = it->first;
            ++it; /* avoid iterator being invalidated */

            if (eji.is_src(dest)) {
                continue; /* conflict; not simple (yet) */
            }

            /* value of destination slot is not used by any remaining joins;
             * we can output this join immediately */
            DEBUG_PRINTF("out %u<-%u\n", dest, src);
            out->push_back(make_gough_ins(GOUGH_INS_MOV, dest, src));

            eji.erase(src, dest);

            if (eji.is_dest(src) && eji.is_src(src)) {
                /* we can unblock src being used as an output by shifting
                 * across everybody using src as input to using dest (as == src
                 * now) */
                eji.remap_src(src, dest);
            }
            changed = true;
        }
    } while (changed);
}

static
void add_joins_to_block(edge_join_info &eji, vector<gough_ins> *out,
                        u32 base_temp_slot) {
    /* joins happen concurrently: none of them should see the outputs of another
     * join happening due to the same entry of the vertex. If there are
     * conflicts we may have to handle things by using a temp output slot for
     * each join and then copying into the final slot.
     */

    add_simple_joins(eji, out);
    while (!eji.empty()) {
        u32 split;
        u32 input_for_split;
        eji.get_pending(&input_for_split, &split);

        assert(eji.is_src(split)); /* otherwise should be handled by simple */

        /* stash the initial value of the split register in a temp register */
        u32 temp = base_temp_slot++;
        DEBUG_PRINTF("out %u<-%u\n", temp, split);
        out->push_back(make_gough_ins(GOUGH_INS_MOV, temp, split));
        eji.remap_src(split, temp); /* update maps */

        /* split can now be safely written out to as all the uses of it as an
         * input now refer to temp instead */

        DEBUG_PRINTF("out %u<-%u\n", split, input_for_split);
        out->push_back(make_gough_ins(GOUGH_INS_MOV, split, input_for_split));
        eji.erase(input_for_split, split);

        /* handle any uncovered simple cases */
        add_simple_joins(eji, out);
     }
}

static
void build_blocks(const GoughGraph &g,
                  map<gough_edge_id, vector<gough_ins> > *blocks,
                  u32 base_temp_slot) {
    for (const auto &e : edges_range(g)) {
        if (g[e].vars.empty()) {
            continue;
        }

        vector<gough_ins> &block = (*blocks)[gough_edge_id(g, e)];
        add_to_block(g[e].vars, &block);
        assert(!block.empty());
    }

    for (const auto t : vertices_range(g)) {
        if (g[t].vars.empty()) {
            continue;
        }

        map<GoughEdge, edge_join_info> eji;
        prep_joins_for_generation(g, t, &eji);

        for (auto &m : eji) {
            vector<gough_ins> &block = (*blocks)[gough_edge_id(g, m.first)];
            u32 cur_base = base_temp_slot;
            if (!block.empty()) {
                /* some temp slots may already be in use by short-lived vars */
                ENSURE_AT_LEAST(&cur_base, highest_slot_used(block) + 1);
            }

            add_joins_to_block(m.second, &block, cur_base);
            if (block.empty()) {
                blocks->erase(gough_edge_id(g, m.first));
            }
        }
    }

    for (vector<gough_ins> &ins_list : *blocks | map_values) {
        assert(!ins_list.empty());
        ins_list.push_back(make_gough_ins(GOUGH_INS_END));
    }
}

static
void copy_in_blocks(raw_som_dfa &raw, u8 alphaShift, const GoughGraph &cfg,
                    const map<gough_edge_id, vector<gough_ins> > &blocks,
                    u32 *edge_blocks, u32 *top_blocks, u32 base_offset,
                    map<vector<gough_ins>, u32> *prog_offsets,
                    vector<gough_ins> *out) {
    u32 impl_alpha_size = 1U << alphaShift;
    UNUSED u32 top_sym = raw.alpha_remap[TOP];
    assert(top_sym == raw.alpha_size - 1U);
    map<vector<gough_ins>, u32> &processed = *prog_offsets;

    for (const auto &e : edges_range(cfg)) {
        if (!contains(blocks, gough_edge_id(cfg, e))) {
            continue;
        }
        const vector<gough_ins> &block = blocks.at(gough_edge_id(cfg, e));
        u32 prog_offset;
        if (!contains(processed, block)) {
            prog_offset = base_offset + byte_length(*out);
            insert(out, out->end(), block);
            processed[block] = prog_offset;
        } else {
            prog_offset = processed[block];
        }

        /* update edges */
        u32 s_id = cfg[source(e, cfg)].state_id;
        UNUSED u32 t_id = cfg[target(e, cfg)].state_id;
        u32 impl_src_id = raw.states[s_id].impl_id;
        DEBUG_PRINTF("%u: writing out block for edge_%u_%u at %u:\n",
                     impl_src_id, s_id, t_id,prog_offset);

        for (u32 j = cfg[e].reach.find_first(); j != CharReach::npos;
             j = cfg[e].reach.find_next(j)) {
            assert(raw.states[s_id].next[j] == t_id);
            u32 edge_index = impl_src_id * impl_alpha_size + j;
            DEBUG_PRINTF("\tsetting on %u, %u\n", j, edge_index);
            edge_blocks[edge_index] = prog_offset;
        }

        if (cfg[e].top) {
            assert(raw.states[s_id].next[top_sym] == t_id);
            DEBUG_PRINTF("\tsetting top on %u to block at %u\n", impl_src_id,
                         prog_offset);
            top_blocks[impl_src_id] = prog_offset;
        }
    }
}

bool find_normal_self_loop(GoughVertex v, const GoughGraph &g, GoughEdge *out) {
    for (const auto &e : out_edges_range(v, g)) {
        if (target(e, g) != v) {
            continue;
        }
        if (g[e].top) {
            assert(g[e].reach.find_first() == CharReach::npos);
            continue; /* corresponds to a top, not a normal transition */
        }

        *out = e;
        return true;
    }

    return false;
}

static never_inline
void update_accel_prog_offset(const gough_build_strat &gbs,
                          const map<gough_edge_id, vector<gough_ins> > &blocks,
                          const map<vector<gough_ins>, u32> &prog_offsets) {
    map<dstate_id_t, GoughVertex> verts;
    for (auto v : vertices_range(gbs.gg)) {
        verts[gbs.gg[v].state_id] = v;
    }

    for (auto &m : gbs.built_accel) {
        gough_accel *ga = m.first;
        assert(!ga->prog_offset);
        GoughVertex v = verts[m.second];
        GoughEdge e;
        UNUSED bool rv = find_normal_self_loop(v, gbs.gg, &e);
        assert(rv);

        if (!rv) {
            continue;
        }

        DEBUG_PRINTF("updating state %u accel with margin %hhu\n",
                     gbs.gg[v].state_id, ga->margin_dist);
        if (contains(blocks, gough_edge_id(gbs.gg, e))) {
            const vector<gough_ins> &block
                = blocks.at(gough_edge_id(gbs.gg, e));
            ga->prog_offset = prog_offsets.at(block);
            DEBUG_PRINTF("prog offset %u\n", ga->prog_offset);
        } else {
            ga->margin_dist = 0;
            DEBUG_PRINTF("removing margin as no som\n");
        }
    }
}

bytecode_ptr<NFA> goughCompile(raw_som_dfa &raw, u8 somPrecision,
                               const CompileContext &cc,
                               const ReportManager &rm) {
    assert(somPrecision == 2 || somPrecision == 4 || somPrecision == 8
           || !cc.streaming);

    if (!cc.grey.allowGough) {
        return nullptr;
    }

    DEBUG_PRINTF("hello world\n");
    unique_ptr<GoughGraph> cfg = makeCFG(raw);
    dump(*cfg, "init", cc.grey);
    copy_propagation(*cfg, cc.grey);
    remove_dead(*cfg);
    dump(*cfg, "prop", cc.grey);
    u32 slot_count = assign_slots(*cfg, cc.grey);
    dump(*cfg, "slots", cc.grey);

    map<gough_edge_id, vector<gough_ins> > blocks;
    build_blocks(*cfg, &blocks, slot_count);
    DEBUG_PRINTF("%u slots\n", highest_slot_used(blocks) + 1);

    u32 scratch_slot_count = highest_slot_used(blocks) + 1;
    assert(slot_count <= scratch_slot_count);

    dump(*cfg, "final", cc.grey);
    dump_blocks(blocks, "final", cc.grey);

    gough_info gi;
    memset(&gi, 0, sizeof(gi));

    map<dstate_id_t, gough_accel_state_info> accel_allowed;
    find_allowed_accel_states(*cfg, blocks, &accel_allowed);
    gough_build_strat gbs(raw, *cfg, rm, accel_allowed);
    auto basic_dfa = mcclellanCompile_i(raw, gbs, cc);
    assert(basic_dfa);
    if (!basic_dfa) {
        return nullptr;
    }

    u8 alphaShift
        = ((const mcclellan *)getImplNfa(basic_dfa.get()))->alphaShift;
    u32 edge_count = (1U << alphaShift) * raw.states.size();

    u32 curr_offset = ROUNDUP_N(basic_dfa->length, 4);

    u32 haig_offset = curr_offset;
    curr_offset += sizeof(gi);
    /* reserve space for edge->program mapping */
    u32 edge_prog_offset = curr_offset;
    curr_offset += sizeof(u32) * edge_count;
    vector<u32> edge_blocks(edge_count);

    u32 top_prog_offset = 0;
    if (is_triggered(raw.kind)) {
        /* reserve space for edge->program mapping */
        top_prog_offset = curr_offset;
        curr_offset += sizeof(u32) * raw.states.size();
    }
    gi.top_prog_offset = top_prog_offset;
    vector<u32> top_blocks(raw.states.size());

    /* reserve space for blocks */
    u32 prog_base_offset = curr_offset;
    gi.prog_base_offset = prog_base_offset;

    vector<gough_ins> temp_blocks;
    map<vector<gough_ins>, u32> prog_offsets;
    copy_in_blocks(raw, alphaShift, *cfg, blocks, &edge_blocks[0],
                   &top_blocks[0], prog_base_offset, &prog_offsets,
                   &temp_blocks);
    update_accel_prog_offset(gbs, blocks, prog_offsets);

    u32 total_prog_size = byte_length(temp_blocks);
    curr_offset += total_prog_size;

    gi.stream_som_loc_count = slot_count;
    gi.stream_som_loc_width = somPrecision;

    u32 gough_size = ROUNDUP_N(curr_offset, 16);
    auto gough_dfa = make_zeroed_bytecode_ptr<NFA>(gough_size);

    memcpy(gough_dfa.get(), basic_dfa.get(), basic_dfa->length);
    memcpy((char *)gough_dfa.get() + haig_offset, &gi, sizeof(gi));
    if (gough_dfa->type == MCCLELLAN_NFA_16) {
        gough_dfa->type = GOUGH_NFA_16;
    } else {
        assert(gough_dfa->type == MCCLELLAN_NFA_8);
        gough_dfa->type = GOUGH_NFA_8;
    }

    /* update stream state requirements */
    u32 base_state_size = gough_dfa->type == GOUGH_NFA_8 ? 1 : 2;
    gough_dfa->streamStateSize = base_state_size + slot_count * somPrecision;
    gough_dfa->scratchStateSize = (u32)(16 + scratch_slot_count * sizeof(u64a));

    mcclellan *m = (mcclellan *)getMutableImplNfa(gough_dfa.get());
    m->haig_offset = haig_offset;

    /* update nfa length, haig_info offset (leave mcclellan length alone) */
    gough_dfa->length = gough_size;

    /* copy in blocks */
    copy_bytes((u8 *)gough_dfa.get() + edge_prog_offset, edge_blocks);
    if (top_prog_offset) {
        copy_bytes((u8 *)gough_dfa.get() + top_prog_offset, top_blocks);
    }
    copy_bytes((u8 *)gough_dfa.get() + prog_base_offset, temp_blocks);

    return gough_dfa;
}

AccelScheme gough_build_strat::find_escape_strings(dstate_id_t this_idx) const {
    AccelScheme rv;
    if (!contains(accel_gough_info, this_idx)) {
        rv.cr = CharReach::dot();
        rv.double_byte.clear();
        return rv;
    }

    rv = mcclellan_build_strat::find_escape_strings(this_idx);

    assert(!rv.offset || rv.cr.all()); /* should have been limited by strat */
    if (rv.offset) {
        rv.cr = CharReach::dot();
        rv.double_byte.clear();
        return rv;
    }

    if (rv.double_offset
        || !accel_gough_info.at(this_idx).two_byte) {
        rv.double_byte.clear();
    }

    return rv;
}

void gough_build_strat::buildAccel(dstate_id_t this_idx, const AccelScheme &info,
                                   void *accel_out) {
    assert(mcclellan_build_strat::accelSize() == sizeof(AccelAux));
    gough_accel *accel = (gough_accel *)accel_out;
    /* build a plain accelaux so we can work out where we can get to */
    mcclellan_build_strat::buildAccel(this_idx, info, &accel->accel);
    DEBUG_PRINTF("state %hu is accel with type %hhu\n", this_idx,
                 accel->accel.accel_type);
    if (accel->accel.accel_type == ACCEL_NONE) {
        return;
    }

    assert(!accel->accel.generic.offset);
    assert(contains(accel_gough_info, this_idx));
    accel->margin_dist = verify_u8(accel_gough_info.at(this_idx).margin);
    built_accel[accel] = this_idx;
    DEBUG_PRINTF("state %hu is accel with margin %hhu\n", this_idx,
                 accel->margin_dist);
}

namespace {
struct raw_gough_report_list {
    set<som_report> reports;

    raw_gough_report_list(
        const vector<pair<ReportID, GoughSSAVar *>> &raw_reports,
        const ReportManager &rm, bool do_remap) {
        for (const auto &m : raw_reports) {
            ReportID r = do_remap ? rm.getProgramOffset(m.first) : m.first;
            u32 impl_slot = INVALID_SLOT;
            if (m.second) {
                impl_slot = m.second->slot;
                assert(impl_slot != INVALID_SLOT);
            }
            reports.emplace(r, impl_slot);
        }
    }

    bool operator<(const raw_gough_report_list &b) const {
        return reports < b.reports;
    }
};

struct raw_gough_report_info_impl : public raw_report_info {
    vector<raw_gough_report_list> rl;
    u32 getReportListSize() const override;
    size_t size() const override;
    void fillReportLists(NFA *n, size_t base_offset,
                         vector<u32> &ro /* out */) const override;
};
}

unique_ptr<raw_report_info> gough_build_strat::gatherReports(
                                                  vector<u32> &reports,
                                                  vector<u32> &reports_eod,
                                                  u8 *isSingleReport,
                                                  ReportID *arbReport) const {
    DEBUG_PRINTF("gathering reports\n");

    const bool remap_reports = has_managed_reports(rdfa.kind);

    auto ri = ue2::make_unique<raw_gough_report_info_impl>();
    map<raw_gough_report_list, u32> rev;

    assert(!rdfa.states.empty());

    vector<GoughVertex> verts(rdfa.states.size());
    for (auto v : vertices_range(gg)) {
        verts[gg[v].state_id] = v;
    }

    for (u32 state_id = 0; state_id < verts.size(); state_id++) {
        assert(state_id < rdfa.states.size());
        GoughVertex v = verts[state_id];
        assert(v != GoughGraph::null_vertex() || !state_id);

        DEBUG_PRINTF("i = %zu [%zu]\n", reports.size(), gg[v].reports.size());
        if (v == GoughGraph::null_vertex() || gg[v].reports.empty()) {
            reports.push_back(MO_INVALID_IDX);
            continue;
        }

        raw_gough_report_list rrl(gg[v].reports, rm, remap_reports);
        DEBUG_PRINTF("non empty r %zu\n", reports.size());
        if (rev.find(rrl) != rev.end()) {
            reports.push_back(rev[rrl]);
        } else {
            DEBUG_PRINTF("adding to rl\n");
            rev[rrl] = ri->size();
            reports.push_back(ri->size());
            ri->rl.push_back(rrl);
        }
    }

    for (auto v : verts) {
        if (v == GoughGraph::null_vertex() || gg[v].reports_eod.empty()) {
            reports_eod.push_back(MO_INVALID_IDX);
            continue;
        }

        DEBUG_PRINTF("non empty r eod\n");
        raw_gough_report_list rrl(gg[v].reports_eod, rm, remap_reports);
        if (rev.find(rrl) != rev.end()) {
            reports_eod.push_back(rev[rrl]);
            continue;
        }

        DEBUG_PRINTF("adding to rl eod %zu\n", gg[v].reports_eod.size());
        rev[rrl] = ri->size();
        reports_eod.push_back(ri->size());
        ri->rl.push_back(rrl);
    }

    /* TODO: support single report in gough */
    *isSingleReport = 0;
    *arbReport = MO_INVALID_IDX;
    assert(!ri->rl.empty()); /* all components should be able to generate
                                reports */
    return move(ri);
}

u32 raw_gough_report_info_impl::getReportListSize() const {
    u32 sz = 0;

    for (const raw_gough_report_list &r : rl) {
        sz += sizeof(gough_report_list);
        sz += sizeof(gough_report) * r.reports.size();
    }

    return sz;
}

size_t raw_gough_report_info_impl::size() const {
    return rl.size();
}

void raw_gough_report_info_impl::fillReportLists(NFA *n, size_t base_offset,
                                                 vector<u32> &ro) const {
    for (const raw_gough_report_list &r : rl) {
        ro.push_back(base_offset);

        gough_report_list *p = (gough_report_list *)((char *)n + base_offset);
        u32 i = 0;

        for (const som_report &sr : r.reports) {
            p->report[i].r = sr.report;
            p->report[i].som = sr.slot;
            i++;
        }

        p->count = verify_u32(r.reports.size());

        base_offset += sizeof(gough_report_list);
        base_offset += sizeof(gough_report) * r.reports.size();
    }
}

} // namespace ue2
