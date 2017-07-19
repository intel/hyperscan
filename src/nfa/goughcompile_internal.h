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

#ifndef GOUGHCOMPILE_INTERNAL_H
#define GOUGHCOMPILE_INTERNAL_H

#include "gough_internal.h"
#include "mcclellancompile.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/flat_containers.h"
#include "util/noncopyable.h"
#include "util/order_check.h"

#include <map>
#include <memory>
#include <set>
#include <vector>

#include <boost/graph/adjacency_list.hpp>

namespace ue2 {

struct Grey;
struct GoughSSAVar;
struct GoughSSAVarJoin;

struct GoughVertexProps {
    GoughVertexProps() {}
    explicit GoughVertexProps(u32 state_in) : state_id(state_in) {}
    u32 state_id = ~0U;

    std::vector<std::shared_ptr<GoughSSAVarJoin> > vars; /* owns variables */

    std::vector<std::pair<ReportID, GoughSSAVar *> > reports; /**< report som,
                                                                som variable */
    std::vector<std::pair<ReportID, GoughSSAVar *> > reports_eod;
};

struct GoughEdgeProps {
    GoughEdgeProps(void) : top(false) {}
    bool top;
    CharReach reach;

    std::vector<std::shared_ptr<GoughSSAVar> > vars; /* owns variables */
};

struct GoughGraphProps {
    boost::adjacency_list_traits<boost::vecS, boost::vecS>::vertex_descriptor
        initial_vertex; /* for triggered nfas, dead state;
                         * for others start anchored or start floating
                         */
};

typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS,
    GoughVertexProps, GoughEdgeProps, GoughGraphProps> GoughGraph;

typedef GoughGraph::vertex_descriptor GoughVertex;
typedef GoughGraph::edge_descriptor GoughEdge;

struct gough_edge_id {
    gough_edge_id(const GoughGraph &g, const GoughEdge &e)
        : src(g[source(e, g)].state_id), dest(g[target(e, g)].state_id),
          first_char(g[e].reach.find_first()) {}
    bool operator<(const gough_edge_id &b) const {
        const gough_edge_id &a = *this;
        ORDER_CHECK(src);
        ORDER_CHECK(dest);
        ORDER_CHECK(first_char);
        return false;
    }
    const u32 src;
    const u32 dest;
    const u32 first_char; /* ~0U if only top */
};

struct GoughSSAVarWithInputs;
struct GoughSSAVarMin;
struct GoughSSAVarJoin;

struct GoughSSAVar : noncopyable {
    GoughSSAVar(void) : seen(false), slot(INVALID_SLOT) {}
    virtual ~GoughSSAVar();
    const flat_set<GoughSSAVar *> &get_inputs() const {
        return inputs;
    }
    const flat_set<GoughSSAVarWithInputs *> &get_outputs() const {
        return outputs;
    }
    virtual void replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) = 0;

    virtual void generate(std::vector<gough_ins> *out) const = 0;

    bool seen; /* for temp use by remove_dead alg */
    u32 slot;

    void clear_outputs();

    /** remove all inputs and outputs of the vertex, call before
     * removing vertex */
    virtual void clear_all() {
        clear_outputs();
    }
protected:
    flat_set<GoughSSAVar *> inputs;
    flat_set<GoughSSAVarWithInputs *> outputs;
    friend struct GoughSSAVarWithInputs;
    friend struct GoughSSAVarMin;
    friend struct GoughSSAVarJoin;
};

struct GoughSSAVarNew : public GoughSSAVar {
    explicit GoughSSAVarNew(u32 adjust_in) : adjust(adjust_in) {}

    void replace_input(GoughSSAVar *, GoughSSAVar *) override {
        assert(0);
    }

    void generate(std::vector<gough_ins> *out) const override;

    const u32 adjust;
};

struct GoughSSAVarWithInputs : public GoughSSAVar {
    GoughSSAVarWithInputs(void) {}
    void replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) override = 0;
    virtual void clear_inputs() = 0;
    void clear_all() override;
protected:
    virtual void remove_input_raw(GoughSSAVar *v) = 0;
    friend struct GoughSSAVar;
};

struct GoughSSAVarMin : public GoughSSAVarWithInputs {
    GoughSSAVarMin(void) {}
    void generate(std::vector<gough_ins> *out) const override;

    void clear_inputs() override;
    void replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) override;

    virtual void add_input(GoughSSAVar *v) {
        inputs.insert(v);
        v->outputs.insert(this);
    }

protected:
    void remove_input_raw(GoughSSAVar *v) override;
};

struct GoughSSAVarJoin : public GoughSSAVarWithInputs {
    GoughSSAVarJoin(void) {}

    /* dummy; all joins at a point must be generated simultaneously */
    void generate(std::vector<gough_ins> *out) const override;
    GoughSSAVar *get_input(const GoughEdge &prev) const;

    void clear_inputs() override;
    void replace_input(GoughSSAVar *old_v, GoughSSAVar *new_v) override;

    void add_input(GoughSSAVar *v, GoughEdge prev);

    const flat_set<GoughEdge> &get_edges_for_input(GoughSSAVar *input) const;
    const std::map<GoughSSAVar *, flat_set<GoughEdge>> &get_input_map() const;

protected:
    void remove_input_raw(GoughSSAVar *v) override;

private:
    std::map<GoughSSAVar *, flat_set<GoughEdge>> input_map;
};

struct gough_accel_state_info {
    u32 margin;
    bool two_byte;

    gough_accel_state_info(u32 margin_in, bool two_byte_in)
        : margin(margin_in), two_byte(two_byte_in) {
    }
};

u32 assign_slots(GoughGraph &g, const Grey &grey);
void find_allowed_accel_states(const GoughGraph &g,
             const std::map<gough_edge_id, std::vector<gough_ins> > &blocks,
             std::map<dstate_id_t, gough_accel_state_info> *out);
bool find_normal_self_loop(GoughVertex v, const GoughGraph &g, GoughEdge *out);

} // namespace ue2

// Note: C structure, can't be in namespace ue2
static inline
bool operator==(const gough_ins &a, const gough_ins &b) {
    return a.op == b.op && a.dest == b.dest && a.src == b.src;
}

static inline
bool operator<(const gough_ins &a, const gough_ins &b) {
    return std::tie(a.op, a.src, a.dest) < std::tie(b.op, b.src, b.dest);
}

#endif
