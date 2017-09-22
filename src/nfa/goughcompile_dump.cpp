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

#include "config.h"

#include "goughcompile_dump.h"
#include "goughcompile_internal.h"
#include "grey.h"
#include "util/container.h"
#include "util/dump_util.h"
#include "util/graph_range.h"

#include <sstream>
#include <string>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

string dump_name(const GoughVertexProps &vp) {
    stringstream ss;
    ss << "vertex_" << vp.state_id;
    return ss.str();
}

static
string dump_name(const GoughGraph &g, const GoughEdge &e) {
    stringstream ss;
    ss << "edge_" << g[source(e, g)].state_id << "_"
       << g[target(e, g)].state_id;
    return ss.str();
}

string dump_name(const gough_edge_id &e) {
    stringstream ss;
    ss << "edge_" << e.src << "_" << e.dest;
    return ss.str();
}

static
void dump_graph(const GoughGraph &g, const string &base, const Grey &grey) {
    StdioFile f(grey.dumpPath + "gough_" + base + ".dot", "w");

    fprintf(f, "digraph NFA {\n");
    fprintf(f, "rankdir=LR;\n");
    fprintf(f, "size=\"11.5,8\"\n");
    fprintf(f, "node [ shape = circle ];\n");
    fprintf(f, "START [style=invis];\n");

    for (auto v : vertices_range(g)) {
        fprintf(f, "%s [ width = 1, fixedsize = true, fontsize = 12, ",
                dump_name(g[v]).c_str());
        if (!g[v].reports.empty() || !g[v].reports_eod.empty()) {
            fprintf(f, "shape = doublecircle ");
        }

        fprintf(f, "label = \"%u\"];\n", g[v].state_id);
    }
    for (const auto &e : edges_range(g)) {
        GoughVertex s = source(e, g);
        GoughVertex t = target(e, g);

        fprintf(f, "%s -> %s\n",
                dump_name(g[s]).c_str(), dump_name(g[t]).c_str());
    }
    fprintf(f, "}\n");
}

static
set<const GoughSSAVar *> uses(const GoughVertexProps &vp) {
    set<const GoughSSAVar *> rv;
    for (const auto &r : vp.reports) {
        if (r.second) {
            rv.insert(r.second);
        }
    }

    for (const auto &r : vp.reports_eod) {
        if (r.second) {
            rv.insert(r.second);
        }
    }

    for (const auto &var : vp.vars) {
        insert(&rv, var->get_inputs());
    }

    return rv;
}

static
set<const GoughSSAVar *> uses(const GoughEdgeProps &ep) {
    set<const GoughSSAVar *> rv;
    for (const auto &var : ep.vars) {
        insert(&rv, var->get_inputs());
    }

    return rv;
}

static
void dump_var_mapping(const GoughGraph &g, const string &base,
                      const Grey &grey) {
    StdioFile f(grey.dumpPath + "gough_" + base + "_vars.txt", "w");
    for (auto v : vertices_range(g)) {
        set<const GoughSSAVar *> used = uses(g[v]);
        if (g[v].vars.empty() && used.empty()) {
            continue;
        }
        fprintf(f, "%s\n", dump_name(g[v]).c_str());
        for (u32 i = 0; i < g[v].vars.size(); i++) {
            const GoughSSAVar *vp = g[v].vars[i].get();
            fprintf(f, "\t%u: slot %u\n", i, vp->slot);
        }
        if (!used.empty()) {
            fprintf(f, "\tuses:");
            vector<u32> used_id;
            for (const GoughSSAVar *var : used) {
                used_id.push_back(var->slot);
            }
            for (const u32 &id : used_id) {
                fprintf(f, " %u", id);
            }
            fprintf(f, "\n");
        }
    }
    for (const auto &e : edges_range(g)) {
        set<const GoughSSAVar *> used = uses(g[e]);
        if (g[e].vars.empty() && used.empty()) {
            continue;
        }
        fprintf(f, "%s\n", dump_name(g, e).c_str());
        for (u32 i = 0; i < g[e].vars.size(); i++) {
            const GoughSSAVar *vp = g[e].vars[i].get();
            fprintf(f, "\t%u: slot %u\n", i, vp->slot);
        }
        if (!used.empty()) {
            fprintf(f, "\tuses:");
            vector<u32> used_id;
            for (const GoughSSAVar *var : used) {
                used_id.push_back(var->slot);
            }
            for (const u32 &id : used_id) {
                fprintf(f, " %u", id);
            }
            fprintf(f, "\n");
        }
    }
}

static
void gather_vars(const GoughGraph &g, vector<const GoughSSAVar *> *vars,
                 map<const GoughSSAVar *, string> *names,
                 map<const GoughSSAVar *, string> *src_label,
                 set<const GoughSSAVar *> *reporters) {
    for (auto v : vertices_range(g)) {
        for (const auto &r : g[v].reports) {
            reporters->insert(r.second);
        }
        for (const auto &r : g[v].reports_eod) {
            reporters->insert(r.second);
        }

        for (u32 i = 0; i < g[v].vars.size(); i++) {
            const GoughSSAVar *vp = g[v].vars[i].get();
            stringstream ss;
            ss << dump_name(g[v]) << "_" << i;
            vars->push_back(vp);
            names->insert(make_pair(vp, ss.str()));
            src_label->insert(make_pair(vp, dump_name(g[v])));
        }
    }

    for (const auto &e : edges_range(g)) {
        for (u32 i = 0; i < g[e].vars.size(); i++) {
            const GoughSSAVar *vp = g[e].vars[i].get();
            stringstream ss;
            ss << dump_name(g, e) << "_" << i;
            vars->push_back(vp);
            names->insert(make_pair(vp, ss.str()));
            src_label->insert(make_pair(vp, dump_name(g, e)));
        }
    }
}

static
void dump_vars(const GoughGraph &g, const string &base, const Grey &grey) {
    StdioFile f(grey.dumpPath + "gough_" + base + "_vars.dot", "w");
    fprintf(f, "digraph NFA {\n");
    fprintf(f, "rankdir=LR;\n");
    fprintf(f, "size=\"11.5,8\"\n");
    fprintf(f, "node [ shape = circle ];\n");
    fprintf(f, "START [style=invis];\n");

    vector<const GoughSSAVar *> vars;
    map<const GoughSSAVar *, string> names;
    map<const GoughSSAVar *, string> src_label;
    set<const GoughSSAVar *> reporters;
    gather_vars(g, &vars, &names, &src_label, &reporters);

    for (const GoughSSAVar *vp : vars) {
        fprintf(f, "%s [ width = 1, fixedsize = true, fontsize = 12, ",
                names[vp].c_str());
        fprintf(f, "label = \"%s\\n", src_label[vp].c_str());

        if (dynamic_cast<const GoughSSAVarMin *>(vp)) {
            fprintf(f, "MIN");
        } else if (dynamic_cast<const GoughSSAVarJoin *>(vp)) {
            fprintf(f, "JOIN");
        } else if (dynamic_cast<const GoughSSAVarNew *>(vp)) {
            fprintf(f, "NEW");
        } else {
            fprintf(f, "???");
        }
        fprintf(f, "\"];\n");
    }

    for (const GoughSSAVar *vp : reporters) {
        if (vp) {
            fprintf(f, "%s [ shape = doublecircle]\n", names[vp].c_str());
        } else {
            fprintf(f, "eps [ label = \"eps\" shape = doublecircle]\n");
        }
    }

    for (const GoughSSAVar *vp : vars) {
        const flat_set<GoughSSAVar *> &inputs = vp->get_inputs();
        for (const GoughSSAVar *v_in : inputs) {
            fprintf(f, "%s -> %s\n", names[v_in].c_str(), names[vp].c_str());
        }
    }

    fprintf(f, "}\n");
}

void dump(const GoughGraph &g, const string &base, const Grey &grey) {
    if (!(grey.dumpFlags & Grey::DUMP_INT_GRAPH)) {
        return;
    }

    dump_graph(g, base, grey);
    dump_var_mapping(g, base, grey);
    dump_vars(g, base, grey);
}

static
void dump_block(FILE *f, const gough_edge_id &e,
                const vector<gough_ins> &block) {
    fprintf(f, "%s:\n", dump_name(e).c_str());
    for (const gough_ins &ins : block) {
        fprintf(f, "\t");
        switch (ins.op) {
        case GOUGH_INS_END:
            fprintf(f, "END");
            break;
        case GOUGH_INS_MOV:
            fprintf(f, "MOV %u %u", ins.dest, ins.src);
            break;
        case GOUGH_INS_NEW:
            fprintf(f, "NEW %u (+%u)", ins.dest, ins.src);
            break;
        case GOUGH_INS_MIN:
            fprintf(f, "MIN %u %u", ins.dest, ins.src);
            break;
        default:
            fprintf(f, "<UNKNOWN>");
            break;
        }
        fprintf(f, "\n");
    }
}

void dump_blocks(const map<gough_edge_id, vector<gough_ins>> &blocks,
                 const string &base, const Grey &grey) {
    if (!(grey.dumpFlags & Grey::DUMP_INT_GRAPH)) {
        return;
    }

    StdioFile f(grey.dumpPath + "gough_" + base + "_programs.txt", "w");

    for (const auto &m : blocks) {
        dump_block(f, m.first, m.second);
    }
}

} // namespace ue2
