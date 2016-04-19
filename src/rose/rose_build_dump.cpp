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

#include "config.h"

#include "rose_build_dump.h"

#include "hwlm/hwlm_build.h"
#include "rose_build_impl.h"
#include "rose_build_matchers.h"
#include "rose/rose_dump.h"
#include "rose_internal.h"
#include "ue2common.h"
#include "nfa/nfa_internal.h"
#include "nfagraph/ng_dump.h"
#include "som/slot_manager_dump.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/graph_range.h"
#include "util/ue2string.h"

#include <iomanip>
#include <ostream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

static
string to_string(nfa_kind k) {
    switch (k) {
    case NFA_PREFIX:
        return "p";
    case NFA_INFIX:
        return "i";
    case NFA_SUFFIX:
        return "s";
    case NFA_OUTFIX:
        return "o";
    case NFA_REV_PREFIX:
        return "r";
    case NFA_OUTFIX_RAW:
        return "O";
    }
    assert(0);
    return "?";
}

namespace {

class RoseGraphWriter {
public:
    RoseGraphWriter(const RoseBuildImpl &b_in, const RoseEngine *t_in) :
        build(b_in), t(t_in) {
        for (const auto &m : build.ghost) {
            ghost.insert(m.second);
        }
    }

    void operator() (ostream &os, const RoseVertex &v) const {
        const RoseGraph &g = build.g;

        if (v == build.root) {
            os << "[label=\"<root>\"]";
            return;
        }

        if (v == build.anchored_root) {
            os << "[label=\"<^>\"]";
            return;
        }

        os << "[label=\"";
        os << "idx=" << g[v].idx <<"\\n";

        for (u32 lit_id : g[v].literals) {
            writeLiteral(os, lit_id);
            os << "\\n";
        }

        os << "min_offset=" << g[v].min_offset;
        if (g[v].max_offset >= ROSE_BOUND_INF) {
            os << ", max_offset=inf";
        } else {
            os << ", max_offset=" << g[v].max_offset;
        }
        os << "\\n";

        if (!g[v].reports.empty()) {
            if (g[v].eod_accept) {
                os << "\\nACCEPT_EOD";
            } else {
                os << "\\nACCEPT";
            }
            os << " (rep=" << as_string_list(g[v].reports) << ")";
        }

        if (g[v].suffix) {
            os << "\\nSUFFIX (TOP " << g[v].suffix.top;
            // Can't dump the queue number, but we can identify the suffix.
            if (g[v].suffix.graph) {
                os << ", graph=" << g[v].suffix.graph.get() << " "
                   << to_string(g[v].suffix.graph->kind);
            }
            if (g[v].suffix.castle) {
                os << ", castle=" << g[v].suffix.castle.get();
            }
            if (g[v].suffix.rdfa) {
                os << ", dfa=" << g[v].suffix.rdfa.get();
            }
            if (g[v].suffix.haig) {
                os << ", haig=" << g[v].suffix.haig.get();
            }

            os << ")";
        }

        if (ghost.find(v) != ghost.end()) {
            os << "\\nGHOST";
        }

        if (g[v].left) {
            const char *roseKind =
                build.isRootSuccessor(v) ? "PREFIX" : "INFIX";
            os << "\\nROSE " << roseKind;
            os << " (";
            os << "report " << g[v].left.leftfix_report << ")";

            if (g[v].left.graph) {
                os << " " << to_string(g[v].left.graph->kind);
            }
        }

        os << "\"";

        // Roles with a rose prefix get a colour.
        if (g[v].left) {
            os << " color=violetred ";
        }

        // Our accepts get different colours.
        if (!g[v].reports.empty()) {
            os << " color=blue ";
        }
        if (g[v].suffix) {
            os << " color=forestgreen ";
        }

        os << "]";
    }

    void operator() (ostream &os, const RoseEdge &e) const {
        const RoseGraph &g = build.g;

        // Render the bounds on this edge.
        u32 minBound = g[e].minBound;
        u32 maxBound = g[e].maxBound;

        os << "[label=\"";
        if (minBound == 0 && maxBound == ROSE_BOUND_INF) {
            os << ".*";
        } else if (minBound == 1 && maxBound == ROSE_BOUND_INF) {
            os << ".+";
        } else {
            os << ".{" << minBound << ",";
            if (maxBound != ROSE_BOUND_INF) {
                os << maxBound;
            }
            os << "}";
        }

        // If we lead to an infix, display which top we're using.
        RoseVertex v = target(e, g);
        if (g[v].left) {
            os << "\\nROSE TOP " << g[e].rose_top;
        }

        switch (g[e].history) {
        case ROSE_ROLE_HISTORY_NONE:
            break;
        case ROSE_ROLE_HISTORY_ANCH:
            os << "\\nANCH history";
            break;
        case ROSE_ROLE_HISTORY_LAST_BYTE:
            os << "\\nLAST_BYTE history";
            break;
        case ROSE_ROLE_HISTORY_INVALID:
            os << "\\nINVALID history";
            break;
        }

        os << "\"]";
    }

private:
    // Render the literal associated with a vertex.
    void writeLiteral(ostream &os, u32 id) const {
        os << "lit=" << id;
        if (id < build.literal_info.size()) {
            os << "/" << build.literal_info[id].final_id << " ";
        } else {
            os << "/nofinal ";
        }

        if (contains(build.literals.right, id)) {
            const auto &lit = build.literals.right.at(id);
            os << '\'' << dotEscapeString(lit.s.get_string()) << '\'';
            if (lit.s.any_nocase()) {
                os << " (nocase)";
            }
            if (lit.delay) {
                os << " +" << lit.delay;
            }
        } else {
            os << "<unknown>";
        }
    }

    set<RoseVertex> ghost;
    const RoseBuildImpl &build;
    const RoseEngine *t;
};

} // namespace

void dumpRoseGraph(const RoseBuild &build_base, const RoseEngine *t,
                   const char *filename) {
    const RoseBuildImpl &build = dynamic_cast<const RoseBuildImpl &>(build_base);

    const Grey &grey = build.cc.grey;
    if (!grey.dumpFlags) {
        return;
    }

    stringstream ss;
    ss << grey.dumpPath << filename;


    DEBUG_PRINTF("dumping graph to %s\n", ss.str().c_str());
    ofstream os(ss.str());

    RoseGraphWriter writer(build, t);
    writeGraphviz(os, build.g, writer, get(&RoseVertexProps::idx, build.g));
}

namespace {
struct CompareVertexRole {
    explicit CompareVertexRole(const RoseGraph &g_in) : g(g_in) {}
    inline bool operator()(const RoseVertex &a, const RoseVertex &b) const {
        return g[a].idx < g[b].idx;
    }
private:
    const RoseGraph &g;
};
}

static
void lit_graph_info(const RoseBuildImpl &build, const rose_literal_info &li,
                    u32 *min_offset, bool *in_root_role) {
    *min_offset = ~0U;
    *in_root_role = false;
    for (auto v : li.vertices) {
        *in_root_role |= build.isRootSuccessor(v);

        LIMIT_TO_AT_MOST(min_offset, build.g[v].min_offset);
    }
}

static
void dumpRoseLiterals(const RoseBuildImpl &build, const char *filename) {
    const RoseGraph &g = build.g;

    DEBUG_PRINTF("dumping literals\n");
    ofstream os(filename);

    os << "ROSE LITERALS: a total of " << build.literals.right.size()
       << " literals and " << num_vertices(g) << " roles." << endl << endl;

    for (const auto &e : build.literals.right) {
        u32 id = e.first;
        const ue2_literal &s = e.second.s;
        const rose_literal_info &lit_info = build.literal_info[id];

        switch (e.second.table) {
        case ROSE_ANCHORED:
            os << "ANCHORED";
            break;
        case ROSE_FLOATING:
            os << "FLOATING";
            break;
        case ROSE_EOD_ANCHORED:
            os << "EOD-ANCHORED";
            break;
        case ROSE_ANCHORED_SMALL_BLOCK:
            os << "SMALL-BLOCK";
            break;
        case ROSE_EVENT:
            os << "EVENT";
            break;
        }

        os << " ID " << id << "/" << lit_info.final_id << ": \""
           << escapeString(s.get_string()) << "\""
           << " (len " << s.length() << ",";
        if (s.any_nocase()) {
            os << " nocase,";
        }
        if (lit_info.requires_benefits) {
            os << " benefits,";
        }

        if (e.second.delay) {
            os << " delayed "<< e.second.delay << ",";
        }

        os << " groups 0x" << hex << setw(16) << setfill('0')
           << lit_info.group_mask << dec << ",";

        if (lit_info.squash_group) {
            os << " squashes group,";
        }

        u32 min_offset;
        bool in_root_role;
        lit_graph_info(build, lit_info, &min_offset, &in_root_role);
        os << " min offset " << min_offset;
        if (in_root_role) {
            os << " root literal";
        }

        os << ") roles=" << lit_info.vertices.size() << endl;

        if (!lit_info.delayed_ids.empty()) {
            os << "  Children:";
            for (u32 d_id : lit_info.delayed_ids) {
                os << " " << d_id;
            }
            os << endl;
        }

        // Temporary vector, so that we can sort the output by role.
        vector<RoseVertex> verts(lit_info.vertices.begin(),
                                 lit_info.vertices.end());
        sort(verts.begin(), verts.end(), CompareVertexRole(g));

        for (RoseVertex v : verts) {
            // role info
            os << "  Index " << g[v].idx << ": groups=0x" << hex << setw(16)
               << setfill('0') << g[v].groups << dec;

            if (g[v].reports.empty()) {
                os << ", report=NONE";
            } else {
                os << ", report={" << as_string_list(g[v].reports) << "}";
            }

            os << ", min_offset=" << g[v].min_offset;
            os << ", max_offset=" << g[v].max_offset << endl;
            // pred info
            for (const auto &ie : in_edges_range(v, g)) {
                const auto &u = source(ie, g);
                os << "    Predecessor idx=";
                if (u == build.root) {
                    os << "ROOT";
                } else if (u == build.anchored_root) {
                    os << "ANCHORED_ROOT";
                } else {
                    os << g[u].idx;
                }
                os << ": bounds [" << g[ie].minBound << ", ";
                if (g[ie].maxBound == ROSE_BOUND_INF) {
                    os << "inf";
                } else {
                    os << g[ie].maxBound;
                }
                os << "]" << endl;
            }
        }
    }

    os.close();
}

template<class Iter>
static
string toHex(Iter i, const Iter &end) {
    ostringstream oss;
    for (; i != end; ++i) {
        u8 c = *i;
        oss << hex << setw(2) << setfill('0') << ((unsigned)c & 0xff);
    }
    return oss.str();
}

static
void dumpTestLiterals(const string &filename, const vector<hwlmLiteral> &lits) {
    ofstream of(filename.c_str());

    for (const hwlmLiteral &lit : lits) {
        of << lit.id << "=";
        if (lit.nocase) {
            of << "!";
        }
        of << toHex(lit.s.begin(), lit.s.end());
        if (!lit.msk.empty()) {
            of << " " << toHex(lit.msk.begin(), lit.msk.end());
            of << " " << toHex(lit.cmp.begin(), lit.cmp.end());
        }

        of << endl;
    }

    of.close();
}

namespace {
struct LongerThanLimit {
    explicit LongerThanLimit(size_t len) : max_len(len) {}
    bool operator()(const hwlmLiteral &lit) const {
        return lit.s.length() > max_len;
    }

  private:
    size_t max_len;
};
}

static
void dumpRoseTestLiterals(const RoseBuildImpl &build, const string &base) {
    auto lits = fillHamsterLiteralList(build, ROSE_ANCHORED);
    dumpTestLiterals(base + "rose_anchored_test_literals.txt", lits);

    lits = fillHamsterLiteralList(build, ROSE_FLOATING);
    dumpTestLiterals(base + "rose_float_test_literals.txt", lits);

    lits = fillHamsterLiteralList(build, ROSE_EOD_ANCHORED);
    dumpTestLiterals(base + "rose_eod_test_literals.txt", lits);

    lits = fillHamsterLiteralList(build, ROSE_FLOATING);
    auto lits2 = fillHamsterLiteralList(build, ROSE_ANCHORED_SMALL_BLOCK);
    lits.insert(end(lits), begin(lits2), end(lits2));
    lits.erase(remove_if(lits.begin(), lits.end(),
                         LongerThanLimit(ROSE_SMALL_BLOCK_LEN)),
               lits.end());
    dumpTestLiterals(base + "rose_smallblock_test_literals.txt", lits);
}

void dumpRose(const RoseBuild &build_base, const RoseEngine *t,
              const Grey &grey) {
    if (!grey.dumpFlags) {
        return;
    }

    const RoseBuildImpl &build = dynamic_cast<const RoseBuildImpl&>(build_base);

    stringstream ss;
    ss << grey.dumpPath << "rose.txt";

    FILE *f = fopen(ss.str().c_str(), "w");

    if (!t) {
        fprintf(f, "<< no rose >>\n");
        fclose(f);
        return;
    }

    // Dump Rose table info
    roseDumpText(t, f);

    fclose(f);

    roseDumpComponents(t, false, grey.dumpPath);

    // Graph.
    dumpRoseGraph(build, t, "rose.dot");

    // Literals.
    ss.str("");
    ss.clear();
    ss << grey.dumpPath << "rose_literals.txt";
    dumpRoseLiterals(build, ss.str().c_str());
    dumpRoseTestLiterals(build, grey.dumpPath);

    f = fopen((grey.dumpPath + "/rose_struct.txt").c_str(), "w");
    roseDumpStructRaw(t, f);
    fclose(f);
}

} // namespace ue2
