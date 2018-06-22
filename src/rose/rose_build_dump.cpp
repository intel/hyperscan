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

#include "config.h"

#include "rose_build_dump.h"

#include "rose_build_impl.h"
#include "rose_build_matchers.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "ue2common.h"
#include "hs_compile.h"
#include "hwlm/hwlm_build.h"
#include "hwlm/hwlm_dump.h"
#include "hwlm/hwlm_literal.h"
#include "nfa/castlecompile.h"
#include "nfa/nfa_build_util.h"
#include "nfa/nfa_dump_api.h"
#include "nfa/nfa_internal.h"
#include "nfagraph/ng_dump.h"
#include "som/slot_manager_dump.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/dump_charclass.h"
#include "util/dump_util.h"
#include "util/graph_range.h"
#include "util/multibit.h"
#include "util/multibit_build.h"
#include "util/ue2string.h"

#include <iomanip>
#include <numeric>
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

/** \brief Return the kind of a left_id or a suffix_id. */
template<class Graph>
string render_kind(const Graph &g) {
    if (g.graph()) {
        return to_string(g.graph()->kind);
    }
    if (g.dfa()) {
        return to_string(g.dfa()->kind);
    }
    if (g.haig()) {
        return to_string(g.haig()->kind);
    }
    if (g.castle()) {
        return to_string(g.castle()->kind);
    }
    return "UNKNOWN";
}

namespace {

struct rose_off {
    explicit rose_off(u32 j) : i(j) {}
    string str(void) const;
    u32 i;
};

ostream &operator<<(ostream &o, const rose_off &to) {
    if (to.i == ROSE_BOUND_INF) {
        o << "inf";
    } else {
        o << to.i;
    }
    return o;
}

string rose_off::str(void) const {
    ostringstream out;
    out << *this;
    return out.str();
}

class RoseGraphWriter {
public:
    RoseGraphWriter(const RoseBuildImpl &b_in, const map<u32, u32> &frag_map_in,
                    const map<left_id, u32> &lqm_in,
                    const map<suffix_id, u32> &sqm_in, const RoseEngine *t_in)
        : frag_map(frag_map_in), leftfix_queue_map(lqm_in),
          suffix_queue_map(sqm_in), build(b_in), t(t_in) {
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
        os << "index=" << g[v].index <<"\\n";

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
            suffix_id suff(g[v].suffix);
            os << "\\n" << render_kind(suff) << " (top " << g[v].suffix.top;
            auto it = suffix_queue_map.find(suff);
            if (it != end(suffix_queue_map)) {
                os << ", queue " << it->second;
            }
            os << ")";
        }

        if (ghost.find(v) != ghost.end()) {
            os << "\\nGHOST";
        }

        if (g[v].left) {
            left_id left(g[v].left);
            os << "\\n" << render_kind(left) << " (queue ";
            auto it = leftfix_queue_map.find(left);
            if (it != end(leftfix_queue_map)) {
                os << it->second;
            } else {
                os << "??";
            }
            os << ", report " << g[v].left.leftfix_report << ")";
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
        if (contains(frag_map, id)) {
            os << "/" << frag_map.at(id) << " ";
        } else {
            os << "/nofrag ";
        }

        const auto &lit = build.literals.at(id);
        os << '\'' << dotEscapeString(lit.s.get_string()) << '\'';
        if (lit.s.any_nocase()) {
            os << " (nocase)";
        }
        if (lit.delay) {
            os << " +" << lit.delay;
        }
    }

    set<RoseVertex> ghost;
    const map<u32, u32> &frag_map;
    const map<left_id, u32> &leftfix_queue_map;
    const map<suffix_id, u32> &suffix_queue_map;
    const RoseBuildImpl &build;
    const RoseEngine *t;
};

} // namespace

static
map<u32, u32> makeFragMap(const vector<LitFragment> &fragments) {
    map<u32, u32> fm;
    for (const auto &f : fragments) {
        for (u32 id : f.lit_ids) {
            fm[id] = f.fragment_id;
        }
    }

    return fm;
}

static
void dumpRoseGraph(const RoseBuildImpl &build, const RoseEngine *t,
                   const vector<LitFragment> &fragments,
                   const map<left_id, u32> &leftfix_queue_map,
                   const map<suffix_id, u32> &suffix_queue_map,
                   const char *filename) {
    const Grey &grey = build.cc.grey;

    /* "early" rose graphs should only be dumped if we are dumping intermediate
     * graphs. Early graphs can be identified by the lack of a RoseEngine. */
    u32 flag_test = t ? Grey::DUMP_IMPL : Grey::DUMP_INT_GRAPH;

    if (!(grey.dumpFlags & flag_test)) {
        return;
    }

    stringstream ss;
    ss << grey.dumpPath << filename;

    DEBUG_PRINTF("dumping graph to %s\n", ss.str().c_str());
    ofstream os(ss.str());

    auto frag_map = makeFragMap(fragments);
    RoseGraphWriter writer(build, frag_map, leftfix_queue_map, suffix_queue_map,
                           t);
    writeGraphviz(os, build.g, writer, get(boost::vertex_index, build.g));
}

void dumpRoseGraph(const RoseBuildImpl &build, const char *filename) {
    dumpRoseGraph(build, nullptr, {}, {}, {}, filename);
}

namespace {
struct CompareVertexRole {
    explicit CompareVertexRole(const RoseGraph &g_in) : g(g_in) {}
    inline bool operator()(const RoseVertex &a, const RoseVertex &b) const {
        return g[a].index < g[b].index;
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
void dumpRoseLiterals(const RoseBuildImpl &build,
                      const vector<LitFragment> &fragments,
                      const Grey &grey) {
    const RoseGraph &g = build.g;
    map<u32, u32> frag_map = makeFragMap(fragments);

    DEBUG_PRINTF("dumping literals\n");
    ofstream os(grey.dumpPath + "rose_literals.txt");

    os << "ROSE LITERALS: a total of " << build.literals.size()
       << " literals and " << num_vertices(g) << " roles." << endl
       << endl;

    for (u32 id = 0; id < build.literals.size(); id++) {
        const auto &lit = build.literals.at(id);
        const ue2_literal &s = lit.s;
        const rose_literal_info &lit_info = build.literal_info[id];

        switch (lit.table) {
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

        os << " ID " << id;
        if (contains(frag_map, id)) {
            os << "/" << frag_map.at(id);
        }
        os << ": \"" << escapeString(s.get_string()) << "\""
           << " (len " << s.length() << ",";
        if (s.any_nocase()) {
            os << " nocase,";
        }
        if (lit_info.requires_benefits) {
            os << " benefits,";
        }

        if (lit.delay) {
            os << " delayed "<< lit.delay << ",";
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
            os << "  Index " << g[v].index << ": groups=0x" << hex << setw(16)
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
                os << "    Predecessor index=";
                if (u == build.root) {
                    os << "ROOT";
                } else if (u == build.anchored_root) {
                    os << "ANCHORED_ROOT";
                } else {
                    os << g[u].index;
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
bool isMetaChar(char c) {
    switch (c) {
    case '#':
    case '$':
    case '(':
    case ')':
    case '*':
    case '+':
    case '.':
    case '/':
    case '?':
    case '[':
    case '\\':
    case ']':
    case '^':
    case '{':
    case '|':
    case '}':
        return true;
    default:
        return false;
    }
}

static
string toRegex(const string &lit) {
    ostringstream os;
    for (char c : lit) {
        if (0x20 <= c && c <= 0x7e) {
            if (isMetaChar(c)) {
                os << "\\" << c;
            } else {
                os << c;
            }
        } else if (c == '\n') {
            os << "\\n";
        } else if (c == '\r') {
            os << "\\r";
        } else if (c == '\t') {
            os << "\\t";
        } else {
            os << "\\x" << hex << setw(2) << setfill('0')
               << (unsigned)(c & 0xff) << dec;
        }
    }
    return os.str();
}

void dumpMatcherLiterals(const vector<hwlmLiteral> &lits, const string &name,
                         const Grey &grey) {
    if (!grey.dumpFlags) {
        return;
    }

    ofstream of(grey.dumpPath + "rose_" + name + "_test_literals.txt");

    // Unique regex index, as literals may share an ID.
    u32 i = 0;

    for (const hwlmLiteral &lit : lits) {
        // First, detail in a comment.
        of << "# id=" << lit.id;
        if (!lit.msk.empty()) {
            of << " msk=0x" << toHex(lit.msk.begin(), lit.msk.end());
            of << " cmp=0x" << toHex(lit.cmp.begin(), lit.cmp.end());
        }
        of << " groups=0x" << hex << setfill('0') << lit.groups << dec;
        if (lit.noruns) {
            of << " noruns";
        }
        of << endl;

        // Second, literal rendered as a regex.
        of << i << ":/" << toRegex(lit.s) << (lit.nocase ? "/i" : "/");

        of << endl;

        i++;
    }

    of.close();
}

static
const void *loadFromByteCodeOffset(const RoseEngine *t, u32 offset) {
    if (!offset) {
        return nullptr;
    }

    const char *lt = (const char *)t + offset;
    return lt;
}

static
const void *getAnchoredMatcher(const RoseEngine *t) {
    return loadFromByteCodeOffset(t, t->amatcherOffset);
}

static
const HWLM *getFloatingMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->fmatcherOffset);
}

static
const HWLM *getDelayRebuildMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->drmatcherOffset);
}

static
const HWLM *getEodMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->ematcherOffset);
}

static
const HWLM *getSmallBlockMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->sbmatcherOffset);
}

static
CharReach bitvectorToReach(const u8 *reach) {
    CharReach cr;

    for (size_t i = 0; i < N_CHARS; i++) {
        if (reach[i / 8] & (1U << (i % 8))) {
            cr.set(i);
        }
    }
    return cr;
}

static
CharReach multiBitvectorToReach(const u8 *reach, u8 path_mask) {
    CharReach cr;
    for (size_t i = 0; i < N_CHARS; i++) {
        if (reach[i] & path_mask) {
            cr.set(i);
        }
    }
    return cr;
}

static
void dumpLookaround(ofstream &os, const RoseEngine *t,
                    const ROSE_STRUCT_CHECK_LOOKAROUND *ri) {
    assert(ri);

    const u8 *base = (const u8 *)t;

    const s8 *look = (const s8 *)base + ri->look_index;
    const s8 *look_end = look + ri->count;
    const u8 *reach = base + ri->reach_index;

    os << "    contents:" << endl;

    for (; look < look_end; look++, reach += REACH_BITVECTOR_LEN) {
        os << "      " << std::setw(4) << std::setfill(' ') << int{*look}
           << ": ";
        describeClass(os, bitvectorToReach(reach), 1000, CC_OUT_TEXT);
        os << endl;
    }
}

static
void dumpMultipathLookaround(ofstream &os, const RoseEngine *t,
                             const ROSE_STRUCT_MULTIPATH_LOOKAROUND *ri) {
    assert(ri);

    const u8 *base = (const u8 *)t;

    const s8 *look_begin = (const s8 *)base + ri->look_index;
    const s8 *look_end = look_begin + ri->count;
    const u8 *reach_begin = base + ri->reach_index;

    os << "    contents:" << endl;

    u32 path_mask = ri->start_mask[0];
    while (path_mask) {
        u32 path = findAndClearLSB_32(&path_mask);
        os << "    Path #" << path << ":" << endl;
        os << "      ";

        const s8 *look = look_begin;
        const u8 *reach = reach_begin;
        for (; look < look_end; look++, reach += MULTI_REACH_BITVECTOR_LEN) {
            CharReach cr = multiBitvectorToReach(reach, 1U << path);
            if (cr.any() && !cr.all()) {
                os << "<" << int(*look) << ": ";
                describeClass(os, cr, 1000, CC_OUT_TEXT);
                os << "> ";
            }
        }
        os << endl;
    }
}

static
vector<u32> sparseIterValues(const mmbit_sparse_iter *it, u32 num_bits) {
    vector<u32> keys;

    if (num_bits == 0) {
        return keys;
    }

    // Populate a multibit structure with all-ones. Note that the multibit
    // runtime assumes that it is always safe to read 8 bytes, so we must
    // over-allocate for smaller sizes.
    const size_t num_bytes = mmbit_size(num_bits);
    vector<u8> bits(max(size_t{8}, num_bytes), u8{0xff}); // All bits on.
    const u8 *b = bits.data();
    if (num_bytes < 8) {
        b += 8 - num_bytes;
    }

    vector<mmbit_sparse_state> state(MAX_SPARSE_ITER_STATES);
    mmbit_sparse_state *s = state.data();

    u32 idx = 0;
    u32 i = mmbit_sparse_iter_begin(b, num_bits, &idx, it, s);
    while (i != MMB_INVALID) {
        keys.push_back(i);
        i = mmbit_sparse_iter_next(b, num_bits, i, &idx, it, s);
    }

    return keys;
}

static
void dumpJumpTable(ofstream &os, const RoseEngine *t,
                   const ROSE_STRUCT_SPARSE_ITER_BEGIN *ri) {
    auto *it =
        (const mmbit_sparse_iter *)loadFromByteCodeOffset(t, ri->iter_offset);
    auto *jumps = (const u32 *)loadFromByteCodeOffset(t, ri->jump_table);

    for (const auto &key : sparseIterValues(it, t->rolesWithStateCount)) {
        os << "      " << std::setw(4) << std::setfill(' ') << key << " : +"
           << *jumps << endl;
        ++jumps;
    }
}

static
void dumpSomOperation(ofstream &os, const som_operation &op) {
    os << "    som (type=" << u32{op.type} << ", onmatch=" << op.onmatch;
    switch (op.type) {
    case SOM_EXTERNAL_CALLBACK_REV_NFA:
    case SOM_INTERNAL_LOC_SET_REV_NFA:
    case SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET:
    case SOM_INTERNAL_LOC_SET_REV_NFA_IF_WRITABLE:
        os << ", revNfaIndex=" << op.aux.revNfaIndex;
        break;
    default:
        os << ", somDistance=" << op.aux.somDistance;
        break;
    }
    os << ")" << endl;
}

static
string dumpStrMask(const u8 *mask, size_t len) {
    ostringstream oss;
    for (size_t i = 0; i < len; i++) {
        oss << std::hex << std::setw(2) << std::setfill('0') << u32{mask[i]}
            << " ";
    }
    return oss.str();
}

static
CharReach shufti2cr(const u8 *lo, const u8 *hi, u8 bucket_mask) {
    CharReach cr;
    for (u32 i = 0; i < N_CHARS; i++) {
        if(lo[i & 0xf] & hi[i >> 4] & bucket_mask) {
            cr.set(i);
        }
    }
    return cr;
}

static
void dumpLookaroundShufti(ofstream &os, u32 len, const u8 *lo, const u8 *hi,
                          const u8 *bucket_mask, u32 neg_mask, s32 offset) {
    assert(len == 16 || len == 32);
    os << "    contents:" << endl;
    for (u32 idx = 0; idx < len; idx++) {
        CharReach cr = shufti2cr(lo, hi, bucket_mask[idx]);

        if (neg_mask & (1U << idx)) {
            cr.flip();
        }

        if (cr.any() && !cr.all()) {
            os << "      " << std::setw(4) << std::setfill(' ')
               << int(offset + idx) << ": ";
            describeClass(os, cr, 1000, CC_OUT_TEXT);
            os << endl;
        }
    }
}

static
void dumpLookaroundShufti(ofstream &os, u32 len, const u8 *lo, const u8 *hi,
                          const u8 *lo_2, const u8 *hi_2, const u8 *bucket_mask,
                          const u8 *bucket_mask_2, u32 neg_mask, s32 offset) {
    assert(len == 16 || len == 32);
    os << "    contents:" << endl;
    for (u32 idx = 0; idx < len; idx++) {
        CharReach cr = shufti2cr(lo, hi, bucket_mask[idx]);
        cr |= shufti2cr(lo_2, hi_2, bucket_mask_2[idx]);

        if (neg_mask & (1U << idx)) {
            cr.flip();
        }

        if (cr.any() && !cr.all()) {
            os << "      " << std::setw(4) << std::setfill(' ')
               << int(offset + idx) << ": ";
            describeClass(os, cr, 1000, CC_OUT_TEXT);
            os << endl;
        }
    }
}

static
void dumpMultipathShufti(ofstream &os, u32 len, const u8 *lo, const u8 *hi,
                         const u8 *bucket_mask, const u8 *data_offset,
                         u64a neg_mask, s32 base_offset) {
    assert(len == 16 || len == 32 || len == 64);
    os << "    contents:" << endl;
    u32 path = 0;
    for (u32 idx = 0; idx < len; idx++) {
        CharReach cr = shufti2cr(lo, hi, bucket_mask[idx]);

        if (neg_mask & (1ULL << idx)) {
            cr.flip();
        }

        if (cr.any() && !cr.all()) {
            if (idx == 0 || data_offset[idx - 1] > data_offset[idx]) {
                path++;
                if (idx) {
                    os << endl;
                }
                os << "    Path #" << path << ":" << endl;
                os << "      ";
            }

            os << "<" << int(base_offset + data_offset[idx]) << ": ";
            describeClass(os, cr, 1000, CC_OUT_TEXT);
            os << "> ";
        }
    }
    os << endl;
}

static
void dumpMultipathShufti(ofstream &os, u32 len, const u8 *lo, const u8 *hi,
                         const u8 *lo_2, const u8 *hi_2, const u8 *bucket_mask,
                         const u8 *bucket_mask_2, const u8 *data_offset,
                         u32 neg_mask, s32 base_offset) {
    assert(len == 16 || len == 32 || len == 64);
    os << "    contents:";
    u32 path = 0;
    for (u32 idx = 0; idx < len; idx++) {
        CharReach cr = shufti2cr(lo, hi, bucket_mask[idx]);
        cr |= shufti2cr(lo_2, hi_2, bucket_mask_2[idx]);

        if (neg_mask & (1ULL << idx)) {
            cr.flip();
        }

        if (cr.any() && !cr.all()) {
            if (idx == 0 || data_offset[idx - 1] > data_offset[idx]) {
                path++;
                os << endl;
                os << "    Path #" << path << ":" << endl;
                os << "      ";
            }

            os << "<" << int(base_offset + data_offset[idx]) << ": ";
            describeClass(os, cr, 1000, CC_OUT_TEXT);
            os << "> ";
        }
    }
    os << endl;
}

           #define PROGRAM_CASE(name)                                                     \
    case ROSE_INSTR_##name: {                                                  \
        os << "  " << std::setw(4) << std::setfill('0') << (pc - pc_base)      \
           << ": " #name "\n";                                                 \
        const auto *ri = (const struct ROSE_STRUCT_##name *)pc;

#define PROGRAM_NEXT_INSTRUCTION                                               \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    break;                                                                     \
    }


static
void dumpProgram(ofstream &os, const RoseEngine *t, const char *pc) {
    const char *pc_base = pc;
    for (;;) {
        u8 code = *(const u8 *)pc;
        assert(code <= LAST_ROSE_INSTRUCTION);
        const size_t offset = pc - pc_base;
        switch (code) {
            PROGRAM_CASE(END) { return; }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(ANCHORED_DELAY) {
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
                os << "    anch_id " << ri->anch_id << "\n";
                os << "    done_jump " << offset + ri->done_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LIT_EARLY) {
                os << "    min_offset " << ri->min_offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_GROUPS) {
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_ONLY_EOD) {
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BOUNDS) {
                os << "    min_bound " << ri->min_bound << endl;
                os << "    max_bound " << ri->max_bound << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_NOT_HANDLED) {
                os << "    key " << ri->key << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SINGLE_LOOKAROUND) {
                os << "    offset " << int{ri->offset} << endl;
                os << "    reach_index " << ri->reach_index << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                const u8 *reach = (const u8 *)t + ri->reach_index;
                os << "    contents ";
                describeClass(os, bitvectorToReach(reach), 1000, CC_OUT_TEXT);
                os << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LOOKAROUND) {
                os << "    look_index " << ri->look_index << endl;
                os << "    reach_index " << ri->reach_index << endl;
                os << "    count " << ri->count << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpLookaround(os, t, ri);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MASK) {
                os << "    and_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->and_mask << std::dec << endl;
                os << "    cmp_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->cmp_mask << std::dec << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MASK_32) {
                os << "    and_mask "
                   << dumpStrMask(ri->and_mask, sizeof(ri->and_mask))
                   << endl;
                os << "    cmp_mask "
                   << dumpStrMask(ri->cmp_mask, sizeof(ri->cmp_mask))
                   << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BYTE) {
                os << "    and_mask 0x" << std::hex << std::setw(2)
                   << std::setfill('0') << u32{ri->and_mask} << std::dec
                   << endl;
                os << "    cmp_mask 0x" << std::hex << std::setw(2)
                   << std::setfill('0') << u32{ri->cmp_mask} << std::dec
                   << endl;
                os << "    negation " << u32{ri->negation} << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_16x8) {
                os << "    nib_mask "
                   << dumpStrMask(ri->nib_mask, sizeof(ri->nib_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpLookaroundShufti(os, 16, ri->nib_mask, ri->nib_mask + 16,
                                     ri->bucket_select_mask, ri->neg_mask,
                                     ri->offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_32x8) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpLookaroundShufti(os, 32, ri->lo_mask, ri->hi_mask,
                                     ri->bucket_select_mask, ri->neg_mask,
                                     ri->offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_16x16) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpLookaroundShufti(os, 16, ri->lo_mask, ri->hi_mask,
                                     ri->lo_mask + 16, ri->hi_mask + 16,
                                     ri->bucket_select_mask,
                                     ri->bucket_select_mask + 16,
                                     ri->neg_mask, ri->offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_SHUFTI_32x16) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask_hi "
                   << dumpStrMask(ri->bucket_select_mask_hi,
                                  sizeof(ri->bucket_select_mask_hi))
                   << endl;
                os << "    bucket_select_mask_lo "
                   << dumpStrMask(ri->bucket_select_mask_lo,
                                  sizeof(ri->bucket_select_mask_lo))
                   << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    offset " << ri->offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpLookaroundShufti(os, 32, ri->lo_mask, ri->hi_mask,
                                     ri->lo_mask + 16, ri->hi_mask + 16,
                                     ri->bucket_select_mask_lo,
                                     ri->bucket_select_mask_hi,
                                     ri->neg_mask, ri->offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_INFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    lag " << ri->lag << endl;
                os << "    report " << ri->report << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_PREFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    lag " << ri->lag << endl;
                os << "    report " << ri->report << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(PUSH_DELAYED) {
                os << "    delay " << u32{ri->delay} << endl;
                os << "    index " << ri->index << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DUMMY_NOP) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CATCH_UP) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CATCH_UP_MPV) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_ADJUST) {
                os << "    distance " << ri->distance << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_LEFTFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    lag " << ri->lag << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_FROM_REPORT) {
                dumpSomOperation(os, ri->som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_ZERO) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_INFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    event " << ri->event << endl;
                os << "    cancel " << u32{ri->cancel} << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_SUFFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    event " << ri->event << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE) {
                os << "    quash_som " << u32{ri->quash_som} << endl;
                os << "    dkey " << ri->dkey << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE_SOM) {
                os << "    quash_som " << u32{ri->quash_som} << endl;
                os << "    dkey " << ri->dkey << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_CHAIN) {
                os << "    event " << ri->event << endl;
                os << "    top_squash_distance " << ri->top_squash_distance
                   << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_INT) {
                dumpSomOperation(os, ri->som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_AWARE) {
                dumpSomOperation(os, ri->som);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT) {
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_EXHAUST) {
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
                os << "    ekey " << ri->ekey << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM) {
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_EXHAUST) {
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
                os << "    ekey " << ri->ekey << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(DEDUPE_AND_REPORT) {
                os << "    quash_som " << u32{ri->quash_som} << endl;
                os << "    dkey " << ri->dkey << endl;
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(FINAL_REPORT) {
                os << "    onmatch " << ri->onmatch << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_EXHAUSTED) {
                os << "    ekey " << ri->ekey << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MIN_LENGTH) {
                os << "    end_adj " << ri->end_adj << endl;
                os << "    min_length " << ri->min_length << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_STATE) {
                os << "    index " << ri->index << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_GROUPS) {
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SQUASH_GROUPS) {
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_STATE) {
                os << "    index " << ri->index << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_BEGIN) {
                os << "    iter_offset " << ri->iter_offset << endl;
                os << "    jump_table " << ri->jump_table << endl;
                dumpJumpTable(os, t, ri);
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_NEXT) {
                os << "    iter_offset " << ri->iter_offset << endl;
                os << "    jump_table " << ri->jump_table << endl;
                os << "    state " << ri->state << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_ANY) {
                os << "    iter_offset " << ri->iter_offset << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(ENGINES_EOD) {
                os << "    iter_offset " << ri->iter_offset << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SUFFIXES_EOD) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(MATCHER_EOD) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LONG_LIT) {
                os << "    lit_offset " << ri->lit_offset << endl;
                os << "    lit_length " << ri->lit_length << endl;
                const char *lit = (const char *)t + ri->lit_offset;
                os << "    literal: \""
                   << escapeString(string(lit, ri->lit_length)) << "\"" << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LONG_LIT_NOCASE) {
                os << "    lit_offset " << ri->lit_offset << endl;
                os << "    lit_length " << ri->lit_length << endl;
                const char *lit = (const char *)t + ri->lit_offset;
                os << "    literal: \""
                   << escapeString(string(lit, ri->lit_length)) << "\"" << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MED_LIT) {
                os << "    lit_offset " << ri->lit_offset << endl;
                os << "    lit_length " << ri->lit_length << endl;
                const char *lit = (const char *)t + ri->lit_offset;
                os << "    literal: \""
                   << escapeString(string(lit, ri->lit_length)) << "\"" << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MED_LIT_NOCASE) {
                os << "    lit_offset " << ri->lit_offset << endl;
                os << "    lit_length " << ri->lit_length << endl;
                const char *lit = (const char *)t + ri->lit_offset;
                os << "    literal: \""
                   << escapeString(string(lit, ri->lit_length)) << "\"" << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CLEAR_WORK_DONE) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(MULTIPATH_LOOKAROUND) {
                os << "    look_index " << ri->look_index << endl;
                os << "    reach_index " << ri->reach_index << endl;
                os << "    count " << ri->count << endl;
                os << "    last_start " << ri->last_start << endl;
                os << "    start_mask "
                   << dumpStrMask(ri->start_mask, sizeof(ri->start_mask))
                   << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpMultipathLookaround(os, t, ri);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_16x8) {
                os << "    nib_mask "
                   << dumpStrMask(ri->nib_mask, sizeof(ri->nib_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    data_select_mask "
                   << dumpStrMask(ri->data_select_mask,
                                  sizeof(ri->data_select_mask))
                   << endl;
                os << "    hi_bits_mask 0x" << std::hex << std::setw(4)
                   << std::setfill('0') << ri->hi_bits_mask << std::dec << endl;
                os << "    lo_bits_mask 0x" << std::hex << std::setw(4)
                   << std::setfill('0') << ri->lo_bits_mask << std::dec << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(4)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    base_offset " << ri->base_offset << endl;
                os << "    last_start " << ri->last_start << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpMultipathShufti(os, 16, ri->nib_mask, ri->nib_mask + 16,
                                    ri->bucket_select_mask,
                                    ri->data_select_mask,
                                    ri->neg_mask, ri->base_offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_32x8) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    data_select_mask "
                   << dumpStrMask(ri->data_select_mask,
                                  sizeof(ri->data_select_mask))
                   << endl;
                os << "    hi_bits_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->hi_bits_mask << std::dec << endl;
                os << "    lo_bits_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->lo_bits_mask << std::dec << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    base_offset " << ri->base_offset << endl;
                os << "    last_start " << ri->last_start << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpMultipathShufti(os, 32, ri->lo_mask, ri->hi_mask,
                                    ri->bucket_select_mask,
                                    ri->data_select_mask,
                                    ri->neg_mask, ri->base_offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_32x16) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask_hi "
                   << dumpStrMask(ri->bucket_select_mask_hi,
                                  sizeof(ri->bucket_select_mask_hi))
                   << endl;
                os << "    bucket_select_mask_lo "
                   << dumpStrMask(ri->bucket_select_mask_lo,
                                  sizeof(ri->bucket_select_mask_lo))
                   << endl;
                os << "    data_select_mask "
                   << dumpStrMask(ri->data_select_mask,
                                  sizeof(ri->data_select_mask))
                   << endl;
                os << "    hi_bits_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->hi_bits_mask << std::dec << endl;
                os << "    lo_bits_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->lo_bits_mask << std::dec << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(8)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    base_offset " << ri->base_offset << endl;
                os << "    last_start " << ri->last_start << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpMultipathShufti(os, 32, ri->lo_mask, ri->hi_mask,
                                    ri->lo_mask + 16, ri->hi_mask + 16,
                                    ri->bucket_select_mask_lo,
                                    ri->bucket_select_mask_hi,
                                    ri->data_select_mask,
                                    ri->neg_mask, ri->base_offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_MULTIPATH_SHUFTI_64) {
                os << "    hi_mask "
                   << dumpStrMask(ri->hi_mask, sizeof(ri->hi_mask))
                   << endl;
                os << "    lo_mask "
                   << dumpStrMask(ri->lo_mask, sizeof(ri->lo_mask))
                   << endl;
                os << "    bucket_select_mask "
                   << dumpStrMask(ri->bucket_select_mask,
                                  sizeof(ri->bucket_select_mask))
                   << endl;
                os << "    data_select_mask "
                   << dumpStrMask(ri->data_select_mask,
                                  sizeof(ri->data_select_mask))
                   << endl;
                os << "    hi_bits_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->hi_bits_mask << std::dec << endl;
                os << "    lo_bits_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->lo_bits_mask << std::dec << endl;
                os << "    neg_mask 0x" << std::hex << std::setw(16)
                   << std::setfill('0') << ri->neg_mask << std::dec << endl;
                os << "    base_offset " << ri->base_offset << endl;
                os << "    last_start " << ri->last_start << endl;
                os << "    fail_jump " << offset + ri->fail_jump << endl;
                dumpMultipathShufti(os, 64, ri->lo_mask, ri->hi_mask,
                                    ri->bucket_select_mask,
                                    ri->data_select_mask,
                                    ri->neg_mask, ri->base_offset);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(INCLUDED_JUMP) {
                os << "    child_offset " << ri->child_offset << endl;
                os << "    squash " << (u32)ri->squash << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_LOGICAL) {
                os << "    lkey " << ri->lkey << endl;
                os << "    offset_adjust " << ri->offset_adjust << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_COMBINATION) {
                os << "    ckey " << ri->ckey << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(FLUSH_COMBINATION) {}
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_EXHAUST) {
                os << "    ekey " << ri->ekey << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

        default:
            os << "  UNKNOWN (code " << int{code} << ")" << endl;
            os << "  <stopping>" << endl;
            return;
        }
    }
}

#undef PROGRAM_CASE
#undef PROGRAM_NEXT_INSTRUCTION

static
void dumpRoseLitPrograms(const vector<LitFragment> &fragments,
                         const RoseEngine *t, const string &filename) {
    ofstream os(filename);

    // Collect all programs referenced by a literal fragment.
    vector<u32> programs;
    for (const auto &frag : fragments) {
        if (frag.lit_program_offset) {
            programs.push_back(frag.lit_program_offset);
        }
        if (frag.delay_program_offset) {
            programs.push_back(frag.delay_program_offset);
        }
    }
    sort_and_unique(programs);

    for (u32 prog_offset : programs) {
        os << "Program @ " << prog_offset << ":" << endl;
        const char *prog = (const char *)loadFromByteCodeOffset(t, prog_offset);
        dumpProgram(os, t, prog);
        os << endl;
    }

    os.close();
}

static
void dumpRoseEodPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);
    const char *base = (const char *)t;

    if (t->eodProgramOffset) {
        os << "EOD Program @ " << t->eodProgramOffset << ":" << endl;
        dumpProgram(os, t, base + t->eodProgramOffset);
        os << endl;
    } else {
        os << "<No EOD Program>" << endl;
    }

    os.close();
}

static
void dumpRoseFlushCombPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);
    const char *base = (const char *)t;

    if (t->flushCombProgramOffset) {
        os << "Flush Combination Program @ " << t->flushCombProgramOffset
           << ":" << endl;
        dumpProgram(os, t, base + t->flushCombProgramOffset);
        os << endl;
    } else {
        os << "<No Flush Combination Program>" << endl;
    }

    os.close();
}

static
void dumpRoseReportPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);

    const u32 *programs =
        (const u32 *)loadFromByteCodeOffset(t, t->reportProgramOffset);

    for (u32 i = 0; i < t->reportProgramCount; i++) {
        os << "Report " << i << endl;
        os << "---------------" << endl;

        if (programs[i]) {
            os << "Program @ " << programs[i] << ":" << endl;
            const char *prog =
                (const char *)loadFromByteCodeOffset(t, programs[i]);
            dumpProgram(os, t, prog);
        } else {
            os << "<No Program>" << endl;
        }
    }

    os.close();
}

static
void dumpRoseAnchoredPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);

    const u32 *programs =
        (const u32 *)loadFromByteCodeOffset(t, t->anchoredProgramOffset);

    for (u32 i = 0; i < t->anchored_count; i++) {
        os << "Anchored entry " << i << endl;
        os << "---------------" << endl;

        if (programs[i]) {
            os << "Program @ " << programs[i] << ":" << endl;
            const char *prog =
                (const char *)loadFromByteCodeOffset(t, programs[i]);
            dumpProgram(os, t, prog);
        } else {
            os << "<No Program>" << endl;
        }
        os << endl;
    }

    os.close();
}

static
void dumpRoseDelayPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);

    const u32 *programs =
        (const u32 *)loadFromByteCodeOffset(t, t->delayProgramOffset);

    for (u32 i = 0; i < t->delay_count; i++) {
        os << "Delay entry " << i << endl;
        os << "---------------" << endl;

        if (programs[i]) {
            os << "Program @ " << programs[i] << ":" << endl;
            const char *prog =
                (const char *)loadFromByteCodeOffset(t, programs[i]);
            dumpProgram(os, t, prog);
        } else {
            os << "<No Program>" << endl;
        }
        os << endl;
    }

    os.close();
}

static
void dumpNfaNotes(ofstream &fout, const RoseEngine *t, const NFA *n) {
    const u32 qindex = n->queueIndex;

    if (qindex < t->outfixBeginQueue) {
        fout << "chained";
        return;
    }

    if (qindex < t->outfixEndQueue) {
        fout << "outfix";
        return;
    }

    const NfaInfo *nfa_info = getNfaInfoByQueue(t, qindex);
    const NFA *nfa = getNfaByInfo(t, nfa_info);

    if (nfa_info->eod) {
        fout << "eod ";
    }

    if (qindex < t->leftfixBeginQueue) {
        fout << "suffix";
        return;
    }

    const LeftNfaInfo *left = getLeftInfoByQueue(t, qindex);
    if (left->eager) {
        fout << "eager ";
    }
    if (left->transient) {
        fout << "transient " << (u32)left->transient << " ";
    }
    if (left->infix) {
        fout << "infix";
        u32 maxQueueLen = left->maxQueueLen;
        if (maxQueueLen != (u32)(-1)) {
            fout << " maxqlen=" << maxQueueLen;
        }
    } else {
        fout << "prefix";
    }
    fout << " maxlag=" << left->maxLag;
    if (left->stopTable) {
        fout << " miracles";
    }
    if (left->countingMiracleOffset) {
        const RoseCountingMiracle *cm
            = (const RoseCountingMiracle *)((const char *)t
                                            + left->countingMiracleOffset);
        fout << " counting_miracle:" << (int)cm->count
             << (cm->shufti ? "s" : "v");
    }
    if (nfaSupportsZombie(nfa)) {
        fout << " zombie";
    }
    if (left->eod_check) {
        fout << " eod";
    }
}

static
void dumpComponentInfo(const RoseEngine *t, const string &base) {
    stringstream ss;
    ss << base << "rose_components.txt";
    ofstream fout(ss.str().c_str());

    fout << "Index  Offset\tEngine               \tStates S.State Bytes   Notes\n";

    for (u32 i = 0; i < t->queueCount; i++) {
        const NfaInfo *nfa_info = getNfaInfoByQueue(t, i);
        const NFA *n = getNfaByInfo(t, nfa_info);

        fout << left << setw(6) << i << " ";

        fout << left << ((const char *)n - (const char *)t) << "\t"; /* offset */

        fout << left << setw(16) << describe(*n) << "\t";

        fout << left << setw(6) << n->nPositions << " ";
        fout << left << setw(7) << n->streamStateSize << " ";
        fout << left << setw(7) << n->length << " ";

        dumpNfaNotes(fout, t, n);

        fout << endl;
    }
}

static
void dumpComponentInfoCsv(const RoseEngine *t, const string &base) {
    StdioFile f(base + "/rose_components.csv", "w");

    fprintf(f, "Index, Offset,Engine Type,States,Stream State,"
               "Bytecode Size,Kind,Notes\n");

    for (u32 i = 0; i < t->queueCount; i++) {
        const NfaInfo *nfa_info = getNfaInfoByQueue(t, i);
        const NFA *n = getNfaByInfo(t, nfa_info);
        nfa_kind kind;
        stringstream notes;

        if (i < t->outfixBeginQueue) {
            notes << "chained;";
        }

        if (nfa_info->eod) {
            notes << "eod;";
        }

        if (i < t->outfixEndQueue) {
            kind = NFA_OUTFIX;
        } else if (i < t->leftfixBeginQueue) {
            kind = NFA_SUFFIX;
        } else {
            const LeftNfaInfo *left = getLeftInfoByQueue(t, i);
            if (left->eager) {
                notes << "eager;";
            }
            if (left->transient) {
                notes << "transient " << (u32)left->transient << ";";
            }
            if (left->infix) {
                kind = NFA_INFIX;
                u32 maxQueueLen = left->maxQueueLen;
                if (maxQueueLen != (u32)(-1)) {
                    notes << "maxqlen=" << maxQueueLen << ";";
                }
            } else {
                kind = NFA_PREFIX;
            }
            notes << "maxlag=" << left->maxLag << ";";
            if (left->stopTable) {
                notes << "miracles;";
            }
            if (left->countingMiracleOffset) {
                auto cm = (const RoseCountingMiracle *)
                    ((const char *)t + left->countingMiracleOffset);
                notes << "counting_miracle:" << (int)cm->count
                      << (cm->shufti ? "s" : "v") << ";";
            }
            if (nfaSupportsZombie(n)) {
                notes << " zombie;";
            }
            if (left->eod_check) {
            notes << "left_eod;";
            }
        }

        fprintf(f, "%u,%zd,\"%s\",%u,%u,%u,%s,%s\n", i,
                (const char *)n - (const char *)t, describe(*n).c_str(),
                n->nPositions, n->streamStateSize, n->length,
                to_string(kind).c_str(), notes.str().c_str());
    }
}

static
void dumpExhaust(const RoseEngine *t, const string &base) {
    StdioFile f(base + "/rose_exhaust.csv", "w");

    const NfaInfo *infos
        = (const NfaInfo *)((const char *)t + t->nfaInfoOffset);

    u32 queue_count = t->activeArrayCount;

    for (u32 i = 0; i < queue_count; ++i) {
        u32 ekey_offset = infos[i].ekeyListOffset;

        fprintf(f, "%u (%u):", i, ekey_offset);

        if (ekey_offset) {
            const u32 *ekeys = (const u32 *)((const char *)t + ekey_offset);
            while (1) {
                u32 e = *ekeys;
                ++ekeys;
                if (e == ~0U) {
                    break;
                }
                fprintf(f, " %u", e);
            }
        }

        fprintf(f, "\n");
    }
}

static
void dumpNfas(const RoseEngine *t, bool dump_raw, const string &base) {
    dumpExhaust(t, base);

    for (u32 i = 0; i < t->queueCount; i++) {
        const NfaInfo *nfa_info = getNfaInfoByQueue(t, i);
        const NFA *n = getNfaByInfo(t, nfa_info);

        stringstream ssbase;
        ssbase << base << "rose_nfa_" << i;
        nfaGenerateDumpFiles(n, ssbase.str());

        if (dump_raw) {
            stringstream ssraw;
            ssraw << base << "rose_nfa_" << i << ".raw";
            StdioFile f(ssraw.str(), "w");
            fwrite(n, 1, n->length, f);
        }
    }
}

static
void dumpRevComponentInfo(const RoseEngine *t, const string &base) {
    stringstream ss;
    ss << base << "som_rev_components.txt";
    ofstream fout(ss.str().c_str());

    fout << "Index  Offset\tEngine               \tStates S.State Bytes\n";

    const char *tp = (const char *)t;
    const u32 *rev_offsets = (const u32 *)(tp + t->somRevOffsetOffset);

    for (u32 i = 0; i < t->somRevCount; i++) {
        u32 offset = rev_offsets[i];
        const NFA *n = (const NFA *)(tp + offset);

        fout << left << setw(6) << i << " ";

        fout << left << offset << "\t"; /* offset */

        fout << left << setw(16) << describe(*n) << "\t";

        fout << left << setw(6) << n->nPositions << " ";
        fout << left << setw(7) << n->streamStateSize << " ";
        fout << left << setw(7) << n->length;
        fout << endl;
    }
}

static
void dumpRevNfas(const RoseEngine *t, bool dump_raw, const string &base) {
    const char *tp = (const char *)t;
    const u32 *rev_offsets = (const u32 *)(tp + t->somRevOffsetOffset);

    for (u32 i = 0; i < t->somRevCount; i++) {
        const NFA *n = (const NFA *)(tp + rev_offsets[i]);

        stringstream ssbase;
        ssbase << base << "som_rev_nfa_" << i;
        nfaGenerateDumpFiles(n, ssbase.str());

        if (dump_raw) {
            stringstream ssraw;
            ssraw << base << "som_rev_nfa_" << i << ".raw";
            StdioFile f(ssraw.str(), "w");
            fwrite(n, 1, n->length, f);
        }
    }
}

static
void dumpAnchored(const RoseEngine *t, const string &base) {
    u32 i = 0;
    const anchored_matcher_info *curr
        = (const anchored_matcher_info *)getALiteralMatcher(t);

    while (curr) {
        const NFA *n = (const NFA *)((const char *)curr + sizeof(*curr));

        stringstream ssbase;
        ssbase << base << "anchored_" << i;
        nfaGenerateDumpFiles(n, ssbase.str());

        curr = curr->next_offset ? (const anchored_matcher_info *)
            ((const char *)curr + curr->next_offset) : nullptr;
        i++;
    };
}

static
void dumpAnchoredStats(const void *atable, FILE *f) {
    assert(atable);

    u32 i = 0;
    const anchored_matcher_info *curr = (const anchored_matcher_info *)atable;

    while (curr) {
        const NFA *n = (const NFA *)((const char *)curr + sizeof(*curr));

        fprintf(f, "  NFA %u: %s, %u states (%u bytes)\n", i,
                describe(*n).c_str(), n->nPositions, n->length);

        curr = curr->next_offset ? (const anchored_matcher_info *)
            ((const char *)curr + curr->next_offset) : nullptr;
        i++;
    };

}

static
void dumpLongLiteralSubtable(const RoseLongLitTable *ll_table,
                             const RoseLongLitSubtable *ll_sub, FILE *f) {
    if (!ll_sub->hashBits) {
        fprintf(f, "      <no table>\n");
        return;
    }

    const char *base = (const char *)ll_table;

    u32 nbits = ll_sub->hashBits;
    u32 num_entries = 1U << nbits;
    const auto *tab = (const RoseLongLitHashEntry *)(base + ll_sub->hashOffset);
    u32 hash_occ =
        count_if(tab, tab + num_entries, [](const RoseLongLitHashEntry &ent) {
            return ent.str_offset != 0;
        });
    float hash_occ_percent = ((float)hash_occ / (float)num_entries) * 100;

    fprintf(f, "      hash table   : %u bits, occupancy %u/%u (%0.1f%%)\n",
            nbits, hash_occ, num_entries, hash_occ_percent);

    u32 bloom_bits = ll_sub->bloomBits;
    u32 bloom_size = 1U << bloom_bits;
    const u8 *bloom = (const u8 *)base + ll_sub->bloomOffset;
    u32 bloom_occ = accumulate(bloom, bloom + bloom_size / 8, 0,
        [](const u32 &sum, const u8 &elem) { return sum + popcount32(elem); });
    float bloom_occ_percent = ((float)bloom_occ / (float)(bloom_size)) * 100;

    fprintf(f, "      bloom filter : %u bits, occupancy %u/%u (%0.1f%%)\n",
            bloom_bits, bloom_occ, bloom_size, bloom_occ_percent);
}

static
void dumpLongLiteralTable(const RoseEngine *t, FILE *f) {
    if (!t->longLitTableOffset) {
        return;
    }

    fprintf(f, "\n");
    fprintf(f, "Long literal table (streaming):\n");

    const auto *ll_table =
        (const struct RoseLongLitTable *)loadFromByteCodeOffset(
            t, t->longLitTableOffset);

    fprintf(f, "    total size     : %u bytes\n", ll_table->size);
    fprintf(f, "    longest len    : %u\n", ll_table->maxLen);
    fprintf(f, "    stream state   : %u bytes\n", ll_table->streamStateBytes);

    fprintf(f, "    caseful:\n");
    dumpLongLiteralSubtable(ll_table, &ll_table->caseful, f);

    fprintf(f, "    nocase:\n");
    dumpLongLiteralSubtable(ll_table, &ll_table->nocase, f);
}

static
void roseDumpText(const RoseEngine *t, FILE *f) {
    if (!t) {
        fprintf(f, "<< no rose >>\n");
        return;
    }

    const void *atable = getAnchoredMatcher(t);
    const HWLM *ftable = getFloatingMatcher(t);
    const HWLM *drtable = getDelayRebuildMatcher(t);
    const HWLM *etable = getEodMatcher(t);
    const HWLM *sbtable = getSmallBlockMatcher(t);

    fprintf(f, "Rose:\n\n");

    fprintf(f, "mode:                : ");
    switch(t->mode) {
    case HS_MODE_BLOCK:
        fprintf(f, "block");
        break;
    case HS_MODE_STREAM:
        fprintf(f, "streaming");
        break;
    case HS_MODE_VECTORED:
        fprintf(f, "vectored");
        break;
    }
    fprintf(f, "\n");

    fprintf(f, "properties           :");
    if (t->canExhaust) {
        fprintf(f, " canExhaust");
    }
    if (t->hasSom) {
        fprintf(f, " hasSom");
    }
    if (t->runtimeImpl == ROSE_RUNTIME_PURE_LITERAL) {
        fprintf(f, " pureLiteral");
    }
    if (t->runtimeImpl == ROSE_RUNTIME_SINGLE_OUTFIX) {
        fprintf(f, " soleOutfix");
    }
    fprintf(f, "\n");

    fprintf(f, "dkey count           : %u\n", t->dkeyCount);
    fprintf(f, "som slot count       : %u\n", t->somLocationCount);
    fprintf(f, "som width            : %u bytes\n", t->somHorizon);
    fprintf(f, "rose count           : %u\n", t->roseCount);
    fprintf(f, "\n");

    fprintf(f, "total engine size    : %u bytes\n", t->size);
    fprintf(f, " - anchored matcher  : %u bytes over %u bytes\n", t->asize,
            t->anchoredDistance);
    fprintf(f, " - floating matcher  : %zu bytes%s",
            ftable ? hwlmSize(ftable) : 0, t->noFloatingRoots ? " (cond)":"");
    if (t->floatingMinDistance) {
        fprintf(f, " from %s bytes\n",
                rose_off(t->floatingMinDistance).str().c_str());
    }
    if (t->floatingDistance != ROSE_BOUND_INF && ftable) {
        fprintf(f, " over %u bytes\n", t->floatingDistance);
    } else {
        fprintf(f, "\n");
    }
    fprintf(f, " - delay-rb matcher  : %zu bytes\n",
            drtable ? hwlmSize(drtable) : 0);
    fprintf(f, " - eod-anch matcher  : %zu bytes over last %u bytes\n",
            etable ? hwlmSize(etable) : 0, t->ematcherRegionSize);
    fprintf(f, " - small-blk matcher : %zu bytes over %u bytes\n",
            sbtable ? hwlmSize(sbtable) : 0, t->smallBlockDistance);
    fprintf(f, " - role state table  : %zu bytes\n",
            t->rolesWithStateCount * sizeof(u32));
    fprintf(f, " - nfa info table    : %zu bytes\n",
            t->queueCount * sizeof(NfaInfo));

    fprintf(f, "state space required : %u bytes\n", t->stateOffsets.end);
    fprintf(f, " - history buffer    : %u bytes\n", t->historyRequired);
    fprintf(f, " - exhaustion vector : %u bytes\n",
            t->stateOffsets.exhausted_size);
    fprintf(f, " - logical vector    : %u bytes\n",
            t->stateOffsets.logicalVec_size);
    fprintf(f, " - combination vector: %u bytes\n",
            t->stateOffsets.combVec_size);
    fprintf(f, " - role state mmbit  : %u bytes\n", t->stateSize);
    fprintf(f, " - long lit matcher  : %u bytes\n", t->longLitStreamState);
    fprintf(f, " - active array      : %u bytes\n",
            t->stateOffsets.activeLeafArray_size);
    fprintf(f, " - active rose       : %u bytes\n",
            t->stateOffsets.activeLeftArray_size);
    fprintf(f, " - anchored state    : %u bytes\n", t->anchorStateSize);
    fprintf(f, " - nfa state         : %u bytes\n",
            t->stateOffsets.end - t->stateOffsets.nfaStateBegin);
    fprintf(f, " - (trans. nfa state): %u bytes\n", t->tStateSize);
    fprintf(f, " - one whole bytes   : %u bytes\n",
            t->stateOffsets.anchorState - t->stateOffsets.leftfixLagTable);
    fprintf(f, " - groups            : %u bytes\n",
            t->stateOffsets.groups_size);
    fprintf(f, "\n");

    fprintf(f, "initial groups       : 0x%016llx\n", t->initialGroups);
    fprintf(f, "floating groups      : 0x%016llx\n", t->floating_group_mask);
    fprintf(f, "handled key count    : %u\n", t->handledKeyCount);
    fprintf(f, "\n");

    fprintf(f, "total literal count  : %u\n", t->totalNumLiterals);
    fprintf(f, "  delayed literals   : %u\n", t->delay_count);

    fprintf(f, "\n");
    fprintf(f, "  minWidth                    : %u\n", t->minWidth);
    fprintf(f, "  minWidthExcludingBoundaries : %u\n",
            t->minWidthExcludingBoundaries);
    fprintf(f, "  maxBiAnchoredWidth          : %s\n",
            rose_off(t->maxBiAnchoredWidth).str().c_str());
    fprintf(f, "  minFloatLitMatchOffset      : %s\n",
            rose_off(t->floatingMinLiteralMatchOffset).str().c_str());
    fprintf(f, "  maxFloatingDelayedMatch     : %s\n",
            rose_off(t->maxFloatingDelayedMatch).str().c_str());

    if (atable) {
        fprintf(f, "\nAnchored literal matcher stats:\n\n");
        dumpAnchoredStats(atable, f);
    }

    dumpLongLiteralTable(t, f);
}

#define DUMP_U8(o, member)                                              \
    fprintf(f, "    %-32s: %hhu/%hhx\n", #member, o->member, o->member)
#define DUMP_U32(o, member)                                             \
    fprintf(f, "    %-32s: %u/%08x\n", #member, o->member, o->member)
#define DUMP_U64(o, member)                                             \
    fprintf(f, "    %-32s: %llu/%016llx\n", #member, o->member, o->member)

static
void roseDumpStructRaw(const RoseEngine *t, FILE *f) {
    fprintf(f, "struct RoseEngine {\n");
    DUMP_U8(t, noFloatingRoots);
    DUMP_U8(t, requiresEodCheck);
    DUMP_U8(t, hasOutfixesInSmallBlock);
    DUMP_U8(t, runtimeImpl);
    DUMP_U8(t, mpvTriggeredByLeaf);
    DUMP_U8(t, canExhaust);
    DUMP_U8(t, hasSom);
    DUMP_U8(t, somHorizon);
    DUMP_U32(t, mode);
    DUMP_U32(t, historyRequired);
    DUMP_U32(t, ekeyCount);
    DUMP_U32(t, lkeyCount);
    DUMP_U32(t, lopCount);
    DUMP_U32(t, ckeyCount);
    DUMP_U32(t, logicalTreeOffset);
    DUMP_U32(t, combInfoMapOffset);
    DUMP_U32(t, dkeyCount);
    DUMP_U32(t, dkeyLogSize);
    DUMP_U32(t, invDkeyOffset);
    DUMP_U32(t, somLocationCount);
    DUMP_U32(t, somLocationFatbitSize);
    DUMP_U32(t, rolesWithStateCount);
    DUMP_U32(t, stateSize);
    DUMP_U32(t, anchorStateSize);
    DUMP_U32(t, tStateSize);
    DUMP_U32(t, smallWriteOffset);
    DUMP_U32(t, amatcherOffset);
    DUMP_U32(t, ematcherOffset);
    DUMP_U32(t, fmatcherOffset);
    DUMP_U32(t, drmatcherOffset);
    DUMP_U32(t, sbmatcherOffset);
    DUMP_U32(t, longLitTableOffset);
    DUMP_U32(t, amatcherMinWidth);
    DUMP_U32(t, fmatcherMinWidth);
    DUMP_U32(t, eodmatcherMinWidth);
    DUMP_U32(t, amatcherMaxBiAnchoredWidth);
    DUMP_U32(t, fmatcherMaxBiAnchoredWidth);
    DUMP_U32(t, reportProgramOffset);
    DUMP_U32(t, reportProgramCount);
    DUMP_U32(t, delayProgramOffset);
    DUMP_U32(t, anchoredProgramOffset);
    DUMP_U32(t, activeArrayCount);
    DUMP_U32(t, activeLeftCount);
    DUMP_U32(t, queueCount);
    DUMP_U32(t, activeQueueArraySize);
    DUMP_U32(t, eagerIterOffset);
    DUMP_U32(t, handledKeyCount);
    DUMP_U32(t, handledKeyFatbitSize);
    DUMP_U32(t, leftOffset);
    DUMP_U32(t, roseCount);
    DUMP_U32(t, eodProgramOffset);
    DUMP_U32(t, flushCombProgramOffset);
    DUMP_U32(t, lastByteHistoryIterOffset);
    DUMP_U32(t, minWidth);
    DUMP_U32(t, minWidthExcludingBoundaries);
    DUMP_U32(t, maxBiAnchoredWidth);
    DUMP_U32(t, anchoredDistance);
    DUMP_U32(t, anchoredMinDistance);
    DUMP_U32(t, floatingDistance);
    DUMP_U32(t, floatingMinDistance);
    DUMP_U32(t, smallBlockDistance);
    DUMP_U32(t, floatingMinLiteralMatchOffset);
    DUMP_U32(t, nfaInfoOffset);
    DUMP_U64(t, initialGroups);
    DUMP_U64(t, floating_group_mask);
    DUMP_U32(t, size);
    DUMP_U32(t, delay_count);
    DUMP_U32(t, delay_fatbit_size);
    DUMP_U32(t, anchored_count);
    DUMP_U32(t, anchored_fatbit_size);
    DUMP_U32(t, maxFloatingDelayedMatch);
    DUMP_U32(t, delayRebuildLength);
    DUMP_U32(t, stateOffsets.history);
    DUMP_U32(t, stateOffsets.exhausted);
    DUMP_U32(t, stateOffsets.exhausted_size);
    DUMP_U32(t, stateOffsets.logicalVec);
    DUMP_U32(t, stateOffsets.logicalVec_size);
    DUMP_U32(t, stateOffsets.combVec);
    DUMP_U32(t, stateOffsets.combVec_size);
    DUMP_U32(t, stateOffsets.activeLeafArray);
    DUMP_U32(t, stateOffsets.activeLeafArray_size);
    DUMP_U32(t, stateOffsets.activeLeftArray);
    DUMP_U32(t, stateOffsets.activeLeftArray_size);
    DUMP_U32(t, stateOffsets.leftfixLagTable);
    DUMP_U32(t, stateOffsets.anchorState);
    DUMP_U32(t, stateOffsets.groups);
    DUMP_U32(t, stateOffsets.groups_size);
    DUMP_U32(t, stateOffsets.longLitState);
    DUMP_U32(t, stateOffsets.longLitState_size);
    DUMP_U32(t, stateOffsets.somLocation);
    DUMP_U32(t, stateOffsets.somValid);
    DUMP_U32(t, stateOffsets.somWritable);
    DUMP_U32(t, stateOffsets.somMultibit_size);
    DUMP_U32(t, stateOffsets.nfaStateBegin);
    DUMP_U32(t, stateOffsets.end);
    DUMP_U32(t, boundary.reportEodOffset);
    DUMP_U32(t, boundary.reportZeroOffset);
    DUMP_U32(t, boundary.reportZeroEodOffset);
    DUMP_U32(t, totalNumLiterals);
    DUMP_U32(t, asize);
    DUMP_U32(t, outfixBeginQueue);
    DUMP_U32(t, outfixEndQueue);
    DUMP_U32(t, leftfixBeginQueue);
    DUMP_U32(t, initMpvNfa);
    DUMP_U32(t, rosePrefixCount);
    DUMP_U32(t, activeLeftIterOffset);
    DUMP_U32(t, ematcherRegionSize);
    DUMP_U32(t, somRevCount);
    DUMP_U32(t, somRevOffsetOffset);
    fprintf(f, "}\n");
    fprintf(f, "sizeof(RoseEngine) = %zu\n", sizeof(RoseEngine));
}

static
void roseDumpComponents(const RoseEngine *t, bool dump_raw,
                        const string &base) {
    dumpComponentInfo(t, base);
    dumpComponentInfoCsv(t, base);
    dumpNfas(t, dump_raw, base);
    dumpAnchored(t, base);
    dumpRevComponentInfo(t, base);
    dumpRevNfas(t, dump_raw, base);
}

static
void roseDumpPrograms(const vector<LitFragment> &fragments, const RoseEngine *t,
                      const string &base) {
    dumpRoseLitPrograms(fragments, t, base + "/rose_lit_programs.txt");
    dumpRoseEodPrograms(t, base + "/rose_eod_programs.txt");
    dumpRoseFlushCombPrograms(t, base + "/rose_flush_comb_programs.txt");
    dumpRoseReportPrograms(t, base + "/rose_report_programs.txt");
    dumpRoseAnchoredPrograms(t, base + "/rose_anchored_programs.txt");
    dumpRoseDelayPrograms(t, base + "/rose_delay_programs.txt");
}

static
void roseDumpLiteralMatchers(const RoseEngine *t, const string &base) {
    if (const HWLM *hwlm = getFloatingMatcher(t)) {
        hwlmGenerateDumpFiles(hwlm, base + "/lit_table_floating");
    }

    if (const HWLM *hwlm = getDelayRebuildMatcher(t)) {
        hwlmGenerateDumpFiles(hwlm, base + "/lit_table_delay_rebuild");
    }

    if (const HWLM *hwlm = getEodMatcher(t)) {
        hwlmGenerateDumpFiles(hwlm, base + "/lit_table_eod");
    }

    if (const HWLM *hwlm = getSmallBlockMatcher(t)) {
        hwlmGenerateDumpFiles(hwlm, base + "/lit_table_small_block");
    }
}

void dumpRose(const RoseBuildImpl &build, const vector<LitFragment> &fragments,
              const map<left_id, u32> &leftfix_queue_map,
              const map<suffix_id, u32> &suffix_queue_map,
              const RoseEngine *t) {
    const Grey &grey = build.cc.grey;

    if (!grey.dumpFlags) {
        return;
    }

    StdioFile f(grey.dumpPath + "/rose.txt", "w");

    if (!t) {
        fprintf(f, "<< no rose >>\n");
        return;
    }

    // Dump Rose table info
    roseDumpText(t, f);

    roseDumpComponents(t, false, grey.dumpPath);
    roseDumpPrograms(fragments, t, grey.dumpPath);
    roseDumpLiteralMatchers(t, grey.dumpPath);

    // Graph.
    dumpRoseGraph(build, t, fragments, leftfix_queue_map, suffix_queue_map,
                  "rose.dot");

    // Literals
    dumpRoseLiterals(build, fragments, grey);

    f = StdioFile(grey.dumpPath + "/rose_struct.txt", "w");
    roseDumpStructRaw(t, f);
}

} // namespace ue2
