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

/** \file
 * \brief Dump code for NFA graphs.
 *
 * The dump support in this file is for internal use only, and thus is not even
 * compiled in release builds, where DUMP_SUPPORT is not switched on.
 */

#include "config.h"

#include "nfagraph/ng_dump.h"

#include "hs_compile.h" /* for HS_MODE_* flags */
#include "ue2common.h"
#include "compiler/compiler.h"
#include "hwlm/hwlm_build.h"
#include "nfa/accel.h"
#include "nfa/nfa_internal.h" // for MO_INVALID_IDX
#include "nfagraph/ng.h"
#include "nfagraph/ng_util.h"
#include "parser/position.h"
#include "rose/rose_build.h"
#include "rose/rose_internal.h"
#include "smallwrite/smallwrite_dump.h"
#include "util/bitutils.h"
#include "util/dump_charclass.h"
#include "util/dump_util.h"
#include "util/report.h"
#include "util/report_manager.h"
#include "util/ue2string.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <ostream>
#include <set>
#include <sstream>
#include <utility>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

static
void describeAssert(ostream &os, u32 flags) {
#define DESCRIBE_ASSERT_CASE(x) case POS_FLAG_##x: s = #x; break
    while (flags) {
        const char *s;
        switch (1 << findAndClearLSB_32(&flags)) {
            DESCRIBE_ASSERT_CASE(NOFLOAT);
            DESCRIBE_ASSERT_CASE(MUST_FLOAT);
            DESCRIBE_ASSERT_CASE(FIDDLE_ACCEPT);
            DESCRIBE_ASSERT_CASE(VIRTUAL_START);
            DESCRIBE_ASSERT_CASE(MULTILINE_START);
            DESCRIBE_ASSERT_CASE(ASSERT_WORD_TO_WORD);
            DESCRIBE_ASSERT_CASE(ASSERT_WORD_TO_NONWORD);
            DESCRIBE_ASSERT_CASE(ASSERT_NONWORD_TO_WORD);
            DESCRIBE_ASSERT_CASE(ASSERT_NONWORD_TO_NONWORD);
            DESCRIBE_ASSERT_CASE(ASSERT_WORD_TO_WORD_UCP);
            DESCRIBE_ASSERT_CASE(ASSERT_WORD_TO_NONWORD_UCP);
            DESCRIBE_ASSERT_CASE(ASSERT_NONWORD_TO_WORD_UCP);
            DESCRIBE_ASSERT_CASE(ASSERT_NONWORD_TO_NONWORD_UCP);
        default:
            s = "unknown flag";
        }
        os << s << "\\n";
    }
#undef DESCRIBE_ASSERT_CASE
}

static
void describeReport(ostream &os, const ReportID report,
                    const ReportManager *rm) {
    if (!rm) {
        os << "\\nReport: " << report;
    } else {
        os << "\\nReport: " << report << " (";
        const Report &ir = rm->getReport(report);
        switch (ir.type) {
        case EXTERNAL_CALLBACK:
            os << "EXTERNAL " << ir.onmatch;
            if (ir.offsetAdjust) {
                os << " adj " << ir.offsetAdjust;
            }
            break;
        case EXTERNAL_CALLBACK_SOM_STORED:
            os << "SOM_STORED " << ir.somDistance;
            break;
        case EXTERNAL_CALLBACK_SOM_REL:
            os << "SOM_REL " << ir.somDistance;
            break;
        case EXTERNAL_CALLBACK_SOM_ABS:
            os << "SOM_ABS " << ir.somDistance;
            break;
        case EXTERNAL_CALLBACK_SOM_REV_NFA:
            os << "SOM_REV_NFA " << ir.revNfaIndex;
            break;
        case INTERNAL_SOM_LOC_SET:
            os << "SOM_LOC_SET " << ir.onmatch;
            break;
        case INTERNAL_SOM_LOC_SET_IF_UNSET:
            os << "SOM_LOC_SET_IF_UNSET " << ir.onmatch;
            break;
        case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
            os << "SOM_LOC_SET_IF_WRITABLE " << ir.onmatch;
            break;
        case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
            os << "SOM_LOC_SET_SOM_REV_NFA " << ir.onmatch << " nfa="
               << ir.revNfaIndex;
            break;
        case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
            os << "SOM_LOC_SET_SOM_REV_NFA_IF_UNSET " << ir.onmatch << " nfa="
               << ir.revNfaIndex;
            break;
        case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
            os << "SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE " << ir.onmatch
               << " nfa=" << ir.revNfaIndex;
            break;
        case INTERNAL_SOM_LOC_COPY:
            os << "SOM_LOC_COPY " << ir.somDistance << " to " << ir.onmatch;
            break;
        case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
            os << "SOM_LOC_COPY_IF_WRITABLE " << ir.somDistance
               << " to " << ir.onmatch;
            break;
        case INTERNAL_SOM_LOC_MAKE_WRITABLE:
            os << "SOM_LOC_MAKE_WRITABLE " << ir.onmatch;
            break;
        default:
            os << "no dump code!";
            break;
        }
        os << ")";
    }
}

namespace {
template <typename VertexT, typename EdgeT, typename GraphT>
class NFAWriter {
public:
    explicit NFAWriter(const GraphT &g_in) : g(g_in) {}

    NFAWriter(const GraphT &g_in, const ReportManager &rm_in)
        : g(g_in), rm(&rm_in) {}

    NFAWriter(const GraphT &g_in,
              const unordered_map<NFAVertex, u32> &region_map_in)
        : g(g_in), region_map(&region_map_in) {}

    void operator()(ostream& os, const VertexT& v) const {
        u32 v_index = g[v].index;

        os << "[";
        os << "fontsize=11, width=2, height=2, ";
        os << "label=\"" << v_index;
        os << "\\n";

        if (is_special(v, g)) {
            switch (v_index) {
                case NODE_START:
                    os << "START"; break;
                case NODE_START_DOTSTAR:
                    os << "START-DS"; break;
                case NODE_ACCEPT:
                    os << "ACCEPT"; break;
                case NODE_ACCEPT_EOD:
                    os << "ACCEPT-EOD"; break;
                default:
                    os << "UNKNOWN-SPECIAL"; break;
            }
            os << "\\n";
        } else {
            // If it's an assert vertex, then display its info.
            u32 assert_flags = g[v].assert_flags;
            if (assert_flags) {
                describeAssert(os, assert_flags);
                os << "\\n";
            }
        }

        // Dump character reachability (in brief).
        describeClass(os, g[v].char_reach, 5, CC_OUT_DOT);

        for (const auto &report : g[v].reports) {
            describeReport(os, report, rm);
        }

        os << "\",";

        if (is_any_start(v, g)) {
            os << "shape=octagon,";
        }

        os << "]";

        // If we have a region map, use it to generate clusters.
        if (region_map) {
            auto region_id = region_map->at(v);
            os << "subgraph cluster_" << region_id << " { label=\"region "
               << region_id << "\"; style=dashed;" << v_index << ";}";
        }
    }

    void operator()(ostream& os, const EdgeT& e) const {
        // Edge label. Print priority.
        os << "[fontsize=9,label=\"";
        // print tops if any set.
        if (!g[e].tops.empty()) {
            os << "TOP " << as_string_list(g[e].tops) << "\\n";
        }

        // If it's an assert vertex, then display its info.
        int assert_flags = g[e].assert_flags;
        if (assert_flags) {
            os << "\\n";
            describeAssert(os, assert_flags);
        }

        os << "\"]";
    }

private:
    const GraphT &g;
    const ReportManager *rm = nullptr;
    const unordered_map<NFAVertex, u32> *region_map = nullptr;
};
}

template <typename GraphT>
void dumpGraphImpl(const char *name, const GraphT &g) {
    typedef typename boost::graph_traits<GraphT>::vertex_descriptor VertexT;
    typedef typename boost::graph_traits<GraphT>::edge_descriptor EdgeT;
    ofstream os(name);
    NFAWriter<VertexT, EdgeT, GraphT> writer(g);
    writeGraphviz(os, g, writer, get(&NFAGraphVertexProps::index, g));
}

template <typename GraphT>
void dumpGraphImpl(const char *name, const GraphT &g, const ReportManager &rm) {
    typedef typename boost::graph_traits<GraphT>::vertex_descriptor VertexT;
    typedef typename boost::graph_traits<GraphT>::edge_descriptor EdgeT;
    ofstream os(name);
    NFAWriter<VertexT, EdgeT, GraphT> writer(g, rm);
    writeGraphviz(os, g, writer, get(&NFAGraphVertexProps::index, g));
}

template <typename GraphT>
void dumpGraphImpl(const char *name, const GraphT &g,
                   const unordered_map<NFAVertex, u32> &region_map) {
    typedef typename boost::graph_traits<GraphT>::vertex_descriptor VertexT;
    typedef typename boost::graph_traits<GraphT>::edge_descriptor EdgeT;
    ofstream os(name);
    NFAWriter<VertexT, EdgeT, GraphT> writer(g, region_map);
    writeGraphviz(os, g, writer, get(&NFAGraphVertexProps::index, g));
}

// manual instantiation of templated dumpGraph above.
template void dumpGraphImpl(const char *, const NGHolder &);

void dumpDotWrapperImpl(const NGHolder &g, const ExpressionInfo &expr,
                        const char *name, const Grey &grey) {
    if (grey.dumpFlags & Grey::DUMP_INT_GRAPH) {
        stringstream ss;
        ss << grey.dumpPath << "Expr_" << expr.index << "_" << name << ".dot";
        DEBUG_PRINTF("dumping dot graph to '%s'\n", ss.str().c_str());
        dumpGraphImpl(ss.str().c_str(), g);
    }
}

void dumpComponentImpl(const NGHolder &g, const char *name, u32 expr,
                       u32 comp, const Grey &grey) {
    if (grey.dumpFlags & Grey::DUMP_INT_GRAPH) {
        stringstream ss;
        ss << grey.dumpPath << "Comp_" << expr << "-" << comp << "_"
           << name << ".dot";
        DEBUG_PRINTF("dumping dot graph to '%s'\n", ss.str().c_str());
        dumpGraphImpl(ss.str().c_str(), g);
    }
}

void dumpSomSubComponentImpl(const NGHolder &g, const char *name, u32 expr,
                             u32 comp, u32 plan, const Grey &grey) {
    if (grey.dumpFlags & Grey::DUMP_INT_GRAPH) {
        stringstream ss;
        ss << grey.dumpPath << "Comp_" << expr << "-" << comp << "_"
           <<  name << "_" << plan << ".dot";
        DEBUG_PRINTF("dumping dot graph to '%s'\n", ss.str().c_str());
        dumpGraphImpl(ss.str().c_str(), g);
    }
}

void dumpHolderImpl(const NGHolder &h, unsigned int stageNumber,
                    const char *stageName, const Grey &grey) {
    if (grey.dumpFlags & Grey::DUMP_INT_GRAPH) {
        stringstream ss;
        ss << grey.dumpPath << "Holder_X_" << stageNumber
           << "-" << stageName << ".dot";
        dumpGraphImpl(ss.str().c_str(), h);
    }
}

void dumpHolderImpl(const NGHolder &h,
                    const unordered_map<NFAVertex, u32> &region_map,
                    unsigned int stageNumber, const char *stageName,
                    const Grey &grey) {
    if (grey.dumpFlags & Grey::DUMP_INT_GRAPH) {
        stringstream ss;
        ss << grey.dumpPath << "Holder_X_" << stageNumber
           << "-" << stageName << ".dot";
        dumpGraphImpl(ss.str().c_str(), h, region_map);
    }
}

void dumpSmallWrite(const RoseEngine *rose, const Grey &grey) {
    if (!grey.dumpFlags) {
        return;
    }

    const struct SmallWriteEngine *smwr = getSmallWrite(rose);
    smwrDumpText(smwr, StdioFile(grey.dumpPath + "smallwrite.txt", "w"));
    smwrDumpNFA(smwr, false, grey.dumpPath);
}

static
const char *reportTypeToString(ReportType type) {
#define REPORT_TYPE_CASE(x) case x: return #x
    switch (type) {
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK);
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK_SOM_REL);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_IF_UNSET);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_IF_WRITABLE);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_SOM_REV_NFA);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_COPY);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_COPY_IF_WRITABLE);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_MAKE_WRITABLE);
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK_SOM_STORED);
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK_SOM_ABS);
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK_SOM_REV_NFA);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_FROM);
        REPORT_TYPE_CASE(INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE);
        REPORT_TYPE_CASE(INTERNAL_ROSE_CHAIN);
        REPORT_TYPE_CASE(EXTERNAL_CALLBACK_SOM_PASS);
    }
#undef REPORT_TYPE_CASE

    assert(0);
    return "<unknown>";
}

static
int isReverseNfaReport(const Report &report) {
    switch (report.type) {
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        return 1;
    default:
        break; // fall through
    }
    return 0;
}

static
int isSomRelSetReport(const Report &report) {
    switch (report.type) {
    case INTERNAL_SOM_LOC_SET:
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
        return 1;
    default:
        break; // fall through
    }
    return 0;
}

void dumpReportManager(const ReportManager &rm, const Grey &grey) {
    if (!grey.dumpFlags) {
        return;
    }

    StdioFile f(grey.dumpPath + "internal_reports.txt", "w");
    const vector<Report> &reports = rm.reports();
    for (size_t i = 0; i < reports.size(); i++) {
        const Report &report = reports[i];
        fprintf(f, "%zu: %s onmatch: %u", i, reportTypeToString(report.type),
                report.onmatch);

        u32 dkey = rm.getDkey(report);
        if (dkey != MO_INVALID_IDX) {
            fprintf(f, " dkey %u", dkey);
        }
        if (report.ekey != INVALID_EKEY) {
            fprintf(f, " ekey %u", report.ekey);
        }
        if (report.hasBounds()) {
            fprintf(f, " hasBounds (minOffset=%llu, maxOffset=%llu, "
                       "minLength=%llu)",
                    report.minOffset, report.maxOffset, report.minLength);
        }
        if (report.quashSom) {
            fprintf(f, " quashSom");
        }
        if (report.offsetAdjust != 0) {
            fprintf(f, " offsetAdjust: %d", report.offsetAdjust);
        }
        if (isReverseNfaReport(report)) {
            fprintf(f, " reverse nfa: %u", report.revNfaIndex);
        }
        if (isSomRelSetReport(report)) {
            fprintf(f, " set, adjust: %llu", report.somDistance);
        }
        if (report.type == EXTERNAL_CALLBACK_SOM_REL) {
            fprintf(f, " relative: %llu", report.somDistance);
        }
        if (report.type == EXTERNAL_CALLBACK_SOM_ABS) {
            fprintf(f, " absolute: %llu", report.somDistance);
        }
        fprintf(f, "\n");
    }
}

} // namespace ue2
