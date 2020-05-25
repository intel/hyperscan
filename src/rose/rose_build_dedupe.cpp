/*
 * Copyright (c) 2017, Intel Corporation
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

#include "rose_build_impl.h"
#include "nfa/castlecompile.h"
#include "nfagraph/ng_repeat.h"
#include "smallwrite/smallwrite_build.h"
#include "util/compile_context.h"
#include "util/boundary_reports.h"
#include "util/make_unique.h"
#include "util/report_manager.h"

using namespace std;

namespace ue2 {

static
bool requiresDedupe(const NGHolder &h, const flat_set<ReportID> &reports,
                    const Grey &grey) {
    /* TODO: tighten */
    NFAVertex seen_vert = NGHolder::null_vertex();

    for (auto v : inv_adjacent_vertices_range(h.accept, h)) {
        if (has_intersection(h[v].reports, reports)) {
            if (seen_vert != NGHolder::null_vertex()) {
                return true;
            }
            seen_vert = v;
        }
    }

    for (auto v : inv_adjacent_vertices_range(h.acceptEod, h)) {
        if (has_intersection(h[v].reports, reports)) {
            if (seen_vert != NGHolder::null_vertex()) {
                return true;
            }
            seen_vert = v;
        }
    }

    if (seen_vert) {
        /* if the reporting vertex is part of of a terminal repeat, the
         * construction process may reform the graph splitting it into two
         * vertices (pos, cyclic) and hence require dedupe */
        vector<GraphRepeatInfo> repeats;
        findRepeats(h, grey.minExtBoundedRepeatSize, &repeats);
        for (const auto &repeat : repeats) {
            if (find(repeat.vertices.begin(), repeat.vertices.end(),
                     seen_vert) != repeat.vertices.end()) {
                return true;
            }
        }
    }

    return false;
}

class RoseDedupeAuxImpl : public RoseDedupeAux {
public:
    explicit RoseDedupeAuxImpl(const RoseBuildImpl &build_in);
    bool requiresDedupeSupport(
        const flat_set<ReportID> &reports) const override;

private:
    bool hasSafeMultiReports(const flat_set<ReportID> &reports) const;

    const RoseBuildImpl &build;
    map<ReportID, set<RoseVertex>> vert_map; //!< ordinary literals
    map<ReportID, set<RoseVertex>> sb_vert_map; //!< small block literals
    map<ReportID, set<suffix_id>> suffix_map;
    map<ReportID, set<const OutfixInfo *>> outfix_map;
    map<ReportID, set<const raw_puff *>> puff_map;

    unordered_set<ReportID> live_reports; //!< all live internal reports.
};

unique_ptr<RoseDedupeAux> RoseBuildImpl::generateDedupeAux() const {
    return ue2::make_unique<RoseDedupeAuxImpl>(*this);
}

RoseDedupeAux::~RoseDedupeAux() = default;

RoseDedupeAuxImpl::RoseDedupeAuxImpl(const RoseBuildImpl &build_in)
    : build(build_in) {
    const RoseGraph &g = build.g;

    set<suffix_id> suffixes;

    for (auto v : vertices_range(g)) {
        insert(&live_reports, g[v].reports);

        // Literals in the small block table are "shadow" copies of literals in
        // the other tables that do not run in the same runtime invocation.
        // Dedupe key assignment will be taken care of by the real literals.
        if (build.hasLiteralInTable(v, ROSE_ANCHORED_SMALL_BLOCK)) {
            for (const auto &report_id : g[v].reports) {
                sb_vert_map[report_id].insert(v);
            }
        } else {
            for (const auto &report_id : g[v].reports) {
                vert_map[report_id].insert(v);
            }
        }

        // Several vertices may share a suffix, so we collect the set of
        // suffixes first to avoid repeating work.
        if (g[v].suffix) {
            suffixes.insert(g[v].suffix);
        }
    }

    for (const auto &suffix : suffixes) {
        for (const auto &report_id : all_reports(suffix)) {
            suffix_map[report_id].insert(suffix);
            live_reports.insert(report_id);
        }
    }

    for (const auto &outfix : build.outfixes) {
        for (const auto &report_id : all_reports(outfix)) {
            outfix_map[report_id].insert(&outfix);
            live_reports.insert(report_id);
        }
    }

    if (build.mpv_outfix) {
        auto *mpv = build.mpv_outfix->mpv();
        for (const auto &puff : mpv->puffettes) {
            puff_map[puff.report].insert(&puff);
            live_reports.insert(puff.report);
        }
        for (const auto &puff : mpv->triggered_puffettes) {
            puff_map[puff.report].insert(&puff);
            live_reports.insert(puff.report);
        }
    }

    for (const auto &report_id : build.smwr.all_reports()) {
        live_reports.insert(report_id);
    }

    // Collect live reports from boundary reports.
    insert(&live_reports, build.boundary.report_at_0);
    insert(&live_reports, build.boundary.report_at_0_eod);
    insert(&live_reports, build.boundary.report_at_eod);

    DEBUG_PRINTF("%zu of %zu reports are live\n", live_reports.size(),
                 build.rm.numReports());
}

static
vector<CharReach> makePath(const rose_literal_id &lit) {
    vector<CharReach> path(begin(lit.s), end(lit.s));
    for (u32 i = 0; i < lit.delay; i++) {
        path.push_back(CharReach::dot());
    }
    return path;
}

/**
 * \brief True if one of the given literals overlaps with the suffix of
 * another, meaning that they could arrive at the same offset.
 */
static
bool literalsCouldRace(const rose_literal_id &lit1,
                       const rose_literal_id &lit2) {
    DEBUG_PRINTF("compare %s (delay %u) and %s (delay %u)\n",
                 dumpString(lit1.s).c_str(), lit1.delay,
                 dumpString(lit2.s).c_str(), lit2.delay);

    // Add dots on the end of each literal for delay.
    const auto v1 = makePath(lit1);
    const auto v2 = makePath(lit2);

    // See if the smaller path is a suffix of the larger path.
    const auto *smaller = v1.size() < v2.size() ? &v1 : &v2;
    const auto *bigger = v1.size() < v2.size() ? &v2 : &v1;
    auto r = mismatch(smaller->rbegin(), smaller->rend(), bigger->rbegin(),
                      overlaps);
    return r.first == smaller->rend();
}

bool RoseDedupeAuxImpl::hasSafeMultiReports(
    const flat_set<ReportID> &reports) const {
    if (reports.size() <= 1) {
        return true;
    }

    /* We have more than one ReportID corresponding to the external ID that is
     * presented to the user. These may differ in offset adjustment, bounds
     * checks, etc. */

    /* TODO: work out if these differences will actually cause problems */

    /* One common case where we know we don't have a problem is if there are
     * precisely two reports, one for the main Rose path and one for the
     * "small block matcher" path. */
    if (reports.size() == 2) {
        ReportID id1 = *reports.begin();
        ReportID id2 = *reports.rbegin();

        bool has_verts_1 = contains(vert_map, id1);
        bool has_verts_2 = contains(vert_map, id2);
        bool has_sb_verts_1 = contains(sb_vert_map, id1);
        bool has_sb_verts_2 = contains(sb_vert_map, id2);

        if (has_verts_1 != has_verts_2 && has_sb_verts_1 != has_sb_verts_2) {
            DEBUG_PRINTF("two reports, one full and one small block: ok\n");
            return true;
        }
    }

    DEBUG_PRINTF("more than one report\n");
    return false;
}

bool RoseDedupeAuxImpl::requiresDedupeSupport(
    const flat_set<ReportID> &reports_in) const {
    /* TODO: this could be expanded to check for offset or character
       constraints */

    // We don't want to consider dead reports (tracked by ReportManager but no
    // longer used) for the purposes of assigning dupe keys.
    flat_set<ReportID> reports;
    for (auto id : reports_in) {
        if (contains(live_reports, id)) {
            reports.insert(id);
        }
    }

    DEBUG_PRINTF("live reports: %s\n", as_string_list(reports).c_str());

    const RoseGraph &g = build.g;

    bool has_suffix = false;
    bool has_outfix = false;

    if (!hasSafeMultiReports(reports)) {
        DEBUG_PRINTF("multiple reports not safe\n");
        return true;
    }

    set<RoseVertex> roles;
    set<suffix_id> suffixes;
    set<const OutfixInfo *> outfixes;
    set<const raw_puff *> puffettes;
    for (ReportID r : reports) {
        if (contains(vert_map, r)) {
            insert(&roles, vert_map.at(r));
        }
        if (contains(suffix_map, r)) {
            insert(&suffixes, suffix_map.at(r));
        }

        if (contains(outfix_map, r)) {
            insert(&outfixes, outfix_map.at(r));
        }

        if (contains(puff_map, r)) {
            insert(&puffettes, puff_map.at(r));
        }
    }

    /* roles */

    map<u32, u32> lits; // Literal ID -> count of occurrences.

    const bool has_role = !roles.empty();
    for (auto v : roles) {
        for (const auto &lit : g[v].literals) {
            lits[lit]++;
        }
        if (g[v].eod_accept) {
            // Literals plugged into this EOD accept must be taken into account
            // as well.
            for (auto u : inv_adjacent_vertices_range(v, g)) {
                for (const auto &lit : g[u].literals) {
                    lits[lit]++;
                }
            }
        }
    }

    /* literals */

    for (const auto &m : lits) {
        if (m.second > 1) {
            DEBUG_PRINTF("lit %u used by >1 reporting roles\n", m.first);
            return true;
        }
    }

    for (auto it = begin(lits); it != end(lits); ++it) {
        const auto &lit1 = build.literals.at(it->first);
        for (auto jt = next(it); jt != end(lits); ++jt) {
            const auto &lit2 = build.literals.at(jt->first);
            if (literalsCouldRace(lit1, lit2)) {
                DEBUG_PRINTF("literals could race\n");
                return true;
            }
        }
    }

    /* suffixes */

    for (const auto &suffix : suffixes) {
        if (has_suffix || has_role) {
            return true; /* scope for badness */
        }

        has_suffix = true;

        /* some lesser suffix engines (nfas, haig, castle) can raise multiple
         * matches for a report id at the same offset if there are multiple
         * report states live. */
        if (suffix.haig()) {
            return true;
        }
        if (suffix.graph() &&
            requiresDedupe(*suffix.graph(), reports, build.cc.grey)) {
            return true;
        }
        if (suffix.castle() && requiresDedupe(*suffix.castle(), reports)) {
            return true;
        }
    }

    /* outfixes */

    for (const auto &outfix_ptr : outfixes) {
        assert(outfix_ptr);
        const OutfixInfo &out = *outfix_ptr;

        if (has_outfix || has_role || has_suffix) {
            return true;
        }
        has_outfix = true;

        if (out.haig()) {
            return true; /* haig may report matches with different SOM at the
                            same offset */
        }

        if (out.holder() &&
            requiresDedupe(*out.holder(), reports, build.cc.grey)) {
            return true;
        }
    }

    /* mpv */
    for (UNUSED const auto &puff : puffettes) {
        if (has_outfix || has_role || has_suffix) {
            return true;
        }
        has_outfix = true;
    }

    /* boundary */
    if (has_intersection(build.boundary.report_at_eod, reports)) {
        if (has_outfix || has_role || has_suffix) {
            return true;
        }
    }

    return false;
}

} // namespace ue2
