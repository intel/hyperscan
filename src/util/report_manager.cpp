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

/** \file
 * \brief ReportManager: tracks Report structures, exhaustion and dedupe keys.
 */

#include "report_manager.h"

#include "grey.h"
#include "ue2common.h"
#include "compiler/compiler.h"
#include "nfagraph/ng.h"
#include "rose/rose_build.h"
#include "util/compile_error.h"
#include "util/container.h"

#include <deque>
#include <map>
#include <sstream>
#include <vector>

using namespace std;

namespace ue2 {

ReportManager::ReportManager(const Grey &g)
    : grey(g), freeEIndex(0), global_exhaust(true) {}

u32 ReportManager::getInternalId(const Report &ir) {
    auto it = reportIdToInternalMap.find(ir);
    if (it != reportIdToInternalMap.end()) {
        DEBUG_PRINTF("existing report %zu\n", it->second);
        return it->second;
    }

    // Construct a new internal report and assign it a ReportID.

    if (numReports() >= grey.limitReportCount) {
        throw ResourceLimitError();
    }

    u32 size = reportIds.size();
    reportIds.push_back(ir);
    reportIdToInternalMap.emplace(ir, size);
    DEBUG_PRINTF("new report %u\n", size);
    return size;
}

const Report &ReportManager::getReport(u32 id) const {
    assert(id < reportIds.size());
    return reportIds.at(id);
}

size_t ReportManager::numReports() const {
    return reportIds.size();
}

u32 ReportManager::getExhaustibleKey(u32 a) {
    auto it = toExhaustibleKeyMap.find(a);
    if (it == toExhaustibleKeyMap.end()) {
        // get size before assigning to avoid wacky LHS shenanigans
        u32 size = toExhaustibleKeyMap.size();
        bool inserted;
        tie(it, inserted) = toExhaustibleKeyMap.emplace(s64a{a}, size);
        assert(inserted);
    }

    DEBUG_PRINTF("%lld -> ekey %u\n", it->first, it->second);
    return it->second;
}

const set<u32> &ReportManager::getRelateCKeys(u32 lkey) {
    auto it = pl.lkey2ckeys.find(lkey);
    assert(it != pl.lkey2ckeys.end());
    return it->second;
}

void ReportManager::logicalKeyRenumber() {
    pl.logicalKeyRenumber();
    // assign to corresponding report
    for (u32 i = 0; i < reportIds.size(); i++) {
        Report &ir = reportIds[i];
        if (contains(pl.toLogicalKeyMap, ir.onmatch)) {
            ir.lkey = pl.toLogicalKeyMap.at(ir.onmatch);
        }
    }
}

const vector<LogicalOp> &ReportManager::getLogicalTree() const {
    return pl.logicalTree;
}

const vector<CombInfo> &ReportManager::getCombInfoMap() const {
    return pl.combInfoMap;
}

u32 ReportManager::getUnassociatedExhaustibleKey(void) {
    u32 rv = toExhaustibleKeyMap.size();
    bool inserted;
    map<s64a, u32>::const_iterator it;
    tie(it, inserted) = toExhaustibleKeyMap.emplace(--freeEIndex, rv);
    assert(inserted);
    assert(it->second == rv);

    return rv;
}

u32 ReportManager::numDkeys() const {
    DEBUG_PRINTF("%zu dkeys\n", reportIdToDedupeKey.size());
    return reportIdToDedupeKey.size();
}

u32 ReportManager::numEkeys() const {
    return (u32) toExhaustibleKeyMap.size();
}

u32 ReportManager::numLogicalKeys() const {
    return (u32) pl.toLogicalKeyMap.size();
}

u32 ReportManager::numLogicalOps() const {
    return (u32) pl.logicalTree.size();
}

u32 ReportManager::numCkeys() const {
    return (u32) pl.toCombKeyMap.size();
}

bool ReportManager::patternSetCanExhaust() const {
    return global_exhaust && !toExhaustibleKeyMap.empty();
}

vector<ReportID> ReportManager::getDkeyToReportTable() const {
    vector<ReportID> rv(reportIdToDedupeKey.size());

    for (const auto &m : reportIdToDedupeKey) {
        assert(m.second < rv.size());
        rv[m.second] = m.first;
    }

    return rv;
}

void ReportManager::assignDkeys(const RoseBuild *rose) {
    DEBUG_PRINTF("assigning...\n");

    map<u32, flat_set<ReportID>> ext_to_int;

    for (u32 i = 0; i < reportIds.size(); i++) {
        const Report &ir = reportIds[i];

        /* need to populate dkey */
        if (isExternalReport(ir)) {
            ext_to_int[ir.onmatch].insert(i);
        }
    }

    auto dedupe = rose->generateDedupeAux();

    for (const auto &m : ext_to_int) {
        u32 ext = m.first;

        if (!dedupe->requiresDedupeSupport(m.second)) {
            DEBUG_PRINTF("%u does not require dedupe\n", ext);
            continue; /* no dedupe required for this set */
        }

        u32 dkey = reportIdToDedupeKey.size();
        reportIdToDedupeKey[ext] = dkey;
        DEBUG_PRINTF("ext=%u -> dkey=%u\n", ext, dkey);
    }
}

u32 ReportManager::getDkey(const Report &r) const {
    if (!isExternalReport(r)) {
        return ~u32{0};
    }

    auto it = reportIdToDedupeKey.find(r.onmatch);
    if (it == reportIdToDedupeKey.end()) {
        return ~u32{0};
    }
    return it->second;
}

void ReportManager::registerExtReport(ReportID id,
                                      const external_report_info &ext) {
    auto it = externalIdMap.find(id);
    if (it != externalIdMap.end()) {
        const external_report_info &eri = it->second;
        if (eri.highlander != ext.highlander) {
            /* we have a problem */
            ostringstream out;
            out << "Expression (index " << ext.first_pattern_index
                << ") with match ID " << id << " ";
            if (!ext.highlander) {
                out << "did not specify ";
            } else {
                out << "specified ";
            }
            out << "HS_FLAG_SINGLEMATCH whereas previous expression (index "
                << eri.first_pattern_index << ") with the same match ID did";
            if (ext.highlander) {
                out << " not";
            }
            out << ".";
            throw CompileError(ext.first_pattern_index, out.str());
        }
    } else {
        externalIdMap.emplace(id, ext);
    }

    // Any non-highlander pattern will render us not globally exhaustible.
    if (!ext.highlander) {
        global_exhaust = false;
    }
}

Report ReportManager::getBasicInternalReport(const ExpressionInfo &expr,
                                             s32 adj) {
    /* validate that we are not violating highlander constraints, this will
     * throw a CompileError if so. */
    registerExtReport(expr.report,
                      external_report_info(expr.highlander, expr.index));

    /* create the internal report */
    u32 ekey = INVALID_EKEY;
    if (expr.highlander) {
        /* all patterns with the same report id share an ekey */
        ekey = getExhaustibleKey(expr.report);
    }

    return makeECallback(expr.report, adj, ekey, expr.quiet);
}

void ReportManager::setProgramOffset(ReportID id, u32 programOffset) {
    assert(id < reportIds.size());
    assert(!contains(reportIdToProgramOffset, id));
    reportIdToProgramOffset.emplace(id, programOffset);
}

u32 ReportManager::getProgramOffset(ReportID id) const {
    assert(id < reportIds.size());
    assert(contains(reportIdToProgramOffset, id));
    return reportIdToProgramOffset.at(id);
}

static
void ekeysUnion(std::set<u32> *ekeys, u32 more) {
    if (!ekeys->empty()) {
        if (more == INVALID_EKEY) {
            ekeys->clear();
        } else {
            ekeys->insert(more);
        }
    }
}

set<u32> reportsToEkeys(const set<ReportID> &reports, const ReportManager &rm) {
    assert(!reports.empty());

    set<u32> ekeys;

    for (auto it = reports.begin(), ite = reports.end(); it != ite; ++it) {
        u32 e = rm.getReport(*it).ekey;
        if (it == reports.begin()) {
            if (e != INVALID_EKEY) {
                ekeys.insert(e);
            }
        } else {
            ekeysUnion(&ekeys, e);
        }
    }

    return ekeys;
}

} // namespace ue2
