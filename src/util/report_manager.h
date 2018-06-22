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
 * \brief ReportManager: tracks Report structures, exhaustion and
 * dedupe keys.
 */

#ifndef REPORT_MANAGER_H
#define REPORT_MANAGER_H

#include "ue2common.h"
#include "util/compile_error.h"
#include "util/noncopyable.h"
#include "util/report.h"
#include "parser/logical_combination.h"

#include <map>
#include <set>
#include <unordered_map>
#include <vector>

namespace ue2 {

struct Grey;
class RoseBuild;
class ExpressionInfo;

struct external_report_info {
    external_report_info(bool h, u32 fpi)
    : highlander(h), first_pattern_index(fpi) { }
    const bool highlander;
    const u32 first_pattern_index;
};

/** \brief Tracks Report structures, exhaustion and dedupe keys. */
class ReportManager : noncopyable {
public:
    explicit ReportManager(const Grey &g);

    /** \brief Fetch the ID associated with the given Report. */
    u32 getInternalId(const Report &r);

    /** \brief Fetch the Report associated with \a id. */
    const Report &getReport(u32 id) const;

    /** \brief Total number of reports. */
    size_t numReports() const;

    /** \brief Return an unused exhaustion key (the next available one). */
    u32 getUnassociatedExhaustibleKey(void);

    /** \brief Total number of dedupe keys. */
    u32 numDkeys() const;

    /** \brief Total number of exhaustion keys. */
    u32 numEkeys() const;

    /** \brief Total number of logical keys. */
    u32 numLogicalKeys() const;

    /** \brief Total number of logical operators. */
    u32 numLogicalOps() const;

    /** \brief Total number of combination keys. */
    u32 numCkeys() const;

    /** \brief True if the pattern set can exhaust (i.e. all patterns are
     * highlander). */
    bool patternSetCanExhaust() const;

    void assignDkeys(const RoseBuild *rose);

    std::vector<ReportID> getDkeyToReportTable() const;

    /** \brief Return a const reference to the table of Report
     * structures. */
    const std::vector<Report> &reports() const { return reportIds; }

    /**
     * Get a simple internal report corresponding to the expression. An ekey
     * will be setup if required.
     *
     * Note: this function may throw a CompileError if constraints on external
     * match id are violated (mixed highlander status for example).
     */
    Report getBasicInternalReport(const ExpressionInfo &expr, s32 adj = 0);

    /** \brief Register an external report and validate that we are not
     * violating highlander constraints (which will cause an exception to be
     * thrown). */
    void registerExtReport(ReportID id, const external_report_info &ext);

    /** \brief Fetch the ekey associated with the given expression index,
     * assigning one if necessary. */
    u32 getExhaustibleKey(u32 expressionIndex);

    /** \brief Get lkey's corresponding ckeys. */
    const std::set<u32> &getRelateCKeys(u32 lkey);

    /** \brief Renumber lkey for logical operations, after parsed
     * all logical expressions. */
    void logicalKeyRenumber();

    /** \brief Used in Rose for writing bytecode. */
    const std::vector<LogicalOp> &getLogicalTree() const;

    /** \brief Used in Rose for writing bytecode. */
    const std::vector<CombInfo> &getCombInfoMap() const;

    /** \brief Fetch the dedupe key associated with the given report. Returns
     * ~0U if no dkey is needed. */
    u32 getDkey(const Report &r) const;

    /** \brief Register a Rose program offset with the given report. */
    void setProgramOffset(ReportID id, u32 programOffset);

    /** \brief Fetch the program offset for a given report. It is a fatal error
     * for this to be called with a report for which no program offset has been
     * set. */
    u32 getProgramOffset(ReportID id) const;

    /** \brief Parsed logical combination structure. */
    ParsedLogical pl;

private:
    /** \brief Grey box ref, for checking resource limits. */
    const Grey &grey;

    /** \brief Report structures, indexed by ID. */
    std::vector<Report> reportIds;

    /** \brief Mapping from Report to ID (inverse of \ref reportIds
     * vector). */
    std::unordered_map<Report, size_t> reportIdToInternalMap;

    /** \brief Mapping from ReportID to dedupe key. */
    std::unordered_map<ReportID, u32> reportIdToDedupeKey;

    /** \brief Mapping from ReportID to Rose program offset in bytecode. */
    std::unordered_map<ReportID, u32> reportIdToProgramOffset;

    /** \brief Mapping from external match ids to information about that
     * id. */
    std::unordered_map<ReportID, external_report_info> externalIdMap;

    /** \brief Mapping from expression index to exhaustion key. */
    std::map<s64a, u32> toExhaustibleKeyMap;

    /** \brief Unallocated expression index, used for \ref
     * getUnassociatedExhaustibleKey.
     *
     * TODO: work out why this is signed.
     */
    s64a freeEIndex;

    /** \brief True if database is globally exhaustible (all patterns must be
     * highlander for this to be the case). */
    bool global_exhaust;
};

std::set<u32> reportsToEkeys(const std::set<ReportID> &reports,
                             const ReportManager &rm);

} // namespace ue2

#endif
