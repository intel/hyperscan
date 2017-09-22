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
 * \brief Rose Build interface.
 *
 * Rose Build interface. Everything you ever needed to feed literals in and
 * get a RoseEngine out. This header should be everything needed by the rest
 * of UE2.
 */

#ifndef ROSE_BUILD_H
#define ROSE_BUILD_H

#include "ue2common.h"
#include "rose_common.h"
#include "rose_in_graph.h"
#include "util/bytecode_ptr.h"
#include "util/charreach.h"
#include "util/flat_containers.h"
#include "util/noncopyable.h"
#include "util/ue2string.h"

#include <memory>
#include <set>
#include <utility>
#include <vector>

struct NFA;
struct SmallWriteEngine;
struct RoseEngine;

namespace ue2 {

struct BoundaryReports;
struct CompileContext;
struct raw_puff;
struct raw_som_dfa;
class  CharReach;
class  NGHolder;
class  ReportManager;
class  SmallWriteBuild;
class  SomSlotManager;

class RoseDedupeAux {
public:
    virtual ~RoseDedupeAux();

    /** \brief True if we can not establish that at most a single callback will
     * be generated at a given offset from this set of reports. */
    virtual bool requiresDedupeSupport(const flat_set<ReportID> &reports)
        const = 0;
};

/** \brief Abstract interface intended for callers from elsewhere in the tree,
 * real underlying implementation is RoseBuildImpl in rose_build_impl.h. */
class RoseBuild : noncopyable {
public:
    virtual ~RoseBuild();

    /** \brief Adds a single literal. */
    virtual void add(bool anchored, bool eod, const ue2_literal &lit,
                     const flat_set<ReportID> &ids) = 0;

    virtual bool addRose(const RoseInGraph &ig, bool prefilter) = 0;
    virtual bool addSombeRose(const RoseInGraph &ig) = 0;

    virtual bool addOutfix(const NGHolder &h) = 0;
    virtual bool addOutfix(const NGHolder &h, const raw_som_dfa &haig) = 0;
    virtual bool addOutfix(const raw_puff &rp) = 0;

    virtual bool addChainTail(const raw_puff &rp, u32 *queue_out,
                              u32 *event_out) = 0;

    /** \brief Returns true if we were able to add it as a mask. */
    virtual bool add(bool anchored, const std::vector<CharReach> &mask,
                     const flat_set<ReportID> &reports) = 0;

    /** \brief Attempts to add the graph to the anchored acyclic table. Returns
     * true on success. */
    virtual bool addAnchoredAcyclic(const NGHolder &graph) = 0;

    virtual bool validateMask(const std::vector<CharReach> &mask,
                              const flat_set<ReportID> &reports,
                              bool anchored, bool eod) const = 0;
    virtual void addMask(const std::vector<CharReach> &mask,
                         const flat_set<ReportID> &reports, bool anchored,
                         bool eod) = 0;

    /** \brief Construct a runtime implementation. */
    virtual bytecode_ptr<RoseEngine> buildRose(u32 minWidth) = 0;

    virtual std::unique_ptr<RoseDedupeAux> generateDedupeAux() const = 0;

    /** Get a unique report identifier for a prefix|infix engine */
    virtual ReportID getNewNfaReport() = 0;

    /** Note that we have seen a SOM pattern. */
    virtual void setSom() = 0;
};

// Construct a usable Rose builder.
std::unique_ptr<RoseBuild> makeRoseBuilder(ReportManager &rm,
                                           SomSlotManager &ssm,
                                           SmallWriteBuild &smwr,
                                           const CompileContext &cc,
                                           const BoundaryReports &boundary);

bool roseCheckRose(const RoseInGraph &ig, bool prefilter,
                   const ReportManager &rm, const CompileContext &cc);

bool roseIsPureLiteral(const RoseEngine *t);

size_t maxOverlap(const ue2_literal &a, const ue2_literal &b, u32 b_delay);

} // namespace ue2

#endif // ROSE_BUILD_H
