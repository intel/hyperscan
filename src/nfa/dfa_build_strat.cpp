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

#include "dfa_build_strat.h"

#include "accel.h"
#include "accelcompile.h"
#include "grey.h"
#include "mcclellan_internal.h"
#include "mcclellancompile_util.h"
#include "nfa_internal.h"
#include "shufticompile.h"
#include "trufflecompile.h"
#include "ue2common.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/container.h"
#include "util/make_unique.h"
#include "util/order_check.h"
#include "util/report_manager.h"
#include "util/ue2_containers.h"
#include "util/unaligned.h"
#include "util/verify_types.h"

#include <vector>

using namespace std;

namespace ue2 {

// prevent weak vtables for raw_report_info, dfa_build_strat and raw_dfa
raw_report_info::~raw_report_info() {
}

dfa_build_strat::~dfa_build_strat() {
}

raw_dfa::~raw_dfa() {
}

namespace {

struct raw_report_list {
    flat_set<ReportID> reports;

    raw_report_list(const flat_set<ReportID> &reports_in,
                    const ReportManager &rm, bool do_remap) {
        if (do_remap) {
            for (auto &id : reports_in) {
                reports.insert(rm.getProgramOffset(id));
            }
        } else {
            reports = reports_in;
        }
    }

    bool operator<(const raw_report_list &b) const {
        return reports < b.reports;
    }
};

struct raw_report_info_impl : public raw_report_info {
    vector<raw_report_list> rl;
    u32 getReportListSize() const override;
    size_t size() const override;
    void fillReportLists(NFA *n, size_t base_offset,
                         std::vector<u32> &ro /* out */) const override;
};
}

unique_ptr<raw_report_info>
dfa_build_strat::gatherReports(vector<u32> &reports, vector<u32> &reports_eod,
                               u8 *isSingleReport, ReportID *arbReport) const {
    auto &rdfa = get_raw();
    DEBUG_PRINTF("gathering reports\n");

    const bool remap_reports = has_managed_reports(rdfa.kind);

    auto ri = ue2::make_unique<raw_report_info_impl>();
    map<raw_report_list, u32> rev;

    for (const dstate &s : rdfa.states) {
        if (s.reports.empty()) {
            reports.push_back(MO_INVALID_IDX);
            continue;
        }

        raw_report_list rrl(s.reports, rm, remap_reports);
        DEBUG_PRINTF("non empty r\n");
        if (rev.find(rrl) != rev.end()) {
            reports.push_back(rev[rrl]);
        } else {
            DEBUG_PRINTF("adding to rl %zu\n", ri->size());
            rev[rrl] = ri->size();
            reports.push_back(ri->size());
            ri->rl.push_back(rrl);
        }
    }

    for (const dstate &s : rdfa.states) {
        if (s.reports_eod.empty()) {
            reports_eod.push_back(MO_INVALID_IDX);
            continue;
        }

        DEBUG_PRINTF("non empty r eod\n");
        raw_report_list rrl(s.reports_eod, rm, remap_reports);
        if (rev.find(rrl) != rev.end()) {
            reports_eod.push_back(rev[rrl]);
            continue;
        }

        DEBUG_PRINTF("adding to rl eod %zu\n", s.reports_eod.size());
        rev[rrl] = ri->size();
        reports_eod.push_back(ri->size());
        ri->rl.push_back(rrl);
    }

    assert(!ri->rl.empty()); /* all components should be able to generate
                                reports */
    if (!ri->rl.empty()) {
        *arbReport = *ri->rl.begin()->reports.begin();
    } else {
        *arbReport = 0;
    }

    /* if we have only a single report id generated from all accepts (not eod)
     * we can take some short cuts */
    set<ReportID> reps;

    for (u32 rl_index : reports) {
        if (rl_index == MO_INVALID_IDX) {
            continue;
        }
        assert(rl_index < ri->size());
        insert(&reps, ri->rl[rl_index].reports);
    }

    if (reps.size() == 1) {
        *isSingleReport = 1;
        *arbReport = *reps.begin();
        DEBUG_PRINTF("single -- %u\n", *arbReport);
    } else {
        *isSingleReport = 0;
    }

    return move(ri);
}

u32 raw_report_info_impl::getReportListSize() const {
    u32 rv = 0;

    for (const auto &reps : rl) {
        rv += sizeof(report_list);
        rv += sizeof(ReportID) * reps.reports.size();
    }

    return rv;
}

size_t raw_report_info_impl::size() const {
    return rl.size();
}

void raw_report_info_impl::fillReportLists(NFA *n, size_t base_offset,
                                           vector<u32> &ro) const {
    for (const auto &reps : rl) {
        ro.push_back(base_offset);

        report_list *p = (report_list *)((char *)n + base_offset);

        u32 i = 0;
        for (const ReportID report : reps.reports) {
            p->report[i++] = report;
        }
        p->count = verify_u32(reps.reports.size());

        base_offset += sizeof(report_list);
        base_offset += sizeof(ReportID) * reps.reports.size();
    }
}

} // namespace ue2
