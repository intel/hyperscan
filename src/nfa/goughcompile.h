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

#ifndef GOUGHCOMPILE_H
#define GOUGHCOMPILE_H

#include "mcclellancompile.h"
#include "nfa_kind.h"
#include "ue2common.h"
#include "util/bytecode_ptr.h"
#include "util/flat_containers.h"
#include "util/order_check.h"

#include <map>
#include <memory>
#include <set>
#include <vector>

namespace ue2 {

#define CREATE_NEW_SOM (~0U)

/* dest nfa state -> som info for dest state is min of provided loc idx som
 * info */
typedef flat_map<u32, std::vector<u32>> som_tran_info;

struct som_report {
    som_report(ReportID r, u32 s) : report(r), slot(s) {}

    ReportID report;
    u32 slot;

    bool operator<(const som_report &b) const {
        const som_report &a = *this;
        ORDER_CHECK(report);
        ORDER_CHECK(slot);
        return false;
    }
};

struct dstate_som {
    std::set<som_report> reports;
    std::set<som_report> reports_eod;
    som_tran_info preds; /* live nfa states mapped back to pred states */
};

struct raw_som_dfa : public raw_dfa {
    raw_som_dfa(nfa_kind k, bool unordered_som_triggers_in, u32 trigger,
                u32 stream_som_loc_width_in)
        : raw_dfa(k), stream_som_loc_width(stream_som_loc_width_in),
        unordered_som_triggers(unordered_som_triggers_in),
        trigger_nfa_state(trigger) {
        assert(!unordered_som_triggers || is_triggered(kind));
    }

    std::vector<dstate_som> state_som;
    u32 stream_som_loc_width;
    bool unordered_som_triggers;
    void stripExtraEodReports(void) override;

    std::map<u32, u32> new_som_nfa_states; /* map nfa vertex id -> offset */
    u32 trigger_nfa_state; /* for triggered cases, slot_id that contains a new
                            * som */
};

bytecode_ptr<NFA> goughCompile(raw_som_dfa &raw, u8 somPrecision,
                               const CompileContext &cc,
                               const ReportManager &rm);

} // namespace ue2

#endif // GOUGHCOMPILE_H
