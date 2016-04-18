/*
 * Copyright (c) 2016, Intel Corporation
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

#include "goughcompile.h"
#include "goughcompile_util.h"
#include "mcclellancompile_util.h"
#include "util/report_manager.h"

#include "ue2common.h"

using namespace std;
using namespace ue2;

namespace ue2 {

static
void remapReportsToPrograms(set<som_report> &reports,
                            const ReportManager &rm) {
    if (reports.empty()) {
        return;
    }
    auto old_reports = reports;
    reports.clear();
    for (const auto &r : old_reports) {
        u32 program = rm.getProgramOffset(r.report);
        reports.emplace(program, r.slot);
    }
}

void remapReportsToPrograms(raw_som_dfa &haig, const ReportManager &rm) {
    DEBUG_PRINTF("remap haig reports\n");

    for (auto &ds : haig.state_som) {
        remapReportsToPrograms(ds.reports, rm);
        remapReportsToPrograms(ds.reports_eod, rm);
    }

    // McClellan-style reports too.
    raw_dfa &rdfa = haig;
    remapReportsToPrograms(rdfa, rm);
}

} // namespace ue2
