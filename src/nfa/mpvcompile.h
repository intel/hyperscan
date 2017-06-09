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

#ifndef MPV_COMPILE_H
#define MPV_COMPILE_H

#include "ue2common.h"
#include "util/bytecode_ptr.h"
#include "util/charreach.h"

#include <memory>
#include <vector>

struct NFA;

namespace ue2 {

class ReportManager;

struct raw_puff {
    raw_puff(u32 repeats_in, bool unbounded_in, ReportID report_in,
             const CharReach &reach_in, bool auto_restart_in = false,
             bool simple_exhaust_in = false)
        : repeats(repeats_in), unbounded(unbounded_in),
          auto_restart(auto_restart_in), simple_exhaust(simple_exhaust_in),
          report(report_in), reach(reach_in) {}
    u32 repeats; /**< report match after this many matching bytes */
    bool unbounded; /**< keep producing matches after repeats are reached */
    bool auto_restart; /**< for /[^X]{n}/ type patterns */
    bool simple_exhaust; /* first report will exhaust us */
    ReportID report;
    CharReach reach; /**< = ~escapes */
};

/*
 * puffs in the triggered_puffs vector are enabled when an TOP_N event is
 * delivered corresponding to their index in the vector
 */
bytecode_ptr<NFA> mpvCompile(const std::vector<raw_puff> &puffs,
                             const std::vector<raw_puff> &triggered_puffs,
                             const ReportManager &rm);

} // namespace ue2

#endif
