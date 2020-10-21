/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#ifndef SHENGCOMPILE_H
#define SHENGCOMPILE_H

#include "accel_dfa_build_strat.h"
#include "rdfa.h"
#include "util/bytecode_ptr.h"
#include "util/charreach.h"
#include "util/flat_containers.h"

#include <memory>
#include <set>

struct NFA;

namespace ue2 {

class ReportManager;
struct CompileContext;
struct raw_dfa;

class sheng_build_strat : public accel_dfa_build_strat {
public:
    sheng_build_strat(raw_dfa &rdfa_in, const ReportManager &rm_in,
                      bool only_accel_init_in)
        : accel_dfa_build_strat(rm_in, only_accel_init_in), rdfa(rdfa_in) {}
    raw_dfa &get_raw() const override { return rdfa; }
    std::unique_ptr<raw_report_info> gatherReports(
                                  std::vector<u32> &reports /* out */,
                                  std::vector<u32> &reports_eod /* out */,
                                  u8 *isSingleReport /* out */,
                                  ReportID *arbReport /* out */) const override;
    size_t accelSize(void) const override;
    u32 max_allowed_offset_accel() const override;
    u32 max_stop_char() const override;
    u32 max_floating_stop_char() const override;
    DfaType getType() const override { return Sheng; }

private:
    raw_dfa &rdfa;
};

bytecode_ptr<NFA> shengCompile(raw_dfa &raw, const CompileContext &cc,
                               const ReportManager &rm, bool only_accel_init,
                               std::set<dstate_id_t> *accel_states = nullptr);

bytecode_ptr<NFA> sheng32Compile(raw_dfa &raw, const CompileContext &cc,
                                 const ReportManager &rm, bool only_accel_init,
                                 std::set<dstate_id_t> *accel_states = nullptr);

bytecode_ptr<NFA> sheng64Compile(raw_dfa &raw, const CompileContext &cc,
                                 const ReportManager &rm, bool only_accel_init,
                                 std::set<dstate_id_t> *accel_states = nullptr);

struct sheng_escape_info {
    CharReach outs;
    CharReach outs2_single;
    flat_set<std::pair<u8, u8>> outs2;
    bool outs2_broken = false;
};

bool has_accel_sheng(const NFA *nfa);

} // namespace ue2

#endif /* SHENGCOMPILE_H */
