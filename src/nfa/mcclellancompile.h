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

#ifndef MCCLELLANCOMPILE_H
#define MCCLELLANCOMPILE_H

#include "accel_dfa_build_strat.h"
#include "rdfa.h"
#include "ue2common.h"
#include "util/bytecode_ptr.h"

#include <memory>
#include <vector>
#include <set>

struct NFA;

namespace ue2 {

class ReportManager;
struct CompileContext;

class mcclellan_build_strat : public accel_dfa_build_strat {
public:
    mcclellan_build_strat(raw_dfa &rdfa_in, const ReportManager &rm_in,
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
    DfaType getType() const override { return McClellan; }

private:
    raw_dfa &rdfa;
};

/**
 * \brief Construct an implementation DFA.
 *
 * \param raw the raw dfa to construct from
 * \param cc compile context
 * \param rm report manger
 * \param only_accel_init if true, only the init states will be examined for
 *        acceleration opportunities
 * \param trust_daddy_states if true, trust the daddy state set in the raw dfa
 *        rather than conducting a search for a better daddy (for Sherman
 *        states)
 * \param accel_states (optional) success, is filled with the set of
 *        accelerable states
 */
bytecode_ptr<NFA>
mcclellanCompile(raw_dfa &raw, const CompileContext &cc,
                 const ReportManager &rm, bool only_accel_init,
                 bool trust_daddy_states = false,
                 std::set<dstate_id_t> *accel_states = nullptr);

/* used internally by mcclellan/haig/gough compile process */
bytecode_ptr<NFA>
mcclellanCompile_i(raw_dfa &raw, accel_dfa_build_strat &strat,
                   const CompileContext &cc, bool trust_daddy_states = false,
                   std::set<dstate_id_t> *accel_states = nullptr);

/**
 * \brief Returns the width of the character reach at start.
 */
u32 mcclellanStartReachSize(const raw_dfa *raw);

std::set<ReportID> all_reports(const raw_dfa &rdfa);

bool has_accel_mcclellan(const NFA *nfa);

} // namespace ue2

#endif // MCCLELLANCOMPILE_H
