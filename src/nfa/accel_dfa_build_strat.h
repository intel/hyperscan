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

#ifndef ACCEL_DFA_BUILD_STRAT_H
#define ACCEL_DFA_BUILD_STRAT_H

#include "rdfa.h"
#include "dfa_build_strat.h"
#include "ue2common.h"
#include "util/accel_scheme.h"

#include <map>

namespace ue2 {

class ReportManager;
struct Grey;
enum DfaType {
    McClellan,
    Sheng,
    Gough
};

class accel_dfa_build_strat : public dfa_build_strat {
public:
    accel_dfa_build_strat(const ReportManager &rm_in, bool only_accel_init_in)
        : dfa_build_strat(rm_in), only_accel_init(only_accel_init_in) {}
    virtual AccelScheme find_escape_strings(dstate_id_t this_idx) const;
    virtual size_t accelSize(void) const = 0;
    virtual u32 max_allowed_offset_accel() const = 0;
    virtual u32 max_stop_char() const = 0;
    virtual u32 max_floating_stop_char() const = 0;
    virtual void buildAccel(dstate_id_t this_idx, const AccelScheme &info,
                            void *accel_out);
    virtual std::map<dstate_id_t, AccelScheme> getAccelInfo(const Grey &grey);
    virtual DfaType getType() const = 0;

private:
    bool only_accel_init;
};

} // namespace ue2

#endif // ACCEL_DFA_BUILD_STRAT_H
