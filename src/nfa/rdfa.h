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

#ifndef RDFA_H
#define RDFA_H

#include "nfa_kind.h"
#include "ue2common.h"

#include "util/flat_containers.h"

#include <array>
#include <vector>

namespace ue2 {

typedef u16 dstate_id_t;
typedef u16 symbol_t;

static constexpr symbol_t TOP = 256;
static constexpr symbol_t ALPHABET_SIZE = 257;
static constexpr symbol_t N_SPECIAL_SYMBOL = 1;
static constexpr dstate_id_t DEAD_STATE = 0;

/** Structure representing a dfa state during construction. */
struct dstate {
    /** Next state; indexed by remapped sym */
    std::vector<dstate_id_t> next;

    /** Set by ng_mcclellan, refined by mcclellancompile */
    dstate_id_t daddy = 0;

    /** Set by mcclellancompile, implementation state id, excludes edge
     * decorations */
    dstate_id_t impl_id = 0;

    /** Reports to fire (at any location). */
    flat_set<ReportID> reports;

    /** Reports to fire (at EOD). */
    flat_set<ReportID> reports_eod;

    explicit dstate(size_t alphabet_size) : next(alphabet_size, 0) {}
};

struct raw_dfa {
    nfa_kind kind;
    std::vector<dstate> states;
    dstate_id_t start_anchored = DEAD_STATE;
    dstate_id_t start_floating = DEAD_STATE;
    u16 alpha_size = 0; /* including special symbols */

    /* mapping from input symbol --> equiv class id */
    std::array<u16, ALPHABET_SIZE> alpha_remap;

    explicit raw_dfa(nfa_kind k) : kind(k) {}
    virtual ~raw_dfa();

    u16 getImplAlphaSize() const { return alpha_size - N_SPECIAL_SYMBOL; }
    virtual void stripExtraEodReports(void);
    bool hasEodReports(void) const;
};

}

#endif
