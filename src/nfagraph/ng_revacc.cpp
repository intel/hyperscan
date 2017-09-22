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
 * \brief Reverse acceleration analysis.
 */
#include "ng_revacc.h"

#include "grey.h"
#include "ng_holder.h"
#include "ue2common.h"
#include "nfa/accel.h"
#include "nfa/nfa_internal.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/graph_range.h"

#include <set>

using namespace std;

namespace ue2 {

static
bool isPseudoNoCaseChar(const CharReach &cr) {
    return cr.count() == 2 && !(cr.find_first() & 32)
        && cr.test(cr.find_first() | 32);
}

static
bool lookForEodSchemes(const RevAccInfo &rev_info, const u32 minWidth,
                       NFA *nfa) {
    DEBUG_PRINTF("pure eod triggered pattern\n");

    /* 2 char */
    for (u8 nocase = 0; nocase < 2; nocase++) {
        for (u8 i = 1; i < MAX_RACCEL_OFFSET; i++) {
            const CharReach &cr = rev_info.acceptEodReach[i];
            const CharReach &cr2 = rev_info.acceptEodReach[i - 1];

            if (!nocase && cr.count() == 1 && cr2.count() == 1) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    goto single;
                }
                nfa->rAccelType = ACCEL_RDEOD;
                nfa->rAccelData.array[0] = (u8)cr.find_first();
                nfa->rAccelData.array[1] = (u8)cr2.find_first();
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel eod x2 %u %04hx\n",
                             nfa->rAccelOffset, nfa->rAccelData.dc);
                return true;
            } else if (nocase && (cr.count() == 1 || isPseudoNoCaseChar(cr))
                       && (cr2.count() == 1 || isPseudoNoCaseChar(cr2))) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    goto single;
                }
                nfa->rAccelType = ACCEL_RDEOD_NOCASE;
                nfa->rAccelData.array[0] = (u8)cr.find_first() & CASE_CLEAR;  /* uppercase */
                nfa->rAccelData.array[1] = (u8)cr2.find_first() & CASE_CLEAR;
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel nc eod x2 %u %04hx\n",
                             nfa->rAccelOffset, nfa->rAccelData.dc);
                return true;
            }
        }
    }

 single:
    /* 1 char */
    for (u8 nocase = 0; nocase < 2; nocase++) {
        for (u8 i = 0; i < MAX_RACCEL_OFFSET; i++) {
            const CharReach &cr = rev_info.acceptEodReach[i];
            if (!nocase && cr.count() == 1) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    return false;
                }
                nfa->rAccelType = ACCEL_REOD;
                nfa->rAccelData.c = (u8) cr.find_first();
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel eod %u %02hhx\n",
                             nfa->rAccelOffset, nfa->rAccelData.c);
                return true;
            } else if (nocase && isPseudoNoCaseChar(cr)) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    return false;
                }
                nfa->rAccelType = ACCEL_REOD_NOCASE;
                nfa->rAccelData.c = (u8)cr.find_first(); /* uppercase */
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel nc eod %u %02hhx\n",
                             nfa->rAccelOffset, nfa->rAccelData.c);
                return true;
            }
        }
    }

    return false;
}

static
bool lookForFloatingSchemes(const RevAccInfo &rev_info,
                            const u32 minWidth, NFA *nfa) {
    /* 2 char */
    for (u8 nocase = 0; nocase < 2; nocase++) {
        for (u8 i = 1; i < MAX_RACCEL_OFFSET; i++) {
            CharReach cr = rev_info.acceptEodReach[i] | rev_info.acceptReach[i];
            CharReach cr2 = rev_info.acceptEodReach[i - 1]
                            | rev_info.acceptReach[i - 1];
            if (!nocase && cr.count() == 1 && cr2.count() == 1) {
                assert((u8)(i - 1) < minWidth);
                if (i > minWidth) {
                    goto single;
                }
                nfa->rAccelType = ACCEL_RDVERM;
                nfa->rAccelData.array[0] = (u8)cr.find_first();
                nfa->rAccelData.array[1] = (u8)cr2.find_first();
                nfa->rAccelOffset = i;
                DEBUG_PRINTF("raccel dverm %u %02hhx%02hhx\n",
                             nfa->rAccelOffset, nfa->rAccelData.array[0],
                             nfa->rAccelData.array[1]);
                return true;
            } else if (nocase && (cr.count() == 1 || isPseudoNoCaseChar(cr))
                        && (cr2.count() == 1 || isPseudoNoCaseChar(cr2))) {
                assert((u8)(i - 1) < minWidth);
                if (i > minWidth) {
                    goto single;
                }
                nfa->rAccelType = ACCEL_RDVERM_NOCASE;
                nfa->rAccelData.array[0] = (u8)cr.find_first() & CASE_CLEAR;
                nfa->rAccelData.array[1] = (u8)cr2.find_first() & CASE_CLEAR;
                nfa->rAccelOffset = i;
                DEBUG_PRINTF("raccel dverm %u %02hhx%02hhx nc\n",
                             nfa->rAccelOffset, nfa->rAccelData.array[0],
                             nfa->rAccelData.array[1]);
                return true;
            }
        }
    }

 single:
    /* 1 char */
    for (u8 nocase = 0; nocase < 2; nocase++) {
        for (u8 i = 0; i < MAX_RACCEL_OFFSET; i++) {
            CharReach cr = rev_info.acceptEodReach[i] | rev_info.acceptReach[i];
            if (!nocase && cr.count() == 1) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    return false;
                }
                nfa->rAccelType = ACCEL_RVERM;
                nfa->rAccelData.c = (u8)cr.find_first();
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel verm %u %02hhx\n", nfa->rAccelOffset,
                             nfa->rAccelData.c);
                return true;
            } else if (nocase && isPseudoNoCaseChar(cr)) {
                assert(i < minWidth);
                if (i >= minWidth) {
                    return false;
                }
                nfa->rAccelType = ACCEL_RVERM_NOCASE;
                nfa->rAccelData.c = (u8)cr.find_first(); /* 'uppercase' char */
                nfa->rAccelOffset = i + 1;
                DEBUG_PRINTF("raccel nc verm %u %02hhx\n", nfa->rAccelOffset,
                             nfa->rAccelData.c);
                return true;
            }
        }
    }

    return false;
}

void buildReverseAcceleration(NFA *nfa, const RevAccInfo &rev_info,
                              u32 min_width, bool eod_only) {
    assert(nfa);

    if (!rev_info.valid) {
        return;
    }

    nfa->rAccelOffset = 1;

    assert(rev_info.acceptReach[0].any() || rev_info.acceptEodReach[0].any());
    if (rev_info.acceptReach[0].none() && rev_info.acceptEodReach[0].none()) {
        DEBUG_PRINTF("expected path to accept\n");
        return;
    }

    if (rev_info.acceptReach[0].none()) {
        /* eod only */

        if (lookForEodSchemes(rev_info, min_width, nfa)) {
            assert(nfa->rAccelOffset <= min_width);
            return;
        }
    }

    if (eod_only) {
        return;
    }

    if (!lookForFloatingSchemes(rev_info, min_width, nfa)) {
        DEBUG_PRINTF("failed to accelerate\n");
    }
}

static
void populateRevAccelInfo(const NGHolder &g, NFAVertex terminal,
                          vector<CharReach> *reach) {
    set<NFAVertex> vset;

    for (auto v : inv_adjacent_vertices_range(terminal, g)) {
        if (!is_special(v, g)) {
            vset.insert(v);
        }
    }

    for (u8 offset = 0; offset < MAX_RACCEL_OFFSET; offset++) {
        set<NFAVertex> next;

        for (auto v : vset) {
            const CharReach &cr = g[v].char_reach;
            (*reach)[offset] |= cr;

            DEBUG_PRINTF("off %u adding %zu to %zu\n", offset, cr.count(),
                         (*reach)[offset].count());

            for (auto u : inv_adjacent_vertices_range(v, g)) {
                if (u == g.start || u == g.startDs) {
                    /* kill all subsequent offsets by setting to dot, setting
                     * to dot is in someways not accurate as there may be no
                     * data at all but neither case can be accelerated */
                    for (u8 i = offset + 1; i < MAX_RACCEL_OFFSET; i++) {
                        (*reach)[i].setall();
                    }
                    break;
                } else if (!is_special(u, g)) {
                    next.insert(u);
                }
            }
        }

        swap(vset, next);
    }
}

void populateReverseAccelerationInfo(RevAccInfo &rai, const NGHolder &g) {
    DEBUG_PRINTF("pop rev info\n");
    populateRevAccelInfo(g, g.accept, &rai.acceptReach);
    populateRevAccelInfo(g, g.acceptEod, &rai.acceptEodReach);
    rai.valid = true;
}

void mergeReverseAccelerationInfo(RevAccInfo &dest, const RevAccInfo &vic) {
    DEBUG_PRINTF("merging ra\n");

    dest.valid &= vic.valid;

    for (u8 i = 0; i < MAX_RACCEL_OFFSET; i++) {
        dest.acceptReach[i]    |= vic.acceptReach[i];
        dest.acceptEodReach[i] |= vic.acceptEodReach[i];
    }
}

RevAccInfo::RevAccInfo(void)
    : valid(false), acceptReach(MAX_RACCEL_OFFSET),
      acceptEodReach(MAX_RACCEL_OFFSET) {}

} // namespace ue2
