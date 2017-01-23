/*
 * Copyright (c) 2017, Intel Corporation
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

#include "rose_build_lit_accel.h"

#include "grey.h"
#include "ue2common.h"
#include "hwlm/hwlm_build.h"
#include "hwlm/hwlm_internal.h"
#include "hwlm/hwlm_literal.h"
#include "nfa/accel.h"
#include "nfa/shufticompile.h"
#include "nfa/trufflecompile.h"
#include "util/compare.h"
#include "util/dump_charclass.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

using namespace std;

namespace ue2 {

static const unsigned int MAX_ACCEL_OFFSET = 16;
static const unsigned int MAX_SHUFTI_WIDTH = 240;

static
size_t mask_overhang(const AccelString &lit) {
    size_t msk_true_size = lit.msk.size();
    assert(msk_true_size <= HWLM_MASKLEN);
    assert(HWLM_MASKLEN <= MAX_ACCEL_OFFSET);
    for (u8 c : lit.msk) {
        if (!c) {
            msk_true_size--;
        } else {
            break;
        }
    }

    if (lit.s.length() >= msk_true_size) {
        return 0;
    }

    /* only short literals should be able to have a mask which overhangs */
    assert(lit.s.length() < MAX_ACCEL_OFFSET);
    return msk_true_size - lit.s.length();
}

static
bool findDVerm(const vector<const AccelString *> &lits, AccelAux *aux) {
    const AccelString &first = *lits.front();

    struct candidate {
        candidate(void)
            : c1(0), c2(0), max_offset(0), b5insens(false), valid(false) {}
        candidate(const AccelString &base, u32 offset)
            : c1(base.s[offset]), c2(base.s[offset + 1]), max_offset(0),
              b5insens(false), valid(true) {}
        char c1;
        char c2;
        u32 max_offset;
        bool b5insens;
        bool valid;

        bool operator>(const candidate &other) const {
            if (!valid) {
                return false;
            }

            if (!other.valid) {
                return true;
            }

            if (other.cdiffers() && !cdiffers()) {
                return false;
            }

            if (!other.cdiffers() && cdiffers()) {
                return true;
            }

            if (!other.b5insens && b5insens) {
                return false;
            }

            if (other.b5insens && !b5insens) {
                return true;
            }

            if (max_offset > other.max_offset) {
                return false;
            }

            return true;
        }

        bool cdiffers(void) const {
            if (!b5insens) {
                return c1 != c2;
            }
            return (c1 & CASE_CLEAR) != (c2 & CASE_CLEAR);
        }
    };

    candidate best;

    for (u32 i = 0; i < MIN(MAX_ACCEL_OFFSET, first.s.length()) - 1; i++) {
        candidate curr(first, i);

        /* check to see if this pair appears in each string */
        for (const auto &lit_ptr : lits) {
            const AccelString &lit = *lit_ptr;
            if (lit.nocase && (ourisalpha(curr.c1) || ourisalpha(curr.c2))) {
                curr.b5insens = true; /* no choice but to be case insensitive */
            }

            bool found = false;
            bool found_nc = false;
            for (u32 j = 0;
                 !found && j < MIN(MAX_ACCEL_OFFSET, lit.s.length()) - 1; j++) {
                found |= curr.c1 == lit.s[j] && curr.c2 == lit.s[j + 1];
                found_nc |= (curr.c1 & CASE_CLEAR) == (lit.s[j] & CASE_CLEAR)
                    && (curr.c2 & CASE_CLEAR) == (lit.s[j + 1] & CASE_CLEAR);

                if (curr.b5insens) {
                    found = found_nc;
                }
            }

            if (!curr.b5insens && !found && found_nc) {
                curr.b5insens = true;
                found = true;
            }

            if (!found) {
                goto next_candidate;
            }
        }

        /* check to find the max offset where this appears */
        for (const auto &lit_ptr : lits) {
            const AccelString &lit = *lit_ptr;
            for (u32 j = 0; j < MIN(MAX_ACCEL_OFFSET, lit.s.length()) - 1;
                 j++) {
                bool found = false;
                if (curr.b5insens) {
                    found = (curr.c1 & CASE_CLEAR) == (lit.s[j] & CASE_CLEAR)
                     && (curr.c2 & CASE_CLEAR) == (lit.s[j + 1] & CASE_CLEAR);
                } else {
                    found = curr.c1 == lit.s[j] && curr.c2 == lit.s[j + 1];
                }

                if (found) {
                    assert(j + mask_overhang(lit) <= MAX_ACCEL_OFFSET);
                    ENSURE_AT_LEAST(&curr.max_offset, j + mask_overhang(lit));
                    break;
                }
            }
        }

        if (curr > best) {
            best = curr;
        }

    next_candidate:;
    }

    if (!best.valid) {
        return false;
    }

    aux->dverm.offset = verify_u8(best.max_offset);

    if (!best.b5insens) {
        aux->dverm.accel_type = ACCEL_DVERM;
        aux->dverm.c1 = best.c1;
        aux->dverm.c2 = best.c2;
        DEBUG_PRINTF("built dverm for %02hhx%02hhx\n",
                     aux->dverm.c1, aux->dverm.c2);
    } else {
        aux->dverm.accel_type = ACCEL_DVERM_NOCASE;
        aux->dverm.c1 = best.c1 & CASE_CLEAR;
        aux->dverm.c2 = best.c2 & CASE_CLEAR;
        DEBUG_PRINTF("built dverm nc for %02hhx%02hhx\n",
                     aux->dverm.c1, aux->dverm.c2);
    }
    return true;
}

static
bool findSVerm(const vector<const AccelString *> &lits, AccelAux *aux) {
    const AccelString &first = *lits.front();

    struct candidate {
        candidate(void)
            : c(0), max_offset(0), b5insens(false), valid(false) {}
        candidate(const AccelString &base, u32 offset)
            : c(base.s[offset]), max_offset(0),
              b5insens(false), valid(true) {}
        char c;
        u32 max_offset;
        bool b5insens;
        bool valid;

        bool operator>(const candidate &other) const {
            if (!valid) {
                return false;
            }

            if (!other.valid) {
                return true;
            }

            if (!other.b5insens && b5insens) {
                return false;
            }

            if (other.b5insens && !b5insens) {
                return true;
            }

            if (max_offset > other.max_offset) {
                return false;
            }

            return true;
        }
    };

    candidate best;

    for (u32 i = 0; i < MIN(MAX_ACCEL_OFFSET, first.s.length()); i++) {
        candidate curr(first, i);

        /* check to see if this pair appears in each string */
        for (const auto &lit_ptr : lits) {
            const AccelString &lit = *lit_ptr;
            if (lit.nocase && ourisalpha(curr.c)) {
                curr.b5insens = true; /* no choice but to be case insensitive */
            }

            bool found = false;
            bool found_nc = false;
            for (u32 j = 0;
                 !found && j < MIN(MAX_ACCEL_OFFSET, lit.s.length()); j++) {
                found |= curr.c == lit.s[j];
                found_nc |= (curr.c & CASE_CLEAR) == (lit.s[j] & CASE_CLEAR);

                if (curr.b5insens) {
                    found = found_nc;
                }
            }

            if (!curr.b5insens && !found && found_nc) {
                curr.b5insens = true;
                found = true;
            }

            if (!found) {
                goto next_candidate;
            }
        }

        /* check to find the max offset where this appears */
        for (const auto &lit_ptr : lits) {
            const AccelString &lit = *lit_ptr;
            for (u32 j = 0; j < MIN(MAX_ACCEL_OFFSET, lit.s.length()); j++) {
                bool found = false;
                if (curr.b5insens) {
                    found = (curr.c & CASE_CLEAR) == (lit.s[j] & CASE_CLEAR);
                } else {
                    found = curr.c == lit.s[j];
                }

                if (found) {
                    assert(j + mask_overhang(lit) <= MAX_ACCEL_OFFSET);
                    ENSURE_AT_LEAST(&curr.max_offset, j + mask_overhang(lit));
                }
            }
        }

        if (curr > best) {
            best = curr;
        }

    next_candidate:;
    }

    if (!best.valid) {
        return false;
    }

    if (!best.b5insens) {
        aux->verm.accel_type = ACCEL_VERM;
        aux->verm.c = best.c;
        DEBUG_PRINTF("built verm for %02hhx\n", aux->verm.c);
    } else {
        aux->verm.accel_type = ACCEL_VERM_NOCASE;
        aux->verm.c = best.c & CASE_CLEAR;
        DEBUG_PRINTF("built verm nc for %02hhx\n", aux->verm.c);
    }
    aux->verm.offset = verify_u8(best.max_offset);

    return true;
}

static
void filterLits(const vector<AccelString> &lits, hwlm_group_t expected_groups,
                vector<const AccelString *> *filtered_lits, u32 *min_len) {
    *min_len = MAX_ACCEL_OFFSET;

    for (const auto &lit : lits) {
        if (!(lit.groups & expected_groups)) {
            continue;
        }

        const size_t lit_len = lit.s.length();
        if (lit_len < *min_len) {
            *min_len = verify_u32(lit_len);
        }

        DEBUG_PRINTF("lit: '%s', nocase=%d, groups=0x%llx\n",
                     escapeString(lit.s).c_str(), lit.nocase ? 1 : 0,
                     lit.groups);
        filtered_lits->push_back(&lit);
    }
}

static
bool litGuardedByCharReach(const CharReach &cr, const AccelString &lit,
                           u32 max_offset) {
    for (u32 i = 0; i <= max_offset && i < lit.s.length(); i++) {
         unsigned char c = lit.s[i];
         if (lit.nocase) {
             if (cr.test(mytoupper(c)) && cr.test(mytolower(c))) {
                 return true;
             }
         } else {
             if (cr.test(c)) {
                 return true;
             }
         }
    }

    return false;
}

static
void findForwardAccelScheme(const vector<AccelString> &lits,
                            hwlm_group_t expected_groups, AccelAux *aux) {
    DEBUG_PRINTF("building accel expected=%016llx\n", expected_groups);
    u32 min_len = MAX_ACCEL_OFFSET;
    vector<const AccelString *> filtered_lits;

    filterLits(lits, expected_groups, &filtered_lits, &min_len);
    if (filtered_lits.empty()) {
        return;
    }

    if (findDVerm(filtered_lits, aux)
        || findSVerm(filtered_lits, aux)) {
        return;
    }

    /* look for shufti/truffle */

    vector<CharReach> reach(MAX_ACCEL_OFFSET, CharReach());
    for (const auto &lit : lits) {
        if (!(lit.groups & expected_groups)) {
            continue;
        }

        u32 overhang = mask_overhang(lit);
        for (u32 i = 0; i < overhang; i++) {
            /* this offset overhangs the start of the real literal; look at the
             * msk/cmp */
            for (u32 j = 0; j < N_CHARS; j++) {
                if ((j & lit.msk[i]) == lit.cmp[i]) {
                    reach[i].set(j);
                }
            }
        }
        for (u32 i = overhang; i < MAX_ACCEL_OFFSET; i++) {
            CharReach &reach_i = reach[i];
            u32 i_effective = i - overhang;

            if (litGuardedByCharReach(reach_i, lit, i_effective)) {
                continue;
            }
            unsigned char c = i_effective < lit.s.length() ? lit.s[i_effective]
                                                           : lit.s.back();
            if (lit.nocase) {
                reach_i.set(mytoupper(c));
                reach_i.set(mytolower(c));
            } else {
                reach_i.set(c);
            }
        }
    }

    u32 min_count = ~0U;
    u32 min_offset = ~0U;
    for (u32 i = 0; i < MAX_ACCEL_OFFSET; i++) {
        size_t count = reach[i].count();
        DEBUG_PRINTF("offset %u is %s (reach %zu)\n", i,
                     describeClass(reach[i]).c_str(), count);
        if (count < min_count) {
            min_count = (u32)count;
            min_offset = i;
        }
    }

    if (min_count > MAX_SHUFTI_WIDTH) {
        DEBUG_PRINTF("FAIL: min shufti with %u chars is too wide\n", min_count);
        return;
    }

    const CharReach &cr = reach[min_offset];
    if (-1 !=
        shuftiBuildMasks(cr, (u8 *)&aux->shufti.lo, (u8 *)&aux->shufti.hi)) {
        DEBUG_PRINTF("built shufti for %s (%zu chars, offset %u)\n",
                     describeClass(cr).c_str(), cr.count(), min_offset);
        aux->shufti.accel_type = ACCEL_SHUFTI;
        aux->shufti.offset = verify_u8(min_offset);
        return;
    }

    truffleBuildMasks(cr, (u8 *)&aux->truffle.mask1, (u8 *)&aux->truffle.mask2);
    DEBUG_PRINTF("built truffle for %s (%zu chars, offset %u)\n",
                 describeClass(cr).c_str(), cr.count(), min_offset);
    aux->truffle.accel_type = ACCEL_TRUFFLE;
    aux->truffle.offset = verify_u8(min_offset);
}

void buildForwardAccel(HWLM *h, const vector<AccelString> &lits,
                       hwlm_group_t expected_groups) {
    findForwardAccelScheme(lits, expected_groups, &h->accel1);
    findForwardAccelScheme(lits, HWLM_ALL_GROUPS, &h->accel0);

    h->accel1_groups = expected_groups;
}

} // namespace ue2
