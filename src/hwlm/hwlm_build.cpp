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

/** \file
 * \brief Hamster Wheel Literal Matcher: build code.
 */
#include "grey.h"
#include "hwlm.h"
#include "hwlm_build.h"
#include "hwlm_internal.h"
#include "noodle_engine.h"
#include "noodle_build.h"
#include "ue2common.h"
#include "fdr/fdr_compile.h"
#include "nfa/shufticompile.h"
#include "util/alloc.h"
#include "util/bitutils.h"
#include "util/charreach.h"
#include "util/compare.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/dump_charclass.h"
#include "util/target_info.h"
#include "util/ue2string.h"
#include "util/verify_types.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

using namespace std;

namespace ue2 {

static const unsigned int MAX_ACCEL_OFFSET = 16;
static const unsigned int MAX_SHUFTI_WIDTH = 240;

static
bool findDVerm(const vector<const hwlmLiteral *> &lits, AccelAux *aux) {
    const hwlmLiteral &first = *lits.front();

    struct candidate {
        candidate(void)
            : c1(0), c2(0), max_offset(0), b5insens(false), valid(false) {}
        candidate(const hwlmLiteral &base, u32 offset)
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
            const hwlmLiteral &lit = *lit_ptr;
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
            const hwlmLiteral &lit = *lit_ptr;
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
                    curr.max_offset = MAX(curr.max_offset, j);
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
bool findSVerm(const vector<const hwlmLiteral *> &lits, AccelAux *aux) {
    const hwlmLiteral &first = *lits.front();

    struct candidate {
        candidate(void)
            : c(0), max_offset(0), b5insens(false), valid(false) {}
        candidate(const hwlmLiteral &base, u32 offset)
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
            const hwlmLiteral &lit = *lit_ptr;
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
            const hwlmLiteral &lit = *lit_ptr;
            for (u32 j = 0; j < MIN(MAX_ACCEL_OFFSET, lit.s.length()); j++) {
                bool found = false;
                if (curr.b5insens) {
                    found = (curr.c & CASE_CLEAR) == (lit.s[j] & CASE_CLEAR);
                } else {
                    found = curr.c == lit.s[j];
                }

                if (found) {
                    curr.max_offset = MAX(curr.max_offset, j);
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
void filterLits(const vector<hwlmLiteral> &lits, hwlm_group_t expected_groups,
                vector<const hwlmLiteral *> *filtered_lits, u32 *min_len) {
    *min_len = MAX_ACCEL_OFFSET;

    for (const auto &lit : lits) {
        if (!(lit.groups & expected_groups)) {
            continue;
        }

        const size_t lit_len = lit.s.length();
        if (lit_len < *min_len) {
            *min_len = verify_u32(lit_len);
        }

        filtered_lits->push_back(&lit);

#ifdef DEBUG
        DEBUG_PRINTF("lit:");
        for (u32 i = 0; i < lit.s.length(); i++) {
            printf("%02hhx", lit.s[i]);
        }
        printf("\n");
#endif
    }
}

static
void findForwardAccelScheme(const vector<hwlmLiteral> &lits,
                            hwlm_group_t expected_groups, AccelAux *aux) {
    DEBUG_PRINTF("building accel expected=%016llx\n", expected_groups);
    u32 min_len = MAX_ACCEL_OFFSET;
    vector<const hwlmLiteral *> filtered_lits;

    filterLits(lits, expected_groups, &filtered_lits, &min_len);
    if (filtered_lits.empty()) {
        return;
    }

    if (findDVerm(filtered_lits, aux)
        || findSVerm(filtered_lits, aux)) {
        return;
    }

    vector<CharReach> reach(MAX_ACCEL_OFFSET, CharReach());
    for (const auto &lit : lits) {
        if (!(lit.groups & expected_groups)) {
            continue;
        }

        for (u32 i = 0; i < MAX_ACCEL_OFFSET && i < lit.s.length(); i++) {
            unsigned char c = lit.s[i];
            if (lit.nocase) {
                DEBUG_PRINTF("adding %02hhx to %u\n", mytoupper(c), i);
                DEBUG_PRINTF("adding %02hhx to %u\n", mytolower(c), i);
                reach[i].set(mytoupper(c));
                reach[i].set(mytolower(c));
            } else {
                DEBUG_PRINTF("adding %02hhx to %u\n", c, i);
                reach[i].set(c);
            }
        }
    }

    u32 min_count = ~0U;
    u32 min_offset = ~0U;
    for (u32 i = 0; i < min_len; i++) {
        size_t count = reach[i].count();
        DEBUG_PRINTF("offset %u is %s (reach %zu)\n", i,
                     describeClass(reach[i]).c_str(), count);
        if (count < min_count) {
            min_count = (u32)count;
            min_offset = i;
        }
    }
    assert(min_offset <= min_len);

    if (min_count > MAX_SHUFTI_WIDTH) {
        DEBUG_PRINTF("min shufti with %u chars is too wide\n", min_count);
        return;
    }

    const CharReach &cr = reach[min_offset];
    if (shuftiBuildMasks(cr, &aux->shufti.lo, &aux->shufti.hi) != -1) {
        DEBUG_PRINTF("built shufti for %s (%zu chars, offset %u)\n",
                     describeClass(cr).c_str(), cr.count(), min_offset);
        aux->shufti.accel_type = ACCEL_SHUFTI;
        aux->shufti.offset = verify_u8(min_offset);
        return;
    }

    DEBUG_PRINTF("fail\n");
}

static
void buildForwardAccel(HWLM *h, const vector<hwlmLiteral> &lits,
                       hwlm_group_t expected_groups) {
    findForwardAccelScheme(lits, expected_groups, &h->accel1);
    findForwardAccelScheme(lits, HWLM_ALL_GROUPS, &h->accel0);

    h->accel1_groups = expected_groups;
}

static
void dumpLits(UNUSED const vector<hwlmLiteral> &lits) {
#ifdef DEBUG
    DEBUG_PRINTF("building lit table for:\n");
    for (const auto &lit : lits) {
        printf("\t%u:%016llx %s%s\n", lit.id, lit.groups,
               escapeString(lit.s).c_str(), lit.nocase ? " (nc)" : "");
    }
#endif
}

#ifndef NDEBUG
// Called by an assertion.
static
bool everyoneHasGroups(const vector<hwlmLiteral> &lits) {
    for (const auto &lit : lits) {
        if (!lit.groups) {
            return false;
        }
    }
    return true;
}
#endif

static
bool isNoodleable(const vector<hwlmLiteral> &lits,
                  const hwlmStreamingControl *stream_control,
                  const CompileContext &cc) {
    if (!cc.grey.allowNoodle) {
        return false;
    }

    if (lits.size() != 1) {
        DEBUG_PRINTF("too many literals for noodle\n");
        return false;
    }

    if (stream_control) { // nullptr if in block mode
        if (lits.front().s.length() + 1 > stream_control->history_max) {
            DEBUG_PRINTF("length of %zu too long for history max %zu\n",
                         lits.front().s.length(),
                         stream_control->history_max);
            return false;
        }
    }

    if (!lits.front().msk.empty()) {
        DEBUG_PRINTF("noodle can't handle supplementary masks\n");
        return false;
    }

    return true;
}

aligned_unique_ptr<HWLM> hwlmBuild(const vector<hwlmLiteral> &lits,
                                   hwlmStreamingControl *stream_control,
                                   bool make_small, const CompileContext &cc,
                                   hwlm_group_t expected_groups) {
    assert(!lits.empty());
    dumpLits(lits);

    if (stream_control) {
        assert(stream_control->history_min <= stream_control->history_max);
    }

    // Check that we haven't exceeded the maximum number of literals.
    if (lits.size() > cc.grey.limitLiteralCount) {
        throw ResourceLimitError();
    }

    // Safety and resource limit checks.
    u64a total_chars = 0;
    for (const auto &lit : lits) {
        assert(!lit.s.empty());

        if (lit.s.length() > cc.grey.limitLiteralLength) {
            throw ResourceLimitError();
        }
        total_chars += lit.s.length();
        if (total_chars > cc.grey.limitLiteralMatcherChars) {
            throw ResourceLimitError();
        }

        // We do not allow the all-ones ID, as we reserve that for internal use
        // within literal matchers.
        if (lit.id == 0xffffffffu) {
            assert(!"reserved id 0xffffffff used");
            throw CompileError("Internal error.");
        }
    }

    u8 engType = 0;
    size_t engSize = 0;
    shared_ptr<void> eng;

    DEBUG_PRINTF("building table with %zu strings\n", lits.size());

    assert(everyoneHasGroups(lits));

    if (isNoodleable(lits, stream_control, cc)) {
        DEBUG_PRINTF("build noodle table\n");
        engType = HWLM_ENGINE_NOOD;
        const hwlmLiteral &lit = lits.front();
        auto noodle = noodBuildTable(lit);
        if (noodle) {
            engSize = noodSize(noodle.get());
        }
        if (stream_control) {
            // For now, a single literal still goes to noodle and asks
            // for a great big history
            stream_control->literal_history_required = lit.s.length() - 1;
            assert(stream_control->literal_history_required
                   <= stream_control->history_max);
            stream_control->literal_stream_state_required = 0;
        }
        eng = move(noodle);
    } else {
        DEBUG_PRINTF("building a new deal\n");
        engType = HWLM_ENGINE_FDR;
        auto fdr = fdrBuildTable(lits, make_small, cc.target_info, cc.grey,
                            stream_control);
        if (fdr) {
            engSize = fdrSize(fdr.get());
        }
        eng = move(fdr);
    }

    if (!eng) {
        return nullptr;
    }

    assert(engSize);
    if (engSize > cc.grey.limitLiteralMatcherSize) {
        throw ResourceLimitError();
    }

    auto h = aligned_zmalloc_unique<HWLM>(ROUNDUP_CL(sizeof(HWLM)) + engSize);

    h->type = engType;
    memcpy(HWLM_DATA(h.get()), eng.get(), engSize);

    if (engType == HWLM_ENGINE_FDR && cc.grey.hamsterAccelForward) {
        buildForwardAccel(h.get(), lits, expected_groups);
    }

    if (stream_control) {
        DEBUG_PRINTF("requires %zu (of max %zu) bytes of history\n",
                     stream_control->literal_history_required,
                     stream_control->history_max);
        assert(stream_control->literal_history_required
                    <= stream_control->history_max);
    }

    return h;
}

size_t hwlmSize(const HWLM *h) {
    size_t engSize = 0;

    switch (h->type) {
    case HWLM_ENGINE_NOOD:
        engSize = noodSize((const noodTable *)HWLM_C_DATA(h));
        break;
    case HWLM_ENGINE_FDR:
        engSize = fdrSize((const FDR *)HWLM_C_DATA(h));
        break;
    }

    if (!engSize) {
        return 0;
    }

    return engSize + ROUNDUP_CL(sizeof(*h));
}

size_t hwlmFloodProneSuffixLen(size_t numLiterals, const CompileContext &cc) {
    const size_t NO_LIMIT = ~(size_t)0;

    // NOTE: this function contains a number of magic numbers which are
    // conservative estimates of flood-proneness based on internal details of
    // the various literal engines that fall under the HWLM aegis. If you
    // change those engines, you might need to change this function too.

    DEBUG_PRINTF("%zu literals\n", numLiterals);

    if (cc.grey.allowNoodle && numLiterals <= 1) {
        DEBUG_PRINTF("noodle\n");
        return NO_LIMIT;
    }

    if (cc.grey.fdrAllowTeddy) {
        if (numLiterals <= 48) {
            DEBUG_PRINTF("teddy\n");
            return 3;
        }
        if (cc.target_info.has_avx2() && numLiterals <= 96) {
            DEBUG_PRINTF("avx2 teddy\n");
            return 3;
        }
    }

    // TODO: we had thought we could push this value up to 9, but it seems that
    // hurts performance on floods in some FDR models. Super-conservative for
    // now.
    DEBUG_PRINTF("fdr\n");
    return 3;
}

} // namespace ue2
