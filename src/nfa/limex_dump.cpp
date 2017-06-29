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

#include "config.h"

#include "limex.h"

#include "accel.h"
#include "accel_dump.h"
#include "limex_internal.h"
#include "nfa_dump_internal.h"
#include "ue2common.h"
#include "util/charreach.h"
#include "util/dump_charclass.h"
#include "util/dump_mask.h"
#include "util/dump_util.h"

#include <algorithm>
#include <cstdio>
#include <cctype>
#include <sstream>
#include <vector>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

template<typename T> struct limex_traits {};
template<> struct limex_traits<LimExNFA512> {
    static const u32 size = 512;
    typedef NFAException512 exception_type;
};
template<> struct limex_traits<LimExNFA384> {
    static const u32 size = 384;
    typedef NFAException384 exception_type;
};
template<> struct limex_traits<LimExNFA256> {
    static const u32 size = 256;
    typedef NFAException256 exception_type;
};
template<> struct limex_traits<LimExNFA128> {
    static const u32 size = 128;
    typedef NFAException128 exception_type;
};
template<> struct limex_traits<LimExNFA64> {
    static const u32 size = 64;
    typedef NFAException64 exception_type;
};
template<> struct limex_traits<LimExNFA32> {
    static const u32 size = 32;
    typedef NFAException32 exception_type;
};

static
void dumpMask(FILE *f, const char *name, const u8 *mask, u32 mask_bits) {
    fprintf(f, "MSK %-20s %s\n", name, dumpMask(mask, mask_bits).c_str());
}

template<typename mask_t>
static
u32 rank_in_mask(const mask_t &mask, u32 bit) {
    assert(bit < 8 * sizeof(mask));

    u32 chunks[sizeof(mask)/sizeof(u32)];
    memcpy(chunks, &mask, sizeof(mask));
    u32 base_rank = 0;
    for (u32 i = 0; i < bit / 32; i++) {
        base_rank += popcount32(chunks[i]);
    }
    u32 chunk = chunks[bit / 32];
    u32 local_bit = bit % 32;
    assert(chunk & (1U << local_bit));
    return base_rank + popcount32(chunk & ((1U << local_bit) - 1));
}

template <typename limex_type>
static
void dumpRepeats(const limex_type *limex, u32 model_size, FILE *f) {
    fprintf(f, "\n");
    fprintf(f, "%u bounded repeats.\n", limex->repeatCount);

    const char *base = (const char *)limex;
    const u32 *repeatOffset = (const u32 *)(base + limex->repeatOffset);

    for (u32 i = 0; i < limex->repeatCount; i++) {
        const NFARepeatInfo *info =
            (const NFARepeatInfo *)(base + repeatOffset[i]);
        const RepeatInfo *repeat =
            (const RepeatInfo *)((const char *)info + sizeof(*info));
        fprintf(f, "  repeat %u: %s {%u,%u} packedCtrlSize=%u, "
                   "stateSize=%u\n",
                i, repeatTypeName(repeat->type), repeat->repeatMin,
                repeat->repeatMax, repeat->packedCtrlSize, repeat->stateSize);
        fprintf(f, "    nfa state: stream offset %u\n", info->stateOffset);
        fprintf(f, "    ");

        const u8 *tug_mask = (const u8 *)info + info->tugMaskOffset;
        dumpMask(f, "tugs", tug_mask, model_size);
    }

    fprintf(f, "\n");
}

static
void dumpLimexReachMasks(u32 model_size, const u8 *reach, u32 reachCount,
                         FILE *f) {
    for (u32 i = 0; i < reachCount; i++) {
        char tmp_common[100];
        const u8 *row = reach + (i * (model_size/8));
        sprintf(tmp_common, "reach mask %u ", i);
        dumpMask(f, tmp_common, row, model_size);
    }
}

static
void dumpLimexReachMap(const u8 *reachMap, FILE *f) {
    for (u32 i = 0; i < N_CHARS; i++) {
        fprintf(f, "reach 0x%02x ", i);
        if (!isprint(i)) {
            fprintf(f, "   ");
        } else {
            fprintf(f, "'%c'", (char)i);
        }
        fprintf(f, " -> mask %hhu\n", reachMap[i]);
    }
}

template<typename limex_type>
static
const NFA *limex_to_nfa(const limex_type *limex) {
    return (const NFA *)((const char *)limex - sizeof(NFA));
}

template<typename limex_type>
static
void dumpAccel(const limex_type *limex, FILE *f) {
    fprintf(f, "\n");
    fprintf(f, "%u acceleration schemes.\n", limex->accelCount);

    if (!limex->accelCount) {
        return;
    }

    u32 tableOffset = limex->accelTableOffset;
    u32 auxOffset = limex->accelAuxOffset;
    const u8 *accelTable = (const u8 *)((const char *)limex + tableOffset);
    const AccelAux *aux = (const AccelAux *)((const char *)limex + auxOffset);

    for (u32 i = 0; i < limex->accelCount; i++) {
        fprintf(f, "  accel %u (aux entry %u): ", i, accelTable[i]);
        dumpAccelInfo(f, aux[accelTable[i]]);
    }
}

static
void dumpAcceptList(const char *limex_base, const struct NFAAccept *accepts,
                    u32 acceptCount, FILE *f) {
    for (u32 i = 0; i < acceptCount; i++) {
        const NFAAccept &a = accepts[i];
        if (a.single_report) {
            fprintf(f, "  idx %u fires single report %u\n", i, a.reports);
            continue;
        }
        fprintf(f, "  idx %u fires report list %u:", i, a.reports);
        const ReportID *report = (const ReportID *)(limex_base + a.reports);
        for (; *report != MO_INVALID_IDX; report++) {
            fprintf(f, " %u", *report);
        }
        fprintf(f, "\n");
    }
}

template<typename limex_type>
static
void dumpAccepts(const limex_type *limex, FILE *f) {
    const char *limex_base = (const char *)limex;

    const u32 acceptCount = limex->acceptCount;
    const u32 acceptEodCount = limex->acceptEodCount;

    fprintf(f, "\n%u accepts.\n", acceptCount);
    const auto *accepts =
        (const struct NFAAccept *)(limex_base + limex->acceptOffset);
    dumpAcceptList(limex_base, accepts, acceptCount, f);
    fprintf(f, "\n%u accepts at EOD.\n", acceptEodCount);
    const auto *accepts_eod =
        (const struct NFAAccept *)(limex_base + limex->acceptEodOffset);
    dumpAcceptList(limex_base, accepts_eod, acceptEodCount, f);
    fprintf(f, "\n");
}

template<typename limex_type>
static
void dumpSquash(const limex_type *limex, FILE *f) {
    u32 size = limex_traits<limex_type>::size;

    // Dump squash masks, if there are any.
    const u8 *squashMask = (const u8 *)limex + limex->squashOffset;
    for (u32 i = 0; i < limex->squashCount; i++) {
        std::ostringstream name;
        name << "squash_" << i;
        dumpMask(f, name.str().c_str(), squashMask, size);
        squashMask += size / 8;
    }
}

template<typename limex_type>
static
const typename limex_traits<limex_type>::exception_type *
getExceptionTable(const limex_type *limex) {
    return (const typename limex_traits<limex_type>::exception_type *)
        ((const char *)limex + limex->exceptionOffset);
}

template<typename limex_type>
static
void dumpLimexExceptions(const limex_type *limex, FILE *f) {
    const typename limex_traits<limex_type>::exception_type *e =
                getExceptionTable(limex);
    const u32 size = limex_traits<limex_type>::size;

    const char *limex_base = (const char *)limex;

    fprintf(f, "\n");
    for (u32 i = 0; i < limex->exceptionCount; i++) {
        fprintf(f, "exception %u: hasSquash=%u, reports offset=%u\n",
                i, e[i].hasSquash, e[i].reports);
        switch (e[i].trigger) {
        case LIMEX_TRIGGER_TUG: fprintf(f, "  trigger: TUG\n"); break;
        case LIMEX_TRIGGER_POS: fprintf(f, "  trigger: POS\n"); break;
        default: break;
        }
        dumpMask(f, "succ", (const u8 *)&e[i].successors, size);
        dumpMask(f, "squash", (const u8 *)&e[i].squash, size);
        fprintf(f, "reports: ");
        if (e[i].reports == MO_INVALID_IDX) {
            fprintf(f, " <none>\n");
        } else {
            const ReportID *r = (const ReportID *)(limex_base + e[i].reports);
            while (*r != MO_INVALID_IDX) {
                fprintf(f, " %u", *r++);
            }
            fprintf(f, "\n");
        }

    }
}

template<typename limex_type>
static
void dumpLimexShifts(const limex_type *limex, FILE *f) {
    u32 size = limex_traits<limex_type>::size;
    fprintf(f, "Shift Masks:\n");
    for(u32 i = 0; i < limex->shiftCount; i++) {
        fprintf(f, "\t Shift %u(%hhu)\t\tMask: %s\n", i, limex->shiftAmount[i],
                dumpMask((const u8 *)&limex->shift[i], size).c_str());
    }
}
template<typename limex_type>
static
void dumpLimexText(const limex_type *limex, FILE *f) {
    u32 size = limex_traits<limex_type>::size;

    fprintf(f, "%u-bit LimEx NFA (%u shifts, %u exceptions)\n", size,
            limex->shiftCount, limex->exceptionCount);
    fprintf(f, "flags: ");
    if (limex->flags & LIMEX_FLAG_COMPRESS_STATE) {
        fprintf(f, "COMPRESS_STATE ");
    }
    if (limex->flags & LIMEX_FLAG_COMPRESS_MASKED) {
        fprintf(f, "COMPRESS_MASKED ");
    }
    if (limex->flags & LIMEX_FLAG_CANNOT_DIE) {
        fprintf(f, "CANNOT_DIE ");
    }
    fprintf(f, "\n\n");

    dumpMask(f, "init", (const u8 *)&limex->init, size);
    dumpMask(f, "init_dot_star", (const u8 *)&limex->initDS, size);
    dumpMask(f, "accept", (const u8 *)&limex->accept, size);
    dumpMask(f, "accept_at_eod", (const u8 *)&limex->acceptAtEOD, size);
    dumpMask(f, "accel", (const u8 *)&limex->accel, size);
    dumpMask(f, "accel_and_friends", (const u8 *)&limex->accel_and_friends,
             size);
    dumpMask(f, "compress_mask", (const u8 *)&limex->compressMask, size);
    dumpMask(f, "emask", (const u8 *)&limex->exceptionMask, size);
    dumpMask(f, "zombie", (const u8 *)&limex->zombieMask, size);

    // Dump top masks, if there are any.
    u32 topCount = limex->topCount;
    const u8 *topMask = (const u8 *)limex + limex->topOffset;
    for (u32 i = 0; i < topCount; i++) {
        std::ostringstream name;
        name << "top_" << i;
        dumpMask(f, name.str().c_str(), topMask, size);
        topMask += size / 8;
    }

    // Dump shift masks
    dumpLimexShifts(limex, f);

    dumpSquash(limex, f);

    dumpLimexReachMap(limex->reachMap, f);
    dumpLimexReachMasks(size, (const u8 *)limex + sizeof(*limex) /* reach*/,
                        limex->reachSize, f);

    dumpAccepts(limex, f);

    dumpLimexExceptions<limex_type>(limex, f);

    dumpAccel<limex_type>(limex, f);

    dumpRepeats(limex, size, f);
    dumpTextReverse(limex_to_nfa(limex), f);
}

static
bool testbit(const u8 *mask, UNUSED u32 mask_bits, u32 bit) {
    u32 byte = bit / 8;
    return mask[byte] & (1 << (bit % 8));
}

static
void setupReach(const u8 *reachMap, const u8 *reachBase, u32 size,
                u32 state_count, vector<CharReach> *perStateReach) {
    for (u32 i = 0; i < state_count; i++) {
        perStateReach->push_back(CharReach());
        for (u32 j = 0; j < N_CHARS; j++) {
            u8 k = reachMap[j];
            const u8 *r = reachBase + k * (size/8);
            if (testbit(r, size, i)) {
                perStateReach->back().set(j);
            }
        }
    }
}

namespace {
struct nfa_labeller {
    virtual ~nfa_labeller() {}
    virtual void label_state(FILE *f, u32 state) const = 0;
};

template<typename limex_type>
struct limex_labeller : public nfa_labeller {
    explicit limex_labeller(const limex_type *limex_in) : limex(limex_in) {}

    void label_state(FILE *f, u32 state) const override {
        const typename limex_traits<limex_type>::exception_type *exceptions
            = getExceptionTable(limex);
        if (!testbit((const u8 *)&limex->exceptionMask,
                     limex_traits<limex_type>::size, state)) {
            return;
        }

        u32 ex_index = rank_in_mask(limex->exceptionMask, state);
        const typename limex_traits<limex_type>::exception_type *e
            = &exceptions[ex_index];

        switch (e->trigger) {
        case LIMEX_TRIGGER_TUG: fprintf(f, "\\nTUG"); break;
        case LIMEX_TRIGGER_POS: fprintf(f, "\\nPOS"); break;
        default: break;
        }
    }

    const limex_type *limex;
};

}

template<typename limex_type>
static
void dumpVertexDotInfo(const limex_type *limex, u32 state_count, FILE *f,
                       const nfa_labeller &labeller) {
    u32 size = sizeof(limex->init) * 8;
    const u8 *reach = (const u8 *)limex + sizeof(*limex);
    vector<CharReach> perStateReach;
    setupReach(limex->reachMap, reach, size, state_count, &perStateReach);

    const u8 *topMask = (const u8 *)limex + limex->topOffset;

    for (u32 state = 0; state < state_count; state++) {
        fprintf(f, "%u [ width = 1, fixedsize = true, fontsize = 12, "
                "label = \"%u\\n", state, state);
        assert(perStateReach[state].any());
        describeClass(f, perStateReach[state], 5, CC_OUT_DOT);
        labeller.label_state(f, state);
        // bung in another couple lines to push char class (the widest thing) up a bit
        fprintf(f, "\\n\\n\" ];\n");

        if (testbit((const u8 *)&limex->acceptAtEOD, size, state)) {
            fprintf(f, "%u [ shape = box ];\n", state);
        } else if (testbit((const u8 *)&limex->accept, size, state)) {
            fprintf(f, "%u [ shape = doublecircle ];\n", state);
        }
        if (testbit((const u8 *)&limex->accel, size, state)) {
            fprintf(f, "%u [ color = red style = diagonals];\n", state);
        }
        if (testbit((const u8 *)&limex->init, size, state)) {
            fprintf(f, "START -> %u [ color = grey ];\n", state);
        }

        // vertex could be in a top mask.
        for (u32 i = 0; i < limex->topCount; i++) {
            const u8 *msk = topMask + i * (size / 8);
            if (testbit(msk, size, state)) {
                fprintf(f, "START -> %u [ color = grey, "
                           "label = \"TOP %u\" ];\n",
                        state, i);
            }
        }
    }
}

template<typename limex_type>
static
void dumpExDotInfo(const limex_type *limex, u32 state, FILE *f) {
    u32 size = limex_traits<limex_type>::size;
    if (!testbit((const u8 *)&limex->exceptionMask, size, state)) {
        return; /* not exceptional */
    }

    const typename limex_traits<limex_type>::exception_type *exceptions
        = getExceptionTable(limex);

    u32 ex_index = rank_in_mask(limex->exceptionMask, state);
    const typename limex_traits<limex_type>::exception_type *e
        = &exceptions[ex_index];

    u32 state_count = limex_to_nfa(limex)->nPositions;

    for (u32 j = 0; j < state_count; j++) {
        if (testbit((const u8 *)&e->successors, size, j)) {
            fprintf(f, "%u -> %u [color = blue];\n", state, j);
        }
        if (!testbit((const u8 *)&e->squash, size, j)) {
            fprintf(f, "%u -> %u [color = grey style = dashed];\n", state, j);
        }
    }
    if (e->trigger != LIMEX_TRIGGER_NONE) {
        fprintf(f, "%u [color = forestgreen];\n", state);
    } else {
        fprintf(f, "%u [color = blue];\n", state);
    }
}

template<typename limex_type>
static
void dumpLimDotInfo(const limex_type *limex, u32 state, FILE *f) {
    for (u32 j = 0; j < limex->shiftCount; j++) {
        const u32 shift_amount = limex->shiftAmount[j];
        if (testbit((const u8 *)&limex->shift[j],
                    limex_traits<limex_type>::size, state)) {
            fprintf(f, "%u -> %u;\n", state, state + shift_amount);
        }
    }
}

template<typename limex_type>
static
void dumpLimexDot(const NFA *nfa, const limex_type *limex, FILE *f) {
    dumpDotPreamble(f);
    u32 state_count = nfa->nPositions;
    dumpVertexDotInfo(limex, state_count, f, limex_labeller<limex_type>(limex));
    for (u32 i = 0; i < state_count; i++) {
        dumpLimDotInfo(limex, i, f);
        dumpExDotInfo(limex, i, f);
    }
    dumpDotTrailer(f);
}

#define LIMEX_DUMP_FN(size)                                                    \
    void nfaExecLimEx##size##_dump(const NFA *nfa, const string &base) {       \
        auto limex = (const LimExNFA##size *)getImplNfa(nfa);                  \
        dumpLimexText(limex, StdioFile(base + ".txt", "w"));                   \
        dumpLimexDot(nfa, limex, StdioFile(base + ".dot", "w"));               \
    }

LIMEX_DUMP_FN(32)
LIMEX_DUMP_FN(64)
LIMEX_DUMP_FN(128)
LIMEX_DUMP_FN(256)
LIMEX_DUMP_FN(384)
LIMEX_DUMP_FN(512)

} // namespace ue2
