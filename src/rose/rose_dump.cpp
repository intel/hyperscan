/*
 * Copyright (c) 2015, Intel Corporation
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

#include "hwlm/hwlm_build.h"
#include "hwlm/hwlm_dump.h"
#include "rose_build.h"
#include "rose_dump.h"
#include "rose_common.h"
#include "rose_internal.h"
#include "rose_program.h"
#include "hs_compile.h"
#include "ue2common.h"
#include "nfa/nfa_build_util.h"
#include "nfa/nfa_dump_api.h"
#include "nfa/nfa_internal.h"
#include "util/dump_charclass.h"
#include "util/multibit_internal.h"
#include "util/multibit.h"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <ostream>
#include <string>
#include <sstream>
#include <utility>

#ifndef DUMP_SUPPORT
#error No dump support!
#endif

using namespace std;

namespace ue2 {

namespace /* anonymous */ {

struct rose_off {
    explicit rose_off(u32 j) : i(j) {}
    string str(void) const;
    u32 i;
};

ostream &operator<< (ostream &o, const rose_off &to) {
    if (to.i == ROSE_BOUND_INF) {
        o << "inf";
    } else {
        o << to.i;
    }
    return o;
}

string rose_off::str(void) const {
    ostringstream out;
    out << *this;
    return out.str();
}

}

static
const void *loadFromByteCodeOffset(const RoseEngine *t, u32 offset) {
    if (!offset) {
        return nullptr;
    }

    const char *lt = (const char *)t + offset;
    return lt;
}

static
const void *getAnchoredMatcher(const RoseEngine *t) {
    return loadFromByteCodeOffset(t, t->amatcherOffset);
}

static
const HWLM *getFloatingMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->fmatcherOffset);
}

static
const HWLM *getEodMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->ematcherOffset);
}

static
const HWLM *getSmallBlockMatcher(const RoseEngine *t) {
    return (const HWLM *)loadFromByteCodeOffset(t, t->sbmatcherOffset);
}

static
u32 literalsWithDirectReports(const RoseEngine *t) {
    return t->totalNumLiterals - t->literalCount;
}

template<typename Predicate>
static
size_t literalsWithPredicate(const RoseEngine *t, Predicate pred) {
    const RoseLiteral *tl = getLiteralTable(t);
    const RoseLiteral *tl_end = tl + t->literalCount;

    return count_if(tl, tl_end, pred);
}

static
size_t literalsInGroups(const RoseEngine *t, u32 from, u32 to) {
    rose_group mask = ~((1ULL << from) - 1);
    if (to < 64) {
        mask &= ((1ULL << to) - 1);
    }

    return literalsWithPredicate(
        t, [&mask](const RoseLiteral &l) { return l.groups & mask; });
}

static
CharReach bitvectorToReach(const u8 *reach) {
    CharReach cr;

    for (size_t i = 0; i < 256; i++) {
        if (reach[i / 8] & (1U << (i % 8))) {
            cr.set(i);

        }
    }
    return cr;
}

static
void dumpLookaround(ofstream &os, const RoseEngine *t,
                    const ROSE_STRUCT_CHECK_LOOKAROUND *ri) {
    assert(ri);

    const u8 *base = (const u8 *)t;
    const s8 *look_base = (const s8 *)(base + t->lookaroundTableOffset);
    const u8 *reach_base = base + t->lookaroundReachOffset;

    const s8 *look = look_base + ri->index;
    const s8 *look_end = look + ri->count;
    const u8 *reach = reach_base + ri->index * REACH_BITVECTOR_LEN;

    os << "    contents:" << endl;

    for (; look < look_end; look++, reach += REACH_BITVECTOR_LEN) {
        os << "      " << std::setw(4) << std::setfill(' ') << int{*look}
           << ": ";
        describeClass(os, bitvectorToReach(reach), 1000, CC_OUT_TEXT);
        os << endl;
    }
}

#define PROGRAM_CASE(name)                                                     \
    case ROSE_INSTR_##name: {                                                  \
        os << "  " << std::setw(4) << std::setfill('0') << (pc - pc_base)      \
           << ": " #name " (" << (int)ROSE_INSTR_##name << ")" << endl;        \
        const auto *ri = (const struct ROSE_STRUCT_##name *)pc;

#define PROGRAM_NEXT_INSTRUCTION                                               \
    pc += ROUNDUP_N(sizeof(*ri), ROSE_INSTR_MIN_ALIGN);                        \
    break;                                                                     \
    }

static
void dumpProgram(ofstream &os, const RoseEngine *t, const char *pc) {
    const char *pc_base = pc;
    for (;;) {
        u8 code = *(const u8 *)pc;
        assert(code <= ROSE_INSTR_END);
        switch (code) {
            PROGRAM_CASE(ANCHORED_DELAY) {
                os << "    depth " << u32{ri->depth} << endl;
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
                os << "    done_jump +" << ri->done_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_DEPTH) {
                os << "    min_depth " << u32{ri->min_depth} << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_ONLY_EOD) {
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_BOUNDS) {
                os << "    min_bound " << ri->min_bound << endl;
                os << "    max_bound " << ri->max_bound << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_NOT_HANDLED) {
                os << "    key " << ri->key << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LOOKAROUND) {
                os << "    index " << ri->index << endl;
                os << "    count " << ri->count << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
                dumpLookaround(os, t, ri);
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(CHECK_LEFTFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    lag " << ri->lag << endl;
                os << "    report " << ri->report << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_ADJUST) {
                os << "    distance " << ri->distance << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SOM_LEFTFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    lag " << ri->lag << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_INFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    event " << ri->event << endl;
                os << "    cancel " << u32{ri->cancel} << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(TRIGGER_SUFFIX) {
                os << "    queue " << ri->queue << endl;
                os << "    event " << ri->event << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_CHAIN) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_EOD) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_INT) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(REPORT_SOM_KNOWN) {
                os << "    report " << ri->report << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_STATE) {
                os << "    depth " << u32{ri->depth} << endl;
                os << "    index " << ri->index << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SET_GROUPS) {
                os << "    groups 0x" << std::hex << ri->groups << std::dec
                   << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_BEGIN) {
                os << "    iter_offset " << ri->iter_offset << endl;
                os << "    jump_table " << ri->jump_table << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(SPARSE_ITER_NEXT) {
                os << "    iter_offset " << ri->iter_offset << endl;
                os << "    jump_table " << ri->jump_table << endl;
                os << "    state " << ri->state << endl;
                os << "    fail_jump +" << ri->fail_jump << endl;
            }
            PROGRAM_NEXT_INSTRUCTION

            PROGRAM_CASE(END) { return; }
            PROGRAM_NEXT_INSTRUCTION

        default:
            os << "  UNKNOWN (code " << int{code} << ")" << endl;
            os << "  <stopping>" << endl;
            return;
        }
    }
}

#undef PROGRAM_CASE
#undef PROGRAM_NEXT_INSTRUCTION

static
void dumpRoseLitPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);

    const RoseLiteral *lits = getLiteralTable(t);
    const char *base = (const char *)t;

    for (u32 i = 0; i < t->literalCount; i++) {
        const RoseLiteral *lit = &lits[i];
        os << "Literal " << i << endl;
        os << "---------------" << endl;

        if (lit->programOffset) {
            os << "Program @ " << lit->programOffset << ":" << endl;
            dumpProgram(os, t, base + lit->programOffset);
        } else {
            os << "<No Program>" << endl;
        }

        os << endl;
    }

    os.close();
}

static
void dumpRoseEodPrograms(const RoseEngine *t, const string &filename) {
    ofstream os(filename);
    const char *base = (const char *)t;

    os << "Unconditional EOD Program:" << endl;

    if (t->eodProgramOffset) {
        dumpProgram(os, t, base + t->eodProgramOffset);
        os << endl;
    } else {
        os << "<No EOD Program>" << endl;
    }

    os << "Sparse Iter EOD Program:" << endl;

    if (t->eodIterProgramOffset) {
        dumpProgram(os, t, base + t->eodIterProgramOffset);
    } else {
        os << "<No EOD Iter Program>" << endl;
    }

    os.close();
}

static
void dumpNfaNotes(ofstream &fout, const RoseEngine *t, const NFA *n) {
    const u32 qindex = n->queueIndex;

    if (qindex < t->outfixBeginQueue) {
        fout << "chained";
        return;
    }

    if (qindex < t->outfixEndQueue) {
        fout << "outfix";
        return;
    }

    const NfaInfo *nfa_info = getNfaInfoByQueue(t, qindex);
    const NFA *nfa = getNfaByInfo(t, nfa_info);

    if (nfa_info->eod) {
        fout << "eod ";
    }

    if (qindex < t->leftfixBeginQueue) {
        fout << "suffix";
        return;
    }

    const LeftNfaInfo *left = getLeftInfoByQueue(t, qindex);
    if (left->transient) {
        fout << "transient " << (u32)left->transient << " ";
    }
    if (left->infix) {
        fout << "infix";
        u32 maxQueueLen = left->maxQueueLen;
        if (maxQueueLen != (u32)(-1)) {
            fout << " maxqlen=" << maxQueueLen;
        }
    } else {
        fout << "prefix";
    }
    fout << " maxlag=" << left->maxLag;
    if (left->stopTable) {
        fout << " miracles";
    }
    if (left->countingMiracleOffset) {
        const RoseCountingMiracle *cm
            = (const RoseCountingMiracle *)((const char *)t
                                            + left->countingMiracleOffset);
        fout << " counting_miracle:" << (int)cm->count
             << (cm->shufti ? "s" : "v");
    }
    if (nfaSupportsZombie(nfa)) {
        fout << " zombie";
    }
    if (left->eod_check) {
        fout << " eod";
    }
}

static
void dumpComponentInfo(const RoseEngine *t, const string &base) {
    stringstream ss;
    ss << base << "rose_components.txt";
    ofstream fout(ss.str().c_str());

    fout << "Index  Offset\tEngine               \tStates S.State Bytes   Notes\n";

    for (u32 i = 0; i < t->queueCount; i++) {
        const NfaInfo *nfa_info = getNfaInfoByQueue(t, i);
        const NFA *n = getNfaByInfo(t, nfa_info);

        fout << left << setw(6) << i << " ";

        fout << left << ((const char *)n - (const char *)t) << "\t"; /* offset */

        fout << left << setw(16) << describe(*n) << "\t";

        fout << left << setw(6) << n->nPositions << " ";
        fout << left << setw(7) << n->streamStateSize << " ";
        fout << left << setw(7) << n->length << " ";

        dumpNfaNotes(fout, t, n);

        fout << endl;
    }
}

static
void dumpExhaust(const RoseEngine *t, const string &base) {
    stringstream sstxt;
    sstxt << base << "rose_exhaust.txt";
    FILE *f = fopen(sstxt.str().c_str(), "w");

    const NfaInfo *infos
        = (const NfaInfo *)((const char *)t + t->nfaInfoOffset);

    u32 queue_count = t->activeArrayCount;

    for (u32 i = 0; i < queue_count; ++i) {
        u32 ekey_offset = infos[i].ekeyListOffset;

        fprintf(f, "%u (%u):", i, ekey_offset);

        if (ekey_offset) {
            const u32 *ekeys = (const u32 *)((const char *)t + ekey_offset);
            while (1) {
                u32 e = *ekeys;
                ++ekeys;
                if (e == ~0U) {
                    break;
                }
                fprintf(f, " %u", e);
            }
        }

        fprintf(f, "\n");
    }

    fclose(f);
}

static
void dumpNfas(const RoseEngine *t, bool dump_raw, const string &base) {
    dumpExhaust(t, base);

    for (u32 i = 0; i < t->queueCount; i++) {
        const NfaInfo *nfa_info = getNfaInfoByQueue(t, i);
        const NFA *n = getNfaByInfo(t, nfa_info);

        stringstream sstxt, ssdot, ssraw;

        sstxt << base << "rose_nfa_" << i << ".txt";
        ssdot << base << "rose_nfa_" << i << ".dot";
        ssraw << base << "rose_nfa_" << i << ".raw";

        FILE *f;

        f = fopen(ssdot.str().c_str(), "w");
        nfaDumpDot(n, f);
        fclose(f);

        f = fopen(sstxt.str().c_str(), "w");
        nfaDumpText(n, f);
        fclose(f);

        if (dump_raw) {
            f = fopen(ssraw.str().c_str(), "w");
            fwrite(n, 1, n->length, f);
            fclose(f);
        }
    }
}

static
void dumpRevComponentInfo(const RoseEngine *t, const string &base) {
    stringstream ss;
    ss << base << "som_rev_components.txt";
    ofstream fout(ss.str().c_str());

    fout << "Index  Offset\tEngine               \tStates S.State Bytes   Notes\n";

    const char *tp = (const char *)t;
    const u32 *rev_offsets = (const u32 *)(tp + t->somRevOffsetOffset);

    for (u32 i = 0; i < t->somRevCount; i++) {
        u32 offset = rev_offsets[i];
        const NFA *n = (const NFA *)(tp + offset);

        fout << left << setw(6) << i << " ";

        fout << left << offset << "\t"; /* offset */

        fout << left << setw(16) << describe(*n) << "\t";

        fout << left << setw(6) << n->nPositions << " ";
        fout << left << setw(7) << n->streamStateSize << " ";
        fout << left << setw(7) << n->length << " ";

        dumpNfaNotes(fout, t, n);

        fout << endl;
    }
}

static
void dumpRevNfas(const RoseEngine *t, bool dump_raw, const string &base) {

    const char *tp = (const char *)t;
    const u32 *rev_offsets = (const u32 *)(tp + t->somRevOffsetOffset);

    for (u32 i = 0; i < t->somRevCount; i++) {
        const NFA *n = (const NFA *)(tp + rev_offsets[i]);

        stringstream sstxt, ssdot, ssraw;

        sstxt << base << "som_rev_nfa_" << i << ".txt";
        ssdot << base << "som_rev_nfa_" << i << ".dot";
        ssraw << base << "som_nfa_nfa_" << i << ".raw";

        FILE *f;

        f = fopen(ssdot.str().c_str(), "w");
        nfaDumpDot(n, f);
        fclose(f);

        f = fopen(sstxt.str().c_str(), "w");
        nfaDumpText(n, f);
        fclose(f);

        if (dump_raw) {
            f = fopen(ssraw.str().c_str(), "w");
            fwrite(n, 1, n->length, f);
            fclose(f);
        }
    }
}

static
void dumpAnchored(const RoseEngine *t, const string &base) {
    u32 i = 0;
    const anchored_matcher_info *curr
        = (const anchored_matcher_info *)getALiteralMatcher(t);

    while (curr) {
        const NFA *n = (const NFA *)((const char *)curr + sizeof(*curr));
        stringstream sstxt, ssdot;

        sstxt << base << "anchored_" << i << ".txt";
        ssdot << base << "anchored_" << i << ".dot";

        FILE *f;

        f = fopen(ssdot.str().c_str(), "w");
        nfaDumpDot(n, f);
        fclose(f);

        f = fopen(sstxt.str().c_str(), "w");
        nfaDumpText(n, f);
        fclose(f);

        curr = curr->next_offset ? (const anchored_matcher_info *)
            ((const char *)curr + curr->next_offset) : nullptr;
        i++;
    };
}

static
void dumpAnchoredStats(const void *atable, FILE *f) {
    assert(atable);

    u32 i = 0;
    const anchored_matcher_info *curr = (const anchored_matcher_info *)atable;

    while (curr) {
        const NFA *n = (const NFA *)((const char *)curr + sizeof(*curr));

        fprintf(f, "  NFA %u: %s, %u states (%u bytes)\n", i,
                describe(*n).c_str(), n->nPositions, n->length);

        curr = curr->next_offset ? (const anchored_matcher_info *)
            ((const char *)curr + curr->next_offset) : nullptr;
        i++;
    };

}

// Externally accessible functions

void roseDumpText(const RoseEngine *t, FILE *f) {
    if (!t) {
        fprintf(f, "<< no rose >>\n");
        return;
    }

    const void *atable = getAnchoredMatcher(t);
    const HWLM *ftable = getFloatingMatcher(t);
    const HWLM *etable = getEodMatcher(t);
    const HWLM *sbtable = getSmallBlockMatcher(t);

    fprintf(f, "Rose:\n\n");

    fprintf(f, "mode:                : ");
    switch(t->mode) {
    case HS_MODE_BLOCK:
        fprintf(f, "block");
        break;
    case HS_MODE_STREAM:
        fprintf(f, "streaming");
        break;
    case HS_MODE_VECTORED:
        fprintf(f, "vectored");
        break;
    }
    fprintf(f, "\n");

    fprintf(f, "properties           :");
    if (t->canExhaust) {
        fprintf(f, " canExhaust");
    }
    if (t->hasSom) {
        fprintf(f, " hasSom");
    }
    if (t->simpleCallback) {
        fprintf(f, " simpleCallback");
    }
    fprintf(f, "\n");

    fprintf(f, "dkey count           : %u\n", t->dkeyCount);
    fprintf(f, "som slot count       : %u\n", t->somLocationCount);
    fprintf(f, "som width            : %u bytes\n", t->somHorizon);
    fprintf(f, "rose count           : %u\n", t->roseCount);
    fprintf(f, "\n");

    fprintf(f, "total engine size    : %u bytes\n", t->size);
    fprintf(f, " - anchored matcher  : %u bytes over %u bytes\n", t->asize,
            t->anchoredDistance);
    fprintf(f, " - floating matcher  : %zu bytes%s",
            ftable ? hwlmSize(ftable) : 0, t->noFloatingRoots ? " (cond)":"");
    if (t->floatingMinDistance) {
        fprintf(f, " from %s bytes\n",
                rose_off(t->floatingMinDistance).str().c_str());
    }
    if (t->floatingDistance != ROSE_BOUND_INF && ftable) {
        fprintf(f, " over %u bytes\n", t->floatingDistance);
    } else {
        fprintf(f, "\n");
    }
    fprintf(f, " - eod-anch matcher  : %zu bytes over last %u bytes\n",
            etable ? hwlmSize(etable) : 0, t->ematcherRegionSize);
    fprintf(f, " - small-blk matcher : %zu bytes over %u bytes\n",
            sbtable ? hwlmSize(sbtable) : 0, t->smallBlockDistance);
    fprintf(f, " - literal table     : %zu bytes\n",
            t->literalCount * sizeof(RoseLiteral));
    fprintf(f, " - role state table  : %zu bytes\n",
            t->rolesWithStateCount * sizeof(u32));
    fprintf(f, " - nfa info table    : %u bytes\n",
            t->anchoredReportMapOffset - t->nfaInfoOffset);
    fprintf(f, " - lookaround table  : %u bytes\n",
            t->nfaInfoOffset - t->lookaroundTableOffset);
    fprintf(f, " - lookaround reach  : %u bytes\n",
            t->lookaroundTableOffset - t->lookaroundReachOffset);

    fprintf(f, "state space required : %u bytes\n", t->stateOffsets.end);
    fprintf(f, " - history buffer    : %u bytes (+1 for len)\n",
            t->historyRequired);
    fprintf(f, " - exhaustion vector : %u bytes\n", (t->ekeyCount + 7) / 8);
    fprintf(f, " - role state mmbit  : %u bytes\n", t->stateSize);
    fprintf(f, " - runtime state     : %zu bytes\n", sizeof(RoseRuntimeState));
    fprintf(f, " - floating matcher  : %u bytes\n", t->floatingStreamState);
    fprintf(f, " - active array      : %u bytes\n",
            mmbit_size(t->activeArrayCount));
    fprintf(f, " - active rose       : %u bytes\n",
            mmbit_size(t->activeLeftCount));
    fprintf(f, " - anchored state    : %u bytes\n", t->anchorStateSize);
    fprintf(f, " - nfa state         : %u bytes\n", t->nfaStateSize);
    fprintf(f, " - (trans. nfa state): %u bytes\n", t->tStateSize);
    fprintf(f, " - one whole bytes   : %u bytes\n",
            t->stateOffsets.anchorState - t->stateOffsets.leftfixLagTable);
    fprintf(f, " - groups            : %u bytes\n",
            t->stateOffsets.groups_size);
    fprintf(f, "\n");

    fprintf(f, "initial groups       : 0x%016llx\n", t->initialGroups);
    fprintf(f, "handled key count    : %u\n", t->handledKeyCount);
    fprintf(f, "\n");

    fprintf(f, "number of literals   : %u\n", t->totalNumLiterals);
    fprintf(f, " - delayed           : %u\n", t->delay_count);
    fprintf(f, " - direct report     : %u\n",
            literalsWithDirectReports(t));
    fprintf(f, " - that squash group : %zu\n",
            literalsWithPredicate(
                t, [](const RoseLiteral &l) { return l.squashesGroup != 0; }));
    fprintf(f, " - with benefits     : %u\n", t->nonbenefits_base_id);
    fprintf(f, " - with program      : %zu\n",
            literalsWithPredicate(
                t, [](const RoseLiteral &l) { return l.programOffset != 0; }));
    fprintf(f, " - in groups ::\n");
    fprintf(f, "   + weak            : %zu\n",
            literalsInGroups(t, 0, t->group_weak_end));
    fprintf(f, "   + general         : %zu\n",
            literalsInGroups(t, t->group_weak_end, sizeof(u64a) * 8));

    fprintf(f, "\n");
    fprintf(f, "  minWidth                    : %u\n", t->minWidth);
    fprintf(f, "  minWidthExcludingBoundaries : %u\n",
            t->minWidthExcludingBoundaries);
    fprintf(f, "  maxBiAnchoredWidth          : %s\n",
            rose_off(t->maxBiAnchoredWidth).str().c_str());
    fprintf(f, "  maxSafeAnchoredDROffset     : %s\n",
            rose_off(t->maxSafeAnchoredDROffset).str().c_str());
    fprintf(f, "  minFloatLitMatchOffset      : %s\n",
            rose_off(t->floatingMinLiteralMatchOffset).str().c_str());
    fprintf(f, "  delay_base_id               : %u\n", t->delay_base_id);
    fprintf(f, "  maxFloatingDelayedMatch     : %s\n",
            rose_off(t->maxFloatingDelayedMatch).str().c_str());

    if (atable) {
        fprintf(f, "\nAnchored literal matcher stats:\n\n");
        dumpAnchoredStats(atable, f);
    }

    if (ftable) {
        fprintf(f, "\nFloating literal matcher stats:\n\n");
        hwlmPrintStats(ftable, f);
    }

    if (etable) {
        fprintf(f, "\nEOD-anchored literal matcher stats:\n\n");
        hwlmPrintStats(etable, f);
    }

    if (sbtable) {
        fprintf(f, "\nSmall-block literal matcher stats:\n\n");
        hwlmPrintStats(sbtable, f);
    }
}

#define DUMP_U8(o, member)                                              \
    fprintf(f, "    %-32s: %hhu/%hhx\n", #member, o->member, o->member)
#define DUMP_U32(o, member)                                             \
    fprintf(f, "    %-32s: %u/%08x\n", #member, o->member, o->member)
#define DUMP_U64(o, member)                                             \
    fprintf(f, "    %-32s: %llu/%016llx\n", #member, o->member, o->member)

void roseDumpStructRaw(const RoseEngine *t, FILE *f) {
    fprintf(f, "struct RoseEngine {\n");
    DUMP_U8(t, hasFloatingDirectReports);
    DUMP_U8(t, noFloatingRoots);
    DUMP_U8(t, requiresEodCheck);
    DUMP_U8(t, hasOutfixesInSmallBlock);
    DUMP_U8(t, runtimeImpl);
    DUMP_U8(t, mpvTriggeredByLeaf);
    DUMP_U8(t, canExhaust);
    DUMP_U8(t, hasSom);
    DUMP_U8(t, somHorizon);
    DUMP_U8(t, simpleCallback);
    DUMP_U32(t, mode);
    DUMP_U32(t, historyRequired);
    DUMP_U32(t, ekeyCount);
    DUMP_U32(t, dkeyCount);
    DUMP_U32(t, invDkeyOffset);
    DUMP_U32(t, somLocationCount);
    DUMP_U32(t, rolesWithStateCount);
    DUMP_U32(t, stateSize);
    DUMP_U32(t, anchorStateSize);
    DUMP_U32(t, nfaStateSize);
    DUMP_U32(t, tStateSize);
    DUMP_U32(t, smallWriteOffset);
    DUMP_U32(t, amatcherOffset);
    DUMP_U32(t, ematcherOffset);
    DUMP_U32(t, fmatcherOffset);
    DUMP_U32(t, sbmatcherOffset);
    DUMP_U32(t, amatcherMinWidth);
    DUMP_U32(t, fmatcherMinWidth);
    DUMP_U32(t, eodmatcherMinWidth);
    DUMP_U32(t, amatcherMaxBiAnchoredWidth);
    DUMP_U32(t, fmatcherMaxBiAnchoredWidth);
    DUMP_U32(t, intReportOffset);
    DUMP_U32(t, intReportCount);
    DUMP_U32(t, literalOffset);
    DUMP_U32(t, literalCount);
    DUMP_U32(t, multidirectOffset);
    DUMP_U32(t, activeArrayCount);
    DUMP_U32(t, activeLeftCount);
    DUMP_U32(t, queueCount);
    DUMP_U32(t, handledKeyCount);
    DUMP_U32(t, leftOffset);
    DUMP_U32(t, roseCount);
    DUMP_U32(t, lookaroundTableOffset);
    DUMP_U32(t, lookaroundReachOffset);
    DUMP_U32(t, eodProgramOffset);
    DUMP_U32(t, eodIterProgramOffset);
    DUMP_U32(t, eodIterOffset);
    DUMP_U32(t, lastByteHistoryIterOffset);
    DUMP_U32(t, minWidth);
    DUMP_U32(t, minWidthExcludingBoundaries);
    DUMP_U32(t, maxBiAnchoredWidth);
    DUMP_U32(t, anchoredDistance);
    DUMP_U32(t, anchoredMinDistance);
    DUMP_U32(t, floatingDistance);
    DUMP_U32(t, floatingMinDistance);
    DUMP_U32(t, smallBlockDistance);
    DUMP_U32(t, maxSafeAnchoredDROffset);
    DUMP_U32(t, floatingMinLiteralMatchOffset);
    DUMP_U32(t, nfaInfoOffset);
    DUMP_U32(t, anchoredReportMapOffset);
    DUMP_U32(t, anchoredReportInverseMapOffset);
    DUMP_U64(t, initialGroups);
    DUMP_U32(t, size);
    DUMP_U32(t, anchoredMatches);
    DUMP_U32(t, delay_count);
    DUMP_U32(t, delay_slot_size);
    DUMP_U32(t, delay_base_id);
    DUMP_U32(t, anchored_count);
    DUMP_U32(t, anchored_base_id);
    DUMP_U32(t, nonbenefits_base_id);
    DUMP_U32(t, maxFloatingDelayedMatch);
    DUMP_U32(t, delayRebuildLength);
    DUMP_U32(t, stateOffsets.history);
    DUMP_U32(t, stateOffsets.exhausted);
    DUMP_U32(t, stateOffsets.activeLeafArray);
    DUMP_U32(t, stateOffsets.activeLeftArray);
    DUMP_U32(t, stateOffsets.activeLeftArray_size);
    DUMP_U32(t, stateOffsets.leftfixLagTable);
    DUMP_U32(t, stateOffsets.anchorState);
    DUMP_U32(t, stateOffsets.groups);
    DUMP_U32(t, stateOffsets.groups_size);
    DUMP_U32(t, stateOffsets.floatingMatcherState);
    DUMP_U32(t, stateOffsets.somLocation);
    DUMP_U32(t, stateOffsets.somValid);
    DUMP_U32(t, stateOffsets.somWritable);
    DUMP_U32(t, stateOffsets.end);
    DUMP_U32(t, boundary.reportEodOffset);
    DUMP_U32(t, boundary.reportZeroOffset);
    DUMP_U32(t, boundary.reportZeroEodOffset);
    DUMP_U32(t, totalNumLiterals);
    DUMP_U32(t, asize);
    DUMP_U32(t, outfixBeginQueue);
    DUMP_U32(t, outfixEndQueue);
    DUMP_U32(t, leftfixBeginQueue);
    DUMP_U32(t, initMpvNfa);
    DUMP_U32(t, rosePrefixCount);
    DUMP_U32(t, activeLeftIterOffset);
    DUMP_U32(t, ematcherRegionSize);
    DUMP_U32(t, literalBenefitsOffsets);
    DUMP_U32(t, somRevCount);
    DUMP_U32(t, somRevOffsetOffset);
    DUMP_U32(t, group_weak_end);
    DUMP_U32(t, floatingStreamState);
    fprintf(f, "}\n");
    fprintf(f, "sizeof(RoseEngine) = %zu\n", sizeof(RoseEngine));
}

void roseDumpComponents(const RoseEngine *t, bool dump_raw,
                        const string &base) {
    dumpComponentInfo(t, base);
    dumpNfas(t, dump_raw, base);
    dumpAnchored(t, base);
    dumpRevComponentInfo(t, base);
    dumpRevNfas(t, dump_raw, base);
    dumpRoseLitPrograms(t, base + "/rose_lit_programs.txt");
    dumpRoseEodPrograms(t, base + "/rose_eod_programs.txt");
}

void roseDumpInternals(const RoseEngine *t, const string &base) {
    if (!t) {
        DEBUG_PRINTF("no rose\n");
        return;
    }

    const void *atable = getAnchoredMatcher(t);
    const HWLM *ftable = getFloatingMatcher(t);
    const HWLM *etable = getEodMatcher(t);

    if (atable) {
        FILE *f = fopen((base + "/anchored.raw").c_str(), "w");
        if (f) {
            fwrite(atable, 1, t->asize, f);
            fclose(f);
        }
    }

    if (ftable) {
        FILE *f = fopen((base + "/floating.raw").c_str(), "w");
        if (f) {
            fwrite(ftable, 1, hwlmSize(ftable), f);
            fclose(f);
        }
    }

    if (etable) {
        FILE *f = fopen((base + "/eod.raw").c_str(), "w");
        if (f) {
            fwrite(etable, 1, hwlmSize(etable), f);
            fclose(f);
        }
    }

    FILE *f = fopen((base + "/rose.raw").c_str(), "w");
    assert(f);
    fwrite(t, 1, roseSize(t), f);
    fclose(f);

    f = fopen((base + "/rose_struct.txt").c_str(), "w");
    roseDumpStructRaw(t, f);
    fclose(f);

    roseDumpComponents(t, true, base);
}

} // namespace ue2
