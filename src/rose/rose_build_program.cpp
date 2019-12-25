/*
 * Copyright (c) 2016-2019, Intel Corporation
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

#include "rose_build_program.h"

#include "rose_build_engine_blob.h"
#include "rose_build_instructions.h"
#include "rose_build_lookaround.h"
#include "rose_build_resources.h"
#include "nfa/nfa_api_queue.h"
#include "nfa/nfa_build_util.h"
#include "nfa/tamaramacompile.h"
#include "nfagraph/ng_util.h"
#include "util/charreach_util.h"
#include "util/container.h"
#include "util/compile_context.h"
#include "util/compile_error.h"
#include "util/report_manager.h"
#include "util/unordered.h"
#include "util/verify_types.h"

#include <boost/range/adaptor/map.hpp>

#include <algorithm>
#include <cstring>

using namespace std;
using boost::adaptors::map_values;
using boost::adaptors::map_keys;

namespace ue2 {

engine_info::engine_info(const NFA *nfa, bool trans)
    : type((NFAEngineType)nfa->type), accepts_eod(nfaAcceptsEod(nfa)),
      stream_size(nfa->streamStateSize),
      scratch_size(nfa->scratchStateSize),
      scratch_align(state_alignment(*nfa)),
      transient(trans) {
    assert(scratch_align);
}

left_build_info::left_build_info(u32 q, u32 l, u32 t, rose_group sm,
                                 const std::vector<u8> &stops, u32 max_ql,
                                 u8 cm_count, const CharReach &cm_cr)
    : queue(q), lag(l), transient(t), squash_mask(sm), stopAlphabet(stops),
      max_queuelen(max_ql), countingMiracleCount(cm_count),
      countingMiracleReach(cm_cr) {
}

left_build_info::left_build_info(const vector<vector<LookEntry>> &looks)
    : has_lookaround(true), lookaround(looks) {
}

using OffsetMap = RoseInstruction::OffsetMap;

static
OffsetMap makeOffsetMap(const RoseProgram &program, u32 *total_len) {
    OffsetMap offset_map;
    u32 offset = 0;
    for (const auto &ri : program) {
        offset = ROUNDUP_N(offset, ROSE_INSTR_MIN_ALIGN);
        DEBUG_PRINTF("instr %p (opcode %d) -> offset %u\n", ri.get(),
                     ri->code(), offset);
        assert(!contains(offset_map, ri.get()));
        offset_map.emplace(ri.get(), offset);
        offset += ri->byte_length();
    }
    *total_len = offset;
    return offset_map;
}

RoseProgram::RoseProgram() {
    prog.push_back(make_unique<RoseInstrEnd>());
}

RoseProgram::~RoseProgram() = default;

RoseProgram::RoseProgram(RoseProgram &&) = default;
RoseProgram &RoseProgram::operator=(RoseProgram &&) = default;

bool RoseProgram::empty() const {
    assert(!prog.empty());
    assert(prog.back()->code() == ROSE_INSTR_END);
    // Empty if we only have one element, the END instruction.
    return next(prog.begin()) == prog.end();
}

const RoseInstruction *RoseProgram::end_instruction() const {
    assert(!prog.empty());
    assert(prog.back()->code() == ROSE_INSTR_END);

    return prog.back().get();
}

void RoseProgram::update_targets(RoseProgram::iterator it,
                                 RoseProgram::iterator it_end,
                                 const RoseInstruction *old_target,
                                 const RoseInstruction *new_target) {
    assert(old_target && new_target && old_target != new_target);
    for (; it != it_end; ++it) {
        unique_ptr<RoseInstruction> &ri = *it;
        assert(ri);
        ri->update_target(old_target, new_target);
    }
}

RoseProgram::iterator RoseProgram::insert(RoseProgram::iterator it,
                                          unique_ptr<RoseInstruction> ri) {
    assert(!prog.empty());
    assert(it != end());
    assert(prog.back()->code() == ROSE_INSTR_END);

    return prog.insert(it, move(ri));
}

RoseProgram::iterator RoseProgram::insert(RoseProgram::iterator it,
                                          RoseProgram &&block) {
    assert(!prog.empty());
    assert(it != end());
    assert(prog.back()->code() == ROSE_INSTR_END);

    if (block.empty()) {
        return it;
    }

    const RoseInstruction *end_ptr = block.end_instruction();
    assert(end_ptr->code() == ROSE_INSTR_END);
    block.prog.pop_back();

    const RoseInstruction *new_target = it->get();
    update_targets(block.prog.begin(), block.prog.end(), end_ptr, new_target);

    // Workaround: container insert() for ranges doesn't return an iterator
    // in the version of the STL distributed with gcc 4.8.
    auto dist = distance(prog.begin(), it);
    prog.insert(it, make_move_iterator(block.prog.begin()),
                make_move_iterator(block.prog.end()));
    it = prog.begin();
    advance(it, dist);
    return it;
}

RoseProgram::iterator RoseProgram::erase(RoseProgram::iterator first,
                                          RoseProgram::iterator last) {
     return prog.erase(first, last);
}

void RoseProgram::add_before_end(std::unique_ptr<RoseInstruction> ri) {
    assert(!prog.empty());
    insert(std::prev(prog.end()), std::move(ri));
}

void RoseProgram::add_before_end(RoseProgram &&block) {
    assert(!prog.empty());
    assert(prog.back()->code() == ROSE_INSTR_END);

    if (block.empty()) {
        return;
    }

    insert(prev(prog.end()), move(block));
}

void RoseProgram::add_block(RoseProgram &&block) {
    assert(!prog.empty());
    assert(prog.back()->code() == ROSE_INSTR_END);

    if (block.empty()) {
        return;
    }

    // Replace pointers to the current END with pointers to the first
    // instruction in the new sequence.
    const RoseInstruction *end_ptr = end_instruction();
    prog.pop_back();
    update_targets(prog.begin(), prog.end(), end_ptr,
                   block.prog.front().get());
    prog.insert(prog.end(), make_move_iterator(block.prog.begin()),
                make_move_iterator(block.prog.end()));
}

bytecode_ptr<char> writeProgram(RoseEngineBlob &blob,
                                const RoseProgram &program) {
    u32 total_len = 0;
    const auto offset_map = makeOffsetMap(program, &total_len);
    DEBUG_PRINTF("%zu instructions, len %u\n", program.size(), total_len);

    auto bytecode = make_zeroed_bytecode_ptr<char>(total_len,
                                                   ROSE_INSTR_MIN_ALIGN);
    char *ptr = bytecode.get();

    for (const auto &ri : program) {
        assert(contains(offset_map, ri.get()));
        const u32 offset = offset_map.at(ri.get());
        ri->write(ptr + offset, blob, offset_map);
    }

    return bytecode;
}

size_t RoseProgramHash::operator()(const RoseProgram &program) const {
    size_t v = 0;
    for (const auto &ri : program) {
        assert(ri);
        hash_combine(v, ri->hash());
    }
    return v;
}

bool RoseProgramEquivalence::operator()(const RoseProgram &prog1,
                                        const RoseProgram &prog2) const {
    if (prog1.size() != prog2.size()) {
        return false;
    }

    u32 len_1 = 0, len_2 = 0;
    const auto offset_map_1 = makeOffsetMap(prog1, &len_1);
    const auto offset_map_2 = makeOffsetMap(prog2, &len_2);

    if (len_1 != len_2) {
        return false;
    }

    auto is_equiv = [&](const unique_ptr<RoseInstruction> &a,
                        const unique_ptr<RoseInstruction> &b) {
        assert(a && b);
        return a->equiv(*b, offset_map_1, offset_map_2);
    };

    return std::equal(prog1.begin(), prog1.end(), prog2.begin(), is_equiv);
}

/* Removes any CHECK_HANDLED instructions from the given program */
static
void stripCheckHandledInstruction(RoseProgram &prog) {
    for (auto it = prog.begin(); it != prog.end();) {
        auto ins = dynamic_cast<const RoseInstrCheckNotHandled *>(it->get());
        if (!ins) {
            ++it;
            continue;
        }

        auto next_it = next(it);
        assert(next_it != prog.end()); /* there should always be an end ins */
        auto next_ins = next_it->get();

        /* update all earlier instructions which point to ins to instead point
         * to the next instruction. Only need to look at earlier as we only ever
         * jump forward. */
        RoseProgram::update_targets(prog.begin(), it, ins, next_ins);

        /* remove check handled instruction */
        it = prog.erase(it, next_it);
    }
}


/** Returns true if the program may read the interpreter's work_done flag */
static
bool reads_work_done_flag(const RoseProgram &prog) {
    for (const auto &ri : prog) {
        if (dynamic_cast<const RoseInstrSquashGroups *>(ri.get())) {
            return true;
        }
    }
    return false;
}

void addEnginesEodProgram(u32 eodNfaIterOffset, RoseProgram &program) {
    if (!eodNfaIterOffset) {
        return;
    }

    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrEnginesEod>(eodNfaIterOffset));
    program.add_block(move(block));
}

void addSuffixesEodProgram(RoseProgram &program) {
    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrSuffixesEod>());
    program.add_block(move(block));
}

void addMatcherEodProgram(RoseProgram &program) {
    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrMatcherEod>());
    program.add_block(move(block));
}

void addFlushCombinationProgram(RoseProgram &program) {
    program.add_before_end(make_unique<RoseInstrFlushCombination>());
}

void addLastFlushCombinationProgram(RoseProgram &program) {
    program.add_before_end(make_unique<RoseInstrLastFlushCombination>());
}

static
void makeRoleCheckLeftfix(const RoseBuildImpl &build,
                          const map<RoseVertex, left_build_info> &leftfix_info,
                          RoseVertex v, RoseProgram &program) {
    auto it = leftfix_info.find(v);
    if (it == end(leftfix_info)) {
        return;
    }
    const left_build_info &lni = it->second;
    if (lni.has_lookaround) {
        return; // Leftfix completely implemented by lookaround.
    }

    assert(!build.cc.streaming ||
           build.g[v].left.lag <= MAX_STORED_LEFTFIX_LAG);

    bool is_prefix = build.isRootSuccessor(v);
    const auto *end_inst = program.end_instruction();

    unique_ptr<RoseInstruction> ri;
    if (is_prefix) {
        ri = make_unique<RoseInstrCheckPrefix>(lni.queue, build.g[v].left.lag,
                                               build.g[v].left.leftfix_report,
                                               end_inst);
    } else {
        ri = make_unique<RoseInstrCheckInfix>(lni.queue, build.g[v].left.lag,
                                              build.g[v].left.leftfix_report,
                                              end_inst);
    }
    program.add_before_end(move(ri));
}

static
void makeAnchoredLiteralDelay(const RoseBuildImpl &build,
                              const ProgramBuild &prog_build, u32 lit_id,
                              RoseProgram &program) {
    // Only relevant for literals in the anchored table.
    const rose_literal_id &lit = build.literals.at(lit_id);
    if (lit.table != ROSE_ANCHORED) {
        return;
    }

    // If this literal match cannot occur after floatingMinLiteralMatchOffset,
    // we do not need this check.
    bool all_too_early = true;
    rose_group groups = 0;

    const auto &lit_vertices = build.literal_info.at(lit_id).vertices;
    for (RoseVertex v : lit_vertices) {
         if (build.g[v].max_offset > prog_build.floatingMinLiteralMatchOffset) {
             all_too_early = false;
         }
         groups |= build.g[v].groups;
    }

    if (all_too_early) {
        return;
    }

    assert(contains(prog_build.anchored_programs, lit_id));
    u32 anch_id = prog_build.anchored_programs.at(lit_id);

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrAnchoredDelay>(groups, anch_id, end_inst);
    program.add_before_end(move(ri));
}

static
void makeDedupe(const ReportManager &rm, const Report &report,
                RoseProgram &program) {
    const auto *end_inst = program.end_instruction();
    auto ri =
        make_unique<RoseInstrDedupe>(report.quashSom, rm.getDkey(report),
                                     report.offsetAdjust, end_inst);
    program.add_before_end(move(ri));
}

static
void makeDedupeSom(const ReportManager &rm, const Report &report,
                   RoseProgram &program) {
    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrDedupeSom>(report.quashSom,
                                              rm.getDkey(report),
                                              report.offsetAdjust, end_inst);
    program.add_before_end(move(ri));
}

static
void makeCatchup(const ReportManager &rm, bool needs_catchup,
                 const flat_set<ReportID> &reports, RoseProgram &program) {
    if (!needs_catchup) {
        return;
    }

    // Everything except the INTERNAL_ROSE_CHAIN report needs catchup to run
    // before reports are triggered.

    auto report_needs_catchup = [&](const ReportID &id) {
        const Report &report = rm.getReport(id);
        return report.type != INTERNAL_ROSE_CHAIN;
    };

    if (!any_of(begin(reports), end(reports), report_needs_catchup)) {
        DEBUG_PRINTF("none of the given reports needs catchup\n");
        return;
    }

    program.add_before_end(make_unique<RoseInstrCatchUp>());
}

static
void writeSomOperation(const Report &report, som_operation *op) {
    assert(op);

    memset(op, 0, sizeof(*op));

    switch (report.type) {
    case EXTERNAL_CALLBACK_SOM_REL:
        op->type = SOM_EXTERNAL_CALLBACK_REL;
        break;
    case INTERNAL_SOM_LOC_SET:
        op->type = SOM_INTERNAL_LOC_SET;
        break;
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
        op->type = SOM_INTERNAL_LOC_SET_IF_UNSET;
        break;
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET;
        break;
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_REV_NFA_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_COPY:
        op->type = SOM_INTERNAL_LOC_COPY;
        break;
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_COPY_IF_WRITABLE;
        break;
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
        op->type = SOM_INTERNAL_LOC_MAKE_WRITABLE;
        break;
    case EXTERNAL_CALLBACK_SOM_STORED:
        op->type = SOM_EXTERNAL_CALLBACK_STORED;
        break;
    case EXTERNAL_CALLBACK_SOM_ABS:
        op->type = SOM_EXTERNAL_CALLBACK_ABS;
        break;
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        op->type = SOM_EXTERNAL_CALLBACK_REV_NFA;
        break;
    case INTERNAL_SOM_LOC_SET_FROM:
        op->type = SOM_INTERNAL_LOC_SET_FROM;
        break;
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
        op->type = SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE;
        break;
    default:
        // This report doesn't correspond to a SOM operation.
        assert(0);
        throw CompileError("Unable to generate bytecode.");
    }

    op->onmatch = report.onmatch;

    switch (report.type) {
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
        op->aux.revNfaIndex = report.revNfaIndex;
        break;
    default:
        op->aux.somDistance = report.somDistance;
        break;
    }
}

static
void addLogicalSetRequired(const Report &report, ReportManager &rm,
                           RoseProgram &program) {
    if (report.lkey == INVALID_LKEY) {
        return;
    }
    // set matching status of current lkey
    auto risl = make_unique<RoseInstrSetLogical>(report.lkey,
                                                 report.offsetAdjust);
    program.add_before_end(move(risl));
    // set current lkey's corresponding ckeys active, pending to check
    for (auto ckey : rm.getRelateCKeys(report.lkey)) {
        auto risc = make_unique<RoseInstrSetCombination>(ckey);
        program.add_before_end(move(risc));
    }
}

static
void makeReport(const RoseBuildImpl &build, const ReportID id,
                const bool has_som, RoseProgram &program) {
    assert(id < build.rm.numReports());
    const Report &report = build.rm.getReport(id);

    RoseProgram report_block;
    const RoseInstruction *end_inst = report_block.end_instruction();

    // Handle min/max offset checks.
    if (report.minOffset > 0 || report.maxOffset < MAX_OFFSET) {
        auto ri = make_unique<RoseInstrCheckBounds>(report.minOffset,
                                                    report.maxOffset, end_inst);
        report_block.add_before_end(move(ri));
    }

    // If this report has an exhaustion key, we can check it in the program
    // rather than waiting until we're in the callback adaptor.
    if (report.ekey != INVALID_EKEY) {
        auto ri = make_unique<RoseInstrCheckExhausted>(report.ekey, end_inst);
        report_block.add_before_end(move(ri));
    }

    // External SOM reports that aren't passthrough need their SOM value
    // calculated.
    if (isExternalSomReport(report) &&
        report.type != EXTERNAL_CALLBACK_SOM_PASS) {
        auto ri = make_unique<RoseInstrSomFromReport>();
        writeSomOperation(report, &ri->som);
        report_block.add_before_end(move(ri));
    }

    // Min length constraint.
    if (report.minLength > 0) {
        assert(build.hasSom);
        auto ri = make_unique<RoseInstrCheckMinLength>(
            report.offsetAdjust, report.minLength, end_inst);
        report_block.add_before_end(move(ri));
    }

    if (report.quashSom) {
        report_block.add_before_end(make_unique<RoseInstrSomZero>());
    }

    switch (report.type) {
    case EXTERNAL_CALLBACK:
        if (build.rm.numCkeys()) {
            addFlushCombinationProgram(report_block);
        }
        if (!has_som) {
            // Dedupe is only necessary if this report has a dkey, or if there
            // are SOM reports to catch up.
            bool needs_dedupe = build.rm.getDkey(report) != ~0U || build.hasSom;
            if (report.ekey == INVALID_EKEY) {
                if (needs_dedupe) {
                    if (!report.quiet) {
                        report_block.add_before_end(
                            make_unique<RoseInstrDedupeAndReport>(
                                report.quashSom, build.rm.getDkey(report),
                                report.onmatch, report.offsetAdjust, end_inst));
                    } else {
                        makeDedupe(build.rm, report, report_block);
                    }
                } else {
                    if (!report.quiet) {
                        report_block.add_before_end(
                            make_unique<RoseInstrReport>(
                                report.onmatch, report.offsetAdjust));
                    }
                }
            } else {
                if (needs_dedupe) {
                    makeDedupe(build.rm, report, report_block);
                }
                if (!report.quiet) {
                    report_block.add_before_end(
                        make_unique<RoseInstrReportExhaust>(
                            report.onmatch, report.offsetAdjust, report.ekey));
                } else {
                    report_block.add_before_end(
                        make_unique<RoseInstrSetExhaust>(report.ekey));
                }
            }
        } else { // has_som
            makeDedupeSom(build.rm, report, report_block);
            if (report.ekey == INVALID_EKEY) {
                if (!report.quiet) {
                    report_block.add_before_end(make_unique<RoseInstrReportSom>(
                        report.onmatch, report.offsetAdjust));
                }
            } else {
                if (!report.quiet) {
                    report_block.add_before_end(
                        make_unique<RoseInstrReportSomExhaust>(
                            report.onmatch, report.offsetAdjust, report.ekey));
                } else {
                    report_block.add_before_end(
                        make_unique<RoseInstrSetExhaust>(report.ekey));
                }
            }
        }
        addLogicalSetRequired(report, build.rm, report_block);
        break;
    case INTERNAL_SOM_LOC_SET:
    case INTERNAL_SOM_LOC_SET_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_IF_WRITABLE:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_UNSET:
    case INTERNAL_SOM_LOC_SET_SOM_REV_NFA_IF_WRITABLE:
    case INTERNAL_SOM_LOC_COPY:
    case INTERNAL_SOM_LOC_COPY_IF_WRITABLE:
    case INTERNAL_SOM_LOC_MAKE_WRITABLE:
    case INTERNAL_SOM_LOC_SET_FROM:
    case INTERNAL_SOM_LOC_SET_FROM_IF_WRITABLE:
        if (build.rm.numCkeys()) {
            addFlushCombinationProgram(report_block);
        }
        if (has_som) {
            auto ri = make_unique<RoseInstrReportSomAware>();
            writeSomOperation(report, &ri->som);
            report_block.add_before_end(move(ri));
        } else {
            auto ri = make_unique<RoseInstrReportSomInt>();
            writeSomOperation(report, &ri->som);
            report_block.add_before_end(move(ri));
        }
        break;
    case INTERNAL_ROSE_CHAIN: {
        report_block.add_before_end(make_unique<RoseInstrReportChain>(
            report.onmatch, report.topSquashDistance));
        break;
    }
    case EXTERNAL_CALLBACK_SOM_REL:
    case EXTERNAL_CALLBACK_SOM_STORED:
    case EXTERNAL_CALLBACK_SOM_ABS:
    case EXTERNAL_CALLBACK_SOM_REV_NFA:
        if (build.rm.numCkeys()) {
            addFlushCombinationProgram(report_block);
        }
        makeDedupeSom(build.rm, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            if (!report.quiet) {
                report_block.add_before_end(make_unique<RoseInstrReportSom>(
                    report.onmatch, report.offsetAdjust));
            }
        } else {
            if (!report.quiet) {
                report_block.add_before_end(
                    make_unique<RoseInstrReportSomExhaust>(
                        report.onmatch, report.offsetAdjust, report.ekey));
            } else {
                report_block.add_before_end(
                    make_unique<RoseInstrSetExhaust>(report.ekey));
            }
        }
        addLogicalSetRequired(report, build.rm, report_block);
        break;
    case EXTERNAL_CALLBACK_SOM_PASS:
        if (build.rm.numCkeys()) {
            addFlushCombinationProgram(report_block);
        }
        makeDedupeSom(build.rm, report, report_block);
        if (report.ekey == INVALID_EKEY) {
            if (!report.quiet) {
                report_block.add_before_end(make_unique<RoseInstrReportSom>(
                    report.onmatch, report.offsetAdjust));
            }
        } else {
            if (!report.quiet) {
                report_block.add_before_end(
                    make_unique<RoseInstrReportSomExhaust>(
                        report.onmatch, report.offsetAdjust, report.ekey));
            } else {
                report_block.add_before_end(
                    make_unique<RoseInstrSetExhaust>(report.ekey));
            }
        }
        addLogicalSetRequired(report, build.rm, report_block);
        break;

    default:
        assert(0);
        throw CompileError("Unable to generate bytecode.");
    }

    program.add_block(move(report_block));
}

static
void makeRoleReports(const RoseBuildImpl &build,
                     const std::map<RoseVertex, left_build_info> &leftfix_info,
                     bool needs_catchup, RoseVertex v, RoseProgram &program) {
    const auto &g = build.g;

    bool report_som = false;
    if (g[v].left.tracksSom()) {
        /* we are a suffaig - need to update role to provide som to the
         * suffix. */
        assert(contains(leftfix_info, v));
        const left_build_info &lni = leftfix_info.at(v);
        program.add_before_end(
            make_unique<RoseInstrSomLeftfix>(lni.queue, g[v].left.lag));
        report_som = true;
    } else if (g[v].som_adjust) {
        program.add_before_end(
            make_unique<RoseInstrSomAdjust>(g[v].som_adjust));
        report_som = true;
    }

    makeCatchup(build.rm, needs_catchup, g[v].reports, program);

    RoseProgram report_block;
    for (ReportID id : g[v].reports) {
        makeReport(build, id, report_som, report_block);
    }
    program.add_before_end(move(report_block));
}

static
void makeRoleSetState(const unordered_map<RoseVertex, u32> &roleStateIndices,
                      RoseVertex v, RoseProgram &program) {
    // We only need this instruction if a state index has been assigned to this
    // vertex.
    auto it = roleStateIndices.find(v);
    if (it == end(roleStateIndices)) {
        return;
    }
    program.add_before_end(make_unique<RoseInstrSetState>(it->second));
}

static
void makePushDelayedInstructions(const RoseLiteralMap &literals,
                                 ProgramBuild &prog_build,
                                 const flat_set<u32> &delayed_ids,
                                 RoseProgram &program) {
    vector<RoseInstrPushDelayed> delay_instructions;

    for (const auto &delayed_lit_id : delayed_ids) {
        DEBUG_PRINTF("delayed lit id %u\n", delayed_lit_id);
        assert(contains(prog_build.delay_programs, delayed_lit_id));
        u32 delay_id = prog_build.delay_programs.at(delayed_lit_id);
        const auto &delay_lit = literals.at(delayed_lit_id);
        delay_instructions.emplace_back(verify_u8(delay_lit.delay), delay_id);
    }

    sort_and_unique(delay_instructions, [](const RoseInstrPushDelayed &a,
                                           const RoseInstrPushDelayed &b) {
        return tie(a.delay, a.index) < tie(b.delay, b.index);
    });

    for (const auto &ri : delay_instructions) {
        program.add_before_end(make_unique<RoseInstrPushDelayed>(ri));
    }
}

static
void makeCheckLiteralInstruction(const rose_literal_id &lit,
                                 size_t longLitLengthThreshold,
                                 RoseProgram &program,
                                 const CompileContext &cc) {
    assert(longLitLengthThreshold > 0);

    DEBUG_PRINTF("lit=%s, long lit threshold %zu\n", dumpString(lit.s).c_str(),
                 longLitLengthThreshold);

    if (lit.s.length() <= ROSE_SHORT_LITERAL_LEN_MAX) {
        DEBUG_PRINTF("lit short enough to not need confirm\n");
        return;
    }

    // Check resource limits as well.
    if (lit.s.length() > cc.grey.limitLiteralLength) {
        throw ResourceLimitError();
    }

    if (lit.s.length() <= longLitLengthThreshold) {
        DEBUG_PRINTF("is a medium-length literal\n");
        const auto *end_inst = program.end_instruction();
        unique_ptr<RoseInstruction> ri;
        if (lit.s.any_nocase()) {
            ri = make_unique<RoseInstrCheckMedLitNocase>(lit.s.get_string(),
                                                         end_inst);
        } else {
            ri = make_unique<RoseInstrCheckMedLit>(lit.s.get_string(),
                                                   end_inst);
        }
        program.add_before_end(move(ri));
        return;
    }

    // Long literal support should only really be used for the floating table
    // in streaming mode.
    assert(lit.table == ROSE_FLOATING && cc.streaming);

    DEBUG_PRINTF("is a long literal\n");

    const auto *end_inst = program.end_instruction();
    unique_ptr<RoseInstruction> ri;
    if (lit.s.any_nocase()) {
        ri = make_unique<RoseInstrCheckLongLitNocase>(lit.s.get_string(),
                                                      end_inst);
    } else {
        ri = make_unique<RoseInstrCheckLongLit>(lit.s.get_string(), end_inst);
    }
    program.add_before_end(move(ri));
}

static
void makeRoleCheckNotHandled(ProgramBuild &prog_build, RoseVertex v,
                             RoseProgram &program) {
    u32 handled_key;
    if (contains(prog_build.handledKeys, v)) {
        handled_key = prog_build.handledKeys.at(v);
    } else {
        handled_key = verify_u32(prog_build.handledKeys.size());
        prog_build.handledKeys.emplace(v, handled_key);
    }

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrCheckNotHandled>(handled_key, end_inst);
    program.add_before_end(move(ri));
}

static
void makeRoleCheckBounds(const RoseBuildImpl &build, RoseVertex v,
                         const RoseEdge &e, RoseProgram &program) {
    const RoseGraph &g = build.g;
    const RoseVertex u = source(e, g);

    // We know that we can trust the anchored table (DFA) to always deliver us
    // literals at the correct offset.
    if (build.isAnchored(v)) {
        DEBUG_PRINTF("literal in anchored table, skipping bounds check\n");
        return;
    }

    // Use the minimum literal length.
    u32 lit_length = g[v].eod_accept ? 0 : verify_u32(build.minLiteralLen(v));

    u64a min_bound = g[e].minBound + lit_length;
    u64a max_bound = g[e].maxBound == ROSE_BOUND_INF
                         ? ROSE_BOUND_INF
                         : g[e].maxBound + lit_length;

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        assert(g[u].fixedOffset());
        // Make offsets absolute.
        min_bound += g[u].max_offset;
        if (max_bound != ROSE_BOUND_INF) {
            max_bound += g[u].max_offset;
        }
    }

    assert(max_bound <= ROSE_BOUND_INF);
    assert(min_bound <= max_bound);

    // CHECK_BOUNDS instruction uses 64-bit bounds, so we can use MAX_OFFSET
    // (max value of a u64a) to represent ROSE_BOUND_INF.
    if (max_bound == ROSE_BOUND_INF) {
        max_bound = MAX_OFFSET;
    }

    // This instruction should be doing _something_ -- bounds should be tighter
    // than just {length, inf}.
    assert(min_bound > lit_length || max_bound < MAX_OFFSET);

    const auto *end_inst = program.end_instruction();
    program.add_before_end(
        make_unique<RoseInstrCheckBounds>(min_bound, max_bound, end_inst));
}

static
void makeRoleGroups(const RoseGraph &g, ProgramBuild &prog_build,
                    RoseVertex v, RoseProgram &program) {
    rose_group groups = g[v].groups;
    if (!groups) {
        return;
    }

    // The set of "already on" groups as we process this vertex is the
    // intersection of the groups set by our predecessors.
    assert(in_degree(v, g) > 0);
    rose_group already_on = ~rose_group{0};
    for (const auto &u : inv_adjacent_vertices_range(v, g)) {
        already_on &= prog_build.vertex_group_map.at(u);
    }

    DEBUG_PRINTF("already_on=0x%llx\n", already_on);
    DEBUG_PRINTF("squashable=0x%llx\n", prog_build.squashable_groups);
    DEBUG_PRINTF("groups=0x%llx\n", groups);

    already_on &= ~prog_build.squashable_groups;
    DEBUG_PRINTF("squashed already_on=0x%llx\n", already_on);

    // We don't *have* to mask off the groups that we know are already on, but
    // this will make bugs more apparent.
    groups &= ~already_on;

    if (!groups) {
        DEBUG_PRINTF("no new groups to set, skipping\n");
        return;
    }

    program.add_before_end(make_unique<RoseInstrSetGroups>(groups));
}

static
bool checkReachMask(const CharReach &cr, u8 &andmask, u8 &cmpmask) {
    size_t reach_size = cr.count();
    assert(reach_size > 0);
    // check whether entry_size is some power of 2.
    if ((reach_size - 1) & reach_size) {
        return false;
    }
    make_and_cmp_mask(cr, &andmask, &cmpmask);
    if ((1 << popcount32((u8)(~andmask))) ^ reach_size) {
        return false;
    }
    return true;
}

static
bool checkReachWithFlip(const CharReach &cr, u8 &andmask,
                       u8 &cmpmask, u8 &flip) {
    if (checkReachMask(cr, andmask, cmpmask)) {
        flip = 0;
        return true;
    }
    if (checkReachMask(~cr, andmask, cmpmask)) {
        flip = 1;
        return true;
    }
    return false;
}

static
bool makeRoleByte(const vector<LookEntry> &look, RoseProgram &program) {
    if (look.size() == 1) {
        const auto &entry = look[0];
        u8 andmask_u8, cmpmask_u8;
        u8 flip;
        if (!checkReachWithFlip(entry.reach, andmask_u8, cmpmask_u8, flip)) {
            return false;
        }
        s32 checkbyte_offset = verify_s32(entry.offset);
        DEBUG_PRINTF("CHECK BYTE offset=%d\n", checkbyte_offset);
        const auto *end_inst = program.end_instruction();
        auto ri = make_unique<RoseInstrCheckByte>(andmask_u8, cmpmask_u8, flip,
                                                  checkbyte_offset, end_inst);
        program.add_before_end(move(ri));
        return true;
    }
    return false;
}

static
bool makeRoleMask(const vector<LookEntry> &look, RoseProgram &program) {
    if (look.back().offset < look.front().offset + 8) {
        s32 base_offset = verify_s32(look.front().offset);
        u64a and_mask = 0;
        u64a cmp_mask = 0;
        u64a neg_mask = 0;
        for (const auto &entry : look) {
            u8 andmask_u8, cmpmask_u8, flip;
            if (!checkReachWithFlip(entry.reach, andmask_u8,
                                    cmpmask_u8, flip)) {
                return false;
            }
            DEBUG_PRINTF("entry offset %d\n", entry.offset);
            u32 shift = (entry.offset - base_offset) << 3;
            and_mask |= (u64a)andmask_u8 << shift;
            cmp_mask |= (u64a)cmpmask_u8 << shift;
            if (flip) {
                neg_mask |= 0xffLLU << shift;
            }
        }
        DEBUG_PRINTF("CHECK MASK and_mask=%llx cmp_mask=%llx\n",
                     and_mask, cmp_mask);
        const auto *end_inst = program.end_instruction();
        auto ri = make_unique<RoseInstrCheckMask>(and_mask, cmp_mask, neg_mask,
                                                  base_offset, end_inst);
        program.add_before_end(move(ri));
        return true;
    }
    return false;
}

static UNUSED
string convertMaskstoString(u8 *p, int byte_len) {
    string s;
    for (int i = 0; i < byte_len; i++) {
        u8 hi = *p >> 4;
        u8 lo = *p & 0xf;
        s += (char)(hi + (hi < 10 ? 48 : 87));
        s += (char)(lo + (lo < 10 ? 48 : 87));
        p++;
    }
    return s;
}

static
bool makeRoleMask32(const vector<LookEntry> &look,
                    RoseProgram &program) {
    if (look.back().offset >= look.front().offset + 32) {
        return false;
    }
    s32 base_offset = verify_s32(look.front().offset);
    array<u8, 32> and_mask, cmp_mask;
    and_mask.fill(0);
    cmp_mask.fill(0);
    u32 neg_mask = 0;
    for (const auto &entry : look) {
        u8 andmask_u8, cmpmask_u8, flip;
        if (!checkReachWithFlip(entry.reach, andmask_u8,
                                cmpmask_u8, flip)) {
            return false;
        }
        u32 shift = entry.offset - base_offset;
        assert(shift < 32);
        and_mask[shift] = andmask_u8;
        cmp_mask[shift] = cmpmask_u8;
        if (flip) {
            neg_mask |= 1 << shift;
        }
    }

    DEBUG_PRINTF("and_mask %s\n",
                 convertMaskstoString(and_mask.data(), 32).c_str());
    DEBUG_PRINTF("cmp_mask %s\n",
                 convertMaskstoString(cmp_mask.data(), 32).c_str());
    DEBUG_PRINTF("neg_mask %08x\n", neg_mask);
    DEBUG_PRINTF("base_offset %d\n", base_offset);

    const auto *end_inst = program.end_instruction();
    auto ri = make_unique<RoseInstrCheckMask32>(and_mask, cmp_mask, neg_mask,
                                                base_offset, end_inst);
    program.add_before_end(move(ri));
    return true;
}

// Sorting by the size of every bucket.
// Used in map<u32, vector<s8>, cmpNibble>.
struct cmpNibble {
    bool operator()(const u32 data1, const u32 data2) const{
        u32 size1 = popcount32(data1 >> 16) * popcount32(data1 << 16);
        u32 size2 = popcount32(data2 >> 16) * popcount32(data2 << 16);
        return std::tie(size1, data1) < std::tie(size2, data2);
    }
};

// Insert all pairs of bucket and offset into buckets.
static really_inline
void getAllBuckets(const vector<LookEntry> &look,
                   map<u32, vector<s8>, cmpNibble> &buckets, u64a &neg_mask) {
    s32 base_offset = verify_s32(look.front().offset);
    for (const auto &entry : look) {
        CharReach cr = entry.reach;
        // Flip heavy character classes to save buckets.
        if (cr.count() > 128 ) {
            cr.flip();
        } else {
            neg_mask ^= 1ULL << (entry.offset - base_offset);
        }
        map <u16, u16> lo2hi;
        // We treat Ascii Table as a 16x16 grid.
        // Push every row in cr into lo2hi and mark the row number.
        for (size_t i = cr.find_first(); i != CharReach::npos;) {
            u8 it_hi = i >> 4;
            u16 low_encode = 0;
            while (i != CharReach::npos && (i >> 4) == it_hi) {
                low_encode |= 1 << (i & 0xf);
                i = cr.find_next(i);
            }
            lo2hi[low_encode] |= 1 << it_hi;
        }
        for (const auto &it : lo2hi) {
            u32 hi_lo = (it.second << 16) | it.first;
            buckets[hi_lo].push_back(entry.offset);
        }
    }
}

// Once we have a new bucket, we'll try to combine it with all old buckets.
static really_inline
void nibUpdate(map<u32, u16> &nib, u32 hi_lo) {
    u16 hi = hi_lo >> 16;
    u16 lo = hi_lo & 0xffff;
    for (const auto pairs : nib) {
        u32 old = pairs.first;
        if ((old >> 16) == hi || (old & 0xffff) == lo) {
            if (!nib[old | hi_lo]) {
                nib[old | hi_lo] = nib[old] | nib[hi_lo];
            }
        }
    }
}

static really_inline
void nibMaskUpdate(array<u8, 32> &mask, u32 data, u8 bit_index) {
    for (u8 index = 0; data > 0; data >>= 1, index++) {
        if (data & 1) {
            // 0 ~ 7 bucket in first 16 bytes,
            // 8 ~ 15 bucket in second 16 bytes.
            if (bit_index >= 8) {
                mask[index + 16] |= 1 << (bit_index - 8);
            } else {
                mask[index] |= 1 << bit_index;
            }
        }
    }
}

static
bool getShuftiMasks(const vector<LookEntry> &look, array<u8, 32> &hi_mask,
                    array<u8, 32> &lo_mask, u8 *bucket_select_hi,
                    u8 *bucket_select_lo, u64a &neg_mask,
                    u8 &bit_idx, size_t len) {
    map<u32, u16> nib; // map every bucket to its bucket number.
    map<u32, vector<s8>, cmpNibble> bucket2offsets;
    s32 base_offset = look.front().offset;

    bit_idx = 0;
    neg_mask = ~0ULL;

    getAllBuckets(look, bucket2offsets, neg_mask);

    for (const auto &it : bucket2offsets) {
        u32 hi_lo = it.first;
        // New bucket.
        if (!nib[hi_lo]) {
            if ((bit_idx >= 8 && len == 64) || bit_idx >= 16) {
                return false;
            }
            nib[hi_lo] = 1 << bit_idx;

            nibUpdate(nib, hi_lo);
            nibMaskUpdate(hi_mask, hi_lo >> 16, bit_idx);
            nibMaskUpdate(lo_mask, hi_lo & 0xffff, bit_idx);
            bit_idx++;
        }

        DEBUG_PRINTF("hi_lo %x bucket %x\n", hi_lo, nib[hi_lo]);

        // Update bucket_select_mask.
        u8 nib_hi = nib[hi_lo] >> 8;
        u8 nib_lo = nib[hi_lo] & 0xff;
        for (const auto offset : it.second) {
            bucket_select_hi[offset - base_offset] |= nib_hi;
            bucket_select_lo[offset - base_offset] |= nib_lo;
        }
    }
    return true;
}

static
unique_ptr<RoseInstruction>
makeCheckShufti16x8(u32 offset_range, u8 bucket_idx,
                    const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                    const array<u8, 32> &bucket_select_mask,
                    u32 neg_mask, s32 base_offset,
                    const RoseInstruction *end_inst) {
    if (offset_range > 16 || bucket_idx > 8) {
        return nullptr;
    }
    array<u8, 32> nib_mask;
    array<u8, 16> bucket_select_mask_16;
    copy(lo_mask.begin(), lo_mask.begin() + 16, nib_mask.begin());
    copy(hi_mask.begin(), hi_mask.begin() + 16, nib_mask.begin() + 16);
    copy(bucket_select_mask.begin(), bucket_select_mask.begin() + 16,
         bucket_select_mask_16.begin());
    return make_unique<RoseInstrCheckShufti16x8>
           (nib_mask, bucket_select_mask_16,
            neg_mask & 0xffff, base_offset, end_inst);
}

static
unique_ptr<RoseInstruction>
makeCheckShufti32x8(u32 offset_range, u8 bucket_idx,
                    const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                    const array<u8, 32> &bucket_select_mask,
                    u32 neg_mask, s32 base_offset,
                    const RoseInstruction *end_inst) {
    if (offset_range > 32 || bucket_idx > 8) {
        return nullptr;
    }

    array<u8, 16> hi_mask_16;
    array<u8, 16> lo_mask_16;
    copy(hi_mask.begin(), hi_mask.begin() + 16, hi_mask_16.begin());
    copy(lo_mask.begin(), lo_mask.begin() + 16, lo_mask_16.begin());
    return make_unique<RoseInstrCheckShufti32x8>
           (hi_mask_16, lo_mask_16, bucket_select_mask,
            neg_mask, base_offset, end_inst);
}

static
unique_ptr<RoseInstruction>
makeCheckShufti16x16(u32 offset_range, u8 bucket_idx,
                     const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                     const array<u8, 32> &bucket_select_mask_lo,
                     const array<u8, 32> &bucket_select_mask_hi,
                     u32 neg_mask, s32 base_offset,
                     const RoseInstruction *end_inst) {
    if (offset_range > 16 || bucket_idx > 16) {
        return nullptr;
    }

    array<u8, 32> bucket_select_mask_32;
    copy(bucket_select_mask_lo.begin(), bucket_select_mask_lo.begin() + 16,
         bucket_select_mask_32.begin());
    copy(bucket_select_mask_hi.begin(), bucket_select_mask_hi.begin() + 16,
         bucket_select_mask_32.begin() + 16);
    return make_unique<RoseInstrCheckShufti16x16>
           (hi_mask, lo_mask, bucket_select_mask_32,
            neg_mask & 0xffff, base_offset, end_inst);
}
static
unique_ptr<RoseInstruction>
makeCheckShufti32x16(u32 offset_range, u8 bucket_idx,
                     const array<u8, 32> &hi_mask, const array<u8, 32> &lo_mask,
                     const array<u8, 32> &bucket_select_mask_lo,
                     const array<u8, 32> &bucket_select_mask_hi,
                     u32 neg_mask, s32 base_offset,
                     const RoseInstruction *end_inst) {
    if (offset_range > 32 || bucket_idx > 16) {
        return nullptr;
    }

    return make_unique<RoseInstrCheckShufti32x16>
           (hi_mask, lo_mask, bucket_select_mask_hi,
            bucket_select_mask_lo, neg_mask, base_offset, end_inst);
}

static
bool makeRoleShufti(const vector<LookEntry> &look, RoseProgram &program) {

    s32 base_offset = verify_s32(look.front().offset);
    if (look.back().offset >= base_offset + 32) {
        return false;
    }

    u8 bucket_idx = 0; // number of buckets
    u64a neg_mask_64;
    array<u8, 32> hi_mask;
    array<u8, 32> lo_mask;
    array<u8, 32> bucket_select_hi;
    array<u8, 32> bucket_select_lo;
    hi_mask.fill(0);
    lo_mask.fill(0);
    bucket_select_hi.fill(0); // will not be used in 16x8 and 32x8.
    bucket_select_lo.fill(0);

    if (!getShuftiMasks(look, hi_mask, lo_mask, bucket_select_hi.data(),
                        bucket_select_lo.data(), neg_mask_64, bucket_idx, 32)) {
        return false;
    }
    u32 neg_mask = (u32)neg_mask_64;

    DEBUG_PRINTF("hi_mask %s\n",
                 convertMaskstoString(hi_mask.data(), 32).c_str());
    DEBUG_PRINTF("lo_mask %s\n",
                 convertMaskstoString(lo_mask.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_hi %s\n",
                 convertMaskstoString(bucket_select_hi.data(), 32).c_str());
    DEBUG_PRINTF("bucket_select_lo %s\n",
                 convertMaskstoString(bucket_select_lo.data(), 32).c_str());

    const auto *end_inst = program.end_instruction();
    s32 offset_range = look.back().offset - base_offset + 1;

    auto ri = makeCheckShufti16x8(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, neg_mask, base_offset,
                                  end_inst);
    if (!ri) {
        ri = makeCheckShufti32x8(offset_range, bucket_idx, hi_mask, lo_mask,
                                 bucket_select_lo, neg_mask, base_offset,
                                 end_inst);
    }
    if (!ri) {
        ri = makeCheckShufti16x16(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, bucket_select_hi,
                                  neg_mask, base_offset, end_inst);
    }
    if (!ri) {
        ri = makeCheckShufti32x16(offset_range, bucket_idx, hi_mask, lo_mask,
                                  bucket_select_lo, bucket_select_hi,
                                  neg_mask, base_offset, end_inst);
    }
    assert(ri);
    program.add_before_end(move(ri));

    return true;
}

/**
 * Builds a lookaround instruction, or an appropriate specialization if one is
 * available.
 */
static
void makeLookaroundInstruction(const vector<LookEntry> &look,
                               RoseProgram &program) {
    assert(!look.empty());

    if (makeRoleByte(look, program)) {
        return;
    }

    if (look.size() == 1) {
        s8 offset = look.begin()->offset;
        const CharReach &reach = look.begin()->reach;
        auto ri = make_unique<RoseInstrCheckSingleLookaround>(offset, reach,
                                                     program.end_instruction());
        program.add_before_end(move(ri));
        return;
    }

    if (makeRoleMask(look, program)) {
        return;
    }

    if (makeRoleMask32(look, program)) {
        return;
    }

    if (makeRoleShufti(look, program)) {
        return;
    }

    auto ri = make_unique<RoseInstrCheckLookaround>(look,
                                                    program.end_instruction());
    program.add_before_end(move(ri));
}

static
void makeCheckLitMaskInstruction(const RoseBuildImpl &build, u32 lit_id,
                                 RoseProgram &program) {
    const auto &info = build.literal_info.at(lit_id);
    if (!info.requires_benefits) {
        return;
    }

    vector<LookEntry> look;

    const auto &lit = build.literals.at(lit_id);
    const ue2_literal &s = lit.s;
    const auto &msk = lit.msk;

    DEBUG_PRINTF("building mask for lit %u: %s\n", lit_id,
                 dumpString(s).c_str());

    assert(s.length() <= MAX_MASK2_WIDTH);

    // Note: the literal matcher will confirm the HWLM mask in lit.msk, so we
    // do not include those entries in the lookaround.
    auto it = s.begin();
    for (s32 i = 0 - s.length(), i_end = 0 - msk.size(); i < i_end; ++i, ++it) {
        if (!it->nocase) {
            look.emplace_back(verify_s8(i), *it);
        }
    }

    if (look.empty()) {
        return; // all caseful chars handled by HWLM mask.
    }

    makeLookaroundInstruction(look, program);
}

static
void makeCheckLitEarlyInstruction(const RoseBuildImpl &build, u32 lit_id,
                                  const vector<RoseEdge> &lit_edges,
                                  u32 floatingMinLiteralMatchOffset,
                                  RoseProgram &prog) {
    if (lit_edges.empty()) {
        return;
    }

    if (floatingMinLiteralMatchOffset == 0) {
        return;
    }

    RoseVertex v = target(lit_edges.front(), build.g);
    if (!build.isFloating(v)) {
        return;
    }

    const auto &lit = build.literals.at(lit_id);
    size_t min_len = lit.elength();
    u32 min_offset = findMinOffset(build, lit_id);
    DEBUG_PRINTF("has min_len=%zu, min_offset=%u, global min is %u\n", min_len,
                 min_offset, floatingMinLiteralMatchOffset);

    // If we can't match before the min offset, we don't need the check.
    if (min_len >= floatingMinLiteralMatchOffset) {
        DEBUG_PRINTF("no need for check, min is %u\n",
                     floatingMinLiteralMatchOffset);
        return;
    }

    assert(min_offset >= floatingMinLiteralMatchOffset);
    assert(min_offset < UINT32_MAX);

    DEBUG_PRINTF("adding lit early check, min_offset=%u\n", min_offset);
    const auto *end = prog.end_instruction();
    prog.add_before_end(make_unique<RoseInstrCheckLitEarly>(min_offset, end));
}

static
void makeGroupCheckInstruction(const RoseBuildImpl &build, u32 lit_id,
                               RoseProgram &prog) {
    const auto &info = build.literal_info.at(lit_id);

    if (!info.group_mask) {
        return;
    }
    prog.add_before_end(make_unique<RoseInstrCheckGroups>(info.group_mask));
}

static
bool hasDelayedLiteral(const RoseBuildImpl &build,
                       const vector<RoseEdge> &lit_edges) {
    auto is_delayed = [&build](u32 lit_id) { return build.isDelayed(lit_id); };
    for (const auto &e : lit_edges) {
        auto v = target(e, build.g);
        const auto &lits = build.g[v].literals;
        if (any_of(begin(lits), end(lits), is_delayed)) {
            return true;
        }
    }
    return false;
}

static
RoseProgram makeLitInitialProgram(const RoseBuildImpl &build,
                                  ProgramBuild &prog_build, u32 lit_id,
                                  const vector<RoseEdge> &lit_edges,
                                  bool is_anchored_replay_program) {
    RoseProgram program;

    // Check long literal info.
    if (!build.isDelayed(lit_id)) {
        makeCheckLiteralInstruction(build.literals.at(lit_id),
                                    prog_build.longLitLengthThreshold,
                                    program, build.cc);
    }

    // Check lit mask.
    makeCheckLitMaskInstruction(build, lit_id, program);

    // Check literal groups. This is an optimisation that we only perform for
    // delayed literals, as their groups may be switched off; ordinarily, we
    // can trust the HWLM matcher.
    if (hasDelayedLiteral(build, lit_edges)) {
        makeGroupCheckInstruction(build, lit_id, program);
    }

    // Add instructions for pushing delayed matches, if there are any.
    makePushDelayedInstructions(build.literals, prog_build,
                                build.literal_info.at(lit_id).delayed_ids,
                                program);

    // Add pre-check for early literals in the floating table.
    makeCheckLitEarlyInstruction(build, lit_id, lit_edges,
                                 prog_build.floatingMinLiteralMatchOffset,
                                 program);

    /* Check if we are able to deliever matches from the anchored table now */
    if (!is_anchored_replay_program) {
        makeAnchoredLiteralDelay(build, prog_build, lit_id, program);
    }

    return program;
}

static
bool makeRoleMultipathShufti(const vector<vector<LookEntry>> &multi_look,
                             RoseProgram &program) {
    if (multi_look.empty()) {
        return false;
    }

    // find the base offset
    assert(!multi_look[0].empty());
    s32 base_offset = multi_look[0].front().offset;
    s32 last_start = base_offset;
    s32 end_offset = multi_look[0].back().offset;
    size_t multi_len = 0;

    for (const auto &look : multi_look) {
        assert(look.size() > 0);
        multi_len += look.size();

        LIMIT_TO_AT_MOST(&base_offset, look.front().offset);
        ENSURE_AT_LEAST(&last_start, look.front().offset);
        ENSURE_AT_LEAST(&end_offset, look.back().offset);
    }

    assert(last_start < 0);

    if (end_offset - base_offset >= MULTIPATH_MAX_LEN) {
        return false;
    }

    if (multi_len <= 16) {
        multi_len = 16;
    } else if (multi_len <= 32) {
        multi_len = 32;
    } else if (multi_len <= 64) {
        multi_len = 64;
    } else {
        DEBUG_PRINTF("too long for multi-path\n");
        return false;
    }

    vector<LookEntry> linear_look;
    array<u8, 64> data_select_mask;
    data_select_mask.fill(0);
    u64a hi_bits_mask = 0;
    u64a lo_bits_mask = 0;

    for (const auto &look : multi_look) {
        assert(linear_look.size() < 64);
        lo_bits_mask |= 1LLU << linear_look.size();
        for (const auto &entry : look) {
            assert(entry.offset - base_offset < MULTIPATH_MAX_LEN);
            data_select_mask[linear_look.size()] =
                                          verify_u8(entry.offset - base_offset);
            linear_look.emplace_back(verify_s8(linear_look.size()), entry.reach);
        }
        hi_bits_mask |= 1LLU << (linear_look.size() - 1);
    }

    u8 bit_index = 0; // number of buckets
    u64a neg_mask;
    array<u8, 32> hi_mask;
    array<u8, 32> lo_mask;
    array<u8, 64> bucket_select_hi;
    array<u8, 64> bucket_select_lo;
    hi_mask.fill(0);
    lo_mask.fill(0);
    bucket_select_hi.fill(0);
    bucket_select_lo.fill(0);

    if (!getShuftiMasks(linear_look, hi_mask, lo_mask, bucket_select_hi.data(),
                        bucket_select_lo.data(), neg_mask, bit_index,
                        multi_len)) {
        return false;
    }

    DEBUG_PRINTF("hi_mask %s\n",
                 convertMaskstoString(hi_mask.data(), 16).c_str());
    DEBUG_PRINTF("lo_mask %s\n",
                 convertMaskstoString(lo_mask.data(), 16).c_str());
    DEBUG_PRINTF("bucket_select_hi %s\n",
                 convertMaskstoString(bucket_select_hi.data(), 64).c_str());
    DEBUG_PRINTF("bucket_select_lo %s\n",
                 convertMaskstoString(bucket_select_lo.data(), 64).c_str());
    DEBUG_PRINTF("data_select_mask %s\n",
                 convertMaskstoString(data_select_mask.data(), 64).c_str());
    DEBUG_PRINTF("hi_bits_mask %llx\n", hi_bits_mask);
    DEBUG_PRINTF("lo_bits_mask %llx\n", lo_bits_mask);
    DEBUG_PRINTF("neg_mask %llx\n", neg_mask);
    DEBUG_PRINTF("base_offset %d\n", base_offset);
    DEBUG_PRINTF("last_start %d\n", last_start);

    // Since we don't have 16x16 now, just call 32x16 instead.
    if (bit_index > 8) {
        assert(multi_len <= 32);
        multi_len = 32;
    }

    const auto *end_inst = program.end_instruction();
    assert(multi_len == 16 || multi_len == 32 || multi_len == 64);
    if (multi_len == 16) {
        neg_mask &= 0xffff;
        assert(!(hi_bits_mask & ~0xffffULL));
        assert(!(lo_bits_mask & ~0xffffULL));
        assert(bit_index <=8);
        array<u8, 32> nib_mask;
        copy(begin(lo_mask), begin(lo_mask) + 16, nib_mask.begin());
        copy(begin(hi_mask), begin(hi_mask) + 16, nib_mask.begin() + 16);

        auto ri = make_unique<RoseInstrCheckMultipathShufti16x8>
                  (nib_mask, bucket_select_lo, data_select_mask, hi_bits_mask,
                   lo_bits_mask, neg_mask, base_offset, last_start, end_inst);
        program.add_before_end(move(ri));
    } else if (multi_len == 32) {
        neg_mask &= 0xffffffff;
        assert(!(hi_bits_mask & ~0xffffffffULL));
        assert(!(lo_bits_mask & ~0xffffffffULL));
        if (bit_index <= 8) {
            auto ri = make_unique<RoseInstrCheckMultipathShufti32x8>
                      (hi_mask, lo_mask, bucket_select_lo, data_select_mask,
                       hi_bits_mask, lo_bits_mask, neg_mask, base_offset,
                       last_start, end_inst);
            program.add_before_end(move(ri));
        } else {
            auto ri = make_unique<RoseInstrCheckMultipathShufti32x16>
                      (hi_mask, lo_mask, bucket_select_hi, bucket_select_lo,
                       data_select_mask, hi_bits_mask, lo_bits_mask, neg_mask,
                       base_offset, last_start, end_inst);
            program.add_before_end(move(ri));
        }
    } else {
        auto ri = make_unique<RoseInstrCheckMultipathShufti64>
                  (hi_mask, lo_mask, bucket_select_lo, data_select_mask,
                   hi_bits_mask, lo_bits_mask, neg_mask, base_offset,
                   last_start, end_inst);
        program.add_before_end(move(ri));
    }
    return true;
}

static
void makeRoleMultipathLookaround(const vector<vector<LookEntry>> &multi_look,
                                 RoseProgram &program) {
    assert(!multi_look.empty());
    assert(multi_look.size() <= MAX_LOOKAROUND_PATHS);
    vector<vector<LookEntry>> ordered_look;
    set<s32> look_offset;

    assert(!multi_look[0].empty());
    s32 last_start = multi_look[0][0].offset;

    // build offset table.
    for (const auto &look : multi_look) {
        assert(look.size() > 0);
        last_start = max(last_start, (s32)look.begin()->offset);

        for (const auto &t : look) {
            look_offset.insert(t.offset);
        }
    }

    array<u8, MULTIPATH_MAX_LEN> start_mask;
    if (multi_look.size() < MAX_LOOKAROUND_PATHS) {
        start_mask.fill((1 << multi_look.size()) - 1);
    } else {
        start_mask.fill(0xff);
    }

    u32 path_idx = 0;
    for (const auto &look : multi_look) {
        for (const auto &t : look) {
            assert(t.offset >= (int)*look_offset.begin());
            size_t update_offset = t.offset - *look_offset.begin() + 1;
            if (update_offset < start_mask.size()) {
                start_mask[update_offset] &= ~(1 << path_idx);
            }
        }
        path_idx++;
    }

    for (u32 i = 1; i < MULTIPATH_MAX_LEN; i++) {
        start_mask[i] &= start_mask[i - 1];
        DEBUG_PRINTF("start_mask[%u] = %x\n", i, start_mask[i]);
    }

    assert(look_offset.size() <= MULTIPATH_MAX_LEN);

    assert(last_start < 0);

    for (const auto &offset : look_offset) {
        vector<LookEntry> multi_entry;
        multi_entry.resize(MAX_LOOKAROUND_PATHS);

        for (size_t i = 0; i < multi_look.size(); i++) {
            for (const auto &t : multi_look[i]) {
                if (t.offset == offset) {
                    multi_entry[i] = t;
                }
            }
        }
        ordered_look.emplace_back(multi_entry);
    }

    auto ri = make_unique<RoseInstrMultipathLookaround>(move(ordered_look),
                                                        last_start, start_mask,
                                                    program.end_instruction());
    program.add_before_end(move(ri));
}

static
void makeRoleLookaround(const RoseBuildImpl &build,
                        const map<RoseVertex, left_build_info> &leftfix_info,
                        RoseVertex v, RoseProgram &program) {
    if (!build.cc.grey.roseLookaroundMasks) {
        return;
    }

    vector<vector<LookEntry>> looks;

    // Lookaround from leftfix (mandatory).
    if (contains(leftfix_info, v) && leftfix_info.at(v).has_lookaround) {
        DEBUG_PRINTF("using leftfix lookaround\n");
        looks = leftfix_info.at(v).lookaround;
    }

    // We may be able to find more lookaround info (advisory) and merge it
    // in.
    if (looks.size() <= 1) {
        vector<LookEntry> look;
        vector<LookEntry> look_more;
        if (!looks.empty()) {
            look = move(looks.front());
        }
        findLookaroundMasks(build, v, look_more);
        mergeLookaround(look, look_more);
        if (!look.empty()) {
            makeLookaroundInstruction(look, program);
        }
        return;
    }

    if (!makeRoleMultipathShufti(looks, program)) {
        assert(looks.size() <= 8);
        makeRoleMultipathLookaround(looks, program);
    }
}

static
void makeRoleSuffix(const RoseBuildImpl &build,
                    const map<suffix_id, u32> &suffixes,
                    const map<u32, engine_info> &engine_info_by_queue,
                    RoseVertex v, RoseProgram &prog) {
    const auto &g = build.g;
    if (!g[v].suffix) {
        return;
    }
    assert(contains(suffixes, g[v].suffix));
    u32 queue = suffixes.at(g[v].suffix);
    u32 event;
    assert(contains(engine_info_by_queue, queue));
    const auto eng_info = engine_info_by_queue.at(queue);
    if (isContainerType(eng_info.type)) {
        auto tamaProto = g[v].suffix.tamarama.get();
        assert(tamaProto);
        event = (u32)MQE_TOP_FIRST +
                  tamaProto->top_remap.at(make_pair(g[v].index,
                                                    g[v].suffix.top));
        assert(event < MQE_INVALID);
    } else if (isMultiTopType(eng_info.type)) {
        assert(!g[v].suffix.haig);
        event = (u32)MQE_TOP_FIRST + g[v].suffix.top;
        assert(event < MQE_INVALID);
    } else {
        // DFAs/Puffs have no MQE_TOP_N support, so they get a classic TOP
        // event.
        assert(!g[v].suffix.graph || onlyOneTop(*g[v].suffix.graph));
        event = MQE_TOP;
    }

    prog.add_before_end(make_unique<RoseInstrTriggerSuffix>(queue, event));
}

static
void addInfixTriggerInstructions(vector<TriggerInfo> triggers,
                                 RoseProgram &prog) {
    // Order, de-dupe and add instructions to the end of program.
    sort_and_unique(triggers, [](const TriggerInfo &a, const TriggerInfo &b) {
        return tie(a.cancel, a.queue, a.event) <
               tie(b.cancel, b.queue, b.event);
    });
    for (const auto &ti : triggers) {
        prog.add_before_end(
             make_unique<RoseInstrTriggerInfix>(ti.cancel, ti.queue, ti.event));
    }
}

static
void makeRoleInfixTriggers(const RoseBuildImpl &build,
                           const map<RoseVertex, left_build_info> &leftfix_info,
                           const map<u32, engine_info> &engine_info_by_queue,
                           RoseVertex u, RoseProgram &program) {
    const auto &g = build.g;

    vector<TriggerInfo> triggers;

    for (const auto &e : out_edges_range(u, g)) {
        RoseVertex v = target(e, g);
        if (!g[v].left) {
            continue;
        }

        assert(contains(leftfix_info, v));
        const left_build_info &lbi = leftfix_info.at(v);
        if (lbi.has_lookaround) {
            continue;
        }

        assert(contains(engine_info_by_queue, lbi.queue));
        const auto &eng_info = engine_info_by_queue.at(lbi.queue);

        // DFAs have no TOP_N support, so they get a classic MQE_TOP event.
        u32 top;
        if (isContainerType(eng_info.type)) {
            auto tamaProto = g[v].left.tamarama.get();
            assert(tamaProto);
            top = MQE_TOP_FIRST + tamaProto->top_remap.at(
                                      make_pair(g[v].index, g[e].rose_top));
            assert(top < MQE_INVALID);
        } else if (!isMultiTopType(eng_info.type)) {
            assert(num_tops(g[v].left) == 1);
            top = MQE_TOP;
        } else {
            top = MQE_TOP_FIRST + g[e].rose_top;
            assert(top < MQE_INVALID);
        }

        triggers.emplace_back(g[e].rose_cancel_prev_top, lbi.queue, top);
    }

    addInfixTriggerInstructions(move(triggers), program);
}


/**
 * \brief True if the given vertex is a role that can only be switched on at
 * EOD.
 */
static
bool onlyAtEod(const RoseBuildImpl &tbi, RoseVertex v) {
    const RoseGraph &g = tbi.g;

    // All such roles have only (0,0) edges to vertices with the eod_accept
    // property, and no other effects (suffixes, ordinary reports, etc, etc).

    if (isLeafNode(v, g) || !g[v].reports.empty() || g[v].suffix) {
        return false;
    }

    for (const auto &e : out_edges_range(v, g)) {
        RoseVertex w = target(e, g);
        if (!g[w].eod_accept) {
            return false;
        }
        assert(!g[w].reports.empty());
        assert(g[w].literals.empty());

        if (g[e].minBound || g[e].maxBound) {
            return false;
        }
    }

    /* There is no pointing enforcing this check at runtime if
     * this role is only fired by the eod event literal */
    if (tbi.eod_event_literal_id != MO_INVALID_IDX &&
        g[v].literals.size() == 1 &&
        *g[v].literals.begin() == tbi.eod_event_literal_id) {
        return false;
    }

    return true;
}

static
void addCheckOnlyEodInstruction(RoseProgram &prog) {
    DEBUG_PRINTF("only at eod\n");
    const auto *end_inst = prog.end_instruction();
    prog.add_before_end(make_unique<RoseInstrCheckOnlyEod>(end_inst));
}

static
void makeRoleEagerEodReports(const RoseBuildImpl &build,
                         const map<RoseVertex, left_build_info> &leftfix_info,
                         bool needs_catchup, RoseVertex v,
                         RoseProgram &program) {
    RoseProgram eod_program;

    for (const auto &e : out_edges_range(v, build.g)) {
        if (canEagerlyReportAtEod(build, e)) {
            RoseProgram block;
            makeRoleReports(build, leftfix_info, needs_catchup,
                            target(e, build.g), block);
            eod_program.add_block(move(block));
        }
    }

    if (eod_program.empty()) {
        return;
    }

    if (!onlyAtEod(build, v)) {
        // The rest of our program wasn't EOD anchored, so we need to guard
        // these reports with a check.
        addCheckOnlyEodInstruction(program);
    }

    program.add_before_end(move(eod_program));
}

/** Makes a program for a role/vertex given a specific pred/in_edge. */
static
RoseProgram makeRoleProgram(const RoseBuildImpl &build,
                        const map<RoseVertex, left_build_info> &leftfix_info,
                        const map<suffix_id, u32> &suffixes,
                        const map<u32, engine_info> &engine_info_by_queue,
                        const unordered_map<RoseVertex, u32> &roleStateIndices,
                        ProgramBuild &prog_build, const RoseEdge &e) {
    const RoseGraph &g = build.g;
    auto v = target(e, g);

    RoseProgram program;

    // First, add program instructions that enforce preconditions without
    // effects.

    if (onlyAtEod(build, v)) {
        addCheckOnlyEodInstruction(program);
    }

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    // This role program may be triggered by different predecessors, with
    // different offset bounds. We must ensure we put this check/set operation
    // after the bounds check to deal with this case.
    if (in_degree(v, g) > 1) {
        assert(!build.isRootSuccessor(v));
        makeRoleCheckNotHandled(prog_build, v, program);
    }

    makeRoleLookaround(build, leftfix_info, v, program);
    makeRoleCheckLeftfix(build, leftfix_info, v, program);

    // Next, we can add program instructions that have effects. This must be
    // done as a series of blocks, as some of them (like reports) are
    // escapable.

    RoseProgram effects_block;

    RoseProgram reports_block;
    makeRoleReports(build, leftfix_info, prog_build.needs_catchup, v,
                    reports_block);
    effects_block.add_block(move(reports_block));

    RoseProgram infix_block;
    makeRoleInfixTriggers(build, leftfix_info, engine_info_by_queue, v,
                          infix_block);
    effects_block.add_block(move(infix_block));

    // Note: SET_GROUPS instruction must be after infix triggers, as an infix
    // going dead may switch off groups.
    RoseProgram groups_block;
    makeRoleGroups(build.g, prog_build, v, groups_block);
    effects_block.add_block(move(groups_block));

    RoseProgram suffix_block;
    makeRoleSuffix(build, suffixes, engine_info_by_queue, v, suffix_block);
    effects_block.add_block(move(suffix_block));

    RoseProgram state_block;
    makeRoleSetState(roleStateIndices, v, state_block);
    effects_block.add_block(move(state_block));

    // Note: EOD eager reports may generate a CHECK_ONLY_EOD instruction (if
    // the program doesn't have one already).
    RoseProgram eod_block;
    makeRoleEagerEodReports(build, leftfix_info, prog_build.needs_catchup, v,
                            eod_block);
    effects_block.add_block(move(eod_block));

    /* a 'ghost role' may do nothing if we know that its groups are already set
     * - in this case we can avoid producing a program at all. */
    if (effects_block.empty()) {
        return {};
    }

    program.add_before_end(move(effects_block));
    return program;
}

static
void makeGroupSquashInstruction(const RoseBuildImpl &build, u32 lit_id,
                                RoseProgram &prog) {
    const auto &info = build.literal_info.at(lit_id);
    if (!info.squash_group) {
        return;
    }

    DEBUG_PRINTF("squashes 0x%llx\n", info.group_mask);
    assert(info.group_mask);
    /* Note: group_mask is negated. */
    prog.add_before_end(make_unique<RoseInstrSquashGroups>(~info.group_mask));
}

namespace {
struct ProgKey {
    ProgKey(const RoseProgram &p) : prog(&p) {}

    bool operator==(const ProgKey &b) const {
        return RoseProgramEquivalence()(*prog, *b.prog);
    }

    size_t hash() const {
        return RoseProgramHash()(*prog);
    }
private:
    const RoseProgram *prog;
};
}

RoseProgram assembleProgramBlocks(vector<RoseProgram> &&blocks_in) {
    DEBUG_PRINTF("%zu blocks before dedupe\n", blocks_in.size());

    vector<RoseProgram> blocks;
    blocks.reserve(blocks_in.size()); /* to ensure stable reference for seen */

    ue2_unordered_set<ProgKey> seen;
    for (auto &block : blocks_in) {
        if (contains(seen, block)) {
            continue;
        }

        blocks.push_back(move(block));
        seen.emplace(blocks.back());
    }

    DEBUG_PRINTF("%zu blocks after dedupe\n", blocks.size());

    RoseProgram prog;
    for (auto &block : blocks) {
        /* If we have multiple blocks from different literals and any of them
         * squash groups, we will have to add a CLEAR_WORK_DONE instruction to
         * each literal program block to clear the work_done flags so that it's
         * only set if a state has been. */
        if (!prog.empty() && reads_work_done_flag(block)) {
            RoseProgram clear_block;
            clear_block.add_before_end(make_unique<RoseInstrClearWorkDone>());
            prog.add_block(move(clear_block));
        }

        prog.add_block(move(block));
    }

    return prog;
}

RoseProgram makeLiteralProgram(const RoseBuildImpl &build,
                         const map<RoseVertex, left_build_info> &leftfix_info,
                         const map<suffix_id, u32> &suffixes,
                         const map<u32, engine_info> &engine_info_by_queue,
                         const unordered_map<RoseVertex, u32> &roleStateIndices,
                         ProgramBuild &prog_build, u32 lit_id,
                         const vector<RoseEdge> &lit_edges,
                         bool is_anchored_replay_program) {
    const auto &g = build.g;

    DEBUG_PRINTF("lit id=%u, %zu lit edges\n", lit_id, lit_edges.size());

    // Construct initial program up front, as its early checks must be able
    // to jump to end and terminate processing for this literal.
    auto lit_program = makeLitInitialProgram(build, prog_build, lit_id,
                                             lit_edges,
                                             is_anchored_replay_program);

    RoseProgram role_programs;

    // Predecessor state id -> program block.
    map<u32, RoseProgram> pred_blocks;

    // Construct sparse iter sub-programs.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (build.isAnyStart(u)) {
            continue; // Root roles are not handled with sparse iterator.
        }
        DEBUG_PRINTF("sparse iter edge (%zu,%zu)\n", g[u].index,
                     g[target(e, g)].index);
        assert(contains(roleStateIndices, u));
        u32 pred_state = roleStateIndices.at(u);
        auto role_prog = makeRoleProgram(build, leftfix_info, suffixes,
                                         engine_info_by_queue, roleStateIndices,
                                         prog_build, e);
        if (!role_prog.empty()) {
            pred_blocks[pred_state].add_block(move(role_prog));
        }
    }

    // Add blocks to deal with non-root edges (triggered by sparse iterator or
    // mmbit_isset checks).
    addPredBlocks(pred_blocks, roleStateIndices.size(), role_programs);

    // Add blocks to handle root roles.
    for (const auto &e : lit_edges) {
        const auto &u = source(e, g);
        if (!build.isAnyStart(u)) {
            continue;
        }
        DEBUG_PRINTF("root edge (%zu,%zu)\n", g[u].index,
                     g[target(e, g)].index);
        auto role_prog = makeRoleProgram(build, leftfix_info, suffixes,
                                         engine_info_by_queue, roleStateIndices,
                                         prog_build, e);
        role_programs.add_block(move(role_prog));
    }

    if (lit_id == build.eod_event_literal_id) {
        /* Note: does not require the lit initial program */
        assert(build.eod_event_literal_id != MO_INVALID_IDX);
        return role_programs;
    }

    /* Instructions to run even if a role program bails out */
    RoseProgram unconditional_block;

    // Literal may squash groups.
    makeGroupSquashInstruction(build, lit_id, unconditional_block);

    role_programs.add_block(move(unconditional_block));
    lit_program.add_before_end(move(role_programs));

    return lit_program;
}

RoseProgram makeDelayRebuildProgram(const RoseBuildImpl &build,
                                    ProgramBuild &prog_build,
                                    const vector<u32> &lit_ids) {
    assert(!lit_ids.empty());
    assert(build.cc.streaming);

    vector<RoseProgram> blocks;

    for (const auto &lit_id : lit_ids) {
        DEBUG_PRINTF("lit_id=%u\n", lit_id);
        const auto &info = build.literal_info.at(lit_id);
        if (info.delayed_ids.empty()) {
            continue; // No delayed IDs, no work to do.
        }

        RoseProgram prog;
        if (!build.isDelayed(lit_id)) {
            makeCheckLiteralInstruction(build.literals.at(lit_id),
                                        prog_build.longLitLengthThreshold, prog,
                                        build.cc);
        }

        makeCheckLitMaskInstruction(build, lit_id, prog);
        makePushDelayedInstructions(build.literals, prog_build,
                                    build.literal_info.at(lit_id).delayed_ids,
                                    prog);
        blocks.push_back(move(prog));
    }

    return assembleProgramBlocks(move(blocks));
}

RoseProgram makeEodAnchorProgram(const RoseBuildImpl &build,
                                 ProgramBuild &prog_build, const RoseEdge &e,
                                 const bool multiple_preds) {
    const RoseGraph &g = build.g;
    const RoseVertex v = target(e, g);

    RoseProgram program;

    if (g[e].history == ROSE_ROLE_HISTORY_ANCH) {
        makeRoleCheckBounds(build, v, e, program);
    }

    if (multiple_preds) {
        // Only necessary when there is more than one pred.
        makeRoleCheckNotHandled(prog_build, v, program);
    }

    makeCatchup(build.rm, prog_build.needs_catchup, g[v].reports, program);

    const bool has_som = false;
    RoseProgram report_block;
    for (const auto &id : g[v].reports) {
        makeReport(build, id, has_som, report_block);
    }
    program.add_before_end(move(report_block));

    return program;
}

static
void makeCatchupMpv(const ReportManager &rm, bool needs_mpv_catchup,
                    ReportID id, RoseProgram &program) {
    if (!needs_mpv_catchup) {
        return;
    }

    const Report &report = rm.getReport(id);
    if (report.type == INTERNAL_ROSE_CHAIN) {
        return;
    }

    program.add_before_end(make_unique<RoseInstrCatchUpMpv>());
}

RoseProgram makeReportProgram(const RoseBuildImpl &build,
                              bool needs_mpv_catchup, ReportID id) {
    RoseProgram prog;

    makeCatchupMpv(build.rm, needs_mpv_catchup, id, prog);

    const bool has_som = false;
    makeReport(build, id, has_som, prog);

    return prog;
}

RoseProgram makeBoundaryProgram(const RoseBuildImpl &build,
                                const set<ReportID> &reports) {
    // Note: no CATCHUP instruction is necessary in the boundary case, as we
    // should always be caught up (and may not even have the resources in
    // scratch to support it).

    const bool has_som = false;
    RoseProgram prog;
    for (const auto &id : reports) {
        makeReport(build, id, has_som, prog);
    }

    return prog;
}

void addIncludedJumpProgram(RoseProgram &program, u32 child_offset,
                            u8 squash) {
    RoseProgram block;
    block.add_before_end(make_unique<RoseInstrIncludedJump>(child_offset,
                                                            squash));
    program.add_block(move(block));
}

static
void addPredBlockSingle(u32 pred_state, RoseProgram &pred_block,
                        RoseProgram &program) {
    // Prepend an instruction to check the pred state is on.
    const auto *end_inst = pred_block.end_instruction();
    pred_block.insert(begin(pred_block),
                      make_unique<RoseInstrCheckState>(pred_state, end_inst));
    program.add_block(move(pred_block));
}

static
void addPredBlocksAny(map<u32, RoseProgram> &pred_blocks, u32 num_states,
                      RoseProgram &program) {
    RoseProgram sparse_program;

    vector<u32> keys;
    for (const u32 &key : pred_blocks | map_keys) {
        keys.push_back(key);
    }

    const RoseInstruction *end_inst = sparse_program.end_instruction();
    auto ri = make_unique<RoseInstrSparseIterAny>(num_states, keys, end_inst);
    sparse_program.add_before_end(move(ri));

    RoseProgram &block = pred_blocks.begin()->second;

    /* we no longer need the check handled instruction as all the pred-role
     * blocks are being collapsed together */
    stripCheckHandledInstruction(block);

    sparse_program.add_before_end(move(block));
    program.add_block(move(sparse_program));
}

static
void addPredBlocksMulti(map<u32, RoseProgram> &pred_blocks,
                        u32 num_states, RoseProgram &program) {
    assert(!pred_blocks.empty());

    RoseProgram sparse_program;
    const RoseInstruction *end_inst = sparse_program.end_instruction();
    vector<pair<u32, const RoseInstruction *>> jump_table;

    // BEGIN instruction.
    auto ri_begin = make_unique<RoseInstrSparseIterBegin>(num_states, end_inst);
    RoseInstrSparseIterBegin *begin_inst = ri_begin.get();
    sparse_program.add_before_end(move(ri_begin));

    // NEXT instructions, one per pred program.
    u32 prev_key = pred_blocks.begin()->first;
    for (auto it = next(begin(pred_blocks)); it != end(pred_blocks); ++it) {
        auto ri = make_unique<RoseInstrSparseIterNext>(prev_key, begin_inst,
                                                       end_inst);
        sparse_program.add_before_end(move(ri));
        prev_key = it->first;
    }

    // Splice in each pred program after its BEGIN/NEXT.
    auto out_it = begin(sparse_program);
    for (auto &m : pred_blocks) {
        u32 key = m.first;
        RoseProgram &flat_prog = m.second;
        assert(!flat_prog.empty());
        const size_t block_len = flat_prog.size() - 1; // without INSTR_END.

        assert(dynamic_cast<const RoseInstrSparseIterBegin *>(out_it->get()) ||
               dynamic_cast<const RoseInstrSparseIterNext *>(out_it->get()));
        out_it = sparse_program.insert(++out_it, move(flat_prog));

        // Jump table target for this key is the beginning of the block we just
        // spliced in.
        jump_table.emplace_back(key, out_it->get());

        assert(distance(begin(sparse_program), out_it) + block_len <=
               sparse_program.size());
        advance(out_it, block_len);
    }

    // Write the jump table back into the SPARSE_ITER_BEGIN instruction.
    begin_inst->jump_table = move(jump_table);

    program.add_block(move(sparse_program));
}

void addPredBlocks(map<u32, RoseProgram> &pred_blocks, u32 num_states,
                   RoseProgram &program) {
    // Trim empty blocks, if any exist.
    for (auto it = pred_blocks.begin(); it != pred_blocks.end();) {
        if (it->second.empty()) {
            it = pred_blocks.erase(it);
        } else {
            ++it;
        }
    }

    const size_t num_preds = pred_blocks.size();
    if (num_preds == 0) {
        return;
    }

    if (num_preds == 1) {
        const auto head = pred_blocks.begin();
        addPredBlockSingle(head->first, head->second, program);
        return;
    }

    // First, see if all our blocks are equivalent, in which case we can
    // collapse them down into one.
    const auto &blocks = pred_blocks | map_values;
    if (all_of(begin(blocks), end(blocks), [&](const RoseProgram &block) {
            return RoseProgramEquivalence()(*begin(blocks), block);
        })) {
        DEBUG_PRINTF("all blocks equiv\n");
        addPredBlocksAny(pred_blocks, num_states, program);
        return;
    }

    addPredBlocksMulti(pred_blocks, num_states, program);
}

void applyFinalSpecialisation(RoseProgram &program) {
    assert(!program.empty());
    assert(program.back().code() == ROSE_INSTR_END);
    if (program.size() < 2) {
        return;
    }

    /* Replace the second-to-last instruction (before END) with a one-shot
     * specialisation if available. */
    auto it = next(program.rbegin());
    if (auto *ri = dynamic_cast<const RoseInstrReport *>(it->get())) {
        DEBUG_PRINTF("replacing REPORT with FINAL_REPORT\n");
        program.replace(it, make_unique<RoseInstrFinalReport>(
                                ri->onmatch, ri->offset_adjust));
    }
}

void recordLongLiterals(vector<ue2_case_string> &longLiterals,
                        const RoseProgram &program) {
    for (const auto &ri : program) {
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLit *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            longLiterals.emplace_back(ri_check->literal, false);
            continue;
        }
        if (const auto *ri_check =
                dynamic_cast<const RoseInstrCheckLongLitNocase *>(ri.get())) {
            DEBUG_PRINTF("found CHECK_LONG_LIT_NOCASE for string '%s'\n",
                         escapeString(ri_check->literal).c_str());
            longLiterals.emplace_back(ri_check->literal, true);
        }
    }
}

void recordResources(RoseResources &resources, const RoseProgram &program) {
    for (const auto &ri : program) {
        switch (ri->code()) {
        case ROSE_INSTR_TRIGGER_SUFFIX:
            resources.has_suffixes = true;
            break;
        case ROSE_INSTR_TRIGGER_INFIX:
        case ROSE_INSTR_CHECK_INFIX:
        case ROSE_INSTR_CHECK_PREFIX:
        case ROSE_INSTR_SOM_LEFTFIX:
            resources.has_leftfixes = true;
            break;
        case ROSE_INSTR_SET_STATE:
        case ROSE_INSTR_CHECK_STATE:
        case ROSE_INSTR_SPARSE_ITER_BEGIN:
        case ROSE_INSTR_SPARSE_ITER_NEXT:
            resources.has_states = true;
            break;
        case ROSE_INSTR_CHECK_GROUPS:
            resources.checks_groups = true;
            break;
        case ROSE_INSTR_PUSH_DELAYED:
            resources.has_lit_delay = true;
            break;
        case ROSE_INSTR_CHECK_LONG_LIT:
        case ROSE_INSTR_CHECK_LONG_LIT_NOCASE:
            resources.has_lit_check = true;
            break;
        default:
            break;
        }
    }
}

} // namespace ue2
