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

#ifndef ROSE_BUILD_PROGRAM_H
#define ROSE_BUILD_PROGRAM_H

#include "rose_build_impl.h"
#include "rose_program.h"
#include "util/bytecode_ptr.h"
#include "util/hash.h"
#include "util/make_unique.h"

#include <unordered_map>
#include <vector>

#include <boost/range/adaptor/map.hpp>

namespace ue2 {

struct LookEntry;
class RoseEngineBlob;
class RoseInstruction;
struct RoseResources;

/**
 * \brief Container for a list of program instructions.
 */
class RoseProgram {
private:
    std::vector<std::unique_ptr<RoseInstruction>> prog;

public:
    RoseProgram();
    ~RoseProgram();
    RoseProgram(const RoseProgram &) = delete;
    RoseProgram(RoseProgram &&);
    RoseProgram &operator=(const RoseProgram &) = delete;
    RoseProgram &operator=(RoseProgram &&);

    bool empty() const;

    size_t size() const { return prog.size(); }

    const RoseInstruction &back() const { return *prog.back(); }
    const RoseInstruction &front() const { return *prog.front(); }

    using iterator = decltype(prog)::iterator;
    iterator begin() { return prog.begin(); }
    iterator end() { return prog.end(); }

    using const_iterator = decltype(prog)::const_iterator;
    const_iterator begin() const { return prog.begin(); }
    const_iterator end() const { return prog.end(); }

    using reverse_iterator = decltype(prog)::reverse_iterator;
    reverse_iterator rbegin() { return prog.rbegin(); }
    reverse_iterator rend() { return prog.rend(); }

    using const_reverse_iterator = decltype(prog)::const_reverse_iterator;
    const_reverse_iterator rbegin() const { return prog.rbegin(); }
    const_reverse_iterator rend() const { return prog.rend(); }

    /** \brief Retrieve a pointer to the terminating ROSE_INSTR_END. */
    const RoseInstruction *end_instruction() const;

    static void update_targets(iterator it, iterator it_end,
                               const RoseInstruction *old_target,
                               const RoseInstruction *new_target);

    iterator insert(iterator it, std::unique_ptr<RoseInstruction> ri);

    iterator insert(iterator it, RoseProgram &&block);

    /* Note: takes iterator rather than const_iterator to support toolchains
     * with pre-C++11 standard libraries (i.e., gcc-4.8). */
    iterator erase(iterator first, iterator last);

    /**
     * \brief Adds this instruction to the program just before the terminating
     * ROSE_INSTR_END.
     */
    void add_before_end(std::unique_ptr<RoseInstruction> ri);

    /**
     * \brief Adds this block to the program just before the terminating
     * ROSE_INSTR_END.
     *
     * Any existing instruction that was jumping to end continues to do so.
     */
    void add_before_end(RoseProgram &&block);
    /**
     * \brief Append this program block, replacing our current ROSE_INSTR_END.
     *
     * Any existing instruction that was jumping to end, now leads to the newly
     * added block.
     */
    void add_block(RoseProgram &&block);

    /**
     * \brief Replace the instruction pointed to by the given iterator.
     */
    template<class Iter>
    void replace(Iter it, std::unique_ptr<RoseInstruction> ri) {
        assert(!prog.empty());

        const RoseInstruction *old_ptr = it->get();
        *it = move(ri);
        update_targets(prog.begin(), prog.end(), old_ptr, it->get());
    }
};

bytecode_ptr<char> writeProgram(RoseEngineBlob &blob,
                                const RoseProgram &program);

class RoseProgramHash {
public:
    size_t operator()(const RoseProgram &program) const;
};

class RoseProgramEquivalence {
public:
    bool operator()(const RoseProgram &prog1, const RoseProgram &prog2) const;
};

/** \brief Data only used during construction of various programs (literal,
 * anchored, delay, etc). */
struct ProgramBuild : noncopyable {
    explicit ProgramBuild(u32 fMinLitOffset, size_t longLitThresh,
                          bool catchup)
        : floatingMinLiteralMatchOffset(fMinLitOffset),
        longLitLengthThreshold(longLitThresh), needs_catchup(catchup) {
    }

    /** \brief Minimum offset of a match from the floating table. */
    const u32 floatingMinLiteralMatchOffset;

    /** \brief Long literal length threshold, used in streaming mode. */
    const size_t longLitLengthThreshold;

    /** \brief True if reports need CATCH_UP instructions to catch up suffixes,
     * outfixes etc. */
    const bool needs_catchup;

    /** \brief Mapping from vertex to key, for vertices with a
     * CHECK_NOT_HANDLED instruction. */
    std::unordered_map<RoseVertex, u32> handledKeys;

    /** \brief Mapping from Rose literal ID to anchored program index. */
    std::map<u32, u32> anchored_programs;

    /** \brief Mapping from Rose literal ID to delayed program index. */
    std::map<u32, u32> delay_programs;

    /** \brief Mapping from every vertex to the groups that must be on for that
     * vertex to be reached. */
    std::unordered_map<RoseVertex, rose_group> vertex_group_map;

    /** \brief Global bitmap of groups that can be squashed. */
    rose_group squashable_groups = 0;
};

void addEnginesEodProgram(u32 eodNfaIterOffset, RoseProgram &program);
void addSuffixesEodProgram(RoseProgram &program);
void addMatcherEodProgram(RoseProgram &program);
void addFlushCombinationProgram(RoseProgram &program);
void addLastFlushCombinationProgram(RoseProgram &program);

static constexpr u32 INVALID_QUEUE = ~0U;

struct left_build_info {
    // Constructor for an engine implementation.
    left_build_info(u32 q, u32 l, u32 t, rose_group sm,
                    const std::vector<u8> &stops, u32 max_ql, u8 cm_count,
                    const CharReach &cm_cr);

    // Constructor for a lookaround implementation.
    explicit left_build_info(const std::vector<std::vector<LookEntry>> &looks);

    u32 queue = INVALID_QUEUE; /* uniquely idents the left_build_info */
    u32 lag = 0;
    u32 transient = 0;
    rose_group squash_mask = ~rose_group{0};
    std::vector<u8> stopAlphabet;
    u32 max_queuelen = 0;
    u8 countingMiracleCount = 0;
    CharReach countingMiracleReach;
    u32 countingMiracleOffset = 0; /* populated later when laying out bytecode */
    bool has_lookaround = false;

    // alternative implementation to the NFA
    std::vector<std::vector<LookEntry>> lookaround;
};

/**
 * \brief Provides a brief summary of properties of an NFA that has already been
 * finalised and stored in the blob.
 */
struct engine_info {
    engine_info(const NFA *nfa, bool trans);

    enum NFAEngineType type;
    bool accepts_eod;
    u32 stream_size;
    u32 scratch_size;
    u32 scratch_align;
    bool transient;
};

/**
 * \brief Consumes list of program blocks corresponding to different literals,
 * checks them for duplicates and then concatenates them into one program.
 *
 * Note: if a block will squash groups, a CLEAR_WORK_DONE instruction is
 * inserted to prevent the work_done flag being contaminated by early blocks.
 */
RoseProgram assembleProgramBlocks(std::vector<RoseProgram> &&blocks);

RoseProgram makeLiteralProgram(const RoseBuildImpl &build,
                    const std::map<RoseVertex, left_build_info> &leftfix_info,
                    const std::map<suffix_id, u32> &suffixes,
                    const std::map<u32, engine_info> &engine_info_by_queue,
                    const std::unordered_map<RoseVertex, u32> &roleStateIndices,
                    ProgramBuild &prog_build, u32 lit_id,
                    const std::vector<RoseEdge> &lit_edges,
                    bool is_anchored_replay_program);

RoseProgram makeDelayRebuildProgram(const RoseBuildImpl &build,
                                    ProgramBuild &prog_build,
                                    const std::vector<u32> &lit_ids);

RoseProgram makeEodAnchorProgram(const RoseBuildImpl &build,
                                 ProgramBuild &prog_build, const RoseEdge &e,
                                 const bool multiple_preds);

RoseProgram makeReportProgram(const RoseBuildImpl &build,
                              bool needs_mpv_catchup, ReportID id);

RoseProgram makeBoundaryProgram(const RoseBuildImpl &build,
                                const std::set<ReportID> &reports);

struct TriggerInfo {
    TriggerInfo(bool c, u32 q, u32 e) : cancel(c), queue(q), event(e) {}
    bool cancel;
    u32 queue;
    u32 event;

    bool operator==(const TriggerInfo &b) const {
        return cancel == b.cancel && queue == b.queue && event == b.event;
    }
};

void addPredBlocks(std::map<u32, RoseProgram> &pred_blocks, u32 num_states,
                   RoseProgram &program);

void applyFinalSpecialisation(RoseProgram &program);

void recordLongLiterals(std::vector<ue2_case_string> &longLiterals,
                        const RoseProgram &program);

void recordResources(RoseResources &resources, const RoseProgram &program);

void addIncludedJumpProgram(RoseProgram &program, u32 child_offset, u8 squash);
} // namespace ue2

#endif // ROSE_BUILD_PROGRAM_H
