/*
 * Copyright (c) 2016-2017, Intel Corporation
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

#include "rose_build_instructions.h"

#include <algorithm>
#include <cstring>

using namespace std;

namespace ue2 {

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
        boost::hash_combine(v, ri->hash());
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

bool reads_work_done_flag(const RoseProgram &prog) {
    for (const auto &ri : prog) {
        if (dynamic_cast<const RoseInstrSquashGroups *>(ri.get())) {
            return true;
        }
    }
    return false;
}

} // namespace ue2
