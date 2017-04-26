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

#ifndef ROSE_BUILD_PROGRAM_H
#define ROSE_BUILD_PROGRAM_H

#include "rose_build_impl.h"
#include "rose_program.h"
#include "util/bytecode_ptr.h"
#include "util/hash.h"
#include "util/make_unique.h"
#include "util/ue2_containers.h"

#include <vector>

#include <boost/range/adaptor/map.hpp>

namespace ue2 {

class RoseEngineBlob;
class RoseInstruction;

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

/* Removes any CHECK_HANDLED instructions from the given program */
void stripCheckHandledInstruction(RoseProgram &prog);

/** Returns true if the program may read the the interpreter's work_done flag */
bool reads_work_done_flag(const RoseProgram &prog);

} // namespace ue2

#endif // ROSE_BUILD_PROGRAM_H
