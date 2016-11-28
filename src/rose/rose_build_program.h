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
#include "som/som_operation.h"
#include "util/alloc.h"
#include "util/container.h"
#include "util/hash.h"
#include "util/make_unique.h"
#include "util/ue2_containers.h"
#include "util/ue2string.h"

#include <algorithm>
#include <array>
#include <vector>

#include <boost/functional/hash/hash_fwd.hpp>
#include <boost/range/adaptor/map.hpp>

namespace ue2 {

class RoseEngineBlob;

/**
 * \brief Abstract base class representing a single Rose instruction.
 */
class RoseInstruction {
public:
    virtual ~RoseInstruction();

    /** \brief Opcode used for the instruction in the bytecode. */
    virtual RoseInstructionCode code() const = 0;

    /**
     * \brief Simple hash used for program equivalence.
     *
     * Note that pointers (jumps, for example) should not be used when
     * calculating the hash: they will be converted to instruction offsets when
     * compared later.
     */
    virtual size_t hash() const = 0;

    /** \brief Length of the bytecode instruction in bytes. */
    virtual size_t byte_length() const = 0;

    using OffsetMap = unordered_map<const RoseInstruction *, u32>;

    /**
     * \brief Writes a concrete implementation of this instruction.
     *
     * Other data that this instruction depends on is written directly into the
     * blob, while the instruction structure itself (of size given by
     * the byte_length() function) is written to dest.
     */
    virtual void write(void *dest, RoseEngineBlob &blob,
                       const OffsetMap &offset_map) const = 0;

    /**
     * \brief Update a target pointer.
     *
     * If this instruction contains any reference to the old target, replace it
     * with the new one.
     */
    virtual void update_target(const RoseInstruction *old_target,
                               const RoseInstruction *new_target) = 0;

    /**
     * \brief True if these instructions are equivalent within their own
     * programs.
     *
     * Checks that any pointers to other instructions point to the same
     * offsets.
     */
    bool equiv(const RoseInstruction &other, const OffsetMap &offsets,
               const OffsetMap &other_offsets) const {
        return equiv_impl(other, offsets, other_offsets);
    }

private:
    virtual bool equiv_impl(const RoseInstruction &other,
                            const OffsetMap &offsets,
                            const OffsetMap &other_offsets) const = 0;
};

/**
 * \brief Templated implementation class to handle boring boilerplate code.
 */
template<RoseInstructionCode Opcode, class ImplType, class RoseInstrType>
class RoseInstrBase : public RoseInstruction {
protected:
    static constexpr RoseInstructionCode opcode = Opcode;
    using impl_type = ImplType;

public:
    RoseInstructionCode code() const override { return opcode; }

    size_t byte_length() const override {
        return sizeof(impl_type);
    }

    /**
     * Note: this implementation simply zeroes the destination region and
     * writes in the correct opcode. This is sufficient for trivial
     * instructions, but instructions with data members will want to override
     * it.
     */
    void write(void *dest, RoseEngineBlob &,
               const RoseInstruction::OffsetMap &) const override {
        assert(dest != nullptr);
        assert(ISALIGNED_N(dest, ROSE_INSTR_MIN_ALIGN));

        impl_type *inst = static_cast<impl_type *>(dest);
        memset(inst, 0, sizeof(impl_type));
        inst->code = verify_u8(opcode);
    }

private:
    bool equiv_impl(const RoseInstruction &other, const OffsetMap &offsets,
                    const OffsetMap &other_offsets) const override {
        const auto *ri_that = dynamic_cast<const RoseInstrType *>(&other);
        if (!ri_that) {
            return false;
        }
        const auto *ri_this = dynamic_cast<const RoseInstrType *>(this);
        assert(ri_this);
        return ri_this->equiv_to(*ri_that, offsets, other_offsets);
    }
};

/**
 * \brief Refinement of RoseInstrBase to use for instructions that have
 * just a single target member, called "target".
 */
template<RoseInstructionCode Opcode, class ImplType, class RoseInstrType>
class RoseInstrBaseOneTarget
    : public RoseInstrBase<Opcode, ImplType, RoseInstrType> {
public:
    void update_target(const RoseInstruction *old_target,
                       const RoseInstruction *new_target) override {
        RoseInstrType *ri = dynamic_cast<RoseInstrType *>(this);
        assert(ri);
        if (ri->target == old_target) {
            ri->target = new_target;
        }
    }
};

/**
 * \brief Refinement of RoseInstrBase to use for instructions that have no
 * targets.
 */
template<RoseInstructionCode Opcode, class ImplType, class RoseInstrType>
class RoseInstrBaseNoTargets
    : public RoseInstrBase<Opcode, ImplType, RoseInstrType> {
public:
    void update_target(const RoseInstruction *,
                       const RoseInstruction *) override {}
};

/**
 * \brief Refinement of RoseInstrBaseNoTargets to use for instructions that
 * have no members at all, just an opcode.
 */
template<RoseInstructionCode Opcode, class ImplType, class RoseInstrType>
class RoseInstrBaseTrivial
    : public RoseInstrBaseNoTargets<Opcode, ImplType, RoseInstrType> {
public:
    virtual bool operator==(const RoseInstrType &) const { return true; }

    size_t hash() const override {
        return boost::hash_value(static_cast<int>(Opcode));
    }

    bool equiv_to(const RoseInstrType &, const RoseInstruction::OffsetMap &,
                  const RoseInstruction::OffsetMap &) const {
        return true;
    }
};

////
//// Concrete implementation classes start here.
////

class RoseInstrAnchoredDelay
    : public RoseInstrBaseOneTarget<ROSE_INSTR_ANCHORED_DELAY,
                                    ROSE_STRUCT_ANCHORED_DELAY,
                                    RoseInstrAnchoredDelay> {
public:
    rose_group groups;
    const RoseInstruction *target;

    RoseInstrAnchoredDelay(rose_group groups_in,
                           const RoseInstruction *target_in)
        : groups(groups_in), target(target_in) {}

    bool operator==(const RoseInstrAnchoredDelay &ri) const {
        return groups == ri.groups && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), groups);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrAnchoredDelay &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return groups == ri.groups &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckLitEarly
    : public RoseInstrBaseNoTargets<ROSE_INSTR_CHECK_LIT_EARLY,
                                    ROSE_STRUCT_CHECK_LIT_EARLY,
                                    RoseInstrCheckLitEarly> {
public:
    u32 min_offset;

    explicit RoseInstrCheckLitEarly(u32 min) : min_offset(min) {}

    bool operator==(const RoseInstrCheckLitEarly &ri) const {
        return min_offset == ri.min_offset;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), min_offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckLitEarly &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return min_offset == ri.min_offset;
    }
};

class RoseInstrCheckGroups
    : public RoseInstrBaseNoTargets<ROSE_INSTR_CHECK_GROUPS,
                                    ROSE_STRUCT_CHECK_GROUPS,
                                    RoseInstrCheckGroups> {
public:
    rose_group groups;

    explicit RoseInstrCheckGroups(rose_group groups_in) : groups(groups_in) {}

    bool operator==(const RoseInstrCheckGroups &ri) const {
        return groups == ri.groups;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), groups);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckGroups &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return groups == ri.groups;
    }
};

class RoseInstrCheckOnlyEod
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_ONLY_EOD,
                                    ROSE_STRUCT_CHECK_ONLY_EOD,
                                    RoseInstrCheckOnlyEod> {
public:
    const RoseInstruction *target;

    explicit RoseInstrCheckOnlyEod(const RoseInstruction *target_in)
        : target(target_in) {}

    bool operator==(const RoseInstrCheckOnlyEod &ri) const {
        return target == ri.target;
    }

    size_t hash() const override {
        return boost::hash_value(static_cast<int>(opcode));
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckOnlyEod &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckBounds
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_BOUNDS,
                                    ROSE_STRUCT_CHECK_BOUNDS,
                                    RoseInstrCheckBounds> {
public:
    u64a min_bound;
    u64a max_bound;
    const RoseInstruction *target;

    RoseInstrCheckBounds(u64a min, u64a max, const RoseInstruction *target_in)
        : min_bound(min), max_bound(max), target(target_in) {}

    bool operator==(const RoseInstrCheckBounds &ri) const {
        return min_bound == ri.min_bound && max_bound == ri.max_bound &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), min_bound, max_bound);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckBounds &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return min_bound == ri.min_bound && max_bound == ri.max_bound &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckNotHandled
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_NOT_HANDLED,
                                    ROSE_STRUCT_CHECK_NOT_HANDLED,
                                    RoseInstrCheckNotHandled> {
public:
    u32 key;
    const RoseInstruction *target;

    RoseInstrCheckNotHandled(u32 key_in, const RoseInstruction *target_in)
        : key(key_in), target(target_in) {}

    bool operator==(const RoseInstrCheckNotHandled &ri) const {
        return key == ri.key && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), key);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckNotHandled &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return key == ri.key &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckSingleLookaround
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_SINGLE_LOOKAROUND,
                                    ROSE_STRUCT_CHECK_SINGLE_LOOKAROUND,
                                    RoseInstrCheckSingleLookaround> {
public:
    s8 offset;
    u32 reach_index;
    const RoseInstruction *target;

    RoseInstrCheckSingleLookaround(s8 offset_in, u32 reach_index_in,
                                   const RoseInstruction *target_in)
        : offset(offset_in), reach_index(reach_index_in), target(target_in) {}

    bool operator==(const RoseInstrCheckSingleLookaround &ri) const {
        return offset == ri.offset && reach_index == ri.reach_index &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), offset, reach_index);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckSingleLookaround &ri,
                  const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return offset == ri.offset && reach_index == ri.reach_index &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckLookaround
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_LOOKAROUND,
                                    ROSE_STRUCT_CHECK_LOOKAROUND,
                                    RoseInstrCheckLookaround> {
public:
    u32 index;
    u32 count;
    const RoseInstruction *target;

    RoseInstrCheckLookaround(u32 index_in, u32 count_in,
                             const RoseInstruction *target_in)
        : index(index_in), count(count_in), target(target_in) {}

    bool operator==(const RoseInstrCheckLookaround &ri) const {
        return index == ri.index && count == ri.count && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), index, count);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckLookaround &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return index == ri.index && count == ri.count &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckMask
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_MASK,
                                    ROSE_STRUCT_CHECK_MASK,
                                    RoseInstrCheckMask> {
public:
    u64a and_mask;
    u64a cmp_mask;
    u64a neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckMask(u64a and_mask_in, u64a cmp_mask_in, u64a neg_mask_in,
                       s32 offset_in, const RoseInstruction *target_in)
        : and_mask(and_mask_in), cmp_mask(cmp_mask_in), neg_mask(neg_mask_in),
          offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckMask &ri) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), and_mask, cmp_mask, neg_mask,
                        offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckMask &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckMask32
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_MASK_32,
                                    ROSE_STRUCT_CHECK_MASK_32,
                                    RoseInstrCheckMask32> {
public:
    std::array<u8, 32> and_mask;
    std::array<u8, 32> cmp_mask;
    u32 neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckMask32(std::array<u8, 32> and_mask_in,
                         std::array<u8, 32> cmp_mask_in, u32 neg_mask_in,
                         s32 offset_in, const RoseInstruction *target_in)
        : and_mask(move(and_mask_in)), cmp_mask(move(cmp_mask_in)),
          neg_mask(neg_mask_in), offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckMask32 &ri) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), and_mask, cmp_mask, neg_mask,
                        offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckMask32 &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckByte
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_BYTE,
                                    ROSE_STRUCT_CHECK_BYTE,
                                    RoseInstrCheckByte> {
public:
    u8 and_mask;
    u8 cmp_mask;
    u8 negation;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckByte(u8 and_mask_in, u8 cmp_mask_in, u8 negation_in,
                       s32 offset_in, const RoseInstruction *target_in)
        : and_mask(and_mask_in), cmp_mask(cmp_mask_in), negation(negation_in),
          offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckByte &ri) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               negation == ri.negation && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), and_mask, cmp_mask, negation,
                        offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckByte &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return and_mask == ri.and_mask && cmp_mask == ri.cmp_mask &&
               negation == ri.negation && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckShufti16x8
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_SHUFTI_16x8,
                                    ROSE_STRUCT_CHECK_SHUFTI_16x8,
                                    RoseInstrCheckShufti16x8> {
public:
    std::array<u8, 32> nib_mask;
    std::array<u8, 16> bucket_select_mask;
    u32 neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckShufti16x8(std::array<u8, 32> nib_mask_in,
                             std::array<u8, 16> bucket_select_mask_in,
                             u32 neg_mask_in, s32 offset_in,
                             const RoseInstruction *target_in)
        : nib_mask(move(nib_mask_in)),
          bucket_select_mask(move(bucket_select_mask_in)),
          neg_mask(neg_mask_in), offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckShufti16x8 &ri) const {
        return nib_mask == ri.nib_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), nib_mask,
                        bucket_select_mask, neg_mask, offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckShufti16x8 &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return nib_mask == ri.nib_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckShufti32x8
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_SHUFTI_32x8,
                                    ROSE_STRUCT_CHECK_SHUFTI_32x8,
                                    RoseInstrCheckShufti32x8> {
public:
    std::array<u8, 16> hi_mask;
    std::array<u8, 16> lo_mask;
    std::array<u8, 32> bucket_select_mask;
    u32 neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckShufti32x8(std::array<u8, 16> hi_mask_in,
                             std::array<u8, 16> lo_mask_in,
                             std::array<u8, 32> bucket_select_mask_in,
                             u32 neg_mask_in, s32 offset_in,
                             const RoseInstruction *target_in)
        : hi_mask(move(hi_mask_in)), lo_mask(move(lo_mask_in)),
          bucket_select_mask(move(bucket_select_mask_in)),
          neg_mask(neg_mask_in), offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckShufti32x8 &ri) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), hi_mask, lo_mask,
                        bucket_select_mask, neg_mask, offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckShufti32x8 &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckShufti16x16
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_SHUFTI_16x16,
                                    ROSE_STRUCT_CHECK_SHUFTI_16x16,
                                    RoseInstrCheckShufti16x16> {
public:
    std::array<u8, 32> hi_mask;
    std::array<u8, 32> lo_mask;
    std::array<u8, 32> bucket_select_mask;
    u32 neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckShufti16x16(std::array<u8, 32> hi_mask_in,
                              std::array<u8, 32> lo_mask_in,
                              std::array<u8, 32> bucket_select_mask_in,
                              u32 neg_mask_in, s32 offset_in,
                              const RoseInstruction *target_in)
        : hi_mask(move(hi_mask_in)), lo_mask(move(lo_mask_in)),
          bucket_select_mask(move(bucket_select_mask_in)),
          neg_mask(neg_mask_in), offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckShufti16x16 &ri) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), hi_mask, lo_mask,
                        bucket_select_mask, neg_mask, offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckShufti16x16 &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask == ri.bucket_select_mask &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckShufti32x16
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_SHUFTI_32x16,
                                    ROSE_STRUCT_CHECK_SHUFTI_32x16,
                                    RoseInstrCheckShufti32x16> {
public:
    std::array<u8, 32> hi_mask;
    std::array<u8, 32> lo_mask;
    std::array<u8, 32> bucket_select_mask_hi;
    std::array<u8, 32> bucket_select_mask_lo;
    u32 neg_mask;
    s32 offset;
    const RoseInstruction *target;

    RoseInstrCheckShufti32x16(std::array<u8, 32> hi_mask_in,
                              std::array<u8, 32> lo_mask_in,
                              std::array<u8, 32> bucket_select_mask_hi_in,
                              std::array<u8, 32> bucket_select_mask_lo_in,
                              u32 neg_mask_in, s32 offset_in,
                              const RoseInstruction *target_in)
        : hi_mask(move(hi_mask_in)), lo_mask(move(lo_mask_in)),
          bucket_select_mask_hi(move(bucket_select_mask_hi_in)),
          bucket_select_mask_lo(move(bucket_select_mask_lo_in)),
          neg_mask(neg_mask_in), offset(offset_in), target(target_in) {}

    bool operator==(const RoseInstrCheckShufti32x16 &ri) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask_hi == ri.bucket_select_mask_hi &&
               bucket_select_mask_lo == ri.bucket_select_mask_lo &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), hi_mask, lo_mask,
                        bucket_select_mask_hi, bucket_select_mask_lo,
                        neg_mask, offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckShufti32x16 &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return hi_mask == ri.hi_mask && lo_mask == ri.lo_mask &&
               bucket_select_mask_hi == ri.bucket_select_mask_hi &&
               bucket_select_mask_lo == ri.bucket_select_mask_lo &&
               neg_mask == ri.neg_mask && offset == ri.offset &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckInfix
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_INFIX,
                                    ROSE_STRUCT_CHECK_INFIX,
                                    RoseInstrCheckInfix> {
public:
    u32 queue;
    u32 lag;
    ReportID report;
    const RoseInstruction *target;

    RoseInstrCheckInfix(u32 queue_in, u32 lag_in, ReportID report_in,
                        const RoseInstruction *target_in)
        : queue(queue_in), lag(lag_in), report(report_in), target(target_in) {}

    bool operator==(const RoseInstrCheckInfix &ri) const {
        return queue == ri.queue && lag == ri.lag && report == ri.report &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), queue, lag, report);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckInfix &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return queue == ri.queue && lag == ri.lag && report == ri.report &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckPrefix
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_PREFIX,
                                    ROSE_STRUCT_CHECK_PREFIX,
                                    RoseInstrCheckPrefix> {
public:
    u32 queue;
    u32 lag;
    ReportID report;
    const RoseInstruction *target;

    RoseInstrCheckPrefix(u32 queue_in, u32 lag_in, ReportID report_in,
                         const RoseInstruction *target_in)
        : queue(queue_in), lag(lag_in), report(report_in), target(target_in) {}

    bool operator==(const RoseInstrCheckPrefix &ri) const {
        return queue == ri.queue && lag == ri.lag && report == ri.report &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), queue, lag, report);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckPrefix &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return queue == ri.queue && lag == ri.lag && report == ri.report &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrPushDelayed
    : public RoseInstrBaseNoTargets<ROSE_INSTR_PUSH_DELAYED,
                                    ROSE_STRUCT_PUSH_DELAYED,
                                    RoseInstrPushDelayed> {
public:
    u8 delay;
    u32 index;

    RoseInstrPushDelayed(u8 delay_in, u32 index_in)
        : delay(delay_in), index(index_in) {}

    bool operator==(const RoseInstrPushDelayed &ri) const {
        return delay == ri.delay && index == ri.index;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), delay, index);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrPushDelayed &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return delay == ri.delay && index == ri.index;
    }
};

class RoseInstrRecordAnchored
    : public RoseInstrBaseNoTargets<ROSE_INSTR_RECORD_ANCHORED,
                                    ROSE_STRUCT_RECORD_ANCHORED,
                                    RoseInstrRecordAnchored> {
public:
    u32 id;

    explicit RoseInstrRecordAnchored(u32 id_in) : id(id_in) {}

    bool operator==(const RoseInstrRecordAnchored &ri) const {
        return id == ri.id;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), id);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrRecordAnchored &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return id == ri.id;
    }
};

class RoseInstrCatchUp
    : public RoseInstrBaseTrivial<ROSE_INSTR_CATCH_UP, ROSE_STRUCT_CATCH_UP,
                                  RoseInstrCatchUp> {
public:
    ~RoseInstrCatchUp() override;
};

class RoseInstrCatchUpMpv
    : public RoseInstrBaseTrivial<ROSE_INSTR_CATCH_UP_MPV,
                                  ROSE_STRUCT_CATCH_UP_MPV,
                                  RoseInstrCatchUpMpv> {
public:
    ~RoseInstrCatchUpMpv() override;
};

class RoseInstrSomAdjust
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SOM_ADJUST,
                                    ROSE_STRUCT_SOM_ADJUST,
                                    RoseInstrSomAdjust> {
public:
    u32 distance;

    explicit RoseInstrSomAdjust(u32 distance_in) : distance(distance_in) {}

    bool operator==(const RoseInstrSomAdjust &ri) const {
        return distance == ri.distance;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), distance);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSomAdjust &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return distance == ri.distance;
    }
};

class RoseInstrSomLeftfix
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SOM_LEFTFIX,
                                    ROSE_STRUCT_SOM_LEFTFIX,
                                    RoseInstrSomLeftfix> {
public:
    u32 queue;
    u32 lag;

    RoseInstrSomLeftfix(u32 queue_in, u32 lag_in)
        : queue(queue_in), lag(lag_in) {}

    bool operator==(const RoseInstrSomLeftfix &ri) const {
        return queue == ri.queue && lag == ri.lag;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), queue, lag);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSomLeftfix &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return queue == ri.queue && lag == ri.lag;
    }
};

class RoseInstrSomFromReport
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SOM_FROM_REPORT,
                                    ROSE_STRUCT_SOM_FROM_REPORT,
                                    RoseInstrSomFromReport> {
public:
    som_operation som;

    RoseInstrSomFromReport() {
        std::memset(&som, 0, sizeof(som));
    }

    bool operator==(const RoseInstrSomFromReport &ri) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), som.type, som.onmatch);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSomFromReport &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }
};

class RoseInstrSomZero
    : public RoseInstrBaseTrivial<ROSE_INSTR_SOM_ZERO, ROSE_STRUCT_SOM_ZERO,
                                  RoseInstrSomZero> {
public:
    ~RoseInstrSomZero() override;
};

class RoseInstrTriggerInfix
    : public RoseInstrBaseNoTargets<ROSE_INSTR_TRIGGER_INFIX,
                                    ROSE_STRUCT_TRIGGER_INFIX,
                                    RoseInstrTriggerInfix> {
public:
    u8 cancel;
    u32 queue;
    u32 event;

    RoseInstrTriggerInfix(u8 cancel_in, u32 queue_in, u32 event_in)
        : cancel(cancel_in), queue(queue_in), event(event_in) {}

    bool operator==(const RoseInstrTriggerInfix &ri) const {
        return cancel == ri.cancel && queue == ri.queue && event == ri.event;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), cancel, queue, event);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrTriggerInfix &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return cancel == ri.cancel && queue == ri.queue && event == ri.event;
    }
};

class RoseInstrTriggerSuffix
    : public RoseInstrBaseNoTargets<ROSE_INSTR_TRIGGER_SUFFIX,
                                    ROSE_STRUCT_TRIGGER_SUFFIX,
                                    RoseInstrTriggerSuffix> {
public:
    u32 queue;
    u32 event;

    RoseInstrTriggerSuffix(u32 queue_in, u32 event_in)
        : queue(queue_in), event(event_in) {}

    bool operator==(const RoseInstrTriggerSuffix &ri) const {
        return queue == ri.queue && event == ri.event;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), queue, event);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrTriggerSuffix &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return queue == ri.queue && event == ri.event;
    }
};

class RoseInstrDedupe
    : public RoseInstrBaseOneTarget<ROSE_INSTR_DEDUPE, ROSE_STRUCT_DEDUPE,
                                    RoseInstrDedupe> {
public:
    u8 quash_som;
    u32 dkey;
    s32 offset_adjust;
    const RoseInstruction *target;

    RoseInstrDedupe(u8 quash_som_in, u32 dkey_in, s32 offset_adjust_in,
                    const RoseInstruction *target_in)
        : quash_som(quash_som_in), dkey(dkey_in),
          offset_adjust(offset_adjust_in), target(target_in) {}

    bool operator==(const RoseInstrDedupe &ri) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               offset_adjust == ri.offset_adjust && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), quash_som, dkey,
                        offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrDedupe &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               offset_adjust == ri.offset_adjust &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrDedupeSom
    : public RoseInstrBaseOneTarget<ROSE_INSTR_DEDUPE_SOM,
                                    ROSE_STRUCT_DEDUPE_SOM,
                                    RoseInstrDedupeSom> {
public:
    u8 quash_som;
    u32 dkey;
    s32 offset_adjust;
    const RoseInstruction *target;

    RoseInstrDedupeSom(u8 quash_som_in, u32 dkey_in, s32 offset_adjust_in,
                       const RoseInstruction *target_in)
        : quash_som(quash_som_in), dkey(dkey_in),
          offset_adjust(offset_adjust_in), target(target_in) {}

    bool operator==(const RoseInstrDedupeSom &ri) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               offset_adjust == ri.offset_adjust && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), quash_som, dkey,
                        offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrDedupeSom &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               offset_adjust == ri.offset_adjust &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrReportChain
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_CHAIN,
                                    ROSE_STRUCT_REPORT_CHAIN,
                                    RoseInstrReportChain> {
public:
    u32 event;
    u64a top_squash_distance;

    RoseInstrReportChain(u32 event_in, u32 top_squash_distance_in)
        : event(event_in), top_squash_distance(top_squash_distance_in) {}

    bool operator==(const RoseInstrReportChain &ri) const {
        return event == ri.event &&
               top_squash_distance == ri.top_squash_distance;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), event, top_squash_distance);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportChain &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return event == ri.event &&
               top_squash_distance == ri.top_squash_distance;
    }
};

class RoseInstrReportSomInt
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_SOM_INT,
                                    ROSE_STRUCT_REPORT_SOM_INT,
                                    RoseInstrReportSomInt> {
public:
    som_operation som;

    RoseInstrReportSomInt() {
        std::memset(&som, 0, sizeof(som));
    }

    bool operator==(const RoseInstrReportSomInt &ri) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), som.type, som.onmatch);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportSomInt &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }
};

class RoseInstrReportSomAware
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_SOM_AWARE,
                                    ROSE_STRUCT_REPORT_SOM_AWARE,
                                    RoseInstrReportSomAware> {
public:
    som_operation som;

    RoseInstrReportSomAware() {
        std::memset(&som, 0, sizeof(som));
    }

    bool operator==(const RoseInstrReportSomAware &ri) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), som.type, som.onmatch);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportSomAware &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return std::memcmp(&som, &ri.som, sizeof(som)) == 0;
    }
};

class RoseInstrReport
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT, ROSE_STRUCT_REPORT,
                                    RoseInstrReport> {
public:
    ReportID onmatch;
    s32 offset_adjust;

    RoseInstrReport(ReportID onmatch_in, s32 offset_adjust_in)
        : onmatch(onmatch_in), offset_adjust(offset_adjust_in) {}

    bool operator==(const RoseInstrReport &ri) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), onmatch, offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReport &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }
};

class RoseInstrReportExhaust
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_EXHAUST,
                                    ROSE_STRUCT_REPORT_EXHAUST,
                                    RoseInstrReportExhaust> {
public:
    ReportID onmatch;
    s32 offset_adjust;
    u32 ekey;

    RoseInstrReportExhaust(ReportID onmatch_in, s32 offset_adjust_in,
                           u32 ekey_in)
        : onmatch(onmatch_in), offset_adjust(offset_adjust_in), ekey(ekey_in) {}

    bool operator==(const RoseInstrReportExhaust &ri) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               ekey == ri.ekey;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), onmatch, offset_adjust, ekey);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportExhaust &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               ekey == ri.ekey;
    }
};

class RoseInstrReportSom
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_SOM,
                                    ROSE_STRUCT_REPORT_SOM,
                                    RoseInstrReportSom> {
public:
    ReportID onmatch;
    s32 offset_adjust;

    RoseInstrReportSom(ReportID onmatch_in, s32 offset_adjust_in)
        : onmatch(onmatch_in), offset_adjust(offset_adjust_in) {}

    bool operator==(const RoseInstrReportSom &ri) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), onmatch, offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportSom &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }
};

class RoseInstrReportSomExhaust
    : public RoseInstrBaseNoTargets<ROSE_INSTR_REPORT_SOM_EXHAUST,
                                    ROSE_STRUCT_REPORT_SOM_EXHAUST,
                                    RoseInstrReportSomExhaust> {
public:
    ReportID onmatch;
    s32 offset_adjust;
    u32 ekey;

    RoseInstrReportSomExhaust(ReportID onmatch_in, s32 offset_adjust_in,
                              u32 ekey_in)
        : onmatch(onmatch_in), offset_adjust(offset_adjust_in), ekey(ekey_in) {}

    bool operator==(const RoseInstrReportSomExhaust &ri) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               ekey == ri.ekey;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), onmatch, offset_adjust, ekey);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrReportSomExhaust &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               ekey == ri.ekey;
    }
};

class RoseInstrDedupeAndReport
    : public RoseInstrBaseOneTarget<ROSE_INSTR_DEDUPE_AND_REPORT,
                                    ROSE_STRUCT_DEDUPE_AND_REPORT,
                                    RoseInstrDedupeAndReport> {
public:
    u8 quash_som;
    u32 dkey;
    ReportID onmatch;
    s32 offset_adjust;
    const RoseInstruction *target;

    RoseInstrDedupeAndReport(u8 quash_som_in, u32 dkey_in, ReportID onmatch_in,
                             s32 offset_adjust_in,
                             const RoseInstruction *target_in)
        : quash_som(quash_som_in), dkey(dkey_in), onmatch(onmatch_in),
          offset_adjust(offset_adjust_in), target(target_in) {}

    bool operator==(const RoseInstrDedupeAndReport &ri) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), quash_som, dkey, onmatch,
                        offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrDedupeAndReport &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return quash_som == ri.quash_som && dkey == ri.dkey &&
               onmatch == ri.onmatch && offset_adjust == ri.offset_adjust &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrFinalReport
    : public RoseInstrBaseNoTargets<ROSE_INSTR_FINAL_REPORT,
                                    ROSE_STRUCT_FINAL_REPORT,
                                    RoseInstrFinalReport> {
public:
    ReportID onmatch;
    s32 offset_adjust;

    RoseInstrFinalReport(ReportID onmatch_in, s32 offset_adjust_in)
        : onmatch(onmatch_in), offset_adjust(offset_adjust_in) {}

    bool operator==(const RoseInstrFinalReport &ri) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), onmatch, offset_adjust);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrFinalReport &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return onmatch == ri.onmatch && offset_adjust == ri.offset_adjust;
    }
};

class RoseInstrCheckExhausted
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_EXHAUSTED,
                                    ROSE_STRUCT_CHECK_EXHAUSTED,
                                    RoseInstrCheckExhausted> {
public:
    u32 ekey;
    const RoseInstruction *target;

    RoseInstrCheckExhausted(u32 ekey_in, const RoseInstruction *target_in)
        : ekey(ekey_in), target(target_in) {}

    bool operator==(const RoseInstrCheckExhausted &ri) const {
        return ekey == ri.ekey && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), ekey);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckExhausted &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return ekey == ri.ekey &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckMinLength
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_MIN_LENGTH,
                                    ROSE_STRUCT_CHECK_MIN_LENGTH,
                                    RoseInstrCheckMinLength> {
public:
    s32 end_adj;
    u64a min_length;
    const RoseInstruction *target;

    RoseInstrCheckMinLength(s32 end_adj_in, u64a min_length_in,
                            const RoseInstruction *target_in)
        : end_adj(end_adj_in), min_length(min_length_in), target(target_in) {}

    bool operator==(const RoseInstrCheckMinLength &ri) const {
        return end_adj == ri.end_adj && min_length == ri.min_length &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), end_adj, min_length);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckMinLength &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return end_adj == ri.end_adj && min_length == ri.min_length &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrSetState
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SET_STATE, ROSE_STRUCT_SET_STATE,
                                    RoseInstrSetState> {
public:
    u32 index;

    explicit RoseInstrSetState(u32 index_in) : index(index_in) {}

    bool operator==(const RoseInstrSetState &ri) const {
        return index == ri.index;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), index);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSetState &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return index == ri.index;
    }
};

class RoseInstrSetGroups
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SET_GROUPS,
                                    ROSE_STRUCT_SET_GROUPS,
                                    RoseInstrSetGroups> {
public:
    rose_group groups;

    explicit RoseInstrSetGroups(rose_group groups_in) : groups(groups_in) {}

    bool operator==(const RoseInstrSetGroups &ri) const {
        return groups == ri.groups;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), groups);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSetGroups &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return groups == ri.groups;
    }
};

class RoseInstrSquashGroups
    : public RoseInstrBaseNoTargets<ROSE_INSTR_SQUASH_GROUPS,
                                    ROSE_STRUCT_SQUASH_GROUPS,
                                    RoseInstrSquashGroups> {
public:
    rose_group groups;

    explicit RoseInstrSquashGroups(rose_group groups_in) : groups(groups_in) {}

    bool operator==(const RoseInstrSquashGroups &ri) const {
        return groups == ri.groups;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), groups);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSquashGroups &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return groups == ri.groups;
    }
};

class RoseInstrCheckState
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_STATE,
                                    ROSE_STRUCT_CHECK_STATE,
                                    RoseInstrCheckState> {
public:
    u32 index;
    const RoseInstruction *target;

    RoseInstrCheckState(u32 index_in, const RoseInstruction *target_in)
        : index(index_in), target(target_in) {}

    bool operator==(const RoseInstrCheckState &ri) const {
        return index == ri.index && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), index);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckState &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return index == ri.index &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrSparseIterBegin
    : public RoseInstrBase<ROSE_INSTR_SPARSE_ITER_BEGIN,
                           ROSE_STRUCT_SPARSE_ITER_BEGIN,
                           RoseInstrSparseIterBegin> {
public:
    u32 num_keys; // total number of multibit keys
    std::vector<std::pair<u32, const RoseInstruction *>> jump_table;
    const RoseInstruction *target;

    RoseInstrSparseIterBegin(u32 num_keys_in,
                             const RoseInstruction *target_in)
        : num_keys(num_keys_in), target(target_in) {}

    bool operator==(const RoseInstrSparseIterBegin &ri) const {
        return num_keys == ri.num_keys && jump_table == ri.jump_table &&
               target == ri.target;
    }

    size_t hash() const override {
        size_t v = hash_all(static_cast<int>(opcode), num_keys);
        for (const u32 &key : jump_table | boost::adaptors::map_keys) {
            boost::hash_combine(v, key);
        }
        return v;
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    void update_target(const RoseInstruction *old_target,
                       const RoseInstruction *new_target) override {
        if (target == old_target) {
            target = new_target;
        }
        for (auto &jump : jump_table) {
            if (jump.second == old_target) {
                jump.second = new_target;
            }
        }
    }

    bool equiv_to(const RoseInstrSparseIterBegin &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        if (iter_offset != ri.iter_offset ||
            offsets.at(target) != other_offsets.at(ri.target)) {
            return false;
        }
        if (jump_table.size() != ri.jump_table.size()) {
            return false;
        }
        auto it1 = jump_table.begin(), it2 = ri.jump_table.begin();
        for (; it1 != jump_table.end(); ++it1, ++it2) {
            if (it1->first != it2->first) {
                return false;
            }
            if (offsets.at(it1->second) != other_offsets.at(it2->second)) {
                return false;
            }
        }
        return true;
    }

private:
    friend class RoseInstrSparseIterNext;

    // These variables allow us to use the same multibit iterator and jump
    // table in subsequent SPARSE_ITER_NEXT write() operations.
    mutable bool is_written = false;
    mutable u32 iter_offset = 0;
    mutable u32 jump_table_offset = 0;
};

class RoseInstrSparseIterNext
    : public RoseInstrBase<ROSE_INSTR_SPARSE_ITER_NEXT,
                           ROSE_STRUCT_SPARSE_ITER_NEXT,
                           RoseInstrSparseIterNext> {
public:
    u32 state;
    const RoseInstrSparseIterBegin *begin;
    const RoseInstruction *target;

    RoseInstrSparseIterNext(u32 state_in,
                            const RoseInstrSparseIterBegin *begin_in,
                            const RoseInstruction *target_in)
        : state(state_in), begin(begin_in), target(target_in) {}

    bool operator==(const RoseInstrSparseIterNext &ri) const {
        return state == ri.state && begin == ri.begin && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), state);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    void update_target(const RoseInstruction *old_target,
                       const RoseInstruction *new_target) override {
        if (target == old_target) {
            target = new_target;
        }
        if (begin == old_target) {
            assert(new_target->code() == ROSE_INSTR_SPARSE_ITER_BEGIN);
            begin = static_cast<const RoseInstrSparseIterBegin *>(new_target);
        }
    }

    bool equiv_to(const RoseInstrSparseIterNext &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return state == ri.state &&
               offsets.at(begin) == other_offsets.at(ri.begin) &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrSparseIterAny
    : public RoseInstrBaseOneTarget<ROSE_INSTR_SPARSE_ITER_ANY,
                                    ROSE_STRUCT_SPARSE_ITER_ANY,
                                    RoseInstrSparseIterAny> {
public:
    u32 num_keys; // total number of multibit keys
    std::vector<u32> keys;
    const RoseInstruction *target;

    RoseInstrSparseIterAny(u32 num_keys_in, std::vector<u32> keys_in,
                           const RoseInstruction *target_in)
        : num_keys(num_keys_in), keys(std::move(keys_in)), target(target_in) {}

    bool operator==(const RoseInstrSparseIterAny &ri) const {
        return num_keys == ri.num_keys && keys == ri.keys &&
               target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), num_keys, keys);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrSparseIterAny &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return num_keys == ri.num_keys && keys == ri.keys &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrEnginesEod
    : public RoseInstrBaseNoTargets<ROSE_INSTR_ENGINES_EOD,
                                    ROSE_STRUCT_ENGINES_EOD,
                                    RoseInstrEnginesEod> {
public:
    u32 iter_offset;

    explicit RoseInstrEnginesEod(u32 iter_in) : iter_offset(iter_in) {}

    bool operator==(const RoseInstrEnginesEod &ri) const {
        return iter_offset == ri.iter_offset;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), iter_offset);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrEnginesEod &ri, const OffsetMap &,
                  const OffsetMap &) const {
        return iter_offset == ri.iter_offset;
    }
};

class RoseInstrSuffixesEod
    : public RoseInstrBaseTrivial<ROSE_INSTR_SUFFIXES_EOD,
                                  ROSE_STRUCT_SUFFIXES_EOD,
                                  RoseInstrSuffixesEod> {
public:
    ~RoseInstrSuffixesEod() override;
};

class RoseInstrMatcherEod : public RoseInstrBaseTrivial<ROSE_INSTR_MATCHER_EOD,
                                                        ROSE_STRUCT_MATCHER_EOD,
                                                        RoseInstrMatcherEod> {
public:
    ~RoseInstrMatcherEod() override;
};

class RoseInstrCheckLongLit
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_LONG_LIT,
                                    ROSE_STRUCT_CHECK_LONG_LIT,
                                    RoseInstrCheckLongLit> {
public:
    std::string literal;
    const RoseInstruction *target;

    RoseInstrCheckLongLit(std::string literal_in,
                          const RoseInstruction *target_in)
        : literal(std::move(literal_in)), target(target_in) {}

    bool operator==(const RoseInstrCheckLongLit &ri) const {
        return literal == ri.literal && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), literal);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckLongLit &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return literal == ri.literal &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckLongLitNocase
    : public RoseInstrBaseOneTarget<ROSE_INSTR_CHECK_LONG_LIT_NOCASE,
                                    ROSE_STRUCT_CHECK_LONG_LIT_NOCASE,
                                    RoseInstrCheckLongLitNocase> {
public:
    std::string literal;
    const RoseInstruction *target;

    RoseInstrCheckLongLitNocase(std::string literal_in,
                                const RoseInstruction *target_in)
        : literal(std::move(literal_in)), target(target_in) {
        upperString(literal);
    }

    bool operator==(const RoseInstrCheckLongLitNocase &ri) const {
        return literal == ri.literal && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), literal);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckLongLitNocase &ri,
                  const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return literal == ri.literal &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckMedLit
    : public RoseInstrBaseNoTargets<ROSE_INSTR_CHECK_MED_LIT,
                                    ROSE_STRUCT_CHECK_MED_LIT,
                                    RoseInstrCheckMedLit> {
public:
    std::string literal;
    const RoseInstruction *target;

    explicit RoseInstrCheckMedLit(std::string literal_in,
                                  const RoseInstruction *target_in)
        : literal(std::move(literal_in)), target(target_in) {}

    bool operator==(const RoseInstrCheckMedLit &ri) const {
        return literal == ri.literal && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), literal);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckMedLit &ri, const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return literal == ri.literal &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrCheckMedLitNocase
    : public RoseInstrBaseNoTargets<ROSE_INSTR_CHECK_MED_LIT_NOCASE,
                                    ROSE_STRUCT_CHECK_MED_LIT_NOCASE,
                                    RoseInstrCheckMedLitNocase> {
public:
    std::string literal;
    const RoseInstruction *target;

    explicit RoseInstrCheckMedLitNocase(std::string literal_in,
                                        const RoseInstruction *target_in)
        : literal(std::move(literal_in)), target(target_in) {
        upperString(literal);
    }

    bool operator==(const RoseInstrCheckMedLitNocase &ri) const {
        return literal == ri.literal && target == ri.target;
    }

    size_t hash() const override {
        return hash_all(static_cast<int>(opcode), literal);
    }

    void write(void *dest, RoseEngineBlob &blob,
               const OffsetMap &offset_map) const override;

    bool equiv_to(const RoseInstrCheckMedLitNocase &ri,
                  const OffsetMap &offsets,
                  const OffsetMap &other_offsets) const {
        return literal == ri.literal &&
               offsets.at(target) == other_offsets.at(ri.target);
    }
};

class RoseInstrEnd
    : public RoseInstrBaseTrivial<ROSE_INSTR_END, ROSE_STRUCT_END,
                                  RoseInstrEnd> {
public:
    ~RoseInstrEnd() override;
};

/**
 * \brief Container for a list of program instructions.
 */
class RoseProgram {
private:
    std::vector<std::unique_ptr<RoseInstruction>> prog;

public:
    RoseProgram() {
        prog.push_back(make_unique<RoseInstrEnd>());
    }

    bool empty() const {
        assert(!prog.empty());
        assert(prog.back()->code() == ROSE_INSTR_END);
        // Empty if we only have one element, the END instruction.
        return std::next(prog.begin()) == prog.end();
    }

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
    const RoseInstruction *end_instruction() const {
        assert(!prog.empty());
        assert(prog.back()->code() == ROSE_INSTR_END);

        return prog.back().get();
    }

private:
    static void update_targets(iterator it, iterator it_end,
                               const RoseInstruction *old_target,
                               const RoseInstruction *new_target) {
        assert(old_target && new_target && old_target != new_target);
        for (; it != it_end; ++it) {
            std::unique_ptr<RoseInstruction> &ri = *it;
            assert(ri);
            ri->update_target(old_target, new_target);
        }
    }

public:
    iterator insert(iterator it, std::unique_ptr<RoseInstruction> ri) {
        assert(!prog.empty());
        assert(it != end());
        assert(prog.back()->code() == ROSE_INSTR_END);

        return prog.insert(it, std::move(ri));
    }

    iterator insert(iterator it, RoseProgram &&block) {
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
        update_targets(block.prog.begin(), block.prog.end(), end_ptr,
                       new_target);

        // Workaround: container insert() for ranges doesn't return an iterator
        // in the version of the STL distributed with gcc 4.8.
        auto dist = distance(prog.begin(), it);
        prog.insert(it, std::make_move_iterator(block.prog.begin()),
                    std::make_move_iterator(block.prog.end()));
        it = prog.begin();
        std::advance(it, dist);
        return it;
    }

    /**
     * \brief Adds this instruction to the program just before the terminating
     * ROSE_INSTR_END.
     */
    void add_before_end(std::unique_ptr<RoseInstruction> ri) {
        assert(!prog.empty());
        insert(std::prev(prog.end()), std::move(ri));
    }

    /**
     * \brief Adds this block to the program just before the terminating
     * ROSE_INSTR_END.
     */
    void add_before_end(RoseProgram &&block) {
        assert(!prog.empty());
        assert(prog.back()->code() == ROSE_INSTR_END);

        if (block.empty()) {
            return;
        }

        insert(std::prev(prog.end()), std::move(block));
    }

    /**
     * \brief Append this program block, replacing our current ROSE_INSTR_END.
     */
    void add_block(RoseProgram &&block) {
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
        prog.insert(prog.end(), std::make_move_iterator(block.prog.begin()),
                    std::make_move_iterator(block.prog.end()));
    }

    /**
     * \brief Replace the instruction pointed to by the given iterator.
     */
    template<class Iter>
    void replace(Iter it, std::unique_ptr<RoseInstruction> ri) {
        assert(!prog.empty());
        assert(prog.back()->code() == ROSE_INSTR_END);

        const RoseInstruction *old_ptr = it->get();
        *it = move(ri);
        update_targets(prog.begin(), prog.end(), old_ptr, it->get());

        assert(prog.back()->code() == ROSE_INSTR_END);
    }
};

aligned_unique_ptr<char>
writeProgram(RoseEngineBlob &blob, const RoseProgram &program, u32 *total_len);

class RoseProgramHash {
public:
    size_t operator()(const RoseProgram &program) const {
        size_t v = 0;
        for (const auto &ri : program) {
            assert(ri);
            boost::hash_combine(v, ri->hash());
        }
        return v;
    }
};

class RoseProgramEquivalence {
public:
    bool operator()(const RoseProgram &prog1, const RoseProgram &prog2) const;
};

} // namespace ue2

#endif // ROSE_BUILD_PROGRAM_H
