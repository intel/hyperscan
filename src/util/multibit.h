/*
 * Copyright (c) 2015-2018, Intel Corporation
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
 * \brief Multibit: fast bitset structure, main runtime.
 *
 * *Structure*
 *
 * For sizes <= MMB_FLAT_MAX_BITS, a flat bit vector is used, stored as N
 * 64-bit blocks followed by one "runt block".
 *
 * In larger cases, we use a sequence of blocks forming a tree. Each bit in an
 * internal block indicates whether its child block contains valid data. Every
 * level bar the last is complete. The last level is just a basic bit vector.
 *
 * -----------------------------------------------------------------------------
 * WARNING:
 *
 * mmbit code assumes that it is legal to load 8 bytes before the end of the
 * mmbit. This means that for small mmbits (< 8byte), data may be read from
 * before the base pointer. It is the user's responsibility to ensure that this
 * is possible.
 * -----------------------------------------------------------------------------
 */
#ifndef MULTIBIT_H
#define MULTIBIT_H

#include "config.h"
#include "ue2common.h"
#include "bitutils.h"
#include "partial_store.h"
#include "unaligned.h"
#include "multibit_internal.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MMB_ONE (1ULL)
#define MMB_ALL_ONES (0xffffffffffffffffULL)

/** \brief Number of bits in a block. */
#define MMB_KEY_BITS (sizeof(MMB_TYPE) * 8)

#define MMB_KEY_MASK (MMB_KEY_BITS - 1)

// Key structure defines
#define MMB_KEY_SHIFT 6

/** \brief Max size of a flat multibit. */
#define MMB_FLAT_MAX_BITS 256

// Utility functions and data
// see multibit.c for contents
extern const u8 mmbit_keyshift_lut[32];
extern const u8 mmbit_maxlevel_from_keyshift_lut[32];
extern const u8 mmbit_maxlevel_direct_lut[32];
extern const u32 mmbit_root_offset_from_level[7];
extern const u64a mmbit_zero_to_lut[65];

static really_inline
MMB_TYPE mmb_load(const u8 * bits) {
    return unaligned_load_u64a(bits);
}

static really_inline
void mmb_store(u8 *bits, MMB_TYPE val) {
    unaligned_store_u64a(bits, val);
}

static really_inline
void mmb_store_partial(u8 *bits, MMB_TYPE val, u32 block_bits) {
    assert(block_bits <= MMB_KEY_BITS);
    partial_store_u64a(bits, val, ROUNDUP_N(block_bits, 8U) / 8U);
}

static really_inline
MMB_TYPE mmb_single_bit(u32 bit) {
    assert(bit < MMB_KEY_BITS);
    return MMB_ONE << bit;
}

static really_inline
MMB_TYPE mmb_mask_zero_to(u32 bit) {
    assert(bit <= MMB_KEY_BITS);
#ifdef ARCH_32_BIT
    return mmbit_zero_to_lut[bit];
#else
    if (bit == MMB_KEY_BITS) {
        return MMB_ALL_ONES;
    } else {
        return mmb_single_bit(bit) - MMB_ONE;
    }
#endif
}

/** \brief Returns a mask of set bits up to position \a bit. Does not handle
 * the case where bit == MMB_KEY_BITS. */
static really_inline
MMB_TYPE mmb_mask_zero_to_nocheck(u32 bit) {
    assert(bit < MMB_KEY_BITS);
#ifdef ARCH_32_BIT
    return mmbit_zero_to_lut[bit];
#else
    return mmb_single_bit(bit) - MMB_ONE;
#endif
}

static really_inline
u32 mmb_test(MMB_TYPE val, u32 bit) {
    assert(bit < MMB_KEY_BITS);
    return (val >> bit) & MMB_ONE;
}

static really_inline
void mmb_set(MMB_TYPE * val, u32 bit) {
    assert(bit < MMB_KEY_BITS);
    *val |= mmb_single_bit(bit);
}

static really_inline
void mmb_clear(MMB_TYPE * val, u32 bit) {
    assert(bit < MMB_KEY_BITS);
    *val &= ~mmb_single_bit(bit);
}

static really_inline
u32 mmb_ctz(MMB_TYPE val) {
    return ctz64(val);
}

static really_inline
u32 mmb_popcount(MMB_TYPE val) {
    return popcount64(val);
}

#ifndef MMMB_DEBUG
#define MDEBUG_PRINTF(x, ...) do { } while(0)
#else
#define MDEBUG_PRINTF DEBUG_PRINTF
#endif

// Switch the following define on to trace writes to multibit.
//#define MMB_TRACE_WRITES
#ifdef MMB_TRACE_WRITES
#define MMB_TRACE(format, ...)                                                 \
    printf("mmb [%u bits @ %p] " format, total_bits, bits, ##__VA_ARGS__)
#else
#define MMB_TRACE(format, ...)                                                 \
    do {                                                                       \
    } while (0)
#endif

static really_inline
u32 mmbit_keyshift(u32 total_bits) {
    assert(total_bits > 1);
    u32 n = clz32(total_bits - 1); // subtract one as we're rounding down
    return mmbit_keyshift_lut[n];
}

static really_inline
u32 mmbit_maxlevel(u32 total_bits) {
    assert(total_bits > 1);
    u32 n = clz32(total_bits - 1); // subtract one as we're rounding down
    u32 max_level = mmbit_maxlevel_direct_lut[n];
    assert(max_level <= MMB_MAX_LEVEL);
    return max_level;
}

static really_inline
u32 mmbit_maxlevel_from_keyshift(u32 ks) {
    assert(ks <= 30);
    assert(ks % MMB_KEY_SHIFT == 0);

    u32 max_level = mmbit_maxlevel_from_keyshift_lut[ks];
    assert(max_level <= MMB_MAX_LEVEL);
    return max_level;
}

/** \brief get our keyshift for the current level */
static really_inline
u32 mmbit_get_ks(u32 max_level, u32 level) {
    assert(max_level <= MMB_MAX_LEVEL);
    assert(level <= max_level);
    return (max_level - level) * MMB_KEY_SHIFT;
}

/** \brief get our key value for the current level */
static really_inline
u32 mmbit_get_key_val(u32 max_level, u32 level, u32 key) {
    return (key >> mmbit_get_ks(max_level, level)) & MMB_KEY_MASK;
}

/** \brief get the level root for the current level */
static really_inline
u8 *mmbit_get_level_root(u8 *bits, u32 level) {
    assert(level < ARRAY_LENGTH(mmbit_root_offset_from_level));
    return bits + mmbit_root_offset_from_level[level] * sizeof(MMB_TYPE);
}

/** \brief get the level root for the current level as const */
static really_inline
const u8 *mmbit_get_level_root_const(const u8 *bits, u32 level) {
    assert(level < ARRAY_LENGTH(mmbit_root_offset_from_level));
    return bits + mmbit_root_offset_from_level[level] * sizeof(MMB_TYPE);
}

/** \brief get the block for this key on the current level as a u8 ptr */
static really_inline
u8 *mmbit_get_block_ptr(u8 *bits, u32 max_level, u32 level, u32 key) {
    u8 *level_root = mmbit_get_level_root(bits, level);
    u32 ks = mmbit_get_ks(max_level, level);
    return level_root + ((u64a)key >> (ks + MMB_KEY_SHIFT)) * sizeof(MMB_TYPE);
}

/** \brief get the block for this key on the current level as a const u8 ptr */
static really_inline
const u8 *mmbit_get_block_ptr_const(const u8 *bits, u32 max_level, u32 level,
                                    u32 key) {
    const u8 *level_root = mmbit_get_level_root_const(bits, level);
    u32 ks = mmbit_get_ks(max_level, level);
    return level_root + ((u64a)key >> (ks + MMB_KEY_SHIFT)) * sizeof(MMB_TYPE);
}

/** \brief get the _byte_ for this key on the current level as a u8 ptr */
static really_inline
u8 *mmbit_get_byte_ptr(u8 *bits, u32 max_level, u32 level, u32 key) {
    u8 *level_root = mmbit_get_level_root(bits, level);
    u32 ks = mmbit_get_ks(max_level, level);
    return level_root + ((u64a)key >> (ks + MMB_KEY_SHIFT - 3));
}

/** \brief get our key value for the current level */
static really_inline
u32 mmbit_get_key_val_byte(u32 max_level, u32 level, u32 key) {
    return (key >> (mmbit_get_ks(max_level, level))) & 0x7;
}

/** \brief Load a flat bitvector block corresponding to N bits. */
static really_inline
MMB_TYPE mmbit_get_flat_block(const u8 *bits, u32 n_bits) {
    assert(n_bits <= MMB_KEY_BITS);
    u32 n_bytes = ROUNDUP_N(n_bits, 8) / 8;
    switch (n_bytes) {
    case 1:
        return *bits;
    case 2:
        return unaligned_load_u16(bits);
    case 3:
    case 4: {
        u32 rv;
        assert(n_bytes <= sizeof(rv));
        memcpy(&rv, bits + n_bytes - sizeof(rv), sizeof(rv));
        rv >>= (sizeof(rv) - n_bytes) * 8; /* need to shift to get things in
                                            * the right position and remove
                                            * junk */
        assert(rv == partial_load_u32(bits, n_bytes));
        return rv;
    }
    default: {
        u64a rv;
        assert(n_bytes <= sizeof(rv));
        memcpy(&rv, bits + n_bytes - sizeof(rv), sizeof(rv));
        rv >>= (sizeof(rv) - n_bytes) * 8; /* need to shift to get things in
                                            * the right position and remove
                                            * junk */
        assert(rv == partial_load_u64a(bits, n_bytes));
        return rv;
    }
    }
}

/** \brief True if this multibit is small enough to use a flat model */
static really_inline
u32 mmbit_is_flat_model(u32 total_bits) {
    return total_bits <= MMB_FLAT_MAX_BITS;
}

static really_inline
u32 mmbit_flat_size(u32 total_bits) {
    assert(mmbit_is_flat_model(total_bits));
    return ROUNDUP_N(total_bits, 8) / 8;
}

static really_inline
u32 mmbit_flat_select_byte(u32 key, UNUSED u32 total_bits) {
    return key / 8;
}

/** \brief returns the dense index of the bit in the given mask. */
static really_inline
u32 mmbit_mask_index(u32 bit, MMB_TYPE mask) {
    assert(bit < MMB_KEY_BITS);
    assert(mmb_test(mask, bit));

    mask &= mmb_mask_zero_to(bit);
    if (mask == 0ULL) {
        return 0; // Common case.
    }
    return mmb_popcount(mask);
}

/** \brief Clear all bits. */
static really_inline
void mmbit_clear(u8 *bits, u32 total_bits) {
    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);
    MMB_TRACE("CLEAR\n");
    if (!total_bits) {
        return;
    }
    if (mmbit_is_flat_model(total_bits)) {
        memset(bits, 0, mmbit_flat_size(total_bits));
        return;
    }
    mmb_store(bits, 0);
}

/** \brief Specialisation of \ref mmbit_set for flat models. */
static really_inline
char mmbit_set_flat(u8 *bits, u32 total_bits, u32 key) {
    bits += mmbit_flat_select_byte(key, total_bits);
    u8 mask = 1U << (key % 8);
    char was_set = !!(*bits & mask);
    *bits |= mask;
    return was_set;
}

static really_inline
char mmbit_set_big(u8 *bits, u32 total_bits, u32 key) {
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    do {
        u8 * byte_ptr = mmbit_get_byte_ptr(bits, max_level, level, key);
        u8 keymask = 1U << mmbit_get_key_val_byte(max_level, level, key);
        u8 byte = *byte_ptr;
        if (likely(!(byte & keymask))) {
            *byte_ptr = byte | keymask;
            while (level++ != max_level) {
                u8 *block_ptr_1 = mmbit_get_block_ptr(bits, max_level, level, key);
                MMB_TYPE keymask_1 = mmb_single_bit(mmbit_get_key_val(max_level, level, key));
                mmb_store(block_ptr_1, keymask_1);
            }
            return 0;
        }
    } while (level++ != max_level);
    return 1;
}

/** Internal version of \ref mmbit_set without MMB_TRACE, so it can be used by
 * \ref mmbit_sparse_iter_dump. */
static really_inline
char mmbit_set_i(u8 *bits, u32 total_bits, u32 key) {
    assert(key < total_bits);
    if (mmbit_is_flat_model(total_bits)) {
        return mmbit_set_flat(bits, total_bits, key);
    } else {
        return mmbit_set_big(bits, total_bits, key);
    }
}

static really_inline
char mmbit_isset(const u8 *bits, u32 total_bits, u32 key);

/** \brief Sets the given key in the multibit. Returns 0 if the key was NOT
 * already set, 1 otherwise. */
static really_inline
char mmbit_set(u8 *bits, u32 total_bits, u32 key) {
    MDEBUG_PRINTF("%p total_bits %u key %u\n", bits, total_bits, key);
    char status = mmbit_set_i(bits, total_bits, key);
    MMB_TRACE("SET %u (prev status: %d)\n", key, (int)status);
    assert(mmbit_isset(bits, total_bits, key));
    return status;
}

/** \brief Specialisation of \ref mmbit_isset for flat models. */
static really_inline
char mmbit_isset_flat(const u8 *bits, u32 total_bits, u32 key) {
    bits += mmbit_flat_select_byte(key, total_bits);
    return !!(*bits & (1U << (key % 8U)));
}

static really_inline
char mmbit_isset_big(const u8 *bits, u32 total_bits, u32 key) {
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    do {
        const u8 *block_ptr = mmbit_get_block_ptr_const(bits, max_level, level, key);
        MMB_TYPE block = mmb_load(block_ptr);
        if (!mmb_test(block, mmbit_get_key_val(max_level, level, key))) {
            return 0;
        }
    } while (level++ != max_level);
    return 1;
}

/** \brief Returns whether the given key is set. */
static really_inline
char mmbit_isset(const u8 *bits, u32 total_bits, u32 key) {
    MDEBUG_PRINTF("%p total_bits %u key %u\n", bits, total_bits, key);
    assert(key < total_bits);
    if (mmbit_is_flat_model(total_bits)) {
        return mmbit_isset_flat(bits, total_bits, key);
    } else {
        return mmbit_isset_big(bits, total_bits, key);
    }
}

/** \brief Specialisation of \ref mmbit_unset for flat models. */
static really_inline
void mmbit_unset_flat(u8 *bits, u32 total_bits, u32 key) {
    bits += mmbit_flat_select_byte(key, total_bits);
    *bits &= ~(1U << (key % 8U));
}

// TODO:
// build two versions of this - unset_dangerous that doesn't clear the summary
// block and a regular unset that actually clears ALL the way up the levels if
// possible - might make a utility function for the clear
static really_inline
void mmbit_unset_big(u8 *bits, u32 total_bits, u32 key) {
    /* This function is lazy as it does not clear the summary block
     * entry if the child becomes empty. This is not a correctness problem as the
     * summary block entries are used to mean that their children are valid
     * rather than that they have a set child. */
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    do {
        u8 *block_ptr = mmbit_get_block_ptr(bits, max_level, level, key);
        u32 key_val = mmbit_get_key_val(max_level, level, key);
        MMB_TYPE block = mmb_load(block_ptr);
        if (!mmb_test(block, key_val)) {
            return;
        }
        if (level == max_level) {
            mmb_clear(&block, key_val);
            mmb_store(block_ptr, block);
        }
    } while (level++ != max_level);
}

/** \brief Switch off a given key. */
static really_inline
void mmbit_unset(u8 *bits, u32 total_bits, u32 key) {
    MDEBUG_PRINTF("%p total_bits %u key %u\n", bits, total_bits, key);
    assert(key < total_bits);
    MMB_TRACE("UNSET %u (prev status: %d)\n", key,
              (int)mmbit_isset(bits, total_bits, key));

    if (mmbit_is_flat_model(total_bits)) {
        mmbit_unset_flat(bits, total_bits, key);
    } else {
        mmbit_unset_big(bits, total_bits, key);
    }
}

/** \brief Specialisation of \ref mmbit_iterate for flat models. */
static really_inline
u32 mmbit_iterate_flat(const u8 *bits, u32 total_bits, u32 it_in) {
    // Short cut for single-block cases.
    if (total_bits <= MMB_KEY_BITS) {
        MMB_TYPE block = mmbit_get_flat_block(bits, total_bits);
        if (it_in != MMB_INVALID) {
            it_in++;
            assert(it_in < total_bits);
            block &= ~mmb_mask_zero_to(it_in);
        }
        if (block) {
            return mmb_ctz(block);
        }
        return MMB_INVALID;
    }

    const u32 last_block = total_bits / MMB_KEY_BITS;
    u32 start; // starting block index

    if (it_in != MMB_INVALID) {
        it_in++;
        assert(it_in < total_bits);

        start = (ROUNDUP_N(it_in, MMB_KEY_BITS) / MMB_KEY_BITS) - 1;
        u32 start_key = start * MMB_KEY_BITS;
        u32 block_size = MIN(MMB_KEY_BITS, total_bits - start_key);
        MMB_TYPE block =
            mmbit_get_flat_block(bits + (start * sizeof(MMB_TYPE)), block_size);
        block &= ~mmb_mask_zero_to(it_in - start_key);

        if (block) {
            return start_key + mmb_ctz(block);
        } else if (start_key + MMB_KEY_BITS >= total_bits) {
            return MMB_INVALID; // That was the final block.
        }
        start++;
    } else {
        start = 0;
    }

    // Remaining full-sized blocks.
    for (; start < last_block; start++) {
        MMB_TYPE block = mmb_load(bits + (start * sizeof(MMB_TYPE)));
        if (block) {
            return (start * MMB_KEY_BITS) + mmb_ctz(block);
        }
    }

    // We may have a final, smaller than full-sized, block to deal with at the
    // end.
    if (total_bits % MMB_KEY_BITS) {
        u32 start_key = start * MMB_KEY_BITS;
        u32 block_size = MIN(MMB_KEY_BITS, total_bits - start_key);
        MMB_TYPE block =
            mmbit_get_flat_block(bits + (start * sizeof(MMB_TYPE)), block_size);
        if (block) {
            return start_key + mmb_ctz(block);
        }
    }

    return MMB_INVALID;
}

static really_inline
u32 mmbit_iterate_big(const u8 * bits, u32 total_bits, u32 it_in) {
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    u32 key = 0;
    u32 key_rem = 0;

    if (it_in != MMB_INVALID) {
        // We're continuing a previous iteration, so we need to go
        // to max_level so we can pick up where we left off.
        // NOTE: assumes that we're valid down the whole tree
        key = it_in >> MMB_KEY_SHIFT;
        key_rem = (it_in & MMB_KEY_MASK) + 1;
        level = max_level;
    }
    while (1) {
        if (key_rem < MMB_KEY_BITS) {
            const u8 *block_ptr = mmbit_get_level_root_const(bits, level) +
                                  key * sizeof(MMB_TYPE);
            MMB_TYPE block
                = mmb_load(block_ptr) & ~mmb_mask_zero_to_nocheck(key_rem);
            if (block) {
                key = (key << MMB_KEY_SHIFT) + mmb_ctz(block);
                if (level++ == max_level) {
                    break;
                }
                key_rem = 0;
                continue; // jump the rootwards step if we found a 'tree' non-zero bit
            }
        }
        // rootwards step (block is zero or key_rem == MMB_KEY_BITS)
        if (level-- == 0) {
            return MMB_INVALID; // if we don't find anything and we're at the top level, we're done
        }
        key_rem = (key & MMB_KEY_MASK) + 1;
        key >>= MMB_KEY_SHIFT;
    }
    assert(key < total_bits);
    assert(mmbit_isset(bits, total_bits, key));
    return key;
}

/** \brief Unbounded iterator. Returns the index of the next set bit after \a
 * it_in, or MMB_INVALID.
 *
 * Note: assumes that if you pass in a value of it_in other than MMB_INVALID,
 * that bit must be on (assumes all its summary blocks are set).
 */
static really_inline
u32 mmbit_iterate(const u8 *bits, u32 total_bits, u32 it_in) {
    MDEBUG_PRINTF("%p total_bits %u it_in %u\n", bits, total_bits, it_in);
    assert(it_in < total_bits || it_in == MMB_INVALID);
    if (!total_bits) {
        return MMB_INVALID;
    }
    if (it_in == total_bits - 1) {
        return MMB_INVALID; // it_in is the last key.
    }

    u32 key;
    if (mmbit_is_flat_model(total_bits)) {
        key = mmbit_iterate_flat(bits, total_bits, it_in);
    } else {
        key = mmbit_iterate_big(bits, total_bits, it_in);
    }
    assert(key == MMB_INVALID || mmbit_isset(bits, total_bits, key));
    return key;
}

/** \brief Specialisation of \ref mmbit_any and \ref mmbit_any_precise for flat
 * models. */
static really_inline
char mmbit_any_flat(const u8 *bits, u32 total_bits) {
    if (total_bits <= MMB_KEY_BITS) {
        return !!mmbit_get_flat_block(bits, total_bits);
    }

    const u8 *end = bits + mmbit_flat_size(total_bits);
    for (const u8 *last = end - sizeof(MMB_TYPE); bits < last;
         bits += sizeof(MMB_TYPE)) {
        if (mmb_load(bits)) {
            return 1;
        }
    }

    // Overlapping load at the end.
    return !!mmb_load(end - sizeof(MMB_TYPE));
}

/** \brief True if any keys are (or might be) on in the given multibit.
 *
 * NOTE: mmbit_any is sloppy (may return true when only summary bits are set).
 * Use \ref mmbit_any_precise if you need/want a correct answer.
 */
static really_inline
char mmbit_any(const u8 *bits, u32 total_bits) {
    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);
    if (!total_bits) {
        return 0;
    }
    if (mmbit_is_flat_model(total_bits)) {
        return mmbit_any_flat(bits, total_bits);
    }
    return !!mmb_load(bits);
}

/** \brief True if there are any keys on. Guaranteed precise. */
static really_inline
char mmbit_any_precise(const u8 *bits, u32 total_bits) {
    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);
    if (!total_bits) {
        return 0;
    }
    if (mmbit_is_flat_model(total_bits)) {
        return mmbit_any_flat(bits, total_bits);
    }

    return mmbit_iterate_big(bits, total_bits, MMB_INVALID) != MMB_INVALID;
}

static really_inline
char mmbit_all_flat(const u8 *bits, u32 total_bits) {
    while (total_bits > MMB_KEY_BITS) {
        if (mmb_load(bits) != MMB_ALL_ONES) {
            return 0;
        }
        bits += sizeof(MMB_TYPE);
        total_bits -= MMB_KEY_BITS;
    }
    while (total_bits > 8) {
        if (*bits != 0xff) {
            return 0;
        }
        bits++;
        total_bits -= 8;
    }
    u8 mask = (u8)mmb_mask_zero_to_nocheck(total_bits);
    return (*bits & mask) == mask;
}

static really_inline
char mmbit_all_big(const u8 *bits, u32 total_bits) {
    u32 ks = mmbit_keyshift(total_bits);

    u32 level = 0;
    for (;;) {
        // Number of bits we expect to see switched on on this level.
        u32 level_bits;
        if (ks != 0) {
            u32 next_level_width = MMB_KEY_BITS << (ks - MMB_KEY_SHIFT);
            level_bits = ROUNDUP_N(total_bits, next_level_width) >> ks;
        } else {
            level_bits = total_bits;
        }

        const u8 *block_ptr = mmbit_get_level_root_const(bits, level);

        // All full-size blocks should be all-ones.
        while (level_bits >= MMB_KEY_BITS) {
            MMB_TYPE block = mmb_load(block_ptr);
            if (block != MMB_ALL_ONES) {
                return 0;
            }
            block_ptr += sizeof(MMB_TYPE);
            level_bits -= MMB_KEY_BITS;
        }

        // If we have bits remaining, we have a runt block on the end.
        if (level_bits > 0) {
            MMB_TYPE block = mmb_load(block_ptr);
            MMB_TYPE mask = mmb_mask_zero_to_nocheck(level_bits);
            if ((block & mask) != mask) {
                return 0;
            }
        }

        if (ks == 0) {
            break;
        }

        ks -= MMB_KEY_SHIFT;
        level++;
    }

    return 1;
}

/** \brief True if all keys are on. Guaranteed precise. */
static really_inline
char mmbit_all(const u8 *bits, u32 total_bits) {
    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);

    if (mmbit_is_flat_model(total_bits)) {
        return mmbit_all_flat(bits, total_bits);
    }
    return mmbit_all_big(bits, total_bits);
}

static really_inline
MMB_TYPE get_flat_masks(u32 base, u32 it_start, u32 it_end) {
    if (it_end <= base) {
        return 0;
    }
    u32 udiff = it_end - base;
    MMB_TYPE mask = udiff < 64 ? mmb_mask_zero_to_nocheck(udiff) : MMB_ALL_ONES;
    if (it_start >= base) {
        u32 ldiff = it_start - base;
        MMB_TYPE lmask = ldiff < 64 ? ~mmb_mask_zero_to_nocheck(ldiff) : 0;
        mask &= lmask;
    }
    return mask;
}

/** \brief Specialisation of \ref mmbit_iterate_bounded for flat models. */
static really_inline
u32 mmbit_iterate_bounded_flat(const u8 *bits, u32 total_bits, u32 begin,
                               u32 end) {
    // Short cut for single-block cases.
    if (total_bits <= MMB_KEY_BITS) {
        MMB_TYPE block = mmbit_get_flat_block(bits, total_bits);
        block &= get_flat_masks(0, begin, end);
        if (block) {
            return mmb_ctz(block);
        }
        return MMB_INVALID;
    }

    const u32 last_block = ROUNDDOWN_N(total_bits, MMB_KEY_BITS);

    // Iterate over full-sized blocks.
    for (u32 i = ROUNDDOWN_N(begin, MMB_KEY_BITS), e = MIN(end, last_block);
         i < e; i += MMB_KEY_BITS) {
        const u8 *block_ptr = bits + i / 8;
        MMB_TYPE block = mmb_load(block_ptr);
        block &= get_flat_masks(i, begin, end);
        if (block) {
            return i + mmb_ctz(block);
        }
    }

    // Final block, which is less than full-sized.
    if (end > last_block) {
        const u8 *block_ptr = bits + last_block / 8;
        u32 num_bits = total_bits - last_block;
        MMB_TYPE block = mmbit_get_flat_block(block_ptr, num_bits);
        block &= get_flat_masks(last_block, begin, end);
        if (block) {
            return last_block + mmb_ctz(block);
        }
    }

    return MMB_INVALID;
}

static really_inline
MMB_TYPE get_lowhi_masks(u32 level, u32 max_level, u64a block_min, u64a block_max,
                         u64a block_base) {
    const u32 level_shift = (max_level - level) * MMB_KEY_SHIFT;
    u64a lshift = (block_min - block_base) >> level_shift;
    u64a ushift = (block_max - block_base) >> level_shift;
    MMB_TYPE lmask = lshift < 64 ? ~mmb_mask_zero_to_nocheck(lshift) : 0;
    MMB_TYPE umask =
        ushift < 63 ? mmb_mask_zero_to_nocheck(ushift + 1) : MMB_ALL_ONES;
    return lmask & umask;
}

static really_inline
u32 mmbit_iterate_bounded_big(const u8 *bits, u32 total_bits, u32 it_start, u32 it_end) {
    u64a key = 0;
    u32 ks = mmbit_keyshift(total_bits);
    const u32 max_level = mmbit_maxlevel_from_keyshift(ks);
    u32 level = 0;
    --it_end; // make end-limit inclusive
    for (;;) {
        assert(level <= max_level);

        u64a block_width = MMB_KEY_BITS << ks;
        u64a block_base = key * block_width;
        u64a block_min = MAX(it_start, block_base);
        u64a block_max = MIN(it_end, block_base + block_width - 1);
        const u8 *block_ptr =
            mmbit_get_level_root_const(bits, level) + key * sizeof(MMB_TYPE);
        MMB_TYPE block = mmb_load(block_ptr);
        block &= get_lowhi_masks(level, max_level, block_min, block_max, block_base);
        if (block) {
            // Found a bit, go down a level
            key = (key << MMB_KEY_SHIFT) + mmb_ctz(block);
            if (level++ == max_level) {
                return key;
            }
            ks -= MMB_KEY_SHIFT;
        } else {
            // No bit found, go up a level
            // we know that this block didn't have any answers, so we can push
            // our start iterator forward.
            u64a next_start = block_base + block_width;
            if (next_start > it_end) {
                break;
            }
            if (level-- == 0) {
                break;
            }
            it_start = next_start;
            key >>= MMB_KEY_SHIFT;
            ks += MMB_KEY_SHIFT;
        }
    }
    return MMB_INVALID;
}

/** \brief Bounded iterator. Returns the index of the first set bit between
 * it_start (inclusive) and it_end (exclusive) or MMB_INVALID if no bits are
 * set in that range.
 */
static really_inline
u32 mmbit_iterate_bounded(const u8 *bits, u32 total_bits, u32 it_start,
                          u32 it_end) {
    MDEBUG_PRINTF("%p total_bits %u it_start %u it_end %u\n", bits, total_bits,
                  it_start, it_end);
    assert(it_start <= it_end);
    assert(it_end <= total_bits);
    if (!total_bits || it_end == it_start) {
        return MMB_INVALID;
    }
    assert(it_start < total_bits);
    u32 key;
    if (mmbit_is_flat_model(total_bits)) {
        key = mmbit_iterate_bounded_flat(bits, total_bits, it_start, it_end);
    } else {
        key = mmbit_iterate_bounded_big(bits, total_bits, it_start, it_end);
    }
    assert(key == MMB_INVALID || mmbit_isset(bits, total_bits, key));
    return key;
}

/** \brief Specialisation of \ref mmbit_unset_range for flat models. */
static really_inline
void mmbit_unset_range_flat(u8 *bits, u32 total_bits, u32 begin, u32 end) {
    const u32 last_block = ROUNDDOWN_N(total_bits, MMB_KEY_BITS);

    // Iterate over full-sized blocks.
    for (u32 i = ROUNDDOWN_N(begin, MMB_KEY_BITS), e = MIN(end, last_block);
         i < e; i += MMB_KEY_BITS) {
        u8 *block_ptr = bits + i / 8;
        MMB_TYPE block = mmb_load(block_ptr);
        MMB_TYPE mask = get_flat_masks(i, begin, end);
        mmb_store(block_ptr, block & ~mask);
    }

    // Final block, which is less than full-sized.
    if (end > last_block) {
        u8 *block_ptr = bits + last_block / 8;
        u32 num_bits = total_bits - last_block;
        MMB_TYPE block = mmbit_get_flat_block(block_ptr, num_bits);
        MMB_TYPE mask = get_flat_masks(last_block, begin, end);
        mmb_store_partial(block_ptr, block & ~mask, num_bits);
    }
}

static really_inline
void mmbit_unset_range_big(u8 *bits, const u32 total_bits, u32 begin,
                           u32 end) {
    // TODO: combine iterator and unset operation; completely replace this
    u32 i = begin;
    for (;;) {
        i = mmbit_iterate_bounded(bits, total_bits, i, end);
        if (i == MMB_INVALID) {
            break;
        }
        mmbit_unset_big(bits, total_bits, i);
        if (++i == end) {
            break;
        }
    }
}

/** \brief Unset a whole range of bits. Ensures that all bits between \a begin
 * (inclusive) and \a end (exclusive) are switched off.  */
static really_inline
void mmbit_unset_range(u8 *bits, const u32 total_bits, u32 begin, u32 end) {
    MDEBUG_PRINTF("%p total_bits %u begin %u end %u\n", bits, total_bits, begin,
                  end);
    assert(begin <= end);
    assert(end <= total_bits);
    if (mmbit_is_flat_model(total_bits)) {
        mmbit_unset_range_flat(bits, total_bits, begin, end);
    } else {
        mmbit_unset_range_big(bits, total_bits, begin, end);
    }
    // No bits are on in [begin, end) once we're done.
    assert(MMB_INVALID == mmbit_iterate_bounded(bits, total_bits, begin, end));
}

/** \brief Specialisation of \ref mmbit_init_range for flat models. */
static really_inline
void mmbit_init_range_flat(u8 *bits, const u32 total_bits, u32 begin, u32 end) {
    const u32 last_block = ROUNDDOWN_N(total_bits, MMB_KEY_BITS);

    // Iterate over full-sized blocks.
    for (u32 i = 0; i < last_block; i += MMB_KEY_BITS) {
        mmb_store(bits + i / 8, get_flat_masks(i, begin, end));
    }

    // Final block, which is less than full-sized.
    if (total_bits % MMB_KEY_BITS) {
        u32 num_bits = total_bits - last_block;
        MMB_TYPE block = get_flat_masks(last_block, begin, end);
        mmb_store_partial(bits + last_block / 8, block, num_bits);
    }
}

static really_inline
void mmbit_init_range_big(u8 *bits, const u32 total_bits, u32 begin, u32 end) {
    u32 ks = mmbit_keyshift(total_bits);
    u32 level = 0;

    for (;;) {
        u8 *block = mmbit_get_level_root(bits, level);
        u32 k1 = begin >> ks, k2 = end >> ks;

        // Summary blocks need to account for the runt block on the end.
        if ((k2 << ks) != end) {
            k2++;
        }

        // Partial block to deal with beginning.
        block += (k1 / MMB_KEY_BITS) * sizeof(MMB_TYPE);
        if (k1 % MMB_KEY_BITS) {
            u32 idx = k1 / MMB_KEY_BITS;
            u32 block_end = (idx + 1) * MMB_KEY_BITS;

            // Because k1 % MMB_KEY_BITS != 0, we can avoid checking edge cases
            // here (see the branch in mmb_mask_zero_to).
            MMB_TYPE mask = MMB_ALL_ONES << (k1 % MMB_KEY_BITS);

            if (k2 < block_end) {
                assert(k2 % MMB_KEY_BITS);
                mask &= mmb_mask_zero_to_nocheck(k2 % MMB_KEY_BITS);
                mmb_store(block, mask);
                goto next_level;
            } else {
                mmb_store(block, mask);
                k1 = block_end;
                block += sizeof(MMB_TYPE);
            }
        }

        // Write blocks filled with ones until we get to the last block.
        for (; k1 < (k2 & ~MMB_KEY_MASK); k1 += MMB_KEY_BITS) {
            mmb_store(block, MMB_ALL_ONES);
            block += sizeof(MMB_TYPE);
        }

        // Final block.
        if (likely(k1 < k2)) {
            // Again, if k2 was at a block boundary, it would have been handled
            // by the previous loop, so we know k2 % MMB_KEY_BITS != 0 and can
            // avoid the branch in mmb_mask_zero_to here.
            assert(k2 % MMB_KEY_BITS);
            MMB_TYPE mask = mmb_mask_zero_to_nocheck(k2 % MMB_KEY_BITS);
            mmb_store(block, mask);
        }

    next_level:
        if (ks == 0) {
            break; // Last level is done, finished.
        }

        ks -= MMB_KEY_SHIFT;
        level++;
    }
}

/** \brief Initialises the multibit so that only the given range of bits are
 * set.
 *
 * Ensures that all bits between \a begin (inclusive) and \a end (exclusive)
 * are switched on.
 */
static really_inline
void mmbit_init_range(u8 *bits, const u32 total_bits, u32 begin, u32 end) {
    MDEBUG_PRINTF("%p total_bits %u begin %u end %u\n", bits, total_bits, begin,
                  end);
    assert(begin <= end);
    assert(end <= total_bits);

    if (!total_bits) {
        return;
    }

    // Short cut for cases where we're not actually setting any bits; just
    // clear the multibit.
    if (begin == end) {
        mmbit_clear(bits, total_bits);
        return;
    }

    if (mmbit_is_flat_model(total_bits)) {
        mmbit_init_range_flat(bits, total_bits, begin, end);
    } else {
        mmbit_init_range_big(bits, total_bits, begin, end);
    }

    assert(begin == end ||
           mmbit_iterate(bits, total_bits, MMB_INVALID) == begin);
    assert(!end || begin == end ||
           mmbit_iterate(bits, total_bits, end - 1) == MMB_INVALID);
}

/** \brief Determine the number of \ref mmbit_sparse_state elements required.
 * */
static really_inline
u32 mmbit_sparse_iter_state_size(u32 total_bits) {
    if (mmbit_is_flat_model(total_bits)) {
        return 2;
    }
    u32 levels = mmbit_maxlevel(total_bits);
    return levels + 1;
}

#ifdef DUMP_SUPPORT
// Dump function, defined in multibit.c.
void mmbit_sparse_iter_dump(const struct mmbit_sparse_iter *it, u32 total_bits);
#endif

/** Internal: common loop used by mmbit_sparse_iter_{begin,next}_big. Returns
 * matching next key given starting state, or MMB_INVALID. */
static really_inline
u32 mmbit_sparse_iter_exec(const u8 *bits, u32 key, u32 *idx, u32 level,
                           const u32 max_level, struct mmbit_sparse_state *s,
                           const struct mmbit_sparse_iter *it_root,
                           const struct mmbit_sparse_iter *it) {
    for (;;) {
        MMB_TYPE block = s[level].mask;
        if (block) {
            u32 bit = mmb_ctz(block);
            key = (key << MMB_KEY_SHIFT) + bit;
            u32 bit_idx = mmbit_mask_index(bit, it->mask);
            if (level++ == max_level) {
                // we've found a key
                *idx = it->val + bit_idx;
                return key;
            } else {
                // iterator record is the start of the level (current it->val)
                // plus N, where N is the dense index of the bit in the current
                // level's itmask
                u32 iter_key = it->val + bit_idx;
                it = it_root + iter_key;
                MMB_TYPE nextblock =
                    mmb_load(mmbit_get_level_root_const(bits, level) +
                             key * sizeof(MMB_TYPE));
                s[level].mask = nextblock & it->mask;
                s[level].itkey = iter_key;
            }
        } else {
            // No bits set in this block
            if (level-- == 0) {
                break; // no key available
            }
            key >>= MMB_KEY_SHIFT;
            // Update state mask and iterator
            s[level].mask &= (s[level].mask - 1);
            it = it_root + s[level].itkey;
        }
    }
    return MMB_INVALID;
}

static really_inline
u32 mmbit_sparse_iter_begin_big(const u8 *bits, u32 total_bits, u32 *idx,
                                const struct mmbit_sparse_iter *it_root,
                                struct mmbit_sparse_state *s) {
    const struct mmbit_sparse_iter *it = it_root;
    u32 key = 0;
    MMB_TYPE block = mmb_load(bits) & it->mask;
    if (!block) {
        return MMB_INVALID;
    }

    // Load first block into top level state.
    const u32 max_level = mmbit_maxlevel(total_bits);
    s[0].mask = block;
    s[0].itkey = 0;
    return mmbit_sparse_iter_exec(bits, key, idx, 0, max_level,
                                  s, it_root, it);
}

/** \brief Specialisation of \ref mmbit_sparse_iter_begin for flat models. */
static really_inline
u32 mmbit_sparse_iter_begin_flat(const u8 *bits, u32 total_bits, u32 *idx,
                                 const struct mmbit_sparse_iter *it_root,
                                 struct mmbit_sparse_state *s) {
    // Small cases have everything in the root iterator mask.
    if (total_bits <= MMB_KEY_BITS) {
        MMB_TYPE block = mmbit_get_flat_block(bits, total_bits);
        block &= it_root->mask;
        if (!block) {
            return MMB_INVALID;
        }

        s->mask = block;
        u32 key = mmb_ctz(block);
        *idx = mmbit_mask_index(key, it_root->mask);
        return key;
    }

    // Otherwise, the root iterator mask tells us which blocks (which we lay out
    // linearly in the flat model) could contain keys.
    assert(mmbit_maxlevel(total_bits) == 1); // Should only be two levels
    MMB_TYPE root = it_root->mask;
    for (; root; root &= (root - 1)) {
        u32 bit = mmb_ctz(root);
        u32 bit_idx = mmbit_mask_index(bit, it_root->mask);
        u32 iter_key = it_root->val + bit_idx;
        const struct mmbit_sparse_iter *it = it_root + iter_key;
        u32 block_key_min = bit * MMB_KEY_BITS;
        u32 block_key_max = block_key_min + MMB_KEY_BITS;
        MMB_TYPE block;
        if (block_key_max > total_bits) {
            block_key_max = total_bits;
            block = mmbit_get_flat_block(bits + (bit * sizeof(MMB_TYPE)),
                                          block_key_max - block_key_min);
        } else {
            block = mmb_load(bits + (bit * sizeof(MMB_TYPE)));
        }

        block &= it->mask;
        if (block) {
            s[0].mask = root;
            s[1].mask = block;
            s[1].itkey = iter_key;
            u32 key = mmb_ctz(block);
            *idx = it->val + mmbit_mask_index(key, it->mask);
            return key + block_key_min;
        }
    }

    return MMB_INVALID;
}

/** \brief Sparse iterator, find first key.
 *
 * Returns the first of the bits specified by the iterator \a it_root that is
 * on, and initialises the state \a s. If none of the bits specified by the
 * iterator are on, returns MMB_INVALID.
 */
static really_inline
u32 mmbit_sparse_iter_begin(const u8 *bits, u32 total_bits, u32 *idx,
                            const struct mmbit_sparse_iter *it_root,
                            struct mmbit_sparse_state *s) {
    assert(ISALIGNED_N(it_root, alignof(struct mmbit_sparse_iter)));

    // Our state _may_ be on the stack
#ifndef _WIN32
    assert(ISALIGNED_N(s, alignof(struct mmbit_sparse_state)));
#else
    assert(ISALIGNED_N(s, 4));
#endif

    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);
    // iterator should have _something_ at the root level
    assert(it_root->mask != 0);
    u32 key;
    if (mmbit_is_flat_model(total_bits)) {
        key = mmbit_sparse_iter_begin_flat(bits, total_bits, idx, it_root, s);
    } else {
        key = mmbit_sparse_iter_begin_big(bits, total_bits, idx, it_root, s);
    }
    if (key != MMB_INVALID) {
        assert(key < total_bits);
        assert(mmbit_isset(bits, total_bits, key));
    }
    return key;
}

static really_inline
u32 mmbit_sparse_iter_next_big(const u8 *bits, u32 total_bits, u32 last_key,
                               u32 *idx,
                               const struct mmbit_sparse_iter *it_root,
                               struct mmbit_sparse_state *s) {
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 key = last_key >> MMB_KEY_SHIFT;
    s[max_level].mask &= (s[max_level].mask - 1);
    const struct mmbit_sparse_iter *it = it_root + s[max_level].itkey;
    return mmbit_sparse_iter_exec(bits, key, idx, max_level, max_level, s,
                                  it_root, it);
}

/** \brief Specialisation of \ref mmbit_sparse_iter_next for flat models. */
static really_inline
u32 mmbit_sparse_iter_next_flat(const u8 *bits, const u32 total_bits, u32 *idx,
                                const struct mmbit_sparse_iter *it_root,
                                struct mmbit_sparse_state *s) {
    if (total_bits <= MMB_KEY_BITS) {
        // All of our data is already in the s->mask, so we just need to scrape
        // off the next match.
        s->mask &= (s->mask - 1);
        if (s->mask) {
            u32 key = mmb_ctz(s->mask);
            *idx = mmbit_mask_index(key, it_root->mask);
            return key;
        }
    } else {
        assert(s[0].mask);

        s[1].mask &= (s[1].mask - 1); // Remove previous key from iter state.
        u32 bit = mmb_ctz(s[0].mask); // Flat block currently being accessed.

        for (;;) {
            if (s[1].mask) {
                u32 key = mmb_ctz(s[1].mask);
                const struct mmbit_sparse_iter *it = it_root + s[1].itkey;
                *idx = it->val + mmbit_mask_index(key, it->mask);
                key += (bit * MMB_KEY_BITS);
                return key;
            }

            // Otherwise, we have no keys left in this block. Consult the root
            // mask and find the next one.

            s[0].mask &= s[0].mask - 1;
            if (!s[0].mask) {
                break;
            }

            bit = mmb_ctz(s[0].mask);
            u32 bit_idx = mmbit_mask_index(bit, it_root->mask);
            u32 iter_key = it_root->val + bit_idx;
            const struct mmbit_sparse_iter *it = it_root + iter_key;
            u32 block_key_min = bit * MMB_KEY_BITS;
            u32 block_key_max = block_key_min + MMB_KEY_BITS;
            MMB_TYPE block;
            if (block_key_max > total_bits) {
                block_key_max = total_bits;
                block = mmbit_get_flat_block(bits + (bit * sizeof(MMB_TYPE)),
                                              block_key_max - block_key_min);
            } else {
                block = mmb_load(bits + (bit * sizeof(MMB_TYPE)));
            }

            s[1].mask = block & it->mask;
            s[1].itkey = iter_key;
        }
    }

    return MMB_INVALID;
}

/** \brief Sparse iterator, find next key.
 *
 * Takes in a sparse iterator tree structure \a it_root and a state array, and
 * finds the next on bit (from the set of bits specified in the iterator).
 *
 * NOTE: The sparse iterator stores copies of the multibit blocks in its state,
 * so it is not necessarily safe to set or unset bits in the multibit while
 * iterating: the changes you make may or may not be taken into account
 * by the iterator.
 */
static really_inline
u32 mmbit_sparse_iter_next(const u8 *bits, u32 total_bits, u32 last_key,
                           u32 *idx, const struct mmbit_sparse_iter *it_root,
                           struct mmbit_sparse_state *s) {
    assert(ISALIGNED_N(it_root, alignof(struct mmbit_sparse_iter)));

    // Our state _may_ be on the stack
#ifndef _WIN32
    assert(ISALIGNED_N(s, alignof(struct mmbit_sparse_state)));
#else
    assert(ISALIGNED_N(s, 4));
#endif

    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);
    MDEBUG_PRINTF("NEXT (total_bits=%u, last_key=%u)\n", total_bits, last_key);
    UNUSED u32 last_idx = *idx; // for assertion at the end
    // our iterator should have _something_ at the root level
    assert(it_root->mask != 0);
    assert(last_key < total_bits);

    u32 key;
    if (mmbit_is_flat_model(total_bits)) {
        key = mmbit_sparse_iter_next_flat(bits, total_bits, idx, it_root, s);
    } else {
        key = mmbit_sparse_iter_next_big(bits, total_bits, last_key, idx,
                                         it_root, s);
    }
    if (key != MMB_INVALID) {
        MDEBUG_PRINTF("END NEXT: key=%u, idx=%u\n", key, *idx);
        assert(key < total_bits);
        assert(key > last_key);
        assert(mmbit_isset(bits, total_bits, key));
        assert(*idx > last_idx);
    } else {
        MDEBUG_PRINTF("END NEXT: no more keys\n");
    }
    return key;
}

/** \brief Specialisation of \ref mmbit_sparse_iter_unset for flat models. */
static really_inline
void mmbit_sparse_iter_unset_flat(u8 *bits, u32 total_bits,
                                  const struct mmbit_sparse_iter *it_root) {
    if (total_bits <= MMB_KEY_BITS) {
        // Everything is in the root mask: we can just mask those bits off.
        MMB_TYPE block = mmbit_get_flat_block(bits, total_bits);
        block &= ~it_root->mask;
        mmb_store_partial(bits, block, total_bits);
        return;
    }

    // Larger case, we have two iterator levels to worry about.
    u32 bit_idx = 0;
    for (MMB_TYPE root = it_root->mask; root; root &= (root - 1), bit_idx++) {
        u32 bit = mmb_ctz(root);
        u32 block_key_min = bit * MMB_KEY_BITS;
        u32 block_key_max = block_key_min + MMB_KEY_BITS;
        u8 *block_ptr = bits + (bit * sizeof(MMB_TYPE));
        u32 iter_key = it_root->val + bit_idx;
        const struct mmbit_sparse_iter *it = it_root + iter_key;
        if (block_key_max <= total_bits) {
            // Full-sized block.
            MMB_TYPE block = mmb_load(block_ptr);
            block &= ~it->mask;
            mmb_store(block_ptr, block);
        } else {
            // Runt (final) block.
            u32 num_bits = total_bits - block_key_min;
            MMB_TYPE block = mmbit_get_flat_block(block_ptr, num_bits);
            block &= ~it->mask;
            mmb_store_partial(block_ptr, block, num_bits);
            break; // We know this is the last block.
        }
    }
}

static really_inline
void mmbit_sparse_iter_unset_big(u8 *bits, u32 total_bits,
                                 const struct mmbit_sparse_iter *it_root,
                                 struct mmbit_sparse_state *s) {
    const struct mmbit_sparse_iter *it = it_root;
    MMB_TYPE block = mmb_load(bits) & it->mask;
    if (!block) {
        return;
    }

    u32 key = 0;
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;

    // Load first block into top level state
    s[level].mask = block;
    s[level].itkey = 0;
    for (;;) {
        block = s[level].mask;
        if (block) {
            if (level == max_level) {
                // bottom level block: we want to mask out the bits specified
                // by the iterator mask and then go back up a level.
                u8 *block_ptr =
                    mmbit_get_level_root(bits, level) + key * sizeof(MMB_TYPE);
                MMB_TYPE real_block = mmb_load(block_ptr);
                real_block &= ~(it->mask);
                mmb_store(block_ptr, real_block);
                goto uplevel; // still cheap and nasty
            } else {
                u32 bit = mmb_ctz(block);
                key = (key << MMB_KEY_SHIFT) + bit;
                level++;

                // iterator record is the start of the level (current it->val)
                // plus N, where N is the dense index of the bit in the current
                // level's itmask
                u32 iter_key = it->val + mmbit_mask_index(bit, it->mask);
                it = it_root + iter_key;
                MMB_TYPE nextblock =
                    mmb_load(mmbit_get_level_root_const(bits, level) +
                             key * sizeof(MMB_TYPE));
                s[level].mask = nextblock & it->mask;
                s[level].itkey = iter_key;
            }
        } else {
uplevel:
            // No bits set in this block
            if (level == 0) {
                return; // we are done
            }
            u8 *block_ptr =
                mmbit_get_level_root(bits, level) + key * sizeof(MMB_TYPE);
            MMB_TYPE real_block = mmb_load(block_ptr);
            key >>= MMB_KEY_SHIFT;
            level--;

            if (real_block == 0) {
                // If we've zeroed our block For Real (unmasked by iterator),
                // we can clear the parent bit that led us to it, so that
                // we don't go down this particular garden path again later.
                u32 bit = mmb_ctz(s[level].mask);
                u8 *parent_ptr =
                    mmbit_get_level_root(bits, level) + key * sizeof(MMB_TYPE);
                MMB_TYPE parent_block = mmb_load(parent_ptr);
                mmb_clear(&parent_block, bit);
                mmb_store(parent_ptr, parent_block);
            }

            // Update state mask and iterator
            s[level].mask &= (s[level].mask - 1);
            it = it_root + s[level].itkey;
        }
    }
}

/** \brief Sparse iterator, unset all bits.
 *
 * Takes in a sparse iterator tree structure and switches off any entries found
 * therein.
 */
static really_inline
void mmbit_sparse_iter_unset(u8 *bits, u32 total_bits,
                             const struct mmbit_sparse_iter *it,
                             struct mmbit_sparse_state *s) {
    assert(ISALIGNED_N(it, alignof(struct mmbit_sparse_iter)));

    // Our state _may_ be on the stack
#ifndef _WIN32
    assert(ISALIGNED_N(s, alignof(struct mmbit_sparse_state)));
#else
    assert(ISALIGNED_N(s, 4));
#endif

    MDEBUG_PRINTF("%p total_bits %u\n", bits, total_bits);

#ifdef MMB_TRACE_WRITES
    MMB_TRACE("ITER-UNSET iter=[");
    mmbit_sparse_iter_dump(it, total_bits);
    printf("] actually on=[");
    struct mmbit_sparse_state tmp[MAX_SPARSE_ITER_STATES];
    u32 idx = 0;
    u32 i = mmbit_sparse_iter_begin(bits, total_bits, &idx, it, tmp);
    for (; i != MMB_INVALID;
         i = mmbit_sparse_iter_next(bits, total_bits, i, &idx, it, tmp)) {
        printf(" %u", i);
    }
    printf("]\n");
#endif

    if (mmbit_is_flat_model(total_bits)) {
        mmbit_sparse_iter_unset_flat(bits, total_bits, it);
    } else {
        mmbit_sparse_iter_unset_big(bits, total_bits, it, s);
    }
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MULTIBIT_H
