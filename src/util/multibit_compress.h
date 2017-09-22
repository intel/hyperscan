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

/** file
 * \brief multibit compression API: compress / decompress / size
 */

#ifndef MULTIBIT_COMPRESS_H
#define MULTIBIT_COMPRESS_H

#include "multibit.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \brief size API. */
static really_inline
size_t mmbit_compsize(const u8 *bits, u32 total_bits) {
    // Deal with flat model.
    if (total_bits <= MMB_FLAT_MAX_BITS) {
        return (ROUNDUP_N(total_bits, 8) / 8);
    }
    // Deal with all cleared mmb.
    if (mmb_load(bits) == 0) {
        return sizeof(MMB_TYPE);
    }
    // Deal with normal pyramid mmb.
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    u32 key = 0;
    u32 key_rem = 0;
    u32 num_block = 0;
    // Iteration-version of DFS
    while (1) {
        if (key_rem < MMB_KEY_BITS) {
            const u8 *block_ptr = mmbit_get_level_root_const(bits, level) +
                                  key * sizeof(MMB_TYPE);
            MMB_TYPE block = mmb_load(block_ptr);
            MMB_TYPE block_1 = block & ~mmb_mask_zero_to_nocheck(key_rem);
            if (mmb_popcount(block) == mmb_popcount(block_1)) {
                num_block++;
            }
            if (level < max_level && block_1) {
                key = (key << MMB_KEY_SHIFT) + mmb_ctz(block_1);
                key_rem = 0;
                level++;
                continue;
            }
        }
        if (level-- == 0) {
            return sizeof(MMB_TYPE) * num_block;
        }
        key_rem = (key & MMB_KEY_MASK) + 1;
        key >>= MMB_KEY_SHIFT;
    }
}

/** \brief compress API. */
static really_inline
char mmbit_compress(const u8 *bits, u32 total_bits, u8 *comp,
                    size_t *comp_space, size_t max_comp_space) {
    UNUSED u8 *comp_init = comp;
    // Compute comp_size first.
    size_t comp_size = mmbit_compsize(bits, total_bits);
    // Check whether out of writable range.
    if (comp_size > max_comp_space) {
        return 0;
    }
    *comp_space = comp_size; // Return comp_size outside.
    // Deal with flat model.
    if (total_bits <= MMB_FLAT_MAX_BITS) {
        memcpy(comp, bits, comp_size);
        return 1;
    }
    // Deal with all cleared mmb.
    if (mmb_load(bits) == 0) {
        memcpy(comp, bits, sizeof(MMB_TYPE));
        return 1;
    }
    // Deal with normal pyramid mmb.
    const u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    u32 key = 0;
    u32 key_rem = 0;
    // Iteration-version of DFS
    while (1) {
        if (key_rem < MMB_KEY_BITS) {
            const u8 *block_ptr = mmbit_get_level_root_const(bits, level) +
                                  key * sizeof(MMB_TYPE);
            MMB_TYPE block = mmb_load(block_ptr);
            MMB_TYPE block_1 = block & ~mmb_mask_zero_to_nocheck(key_rem);
            if (mmb_popcount(block) == mmb_popcount(block_1)) {
                memcpy(comp, &block, sizeof(MMB_TYPE));
                comp += sizeof(MMB_TYPE);
            }
            if (level < max_level && block_1) {
                key = (key << MMB_KEY_SHIFT) + mmb_ctz(block_1);
                key_rem = 0;
                level++;
                continue;
            }
        }
        if (level-- == 0) {
            break;
        }
        key_rem = (key & MMB_KEY_MASK) + 1;
        key >>= MMB_KEY_SHIFT;
    }
    assert((u32)(comp - comp_init) == comp_size);
    return 1;
}

/** \brief decompress API. */
static really_inline
char mmbit_decompress(u8 *bits, u32 total_bits, const u8 *comp,
                      size_t *comp_space, size_t max_comp_space) {
    UNUSED const u8 *comp_init = comp;
    size_t comp_size;
    // Deal with flat model.
    if (total_bits <= MMB_FLAT_MAX_BITS) {
        comp_size = ROUNDUP_N(total_bits, 8) / 8;
        memcpy(bits, comp, comp_size);
        *comp_space = comp_size;
        return 1;
    }
    // Deal with all cleared mmb.
    if (mmb_load(comp) == 0) {
        comp_size = sizeof(MMB_TYPE);
        memcpy(bits, comp, comp_size);
        *comp_space = comp_size;
        return 1;
    }
    // Deal with normal mmb.
    u32 max_level = mmbit_maxlevel(total_bits);
    u32 level = 0;
    u32 key = 0;
    u32 key_rem = 0;
    UNUSED const u8 *comp_end = comp_init + max_comp_space;
    // Iteration-version of DFS
    memcpy(bits, comp, sizeof(MMB_TYPE)); // Copy root block first.
    comp += sizeof(MMB_TYPE);
    while (1) {
        if (key_rem < MMB_KEY_BITS) {
            u8 *block_ptr = mmbit_get_level_root(bits, level) +
                            key * sizeof(MMB_TYPE);
            MMB_TYPE block = mmb_load(block_ptr);
            MMB_TYPE block_1 = block & ~mmb_mask_zero_to_nocheck(key_rem);
            if (level < max_level && block_1) {
                key = (key << MMB_KEY_SHIFT) + mmb_ctz(block_1);
                u8 *block_ptr_1 = mmbit_get_level_root(bits, level + 1) +
                                  key * sizeof(MMB_TYPE);
                memcpy(block_ptr_1, comp, sizeof(MMB_TYPE));
                comp += sizeof(MMB_TYPE);
		        if (comp > comp_end) {
                    return 0; // Out of buffer.
                }
                key_rem = 0;
                level++;
                continue;
            }
        }
        if (level-- == 0) {
            break;
        }
        key_rem = (key & MMB_KEY_MASK) + 1;
        key >>= MMB_KEY_SHIFT;
    }
    comp_size = (u32)(comp - comp_init);
    *comp_space = comp_size;
    return 1;
}

#ifdef __cplusplus
} // extern "C"
#endif

#endif // MULTBIT_COMPRESS_H

