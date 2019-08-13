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

/** \file
 * \brief Multibit: build code (for sparse iterators)
 */
#include "multibit.h"
#include "multibit_build.h"
#include "scatter.h"
#include "ue2common.h"
#include "rose/rose_build_scatter.h"
#include "util/compile_error.h"

#include <cassert>
#include <cstring> // for memset
#include <map>
#include <queue>
#include <vector>

using namespace std;

namespace ue2 {

u32 mmbit_size(u32 total_bits) {
    if (total_bits > MMB_MAX_BITS) {
        throw ResourceLimitError();
    }

    // Flat model multibit structures are just stored as a bit vector.
    if (total_bits <= MMB_FLAT_MAX_BITS) {
        return ROUNDUP_N(total_bits, 8) / 8;
    }

    u64a current_level = 1; // Number of blocks on current level.
    u64a total = 0;         // Total number of blocks.
    while (current_level * MMB_KEY_BITS < total_bits) {
        total += current_level;
        current_level <<= MMB_KEY_SHIFT;
    }

    // Last level is a one-for-one bit vector. It needs room for total_bits
    // elements, rounded up to the nearest block.
    u64a last_level = ((u64a)total_bits + MMB_KEY_BITS - 1) / MMB_KEY_BITS;
    total += last_level;

    assert(total * sizeof(MMB_TYPE) <= UINT32_MAX);
    return (u32)(total * sizeof(MMB_TYPE));
}

namespace {
struct TreeNode {
    MMB_TYPE mask = 0;
    u32 depth = 0;
    map<u32, TreeNode> children; // keyed by rkey
};
} // namespace

static
void addNode(TreeNode &tree, u32 depth, u32 key, s32 ks, u32 rkey) {
    u32 bit = (key >> ks) & MMB_KEY_MASK;
    DEBUG_PRINTF("depth=%u, key=%u, ks=%d, rkey=%u, bit=%u\n", depth, key, ks,
                 rkey, bit);
    mmb_set(&tree.mask, bit); // add bit to this level
    tree.depth = depth; // record depth
    // next level
    rkey = (rkey << MMB_KEY_SHIFT) + bit;
    ks -= MMB_KEY_SHIFT;
    depth++;
    if (ks >= 0) {
        addNode(tree.children[rkey], depth, key, ks, rkey);
    }
}

static
void bfs(vector<mmbit_sparse_iter> &out, const TreeNode &tree) {
    queue<const TreeNode *> q;
    q.push(&tree);

    vector<u32> levels;
    u32 depth = 0;

    DEBUG_PRINTF("walking q\n");

    while (!q.empty()) {
        const TreeNode *t = q.front();
        q.pop();

        if (depth != t->depth) {
            depth = t->depth;
            levels.push_back(out.size());
        }

        DEBUG_PRINTF("pop: mask=0x%08llx, depth=%u, children.size()=%zu\n",
                     t->mask, t->depth, t->children.size());

        out.push_back(mmbit_sparse_iter());
        memset(&out.back(), 0, sizeof(mmbit_sparse_iter));
        mmbit_sparse_iter &record = out.back();
        record.mask = t->mask;
        record.val = 0;

        for (auto &e : t->children) {
            q.push(&e.second);
        }
    }

    // val for records in non-last levels is the iterator array start offset
    // for that iterator record's children
    u32 start = 0;
    for (size_t i = 0; i < levels.size(); i++) {
        u32 start_next = levels[i];
        u32 population = 0;
        DEBUG_PRINTF("next level starts at %u\n", start_next);
        for (u32 j = start; j < start_next; j++) {
            out[j].val = start_next + population;
            DEBUG_PRINTF("  children of %u start at %u\n", j, out[j].val);
            population += mmb_popcount(out[j].mask);
        }
        start = start_next;
    }

    // val for records in the last level is the cumulative popcount
    u32 population = 0;
    for (size_t i = start; i < out.size(); i++) {
        DEBUG_PRINTF("last level: i=%zu, population=%u\n", i, population);
        out[i].val = population;
        population += mmb_popcount(out[i].mask);
    }
}

/** \brief Construct a sparse iterator over the values in \a bits for a
 * multibit of size \a total_bits. */
vector<mmbit_sparse_iter> mmbBuildSparseIterator(const vector<u32> &bits,
                                                 u32 total_bits) {
    vector<mmbit_sparse_iter> out;
    assert(!bits.empty());
    assert(total_bits > 0);
    assert(total_bits <= MMB_MAX_BITS);

    DEBUG_PRINTF("building sparse iter for %zu of %u bits\n",
                 bits.size(), total_bits);

    s32 ks = (total_bits > 1 ? mmbit_keyshift(total_bits) : 0);

    // Construct an intermediate tree
    TreeNode tree;
    for (const auto &bit : bits) {
        assert(bit < total_bits);
        addNode(tree, 0, bit, ks, 0);
    }

    // From our intermediate tree, lay the data out with a breadth-first walk
    bfs(out, tree);
    assert(!out.empty());

#ifdef DEBUG
    DEBUG_PRINTF("dump of iterator tree:\n");
    for (size_t i = 0; i < out.size(); ++i) {
        printf("    %zu:\tmask=0x%08llx, val=%u\n", i, out[i].mask, out[i].val);
    }
#endif

    DEBUG_PRINTF("iter has %zu records\n", out.size());
    return out;
}

template<typename T>
static
void add_scatter(vector<T> *out, u32 offset, u64a mask) {
    out->emplace_back();
    T &su = out->back();
    memset(&su, 0, sizeof(su));
    su.offset = offset;
    su.val = mask;
    DEBUG_PRINTF("add %llu at offset %u\n", mask, offset);
}

static
u32 mmbit_get_level_root_offset(u32 level) {
    return mmbit_root_offset_from_level[level] * sizeof(MMB_TYPE);
}

void mmbBuildInitRangePlan(u32 total_bits, u32 begin, u32 end,
                           scatter_plan_raw *out) {
    DEBUG_PRINTF("building scatter plan for [%u, %u]/%u\n", begin, end,
                 total_bits);
    if (!total_bits) {
        return;
    }

    if (total_bits <= MMB_FLAT_MAX_BITS) {
        // Handle flat model cases: first a bunch of 64-bit full-sized blocks,
        // then a single runt block at the end.
        u32 dest = 0; // dest offset
        u32 bits = total_bits;
        u32 base = 0;
        for (; bits > 64; bits -= 64, base += 64, dest += 8) {
            MMB_TYPE mask = get_flat_masks(base, begin, end);
            add_scatter(&out->p_u64a, dest, mask);
        }

        // Last chunk.
        assert(bits > 0 && bits <= 64);

        MMB_TYPE mask = get_flat_masks(base, begin, end);
        if (bits <= 8) {
            add_scatter(&out->p_u8, dest + 0, mask);
        } else if (bits <= 16) {
            add_scatter(&out->p_u16, dest + 0, mask);
        } else if (bits <= 24) {
            add_scatter(&out->p_u16, dest + 0, mask);
            add_scatter(&out->p_u8,  dest + 2, mask >> 16);
        } else if (bits <= 32) {
            add_scatter(&out->p_u32, dest + 0, mask);
        } else if (bits <= 40) {
            add_scatter(&out->p_u32, dest + 0, mask);
            add_scatter(&out->p_u8,  dest + 4, mask >> 32);
        } else if (bits <= 48) {
            add_scatter(&out->p_u32, dest + 0, mask);
            add_scatter(&out->p_u16, dest + 4, mask >> 32);
        } else if (bits <= 56) {
            add_scatter(&out->p_u32, dest + 0, mask);
            add_scatter(&out->p_u16, dest + 4, mask >> 32);
            add_scatter(&out->p_u8,  dest + 6, mask >> 48);
        } else {
            add_scatter(&out->p_u64a, dest + 0, mask);
        }
        return;
    }

    /* handle the multilevel case */
    s32 ks = mmbit_keyshift(total_bits);
    u32 level = 0;
    assert(sizeof(MMB_TYPE) == sizeof(u64a));

    if (begin == end) {
        add_scatter(&out->p_u64a, 0, 0);
        return;
    }

    for (;;) {
        u32 block_offset = mmbit_get_level_root_offset(level);
        u32 k1 = begin >> ks, k2 = end >> ks;

        // Summary blocks need to account for the runt block on the end.
        if ((k2 << ks) != end) {
            k2++;
        }

        // Partial block to deal with beginning.
        block_offset += (k1 / MMB_KEY_BITS) * sizeof(MMB_TYPE);
        if (k1 % MMB_KEY_BITS) {
            u32 idx = k1 / MMB_KEY_BITS;
            u32 block_end = (idx + 1) * MMB_KEY_BITS;

            // Because k1 % MMB_KEY_BITS != 0, we can avoid checking edge cases
            // here (see the branch in mmb_mask_zero_to).
            MMB_TYPE mask = (-MMB_ONE) << (k1 % MMB_KEY_BITS);

            if (k2 < block_end) {
                assert(k2 % MMB_KEY_BITS);
                mask &= mmb_mask_zero_to_nocheck(k2 % MMB_KEY_BITS);
                add_scatter(&out->p_u64a, block_offset, mask);
                goto next_level;
            } else {
                add_scatter(&out->p_u64a, block_offset, mask);
                k1 = block_end;
                block_offset += sizeof(MMB_TYPE);
            }
        }

        // Write blocks filled with ones until we get to the last block.
        for (; k1 < (k2 & ~MMB_KEY_MASK); k1 += MMB_KEY_BITS) {
            add_scatter(&out->p_u64a, block_offset, -MMB_ONE);
            block_offset += sizeof(MMB_TYPE);
        }

        // Final block.
        if (likely(k1 < k2)) {
            // Again, if k2 was at a block boundary, it would have been handled
            // by the previous loop, so we know k2 % MMB_KEY_BITS != 0 and can
            // avoid the branch in mmb_mask_zero_to here.
            assert(k2 % MMB_KEY_BITS);
            MMB_TYPE mask = mmb_mask_zero_to_nocheck(k2 % MMB_KEY_BITS);

            add_scatter(&out->p_u64a, block_offset, mask);
        }

    next_level:
        if (ks == 0) {
            break; // Last level is done, finished.
        }

        ks -= MMB_KEY_SHIFT;
        level++;
    }
}

void mmbBuildClearPlan(u32 total_bits, scatter_plan_raw *out) {
    return mmbBuildInitRangePlan(total_bits, 0, 0, out);
}

} // namespace ue2
