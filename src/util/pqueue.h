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

#ifndef PQUEUE_H
#define PQUEUE_H

#include "ue2common.h"

static really_inline u32
pq_left(u32 i) {
    return (i << 1) + 1;
}

static really_inline u32
pq_right(u32 i) {
    return (i << 1) + 2;
}

static really_inline
u32 pq_parent(u32 i) {
    return (i - 1) >> 1;
}

static really_inline
void pq_sift(PQ_T *items, u32 start, u32 end) {
    u32 j = start;
    PQ_T j_temp = items[j];

    while (pq_left(j) < end) {
        u32 max_child;

        if (pq_right(j) < end && PQ_COMP(items, pq_right(j), pq_left(j))) {
            max_child = pq_right(j);
        } else {
            max_child = pq_left(j);
        }

        if (PQ_COMP_B(items, max_child, j_temp)) {
            items[j] = items[max_child];
            j = max_child;
        } else {
            /* j is already less than its children. We know heap property
             * is already maintained for children we are done */
            break;
        }
    }
    items[j] = j_temp;
}

static really_inline
PQ_T *pq_top(PQ_T *items) {
    return items;
}

static really_inline
void pq_pop(PQ_T *items, u32 item_count) {
    item_count--;
    items[0] = items[item_count];
    pq_sift(items, 0, item_count);
}

static really_inline
void pq_insert(PQ_T *items, u32 item_count, PQ_T new_item) {
    u32 pos = item_count;
    while (pos) {
        u32 parent = pq_parent(pos);
        if (!PQ_COMP_B(items, parent, new_item)) {
            items[pos] = items[parent];
            pos = parent;
        } else {
            break;
        }
    }
    items[pos] = new_item;
}

static really_inline
void pq_replace_top(PQ_T *items, u32 item_count, PQ_T new_item) {
    items[0] = new_item;
    pq_sift(items, 0, item_count);
}

#endif

