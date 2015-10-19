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

#include "gtest/gtest.h"
#include "ue2common.h"

#define PQ_T u32
#define PQ_COMP(pqc_items, pqc_i, pqc_j) (pqc_items[pqc_i] > pqc_items[pqc_j])
#define PQ_COMP_B(pqc_items, pqc_i, pqc_jj) (pqc_items[pqc_i] > pqc_jj)

#include "util/pqueue.h"

static void sort(u32 *items, u32 count) {
    for (u32 i = 0; i < count; i++) {
        pq_insert(items, i, items[i]);
    }

    for (u32 i = 0; i < count; i++) {
        u32 top = *pq_top(items);
        pq_pop(items, count - i);
        items[count - i - 1] = top;
    }
}

TEST(pqueue, sort1) {
    u32 in[] = {1, 2, 3, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort2) {
    u32 in[] = {1, 2, 4, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort3) {
    u32 in[] = {1, 3, 2, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort4) {
    u32 in[] = {1, 3, 4, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort5) {
    u32 in[] = {1, 4, 2, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort6) {
    u32 in[] = {1, 4, 3, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort7) {
    u32 in[] = {2, 1, 3, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort8) {
    u32 in[] = {2, 1, 4, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort9) {
    u32 in[] = {2, 3, 1, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort10) {
    u32 in[] = {2, 3, 4, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort11) {
    u32 in[] = {2, 4, 1, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort12) {
    u32 in[] = {2, 4, 3, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort13) {
    u32 in[] = {3, 1, 2, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort14) {
    u32 in[] = {3, 1, 4, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort15) {
    u32 in[] = {3, 2, 1, 4};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort16) {
    u32 in[] = {3, 2, 4, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort17) {
    u32 in[] = {3, 4, 1, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort18) {
    u32 in[] = {3, 4, 2, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort19) {
    u32 in[] = {4, 1, 2, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort20) {
    u32 in[] = {4, 1, 3, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort21) {
    u32 in[] = {4, 2, 1, 3};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort22) {
    u32 in[] = {4, 2, 3, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort23) {
    u32 in[] = {4, 3, 1, 2};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, sort24) {
    u32 in[] = {4, 3, 2, 1};
    u32 out[] = {1, 2, 3, 4};

    sort(in, ARRAY_LENGTH(in));
    ASSERT_EQ(0, memcmp(in, out, sizeof(in)));
}

TEST(pqueue, queue1) {
    u32 in[] = {1, 2, 3, 4, 5, 6, 7, 8};
    u32 expected[] = {4, 5, 6, 7, 8, 3, 2, 1};
    u32 temp[ARRAY_LENGTH(in)];
    u32 output[ARRAY_LENGTH(in)];

    u32 queue_size = 0;
    u32 i = 0, o = 0;
    for (; i < 4; i++) {
        pq_insert(temp, queue_size, in[i]);
        queue_size++;
    }

    while (queue_size) {
        output[o++] = *pq_top(temp);
        pq_pop(temp, queue_size);
        queue_size--;
        if (i < ARRAY_LENGTH(in)) {
            pq_insert(temp, queue_size, in[i++]);
            queue_size++;
        }
    }

//     for (u32 j = 0; j < o; j++) {
//         printf("%u\n", output[j]);
//     }

    ASSERT_EQ(0, memcmp(output, expected, sizeof(in)));
}

TEST(pqueue, queue2) {
    u32 in[] = {8, 7, 6, 5, 4, 3, 2, 1};
    u32 expected[] = {8, 7, 6, 5, 4, 3, 2, 1};
    u32 temp[ARRAY_LENGTH(in)];
    u32 output[ARRAY_LENGTH(in)];

    u32 queue_size = 0;
    u32 i = 0, o = 0;
    for (; i < 4; i++) {
        pq_insert(temp, queue_size, in[i]);
        queue_size++;
    }

    while (queue_size) {
        output[o++] = *pq_top(temp);
        pq_pop(temp, queue_size);
        queue_size--;
        if (i < ARRAY_LENGTH(in)) {
            pq_insert(temp, queue_size, in[i++]);
            queue_size++;
        }
    }

    ASSERT_EQ(0, memcmp(output, expected, sizeof(in)));
}

TEST(pqueue, queue3) {
    u32 in[] = {1, 8, 2, 7, 3, 6, 4, 5};
    u32 expected[] = {8, 7, 6, 4, 5, 3, 2, 1};
    u32 temp[ARRAY_LENGTH(in)];
    u32 output[ARRAY_LENGTH(in)];

    u32 queue_size = 0;
    u32 i = 0, o = 0;
    for (; i < 4; i++) {
        pq_insert(temp, queue_size, in[i]);
        queue_size++;
    }

    while (queue_size) {
        output[o++] = *pq_top(temp);
        pq_pop(temp, queue_size);
        queue_size--;
        if (i < ARRAY_LENGTH(in)) {
            pq_insert(temp, queue_size, in[i++]);
            queue_size++;
        }
    }

    ASSERT_EQ(0, memcmp(output, expected, sizeof(in)));
}

