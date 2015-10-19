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

/** \file
 * \brief Functions for packing/unpacking arrays.
 */

#ifndef UTIL_PACK_BITS_H
#define UTIL_PACK_BITS_H

#include "ue2common.h"
#include "unaligned.h"
#include "partial_store.h"

/**
 * \brief Pack bits from an array of 32-bit words into \a out.
 *
 * \param out Output array. Must be large enough to store sum(bits).
 * \param v Input array.
 * \param bits Number of low bits in the corresponding element of \a v to pack.
 * \param elements Size of the \a v and \a bits arrays.
 */
static really_inline
void pack_bits_32(char *out, const u32 *v, const u32 *bits,
                  const unsigned int elements);

/**
 * \brief Pack bits from an array of 64-bit words into \a out.
 *
 * \param out Output array. Must be large enough to store sum(bits).
 * \param v Input array.
 * \param bits Number of low bits in the corresponding element of \a v to pack.
 * \param elements Size of the \a v and \a bits arrays.
 */
static really_inline
void pack_bits_64(char *out, const u64a *v, const u32 *bits,
                  const unsigned int elements);

/**
 * \brief Unpack bits into an array of 32-bit words according to the counts
 * given.
 *
 * \param v Output array.
 * \param in Packed input array.
 * \param bits Number of bits to unpack into the corresponding element of \a v.
 * \param elements Size of the \a v and \a bits arrays.
 */
static really_inline
void unpack_bits_32(u32 *v, const u8 *in, const u32 *bits,
                    const unsigned int elements);

/**
 * \brief Unpack bits into an array of 64-bit words according to the counts
 * given.
 *
 * \param v Output array.
 * \param in Packed input array.
 * \param bits Number of bits to unpack into the corresponding element of \a v.
 * \param elements Size of the \a v and \a bits arrays.
 */
static really_inline
void unpack_bits_64(u64a *v, const u8 *in, const u32 *bits,
                    const unsigned int elements);

/*
 * Inline implementations follow.
 */

static really_inline
void pack_bits_32(char *out, const u32 *v, const u32 *bits,
                     const unsigned int elements) {
    u32 write = 0; // accumulator
    u32 idx = 0;   // acc holds this many bits

    for (unsigned int i = 0; i < elements; i++) {
        assert(bits[i] <= 32);
        write |= (v[i] << idx);
        idx += bits[i];
        if (idx >= 32) {
            unaligned_store_u32(out, write);
            out += 4;
            idx -= 32;
            u32 leftover = bits[i] - idx;
            if (leftover == 32) {
                write = 0;
            } else {
                assert(leftover < 32);
                write = v[i] >> leftover;
            }
        }
    }

    // There might be a write left over.
    partial_store_u32(out, write, (idx + 7) / 8);
}

static really_inline
void pack_bits_64(char *out, const u64a *v, const u32 *bits,
                     const unsigned int elements) {
    u64a write = 0; // accumulator
    u32 idx = 0;    // acc holds this many bits

    for (unsigned int i = 0; i < elements; i++) {
        assert(bits[i] <= 64);
        write |= (v[i] << idx);
        idx += bits[i];
        if (idx >= 64) {
            unaligned_store_u64a(out, write);
            out += 8;
            idx -= 64;
            u32 leftover = bits[i] - idx;
            if (leftover == 64) {
                write = 0;
            } else {
                assert(leftover < 64);
                write = v[i] >> leftover;
            }
        }
    }

    // There might be a write left over.
    DEBUG_PRINTF("partial store of idx=%u\n", idx);
    partial_store_u64a(out, write, (idx + 7) / 8);
}

static really_inline
void unpack_bits_32(u32 *v, const u8 *in, const u32 *bits,
                   const unsigned int elements) {
    u32 used = 0; // bits used from *in

    for (unsigned int i = 0; i < elements; i++) {
        assert(bits[i] <= 32);
        u32 v_out = 0;   // accumulator for v[i]
        u32 b = bits[i]; // bits left to read for v[i]
        u32 vidx = 0;    // bits written to v[i]

        while (b) {
            u32 read = *in >> used;
            u32 bits_read = 8 - used;

            if (b <= bits_read) {
                u32 mask = read & ((1U << b) - 1);
                v_out |= mask << vidx;
                vidx += b;
                used += b;
                b = 0;
                if (used < 8) {
                    continue; // more from this *in
                }
            } else {
                v_out |= read << vidx;
                vidx += bits_read;
                b -= bits_read;
            }

            used = 0;
            in++;
        }

        v[i] = v_out;
    }
}

static really_inline
void unpack_bits_64(u64a *v, const u8 *in, const u32 *bits,
                    const unsigned int elements) {
    u32 used = 0; // bits used from *in

    for (unsigned int i = 0; i < elements; i++) {
        assert(bits[i] <= 64);
        u64a v_out = 0;  // accumulator for v[i]
        u32 b = bits[i]; // bits left to read for v[i]
        u32 vidx = 0;    // bits written to v[i]

        while (b) {
            u64a read = *in >> used;
            u32 bits_read = 8 - used;

            if (b <= bits_read) {
                u64a mask = read & ((1U << b) - 1);
                v_out |= mask << vidx;
                vidx += b;
                used += b;
                b = 0;
                if (used < 8) {
                    continue; // more from this *in
                }
            } else {
                v_out |= read << vidx;
                vidx += bits_read;
                b -= bits_read;
            }

            used = 0;
            in++;
        }

        v[i] = v_out;
    }
}

#endif // UTIL_PACK_BITS_H
