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
 * \brief Acceleration: data structures and common definitions.
 */

#ifndef ACCEL_H
#define ACCEL_H

#include "ue2common.h"

/* run time defs */
#define BAD_ACCEL_DIST      4
#define SMALL_ACCEL_PENALTY 8
#define BIG_ACCEL_PENALTY   32

/// Minimum length of the scan buffer for us to attempt acceleration.
#define ACCEL_MIN_LEN       16

enum AccelType {
    ACCEL_NONE,
    ACCEL_VERM,
    ACCEL_VERM_NOCASE,
    ACCEL_DVERM,
    ACCEL_DVERM_NOCASE,
    ACCEL_RVERM,
    ACCEL_RVERM_NOCASE,
    ACCEL_RDVERM,
    ACCEL_RDVERM_NOCASE,
    ACCEL_REOD,
    ACCEL_REOD_NOCASE,
    ACCEL_RDEOD,
    ACCEL_RDEOD_NOCASE,
    ACCEL_SHUFTI,
    ACCEL_DSHUFTI,
    ACCEL_TRUFFLE,
    ACCEL_RED_TAPE,
    ACCEL_DVERM_MASKED,
};

/** \brief Structure for accel framework. */
union AccelAux {
    u8 accel_type;
    struct {
        u8 accel_type;
        u8 offset;
    } generic;
    struct {
        u8 accel_type;
        u8 offset;
        u8 c; // uppercase if nocase
    } verm;
    struct {
        u8 accel_type;
        u8 offset;
        u8 c1; // uppercase if nocase
        u8 c2; // uppercase if nocase
        u8 m1; // masked variant
        u8 m2; // masked variant
    } dverm;
    struct {
        u8 accel_type;
        u8 offset;
        u8 c; // uppercase if nocase
        u8 len;
    } mverm;
    struct {
        u8 accel_type;
        u8 offset;
        u8 c; // uppercase if nocase
        u8 len1;
        u8 len2;
    } mdverm;
    struct {
        u8 accel_type;
        u8 offset;
        m128 lo;
        m128 hi;
    } shufti;
    struct {
        u8 accel_type;
        u8 offset;
        m128 lo1;
        m128 hi1;
        m128 lo2;
        m128 hi2;
    } dshufti;
    struct {
        u8 accel_type;
        u8 offset;
        m128 mask1;
        m128 mask2;
    } truffle;
};

/**
 * Runs the specified acceleration scheme between c and c_end, returns a point
 * such that the acceleration scheme does not match before.
 */
const u8 *run_accel(const union AccelAux *accel, const u8 *c, const u8 *c_end);

#endif
