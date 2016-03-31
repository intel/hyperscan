/*
 * Copyright (c) 2016, Intel Corporation
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
 * \brief Teddy literal matcher: AVX2 engine runtime.
 */

#include "fdr_internal.h"
#include "flood_runtime.h"
#include "teddy.h"
#include "teddy_internal.h"
#include "teddy_runtime_common.h"
#include "util/simd_utils.h"
#include "util/simd_utils_ssse3.h"

#if defined(__AVX2__)

static const u8 ALIGN_AVX_DIRECTIVE p_mask_arr256[33][64] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}
};

#ifdef ARCH_64_BIT
#define CONFIRM_FAT_TEDDY(var, bucket, offset, reason, conf_fn)             \
do {                                                                        \
    if (unlikely(isnonzero256(var))) {                                      \
        m256 swap = swap128in256(var);                                      \
        m256 r = interleave256lo(var, swap);                                \
        u64a part1 = extractlow64from256(r);                                \
        u64a part2 = extract64from256(r, 1);                                \
        r = interleave256hi(var, swap);                                     \
        u64a part3 = extractlow64from256(r);                                \
        u64a part4 = extract64from256(r, 1);                                \
        if (unlikely(part1)) {                                              \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2)) {                                              \
            conf_fn(&part2, bucket, offset + 4, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part3)) {                                              \
            conf_fn(&part3, bucket, offset + 8, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4)) {                                              \
            conf_fn(&part4, bucket, offset + 12, confBase, reason, a, ptr,  \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);
#else
#define CONFIRM_FAT_TEDDY(var, bucket, offset, reason, conf_fn)             \
do {                                                                        \
    if (unlikely(isnonzero256(var))) {                                      \
        m256 swap = swap128in256(var);                                      \
        m256 r = interleave256lo(var, swap);                                \
        u32 part1 = extractlow32from256(r);                                 \
        u32 part2 = extract32from256(r, 1);                                 \
        u32 part3 = extract32from256(r, 2);                                 \
        u32 part4 = extract32from256(r, 3);                                 \
        r = interleave256hi(var, swap);                                     \
        u32 part5 = extractlow32from256(r);                                 \
        u32 part6 = extract32from256(r, 1);                                 \
        u32 part7 = extract32from256(r, 2);                                 \
        u32 part8 = extract32from256(r, 3);                                 \
        if (unlikely(part1)) {                                              \
            conf_fn(&part1, bucket, offset, confBase, reason, a, ptr,       \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part2)) {                                              \
            conf_fn(&part2, bucket, offset + 2, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
        }                                                                   \
        if (unlikely(part3)) {                                              \
            conf_fn(&part3, bucket, offset + 4, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part4)) {                                              \
            conf_fn(&part4, bucket, offset + 6, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part5)) {                                              \
            conf_fn(&part5, bucket, offset + 8, confBase, reason, a, ptr,   \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part6)) {                                              \
            conf_fn(&part6, bucket, offset + 10, confBase, reason, a, ptr,  \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part7)) {                                              \
            conf_fn(&part7, bucket, offset + 12, confBase, reason, a, ptr,  \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
        if (unlikely(part8)) {                                              \
            conf_fn(&part8, bucket, offset + 14, confBase, reason, a, ptr,  \
                    control, &last_match);                                  \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);
#endif

#define CONFIRM_FAST_TEDDY(var, offset, reason, conf_fn)                    \
do {                                                                        \
    if (unlikely(isnonzero256(var))) {                                      \
        u32 arrCnt = 0;                                                     \
        m128 lo = cast256to128(var);                                        \
        m128 hi = cast256to128(swap128in256(var));                          \
        bit_array_fast_teddy(lo, bitArr, &arrCnt, offset);                  \
        bit_array_fast_teddy(hi, bitArr, &arrCnt, offset + 2);              \
        for (u32 i = 0; i < arrCnt; i++) {                                  \
            conf_fn(bitArr[i], confBase, reason, a, ptr, control,           \
                    &last_match);                                           \
            CHECK_HWLM_TERMINATE_MATCHING;                                  \
        }                                                                   \
    }                                                                       \
} while (0);

static really_inline
m256 vectoredLoad2x128(m256 *p_mask, const u8 *ptr, const u8 *lo, const u8 *hi,
                       const u8 *buf_history, size_t len_history,
                       const u32 nMasks) {
    m128 p_mask128;
    m256 ret = set2x128(vectoredLoad128(&p_mask128, ptr, lo, hi, buf_history,
                                        len_history, nMasks));
    *p_mask = set2x128(p_mask128);
    return ret;
}

/*
 * \brief Copy a block of [0,31] bytes efficiently.
 *
 * This function is a workaround intended to stop some compilers from
 * synthesizing a memcpy function call out of the copy of a small number of
 * bytes that we do in vectoredLoad128.
 */
static really_inline
void copyRuntBlock256(u8 *dst, const u8 *src, size_t len) {
    switch (len) {
    case 0:
        break;
    case 1:
        *dst = *src;
        break;
    case 2:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        break;
    case 3:
        unaligned_store_u16(dst, unaligned_load_u16(src));
        dst[2] = src[2];
        break;
    case 4:
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 5:
    case 6:
    case 7:
        /* Perform copy with two overlapping 4-byte chunks. */
        unaligned_store_u32(dst + len - 4, unaligned_load_u32(src + len - 4));
        unaligned_store_u32(dst, unaligned_load_u32(src));
        break;
    case 8:
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    case 9:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
        /* Perform copy with two overlapping 8-byte chunks. */
        unaligned_store_u64a(dst + len - 8, unaligned_load_u64a(src + len - 8));
        unaligned_store_u64a(dst, unaligned_load_u64a(src));
        break;
    case 16:
        storeu128(dst, loadu128(src));
        break;
    default:
        /* Perform copy with two overlapping 16-byte chunks. */
        assert(len < 32);
        storeu128(dst + len - 16, loadu128(src + len - 16));
        storeu128(dst, loadu128(src));
        break;
    }
}

static really_inline
m256 vectoredLoad256(m256 *p_mask, const u8 *ptr, const u8 *lo, const u8 *hi,
                     const u8 *buf_history, size_t len_history) {
    union {
        u8 val8[32];
        m256 val256;
    } u;

    uintptr_t copy_start;
    uintptr_t copy_len;

    if (ptr >= lo) {
        uintptr_t avail = (uintptr_t)(hi - ptr);
        if (avail >= 32) {
            *p_mask = load256(p_mask_arr256[32] + 32);
            return loadu256(ptr);
        }
        *p_mask = load256(p_mask_arr256[avail] + 32);
        copy_start = 0;
        copy_len = avail;
    } else {
        // need contains "how many chars to pull from history"
        // calculate based on what we need, what we have in the buffer
        // and only what we need to make primary confirm work
        uintptr_t start = (uintptr_t)(lo - ptr);
        uintptr_t i;
        for (i = start; ptr + i < lo; i++) {
            u.val8[i] = buf_history[len_history - (lo - (ptr + i))];
        }
        uintptr_t end = MIN(32, (uintptr_t)(hi - ptr));
        *p_mask = loadu256(p_mask_arr256[end - start] + 32 - start);
        copy_start = i;
        copy_len = end - i;
    }

    // Runt block from the buffer.
    copyRuntBlock256(&u.val8[copy_start], &ptr[copy_start], copy_len);

    return u.val256;
}

static really_inline
void do_confWithBit1_fast_teddy(u16 bits, const u32 *confBase,
                                CautionReason reason,
                                const struct FDR_Runtime_Args *a,
                                const u8 *ptr, hwlmcb_rv_t *control,
                                u32 *last_match) {
    u32 byte = bits / 8;
    u32 cf = confBase[bits % 8];
    const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                    ((const u8 *)confBase + cf);
    u64a confVal = getConfVal(a, ptr, byte, reason);
    confWithBit1(fdrc, a, ptr - a->buf + byte, control, last_match, confVal);
}

static really_inline
void do_confWithBit_fast_teddy(u16 bits, const u32 *confBase,
                               CautionReason reason,
                               const struct FDR_Runtime_Args *a, const u8 *ptr,
                               hwlmcb_rv_t *control, u32 *last_match) {
    u32 byte = bits / 8;
    u32 bitRem = bits % 8;
    u32 confSplit = *(ptr+byte) & 0x1f;
    u32 idx = confSplit * 8 + bitRem;
    u32 cf = confBase[idx];
    if (!cf) {
        return;
    }
    const struct FDRConfirm *fdrc = (const struct FDRConfirm *)
                                    ((const u8 *)confBase + cf);
    if (!(fdrc->groups & *control)) {
        return;
    }
    u64a confVal = getConfVal(a, ptr, byte, reason);
    confWithBit(fdrc, a, ptr - a->buf + byte, 0, control, last_match, confVal);
}

static really_inline
void bit_array_fast_teddy(m128 var, u16 *bitArr, u32 *arrCnt, u32 offset) {
    if (unlikely(isnonzero128(var))) {
#ifdef ARCH_64_BIT
        u64a part_0 = movq(var);
        while (unlikely(part_0)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_0) +
                                    64 * (offset);
            *arrCnt += 1;
        }
        u64a part_1 = movq(byteShiftRight128(var, 8));
        while (unlikely(part_1)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_1) +
                                    64 * (offset + 1);
            *arrCnt += 1;
        }
#else
        u32 part_0 = movd(var);
        while (unlikely(part_0)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_0) +
                                    32 * (offset * 2);
            *arrCnt += 1;
        }
        u32 part_1 = movd(byteShiftRight128(var, 4));
        while (unlikely(part_1)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_1) +
                                    32 * (offset * 2 + 1);
            *arrCnt += 1;
        }
        u32 part_2 = movd(byteShiftRight128(var, 8));
        while (unlikely(part_2)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_2) +
                                    32 * (offset * 2 + 2);
            *arrCnt += 1;
        }
        u32 part_3 = movd(byteShiftRight128(var, 12));
        while (unlikely(part_3)) {
            bitArr[*arrCnt] = (u16) TEDDY_FIND_AND_CLEAR_LSB(&part_3) +
                                    32 * (offset * 2 + 3);
            *arrCnt += 1;
        }
#endif
    }
}

static really_inline
m256 prep_conf_fat_teddy_m1(const m256 *maskBase, m256 p_mask, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift4x64(val, 4), mask);
    return and256(and256(vpshufb(maskBase[0*2], lo),
                         vpshufb(maskBase[0*2+1], hi)), p_mask);
}

static really_inline
m256 prep_conf_fat_teddy_m2(const m256 *maskBase, m256 *old_1, m256 p_mask,
                            m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift4x64(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m1(maskBase, p_mask, val);

    m256 res_1 = and256(vpshufb(maskBase[1*2], lo),
                        vpshufb(maskBase[1*2+1], hi));
    m256 res_shifted_1 = vpalignr(res_1, *old_1, 16-1);
    *old_1 = res_1;
    return and256(and256(r, p_mask), res_shifted_1);
}

static really_inline
m256 prep_conf_fat_teddy_m3(const m256 *maskBase, m256 *old_1, m256 *old_2,
                            m256 p_mask, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift4x64(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m2(maskBase, old_1, p_mask, val);

    m256 res_2 = and256(vpshufb(maskBase[2*2], lo),
                        vpshufb(maskBase[2*2+1], hi));
    m256 res_shifted_2 = vpalignr(res_2, *old_2, 16-2);
    *old_2 = res_2;
    return and256(r, res_shifted_2);
}

static really_inline
m256 prep_conf_fat_teddy_m4(const m256 *maskBase, m256 *old_1, m256 *old_2,
                            m256 *old_3, m256 p_mask, m256 val) {
    m256 mask = set32x8(0xf);
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift4x64(val, 4), mask);
    m256 r = prep_conf_fat_teddy_m3(maskBase, old_1, old_2, p_mask, val);

    m256 res_3 = and256(vpshufb(maskBase[3*2], lo),
                        vpshufb(maskBase[3*2+1], hi));
    m256 res_shifted_3 = vpalignr(res_3, *old_3, 16-3);
    *old_3 = res_3;
    return and256(r, res_shifted_3);
}

static really_inline
m256 prep_conf_fast_teddy_m1(m256 val, m256 mask, m256 maskLo, m256 maskHi,
                             m256 p_mask) {
    m256 lo = and256(val, mask);
    m256 hi = and256(rshift4x64(val, 4), mask);
    m256 res = and256(vpshufb(maskLo, lo), vpshufb(maskHi, hi));
    return and256(res, p_mask);
}

static really_inline
const m256 * getMaskBase_avx2(const struct Teddy *teddy) {
    return (const m256 *)((const u8 *)teddy + sizeof(struct Teddy));
}

static really_inline
const u32 * getConfBase_avx2(const struct Teddy *teddy, u8 numMask) {
    return (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) +
                         (numMask*32*2));
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit1_teddy);
        m256 r_1 = prep_conf_fat_teddy_m1(maskBase, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit1_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit1_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 1);

    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m1(maskBase, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 1);
        m256 r_0 = prep_conf_fat_teddy_m1(maskBase, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks2_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 2);

    m256 res_old_1 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks2_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 2);

    m256 res_old_1 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
         ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 2);
        m256 r_0 = prep_conf_fat_teddy_m2(maskBase, &res_old_1, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks3_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 3);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks3_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 3);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          ones256(), load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 3);
        m256 r_0 = prep_conf_fat_teddy_m3(maskBase, &res_old_1, &res_old_2,
                                          p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks4_fat(const struct FDR *fdr,
                                           const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 4);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    m256 res_old_3 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBitMany_teddy);
        m256 r_1 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBitMany_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBitMany_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks4_pck_fat(const struct FDR *fdr,
                                               const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 32;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m256 *maskBase = getMaskBase_avx2(teddy);
    const u32 *confBase = getConfBase_avx2(teddy, 4);

    m256 res_old_1 = ones256();
    m256 res_old_2 = ones256();
    m256 res_old_3 = ones256();
    const u8 *mainStart = ROUNDUP_PTR(ptr, 16);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 16;
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    if (ptr + 16 < buf_end) {
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
        ptr += 16;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr));
        CONFIRM_FAT_TEDDY(r_0, 16, 0, NOT_CAUTIOUS, do_confWithBit_teddy);
        m256 r_1 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, ones256(),
                                          load2x128(ptr + 16));
        CONFIRM_FAT_TEDDY(r_1, 16, 16, NOT_CAUTIOUS, do_confWithBit_teddy);
    }

    for (; ptr < buf_end; ptr += 16) {
        m256 p_mask;
        m256 val_0 = vectoredLoad2x128(&p_mask, ptr, a->buf, buf_end,
                                       a->buf_history, a->len_history, 4);
        m256 r_0 = prep_conf_fat_teddy_m4(maskBase, &res_old_1, &res_old_2,
                                          &res_old_3, p_mask, val_0);
        CONFIRM_FAT_TEDDY(r_0, 16, 0, VECTORING, do_confWithBit_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_fast(const struct FDR *fdr,
                                            const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 64;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 1);

    const m256 maskLo = set2x128(maskBase[0]);
    const m256 maskHi = set2x128(maskBase[1]);
    const m256 mask = set32x8(0xf);
    u16 bitArr[512];

    const u8 *mainStart = ROUNDUP_PTR(ptr, 32);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 32;
        m256 p_mask;
        m256 val_0 = vectoredLoad256(&p_mask, ptr, a->buf + a->start_offset,
                                     buf_end, a->buf_history, a->len_history);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             p_mask);
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit1_fast_teddy);
        ptr += 32;
    }

    if (ptr + 32 < buf_end) {
        m256 val_0 = load256(ptr + 0);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit1_fast_teddy);
        ptr += 32;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;

        m256 val_0 = load256(ptr + 0);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_0, 0, NOT_CAUTIOUS, do_confWithBit1_fast_teddy);

        m256 val_1 = load256(ptr + 32);
        m256 res_1 = prep_conf_fast_teddy_m1(val_1, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_1, 4, NOT_CAUTIOUS, do_confWithBit1_fast_teddy);
    }

    for (; ptr < buf_end; ptr += 32) {
        m256 p_mask;
        m256 val_0 = vectoredLoad256(&p_mask, ptr, a->buf + a->start_offset,
                                     buf_end, a->buf_history, a->len_history);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             p_mask);
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit1_fast_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

hwlm_error_t fdr_exec_teddy_avx2_msks1_pck_fast(const struct FDR *fdr,
                                                const struct FDR_Runtime_Args *a) {
    const u8 *buf_end = a->buf + a->len;
    const u8 *ptr = a->buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t *control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 *tryFloodDetect = a->firstFloodDetect;
    u32 last_match = (u32)-1;
    const struct Teddy *teddy = (const struct Teddy *)fdr;
    const size_t iterBytes = 64;
    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\n",
                 a->buf, a->len, a->start_offset);

    const m128 *maskBase = getMaskBase(teddy);
    const u32 *confBase = getConfBase(teddy, 1);

    const m256 maskLo = set2x128(maskBase[0]);
    const m256 maskHi = set2x128(maskBase[1]);
    const m256 mask = set32x8(0xf);
    u16 bitArr[512];

    const u8 *mainStart = ROUNDUP_PTR(ptr, 32);
    DEBUG_PRINTF("derive: ptr: %p mainstart %p\n", ptr, mainStart);
    if (ptr < mainStart) {
        ptr = mainStart - 32;
        m256 p_mask;
        m256 val_0 = vectoredLoad256(&p_mask, ptr, a->buf + a->start_offset,
                                     buf_end, a->buf_history, a->len_history);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             p_mask);
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit_fast_teddy);
        ptr += 32;
    }

    if (ptr + 32 < buf_end) {
        m256 val_0 = load256(ptr + 0);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit_fast_teddy);
        ptr += 32;
    }

    for ( ; ptr + iterBytes <= buf_end; ptr += iterBytes) {
        __builtin_prefetch(ptr + (iterBytes*4));
        CHECK_FLOOD;

        m256 val_0 = load256(ptr + 0);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_0, 0, NOT_CAUTIOUS, do_confWithBit_fast_teddy);

        m256 val_1 = load256(ptr + 32);
        m256 res_1 = prep_conf_fast_teddy_m1(val_1, mask, maskLo, maskHi,
                                             ones256());
        CONFIRM_FAST_TEDDY(res_1, 4, NOT_CAUTIOUS, do_confWithBit_fast_teddy);
    }

    for (; ptr < buf_end; ptr += 32) {
        m256 p_mask;
        m256 val_0 = vectoredLoad256(&p_mask, ptr, a->buf + a->start_offset,
                                     buf_end, a->buf_history, a->len_history);
        m256 res_0 = prep_conf_fast_teddy_m1(val_0, mask, maskLo, maskHi,
                                             p_mask);
        CONFIRM_FAST_TEDDY(res_0, 0, VECTORING, do_confWithBit_fast_teddy);
    }
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}

#endif // __AVX2__
