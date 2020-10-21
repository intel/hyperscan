/*
 * Copyright (c) 2016-2020, Intel Corporation
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

#include "mcsheng_internal.h"

/* This table is in a separate translation unit from mcsheng.c as we want to
 * prevent the compiler from seeing these constants. We have the load resources
 * free at runtime to load the masks with no problems. */
const u64a mcsheng_pext_mask[8] = {
    0, /* dummy */
    0x000000000000ff0f,
    0x0000000000ff000f,
    0x00000000ff00000f,
    0x000000ff0000000f,
    0x0000ff000000000f,
    0x00ff00000000000f,
    0xff0000000000000f,
};
#if defined(HAVE_AVX512VBMI)
const u64a mcsheng64_pext_mask[8] = {
    0, /* dummy */
    0x000000000000ff3f,
    0x0000000000ff003f,
    0x00000000ff00003f,
    0x000000ff0000003f,
    0x0000ff000000003f,
    0x00ff00000000003f,
    0xff0000000000003f,
};
#endif
