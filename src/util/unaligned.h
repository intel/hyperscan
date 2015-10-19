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
 * \brief Helper functions for unaligned loads and stores.
 */

#ifndef UNALIGNED_H
#define UNALIGNED_H

#include "ue2common.h"

#if !defined(_WIN32)
#define PACKED__MAY_ALIAS __attribute__((packed, may_alias))
#else
#define PACKED__MAY_ALIAS
#pragma pack(push, 1) // pack everything until told otherwise
#endif

/// Perform an unaligned 16-bit load
static really_inline
u16 unaligned_load_u16(const void *ptr) {
    struct unaligned { u16 u; } PACKED__MAY_ALIAS;
    const struct unaligned *uptr = (const struct unaligned *)ptr;
    return uptr->u;
}

/// Perform an unaligned 32-bit load
static really_inline
u32 unaligned_load_u32(const void *ptr) {
    struct unaligned { u32 u; } PACKED__MAY_ALIAS;
    const struct unaligned *uptr = (const struct unaligned *)ptr;
    return uptr->u;
}

/// Perform an unaligned 64-bit load
static really_inline
u64a unaligned_load_u64a(const void *ptr) {
    struct unaligned { u64a u; } PACKED__MAY_ALIAS;
    const struct unaligned *uptr = (const struct unaligned *)ptr;
    return uptr->u;
}

/// Perform an unaligned 16-bit store
static really_inline
void unaligned_store_u16(void *ptr, u16 val) {
    struct unaligned { u16 u; } PACKED__MAY_ALIAS;
    struct unaligned *uptr = (struct unaligned *)ptr;
    uptr->u = val;
}

/// Perform an unaligned 32-bit store
static really_inline
void unaligned_store_u32(void *ptr, u32 val) {
    struct unaligned { u32 u; } PACKED__MAY_ALIAS;
    struct unaligned *uptr = (struct unaligned *)ptr;
    uptr->u = val;
}

/// Perform an unaligned 64-bit store
static really_inline
void unaligned_store_u64a(void *ptr, u64a val) {
    struct unaligned { u64a u; } PACKED__MAY_ALIAS;
    struct unaligned *uptr = (struct unaligned *)ptr;
    uptr->u = val;
}
#if defined(_WIN32)
#pragma pack(pop)
#endif // win32

#undef PACKED__MAY_ALIAS

#endif // UNALIGNED_H
