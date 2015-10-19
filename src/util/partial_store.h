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

#ifndef PARTIAL_STORE_H
#define PARTIAL_STORE_H

#include "ue2common.h"
#include "unaligned.h"

/* loads/stores the least significant bytes of the values. */

static really_inline
void partial_store_u32(void *ptr, u32 value, u32 numBytes) {
    assert(numBytes <= 4);
    switch (numBytes) {
    case 4:
        unaligned_store_u32(ptr, value);
        break;
    case 3:
        unaligned_store_u16(ptr, (u16)value);
        *((u8 *)ptr + 2) = (u8)(value >> 16);
        break;
    case 2:
        unaligned_store_u16(ptr, (u16)value);
        break;
    case 1:
        *(u8 *)ptr = (u8)value;
        break;
    case 0:
        break;
    }
}

static really_inline
u32 partial_load_u32(const void *ptr, u32 numBytes) {
    u32 value;
    assert(numBytes <= 4);
    switch (numBytes) {
    case 4:
        value = unaligned_load_u32(ptr);
        return value;
    case 3:
        value = unaligned_load_u16(ptr);
        value |= ((u32)(*((const u8 *)ptr + 2)) << 16);
        return value;
    case 2:
        value = unaligned_load_u16(ptr);
        return value;
    case 1:
        value = *(const u8 *)ptr;
        return value;
    case 0:
        break;
    }

    return 0;
}

static really_inline
void partial_store_u64a(void *ptr, u64a value, u32 numBytes) {
    assert(numBytes <= 8);
    switch (numBytes) {
    case 8:
        unaligned_store_u64a(ptr, value);
        break;
    case 7:
        unaligned_store_u32(ptr, (u32)value);
        unaligned_store_u16((u8 *)ptr + 4, (u16)(value >> 32));
        *((u8 *)ptr + 6) = (u8)(value >> 48);
        break;
    case 6:
        unaligned_store_u32(ptr, (u32)value);
        unaligned_store_u16((u8 *)ptr + 4, (u16)(value >> 32));
        break;
    case 5:
        unaligned_store_u32(ptr, (u32)value);
        *((u8 *)ptr + 4) = (u8)(value >> 32);
        break;
    case 4:
        unaligned_store_u32(ptr, (u32)value);
        break;
    case 3:
        unaligned_store_u16(ptr, (u16)value);
        *((u8 *)ptr + 2) = (u8)(value >> 16);
        break;
    case 2:
        unaligned_store_u16(ptr, (u16)value);
        break;
    case 1:
        *(u8 *)ptr = (u8)value;
        break;
    case 0:
        break;
    }
}

static really_inline
u64a partial_load_u64a(const void *ptr, u32 numBytes) {
    u64a value;
    assert(numBytes <= 8);
    switch (numBytes) {
    case 8:
        value = unaligned_load_u64a(ptr);
        return value;
    case 7:
        value = unaligned_load_u32(ptr);
        value |= (u64a)unaligned_load_u16((const u8 *)ptr + 4) << 32;
        value |= (u64a)(*((const u8 *)ptr + 6)) << 48;
        return value;
    case 6:
        value = unaligned_load_u32(ptr);
        value |= (u64a)unaligned_load_u16((const u8 *)ptr + 4) << 32;
        return value;
    case 5:
        value = unaligned_load_u32(ptr);
        value |= (u64a)(*((const u8 *)ptr + 4)) << 32;
        return value;
    case 4:
        value = unaligned_load_u32(ptr);
        return value;
    case 3:
        value = unaligned_load_u16(ptr);
        value |= (u64a)(*((const u8 *)ptr + 2)) << 16;
        return value;
    case 2:
        value = unaligned_load_u16(ptr);
        return value;
    case 1:
        value = *(const u8 *)ptr;
        return value;
    case 0:
        break;
    }

    return 0;
}

#endif
