/*
 * Copyright (c) 2016-2026, Intel Corporation
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

/**
 * \file
 * \brief Centralized HMAC key for database integrity verification.
 *
 * Override at build time with -DHS_DB_HMAC_KEY_OVERRIDE to supply a
 * site-specific 32-byte key for additional hardening.
 */

#ifndef HS_DB_HMAC_KEY_H
#define HS_DB_HMAC_KEY_H

#include "ue2common.h"

#ifndef HS_DB_HMAC_KEY_OVERRIDE
static const u8 HS_DB_HMAC_KEY[32] = {
    0xD7, 0xC4, 0xA1, 0xB3, 0x9E, 0x2F, 0x5D, 0x8C,
    0x6B, 0x4A, 0x71, 0xF0, 0x3E, 0x82, 0xC5, 0x19,
    0xA8, 0x57, 0xD4, 0x6E, 0x0B, 0x93, 0xFA, 0x2C,
    0x7D, 0x16, 0xE9, 0x45, 0xB8, 0x60, 0x3F, 0xCE
};
#else
static const u8 HS_DB_HMAC_KEY[32] = HS_DB_HMAC_KEY_OVERRIDE;
#endif

#endif /* HS_DB_HMAC_KEY_H */
