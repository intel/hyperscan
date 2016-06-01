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

/**
 * \file
 * \brief SOM runtime: data structures.
 *
 * Data structures used for SOM operations.
 */

#ifndef SOM_OPERATION_H
#define SOM_OPERATION_H

#include "ue2common.h"

#define SOM_EXTERNAL_CALLBACK_REL                     1
#define SOM_INTERNAL_LOC_SET                          2
#define SOM_INTERNAL_LOC_SET_IF_UNSET                 3
#define SOM_INTERNAL_LOC_SET_IF_WRITABLE              4
#define SOM_INTERNAL_LOC_SET_REV_NFA                  5
#define SOM_INTERNAL_LOC_SET_REV_NFA_IF_UNSET         6
#define SOM_INTERNAL_LOC_SET_REV_NFA_IF_WRITABLE      7
#define SOM_INTERNAL_LOC_COPY                         8
#define SOM_INTERNAL_LOC_COPY_IF_WRITABLE             9
#define SOM_INTERNAL_LOC_MAKE_WRITABLE               10
#define SOM_EXTERNAL_CALLBACK_STORED                 11
#define SOM_EXTERNAL_CALLBACK_ABS                    12
#define SOM_EXTERNAL_CALLBACK_REV_NFA                13
#define SOM_INTERNAL_LOC_SET_FROM                    14
#define SOM_INTERNAL_LOC_SET_FROM_IF_WRITABLE        15

struct som_operation {
    /** \brief Report type, from the definitions above. */
    u8 type;

    /* \brief SOM loc to modify. */
    u32 onmatch;

    union {
        /** \brief SOM distance value, use varies according to type.
         *
         *  - for SOM_EXTERNAL_CALLBACK_REL, from-offset is this many bytes
         *    before the to-offset.
         *  - for SOM_EXTERNAL_CALLBACK_ABS, set from-offset to this value.
         *  - for SOM_INTERNAL_LOC_COPY*, som location read_from.
         */
        u64a somDistance;

        /** \brief Index of the reverse nfa.
         *
         * Used by SOM_EXTERNAL_CALLBACK_REV_NFA and
         * SOM_INTERNAL_LOC_SET_REV_NFA*.
         */
        u64a revNfaIndex;
    } aux;
};

#endif // SOM_OPERATION_H

