/*
 * Copyright (c) 2018, Intel Corporation
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
 * \brief Inline functions for manipulating logical combinations.
 */

#ifndef LOGICAL_H
#define LOGICAL_H

#include "ue2common.h"

/** Index meaning a given logical key is invalid. */
#define INVALID_LKEY    (~(u32)0)
#define INVALID_CKEY    INVALID_LKEY

/** Logical operation type, the priority is from high to low. */
enum LogicalOpType {
    LOGICAL_OP_NOT,
    LOGICAL_OP_AND,
    LOGICAL_OP_OR,
    LAST_LOGICAL_OP =  LOGICAL_OP_OR //!< Sentinel.
};

#define UNKNOWN_OP      (~(u32)0)

/** Logical Operation is consist of 4 parts. */
struct LogicalOp {
    u32 id; //!< logical operator/operation id
    u32 op; //!< LogicalOpType
    u32 lo; //!< left operand
    u32 ro; //!< right operand
};

/** Each logical combination has its info:
 * It occupies a region in LogicalOp vector.
 * It has an exhaustion key for single-match mode. */
struct CombInfo {
    u32 id;
    u32 ekey; //!< exhaustion key
    u32 start; //!< ckey of logical operation to start calculating
    u32 result; //!< ckey of logical operation to give final result
    u64a min_offset;
    u64a max_offset;
};

/** Temporarily use to seperate operations' id from reports' lkey
  * when building logicalTree in shunting yard algorithm,
  * operations' id will be finally renumbered following reports' lkey. */
#define LOGICAL_OP_BIT 0x80000000UL

#endif
