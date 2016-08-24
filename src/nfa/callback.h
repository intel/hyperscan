/*
 * Copyright (c) 2015-2016, Intel Corporation
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
 * \brief NFA Callback definitions, used at runtime.
 */

#ifndef NFA_CALLBACK_H
#define NFA_CALLBACK_H

#include "ue2common.h"

/** \brief The type for an NFA callback.
 *
 * This is a function that takes as arguments the current start and end offsets
 * where the match occurs, the id of the match and the context pointer that was
 * passed into the NFA API function that executed the NFA.
 *
 * The start offset is the "start of match" (SOM) offset for the match. It is
 * only provided by engines that natively support SOM tracking (e.g. Gough).
 *
 * The end offset will be the offset after the character that caused the match.
 * Thus, if we have a buffer containing 'abc', then a pattern that matches an
 * empty string will have an offset of 0, a pattern that matches 'a' will have
 * an offset of 1, and a pattern that matches 'abc' will have an offset of 3,
 * which will be a value that is 'beyond' the size of the buffer. That is, if
 * we have n characters in the buffer, there are n+1 different potential
 * offsets for matches.
 *
 * This function should return an int - currently the possible return values
 * are 0, which means 'stop running the engine' or non-zero, which means
 * 'continue matching'.
 */
typedef int (*NfaCallback)(u64a start, u64a end, ReportID id, void *context);

/**
 * standard \ref NfaCallback return value indicating that engine execution
 * should continue. (any non-zero value will serve this purpose)
 */
#define MO_CONTINUE_MATCHING 1

/**
 * \ref NfaCallback return value indicating that engine execution should halt.
 */
#define MO_HALT_MATCHING 0

#endif // NFA_CALLBACK_H
