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

#ifndef ACCEL_COMPILE_H
#define ACCEL_COMPILE_H

#include "ue2common.h"
#include "util/charreach.h"
#include "util/flat_containers.h"

union AccelAux;

namespace ue2 {

struct AccelInfo {
    AccelInfo() : single_offset(0U), double_offset(0U),
                  single_stops(CharReach::dot()) {}
    u32 single_offset; /**< offset correction to apply to single schemes */
    u32 double_offset; /**< offset correction to apply to double schemes */
    CharReach double_stop1;  /**<  single-byte accel stop literals for double
                            * schemes */
    flat_set<std::pair<u8, u8>> double_stop2; /**< double-byte accel stop
                                               * literals */
    CharReach single_stops; /**< escapes for single byte acceleration */
};

bool buildAccelAux(const AccelInfo &info, AccelAux *aux);

/* returns true is the escape set can be handled with a masked double_verm */
bool buildDvermMask(const flat_set<std::pair<u8, u8>> &escape_set,
                    u8 *m1_out = nullptr, u8 *m2_out = nullptr);

} // namespace ue2

#endif
