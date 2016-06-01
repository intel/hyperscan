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

#ifndef MULTIACCELCOMPILE_H_
#define MULTIACCELCOMPILE_H_

#include "ue2common.h"

#include "nfagraph/ng_limex_accel.h"

#include <vector>

namespace ue2 {

/* accel scheme state machine */
enum accel_scheme_state {
    STATE_FIRST_RUN,
    STATE_SECOND_RUN,
    STATE_WAITING_FOR_GRAB,
    STATE_FIRST_TAIL,
    STATE_SECOND_TAIL,
    STATE_STOPPED,
    STATE_INVALID
};

struct accel_data {
    MultibyteAccelInfo::multiaccel_type type = MultibyteAccelInfo::MAT_NONE;
    accel_scheme_state state = STATE_INVALID;
    unsigned len1 = 0; /* length of first run */
    unsigned len2 = 0; /* length of second run, if present */
    unsigned tlen1 = 0; /* first tail length */
    unsigned tlen2 = 0; /* second tail length */
};

class MultiaccelCompileHelper {
private:
    const CharReach &cr;
    u32 offset;
    std::vector<accel_data> accels;
    unsigned max_len;
public:
    MultiaccelCompileHelper(const CharReach &cr, u32 off, unsigned max_len);
    bool canAdvance();
    MultibyteAccelInfo getBestScheme();
    void advance(const ue2::CharReach &cr);
};

}; // namespace

#endif /* MULTIACCELCOMPILE_H_ */
