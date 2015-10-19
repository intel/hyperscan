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
 * \brief Data types used to represent depth quantities.
 */
#include "depth.h"
#include "ue2common.h"

#include <algorithm> // std::min, std::max

namespace ue2 {

DepthMinMax unionDepthMinMax(const DepthMinMax &a, const DepthMinMax &b) {
    DepthMinMax rv;

    if (a.min.is_unreachable()) {
        rv.min = b.min;
    } else if (b.min.is_unreachable()) {
        rv.min = a.min;
    } else {
        rv.min = std::min(a.min, b.min);
    }

    if (a.max.is_infinite() || b.max.is_infinite()) {
        rv.max = depth::infinity();
    } else if (a.max.is_unreachable()) {
        rv.max = b.max;
    } else if (b.max.is_unreachable()) {
        rv.max = a.max;
    } else {
        rv.max = std::max(a.max, b.max);
    }

    return rv;
}

} // namespace ue2

#ifdef DUMP_SUPPORT

#include <sstream>
#include <string>

namespace ue2 {

std::string depth::str() const {
    if (is_unreachable()) {
        return "unr";
    } else if (is_infinite()) {
        return "inf";
    }
    std::ostringstream oss;
    oss << val;
    return oss.str();
}

std::string DepthMinMax::str() const {
    std::ostringstream oss;
    oss << "[" << min.str() << "," << max.str() << "]";
    return oss.str();
}

} // namespace ue2

#endif // DUMP_SUPPORT
