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

#ifndef UTIL_VERIFY_TYPES
#define UTIL_VERIFY_TYPES

#include "ue2common.h"
#include "util/compile_error.h"

#include <cassert>
#include <type_traits>

namespace ue2 {

template<typename To_T, typename From_T>
To_T verify_cast(From_T val) {
    static_assert(std::is_integral<To_T>::value,
                  "Output type must be integral.");
    static_assert(std::is_integral<From_T>::value ||
                      std::is_enum<From_T>::value ||
                      std::is_convertible<From_T, To_T>::value,
                  "Must be integral or enum type, or convertible to output.");

    To_T conv_val = static_cast<To_T>(val);
    if (static_cast<From_T>(conv_val) != val) {
        assert(0);
        throw ResourceLimitError();
    }

    return conv_val;
}

template<typename T>
s8 verify_s8(T val) {
    return verify_cast<s8>(val);
}

template<typename T>
u8 verify_u8(T val) {
    return verify_cast<u8>(val);
}

template<typename T>
s16 verify_s16(T val) {
    return verify_cast<s16>(val);
}

template<typename T>
u16 verify_u16(T val) {
    return verify_cast<u16>(val);
}

template<typename T>
s32 verify_s32(T val) {
    return verify_cast<s32>(val);
}

template<typename T>
u32 verify_u32(T val) {
    return verify_cast<u32>(val);
}

} // namespace ue2

#endif // UTIL_VERIFY_TYPES
