/*
 * Copyright (c) 2017, Intel Corporation
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
 * \brief Ordered operators: provides all the other compare operators for types
 * that provide equal and less-than.
 *
 * This is similar to Boost's totally_ordered class, but much simpler and
 * without injecting the boost namespace into ADL lookup.
 */

#ifndef UTIL_OPERATORS_H
#define UTIL_OPERATORS_H

namespace ue2 {

/**
 * \brief Ordered operators: provides all the other compare operators for types
 * that provide equal and less-than.
 *
 * Simply inherit from this class with your class name as its template
 * parameter.
 */
template<typename T>
class totally_ordered {
public:
    friend bool operator!=(const T &a, const T &b) { return !(a == b); }
    friend bool operator<=(const T &a, const T &b) { return !(b < a); }
    friend bool operator>(const T &a, const T &b) { return b < a; }
    friend bool operator>=(const T &a, const T &b) { return !(a < b); }
};

} // namespace

#endif // UTIL_OPERATORS_H
