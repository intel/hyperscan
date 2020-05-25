/*
 * Copyright (c) 2015-2020, Intel Corporation
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

#ifndef STRING_UTIL_H
#define STRING_UTIL_H

#include "ue2common.h"
#include <iomanip>
#include <string>
#include <sstream>
#include <vector>
#include <cstring>
#include <iostream>

//
// Utility functions
//

// read a string in and convert it to another type, anything supported
// by stringstream
template<typename T>
inline bool fromString(const std::string &s, T& val)
{
    std::istringstream i(s);
    char c;
    if (!(i >> val) || i.get(c)) {
        return false;
    }
    return true;
}

// read in a comma-separated or hyphen-connected set of values: very simple
// impl, not for external consumption
template<typename T>
inline bool strToList(const std::string &s, std::vector<T>& out)
{
    std::istringstream i(s);
    char c;
    do {
        T val;
        if (!(i >> val)) {
            break;
        }

        out.push_back(val);

        i.get(c);
        if (c == '-') {
            T val_end;
            i >> val_end;
            while (val < val_end) {
                out.push_back(++val);
            }
            break;
        }
    } while (c == ',');

    return !out.empty();
}

// return a nicely escaped version of a string: this should probably become
// an IO manipulator or something
UNUSED static
const std::string printable(const std::string &in) {
    std::ostringstream oss;
    for (size_t i = 0; i < in.size(); ++i) {
        unsigned char c = in[i];
        if (c == '\"') {
            oss << "\\\"";
        } else if (c == '\n') {
            oss << "\\n";
        } else if (c == '\t') {
            oss << "\\t";
        } else if (c == '\r') {
            oss << "\\r";
        } else if (0x20 <= c && c <= 0x7e && c != '\\') {
            oss << c;
        } else {
            oss << "\\x"
                << std::hex << std::setw(2) << std::setfill('0')
                << (unsigned)(in[i] & 0xff)
                << std::dec;
        }
    }
    return oss.str();
}

template<typename it_t>
void prettyPrintRange(std::ostream &out, it_t begin, it_t end) {
    bool in_range = false;
    it_t it = begin;
    it_t itp = it;

    for (; it != end; itp = it++) {
        if (it != begin && *it == *itp + 1) {
            in_range = true;
            continue;
        } else if (it != begin) {
            if (in_range) {
                out << "-" << *itp;
            }

            out << ", ";
            in_range = false;
        }

        out << *it;
    }

    if (in_range) {
        out << "-" << *itp;
    }
}

// Transfer given string into a hex-escaped pattern.
static really_inline
char *makeHex(const unsigned char *pat, unsigned patlen) {
    size_t hexlen = patlen * 4;
    char *hexbuf = (char *)malloc(hexlen + 1);
    unsigned i;
    char *buf;
    for (i = 0, buf = hexbuf; i < patlen; i++, buf += 4) {
        snprintf(buf, 5, "\\x%02x", (unsigned char)pat[i]);
    }
    hexbuf[hexlen] = '\0';
    return hexbuf;
}

#endif // STRING_UTIL_H
