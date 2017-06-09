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

/** \file
 * \brief Interface to Parser.
 */

#ifndef _RE_PARSER_H_
#define _RE_PARSER_H_

#include "ue2common.h"

#include <memory>

namespace ue2 {

class Component;

/** \brief Represents the current "mode flags" at any point in the parsing
 * process.
 *
 * This is necessary as some modes can be changed part-way through an
 * expression, such as in:
 *
 *     /foo(?i)bar/
 */
struct ParseMode {
    ParseMode() {}
    explicit ParseMode(u32 hs_flags);

    bool caseless = false;
    bool dotall = false;
    bool ignore_space = false;
    bool multiline = false;
    bool ucp = false;
    bool utf8 = false;
};

/** \brief Parse the given regular expression into a \ref Component tree.
 *
 * The \a mode parameter should contain the initial mode flags, and will be
 * updated by the parser if additional global flags are introduced in the
 * expression (for example, via "(*UTF8)".)
 *
 * This call will throw a ParseError on failure.
 */
std::unique_ptr<Component> parse(const char *ptr, ParseMode &mode);

} // namespace ue2

#endif // _RE_PARSER_H_
