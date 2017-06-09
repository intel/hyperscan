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
 * \file
 * \brief Parser for control verbs that can occur at the beginning of a pattern.
 */

#include "parser/control_verbs.h"

#include "parser/Parser.h"
#include "parser/parse_error.h"

#include <cstring>
#include <sstream>

using namespace std;

namespace ue2 {

const char *read_control_verbs(const char *ptr, const char *end, size_t start,
                               ParseMode &mode) {
    const char *p = ptr;
    const char *pe = end;
    const char *eof = pe;
    const char *ts, *te;
    int cs;
    UNUSED int act;

    %%{
        machine ControlVerbs;

        # Verbs that we recognise but do not support.
        unhandledVerbs = '(*' (
            'LIMIT_MATCH=' [0-9]+ |
            'LIMIT_RECURSION=' [0-9]+ |
            'NO_AUTO_POSSESS' |
            'NO_START_OPT' |
            'UTF16' |
            'UTF32' |
            'CR' |
            'LF' |
            'CRLF' |
            'ANYCRLF' |
            'ANY' |
            'BSR_ANYCRLF' |
            'BSR_UNICODE'
            ) . ')';

        main := |*
            '(*UTF8)' | '(*UTF)' => {
                mode.utf8 = true;
            };

            '(*UCP)' => {
                mode.ucp = true;
            };

            unhandledVerbs => {
                ostringstream str;
                str << "Unsupported control verb " << string(ts, te - ts);
                throw LocatedParseError(str.str());
            };

            '(*' [^)]+ ')' => {
                ostringstream str;
                str << "Unknown control verb " << string(ts, te - ts);
                throw LocatedParseError(str.str());
            };

            # Anything else means we're done.
            any => {
                fhold;
                fbreak;
            };
        *|;

        write data;
        write init;
    }%%

    try {
        %% write exec;
    } catch (LocatedParseError &error) {
        if (ts >= ptr && ts <= pe) {
            error.locate(ts - ptr + start);
        } else {
            error.locate(0);
        }
        throw;
    }

    return p;
}

} // namespace ue2
