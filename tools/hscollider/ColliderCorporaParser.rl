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

#include "config.h"

#include "ColliderCorporaParser.h"
#include "Corpora.h"

#include "ue2common.h"

#include <cassert>
#include <cstdlib>
#include <string>
#include <cstdio>

using namespace std;

namespace /* anonymous */ {

// Take a string like '\xFF' and convert it to the character it represents
char unhex(const char *start, UNUSED const char *end) {
    assert(start + 4 == end);
    assert(start[0] == '\\');
    assert(start[1] == 'x');
    assert(isxdigit(start[2]));
    assert(isxdigit(start[2]));

    char temp[3] = {start[2], start[3], 0};

    return strtol(temp, nullptr, 16);
}

%%{
    machine FileCorporaParser;

    action accumulateNum {
        num = (num * 10) + (fc - '0');
    }

    action handleHexEscaped {
        sout.push_back(unhex(ts, te));
    }

    action handleSpecial {
        switch (*(ts+1)) {
            case '0': sout.push_back('\x00'); break;
            case 'a': sout.push_back('\x07'); break;
            case 'e': sout.push_back('\x1b'); break;
            case 'f': sout.push_back('\x0c'); break;
            case 'n': sout.push_back('\x0a'); break;
            case 'v': sout.push_back('\x0b'); break;
            case 'r': sout.push_back('\x0d'); break;
            case 't': sout.push_back('\x09'); break;
            default: fbreak;
        }
    }

    action handleMatch {
        c.matches.insert(num);
    }

    write data;
}%%

} // namespace

bool parseCorpus(const string &line, Corpus &c, unsigned int &id) {
    const char *p = line.c_str();
    const char *pe = p + line.size();
    const char *eof = pe;
    const char *ts;
    const char *te;
    int cs;
    UNUSED int act;

    // For storing integers as they're scanned
    unsigned int num = 0;

    string &sout = c.data;

    %%{
        id = ( digit @accumulateNum)+ >{num = 0;} @{id = num;};

        backslashed = '\\' ^alnum;
        specials = '\\' [0aefnvrt];
        hexescaped = '\\x' xdigit{2};

        corpus_old := |*
            hexescaped => handleHexEscaped;
            specials => handleSpecial;
            backslashed => { sout.push_back(*(ts + 1)); };
            any => { sout.push_back(*ts); };
        *|;

        corpus_new := |*
            hexescaped => handleHexEscaped;
            specials => handleSpecial;
            backslashed => { sout.push_back(*(ts + 1)); };
            any - '"' => { sout.push_back(*ts); };
            '"' => { fgoto colon_sep; };
        *|;

        colon_sep := |*
            ':' => {fgoto match_list; };
        *|;

        match_list := |*
            (' '* (digit @accumulateNum)+ ' '* ','?) >{num = 0;} => handleMatch;
        *|;

        # Old simple line format
        line_old = id ':' @{ fgoto corpus_old; };

        # New line format with matches
        line_new = id "=\"" @{ c.hasMatches = true; fgoto corpus_new; };

        main := ( line_new | line_old );

        # Initialize and execute
        write init;
        write exec;
    }%%

    return (cs != FileCorporaParser_error) && (p == pe);
}
