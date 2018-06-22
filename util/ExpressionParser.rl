/*
 * Copyright (c) 2015-2018, Intel Corporation
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

#include "ExpressionParser.h"

#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "ue2common.h"
#include "hs_compile.h"


using std::string;

namespace { // anon

enum ParamKey {
    PARAM_NONE,
    PARAM_MIN_OFFSET,
    PARAM_MAX_OFFSET,
    PARAM_MIN_LENGTH,
    PARAM_EDIT_DISTANCE,
    PARAM_HAMM_DISTANCE
};

%%{
    machine ExpressionParser;

    action accumulateNum {
        num = (num * 10) + (fc - '0');
    }

    action handleFlag {
        switch (fc) {
            case 'i': *flags |= HS_FLAG_CASELESS; break;
            case 's': *flags |= HS_FLAG_DOTALL; break;
            case 'm': *flags |= HS_FLAG_MULTILINE; break;
            case 'H': *flags |= HS_FLAG_SINGLEMATCH; break;
            case 'O':
                if (must_be_ordered) {
                    *must_be_ordered = true;
                }
                break;
            case 'V': *flags |= HS_FLAG_ALLOWEMPTY; break;
            case 'W': *flags |= HS_FLAG_UCP; break;
            case '8': *flags |= HS_FLAG_UTF8; break;
            case 'P': *flags |= HS_FLAG_PREFILTER; break;
            case 'L': *flags |= HS_FLAG_SOM_LEFTMOST; break;
            case 'C': *flags |= HS_FLAG_COMBINATION; break;
            case 'Q': *flags |= HS_FLAG_QUIET; break;
            default: fbreak;
        }
    }

    action handleExtParam {
        switch (key) {
            case PARAM_MIN_OFFSET:
                ext->flags |= HS_EXT_FLAG_MIN_OFFSET;
                ext->min_offset = num;
                break;
            case PARAM_MAX_OFFSET:
                ext->flags |= HS_EXT_FLAG_MAX_OFFSET;
                ext->max_offset = num;
                break;
            case PARAM_MIN_LENGTH:
                ext->flags |= HS_EXT_FLAG_MIN_LENGTH;
                ext->min_length = num;
                break;
            case PARAM_EDIT_DISTANCE:
                ext->flags |= HS_EXT_FLAG_EDIT_DISTANCE;
                ext->edit_distance = num;
                break;
            case PARAM_HAMM_DISTANCE:
                ext->flags |= HS_EXT_FLAG_HAMMING_DISTANCE;
                ext->hamming_distance = num;
                break;
            case PARAM_NONE:
            default:
                // No key specified, syntax invalid.
                return false;
        }
    }

    write data;
}%%

} // namespace

static
void initExt(hs_expr_ext *ext) {
    memset(ext, 0, sizeof(*ext));
    ext->max_offset = MAX_OFFSET;
}

bool HS_CDECL readExpression(const std::string &input, std::string &expr,
                             unsigned int *flags, hs_expr_ext *ext,
                             bool *must_be_ordered) {
    assert(flags);
    assert(ext);

    // Init flags and ext params.
    *flags = 0;
    initExt(ext);
    if (must_be_ordered) {
        *must_be_ordered = false;
    }

    // Extract expr, which is easier to do in straight C++ than with Ragel.
    if (input.empty() || input[0] != '/') {
        return false;
    }
    size_t end = input.find_last_of('/');
    if (end == string::npos || end == 0) {
        return false;
    }
    expr = input.substr(1, end - 1);

    // Use a Ragel scanner to handle flags and params.
    const char *p = input.c_str() + end + 1;
    const char *pe = input.c_str() + input.size();
    UNUSED const char *eof = pe;
    UNUSED const char *ts = p, *te = p;
    int cs;
    UNUSED int act;

    assert(p);
    assert(pe);

    // For storing integers as they're scanned.
    u64a num = 0;
    enum ParamKey key = PARAM_NONE;

    %%{
        single_flag = [ismW8HPLVOCQ];
        param = ('min_offset' @{ key = PARAM_MIN_OFFSET; } |
                 'max_offset' @{ key = PARAM_MAX_OFFSET; } |
                 'min_length' @{ key = PARAM_MIN_LENGTH; } |
                 'edit_distance' @{ key = PARAM_EDIT_DISTANCE; } |
                 'hamming_distance' @{ key = PARAM_HAMM_DISTANCE; });

        value = (digit @accumulateNum)+ >{num = 0;};
        param_spec = (' '* param '=' value ' '*) >{ key = PARAM_NONE; }
                                                 %handleExtParam;

        main := ( single_flag @handleFlag )*                # single-char flags
                ( '{' param_spec (',' param_spec)* '}' )?   # list of ext params
                        $^{ return false; };

        # Intialize and execute.
        write init;
        write exec;
    }%%

    DEBUG_PRINTF("expr='%s', flags=%u\n", expr.c_str(), *flags);

    return (cs != ExpressionParser_error) && (p == pe);
}
