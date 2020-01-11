

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

#include "hs_compile.h"
#include "ue2common.h"

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

static const char _ExpressionParser_actions[] = {0, 1, 0,  1, 1, 1, 2, 1, 3,
                                                 1, 4, 1,  5, 1, 6, 1, 7, 1,
                                                 9, 1, 10, 2, 8, 0

};

static const char _ExpressionParser_key_offsets[] = {
    0,  0,  4,  8,  9,  10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 23, 28, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41,
    42, 43, 44, 45, 46, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
    58, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 82};

static const char _ExpressionParser_trans_keys[] = {
    32,  101, 104, 109, 32,  101, 104, 109, 100, 105, 116, 95,  100, 105,
    115, 116, 97,  110, 99,  101, 61,  48,  57,  32,  44,  125, 48,  57,
    32,  44,  125, 97,  109, 109, 105, 110, 103, 95,  100, 105, 115, 116,
    97,  110, 99,  101, 97,  105, 120, 95,  111, 102, 102, 115, 101, 116,
    110, 95,  108, 111, 101, 110, 103, 116, 104, 102, 102, 115, 101, 116,
    56,  67,  72,  76,  105, 109, 115, 123, 79,  81,  86,  87,  0};

static const char _ExpressionParser_single_lengths[] = {
    0, 4, 4, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 3, 3, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 8, 0};

static const char _ExpressionParser_range_lengths[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0};

static const unsigned char _ExpressionParser_index_offsets[] = {
    0,   0,   5,   10,  12,  14,  16,  18,  20,  22,  24,  26,  28, 30, 32,
    34,  36,  38,  43,  47,  49,  51,  53,  55,  57,  59,  61,  63, 65, 67,
    69,  71,  73,  75,  77,  80,  82,  84,  86,  88,  90,  92,  94, 96, 98,
    100, 103, 105, 107, 109, 111, 113, 115, 117, 119, 121, 123, 134};

static const char _ExpressionParser_trans_targs[] = {
    2,  3,  19, 34, 0,  2,  3,  19, 34, 0,  4,  0,  5,  0,  6,  0,  7,
    0,  8,  0,  9,  0,  10, 0,  11, 0,  12, 0,  13, 0,  14, 0,  15, 0,
    16, 0,  17, 0,  18, 1,  57, 17, 0,  18, 1,  57, 0,  20, 0,  21, 0,
    22, 0,  23, 0,  24, 0,  25, 0,  26, 0,  27, 0,  28, 0,  29, 0,  30,
    0,  31, 0,  32, 0,  33, 0,  15, 0,  35, 43, 0,  36, 0,  37, 0,  38,
    0,  39, 0,  40, 0,  41, 0,  42, 0,  15, 0,  44, 0,  45, 0,  46, 51,
    0,  47, 0,  48, 0,  49, 0,  50, 0,  15, 0,  52, 0,  53, 0,  54, 0,
    55, 0,  15, 0,  56, 56, 56, 56, 56, 56, 56, 1,  56, 56, 0,  0,  0};

static const char _ExpressionParser_trans_actions[] = {
    17, 17, 17, 17, 19, 0,  0,  0,  0,  19, 0,  19, 0,  19, 0,  19, 0,
    19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 13, 19,
    0,  19, 21, 19, 0,  5,  5,  1,  19, 0,  5,  5,  19, 0,  19, 0,  19,
    0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,  19, 0,
    19, 0,  19, 0,  19, 0,  19, 15, 19, 0,  0,  19, 0,  19, 0,  19, 0,
    19, 0,  19, 0,  19, 0,  19, 0,  19, 9,  19, 0,  19, 0,  19, 0,  0,
    19, 0,  19, 0,  19, 0,  19, 0,  19, 11, 19, 0,  19, 0,  19, 0,  19,
    0,  19, 7,  19, 3,  3,  3,  3,  3,  3,  3,  0,  3,  3,  19, 19, 0};

static const char _ExpressionParser_eof_actions[] = {
    0,  19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19,
    19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 0,  0};

static const int ExpressionParser_start = 56;
static const int ExpressionParser_first_final = 56;
static const int ExpressionParser_error = 0;

static const int ExpressionParser_en_main = 56;

} // namespace

static void initExt(hs_expr_ext *ext) {
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

    { cs = ExpressionParser_start; }

    {
        int _klen;
        unsigned int _trans;
        const char *_acts;
        unsigned int _nacts;
        const char *_keys;

        if (p == pe)
            goto _test_eof;
        if (cs == 0)
            goto _out;
    _resume:
        _keys =
            _ExpressionParser_trans_keys + _ExpressionParser_key_offsets[cs];
        _trans = _ExpressionParser_index_offsets[cs];

        _klen = _ExpressionParser_single_lengths[cs];
        if (_klen > 0) {
            const char *_lower = _keys;
            const char *_mid;
            const char *_upper = _keys + _klen - 1;
            while (1) {
                if (_upper < _lower)
                    break;

                _mid = _lower + ((_upper - _lower) >> 1);
                if ((*p) < *_mid)
                    _upper = _mid - 1;
                else if ((*p) > *_mid)
                    _lower = _mid + 1;
                else {
                    _trans += (unsigned int)(_mid - _keys);
                    goto _match;
                }
            }
            _keys += _klen;
            _trans += _klen;
        }

        _klen = _ExpressionParser_range_lengths[cs];
        if (_klen > 0) {
            const char *_lower = _keys;
            const char *_mid;
            const char *_upper = _keys + (_klen << 1) - 2;
            while (1) {
                if (_upper < _lower)
                    break;

                _mid = _lower + (((_upper - _lower) >> 1) & ~1);
                if ((*p) < _mid[0])
                    _upper = _mid - 2;
                else if ((*p) > _mid[1])
                    _lower = _mid + 2;
                else {
                    _trans += (unsigned int)((_mid - _keys) >> 1);
                    goto _match;
                }
            }
            _trans += _klen;
        }

    _match:
        cs = _ExpressionParser_trans_targs[_trans];

        if (_ExpressionParser_trans_actions[_trans] == 0)
            goto _again;

        _acts =
            _ExpressionParser_actions + _ExpressionParser_trans_actions[_trans];
        _nacts = (unsigned int)*_acts++;
        while (_nacts-- > 0) {
            switch (*_acts++) {
            case 0:

            {
                num = (num * 10) + ((*p) - '0');
            } break;
            case 1:

            {
                switch ((*p)) {
                case 'i':
                    *flags |= HS_FLAG_CASELESS;
                    break;
                case 's':
                    *flags |= HS_FLAG_DOTALL;
                    break;
                case 'm':
                    *flags |= HS_FLAG_MULTILINE;
                    break;
                case 'H':
                    *flags |= HS_FLAG_SINGLEMATCH;
                    break;
                case 'O':
                    if (must_be_ordered) {
                        *must_be_ordered = true;
                    }
                    break;
                case 'V':
                    *flags |= HS_FLAG_ALLOWEMPTY;
                    break;
                case 'W':
                    *flags |= HS_FLAG_UCP;
                    break;
                case '8':
                    *flags |= HS_FLAG_UTF8;
                    break;
                case 'P':
                    *flags |= HS_FLAG_PREFILTER;
                    break;
                case 'L':
                    *flags |= HS_FLAG_SOM_LEFTMOST;
                    break;
                case 'C':
                    *flags |= HS_FLAG_COMBINATION;
                    break;
                case 'Q':
                    *flags |= HS_FLAG_QUIET;
                    break;
                default: {
                    p++;
                    goto _out;
                }
                }
            } break;
            case 2:

            {
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
            } break;
            case 3:

            {
                key = PARAM_MIN_OFFSET;
            } break;
            case 4:

            {
                key = PARAM_MAX_OFFSET;
            } break;
            case 5:

            {
                key = PARAM_MIN_LENGTH;
            } break;
            case 6:

            {
                key = PARAM_EDIT_DISTANCE;
            } break;
            case 7:

            {
                key = PARAM_HAMM_DISTANCE;
            } break;
            case 8:

            {
                num = 0;
            } break;
            case 9:

            {
                key = PARAM_NONE;
            } break;
            case 10:

            {
                return false;
            } break;
            }
        }

    _again:
        if (cs == 0)
            goto _out;
        if (++p != pe)
            goto _resume;
    _test_eof : {}
        if (p == eof) {
            const char *__acts =
                _ExpressionParser_actions + _ExpressionParser_eof_actions[cs];
            unsigned int __nacts = (unsigned int)*__acts++;
            while (__nacts-- > 0) {
                switch (*__acts++) {
                case 10:

                {
                    return false;
                } break;
                }
            }
        }

    _out : {}
    }

    DEBUG_PRINTF("expr='%s', flags=%u\n", expr.c_str(), *flags);

    return (cs != ExpressionParser_error) && (p == pe);
}
