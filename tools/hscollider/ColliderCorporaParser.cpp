

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
#include <cstdio>
#include <cstdlib>
#include <string>

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

static const char _FileCorporaParser_actions[] = {
    0,  1,  0,  1,  3,  1,  4,  1,  5,  1,  6, 1,  7, 1,  8, 1,  9, 1,  10,
    1,  11, 1,  12, 1,  13, 1,  14, 1,  15, 1, 16, 1, 17, 1, 18, 1, 19, 1,
    20, 1,  21, 1,  22, 1,  23, 1,  24, 2,  0, 2,  2, 3,  0, 3,  1, 0,  2};

static const char _FileCorporaParser_key_offsets[] = {
    0, 0, 2, 6, 7, 13, 19, 25, 31, 34, 34, 35, 52, 54, 71, 72, 75, 79};

static const char _FileCorporaParser_trans_keys[] = {
    48,  57,  58,  61,  48,  57,  34,  48,  57,  65,  70,  97,  102, 48,
    57,  65,  70,  97,  102, 48,  57,  65,  70,  97,  102, 48,  57,  65,
    70,  97,  102, 32,  48,  57,  92,  48,  97,  110, 114, 116, 118, 120,
    49,  57,  65,  90,  98,  100, 101, 102, 103, 122, 34,  92,  48,  97,
    110, 114, 116, 118, 120, 49,  57,  65,  90,  98,  100, 101, 102, 103,
    122, 58,  32,  48,  57,  32,  44,  48,  57,  32,  44,  0};

static const char _FileCorporaParser_single_lengths[] = {
    0, 0, 2, 1, 0, 0, 0, 0, 1, 0, 1, 7, 2, 7, 1, 1, 2, 2};

static const char _FileCorporaParser_range_lengths[] = {
    0, 1, 1, 0, 3, 3, 3, 3, 1, 0, 0, 5, 0, 5, 0, 1, 1, 0};

static const char _FileCorporaParser_index_offsets[] = {
    0, 0, 2, 6, 8, 12, 16, 20, 24, 27, 28, 30, 43, 46, 59, 61, 64, 68};

static const char _FileCorporaParser_indicies[] = {
    0,  1,  3,  4,  2,  1,  5,  1,  7,  7,  7,  6,  8,  8,  8,  6,  10, 10,
    10, 9,  11, 11, 11, 9,  12, 13, 1,  1,  15, 14, 18, 18, 18, 18, 18, 18,
    19, 16, 16, 16, 18, 16, 17, 21, 22, 20, 25, 25, 25, 25, 25, 25, 26, 23,
    23, 23, 25, 23, 24, 27, 1,  28, 29, 1,  31, 32, 13, 30, 31, 32, 30, 0};

static const char _FileCorporaParser_trans_targs[] = {
    2,  0,  2, 9,  3,  9,  10, 5,  10, 12, 7,  12, 8,  16, 10, 11, 10,
    10, 10, 4, 12, 12, 13, 12, 12, 12, 6,  14, 8,  16, 15, 17, 15};

static const char _FileCorporaParser_trans_actions[] = {
    53, 0,  47, 5,  0,  7,  25, 0,  15, 39, 0,  27, 0,  1,  21, 13, 23,
    19, 17, 0,  33, 35, 13, 37, 31, 29, 0,  41, 3,  50, 45, 0,  43};

static const char _FileCorporaParser_to_state_actions[] = {
    0, 9, 0, 0, 0, 0, 0, 0, 0, 0, 9, 0, 9, 0, 9, 9, 0, 0};

static const char _FileCorporaParser_from_state_actions[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 11, 0, 11, 0, 11, 11, 0, 0};

static const char _FileCorporaParser_eof_trans[] = {
    0, 0, 0, 0, 7, 7, 10, 10, 0, 0, 0, 17, 0, 24, 0, 0, 31, 31};

static const int FileCorporaParser_start = 1;
static const int FileCorporaParser_first_final = 9;
static const int FileCorporaParser_error = 0;

static const int FileCorporaParser_en_corpus_old = 10;
static const int FileCorporaParser_en_corpus_new = 12;
static const int FileCorporaParser_en_colon_sep = 14;
static const int FileCorporaParser_en_match_list = 15;
static const int FileCorporaParser_en_main = 1;

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

    {
        cs = FileCorporaParser_start;
        ts = 0;
        te = 0;
        act = 0;
    }

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
        _acts = _FileCorporaParser_actions +
                _FileCorporaParser_from_state_actions[cs];
        _nacts = (unsigned int)*_acts++;
        while (_nacts-- > 0) {
            switch (*_acts++) {
            case 7:

            {
                ts = p;
            } break;
            }
        }

        _keys =
            _FileCorporaParser_trans_keys + _FileCorporaParser_key_offsets[cs];
        _trans = _FileCorporaParser_index_offsets[cs];

        _klen = _FileCorporaParser_single_lengths[cs];
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

        _klen = _FileCorporaParser_range_lengths[cs];
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
        _trans = _FileCorporaParser_indicies[_trans];
    _eof_trans:
        cs = _FileCorporaParser_trans_targs[_trans];

        if (_FileCorporaParser_trans_actions[_trans] == 0)
            goto _again;

        _acts = _FileCorporaParser_actions +
                _FileCorporaParser_trans_actions[_trans];
        _nacts = (unsigned int)*_acts++;
        while (_nacts-- > 0) {
            switch (*_acts++) {
            case 0:

            {
                num = (num * 10) + ((*p) - '0');
            } break;
            case 1:

            {
                num = 0;
            } break;
            case 2:

            {
                id = num;
            } break;
            case 3:

            {
                num = 0;
            } break;
            case 4:

            {
                {
                    cs = 10;
                    goto _again;
                }
            } break;
            case 5:

            {
                c.hasMatches = true;
                {
                    cs = 12;
                    goto _again;
                }
            } break;
            case 8:

            {
                te = p + 1;
            } break;
            case 9:

            {
                te = p + 1;
                { sout.push_back(unhex(ts, te)); }
            } break;
            case 10:

            {
                te = p + 1;
                {
                    switch (*(ts + 1)) {
                    case '0':
                        sout.push_back('\x00');
                        break;
                    case 'a':
                        sout.push_back('\x07');
                        break;
                    case 'e':
                        sout.push_back('\x1b');
                        break;
                    case 'f':
                        sout.push_back('\x0c');
                        break;
                    case 'n':
                        sout.push_back('\x0a');
                        break;
                    case 'v':
                        sout.push_back('\x0b');
                        break;
                    case 'r':
                        sout.push_back('\x0d');
                        break;
                    case 't':
                        sout.push_back('\x09');
                        break;
                    default: {
                        p++;
                        goto _out;
                    }
                    }
                }
            } break;
            case 11:

            {
                te = p + 1;
                { sout.push_back(*(ts + 1)); }
            } break;
            case 12:

            {
                te = p + 1;
                { sout.push_back(*ts); }
            } break;
            case 13:

            {
                te = p;
                p--;
                { sout.push_back(*ts); }
            } break;
            case 14:

            {
                { p = ((te)) - 1; }
                { sout.push_back(*ts); }
            } break;
            case 15:

            {
                te = p + 1;
                { sout.push_back(unhex(ts, te)); }
            } break;
            case 16:

            {
                te = p + 1;
                {
                    switch (*(ts + 1)) {
                    case '0':
                        sout.push_back('\x00');
                        break;
                    case 'a':
                        sout.push_back('\x07');
                        break;
                    case 'e':
                        sout.push_back('\x1b');
                        break;
                    case 'f':
                        sout.push_back('\x0c');
                        break;
                    case 'n':
                        sout.push_back('\x0a');
                        break;
                    case 'v':
                        sout.push_back('\x0b');
                        break;
                    case 'r':
                        sout.push_back('\x0d');
                        break;
                    case 't':
                        sout.push_back('\x09');
                        break;
                    default: {
                        p++;
                        goto _out;
                    }
                    }
                }
            } break;
            case 17:

            {
                te = p + 1;
                { sout.push_back(*(ts + 1)); }
            } break;
            case 18:

            {
                te = p + 1;
                { sout.push_back(*ts); }
            } break;
            case 19:

            {
                te = p + 1;
                {
                    {
                        cs = 14;
                        goto _again;
                    }
                }
            } break;
            case 20:

            {
                te = p;
                p--;
                { sout.push_back(*ts); }
            } break;
            case 21:

            {
                { p = ((te)) - 1; }
                { sout.push_back(*ts); }
            } break;
            case 22:

            {
                te = p + 1;
                {
                    {
                        cs = 15;
                        goto _again;
                    }
                }
            } break;
            case 23:

            {
                te = p + 1;
                { c.matches.insert(num); }
            } break;
            case 24:

            {
                te = p;
                p--;
                { c.matches.insert(num); }
            } break;
            }
        }

    _again:
        _acts = _FileCorporaParser_actions +
                _FileCorporaParser_to_state_actions[cs];
        _nacts = (unsigned int)*_acts++;
        while (_nacts-- > 0) {
            switch (*_acts++) {
            case 6:

            {
                ts = 0;
            } break;
            }
        }

        if (cs == 0)
            goto _out;
        if (++p != pe)
            goto _resume;
    _test_eof : {}
        if (p == eof) {
            if (_FileCorporaParser_eof_trans[cs] > 0) {
                _trans = _FileCorporaParser_eof_trans[cs] - 1;
                goto _eof_trans;
            }
        }

    _out : {}
    }

    return (cs != FileCorporaParser_error) && (p == pe);
}
