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

#include "multiaccel_compilehelper.h"

using namespace std;
using namespace ue2;

#ifdef DEBUG
static const char* state_to_str[] = {
    "FIRST_RUN",
    "SECOND_RUN",
    "WAITING_FOR_GRAB",
    "FIRST_TAIL",
    "SECOND_TAIL",
    "STOPPED",
    "INVALID"
};
static const char* type_to_str[] = {
    "SHIFT",
    "SHIFTGRAB",
    "DOUBLESHIFT",
    "DOUBLESHIFTGRAB",
    "LONG",
    "LONGGRAB",
    "NONE"
};

static
void dumpMultiaccelState(const accel_data &d) {
    DEBUG_PRINTF("type: %s state: %s len1: %u tlen1: %u len2: %u tlen2: %u\n",
                 type_to_str[(unsigned) d.type],
                 state_to_str[(unsigned) d.state],
                 d.len1, d.tlen1, d.len2, d.tlen2);
}
#endif

/* stop all the matching. this may render most schemes invalid. */
static
void stop(accel_data &d) {
    switch (d.state) {
    case STATE_STOPPED:
    case STATE_INVALID:
        break;
    case STATE_FIRST_TAIL:
    case STATE_SECOND_RUN:
        /*
         * Shift matchers are special case, because they have "tails".
         * When shift matcher reaches a mid/endpoint, tail mode is
         * activated, which looks for more matches to extend the match.
         *
         * For example, consider pattern /a{5}ba{3}/. Under normal circumstances,
         * long-grab matcher will be picked for this pattern (matching a run of a's,
         * followed by a not-a), because doubleshift matcher would be confused by
         * consecutive a's and would parse the pattern as a.{0}a.{0}a (two shifts
         * by 1) and throw out the rest of the pattern.
         *
         * With tails, we defer ending the run until we actually run out of
         * matching characters, so the above pattern will now be parsed by
         * doubleshift matcher as /a.{3}a.{3}a/ (two shifts by 4).
         *
         * So if we are stopping shift matchers, we should check if we aren't in
         * the process of matching first tail or second run. If we are, we can't
         * finish the second run as we are stopping, but we can try and split
         * the first tail instead to obtain a valid second run.
         */
        if ((d.type == MultibyteAccelInfo::MAT_DSHIFT ||
                 d.type == MultibyteAccelInfo::MAT_DSHIFTGRAB) && d.tlen1 == 0) {
            // can't split an empty void...
            d.state = STATE_INVALID;
            break;
        }
        d.len2 = 0;
        d.state = STATE_STOPPED;
        break;
    case STATE_SECOND_TAIL:
        d.state = STATE_STOPPED;
        break;
    case STATE_WAITING_FOR_GRAB:
    case STATE_FIRST_RUN:
        if (d.type == MultibyteAccelInfo::MAT_LONG) {
            d.state = STATE_STOPPED;
        } else {
            d.state = STATE_INVALID;
        }
        break;
    }
}

static
void validate(accel_data &d, unsigned max_len) {
    // try and fit in all our tails
    if (d.len1 + d.tlen1 + d.len2 + d.tlen2 < max_len && d.len2 > 0) {
        // case 1: everything fits in
        d.len1 += d.tlen1;
        d.len2 += d.tlen2;
        d.tlen1 = 0;
        d.tlen2 = 0;
    } else if (d.len1 + d.tlen1 + d.len2 < max_len && d.len2 > 0) {
        // case 2: everything but the second tail fits in
        d.len1 += d.tlen1;
        d.tlen1 = 0;
        // try going for a partial tail
        if (d.tlen2 != 0) {
            int new_tlen2 = max_len - 1 - d.len1 - d.len2;
            if (new_tlen2 > 0) {
                d.len2 += new_tlen2;
            }
            d.tlen2 = 0;
        }
    } else if (d.len1 + d.tlen1 < max_len) {
        // case 3: first run and its tail fits in
        if (d.type == MultibyteAccelInfo::MAT_DSHIFT ||
                 d.type == MultibyteAccelInfo::MAT_DSHIFTGRAB) {
            // split the tail into a second run
            d.len2 = d.tlen1;
        } else {
            d.len1 += d.tlen1;
            d.len2 = 0;
        }
        d.tlen1 = 0;
        d.tlen2 = 0;
    } else if (d.len1 < max_len) {
        // case 4: nothing but the first run fits in
        // try going for a partial tail
        if (d.tlen1 != 0) {
            int new_tlen1 = max_len - 1 - d.len1;
            if (new_tlen1 > 0) {
                d.len1 += new_tlen1;
            }
            d.tlen1 = 0;
        }
        d.len2 = 0;
        d.tlen2 = 0;
    }
    // if we removed our second run, doubleshift matchers are no longer valid
    if ((d.type == MultibyteAccelInfo::MAT_DSHIFT ||
                 d.type == MultibyteAccelInfo::MAT_DSHIFTGRAB) && d.len2 == 0) {
        d.state = STATE_INVALID;
    } else if ((d.type == MultibyteAccelInfo::MAT_LONG) && d.len1 >= max_len) {
        // long matchers can just stop whenever they want to
        d.len1 = max_len - 1;
    }

    // now, general sanity checks
    if ((d.len1 + d.tlen1 + d.len2 + d.tlen2) >= max_len) {
        d.state = STATE_INVALID;
    }
    if ((d.len1 + d.tlen1 + d.len2 + d.tlen2) < MULTIACCEL_MIN_LEN) {
        d.state = STATE_INVALID;
    }
}

static
void match(accel_data &d, const CharReach &ref_cr, const CharReach &cur_cr) {
    switch (d.type) {
    case MultibyteAccelInfo::MAT_LONG:
        {
            /*
             * For long matcher, we want lots of consecutive same-or-subset
             * char-reaches
             */
            if ((ref_cr & cur_cr) == cur_cr) {
                d.len1++;
            } else {
                d.state = STATE_STOPPED;
            }
        }
        break;

    case MultibyteAccelInfo::MAT_LONGGRAB:
        {
            /*
             * For long-grab matcher, we want lots of consecutive same-or-subset
             * char-reaches with a negative match in the end.
             */
            if ((ref_cr & cur_cr) == cur_cr) {
                d.len1++;
            } else if (!(ref_cr & cur_cr).any()) {
                /* we grabbed, stop immediately */
                d.state = STATE_STOPPED;
            } else {
                /* our run-n-grab was interrupted; mark as invalid */
                d.state = STATE_INVALID;
            }
        }
        break;

    case MultibyteAccelInfo::MAT_SHIFTGRAB:
        {
            /*
             * For shift-grab matcher, we want two matches separated by anything;
             * however the second vertex *must* be a negative (non-overlapping) match.
             *
             * Shiftgrab matcher is identical to shift except for presence of grab.
             */
            if (d.state == STATE_WAITING_FOR_GRAB) {
                if ((ref_cr & cur_cr).any()) {
                    d.state = STATE_INVALID;
                } else {
                    d.state = STATE_FIRST_RUN;
                    d.len1++;
                }
                return;
            }
        }
        /* no break, falling through */
    case MultibyteAccelInfo::MAT_SHIFT:
        {
            /*
             * For shift-matcher, we want two matches separated by anything.
             */
            if (ref_cr == cur_cr) {
                // keep matching tail
                switch (d.state) {
                case STATE_FIRST_RUN:
                    d.state = STATE_FIRST_TAIL;
                    break;
                case STATE_FIRST_TAIL:
                    d.tlen1++;
                    break;
                default:
                    // shouldn't happen
                    assert(0);
                }
            } else {
                switch (d.state) {
                case STATE_FIRST_RUN:
                    // simply advance
                    d.len1++;
                    break;
                case STATE_FIRST_TAIL:
                    // we found a non-matching char after tail, so stop
                    d.state = STATE_STOPPED;
                    break;
                default:
                    // shouldn't happen
                    assert(0);
                }
            }
        }
        break;

    case MultibyteAccelInfo::MAT_DSHIFTGRAB:
        {
            /*
             * For double shift-grab matcher, we want two matches separated by
             * either negative matches or dots; however the second vertex *must*
             * be a negative match.
             *
             * Doubleshiftgrab matcher is identical to doubleshift except for
             * presence of grab.
             */
            if (d.state == STATE_WAITING_FOR_GRAB) {
                if ((ref_cr & cur_cr).any()) {
                    d.state = STATE_INVALID;
                } else {
                    d.state = STATE_FIRST_RUN;
                    d.len1++;
                }
                return;
            }
        }
        /* no break, falling through */
    case MultibyteAccelInfo::MAT_DSHIFT:
        {
            /*
             * For double shift matcher, we want three matches, each separated
             * by a lot of anything.
             *
             * Doubleshift matcher is complicated by presence of tails.
             */
            if (ref_cr == cur_cr) {
                // decide if we are activating second shift or matching tails
                switch (d.state) {
                case STATE_FIRST_RUN:
                    d.state = STATE_FIRST_TAIL;
                    d.len2 = 1; // we're now ready for our second run
                    break;
                case STATE_FIRST_TAIL:
                    d.tlen1++;
                    break;
                case STATE_SECOND_RUN:
                    d.state = STATE_SECOND_TAIL;
                    break;
                case STATE_SECOND_TAIL:
                    d.tlen2++;
                    break;
                default:
                    // shouldn't happen
                    assert(0);
                }
            } else {
                switch (d.state) {
                case STATE_FIRST_RUN:
                    d.len1++;
                    break;
                case STATE_FIRST_TAIL:
                    // start second run
                    d.state = STATE_SECOND_RUN;
                    d.len2++;
                    break;
                case STATE_SECOND_RUN:
                    d.len2++;
                    break;
                case STATE_SECOND_TAIL:
                    // stop
                    d.state = STATE_STOPPED;
                    break;
                default:
                    // shouldn't happen
                    assert(0);
                }
            }
        }
        break;

    default:
        // shouldn't happen
        assert(0);
        break;
    }
}

MultiaccelCompileHelper::MultiaccelCompileHelper(const CharReach &ref_cr, u32 off,
                                                 unsigned max_len) :
        cr(ref_cr), offset(off), max_len(max_len) {
    int accel_num = (int) MultibyteAccelInfo::MAT_MAX;
    accels.resize(accel_num);

    // mark everything as valid
    for (int i = 0; i < accel_num; i++) {
        accel_data &ad = accels[i];
        ad.len1 = 1;
        ad.type = (MultibyteAccelInfo::multiaccel_type) i;

        /* for shift-grab matchers, we are waiting for the grab right at the start */
        if (ad.type == MultibyteAccelInfo::MAT_SHIFTGRAB
                || ad.type == MultibyteAccelInfo::MAT_DSHIFTGRAB) {
            ad.state = STATE_WAITING_FOR_GRAB;
        } else {
            ad.state = STATE_FIRST_RUN;
        }
    }
}

bool MultiaccelCompileHelper::canAdvance() {
    for (const accel_data &ad : accels) {
        if (ad.state != STATE_STOPPED && ad.state != STATE_INVALID) {
            return true;
        }
    }
    return false;
}

void MultiaccelCompileHelper::advance(const CharReach &cur_cr) {
    for (accel_data &ad : accels) {
        if (ad.state == STATE_STOPPED || ad.state == STATE_INVALID) {
            continue;
        }
        match(ad, cr, cur_cr);
#ifdef DEBUG
        dumpMultiaccelState(ad);
#endif
    }
}

MultibyteAccelInfo MultiaccelCompileHelper::getBestScheme() {
    int best_len = 0;
    accel_data best;

    DEBUG_PRINTF("Stopping multiaccel compile\n");

    for (accel_data &ad : accels) {
        // stop our matching
        stop(ad);
        validate(ad, max_len);

#ifdef DEBUG
        dumpMultiaccelState(ad);
#endif

        // skip invalid schemes
        if (ad.state == STATE_INVALID) {
            continue;
        }
        DEBUG_PRINTF("Marking as viable\n");

        // TODO: relative strengths of accel schemes? maybe e.g. a shorter
        // long match would in some cases be preferable to a longer
        // double shift match (for example, depending on length)?
        int as_len = ad.len1 + ad.len2;
        if (as_len >= best_len) {
            DEBUG_PRINTF("Marking as best\n");
            best_len = as_len;
            best = ad;
        }
    }
    // if we found at least one accel scheme, return it
    if (best.state != STATE_INVALID) {
#ifdef DEBUG
        DEBUG_PRINTF("Picked best multiaccel state:\n");
        dumpMultiaccelState(best);
#endif
        MultibyteAccelInfo info;
        info.cr = cr;
        info.offset = offset;
        info.len1 = best.len1;
        info.len2 = best.len2;
        info.type = best.type;
        return info;
    }
    return MultibyteAccelInfo();
}
