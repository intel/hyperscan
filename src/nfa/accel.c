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

#include "accel.h"
#include "shufti.h"
#include "truffle.h"
#include "vermicelli.h"
#include "ue2common.h"

const u8 *run_accel(const union AccelAux *accel, const u8 *c, const u8 *c_end) {
    assert(ISALIGNED_N(accel, alignof(union AccelAux)));
    const u8 *rv;

    switch (accel->accel_type) {
    case ACCEL_NONE:
        DEBUG_PRINTF("accel none %p %p\n", c, c_end);
        return c;

    case ACCEL_VERM:
        DEBUG_PRINTF("accel verm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = vermicelliExec(accel->verm.c, 0, c, c_end);
        break;

    case ACCEL_VERM_NOCASE:
        DEBUG_PRINTF("accel verm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = vermicelliExec(accel->verm.c, 1, c, c_end);
        break;

    case ACCEL_DVERM:
        DEBUG_PRINTF("accel dverm %p %p\n", c, c_end);
        if (c + 16 + 1 >= c_end) {
            return c;
        }

        /* need to stop one early to get an accurate end state */
        rv = vermicelliDoubleExec(accel->dverm.c1, accel->dverm.c2, 0, c,
                                  c_end - 1);
        break;

    case ACCEL_DVERM_NOCASE:
        DEBUG_PRINTF("accel dverm nc %p %p\n", c, c_end);
        if (c + 16 + 1 >= c_end) {
            return c;
        }

        /* need to stop one early to get an accurate end state */
        rv = vermicelliDoubleExec(accel->dverm.c1, accel->dverm.c2, 1, c,
                                  c_end - 1);
        break;

    case ACCEL_DVERM_MASKED:
        DEBUG_PRINTF("accel dverm masked %p %p\n", c, c_end);
        if (c + 16 + 1 >= c_end) {
            return c;
        }

        /* need to stop one early to get an accurate end state */
        rv = vermicelliDoubleMaskedExec(accel->dverm.c1, accel->dverm.c2,
                                        accel->dverm.m1, accel->dverm.m2,
                                        c, c_end - 1);
        break;

    case ACCEL_SHUFTI:
        DEBUG_PRINTF("accel shufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shuftiExec(accel->shufti.lo, accel->shufti.hi, c, c_end);
        break;

    case ACCEL_TRUFFLE:
        DEBUG_PRINTF("accel Truffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = truffleExec(accel->truffle.mask1, accel->truffle.mask2, c, c_end);
        break;

    case ACCEL_DSHUFTI:
        DEBUG_PRINTF("accel dshufti %p %p\n", c, c_end);
        if (c + 15 + 1 >= c_end) {
            return c;
        }

        /* need to stop one early to get an accurate end state */
        rv = shuftiDoubleExec(accel->dshufti.lo1,
                              accel->dshufti.hi1,
                              accel->dshufti.lo2,
                              accel->dshufti.hi2, c, c_end - 1);
        break;

    case ACCEL_RED_TAPE:
        DEBUG_PRINTF("accel red tape %p %p\n", c, c_end);
        rv = c_end;
        break;


    default:
        assert(!"not here");
        return c;
    }

    DEBUG_PRINTF("adjusting for offset %u\n", accel->generic.offset);
    /* adjust offset to take into account the offset */
    rv = MAX(c + accel->generic.offset, rv);
    rv -= accel->generic.offset;

    DEBUG_PRINTF("advanced %zd\n", rv - c);

    return rv;
}
