/*
 * Copyright (c) 2015-2016, Intel Corporation
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
#include "multishufti.h"
#include "multitruffle.h"
#include "multivermicelli.h"
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

    /* multibyte matchers */
    case ACCEL_MLVERM:
        DEBUG_PRINTF("accel mlverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = long_vermicelliExec(accel->mverm.c, 0, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MLVERM_NOCASE:
        DEBUG_PRINTF("accel mlverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = long_vermicelliExec(accel->mverm.c, 1, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MLGVERM:
        DEBUG_PRINTF("accel mlgverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = longgrab_vermicelliExec(accel->mverm.c, 0, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MLGVERM_NOCASE:
        DEBUG_PRINTF("accel mlgverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = longgrab_vermicelliExec(accel->mverm.c, 1, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MSVERM:
        DEBUG_PRINTF("accel msverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shift_vermicelliExec(accel->mverm.c, 0, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MSVERM_NOCASE:
        DEBUG_PRINTF("accel msverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shift_vermicelliExec(accel->mverm.c, 1, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MSGVERM:
        DEBUG_PRINTF("accel msgverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shiftgrab_vermicelliExec(accel->mverm.c, 0, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MSGVERM_NOCASE:
        DEBUG_PRINTF("accel msgverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shiftgrab_vermicelliExec(accel->mverm.c, 1, c, c_end, accel->mverm.len);
        break;
    case ACCEL_MDSVERM:
        DEBUG_PRINTF("accel mdsverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshift_vermicelliExec(accel->mdverm.c, 0, c, c_end,
                                        accel->mdverm.len1, accel->mdverm.len2);
        break;
    case ACCEL_MDSVERM_NOCASE:
        DEBUG_PRINTF("accel mdsverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshift_vermicelliExec(accel->mdverm.c, 1, c, c_end,
                                        accel->mdverm.len1, accel->mdverm.len2);
        break;
    case ACCEL_MDSGVERM:
        DEBUG_PRINTF("accel mdsgverm %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshiftgrab_vermicelliExec(accel->mdverm.c, 0, c, c_end,
                                            accel->mdverm.len1, accel->mdverm.len2);
        break;
    case ACCEL_MDSGVERM_NOCASE:
        DEBUG_PRINTF("accel mdsgverm nc %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshiftgrab_vermicelliExec(accel->mdverm.c, 1, c, c_end,
                                            accel->mdverm.len1, accel->mdverm.len2);
        break;
    case ACCEL_MLSHUFTI:
        DEBUG_PRINTF("accel mlshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = long_shuftiExec(accel->mshufti.lo, accel->mshufti.hi, c, c_end,
                             accel->mshufti.len);
        break;
    case ACCEL_MLGSHUFTI:
        DEBUG_PRINTF("accel mlgshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = longgrab_shuftiExec(accel->mshufti.lo, accel->mshufti.hi, c, c_end,
                                 accel->mshufti.len);
        break;
    case ACCEL_MSSHUFTI:
        DEBUG_PRINTF("accel msshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shift_shuftiExec(accel->mshufti.lo, accel->mshufti.hi, c, c_end,
                              accel->mshufti.len);
        break;
    case ACCEL_MSGSHUFTI:
        DEBUG_PRINTF("accel msgshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shiftgrab_shuftiExec(accel->mshufti.lo, accel->mshufti.hi, c, c_end,
                                  accel->mshufti.len);
        break;
    case ACCEL_MDSSHUFTI:
        DEBUG_PRINTF("accel mdsshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshift_shuftiExec(accel->mdshufti.lo, accel->mdshufti.hi, c, c_end,
                                     accel->mdshufti.len1, accel->mdshufti.len2);
        break;
    case ACCEL_MDSGSHUFTI:
        DEBUG_PRINTF("accel msgshufti %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshiftgrab_shuftiExec(accel->mdshufti.lo, accel->mdshufti.hi, c, c_end,
                                         accel->mdshufti.len1, accel->mdshufti.len2);
        break;
    case ACCEL_MLTRUFFLE:
        DEBUG_PRINTF("accel mltruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = long_truffleExec(accel->mtruffle.mask1, accel->mtruffle.mask2,
                               c, c_end, accel->mtruffle.len);
        break;
    case ACCEL_MLGTRUFFLE:
        DEBUG_PRINTF("accel mlgtruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = longgrab_truffleExec(accel->mtruffle.mask1, accel->mtruffle.mask2,
                                   c, c_end, accel->mtruffle.len);
        break;
    case ACCEL_MSTRUFFLE:
        DEBUG_PRINTF("accel mstruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shift_truffleExec(accel->mtruffle.mask1, accel->mtruffle.mask2,
                               c, c_end, accel->mtruffle.len);
        break;
    case ACCEL_MSGTRUFFLE:
        DEBUG_PRINTF("accel msgtruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = shiftgrab_truffleExec(accel->mtruffle.mask1, accel->mtruffle.mask2,
                                   c, c_end, accel->mtruffle.len);
        break;
    case ACCEL_MDSTRUFFLE:
        DEBUG_PRINTF("accel mdstruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshift_truffleExec(accel->mdtruffle.mask1,
                                     accel->mdtruffle.mask2, c, c_end,
                                     accel->mdtruffle.len1,
                                     accel->mdtruffle.len2);
        break;
    case ACCEL_MDSGTRUFFLE:
        DEBUG_PRINTF("accel mdsgtruffle %p %p\n", c, c_end);
        if (c + 15 >= c_end) {
            return c;
        }

        rv = doubleshiftgrab_truffleExec(accel->mdtruffle.mask1,
                                         accel->mdtruffle.mask2, c, c_end,
                                         accel->mdtruffle.len1,
                                         accel->mdtruffle.len2);
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
