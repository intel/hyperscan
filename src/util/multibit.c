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

/** \file
 * \brief Multibit: lookup tables and support code.
 *
 * This C file contains the constant tables used by multibit, so we don't end
 * up creating copies of them for every unit that uses it.
 */

#include "multibit.h"
#include "ue2common.h"

const u8 mmbit_keyshift_lut[32] = {
    30, 30, 24, 24, 24, 24, 24, 24, 18, 18, 18,
    18, 18, 18, 12, 12, 12, 12, 12, 12, 6, 6,
    6,  6,  6,  6,  0,  0,  0,  0,  0,  0
};

// The only actually valid values of ks are as shown in the LUT above, but a
// division is just too expensive.
const u8 mmbit_maxlevel_from_keyshift_lut[32] = {
    0, 0, 0, 0, 0, 0,
    1, 1, 1, 1, 1, 1,
    2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3,
    4, 4, 4, 4, 4, 4,
    5, 5
};

const u8 mmbit_maxlevel_direct_lut[32] = {
    5, 5, 4, 4, 4, 4, 4, 4, 3, 3, 3,
    3, 3, 3, 2, 2, 2, 2, 2, 2, 1, 1,
    1, 1, 1, 1, 0, 0, 0, 0, 0, 0
};

#define ZERO_TO_LUT(x) ((1ULL << x) - 1)

const u64a mmbit_zero_to_lut[65] = {
    ZERO_TO_LUT(0),
    ZERO_TO_LUT(1),
    ZERO_TO_LUT(2),
    ZERO_TO_LUT(3),
    ZERO_TO_LUT(4),
    ZERO_TO_LUT(5),
    ZERO_TO_LUT(6),
    ZERO_TO_LUT(7),
    ZERO_TO_LUT(8),
    ZERO_TO_LUT(9),
    ZERO_TO_LUT(10),
    ZERO_TO_LUT(11),
    ZERO_TO_LUT(12),
    ZERO_TO_LUT(13),
    ZERO_TO_LUT(14),
    ZERO_TO_LUT(15),
    ZERO_TO_LUT(16),
    ZERO_TO_LUT(17),
    ZERO_TO_LUT(18),
    ZERO_TO_LUT(19),
    ZERO_TO_LUT(20),
    ZERO_TO_LUT(21),
    ZERO_TO_LUT(22),
    ZERO_TO_LUT(23),
    ZERO_TO_LUT(24),
    ZERO_TO_LUT(25),
    ZERO_TO_LUT(26),
    ZERO_TO_LUT(27),
    ZERO_TO_LUT(28),
    ZERO_TO_LUT(29),
    ZERO_TO_LUT(30),
    ZERO_TO_LUT(31),
    ZERO_TO_LUT(32),
    ZERO_TO_LUT(33),
    ZERO_TO_LUT(34),
    ZERO_TO_LUT(35),
    ZERO_TO_LUT(36),
    ZERO_TO_LUT(37),
    ZERO_TO_LUT(38),
    ZERO_TO_LUT(39),
    ZERO_TO_LUT(40),
    ZERO_TO_LUT(41),
    ZERO_TO_LUT(42),
    ZERO_TO_LUT(43),
    ZERO_TO_LUT(44),
    ZERO_TO_LUT(45),
    ZERO_TO_LUT(46),
    ZERO_TO_LUT(47),
    ZERO_TO_LUT(48),
    ZERO_TO_LUT(49),
    ZERO_TO_LUT(50),
    ZERO_TO_LUT(51),
    ZERO_TO_LUT(52),
    ZERO_TO_LUT(53),
    ZERO_TO_LUT(54),
    ZERO_TO_LUT(55),
    ZERO_TO_LUT(56),
    ZERO_TO_LUT(57),
    ZERO_TO_LUT(58),
    ZERO_TO_LUT(59),
    ZERO_TO_LUT(60),
    ZERO_TO_LUT(61),
    ZERO_TO_LUT(62),
    ZERO_TO_LUT(63),
    ~0ULL
};

const u32 mmbit_root_offset_from_level[7] = {
    0,
    1,
    1 + (1 << MMB_KEY_SHIFT),
    1 + (1 << MMB_KEY_SHIFT) + (1 << MMB_KEY_SHIFT * 2),
    1 + (1 << MMB_KEY_SHIFT) + (1 << MMB_KEY_SHIFT * 2) + (1 << MMB_KEY_SHIFT * 3),
    1 + (1 << MMB_KEY_SHIFT) + (1 << MMB_KEY_SHIFT * 2) + (1 << MMB_KEY_SHIFT * 3) + (1 << MMB_KEY_SHIFT * 4),
    1 + (1 << MMB_KEY_SHIFT) + (1 << MMB_KEY_SHIFT * 2) + (1 << MMB_KEY_SHIFT * 3) + (1 << MMB_KEY_SHIFT * 4) + (1 << MMB_KEY_SHIFT * 5),
};
