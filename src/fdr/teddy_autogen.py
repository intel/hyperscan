#!/usr/bin/python

# Copyright (c) 2015, Intel Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of Intel Corporation nor the names of its contributors
#       may be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
from autogen_utils import *
from base_autogen import *
from string import Template

class MT(MatcherBase):
    def produce_confirm(self, iter, var_name, offset, bits, cautious = True):
        if self.packed:
            print self.produce_confirm_base(var_name, bits, iter*16 + offset, cautious, enable_confirmless = False, do_bailout = False)
        else:
            if self.num_masks == 1:
                conf_func = "confWithBit1"
            else:
                conf_func = "confWithBitMany"

            if cautious:
                caution_string = "VECTORING"
            else:
                caution_string = "NOT_CAUTIOUS"

            print "            if (P0(!!%s)) {" % var_name
            print "                do  {"
            if bits == 64:
                print "                    bit = findAndClearLSB_64(&%s);" % (var_name)
            else:
                print "                    bit = findAndClearLSB_32(&%s);" % (var_name)
            print "                    byte  = bit / %d + %d;" % (self.num_buckets, iter*16 + offset)
            print "                    idx  = bit %% %d;" % self.num_buckets
            print "                    cf = confBase[idx];"
            print "                    fdrc = (const struct FDRConfirm *)((const u8 *)confBase + cf);"
            print "                    if (!(fdrc->groups & *control))"
            print "                        continue;"
            print "                    %s(fdrc, a, ptr - buf + byte, %s, control, &last_match);" % (conf_func, caution_string)
            print "                } while(P0(!!%s));" % var_name
            print "                if (P0(controlVal == HWLM_TERMINATE_MATCHING)) {"
            print "                    *a->groups = controlVal;"
            print "                    return HWLM_TERMINATED;"
            print "                }"
            print "            }"

    def produce_needed_temporaries(self, max_iterations):
        print "        m128 p_mask;"
        for iter in range(0, max_iterations):
            print "        m128 val_%d;" % iter
            print "        m128 val_%d_lo;" % iter
            print "        m128 val_%d_hi;" % iter
            for x in range(self.num_masks):
                print "        m128 res_%d_%d;" % (iter, x)
                if x != 0:
                    print "        m128 res_shifted_%d_%d;" % (iter, x)
            print "        m128 r_%d;" % iter
            print "#ifdef ARCH_64_BIT"
            print "            u64a r_%d_lopart;" % iter
            print "            u64a r_%d_hipart;" % iter
            print "#else"
            print "            u32 r_%d_part1;" % iter
            print "            u32 r_%d_part2;" % iter
            print "            u32 r_%d_part3;" % iter
            print "            u32 r_%d_part4;" % iter
            print "#endif"

    def produce_one_iteration_state_calc(self, iter, effective_num_iterations,
                                         cautious, save_old):
        if cautious:
            print "        val_%d = vectoredLoad128(&p_mask, ptr + %d, buf, buf+len, a->buf_history, a->len_history, %d);" % (iter, iter*16, self.num_masks)
        else:
            print "        val_%d = load128(ptr + %d);" % (iter, iter*16)
        print "        val_%d_lo = and128(val_%d, lomask);" % (iter, iter)
        print "        val_%d_hi = rshift2x64(val_%d, 4);" % (iter, iter)
        print "        val_%d_hi = and128(val_%d_hi, lomask);" % (iter, iter)
        print
        for x in range(self.num_masks):
            print Template("""
        res_${ITER}_${X} = and128(pshufb(maskBase[${X}*2]  , val_${ITER}_lo),
                                  pshufb(maskBase[${X}*2+1], val_${ITER}_hi));""").substitute(ITER = iter, X = x)
            if x != 0:
                if iter == 0:
                    print "        res_shifted_%d_%d = palignr(res_%d_%d, res_old_%d, 16-%d);" % (iter, x,   iter, x,         x,   x)
                else:
                    print "        res_shifted_%d_%d = palignr(res_%d_%d, res_%d_%d, 16-%d);" % (iter, x,    iter, x, iter-1, x,   x)
            if x != 0 and iter == effective_num_iterations - 1 and save_old:
                print "        res_old_%d = res_%d_%d;" % (x, iter, x)
        print
        if cautious:
            print "        r_%d = and128(res_%d_0, p_mask);" % (iter, iter)
        else:
            print "        r_%d = res_%d_0;" % (iter, iter)
        for x in range(1, self.num_masks):
            print "        r_%d = and128(r_%d, res_shifted_%d_%d);" % (iter, iter, iter, x)
        print

    def produce_one_iteration_confirm(self, iter, confirmCautious):
        setup64 = [ (0, "r_%d_lopart" % iter, "movq(r_%d)" % iter),
                    (8, "r_%d_hipart" % iter, "movq(byteShiftRight128(r_%d, 8))" % iter) ]

        setup32 = [ (0, "r_%d_part1" % iter, "movd(r_%d)" % iter),
                    (4, "r_%d_part2" % iter, "movd(byteShiftRight128(r_%d, 4))" % iter),
                    (8, "r_%d_part3" % iter, "movd(byteShiftRight128(r_%d, 8))" % iter),
                    (12, "r_%d_part4" % iter, "movd(byteShiftRight128(r_%d, 12))" % iter) ]

        print "        if (P0(isnonzero128(r_%d))) {" % (iter)
        print "#ifdef ARCH_64_BIT"
        for (off, val, init) in setup64:
            print "            %s = %s;" % (val, init)
        for (off, val, init) in setup64:
            self.produce_confirm(iter, val, off, 64, cautious = confirmCautious)
        print "#else"
        for (off, val, init) in setup32:
            print "            %s = %s;" % (val, init)
        for (off, val, init) in setup32:
            self.produce_confirm(iter, val, off, 32, cautious = confirmCautious)
        print "#endif"
        print "        }"

    def produce_one_iteration(self, iter, effective_num_iterations, cautious = False,
                              confirmCautious = True, save_old = True):
        self.produce_one_iteration_state_calc(iter, effective_num_iterations, cautious, save_old)
        self.produce_one_iteration_confirm(iter, confirmCautious)

    def produce_code(self):
        print self.produce_header(visible = True, header_only = False)
        print self.produce_common_declarations()
        print

        self.produce_needed_temporaries(self.num_iterations)
        print

        print "    const struct Teddy * teddy = (const struct Teddy *)fdr;"
        print "    const m128 * maskBase = (const m128 *)((const u8 *)fdr + sizeof(struct Teddy));"
        print "    const u32 * confBase = (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) + (%d*32));" % self.num_masks
        print "    const u8 * mainStart = ROUNDUP_PTR(ptr, 16);"
        print "    const size_t iterBytes = %d;" % (self.num_iterations * 16)

        print '    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\\n",' \
                                ' buf, len, a->start_offset);'
        print '    DEBUG_PRINTF("derive: ptr: %p mainstart %p\\n", ptr,' \
                                ' mainStart);'

        for x in range(self.num_masks):
            if (x != 0):
                print "    m128 res_old_%d = ones128();" % x
        print "    m128 lomask = set16x8(0xf);"

        print "    if (ptr < mainStart) {"
        print "         ptr = mainStart - 16;"
        self.produce_one_iteration(0, 1, cautious = True, confirmCautious = True, save_old = True)
        print "         ptr += 16;"
        print "    }"

        print "    if (ptr + 16 < buf + len) {"
        self.produce_one_iteration(0, 1, cautious = False, confirmCautious = True, save_old = True)
        print "         ptr += 16;"
        print "    }"

        print "    for ( ; ptr + iterBytes <= buf + len; ptr += iterBytes) {"
        print "        __builtin_prefetch(ptr + (iterBytes*4));"
        print self.produce_flood_check()

        for iter in range(self.num_iterations):
            self.produce_one_iteration(iter, self.num_iterations, cautious = False, confirmCautious = False)

        print "    }"

        print "    for (; ptr < buf + len; ptr += 16) {"
        self.produce_one_iteration(0, 1, cautious = True, confirmCautious = True, save_old = True)
        print "    }"

        print self.produce_footer()

    def produce_compile_call(self):
        packed_str = { False : "false", True : "true"}[self.packed]
        print "        { %d, %s, %d, %d, %s, %d, %d }," % (
            self.id, self.arch.target, self.num_masks, self.num_buckets, packed_str,
            self.conf_pull_back, self.conf_top_level_split)

    def get_name(self):
        if self.packed:
            pck_string = "_pck"
        else:
            pck_string = ""

        if self.num_buckets == 16:
            type_string = "_fat"
        else:
            type_string = ""

        return "fdr_exec_teddy_%s_msks%d%s%s" % (self.arch.name, self.num_masks, pck_string, type_string)

    def __init__(self, arch, packed = False, num_masks = 1, num_buckets = 8):
        self.arch = arch
        self.packed = packed
        self.num_masks = num_masks
        self.num_buckets = num_buckets
        self.num_iterations = 2

        if packed:
            self.conf_top_level_split = 32
        else:
            self.conf_top_level_split = 1
        self.conf_pull_back = 0

class MTFat(MT):
    def produce_needed_temporaries(self, max_iterations):
        print "        m256 p_mask;"
        for iter in range(0, max_iterations):
            print "        m256 val_%d;" % iter
            print "        m256 val_%d_lo;" % iter
            print "        m256 val_%d_hi;" % iter
            for x in range(self.num_masks):
                print "        m256 res_%d_%d;" % (iter, x)
                if x != 0:
                    print "        m256 res_shifted_%d_%d;" % (iter, x)
            print "        m256 r_%d;" % iter
            print "#ifdef ARCH_64_BIT"
            print "            u64a r_%d_part1;" % iter
            print "            u64a r_%d_part2;" % iter
            print "            u64a r_%d_part3;" % iter
            print "            u64a r_%d_part4;" % iter
            print "#else"
            print "            u32 r_%d_part1;" % iter
            print "            u32 r_%d_part2;" % iter
            print "            u32 r_%d_part3;" % iter
            print "            u32 r_%d_part4;" % iter
            print "            u32 r_%d_part5;" % iter
            print "            u32 r_%d_part6;" % iter
            print "            u32 r_%d_part7;" % iter
            print "            u32 r_%d_part8;" % iter
            print "#endif"

    def produce_code(self):
        print self.produce_header(visible = True, header_only = False)
        print self.produce_common_declarations()
        print

        self.produce_needed_temporaries(self.num_iterations)
        print

        print "    const struct Teddy * teddy = (const struct Teddy *)fdr;"
        print "    const m256 * maskBase = (const m256 *)((const u8 *)fdr + sizeof(struct Teddy));"
        print "    const u32 * confBase = (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) + (%d*32*2));" % self.num_masks
        print "    const u8 * mainStart = ROUNDUP_PTR(ptr, 16);"
        print "    const size_t iterBytes = %d;" % (self.num_iterations * 16)

        print '    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\\n",' \
                                ' buf, len, a->start_offset);'
        print '    DEBUG_PRINTF("derive: ptr: %p mainstart %p\\n", ptr,' \
                                ' mainStart);'

        for x in range(self.num_masks):
            if (x != 0):
                print "    m256 res_old_%d = ones256();" % x
        print "    m256 lomask = set32x8(0xf);"

        print "    if (ptr < mainStart) {"
        print "         ptr = mainStart - 16;"
        self.produce_one_iteration(0, 1, cautious = True, confirmCautious = True, save_old = True)
        print "         ptr += 16;"
        print "    }"

        print "    if (ptr + 16 < buf + len) {"
        self.produce_one_iteration(0, 1, cautious = False, confirmCautious = True, save_old = True)
        print "         ptr += 16;"
        print "    }"

        print "    for ( ; ptr + iterBytes <= buf + len; ptr += iterBytes) {"
        print "        __builtin_prefetch(ptr + (iterBytes*4));"
        print self.produce_flood_check()

        for iter in range(self.num_iterations):
            self.produce_one_iteration(iter, self.num_iterations, False, confirmCautious = False)

        print "    }"

        print "    for (; ptr < buf + len; ptr += 16) {"
        self.produce_one_iteration(0, 1, cautious = True, confirmCautious = True, save_old = True)
        print "    }"

        print self.produce_footer()

    def produce_one_iteration_state_calc(self, iter, effective_num_iterations,
                                         cautious, save_old):
        if cautious:
            print "        val_%d = vectoredLoad2x128(&p_mask, ptr + %d, buf, buf+len, a->buf_history, a->len_history, %d);" % (iter, iter*16, self.num_masks)
        else:
            print "        val_%d = load2x128(ptr + %d);" % (iter, iter*16)
        print "        val_%d_lo = and256(val_%d, lomask);" % (iter, iter)
        print "        val_%d_hi = rshift4x64(val_%d, 4);" % (iter, iter)
        print "        val_%d_hi = and256(val_%d_hi, lomask);" % (iter, iter)
        print
        for x in range(self.num_masks):
            print Template("""
        res_${ITER}_${X} = and256(vpshufb(maskBase[${X}*2]  , val_${ITER}_lo),
                                  vpshufb(maskBase[${X}*2+1], val_${ITER}_hi));""").substitute(ITER = iter, X = x)
            if x != 0:
                if iter == 0:
                    print "        res_shifted_%d_%d = vpalignr(res_%d_%d, res_old_%d, 16-%d);" % (iter, x,   iter, x,         x,   x)
                else:
                    print "        res_shifted_%d_%d = vpalignr(res_%d_%d, res_%d_%d, 16-%d);" % (iter, x,    iter, x, iter-1, x,   x)
            if x != 0 and iter == effective_num_iterations - 1 and save_old:
                print "        res_old_%d = res_%d_%d;" % (x, iter, x)
        print
        if cautious:
            print "        r_%d = and256(res_%d_0, p_mask);" % (iter, iter)
        else:
            print "        r_%d = res_%d_0;" % (iter, iter)
        for x in range(1, self.num_masks):
            print "        r_%d = and256(r_%d, res_shifted_%d_%d);" % (iter, iter, iter, x)
        print

    def produce_one_iteration_confirm(self, iter, confirmCautious):
        setup64 = [ (0, "r_%d_part1" % iter, "extractlow64from256(r)"),
                    (4, "r_%d_part2" % iter, "extract64from256(r, 1);\n            r = interleave256hi(r_%d, r_swap)" % (iter)),
                    (8, "r_%d_part3" % iter, "extractlow64from256(r)"),
                    (12, "r_%d_part4" % iter, "extract64from256(r, 1)") ]

        setup32 = [ (0, "r_%d_part1" % iter, "extractlow32from256(r)"),
                    (2, "r_%d_part2" % iter, "extract32from256(r, 1)"),
                    (4, "r_%d_part3" % iter, "extract32from256(r, 2)"),
                    (6, "r_%d_part4" % iter, "extract32from256(r, 3);\n            r = interleave256hi(r_%d, r_swap)" % (iter)),
                    (8, "r_%d_part5" % iter, "extractlow32from256(r)"),
                    (10, "r_%d_part6" % iter, "extract32from256(r, 1)"),
                    (12, "r_%d_part7" % iter, "extract32from256(r, 2)"),
                    (14, "r_%d_part8" % iter, "extract32from256(r, 3)") ]

        print "        if (P0(isnonzero256(r_%d))) {" % (iter)
        print "            m256 r_swap = swap128in256(r_%d);" % (iter)
        print "            m256 r = interleave256lo(r_%d, r_swap);" % (iter)
        print "#ifdef ARCH_64_BIT"
        for (off, val, init) in setup64:
            print "            %s = %s;" % (val, init)

        for (off, val, init) in setup64:
            self.produce_confirm(iter, val, off, 64, cautious = confirmCautious)
        print "#else"
        for (off, val, init) in setup32:
            print "            %s = %s;" % (val, init)

        for (off, val, init) in setup32:
            self.produce_confirm(iter, val, off, 32, cautious = confirmCautious)
        print "#endif"
        print "        }"

class MTFast(MatcherBase):

    def produce_confirm(self, cautious):
        if cautious:
            cautious_str = "VECTORING"
        else:
            cautious_str = "NOT_CAUTIOUS"

        print "            for (u32 i = 0; i < arrCnt; i++) {"
        print "                byte = bitArr[i] / 8;"
        if self.packed:
            conf_split_mask = IntegerType(32).constant_to_string(
                                self.conf_top_level_split - 1)
            print "                bitRem  = bitArr[i] % 8;"
            print "                confSplit = *(ptr+byte) & 0x1f;"
            print "                idx = confSplit * %d + bitRem;" % self.num_buckets
            print "                cf = confBase[idx];"
            print "                if (!cf)"
            print "                    continue;"
            print "                fdrc = (const struct FDRConfirm *)((const u8 *)confBase + cf);"
            print "                if (!(fdrc->groups & *control))"
            print "                    continue;"
            print "                confWithBit(fdrc, a, ptr - buf + byte, %s, 0, control, &last_match);" % cautious_str
        else:
            print "                cf = confBase[bitArr[i] % 8];"
            print "                fdrc = (const struct FDRConfirm *)((const u8 *)confBase + cf);"
            print "                confWithBit1(fdrc, a, ptr - buf + byte, %s, control, &last_match);" % cautious_str
        print "                if (P0(controlVal == HWLM_TERMINATE_MATCHING)) {"
        print "                    *a->groups = controlVal;"
        print "                    return HWLM_TERMINATED;"
        print "                }"
        print "            }"

    def produce_needed_temporaries(self, max_iterations):
        print "        u32 arrCnt;"
        print "        u16 bitArr[512];"
        print "        m256 p_mask;"
        print "        m256 val_0;"
        print "        m256 val_0_lo;"
        print "        m256 val_0_hi;"
        print "        m256 res_0;"
        print "        m256 res_1;"
        print "        m128 lo_part;"
        print "        m128 hi_part;"
        print "#ifdef ARCH_64_BIT"
        print "        u64a r_0_part;"
        print "#else"
        print "        u32 r_0_part;"
        print "#endif"

    def produce_bit_scan(self, offset, bits):
        print "                while (P0(!!r_0_part)) {"
        if bits == 64:
            print "                    bitArr[arrCnt++] = (u16)findAndClearLSB_64(&r_0_part) + 64 * %d;" % (offset)
        else:
            print "                    bitArr[arrCnt++] = (u16)findAndClearLSB_32(&r_0_part) + 32 * %d;" % (offset)
        print "                }"

    def produce_bit_check_128(self, var_name, offset):
        print "            if (P0(isnonzero128(%s))) {" % (var_name)
        print "#ifdef ARCH_64_BIT"
        print "                r_0_part = movq(%s);" % (var_name)
        self.produce_bit_scan(offset, 64)
        print "                r_0_part = movq(byteShiftRight128(%s, 8));" % (var_name)
        self.produce_bit_scan(offset + 1, 64)
        print "#else"
        print "                r_0_part = movd(%s);" % (var_name)
        self.produce_bit_scan(offset * 2, 32)
        for step in range(1, 4):
            print "                r_0_part = movd(byteShiftRight128(%s, %d));" % (var_name, step * 4)
            self.produce_bit_scan(offset * 2 + step, 32)
        print "#endif"
        print "            }"

    def produce_bit_check_256(self, iter, single_iter, cautious):
        print "        if (P0(isnonzero256(res_%d))) {" % (iter)
        if single_iter:
            print "            arrCnt = 0;"
        print "            lo_part = cast256to128(res_%d);" % (iter)
        print "            hi_part = cast256to128(swap128in256(res_%d));" % (iter)
        self.produce_bit_check_128("lo_part", iter * 4)
        self.produce_bit_check_128("hi_part", iter * 4 + 2)
        if single_iter:
            self.produce_confirm(cautious)
        print "        }"

    def produce_one_iteration_state_calc(self, iter, cautious):
        if cautious:
            print "        val_0 = vectoredLoad256(&p_mask, ptr + %d, buf+a->start_offset, buf+len, a->buf_history, a->len_history);" % (iter * 32)
        else:
            print "        val_0 = load256(ptr + %d);" % (iter * 32)
        print "        val_0_lo = and256(val_0, lomask);"
        print "        val_0_hi = rshift4x64(val_0, 4);"
        print "        val_0_hi = and256(val_0_hi, lomask);"
        print "        res_%d = and256(vpshufb(maskLo  , val_0_lo), vpshufb(maskHi, val_0_hi));" % (iter)
        if cautious:
            print "        res_%d = and256(res_%d, p_mask);" % (iter, iter)

    def produce_code(self):
        print self.produce_header(visible = True, header_only = False)
        print self.produce_common_declarations()
        print

        self.produce_needed_temporaries(self.num_iterations)

        print "    const struct Teddy * teddy = (const struct Teddy *)fdr;"
        print "    const m128 * maskBase = (const m128 *)((const u8 *)fdr + sizeof(struct Teddy));"
        print "    const m256 maskLo = set2x128(maskBase[0]);"
        print "    const m256 maskHi = set2x128(maskBase[1]);"
        print "    const u32 * confBase = (const u32 *)((const u8 *)teddy + sizeof(struct Teddy) + 32);"
        print "    const u8 * mainStart = ROUNDUP_PTR(ptr, 32);"
        print "    const size_t iterBytes = %d;" % (self.num_iterations * 32)

        print '    DEBUG_PRINTF("params: buf %p len %zu start_offset %zu\\n",' \
                                ' buf, len, a->start_offset);'
        print '    DEBUG_PRINTF("derive: ptr: %p mainstart %p\\n", ptr,' \
                                ' mainStart);'
        print "    const m256 lomask = set32x8(0xf);"

        print "    if (ptr < mainStart) {"
        print "        ptr = mainStart - 32;"
        self.produce_one_iteration_state_calc(iter = 0, cautious = True)
        self.produce_bit_check_256(iter = 0, single_iter = True, cautious = True)
        print "        ptr += 32;"
        print "    }"

        print "    if (ptr + 32 < buf + len) {"
        self.produce_one_iteration_state_calc(iter = 0, cautious = False)
        self.produce_bit_check_256(iter = 0, single_iter = True, cautious = True)
        print "        ptr += 32;"
        print "    }"
        print "    for ( ; ptr + iterBytes <= buf + len; ptr += iterBytes) {"
        print "        __builtin_prefetch(ptr + (iterBytes*4));"
        print self.produce_flood_check()
        for iter in range (0, self.num_iterations):
            self.produce_one_iteration_state_calc(iter = iter, cautious = False)
        print "        arrCnt = 0;"
        for iter in range (0, self.num_iterations):
            self.produce_bit_check_256(iter = iter, single_iter = False, cautious = False)
        self.produce_confirm(cautious = False)
        print "    }"

        print "    for (; ptr < buf + len; ptr += 32) {"
        self.produce_one_iteration_state_calc(iter = 0, cautious = True)
        self.produce_bit_check_256(iter = 0, single_iter = True, cautious = True)
        print "    }"

        print self.produce_footer()

    def get_name(self):
        if self.packed:
            pck_string = "_pck"
        else:
            pck_string = ""
        return "fdr_exec_teddy_%s_msks%d%s_fast" % (self.arch.name, self.num_masks, pck_string)

    def produce_compile_call(self):
        packed_str = { False : "false", True : "true"}[self.packed]
        print "        { %d, %s, %d, %d, %s, %d, %d }," % (
            self.id, self.arch.target, self.num_masks, self.num_buckets, packed_str,
            self.conf_pull_back, self.conf_top_level_split)

    def __init__(self, arch, packed = False):
        self.arch = arch
        self.packed = packed
        self.num_masks = 1
        self.num_buckets = 8
        self.num_iterations = 2

        self.conf_top_level_split = 1
        self.conf_pull_back = 0
        if packed:
            self.conf_top_level_split = 32
        else:
            self.conf_top_level_split = 1
        self.conf_pull_back = 0
