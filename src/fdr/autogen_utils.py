#!/usr/bin/python

# Copyright (c) 2015-2016, Intel Corporation
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

def fail_out(msg = ""):
    print >>sys.stderr, "Internal failure in autogen.py: " + msg
    sys.exit(1)

class IntegerType:
    def __init__(self, size):
        self.size = size

    def get_name(self):
        return { 256: "m256", 128 : "m128", 64 : "u64a", 32 : "u32" , 16 : "u16", 8 : "u8"}[self.size]

    def size_in_bytes(self):
        return self.size / 8

    def zero_expression(self):
        return "0"

    def constant_to_string(self, n):
        if self.size == 64:
            suffix = "ULL"
        else:
            suffix = ""
        return "0x%x%s" % (n & ((1 << self.size) - 1), suffix)

    def lowbits(self, n):
        return (1 << n) - 1

    def highbits(self, n):
        return ~(self.lowbits(self.size - n))

    def lowbit_mask(self, n):
        return self.constant_to_string(self.lowbits(n))

    def lowbit_extract_expr(self, expr_string, n):
         return "(%s & %s)" % ( expr_string, self.lowbit_mask(n))

    def flip_lowbits_expr(self, expr_string, n):
         return "(%s ^ %s)" % ( expr_string, self.lowbit_mask(n))

    def bit_extract_expr(self, expr_string, low, high):
        lbm = self.lowbit_mask(high - low)
        return "((%s >> %d) & %s)" % (expr_string, low, lbm)

    # shifts are +ve if left and -ve if right
    def shift_expr(self, expr_string, n):
        if n <= -self.size or n >= self.size:
            return self.zero_expression()
        elif (n > 0):
            return "(%s << %d)" % (expr_string, n)
        elif (n < 0):
            return "(%s >> %d)" % (expr_string, -n)
        else:
            return "(%s)" % (expr_string)

class SIMDIntegerType(IntegerType):
    def __init__(self, size):
        IntegerType.__init__(self, size)

    def zero_expression(self):
        return "zeroes128()"

    def lowbit_extract_expr(self, expr_string, n):
        if (n <= 32):
            tmpType = IntegerType(32)
            tmpExpr = "movd(%s)" % expr_string
        elif (32 < n <= 64):
            tmpType = IntegerType(64)
            tmpExpr = "movq(%s)" % expr_string
        return tmpType.lowbit_extract_expr(tmpExpr, n)

    def bit_extract_expr(self, expr_string, low, high, flip):
        fail_out("Unimplemented bit extract on m128")

    def shift_expr(self, expr_string, n):
        if n % 8 != 0:
            fail_out("Trying to shift a m128 by a bit granular value")

        # should check that n is divisible by 8
        if n <= -self.size or n >= self.size:
            return self.zero_expression()
        elif (n > 0):
            return "byteShiftLeft128(%s, %s)" % (expr_string, n / 8)
        elif (n < 0):
            return "byteShiftRight128(%s, %s)" % (expr_string, -n / 8)
        else:
            return "(%s)" % (expr_string)

    def lowbit_mask(self, n):
        if n % 8 != 0:
            fail_out("Trying to make a lowbit mask in a m128 by a bit granular value")
        return self.shift_expr("ones128()", -(128 - n))
