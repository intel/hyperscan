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

    def isSIMDOnIntel(self):
        return False

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

    def highbit_mask(self, n):
        return self.constant_to_string(self.highbits(n))

    def lowbit_extract_expr(self, expr_string, n):
         return "(%s & %s)" % ( expr_string, self.lowbit_mask(n))

    def highbit_extract_expr(self, expr_string, n):
        return "(%s >> %d)" % (expr_string, self.size - n)

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

    # code is:
    # "normal" (always between buf and len) - the default
    # "aligned" (means normal + aligned to a natural boundary)
    # "cautious_forward" (means may go off the end of buf+len)
    # "cautious_backwards" (means may go off the start of buf)
    # "cautious_everywhere" (means may go off both)

    def load_expr_data(self, offset = 0, code = "normal",
                       base_string = "ptr", bounds_lo = "buf", bounds_hi = "buf + len"):
        if code is "normal":
            return "lv_%s(%s + %d, %s, %s)" % (self.get_name(), base_string, offset, bounds_lo, bounds_hi)
        elif code is "aligned":
            if self.size is 8:
                fail_out("no aligned byte loads")
            return "lv_%s_a(%s + %d, %s, %s)" % (self.get_name(), base_string, offset, bounds_lo, bounds_hi)
        elif code is "cautious_forward":
            return "lv_%s_cf(%s + %d, %s, %s)" % (self.get_name(), base_string, offset, bounds_lo, bounds_hi)
        elif code is "cautious_backward":
            return "lv_%s_cb(%s + %d, %s, %s)" % (self.get_name(), base_string, offset, bounds_lo, bounds_hi)
        elif code is "cautious_everywhere":
            return "lv_%s_ce(%s + %d, %s, %s)" % (self.get_name(), base_string, offset, bounds_lo, bounds_hi)


class SIMDIntegerType(IntegerType):
    def __init__(self, size):
        IntegerType.__init__(self, size)

    def isSIMDOnIntel(self):
        return True

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

    def highbit_extract_expr(self, expr_string, n):
        fail_out("Unimplemented high bit extract on m128")

    def bit_extract_expr(self, expr_string, low, high, flip):
        fail_out("Unimplemented bit extract on m128")

    def shift_expr(self, expr_string, n):
        if n % 8 != 0:
            fail_out("Trying to shift a m128 by a bit granular value")

        # should check that n is divisible by 8
        if n <= -self.size or n >= self.size:
            return self.zero_expression()
        elif (n > 0):
            return "_mm_slli_si128(%s, %s)" % (expr_string, n / 8)
        elif (n < 0):
            return "_mm_srli_si128(%s, %s)" % (expr_string, -n / 8)
        else:
            return "(%s)" % (expr_string)

    def lowbit_mask(self, n):
        if n % 8 != 0:
            fail_out("Trying to make a lowbit mask in a m128 by a bit granular value")
        return self.shift_expr("ones128()", -(128 - n))

def getRequiredType(bits):
    if bits == 128:
        return SIMDIntegerType(bits)
    for b in [ 8, 16, 32, 64]:
        if (bits <= b):
            return IntegerType(b)
    return None

class IntegerVariable:
    def __init__(self, name, type):
        self.name = name
        self.type = type

    def gen_initializer_stmt(self, initialization_string = None):
        if initialization_string:
            return "%s %s = %s;" % (self.type.get_name(), self.name, initialization_string)
        else:
            return "%s %s;" % (self.type.get_name(), self.name)


class Step:
    def __init__(self, context, offset = 0):
        self.context = context
        self.matcher = context.matcher
        self.offset = offset
        self.latency = 1
        self.dependency_list = []
        self.latest = None
        self.context.add_step(self)

    # return a string, complete with indentation
    def emit(self):
        indent = " " * (self.offset*2 + self.matcher.default_body_indent)
        s = "\n".join( [ indent + line for line in self.val.split("\n")] )
        if self.latest:
            s += " // " + str(self.debug_step) + " L" + str(self.latency) + " LTST:%d" % self.latest
            if self.dependency_list:
                s += " Derps: "
                for (d,l) in self.dependency_list:
                    s += "%d/%d " % (d.debug_step,l)
        return s

    def add_dependency(self, step, anti_dependency = False, output_dependency = False):
        if anti_dependency or output_dependency:
            self.dependency_list += [ (step, 1) ]
        else:
            self.dependency_list += [ (step, step.latency) ]

    def nv(self, type, var_name):
        return self.context.new_var(self, type, var_name)

    def gv(self, var_name, reader = True, writer = False):
        return self.context.get_var(self, var_name, reader = reader, writer = writer)

# utility steps, generic

class LabelStep(Step):
    def __init__(self, context, offset = 0, label_prefix = "off"):
        Step.__init__(self, context, offset)
        self.val = "%s%d: UNUSED;" % (label_prefix, offset)

class OpenScopeStep(Step):
    def __init__(self, context, offset = 0):
        Step.__init__(self, context, offset)
        self.val = "{"

class CloseScopeStep(Step):
    def __init__(self, context, offset = 0):
        Step.__init__(self, context, offset)
        self.val = "}"


class CodeGenContext:
    def __init__(self, matcher):
        self.vars = {}
        self.steps = []
        self.ctr = 0
        self.matcher = matcher
        self.var_writer = {} # var to a single writer
        self.var_readers = {} # var to a list of all the readers that read the last value

    def new_var(self, step, type, var_name):
        var = IntegerVariable(var_name, type)
        self.vars[var_name] = var
        self.var_writer[var_name] = step
        return var

    def get_var(self, step, var_name, reader = True, writer = False):
        if reader:
            writer_step = self.var_writer[var_name]
            if writer_step:
                step.add_dependency(writer_step)
            self.var_readers.setdefault(var_name, []).append(step)
        if writer and not reader:
            if self.var_writer[var_name]:
                step.add_dependency(self.var_writer[var_name], output_dependency = True)
        if writer:
            if self.var_readers.has_key(var_name):
                for reader in [ r for r in self.var_readers[var_name] if r is not step ]:
                    step.add_dependency(reader, anti_dependency = True)
                self.var_readers[var_name] = []
            self.var_writer[var_name] = step
        return self.vars[var_name]

    def add_step(self, step):
        self.steps += [ step ]
        step.debug_step = self.ctr
        self.ctr += 1

    def dontschedule(self, finals):
        return "\n".join( [ s.emit() for s in self.steps ] )

    def schedule(self, finals):
        for f in finals:
            f.latest = f.latency
        worklist = finals
        while worklist:
            current = worklist[0]
            worklist = worklist[1:]
            for (dep, lat) in current.dependency_list:
                if dep.latest is None or dep.latest < (current.latest + dep.latency):
                    dep.latest = current.latest + lat
                    if dep not in worklist:
                        worklist += [ dep ]
        self.steps.sort(reverse = True, key = lambda s : s.latest)
        return "\n".join( [ s.emit() for s in self.steps ] )
