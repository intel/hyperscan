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

class OrStep(Step):
    def __init__(self, context, offset, width):
        Step.__init__(self, context, offset)
        s_var = self.gv("st%d" % offset)
        if width < 128:
            self.val = "s |= %s;" % s_var.name
        else:
            self.val = "s = or%d(s, %s);" % (width, s_var.name)

class ShiftStateStep(Step):
    def __init__(self, context, offset = 0, stride_used = 1):
        Step.__init__(self, context, offset)
        m = self.matcher
        state = m.state_variable
        shift_distance = -1 * stride_used * m.num_buckets
        self.val = "%s = %s;" % (state.name, state.type.shift_expr(state.name, shift_distance))

class BulkLoadStep(Step):
    def __init__(self, context, offset, size, define_var = True, aligned = True):
        Step.__init__(self, context, offset)
        m = self.matcher
        self.latency = 4
        blt = m.bulk_load_type
        if aligned:
            init_string = blt.load_expr_data(self.offset, code = "aligned")
        else:
            init_string = blt.load_expr_data(self.offset)

        var_name = "current_data_%d" % offset
        if define_var:
            lb_var = self.nv(blt, var_name)
            self.val = lb_var.gen_initializer_stmt(init_string)
        else:
            lb_var = self.gv(var_name, reader = False, writer = True)
            self.val = "%s = %s;" % (var_name, init_string)

class ValueExtractStep(Step):
    def __init__(self, context, offset, sub_load_cautious = False):
        Step.__init__(self, context, offset)
        m = self.matcher
        self.latency = 2
        dsb = m.datasize_bytes
        modval = offset % dsb

        if m.domain > 8 and modval == dsb - 1:
            # Case 1: reading more than one byte over the end of the bulk load

            self.latency = 4
            if sub_load_cautious:
                code_string = "cautious_forward" 
            else:
                code_string = "normal"
            load_string = m.single_load_type.load_expr_data(self.offset, code_string)
            temp_string = "(%s << %d)" % (load_string, m.reach_shift_adjust)
        else:
            # Case 2: reading a value that can be found entirely in the current register
            if m.fdr2_force_naive_load:
                load_string = m.single_load_type.load_expr_data(self.offset, "normal")
                temp_string = "(%s << %d)" % (load_string, m.reach_shift_adjust)
            else:
                lb_var = self.gv("current_data_%d" % (offset - modval))
                if modval == 0:
                    # Case 2a: value is at LSB end of the register and must be left-
                    # shifted into place if there is a "reach_shift_adjust" required
                    temp_string = "(%s << %d)" % (lb_var.name, m.reach_shift_adjust)
                else:
                    # Case 2b: value is in the middle of the register and will be
                    # right-shifted into place (adjusted by "reach_shift_adjust")
                    temp_string = "(%s >> %d)" % (lb_var.name, modval*8 - m.reach_shift_adjust)


        init_string = "(%s) & 0x%x" % (temp_string, m.reach_mask)
        v_var = self.nv(m.value_extract_type, "v%d" % offset)
        self.val = v_var.gen_initializer_stmt(init_string)

class TableLookupStep(Step):
    def __init__(self, context, reach_multiplier, offset = 0):
        Step.__init__(self, context, offset)
        m = self.matcher
        self.latency = 4
        v_var = self.gv("v%d" % offset)
        s_var = self.nv(m.state_type, "st%d" % offset)
        init_string = "*(const %s *)(ft + %s*%dU)" % ( m.state_type.get_name(),
                                                       v_var.name, reach_multiplier)
        self.val = s_var.gen_initializer_stmt(init_string)

class ShiftReachMaskStep(Step):
    def __init__(self, context, offset):
        Step.__init__(self, context, offset)
        m = self.matcher
        extr = m.extract_frequency
        modval = offset % extr
        s_var = self.gv("st%d" % offset, writer = True)
        self.val = "%s = %s;" % (s_var.name, s_var.type.shift_expr(s_var.name, modval * m.num_buckets))

class ConfExtractStep(Step):
    def __init__(self, context, offset):
        Step.__init__(self, context, offset)
        m = self.matcher
        if m.state_type.isSIMDOnIntel():
            self.latency = 2
        init_string = m.state_type.lowbit_extract_expr("s", m.extract_size)
        extr_var = self.nv(m.extr_type, "extr%d" % offset)
        self.val = extr_var.gen_initializer_stmt(init_string)

class ConfAccumulateStep(Step):
    def __init__(self, context, extract_offset, conf_offset, define_var = True):
        Step.__init__(self, context, extract_offset)
        m = self.matcher
        extr_var = self.gv("extr%d" % extract_offset)
        extr_var_cast = "((%s)%s)" % (m.conf_type.get_name(), extr_var.name)
        if extract_offset == conf_offset:
            # create conf_var as a straight copy of extr
            if define_var:
                conf_var = self.nv(m.conf_type, "conf%d" % conf_offset)
                self.val = conf_var.gen_initializer_stmt(extr_var_cast)
            else:
                conf_var = self.gv("conf%d" % conf_offset, writer = True, reader = True)
                self.val = "%s = %s;" % (conf_var.name, extr_var_cast)
        else:
            # shift extr_var and insert/OR it in conf_var
            conf_var = self.gv("conf%d" % conf_offset, writer = True, reader = True)
            shift_dist = (extract_offset - conf_offset) * m.num_buckets
            self.val = "%s |= %s;" % (conf_var.name, m.conf_type.shift_expr(extr_var_cast, shift_dist))
            self.latency = 2

class ConfirmFlipStep(Step):
    def __init__(self, context, offset):
        Step.__init__(self, context, offset)
        m = self.matcher
        conf_var = self.gv("conf%d" % self.offset, writer = True)
        self.val = "%s = %s;" % (conf_var.name,
                       conf_var.type.flip_lowbits_expr(conf_var.name, self.matcher.confirm_frequency * m.num_buckets))

class ConfirmStep(Step):
    def __init__(self, context, offset, cautious = False):
        Step.__init__(self, context, offset)
        m = self.matcher
        conf_var = self.gv("conf%d" % offset, writer = True)
        self.val = m.produce_confirm_base(conf_var.name, conf_var.type.size, offset, cautious,
                                          enable_confirmless = m.stride == 1, do_bailout = False)

class M3(MatcherBase):
    def get_hash_safety_parameters(self):
        h_size = self.single_load_type.size_in_bytes()
        return (0, h_size - 1)

    def produce_compile_call(self):
        print "    { %d, %d, %d, %d, %d, %s, %d, %d }," % (
              self.id, self.state_width, self.num_buckets,
              self.stride, self.domain,
              self.arch.target, self.conf_pull_back, self.conf_top_level_split)

    def produce_main_loop(self, switch_variant = False):
        stride_offsets = xrange(0, self.loop_bytes, self.stride)
        stride_offsetSet = set(stride_offsets)
        so_steps_last_block = []
        sh = None
        last_confirm = None
        ctxt = CodeGenContext(self)

        if switch_variant:
            print " ptr -= (iterBytes - dist);"
            print " { " # need an extra scope around switch variant to stop its globals escaping
        else:
            print "    if (doMainLoop) {"
            print "    for (; ptr + LOOP_READ_AHEAD < buf + len; ptr += iterBytes) {"
            print self.produce_flood_check()
            print "        __builtin_prefetch(ptr + (iterBytes*4));"
            print "        assert(((size_t)ptr % START_MOD) == 0);"


        # just do globally for now
        if switch_variant:
            subsidiary_load_cautious = True
            confirm_cautious = True
        else:
            subsidiary_load_cautious = False
            confirm_cautious = False

        if not self.fdr2_force_naive_load:
            bulk_load_steps = [ off for off in range(self.loop_bytes)
                                if off % self.datasize_bytes == 0 and
                                   (set(range(off, off + self.datasize_bytes - 1)) & stride_offsetSet)]
        else:
            bulk_load_steps = []

        confirm_steps = [ off for off in range(self.loop_bytes) if off % self.confirm_frequency == 0 ]

        for off in bulk_load_steps:
            lb_var = ctxt.new_var(None, self.bulk_load_type, "current_data_%d" % off)
            print "        " + lb_var.gen_initializer_stmt()


        for off in confirm_steps:
            var_name = "conf%d" % off
            conf_def_var = ctxt.new_var(None, self.conf_type, var_name)
            if switch_variant:
                init_string = "(%s)-1" % self.conf_type.get_name()
            else:
                init_string = ""
            print "        " + conf_def_var.gen_initializer_stmt(init_string)

        if switch_variant:
            print "        switch(iterBytes - dist) {"
            for i in range(0, self.loop_bytes):
                print "            case %d:" % i

                # init and poison conf; over-precise but harmless
                conf_id = (i / self.confirm_frequency) * self.confirm_frequency
                if i % self.confirm_frequency:
                    conf_fixup_bits = self.conf_type.size - (self.num_buckets * (i % self.confirm_frequency))
                    print "                conf%d >>= %d;" % (conf_id, conf_fixup_bits)
                else:
                    print "                conf%d = 0;" % conf_id

                # init state
                state_fixup = i % self.extract_frequency
                state = self.state_variable
                shift_distance = self.num_buckets * state_fixup
                if state_fixup:
                    print "                %s = %s;" % (state.name, state.type.shift_expr(state.name, shift_distance))
                    if self.state_width < 128:
                        print "                %s |= %s;" % (state.name, state.type.lowbit_mask(shift_distance))
                    else:
                        print "                %s = or%d(%s, %s);" % (state.name, self.state_width, state.name, state.type.lowbit_mask(shift_distance))

                if not self.fdr2_force_naive_load:
                    # init current_data (could poison it in some cases)
                    load_mod = i % self.datasize_bytes
                    load_offset = i - load_mod
                    if load_mod:
                        # not coming in on an even boundary means having to do a load var
                        # actually, there are a bunch of things we can do on this bulk load
                        # to avoid having to be 'cautious_backwards' but I'm not completely
                        # sure they are good ideas
                        init_string = self.bulk_load_type.load_expr_data(load_offset,
                                                                         code = "cautious_backward")
                        var_name = "current_data_%d" % load_offset
                        lb_var = ctxt.get_var(None, var_name, reader = False, writer = True)
                        print "                %s = %s;" % (lb_var.name, init_string)

                print "                goto off%d;" % i
            print "            case %d: goto skipSwitch;" % self.loop_bytes
            print "        }"
            print "        {"


        for off in range(self.loop_bytes):
            # X_mod is the offset we're up to relative to the last X operation
            # X_offset is which of the last X operations matches this iteration

            if (switch_variant):
                LabelStep(ctxt, off)

            if off in bulk_load_steps:
                if not self.fdr2_force_naive_load:
                    BulkLoadStep(ctxt, off, self.datasize, define_var = False, aligned = not switch_variant)

            if off in stride_offsets:
                if switch_variant:
                    OpenScopeStep(ctxt, off)
                ValueExtractStep(ctxt, off, sub_load_cautious = subsidiary_load_cautious)
                TableLookupStep(ctxt, self.reach_mult, off)
                if off % self.extract_frequency:
                    ShiftReachMaskStep(ctxt, off)
                so = OrStep(ctxt, off, self.state_width)
                if switch_variant:
                    CloseScopeStep(ctxt, off)
                if sh != None:
                    so.add_dependency(sh)
                so_steps_last_block += [ so ]

            extract_mod = off % self.extract_frequency
            extract_offset = off - extract_mod
            extract_ready = extract_mod == self.extract_frequency - 1
            if extract_ready:
                if switch_variant:
                    OpenScopeStep(ctxt, off)
                ex = ConfExtractStep(ctxt, extract_offset)
                ConfAccumulateStep(ctxt, extract_offset, confirm_offset, define_var = False)
                for so_step in so_steps_last_block:
                    ex.add_dependency(so_step)
                if switch_variant:
                    CloseScopeStep(ctxt, off)
                so_steps_last_block = []
                sh = ShiftStateStep(ctxt, extract_offset, stride_used = self.extract_frequency)
                sh.add_dependency(ex)

            confirm_mod = off % self.confirm_frequency
            confirm_offset = off - confirm_mod
            confirm_ready = confirm_mod == self.confirm_frequency - 1
            if confirm_ready:
                cflip = ConfirmFlipStep(ctxt, confirm_offset)
                cf = ConfirmStep(ctxt, confirm_offset, cautious = confirm_cautious )
                if last_confirm:
                    cf.add_dependency(last_confirm)
                last_confirm = cf


        if not switch_variant:
            print ctxt.schedule([ last_confirm, sh ])
        else:
            print ctxt.dontschedule([ last_confirm, sh ])

        if switch_variant:
            print "skipSwitch:;"
            print "    ptr += iterBytes;"
        print "    }" # close extra scope around switch variant
        print "    }"


    def produce_init_state(self):
        state = self.state_variable
        s_type = self.state_type
        shift_distance = -1 * self.num_buckets
        shift_expr = "%s = %s" % (state.name, state.type.shift_expr(state.name, shift_distance))

        s = Template("""
            $TYPENAME s;
            if (a->len_history) {
                u32 tmp = getPreStartVal(a, $DOMAIN);
                s = *((const $TYPENAME *)ft + tmp);
                $SHIFT_EXPR;
            } else {
                s = *(const $TYPENAME *)&fdr->start;
            }
""").substitute(TYPENAME = s_type.get_name(),
                ZERO_EXPR = s_type.zero_expression(),
                DOMAIN = self.domain,
                SHIFT_EXPR = shift_expr)
        return s

    def produce_code(self):

        (behind, ahead) = self.get_hash_safety_parameters()
        loop_read_behind = behind
        loop_read_ahead = self.loop_bytes + ahead

        # we set up mask and shift stuff for extracting our masks from registers
        #
        # we have a choice as to whether to mask out the value early or
        # extract the value (shift first) then mask it
        #
        # Intel has a free scaling factor from 1/2/4/8 so we want to combine
        # the extra needed shift for SSE registers with the mask operation

        ssb = self.state_type.size / 8 # state size in bytes

        # Intel path
        if ssb == 16 and self.domain == 16:
            # obscure corner - we don't have the room in the register to
            # do this for all values so we don't. domain==16 is pretty
            # bad anyhow, of course
            self.reach_mult = 8
        else:
            self.reach_mult = ssb

        shift_amts = { 1 : 0, 2 : 1, 4 : 2, 8 : 3, 16: 4 }
        self.reach_shift_adjust = shift_amts[ ssb/self.reach_mult ]
        self.reach_mask = ((1 << self.domain) - 1) << self.reach_shift_adjust

        print self.produce_header(visible = False)

        print "// ",
        print " Arch: " + self.arch.name,
        print " State type: " + self.state_type.get_name(),
        print " Num buckets: %d" % self.num_buckets,
        print " Domain: %d" % self.domain,
        print " Stride: %d" % self.stride

        print self.produce_common_declarations()
        print

        print "\tconst size_t tabSize = %d;" % self.table_size
        print """
    const u8 * ft = (const u8 *)fdr + ROUNDUP_16(sizeof(struct FDR));
    const u32 * confBase = (const u32 *)(ft + tabSize);
"""
        print self.produce_init_state()
        print "\tconst size_t iterBytes = %d;" % self.loop_bytes
        print "\tconst size_t START_MOD = %d;" % self.datasize_bytes
        print "\tconst size_t LOOP_READ_AHEAD = %d;" % loop_read_ahead

        print """
    while (ptr < buf + len) {

        u8 doMainLoop = 1;
        size_t remaining = len - (ptr - buf);
        size_t dist;
        if (remaining <= iterBytes) {
            dist = remaining; // once through the switch and we're done
        } else if (remaining < 2 * iterBytes) {
            // nibble some stuff off the front, skip the main loop,
            // then come back here
            dist = iterBytes;  // maybe could be cleverer
        } else {
            // now, we need to see if we can make it to a main loop iteration
            // if so, we need to ensure that the main loop iteration is aligned
            // to a START_MOD boundary and i >= 8 so we can read ptr + i - 8

            // see if we can do it - if not, just switch the main loop off,
            // eat iterBytes in cautious mode, and come back to this loop

            const u8 * target = MAX(buf + 8, ptr);
            target = ROUNDUP_PTR(target, START_MOD);
            dist = target - ptr;
            if (dist > iterBytes) {
                doMainLoop = 0;
                dist = iterBytes;
            }
        }
"""
        self.produce_main_loop(switch_variant = True)
        self.produce_main_loop(switch_variant = False)
        print """
    }
"""
        print self.produce_footer()

    def get_name(self):
        return "fdr_exec_%s_d%d_s%d_w%d" % (self.arch.name, self.domain, self.stride, self.state_width)

    def __init__(self, state_width, domain, stride,
                 arch,
                 table_state_width = None,
                 num_buckets = 8,
                 extract_frequency = None,
                 confirm_frequency = None):

        # First - set up the values that are fundamental to how this matcher will operate
        self.arch = arch

        # get the width of the state width on which we operate internally
        if state_width not in [ 128 ]:
            fail_out("Unknown state width: %d" % state_width)
        self.state_width = state_width
        self.state_type = getRequiredType(self.state_width)
        self.state_variable = IntegerVariable("s", self.state_type)

        table_state_width = state_width
        self.table_state_width = state_width
        self.table_state_type = getRequiredType(self.table_state_width)

        # domain is the number of bits that we draw from our input to
        # index our 'reach' table
        if not 8 <= domain <= 16:
            fail_out("Unsupported domain: %d" % domain)
        self.domain = domain
        # this is the load type required for this domain if we want to
        # load it one at a time
        self.single_load_type = getRequiredType(self.domain)

        # table size
        self.table_size = 2**domain * table_state_width // 8

        # stride is the frequency with which we make data-driven
        # accesses to our reach table
        if stride not in [ 1, 2, 4, 8]:
            fail_out("Unsupported stride: %d" % stride)
        if stride * num_buckets > state_width:
            fail_out("Stride %d is too big for the number of buckets %d given state width %d\n" % (stride, num_buckets, state_width))
        self.stride = stride

        if num_buckets != 8:
            fail_out("Unsupported number of buckets: %d" % num_buckets)
        if state_width % num_buckets and state_width == 128:
            fail_out("Bucket scheme requires bit-shifts on m128 (failing)")
        self.num_buckets = num_buckets

        # Second - set up derived or optimization values - these can be
        # overridden by arguments that are passed in

        self.datasize = 64
        self.bulk_load_type = IntegerType(self.datasize)
        self.datasize_bytes = self.datasize/8

        self.value_extract_type = IntegerType(self.datasize)

        self.fdr2_force_naive_load = False # disable everywhere for trunk

        # extract frequency is how frequently (in bytes) we destructively shift
        # our state value after having pulled out that many bytes into a
        # confirm register (of one sort or another).
        # none means a default value - datasize, our biggest easily available GPR
        if extract_frequency is None:
            extract_frequency = self.datasize_bytes
        self.extract_frequency = extract_frequency
        self.extract_size = self.extract_frequency*self.num_buckets
        if extract_frequency < stride:
            fail_out("Can't extract at extract frequency %d with stride %d" % (extract_frequency, stride))
        if extract_frequency not in [ None, 1, 2, 4, 8, 16]:
            fail_out("Weird extract frequency: %d" % extract_frequency)

        if self.extract_size <= 32:
            self.extr_type = IntegerType(32)
        elif self.extract_size <= 64:
            self.extr_type = IntegerType(64)
        else:
            fail_out("Implausible size %d required for confirm extract step" % size)

        # extract_frequency is how often we pull out our state and place
        # it somewhere in a lossless fashion
        # confirm_frequency, on the other hand, is how frequently we
        # take the state extracted by extract_frequency and cobble it
        # together into a matching loop
        # confirm_frequency must be a multiple of extract_frequency
        # and must fit into a fast register; for now; we're going to
        # stay in the GPR domain
        if confirm_frequency is None:
            confirm_frequency = self.extract_frequency
        self.confirm_frequency = confirm_frequency
        if confirm_frequency % self.extract_frequency:
            fail_out("Confirm frequency %d must be evenly divisible by extract_frequency %d" % (confirm_frequency, self.extract_frequency))

        self.conf_size = self.confirm_frequency * self.num_buckets
        if self.conf_size <= 32:
            self.conf_type = IntegerType(32)
        elif self.conf_size <= 64:
            self.conf_type = IntegerType(64)
        else:
            fail_out("Implausible size %d required for confirm accumulate step" % self.conf_size)

        # how many bytes in flight at once
        self.loop_bytes = 16

        # confirm configuration

        # how many entries in the top-level confirm table - 256 means
        # complete split on the last character
        self.conf_top_level_split = 256

        # how much we 'pull back' in confirm - this is obviously related
        # to the first level conf but we will keep two separate paramters
        # for this to avoid the risk of conflating these
        self.conf_pull_back = 1

        if self.conf_pull_back > 0 and self.conf_top_level_split < 256:
            fail_out("Pull back distance %d not supported by top level split %d" % (self.conf_pull_back, self.conf_top_level_split))

        # minor stuff
        self.default_body_indent = 8
