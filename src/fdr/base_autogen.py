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

class MatcherBase:

    def __init__(self):
        pass

    def get_name(self):
        return "fdr_exec_%03d" % self.id

    def produce_header(self, visible, header_only = False):
        s = ""
        if not visible:
            s += "static never_inline"
        s += """
hwlm_error_t %s(UNUSED const struct FDR *fdr,
                UNUSED const struct FDR_Runtime_Args * a)""" % self.get_name()
        if header_only:
            s += ";"
        else:
            s += "{"
        s += "\n"
        return s

    def produce_guard(self):
	print self.arch.get_guard()
    
    def produce_zero_alternative(self):
	print """
#else
#define %s 0
#endif
""" % self.get_name()

    # trivial function for documentation/modularity
    def close_guard(self):
	print "#endif"

    def produce_common_declarations(self):
        return """
    const u8 * buf = a->buf;
    const size_t len = a->len;
    const u8 * ptr = buf + a->start_offset;
    hwlmcb_rv_t controlVal = *a->groups;
    hwlmcb_rv_t * control = &controlVal;
    u32 floodBackoff = FLOOD_BACKOFF_START;
    const u8 * tryFloodDetect = a->firstFloodDetect;
    UNUSED u32 bit, bitRem, confSplit, idx;
    u32 byte, cf;
    const struct FDRConfirm *fdrc;
    u32 last_match = (u32)-1;
"""

    def produce_continue_check(self):
        return """if (P0(controlVal == HWLM_TERMINATE_MATCHING)) {
    *a->groups = controlVal;
    return HWLM_TERMINATED;
}
"""
    def produce_flood_check(self):
        return """
        if (P0(ptr > tryFloodDetect)) {
            tryFloodDetect = floodDetect(fdr, a, &ptr, tryFloodDetect, &floodBackoff, &controlVal, iterBytes);
            if (P0(controlVal == HWLM_TERMINATE_MATCHING)) {
                *a->groups = controlVal;
                return HWLM_TERMINATED;
            }
        }
"""

    def produce_footer(self):
        return """
    *a->groups = controlVal;
    return HWLM_SUCCESS;
}
"""

    def produce_confirm_base(self, conf_var_name, conf_var_size, offset, cautious, enable_confirmless, do_bailout = False):
        if cautious:
            caution_string = "VECTORING"
        else:
            caution_string = "NOT_CAUTIOUS"
        conf_split_mask = IntegerType(32).constant_to_string(
                            self.conf_top_level_split - 1)
        if enable_confirmless:
            quick_check_string = """
        if (!fdrc->mult) {
            u32 id = fdrc->nBitsOrSoleID;
            if ((last_match == id) && (fdrc->flags & NoRepeat))
                continue;
           last_match = id;
           controlVal = a->cb(ptr+byte-buf, ptr+byte-buf, id, a->ctxt);
           continue;
        } """
        else:
            quick_check_string = ""
        if do_bailout:
            bailout_string = """
        if ((ptr + byte < buf + a->start_offset) || (ptr + byte >= buf + len)) continue;"""
        else:
            bailout_string = ""

        return Template("""
if (P0(!!$CONFVAR)) {
    do  {
        bit = findAndClearLSB_$CONFVAR_SIZE(&$CONFVAR);
        byte  = bit / $NUM_BUCKETS + $OFFSET;
        bitRem  = bit % $NUM_BUCKETS;
        $BAILOUT_STRING
        confSplit = *(ptr+byte) & $SPLIT_MASK;
        idx = confSplit * $NUM_BUCKETS + bitRem;
        cf = confBase[idx];
        if (!cf)
            continue;
        fdrc = (const struct FDRConfirm *)((const u8 *)confBase + cf);
        if (!(fdrc->groups & *control))
            continue;
        $QUICK_CHECK_STRING
        confWithBit(fdrc, a, ptr - buf + byte, $CAUTION_STRING, $CONF_PULL_BACK, control, &last_match);
    } while(P0(!!$CONFVAR));
    if (P0(controlVal == HWLM_TERMINATE_MATCHING)) {
        *a->groups = controlVal;
        return HWLM_TERMINATED;
    }
}""").substitute(CONFVAR = conf_var_name,
                 CONFVAR_SIZE = conf_var_size,
                 NUM_BUCKETS = self.num_buckets,
                 OFFSET = offset,
                 SPLIT_MASK = conf_split_mask,
                 QUICK_CHECK_STRING = quick_check_string,
                 BAILOUT_STRING = bailout_string,
                 CAUTION_STRING = caution_string,
                 CONF_PULL_BACK = self.conf_pull_back)


def indent(block, depth):
    return "\n".join([ (" " * (4*depth)) + line for line in block.splitlines() ] )
