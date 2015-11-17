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
from fdr_autogen import *
from teddy_autogen import *
from arch import *

# FDR setup

# these are either produced - if the guard succeeds, or #defined to zeroes.
# either the function or the zero is fine in our array of function pointers

def produce_fdr_runtimes(l):
    for m in l:
        m.produce_code()

def produce_fdr_compiles(l):
    print "void getFdrDescriptions(vector<FDREngineDescription> *out) {"
    print "    static const FDREngineDef defns[] = {"
    for m in l:
        m.produce_compile_call()
    print "    };"
    print "    out->clear();"
    print "    for (size_t i = 0; i < ARRAY_LENGTH(defns); i++) {"
    print "        out->push_back(FDREngineDescription(defns[i]));"
    print "    }"
    print "}"

def build_fdr_matchers():
    all_matchers = [ ]
    strides = [ 1, 2, 4 ]

    common = { "state_width" : 128, "num_buckets" : 8, "extract_frequency" : 8, "arch" : arch_x86_64 }
    for s in strides:
        all_matchers += [ M3(stride = s, **common) ]

    return all_matchers

# teddy setup

def build_teddy_matchers():
    all_matchers = [ ]

    # AVX2
    all_matchers += [ MTFast(arch = arch_x86_64_avx2, packed = False) ]
    all_matchers += [ MTFast(arch = arch_x86_64_avx2, packed = True) ]
    for n_msk in range(1, 5):
        all_matchers += [ MTFat(arch = arch_x86_64_avx2, packed = False, num_masks = n_msk, num_buckets = 16) ]
        all_matchers += [ MTFat(arch = arch_x86_64_avx2, packed = True, num_masks = n_msk, num_buckets = 16) ]

    # SSE/SSE2/SSSE3
    for n_msk in range(1, 5):
        all_matchers += [ MT(arch = arch_x86_64, packed = False, num_masks = n_msk, num_buckets = 8) ]
        all_matchers += [ MT(arch = arch_x86_64, packed = True, num_masks = n_msk, num_buckets = 8) ]

    return all_matchers

def produce_teddy_compiles(l):
    print "void getTeddyDescriptions(vector<TeddyEngineDescription> *out) {"
    print "    static const TeddyEngineDef defns[] = {"
    for m in l:
        m.produce_compile_call()
    print "    };"
    print "    out->clear();"
    print "    for (size_t i = 0; i < ARRAY_LENGTH(defns); i++) {"
    print "        out->push_back(TeddyEngineDescription(defns[i]));"
    print "    }"
    print "}"

# see below - we don't produce our 'zeros' at the point of the teddy runtimes as they
# are linked. So we either generate the function or we don't - then at the point of the
# header in fdr_autogen.c we either generate the header or we #define the zero.

def produce_teddy_runtimes(l):
    # Since we're using -Wmissing-prototypes, we need headers first.
    for m in l:
	m.produce_guard()
        print m.produce_header(visible = True, header_only = True)
	m.close_guard()

    for m in l:
	m.produce_guard()
        m.produce_code()
	m.close_guard()

# see produce_teddy_runtimes() comment for the rationale

def produce_teddy_headers(l):
    for m in l:
	m.produce_guard()
        print m.produce_header(visible = True, header_only = True)
	m.produce_zero_alternative()

# general utilities

def make_fdr_function_pointers(matcher_list):
    print  """
typedef hwlm_error_t (*FDRFUNCTYPE)(const struct FDR *fdr, const struct FDR_Runtime_Args *a);
static FDRFUNCTYPE funcs[] = {
"""
    all_funcs = ",\n".join([ "    %s" % m.get_name() for m in matcher_list ])
    print all_funcs
    print """
};
"""

def assign_ids(matcher_list, next_id):
    for m in matcher_list:
        m.id = next_id
        next_id += 1
    return next_id

# Main entry point

m = build_fdr_matchers()
next_id = assign_ids(m, 0)
tm = build_teddy_matchers()
next_id = assign_ids(tm, next_id)
if sys.argv[1] == "compiler":
    produce_fdr_compiles(m)
elif sys.argv[1] == "runtime":
    produce_fdr_runtimes(m)
    produce_teddy_headers(tm)
    make_fdr_function_pointers(m+tm)
elif sys.argv[1] == "teddy_runtime":
    produce_teddy_runtimes(tm)
elif sys.argv[1] == "teddy_compiler":
    produce_teddy_compiles(tm)
