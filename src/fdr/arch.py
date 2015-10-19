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

import autogen_utils

# wrapper for architectures

class Arch:
    def __init__(self, name, extensions = []):
        self.name = name
        self.extensions = extensions
        self.target = None

    def get_guard(self):
        # these defines definitely fall into the "belt-and-suspenders"
        # category of paranoia
        if (self.guard_list == []):
            return "#if 1"

        return "#if " + " && ".join(self.guard_list)

class X86Arch(Arch):
    def __init__(self, name, extensions = []):
        Arch.__init__(self, name, extensions)
        self.guard_list = [ ]
        self.target = "0"

        if "AVX2" in extensions:
            self.target += " | HS_CPU_FEATURES_AVX2"
            self.guard_list += [ "defined(__AVX2__)" ]


arch_x86_64            = X86Arch("x86_64", extensions = [ ])
arch_x86_64_avx2       = X86Arch("x86_64_avx2", extensions = [ "AVX2" ])
