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

/** \file
 * \brief Peak heap usage code.
 *
 * At present, we only have an implementation for modern glibc systems, using
 * the malloc_info() call. We return zero elsewhere.
 */

#include "config.h"

#include "heapstats.h"

#if defined HAVE_MALLOC_INFO

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <malloc.h>

size_t getPeakHeap(void) {
    size_t fsize;
    char *fptr;
    FILE *fstr = open_memstream(&fptr, &fsize);
    if (!fstr) {
        return 0;
    }

    int rv = malloc_info(0, fstr);
    if (rv != 0) {
        fclose(fstr);
        free(fptr);
        return 0;
    }

    rewind(fstr);

    // We don't want to depend on a real XML parser. This is ugly and brittle
    // and hopefully good enough for the time being. We look for the last
    // system tag with type max, which should be the malloc-wide one.

    static const char begin[] = "<system type=\"max\" size=\"";
    const size_t begin_len = strlen(begin);

    char *line = nullptr;
    size_t len = 0, maxheap = 0;
    ssize_t read;

    while ((read = getline(&line, &len, fstr)) != -1) {
        if (strncmp(line, begin, begin_len) == 0) {
            errno = 0;
            maxheap = (size_t)strtoull(line + begin_len, nullptr, 10);
            if (errno != 0) {
                goto finish;
            }
        }
    }

finish:
    free(line);
    fclose(fstr);
    free(fptr);
    return maxheap;
}

#elif defined __linux

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

#include <sys/types.h>
#include <unistd.h>

using namespace std;

size_t getPeakHeap(void) {
    // Modern Linux kernels write a 'VmPeak' value into /proc/$PID/status. This
    // is a reasonable approximation, though it likely includes shared libs and
    // the like as well...
    ostringstream path;
    path << "/proc/" << getpid() << "/status";

    ifstream f(path.str().c_str());
    if (!f.good()) {
        return 0;
    }

    const string vmpeak("VmPeak:");

    string line;
    while (getline(f, line)) {
        istringstream iss(line, istringstream::in);
        string word;
        iss >> word;
        if (word != vmpeak) {
            continue;
        }

        // Skip spaces
        while (iss.good() && !isdigit(iss.peek())) {
            iss.ignore();
        }

        size_t num = 0;
        iss >> num;
        return num * 1024;
    }

    f.close();
    return 0;
}

#else

// Stub.
size_t getPeakHeap(void) {
    return 0;
}

#endif
