/*
 * Copyright (c) 2015, Intel Corporation
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

/*
 * Hyperscan example program 1: simplegrep
 *
 * This is a simple example of Hyperscan's most basic functionality: it will
 * search a given input file for a pattern supplied as a command-line argument.
 * It is intended to demonstrate correct usage of the hs_compile and hs_scan
 * functions of Hyperscan.
 *
 * Patterns are scanned in 'DOTALL' mode, which is equivalent to PCRE's '/s'
 * modifier. This behaviour can be changed by modifying the "flags" argument to
 * hs_compile.
 *
 * Build instructions:
 *
 *     gcc -o simplegrep simplegrep.c $(pkg-config --cflags --libs libhs)
 *
 * Usage:
 *
 *     ./simplegrep <pattern> <input file>
 *
 * Example:
 *
 *     ./simplegrep int simplegrep.c
 *
 */

#include <errno.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <hs.h>

/**
 * This is the function that will be called for each match that occurs. @a ctx
 * is to allow you to have some application-specific state that you will get
 * access to for each match. In our simple example we're just going to use it
 * to pass in the pattern that was being searched for so we can print it out.
 */
static int eventHandler(unsigned int id, unsigned long long from,
                        unsigned long long to, unsigned int flags, void *ctx) {
    printf("Match for pattern \"%s\" at offset %llu\n", (char *)ctx, to);
    return 0;
}

/**
 * Fill a data buffer from the given filename, returning it and filling @a
 * length with its length. Returns NULL on failure.
 */
static char *readInputData(const char *inputFN, unsigned int *length) {
    FILE *f = fopen(inputFN, "rb");
    if (!f) {
        fprintf(stderr, "ERROR: unable to open file \"%s\": %s\n", inputFN,
                strerror(errno));
        return NULL;
    }

    /* We use fseek/ftell to get our data length, in order to keep this example
     * code as portable as possible. */
    if (fseek(f, 0, SEEK_END) != 0) {
        fprintf(stderr, "ERROR: unable to seek file \"%s\": %s\n", inputFN,
                strerror(errno));
        fclose(f);
        return NULL;
    }
    long dataLen = ftell(f);
    if (dataLen < 0) {
        fprintf(stderr, "ERROR: ftell() failed: %s\n", strerror(errno));
        fclose(f);
        return NULL;
    }
    if (fseek(f, 0, SEEK_SET) != 0) {
        fprintf(stderr, "ERROR: unable to seek file \"%s\": %s\n", inputFN,
                strerror(errno));
        fclose(f);
        return NULL;
    }

    /* Hyperscan's hs_scan function accepts length as an unsigned int, so we
     * limit the size of our buffer appropriately. */
    if ((unsigned long)dataLen > UINT_MAX) {
        dataLen = UINT_MAX;
        printf("WARNING: clipping data to %ld bytes\n", dataLen);
    } else if (dataLen == 0) {
        fprintf(stderr, "ERROR: input file \"%s\" is empty\n", inputFN);
        fclose(f);
        return NULL;
    }

    char *inputData = malloc(dataLen);
    if (!inputData) {
        fprintf(stderr, "ERROR: unable to malloc %ld bytes\n", dataLen);
        fclose(f);
        return NULL;
    }

    char *p = inputData;
    size_t bytesLeft = dataLen;
    while (bytesLeft) {
        size_t bytesRead = fread(p, 1, bytesLeft, f);
        bytesLeft -= bytesRead;
        p += bytesRead;
        if (ferror(f) != 0) {
            fprintf(stderr, "ERROR: fread() failed\n");
            free(inputData);
            fclose(f);
            return NULL;
        }
    }

    fclose(f);

    *length = (unsigned int)dataLen;
    return inputData;
}

int main(int argc, char *argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <pattern> <input file>\n", argv[0]);
        return -1;
    }

    char *pattern = argv[1];
    char *inputFN = argv[2];

    /* First, we attempt to compile the pattern provided on the command line.
     * We assume 'DOTALL' semantics, meaning that the '.' meta-character will
     * match newline characters. The compiler will analyse the given pattern and
     * either return a compiled Hyperscan database, or an error message
     * explaining why the pattern didn't compile.
     */
    hs_database_t *database;
    hs_compile_error_t *compile_err;
    if (hs_compile(pattern, HS_FLAG_DOTALL, HS_MODE_BLOCK, NULL, &database,
                   &compile_err) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to compile pattern \"%s\": %s\n",
                pattern, compile_err->message);
        hs_free_compile_error(compile_err);
        return -1;
    }

    /* Next, we read the input data file into a buffer. */
    unsigned int length;
    char *inputData = readInputData(inputFN, &length);
    if (!inputData) {
        hs_free_database(database);
        return -1;
    }

    /* Finally, we issue a call to hs_scan, which will search the input buffer
     * for the pattern represented in the bytecode. Note that in order to do
     * this, scratch space needs to be allocated with the hs_alloc_scratch
     * function. In typical usage, you would reuse this scratch space for many
     * calls to hs_scan, but as we're only doing one, we'll be allocating it
     * and deallocating it as soon as our matching is done.
     *
     * When matches occur, the specified callback function (eventHandler in
     * this file) will be called. Note that although it is reminiscent of
     * asynchronous APIs, Hyperscan operates synchronously: all matches will be
     * found, and all callbacks issued, *before* hs_scan returns.
     *
     * In this example, we provide the input pattern as the context pointer so
     * that the callback is able to print out the pattern that matched on each
     * match event.
     */
    hs_scratch_t *scratch = NULL;
    if (hs_alloc_scratch(database, &scratch) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to allocate scratch space. Exiting.\n");
        free(inputData);
        hs_free_database(database);
        return -1;
    }

    printf("Scanning %u bytes with Hyperscan\n", length);

    if (hs_scan(database, inputData, length, 0, scratch, eventHandler,
                pattern) != HS_SUCCESS) {
        fprintf(stderr, "ERROR: Unable to scan input buffer. Exiting.\n");
        hs_free_scratch(scratch);
        free(inputData);
        hs_free_database(database);
        return -1;
    }

    /* Scanning is complete, any matches have been handled, so now we just
     * clean up and exit.
     */
    hs_free_scratch(scratch);
    free(inputData);
    hs_free_database(database);
    return 0;
}
