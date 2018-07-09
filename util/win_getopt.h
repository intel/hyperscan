/*
 * Copyright (c) 2018, Intel Corporation
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

#ifndef WIN_GETOPT_H
#define WIN_GETOPT_H

#include <windows.h>
#define ILLEGAL (int)'?'
#define END -1
#define SPECIAL_OPT 1

int optind = 0;
char *optarg;
static char EMPT[] = "";
static char *ptr = EMPT;
static int no_argument = 0;
static int required_argument = 1;
static const char no_arg[] = "option doesn't take an argument --%.*s";
static const char non_opt_string[] = "not an option : %s";
static const char ill_shortopt_char[] = "unknown option -%c";
static const char ill_longopt_string[] = "unknown option --%s";
static const char req_arg_string[] = "option requires an argument --%s";
static const char req_arg_char[] = "option requires an argument -%c";

struct option {
    const char *name;
    int has_arg;
    int *flag;
    int value;
};

static
void warn(const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vfprintf(stdout, fmt, args);
    fprintf(stdout, "\n");
    va_end(args);
}

int getopt_long(int nargc, char *const *nargv, const char *options,
                const struct option *long_options, int *idx) {
    char *check, *equal;
    size_t current_opt_len;
    bool all_flag = false;
    int match = -1;
    // illegal
    if (options == NULL) {
        return ILLEGAL;
    }
    if (optind == 0) {
        optind = 1;
    }
    if (optind >= nargc) {
        return END;
    }
    if (*options == '-') {
        all_flag = true;
        ++options;
    }
    optarg = NULL;
    // illegal
    if (*(ptr = nargv[optind]) != '-') {
        ptr = EMPT;
        if (all_flag) {
            optarg = nargv[optind++];
            return SPECIAL_OPT;
        } else {
            warn(non_opt_string, nargv[optind]);
            return ILLEGAL;
        }
    }
    // likely a short option ?
    if (ptr[1] != '\0' && *++ptr != '-' && ptr[1] == '\0') {
        char opt_char = *ptr;
        ptr = EMPT;
        // really short option ?
        if ((check = (char *)strchr(options, opt_char)) != NULL) {
            if (check[1] == ':') {
                ++optind;
                if (optind >= nargc) {
                    warn(req_arg_char, opt_char);
                    return ILLEGAL;
                } else {
                    optarg = nargv[optind];
                }
            }
            ++optind;
            return opt_char;
        } else { // illegal
            warn(ill_shortopt_char, opt_char);
            return ILLEGAL;
        }
    }
    // we meet '--'
    if (*ptr == '-' && ptr[1] == '\0') {
        ptr = EMPT;
        return END;
    }
    // we meet '--foo' , long option
    if (long_options != NULL && *ptr == '-' && ptr[1] != '\0') {
        ++ptr;
        if ((equal = strchr(ptr, '=')) != NULL) {
            // found --option=arg
            current_opt_len = equal - ptr;
            ++equal;
        } else {
            current_opt_len = strlen(ptr);
        }
        for (int i = 0; long_options[i].name; i++) {
            if (!strcmp(ptr, long_options[i].name )) {
                match = i;
                break;
            }
        }
        if (match == -1) { // don't match
            warn(ill_longopt_string, ptr);
            ptr = EMPT;
            return ILLEGAL;
        } else {
            ++optind;
            if (long_options[match].has_arg == required_argument) {
                if (equal) {
                    optarg = equal;
                } else if (optind < nargc) {
                    optarg = nargv[optind++];
                } else {
                    warn(req_arg_string, ptr);
                    ptr = EMPT;
                    return ILLEGAL;
                }
            }
            if (long_options[match].has_arg == no_argument && equal) {
                warn(no_arg, (int)current_opt_len, ptr);
                ptr = EMPT;
                return ILLEGAL;
            }
            ptr = EMPT;
            if (long_options[match].flag) {
                *long_options[match].flag = long_options[match].value;
                return 0;
            } else {
                return (long_options[match].value);
            }
        }
    }
    warn(non_opt_string, ptr);
    ptr = EMPT;
    return ILLEGAL;
}

#endif
